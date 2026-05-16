from __future__ import annotations

import platform
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal

from backend.config_manager import ConfigManager
from backend.research_logger import ResearchLogger
from vision.camera_calibration import CameraCalibrationManager
from vision.object_detector import DetectionBundle, DetectionResult, ObjectDetector
from vision.workspace_validator import WorkspaceValidator


class _SyntheticCapture:
    def __init__(self, size: tuple[int, int] = (640, 480)) -> None:
        self.width = int(size[0])
        self.height = int(size[1])
        self.frame_index = 0

    def isOpened(self) -> bool:
        return True

    def read(self) -> tuple[bool, np.ndarray]:
        frame = np.full((self.height, self.width, 3), (20, 28, 42), dtype=np.uint8)
        offset = int(80 * np.sin(self.frame_index / 18.0))
        object_x = 220 + offset
        object_y = 160
        object_w = 160
        object_h = 90
        cv2.rectangle(frame, (object_x, object_y), (object_x + object_w, object_y + object_h), (210, 210, 220), -1)
        cv2.rectangle(frame, (object_x, object_y), (object_x + object_w, object_y + 14), (60, 60, 60), -1)
        cv2.rectangle(frame, (object_x, object_y + object_h - 14), (object_x + object_w, object_y + object_h), (60, 60, 60), -1)
        cv2.putText(frame, "MOCK CAMERA", (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 168, 75), 2, cv2.LINE_AA)
        self.frame_index += 1
        return True, frame

    def release(self) -> None:
        return None


class VisionWorkerThread(QThread):
    objects_detected = pyqtSignal(list)
    frame_ready = pyqtSignal(object)
    status_changed = pyqtSignal(str, str)
    calibration_updated = pyqtSignal(dict)
    bundle_detected = pyqtSignal(object)  # emits DetectionBundle or None

    def __init__(self, source: str | int | None = None) -> None:
        super().__init__()
        self._config = ConfigManager.instance()
        self._logger = ResearchLogger.instance()
        self._workspace_validator = WorkspaceValidator(self._config)
        self._detector = ObjectDetector(self._config, self._workspace_validator)
        self._model_detector = None  # lazy: ModelDetector
        self._calibration = CameraCalibrationManager(self._config)
        self._source = source if source is not None else self._config.get("vision.video_source", 0)
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._last_detection: DetectionResult | None = None
        self._last_bundle: DetectionBundle | None = None
        self._last_frame: np.ndarray | None = None
        self._last_mask: np.ndarray | None = None
        self._model_warn_emitted: bool = False

    def stop(self) -> None:
        self._stop_event.set()
        self.wait(2000)

    def apply_settings(self, settings: dict) -> None:
        for key, value in settings.items():
            if key == "video_source":
                self._source = value
                continue
            self._config.set(f"vision.{key}", value)
        # If the user just switched into model mode, reset the warning latch so
        # the next mismatch (if any) is re-announced, and try to pick up a new
        # model path immediately.
        if "detection_method" in settings or "model_path" in settings:
            self._model_warn_emitted = False
            try:
                self._ensure_model_loaded_for_startup()
            except Exception:
                pass

    def get_latest_detection(self) -> DetectionResult | None:
        with self._lock:
            return self._last_detection

    def get_latest_bundle(self) -> DetectionBundle | None:
        with self._lock:
            return self._last_bundle

    def get_latest_frame(self) -> np.ndarray | None:
        with self._lock:
            return None if self._last_frame is None else self._last_frame.copy()

    def load_model(self, model_path: str | None = None) -> str:
        """Load the ONNX model detector. Returns status string."""
        import logging
        import traceback as _tb
        _log = logging.getLogger(__name__)
        try:
            from vision.model_detector import ModelDetector

            if self._model_detector is None:
                self._model_detector = ModelDetector(self._config, self._workspace_validator)
            self._model_detector.load_model(model_path)
            self._model_warn_emitted = False
            self.status_changed.emit(
                "info",
                f"ONNX model loaded ({model_path or self._config.get('vision.model_path', '')})",
            )
            return "LOADED"
        except Exception as exc:
            # Print full traceback to terminal so the *real* root cause
            # (e.g. DLL error, missing class count, bad path) is never hidden
            # behind a single-line status message.
            _log.error("Model load failed for path=%r:\n%s", model_path, _tb.format_exc())
            self.status_changed.emit("error", f"Model load failed: {exc}")
            return f"ERROR: {exc}"

    def _ensure_model_loaded_for_startup(self) -> None:
        """Auto-load the ONNX model when the worker starts in model mode.

        Previously the worker only used :class:`ModelDetector` when callers
        explicitly invoked ``load_model``. Nothing in the UI pipeline actually
        did that, so the worker silently fell back to contour detection even
        when the user had selected ``detection_method = "model"``. That is the
        exact symptom the user reports (fixture never seen, selongsong bbox
        looks wrong): the live feed was showing adaptive-threshold blobs, not
        YOLO boxes.
        """
        method = str(self._config.get("vision.detection_method", "adaptive")).strip().lower()
        if method != "model":
            return
        if self._model_detector is not None and self._model_detector.is_loaded:
            return
        configured_path = str(self._config.get("vision.model_path", "")).strip()
        if not configured_path:
            self.status_changed.emit(
                "warn",
                "Model mode selected but vision.model_path is empty; set it in Vision tab.",
            )
            return
        self.load_model(configured_path)

    def capture_snapshot(self) -> int:
        frame = self.get_latest_frame()
        if frame is None:
            raise RuntimeError("No active frame available")
        count = self._calibration.add_snapshot(frame)
        self.calibration_updated.emit(self._calibration.current_summary())
        return count

    def run_calibration(self, chessboard_size: tuple[int, int], square_size_mm: float) -> dict:
        calibration = self._calibration.run_calibration(chessboard_size, square_size_mm)
        summary = self._calibration.current_summary()
        self.calibration_updated.emit(summary)
        return calibration.summary()

    def calibration_summary(self) -> dict:
        return self._calibration.current_summary()

    def run(self) -> None:
        capture = self._open_capture(self._source)
        source_label = str(self._source)
        self.status_changed.emit("info", f"Vision started: {source_label}")
        self.calibration_updated.emit(self._calibration.current_summary())

        # Auto-load ONNX model if detection_method is set to "model" at startup.
        # Without this, the worker silently falls back to contour detection and
        # the live feed shows threshold blobs instead of YOLO boxes.
        self._ensure_model_loaded_for_startup()

        try:
            while not self._stop_event.is_set():
                started_at = time.perf_counter()
                ok, frame = capture.read()
                if not ok or frame is None:
                    self.status_changed.emit("warn", f"Vision source unavailable: {source_label}")
                    break

                frame = self._apply_camera_adjustments(frame)
                intrinsic_matrix = self._calibration.intrinsic_matrix_for_frame(frame.shape)
                z_mm = float(self._config.get("workspace.z_fixed_mm", 200.0))

                method = str(self._config.get("vision.detection_method", "adaptive")).strip().lower()
                detection: DetectionResult | None = None
                bundle: DetectionBundle | None = None
                mask: np.ndarray | None = None

                if method == "model":
                    if self._model_detector is not None and self._model_detector.is_loaded:
                        # Model-based detection
                        bundles = self._model_detector.detect_bundles(frame, intrinsic_matrix, z_mm)
                        if bundles:
                            bundle = bundles[0]
                            detection, mask = self._model_detector.detect(frame, intrinsic_matrix, z_mm)
                        else:
                            mask = np.zeros(frame.shape[:2], dtype=np.uint8)
                    else:
                        # Do NOT silently fall back to contour – that is exactly
                        # what produces the "aneh" detections the user sees.
                        # Instead show the raw frame and flag the issue once.
                        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
                        if not self._model_warn_emitted:
                            self.status_changed.emit(
                                "warn",
                                "Model mode selected but ONNX is not loaded; live detection disabled. "
                                "Click 'Load Model' or verify vision.model_path.",
                            )
                            self._model_warn_emitted = True
                else:
                    # Contour-based detection
                    detection, mask = self._detector.detect(frame, intrinsic_matrix, z_mm)

                overlay = self._workspace_validator.draw_overlay(frame, intrinsic_matrix, z_mm)
                preview_pick_zone = bool(self._config.get("vision.preview_pick_zone", True))

                if method == "model" and bundle is not None and self._model_detector is not None:
                    overlay = self._model_detector.draw_bundle_overlay(overlay, bundle)
                elif detection is not None:
                    overlay = self._detector.draw_detection_overlay(overlay, detection, preview_pick_zone)

                composed = self._compose_frame(overlay, mask)

                with self._lock:
                    self._last_detection = detection
                    self._last_bundle = bundle
                    self._last_frame = frame.copy()
                    self._last_mask = None if mask is None else mask.copy()

                self.objects_detected.emit([] if detection is None else [detection])
                self.bundle_detected.emit(bundle)
                self.frame_ready.emit(composed)
                elapsed_ms = (time.perf_counter() - started_at) * 1000.0
                self._logger.record("vision_process_time", elapsed_ms)
                delay_ms = max(1, int(33 - elapsed_ms))
                self.msleep(delay_ms)
        finally:
            capture.release()
            self.status_changed.emit("info", "Vision stopped")

    def _compose_frame(self, overlay: np.ndarray, mask: np.ndarray | None) -> np.ndarray:
        if not bool(self._config.get("vision.tune_live", False)) or mask is None:
            return overlay
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_bgr = cv2.resize(mask_bgr, (overlay.shape[1], overlay.shape[0]))
        return np.hstack([overlay, mask_bgr])

    def _apply_camera_adjustments(self, frame: np.ndarray) -> np.ndarray:
        brightness = int(self._config.get("vision.brightness", 0))
        contrast = float(self._config.get("vision.contrast", 1.0))
        zoom = float(self._config.get("vision.zoom", 1.0))
        adjusted = cv2.convertScaleAbs(frame, alpha=contrast, beta=brightness)
        if zoom <= 1.0:
            return adjusted
        height, width = adjusted.shape[:2]
        crop_width = max(1, int(round(width / zoom)))
        crop_height = max(1, int(round(height / zoom)))
        x1 = max(0, (width - crop_width) // 2)
        y1 = max(0, (height - crop_height) // 2)
        cropped = adjusted[y1 : y1 + crop_height, x1 : x1 + crop_width]
        return cv2.resize(cropped, (width, height), interpolation=cv2.INTER_LINEAR)

    def _open_capture(self, source: str | int | None):
        normalized = source
        if isinstance(source, str):
            stripped = source.strip()
            if stripped.upper() == "MOCK":
                return _SyntheticCapture()
            if stripped.isdigit():
                normalized = int(stripped)
            elif Path(stripped).exists():
                capture = cv2.VideoCapture(stripped)
                if capture.isOpened():
                    return capture
                self.status_changed.emit("warn", f"Unable to open video file: {stripped}; falling back to MOCK")
                return _SyntheticCapture()
            else:
                self.status_changed.emit("warn", f"Unknown vision source '{stripped}'; falling back to MOCK")
                return _SyntheticCapture()
        cam_index = 0 if normalized is None else normalized
        backend = cv2.CAP_DSHOW if platform.system() == "Windows" else cv2.CAP_ANY
        capture = cv2.VideoCapture(cam_index, backend)
        if capture.isOpened():
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            capture.set(cv2.CAP_PROP_FPS, 30)
            return capture
        self.status_changed.emit("warn", f"Unable to open camera '{normalized}'; falling back to MOCK")
        return _SyntheticCapture()