from __future__ import annotations

import logging
from pathlib import Path

import cv2
import numpy as np

from backend.config_manager import ConfigManager
from backend.exceptions import ModelNotFoundError
from vision.object_detector import BaseDetector, DetectionBundle, DetectionResult
from vision.workspace_validator import WorkspaceValidator

_log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Letterbox helpers
# ---------------------------------------------------------------------------

def _letterbox(
    frame: np.ndarray,
    new_shape: tuple[int, int] = (640, 640),
) -> tuple[np.ndarray, float, float, float]:
    """Resize *frame* with letterbox padding and return (img, scale, dw, dh)."""
    h, w = frame.shape[:2]
    scale = min(new_shape[0] / h, new_shape[1] / w)
    new_w = int(round(w * scale))
    new_h = int(round(h * scale))
    dw = (new_shape[1] - new_w) / 2.0
    dh = (new_shape[0] - new_h) / 2.0
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    padded = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))
    # ensure exact target size after rounding
    if padded.shape[0] != new_shape[0] or padded.shape[1] != new_shape[1]:
        padded = cv2.resize(padded, (new_shape[1], new_shape[0]))
    return padded, scale, dw, dh


def _unpad_rescale(boxes: np.ndarray, scale: float, dw: float, dh: float) -> np.ndarray:
    """Convert boxes from letterbox coords back to original frame coords.

    boxes shape: (N, 4) with [x1, y1, x2, y2].
    """
    if boxes.size == 0:
        return boxes
    out = boxes.copy().astype(np.float64)
    out[:, [0, 2]] = (out[:, [0, 2]] - dw) / scale
    out[:, [1, 3]] = (out[:, [1, 3]] - dh) / scale
    return np.clip(out, 0, None)


# ---------------------------------------------------------------------------
# IoU + pairing
# ---------------------------------------------------------------------------

def _iou(box_a: np.ndarray, box_b: np.ndarray) -> float:
    xi1 = max(float(box_a[0]), float(box_b[0]))
    yi1 = max(float(box_a[1]), float(box_b[1]))
    xi2 = min(float(box_a[2]), float(box_b[2]))
    yi2 = min(float(box_a[3]), float(box_b[3]))
    inter = max(0.0, xi2 - xi1) * max(0.0, yi2 - yi1)
    area_a = (float(box_a[2]) - float(box_a[0])) * (float(box_a[3]) - float(box_a[1]))
    area_b = (float(box_b[2]) - float(box_b[0])) * (float(box_b[3]) - float(box_b[1]))
    return inter / (area_a + area_b - inter + 1e-6)


def _pair_detections(
    boxes_selongsong: np.ndarray,
    confs_selongsong: np.ndarray,
    boxes_fixture: np.ndarray,
    confs_fixture: np.ndarray,
) -> list[tuple[np.ndarray, float, np.ndarray | None, float]]:
    """Pair each selongsong with its best-IoU fixture.

    Returns list of (sel_box, sel_conf, fix_box_or_None, fix_conf).
    """
    pairs: list[tuple[np.ndarray, float, np.ndarray | None, float]] = []
    for i in range(len(boxes_selongsong)):
        s_box = boxes_selongsong[i]
        s_conf = float(confs_selongsong[i])
        if len(boxes_fixture) == 0:
            pairs.append((s_box, s_conf, None, 0.0))
            continue
        best_idx = int(np.argmax([_iou(s_box, f) for f in boxes_fixture]))
        best_iou = _iou(s_box, boxes_fixture[best_idx])
        if best_iou > 0.0:
            pairs.append((s_box, s_conf, boxes_fixture[best_idx], float(confs_fixture[best_idx])))
        else:
            pairs.append((s_box, s_conf, None, 0.0))
    return pairs


# ---------------------------------------------------------------------------
# NMS wrapper
# ---------------------------------------------------------------------------

def _nms_per_class(
    boxes_xyxy: np.ndarray,
    confs: np.ndarray,
    iou_threshold: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply NMS on xyxy boxes. Returns filtered (boxes, confs)."""
    if len(boxes_xyxy) == 0:
        return np.empty((0, 4), dtype=np.float32), np.empty((0,), dtype=np.float32)
    # cv2.dnn.NMSBoxes needs xywh format
    xywh = np.zeros_like(boxes_xyxy)
    xywh[:, 0] = boxes_xyxy[:, 0]
    xywh[:, 1] = boxes_xyxy[:, 1]
    xywh[:, 2] = boxes_xyxy[:, 2] - boxes_xyxy[:, 0]
    xywh[:, 3] = boxes_xyxy[:, 3] - boxes_xyxy[:, 1]
    indices = cv2.dnn.NMSBoxes(
        xywh.tolist(),
        confs.tolist(),
        score_threshold=0.0,  # already pre-filtered
        nms_threshold=iou_threshold,
    )
    if len(indices) == 0:
        return np.empty((0, 4), dtype=np.float32), np.empty((0,), dtype=np.float32)
    indices = np.asarray(indices).flatten()
    return boxes_xyxy[indices], confs[indices]


# ---------------------------------------------------------------------------
# ModelDetector
# ---------------------------------------------------------------------------

class ModelDetector(BaseDetector):
    """YOLOv8 two-class ONNX detector: class 0 = selongsong, class 1 = fixture."""

    def __init__(
        self,
        config: ConfigManager | None = None,
        workspace_validator: WorkspaceValidator | None = None,
    ) -> None:
        super().__init__(config, workspace_validator)
        self._session = None  # lazy-loaded
        self._mock_mode = False
        self._mock_frame_idx = 0
        self._input_name: str = "images"
        self._input_shape: tuple[int, int] = (640, 640)

    # ------ public API ------

    def load_model(self, model_path: str | None = None) -> None:
        """Eagerly load (or reload) the ONNX model.

        If *model_path* is ``"MOCK"`` (case-insensitive), the detector enters
        mock mode and will generate synthetic DetectionBundles without an ONNX
        Runtime session.
        """
        path = model_path or str(self._config.get("vision.model_path", ""))
        if path.strip().upper() == "MOCK":
            self._session = None
            self._mock_mode = True
            _log.info("ModelDetector running in MOCK mode")
            return

        # Reset state up-front so a failed real-model load can NEVER leave a
        # stale MOCK session active. Without this, switching config from
        # ``"MOCK"`` to a real path while onnxruntime fails to import would
        # still report ``is_loaded == True`` and keep emitting synthetic
        # bounding boxes — exactly the "bbox in a black frame" symptom.
        self._session = None
        self._mock_mode = False

        import onnxruntime as ort

        resolved = self._resolve_model_path(path)
        if not resolved or not resolved.is_file():
            raise ModelNotFoundError(f"ONNX model not found: {path}")
        self._session = ort.InferenceSession(
            str(resolved),
            providers=["CUDAExecutionProvider", "CPUExecutionProvider"],
        )
        meta = self._session.get_inputs()[0]
        self._input_name = meta.name
        shape = meta.shape  # e.g. [1, 3, 640, 640]
        if len(shape) == 4:
            self._input_shape = (int(shape[2]), int(shape[3]))
        self._mock_mode = False
        _log.info("ONNX model loaded: %s (input %s)", path, self._input_shape)

    @property
    def is_loaded(self) -> bool:
        return self._session is not None or self._mock_mode

    def detect(
        self,
        frame: np.ndarray,
        intrinsic_matrix: np.ndarray,
        z_mm: float,
    ) -> tuple[DetectionResult | None, np.ndarray]:
        """Run ONNX inference, returning first selongsong as DetectionResult for compat."""
        bundles = self.detect_bundles(frame, intrinsic_matrix, z_mm)
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        if not bundles:
            return None, mask
        bundle = bundles[0]
        if bundle.selongsong_box is None or bundle.pick_point_px is None:
            return None, mask
        box = bundle.selongsong_box.astype(int)
        x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
        w, h = x2 - x1, y2 - y1
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        contour = np.array([[[x1, y1]], [[x2, y1]], [[x2, y2]], [[x1, y2]]], dtype=np.int32)
        fx = max(float(intrinsic_matrix[0, 0]), 1.0)
        fy = max(float(intrinsic_matrix[1, 1]), 1.0)
        width_mm = float(w) * float(z_mm) / fx
        height_mm = float(h) * float(z_mm) / fy
        centroid_world = self._workspace_validator.pixel_to_world(cx, cy, intrinsic_matrix, z_mm)
        pick_world = bundle.pick_point_world or centroid_world
        workspace_status, workspace_message = self._workspace_validator.check(pick_world[0], pick_world[1])
        result = DetectionResult(
            bbox_px=(x1, y1, w, h),
            centroid_px=(cx, cy),
            contour=contour,
            orientation_deg=0.0,
            centroid_world=centroid_world,
            width_mm=width_mm,
            height_mm=height_mm,
            pick_point_px=bundle.pick_point_px,
            pick_point_world=pick_world,
            workspace_status=workspace_status,
            safe_zone_px=(x1, y1, w, h),
            workspace_message=workspace_message,
        )
        return result, mask

    def detect_bundles(
        self,
        frame: np.ndarray,
        intrinsic_matrix: np.ndarray,
        z_mm: float,
    ) -> list[DetectionBundle]:
        """Full model detection returning list of DetectionBundle."""
        if not self.is_loaded:
            self._lazy_load()
        if self._mock_mode:
            return self._generate_mock_bundles(frame, intrinsic_matrix, z_mm)
        if self._session is None:
            return []

        conf_threshold = float(self._config.get("vision.model_conf_threshold", 0.5))
        iou_threshold = float(self._config.get("vision.model_iou_threshold", 0.45))

        # Preprocess
        padded, scale, dw, dh = _letterbox(frame, self._input_shape)
        blob = padded[:, :, ::-1].astype(np.float32) / 255.0  # BGR→RGB, normalize
        blob = np.transpose(blob, (2, 0, 1))[np.newaxis]  # (1, 3, H, W)

        # Inference
        outputs = self._session.run(None, {self._input_name: blob})
        preds = np.asarray(outputs[0])  # (1, 4+num_cls, N) or (1, N, 4+num_cls)

        # Reshape to (N, 4+num_cls)
        if preds.ndim == 3:
            preds = preds[0]
        if preds.shape[0] < preds.shape[1]:
            preds = preds.T  # (N, 4+num_cls)

        # Separate box coords and class scores
        # YOLOv8: first 4 = cx, cy, w, h; rest = class scores
        num_classes = preds.shape[1] - 4
        if num_classes < 2:
            _log.warning("Model has %d classes; expected >= 2", num_classes)
            return []

        cx_cy_wh = preds[:, :4]
        class_scores = preds[:, 4:]

        # Convert cx,cy,w,h → x1,y1,x2,y2
        boxes_xyxy = np.zeros_like(cx_cy_wh)
        boxes_xyxy[:, 0] = cx_cy_wh[:, 0] - cx_cy_wh[:, 2] / 2  # x1
        boxes_xyxy[:, 1] = cx_cy_wh[:, 1] - cx_cy_wh[:, 3] / 2  # y1
        boxes_xyxy[:, 2] = cx_cy_wh[:, 0] + cx_cy_wh[:, 2] / 2  # x2
        boxes_xyxy[:, 3] = cx_cy_wh[:, 1] + cx_cy_wh[:, 3] / 2  # y2

        # Per-class extraction
        best_class = np.argmax(class_scores, axis=1)
        best_conf = np.max(class_scores, axis=1)

        # Filter by confidence
        keep = best_conf > conf_threshold
        boxes_xyxy = boxes_xyxy[keep]
        best_class = best_class[keep]
        best_conf = best_conf[keep]

        # Separate by class
        mask_cls0 = best_class == 0
        mask_cls1 = best_class == 1
        boxes_cls0 = boxes_xyxy[mask_cls0]
        confs_cls0 = best_conf[mask_cls0]
        boxes_cls1 = boxes_xyxy[mask_cls1]
        confs_cls1 = best_conf[mask_cls1]

        # NMS per class
        boxes_cls0, confs_cls0 = _nms_per_class(boxes_cls0, confs_cls0, iou_threshold)
        boxes_cls1, confs_cls1 = _nms_per_class(boxes_cls1, confs_cls1, iou_threshold)

        # UNPAD + rescale to original frame coords
        boxes_cls0 = _unpad_rescale(boxes_cls0, scale, dw, dh)
        boxes_cls1 = _unpad_rescale(boxes_cls1, scale, dw, dh)

        # Clip to frame dimensions
        h_frame, w_frame = frame.shape[:2]
        if len(boxes_cls0) > 0:
            boxes_cls0[:, [0, 2]] = np.clip(boxes_cls0[:, [0, 2]], 0, w_frame)
            boxes_cls0[:, [1, 3]] = np.clip(boxes_cls0[:, [1, 3]], 0, h_frame)
        if len(boxes_cls1) > 0:
            boxes_cls1[:, [0, 2]] = np.clip(boxes_cls1[:, [0, 2]], 0, w_frame)
            boxes_cls1[:, [1, 3]] = np.clip(boxes_cls1[:, [1, 3]], 0, h_frame)

        if len(boxes_cls0) == 0:
            return []

        # Pair selongsong ↔ fixture
        pairs = _pair_detections(boxes_cls0, confs_cls0, boxes_cls1, confs_cls1)

        # Build DetectionBundle for each pair
        from vision.safe_pick_validator import SafePickValidator

        validator = SafePickValidator(self._config)
        margin_pct = float(self._config.get("vision.safe_pick_margin_pct", 0.15))
        bundles: list[DetectionBundle] = []
        for s_box, s_conf, f_box, f_conf in pairs:
            safety, pick_px = validator.check_pair(s_box, f_box, margin_pct)
            pick_world = None
            if pick_px is not None:
                pick_world = self._workspace_validator.pixel_to_world(
                    pick_px[0], pick_px[1], intrinsic_matrix, z_mm,
                )
            bundles.append(DetectionBundle(
                selongsong_box=s_box,
                fixture_box=f_box,
                pick_point_px=pick_px,
                pick_point_world=pick_world,
                pick_safety=safety,
                conf_selongsong=s_conf,
                conf_fixture=f_conf,
            ))
        return bundles

    def draw_bundle_overlay(
        self,
        frame: np.ndarray,
        bundle: DetectionBundle,
    ) -> np.ndarray:
        overlay = frame.copy()
        # Draw selongsong bbox (green)
        if bundle.selongsong_box is not None:
            box = bundle.selongsong_box.astype(int)
            cv2.rectangle(overlay, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)
            label = f"sel {bundle.conf_selongsong:.2f}"
            cv2.putText(overlay, label, (box[0], max(box[1] - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        # Draw fixture bbox (red dashed)
        if bundle.fixture_box is not None:
            box = bundle.fixture_box.astype(int)
            self._draw_dashed_rect(overlay, (box[0], box[1]), (box[2], box[3]), (0, 0, 255), 2)
            label = f"fix {bundle.conf_fixture:.2f}"
            cv2.putText(overlay, label, (box[0], max(box[1] - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

        # Resolve pick-point coordinates and safety colour.
        #
        # Earlier the crosshair was suppressed whenever ``pick_point_px`` was
        # ``None`` (i.e. ``UNSAFE``), which made the live feed look exactly
        # like the user's complaint: bboxes but no "titik pick point". We now
        # fall back to the selongsong centroid so that every detected
        # selongsong gets *some* marker — UNSAFE just turns it red.
        status = bundle.pick_safety or "UNKNOWN"
        color = {
            "SAFE": (0, 255, 0),
            "MARGINAL": (0, 191, 255),
            "UNSAFE": (0, 0, 255),
            "UNKNOWN": (180, 180, 180),
        }.get(status, (180, 180, 180))

        pick_px = bundle.pick_point_px
        if pick_px is None and bundle.selongsong_box is not None:
            sx1, sy1, sx2, sy2 = bundle.selongsong_box.astype(int)
            pick_px = ((sx1 + sx2) // 2, (sy1 + sy2) // 2)

        # Optional safe-zone rectangle (matches Colab inference overlay).
        # Computed on-the-fly from selongsong − fixture so we do not have to
        # change the DetectionBundle schema.
        safe_zone = self._compute_safe_zone_rect(bundle)
        if safe_zone is not None:
            zx1, zy1, zx2, zy2 = safe_zone
            cv2.rectangle(overlay, (zx1, zy1), (zx2, zy2), color, 1, cv2.LINE_AA)

        # Draw pickup crosshair + textual status badge.
        if pick_px is not None:
            pu, pv = int(pick_px[0]), int(pick_px[1])
            cv2.drawMarker(overlay, (pu, pv), color, cv2.MARKER_CROSS, 20, 2)
            label_lines = [f"{status} ({pu},{pv})"]
            if bundle.pick_point_world is not None:
                label_lines.append(
                    f"({bundle.pick_point_world[0]:.1f}, {bundle.pick_point_world[1]:.1f}) mm"
                )
            for i, text in enumerate(label_lines):
                cv2.putText(overlay, text, (pu + 12, pv - 8 + i * 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        return overlay

    @staticmethod
    def _compute_safe_zone_rect(bundle: DetectionBundle) -> tuple[int, int, int, int] | None:
        """Reconstruct the largest safe-zone rectangle from the bundle boxes.

        Mirrors the geometry in :class:`SafePickValidator.check_pair`, but
        only returns the rectangle so the overlay can render it. Returns
        ``None`` when no meaningful safe zone exists.
        """
        if bundle.selongsong_box is None:
            return None
        sx1, sy1, sx2, sy2 = bundle.selongsong_box.astype(float)
        if bundle.fixture_box is None:
            return int(sx1), int(sy1), int(sx2), int(sy2)
        fx1, fy1, fx2, fy2 = bundle.fixture_box.astype(float)
        candidates: list[tuple[float, float, float, float]] = []
        if fx1 > sx1:
            candidates.append((sx1, sy1, min(sx2, fx1), sy2))
        if fx2 < sx2:
            candidates.append((max(sx1, fx2), sy1, sx2, sy2))
        if fy1 > sy1:
            candidates.append((sx1, sy1, sx2, min(sy2, fy1)))
        if fy2 < sy2:
            candidates.append((sx1, max(sy1, fy2), sx2, sy2))
        candidates = [c for c in candidates if (c[2] - c[0]) > 0 and (c[3] - c[1]) > 0]
        if not candidates:
            return None
        best = max(candidates, key=lambda c: (c[2] - c[0]) * (c[3] - c[1]))
        return int(best[0]), int(best[1]), int(best[2]), int(best[3])

    # ------ private ------

    def _lazy_load(self) -> None:
        try:
            self.load_model()
        except (ModelNotFoundError, ImportError, Exception) as exc:
            _log.warning("Model lazy-load failed: %s", exc)
            self._session = None

    @staticmethod
    def _resolve_model_path(path: str) -> Path | None:
        """Resolve *path* to an existing file.

        Tries, in order: as-is (absolute or cwd-relative), then relative to
        the PAROL6_app project root (i.e. the directory containing
        ``main.py``). Returns the first hit or ``None``.
        """
        candidate = Path(path).expanduser()
        if candidate.is_file():
            return candidate.resolve()
        # project root = PAROL6_app/ (this file lives in PAROL6_app/vision/)
        project_root = Path(__file__).resolve().parent.parent
        rooted = (project_root / path).resolve()
        if rooted.is_file():
            return rooted
        return None

    def _generate_mock_bundles(
        self,
        frame: np.ndarray,
        intrinsic_matrix: np.ndarray,
        z_mm: float,
    ) -> list[DetectionBundle]:
        """Return a synthetic DetectionBundle for MOCK mode."""
        h, w = frame.shape[:2]
        rng = np.random.default_rng(seed=self._mock_frame_idx)
        self._mock_frame_idx += 1

        # Selongsong bounding box — oscillate around center
        sx1 = int(w * 0.3 + 40 * np.sin(self._mock_frame_idx / 15.0))
        sy1 = int(h * 0.3 + 30 * np.cos(self._mock_frame_idx / 20.0))
        s_w, s_h = 130, 70
        s_box = np.array([sx1, sy1, sx1 + s_w, sy1 + s_h], dtype=np.float64)

        # Fixture box — slightly larger, overlapping top
        fx1 = sx1 - 10
        fy1 = sy1 - 14
        f_w, f_h = s_w + 20, 14
        f_box: np.ndarray | None = np.array([fx1, fy1, fx1 + f_w, fy1 + f_h], dtype=np.float64)

        # Randomly remove fixture 10% of the time → UNKNOWN
        if rng.random() < 0.1:
            f_box = None

        from vision.safe_pick_validator import SafePickValidator

        validator = SafePickValidator(self._config)
        margin_pct = float(self._config.get("vision.safe_pick_margin_pct", 0.15))
        safety, pick_px = validator.check_pair(s_box, f_box, margin_pct)
        pick_world = None
        if pick_px is not None:
            pick_world = self._workspace_validator.pixel_to_world(
                pick_px[0], pick_px[1], intrinsic_matrix, z_mm,
            )

        bundle = DetectionBundle(
            selongsong_box=s_box,
            fixture_box=f_box,
            pick_point_px=pick_px,
            pick_point_world=pick_world,
            pick_safety=safety,
            conf_selongsong=round(float(rng.uniform(0.75, 0.98)), 2),
            conf_fixture=round(float(rng.uniform(0.70, 0.95)), 2) if f_box is not None else 0.0,
        )
        return [bundle]

    @classmethod
    def _draw_dashed_rect(
        cls,
        frame: np.ndarray,
        top_left: tuple[int, int],
        bottom_right: tuple[int, int],
        color: tuple[int, int, int],
        thickness: int,
    ) -> None:
        x1 = min(top_left[0], bottom_right[0])
        y1 = min(top_left[1], bottom_right[1])
        x2 = max(top_left[0], bottom_right[0])
        y2 = max(top_left[1], bottom_right[1])
        cls._draw_dashed_line(frame, (x1, y1), (x2, y1), color, thickness)
        cls._draw_dashed_line(frame, (x2, y1), (x2, y2), color, thickness)
        cls._draw_dashed_line(frame, (x2, y2), (x1, y2), color, thickness)
        cls._draw_dashed_line(frame, (x1, y2), (x1, y1), color, thickness)

    @staticmethod
    def _draw_dashed_line(
        frame: np.ndarray,
        start: tuple[int, int],
        end: tuple[int, int],
        color: tuple[int, int, int],
        thickness: int,
        dash_length: int = 8,
        gap_length: int = 4,
    ) -> None:
        delta = np.array(end, dtype=np.float64) - np.array(start, dtype=np.float64)
        distance = float(np.linalg.norm(delta))
        if distance <= 0:
            return
        direction = delta / distance
        offset = 0.0
        while offset < distance:
            dash_end = min(offset + dash_length, distance)
            p1 = tuple(np.round(np.array(start) + direction * offset).astype(int))
            p2 = tuple(np.round(np.array(start) + direction * dash_end).astype(int))
            cv2.line(frame, p1, p2, color, thickness, cv2.LINE_AA)
            offset += dash_length + gap_length
