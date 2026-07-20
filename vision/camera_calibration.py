from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from typing import Sequence

from pathlib import Path

import cv2
import numpy as np

from backend.config_manager import ConfigManager


@dataclass(slots=True)
class CalibrationData:
    intrinsic_matrix: np.ndarray
    distortion: np.ndarray
    image_size: tuple[int, int]
    rms_error: float | None = None

    @property
    def is_valid(self) -> bool:
        return self.intrinsic_matrix.shape == (3, 3) and self.distortion.size > 0

    def to_config_dict(self) -> dict:
        return {
            "intrinsic_matrix": self.intrinsic_matrix.tolist(),
            "distortion": self.distortion.reshape(-1).tolist(),
            "image_size": [int(self.image_size[0]), int(self.image_size[1])],
            "rms_error": self.rms_error,
        }

    def summary(self) -> dict:
        fx = float(self.intrinsic_matrix[0, 0])
        fy = float(self.intrinsic_matrix[1, 1])
        cx = float(self.intrinsic_matrix[0, 2])
        cy = float(self.intrinsic_matrix[1, 2])
        return {
            "fx": round(fx, 3),
            "fy": round(fy, 3),
            "cx": round(cx, 3),
            "cy": round(cy, 3),
            "distortion": [round(float(value), 6) for value in self.distortion.reshape(-1)],
            "image_size": [int(self.image_size[0]), int(self.image_size[1])],
            "rms_error": None if self.rms_error is None else round(float(self.rms_error), 6),
        }

    @classmethod
    def from_config(cls, config_value: dict | None) -> CalibrationData | None:
        if not config_value:
            return None
        matrix = np.array(config_value.get("intrinsic_matrix") or [], dtype=np.float64)
        distortion = np.array(config_value.get("distortion") or [], dtype=np.float64)
        image_size = tuple(config_value.get("image_size") or [0, 0])
        if matrix.size != 9 or distortion.size == 0:
            return None
        return cls(
            intrinsic_matrix=matrix.reshape(3, 3),
            distortion=distortion.reshape(1, -1),
            image_size=(int(image_size[0]), int(image_size[1])),
            rms_error=config_value.get("rms_error"),
        )


class CameraCalibrationManager:
    def __init__(
        self,
        config: ConfigManager | None = None,
        snapshot_dir: Path | str | None = None,
    ) -> None:
        self._config = config or ConfigManager.instance()
        self._snapshots: list[np.ndarray] = []
        self._snapshot_paths: list[Path] = []
        self._snapshot_dir_override = Path(snapshot_dir) if snapshot_dir is not None else None
        self._last_snapshot_valid: bool | None = None
        self._calibration = CalibrationData.from_config(self._config.get("vision.calibration"))

    def add_snapshot(self, frame: np.ndarray) -> int:
        if frame is None or frame.size == 0:
            raise ValueError("Calibration snapshot frame is empty")
        calibration_cfg = self._config.get("vision.calibration", {})
        if not isinstance(calibration_cfg, dict):
            calibration_cfg = {}
        pattern_size = self.normalize_chessboard_size(
            calibration_cfg.get("chessboard_size", (9, 6))
        )
        found, _ = self.find_chessboard_corners(frame, pattern_size)
        snapshot_path = self._save_snapshot_image(frame)
        self._snapshots.append(frame.copy())
        self._snapshot_paths.append(snapshot_path)
        self._last_snapshot_valid = found
        calibration_cfg.update(
            {
                "snapshots_captured": len(self._snapshots),
                "last_snapshot_path": str(snapshot_path),
                "last_snapshot_valid": bool(found),
            }
        )
        self._config.set("vision.calibration", calibration_cfg)
        return len(self._snapshots)

    def clear_snapshots(self) -> None:
        self._snapshots.clear()
        self._snapshot_paths.clear()
        self._last_snapshot_valid = None
        calibration_cfg = self._config.get("vision.calibration", {})
        if not isinstance(calibration_cfg, dict):
            calibration_cfg = {}
        calibration_cfg.update(
            {
                "snapshots_captured": 0,
                "last_snapshot_path": "",
                "last_snapshot_valid": None,
            }
        )
        self._config.set("vision.calibration", calibration_cfg)

    def reset_intrinsic_calibration(
        self,
        delete_snapshot_files: bool = True,
        intrinsics_path: Path | str | None = None,
    ) -> dict:
        calibration_cfg = self._config.get("vision.calibration", {})
        calibration_cfg = calibration_cfg if isinstance(calibration_cfg, dict) else {}
        preserved = {
            key: calibration_cfg[key]
            for key in (
                "chessboard_size",
                "square_size_mm",
                "preview_enabled",
                "snapshot_dir",
            )
            if key in calibration_cfg
        }
        preserved.update(
            {
                "snapshots_captured": 0,
                "last_snapshot_path": "",
                "last_snapshot_valid": None,
            }
        )

        deleted_snapshots = 0
        if delete_snapshot_files:
            snapshot_dir = self.snapshot_directory()
            if snapshot_dir.exists():
                for path in snapshot_dir.glob("intrinsic_*.jpg"):
                    if path.is_file():
                        path.unlink()
                        deleted_snapshots += 1

        target_intrinsics = (
            Path(intrinsics_path)
            if intrinsics_path is not None
            else self._default_intrinsics_path()
        )
        csv_deleted = False
        if target_intrinsics.exists():
            target_intrinsics.unlink()
            csv_deleted = True

        self._snapshots.clear()
        self._snapshot_paths.clear()
        self._last_snapshot_valid = None
        self._calibration = None
        self._config.set("vision.calibration", preserved)
        self._config.set(
            "vision.camera_to_base",
            {
                "valid": False,
                "homography": [],
                "image_size": [0, 0],
                "point_count": 0,
                "rms_error_mm": None,
                "max_error_mm": None,
            },
        )
        self._config.set("vision.camera_to_base_points", [])
        return {
            "calibrated": False,
            "deleted_snapshots": deleted_snapshots,
            "intrinsics_csv_deleted": csv_deleted,
            **self._snapshot_summary(),
        }

    def snapshot_count(self) -> int:
        return len(self._snapshots)

    def snapshot_directory(self) -> Path:
        if self._snapshot_dir_override is not None:
            return self._snapshot_dir_override
        configured = self._config.get("vision.calibration.snapshot_dir", "")
        if configured:
            path = Path(str(configured)).expanduser()
            if path.is_absolute():
                return path
            return self._project_root() / path
        return self._project_root() / "tools" / "Camera" / "calibration_snapshots"

    @staticmethod
    def normalize_chessboard_size(chessboard_size: Sequence[int]) -> tuple[int, int]:
        try:
            if len(chessboard_size) < 2:
                raise ValueError
            pattern_size = (int(chessboard_size[0]), int(chessboard_size[1]))
        except (TypeError, ValueError, IndexError) as exc:
            raise ValueError(
                "Chessboard size must contain integer columns and rows"
            ) from exc
        if pattern_size[0] < 2 or pattern_size[1] < 2:
            raise ValueError("Chessboard size must be at least 2x2 inner corners")
        return pattern_size

    @staticmethod
    def find_chessboard_corners(
        frame: np.ndarray,
        chessboard_size: Sequence[int],
        *,
        fast_check: bool = False,
    ) -> tuple[bool, np.ndarray | None]:
        if frame is None or frame.size == 0:
            return False, None
        pattern_size = CameraCalibrationManager.normalize_chessboard_size(chessboard_size)
        gray = (
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if frame.ndim == 3
            else frame.copy()
        )
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        if fast_check:
            flags |= cv2.CALIB_CB_FAST_CHECK
        found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
        if not found or corners is None:
            return False, None
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )
        refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        return True, refined

    @staticmethod
    def draw_chessboard_corners(
        frame: np.ndarray,
        chessboard_size: Sequence[int],
        corners: np.ndarray | None,
        found: bool,
    ) -> np.ndarray:
        overlay = frame.copy()
        if found and corners is not None:
            cv2.drawChessboardCorners(
                overlay,
                CameraCalibrationManager.normalize_chessboard_size(chessboard_size),
                corners,
                True,
            )
        return overlay

    def _save_snapshot_image(self, frame: np.ndarray) -> Path:
        output_dir = self.snapshot_directory()
        output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = output_dir / f"intrinsic_{timestamp}_{len(self._snapshots) + 1:03d}.jpg"
        if not cv2.imwrite(str(path), frame):
            raise OSError(f"Could not save calibration snapshot to {path}")
        return path

    @staticmethod
    def _project_root() -> Path:
        return Path(__file__).resolve().parents[1]

    def _snapshot_summary(self) -> dict:
        configured = self._config.get("vision.calibration", {})
        configured = configured if isinstance(configured, dict) else {}
        last_path = (
            str(self._snapshot_paths[-1])
            if self._snapshot_paths
            else str(configured.get("last_snapshot_path", ""))
        )
        last_valid = (
            self._last_snapshot_valid
            if self._last_snapshot_valid is not None
            else configured.get("last_snapshot_valid")
        )
        return {
            "snapshots_captured": self.snapshot_count(),
            "snapshot_dir": str(self.snapshot_directory()),
            "last_snapshot_path": last_path,
            "last_snapshot_valid": last_valid,
        }

    def has_calibration(self) -> bool:
        return self._calibration is not None and self._calibration.is_valid

    def current(self) -> CalibrationData | None:
        return self._calibration

    def current_summary(self) -> dict:
        if self._calibration is None:
            summary = {
                "fx": 0.0,
                "fy": 0.0,
                "cx": 0.0,
                "cy": 0.0,
                "distortion": [],
                "image_size": [640, 480],
                "rms_error": None,
                "calibrated": False,
            }
            summary.update(self._snapshot_summary())
            return summary
        summary = self._calibration.summary()
        summary.update(self._snapshot_summary())
        summary["calibrated"] = True
        return summary

    def estimate_default_matrix(self, frame_shape: tuple[int, ...]) -> np.ndarray:
        height, width = frame_shape[:2]
        fx = float(max(width, height))
        fy = float(max(width, height))
        cx = float(width) / 2.0
        cy = float(height) / 2.0
        return np.array(
            [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    def intrinsic_matrix_for_frame(self, frame_shape: tuple[int, ...]) -> np.ndarray:
        if self._calibration is not None and self._calibration.is_valid:
            return self._calibration.intrinsic_matrix.copy()
        return self.estimate_default_matrix(frame_shape)

    def distortion_for_frame(self) -> np.ndarray:
        if self._calibration is not None and self._calibration.is_valid:
            return self._calibration.distortion.copy()
        return np.zeros((1, 5), dtype=np.float64)

    def run_calibration(
        self,
        chessboard_size: Sequence[int] = (9, 6),
        square_size_mm: float = 25.0,
    ) -> CalibrationData:
        pattern_size = self.normalize_chessboard_size(chessboard_size)
        if float(square_size_mm) <= 0.0:
            raise ValueError("Chessboard square size must be greater than 0 mm")
        valid_frames: list[np.ndarray] = []
        object_points: list[np.ndarray] = []
        image_points: list[np.ndarray] = []

        base_object_points = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        base_object_points[:, :2] = np.mgrid[0 : pattern_size[0], 0 : pattern_size[1]].T.reshape(-1, 2)
        base_object_points *= float(square_size_mm)

        for frame in self._snapshots:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, refined = self.find_chessboard_corners(frame, pattern_size)
            if not found or refined is None:
                continue
            object_points.append(base_object_points.copy())
            image_points.append(refined)
            valid_frames.append(gray)

        if len(valid_frames) < 3:
            raise ValueError("At least 3 valid chessboard snapshots are required for calibration")

        image_size = (valid_frames[0].shape[1], valid_frames[0].shape[0])
        rms_error, intrinsic_matrix, distortion, _, _ = cv2.calibrateCamera(
            object_points,
            image_points,
            image_size,
            None,
            None,
        )
        calibration = CalibrationData(
            intrinsic_matrix=intrinsic_matrix,
            distortion=distortion,
            image_size=image_size,
            rms_error=float(rms_error),
        )
        self.save_calibration(calibration, chessboard_size=pattern_size, square_size_mm=square_size_mm)
        return calibration

    def run_camera_to_base_calibration(
        self,
        pixel_points: Sequence[Sequence[float]],
        base_points_mm: Sequence[Sequence[float]],
        image_size: tuple[int, int],
    ) -> dict:
        if not self.has_calibration():
            raise ValueError("Intrinsic camera calibration must be completed first")
        if len(pixel_points) != len(base_points_mm):
            raise ValueError("Pixel and Base point counts must match")
        if len(pixel_points) < 9:
            raise ValueError("At least 9 Camera-to-Base point pairs are required")

        pixels = np.asarray(pixel_points, dtype=np.float64).reshape(-1, 1, 2)
        base_points = np.asarray(base_points_mm, dtype=np.float64).reshape(-1, 2)
        calibration = self.current()
        assert calibration is not None
        undistorted = cv2.undistortPoints(
            pixels,
            calibration.intrinsic_matrix,
            calibration.distortion,
            P=calibration.intrinsic_matrix,
        ).reshape(-1, 2)
        homography, inlier_mask = cv2.findHomography(
            undistorted,
            base_points,
            method=cv2.RANSAC,
        )
        if homography is None:
            raise ValueError("Camera-to-Base homography could not be solved")

        projected = cv2.perspectiveTransform(
            undistorted.reshape(-1, 1, 2),
            homography,
        ).reshape(-1, 2)
        errors = np.linalg.norm(projected - base_points, axis=1)
        rms_error = float(np.sqrt(np.mean(np.square(errors))))
        max_error = float(np.max(errors))
        valid = rms_error <= 3.0 and max_error <= 5.0
        payload = {
            "valid": valid,
            "homography": homography.tolist(),
            "image_size": [int(image_size[0]), int(image_size[1])],
            "video_source": str(self._config.get("vision.video_source", "")),
            "zoom": float(self._config.get("vision.zoom", 1.0)),
            "point_count": int(len(pixel_points)),
            "inlier_count": int(np.sum(inlier_mask)) if inlier_mask is not None else int(len(pixel_points)),
            "rms_error_mm": rms_error,
            "max_error_mm": max_error,
            "raw_pixel_points": np.asarray(pixel_points, dtype=np.float64).reshape(-1, 2).tolist(),
            "pixel_points": undistorted.tolist(),
            "base_points_mm": base_points.tolist(),
        }
        self._config.set("vision.camera_to_base", payload)
        if not valid:
            raise ValueError(
                f"Camera-to-Base validation failed: RMS={rms_error:.3f} mm, "
                f"max={max_error:.3f} mm"
            )
        return payload

    def save_calibration(
        self,
        calibration: CalibrationData,
        chessboard_size: Sequence[int] = (9, 6),
        square_size_mm: float = 25.0,
    ) -> None:
        self._calibration = calibration
        existing = self._config.get("vision.calibration", {})
        payload = dict(existing) if isinstance(existing, dict) else {}
        payload.update(calibration.to_config_dict())
        payload["chessboard_size"] = [int(chessboard_size[0]), int(chessboard_size[1])]
        payload["square_size_mm"] = float(square_size_mm)
        payload["snapshots_captured"] = self.snapshot_count()
        self._config.set("vision.calibration", payload)
        self._config.set("vision.camera_to_base.valid", False)
        try:
            self.save_intrinsics_csv(calibration)
        except Exception:
            pass

    @staticmethod
    def _default_intrinsics_path() -> Path:
        project_root = Path(__file__).resolve().parents[1]
        return project_root / "tools" / "Camera" / "param" / "camera_intrinsic_matrix.csv"

    def save_intrinsics_csv(
        self,
        calibration: CalibrationData | None = None,
        path: Path | str | None = None,
    ) -> Path | None:
        target_calibration = calibration if calibration is not None else self._calibration
        if target_calibration is None or not target_calibration.is_valid:
            return None
        target_path = Path(path) if path is not None else self._default_intrinsics_path()
        target_path.parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(target_path, target_calibration.intrinsic_matrix, delimiter=",", fmt="%.6f")
        return target_path

    def load_intrinsics_csv(self, path: Path | str | None = None) -> CalibrationData | None:
        target_path = Path(path) if path is not None else self._default_intrinsics_path()
        if not target_path.exists():
            return None
        matrix = np.loadtxt(target_path, delimiter=",").astype(float)
        if matrix.shape != (3, 3):
            return None
        existing = self._calibration
        distortion = existing.distortion.copy() if existing is not None else np.zeros((1, 5), dtype=np.float64)
        image_size = existing.image_size if existing is not None else (640, 480)
        rms_error = existing.rms_error if existing is not None else None
        calibration = CalibrationData(
            intrinsic_matrix=matrix,
            distortion=distortion,
            image_size=image_size,
            rms_error=rms_error,
        )
        self._calibration = calibration
        return calibration
