from __future__ import annotations

from dataclasses import dataclass
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
    def __init__(self, config: ConfigManager | None = None) -> None:
        self._config = config or ConfigManager.instance()
        self._snapshots: list[np.ndarray] = []
        self._calibration = CalibrationData.from_config(self._config.get("vision.calibration"))

    def add_snapshot(self, frame: np.ndarray) -> int:
        self._snapshots.append(frame.copy())
        self._config.set("vision.calibration.snapshots_captured", len(self._snapshots))
        return len(self._snapshots)

    def clear_snapshots(self) -> None:
        self._snapshots.clear()
        self._config.set("vision.calibration.snapshots_captured", 0)

    def snapshot_count(self) -> int:
        return len(self._snapshots)

    def has_calibration(self) -> bool:
        return self._calibration is not None and self._calibration.is_valid

    def current(self) -> CalibrationData | None:
        return self._calibration

    def current_summary(self) -> dict:
        if self._calibration is None:
            return {
                "fx": 0.0,
                "fy": 0.0,
                "cx": 0.0,
                "cy": 0.0,
                "distortion": [],
                "image_size": [640, 480],
                "rms_error": None,
                "snapshots_captured": self.snapshot_count(),
                "calibrated": False,
            }
        summary = self._calibration.summary()
        summary["snapshots_captured"] = self.snapshot_count()
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
        pattern_size = (int(chessboard_size[0]), int(chessboard_size[1]))
        valid_frames: list[np.ndarray] = []
        object_points: list[np.ndarray] = []
        image_points: list[np.ndarray] = []

        base_object_points = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        base_object_points[:, :2] = np.mgrid[0 : pattern_size[0], 0 : pattern_size[1]].T.reshape(-1, 2)
        base_object_points *= float(square_size_mm)

        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )

        for frame in self._snapshots:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, pattern_size, None)
            if not found:
                continue
            refined = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                criteria,
            )
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

    def save_calibration(
        self,
        calibration: CalibrationData,
        chessboard_size: Sequence[int] = (9, 6),
        square_size_mm: float = 25.0,
    ) -> None:
        self._calibration = calibration
        payload = calibration.to_config_dict()
        payload["chessboard_size"] = [int(chessboard_size[0]), int(chessboard_size[1])]
        payload["square_size_mm"] = float(square_size_mm)
        payload["snapshots_captured"] = self.snapshot_count()
        self._config.set("vision.calibration", payload)
