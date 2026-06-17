from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from backend.config_manager import ConfigManager  # noqa: E402
from vision.camera_calibration import CameraCalibrationManager  # noqa: E402


def _chessboard_frame(pattern_size: tuple[int, int] = (8, 6)) -> np.ndarray:
    columns, rows = pattern_size
    square_px = 48
    margin_px = 32
    frame = np.full(
        (
            (rows + 1) * square_px + margin_px * 2,
            (columns + 1) * square_px + margin_px * 2,
            3,
        ),
        255,
        dtype=np.uint8,
    )
    for row in range(rows + 1):
        for column in range(columns + 1):
            if (row + column) % 2 == 0:
                top_left = (
                    margin_px + column * square_px,
                    margin_px + row * square_px,
                )
                bottom_right = (
                    margin_px + (column + 1) * square_px,
                    margin_px + (row + 1) * square_px,
                )
                cv2.rectangle(frame, top_left, bottom_right, (0, 0, 0), -1)
    return frame


def test_detect_and_draw_chessboard_corners():
    frame = _chessboard_frame()

    found, corners = CameraCalibrationManager.find_chessboard_corners(
        frame,
        (8, 6),
    )
    overlay = CameraCalibrationManager.draw_chessboard_corners(
        frame,
        (8, 6),
        corners,
        found,
    )

    assert found
    assert corners is not None
    assert len(corners) == 48
    assert np.any(overlay != frame)


def test_snapshot_is_saved_and_reported(tmp_path):
    config = ConfigManager(tmp_path / "config.json")
    config.set(
        "vision.calibration",
        {
            "chessboard_size": [8, 6],
            "square_size_mm": 53.0,
        },
    )
    manager = CameraCalibrationManager(config, snapshot_dir=tmp_path / "snapshots")

    count = manager.add_snapshot(_chessboard_frame())
    summary = manager.current_summary()
    saved_path = Path(summary["last_snapshot_path"])

    assert count == 1
    assert saved_path.exists()
    assert saved_path.parent == tmp_path / "snapshots"
    assert summary["last_snapshot_valid"] is True


def test_reset_intrinsic_removes_outputs_and_preserves_board_settings(tmp_path):
    config = ConfigManager(tmp_path / "config.json")
    config.set(
        "vision.calibration",
        {
            "chessboard_size": [8, 6],
            "square_size_mm": 53.0,
            "preview_enabled": True,
            "snapshot_dir": "snapshots",
            "intrinsic_matrix": np.eye(3).tolist(),
            "distortion": [0.1, 0.0, 0.0, 0.0, 0.0],
            "image_size": [640, 480],
            "rms_error": 0.4,
        },
    )
    config.set(
        "vision.camera_to_base",
        {
            "valid": True,
            "homography": np.eye(3).tolist(),
            "point_count": 9,
        },
    )
    snapshot_dir = tmp_path / "snapshots"
    intrinsics_path = tmp_path / "camera_intrinsic_matrix.csv"
    manager = CameraCalibrationManager(config, snapshot_dir=snapshot_dir)
    manager.add_snapshot(_chessboard_frame())
    np.savetxt(intrinsics_path, np.eye(3), delimiter=",")

    summary = manager.reset_intrinsic_calibration(
        delete_snapshot_files=True,
        intrinsics_path=intrinsics_path,
    )
    calibration_cfg = config.get("vision.calibration")
    camera_to_base = config.get("vision.camera_to_base")

    assert summary["calibrated"] is False
    assert summary["deleted_snapshots"] == 1
    assert summary["intrinsics_csv_deleted"] is True
    assert not list(snapshot_dir.glob("intrinsic_*.jpg"))
    assert not intrinsics_path.exists()
    assert manager.current() is None
    assert calibration_cfg["chessboard_size"] == [8, 6]
    assert calibration_cfg["square_size_mm"] == 53.0
    assert calibration_cfg["preview_enabled"] is True
    assert "intrinsic_matrix" not in calibration_cfg
    assert "distortion" not in calibration_cfg
    assert camera_to_base["valid"] is False
    assert camera_to_base["homography"] == []
