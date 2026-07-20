from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from vision.workspace_validator import WorkspaceValidator  # noqa: E402


class FakeConfig:
    def __init__(self, vision_extra: dict | None = None, calibration_extra: dict | None = None):
        calibration = {
            "valid": True,
            "homography": [
                [2.0, 0.0, 10.0],
                [0.0, 3.0, 20.0],
                [0.0, 0.0, 1.0],
            ],
            "image_size": [640, 480],
        }
        calibration.update(calibration_extra or {})
        vision = {"camera_to_base": calibration}
        vision.update(vision_extra or {})
        self.data = {"vision": vision}

    def get(self, key, default=None):
        value = self.data
        for part in key.split("."):
            if not isinstance(value, dict) or part not in value:
                return default
            value = value[part]
        return value


def test_pixel_to_base_uses_homography_and_frame_guard():
    validator = WorkspaceValidator(FakeConfig())
    intrinsic = np.eye(3)

    base = validator.pixel_to_base(
        5.0,
        7.0,
        intrinsic,
        np.zeros((1, 5)),
        (640, 480),
    )

    assert base == (20.0, 41.0)
    assert not validator.has_camera_to_base_calibration((1280, 720))


def test_calibration_applicability_checks_zoom_not_source():
    recorded = {"video_source": "2", "zoom": 1.0}

    matching = WorkspaceValidator(
        FakeConfig({"video_source": "2", "zoom": 1.0}, recorded)
    )
    assert matching.has_camera_to_base_calibration((640, 480))

    # OS camera indices shift across reboots/replugs, so a differing
    # video_source must NOT invalidate the homography
    other_source = WorkspaceValidator(
        FakeConfig({"video_source": "0", "zoom": 1.0}, recorded)
    )
    assert other_source.has_camera_to_base_calibration((640, 480))

    other_zoom = WorkspaceValidator(
        FakeConfig({"video_source": "2", "zoom": 1.5}, recorded)
    )
    assert not other_zoom.has_camera_to_base_calibration((640, 480))

    # records solved before zoom was stored skip that check
    legacy = WorkspaceValidator(FakeConfig({"zoom": 1.5}))
    assert legacy.has_camera_to_base_calibration((640, 480))
