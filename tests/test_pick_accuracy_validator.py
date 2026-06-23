from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from vision.pick_accuracy_validator import PickAccuracyValidator, PickTrial  # noqa: E402


class FakeConfig:
    def __init__(self, enabled: bool = True, threshold_mm: float = 3.0):
        self.data = {
            "vision": {
                "safe_pick_margin_pct": 0.25,
                "safe_pick_left_offset_px": 0.0,
                "pick_accuracy_test": {
                    "enabled": enabled,
                    "threshold_mm": threshold_mm,
                    "log_path": "logs/pick_accuracy_trials.json",
                },
                "camera_to_base": {
                    "valid": True,
                    "homography": [
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0],
                    ],
                    "image_size": [640, 480],
                },
            }
        }
        self.config_path = _project_root / "config.json"

    def get(self, key, default=None):
        value = self.data
        for part in key.split("."):
            if not isinstance(value, dict) or part not in value:
                return default
            value = value[part]
        return value


# Selongsong left of fixture so the safe zone exists.
SELONGSONG = [100.0, 100.0, 200.0, 180.0]
FIXTURE = [170.0, 90.0, 210.0, 190.0]
INTRINSIC = np.eye(3)


def _validator(**kwargs) -> PickAccuracyValidator:
    return PickAccuracyValidator(FakeConfig(**kwargs))


def test_disabled_by_default_blocks_collection():
    validator = _validator(enabled=False)
    assert validator.is_enabled() is False
    with pytest.raises(RuntimeError):
        validator.build_trial(
            "t0",
            detected_selongsong_box=SELONGSONG,
            detected_fixture_box=FIXTURE,
            ground_truth_selongsong_box=SELONGSONG,
            ground_truth_fixture_box=FIXTURE,
            intrinsic_matrix=INTRINSIC,
            frame_size=(640, 480),
        )


def test_perfect_detection_has_zero_selection_error():
    validator = _validator()
    trial = validator.build_trial(
        "t1",
        detected_selongsong_box=SELONGSONG,
        detected_fixture_box=FIXTURE,
        ground_truth_selongsong_box=SELONGSONG,
        ground_truth_fixture_box=FIXTURE,
        intrinsic_matrix=INTRINSIC,
        frame_size=(640, 480),
        measured_offset_mm=(2.0, 1.0),
    )
    assert trial.detected_pick_px is not None
    assert trial.selection_error_mm == pytest.approx(0.0)
    assert trial.status_match is True
    assert trial.landing_offset_mm == pytest.approx(math.hypot(2.0, 1.0))


def test_detection_shift_produces_selection_error():
    validator = _validator()
    shifted = [s + 12.0 for s in SELONGSONG]  # detector mis-located the box
    trial = validator.build_trial(
        "t2",
        detected_selongsong_box=shifted,
        detected_fixture_box=FIXTURE,
        ground_truth_selongsong_box=SELONGSONG,
        ground_truth_fixture_box=FIXTURE,
        intrinsic_matrix=INTRINSIC,
        frame_size=(640, 480),
    )
    assert (trial.selection_error_mm or 0.0) > 0.0


def test_analyze_success_rate_and_report():
    validator = _validator(threshold_mm=3.0)
    trials = [
        PickTrial("a", measured_offset_mm=(2.0, 1.0), detected_status="SAFE", ideal_status="SAFE"),
        PickTrial("b", measured_offset_mm=(4.0, 3.0), detected_status="SAFE", ideal_status="SAFE"),
    ]
    report = validator.analyze(trials)
    assert report.measured_trial_count == 2
    assert report.success_count == 1  # 2.23mm passes, 5.0mm fails
    assert report.success_rate_pct == pytest.approx(50.0)
    assert report.status_accuracy_pct == pytest.approx(100.0)
    assert "Test B" in validator.format_report(report)


def test_build_trial_1d_landing_error():
    validator = _validator()
    trial = validator.build_trial(
        "t1d",
        detected_selongsong_box=SELONGSONG,
        detected_fixture_box=FIXTURE,
        ground_truth_selongsong_box=SELONGSONG,
        ground_truth_fixture_box=FIXTURE,
        intrinsic_matrix=INTRINSIC,
        frame_size=(640, 480),
        measured_distance_mm=42.0,
    )
    # tip datum = left edge at vertical centre of the selongsong box
    assert trial.ref_point_px == (100.0, 140.0)
    assert trial.reference_distance_mm is not None and trial.reference_distance_mm > 0.0
    expected_signed = 42.0 - trial.reference_distance_mm
    assert trial.landing_error_signed_mm == pytest.approx(expected_signed)
    assert trial.landing_offset_mm == pytest.approx(abs(expected_signed))


def test_reference_distance_helper_not_gated():
    # pure geometry helper works even when the feature is disabled
    validator = _validator(enabled=False)
    pick_base = (160.0, 140.0)
    dist = validator.reference_distance(SELONGSONG, pick_base, INTRINSIC, frame_size=(640, 480))
    # identity homography: tip=(100,140) -> pick=(160,140) -> 60mm
    assert dist == pytest.approx(60.0)


def test_1d_roundtrip_preserves_distances(tmp_path):
    validator = _validator()
    trials = [validator.build_trial(
        "a",
        detected_selongsong_box=SELONGSONG,
        detected_fixture_box=FIXTURE,
        ground_truth_selongsong_box=SELONGSONG,
        ground_truth_fixture_box=FIXTURE,
        intrinsic_matrix=INTRINSIC,
        frame_size=(640, 480),
        measured_distance_mm=30.0,
    )]
    out = tmp_path / "trials.json"
    validator.save_trials(trials, out)
    loaded = validator.load_trials(out)
    assert loaded[0].measured_distance_mm == pytest.approx(30.0)
    assert loaded[0].reference_distance_mm == pytest.approx(trials[0].reference_distance_mm, abs=1e-3)
    assert loaded[0].landing_offset_mm == pytest.approx(trials[0].landing_offset_mm, abs=1e-3)


def test_save_and_load_roundtrip(tmp_path):
    validator = _validator()
    trials = [PickTrial("a", measured_offset_mm=(1.0, 1.0), detected_status="SAFE", ideal_status="SAFE")]
    out = tmp_path / "trials.json"
    validator.save_trials(trials, out)
    loaded = validator.load_trials(out)
    assert len(loaded) == 1
    assert loaded[0].trial_id == "a"
    assert loaded[0].measured_offset_mm == (1.0, 1.0)
