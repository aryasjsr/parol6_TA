from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from vision.safe_pick_validator import SafePickValidator  # noqa: E402


@pytest.fixture
def validator() -> SafePickValidator:
    instance = SafePickValidator.__new__(SafePickValidator)
    instance._config = None  # type: ignore[attr-defined]
    return instance


def test_pick_is_always_left_of_fixture(validator):
    selongsong = np.array([0, 100, 400, 200], dtype=float)
    fixture = np.array([100, 80, 160, 220], dtype=float)

    status, pick = validator.check_pair(selongsong, fixture, margin_pct=0.25)

    assert status == "SAFE"
    assert pick == (50, 150)
    assert pick[0] < fixture[0]


def test_does_not_switch_to_larger_right_region(validator):
    selongsong = np.array([0, 100, 400, 200], dtype=float)
    fixture = np.array([100, 80, 160, 220], dtype=float)

    _, pick = validator.check_pair(selongsong, fixture, margin_pct=0.25)

    assert pick is not None
    assert pick[0] < fixture[0]


def test_no_fixture_is_unknown_centroid(validator):
    status, pick = validator.check_pair(
        np.array([0, 0, 100, 50], dtype=float),
        None,
        margin_pct=0.25,
    )

    assert status == "UNKNOWN"
    assert pick == (50, 25)


def test_no_left_region_is_unsafe(validator):
    selongsong = np.array([100, 100, 200, 150], dtype=float)
    fixture = np.array([0, 0, 150, 400], dtype=float)

    status, pick = validator.check_pair(selongsong, fixture, margin_pct=0.25)

    assert status == "UNSAFE"
    assert pick is None


def test_fixture_to_right_uses_full_selongsong(validator):
    selongsong = np.array([0, 0, 100, 50], dtype=float)
    fixture = np.array([200, 0, 300, 50], dtype=float)

    status, pick = validator.check_pair(selongsong, fixture, margin_pct=0.25)

    assert status == "SAFE"
    assert pick == (50, 25)


def test_left_zone_below_one_margin_is_unsafe(validator):
    selongsong = np.array([0, 0, 100, 100], dtype=float)
    fixture = np.array([20, 0, 80, 100], dtype=float)

    status, pick = validator.check_pair(selongsong, fixture, margin_pct=0.25)

    assert status == "UNSAFE"
    assert pick is None


def test_left_zone_between_one_and_two_margins_is_marginal(validator):
    selongsong = np.array([0, 0, 100, 100], dtype=float)
    fixture = np.array([40, 0, 80, 100], dtype=float)

    status, pick = validator.check_pair(selongsong, fixture, margin_pct=0.25)

    assert status == "MARGINAL"
    assert pick == (20, 50)


def test_custom_left_offset_moves_pick_toward_image_left(validator):
    selongsong = np.array([0, 0, 200, 100], dtype=float)
    fixture = np.array([160, 0, 200, 100], dtype=float)

    status, pick = validator.check_pair(
        selongsong,
        fixture,
        margin_pct=0.25,
        left_offset_px=30,
    )

    assert status == "SAFE"
    assert pick == (50, 50)


def test_custom_left_offset_is_clamped_inside_safe_clearance(validator):
    selongsong = np.array([0, 0, 200, 100], dtype=float)
    fixture = np.array([160, 0, 200, 100], dtype=float)

    status, pick = validator.check_pair(
        selongsong,
        fixture,
        margin_pct=0.25,
        left_offset_px=1000,
    )

    assert status == "SAFE"
    assert pick == (12, 50)
    assert pick[0] < fixture[0]
