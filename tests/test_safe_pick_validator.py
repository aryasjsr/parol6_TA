"""Regression tests for SafePickValidator.

Documents the bug fixed in this commit: the old per-side intersection
algorithm produced an empty safe zone whenever a fixture overlapped the
selongsong center, then fell back to a margin around the selongsong centroid
that was *inside* the fixture. The pickup point therefore "stacked" on the
fixture and the robot would crash into it.

The new algorithm picks the largest sub-rectangle of selongsong on one side
of fixture (LEFT / RIGHT / TOP / BOTTOM) and places the pickup at its
center, guaranteeing the pick lies strictly outside the fixture bbox.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

# Allow import without installing the package
_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from vision.safe_pick_validator import SafePickValidator  # noqa: E402


def _point_in_box(pt: tuple[int, int], box: tuple[float, float, float, float]) -> bool:
    px, py = pt
    x1, y1, x2, y2 = box
    return x1 <= px <= x2 and y1 <= py <= y2


@pytest.fixture
def validator() -> SafePickValidator:
    """Validator with margin_pct passed explicitly per call (no config dep)."""
    # Patch instance to bypass ConfigManager.instance() in tests.
    v = SafePickValidator.__new__(SafePickValidator)
    v._config = None  # type: ignore[attr-defined]
    return v


# ---------------------------------------------------------------------------
# Bug repro: horizontal pipe (selongsong) crossed by vertical fixture
# ---------------------------------------------------------------------------

def test_horizontal_pipe_with_central_vertical_fixture(validator):
    """Reproduces the exact failure mode shown in the user's overlay image.

    Selongsong is a horizontal pipe; fixture is a vertical jig that overlaps
    the pipe in the middle. Pickup MUST lie outside the fixture bbox.
    """
    sel = np.array([60, 200, 520, 290], dtype=float)   # horizontal pipe
    fix = np.array([180, 100, 440, 470], dtype=float)  # vertical jig

    status, pick = validator.check_pair(sel, fix, margin_pct=0.15)

    assert pick is not None
    assert status in {"SAFE", "MARGINAL"}
    # CRUCIAL: pickup must NOT be inside fixture bbox (regression check)
    assert not _point_in_box(pick, tuple(fix)), (
        f"Pickup {pick} fell inside fixture {tuple(fix)} — bug regression!"
    )
    # And must lie within the selongsong bbox
    assert _point_in_box(pick, tuple(sel))


def test_pick_point_on_largest_safe_side(validator):
    """When fixture is off-center, pickup goes to the larger safe side."""
    # Selongsong 0..400, fixture 100..160 → LEFT region (0..100, w=100) <
    # RIGHT region (160..400, w=240); pickup must land on RIGHT.
    sel = np.array([0, 100, 400, 200], dtype=float)
    fix = np.array([100, 80, 160, 220], dtype=float)

    status, pick = validator.check_pair(sel, fix, margin_pct=0.15)

    assert pick is not None
    assert status == "SAFE"
    px, _ = pick
    assert px > 160, f"Expected pick on RIGHT side (>160), got {pick}"
    assert not _point_in_box(pick, tuple(fix))


def test_no_fixture_returns_unknown_at_centroid(validator):
    sel = np.array([0, 0, 100, 50], dtype=float)
    status, pick = validator.check_pair(sel, None, margin_pct=0.15)
    assert status == "UNKNOWN"
    assert pick == (50, 25)


def test_fixture_completely_covers_selongsong_is_unsafe(validator):
    sel = np.array([100, 100, 200, 150], dtype=float)
    fix = np.array([0, 0, 400, 400], dtype=float)  # fully envelops sel
    status, pick = validator.check_pair(sel, fix, margin_pct=0.15)
    assert status == "UNSAFE"
    assert pick is None


def test_fixture_disjoint_from_selongsong_is_safe(validator):
    sel = np.array([0, 0, 100, 50], dtype=float)
    fix = np.array([200, 200, 300, 250], dtype=float)  # far away
    status, pick = validator.check_pair(sel, fix, margin_pct=0.15)
    assert status == "SAFE"
    assert pick is not None
    assert _point_in_box(pick, tuple(sel))
    assert not _point_in_box(pick, tuple(fix))


def test_fixture_touching_left_edge_only(validator):
    """Fixture overlaps left side; safe pickup is on the right of fixture."""
    sel = np.array([0, 0, 200, 100], dtype=float)
    fix = np.array([0, 0, 80, 100], dtype=float)  # overlaps left half
    status, pick = validator.check_pair(sel, fix, margin_pct=0.15)
    assert pick is not None
    assert status == "SAFE"
    px, _ = pick
    assert px > 80
    assert not _point_in_box(pick, tuple(fix))


def test_marginal_when_safe_zone_narrow(validator):
    """Tiny remaining strip should be flagged MARGINAL, not SAFE."""
    # selongsong 100x100, fixture covers 0..85 → safe strip width 15 (15%)
    sel = np.array([0, 0, 100, 100], dtype=float)
    fix = np.array([0, 0, 85, 100], dtype=float)
    status, pick = validator.check_pair(sel, fix, margin_pct=0.15)
    assert pick is not None
    assert status == "MARGINAL", f"expected MARGINAL, got {status}"
    assert not _point_in_box(pick, tuple(fix))
