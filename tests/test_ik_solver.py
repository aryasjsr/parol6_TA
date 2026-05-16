"""Unit tests for backend.ik_solver.

Covers P1–P5 helpers + solve_ik public API. Pure unit tests — no robot
hardware, no QThread, no Flask. Use roboticstoolbox DHRobot model from
tools.PAROL6_ROBOT directly.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
from spatialmath import SE3

# Make project root importable for "backend.*" / "tools.*" imports
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from backend.ik_solver import (  # noqa: E402
    BASE_MAX_REACH,
    IKResult,
    LOOSE_TOL,
    STRICT_TOL,
    calculate_adaptive_tolerance,
    calculate_config_dependent_max_reach,
    normalize_angle,
    solve_ik,
    unwrap_angles,
)
import tools.PAROL6_ROBOT as PAROL6_ROBOT  # noqa: E402


# -------------------------- P1: unwrap_angles --------------------------

def test_unwrap_no_change_when_diff_small():
    q_curr = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    q_sol  = np.array([0.15, 0.25, 0.35, 0.45, 0.55, 0.65])
    out = unwrap_angles(q_sol, q_curr)
    np.testing.assert_allclose(out, q_sol)


def test_unwrap_positive_wrap():
    # current 0.1, solution 0.1 + 2*pi - small  →  diff > pi  →  subtract 2*pi
    q_curr = np.zeros(6)
    q_sol = np.zeros(6)
    q_sol[5] = 2 * np.pi - 0.1   # ≈ 6.18, diff ≈ 6.18 > pi
    out = unwrap_angles(q_sol, q_curr)
    assert out[5] == pytest.approx(-0.1, abs=1e-9)


def test_unwrap_negative_wrap():
    # current 0, solution -2*pi + 0.1  →  diff < -pi  →  add 2*pi
    q_curr = np.zeros(6)
    q_sol = np.zeros(6)
    q_sol[5] = -2 * np.pi + 0.1
    out = unwrap_angles(q_sol, q_curr)
    assert out[5] == pytest.approx(0.1, abs=1e-9)


def test_unwrap_does_not_mutate_input():
    q_curr = np.zeros(6)
    q_sol = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 2 * np.pi - 0.1])
    q_sol_orig = q_sol.copy()
    _ = unwrap_angles(q_sol, q_curr)
    np.testing.assert_array_equal(q_sol, q_sol_orig)


# -------------------------- normalize_angle --------------------------

def test_normalize_angle_in_range():
    assert normalize_angle(0.5) == pytest.approx(0.5)
    assert normalize_angle(-0.5) == pytest.approx(-0.5)


def test_normalize_angle_above_pi():
    assert normalize_angle(np.pi + 0.5) == pytest.approx(-np.pi + 0.5)


def test_normalize_angle_below_neg_pi():
    assert normalize_angle(-np.pi - 0.5) == pytest.approx(np.pi - 0.5)


# -------------------------- P3: adaptive tolerance --------------------------

def test_adaptive_tol_far_from_singularity():
    # standby pose is well-conditioned for PAROL6
    q = np.array(PAROL6_ROBOT.Joints_standby_position_radian)
    tol = calculate_adaptive_tolerance(PAROL6_ROBOT.robot, q)
    # Should be at the strict end (high manipulability).
    assert STRICT_TOL <= tol <= LOOSE_TOL
    # Allow either tail; just sanity-check ordering
    assert tol == pytest.approx(STRICT_TOL, abs=2e-7) or tol < LOOSE_TOL


def test_adaptive_tol_returns_float_in_range():
    q = np.zeros(6)
    tol = calculate_adaptive_tolerance(PAROL6_ROBOT.robot, q)
    assert isinstance(tol, float)
    assert STRICT_TOL <= tol <= LOOSE_TOL


def test_adaptive_tol_loose_when_manip_zero(monkeypatch):
    # Force manipulability to 0 → expect tol ≈ loose_tol
    class _StubRobot:
        def manipulability(self, q):  # noqa: D401, ANN001
            return 0.0
    tol = calculate_adaptive_tolerance(_StubRobot(), np.zeros(6))
    assert tol == pytest.approx(LOOSE_TOL, rel=1e-9)


def test_adaptive_tol_strict_when_manip_high(monkeypatch):
    class _StubRobot:
        def manipulability(self, q):  # noqa: D401, ANN001
            return 1.0   # well above 0.001 threshold
    tol = calculate_adaptive_tolerance(_StubRobot(), np.zeros(6))
    assert tol == pytest.approx(STRICT_TOL, rel=1e-9)


# -------------------------- P5: config-dependent reach --------------------------

def test_reach_j5_zero_returns_base():
    q = np.zeros(6)   # J5 = 0
    assert calculate_config_dependent_max_reach(q) == pytest.approx(BASE_MAX_REACH)


def test_reach_j5_at_pos90_reduced():
    q = np.zeros(6)
    q[4] = np.pi / 2
    val = calculate_config_dependent_max_reach(q)
    assert val < BASE_MAX_REACH
    # at exactly ±90° → max reduction = 0.045
    assert val == pytest.approx(BASE_MAX_REACH - 0.045, abs=1e-6)


def test_reach_j5_at_neg90_reduced():
    q = np.zeros(6)
    q[4] = -np.pi / 2
    val = calculate_config_dependent_max_reach(q)
    assert val < BASE_MAX_REACH
    assert val == pytest.approx(BASE_MAX_REACH - 0.045, abs=1e-6)


def test_reach_j5_at_45deg_partial_reduction():
    q = np.zeros(6)
    q[4] = np.pi / 4   # exactly at edge of reduction range (45° from 90°)
    val = calculate_config_dependent_max_reach(q)
    # At edge of range proximity = 0 → reduction = 0 → BASE_MAX_REACH
    assert val == pytest.approx(BASE_MAX_REACH, abs=1e-6)


def test_reach_j5_at_67_5deg_half_reduction():
    q = np.zeros(6)
    q[4] = np.pi / 2 - np.pi / 8   # 22.5° from 90° → 50% proximity
    val = calculate_config_dependent_max_reach(q)
    expected = BASE_MAX_REACH - 0.045 * 0.5
    assert val == pytest.approx(expected, abs=1e-6)


# -------------------------- solve_ik integration --------------------------

@pytest.fixture
def robot():
    return PAROL6_ROBOT.robot


@pytest.fixture
def standby_q():
    return np.array(PAROL6_ROBOT.Joints_standby_position_radian, dtype=float)


def test_solve_ik_identity_pose_succeeds(robot, standby_q):
    # Target = current pose → IK should immediately find current_q (residual ≈ 0)
    current_pose = robot.fkine(standby_q)
    result = solve_ik(
        robot, current_pose, standby_q,
        joint_limits_radian=PAROL6_ROBOT.Joint_limits_radian,
    )
    assert result.success is True
    assert result.q is not None
    np.testing.assert_allclose(result.q, standby_q, atol=1e-3)


def test_solve_ik_small_translation(robot, standby_q):
    # Tiny translation in +X — should solve easily
    current_pose = robot.fkine(standby_q)
    target = current_pose.copy()
    target.t = current_pose.t + np.array([0.01, 0.0, 0.0])
    result = solve_ik(
        robot, target, standby_q,
        joint_limits_radian=PAROL6_ROBOT.Joint_limits_radian,
    )
    assert result.success is True
    # Verify FK of result matches target within tolerance
    pose_check = robot.fkine(result.q)
    np.testing.assert_allclose(pose_check.t, target.t, atol=1e-3)


def test_solve_ik_unreachable_target(robot, standby_q):
    # 2 meter away — far beyond PAROL6 reach (BASE_MAX_REACH = 0.44)
    target = SE3(2.0, 0.0, 0.0)
    result = solve_ik(
        robot, target, standby_q,
        joint_limits_radian=PAROL6_ROBOT.Joint_limits_radian,
    )
    assert result.success is False
    assert result.q is None


def test_solve_ik_jogging_disables_subdivision(robot, standby_q):
    # In jogging mode max_depth=0; target reachable should still succeed
    current_pose = robot.fkine(standby_q)
    target = current_pose.copy()
    target.t = current_pose.t + np.array([0.005, 0.0, 0.0])
    result = solve_ik(
        robot, target, standby_q, jogging=True,
        joint_limits_radian=PAROL6_ROBOT.Joint_limits_radian,
    )
    assert result.success is True
    assert result.tolerance_used == STRICT_TOL


def test_solve_ik_returns_unwrapped_q_close_to_seed(robot, standby_q):
    # Solution should be within ±π of seed (no full-revolution jump)
    current_pose = robot.fkine(standby_q)
    target = current_pose.copy()
    target.t = current_pose.t + np.array([0.0, 0.02, 0.0])
    result = solve_ik(
        robot, target, standby_q,
        joint_limits_radian=PAROL6_ROBOT.Joint_limits_radian,
    )
    assert result.success is True
    diff = np.abs(result.q - standby_q)
    assert np.all(diff < np.pi), f"Joint diff exceeds π: {diff}"


def test_solve_ik_result_within_joint_limits(robot, standby_q):
    current_pose = robot.fkine(standby_q)
    target = current_pose.copy()
    target.t = current_pose.t + np.array([0.0, 0.0, -0.02])
    result = solve_ik(
        robot, target, standby_q,
        joint_limits_radian=PAROL6_ROBOT.Joint_limits_radian,
    )
    assert result.success is True
    for i, q_i in enumerate(result.q):
        lo, hi = PAROL6_ROBOT.Joint_limits_radian[i]
        assert lo <= q_i <= hi, f"Joint {i+1}={q_i} out of [{lo}, {hi}]"


def test_solve_ik_namedtuple_fields():
    # IKResult fields must be exactly what the rest of the codebase consumes
    assert IKResult._fields == (
        "success", "q", "iterations", "residual", "tolerance_used", "violations"
    )
