"""
ik_solver.py — Enhanced IK solver wrapping roboticstoolbox.ikine_LMS.

Adopted from PAROL-commander-software branch `experimental_kinematics`
(file: GUI/files/Serial_sender_good_latest.py).

Features:
  P1 — Angle unwrapping (eliminates joint sign flips)
  P2 — Higher ilimit + explicit damping
  P3 — Adaptive tolerance based on proximity to kinematic singularities
  P4 — Recursive path subdivision when ikine_LMS fails
  P5 — Configuration-dependent reach check (Joint 5 dependency)

Public API:
  - IKResult                                 namedtuple
  - normalize_angle(angle)                   helper
  - unwrap_angles(q_solution, q_current)     post-process IK output
  - calculate_adaptive_tolerance(robot, q)   tolerance selector
  - calculate_config_dependent_max_reach(q_seed)
  - solve_ik(robot, target_pose, current_q, ...) -> IKResult

This module has NO PyQt6 dependency and NO I/O. All functions are pure
(or only call read-only methods of the robot model). Safe to import from
any layer.
"""

from __future__ import annotations

from collections import namedtuple

import numpy as np
from spatialmath import SE3
from spatialmath.base import trinterp


IKResult = namedtuple(
    "IKResult",
    ["success", "q", "iterations", "residual", "tolerance_used", "violations"],
)


# === Constants (calibrated from paro-commander-software experiments) ===
# NOTE: BASE_MAX_REACH and J5_REACH_REDUCTION are empirical values that may
# need re-calibration for the specific PAROL6 unit used in this TA.
# See PLAN_experimental.md §5 / tasks_experimental.md E5.3-E5.4.
BASE_MAX_REACH = 0.44                # meters
J5_REACH_REDUCTION = 0.045           # meters reduction at J5 = ±90°
J5_REDUCTION_RANGE = np.pi / 4       # 45° band around ±90°

# Default damping for ikine_LMS Levenberg-Marquardt step.
# Very small value → behaves close to Gauss-Newton, faster convergence
# when far from singularity. Adaptive tol handles singularity case instead.
DEFAULT_DAMPING = 1e-7

# Singularity detection threshold (manipulability index).
# Below this value → considered "near singular" → use looser tolerance.
SINGULARITY_THRESHOLD = 0.001

# Adaptive tolerance bounds.
STRICT_TOL = 1e-10                   # used when far from singularity
LOOSE_TOL = 1e-7                     # used when near singularity

# Default ilimit values (max iterations of ikine_LMS).
ILIMIT_MOTION = 100                  # MoveCart, MovePose, vision coarse pick
ILIMIT_JOG = 20                      # cartesian_jog (IBVS correction)


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi] range to handle angle wrapping."""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def unwrap_angles(q_solution: np.ndarray, q_current: np.ndarray) -> np.ndarray:
    """Unwrap joint solution to be closest to current position.

    IK solver may return e.g. -179° when +181° is the geometrically equivalent
    solution closer to the current joint state. Without unwrapping, executing
    the raw solution would cause the joint to spin nearly a full revolution
    (sign flip). This helper rewraps each joint to within ±π of the current.
    """
    q_unwrapped = np.asarray(q_solution, dtype=float).copy()
    q_curr = np.asarray(q_current, dtype=float)
    for i in range(len(q_unwrapped)):
        diff = q_unwrapped[i] - q_curr[i]
        if diff > np.pi:
            q_unwrapped[i] -= 2 * np.pi
        elif diff < -np.pi:
            q_unwrapped[i] += 2 * np.pi
    return q_unwrapped


def calculate_adaptive_tolerance(
    robot,
    q,
    strict_tol: float = STRICT_TOL,
    loose_tol: float = LOOSE_TOL,
) -> float:
    """Calculate adaptive tolerance based on proximity to singularities.

    Near singular configurations the Jacobian is ill-conditioned and a strict
    tolerance can prevent ikine_LMS from converging. Use a looser tolerance
    near singularities, stricter when manipulability is high.
    """
    q_array = np.asarray(q, dtype=float)
    manip = float(robot.manipulability(q_array))
    sing_normalized = float(np.clip(manip / SINGULARITY_THRESHOLD, 0.0, 1.0))
    return loose_tol + (strict_tol - loose_tol) * sing_normalized


def calculate_config_dependent_max_reach(q_seed: np.ndarray) -> float:
    """Maximum reach (meters) accounting for J5 configuration.

    When Joint 5 approaches ±90°, the wrist offset effectively reduces the
    maximum reachable distance from the base. This returns a reach threshold
    that should be used as an upper bound before invoking IK.
    """
    q_array = np.asarray(q_seed, dtype=float)
    j5 = q_array[4] if q_array.size > 4 else 0.0
    j5_norm = normalize_angle(float(j5))
    dist_from_pos90 = abs(j5_norm - np.pi / 2)
    dist_from_neg90 = abs(j5_norm + np.pi / 2)
    dist_from_90 = min(dist_from_pos90, dist_from_neg90)

    if dist_from_90 <= J5_REDUCTION_RANGE:
        proximity = 1.0 - (dist_from_90 / J5_REDUCTION_RANGE)
        return BASE_MAX_REACH - J5_REACH_REDUCTION * proximity
    return BASE_MAX_REACH


def _check_joint_limits(
    q: np.ndarray,
    joint_limits_radian: list[list[float]],
) -> tuple[bool, dict]:
    """Check whether each joint of `q` is within its `[lo, hi]` limits.

    Returns (all_valid, violations_dict). `violations_dict` keyed by joint
    index (1-based) only contains entries for out-of-limit joints.
    """
    q_arr = np.asarray(q, dtype=float)
    violations: dict[str, dict] = {}
    all_valid = True
    n = min(q_arr.size, len(joint_limits_radian))
    for i in range(n):
        lo, hi = joint_limits_radian[i]
        if q_arr[i] < lo or q_arr[i] > hi:
            all_valid = False
            violations[f"joint_{i+1}"] = {
                "value": float(q_arr[i]),
                "min_limit": float(lo),
                "max_limit": float(hi),
                "violation": "below_min" if q_arr[i] < lo else "above_max",
            }
    return all_valid, violations


def _solve_recursive(
    robot,
    Ta: SE3,
    Tb: SE3,
    q_seed: np.ndarray,
    depth: int,
    max_depth: int,
    ilimit: int,
    tol: float,
    damping: float,
    enforce_reach: bool,
) -> tuple[list[np.ndarray], bool, int, float]:
    """Inner recursive IK solver with path subdivision.

    Returns (path, success, iterations, residual). `path` is a list of joint
    configurations (one per converged sub-segment). Final converged q is
    `path[-1]` if success.
    """
    if enforce_reach:
        target_reach = float(np.linalg.norm(Tb.t))
        max_reach_threshold = calculate_config_dependent_max_reach(q_seed)
        if target_reach >= max_reach_threshold:
            return [], False, 0, float("inf")

    res = robot.ikine_LMS(Tb, q0=q_seed, ilimit=ilimit, tol=tol, wN=damping)
    if getattr(res, "success", False):
        q_good = unwrap_angles(np.asarray(res.q, dtype=float), q_seed)
        return [q_good], True, int(res.iterations), float(res.residual)

    if depth >= max_depth:
        return [], False, int(res.iterations), float(res.residual)

    Tc = SE3(trinterp(Ta.A, Tb.A, 0.5))

    left_path, ok_L, it_L, r_L = _solve_recursive(
        robot, Ta, Tc, q_seed, depth + 1,
        max_depth, ilimit, tol, damping, enforce_reach,
    )
    if not ok_L:
        return [], False, it_L, r_L

    q_mid = left_path[-1]
    right_path, ok_R, it_R, r_R = _solve_recursive(
        robot, Tc, Tb, q_mid, depth + 1,
        max_depth, ilimit, tol, damping, enforce_reach,
    )
    return left_path + right_path, ok_R, it_L + it_R, r_R


def solve_ik(
    robot,
    target_pose: SE3,
    current_q: np.ndarray,
    current_pose: SE3 | None = None,
    jogging: bool = False,
    max_depth: int = 4,
    ilimit: int | None = None,
    joint_limits_radian: list[list[float]] | None = None,
    enforce_reach: bool = True,
    damping: float = DEFAULT_DAMPING,
) -> IKResult:
    """Solve IK for `target_pose` given `current_q`, with subdivision + unwrap.

    Args:
        robot: roboticstoolbox DHRobot model (PAROL6_ROBOT.robot)
        target_pose: SE3 target pose
        current_q: current joint angles in radians (length 6)
        current_pose: SE3 current pose. If None, computed from FK.
        jogging: True → no subdivision, fixed strict tolerance, lower ilimit.
                 Use this for IBVS / cartesian jog where latency matters.
                 False → adaptive tolerance + subdivision, used for motion.
        max_depth: max subdivision depth (only when jogging=False)
        ilimit: max ikine_LMS iterations. Defaults to ILIMIT_MOTION (motion)
                or ILIMIT_JOG (jogging).
        joint_limits_radian: list of [lo, hi] per joint (radian). If provided,
                             post-IK validation rejects out-of-limit solutions.
        enforce_reach: if True, reject targets beyond
                       calculate_config_dependent_max_reach(q_seed) before
                       calling ikine_LMS. Set False to skip (e.g., when
                       a higher-level workspace check already validated reach).
        damping: ikine_LMS wN parameter (Levenberg-Marquardt damping factor).

    Returns:
        IKResult(success, q, iterations, residual, tolerance_used, violations)
        On failure, q is None.
    """
    current_q = np.asarray(current_q, dtype=float)

    if current_pose is None:
        current_pose = robot.fkine(current_q)

    if jogging:
        tol = STRICT_TOL
        eff_ilimit = ilimit if ilimit is not None else ILIMIT_JOG
        eff_max_depth = 0
    else:
        tol = calculate_adaptive_tolerance(robot, current_q)
        eff_ilimit = ilimit if ilimit is not None else ILIMIT_MOTION
        eff_max_depth = max_depth

    path, ok, its, resid = _solve_recursive(
        robot,
        current_pose,
        target_pose,
        current_q,
        depth=0,
        max_depth=eff_max_depth,
        ilimit=eff_ilimit,
        tol=tol,
        damping=damping,
        enforce_reach=enforce_reach,
    )

    if not ok or len(path) == 0:
        return IKResult(False, None, its, resid, tol, {})

    q_final = path[-1]

    violations: dict = {}
    if joint_limits_radian is not None:
        within, violations = _check_joint_limits(q_final, joint_limits_radian)
        if not within:
            return IKResult(False, None, its, resid, tol, violations)

    return IKResult(True, q_final, its, resid, tol, violations)
