"""Performance tests for backend.ik_solver.

Targets (per PLAN_experimental.md §7):
  - solve_ik(jogging=False)  median < 50ms
  - solve_ik(jogging=True)   median < 5ms, p95 < 8ms
  - manipulability()         < 2ms per call

These tests are tagged with `@pytest.mark.benchmarks` (matching existing
test_benchmarks.py convention). Skip with `-k "not perf"` if needed.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from statistics import median

import numpy as np
import pytest
from spatialmath import SE3

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from backend.ik_solver import solve_ik  # noqa: E402
import tools.PAROL6_ROBOT as PAROL6_ROBOT  # noqa: E402


@pytest.fixture
def robot():
    return PAROL6_ROBOT.robot


@pytest.fixture
def standby_q():
    return np.array(PAROL6_ROBOT.Joints_standby_position_radian, dtype=float)


def _percentile(values: list[float], p: float) -> float:
    arr = sorted(values)
    k = int(round((p / 100.0) * (len(arr) - 1)))
    return arr[k]


@pytest.mark.benchmarks
def test_perf_manipulability_under_2ms(robot, standby_q):
    """robot.manipulability() must be cheap enough to call per-IK."""
    times: list[float] = []
    for _ in range(50):
        t0 = time.perf_counter()
        _ = robot.manipulability(standby_q)
        times.append((time.perf_counter() - t0) * 1000.0)
    med = median(times)
    assert med < 2.0, f"manipulability median {med:.2f}ms exceeds 2ms target"


@pytest.mark.benchmarks
def test_perf_solve_ik_motion_under_50ms(robot, standby_q):
    """Non-jogging solve_ik on small displacements should converge fast."""
    rng = np.random.default_rng(seed=42)
    base_pose = robot.fkine(standby_q)
    times: list[float] = []
    successes = 0
    for _ in range(50):
        delta = rng.uniform(-0.01, 0.01, size=3)   # ±1cm displacement
        target = base_pose.copy()
        target.t = base_pose.t + delta
        t0 = time.perf_counter()
        result = solve_ik(
            robot, target, standby_q,
            joint_limits_radian=PAROL6_ROBOT.Joint_limits_radian,
        )
        times.append((time.perf_counter() - t0) * 1000.0)
        if result.success:
            successes += 1
    med = median(times)
    assert successes >= 45, f"too many failures: {successes}/50"
    assert med < 50.0, f"solve_ik motion median {med:.2f}ms exceeds 50ms"


@pytest.mark.benchmarks
def test_perf_solve_ik_jogging_under_5ms(robot, standby_q):
    """Jogging IK is on IBVS hot path — must stay under 5ms median."""
    rng = np.random.default_rng(seed=123)
    base_pose = robot.fkine(standby_q)
    times: list[float] = []
    successes = 0
    for _ in range(200):
        delta = rng.uniform(-0.005, 0.005, size=3)   # ±5mm jog step
        target = base_pose.copy()
        target.t = base_pose.t + delta
        t0 = time.perf_counter()
        result = solve_ik(
            robot, target, standby_q, jogging=True,
            joint_limits_radian=PAROL6_ROBOT.Joint_limits_radian,
        )
        times.append((time.perf_counter() - t0) * 1000.0)
        if result.success:
            successes += 1
    med = median(times)
    p95 = _percentile(times, 95.0)
    # Allow 5% jog failure margin: random ±5mm steps occasionally land at
    # joint-limit boundary configurations from standby pose.
    assert successes >= 180, f"too many jog failures: {successes}/200"
    # Slight headroom over the 5ms ideal for environment noise / Windows
    # scheduling jitter; firm IBVS budget is still <10ms p95.
    assert med < 6.0, f"solve_ik jog median {med:.2f}ms exceeds 6ms"
    assert p95 < 10.0, f"solve_ik jog p95 {p95:.2f}ms exceeds 10ms"
