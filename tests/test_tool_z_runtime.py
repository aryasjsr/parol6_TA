from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from vision.tool_z_runtime import (  # noqa: E402
    calculate_tool_z,
    is_vision_diagnostic_program,
    resolve_vision_placeholders,
    stable_pick_from_history,
)


REFERENCE_TCP_MM = np.array([0.0, 271.124, 293.080])
REFERENCE_ROTATION = np.array(
    [
        [0.0, 1.0, 0.0],
        [-0.05685, 0.0, 0.998383],
        [0.998383, 0.0, 0.05685],
    ]
)


def target_for_tool_z(z_mm: float) -> np.ndarray:
    start_xy = REFERENCE_TCP_MM[:2] + REFERENCE_ROTATION[:2, 0] * -60.0
    return start_xy + REFERENCE_ROTATION[:2, 2] * float(z_mm)


def test_tool_z_result_and_retreat():
    result = calculate_tool_z(
        target_for_tool_z(70.0),
        REFERENCE_TCP_MM,
        REFERENCE_ROTATION,
    )

    assert result.z_raw_mm == pytest.approx(70.0, abs=1e-3)
    assert result.z_plus_mm == pytest.approx(70.0, abs=1e-3)
    assert result.z_minus_mm == pytest.approx(-100.0, abs=1e-3)
    assert result.residual_mm == pytest.approx(0.0, abs=1e-6)
    assert result.clamped is False


def test_tool_z_clamps_to_78_mm():
    result = calculate_tool_z(
        target_for_tool_z(100.0),
        REFERENCE_TCP_MM,
        REFERENCE_ROTATION,
    )

    assert result.z_plus_mm == pytest.approx(78.0)
    assert result.z_minus_mm == pytest.approx(-108.0)
    assert result.clamped is True


def test_stable_pick_uses_five_frame_median_and_propagates_marginal():
    history = []
    for index, value in enumerate([10, 11, 12, 13, 100]):
        history.append(
            {
                "captured_at": 9.5 + index * 0.1,
                "pick_safety": "MARGINAL" if index == 2 else "SAFE",
                "selongsong_box": [0, 0, 100, 50],
                "fixture_box": [80, -10, 110, 60],
                "pick_point_px": [value, 20],
                "pick_point_base": [float(value), 300.0],
                "pick_point_world": [float(value), 300.0],
                "coordinate_frame": "BASE",
            }
        )

    stable = stable_pick_from_history(history, now=10.0)

    assert stable["pick_point_px"] == [12, 20]
    assert stable["pick_point_base"] == pytest.approx([12.0, 300.0])
    assert stable["pick_safety"] == "MARGINAL"


def test_stable_pick_rejects_unknown_or_stale_samples():
    history = [
        {
            "captured_at": 1.0,
            "pick_safety": "UNKNOWN",
            "selongsong_box": [0, 0, 100, 50],
            "fixture_box": None,
            "pick_point_px": [20, 20],
            "pick_point_base": [0.0, 300.0],
            "pick_point_world": [0.0, 300.0],
            "coordinate_frame": "BASE",
        }
    ]

    with pytest.raises(ValueError):
        stable_pick_from_history(history, now=10.0)


def test_stable_pick_rejects_samples_without_explicit_base_frame():
    history = []
    for index in range(5):
        history.append(
            {
                "captured_at": 9.5 + index * 0.1,
                "pick_safety": "SAFE",
                "selongsong_box": [0, 0, 100, 50],
                "fixture_box": [80, -10, 110, 60],
                "pick_point_px": [20, 20],
                "pick_point_base": [0.0, 300.0],
            }
        )

    with pytest.raises(ValueError):
        stable_pick_from_history(history, now=10.0)


def test_placeholder_interpolation_and_stale_error():
    runtime = {
        "valid": True,
        "status": "SAFE",
        "z_plus_mm": 70.0,
        "z_minus_mm": -100.0,
        "expires_at": 20.0,
    }

    assert resolve_vision_placeholders(
        "Z+ Tool = $vision.z_plus mm",
        runtime,
        now=10.0,
    ) == "Z+ Tool = 70.000 mm"
    assert resolve_vision_placeholders(
        "$vision.z_minus",
        runtime,
        now=10.0,
    ) == "-100.000"

    with pytest.raises(ValueError):
        resolve_vision_placeholders("$vision.z_plus", runtime, now=21.0)
    with pytest.raises(ValueError):
        resolve_vision_placeholders("$vision.z_plus", {}, now=10.0)


def test_program_classification_allows_offline_diagnostic_only():
    assert is_vision_diagnostic_program(
        [
            "Begin()",
            'print("pick start")',
            "vision()",
            'print("$vision.z_minus")',
            'print("$vision.z_plus")',
            "End()",
        ]
    )
    assert not is_vision_diagnostic_program(
        [
            "Begin()",
            "vision()",
            "MoveCartRelTRF(-60,0,$vision.z_plus,0,0,0,t=4)",
            "End()",
        ]
    )
