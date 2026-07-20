from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import re
import time
from typing import Any, Iterable

import numpy as np


VISION_PLACEHOLDER_PATTERN = re.compile(
    r"\$vision\.(z_plus|z_minus|x_comp|grip_x|grip_y|grip_z)"
)
DIAGNOSTIC_COMMANDS = {
    "Begin",
    "End",
    "vision",
    "print",
    "Delay",
    "timestamp",
    "Dummy",
}


@dataclass(slots=True)
class ToolZResult:
    target_base_x_mm: float
    target_base_y_mm: float
    z_raw_mm: float
    z_plus_mm: float
    z_minus_mm: float
    x_comp_mm: float
    residual_mm: float
    clamped: bool

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def calculate_tool_z(
    target_base_xy_mm: Iterable[float],
    tcp_base_mm: Iterable[float],
    rotation_base_tool: Iterable[Iterable[float]],
    *,
    x_tool_fixed_mm: float = -60.0,
    y_tool_fixed_mm: float = 0.0,
    base_offset_x_mm: float = 0.0,
    base_offset_y_mm: float = 0.0,
    z_tool_offset_mm: float = 0.0,
    z_plus_min_mm: float = 0.0,
    z_plus_max_mm: float = 78.0,
    retreat_margin_mm: float = 30.0,
    x_comp_base_mm: float = -20.0,
    x_comp_slope: float = 0.0875,
) -> ToolZResult:
    target_xy = np.asarray(list(target_base_xy_mm), dtype=np.float64).reshape(2)
    target_xy = target_xy + np.array(
        [float(base_offset_x_mm), float(base_offset_y_mm)],
        dtype=np.float64,
    )
    tcp = np.asarray(list(tcp_base_mm), dtype=np.float64).reshape(3)
    rotation = np.asarray(rotation_base_tool, dtype=np.float64).reshape(3, 3)

    start_xy = (
        tcp[:2]
        + rotation[:2, 0] * float(x_tool_fixed_mm)
        + rotation[:2, 1] * float(y_tool_fixed_mm)
    )
    tool_z_xy = rotation[:2, 2]
    denominator = float(np.dot(tool_z_xy, tool_z_xy))
    if denominator <= 1e-12:
        raise ValueError("Tool Z axis has no usable projection on the Base XY plane")

    delta_xy = target_xy - start_xy
    z_raw = float(np.dot(tool_z_xy, delta_xy) / denominator)
    projected_xy = start_xy + tool_z_xy * z_raw
    residual = float(np.linalg.norm(target_xy - projected_xy))

    z_corrected = z_raw + float(z_tool_offset_mm)
    z_min = float(z_plus_min_mm)
    z_max = float(z_plus_max_mm)
    if z_min > z_max:
        raise ValueError("Z+ minimum must not exceed Z+ maximum")
    z_plus = float(np.clip(z_corrected, z_min, z_max))
    z_minus = -(z_plus + float(retreat_margin_mm))
    # Tool-X compensation so the world-Z descent stays constant regardless of
    # z_plus: cancels the vertical component of the (slightly tilted) tool-Z axis.
    x_comp = float(x_comp_base_mm) + float(x_comp_slope) * z_plus

    return ToolZResult(
        target_base_x_mm=float(target_xy[0]),
        target_base_y_mm=float(target_xy[1]),
        z_raw_mm=z_raw,
        z_plus_mm=z_plus,
        z_minus_mm=z_minus,
        x_comp_mm=x_comp,
        residual_mm=residual,
        clamped=not math.isclose(z_plus, z_corrected, abs_tol=1e-9),
    )


def bbox_iou(box_a: Iterable[float], box_b: Iterable[float]) -> float:
    a = np.asarray(list(box_a), dtype=np.float64).reshape(4)
    b = np.asarray(list(box_b), dtype=np.float64).reshape(4)
    x1 = max(float(a[0]), float(b[0]))
    y1 = max(float(a[1]), float(b[1]))
    x2 = min(float(a[2]), float(b[2]))
    y2 = min(float(a[3]), float(b[3]))
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area_a = max(0.0, float(a[2] - a[0])) * max(0.0, float(a[3] - a[1]))
    area_b = max(0.0, float(b[2] - b[0])) * max(0.0, float(b[3] - b[1]))
    union = area_a + area_b - intersection
    return intersection / union if union > 0.0 else 0.0


def stable_pick_from_history(
    history: Iterable[dict[str, Any]],
    *,
    sample_count: int = 5,
    max_age_s: float = 1.0,
    min_iou: float = 0.3,
    now: float | None = None,
) -> dict[str, Any]:
    timestamp = time.time() if now is None else float(now)
    candidates: list[dict[str, Any]] = []
    rejections: dict[str, int] = {}

    def _reject(reason: str) -> None:
        rejections[reason] = rejections.get(reason, 0) + 1

    for item in history:
        if not isinstance(item, dict):
            continue
        captured_at = float(item.get("captured_at", 0.0) or 0.0)
        if captured_at <= 0.0 or timestamp - captured_at > float(max_age_s):
            _reject(f"older than {float(max_age_s):.1f}s")
            continue
        safety = str(item.get("pick_safety", "")).upper()
        if safety not in {"SAFE", "MARGINAL"}:
            _reject(f"status {safety or 'UNKNOWN'}")
            continue
        if str(item.get("coordinate_frame", "")).upper() != "BASE":
            _reject("not in BASE frame")
            continue
        if item.get("fixture_box") is None:
            _reject("no fixture")
            continue
        pick_point_base = item.get("pick_point_base")
        if pick_point_base is None:
            pick_point_base = item.get("pick_point_world")
        if item.get("pick_point_px") is None or pick_point_base is None:
            _reject("no base coordinates (pixel-to-base calibration inactive?)")
            continue
        if item.get("selongsong_box") is None:
            _reject("no selongsong box")
            continue
        candidate = dict(item)
        candidate["pick_point_base"] = pick_point_base
        candidates.append(candidate)

    candidates.sort(key=lambda item: float(item.get("captured_at", 0.0)))
    if not candidates:
        detail = ", ".join(
            f"{count}x {reason}" for reason, count in sorted(rejections.items())
        )
        raise ValueError(
            "no recent SAFE or MARGINAL detection with fixture"
            + (f" — rejected: {detail}" if detail else " — detection history is empty")
        )

    reference_box = candidates[-1]["selongsong_box"]
    same_object = [
        item
        for item in candidates
        if bbox_iou(item["selongsong_box"], reference_box) >= float(min_iou)
    ]
    required = max(1, int(sample_count))
    if len(same_object) < required:
        raise ValueError(
            f"need {required} consistent detections within {float(max_age_s):.1f}s; "
            f"got {len(same_object)}"
        )

    samples = same_object[-required:]
    pick_px = np.median(
        np.asarray([item["pick_point_px"][:2] for item in samples], dtype=np.float64),
        axis=0,
    )
    pick_base = np.median(
        np.asarray([item["pick_point_base"][:2] for item in samples], dtype=np.float64),
        axis=0,
    )
    safety = (
        "MARGINAL"
        if any(str(item.get("pick_safety", "")).upper() == "MARGINAL" for item in samples)
        else "SAFE"
    )
    return {
        "pick_point_px": [int(round(float(pick_px[0]))), int(round(float(pick_px[1])))],
        "pick_point_base": [float(pick_base[0]), float(pick_base[1])],
        "pick_safety": safety,
        "sample_count": len(samples),
        "captured_at": max(float(item["captured_at"]) for item in samples),
    }


def command_name(command: str) -> str:
    match = re.match(r"\s*([A-Za-z_][A-Za-z0-9_]*)\s*\(", str(command))
    return match.group(1) if match else ""


def is_vision_diagnostic_program(commands: Iterable[str]) -> bool:
    names = [command_name(command) for command in commands if str(command).strip()]
    return bool(names) and all(name in DIAGNOSTIC_COMMANDS for name in names)


def validate_runtime(runtime: dict[str, Any] | None, *, now: float | None = None) -> dict[str, Any]:
    if not isinstance(runtime, dict) or not bool(runtime.get("valid", False)):
        raise ValueError("vision() has not produced a valid result")
    status = str(runtime.get("status", "")).upper()
    if status not in {"SAFE", "MARGINAL"}:
        raise ValueError(f"vision result is not usable: {status or 'UNKNOWN'}")
    timestamp = time.time() if now is None else float(now)
    expires_at = float(runtime.get("expires_at", 0.0) or 0.0)
    if expires_at <= 0.0 or timestamp > expires_at:
        raise ValueError("vision result is stale; run vision() again")
    return runtime


def resolve_vision_placeholders(
    text: str,
    runtime: dict[str, Any] | None,
    *,
    now: float | None = None,
) -> str:
    if not VISION_PLACEHOLDER_PATTERN.search(str(text)):
        return str(text)
    valid_runtime = validate_runtime(runtime, now=now)
    runtime_keys = {
        "z_plus": "z_plus_mm",
        "z_minus": "z_minus_mm",
        "x_comp": "x_comp_mm",
        "grip_x": "grip_x_mm",
        "grip_y": "grip_y_mm",
        "grip_z": "grip_z_mm",
    }

    def _substitute(match: re.Match[str]) -> str:
        key = runtime_keys[match.group(1)]
        if key not in valid_runtime:
            raise ValueError(
                f"vision runtime has no {key}; run vision() again"
            )
        return f"{float(valid_runtime[key]):.3f}"

    return VISION_PLACEHOLDER_PATTERN.sub(_substitute, str(text))
