"""Centroid-based Image-Based Visual Servoing (IBVS) controller.

Ported from experimental branch (CentroidIBVSReactor) and adapted to the
comsoft IPC model (multiprocessing shared arrays) so it does not depend on
the Reactor / Commander architecture.

Usage:
    ctrl = CentroidIBVSController(image_size=(640, 480))
    ctrl.load_intrinsics()                       # auto from tools/Camera/param/*.csv
    result = ctrl.update(bundle_dict)            # framework-agnostic; returns deltas
    if result["should_command"]:
        ctrl.dispatch(result, ipc_arrays)        # explicit IPC write (caller-gated)

The controller never writes IPC unless `dispatch` is called explicitly, so it
is safe to run inside the vision capture loop for telemetry only.
"""

from __future__ import annotations

import math
import os
import threading
from pathlib import Path
from typing import Any

import numpy as np


_PROJECT_ROOT = Path(__file__).resolve().parents[1]
_DEFAULT_PARAM_CSV = _PROJECT_ROOT / "tools" / "Camera" / "param" / "camera_intrinsic_matrix.csv"


class CentroidIBVSController:
    def __init__(
        self,
        image_size: tuple[int, int] = (640, 480),
        fx: float | None = None,
        fy: float | None = None,
        cx: float | None = None,
        cy: float | None = None,
        desired_uv: tuple[float, float] | None = None,
        depth_m: float = 0.5,
        lam: float = 1.0,
        deadband_px: float = 4.0,
        max_step_mm: float = 20.0,
        invert_x: bool = False,
        invert_y: bool = True,
        speed_pct: float = 30.0,
        interval_s: float = 0.1,
        frame: str = "TRF",
    ) -> None:
        self._lock = threading.Lock()
        self.image_size = (int(image_size[0]), int(image_size[1]))
        self.fx = float(fx) if fx is not None else 800.0
        self.fy = float(fy) if fy is not None else 800.0
        self.cx = float(cx) if cx is not None else float(self.image_size[0]) / 2.0
        self.cy = float(cy) if cy is not None else float(self.image_size[1]) / 2.0
        if desired_uv is not None:
            self.desired_uv: tuple[float, float] = (float(desired_uv[0]), float(desired_uv[1]))
        else:
            self.desired_uv = (self.cx, self.cy)
        self.depth_m = float(depth_m)
        self.lam = float(lam)
        self.deadband_px = float(deadband_px)
        self.max_step_mm = float(max_step_mm)
        self.invert_x = bool(invert_x)
        self.invert_y = bool(invert_y)
        self.speed_pct = float(speed_pct)
        self.interval_s = float(interval_s)
        self.frame = str(frame)

        self.last_centroid: tuple[float, float] | None = None
        self.last_err_px: tuple[float, float] = (0.0, 0.0)
        self.last_cmd_mm: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.in_deadband: bool = True
        self.last_status: str = "idle"

    def load_intrinsics(self, path: str | os.PathLike[str] | None = None) -> bool:
        target = Path(path) if path is not None else _DEFAULT_PARAM_CSV
        if not target.exists():
            return False
        try:
            matrix = np.loadtxt(target, delimiter=",").astype(float)
            if matrix.shape != (3, 3):
                return False
            with self._lock:
                self.fx = float(matrix[0, 0])
                self.fy = float(matrix[1, 1])
                self.cx = float(matrix[0, 2])
                self.cy = float(matrix[1, 2])
                self.desired_uv = (self.cx, self.cy)
            return True
        except Exception:
            return False

    def configure(self, **kwargs: Any) -> None:
        with self._lock:
            for key, value in kwargs.items():
                if hasattr(self, key) and value is not None:
                    setattr(self, key, value)

    def _extract_centroid(self, payload: Any) -> tuple[float, float] | None:
        if payload is None:
            return None
        if isinstance(payload, dict):
            for key in ("centroid_px", "pick_point_px", "center"):
                value = payload.get(key)
                if value and len(value) >= 2:
                    try:
                        return float(value[0]), float(value[1])
                    except (TypeError, ValueError):
                        continue
            sel = payload.get("selongsong_box")
            if sel and len(sel) >= 4:
                try:
                    x1, y1, x2, y2 = (float(v) for v in sel[:4])
                    return (x1 + x2) / 2.0, (y1 + y2) / 2.0
                except (TypeError, ValueError):
                    return None
            return None
        center = getattr(payload, "center", None)
        if center is not None and len(center) >= 2:
            try:
                return float(center[0]), float(center[1])
            except (TypeError, ValueError):
                return None
        return None

    def update(self, detection_payload: Any) -> dict[str, Any]:
        centroid = self._extract_centroid(detection_payload)
        with self._lock:
            if centroid is None:
                self.last_status = "no_target"
                self.in_deadband = True
                return self._snapshot_locked(should_command=False)

            u_cur, v_cur = centroid
            u_des, v_des = self.desired_uv
            u_err = u_des - u_cur
            v_err = v_des - v_cur

            self.last_centroid = (u_cur, v_cur)
            self.last_err_px = (u_err, v_err)

            if abs(u_err) <= self.deadband_px and abs(v_err) <= self.deadband_px:
                self.in_deadband = True
                self.last_status = "in_deadband"
                self.last_cmd_mm = (0.0, 0.0, 0.0)
                return self._snapshot_locked(should_command=False)

            dx_m = (u_err / max(1e-6, self.fx)) * self.depth_m
            dy_m = (v_err / max(1e-6, self.fy)) * self.depth_m
            if self.invert_x:
                dx_m = -dx_m
            if self.invert_y:
                dy_m = -dy_m

            dx_mm = float(self.lam * dx_m * 1000.0)
            dy_mm = float(self.lam * dy_m * 1000.0)

            mag = math.hypot(dx_mm, dy_mm)
            if mag > self.max_step_mm and mag > 0:
                scale = self.max_step_mm / mag
                dx_mm *= scale
                dy_mm *= scale

            self.last_cmd_mm = (dx_mm, dy_mm, 0.0)
            self.in_deadband = False
            self.last_status = "tracking"
            return self._snapshot_locked(should_command=True)

    def _snapshot_locked(self, should_command: bool) -> dict[str, Any]:
        return {
            "status": self.last_status,
            "should_command": should_command,
            "in_deadband": self.in_deadband,
            "centroid_px": self.last_centroid,
            "err_px": {"u": self.last_err_px[0], "v": self.last_err_px[1]},
            "cmd_mm": {"dx": self.last_cmd_mm[0], "dy": self.last_cmd_mm[1], "dz": self.last_cmd_mm[2]},
            "frame": self.frame,
            "speed_pct": self.speed_pct,
            "interval_s": self.interval_s,
        }

    def dispatch(self, snapshot: dict[str, Any], ipc_arrays: dict[str, Any]) -> bool:
        """Translate `snapshot` (from update()) into MoveCartRelTRF command string.

        ipc_arrays must contain key `shared_string` (multiprocessing Array of c_char).
        Returns True if a command was emitted, False if skipped (deadband/no-target).

        This does NOT directly mutate Position_out / Speed_out / Command_out — those
        are owned by Task1/Task2/Task3 (Serial_sender_good_latest.py). Instead we
        log the intended command to shared_string so a higher-level orchestrator
        (program executor, or the GUI button) can pick it up.
        """
        if not snapshot.get("should_command"):
            return False
        shared = ipc_arrays.get("shared_string") if isinstance(ipc_arrays, dict) else None
        cmd = snapshot["cmd_mm"]
        text = (
            f"MoveCartRelTRF({cmd['dx']:.3f},{cmd['dy']:.3f},0,0,0,0,"
            f"t={self.interval_s:.3f})"
        )
        if shared is not None:
            try:
                shared.value = ("Log: IBVS " + text).encode()[:120]
            except Exception:
                pass
        return True

    def reset(self) -> None:
        with self._lock:
            self.last_centroid = None
            self.last_err_px = (0.0, 0.0)
            self.last_cmd_mm = (0.0, 0.0, 0.0)
            self.in_deadband = True
            self.last_status = "reset"

    def to_dict(self) -> dict[str, Any]:
        with self._lock:
            return {
                "type": "CentroidIBVS",
                "desired_uv": [self.desired_uv[0], self.desired_uv[1]],
                "last_centroid": list(self.last_centroid) if self.last_centroid else None,
                "last_err_px": {"u_err": self.last_err_px[0], "v_err": self.last_err_px[1]},
                "last_cmd_mm": {
                    "dx": self.last_cmd_mm[0],
                    "dy": self.last_cmd_mm[1],
                    "dz": self.last_cmd_mm[2],
                },
                "depth_m": self.depth_m,
                "lam": self.lam,
                "deadband_px": self.deadband_px,
                "max_step_mm": self.max_step_mm,
                "fx": self.fx,
                "fy": self.fy,
                "cx": self.cx,
                "cy": self.cy,
                "in_deadband": self.in_deadband,
                "status": self.last_status,
            }
