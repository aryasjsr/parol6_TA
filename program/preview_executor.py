"""Dry-run program executor for Preview Mode.

Runs the program **without** sending any commands to hardware.
Instead it computes FK/IK for each motion command and emits
``sim_position_changed`` so the 3D preview canvas updates.

Non-motion commands (Delay, Gripper, Output, Input, Home, vision)
are logged but skipped – no hardware side-effects.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable

import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal
from spatialmath import SE3

from backend.exceptions import IKFailureError, PAROL6Error
from program.program_model import ProgramCommand, ProgramModel
import tools.PAROL6_ROBOT as PAROL6_ROBOT


class PreviewProgramExecutor(QThread):
    """Execute a program in simulation-only mode (no hardware)."""

    log_message = pyqtSignal(str, str)
    line_changed = pyqtSignal(int)
    state_changed = pyqtSignal(str)
    finished_execution = pyqtSignal(bool, str)
    sim_position_changed = pyqtSignal(list)  # 6 floats, radians

    # Time to wait between simulated steps so the user can see the 3D
    # canvas animating.  Adjustable via config.
    DEFAULT_STEP_DELAY_S = 0.6

    def __init__(
        self,
        model: ProgramModel,
        initial_radians: list[float],
        config_getter: Callable[[str, Any], Any] | None = None,
        single_line: int | None = None,
    ) -> None:
        super().__init__()
        self._model = model
        self._q = list(initial_radians[:6])  # current simulated joint state (rad)
        self._config_getter = config_getter or (lambda k, d: d)
        self._single_line = single_line
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._loop_frame: dict[str, int] | None = None
        self._step_delay = float(
            self._config_getter("preview.step_delay_s", self.DEFAULT_STEP_DELAY_S)
        )

        self.COMMAND_HANDLERS: dict[str, Callable[[ProgramCommand, int], int | None]] = {
            "start": self._noop,
            "Begin": self._noop,
            "End": self._noop,
            "end": self._noop,
            "Dummy": self._noop,
            "MoveJoint": self._preview_move_joint,
            "MovePose": self._preview_move_pose,
            "MoveCart": self._preview_move_cart,
            "MoveCartRelTRF": self._preview_move_cart_rel_trf,
            "SpeedJoint": self._noop,
            "Delay": self._preview_delay,
            "Loop": self._preview_loop,
            "EndLoop": self._preview_end_loop,
            "Home": self._preview_home,
            "Gripper": self._log_skip,
            "Gripper_cal": self._log_skip,
            "Output": self._log_skip,
            "Input": self._log_skip,
            "vision": self._log_skip,
        }

    # -----------------------------------------------------------------
    # QThread entry
    # -----------------------------------------------------------------
    def run(self) -> None:
        self.state_changed.emit("running")
        # If sim position is uninitialized (all zeros), start from standby
        if all(abs(r) < 1e-9 for r in self._q):
            self._q = list(PAROL6_ROBOT.Joints_standby_position_radian)
            self.sim_position_changed.emit(list(self._q))
        self._log("[Preview] Dry-run started", "info")
        try:
            if self._single_line is not None:
                idx = max(0, min(self._single_line - 1, len(self._model.commands) - 1))
                cmd = self._model.commands[idx]
                self.line_changed.emit(idx + 1)
                self._dispatch(cmd, idx)
            else:
                idx = 0
                while idx < len(self._model.commands):
                    self._check_stopped()
                    self._wait_if_paused()
                    cmd = self._model.commands[idx]
                    self.line_changed.emit(idx + 1)
                    nxt = self._dispatch(cmd, idx)
                    idx = idx + 1 if nxt is None else nxt
            self.state_changed.emit("idle")
            self.finished_execution.emit(True, "[Preview] Dry-run completed")
        except Exception as exc:
            self._log(f"[Preview] {exc}", "error")
            self.state_changed.emit("error")
            self.finished_execution.emit(False, f"[Preview] {exc}")

    def stop(self) -> None:
        self._stop_event.set()
        self._pause_event.clear()

    def pause(self) -> None:
        if not self._stop_event.is_set() and not self._pause_event.is_set():
            self._pause_event.set()
            self.state_changed.emit("paused")

    def resume(self) -> None:
        if not self._stop_event.is_set() and self._pause_event.is_set():
            self._pause_event.clear()
            self.state_changed.emit("running")

    def is_paused(self) -> bool:
        return self._pause_event.is_set()

    # -----------------------------------------------------------------
    # Dispatch
    # -----------------------------------------------------------------
    def _dispatch(self, cmd: ProgramCommand, idx: int) -> int | None:
        handler = self.COMMAND_HANDLERS.get(cmd.name)
        if handler is None:
            self._log(f"[Preview] Skipping unsupported: {cmd.name}", "warn")
            return None
        return handler(cmd, idx)

    # -----------------------------------------------------------------
    # Motion handlers (sim-only)
    # -----------------------------------------------------------------
    def _preview_move_joint(self, cmd: ProgramCommand, idx: int) -> int | None:
        target_deg = self._resolve_joint_target(cmd)
        target_rad = [PAROL6_ROBOT.DEG2RAD(d) for d in target_deg]
        self._apply_joint_target(target_rad)
        self._log(f"[Preview] MoveJoint -> {[f'{d:.1f}' for d in target_deg]}", "info")
        return None

    def _preview_move_pose(self, cmd: ProgramCommand, idx: int) -> int | None:
        target_pose = self._build_absolute_pose(cmd)
        try:
            q_rad = self._solve_ik(target_pose)
        except IKFailureError as e:
            self._log(f"[Preview] MovePose SKIPPED: {e}", "warn")
            return None
        self._apply_joint_target(list(q_rad))
        q_deg = [float(np.rad2deg(v)) for v in q_rad]
        self._log(f"[Preview] MovePose -> {[f'{d:.1f}' for d in q_deg]}", "info")
        return None

    def _preview_move_cart(self, cmd: ProgramCommand, idx: int) -> int | None:
        target_pose = self._build_absolute_pose(cmd)
        try:
            q_rad = self._solve_ik(target_pose)
        except IKFailureError as e:
            self._log(f"[Preview] MoveCart SKIPPED: {e}", "warn")
            return None
        self._apply_joint_target(list(q_rad))
        q_deg = [float(np.rad2deg(v)) for v in q_rad]
        self._log(f"[Preview] MoveCart -> {[f'{d:.1f}' for d in q_deg]}", "info")
        return None

    def _preview_move_cart_rel_trf(self, cmd: ProgramCommand, idx: int) -> int | None:
        dx = float(self._kw(cmd, "dx", 0.0)) / 1000.0
        dy = float(self._kw(cmd, "dy", 0.0)) / 1000.0
        dz = float(self._kw(cmd, "dz", 0.0)) / 1000.0
        dr = float(self._kw(cmd, "dr", self._kw(cmd, "rx", 0.0)))
        dp = float(self._kw(cmd, "dp", self._kw(cmd, "ry", 0.0)))
        dyaw = float(self._kw(cmd, "dyaw", self._kw(cmd, "rz", 0.0)))

        current_pose = PAROL6_ROBOT.robot.fkine(np.array(self._q))
        target = current_pose.copy()
        target.t = target.t + current_pose.R @ np.array([dx, dy, dz], dtype=float)
        target = target * SE3.Rx(dr, unit="deg") * SE3.Ry(dp, unit="deg") * SE3.Rz(dyaw, unit="deg")
        try:
            q_rad = self._solve_ik(target)
        except IKFailureError as e:
            self._log(f"[Preview] MoveCartRelTRF SKIPPED: {e}", "warn")
            return None
        self._apply_joint_target(list(q_rad))
        self._log("[Preview] MoveCartRelTRF executed", "info")
        return None

    def _preview_home(self, cmd: ProgramCommand, idx: int) -> int | None:
        home_rad = list(PAROL6_ROBOT.Joints_standby_position_radian)
        self._apply_joint_target(home_rad)
        self._log(f"[Preview] Home -> {[f'{np.rad2deg(r):.1f}' for r in home_rad]}\u00b0", "info")
        return None

    def _preview_delay(self, cmd: ProgramCommand, idx: int) -> int | None:
        raw = cmd.args[0] if cmd.args else self._kw(cmd, "ms", 0.0)
        duration_s = float(raw)
        if duration_s > 100:
            duration_s /= 1000.0
        # In preview mode, sleep a scaled-down amount just for visual feedback
        visual_delay = min(duration_s, 1.0)
        remaining = visual_delay
        while remaining > 0:
            self._check_stopped()
            self._wait_if_paused()
            sleep_s = min(0.05, remaining)
            time.sleep(sleep_s)
            remaining -= sleep_s
        self._log(f"[Preview] Delay({duration_s:.2f}s)", "info")
        return None

    def _preview_loop(self, cmd: ProgramCommand, idx: int) -> int | None:
        if self._single_line is not None:
            return None
        count_raw = cmd.args[0] if cmd.args else self._kw(cmd, "count", None)
        if count_raw is None or str(count_raw).strip() == "":
            self._log("[Preview] Loop() -> back to Begin", "info")
            return 0
        if self._loop_frame is not None:
            raise PAROL6Error("Nested Loop is not supported")
        self._loop_frame = {"start_index": idx, "remaining": int(count_raw)}
        return None

    def _preview_end_loop(self, cmd: ProgramCommand, idx: int) -> int | None:
        if self._single_line is not None:
            return None
        if self._loop_frame is None:
            raise PAROL6Error("EndLoop without matching Loop")
        if self._loop_frame["remaining"] <= 1:
            self._loop_frame = None
            return None
        self._loop_frame["remaining"] -= 1
        return self._loop_frame["start_index"] + 1

    # -----------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------
    def _apply_joint_target(self, q_rad: list[float]) -> None:
        """Clamp to joint limits, update internal state, emit signal, and pause briefly."""
        for i in range(6):
            lo, hi = PAROL6_ROBOT.Joint_limits_radian[i]
            q_rad[i] = max(lo, min(hi, q_rad[i]))
        self._q = list(q_rad)
        self.sim_position_changed.emit(list(self._q))
        # Brief pause so the user can see the 3D sim move step-by-step
        self._interruptible_sleep(self._step_delay)

    def _solve_ik(self, target_pose: SE3) -> np.ndarray:
        """Solve IK with multiple seed strategies for robustness."""
        seeds = [
            np.array(self._q),
            np.array(PAROL6_ROBOT.Joints_standby_position_radian),
            np.array([(lo + hi) / 2 for lo, hi in PAROL6_ROBOT.Joint_limits_radian]),
        ]
        last_q = None
        for q0 in seeds:
            result, q_raw = self._try_ik(target_pose, q0)
            if result is not None:
                return result
            if q_raw is not None:
                last_q = q_raw
        # Build a descriptive error showing which joints are out of range
        violations = []
        if last_q is not None:
            for i in range(6):
                lo, hi = PAROL6_ROBOT.Joint_limits_radian[i]
                if not (lo <= last_q[i] <= hi):
                    violations.append(
                        f"J{i+1}={np.rad2deg(last_q[i]):.1f}\u00b0 "
                        f"[{np.rad2deg(lo):.0f}\u00b0..{np.rad2deg(hi):.0f}\u00b0]"
                    )
        detail = f" ({', '.join(violations)})" if violations else ""
        raise IKFailureError(f"[Preview] Target pose unreachable within joint limits{detail}")

    def _try_ik(self, target_pose: SE3, q0: np.ndarray) -> tuple[np.ndarray | None, np.ndarray | None]:
        """Attempt IK with given seed. Returns (clamped_result, raw_solution)."""
        sol = PAROL6_ROBOT.robot.ikine_LMS(target_pose, q0=q0, ilimit=50)
        if not hasattr(sol, "q"):
            return None, None
        q_raw = np.asarray(sol.q).reshape(-1)[:6]
        q_rad = q_raw.copy()
        CLAMP_TOL = np.deg2rad(0.5)  # 0.5\u00b0 tolerance for boundary clamping
        for i in range(6):
            lo, hi = PAROL6_ROBOT.Joint_limits_radian[i]
            if q_rad[i] < lo and q_rad[i] >= lo - CLAMP_TOL:
                q_rad[i] = lo
            elif q_rad[i] > hi and q_rad[i] <= hi + CLAMP_TOL:
                q_rad[i] = hi
            if not (lo <= q_rad[i] <= hi):
                return None, q_raw
        return q_rad, q_raw

    def _resolve_joint_target(self, cmd: ProgramCommand) -> list[float]:
        current = [float(np.rad2deg(r)) for r in self._q]
        aliases = ["j1", "j2", "j3", "j4", "j5", "j6"]
        for i, alias in enumerate(aliases):
            if alias in cmd.kwargs:
                current[i] = float(cmd.kwargs[alias])
            elif len(cmd.args) > i:
                current[i] = float(cmd.args[i])
        return current

    def _build_absolute_pose(self, cmd: ProgramCommand) -> SE3:
        args = cmd.args
        x_mm = float(args[0]) if len(args) > 0 and cmd.kwargs.get("x") is None else float(self._kw(cmd, "x", 0.0))
        y_mm = float(args[1]) if len(args) > 1 and cmd.kwargs.get("y") is None else float(self._kw(cmd, "y", 0.0))
        z_mm = float(args[2]) if len(args) > 2 and cmd.kwargs.get("z") is None else float(
            self._kw(cmd, "z", self._config_getter("workspace.z_fixed_mm", 200.0))
        )
        r_deg = float(args[3]) if len(args) > 3 and cmd.kwargs.get("r") is None else float(
            self._kw(cmd, "r", self._kw(cmd, "rx", 0.0))
        )
        p_deg = float(args[4]) if len(args) > 4 and cmd.kwargs.get("p") is None else float(
            self._kw(cmd, "p", self._kw(cmd, "ry", 0.0))
        )
        yaw_deg = float(args[5]) if len(args) > 5 and cmd.kwargs.get("yaw") is None else float(
            self._kw(cmd, "yaw", self._kw(cmd, "rz", 0.0))
        )
        pose = SE3.RPY([r_deg, p_deg, yaw_deg], unit="deg", order="xyz")
        pose.t[0] = x_mm / 1000
        pose.t[1] = y_mm / 1000
        pose.t[2] = z_mm / 1000
        return pose

    @staticmethod
    def _kw(cmd: ProgramCommand, key: str, default: Any) -> Any:
        return cmd.kwargs.get(key, default)

    def _noop(self, cmd: ProgramCommand, idx: int) -> int | None:
        return None

    def _log_skip(self, cmd: ProgramCommand, idx: int) -> int | None:
        self._log(f"[Preview] {cmd.name}() skipped (no hardware)", "info")
        return None

    def _check_stopped(self) -> None:
        if self._stop_event.is_set():
            raise PAROL6Error("Program stopped")

    def _wait_if_paused(self) -> None:
        while self._pause_event.is_set():
            self._check_stopped()
            time.sleep(0.05)

    def _interruptible_sleep(self, duration: float) -> None:
        remaining = max(0.0, duration)
        while remaining > 0:
            self._check_stopped()
            self._wait_if_paused()
            chunk = min(0.05, remaining)
            time.sleep(chunk)
            remaining -= chunk

    def _log(self, message: str, level: str = "info") -> None:
        self.log_message.emit(message, level)
