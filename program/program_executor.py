from __future__ import annotations

import math
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Any, Callable

import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal
import scipy
from spatialmath import SE3

if not hasattr(scipy, "randn"):
    scipy.randn = np.random.randn

backend_dir = Path(__file__).resolve().parents[1] / "backend"
project_dir = backend_dir.parent
if str(backend_dir) not in sys.path:
    sys.path.insert(0, str(backend_dir))
if str(project_dir) not in sys.path:
    sys.path.insert(0, str(project_dir))

from backend.Centroid_IBVS import CentroidIBVSReactor
from backend.action import Action, HomeRobotAction, Move2JointsAction
from backend.exceptions import (
    FailsafeTriggeredError,
    IKFailureError,
    PAROL6Error,
    WorkspaceViolationError,
)
from backend.failsafe_manager import FailsafeManager
from backend.research_logger import ResearchLogger
from program.program_model import ProgramCommand, ProgramModel
from tools.shared_struct import RobotInputData, RobotOutputData
from vision.object_detector import DetectionBundle, DetectionResult
from vision.workspace_validator import WorkspaceValidator
import tools.PAROL6_ROBOT as PAROL6_ROBOT


@dataclass(slots=True)
class ProgramExecutorDependencies:
    commander: Any
    robot_data: RobotInputData
    command_data: RobotOutputData
    shared_string: Any
    jog_control: Any | None
    failsafe: FailsafeManager
    research_logger: ResearchLogger
    workspace_validator: WorkspaceValidator
    config_getter: Callable[[str, Any], Any]
    vision_getter: Callable[[], DetectionResult | None]
    modbus_value_getter: Callable[[Any], Any]
    modbus_writer: Callable[[str, bool], bool] | None = None
    bundle_getter: Callable[[], DetectionBundle | None] | None = None


class _DigitalOutputAction(Action):
    modal = False

    def __init__(self, output_index: int, state: bool, shared_string: Any = None) -> None:
        self.output_index = int(output_index)
        self.state = bool(state)
        self.shared_string = shared_string
        self.done = False

    def step(self, robot_data: RobotInputData, cmd_data: RobotOutputData):
        if self.done:
            return None
        cmd_data.inout[self.output_index] = 1 if self.state else 0
        robot_data.inout[self.output_index] = 1 if self.state else 0
        cmd_data.command.value = 255
        if self.shared_string is not None:
            text = f"Log: Output {self.output_index} -> {'HIGH' if self.state else 'LOW'}"
            self.shared_string.value = text.encode()[:120]
        self.done = True
        return None

    def is_done(self) -> bool:
        return self.done


class ProgramExecutor(QThread):
    log_message = pyqtSignal(str, str)
    line_changed = pyqtSignal(int)
    state_changed = pyqtSignal(str)
    finished_execution = pyqtSignal(bool, str)
    marginal_detected = pyqtSignal(object)  # emits DetectionBundle

    def __init__(
        self,
        model: ProgramModel,
        dependencies: ProgramExecutorDependencies,
        single_line: int | None = None,
    ) -> None:
        super().__init__()
        self._model = model
        self._deps = dependencies
        self._single_line = single_line
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._marginal_event = threading.Event()  # set when user confirms/skips
        self._marginal_action: str = ""  # "confirm" or "skip"
        self._loop_frame: dict[str, int] | None = None
        self.COMMAND_HANDLERS: dict[str, Callable[[ProgramCommand, int], int | None]] = {
            "start": self._execute_start,
            "MoveCart": self._execute_move_cart,
            "MoveJoint": self._execute_move_joint,
            "MoveCartRelTRF": self._execute_move_cart_rel_trf,
            "vision": self._execute_vision,
            "Home": self._execute_home,
            "Gripper": self._execute_gripper,
            "Output": self._execute_output,
            "Input": self._execute_input,
            "Delay": self._execute_delay,
            "Loop": self._execute_loop,
            "EndLoop": self._execute_end_loop,
            "end": self._execute_end,
        }

    def stop(self) -> None:
        self._stop_event.set()
        self._pause_event.clear()

    def pause(self) -> None:
        if self._stop_event.is_set() or self._pause_event.is_set():
            return
        self._pause_event.set()
        self.state_changed.emit("paused")
        self._log("Program paused", "warn")

    def resume(self) -> None:
        if self._stop_event.is_set() or not self._pause_event.is_set():
            return
        self._pause_event.clear()
        self.state_changed.emit("running")
        self._log("Program resumed", "info")

    def is_paused(self) -> bool:
        return self._pause_event.is_set()

    def confirm_pick(self) -> None:
        """User confirmed MARGINAL pick — resume execution."""
        self._marginal_action = "confirm"
        self._marginal_event.set()

    def skip_pick(self) -> None:
        """User skipped MARGINAL pick — skip to next command."""
        self._marginal_action = "skip"
        self._marginal_event.set()

    def run(self) -> None:
        self.state_changed.emit("running")
        try:
            if self._single_line is not None:
                line_index = max(0, min(self._single_line - 1, len(self._model.commands) - 1))
                command = self._model.commands[line_index]
                self._wait_if_paused()
                self.line_changed.emit(line_index + 1)
                self._dispatch(command, line_index)
            else:
                index = 0
                while index < len(self._model.commands):
                    self._check_stopped()
                    self._wait_if_paused()
                    command = self._model.commands[index]
                    self.line_changed.emit(index + 1)
                    next_index = self._dispatch(command, index)
                    index = index + 1 if next_index is None else next_index
            message = "Program completed"
            self.state_changed.emit("idle")
            self.finished_execution.emit(True, message)
        except Exception as exc:
            level = "warn" if isinstance(exc, FailsafeTriggeredError) else "error"
            self._log(str(exc), level)
            self.state_changed.emit("error")
            self.finished_execution.emit(False, str(exc))

    def _dispatch(self, command: ProgramCommand, index: int) -> int | None:
        handler = self.COMMAND_HANDLERS.get(command.name)
        if handler is None:
            raise PAROL6Error(f"Unsupported command: {command.name}")
        return handler(command, index)

    def _execute_start(self, command: ProgramCommand, index: int) -> int | None:
        self._log("Program start", "info")
        return None

    def _execute_end(self, command: ProgramCommand, index: int) -> int | None:
        self._log("Program end", "info")
        return None

    def _execute_move_joint(self, command: ProgramCommand, index: int) -> int | None:
        self._ensure_motion_safe()
        joint_target = self._resolve_joint_target(command)
        action = Move2JointsAction(
            q_deg=joint_target,
            v_pct=float(command.kwargs.get("speed_pct", self._deps.config_getter("robot.default_speed_pct", 30))),
            shared_string=self._deps.shared_string,
        )
        self._inject_and_wait(action, timeout_s=30.0)
        self._log(f"MoveJoint -> {joint_target}", "info")
        return None

    def _execute_move_cart(self, command: ProgramCommand, index: int) -> int | None:
        self._ensure_motion_safe()
        target_pose = self._build_absolute_pose(command)
        self._move_to_pose(target_pose, speed_pct=float(command.kwargs.get("speed_pct", self._deps.config_getter("robot.default_speed_pct", 30))))
        self._log("MoveCart executed", "info")
        return None

    def _execute_move_cart_rel_trf(self, command: ProgramCommand, index: int) -> int | None:
        self._ensure_motion_safe()
        dx = float(self._first_value(command, "dx", 0.0)) / 1000.0
        dy = float(self._first_value(command, "dy", 0.0)) / 1000.0
        dz = float(self._first_value(command, "dz", 0.0)) / 1000.0
        dr = float(self._first_value(command, "dr", self._first_value(command, "rx", 0.0)))
        dp = float(self._first_value(command, "dp", self._first_value(command, "ry", 0.0)))
        dyaw = float(self._first_value(command, "dyaw", self._first_value(command, "rz", 0.0)))
        current_pose = self._current_pose()
        target_pose = current_pose.copy()
        target_pose.t = target_pose.t + current_pose.R @ np.array([dx, dy, dz], dtype=float)
        target_pose = target_pose * SE3.Rx(dr, unit="deg") * SE3.Ry(dp, unit="deg") * SE3.Rz(dyaw, unit="deg")
        self._move_to_pose(target_pose, speed_pct=float(command.kwargs.get("speed_pct", self._deps.config_getter("robot.default_speed_pct", 30))))
        self._log("MoveCartRelTRF executed", "info")
        return None

    def _execute_home(self, command: ProgramCommand, index: int) -> int | None:
        action = HomeRobotAction(self._deps.shared_string)
        self._inject_and_wait(action, timeout_s=45.0)
        self._log("Home executed", "info")
        return None

    def _execute_gripper(self, command: ProgramCommand, index: int) -> int | None:
        state = str(self._first_value(command, "state", command.args[0] if command.args else "open")).strip().lower()
        pin = int(self._deps.config_getter("robot.gripper_output_pin", 1))
        mapped_state = state in {"close", "on", "high", "1", "true"}
        output_command = ProgramCommand(
            name="Output",
            args=[pin, "ON" if mapped_state else "OFF"],
            kwargs={},
            notes=command.notes,
            source=f"Output({pin}, {'ON' if mapped_state else 'OFF'})",
            parameters_text=f"{pin}, {'ON' if mapped_state else 'OFF'}",
        )
        self._execute_output(output_command, index)
        self._log(f"Gripper -> {state}", "info")
        return None

    def _execute_output(self, command: ProgramCommand, index: int) -> int | None:
        pin = int(self._first_value(command, "n", command.args[0] if command.args else 1))
        state_value = self._first_value(command, "state", command.args[1] if len(command.args) > 1 else "OFF")
        state = str(state_value).strip().lower() in {"on", "high", "1", "true", "close"}
        output_index = self._map_output_pin(pin)
        action = _DigitalOutputAction(output_index, state, self._deps.shared_string)
        self._inject_and_wait(action, timeout_s=2.0)
        return None

    def _execute_input(self, command: ProgramCommand, index: int) -> int | None:
        selector = self._first_value(command, "n", command.args[0] if command.args else 1)
        expected = self._first_value(command, "value", command.args[1] if len(command.args) > 1 else 1)
        expected_bool = str(expected).strip().lower() in {"1", "true", "on", "high"}
        timeout_s = float(command.kwargs.get("timeout_s", self._deps.config_getter("failsafe.workspace_timeout_s", 10)))
        started_at = time.time()
        while time.time() - started_at <= timeout_s:
            self._check_stopped()
            self._wait_if_paused()
            value = self._deps.modbus_value_getter(selector)
            if value is None and isinstance(selector, (int, float)):
                value = self._deps.modbus_value_getter(int(selector) - 1)
            if value is not None and bool(value) == expected_bool:
                self._log(f"Input({selector}) matched {expected_bool}", "info")
                return None
            time.sleep(0.05)
        raise PAROL6Error(f"Input({selector}) timed out waiting for {expected_bool}")

    def _execute_delay(self, command: ProgramCommand, index: int) -> int | None:
        duration_ms = float(self._first_value(command, "ms", command.args[0] if command.args else 0.0))
        remaining_s = max(duration_ms, 0.0) / 1000.0
        while remaining_s > 0:
            self._check_stopped()
            self._wait_if_paused()
            sleep_s = min(0.05, remaining_s)
            started_at = time.time()
            self._stop_event.wait(sleep_s)
            remaining_s -= max(0.0, time.time() - started_at)
        self._check_stopped()
        return None

    def _execute_loop(self, command: ProgramCommand, index: int) -> int | None:
        if self._single_line is not None:
            self._log("Loop step inspected", "info")
            return None
        if self._loop_frame is not None:
            raise PAROL6Error("Nested Loop is not supported in v1.1")
        remaining = int(command.args[0] if command.args else self._first_value(command, "count", 0))
        self._loop_frame = {"start_index": index, "remaining": remaining}
        return None

    def _execute_end_loop(self, command: ProgramCommand, index: int) -> int | None:
        if self._single_line is not None:
            self._log("EndLoop step inspected", "info")
            return None
        if self._loop_frame is None:
            raise PAROL6Error("EndLoop without matching Loop")
        if self._loop_frame["remaining"] == 0:
            return self._loop_frame["start_index"] + 1
        if self._loop_frame["remaining"] > 1:
            self._loop_frame["remaining"] -= 1
            return self._loop_frame["start_index"] + 1
        self._loop_frame = None
        return None

    def _execute_vision(self, command: ProgramCommand, index: int) -> int | None:
        self._ensure_motion_safe()

        # Check if model mode is active and a bundle is available
        method = str(self._deps.config_getter("vision.detection_method", "adaptive")).strip().lower()
        if method == "model" and self._deps.bundle_getter is not None:
            bundle = self._deps.bundle_getter()
            if bundle is not None and bundle.selongsong_box is not None:
                return self._execute_vision_model(bundle, index)
            # Fallback to contour if no bundle
            self._log("Model detection unavailable; falling back to contour mode", "warn")

        # Contour-based flow
        result = self._deps.vision_getter()
        if result is None:
            raise PAROL6Error("vision(): No objects detected")

        x_pick, y_pick = result.pick_point_world
        status, _ = self._deps.workspace_validator.check(x_pick, y_pick)
        if status == WorkspaceValidator.ValidationResult.OUT_OF_BOUNDS:
            self._deps.research_logger.record("workspace_violation", 1)
            self._deps.failsafe.trigger_workspace_violation(x_pick, y_pick)
            raise WorkspaceViolationError(x_pick, y_pick, self._deps.config_getter("workspace", {}))
        if status == WorkspaceValidator.ValidationResult.NEAR_BOUNDARY:
            self._deps.research_logger.record("workspace_warning", 1)
            self._log("Vision target is near workspace boundary", "warn")

        self._execute_ibvs_pick(x_pick, y_pick)
        return None

    def _execute_vision_model(self, bundle: DetectionBundle, index: int) -> int | None:
        """Handle vision pick with model-based safety gate (SAFE/MARGINAL/UNSAFE/UNKNOWN)."""
        safety = bundle.pick_safety

        if safety == "UNSAFE":
            self._deps.research_logger.record("workspace_violation", 1)
            self._log("UNSAFE: pick blocked — safe zone too small", "error")
            if self._deps.modbus_writer is not None:
                self._deps.modbus_writer("error_flag", True)
            raise WorkspaceViolationError(0.0, 0.0, {"reason": "UNSAFE pick safety"})

        if safety == "MARGINAL":
            self._log("MARGINAL: waiting for user confirmation", "warn")
            self._deps.research_logger.record("marginal_event", 1)
            self._marginal_event.clear()
            self._marginal_action = ""
            self.marginal_detected.emit(bundle)
            # Wait for user action or timeout
            timeout_s = float(self._deps.config_getter("vision.marginal_confirm_timeout_s", 10))
            self._marginal_event.wait(timeout=timeout_s)
            self._check_stopped()
            if self._marginal_action == "confirm":
                self._log("MARGINAL confirmed by user — proceeding with pick", "info")
            else:
                self._log("MARGINAL skipped (user/timeout) — moving to next command", "warn")
                self._deps.research_logger.record("marginal_skip", 1)
                return None  # skip to next command

        if safety == "UNKNOWN":
            self._log("UNKNOWN: fixture not detected — picking with fallback margin", "warn")
            self._deps.research_logger.record("unknown_fixture", 1)

        # Proceed with pick
        if bundle.pick_point_world is None:
            raise PAROL6Error("vision(): No valid pick point from model detection")

        x_pick, y_pick = bundle.pick_point_world
        ws_status, _ = self._deps.workspace_validator.check(x_pick, y_pick)
        if ws_status == WorkspaceValidator.ValidationResult.OUT_OF_BOUNDS:
            self._deps.research_logger.record("workspace_violation", 1)
            self._deps.failsafe.trigger_workspace_violation(x_pick, y_pick)
            raise WorkspaceViolationError(x_pick, y_pick, self._deps.config_getter("workspace", {}))

        self._execute_ibvs_pick(x_pick, y_pick)
        return None

    def _execute_ibvs_pick(self, x_pick: float, y_pick: float) -> None:
        z_mm = float(self._deps.config_getter("workspace.z_fixed_mm", 200.0))
        speed_pct = float(self._deps.config_getter("robot.default_speed_pct", 30))
        current_pose = self._current_pose()
        target_pose = current_pose.copy()
        target_pose.t = np.array([float(x_pick) / 1000.0, float(y_pick) / 1000.0, z_mm / 1000.0], dtype=float)
        self._move_to_pose(target_pose, speed_pct=speed_pct)

        reactor = self._build_contour_ibvs_reactor(z_mm, speed_pct)
        ibvs_steps = 0
        aligned = False
        if reactor is None:
            self._log("IBVS contour adapter unavailable; using coarse pick pose", "warn")
        else:
            ibvs_steps, aligned = self._run_contour_ibvs_loop(reactor)
        self._deps.research_logger.record("ibvs_iterations", ibvs_steps)
        self._deps.research_logger.record("ibvs_aligned", int(aligned))
        self._execute_gripper(ProgramCommand(name="Gripper", args=["close"], kwargs={}, source="Gripper(close)"), -1)
        self._deps.research_logger.record("vision_pick_success", 1)
        if aligned:
            self._log(f"Contour IBVS completed after {ibvs_steps} correction step(s)", "info")
        elif ibvs_steps > 0:
            self._log("Contour IBVS stopped before deadband; pick completed from latest coarse pose", "warn")
        else:
            self._log("Pick completed from coarse pose without additional contour IBVS jog", "warn")

    def _build_contour_ibvs_reactor(self, z_mm: float, speed_pct: float) -> CentroidIBVSReactor | None:
        calibration = self._deps.config_getter("vision.calibration", {}) or {}
        intrinsic_matrix = calibration.get("intrinsic_matrix") or []
        fx = fy = None
        desired_uv = None
        if len(intrinsic_matrix) >= 2 and len(intrinsic_matrix[0]) >= 3 and len(intrinsic_matrix[1]) >= 3:
            fx = float(intrinsic_matrix[0][0])
            fy = float(intrinsic_matrix[1][1])
            desired_uv = (float(intrinsic_matrix[0][2]), float(intrinsic_matrix[1][2]))
        else:
            image_size = calibration.get("image_size") or [640, 480]
            if len(image_size) >= 2:
                desired_uv = (float(image_size[0]) / 2.0, float(image_size[1]) / 2.0)

        try:
            tags_queue: queue.Queue = queue.Queue(maxsize=1)
            return CentroidIBVSReactor(
                detected_tags_queue=tags_queue,
                jog_control=self._deps.jog_control,
                shared_string=self._deps.shared_string,
                desired_uv=desired_uv,
                fx=fx,
                fy=fy,
                depth_m=float(self._deps.config_getter("vision.ibvs_depth_m", max(z_mm, 1.0) / 1000.0)),
                lam=float(self._deps.config_getter("vision.ibvs_lambda", 1.0)),
                deadband_px=float(self._deps.config_getter("vision.ibvs_deadband_px", 6.0)),
                max_step_m=float(self._deps.config_getter("vision.ibvs_max_step_mm", 5.0)) / 1000.0,
                invert_x=bool(self._deps.config_getter("vision.ibvs_invert_x", True)),
                invert_y=bool(self._deps.config_getter("vision.ibvs_invert_y", True)),
                frame=str(self._deps.config_getter("vision.ibvs_frame", "WRF")).strip().upper(),
                speed_pct_default=float(speed_pct),
            )
        except Exception as exc:
            self._log(f"Failed to initialize contour IBVS adapter: {exc}", "warn")
            return None

    def _run_contour_ibvs_loop(self, reactor: CentroidIBVSReactor) -> tuple[int, bool]:
        max_iterations = max(1, int(self._deps.config_getter("vision.ibvs_max_iterations", 3)))
        detection_timeout_s = float(self._deps.config_getter("vision.ibvs_detection_timeout_s", 0.5))
        settle_delay_s = float(self._deps.config_getter("vision.ibvs_settle_delay_s", 0.12))
        stagnation_px = float(self._deps.config_getter("vision.ibvs_stagnation_px", 0.5))
        stagnation_limit = max(1, int(self._deps.config_getter("vision.ibvs_stagnation_limit", 1)))
        stagnant_cycles = 0

        for iteration in range(1, max_iterations + 1):
            detection = self._wait_for_detection(timeout_s=detection_timeout_s)
            if detection is None:
                self._log("Contour IBVS skipped because vision detection is unavailable", "warn")
                return iteration - 1, False

            self._publish_contour_ibvs_feature(reactor, detection)
            error_u, error_v = self._compute_contour_ibvs_error(reactor, detection)
            error_norm = math.hypot(error_u, error_v)
            self._deps.research_logger.record("centroid_error_px", round(error_norm, 3))
            if error_norm <= reactor.deadband_px:
                return iteration - 1, True

            action = reactor.plan(self._deps.robot_data)
            self._deps.research_logger.record(
                "ibvs_error_px",
                round(error_norm, 3),
                f"u={error_u:.3f}, v={error_v:.3f}",
            )
            self._inject_and_wait(action, timeout_s=2.0)
            time.sleep(settle_delay_s)

            next_detection = self._wait_for_detection(timeout_s=detection_timeout_s)
            if next_detection is None:
                return iteration, False
            next_error_u, next_error_v = self._compute_contour_ibvs_error(reactor, next_detection)
            next_error_norm = math.hypot(next_error_u, next_error_v)
            improvement = error_norm - next_error_norm
            self._log(
                f"Contour IBVS step {iteration}: error {error_norm:.2f}px -> {next_error_norm:.2f}px",
                "info",
            )
            if next_error_norm <= reactor.deadband_px:
                self._deps.research_logger.record("centroid_error_px", round(next_error_norm, 3))
                return iteration, True
            if improvement <= stagnation_px:
                stagnant_cycles += 1
                if stagnant_cycles >= stagnation_limit:
                    self._deps.research_logger.record("ibvs_stagnation", stagnant_cycles, f"error={next_error_norm:.3f}")
                    self._log("Contour IBVS stalled; stopping bounded correction loop", "warn")
                    return iteration, False
            else:
                stagnant_cycles = 0

        return max_iterations, False

    def _wait_for_detection(self, timeout_s: float) -> DetectionResult | None:
        deadline = time.time() + max(timeout_s, 0.0)
        latest = self._deps.vision_getter()
        while latest is None and time.time() <= deadline:
            self._check_stopped()
            self._wait_if_paused()
            time.sleep(0.02)
            latest = self._deps.vision_getter()
        return latest

    @staticmethod
    def _publish_contour_ibvs_feature(reactor: CentroidIBVSReactor, detection: DetectionResult) -> None:
        while True:
            try:
                reactor.tags_q.get_nowait()
            except queue.Empty:
                break
        reactor.tags_q.put_nowait([
            {
                "id": 0,
                "center": tuple(detection.pick_point_px),
                "source": "contour_pick_point",
            }
        ])

    @staticmethod
    def _compute_contour_ibvs_error(
        reactor: CentroidIBVSReactor,
        detection: DetectionResult,
    ) -> tuple[float, float]:
        target_u, target_v = reactor.desired_uv
        current_u, current_v = detection.pick_point_px
        return target_u - float(current_u), target_v - float(current_v)

    def _move_to_pose(self, target_pose: SE3, speed_pct: float) -> None:
        q0 = self._current_joint_radians()
        solution = PAROL6_ROBOT.robot.ikine_LMS(target_pose, q0=q0, ilimit=30, mask=[1, 1, 1, 1, 1, 1])
        if getattr(solution, "success", False) is False:
            solution = PAROL6_ROBOT.robot.ikine_LMS(target_pose, q0=q0, ilimit=60)
        if getattr(solution, "success", True) is False or not hasattr(solution, "q"):
            raise IKFailureError("Failed to solve inverse kinematics")
        q_deg = [float(np.rad2deg(value)) for value in np.asarray(solution.q).reshape(-1)[:6]]
        action = Move2JointsAction(q_deg=q_deg, v_pct=float(speed_pct), shared_string=self._deps.shared_string)
        self._inject_and_wait(action, timeout_s=30.0)

    def _build_absolute_pose(self, command: ProgramCommand) -> SE3:
        x_mm = float(self._first_value(command, "x", 0.0)) / 1000.0
        y_mm = float(self._first_value(command, "y", 0.0)) / 1000.0
        z_mm = float(self._first_value(command, "z", self._deps.config_getter("workspace.z_fixed_mm", 200.0))) / 1000.0
        r_deg = float(self._first_value(command, "r", 0.0))
        p_deg = float(self._first_value(command, "p", 0.0))
        yaw_deg = float(self._first_value(command, "yaw", 0.0))
        return SE3(x_mm, y_mm, z_mm) * SE3.Rx(r_deg, unit="deg") * SE3.Ry(p_deg, unit="deg") * SE3.Rz(yaw_deg, unit="deg")

    def _resolve_joint_target(self, command: ProgramCommand) -> list[float]:
        current = self._current_joint_degrees()
        aliases = ["j1", "j2", "j3", "j4", "j5", "j6"]
        for index, alias in enumerate(aliases):
            if alias in command.kwargs:
                current[index] = float(command.kwargs[alias])
            elif len(command.args) > index:
                current[index] = float(command.args[index])
        return current

    def _inject_and_wait(self, action: Action, timeout_s: float) -> None:
        if not self._deps.commander.inject_action(action):
            raise PAROL6Error("Commander queue is full")
        started_at = time.time()
        while not action.is_done():
            self._check_stopped()
            if time.time() - started_at > timeout_s:
                raise PAROL6Error(f"Action timed out after {timeout_s:.1f}s")
            time.sleep(0.02)

    def _ensure_motion_safe(self) -> None:
        if not self._deps.failsafe.is_safe_to_move():
            raise FailsafeTriggeredError("Cannot execute motion: failsafe active")

    def _current_joint_radians(self) -> np.ndarray:
        return np.array([PAROL6_ROBOT.STEPS2RADS(self._deps.robot_data.position[index], index) for index in range(6)], dtype=float)

    def _current_joint_degrees(self) -> list[float]:
        return [float(PAROL6_ROBOT.STEPS2DEG(self._deps.robot_data.position[index], index)) for index in range(6)]

    def _current_pose(self) -> SE3:
        return PAROL6_ROBOT.robot.fkine(self._current_joint_radians())

    def _map_output_pin(self, pin: int) -> int:
        if pin in (1, 2):
            return pin + 1
        if pin in (2, 3):
            return pin
        raise PAROL6Error(f"Unsupported Output() pin: {pin}")

    @staticmethod
    def _first_value(command: ProgramCommand, key: str, default: Any) -> Any:
        return command.kwargs.get(key, default)

    def _check_stopped(self) -> None:
        if self._stop_event.is_set():
            raise PAROL6Error("Program stopped")

    def _wait_if_paused(self) -> None:
        while self._pause_event.is_set():
            self._check_stopped()
            time.sleep(0.05)

    def _log(self, message: str, level: str = "info") -> None:
        self.log_message.emit(message, level)