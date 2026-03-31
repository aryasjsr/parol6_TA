from __future__ import annotations

import threading
import time
from multiprocessing import Array, Value
from pathlib import Path
from typing import Any

from PyQt6.QtCore import QObject, pyqtSignal
import numpy as np
import scipy

from backend.config_manager import ConfigManager
from backend.failsafe_manager import FailsafeManager
from backend.jog_source_lock import JogSourceLock
from backend.modbus_client import ModbusPollerThread
from backend.research_logger import ResearchLogger
from backend.serial_worker import SerialWorkerThread
from program.program_executor import ProgramExecutor, ProgramExecutorDependencies
from program.program_model import ProgramModel
from tools.shared_struct import RobotInputData, RobotOutputData
from vision.camera_pipeline import VisionWorkerThread
from vision.object_detector import DetectionResult
from vision.workspace_validator import WorkspaceValidator

backend_dir = Path(__file__).resolve().parent
project_dir = backend_dir.parent
import sys

if str(backend_dir) not in sys.path:
    sys.path.insert(0, str(backend_dir))
if str(project_dir) not in sys.path:
    sys.path.insert(0, str(project_dir))

if not hasattr(scipy, "randn"):
    scipy.randn = np.random.randn

from Commander import Commander, Mode
from Reactor import GUIReactor
from action import TimedCartesianJogAction, TimedJointJogAction
from commander_loop import commander_loop


class AppRuntime(QObject):
    """Own shared robot state, commander loop, and serial worker lifecycle."""

    log_message = pyqtSignal(str, str)
    connection_changed = pyqtSignal(bool, str)
    snapshot_updated = pyqtSignal(dict)
    queue_updated = pyqtSignal(list)
    vision_frame_ready = pyqtSignal(object)
    vision_detection_changed = pyqtSignal(dict)
    vision_status_changed = pyqtSignal(str, str)
    vision_calibration_changed = pyqtSignal(dict)
    modbus_snapshot_changed = pyqtSignal(dict)
    modbus_connection_changed = pyqtSignal(bool, str)
    program_line_changed = pyqtSignal(int)
    program_state_changed = pyqtSignal(str)
    program_finished = pyqtSignal(bool, str)
    metric_recorded = pyqtSignal(str, float)        # metric_name, value
    failsafe_event = pyqtSignal(str, str)            # message, level
    sim_position_changed = pyqtSignal(list)           # 6 floats, radians
    jog_source_changed = pyqtSignal(str)              # 'gui' | 'modbus' | 'none'

    def __init__(self) -> None:
        super().__init__()
        self.config = ConfigManager.instance()
        self.logger = ResearchLogger.instance()
        self.failsafe = FailsafeManager.instance()
        self.workspace_validator = WorkspaceValidator(self.config)

        self.command_data = RobotOutputData()
        self.command_data.initialize(
            position_init=[0, 0, 0, 0, 0, 0],
            speed_init=[0, 0, 0, 0, 0, 0],
            affected_joint_init=[1, 1, 1, 1, 1, 1, 1, 1],
            inout_init=[0, 0, 0, 0, 1, 0, 0, 0],
            gripper_init=[0, 0, 0, 0, 0, 1],
            command_init=255,
            timeout_init=0,
        )
        self.robot_data = RobotInputData()
        self.robot_data.initialize(
            position_init=[0, 0, 0, 0, 0, 0],
            speed_init=[0, 0, 0, 0, 0, 0],
            homed_init=[0, 0, 0, 0, 0, 0, 0, 0],
            inout_init=[0, 0, 0, 0, 1, 0, 0, 0],
            temperature_error_init=[0, 0, 0, 0, 0, 0, 0, 0],
            position_error_init=[0, 0, 0, 0, 0, 0, 0, 0],
            gripper_init=[0, 0, 0, 0, 0, 0],
        )

        self.shared_string = Array("c", b" " * 120)
        self.robot_mode = Value("i", Mode.GUI)
        self.general_data = Array("i", [3, 3000000], lock=False)
        self.joint_jog_buttons = Array("i", [0] * 12, lock=False)
        self.cart_jog_buttons = Array("i", [0] * 12, lock=False)
        self.jog_control = Array("i", [30, 0, 1, 0], lock=False)
        self.buttons = Array("i", [0, 0, 0, 0, 1, 1, 0, 0, 0], lock=False)

        self.gui_reactor = GUIReactor(
            shared_string=self.shared_string,
            Joint_jog_buttons=self.joint_jog_buttons,
            Cart_jog_buttons=self.cart_jog_buttons,
            Jog_control=self.jog_control,
            Buttons=self.buttons,
        )
        self.commander = Commander(
            cmd_data=self.command_data,
            robot_data=self.robot_data,
            plugins={Mode.GUI: self.gui_reactor},
            current_mode=self.robot_mode,
        )

        self._shutdown_event = threading.Event()
        self.sync_sema = threading.Semaphore(0)
        self.ready_sema = threading.Semaphore(0)
        self._commander_thread = threading.Thread(
            target=commander_loop,
            args=(self.commander, self._shutdown_event, self.sync_sema, self.ready_sema),
            daemon=True,
        )
        self._poll_thread = threading.Thread(target=self._poll_status, daemon=True)
        self._serial_worker: SerialWorkerThread | None = None
        self._vision_worker: VisionWorkerThread | None = None
        self._modbus_worker: ModbusPollerThread | None = None
        self._program_executor: ProgramExecutor | None = None
        self._modbus_snapshot: dict[str, Any] = {}
        self._last_program_rows: list[dict[str, str]] = []
        self._last_log_value = ""
        self._started = False

        # --- Phase 8: sim preview position (radians) ---
        import tools.PAROL6_ROBOT as _P6
        self._parol6 = _P6
        self._sim_position = [0.0] * 6
        self._preview_only = False

        # --- Phase 9: jog interlock ---
        _timeout = float(self.config.get("modbus.jog_lock_timeout_s", 5.0))
        self._jog_lock = JogSourceLock(timeout_s=_timeout)

    def start(self) -> None:
        """Start background commander and status poll loops once."""
        if self._started:
            return
        self._started = True
        self._commander_thread.start()
        self._poll_thread.start()
        self.failsafe.register_metric_logger(self.logger.record)

        # Relay failsafe events to the research tab
        self.failsafe.estop_triggered.connect(
            lambda reason: self.failsafe_event.emit(f"E-STOP: {reason}", "estop")
        )
        self.failsafe.warning_raised.connect(
            lambda message: self.failsafe_event.emit(message, "warn")
        )
        self.failsafe.state_changed.connect(
            lambda state: self.failsafe_event.emit(f"State → {state}", "info")
        )

        self.log_message.emit("Runtime initialized", "info")

    def stop(self) -> None:
        """Stop runtime background workers and unblock waiting semaphores."""
        self.stop_program()
        self.stop_modbus()
        self.stop_vision()
        if self._serial_worker is not None:
            self._serial_worker.stop()
            self._serial_worker = None
        self._shutdown_event.set()
        self.sync_sema.release()
        self.ready_sema.release()
        if self._commander_thread.is_alive():
            self._commander_thread.join(timeout=2.0)
        if self._poll_thread.is_alive():
            self._poll_thread.join(timeout=2.0)
        self._started = False

    def connect_serial(self, port_name: str, baudrate: int) -> None:
        """Start the serial worker for the selected port."""
        if self._serial_worker is not None and self._serial_worker.isRunning():
            self.log_message.emit("Serial worker already running", "warn")
            return
        self.general_data[0] = self._extract_port_index(port_name, self.general_data[0])
        self.general_data[1] = int(baudrate)
        self._serial_worker = SerialWorkerThread(self, port_name, int(baudrate))
        self._serial_worker.connection_changed.connect(self._on_connection_changed)
        self._serial_worker.snapshot_ready.connect(self.snapshot_updated)
        self._serial_worker.error_occurred.connect(lambda msg: self.log_message.emit(msg, "error"))
        self._serial_worker.log_message.connect(self.log_message)
        self.failsafe.register_serial_disable_callback(self._serial_worker.send_disable_now)
        self._serial_worker.start()

    def disconnect_serial(self) -> None:
        """Stop the current serial worker if one is active."""
        if self._serial_worker is None:
            return
        self._serial_worker.stop()
        self._serial_worker = None
        self.connection_changed.emit(False, "Disconnected")

    def request_enable(self) -> None:
        """Queue enable command via GUI reactor state."""
        self.commander.set_enable(True)
        self.buttons[1] = 1
        self.log_message.emit("Enable requested", "info")

    def request_disable(self) -> None:
        """Queue disable command via GUI reactor state."""
        self.buttons[2] = 1
        self.commander.set_enable(False)
        self.log_message.emit("Disable requested", "warn")

    def request_home(self) -> None:
        """Queue homing command via GUI reactor state."""
        self.buttons[0] = 1
        self.log_message.emit("Home requested", "info")

    def request_clear_error(self) -> None:
        """Queue clear-error command via GUI reactor state."""
        self.buttons[3] = 1
        self.log_message.emit("Clear error requested", "info")

    def set_speed_percent(self, value: int) -> None:
        """Update GUI jog speed percent for the GUI reactor."""
        self.jog_control[0] = int(value)
        self.log_message.emit(f"Speed set to {value}%", "info")

    def set_frame_mode(self, wrf_enabled: bool) -> None:
        """Update GUI jog frame selection for the GUI reactor."""
        self.jog_control[2] = 1 if wrf_enabled else 0
        self.log_message.emit(
            f"Frame set to {'WRF' if wrf_enabled else 'TRF'}",
            "info",
        )

    def set_preview_only(self, flag: bool) -> None:
        """Toggle preview-only mode (jog updates 3D sim but not hardware)."""
        was_preview = self._preview_only
        self._preview_only = flag
        state = "ON" if flag else "OFF"

        if was_preview and not flag:
            # Leaving preview → sync sim back to actual hardware position
            self.commander.clear_queue()
            hw_pos = list(self.robot_data.position)
            self.sync_sim_from_hardware(hw_pos)
            self.log_message.emit(
                "Preview OFF – sim synced to hardware, action queue cleared", "warn"
            )
        elif flag:
            self.log_message.emit(f"Preview mode {state}", "info")

    def jog_joint(self, joint_id: int, direction: int) -> None:
        """Inject one short timed joint jog action (GUI source)."""
        if self._preview_only:
            # Preview mode: update simulation only, no hardware
            self._update_sim_joint(joint_id, direction)
            return

        # Real-time mode: send to hardware only if connected + enabled
        if not self.is_connected() or not self.commander.is_enabled():
            self.log_message.emit("Jog blocked – robot not connected/enabled", "warn")
            return

        if not self._jog_lock.acquire("gui"):
            self.log_message.emit("Jog locked by Modbus", "warn")
            return
        self._jog_lock.renew("gui")
        self.jog_source_changed.emit("gui")

        action = TimedJointJogAction(
            joint_id=joint_id,
            percent=float(self.jog_control[0]),
            direction=direction,
            duration_s=0.15,
            shared_string=self.shared_string,
        )
        self.commander.inject_action(action)

        self.log_message.emit(f"Joint jog J{joint_id + 1} {'+' if direction > 0 else '-'}", "info")

    def jog_cartesian(self, axis: str, direction: int) -> None:
        """Inject one short timed cartesian jog action."""
        if self._preview_only:
            self._update_sim_cartesian(axis, direction)
            return

        if not self.is_connected() or not self.commander.is_enabled():
            self.log_message.emit("Cartesian jog blocked – robot not connected/enabled", "warn")
            return

        step = 0.005 * direction
        params: dict[str, float | str] = {
            "dx": 0.0,
            "dy": 0.0,
            "dz": 0.0,
            "rx": 0.0,
            "ry": 0.0,
            "rz": 0.0,
            "frame": "WRF" if self.jog_control[2] == 1 else "TRF",
            "speed_pct": float(self.jog_control[0]),
            "interval_s": 0.01,
            "duration_s": 0.15,
            "shared_string": self.shared_string,
        }
        rotation_step = 1.5 * direction
        if axis == "X":
            params["dx"] = step
        elif axis == "Y":
            params["dy"] = step
        elif axis == "Z":
            params["dz"] = step
        elif axis == "Rx":
            params["rx"] = rotation_step
        elif axis == "Ry":
            params["ry"] = rotation_step
        elif axis == "Rz":
            params["rz"] = rotation_step
        if not self._jog_lock.acquire("gui"):
            self.log_message.emit("Jog locked by Modbus", "warn")
            return
        self._jog_lock.renew("gui")
        self.jog_source_changed.emit("gui")

        action = TimedCartesianJogAction(**params)
        self.commander.inject_action(action)
        self.log_message.emit(f"Cartesian jog {axis} {'+' if direction > 0 else '-'}", "info")

    # ------------------------------------------------------------------
    # Phase 8 helpers
    # ------------------------------------------------------------------
    def set_sim_joint_absolute(self, joint_id: int, degrees: float) -> None:
        """Set a single joint to an absolute degree value (preview sim only).

        Sliders only affect the 3D preview – they never send commands to
        hardware.  When preview mode is OFF, this call is ignored so the
        user cannot accidentally desync the sim from the real robot.
        """
        if not self._preview_only:
            return
        import math
        rad = math.radians(degrees)
        lo, hi = self._parol6.Joint_limits_radian[joint_id]
        self._sim_position[joint_id] = max(lo, min(hi, rad))
        self.sim_position_changed.emit(list(self._sim_position))

    def _update_sim_joint(self, joint_id: int, direction: int) -> None:
        """Advance the sim-preview position for one joint by a small delta."""
        step_deg = float(self.jog_control[0]) * 0.05 * direction
        step_rad = self._parol6.DEG2RAD(step_deg)
        self._sim_position[joint_id] += step_rad
        lo, hi = self._parol6.Joint_limits_radian[joint_id]
        self._sim_position[joint_id] = max(lo, min(hi, self._sim_position[joint_id]))
        self.sim_position_changed.emit(list(self._sim_position))

    def _update_sim_cartesian(self, axis: str, direction: int) -> None:
        """Advance sim-preview via cartesian delta + IK."""
        import numpy as np
        from spatialmath import SE3

        robot = self._parol6.robot
        q = np.array(self._sim_position)

        T = robot.fkine(q)

        lin_step = 0.005 * direction   # 5 mm per tick
        rot_step = 1.5 * direction       # 1.5 deg per tick

        T_new = SE3(T)

        # --- Translation: WRF vs TRF ---
        if axis in ("X", "Y", "Z"):
            d = np.zeros(3)
            idx = {"X": 0, "Y": 1, "Z": 2}[axis]
            d[idx] = lin_step
            if self.jog_control[2] == 1:  # WRF
                T_new.t = T_new.t + d
            else:  # TRF
                T_new.t = T_new.t + T_new.R @ d

        # --- Rotation: always post-multiply (tool-frame) ---
        elif axis == "Rx":
            T_new = T_new * SE3.Rx(rot_step, unit='deg')
        elif axis == "Ry":
            T_new = T_new * SE3.Ry(rot_step, unit='deg')
        elif axis == "Rz":
            T_new = T_new * SE3.Rz(rot_step, unit='deg')
        else:
            return

        sol = robot.ikine_LMS(T_new, q0=q, ilimit=10)
        if not sol.success:
            self.log_message.emit("[Preview] Cartesian IK failed – unreachable", "warn")
            return

        q_new = sol.q
        for i in range(6):
            lo, hi = self._parol6.Joint_limits_radian[i]
            q_new[i] = max(lo, min(hi, q_new[i]))

        self._sim_position = list(q_new)
        self.sim_position_changed.emit(list(self._sim_position))

    def sync_sim_from_hardware(self, positions: list[int]) -> None:
        """Overwrite preview state from hardware position (steps)."""
        for i in range(min(6, len(positions))):
            self._sim_position[i] = self._parol6.STEPS2RADS(positions[i], i)
        self.sim_position_changed.emit(list(self._sim_position))

    def is_connected(self) -> bool:
        """Return True when the serial worker is actively connected."""
        return self._serial_worker is not None and self._serial_worker.isRunning()

    # ------------------------------------------------------------------
    # Phase 9 helpers
    # ------------------------------------------------------------------
    def on_modbus_jog_requested(self, joint_id: int, direction: int) -> None:
        """Slot for Modbus jog trigger – obeys jog interlock."""
        if not self._jog_lock.acquire("modbus"):
            self.log_message.emit("Jog locked by GUI", "warn")
            return
        self._jog_lock.renew("modbus")
        self.jog_source_changed.emit("modbus")
        speed_pct = float(self.config.get("modbus.jog_speed_pct", 20))
        action = TimedJointJogAction(
            joint_id=joint_id,
            percent=speed_pct,
            direction=direction,
            duration_s=0.15,
            shared_string=self.shared_string,
        )
        self.commander.inject_action(action)
        self._update_sim_joint(joint_id, direction)
        self.log_message.emit(
            f"Modbus jog J{joint_id + 1} {'+' if direction > 0 else '-'}", "info",
        )

    def build_snapshot(self) -> dict[str, Any]:
        """Return a UI-facing runtime snapshot."""
        snapshot = self.commander.to_dict()
        snapshot["shared_log"] = self.shared_string.value.decode(errors="ignore").strip()
        return snapshot

    def start_vision(self, source: str | int | None = None, settings: dict | None = None) -> None:
        """Start or restart the vision worker using the current UI settings."""
        self.stop_vision()
        vision_source = source if source not in (None, "") else self.config.get("vision.video_source", 0)
        self._vision_worker = VisionWorkerThread(vision_source)
        self._vision_worker.frame_ready.connect(self.vision_frame_ready)
        self._vision_worker.objects_detected.connect(self._on_vision_objects_detected)
        self._vision_worker.status_changed.connect(self._on_vision_status_changed)
        self._vision_worker.calibration_updated.connect(self.vision_calibration_changed)
        if settings:
            self._vision_worker.apply_settings(settings)
        self._vision_worker.start()

    def stop_vision(self) -> None:
        """Stop the active vision worker if it is running."""
        if self._vision_worker is None:
            return
        self._vision_worker.stop()
        self._vision_worker = None

    def apply_vision_settings(self, settings: dict, persist: bool = False) -> None:
        """Update live vision settings and optionally persist them to config.json."""
        if persist:
            for key, value in settings.items():
                self.config.set(f"vision.{key}", value)
        if self._vision_worker is not None:
            self._vision_worker.apply_settings(settings)
        self.vision_status_changed.emit("info", "Vision settings applied")

    def save_workspace_settings(self, settings: dict) -> None:
        """Persist workspace configuration to config.json."""
        for key, value in settings.items():
            self.config.set(f"workspace.{key}", value)
        self.vision_status_changed.emit("info", "Workspace settings saved")

    def save_pick_zone_settings(self, settings: dict) -> None:
        """Persist safe pick zone configuration to config.json."""
        for key, value in settings.items():
            self.config.set(f"vision.{key}", value)
        if self._vision_worker is not None:
            self._vision_worker.apply_settings(settings)
        self.vision_status_changed.emit("info", "Safe pick zone saved")

    def capture_vision_snapshot(self) -> None:
        """Store the current vision frame as a calibration snapshot."""
        if self._vision_worker is None:
            self.vision_status_changed.emit("warn", "Vision must be running before taking a snapshot")
            return
        try:
            count = self._vision_worker.capture_snapshot()
        except Exception as exc:
            self.vision_status_changed.emit("error", f"Calibration snapshot failed: {exc}")
            return
        self.vision_status_changed.emit("info", f"Calibration snapshot stored ({count})")

    def run_vision_calibration(
        self,
        chessboard_size: tuple[int, int] = (9, 6),
        square_size_mm: float = 25.0,
    ) -> None:
        """Run camera calibration using the snapshots captured from the live feed."""
        if self._vision_worker is None:
            self.vision_status_changed.emit("warn", "Vision must be running before calibration")
            return
        try:
            summary = self._vision_worker.run_calibration(chessboard_size, square_size_mm)
        except Exception as exc:
            self.vision_status_changed.emit("error", f"Calibration failed: {exc}")
            return
        self.vision_status_changed.emit(
            "info",
            f"Calibration saved (fx={summary['fx']:.2f}, fy={summary['fy']:.2f})",
        )

    def get_config_snapshot(self) -> dict[str, Any]:
        """Expose the full config tree for UI initialization."""
        return self.config.as_dict()

    def start_modbus(self) -> None:
        """Start the Modbus poller thread using config.json settings."""
        if self._modbus_worker is not None and self._modbus_worker.isRunning():
            self.vision_status_changed.emit("warn", "Modbus worker already running")
            return
        self._modbus_worker = ModbusPollerThread()
        self._modbus_worker.connection_state_changed.connect(self._on_modbus_connection_changed)
        self._modbus_worker.register_updated.connect(self._on_modbus_register_updated)
        self._modbus_worker.error_occurred.connect(lambda message: self.log_message.emit(message, "error"))
        self._modbus_worker.trigger_received.connect(self._on_modbus_trigger_received)
        self._modbus_worker.modbus_jog_requested.connect(self.on_modbus_jog_requested)
        self._modbus_worker.start()

    def stop_modbus(self) -> None:
        """Stop the Modbus poller thread."""
        if self._modbus_worker is None:
            return
        self._modbus_worker.stop()
        self._modbus_worker = None
        self._modbus_snapshot = {}
        self.modbus_connection_changed.emit(False, "Disconnected")
        self.modbus_snapshot_changed.emit({})

    def save_modbus_config(
        self,
        ip: str,
        port: int,
        slave_id: int,
        addresses: list[dict[str, Any]],
    ) -> None:
        """Persist Modbus connection and mapping settings to config.json."""
        self.config.set("modbus.ip", ip)
        self.config.set("modbus.port", int(port))
        self.config.set("modbus.slave_id", int(slave_id))
        self.config.set("modbus.addresses", addresses)
        self.log_message.emit("Modbus configuration saved", "info")

    def get_modbus_snapshot(self) -> dict[str, Any]:
        return dict(self._modbus_snapshot)

    def read_modbus_value(self, selector: Any) -> Any:
        if selector in self._modbus_snapshot:
            return self._modbus_snapshot[selector]
        try:
            selector_int = int(selector)
        except Exception:
            selector_int = None
        if selector_int is None:
            return None
        addresses = self.config.get("modbus.addresses", [])
        for entry in addresses:
            if int(entry.get("address", -999)) == selector_int:
                return self._modbus_snapshot.get(entry.get("name"))
        return None

    def write_modbus_coil(self, name: str, value: bool) -> bool:
        if self._modbus_worker is None:
            raise ConnectionError("Modbus worker is not running")
        result = self._modbus_worker.write_coil_by_name(name, value)
        self._modbus_snapshot[name] = bool(value)
        self.modbus_snapshot_changed.emit(dict(self._modbus_snapshot))
        return result

    def run_program(self, rows: list[dict[str, str]]) -> None:
        """Execute the full program from UI rows in a background worker."""
        self._start_program_executor(rows, None)

    def step_program(self, rows: list[dict[str, str]], line_number: int) -> None:
        """Execute a single selected program line."""
        self._start_program_executor(rows, line_number)

    def stop_program(self) -> None:
        """Stop the active program executor if present."""
        if self._program_executor is None:
            return
        self._program_executor.stop()
        self._program_executor.wait(2000)
        self._program_executor = None
        self.program_state_changed.emit("idle")

    def pause_program(self) -> None:
        """Pause the active program executor between cooperative checkpoints."""
        if self._program_executor is None or not self._program_executor.isRunning():
            self.log_message.emit("No active program to pause", "warn")
            return
        self._program_executor.pause()

    def resume_program(self) -> None:
        """Resume a paused program executor."""
        if self._program_executor is None or not self._program_executor.isRunning():
            self.log_message.emit("No paused program to resume", "warn")
            return
        self._program_executor.resume()

    def toggle_program_pause(self) -> None:
        """Toggle cooperative pause/resume on the active program executor."""
        if self._program_executor is None or not self._program_executor.isRunning():
            self.log_message.emit("No active program to pause or resume", "warn")
            return
        if self._program_executor.is_paused():
            self._program_executor.resume()
        else:
            self._program_executor.pause()

    def is_program_running(self) -> bool:
        return self._program_executor is not None and self._program_executor.isRunning()

    def is_program_paused(self) -> bool:
        return self._program_executor is not None and self._program_executor.isRunning() and self._program_executor.is_paused()

    def record_metric(self, metric_name: str, value: float, note: str = "") -> None:
        """Record a metric to the CSV logger and emit a signal for the research UI."""
        self.logger.record(metric_name, value, note)
        self.metric_recorded.emit(metric_name, float(value))

    def emit_vision_calibration_summary(self) -> None:
        """Push the current calibration summary to the UI without starting vision."""
        if self._vision_worker is not None:
            self.vision_calibration_changed.emit(self._vision_worker.calibration_summary())
            return
        config_value = self.config.get("vision.calibration", {})
        intrinsic_matrix = config_value.get("intrinsic_matrix") or []
        distortion = config_value.get("distortion") or []
        summary = {
            "fx": round(float(intrinsic_matrix[0][0]), 3) if intrinsic_matrix else 0.0,
            "fy": round(float(intrinsic_matrix[1][1]), 3) if intrinsic_matrix else 0.0,
            "cx": round(float(intrinsic_matrix[0][2]), 3) if intrinsic_matrix else 0.0,
            "cy": round(float(intrinsic_matrix[1][2]), 3) if intrinsic_matrix else 0.0,
            "distortion": [round(float(value), 6) for value in distortion],
            "image_size": config_value.get("image_size", [640, 480]),
            "rms_error": config_value.get("rms_error"),
            "snapshots_captured": config_value.get("snapshots_captured", 0),
            "calibrated": bool(intrinsic_matrix and distortion),
        }
        self.vision_calibration_changed.emit(summary)

    def _on_connection_changed(self, connected: bool, description: str) -> None:
        self.connection_changed.emit(connected, description)
        self.logger.record("serial_connection", int(connected), description)

    def _poll_status(self) -> None:
        while not self._shutdown_event.is_set():
            try:
                snapshot = self.build_snapshot()
                self.snapshot_updated.emit(snapshot)
                self.queue_updated.emit(self.commander.get_action_queue())
                current_log = snapshot.get("shared_log", "")
                if current_log and current_log != self._last_log_value:
                    self._last_log_value = current_log
                    self.log_message.emit(current_log, "info")
            except RuntimeError:
                break
            time.sleep(0.01)

    @staticmethod
    def _extract_port_index(port_name: str, fallback: int) -> int:
        digits = "".join(character for character in str(port_name) if character.isdigit())
        return int(digits) if digits else int(fallback)

    def _on_vision_objects_detected(self, detections: list[DetectionResult]) -> None:
        if not detections:
            self.vision_detection_changed.emit({
                "status": "no_object",
                "workspace_message": "NO OBJECT",
            })
            return
        self.vision_detection_changed.emit(detections[0].as_dict())

    def _on_vision_status_changed(self, level: str, message: str) -> None:
        self.vision_status_changed.emit(level, message)
        self.log_message.emit(message, level)

    def _start_program_executor(self, rows: list[dict[str, str]], single_line: int | None) -> None:
        if self.is_program_running():
            self.log_message.emit("Program executor is already running", "warn")
            return
        model = ProgramModel.from_rows(rows)
        if not model.commands:
            self.log_message.emit("Program is empty", "warn")
            return
        self._last_program_rows = rows
        dependencies = ProgramExecutorDependencies(
            commander=self.commander,
            robot_data=self.robot_data,
            command_data=self.command_data,
            shared_string=self.shared_string,
            jog_control=self.jog_control,
            failsafe=self.failsafe,
            research_logger=self.logger,
            workspace_validator=self.workspace_validator,
            config_getter=self.config.get,
            vision_getter=lambda: self._vision_worker.get_latest_detection() if self._vision_worker is not None else None,
            modbus_value_getter=self.read_modbus_value,
            modbus_writer=self.write_modbus_coil if self._modbus_worker is not None else None,
        )
        self._program_executor = ProgramExecutor(model, dependencies, single_line=single_line)
        self._program_executor.log_message.connect(self.log_message)
        self._program_executor.line_changed.connect(self.program_line_changed)
        self._program_executor.state_changed.connect(self.program_state_changed)
        self._program_executor.finished_execution.connect(self._on_program_finished)
        if self._modbus_worker is not None:
            try:
                self.write_modbus_coil("cycle_done", False)
                self.write_modbus_coil("error_flag", False)
            except Exception:
                pass
        self._program_executor.start()

    def _on_modbus_connection_changed(self, connected: bool, description: str) -> None:
        self.modbus_connection_changed.emit(connected, description)
        level = "info" if connected else "warn"
        self.log_message.emit(f"Modbus {'connected' if connected else 'disconnected'}: {description}", level)

    def _on_modbus_register_updated(self, snapshot: dict) -> None:
        self._modbus_snapshot = dict(snapshot)
        self.modbus_snapshot_changed.emit(dict(self._modbus_snapshot))

    def _on_modbus_trigger_received(self) -> None:
        self.log_message.emit("Modbus trigger_pick received", "info")
        if self._last_program_rows and not self.is_program_running():
            self.run_program(self._last_program_rows)

    def _on_program_finished(self, success: bool, message: str) -> None:
        self.program_finished.emit(success, message)
        if self._modbus_worker is not None:
            try:
                self.write_modbus_coil("cycle_done", success)
                self.write_modbus_coil("error_flag", not success)
                if success and "trigger_pick" in self._modbus_snapshot:
                    self.write_modbus_coil("trigger_pick", False)
            except Exception as exc:
                self.log_message.emit(f"Failed updating Modbus completion flags: {exc}", "warn")
        self._program_executor = None
