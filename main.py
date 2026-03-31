from __future__ import annotations

# Monkey-patch scipy.randn removed in modern scipy but still referenced by
# roboticstoolbox-python 1.0.x (mobile/EKF.py).  Must run before any
# roboticstoolbox import.
import scipy as _scipy  # noqa: E402
if not hasattr(_scipy, "randn"):
    import numpy as _np
    _scipy.randn = _np.random.randn

import logging
import sys
from pathlib import Path

from PyQt6.QtWidgets import QApplication

from backend.failsafe_manager import FailsafeManager
from backend.flask_thread import FlaskThread
from backend.runtime_context import AppRuntime
from ui.main_window import MainWindow


def load_stylesheet() -> str:
    """Load the application QSS stylesheet from assets/styles."""
    stylesheet_path = Path(__file__).resolve().parent / "assets" / "styles" / "dark_theme.qss"
    return stylesheet_path.read_text(encoding="utf-8") if stylesheet_path.exists() else ""


def configure_logging() -> None:
    """Configure a simple console logger for local development."""
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )


def main() -> int:
    """Bootstrap the Qt application and wire the global failsafe signals."""
    configure_logging()
    app = QApplication(sys.argv)
    app.setStyleSheet(load_stylesheet())

    failsafe = FailsafeManager.instance()
    runtime = AppRuntime()
    runtime.start()

    window = MainWindow()
    flask_thread = FlaskThread(runtime)
    control = window.control_tab
    vision = window.vision_tab
    config_snapshot = runtime.get_config_snapshot()
    vision.apply_config(config_snapshot)
    modbus = window.modbus_tab
    modbus.apply_config(config_snapshot)
    research = window.research_tab

    window.estop_requested.connect(failsafe.trigger_estop)
    failsafe.state_changed.connect(window.set_safety_state)
    failsafe.warning_raised.connect(lambda message: window.append_log(message, "warn"))
    failsafe.estop_triggered.connect(
        lambda reason: window.append_log(f"E-Stop triggered: {reason}", "estop")
    )
    runtime.log_message.connect(window.append_log)
    runtime.connection_changed.connect(window.set_connection_status)
    runtime.snapshot_updated.connect(_handle_runtime_snapshot(window))
    runtime.vision_frame_ready.connect(vision.update_frame)
    runtime.vision_detection_changed.connect(vision.update_detection)
    runtime.vision_status_changed.connect(vision.update_status)
    runtime.vision_calibration_changed.connect(vision.update_calibration)
    runtime.modbus_connection_changed.connect(modbus.set_connection_status)
    runtime.modbus_snapshot_changed.connect(modbus.update_monitor)
    runtime.program_line_changed.connect(control.set_execution_line)
    runtime.program_state_changed.connect(control.update_program_state)
    runtime.program_state_changed.connect(
        lambda state: window.append_log(f"Program state: {state}", "info")
    )
    runtime.program_finished.connect(
        lambda success, message: _handle_program_finished(window, success, message)
    )

    # -- Research tab wiring --
    runtime.metric_recorded.connect(research.feed_metric)
    runtime.failsafe_event.connect(research.append_event)
    runtime.program_finished.connect(
        lambda success, _msg: research.feed_pick_result(success)
    )

    flask_thread.server_started.connect(
        lambda url: window.append_log(f"Flask API started at {url}", "info")
    )
    flask_thread.error_occurred.connect(lambda message: window.append_log(message, "error"))

    control.connect_button.clicked.connect(
        lambda: runtime.connect_serial(
            control.port_combo.currentText(),
            int(control.baud_combo.currentText()),
        )
    )
    control.disconnect_button.clicked.connect(runtime.disconnect_serial)
    control.enable_button.clicked.connect(runtime.request_enable)
    control.disable_button.clicked.connect(runtime.request_disable)
    control.home_button.clicked.connect(runtime.request_home)
    control.clear_error_button.clicked.connect(runtime.request_clear_error)
    control.speed_slider.valueChanged.connect(runtime.set_speed_percent)
    control.frame_combo.currentTextChanged.connect(
        lambda text: runtime.set_frame_mode(text.upper() == "WRF")
    )

    for index, button in enumerate(control.joint_minus_buttons):
        _j, _d = index, -1
        button.pressed.connect(
            lambda j=_j, d=_d: control._start_jog_repeat(lambda: runtime.jog_joint(j, d))
        )
        button.released.connect(control._stop_jog_repeat)
    for index, button in enumerate(control.joint_plus_buttons):
        _j, _d = index, 1
        button.pressed.connect(
            lambda j=_j, d=_d: control._start_jog_repeat(lambda: runtime.jog_joint(j, d))
        )
        button.released.connect(control._stop_jog_repeat)
    for index, slider in enumerate(control.joint_sliders):
        slider.valueChanged.connect(
            lambda val, jid=index: runtime.set_sim_joint_absolute(jid, val / 10.0)
        )
    for axis, button in control.cart_minus_buttons.items():
        _a, _d = axis, -1
        button.pressed.connect(
            lambda a=_a, d=_d: control._start_jog_repeat(lambda: runtime.jog_cartesian(a, d))
        )
        button.released.connect(control._stop_jog_repeat)
    for axis, button in control.cart_plus_buttons.items():
        _a, _d = axis, 1
        button.pressed.connect(
            lambda a=_a, d=_d: control._start_jog_repeat(lambda: runtime.jog_cartesian(a, d))
        )
        button.released.connect(control._stop_jog_repeat)

    control.run_button.clicked.connect(lambda: runtime.run_program(control.program_rows()))
    control.pause_button.clicked.connect(runtime.toggle_program_pause)
    control.stop_button.clicked.connect(runtime.stop_program)
    control.step_button.clicked.connect(
        lambda: runtime.step_program(control.program_rows(), control.selected_program_line())
    )

    vision.start_camera_button.clicked.connect(
        lambda: runtime.start_vision(
            vision.selected_camera_source(),
            vision.build_detection_settings(),
        )
    )
    vision.stop_camera_button.clicked.connect(runtime.stop_vision)
    vision.apply_detection_button.clicked.connect(
        lambda: runtime.apply_vision_settings(vision.build_detection_settings())
    )
    vision.save_detection_button.clicked.connect(
        lambda: runtime.apply_vision_settings(vision.build_detection_settings(), persist=True)
    )
    vision.save_pick_zone_button.clicked.connect(
        lambda: runtime.save_pick_zone_settings(vision.build_pick_zone_settings())
    )
    vision.save_workspace_button.clicked.connect(
        lambda: runtime.save_workspace_settings(vision.build_workspace_settings())
    )
    vision.snap_frame_button.clicked.connect(runtime.capture_vision_snapshot)
    vision.run_calibration_button.clicked.connect(
        lambda: _run_vision_calibration(runtime, vision)
    )
    vision.save_calibration_button.clicked.connect(
        lambda: runtime.apply_vision_settings(
            {
                "brightness": vision.build_detection_settings()["brightness"],
                "contrast": vision.build_detection_settings()["contrast"],
                "zoom": vision.build_detection_settings()["zoom"],
            },
            persist=True,
        )
    )
    runtime.emit_vision_calibration_summary()

    modbus.connect_button.clicked.connect(runtime.start_modbus)
    modbus.disconnect_button.clicked.connect(runtime.stop_modbus)
    modbus.save_button.clicked.connect(lambda: _save_modbus_config(runtime, modbus))
    modbus.add_row_button.clicked.connect(lambda: modbus.append_address_row())
    modbus.remove_row_button.clicked.connect(modbus.remove_selected_row)

    # -- Modbus jog config save button --
    if modbus.jog_save_button is not None:
        modbus.jog_save_button.clicked.connect(
            lambda: _save_modbus_jog_config(runtime, modbus)
        )

    # -- Preview mode toggle wiring --
    control.preview_mode_button.toggled.connect(runtime.set_preview_only)

    # -- Phase 9: Modbus jog signal wiring --
    runtime.jog_source_changed.connect(control.update_jog_source)
    runtime.jog_source_changed.connect(modbus.update_jog_source)

    # -- Phase 8: 3D Simulator (matplotlib embedded) --
    import tools.PAROL6_ROBOT as _P6
    control.init_simulator_canvases(_P6.robot)
    if control.simulation_view is not None:
        runtime.sim_position_changed.connect(control.simulation_view.update_joints)
    runtime.sim_position_changed.connect(control.update_preview_joints)
    if control.simulation_view_rt is not None:
        runtime.snapshot_updated.connect(
            lambda snap: _feed_realtime_canvas(snap, control.simulation_view_rt)
        )

    control.update_program_state("idle")

    window.append_log("Application bootstrap complete", "info")
    flask_thread.start()

    def shutdown() -> None:
        flask_thread.stop()
        runtime.stop()

    app.aboutToQuit.connect(shutdown)
    window.show()
    return app.exec()


def _handle_runtime_snapshot(window: MainWindow):
    """Build a slot that updates visible UI fragments from runtime snapshots."""

    def _slot(snapshot: dict) -> None:
        robot_data = snapshot.get("robot_data", {})
        positions = robot_data.get("position", [0, 0, 0, 0, 0, 0])
        window.control_tab.update_simulation_status(positions)

    return _slot


def _run_vision_calibration(runtime: AppRuntime, vision_tab) -> None:
    chessboard_size, square_size_mm = vision_tab.calibration_request()
    runtime.run_vision_calibration(chessboard_size, square_size_mm)


def _save_modbus_config(runtime: AppRuntime, modbus_tab) -> None:
    ip, port, slave_id, addresses = modbus_tab.build_config()
    runtime.save_modbus_config(ip, port, slave_id, addresses)


def _save_modbus_jog_config(runtime: AppRuntime, modbus_tab) -> None:
    """Persist jog speed and lock timeout from the Modbus tab."""
    cfg = runtime._cfg
    cfg.set("modbus.jog_speed_pct", modbus_tab.jog_speed_spin.value())
    cfg.set("modbus.jog_lock_timeout_s", modbus_tab.jog_timeout_spin.value())
    cfg.save()


def _feed_realtime_canvas(snapshot: dict, canvas) -> None:
    """Extract joint positions from a runtime snapshot and push to the 3D canvas."""
    robot_data = snapshot.get("robot_data", {})
    positions = robot_data.get("position", None)
    if positions:
        import math
        rads = [math.radians(p) for p in positions]
        canvas.update_joints(rads)


def _handle_program_finished(window: MainWindow, success: bool, message: str) -> None:
    window.control_tab.set_execution_line(None)
    window.append_log(message, "info" if success else "error")


if __name__ == "__main__":
    raise SystemExit(main())
