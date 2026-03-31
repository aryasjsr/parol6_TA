from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING

from PyQt6.QtCore import QThread, pyqtSignal

if TYPE_CHECKING:
    from backend.runtime_context import AppRuntime


class SerialWorkerThread(QThread):
    """Bridge the reference serial sender/receiver loop into a QThread lifecycle."""

    connection_changed = pyqtSignal(bool, str)
    snapshot_ready = pyqtSignal(dict)
    error_occurred = pyqtSignal(str)
    log_message = pyqtSignal(str, str)

    def __init__(self, runtime: "AppRuntime", port_name: str, baudrate: int) -> None:
        super().__init__()
        self._runtime = runtime
        self._port_name = port_name
        self._baudrate = baudrate
        self._stop_event = threading.Event()
        self._sender_thread: threading.Thread | None = None
        self._receiver_thread: threading.Thread | None = None
        self._is_mock = port_name.upper() == "MOCK"

    def stop(self) -> None:
        """Stop the serial worker and wait for the thread to exit."""
        self._stop_event.set()
        self.wait(2000)

    def send_disable_now(self) -> None:
        """Best-effort direct disable path used by the failsafe manager."""
        self._runtime.commander.clear_queue()
        self._runtime.commander.set_enable(False)
        self._runtime.command_data.command.value = 102
        self._runtime.buttons[2] = 0
        self.log_message.emit("Direct disable command issued", "estop")

    def run(self) -> None:
        """Run either the real serial bridge or the deterministic mock bridge."""
        if self._is_mock:
            self._run_mock_bridge()
            return

        try:
            self._run_real_bridge()
        except Exception as exc:
            message = f"Serial worker failed: {exc}"
            logging.exception(message)
            self.error_occurred.emit(message)
            self.connection_changed.emit(False, message)

    def _run_real_bridge(self) -> None:
        backend_dir = Path(__file__).resolve().parent
        project_dir = backend_dir.parent
        import sys

        if str(backend_dir) not in sys.path:
            sys.path.insert(0, str(backend_dir))
        if str(project_dir) not in sys.path:
            sys.path.insert(0, str(project_dir))

        import Serial_sender_latest
        from tools.init_tools import get_my_os, init_serial

        serial_handle, detected_port = init_serial(starting_port_win=self._runtime.general_data[0])
        Serial_sender_latest.ser = serial_handle
        Serial_sender_latest.my_os = get_my_os()
        Serial_sender_latest.LOGINTERVAL = 3

        try:
            desired_port = self._port_name.upper()
            serial_handle.port = desired_port
            serial_handle.baudrate = self._baudrate
            if not serial_handle.is_open:
                serial_handle.open()
        except Exception as exc:
            raise RuntimeError(f"Unable to open serial port {self._port_name}: {exc}") from exc

        self._runtime.general_data[0] = self._extract_port_index(self._port_name, detected_port)
        self._runtime.general_data[1] = self._baudrate
        self._runtime.commander.set_enable(True)

        self._sender_thread = threading.Thread(
            target=Serial_sender_latest.Send_data,
            args=(
                self._runtime.general_data,
                self._runtime.command_data,
                self._runtime.robot_mode,
                self._stop_event,
                self._runtime.sync_sema,
                self._runtime.ready_sema,
            ),
            daemon=True,
        )
        self._receiver_thread = threading.Thread(
            target=Serial_sender_latest.Receive_data,
            args=(self._runtime.general_data, self._runtime.robot_data, self._stop_event),
            daemon=True,
        )
        self._sender_thread.start()
        self._receiver_thread.start()
        self.connection_changed.emit(True, f"{self._port_name}@{self._baudrate}")
        self.log_message.emit(f"Serial connected on {self._port_name}@{self._baudrate}", "info")

        while not self._stop_event.is_set():
            self.snapshot_ready.emit(self._runtime.build_snapshot())
            time.sleep(0.1)

        try:
            if serial_handle.is_open:
                serial_handle.close()
        except Exception:
            logging.exception("Failed to close serial handle")
        self.connection_changed.emit(False, "Serial disconnected")

    def _run_mock_bridge(self) -> None:
        self._runtime.commander.set_enable(True)
        self.connection_changed.emit(True, "MOCK@virtual")
        self.log_message.emit("Serial worker running in MOCK mode", "warn")

        while not self._stop_event.is_set():
            self._runtime.sync_sema.release()
            if not self._runtime.ready_sema.acquire(timeout=0.2):
                continue
            self._apply_mock_feedback()
            self.snapshot_ready.emit(self._runtime.build_snapshot())
            time.sleep(0.01)

        self.connection_changed.emit(False, "MOCK disconnected")

    def _apply_mock_feedback(self) -> None:
        command_value = self._runtime.command_data.command.value
        if command_value == 100:
            for index in range(6):
                self._runtime.robot_data.homed[index] = 1
                self._runtime.robot_data.position[index] = 0
        elif command_value == 101:
            self._runtime.robot_data.inout[4] = 1
        elif command_value == 102:
            self._runtime.robot_data.inout[4] = 0
        elif command_value == 103:
            self._runtime.robot_data.timeout_error.value = 0
        elif command_value in (123, 156):
            for index in range(6):
                self._runtime.robot_data.position[index] = self._runtime.command_data.position[index]
                self._runtime.robot_data.speed[index] = self._runtime.command_data.speed[index]
        self._runtime.robot_data.timing_data.value = 10

    @staticmethod
    def _extract_port_index(port_name: str, fallback: int) -> int:
        digits = "".join(character for character in port_name if character.isdigit())
        return int(digits) if digits else int(fallback)
