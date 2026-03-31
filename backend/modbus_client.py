from __future__ import annotations

import logging
from dataclasses import dataclass
from threading import Event
from time import sleep
from typing import Any

from PyQt6.QtCore import QThread, pyqtSignal

from backend.config_manager import ConfigManager
from backend.failsafe_manager import FailsafeManager

try:
    from pymodbus.client import ModbusTcpClient
except Exception:
    ModbusTcpClient = None


@dataclass(slots=True)
class RegisterAddress:
    """Definition of one configurable Modbus address mapping entry."""

    name: str
    type: str
    address: int
    rw: str
    desc: str = ""


class ModbusPollerThread(QThread):
    """QThread worker that polls PLC registers and emits Qt signals with updates."""

    trigger_received = pyqtSignal()
    register_updated = pyqtSignal(dict)
    connection_state_changed = pyqtSignal(bool, str)
    error_occurred = pyqtSignal(str)
    modbus_jog_requested = pyqtSignal(int, int)  # (joint_id 0-5, direction +1/-1)

    def __init__(self, poll_interval_ms: int = 100) -> None:
        super().__init__()
        self._cfg = ConfigManager.instance()
        self._failsafe = FailsafeManager.instance()
        self._poll_interval_s = poll_interval_ms / 1000
        self._stop_event = Event()
        self._last_snapshot: dict[str, Any] = {}
        self._last_trigger_state = False
        self._last_jog_trigger_state = False
        self._client: ModbusTcpClient | None = None

    def stop(self) -> None:
        """Stop polling and close the Modbus client."""
        self._stop_event.set()
        self.wait(1000)
        self._disconnect_client()

    def write_coil_by_name(self, name: str, value: bool) -> bool:
        """Write a coil by mapped name using the active configuration."""
        register = self._get_address_by_name(name)
        if register is None:
            raise KeyError(f"Unknown Modbus address mapping: {name}")
        if register.type != "coil":
            raise ValueError(f"Address '{name}' is not configured as a coil")
        if self._cfg.get("modbus.ip") == "MOCK":
            self._last_snapshot[name] = value
            self.register_updated.emit(dict(self._last_snapshot))
            return True
        if self._client is None:
            raise ConnectionError("Modbus client is not connected")
        slave_id = int(self._cfg.get("modbus.slave_id", 1))
        result = self._client.write_coil(address=register.address, value=value, slave=slave_id)
        return not bool(getattr(result, "isError", lambda: False)())

    def run(self) -> None:
        """Start the polling loop until stop() is called."""
        if self._cfg.get("modbus.ip") == "MOCK":
            self._run_mock_mode()
            return
        if ModbusTcpClient is None:
            message = "pymodbus is not available in the active environment"
            self.error_occurred.emit(message)
            self._failsafe.trigger_modbus_warning(message)
            return

        reconnect_interval = float(self._cfg.get("failsafe.modbus_reconnect_interval_s", 5))
        while not self._stop_event.is_set():
            if not self._ensure_connected():
                self._stop_event.wait(reconnect_interval)
                continue

            try:
                snapshot = self._poll_snapshot()
                if snapshot != self._last_snapshot:
                    self._last_snapshot = snapshot
                    self.register_updated.emit(dict(snapshot))
                self._handle_trigger(snapshot.get("trigger_pick", False))
                self._handle_jog_trigger(snapshot)
            except Exception as exc:
                message = f"Modbus polling failed: {exc}"
                logging.exception(message)
                self.error_occurred.emit(message)
                self._failsafe.trigger_modbus_warning(message)
                self._disconnect_client()
                self._stop_event.wait(reconnect_interval)
                continue

            self._stop_event.wait(self._poll_interval_s)

        self._disconnect_client()

    def _run_mock_mode(self) -> None:
        self.connection_state_changed.emit(True, "mock")
        while not self._stop_event.is_set():
            self.register_updated.emit(dict(self._last_snapshot))
            self._handle_trigger(bool(self._last_snapshot.get("trigger_pick", False)))
            self._handle_jog_trigger(self._last_snapshot)
            self._stop_event.wait(self._poll_interval_s)

    def _ensure_connected(self) -> bool:
        if self._client is not None:
            return True

        host = self._cfg.get("modbus.ip", "127.0.0.1")
        port = int(self._cfg.get("modbus.port", 502))
        self._client = ModbusTcpClient(host=host, port=port)
        if self._client.connect():
            self.connection_state_changed.emit(True, f"{host}:{port}")
            return True

        message = f"Unable to connect to Modbus server at {host}:{port}"
        self.connection_state_changed.emit(False, message)
        self.error_occurred.emit(message)
        self._failsafe.trigger_modbus_warning(message)
        self._disconnect_client()
        return False

    def _disconnect_client(self) -> None:
        if self._client is not None:
            try:
                self._client.close()
            finally:
                self._client = None

    def _poll_snapshot(self) -> dict[str, Any]:
        if self._client is None:
            raise ConnectionError("Modbus client is not connected")

        slave_id = int(self._cfg.get("modbus.slave_id", 1))
        snapshot: dict[str, Any] = {}
        for register in self._load_addresses():
            if register.type == "coil":
                result = self._client.read_coils(address=register.address, count=1, slave=slave_id)
                if result.isError():
                    raise RuntimeError(f"Failed reading coil '{register.name}'")
                snapshot[register.name] = bool(result.bits[0])
            else:
                raise NotImplementedError(
                    f"Unsupported Modbus address type '{register.type}' for '{register.name}'"
                )
        return snapshot

    def _handle_trigger(self, current_trigger_state: bool) -> None:
        if current_trigger_state and not self._last_trigger_state:
            self.trigger_received.emit()
        self._last_trigger_state = current_trigger_state

    def _handle_jog_trigger(self, snapshot: dict[str, Any]) -> None:
        """Detect rising edge on jog_trigger and emit modbus_jog_requested."""
        jog_enable = snapshot.get("jog_enable", False)
        jog_trigger = snapshot.get("jog_trigger", False)
        rising = jog_trigger and not self._last_jog_trigger_state
        self._last_jog_trigger_state = jog_trigger
        if not (jog_enable and rising):
            return
        bit0 = int(snapshot.get("jog_joint_bit0", False))
        bit1 = int(snapshot.get("jog_joint_bit1", False))
        bit2 = int(snapshot.get("jog_joint_bit2", False))
        joint_id = bit0 | (bit1 << 1) | (bit2 << 2)
        if joint_id > 5:
            return
        direction = 1 if snapshot.get("jog_direction", False) else -1
        self.modbus_jog_requested.emit(joint_id, direction)

    def _load_addresses(self) -> list[RegisterAddress]:
        raw_addresses = self._cfg.get("modbus.addresses", [])
        return [RegisterAddress(**entry) for entry in raw_addresses]

    def _get_address_by_name(self, name: str) -> RegisterAddress | None:
        for register in self._load_addresses():
            if register.name == name:
                return register
        return None
