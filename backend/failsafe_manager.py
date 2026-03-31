from __future__ import annotations

import logging
from enum import Enum
from threading import RLock
from typing import Callable

from PyQt6.QtCore import QObject, pyqtSignal


class FailsafeManager(QObject):
    """Singleton that centralizes robot safety state and emergency actions."""

    estop_triggered = pyqtSignal(str)
    warning_raised = pyqtSignal(str)
    state_changed = pyqtSignal(str)

    class State(Enum):
        """Discrete safety states that can block or warn robot execution."""

        NORMAL = "normal"
        E_STOP = "estop"
        WORKSPACE_ERROR = "workspace_error"
        IK_ERROR = "ik_error"
        SERIAL_ERROR = "serial_error"
        MODBUS_WARNING = "modbus_warning"

    _instance: "FailsafeManager | None" = None
    _instance_lock = RLock()
    _state_priority = (
        State.E_STOP,
        State.WORKSPACE_ERROR,
        State.IK_ERROR,
        State.SERIAL_ERROR,
        State.MODBUS_WARNING,
    )

    def __init__(self) -> None:
        super().__init__()
        self._lock = RLock()
        self._active_states: set[FailsafeManager.State] = set()
        self._serial_disable_callback: Callable[[], None] | None = None
        self._metric_logger: Callable[[str, int | float], None] | None = None

    @classmethod
    def instance(cls) -> "FailsafeManager":
        """Return the shared FailsafeManager instance."""
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
        return cls._instance

    def register_serial_disable_callback(self, callback: Callable[[], None]) -> None:
        """Register the direct serial disable callback used by E-Stop."""
        self._serial_disable_callback = callback

    def register_metric_logger(self, callback: Callable[[str, int | float], None]) -> None:
        """Register a metric logger callback for failsafe events."""
        self._metric_logger = callback

    def trigger_estop(self, reason: str) -> None:
        """Immediately enter E-Stop state and invoke the serial disable bypass callback."""
        self._set_state(self.State.E_STOP)
        if self._serial_disable_callback is not None:
            try:
                self._serial_disable_callback()
            except Exception:
                logging.exception("Failed to execute serial disable callback during E-Stop")
        if self._metric_logger is not None:
            self._metric_logger("estop_event", 1)
        logging.error("E-STOP triggered: %s", reason)
        self.estop_triggered.emit(reason)

    def trigger_workspace_violation(self, x: float, y: float) -> None:
        """Enter workspace error state when a pick point is outside the safe boundary."""
        self._set_state(self.State.WORKSPACE_ERROR)
        if self._metric_logger is not None:
            self._metric_logger("workspace_violation", 1)
        message = f"Workspace violation at X={x:.1f} mm, Y={y:.1f} mm"
        logging.error(message)
        self.warning_raised.emit(message)

    def trigger_ik_error(self, reason: str) -> None:
        """Enter IK error state and raise a warning signal."""
        self._set_state(self.State.IK_ERROR)
        if self._metric_logger is not None:
            self._metric_logger("ik_failure", 1)
        logging.error("IK failure: %s", reason)
        self.warning_raised.emit(reason)

    def trigger_serial_error(self, reason: str) -> None:
        """Enter serial error state and raise a warning signal."""
        self._set_state(self.State.SERIAL_ERROR)
        logging.error("Serial error: %s", reason)
        self.warning_raised.emit(reason)

    def trigger_modbus_warning(self, reason: str) -> None:
        """Enter Modbus warning state and emit a non-critical warning signal."""
        self._set_state(self.State.MODBUS_WARNING)
        logging.warning("Modbus warning: %s", reason)
        self.warning_raised.emit(reason)

    def clear(self, state: "FailsafeManager.State") -> None:
        """Clear a specific safety state and emit the resulting aggregate state."""
        with self._lock:
            self._active_states.discard(state)
        self._emit_state_changed()

    def is_safe_to_move(self) -> bool:
        """Return True only when no safety state is active."""
        with self._lock:
            return not self._active_states

    def current_state(self) -> "FailsafeManager.State":
        """Return the highest-priority active safety state or NORMAL."""
        with self._lock:
            for state in self._state_priority:
                if state in self._active_states:
                    return state
        return self.State.NORMAL

    def _set_state(self, state: "FailsafeManager.State") -> None:
        with self._lock:
            self._active_states.add(state)
        self._emit_state_changed()

    def _emit_state_changed(self) -> None:
        self.state_changed.emit(self.current_state().value)
