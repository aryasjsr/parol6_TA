"""Pure helpers for safe program pause, jog, and resume transitions."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any


MOTION_COMMANDS = frozenset(
    {
        "MoveJoint()",
        "MovePose()",
        "MoveCart()",
        "MoveCartRelTRF()",
    }
)

# Buttons[8] is the legacy Park flag. Keep real-time program control on its own
# shared-memory slot so it cannot be lost to cross-process runtime_state.json
# updates.
PROGRAM_CONTROL_INDEX = 9
PROGRAM_SIGNAL_RUNNING = 0
PROGRAM_SIGNAL_PAUSED = 1
PROGRAM_SIGNAL_STOP_REQUESTED = 2
PROGRAM_SIGNAL_STEP = 3
RUN_ACTION_START = "START"
RUN_ACTION_QUEUE = "QUEUE"
RUN_ACTION_REJECT = "REJECT"


def has_realtime_program_signal(buttons: Any) -> bool:
    """Return whether *buttons* includes the dedicated control slot."""
    try:
        return len(buttons) > PROGRAM_CONTROL_INDEX
    except TypeError:
        return False


def read_program_signal(buttons: Any) -> int:
    """Read the real-time signal, tolerating older 9-element button arrays."""
    try:
        return int(buttons[PROGRAM_CONTROL_INDEX])
    except (IndexError, TypeError):
        return PROGRAM_SIGNAL_RUNNING


def write_program_signal(buttons: Any, signal: int) -> None:
    """Write the real-time signal when the shared array supports it."""
    try:
        buttons[PROGRAM_CONTROL_INDEX] = int(signal)
    except (IndexError, TypeError):
        pass


def is_paused(control: Mapping[str, Any] | None) -> bool:
    """Return whether the shared program-control state requests a pause."""
    if not control:
        return False
    return bool(control.get("paused", False)) or str(control.get("state", "")).upper() == "PAUSED"


def manual_jog_allowed(
    program_running: bool,
    control: Mapping[str, Any] | None,
    realtime_signal: int | None = None,
) -> bool:
    """Manual jog is safe only while no program runs, or while it is paused."""
    if not program_running:
        return True
    if realtime_signal is not None:
        return realtime_signal == PROGRAM_SIGNAL_PAUSED
    return is_paused(control)


def motion_needs_replan(command: str | None, command_step: int) -> bool:
    """A partially executed motion must restart its plan from the live pose."""
    return bool(command in MOTION_COMMANDS and command_step > 0)


def classify_run_request(program_running: bool, stop_requested: bool) -> str:
    """Choose whether a second editor starts now, queues, or is rejected."""
    if not program_running:
        return RUN_ACTION_START
    if stop_requested:
        return RUN_ACTION_QUEUE
    return RUN_ACTION_REJECT
