from backend.program_control import (
    PROGRAM_SIGNAL_PAUSED,
    PROGRAM_SIGNAL_RUNNING,
    RUN_ACTION_QUEUE,
    RUN_ACTION_REJECT,
    RUN_ACTION_START,
    classify_run_request,
    has_realtime_program_signal,
    is_paused,
    manual_jog_allowed,
    motion_needs_replan,
    read_program_signal,
    write_program_signal,
)


def test_jog_is_blocked_during_running_program_and_allowed_during_pause():
    assert not manual_jog_allowed(True, {"state": "RUNNING", "paused": False})
    assert manual_jog_allowed(True, {"state": "PAUSED", "paused": True})
    assert manual_jog_allowed(False, {"state": "IDLE", "paused": False})
    # The shared-memory signal is authoritative if JSON status is stale.
    assert not manual_jog_allowed(
        True, {"state": "PAUSED", "paused": True}, PROGRAM_SIGNAL_RUNNING
    )
    assert manual_jog_allowed(
        True, {"state": "RUNNING", "paused": False}, PROGRAM_SIGNAL_PAUSED
    )


def test_paused_accepts_flag_or_explicit_state():
    assert is_paused({"state": "RUNNING", "paused": True})
    assert is_paused({"state": "PAUSED", "paused": False})
    assert not is_paused({"state": "RUNNING", "paused": False})


def test_only_an_in_progress_motion_is_replanned_after_pause():
    assert motion_needs_replan("MoveJoint()", 25)
    assert motion_needs_replan("MoveCart()", 1)
    assert not motion_needs_replan("MoveJoint()", 0)
    assert not motion_needs_replan("Delay()", 25)


def test_realtime_signal_uses_dedicated_slot_and_supports_legacy_arrays():
    buttons = [0] * 10
    assert has_realtime_program_signal(buttons)
    write_program_signal(buttons, PROGRAM_SIGNAL_PAUSED)
    assert read_program_signal(buttons) == PROGRAM_SIGNAL_PAUSED

    legacy_buttons = [0] * 9
    assert not has_realtime_program_signal(legacy_buttons)
    write_program_signal(legacy_buttons, PROGRAM_SIGNAL_PAUSED)
    assert read_program_signal(legacy_buttons) == PROGRAM_SIGNAL_RUNNING


def test_second_program_starts_only_after_active_program_is_stopping():
    assert classify_run_request(False, False) == RUN_ACTION_START
    assert classify_run_request(True, True) == RUN_ACTION_QUEUE
    assert classify_run_request(True, False) == RUN_ACTION_REJECT
