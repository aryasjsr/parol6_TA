"""End-to-end cycle tests using full MOCK mode stack.

These tests instantiate the ProgramExecutor with mock serial, mock modbus,
mock vision, and mock model to validate the three core safety flows:
  - SAFE:     trigger → model detect → SAFE → pick → cycle_done
  - MARGINAL: trigger → model detect → MARGINAL → UI confirm → pick
  - UNSAFE:   trigger → model detect → UNSAFE → block → error_flag
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

# Ensure project root is importable
_project = Path(__file__).resolve().parents[1]
if str(_project) not in sys.path:
    sys.path.insert(0, str(_project))

# Qt app needed for signal/slot mechanism
from PyQt6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from backend.config_manager import ConfigManager
from backend.exceptions import WorkspaceViolationError
from backend.failsafe_manager import FailsafeManager
from backend.research_logger import ResearchLogger
from program.program_executor import ProgramExecutor, ProgramExecutorDependencies
from program.program_model import ProgramCommand, ProgramModel
from tools.shared_struct import RobotInputData, RobotOutputData
from vision.object_detector import DetectionBundle, DetectionResult
from vision.workspace_validator import WorkspaceValidator


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------
@pytest.fixture(autouse=True)
def _reset_singletons(tmp_path):
    """Reset singletons so tests are isolated."""
    ConfigManager._instance = None
    FailsafeManager._instance = None
    ResearchLogger._instance = None
    cfg = ConfigManager.instance()
    cfg.set("workspace.x_min_mm", -200)
    cfg.set("workspace.x_max_mm", 200)
    cfg.set("workspace.y_min_mm", -200)
    cfg.set("workspace.y_max_mm", 200)
    cfg.set("workspace.z_fixed_mm", 200)
    cfg.set("workspace.margin_mm", 10)
    cfg.set("vision.detection_method", "model")
    cfg.set("vision.model_path", "MOCK")
    cfg.set("vision.safe_pick_margin_pct", 0.15)
    cfg.set("vision.marginal_confirm_timeout_s", 2)
    cfg.set("robot.default_speed_pct", 30)
    yield


def _build_deps(
    bundle: DetectionBundle | None = None,
    detection: DetectionResult | None = None,
) -> ProgramExecutorDependencies:
    """Build a ProgramExecutorDependencies with mock components."""
    cfg = ConfigManager.instance()
    failsafe = FailsafeManager.instance()
    logger = ResearchLogger.instance()
    ws = WorkspaceValidator(cfg)

    robot_data = RobotInputData()
    robot_data.initialize(
        position_init=[0, 0, 0, 0, 0, 0],
        speed_init=[0, 0, 0, 0, 0, 0],
        homed_init=[1, 1, 1, 1, 1, 1, 0, 0],
        inout_init=[0, 0, 0, 0, 1, 0, 0, 0],
        temperature_error_init=[0] * 8,
        position_error_init=[0] * 8,
        gripper_init=[0] * 6,
    )
    command_data = RobotOutputData()
    command_data.initialize(
        position_init=[0, 0, 0, 0, 0, 0],
        speed_init=[0, 0, 0, 0, 0, 0],
        affected_joint_init=[1] * 8,
        inout_init=[0, 0, 0, 0, 1, 0, 0, 0],
        gripper_init=[0] * 6,
        command_init=255,
        timeout_init=0,
    )

    commander = MagicMock()

    def _mock_inject(action):
        """Immediately mark the action as done so _inject_and_wait doesn't block."""
        if hasattr(action, "done"):
            action.done = True
        return True

    commander.inject_action = MagicMock(side_effect=_mock_inject)
    commander.giveCommand = MagicMock()
    commander.clear_queue = MagicMock()

    modbus_writer = MagicMock(return_value=True)

    return ProgramExecutorDependencies(
        commander=commander,
        robot_data=robot_data,
        command_data=command_data,
        shared_string=MagicMock(),
        jog_control=None,
        failsafe=failsafe,
        research_logger=logger,
        workspace_validator=ws,
        config_getter=cfg.get,
        vision_getter=lambda: detection,
        modbus_value_getter=lambda _: None,
        modbus_writer=modbus_writer,
        bundle_getter=lambda: bundle,
    )


def _make_bundle(safety: str, pick_world=(50.0, 50.0)) -> DetectionBundle:
    """Create a DetectionBundle with the specified safety status."""
    return DetectionBundle(
        selongsong_box=np.array([200, 150, 350, 230], dtype=np.float64),
        fixture_box=np.array([190, 136, 360, 150], dtype=np.float64),
        pick_point_px=(275, 190) if safety != "UNSAFE" else None,
        pick_point_world=pick_world if safety != "UNSAFE" else None,
        pick_safety=safety,
        conf_selongsong=0.92,
        conf_fixture=0.87,
    )


def _single_vision_program() -> ProgramModel:
    return ProgramModel(commands=[
        ProgramCommand(name="vision", args=[], kwargs={}, source="vision()"),
    ])


# ---------------------------------------------------------------------------
# 7.5 — SAFE flow: trigger → vision → SAFE → pick → done
# ---------------------------------------------------------------------------
class TestSafeFlow:
    def test_safe_flow_completes(self):
        """Program with SAFE bundle should complete successfully."""
        bundle = _make_bundle("SAFE")
        deps = _build_deps(bundle=bundle)
        model = _single_vision_program()
        executor = ProgramExecutor(model, deps)

        finished_events: list[tuple[bool, str]] = []
        executor.finished_execution.connect(lambda ok, msg: finished_events.append((ok, msg)))

        # Run synchronously (call run() directly to avoid needing event loop)
        executor.run()

        assert len(finished_events) == 1
        assert finished_events[0][0] is True, f"Expected success, got: {finished_events[0]}"


# ---------------------------------------------------------------------------
# 7.6 — MARGINAL flow: → MARGINAL → confirm → pick
# ---------------------------------------------------------------------------
class TestMarginalFlow:
    def test_marginal_confirm_proceeds(self):
        """MARGINAL + user confirm should complete the pick."""
        bundle = _make_bundle("MARGINAL")
        deps = _build_deps(bundle=bundle)
        model = _single_vision_program()
        executor = ProgramExecutor(model, deps)

        finished_events: list[tuple[bool, str]] = []
        executor.finished_execution.connect(lambda ok, msg: finished_events.append((ok, msg)))

        # Auto-confirm in background before run() blocks on the event
        def _auto_confirm():
            time.sleep(0.3)
            executor.confirm_pick()

        t = threading.Thread(target=_auto_confirm, daemon=True)
        t.start()

        executor.run()

        assert len(finished_events) == 1
        assert finished_events[0][0] is True

    def test_marginal_timeout_skips(self):
        """MARGINAL with no confirmation should auto-skip after timeout."""
        bundle = _make_bundle("MARGINAL")
        deps = _build_deps(bundle=bundle)
        ConfigManager.instance().set("vision.marginal_confirm_timeout_s", 1)
        model = _single_vision_program()
        executor = ProgramExecutor(model, deps)

        finished_events: list[tuple[bool, str]] = []
        executor.finished_execution.connect(lambda ok, msg: finished_events.append((ok, msg)))

        executor.run()

        # Should finish (skip) rather than crash
        assert len(finished_events) == 1


# ---------------------------------------------------------------------------
# 7.7 — UNSAFE flow: → UNSAFE → block → error
# ---------------------------------------------------------------------------
class TestUnsafeFlow:
    def test_unsafe_flow_blocks(self):
        """UNSAFE bundle should fail the program and flag error."""
        bundle = _make_bundle("UNSAFE")
        deps = _build_deps(bundle=bundle)
        model = _single_vision_program()
        executor = ProgramExecutor(model, deps)

        finished_events: list[tuple[bool, str]] = []
        executor.finished_execution.connect(lambda ok, msg: finished_events.append((ok, msg)))

        executor.run()

        assert len(finished_events) == 1
        assert finished_events[0][0] is False, "UNSAFE should fail"
        # Modbus error_flag should have been written
        deps.modbus_writer.assert_any_call("error_flag", True)
