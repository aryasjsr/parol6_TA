"""Performance and timing benchmarks (Phase 7.8 / 7.9).

Tests:
  7.8  E-Stop response time (< 100 ms)
  7.9a Vision contour processing (< 50 ms)
  7.9b Vision ONNX mock processing (< 100 ms)
  7.9c Workspace validation (< 1 ms)
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import pytest

_project = Path(__file__).resolve().parents[1]
if str(_project) not in sys.path:
    sys.path.insert(0, str(_project))

from backend.config_manager import ConfigManager
from backend.failsafe_manager import FailsafeManager
from backend.research_logger import ResearchLogger
from vision.workspace_validator import WorkspaceValidator


@pytest.fixture(autouse=True)
def _reset_singletons():
    ConfigManager._instance = None
    FailsafeManager._instance = None
    ResearchLogger._instance = None
    cfg = ConfigManager.instance()
    cfg.set("workspace.x_min_mm", -200)
    cfg.set("workspace.x_max_mm", 200)
    cfg.set("workspace.y_min_mm", -200)
    cfg.set("workspace.y_max_mm", 200)
    cfg.set("workspace.margin_mm", 10)
    cfg.set("workspace.z_fixed_mm", 200)
    cfg.set("vision.model_path", "MOCK")
    cfg.set("vision.safe_pick_margin_pct", 0.15)
    yield


# ---------------------------------------------------------------------------
# 7.8 — E-Stop response time < 100 ms
# ---------------------------------------------------------------------------
class TestEStopTiming:
    def test_estop_under_100ms(self):
        """Triggering E-Stop via FailsafeManager should complete in < 100 ms."""
        failsafe = FailsafeManager.instance()
        # register a no-op serial disable (simulates fastest path)
        failsafe.register_serial_disable_callback(lambda: None)

        trials: list[float] = []
        for _ in range(50):
            # Reset to normal each iteration
            failsafe.clear(FailsafeManager.State.E_STOP)
            t0 = time.perf_counter()
            failsafe.trigger_estop("benchmark test")
            elapsed_ms = (time.perf_counter() - t0) * 1000
            trials.append(elapsed_ms)

        avg = sum(trials) / len(trials)
        p99 = sorted(trials)[int(len(trials) * 0.99)]
        assert avg < 100, f"E-Stop average latency {avg:.2f} ms exceeds 100 ms"
        assert p99 < 100, f"E-Stop p99 latency {p99:.2f} ms exceeds 100 ms"


# ---------------------------------------------------------------------------
# 7.9a — Workspace validation < 1 ms
# ---------------------------------------------------------------------------
class TestWorkspaceValidationTiming:
    def test_workspace_check_under_1ms(self):
        """Workspace.check() should complete in < 1 ms."""
        cfg = ConfigManager.instance()
        ws = WorkspaceValidator(cfg)
        trials: list[float] = []
        rng = np.random.default_rng(42)
        for _ in range(200):
            x = float(rng.uniform(-250, 250))
            y = float(rng.uniform(-250, 250))
            t0 = time.perf_counter()
            ws.check(x, y)
            elapsed_ms = (time.perf_counter() - t0) * 1000
            trials.append(elapsed_ms)

        avg = sum(trials) / len(trials)
        assert avg < 1.0, f"Workspace check avg {avg:.4f} ms exceeds 1 ms"


# ---------------------------------------------------------------------------
# 7.9b — Vision contour detection < 50 ms
# ---------------------------------------------------------------------------
class TestVisionContourTiming:
    def test_contour_detection_under_50ms(self):
        """ObjectDetector.detect() on a 640×480 frame should complete in < 50 ms."""
        from vision.object_detector import ObjectDetector

        cfg = ConfigManager.instance()
        ws = WorkspaceValidator(cfg)
        detector = ObjectDetector(cfg, ws)

        frame = np.full((480, 640, 3), (20, 28, 42), dtype=np.uint8)
        # Draw a bright rectangle to be detected
        frame[160:250, 220:380] = (210, 210, 220)

        K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]], dtype=np.float64)

        trials: list[float] = []
        for _ in range(30):
            t0 = time.perf_counter()
            detector.detect(frame, K, 200.0)
            elapsed_ms = (time.perf_counter() - t0) * 1000
            trials.append(elapsed_ms)

        avg = sum(trials) / len(trials)
        assert avg < 50, f"Contour detection avg {avg:.2f} ms exceeds 50 ms"


# ---------------------------------------------------------------------------
# 7.9c — Mock ONNX model detection < 100 ms
# ---------------------------------------------------------------------------
class TestVisionModelMockTiming:
    def test_mock_model_under_100ms(self):
        """ModelDetector in MOCK mode should complete in < 100 ms."""
        from vision.model_detector import ModelDetector

        cfg = ConfigManager.instance()
        ws = WorkspaceValidator(cfg)
        detector = ModelDetector(cfg, ws)
        detector.load_model("MOCK")
        assert detector.is_loaded

        frame = np.full((480, 640, 3), (20, 28, 42), dtype=np.uint8)
        K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]], dtype=np.float64)

        trials: list[float] = []
        for _ in range(30):
            t0 = time.perf_counter()
            bundles = detector.detect_bundles(frame, K, 200.0)
            elapsed_ms = (time.perf_counter() - t0) * 1000
            trials.append(elapsed_ms)

        avg = sum(trials) / len(trials)
        assert len(bundles) >= 1, "MOCK model should produce bundles"
        assert avg < 100, f"Mock model detection avg {avg:.2f} ms exceeds 100 ms"
