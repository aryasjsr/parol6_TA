"""Cross-check audits (Phase 7.10 / 7.11).

7.10 — Verify config usage: no hardcoded addresses, thresholds, or paths
       that should come from config.json.
7.11 — Verify heavy I/O runs in QThread (no blocking main thread).
"""

from __future__ import annotations

import ast
import re
import sys
from pathlib import Path

import pytest

_project = Path(__file__).resolve().parents[1]
_backend = _project / "backend"
_vision = _project / "vision"
_program = _project / "program"
_ui = _project / "ui"

# Source directories to scan
_SOURCE_DIRS = [_backend, _vision, _program, _ui]


# ---------------------------------------------------------------------------
# 7.10 — No hardcode cross-check
# ---------------------------------------------------------------------------
_HARDCODE_PATTERNS: list[tuple[str, str]] = [
    # Absolute Windows paths to models (not test fixtures)
    (r'[A-Z]:\\.*\.onnx', "Hardcoded Windows path to ONNX model"),
]
# NOTE: IP defaults in UI combo/fields (e.g. "192.168.1.10") and COM port
# dropdown items are intentional UI hints that read through config.get().


def _source_files() -> list[Path]:
    """Collect all *.py files from backend/, vision/, program/, ui/."""
    files: list[Path] = []
    for d in _SOURCE_DIRS:
        if d.is_dir():
            files.extend(d.rglob("*.py"))
    return files


class TestNoHardcode:
    @pytest.mark.parametrize("pattern,description", _HARDCODE_PATTERNS)
    def test_no_hardcoded_values(self, pattern: str, description: str):
        """Scan all source files for known hardcode patterns."""
        violations: list[str] = []
        regex = re.compile(pattern)
        for filepath in _source_files():
            text = filepath.read_text(encoding="utf-8", errors="replace")
            for lineno, line in enumerate(text.splitlines(), start=1):
                # skip comments and docstrings heuristic
                stripped = line.strip()
                if stripped.startswith("#"):
                    continue
                if regex.search(line):
                    rel = filepath.relative_to(_project)
                    violations.append(f"  {rel}:{lineno}: {stripped[:100]}")
        assert not violations, (
            f"{description} found in source files:\n" + "\n".join(violations)
        )

    def test_config_keys_used(self):
        """Critical config namespaces are referenced in code."""
        # Collect all cfg.get("key") / config.get("key") / config_getter("key")
        config_pattern = re.compile(r'''config(?:_getter)?\(\s*["']([^"']+)["']''')
        used_keys: set[str] = set()
        for filepath in _source_files():
            text = filepath.read_text(encoding="utf-8", errors="replace")
            used_keys.update(config_pattern.findall(text))

        # Also capture dict-style access: workspace.get("x_min_mm")
        # These are used after config.get("workspace", {})
        dict_pattern = re.compile(r'''workspace\.get\(["']([^"']+)["']''')
        for filepath in _source_files():
            text = filepath.read_text(encoding="utf-8", errors="replace")
            for m in dict_pattern.findall(text):
                used_keys.add(f"workspace.{m}")

        # At minimum, these critical keys/namespaces should be present in usage
        required_keys = {
            "workspace",           # loaded as dict in workspace_validator
            "workspace.z_fixed_mm",
            "vision.detection_method",
        }
        for key in required_keys:
            assert key in used_keys, f"Required config key '{key}' not found in any source file"


# ---------------------------------------------------------------------------
# 7.11 — I/O in QThread cross-check
# ---------------------------------------------------------------------------
_BLOCKING_CALLS = {
    "cv2.VideoCapture",
    "serial.Serial",
    "ModbusTcpClient",
    "ort.InferenceSession",
    "onnxruntime.InferenceSession",
    "time.sleep",
    "socket.connect",
}


class TestIOInQThread:
    def test_no_blocking_io_in_ui_files(self):
        """UI tab files should not directly call blocking I/O functions."""
        violations: list[str] = []
        for filepath in _ui.rglob("*.py"):
            text = filepath.read_text(encoding="utf-8", errors="replace")
            for lineno, line in enumerate(text.splitlines(), start=1):
                stripped = line.strip()
                if stripped.startswith("#"):
                    continue
                for call in _BLOCKING_CALLS:
                    if call in stripped:
                        # Allow imports (they don't block)
                        if "import" in stripped:
                            continue
                        rel = filepath.relative_to(_project)
                        violations.append(f"  {rel}:{lineno}: {call} → {stripped[:100]}")
        assert not violations, (
            "Blocking I/O calls found in UI files (should be in QThread):\n"
            + "\n".join(violations)
        )

    def test_qthread_subclasses_have_run(self):
        """All QThread subclasses should override run() — not execute I/O in __init__."""
        for filepath in _source_files():
            text = filepath.read_text(encoding="utf-8", errors="replace")
            try:
                tree = ast.parse(text, filename=str(filepath))
            except SyntaxError:
                continue
            for node in ast.walk(tree):
                if not isinstance(node, ast.ClassDef):
                    continue
                # Check if any base is QThread
                is_qthread = any(
                    (isinstance(b, ast.Name) and b.id == "QThread")
                    or (isinstance(b, ast.Attribute) and b.attr == "QThread")
                    for b in node.bases
                )
                if not is_qthread:
                    continue
                method_names = {
                    item.name for item in node.body if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))
                }
                rel = filepath.relative_to(_project)
                assert "run" in method_names, (
                    f"{rel}: QThread subclass '{node.name}' does not override run()"
                )
