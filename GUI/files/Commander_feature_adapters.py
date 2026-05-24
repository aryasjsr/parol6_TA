from __future__ import annotations

import ast
import csv
import json
import math
import shutil
import statistics
import sys
import threading
import time
from collections import defaultdict, deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

CONFIG_PATH = PROJECT_ROOT / "config.json"
STATE_PATH = PROJECT_ROOT / "runtime_state.json"
LOG_DIR = PROJECT_ROOT / "logs"
TIMESTAMP_TABLE_FIELDS = ["time", "event", "label", "elapsed_s", "duration_s"]


DEFAULT_CONFIG: dict[str, Any] = {
    "modbus": {
        "ip": "MOCK",
        "port": 502,
        "slave_id": 1,
        "poll_interval_ms": 100,
        "addresses": [
            {"name": "trigger_pick", "type": "coil", "address": 0, "rw": "read", "desc": "Pick trigger from PLC"},
            {"name": "cycle_done", "type": "coil", "address": 1, "rw": "write", "desc": "Done signal to PLC"},
            {"name": "error_flag", "type": "coil", "address": 2, "rw": "write", "desc": "Error signal to PLC"},
            {"name": "jog_enable", "type": "coil", "address": 10, "rw": "read", "desc": "Modbus jog master enable"},
            {"name": "jog_joint_bit0", "type": "coil", "address": 11, "rw": "read", "desc": "Joint ID bit 0"},
            {"name": "jog_joint_bit1", "type": "coil", "address": 12, "rw": "read", "desc": "Joint ID bit 1"},
            {"name": "jog_joint_bit2", "type": "coil", "address": 13, "rw": "read", "desc": "Joint ID bit 2"},
            {"name": "jog_direction", "type": "coil", "address": 14, "rw": "read", "desc": "0 negative, 1 positive"},
            {"name": "jog_trigger", "type": "coil", "address": 15, "rw": "read", "desc": "Rising edge jog trigger"},
        ],
        "jog_lock_timeout_s": 5.0,
        "jog_speed_pct": 20,
        "block_a": {
            "enabled": False,
            "ip": "MOCK",
            "port": 502,
            "slave_id": 1,
            "timeout_ms": 1000,
            "iterations": 1000,
            "function": "read_holding_register",
            "address": 100,
            "count": 1,
        },
        "block_b": {
            "enabled": False,
            "iterations": 100,
            "trigger_name": "trigger_pick",
            "done_name": "cycle_done",
            "error_name": "error_flag",
        },
    },
    "serial": {"port": "COM3", "baudrate": 115200},
    "vision": {
        "video_source": "MOCK",
        "camera_index": 0,
        "detection_method": "adaptive",
        "adaptive_block_size": 11,
        "adaptive_c": 2,
        "canny_threshold1": 50,
        "canny_threshold2": 150,
        "hsv_lower": [0, 0, 150],
        "hsv_upper": [180, 60, 255],
        "min_contour_area": 500,
        "max_contour_area": 50000,
        "morph_kernel_size": 3,
        "brightness": 0,
        "contrast": 1.0,
        "zoom": 1.0,
        "tune_live": False,
        "preview_pick_zone": True,
        "pick_inset_x_mm": 10.0,
        "pick_inset_y_mm": 8.0,
        "model_path": "vision/models/best.onnx",
        "model_conf_threshold": 0.5,
        "model_iou_threshold": 0.45,
        "safe_pick_margin_pct": 0.15,
        "marginal_confirm_timeout_s": 2.0,
        "offset_x_mm": 0.0,
        "offset_y_mm": 0.0,
        "calibration": {
            "snapshots_captured": 0,
            "chessboard_size": [9, 6],
            "square_size_mm": 25.0,
        },
        "auto_pick_enabled": True,
        "pick_pose_rpy_deg": [0.0, 0.0, 0.0],
        "pick_move_time_s": 4.0,
        "post_pick_descent_mm": 50.0,
        "post_pick_move_time_s": 2.0,
        "gripper_close": [255, 100, 120],
        "ibvs_enabled": False,
    },
    "workspace": {
        "x_min_mm": -200,
        "x_max_mm": 200,
        "y_min_mm": -200,
        "y_max_mm": 200,
        "z_fixed_mm": 200,
        "margin_mm": 10,
        "enabled": True,
    },
    "research": {
        "enabled": True,
        "max_plot_points": 300,
    },
    "failsafe": {
        "modbus_reconnect_interval_s": 5,
        "watchdog_timeout_s": 1,
    },
    "robot": {
        "default_speed_pct": 30,
        "gripper_output_pin": 1,
    },
}


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = json.loads(json.dumps(base))
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _normalize_config(raw: dict[str, Any]) -> dict[str, Any]:
    config = _deep_merge(DEFAULT_CONFIG, raw if isinstance(raw, dict) else {})
    legacy_workspace = raw.get("vision", {}).get("workspace") if isinstance(raw.get("vision"), dict) else None
    if legacy_workspace and "workspace" not in raw:
        config["workspace"].update(legacy_workspace)
    config.get("vision", {}).pop("workspace", None)
    model_path = str(config.get("vision", {}).get("model_path", "")).strip()
    if model_path:
        try:
            path = Path(model_path)
            if path.is_absolute():
                config["vision"]["model_path"] = str(path)
        except Exception:
            pass
    return config


def _reload_config_manager() -> None:
    try:
        from backend.config_manager import ConfigManager

        ConfigManager.instance().reload()
    except Exception:
        pass


def load_config() -> dict[str, Any]:
    if not CONFIG_PATH.exists():
        save_config(DEFAULT_CONFIG)
        return json.loads(json.dumps(DEFAULT_CONFIG))
    try:
        raw = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    except Exception:
        raw = {}
    return _normalize_config(raw if isinstance(raw, dict) else {})


def save_config(config: dict[str, Any]) -> None:
    normalized = _normalize_config(config)
    CONFIG_PATH.write_text(json.dumps(normalized, indent=2) + "\n", encoding="utf-8")
    _reload_config_manager()


def _read_state() -> dict[str, Any]:
    if not STATE_PATH.exists():
        return {}
    try:
        raw = json.loads(STATE_PATH.read_text(encoding="utf-8"))
        return raw if isinstance(raw, dict) else {}
    except Exception:
        return {}


def _write_state(state: dict[str, Any]) -> None:
    tmp_path = STATE_PATH.with_name(f"{STATE_PATH.stem}.{threading.get_ident()}.tmp")
    tmp_path.write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")
    for attempt in range(5):
        try:
            tmp_path.replace(STATE_PATH)
            return
        except PermissionError:
            if attempt == 4:
                raise
            time.sleep(0.02)


_state_lock = threading.Lock()


def update_state(section: str, payload: dict[str, Any]) -> None:
    with _state_lock:
        state = _read_state()
        state[section] = payload
        state["updated_at"] = time.time()
        _write_state(state)


def read_state(section: str, default: Any = None) -> Any:
    return _read_state().get(section, default)


def _set_shared(shared_string: Any, message: str) -> None:
    try:
        shared_string.value = message.encode("utf-8")[:99]
    except Exception:
        pass


def parse_command_values(command_text: str) -> list[Any]:
    parsed = ast.parse(command_text.strip(), mode="eval")
    if not isinstance(parsed.body, ast.Call):
        return []
    values: list[Any] = []
    for arg in parsed.body.args:
        values.append(_ast_value(arg))
    for keyword in parsed.body.keywords:
        values.append((keyword.arg, _ast_value(keyword.value)))
    return values


def _ast_value(node: ast.AST) -> Any:
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
        value = _ast_value(node.operand)
        if isinstance(value, (int, float)):
            return -value if isinstance(node.op, ast.USub) else value
    if isinstance(node, ast.Tuple):
        return tuple(_ast_value(item) for item in node.elts)
    if isinstance(node, ast.List):
        return [_ast_value(item) for item in node.elts]
    raise ValueError(f"Unsupported command parameter: {ast.dump(node)}")


def _coerce_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in {"1", "true", "high", "on", "yes", "close"}


def _resolve_project_path(value: str | Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else PROJECT_ROOT / path


class ResearchLogger:
    _fieldnames = ["timestamp", "metric", "value", "note"]

    def __init__(self) -> None:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        cfg = load_config().get("research", {})
        self._enabled = bool(cfg.get("enabled", True))
        self._path = LOG_DIR / f"research_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self._records: deque[dict[str, Any]] = deque(maxlen=1000)
        self._metric_buffers: dict[str, deque[float]] = defaultdict(
            lambda: deque(maxlen=int(load_config().get("research", {}).get("max_plot_points", 300)))
        )
        self._ensure_file()
        update_state("research", self.state_payload())

    @property
    def path(self) -> Path:
        return self._path

    def state_payload(self) -> dict[str, Any]:
        last = self._records[-1] if self._records else ""
        return {
            "enabled": self._enabled,
            "path": str(self._path),
            "last_event": last,
            "summary": self.summary(),
            "recent": list(self._records)[-80:],
        }

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = bool(enabled)
        cfg = load_config()
        cfg.setdefault("research", {})["enabled"] = self._enabled
        save_config(cfg)
        update_state("research", self.state_payload())

    def reset(self) -> None:
        with self._lock:
            self._records.clear()
            self._metric_buffers.clear()
        update_state("research", self.state_payload())

    def record(self, metric_name: str, value: int | float | str = "", note: str = "") -> None:
        if not self._enabled:
            return
        row = {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "metric": str(metric_name),
            "value": value,
            "note": str(note),
        }
        with self._lock:
            self._ensure_file()
            with self._path.open("a", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=self._fieldnames)
                writer.writerow(row)
            self._records.append(row)
            try:
                self._metric_buffers[str(metric_name)].append(float(value))
            except Exception:
                pass
        update_state("research", self.state_payload())

    def summary(self) -> dict[str, Any]:
        summary: dict[str, Any] = {}
        for metric, values in self._metric_buffers.items():
            if values:
                summary[metric] = round(float(values[-1]), 3)
        total = sum(1 for row in self._records if row.get("metric") == "vision_pick_result")
        success = sum(1 for row in self._records if row.get("metric") == "vision_pick_result" and str(row.get("value")) in {"1", "True", "true"})
        if total:
            summary["pick_success_rate"] = round(success / total * 100.0, 1)
        return summary

    def metric_series(self, names: list[str]) -> dict[str, list[float]]:
        with self._lock:
            return {name: list(self._metric_buffers.get(name, [])) for name in names}

    def export_csv(self, destination: str | Path | None = None) -> Path:
        destination_path = Path(destination) if destination else self._path
        if destination_path != self._path:
            destination_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(self._path, destination_path)
        return destination_path

    def export_excel(self, destination: str | Path | None = None) -> Path:
        destination_path = Path(destination) if destination else self._path.with_suffix(".xlsx")
        destination_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            from openpyxl import Workbook
        except Exception as exc:
            raise RuntimeError("openpyxl is required for Excel export") from exc

        workbook = Workbook()
        sheet = workbook.active
        sheet.title = "Research Log"
        sheet.append(self._fieldnames)
        if self._path.exists():
            with self._path.open("r", newline="", encoding="utf-8") as handle:
                reader = csv.DictReader(handle)
                for row in reader:
                    sheet.append([row.get(field, "") for field in self._fieldnames])
        workbook.save(destination_path)
        return destination_path

    def _ensure_file(self) -> None:
        if self._path.exists():
            return
        with self._path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=self._fieldnames)
            writer.writeheader()


research_logger = ResearchLogger()


def _timestamp_table_rows() -> list[dict[str, Any]]:
    state = read_state("research_timestamp_table", {})
    rows = state.get("rows", []) if isinstance(state, dict) else []
    if not isinstance(rows, list):
        return []
    return [row for row in rows if isinstance(row, dict)]


def _timestamp_table_payload(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "rows": rows[-1000:],
        "last_row": rows[-1] if rows else None,
        "updated_at": time.time(),
    }


def _append_timestamp_table_row(
    event: str,
    label: str = "",
    elapsed_s: int | float | str = "",
    duration_s: int | float | str = "",
    time_iso: str | None = None,
) -> dict[str, Any]:
    row = {
        "time": time_iso or datetime.now().isoformat(timespec="milliseconds"),
        "event": str(event),
        "label": str(label),
        "elapsed_s": elapsed_s,
        "duration_s": duration_s,
    }
    rows = _timestamp_table_rows()
    rows.append(row)
    update_state("research_timestamp_table", _timestamp_table_payload(rows))
    return row


def clear_timestamp_table() -> None:
    update_state("research_timestamp_table", _timestamp_table_payload([]))


def export_timestamp_table(destination: str | Path) -> Path:
    destination_path = Path(destination)
    destination_path.parent.mkdir(parents=True, exist_ok=True)
    rows = _timestamp_table_rows()
    if destination_path.suffix.lower() == ".xlsx":
        try:
            from openpyxl import Workbook
        except Exception as exc:
            raise RuntimeError("openpyxl is required for Excel export") from exc

        workbook = Workbook()
        sheet = workbook.active
        sheet.title = "Timestamp Table"
        sheet.append(TIMESTAMP_TABLE_FIELDS)
        for row in rows:
            sheet.append([row.get(field, "") for field in TIMESTAMP_TABLE_FIELDS])
        workbook.save(destination_path)
        return destination_path

    with destination_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=TIMESTAMP_TABLE_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in TIMESTAMP_TABLE_FIELDS})
    return destination_path


def _split_command_args(command_text: str) -> list[str]:
    start = command_text.find("(")
    end = command_text.rfind(")")
    if start < 0 or end <= start:
        return []
    inner = command_text[start + 1:end].strip()
    if not inner:
        return []
    try:
        return [item.strip() for item in next(csv.reader([inner], skipinitialspace=True))]
    except Exception:
        return [part.strip() for part in inner.split(",") if part.strip()]


def _strip_string_literal(value: str) -> str:
    text = str(value).strip()
    try:
        parsed = ast.literal_eval(text)
        if isinstance(parsed, str):
            return parsed
    except Exception:
        pass
    return text.strip("\"'")


def _parse_timestamp_command(command_text: str) -> tuple[str, str]:
    mode = "record"
    label = ""
    for raw_token in _split_command_args(command_text):
        token = raw_token.strip()
        if not token:
            continue
        if "=" in token:
            key, value = token.split("=", 1)
            if key.strip().lower() == "label":
                label = _strip_string_literal(value)
            continue
        lowered = token.strip().lower()
        if lowered in {"start", "stop", "record"}:
            mode = lowered
        elif not label:
            label = _strip_string_literal(token)
    return mode, label


def execute_timestamp_command(command_text: str, shared_string: Any) -> bool:
    mode, label = _parse_timestamp_command(command_text)
    now = time.time()
    now_iso = datetime.now().isoformat(timespec="milliseconds")
    state = read_state("research_timestamp", {})
    state = state if isinstance(state, dict) else {}
    note = label

    if mode == "start":
        payload = {
            "active": True,
            "start_epoch": now,
            "start_iso": now_iso,
            "last_label": label,
            "last_duration_s": None,
            "updated_at": now,
        }
        update_state("research_timestamp", payload)
        _append_timestamp_table_row("start", label, time_iso=now_iso)
        research_logger.record("timestamp_start", 1, note)
        _set_shared(shared_string, f"Log: timestamp start {label}".strip())
        return True

    if mode == "stop":
        start_epoch = state.get("start_epoch")
        duration = round(float(now - float(start_epoch)), 6) if start_epoch is not None else 0.0
        payload = {
            "active": False,
            "start_epoch": start_epoch,
            "start_iso": state.get("start_iso"),
            "last_label": label or state.get("last_label", ""),
            "last_duration_s": duration,
            "updated_at": now,
        }
        update_state("research_timestamp", payload)
        _append_timestamp_table_row("stop", label, duration_s=duration, time_iso=now_iso)
        research_logger.record("timestamp_stop", 1, note)
        research_logger.record("timestamp_duration_s", duration, note)
        _set_shared(shared_string, f"Log: timestamp stop {duration:.3f}s")
        return True

    elapsed = 0.0
    if state.get("active") and state.get("start_epoch") is not None:
        elapsed = round(float(now - float(state.get("start_epoch"))), 6)
    update_state(
        "research_timestamp",
        {
            **state,
            "active": bool(state.get("active", False)),
            "last_label": label,
            "last_record_elapsed_s": elapsed,
            "updated_at": now,
        },
    )
    _append_timestamp_table_row("record", label, elapsed_s=elapsed, time_iso=now_iso)
    research_logger.record("timestamp_record", elapsed if elapsed else 1, note)
    _set_shared(shared_string, f"Log: timestamp record {label}".strip())
    return True


@dataclass(slots=True)
class ModbusAddress:
    name: str
    type: str
    address: int
    rw: str
    desc: str = ""


class ModbusManager:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._snapshot: dict[str, Any] = {}
        self._connected = False
        self._description = "Disconnected"
        self._client: Any = None
        self._last_cycle_ms = 0.0
        self._block_a_stop = threading.Event()
        self._block_a_thread: threading.Thread | None = None
        self._block_b_lock = threading.Lock()
        self._block_b_state = self._new_block_b_state(load_config()["modbus"].get("block_b", {}))
        update_state("modbus_block_a", self._new_block_a_state(load_config()["modbus"].get("block_a", {})))
        update_state("modbus_block_b", dict(self._block_b_state))

    def config(self) -> dict[str, Any]:
        return load_config()["modbus"]

    def save_config(self, ip: str, port: int, slave_id: int, addresses: list[dict[str, Any]], jog_speed_pct: float | None = None, jog_timeout_s: float | None = None) -> None:
        cfg = load_config()
        cfg["modbus"]["ip"] = str(ip).strip() or "MOCK"
        cfg["modbus"]["port"] = int(port)
        cfg["modbus"]["slave_id"] = int(slave_id)
        cfg["modbus"]["addresses"] = addresses
        if jog_speed_pct is not None:
            cfg["modbus"]["jog_speed_pct"] = float(jog_speed_pct)
        if jog_timeout_s is not None:
            cfg["modbus"]["jog_lock_timeout_s"] = float(jog_timeout_s)
        save_config(cfg)
        self._publish()

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None
        self._disconnect()
        self._connected = False
        self._description = "Disconnected"
        self._publish()

    def save_block_test_config(self, block_a: dict[str, Any] | None = None, block_b: dict[str, Any] | None = None) -> None:
        cfg = load_config()
        if block_a is not None:
            cfg.setdefault("modbus", {}).setdefault("block_a", {}).update(block_a)
        if block_b is not None:
            cfg.setdefault("modbus", {}).setdefault("block_b", {}).update(block_b)
        save_config(cfg)

    def start_block_a(self, settings: dict[str, Any]) -> None:
        self.stop_block_a(join_timeout=1.0)
        self.save_block_test_config(block_a=settings)
        self._block_a_stop = threading.Event()
        self._block_a_thread = threading.Thread(target=self._run_block_a_test, args=(dict(settings), self._block_a_stop), daemon=True)
        self._block_a_thread.start()

    def stop_block_a(self, join_timeout: float = 1.0) -> None:
        self._block_a_stop.set()
        if self._block_a_thread is not None and self._block_a_thread.is_alive():
            self._block_a_thread.join(timeout=join_timeout)
        self._block_a_thread = None

    def start_block_b(self, settings: dict[str, Any]) -> None:
        self.save_block_test_config(block_b=settings)
        with self._block_b_lock:
            self._block_b_state = self._new_block_b_state(settings)
            self._block_b_state["running"] = True
            self._block_b_state["started_at"] = time.time()
            update_state("modbus_block_b", dict(self._block_b_state))
        research_logger.record("modbus_block_b_test_start", int(self._block_b_state["target"]), "cycle time test")

    def stop_block_b(self) -> None:
        with self._block_b_lock:
            self._block_b_state["running"] = False
            self._block_b_state["active_cycle"] = False
            self._block_b_state["updated_at"] = time.time()
            update_state("modbus_block_b", dict(self._block_b_state))
        research_logger.record("modbus_block_b_test_stop", int(self._block_b_state.get("completed", 0)))

    def block_b_cycle_started(self) -> bool:
        with self._block_b_lock:
            if not self._block_b_state.get("running") or self._block_b_state.get("active_cycle"):
                return False
            if int(self._block_b_state.get("completed", 0)) >= int(self._block_b_state.get("target", 100)):
                self._block_b_state["running"] = False
                update_state("modbus_block_b", dict(self._block_b_state))
                return False
            next_index = int(self._block_b_state.get("completed", 0)) + 1
            now = time.time()
            self._block_b_state["active_cycle"] = True
            self._block_b_state["active_index"] = next_index
            self._block_b_state["cycle_start_epoch"] = now
            self._block_b_state["latest"] = {"index": next_index, "status": "started", "timestamp": now}
            self._block_b_state["updated_at"] = now
            update_state("modbus_block_b", dict(self._block_b_state))
        research_logger.record("modbus_block_b_cycle_start", next_index)
        return True

    def block_b_cycle_finished(self, success: bool, note: str = "") -> bool:
        with self._block_b_lock:
            if not self._block_b_state.get("active_cycle"):
                return False
            done_name = str(self._block_b_state.get("done_name", "cycle_done"))
            error_name = str(self._block_b_state.get("error_name", "error_flag"))
            now = time.time()
            start_epoch = float(self._block_b_state.get("cycle_start_epoch") or now)
            duration = max(0.0, now - start_epoch)
            samples = list(self._block_b_state.get("samples", []))
            if success:
                samples.append(round(duration, 6))
                self._block_b_state["success_count"] = int(self._block_b_state.get("success_count", 0)) + 1
            else:
                self._block_b_state["failed_count"] = int(self._block_b_state.get("failed_count", 0)) + 1
            self._block_b_state["completed"] = int(self._block_b_state.get("completed", 0)) + 1
            self._block_b_state["samples"] = samples[-300:]
            stats = _cycle_stats(samples)
            self._block_b_state["stats"] = stats
            self._block_b_state["latest"] = {
                "index": self._block_b_state.get("completed", 0),
                "status": "success" if success else "failed",
                "duration_s": round(duration, 6),
                "note": note,
            }
            self._block_b_state["active_cycle"] = False
            self._block_b_state["cycle_start_epoch"] = None
            if int(self._block_b_state.get("completed", 0)) >= int(self._block_b_state.get("target", 100)):
                self._block_b_state["running"] = False
            self._block_b_state["updated_at"] = now
            payload = dict(self._block_b_state)
            update_state("modbus_block_b", payload)

        try:
            self.write_value(done_name, bool(success))
            self.write_value(error_name, not bool(success))
        except Exception as exc:
            research_logger.record("modbus_block_b_flag_error", 1, str(exc))

        if success:
            research_logger.record("modbus_block_b_cycle_time_s", round(duration, 6), note)
            research_logger.record("cycle_time", round(duration, 6), "modbus block b")
            research_logger.record("modbus_block_b_cycle_success", 1, note)
        else:
            research_logger.record("modbus_block_b_cycle_failed", 1, note)
        stats = payload.get("stats", {})
        research_logger.record("modbus_block_b_avg_s", stats.get("avg_s", 0.0))
        research_logger.record("modbus_block_b_min_s", stats.get("min_s", 0.0))
        research_logger.record("modbus_block_b_max_s", stats.get("max_s", 0.0))
        research_logger.record("modbus_block_b_std_s", stats.get("std_s", 0.0))
        return True

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._snapshot)

    def read_value(self, selector: Any) -> Any:
        snapshot = self.snapshot()
        state_snapshot = read_state("modbus", {})
        if isinstance(state_snapshot, dict) and isinstance(state_snapshot.get("snapshot"), dict):
            snapshot.update(state_snapshot["snapshot"])
        if selector in snapshot:
            return snapshot[selector]
        selector_text = str(selector)
        if selector_text in snapshot:
            return snapshot[selector_text]
        try:
            address = int(selector)
        except Exception:
            return None
        for entry in self._addresses():
            if entry.address == address:
                return snapshot.get(entry.name)
        return None

    def write_value(self, selector: Any, value: Any) -> bool:
        entry = self._address_for(selector)
        if entry is None:
            raise KeyError(f"Unknown Modbus selector: {selector}")
        cfg = self.config()
        if str(cfg.get("ip", "MOCK")).upper() == "MOCK":
            with self._lock:
                self._snapshot[entry.name] = _mock_value_for_type(entry.type, value)
            self._publish()
            return True
        if self._client is None:
            self._ensure_connected(cfg)
        slave_id = int(cfg.get("slave_id", 1))
        register_type = entry.type.lower()
        if register_type == "coil":
            result = self._client.write_coil(address=entry.address, value=_coerce_bool(value), slave=slave_id)
        elif register_type in {"holding_register", "register"}:
            result = self._client.write_register(address=entry.address, value=int(value), slave=slave_id)
        else:
            raise ValueError(f"Address '{entry.name}' is not writable as type {entry.type}")
        ok = not bool(getattr(result, "isError", lambda: False)())
        if ok:
            with self._lock:
                self._snapshot[entry.name] = _mock_value_for_type(entry.type, value)
            self._publish()
        return ok

    def _poll_loop(self) -> None:
        while not self._stop.is_set():
            started = time.perf_counter()
            cfg = self.config()
            if str(cfg.get("ip", "MOCK")).upper() == "MOCK":
                self._connected = True
                self._description = "MOCK"
                self._publish()
                self._stop.wait(max(0.05, float(cfg.get("poll_interval_ms", 100)) / 1000.0))
                continue
            try:
                self._ensure_connected(cfg)
                snapshot = self._poll_snapshot(cfg)
                with self._lock:
                    self._snapshot.update(snapshot)
                self._last_cycle_ms = (time.perf_counter() - started) * 1000.0
                research_logger.record("modbus_cycle", round(self._last_cycle_ms, 3))
                self._publish()
            except Exception as exc:
                self._connected = False
                self._description = str(exc)
                self._disconnect()
                self._publish()
                research_logger.record("modbus_error", 1, str(exc))
            self._stop.wait(max(0.05, float(cfg.get("poll_interval_ms", 100)) / 1000.0))

    def _run_block_a_test(self, settings: dict[str, Any], stop_event: threading.Event) -> None:
        target = max(1, int(float(settings.get("iterations", 1000))))
        settings = {
            **load_config()["modbus"].get("block_a", {}),
            **settings,
            "iterations": target,
        }
        state = self._new_block_a_state(settings)
        state["running"] = True
        state["started_at"] = time.time()
        update_state("modbus_block_a", state)
        research_logger.record("modbus_block_a_test_start", target, str(settings.get("function", "")))

        response_times: list[float] = []
        success_count = 0
        failed_count = 0
        timeout_count = 0
        samples: list[dict[str, Any]] = []
        start_time = time.perf_counter()
        client = None
        client_error = ""
        if str(settings.get("ip", "MOCK")).strip().upper() != "MOCK":
            try:
                client = self._open_block_a_client(settings)
            except Exception as exc:
                client_error = str(exc)

        for index in range(1, target + 1):
            if stop_event.is_set():
                break
            t0 = time.perf_counter()
            status = "success"
            ok = True
            detail = ""
            try:
                if client_error:
                    raise ConnectionError(client_error)
                ok, detail = self._perform_block_a_request(client, settings)
                status = "success" if ok else "failed"
            except Exception as exc:
                ok = False
                detail = str(exc)
                status = "timeout" if "timeout" in detail.lower() else "failed"
            response_ms = round((time.perf_counter() - t0) * 1000.0, 3)
            timeout_ms = float(settings.get("timeout_ms", 1000))
            if not ok and response_ms >= timeout_ms:
                status = "timeout"

            if status == "success":
                success_count += 1
                response_times.append(response_ms)
                research_logger.record("modbus_block_a_response_ms", response_ms, f"n={index}")
                research_logger.record("modbus_block_a_success", 1, f"n={index}")
            elif status == "timeout":
                timeout_count += 1
                research_logger.record("modbus_block_a_timeout", 1, f"n={index}; {detail}")
            else:
                failed_count += 1
                research_logger.record("modbus_block_a_failed", 1, f"n={index}; {detail}")

            samples.append({"n": index, "response_ms": response_ms, "status": status, "detail": detail})
            stats = _protocol_stats(
                completed=index,
                success_count=success_count,
                failed_count=failed_count,
                timeout_count=timeout_count,
                response_times=response_times,
                elapsed_s=max(time.perf_counter() - start_time, 1e-9),
            )
            update_state(
                "modbus_block_a",
                {
                    "running": True,
                    "target": target,
                    "completed": index,
                    "settings": settings,
                    "stats": stats,
                    "latest": samples[-1],
                    "samples": samples[-300:],
                    "updated_at": time.time(),
                },
            )

        elapsed_s = max(time.perf_counter() - start_time, 1e-9)
        completed = success_count + failed_count + timeout_count
        stats = _protocol_stats(completed, success_count, failed_count, timeout_count, response_times, elapsed_s)
        final_state = {
            "running": False,
            "target": target,
            "completed": completed,
            "settings": settings,
            "stats": stats,
            "latest": samples[-1] if samples else {},
            "samples": samples[-300:],
            "stopped": stop_event.is_set() and completed < target,
            "updated_at": time.time(),
        }
        update_state("modbus_block_a", final_state)
        research_logger.record("modbus_block_a_packet_loss_pct", stats.get("packet_loss_pct", 0.0))
        research_logger.record("modbus_block_a_throughput_rps", stats.get("throughput_rps", 0.0))
        research_logger.record("modbus_block_a_test_finished", completed)
        if client is not None:
            try:
                client.close()
            except Exception:
                pass

    def _open_block_a_client(self, settings: dict[str, Any]) -> Any:
        try:
            from pymodbus.client import ModbusTcpClient
        except Exception as exc:
            raise RuntimeError(f"pymodbus unavailable: {exc}") from exc
        timeout_s = max(0.001, float(settings.get("timeout_ms", 1000)) / 1000.0)
        client = ModbusTcpClient(
            host=str(settings.get("ip", "127.0.0.1")),
            port=int(settings.get("port", 502)),
            timeout=timeout_s,
        )
        if not client.connect():
            raise ConnectionError(f"Unable to connect to {settings.get('ip')}:{settings.get('port')}")
        return client

    def _perform_block_a_request(self, client: Any, settings: dict[str, Any]) -> tuple[bool, str]:
        function = str(settings.get("function", "read_holding_register")).strip().lower()
        address = int(float(settings.get("address", 100)))
        count = max(1, int(float(settings.get("count", 1))))
        slave_id = int(float(settings.get("slave_id", 1)))
        if client is None:
            time.sleep(0.001)
            with self._lock:
                self._snapshot["block_a_mock_last_address"] = address
                self._snapshot["block_a_mock_count"] = count
            self._publish()
            return True, "MOCK"

        if function == "read_holding_register":
            result = _call_modbus(client.read_holding_registers, {"address": address, "count": count}, slave_id)
        elif function == "read_input_register":
            result = _call_modbus(client.read_input_registers, {"address": address, "count": count}, slave_id)
        elif function == "read_coil":
            result = _call_modbus(client.read_coils, {"address": address, "count": count}, slave_id)
        elif function == "read_discrete_input":
            result = _call_modbus(client.read_discrete_inputs, {"address": address, "count": count}, slave_id)
        elif function == "write_register":
            result = _call_modbus(client.write_register, {"address": address, "value": 1}, slave_id)
        elif function == "write_coil":
            result = _call_modbus(client.write_coil, {"address": address, "value": True}, slave_id)
        else:
            raise ValueError(f"Unsupported Block A function: {function}")
        is_error = bool(getattr(result, "isError", lambda: False)())
        return not is_error, "modbus_error" if is_error else "ok"

    def _ensure_connected(self, cfg: dict[str, Any]) -> None:
        if self._client is not None:
            self._connected = True
            return
        try:
            from pymodbus.client import ModbusTcpClient
        except Exception as exc:
            raise RuntimeError(f"pymodbus unavailable: {exc}") from exc
        host = str(cfg.get("ip", "127.0.0.1"))
        port = int(cfg.get("port", 502))
        self._client = ModbusTcpClient(host=host, port=port)
        if not self._client.connect():
            raise ConnectionError(f"Unable to connect to {host}:{port}")
        self._connected = True
        self._description = f"{host}:{port}"

    def _disconnect(self) -> None:
        if self._client is not None:
            try:
                self._client.close()
            finally:
                self._client = None

    def _poll_snapshot(self, cfg: dict[str, Any]) -> dict[str, Any]:
        if self._client is None:
            return {}
        slave_id = int(cfg.get("slave_id", 1))
        snapshot: dict[str, Any] = {}
        for entry in self._addresses():
            register_type = entry.type.lower()
            if register_type == "coil":
                result = self._client.read_coils(address=entry.address, count=1, slave=slave_id)
                if result.isError():
                    raise RuntimeError(f"Read failed for {entry.name}")
                snapshot[entry.name] = bool(result.bits[0])
            elif register_type == "discrete_input":
                result = self._client.read_discrete_inputs(address=entry.address, count=1, slave=slave_id)
                if result.isError():
                    raise RuntimeError(f"Read failed for {entry.name}")
                snapshot[entry.name] = bool(result.bits[0])
            elif register_type in {"holding_register", "register"}:
                result = self._client.read_holding_registers(address=entry.address, count=1, slave=slave_id)
                if result.isError():
                    raise RuntimeError(f"Read failed for {entry.name}")
                snapshot[entry.name] = int(result.registers[0])
            elif register_type == "input_register":
                result = self._client.read_input_registers(address=entry.address, count=1, slave=slave_id)
                if result.isError():
                    raise RuntimeError(f"Read failed for {entry.name}")
                snapshot[entry.name] = int(result.registers[0])
        return snapshot

    def _addresses(self) -> list[ModbusAddress]:
        parsed: list[ModbusAddress] = []
        for entry in self.config().get("addresses", []):
            try:
                parsed.append(ModbusAddress(**entry))
            except Exception:
                continue
        return parsed

    def _address_for(self, selector: Any) -> ModbusAddress | None:
        selector_text = str(selector).strip()
        for entry in self._addresses():
            if entry.name == selector_text:
                return entry
        try:
            selector_int = int(selector)
        except Exception:
            return None
        for entry in self._addresses():
            if entry.address == selector_int:
                return entry
        return None

    def _new_block_a_state(self, settings: dict[str, Any]) -> dict[str, Any]:
        target = max(1, int(float(settings.get("iterations", 1000))))
        return {
            "running": False,
            "target": target,
            "completed": 0,
            "settings": dict(settings),
            "stats": _protocol_stats(0, 0, 0, 0, [], 0.0),
            "latest": {},
            "samples": [],
            "updated_at": time.time(),
        }

    def _new_block_b_state(self, settings: dict[str, Any]) -> dict[str, Any]:
        target = max(1, int(float(settings.get("iterations", 100))))
        return {
            "running": False,
            "target": target,
            "completed": 0,
            "active_cycle": False,
            "active_index": 0,
            "cycle_start_epoch": None,
            "trigger_name": str(settings.get("trigger_name", "trigger_pick")),
            "done_name": str(settings.get("done_name", "cycle_done")),
            "error_name": str(settings.get("error_name", "error_flag")),
            "success_count": 0,
            "failed_count": 0,
            "samples": [],
            "stats": _cycle_stats([]),
            "latest": {},
            "updated_at": time.time(),
        }

    def _publish(self) -> None:
        update_state(
            "modbus",
            {
                "connected": self._connected,
                "description": self._description,
                "snapshot": self.snapshot(),
                "cycle_ms": round(float(self._last_cycle_ms), 3),
                "updated_at": time.time(),
            },
        )


def _protocol_stats(
    completed: int,
    success_count: int,
    failed_count: int,
    timeout_count: int,
    response_times: list[float],
    elapsed_s: float,
) -> dict[str, Any]:
    loss_count = int(failed_count) + int(timeout_count)
    avg_rt = sum(response_times) / len(response_times) if response_times else 0.0
    return {
        "avg_rt_ms": round(avg_rt, 3),
        "throughput_rps": round(float(completed) / max(float(elapsed_s), 1e-9), 3) if completed else 0.0,
        "packet_loss_pct": round(loss_count / max(int(completed), 1) * 100.0, 3) if completed else 0.0,
        "success_count": int(success_count),
        "failed_count": int(failed_count),
        "timeout_count": int(timeout_count),
    }


def _cycle_stats(samples: list[float]) -> dict[str, Any]:
    if not samples:
        return {"avg_s": 0.0, "min_s": 0.0, "max_s": 0.0, "std_s": 0.0}
    return {
        "avg_s": round(sum(samples) / len(samples), 6),
        "min_s": round(min(samples), 6),
        "max_s": round(max(samples), 6),
        "std_s": round(statistics.pstdev(samples), 6) if len(samples) > 1 else 0.0,
    }


def _call_modbus(method: Any, kwargs: dict[str, Any], slave_id: int) -> Any:
    try:
        return method(**kwargs, slave=slave_id)
    except TypeError:
        try:
            return method(**kwargs, unit=slave_id)
        except TypeError:
            return method(**kwargs)


def _mock_value_for_type(register_type: str, value: Any) -> Any:
    if register_type.lower() in {"coil", "discrete_input"}:
        return _coerce_bool(value)
    return int(value)


class _SyntheticCapture:
    def __init__(self, size: tuple[int, int] = (640, 480)) -> None:
        self.width, self.height = size
        self.frame_index = 0

    def isOpened(self) -> bool:
        return True

    def read(self) -> tuple[bool, Any]:
        import cv2
        import numpy as np

        frame = np.full((self.height, self.width, 3), (20, 28, 42), dtype=np.uint8)
        offset = int(80 * math.sin(self.frame_index / 18.0))
        object_x = self.width // 2 - 80 + offset
        object_y = self.height // 2 - 38
        cv2.rectangle(frame, (object_x, object_y), (object_x + 160, object_y + 76), (210, 210, 220), -1)
        cv2.rectangle(frame, (object_x + 115, object_y + 8), (object_x + 150, object_y + 68), (55, 55, 60), -1)
        cv2.putText(frame, "MOCK CAMERA", (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 168, 75), 2, cv2.LINE_AA)
        self.frame_index += 1
        return True, frame

    def release(self) -> None:
        pass


class VisionManager:
    def __init__(self) -> None:
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._latest: dict[str, Any] | None = None
        self._latest_bundle: dict[str, Any] | None = None
        self._display_frame: Any = None
        self._raw_frame: Any = None
        self._status = "Stopped"
        self._model_status = "NOT LOADED"
        self._model_detector: Any = None
        self._calibration_manager: Any = None
        self._model_warned = False
        self._ibvs_controller: Any = None
        self._ibvs_active: bool = False
        self._ibvs_ipc: dict[str, Any] | None = None
        self._ibvs_dispatch: bool = False

    def config(self) -> dict[str, Any]:
        return load_config()["vision"]

    def save_settings(self, settings: dict[str, Any], workspace: dict[str, Any] | None = None) -> None:
        cfg = load_config()
        cfg.setdefault("vision", {}).update(settings)
        if workspace:
            cfg.setdefault("workspace", {}).update(workspace)
        save_config(cfg)
        self._publish()

    def available_sources(self, max_index: int = 8) -> list[str]:
        values = ["MOCK"]
        try:
            import cv2

            for index in range(max_index):
                cap = cv2.VideoCapture(index, cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY)
                if cap.isOpened():
                    values.append(str(index))
                    cap.release()
                elif index == 0:
                    cap.release()
        except Exception:
            if "0" not in values:
                values.append("0")
        return values

    def start(self, source: str | None = None) -> None:
        if source not in (None, ""):
            cfg = load_config()
            cfg["vision"]["video_source"] = str(source)
            save_config(cfg)
        self.stop()
        self._stop.clear()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None
        self._status = "Stopped"
        self._publish()

    def latest(self) -> dict[str, Any] | None:
        state_latest = read_state("vision", {}).get("latest")
        return state_latest if isinstance(state_latest, dict) else self._latest

    def latest_bundle(self) -> dict[str, Any] | None:
        state_bundle = read_state("vision", {}).get("latest_bundle")
        return state_bundle if isinstance(state_bundle, dict) else self._latest_bundle

    def latest_frame_image(self, max_size: tuple[int, int] = (980, 720)) -> Any:
        with self._lock:
            frame = None if self._display_frame is None else self._display_frame.copy()
        if frame is None:
            return None
        try:
            from PIL import Image

            rgb = frame[:, :, ::-1]
            image = Image.fromarray(rgb)
            image.thumbnail(max_size)
            return image
        except Exception:
            return None

    def set_mock_detection(self, x_mm: float = 0.0, y_mm: float = 0.0, safety: str = "SAFE") -> None:
        payload = {
            "status": "mock",
            "pick_point_px": [320, 240],
            "pick_point_world": [float(x_mm), float(y_mm)],
            "centroid_px": [320, 240],
            "centroid_world": [float(x_mm), float(y_mm)],
            "bbox_px": [250, 210, 140, 70],
            "width_mm": 80.0,
            "height_mm": 40.0,
            "orientation_deg": 0.0,
            "workspace_status": "valid",
            "workspace_message": "READY TO PICK",
            "timestamp": time.time(),
        }
        bundle = {
            "selongsong_box": [250, 210, 390, 280],
            "fixture_box": None,
            "pick_point_px": [320, 240],
            "pick_point_world": [float(x_mm), float(y_mm)],
            "pick_safety": safety,
            "conf_selongsong": 1.0,
            "conf_fixture": 0.0,
        }
        with self._lock:
            self._latest = payload
            self._latest_bundle = bundle
        self._status = "MOCK detection ready"
        self._publish()

    def load_model(self, model_path: str | None = None) -> str:
        try:
            from backend.config_manager import ConfigManager
            from vision.model_detector import ModelDetector
            from vision.workspace_validator import WorkspaceValidator

            cfg = ConfigManager.instance()
            cfg.reload()
            validator = WorkspaceValidator(cfg)
            if self._model_detector is None:
                self._model_detector = ModelDetector(cfg, validator)
            self._model_detector.load_model(model_path or self.config().get("model_path", ""))
            self._model_status = "LOADED"
            self._model_warned = False
        except Exception as exc:
            self._model_status = f"ERROR: {exc}"
            research_logger.record("vision_model_error", 1, str(exc))
        self._publish()
        return self._model_status

    def capture_snapshot(self) -> int:
        with self._lock:
            frame = None if self._raw_frame is None else self._raw_frame.copy()
        if frame is None:
            raise RuntimeError("No active frame available")
        manager = self._get_calibration_manager()
        count = manager.add_snapshot(frame)
        self._publish(calibration=manager.current_summary())
        return count

    def run_calibration(self, chessboard_size: tuple[int, int], square_size_mm: float) -> dict[str, Any]:
        manager = self._get_calibration_manager()
        calibration = manager.run_calibration(chessboard_size, square_size_mm)
        summary = calibration.summary()
        try:
            csv_path = manager.save_intrinsics_csv(calibration)
            if csv_path is not None:
                research_logger.record("vision_intrinsics_saved", 1, str(csv_path))
        except Exception as exc:
            research_logger.record("vision_intrinsics_error", 1, str(exc))
        if self._ibvs_controller is not None:
            try:
                self._ibvs_controller.load_intrinsics()
            except Exception:
                pass
        self._publish(calibration=manager.current_summary())
        return summary

    def start_ibvs(
        self,
        ipc_arrays: dict[str, Any] | None = None,
        dispatch: bool = False,
        **controller_kwargs: Any,
    ) -> dict[str, Any]:
        from backend.Centroid_IBVS import CentroidIBVSController

        cfg = self.config()
        defaults: dict[str, Any] = {
            "depth_m": float(cfg.get("ibvs_depth_m", 0.5)),
            "lam": float(cfg.get("ibvs_lambda", 1.0)),
            "deadband_px": float(cfg.get("ibvs_deadband_px", 4.0)),
            "max_step_mm": float(cfg.get("ibvs_max_step_mm", 20.0)),
            "speed_pct": float(cfg.get("ibvs_speed_pct", 30.0)),
            "interval_s": float(cfg.get("ibvs_interval_s", 0.1)),
        }
        defaults.update({k: v for k, v in controller_kwargs.items() if v is not None})
        controller = CentroidIBVSController(**defaults)
        controller.load_intrinsics()
        with self._lock:
            self._ibvs_controller = controller
            self._ibvs_ipc = ipc_arrays
            self._ibvs_dispatch = bool(dispatch)
            self._ibvs_active = True
        update_state("vision_ibvs", controller.to_dict() | {"active": True})
        return controller.to_dict()

    def stop_ibvs(self) -> None:
        with self._lock:
            controller = self._ibvs_controller
            self._ibvs_active = False
            self._ibvs_dispatch = False
            self._ibvs_ipc = None
        if controller is not None:
            controller.reset()
        update_state("vision_ibvs", {"active": False, "status": "stopped"})

    def ibvs_status(self) -> dict[str, Any]:
        with self._lock:
            controller = self._ibvs_controller
            active = self._ibvs_active
        if controller is None:
            return {"active": False, "status": "not_started"}
        return controller.to_dict() | {"active": active}

    def calibration_summary(self) -> dict[str, Any]:
        try:
            return self._get_calibration_manager().current_summary()
        except Exception:
            return load_config().get("vision", {}).get("calibration", {})

    def _get_calibration_manager(self) -> Any:
        self._ensure_vision_modules()
        from backend.config_manager import ConfigManager
        from vision.camera_calibration import CameraCalibrationManager

        if self._calibration_manager is None:
            self._calibration_manager = CameraCalibrationManager(ConfigManager.instance())
        return self._calibration_manager

    def _capture_loop(self) -> None:
        try:
            import cv2
        except Exception as exc:
            self._status = f"OpenCV unavailable: {exc}"
            self._publish()
            return
        source = str(self.config().get("video_source", "MOCK")).strip() or "MOCK"
        cap = self._open_capture(source)
        if not cap or not cap.isOpened():
            self._status = f"Camera unavailable: {source}"
            self._publish()
            return
        self._status = f"Running: {source}"
        self._publish()
        if str(self.config().get("detection_method", "adaptive")).lower() == "model":
            self.load_model(self.config().get("model_path", ""))
        try:
            while not self._stop.is_set():
                started = time.perf_counter()
                ok, frame = cap.read()
                if not ok or frame is None:
                    self._status = "Camera frame read failed"
                    self._publish()
                    self._stop.wait(0.1)
                    continue
                frame = self._apply_camera_adjustments(frame)
                detection, bundle, display_frame = self._process_frame(frame)
                with self._lock:
                    self._raw_frame = frame.copy()
                    self._display_frame = display_frame.copy() if display_frame is not None else frame.copy()
                    self._latest = detection
                    self._latest_bundle = bundle
                self._run_ibvs_step(detection, bundle)
                elapsed_ms = (time.perf_counter() - started) * 1000.0
                research_logger.record("vision_time", round(elapsed_ms, 3))
                self._status = "Running" if detection or bundle else "No object"
                self._publish()
                self._stop.wait(0.05)
        finally:
            try:
                cap.release()
            except Exception:
                pass
            self._status = "Stopped"
            self._publish()

    def _run_ibvs_step(self, detection: dict[str, Any] | None, bundle: dict[str, Any] | None) -> None:
        with self._lock:
            controller = self._ibvs_controller
            active = self._ibvs_active
            dispatch_enabled = self._ibvs_dispatch
            ipc_arrays = self._ibvs_ipc
        if controller is None or not active:
            return
        if not bool(self.config().get("ibvs_enabled", False)):
            return
        payload = bundle if bundle else detection
        if payload is None:
            return
        try:
            snapshot = controller.update(payload)
        except Exception as exc:
            research_logger.record("ibvs_error", 1, str(exc))
            return
        if dispatch_enabled and ipc_arrays is not None and snapshot.get("should_command"):
            try:
                controller.dispatch(snapshot, ipc_arrays)
                research_logger.record(
                    "ibvs_dispatch",
                    1,
                    f"dx={snapshot['cmd_mm']['dx']:.3f},dy={snapshot['cmd_mm']['dy']:.3f}",
                )
            except Exception as exc:
                research_logger.record("ibvs_dispatch_error", 1, str(exc))
        update_state("vision_ibvs", controller.to_dict() | {"active": True, "dispatch": dispatch_enabled})

    def _open_capture(self, source: str) -> Any:
        if source.upper() == "MOCK":
            return _SyntheticCapture()
        import cv2

        if source.isdigit():
            index = int(source)
            if sys.platform.startswith("win"):
                cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
                if cap.isOpened():
                    return cap
                try:
                    cap.release()
                except Exception:
                    pass
            return cv2.VideoCapture(index)
        return cv2.VideoCapture(source)

    def _process_frame(self, frame: Any) -> tuple[dict[str, Any] | None, dict[str, Any] | None, Any]:
        self._ensure_vision_modules()
        import cv2
        import numpy as np
        from backend.config_manager import ConfigManager
        from vision.camera_calibration import CameraCalibrationManager
        from vision.object_detector import ObjectDetector
        from vision.workspace_validator import WorkspaceValidator

        cfg_manager = ConfigManager.instance()
        cfg_manager.reload()
        workspace_validator = WorkspaceValidator(cfg_manager)
        calibration = CameraCalibrationManager(cfg_manager)
        intrinsic = calibration.intrinsic_matrix_for_frame(frame.shape)
        z_mm = float(load_config().get("workspace", {}).get("z_fixed_mm", 200.0))
        method = str(load_config().get("vision", {}).get("detection_method", "adaptive")).lower()
        preview_pick_zone = bool(load_config().get("vision", {}).get("preview_pick_zone", True))
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        detection_payload: dict[str, Any] | None = None
        bundle_payload: dict[str, Any] | None = None
        overlay = workspace_validator.draw_overlay(frame, intrinsic, z_mm)
        try:
            if method == "model":
                if self._model_detector is None:
                    self.load_model(load_config().get("vision", {}).get("model_path", ""))
                if self._model_detector is not None and getattr(self._model_detector, "is_loaded", False):
                    bundles = self._model_detector.detect_bundles(frame, intrinsic, z_mm)
                    if bundles:
                        bundle = bundles[0]
                        bundle_payload = bundle.as_dict()
                        overlay = self._model_detector.draw_bundle_overlay(overlay, bundle)
                        result, mask = self._model_detector.detect(frame, intrinsic, z_mm)
                        detection_payload = result.as_dict() if result is not None else None
                elif not self._model_warned:
                    self._status = "Model mode selected but ONNX is not loaded"
                    self._model_warned = True
            else:
                detector = ObjectDetector(cfg_manager, workspace_validator)
                result, mask = detector.detect(frame, intrinsic, z_mm)
                if result is not None:
                    detection_payload = result.as_dict()
                    overlay = detector.draw_detection_overlay(overlay, result, preview_pick_zone)
        except Exception as exc:
            self._status = f"Vision detection error: {exc}"
            research_logger.record("vision_error", 1, str(exc))
        composed = self._compose_frame(overlay, mask)
        return detection_payload, bundle_payload, composed

    def _compose_frame(self, overlay: Any, mask: Any | None) -> Any:
        if not bool(self.config().get("show_mask_preview", False)) or mask is None:
            return overlay
        import cv2
        import numpy as np

        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) if len(mask.shape) == 2 else mask
        target_h = overlay.shape[0]
        ratio = target_h / max(mask_bgr.shape[0], 1)
        target_w = max(1, int(mask_bgr.shape[1] * ratio))
        mask_bgr = cv2.resize(mask_bgr, (target_w, target_h))
        separator = np.full((target_h, 4, 3), (36, 61, 107), dtype=np.uint8)
        return np.hstack([overlay, separator, mask_bgr])

    def _apply_camera_adjustments(self, frame: Any) -> Any:
        import cv2

        cfg = self.config()
        brightness = int(cfg.get("brightness", 0))
        contrast = float(cfg.get("contrast", 1.0))
        adjusted = cv2.convertScaleAbs(frame, alpha=contrast, beta=brightness)
        zoom = max(1.0, float(cfg.get("zoom", 1.0)))
        if zoom <= 1.01:
            return adjusted
        height, width = adjusted.shape[:2]
        crop_w = max(1, int(width / zoom))
        crop_h = max(1, int(height / zoom))
        x1 = max(0, (width - crop_w) // 2)
        y1 = max(0, (height - crop_h) // 2)
        cropped = adjusted[y1:y1 + crop_h, x1:x1 + crop_w]
        return cv2.resize(cropped, (width, height))

    def _publish(self, calibration: dict[str, Any] | None = None) -> None:
        update_state(
            "vision",
            {
                "status": self._status,
                "model_status": self._model_status,
                "latest": self._latest,
                "latest_bundle": self._latest_bundle,
                "calibration": calibration if calibration is not None else self.calibration_summary_safe(),
                "updated_at": time.time(),
            },
        )

    def calibration_summary_safe(self) -> dict[str, Any]:
        try:
            return self.calibration_summary()
        except Exception:
            return load_config().get("vision", {}).get("calibration", {})

    @staticmethod
    def _ensure_vision_modules() -> None:
        missing = []
        for path in [
            PROJECT_ROOT / "backend" / "config_manager.py",
            PROJECT_ROOT / "vision" / "object_detector.py",
            PROJECT_ROOT / "vision" / "workspace_validator.py",
        ]:
            if not path.exists():
                missing.append(str(path))
        if missing:
            raise RuntimeError("Missing experimental vision source files: " + ", ".join(missing))


modbus_manager = ModbusManager()
vision_manager = VisionManager()


def _workspace_check(x_mm: float, y_mm: float) -> tuple[str, str]:
    workspace = load_config().get("workspace", {})
    if not bool(workspace.get("enabled", True)):
        return "valid", "WORKSPACE CHECK DISABLED"
    x_min = float(workspace.get("x_min_mm", -200.0))
    x_max = float(workspace.get("x_max_mm", 200.0))
    y_min = float(workspace.get("y_min_mm", -200.0))
    y_max = float(workspace.get("y_max_mm", 200.0))
    margin = float(workspace.get("margin_mm", 10.0))
    if x_mm < x_min or x_mm > x_max or y_mm < y_min or y_mm > y_max:
        return "out", "OUT OF WORKSPACE"
    if x_mm < x_min + margin or x_mm > x_max - margin or y_mm < y_min + margin or y_mm > y_max - margin:
        return "margin", "NEAR BOUNDARY"
    return "valid", "READY TO PICK"


def _wait_for_marginal_confirmation(timeout_s: float, shared_string: Any) -> bool:
    token = time.time()
    update_state("vision_confirmation", {"status": "pending", "token": token, "decision": "", "updated_at": time.time()})
    _set_shared(shared_string, "Log: vision() waiting for marginal confirmation")
    deadline = time.time() + max(0.1, timeout_s)
    while time.time() < deadline:
        control = read_state("program_control", {})
        if isinstance(control, dict) and control.get("stop_requested"):
            update_state("vision_confirmation", {"status": "stopped", "token": token, "decision": "skip", "updated_at": time.time()})
            return False
        state = read_state("vision_confirmation", {})
        if isinstance(state, dict) and state.get("token") == token:
            decision = str(state.get("decision", "")).lower()
            if decision == "confirm":
                update_state("vision_confirmation", {"status": "confirmed", "token": token, "decision": "confirm", "updated_at": time.time()})
                return True
            if decision == "skip":
                update_state("vision_confirmation", {"status": "skipped", "token": token, "decision": "skip", "updated_at": time.time()})
                return False
        time.sleep(0.05)
    update_state("vision_confirmation", {"status": "timeout", "token": token, "decision": "skip", "updated_at": time.time()})
    return False


def build_vision_pick_sequence(shared_string: Any) -> list[str] | None:
    cfg = load_config()
    vision_cfg = cfg.get("vision", {})
    if not bool(vision_cfg.get("auto_pick_enabled", True)):
        _set_shared(shared_string, "Error: vision() auto pick disabled")
        research_logger.record("vision_pick_result", 0, "auto pick disabled")
        return None
    latest = vision_manager.latest()
    bundle = vision_manager.latest_bundle()
    pick_world = None
    safety = ""
    if isinstance(bundle, dict):
        pick_world = bundle.get("pick_point_world")
        safety = str(bundle.get("pick_safety", "")).upper()
    if not pick_world and isinstance(latest, dict):
        pick_world = latest.get("pick_point_world")
        safety = str(latest.get("workspace_status", "")).upper()
    if not pick_world:
        _set_shared(shared_string, "Error: vision() no detection available")
        research_logger.record("vision_pick_result", 0, "no detection")
        return None
    x_mm, y_mm = [float(value) for value in pick_world[:2]]
    workspace_status, workspace_message = _workspace_check(x_mm, y_mm)
    if safety == "UNSAFE" or workspace_status == "out":
        try:
            modbus_manager.write_value("error_flag", True)
        except Exception:
            pass
        reason = "unsafe pick" if safety == "UNSAFE" else workspace_message
        _set_shared(shared_string, f"Error: vision() {reason}")
        research_logger.record("workspace_violation", 1, f"x={x_mm:.3f},y={y_mm:.3f},safety={safety}")
        research_logger.record("vision_pick_result", 0, reason)
        return None
    if safety == "MARGINAL":
        research_logger.record("marginal_event", 1, f"x={x_mm:.3f},y={y_mm:.3f}")
        confirmed = _wait_for_marginal_confirmation(float(vision_cfg.get("marginal_confirm_timeout_s", 2.0)), shared_string)
        if not confirmed:
            _set_shared(shared_string, "Log: vision() marginal pick skipped")
            research_logger.record("vision_pick_result", 0, "marginal skipped")
            return []
    elif safety == "UNKNOWN":
        _set_shared(shared_string, "Log: vision() fixture unknown, using fallback pick point")
        research_logger.record("unknown_fixture", 1, f"x={x_mm:.3f},y={y_mm:.3f}")
    elif workspace_status == "margin":
        _set_shared(shared_string, "Log: vision() target near workspace boundary")
        research_logger.record("workspace_warning", 1, f"x={x_mm:.3f},y={y_mm:.3f}")

    z_mm = float(cfg.get("workspace", {}).get("z_fixed_mm", 200.0))
    rpy = list(vision_cfg.get("pick_pose_rpy_deg", [0.0, 0.0, 0.0]))[:3]
    while len(rpy) < 3:
        rpy.append(0.0)
    pick_time = float(vision_cfg.get("pick_move_time_s", 4.0))
    descent = float(vision_cfg.get("post_pick_descent_mm", 50.0))
    descent_time = float(vision_cfg.get("post_pick_move_time_s", 2.0))
    gripper = list(vision_cfg.get("gripper_close", [255, 100, 120]))[:3]
    while len(gripper) < 3:
        gripper.append([255, 100, 120][len(gripper)])
    commands = [
        f"MovePose({x_mm:.3f},{y_mm:.3f},{z_mm:.3f},{float(rpy[0]):.3f},{float(rpy[1]):.3f},{float(rpy[2]):.3f},t={pick_time:.3f})",
        f"MoveCartRelTRF(0,0,{descent:.3f},0,0,0,t={descent_time:.3f})",
        f"Gripper({int(gripper[0])},{int(gripper[1])},{int(gripper[2])})",
    ]
    research_logger.record("vision_pick_target", 1, f"x={x_mm:.3f},y={y_mm:.3f},z={z_mm:.3f},safety={safety or workspace_status}")
    research_logger.record("vision_sequence_generated", len(commands), "|".join(commands))
    _set_shared(shared_string, f"Log: vision() target x={x_mm:.1f}, y={y_mm:.1f}")
    return commands


def execute_vision_command(shared_string: Any) -> bool:
    sequence = build_vision_pick_sequence(shared_string)
    return sequence is not None


def execute_modbus_read(command_text: str, shared_string: Any) -> bool:
    try:
        values = parse_command_values(command_text)
        selector = values[0] if values else "trigger_pick"
        expected = values[1] if len(values) > 1 and not isinstance(values[1], tuple) else None
        timeout = 0.0
        for value in values:
            if isinstance(value, tuple) and value[0] == "timeout":
                timeout = float(value[1])
        deadline = time.time() + max(timeout, 0.0)
        while True:
            current = modbus_manager.read_value(selector)
            matched = current is not None if expected is None else _coerce_bool(current) == _coerce_bool(expected)
            if matched:
                _set_shared(shared_string, f"Log: ModbusRead({selector})={current}")
                research_logger.record("modbus_read", int(_coerce_bool(current)), str(selector))
                return True
            control = read_state("program_control", {})
            if isinstance(control, dict) and control.get("stop_requested"):
                _set_shared(shared_string, f"Log: ModbusRead({selector}) stopped")
                research_logger.record("modbus_read_stopped", 1, str(selector))
                return False
            if time.time() >= deadline:
                _set_shared(shared_string, f"Error: ModbusRead({selector}) timeout")
                research_logger.record("modbus_timeout", 1, str(selector))
                return False
            time.sleep(0.02)
    except Exception as exc:
        _set_shared(shared_string, f"Error: ModbusRead failed: {exc}")
        research_logger.record("modbus_error", 1, str(exc))
        return False


def execute_modbus_write(command_text: str, shared_string: Any) -> bool:
    try:
        values = parse_command_values(command_text)
        if len(values) < 2:
            raise ValueError("ModbusWrite requires selector and value")
        selector, value = values[0], values[1]
        ok = modbus_manager.write_value(selector, value)
        _set_shared(shared_string, f"Log: ModbusWrite({selector})={value}")
        research_logger.record("modbus_write", int(ok), str(selector))
        return ok
    except Exception as exc:
        _set_shared(shared_string, f"Error: ModbusWrite failed: {exc}")
        research_logger.record("modbus_error", 1, str(exc))
        return False
