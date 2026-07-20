from __future__ import annotations

import ast
import csv
import json
import math
import shutil
import sys
import threading
import time
import uuid
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
# Cross-process Modbus write queue. Only the process that owns the poll loop
# holds the single Modbus-TCP connection the PLC allows, so other processes
# drop write requests here for the owner to execute on that live connection.
MODBUS_WRITE_DIR = PROJECT_ROOT / "runtime_modbus_writes"
MODBUS_WRITE_ROUTE_TIMEOUT_S = 5.0
LOG_DIR = PROJECT_ROOT / "logs"
TIMESTAMP_TABLE_FIELDS = ["time", "event", "label", "elapsed_s", "duration_s"]
MODBUS_CYCLE_LOG_COLUMNS = [
    "cycle_index",
    "timestamp",
    "response_time_ms",
    "packet_status",
    "cycle_time_s",
    "outcome",
]
MODBUS_CYCLE_LOG_HEADINGS = {
    "cycle_index": "No. Siklus",
    "timestamp": "Timestamp",
    "response_time_ms": "Response Time (ms)",
    "packet_status": "Packet Status",
    "cycle_time_s": "Cycle Time (s)",
    "outcome": "Outcome",
}


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
        "camera_width": 1280,
        "camera_height": 720,
        "detection_enabled": True,
        "detection_method": "model",
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
        "model_runtime": "cuda",
        "model_conf_threshold": 0.5,
        "model_iou_threshold": 0.45,
        "safe_pick_margin_pct": 0.25,
        "safe_pick_left_offset_px": 0.0,
        "pick_accuracy_test": {
            "enabled": False,
            "threshold_mm": 3.0,
            "log_path": "logs/pick_accuracy_trials.json",
        },
        "marginal_confirm_timeout_s": 2.0,
        "offset_x_mm": 0.0,
        "offset_y_mm": 0.0,
        "z_tool_offset_mm": 0.0,
        "camera_to_base": {
            "valid": False,
            "homography": [],
            "image_size": [0, 0],
            "point_count": 0,
            "rms_error_mm": None,
            "max_error_mm": None,
        },
        "camera_to_base_points": [],
        "camera_to_base_points_visible": True,
        "tool_z": {
            "reference_joint_deg": [90.0, -88.0, 182.259, 0.0, 3.0, 180.0],
            "pose_tolerance_deg": 1.0,
            "x_tool_fixed_mm": -60.0,
            "y_tool_fixed_mm": 0.0,
            "z_plus_min_mm": 0.0,
            "z_plus_max_mm": 78.0,
            "retreat_margin_mm": 30.0,
            "sample_count": 5,
            "detection_max_age_s": 1.0,
            "runtime_max_age_s": 30.0,
            "same_object_min_iou": 0.3,
        },
        "calibration": {
            "snapshots_captured": 0,
            "chessboard_size": [9, 6],
            "square_size_mm": 25.0,
            "preview_enabled": False,
            "snapshot_dir": "tools/Camera/calibration_snapshots",
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
_vision_runtime_lock = threading.Lock()
_vision_runtime_cache: dict[str, Any] = {}


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


def emit_program_log(
    shared_string: Any,
    program_log_queue: Any,
    message: str,
) -> None:
    _set_shared(shared_string, message)
    if program_log_queue is None:
        return
    try:
        program_log_queue.put_nowait(
            {"timestamp": time.time(), "message": str(message)}
        )
    except Exception:
        pass


def set_vision_runtime(runtime: dict[str, Any]) -> None:
    payload = dict(runtime)
    with _vision_runtime_lock:
        _vision_runtime_cache.clear()
        _vision_runtime_cache.update(payload)
    update_state("vision_runtime", payload)


def get_vision_runtime() -> dict[str, Any]:
    with _vision_runtime_lock:
        if _vision_runtime_cache:
            return dict(_vision_runtime_cache)
    runtime = read_state("vision_runtime", {})
    return dict(runtime) if isinstance(runtime, dict) else {}


def reset_vision_runtime() -> None:
    set_vision_runtime(
        {
            "valid": False,
            "status": "EMPTY",
            "updated_at": time.time(),
            "expires_at": 0.0,
        }
    )


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
        # Serializes actual client socket I/O so the poll thread and any
        # write caller in the same process never interleave Modbus frames
        # (interleaved frames make the PLC reset the connection).
        self._client_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._snapshot: dict[str, Any] = {}
        self._connected = False
        self._description = "Disconnected"
        self._client: Any = None
        self._last_cycle_ms = 0.0
        self._block_b_lock = threading.Lock()
        self._block_b_full_samples: list[dict[str, Any]] = []
        self._block_b_full_meta: dict[str, Any] = {}
        self._modbus_cycle_log: list[dict[str, Any]] = []
        self._modbus_cycle_meta: dict[str, Any] = {}
        self._block_b_state = self._new_block_b_state(load_config()["modbus"].get("block_b", {}))
        self._block_b_clear_done = False
        update_state("modbus_block_b", dict(self._block_b_state))
        update_state("modbus_cycle_log", self._cycle_log_payload_unlocked())

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
        with self._client_lock:
            self._disconnect()
        self._connected = False
        self._description = "Disconnected"
        self._publish()

    def save_block_test_config(self, block_b: dict[str, Any] | None = None) -> None:
        cfg = load_config()
        if block_b is not None:
            cfg.setdefault("modbus", {}).setdefault("block_b", {}).update(block_b)
        save_config(cfg)

    def start_block_b(self, settings: dict[str, Any]) -> None:
        self.save_block_test_config(block_b=settings)
        with self._block_b_lock:
            self._block_b_state = self._new_block_b_state(settings)
            self._block_b_state["running"] = True
            self._block_b_state["started_at"] = time.time()
            self._block_b_state["cycle_log_count"] = 0
            self._block_b_full_samples.clear()
            self._modbus_cycle_log.clear()
            self._modbus_cycle_meta = {
                "started_at": self._block_b_state["started_at"],
                "settings": dict(settings),
                "target": int(self._block_b_state["target"]),
            }
            update_state("modbus_block_b", dict(self._block_b_state))
            update_state("modbus_cycle_log", self._cycle_log_payload_unlocked())
            self._block_b_full_meta = dict(self._modbus_cycle_meta)
        # A done/error coil left high by an earlier run (e.g. the app was
        # closed mid-handshake) blocks the PLC from raising the first trigger.
        self._block_b_clear_done = False
        try:
            self.write_value(str(settings.get("done_name", "cycle_done")), False)
        except Exception:
            pass
        try:
            self.write_value(str(settings.get("error_name", "error_flag")), False)
        except Exception:
            pass
        research_logger.record("modbus_block_b_test_start", int(self._block_b_state["target"]), "cycle time test")

    def stop_block_b(self) -> None:
        with self._block_b_lock:
            self._block_b_state["running"] = False
            self._block_b_state["active_cycle"] = False
            self._block_b_state["updated_at"] = time.time()
            update_state("modbus_block_b", dict(self._block_b_state))
            self._block_b_full_meta.update({
                "finished_at": time.time(),
                "completed": int(self._block_b_state.get("completed", 0)),
                "stats": dict(self._block_b_state.get("stats", {})),
                "stopped_early": int(self._block_b_state.get("completed", 0)) < int(self._block_b_state.get("target", 0)),
            })
            self._modbus_cycle_meta.update({
                "finished_at": time.time(),
                "completed": int(self._block_b_state.get("completed", 0)),
                "stopped_early": int(self._block_b_state.get("completed", 0)) < int(self._block_b_state.get("target", 0)),
            })
            update_state("modbus_cycle_log", self._cycle_log_payload_unlocked())
        research_logger.record("modbus_block_b_test_stop", int(self._block_b_state.get("completed", 0)))

    def block_b_cycle_started(self, response_time_ms: int | float | None = None, packet_status: str = "OK") -> bool:
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
            if response_time_ms is None:
                response_time_ms = self._last_cycle_ms
            cycle_row = {
                "cycle_index": next_index,
                "timestamp": now,
                "response_time_ms": _round_float(response_time_ms, 3, 0.0),
                "packet_status": _normalize_packet_status(packet_status),
                "cycle_time_s": "",
                "outcome": "",
            }
            self._modbus_cycle_log.append(cycle_row)
            self._block_b_state["cycle_log_count"] = len(self._modbus_cycle_log)
            self._block_b_state["updated_at"] = now
            update_state("modbus_block_b", dict(self._block_b_state))
            update_state("modbus_cycle_log", self._cycle_log_payload_unlocked())
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
            active_index = int(self._block_b_state.get("active_index", self._block_b_state.get("completed", 0) + 1))
            samples = list(self._block_b_state.get("samples", []))
            if success:
                samples.append(round(duration, 6))
                self._block_b_state["success_count"] = int(self._block_b_state.get("success_count", 0)) + 1
            else:
                self._block_b_state["failed_count"] = int(self._block_b_state.get("failed_count", 0)) + 1
            self._block_b_state["completed"] = int(self._block_b_state.get("completed", 0)) + 1
            self._block_b_state["samples"] = samples[-300:]
            self._block_b_full_samples.append({
                "index": active_index,
                "timestamp": now,
                "status": "success" if success else "failed",
                "duration_s": round(duration, 6),
                "note": note,
            })
            cycle_row = None
            for row in reversed(self._modbus_cycle_log):
                if int(row.get("cycle_index", 0) or 0) == active_index:
                    cycle_row = row
                    break
            if cycle_row is None:
                cycle_row = {
                    "cycle_index": active_index,
                    "timestamp": start_epoch,
                    "response_time_ms": _round_float(self._last_cycle_ms, 3, 0.0),
                    "packet_status": "OK",
                    "cycle_time_s": "",
                    "outcome": "",
                }
                self._modbus_cycle_log.append(cycle_row)
            cycle_row["cycle_time_s"] = round(duration, 6)
            cycle_row["outcome"] = "Sukses" if success else "Gagal"
            stats = _cycle_stats(samples)
            self._block_b_state["stats"] = stats
            self._block_b_state["cycle_log_count"] = len(self._modbus_cycle_log)
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
                self._block_b_full_meta.update({
                    "finished_at": now,
                    "completed": int(self._block_b_state.get("completed", 0)),
                    "stats": dict(self._block_b_state.get("stats", {})),
                    "stopped_early": False,
                })
                self._modbus_cycle_meta.update({
                    "finished_at": now,
                    "completed": int(self._block_b_state.get("completed", 0)),
                    "stopped_early": False,
                })
            self._block_b_state["updated_at"] = now
            payload = dict(self._block_b_state)
            update_state("modbus_block_b", payload)
            update_state("modbus_cycle_log", self._cycle_log_payload_unlocked())

        try:
            self.write_value(done_name, bool(success))
            self.write_value(error_name, not bool(success))
        except Exception as exc:
            research_logger.record("modbus_block_b_flag_error", 1, str(exc))
        if success:
            # Phase 4 of the handshake still owes the PLC a falling edge on
            # the done coil; the poll loop performs it once the PLC lowers
            # its trigger (see _finish_block_b_handshake).
            self._block_b_clear_done = True

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
        # The PLC accepts a single Modbus-TCP connection, held by whichever
        # process runs the poll loop. If that owner is a different process
        # (e.g. the GUI while a program executes in the executor process),
        # route the write through it instead of opening a second socket the
        # PLC would refuse ("Unable to connect"). Fall back to a direct write
        # if the request could not be routed.
        if not self._owns_poll_loop() and self._poll_loop_active_elsewhere():
            routed = self._write_via_queue(entry, value)
            if routed is not None:
                return routed
        return self._write_direct(entry, cfg, value)

    def _write_direct(self, entry: "ModbusAddress", cfg: dict[str, Any], value: Any) -> bool:
        slave_id = int(cfg.get("slave_id", 1))
        register_type = entry.type.lower()

        def _do_write() -> Any:
            with self._client_lock:
                if self._client is None:
                    self._ensure_connected(cfg)
                if register_type == "coil":
                    # PLC only supports 0F (Write Multiple Coils) for the Work
                    # Area, so a single bit is written as a one-element block.
                    return _call_modbus(
                        self._client.write_coils,
                        {"address": entry.address, "values": [_coerce_bool(value)]},
                        slave_id,
                    )
                if register_type in {"holding_register", "register"}:
                    # 06 (Write Single Register) into the Data Memory Area.
                    return _call_modbus(
                        self._client.write_register,
                        {"address": entry.address, "value": int(value)},
                        slave_id,
                    )
                raise ValueError(f"Address '{entry.name}' is not writable as type {entry.type}")

        try:
            result = _do_write()
        except _modbus_link_errors():
            # Transient link drop (e.g. Wi-Fi blip): drop the stale socket and
            # retry once with a fresh connection before failing the command.
            with self._client_lock:
                self._disconnect()
            try:
                result = _do_write()
            except _modbus_link_errors():
                # Still down: drop the dead client too, otherwise a process
                # without a poll loop caches it and every later write fails.
                with self._client_lock:
                    self._disconnect()
                raise
        ok = not bool(getattr(result, "isError", lambda: False)())
        if not self._owns_poll_loop():
            # This process only borrowed the PLC's single connection slot for
            # a fallback write; caching the socket would lock the poll-loop
            # owner out ("Unable to connect") for as long as we live.
            with self._client_lock:
                self._disconnect()
        if ok:
            with self._lock:
                self._snapshot[entry.name] = _mock_value_for_type(entry.type, value)
            self._publish()
        return ok

    def _owns_poll_loop(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _poll_loop_active_elsewhere(self) -> bool:
        """True if some other process is actively running the Modbus poll loop
        (recent heartbeat in shared state), so it owns the PLC connection."""
        state = read_state("modbus", {})
        if not isinstance(state, dict):
            return False
        updated_at = float(state.get("updated_at", 0.0) or 0.0)
        interval_s = float(self.config().get("poll_interval_ms", 100)) / 1000.0
        heartbeat = max(3.0, 5.0 * interval_s)
        return (time.time() - updated_at) < heartbeat

    def _write_via_queue(self, entry: "ModbusAddress", value: Any) -> bool | None:
        """Ask the poll-loop-owning process to perform the write. Returns the
        boolean result, or None if it could not be routed (caller falls back)."""
        try:
            MODBUS_WRITE_DIR.mkdir(parents=True, exist_ok=True)
            request_id = uuid.uuid4().hex
            req_path = MODBUS_WRITE_DIR / f"req_{request_id}.json"
            tmp_path = req_path.with_suffix(".tmp")
            tmp_path.write_text(
                json.dumps({"name": entry.name, "value": value, "ts": time.time()}),
                encoding="utf-8",
            )
            tmp_path.replace(req_path)  # atomic: the owner only ever sees a complete file
        except Exception:
            return None
        res_path = MODBUS_WRITE_DIR / f"res_{request_id}.json"
        deadline = time.time() + MODBUS_WRITE_ROUTE_TIMEOUT_S
        while time.time() < deadline:
            if res_path.exists():
                try:
                    result = json.loads(res_path.read_text(encoding="utf-8"))
                except Exception:
                    result = {}
                try:
                    res_path.unlink()
                except OSError:
                    pass
                if result.get("error"):
                    raise RuntimeError(str(result["error"]))
                return bool(result.get("ok", False))
            time.sleep(0.02)
        # Timed out waiting for the owner: abandon the request so it is skipped.
        try:
            req_path.unlink()
        except OSError:
            pass
        return None

    def _drain_write_queue(self, cfg: dict[str, Any]) -> None:
        """Owner side: execute any pending cross-process write requests on the
        live connection and post their results. Runs inside the poll thread."""
        if not MODBUS_WRITE_DIR.exists():
            return
        now = time.time()
        for req_path in sorted(MODBUS_WRITE_DIR.glob("req_*.json")):
            try:
                payload = json.loads(req_path.read_text(encoding="utf-8"))
            except Exception:
                self._safe_unlink(req_path)
                continue
            request_id = req_path.stem[len("req_"):]
            # Skip requests the caller already gave up on (avoids double writes).
            if now - float(payload.get("ts", 0.0) or 0.0) > MODBUS_WRITE_ROUTE_TIMEOUT_S:
                self._safe_unlink(req_path)
                continue
            result: dict[str, Any] = {"ok": False, "error": ""}
            try:
                entry = self._address_for(payload.get("name"))
                if entry is None:
                    raise KeyError(f"Unknown Modbus selector: {payload.get('name')}")
                result["ok"] = self._write_direct(entry, cfg, payload.get("value"))
            except Exception as exc:
                result["error"] = str(exc)
            res_path = MODBUS_WRITE_DIR / f"res_{request_id}.json"
            try:
                tmp_path = res_path.with_suffix(".tmp")
                tmp_path.write_text(json.dumps(result), encoding="utf-8")
                tmp_path.replace(res_path)
            except Exception:
                pass
            self._safe_unlink(req_path)
        # Clean orphaned result files (caller died before reading them).
        for res_path in MODBUS_WRITE_DIR.glob("res_*.json"):
            try:
                if now - res_path.stat().st_mtime > MODBUS_WRITE_ROUTE_TIMEOUT_S * 2:
                    res_path.unlink()
            except OSError:
                pass

    @staticmethod
    def _safe_unlink(path: Path) -> None:
        try:
            path.unlink()
        except OSError:
            pass

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
            wait_s = max(0.05, float(cfg.get("poll_interval_ms", 100)) / 1000.0)
            try:
                try:
                    snapshot = self._poll_pass(cfg)
                except Exception:
                    # A failed poll at a cycle boundary is usually transient
                    # (the PLC scan stalls while it handles the handshake):
                    # retry once on a fresh socket so one slow response does
                    # not tear down the connection between cycles.
                    with self._client_lock:
                        self._disconnect()
                    snapshot = self._poll_pass(cfg)
                with self._lock:
                    self._snapshot.update(snapshot)
                self._last_cycle_ms = (time.perf_counter() - started) * 1000.0
                research_logger.record("modbus_cycle", round(self._last_cycle_ms, 3))
                self._publish()
                self._finish_block_b_handshake(snapshot)
            except Exception as exc:
                self._connected = False
                self._description = str(exc)
                with self._client_lock:
                    self._disconnect()
                self._publish()
                research_logger.record("modbus_error", 1, str(exc))
                # Redialing at poll rate hammers the PLC's single server slot
                # with connection attempts and can keep it from ever freeing
                # the old session; back off to the failsafe interval instead.
                try:
                    failsafe = load_config().get("failsafe", {})
                    wait_s = max(wait_s, float(failsafe.get("modbus_reconnect_interval_s", 5)))
                except Exception:
                    wait_s = max(wait_s, 5.0)
            # Execute writes routed from other processes on our live connection.
            try:
                self._drain_write_queue(cfg)
            except Exception:
                pass
            self._stop.wait(wait_s)

    def _poll_pass(self, cfg: dict[str, Any]) -> dict[str, Any]:
        with self._client_lock:
            self._ensure_connected(cfg)
        return self._poll_snapshot(cfg)

    def _finish_block_b_handshake(self, snapshot: dict[str, Any]) -> None:
        # Robot side of handshake phase 4: once the PLC has acknowledged our
        # done signal by lowering its trigger, lower the done coil again. A
        # done coil that stays high keeps the PLC from raising the trigger
        # for the next cycle (it times out and latches its error bit instead).
        if not self._block_b_clear_done:
            return
        with self._block_b_lock:
            trigger_name = str(self._block_b_state.get("trigger_name", "trigger_pick"))
            done_name = str(self._block_b_state.get("done_name", "cycle_done"))
        if _coerce_bool(snapshot.get(trigger_name, False)):
            return  # PLC has not acknowledged yet
        try:
            if self.write_value(done_name, False):
                self._block_b_clear_done = False
                research_logger.record("modbus_block_b_done_cleared", 1)
        except Exception as exc:
            research_logger.record("modbus_block_b_flag_error", 1, str(exc))

    def _ensure_connected(self, cfg: dict[str, Any]) -> None:
        # Caller must hold _client_lock: two threads creating clients at once
        # would leak a socket on the PLC's single-connection slot.
        if self._client is not None:
            self._connected = True
            return
        try:
            from pymodbus.client import ModbusTcpClient
        except Exception as exc:
            raise RuntimeError(f"pymodbus unavailable: {exc}") from exc
        host = str(cfg.get("ip", "127.0.0.1"))
        port = int(cfg.get("port", 502))
        timeout_s = max(0.2, float(cfg.get("timeout_ms", 3000)) / 1000.0)
        self._client = ModbusTcpClient(host=host, port=port, timeout=timeout_s)
        if not self._client.connect():
            # Do not keep a dead client around: a cached, unconnected client
            # would make the next _ensure_connected() falsely return "connected".
            self._disconnect()
            self._connected = False
            raise ConnectionError(f"Unable to connect to {host}:{port}")
        self._connected = True
        self._description = f"{host}:{port}"

    def _disconnect(self) -> None:
        # Caller must hold _client_lock so the client is never nulled out
        # underneath a thread that is mid-read or mid-write on it.
        if self._client is not None:
            try:
                self._client.close()
            finally:
                self._client = None

    def _poll_snapshot(self, cfg: dict[str, Any]) -> dict[str, Any]:
        slave_id = int(cfg.get("slave_id", 1))
        snapshot: dict[str, Any] = {}
        # Hold the client lock for the whole read burst so a concurrent write
        # in this process cannot interleave a frame mid-poll.
        with self._client_lock:
            if self._client is None:
                return {}
            for entry in self._addresses():
                register_type = entry.type.lower()
                if register_type == "coil":
                    result = _call_modbus(self._client.read_coils, {"address": entry.address, "count": 1}, slave_id)
                    if result.isError():
                        raise RuntimeError(f"Read failed for {entry.name}")
                    snapshot[entry.name] = bool(result.bits[0])
                elif register_type == "discrete_input":
                    result = _call_modbus(self._client.read_discrete_inputs, {"address": entry.address, "count": 1}, slave_id)
                    if result.isError():
                        raise RuntimeError(f"Read failed for {entry.name}")
                    snapshot[entry.name] = bool(result.bits[0])
                elif register_type in {"holding_register", "register"}:
                    result = _call_modbus(self._client.read_holding_registers, {"address": entry.address, "count": 1}, slave_id)
                    if result.isError():
                        raise RuntimeError(f"Read failed for {entry.name}")
                    snapshot[entry.name] = int(result.registers[0])
                elif register_type == "input_register":
                    result = _call_modbus(self._client.read_input_registers, {"address": entry.address, "count": 1}, slave_id)
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

    def get_block_b_full_samples(self) -> list[dict[str, Any]]:
        with self._block_b_lock:
            return list(self._block_b_full_samples)

    def get_block_b_full_meta(self) -> dict[str, Any]:
        with self._block_b_lock:
            return dict(self._block_b_full_meta)

    def get_modbus_cycle_log(self) -> list[dict[str, Any]]:
        with self._block_b_lock:
            return [dict(row) for row in self._modbus_cycle_log]

    def clear_modbus_cycle_log(self, force: bool = False) -> bool:
        with self._block_b_lock:
            if not force and (self._block_b_state.get("running") or self._block_b_state.get("active_cycle")):
                return False
            self._modbus_cycle_log.clear()
            self._modbus_cycle_meta = {}
            self._block_b_state["cycle_log_count"] = 0
            update_state("modbus_block_b", dict(self._block_b_state))
            update_state("modbus_cycle_log", self._cycle_log_payload_unlocked())
        research_logger.record("modbus_cycle_log_clear", 1)
        return True

    def export_modbus_cycle_log(self, destination: str | Path) -> Path:
        destination_path = Path(destination)
        destination_path.parent.mkdir(parents=True, exist_ok=True)
        with self._block_b_lock:
            rows = [dict(row) for row in self._modbus_cycle_log]
        if destination_path.suffix.lower() == ".csv":
            with destination_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow([MODBUS_CYCLE_LOG_HEADINGS[column] for column in MODBUS_CYCLE_LOG_COLUMNS])
                for row in rows:
                    writer.writerow(_modbus_cycle_log_export_row(row))
            return destination_path

        try:
            from openpyxl import Workbook
        except Exception as exc:
            raise RuntimeError("openpyxl is required for Excel export") from exc

        workbook = Workbook()
        raw_sheet = workbook.active
        raw_sheet.title = "Data"
        raw_sheet.append([MODBUS_CYCLE_LOG_HEADINGS[column] for column in MODBUS_CYCLE_LOG_COLUMNS])
        for row in rows:
            raw_sheet.append(_modbus_cycle_log_export_row(row))

        summary = _modbus_cycle_log_stats(rows)
        summary_sheet = workbook.create_sheet("Ringkasan")
        summary_sheet.append(["Metrik", "Nilai"])
        for metric, value in [
            ("Avg Response Time", f"{summary['avg_rt_ms']} ms"),
            ("Throughput", f"{summary['throughput_rps']} req/s"),
            ("Packet Loss Rate", f"{summary['packet_loss_pct']} %"),
            ("Avg Cycle Time", f"{summary['avg_cycle_time_s']} s"),
            ("Jumlah Data", summary["cycle_time_count"]),
        ]:
            summary_sheet.append([metric, value])

        workbook.save(destination_path)
        return destination_path

    def _cycle_log_payload_unlocked(self) -> dict[str, Any]:
        return {
            "rows": [dict(row) for row in self._modbus_cycle_log[-300:]],
            "count": len(self._modbus_cycle_log),
            "stats": _modbus_cycle_log_stats(self._modbus_cycle_log),
            "updated_at": time.time(),
        }


def _cycle_stats(samples: list[float]) -> dict[str, Any]:
    if not samples:
        return {"avg_s": 0.0, "count": 0}
    return {
        "avg_s": round(sum(samples) / len(samples), 6),
        "count": len(samples),
    }


def _round_float(value: Any, digits: int, default: float = 0.0) -> float:
    try:
        return round(float(value), digits)
    except Exception:
        return round(float(default), digits)


def _number_or_none(value: Any) -> float | None:
    if value in ("", None):
        return None
    try:
        return float(value)
    except Exception:
        return None


def _normalize_packet_status(value: Any) -> str:
    text = str(value or "OK").strip().lower()
    if "timeout" in text:
        return "Timeout"
    if text in {"loss", "lost", "failed", "fail", "error", "disconnected"}:
        return "Loss"
    return "OK"


def _format_epoch_iso(value: Any) -> str:
    try:
        return datetime.fromtimestamp(float(value)).isoformat(timespec="milliseconds")
    except Exception:
        return str(value or "")


def _modbus_cycle_log_export_row(row: dict[str, Any]) -> list[Any]:
    values: list[Any] = []
    for column in MODBUS_CYCLE_LOG_COLUMNS:
        value = row.get(column, "")
        if column == "timestamp" and value not in ("", None):
            value = _format_epoch_iso(value)
        values.append(value)
    return values


def _modbus_cycle_log_stats(rows: list[dict[str, Any]]) -> dict[str, Any]:
    response_times = [
        value for value in (_number_or_none(row.get("response_time_ms")) for row in rows) if value is not None
    ]
    cycle_times = [
        value for value in (_number_or_none(row.get("cycle_time_s")) for row in rows) if value is not None
    ]
    timestamps = [
        value for value in (_number_or_none(row.get("timestamp")) for row in rows) if value is not None
    ]
    total = len(rows)
    loss_count = sum(1 for row in rows if _normalize_packet_status(row.get("packet_status")) in {"Loss", "Timeout"})
    if len(timestamps) >= 2:
        elapsed_s = max(max(timestamps) - min(timestamps), 1e-9)
        throughput = total / elapsed_s
    else:
        throughput = 0.0
    avg_rt = sum(response_times) / len(response_times) if response_times else 0.0
    avg_cycle = sum(cycle_times) / len(cycle_times) if cycle_times else 0.0
    packet_loss_pct = loss_count / max(total, 1) * 100.0 if total else 0.0
    return {
        "avg_rt_ms": round(avg_rt, 3),
        "throughput_rps": round(throughput, 3),
        "packet_loss_pct": round(packet_loss_pct, 3),
        "avg_cycle_time_s": round(avg_cycle, 6),
        "cycle_time_count": len(cycle_times),
    }


def _modbus_link_errors() -> tuple[type[BaseException], ...]:
    # pymodbus signals a dropped link with its own ModbusException family
    # (ConnectionException, ModbusIOException), not OSError/ConnectionError.
    try:
        from pymodbus.exceptions import ModbusException
    except Exception:
        return (ConnectionError, OSError)
    return (ConnectionError, OSError, ModbusException)


def _call_modbus(method: Any, kwargs: dict[str, Any], slave_id: int) -> Any:
    for device_keyword in ("slave", "unit", "device_id"):
        try:
            return method(**kwargs, **{device_keyword: slave_id})
        except TypeError as exc:
            if not _is_unexpected_modbus_keyword(exc, device_keyword):
                raise
    return method(**kwargs)


def _is_unexpected_modbus_keyword(exc: TypeError, keyword: str) -> bool:
    message = str(exc).lower()
    return keyword.lower() in message and "unexpected keyword" in message


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
        self._detection_enabled = bool(
            load_config().get("vision", {}).get("detection_enabled", True)
        )
        self._latest: dict[str, Any] | None = None
        self._latest_bundle: dict[str, Any] | None = None
        self._bundle_history: deque[dict[str, Any]] = deque(maxlen=20)
        self._display_frame: Any = None
        self._raw_frame: Any = None
        self._frame_size: tuple[int, int] = (0, 0)
        self._status = "Stopped"
        self._model_status = "NOT LOADED"
        self._model_detector: Any = None
        self._calibration_manager: Any = None
        self._chessboard_found = False
        self._chessboard_corner_count = 0
        self._chessboard_pattern: tuple[int, int] = (0, 0)
        self._chessboard_corners: Any = None
        self._chessboard_last_detection_at = 0.0
        self._model_warned = False
        self._ibvs_controller: Any = None
        self._ibvs_active: bool = False
        self._ibvs_ipc: dict[str, Any] | None = None
        self._ibvs_dispatch: bool = False
        self._last_publish_at = 0.0
        self._last_vision_time_log_at = 0.0

    def config(self) -> dict[str, Any]:
        return load_config()["vision"]

    def save_settings(self, settings: dict[str, Any], workspace: dict[str, Any] | None = None) -> None:
        cfg = load_config()
        cfg.setdefault("vision", {}).update(settings)
        if workspace:
            cfg.setdefault("workspace", {}).update(workspace)
        save_config(cfg)
        self._publish()

    def detection_enabled(self) -> bool:
        with self._lock:
            return self._detection_enabled

    def set_detection_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        camera_running = self._thread is not None and self._thread.is_alive()
        with self._lock:
            self._detection_enabled = enabled
            if not enabled:
                self._latest = None
                self._latest_bundle = None
                self._bundle_history.clear()
        cfg = load_config()
        cfg.setdefault("vision", {})["detection_enabled"] = enabled
        save_config(cfg)
        if camera_running:
            self._status = "Running | Detection ON" if enabled else "Camera running | Detection OFF"
        else:
            self._status = "Stopped"
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
        with self._lock:
            self._bundle_history.clear()
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

    def recent_bundles(self) -> list[dict[str, Any]]:
        state_history = read_state("vision", {}).get("bundle_history")
        if isinstance(state_history, list):
            return [item for item in state_history if isinstance(item, dict)]
        with self._lock:
            return list(self._bundle_history)

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
        now = time.time()
        payload = {
            "status": "mock",
            "pick_point_px": [320, 240],
            "pick_point_base": [float(x_mm), float(y_mm)],
            "pick_point_world": [float(x_mm), float(y_mm)],
            "centroid_px": [320, 240],
            "centroid_world": [float(x_mm), float(y_mm)],
            "bbox_px": [250, 210, 140, 70],
            "width_mm": 80.0,
            "height_mm": 40.0,
            "orientation_deg": 0.0,
            "workspace_status": "valid",
            "workspace_message": "READY TO PICK",
            "timestamp": now,
        }
        bundle = {
            "selongsong_box": [250, 210, 390, 280],
            "fixture_box": [350, 190, 385, 300],
            "pick_point_px": [300, 245],
            "pick_point_base": [float(x_mm), float(y_mm)],
            "pick_point_world": [float(x_mm), float(y_mm)],
            "pick_safety": safety,
            "conf_selongsong": 1.0,
            "conf_fixture": 1.0,
            "captured_at": now,
            "coordinate_frame": "BASE",
            "mock": True,
        }
        with self._lock:
            self._latest = payload
            self._latest_bundle = bundle
            self._bundle_history.clear()
            for index in range(5):
                sample = dict(bundle)
                sample["captured_at"] = now - (4 - index) * 0.05
                self._bundle_history.append(sample)
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
            provider = str(getattr(self._model_detector, "execution_provider", "") or "").strip()
            self._model_status = f"LOADED ({provider})" if provider else "LOADED"
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

    def reset_intrinsic_calibration(self, delete_snapshots: bool = True) -> dict[str, Any]:
        manager = self._get_calibration_manager()
        summary = manager.reset_intrinsic_calibration(
            delete_snapshot_files=delete_snapshots,
        )
        with self._lock:
            self._ibvs_controller = None
            self._ibvs_active = False
        update_state("vision_ibvs", {"active": False, "status": "intrinsic_reset"})
        self._publish(calibration=self.calibration_summary())
        return summary

    def run_camera_to_base_calibration(
        self,
        pixel_points: list[list[float]],
        base_points_mm: list[list[float]],
    ) -> dict[str, Any]:
        with self._lock:
            image_size = self._frame_size
        if image_size[0] <= 0 or image_size[1] <= 0:
            state_size = read_state("vision", {}).get("frame_size", [0, 0])
            image_size = (int(state_size[0]), int(state_size[1]))
        if image_size[0] <= 0 or image_size[1] <= 0:
            raise RuntimeError("No active camera frame size is available")
        manager = self._get_calibration_manager()
        result = manager.run_camera_to_base_calibration(
            pixel_points,
            base_points_mm,
            image_size,
        )
        self._publish(calibration=self.calibration_summary())
        return result

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
            summary = self._get_calibration_manager().current_summary()
            camera_to_base = load_config().get("vision", {}).get("camera_to_base", {})
            summary["camera_to_base"] = camera_to_base if isinstance(camera_to_base, dict) else {}
            return summary
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
        if self.detection_enabled() and str(
            self.config().get("detection_method", "model")
        ).lower() == "model":
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
                if self.detection_enabled():
                    detection, bundle, display_frame = self._process_frame(frame)
                else:
                    detection, bundle, display_frame = None, None, frame
                detection_active = self.detection_enabled()
                visible_frame = (
                    display_frame
                    if detection_active and display_frame is not None
                    else frame
                )
                visible_frame = self._draw_chessboard_preview(frame, visible_frame)
                with self._lock:
                    self._raw_frame = frame.copy()
                    self._frame_size = (int(frame.shape[1]), int(frame.shape[0]))
                    self._display_frame = visible_frame.copy()
                    self._latest = detection if detection_active else None
                    self._latest_bundle = bundle if detection_active else None
                    if detection_active and isinstance(bundle, dict):
                        history_item = dict(bundle)
                        history_item["captured_at"] = time.time()
                        history_item["coordinate_frame"] = "BASE"
                        self._bundle_history.append(history_item)
                if detection_active:
                    self._run_ibvs_step(detection, bundle)
                elapsed_ms = (time.perf_counter() - started) * 1000.0
                now = time.monotonic()
                if now - self._last_vision_time_log_at >= 1.0:
                    research_logger.record("vision_time", round(elapsed_ms, 3))
                    self._last_vision_time_log_at = now
                if not detection_active:
                    self._status = "Camera running | Detection OFF"
                else:
                    self._status = "Running" if detection or bundle else "No object"
                self._publish(rate_limited=True)
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
                    return self._configure_camera_capture(cap)
                try:
                    cap.release()
                except Exception:
                    pass
            return self._configure_camera_capture(cv2.VideoCapture(index))
        return cv2.VideoCapture(source)

    def _configure_camera_capture(self, cap: Any) -> Any:
        if cap is None or not cap.isOpened():
            return cap
        import cv2

        cfg = self.config()
        width = max(1, int(cfg.get("camera_width", 1280)))
        height = max(1, int(cfg.get("camera_height", 720)))
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        except Exception:
            pass
        return cap

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
        distortion = calibration.distortion_for_frame()
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
                    bundles = self._model_detector.detect_bundles(
                        frame,
                        intrinsic,
                        z_mm,
                        distortion,
                    )
                    if bundles:
                        bundle = bundles[0]
                        bundle_payload = bundle.as_dict()
                        overlay = self._model_detector.draw_bundle_overlay(overlay, bundle)
                        result, mask = self._model_detector.detect(
                            frame,
                            intrinsic,
                            z_mm,
                            distortion,
                            bundles,
                        )
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

    def _draw_chessboard_preview(self, frame: Any, display_frame: Any) -> Any:
        import cv2

        calibration_cfg = self.config().get("calibration", {})
        calibration_cfg = calibration_cfg if isinstance(calibration_cfg, dict) else {}
        if not bool(calibration_cfg.get("preview_enabled", False)):
            self._chessboard_found = False
            self._chessboard_corner_count = 0
            self._chessboard_corners = None
            return display_frame

        manager = self._get_calibration_manager()
        try:
            pattern_size = manager.normalize_chessboard_size(
                calibration_cfg.get("chessboard_size", (9, 6))
            )
        except ValueError:
            self._chessboard_found = False
            self._chessboard_corner_count = 0
            self._chessboard_pattern = (0, 0)
            self._chessboard_corners = None
            overlay = display_frame.copy()
            cv2.rectangle(
                overlay,
                (8, 8),
                (min(420, overlay.shape[1] - 8), 40),
                (0, 0, 0),
                -1,
            )
            cv2.putText(
                overlay,
                "INTRINSIC: INVALID CHESSBOARD SIZE",
                (16, 31),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            return overlay
        now = time.monotonic()
        should_detect = (
            pattern_size != self._chessboard_pattern
            or now - self._chessboard_last_detection_at >= 0.2
        )
        if should_detect:
            detection_frame = frame
            scale_x = 1.0
            scale_y = 1.0
            height, width = frame.shape[:2]
            max_detection_side = 640
            if max(width, height) > max_detection_side:
                resize_scale = max_detection_side / float(max(width, height))
                resized_size = (
                    max(1, int(width * resize_scale)),
                    max(1, int(height * resize_scale)),
                )
                detection_frame = cv2.resize(frame, resized_size, interpolation=cv2.INTER_AREA)
                scale_x = width / float(detection_frame.shape[1])
                scale_y = height / float(detection_frame.shape[0])
            found, corners = manager.find_chessboard_corners(
                detection_frame,
                pattern_size,
                fast_check=True,
            )
            if corners is not None and (scale_x != 1.0 or scale_y != 1.0):
                corners = corners.copy()
                corners[:, :, 0] *= scale_x
                corners[:, :, 1] *= scale_y
            self._chessboard_pattern = pattern_size
            self._chessboard_found = bool(found)
            self._chessboard_corners = corners
            self._chessboard_corner_count = 0 if corners is None else int(len(corners))
            self._chessboard_last_detection_at = now

        overlay = manager.draw_chessboard_corners(
            display_frame,
            pattern_size,
            self._chessboard_corners,
            self._chessboard_found,
        )
        status = "FOUND" if self._chessboard_found else "NOT FOUND"
        color = (0, 210, 0) if self._chessboard_found else (0, 170, 255)
        text = (
            f"INTRINSIC {pattern_size[0]}x{pattern_size[1]}: {status} "
            f"({self._chessboard_corner_count} corners)"
        )
        cv2.rectangle(overlay, (8, 8), (min(520, overlay.shape[1] - 8), 40), (0, 0, 0), -1)
        cv2.putText(
            overlay,
            text,
            (16, 31),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            color,
            2,
            cv2.LINE_AA,
        )
        return overlay

    def _publish(
        self,
        calibration: dict[str, Any] | None = None,
        rate_limited: bool = False,
    ) -> None:
        now = time.monotonic()
        if rate_limited and now - self._last_publish_at < 0.2:
            return
        self._last_publish_at = now
        with self._lock:
            bundle_history = list(self._bundle_history)
            frame_size = [int(self._frame_size[0]), int(self._frame_size[1])]
            display_frame_size = (
                [int(self._display_frame.shape[1]), int(self._display_frame.shape[0])]
                if self._display_frame is not None
                else [0, 0]
            )
        model_shape = getattr(self._model_detector, "input_shape", (0, 0))
        model_input_size = [
            int(model_shape[1]),
            int(model_shape[0]),
        ] if len(model_shape) == 2 else [0, 0]
        model_provider = str(getattr(self._model_detector, "execution_provider", "") or "")
        update_state(
            "vision",
            {
                "status": self._status,
                "detection_enabled": self.detection_enabled(),
                "model_status": self._model_status,
                "model_provider": model_provider,
                "model_input_size": model_input_size,
                "latest": self._latest,
                "latest_bundle": self._latest_bundle,
                "bundle_history": bundle_history,
                "frame_size": frame_size,
                "display_frame_size": display_frame_size,
                "chessboard_found": self._chessboard_found,
                "chessboard_corner_count": self._chessboard_corner_count,
                "chessboard_pattern": list(self._chessboard_pattern),
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


def _wait_for_marginal_confirmation(
    timeout_s: float,
    shared_string: Any,
    program_log_queue: Any = None,
) -> bool:
    token = time.time()
    update_state("vision_confirmation", {"status": "pending", "token": token, "decision": "", "updated_at": time.time()})
    emit_program_log(
        shared_string,
        program_log_queue,
        "Log: vision() waiting for marginal confirmation",
    )
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


def camera_to_base_is_usable(camera_to_base: Any) -> bool:
    """Return True if a real 3x3 homography is present.

    The calibration is treated as usable based on the actual homography data,
    not the ``valid`` flag alone. Older configs may still carry ``valid: False``
    from the days when the GUI flipped it on every settings save; as long as a
    proper 3x3 numeric homography exists, the pixel->base mapping still works.
    Applicability to the active source/zoom/frame size is enforced separately
    by WorkspaceValidator.has_camera_to_base_calibration().
    """
    if not isinstance(camera_to_base, dict):
        return False
    if bool(camera_to_base.get("valid", False)):
        return True
    homography = camera_to_base.get("homography")
    if not isinstance(homography, list) or len(homography) != 3:
        return False
    for row in homography:
        if not isinstance(row, list) or len(row) != 3:
            return False
        for value in row:
            if not isinstance(value, (int, float)):
                return False
    return True


def execute_vision_tool_z(
    shared_string: Any,
    program_log_queue: Any = None,
) -> bool:
    import numpy as np
    import PAROL6_ROBOT
    from vision.tool_z_runtime import calculate_tool_z, stable_pick_from_history

    cfg = load_config()
    vision_cfg = cfg.get("vision", {})
    tool_cfg = vision_cfg.get("tool_z", {})
    if not bool(vision_cfg.get("detection_enabled", True)):
        reset_vision_runtime()
        emit_program_log(
            shared_string,
            program_log_queue,
            "Error: vision() Detection is OFF",
        )
        return False
    history = vision_manager.recent_bundles()
    samples_are_mock = any(bool(item.get("mock", False)) for item in history[-10:])
    camera_to_base = vision_cfg.get("camera_to_base", {})
    if not samples_are_mock and not camera_to_base_is_usable(camera_to_base):
        reset_vision_runtime()
        emit_program_log(
            shared_string,
            program_log_queue,
            "Error: vision() Camera-to-Base calibration is missing",
        )
        return False

    try:
        stable = stable_pick_from_history(
            history,
            sample_count=int(tool_cfg.get("sample_count", 5)),
            max_age_s=float(tool_cfg.get("detection_max_age_s", 1.0)),
            min_iou=float(tool_cfg.get("same_object_min_iou", 0.3)),
        )
    except ValueError as exc:
        reset_vision_runtime()
        emit_program_log(shared_string, program_log_queue, f"Error: vision() {exc}")
        research_logger.record("vision_pick_result", 0, str(exc))
        return False

    safety = str(stable["pick_safety"]).upper()
    raw_x_mm, raw_y_mm = [float(value) for value in stable["pick_point_base"]]
    adjusted_x = raw_x_mm + float(vision_cfg.get("offset_x_mm", 0.0))
    adjusted_y = raw_y_mm + float(vision_cfg.get("offset_y_mm", 0.0))
    workspace_status, workspace_message = _workspace_check(adjusted_x, adjusted_y)
    if workspace_status == "out":
        reset_vision_runtime()
        emit_program_log(
            shared_string,
            program_log_queue,
            f"Error: vision() {workspace_message}",
        )
        research_logger.record(
            "workspace_violation",
            1,
            f"x={adjusted_x:.3f},y={adjusted_y:.3f},safety={safety}",
        )
        return False
    if safety == "MARGINAL":
        confirmed = _wait_for_marginal_confirmation(
            float(vision_cfg.get("marginal_confirm_timeout_s", 2.0)),
            shared_string,
            program_log_queue,
        )
        if not confirmed:
            reset_vision_runtime()
            emit_program_log(
                shared_string,
                program_log_queue,
                "Error: vision() MARGINAL pick was not confirmed",
            )
            return False

    reference_joint_deg = list(
        tool_cfg.get(
            "reference_joint_deg",
            [90.0, -88.0, 182.259, 0.0, 3.0, 180.0],
        )
    )[:6]
    if len(reference_joint_deg) != 6:
        reset_vision_runtime()
        emit_program_log(
            shared_string,
            program_log_queue,
            "Error: vision() reference joint pose must contain 6 values",
        )
        return False
    try:
        reference_transform = PAROL6_ROBOT.robot.fkine(
            np.deg2rad(np.asarray(reference_joint_deg, dtype=np.float64))
        )
        result = calculate_tool_z(
            [raw_x_mm, raw_y_mm],
            np.asarray(reference_transform.t, dtype=np.float64) * 1000.0,
            np.asarray(reference_transform.R, dtype=np.float64),
            x_tool_fixed_mm=float(tool_cfg.get("x_tool_fixed_mm", -60.0)),
            y_tool_fixed_mm=float(tool_cfg.get("y_tool_fixed_mm", 0.0)),
            base_offset_x_mm=float(vision_cfg.get("offset_x_mm", 0.0)),
            base_offset_y_mm=float(vision_cfg.get("offset_y_mm", 0.0)),
            z_tool_offset_mm=float(vision_cfg.get("z_tool_offset_mm", 0.0)),
            z_plus_min_mm=float(tool_cfg.get("z_plus_min_mm", 0.0)),
            z_plus_max_mm=float(tool_cfg.get("z_plus_max_mm", 78.0)),
            retreat_margin_mm=float(tool_cfg.get("retreat_margin_mm", 30.0)),
            x_comp_base_mm=float(tool_cfg.get("x_comp_base_mm", -20.0)),
            x_comp_slope=float(tool_cfg.get("x_comp_slope", 0.0875)),
        )
    except Exception as exc:
        reset_vision_runtime()
        emit_program_log(
            shared_string,
            program_log_queue,
            f"Error: vision() Tool-Z calculation failed: {exc}",
        )
        research_logger.record("vision_tool_z_error", 1, str(exc))
        return False

    # Absolute grip target for MoveCart($vision.grip_x, ...): vision XY at a
    # fixed world height, pulled back along the approach direction so the
    # protruding fingers (not the TCP) meet the object — the clamped
    # MoveCartRelTRF path did this implicitly. Only published when within safe
    # reach — beyond it the near-singular arm violates joint/speed limits
    # before touching the object.
    grip_z_mm = float(tool_cfg.get("grip_z_mm", 238.5))
    grip_max_reach_mm = float(tool_cfg.get("grip_max_reach_mm", 425.0))
    grip_pullback_mm = float(tool_cfg.get("grip_pullback_mm", 12.0))
    approach_xy = np.asarray(reference_transform.R, dtype=np.float64)[:2, 2]
    approach_norm = float(np.linalg.norm(approach_xy))
    if approach_norm > 1e-9:
        approach_xy = approach_xy / approach_norm
    grip_x_mm = float(
        result.target_base_x_mm - grip_pullback_mm * approach_xy[0]
    )
    grip_y_mm = float(
        result.target_base_y_mm - grip_pullback_mm * approach_xy[1]
    )
    grip_reach_mm = float(np.linalg.norm([grip_x_mm, grip_y_mm, grip_z_mm]))
    grip_in_reach = grip_reach_mm <= grip_max_reach_mm

    now = time.time()
    runtime = {
        "valid": True,
        "status": safety,
        "pick_point_px": list(stable["pick_point_px"]),
        "pick_point_base_mm": [
            result.target_base_x_mm,
            result.target_base_y_mm,
        ],
        "z_raw_mm": result.z_raw_mm,
        "z_plus_mm": result.z_plus_mm,
        "z_minus_mm": result.z_minus_mm,
        "x_comp_mm": result.x_comp_mm,
        "residual_mm": result.residual_mm,
        "clamped": result.clamped,
        "sample_count": int(stable["sample_count"]),
        "reference_joint_deg": [float(value) for value in reference_joint_deg],
        "pose_tolerance_deg": float(tool_cfg.get("pose_tolerance_deg", 1.0)),
        "computed_at": now,
        "updated_at": now,
        "expires_at": now + float(tool_cfg.get("runtime_max_age_s", 30.0)),
    }
    if grip_in_reach:
        runtime["grip_x_mm"] = grip_x_mm
        runtime["grip_y_mm"] = grip_y_mm
        runtime["grip_z_mm"] = grip_z_mm
    set_vision_runtime(runtime)
    px_u, px_v = runtime["pick_point_px"]
    emit_program_log(
        shared_string,
        program_log_queue,
        (
            f"Log: Vision status={safety} px=({px_u},{px_v}) "
            f"base=({result.target_base_x_mm:.3f},{result.target_base_y_mm:.3f})"
        ),
    )
    emit_program_log(
        shared_string,
        program_log_queue,
        (
            f"Log: Zraw={result.z_raw_mm:.3f} Z+={result.z_plus_mm:.3f} "
            f"Z-={result.z_minus_mm:.3f} Xcomp={result.x_comp_mm:.3f} "
            f"residual={result.residual_mm:.3f} "
            f"clamp={'YES' if result.clamped else 'NO'}"
        ),
    )
    emit_program_log(
        shared_string,
        program_log_queue,
        (
            f"Log: Grip=({grip_x_mm:.3f},{grip_y_mm:.3f},{grip_z_mm:.3f}) "
            f"pullback={grip_pullback_mm:.1f} reach={grip_reach_mm:.1f} "
            + (
                "OK"
                if grip_in_reach
                else f"OUT-OF-REACH (max {grip_max_reach_mm:.0f}) grip disabled"
            )
        ),
    )
    research_logger.record(
        "vision_tool_z",
        1,
        (
            f"x={result.target_base_x_mm:.3f},y={result.target_base_y_mm:.3f},"
            f"zplus={result.z_plus_mm:.3f},zminus={result.z_minus_mm:.3f},"
            f"xcomp={result.x_comp_mm:.3f},"
            f"residual={result.residual_mm:.3f},safety={safety}"
        ),
    )
    return True


def build_vision_pick_sequence(
    shared_string: Any,
    program_log_queue: Any = None,
) -> list[str] | None:
    """Compatibility wrapper: vision now computes values and injects no motion."""
    return [] if execute_vision_tool_z(shared_string, program_log_queue) else None


def execute_vision_command(shared_string: Any, program_log_queue: Any = None) -> bool:
    return execute_vision_tool_z(shared_string, program_log_queue)


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
