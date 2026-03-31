from __future__ import annotations

import time
from collections import deque
from pathlib import Path

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtWidgets import (
    QFileDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QSizePolicy,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

import pyqtgraph as pg

from backend.research_logger import ResearchLogger


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
_MAX_PLOT_POINTS = 300  # rolling window for live plots
_REFRESH_INTERVAL_MS = 500  # UI refresh timer


def _kpi_color(metric: str) -> str:
    """Return accent color per metric for the KPI card."""
    palette = {
        "serial_latency": "#4A90D9",
        "modbus_cycle": "#22C97A",
        "vision_time": "#3B82F6",
        "pick_success_rate": "#E84040",
        "cycle_time": "#A76FDB",
    }
    return palette.get(metric, "#8A9AB8")


_METRIC_UNITS = {
    "serial_latency": "ms",
    "modbus_cycle": "ms",
    "vision_time": "ms",
    "pick_success_rate": "%",
    "cycle_time": "s",
}

_METRIC_LABELS = {
    "serial_latency": "Serial Latency",
    "modbus_cycle": "Modbus Cycle",
    "vision_time": "Vision Time",
    "pick_success_rate": "Pick Success",
    "cycle_time": "Cycle Time",
}


# ---------------------------------------------------------------------------
# KPI Card Widget
# ---------------------------------------------------------------------------
class _KPICard(QFrame):
    """Single KPI card displaying label, value, and unit."""

    def __init__(self, key: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("kpiCard")
        self._key = key
        accent = _kpi_color(key)
        self.setStyleSheet(
            f"QFrame#kpiCard {{ background: #0D1B35; border: 1px solid {accent};"
            f" border-radius: 6px; padding: 8px; }}"
        )
        self.setMinimumWidth(140)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(2)

        title = QLabel(_METRIC_LABELS.get(key, key))
        title.setStyleSheet(f"color: {accent}; font-size: 10px; font-weight: 600;")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        self._value_label = QLabel("--")
        self._value_label.setStyleSheet("color: #E8EDF5; font-size: 22px; font-weight: 700;")
        self._value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self._value_label)

        unit = QLabel(_METRIC_UNITS.get(key, ""))
        unit.setStyleSheet("color: #8A9AB8; font-size: 9px;")
        unit.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(unit)

    def set_value(self, value: float | int | str) -> None:
        if isinstance(value, float):
            self._value_label.setText(f"{value:.1f}")
        else:
            self._value_label.setText(str(value))


# ---------------------------------------------------------------------------
# ResearchTab
# ---------------------------------------------------------------------------
class ResearchTab(QWidget):
    """Tab 4 — Research metrics dashboard with live plots and failsafe event log."""

    export_requested = pyqtSignal()

    # Keys for KPI cards (order matters for layout)
    _KPI_KEYS = ["serial_latency", "modbus_cycle", "vision_time", "pick_success_rate", "cycle_time"]

    # Keys for plot curves
    _PLOT_KEYS = ["serial_latency", "modbus_cycle", "vision_time", "cycle_time"]

    def __init__(self) -> None:
        super().__init__()
        self._recording = False
        self._record_start: float = 0.0
        self._total_picks = 0
        self._success_picks = 0

        # Per-metric rolling buffers for live plots
        self._buffers: dict[str, deque[float]] = {k: deque(maxlen=_MAX_PLOT_POINTS) for k in self._PLOT_KEYS}

        self._kpi_cards: dict[str, _KPICard] = {}
        self._curves: dict[str, pg.PlotDataItem] = {}
        self._plots: dict[str, pg.PlotWidget] = {}

        self._build_ui()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(_REFRESH_INTERVAL_MS)

    # ----- public API (called from main.py wiring) -----

    def feed_metric(self, metric_name: str, value: float) -> None:
        """Push a new metric data point from any runtime signal."""
        if metric_name in self._buffers:
            self._buffers[metric_name].append(value)
        if metric_name in self._kpi_cards:
            self._kpi_cards[metric_name].set_value(value)

    def feed_pick_result(self, success: bool) -> None:
        """Track pick success/failure for the success rate KPI."""
        self._total_picks += 1
        if success:
            self._success_picks += 1
        rate = (self._success_picks / self._total_picks * 100) if self._total_picks else 0
        self.feed_metric("pick_success_rate", round(rate, 1))

    def feed_cycle_time(self, elapsed_s: float) -> None:
        """Push cycle time data point."""
        self.feed_metric("cycle_time", round(elapsed_s, 2))

    def append_event(self, message: str, level: str = "info") -> None:
        """Add one entry to the failsafe event log."""
        color_map = {
            "estop": "#FF0000",
            "error": "#E84040",
            "warn": "#F5A623",
            "info": "#8A9AB8",
            "marginal": "#F5A623",
        }
        color = color_map.get(level, "#8A9AB8")
        ts = time.strftime("%H:%M:%S")
        text = f"[{ts}] {message}"
        item = QListWidgetItem(text)
        item.setForeground(Qt.GlobalColor.white)
        item.setToolTip(text)
        self._event_list.addItem(item)
        # Color the text
        item.setForeground(pg.mkColor(color))
        self._event_list.scrollToBottom()
        # Cap list at 500 entries
        while self._event_list.count() > 500:
            self._event_list.takeItem(0)

    def is_recording(self) -> bool:
        return self._recording

    # ----- UI build -----

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        # -- Top row: recording controls + KPI cards --
        top_row = QWidget()
        top_layout = QHBoxLayout(top_row)
        top_layout.setContentsMargins(0, 0, 0, 0)
        top_layout.setSpacing(8)
        top_layout.addWidget(self._build_recording_controls())
        top_layout.addWidget(self._build_kpi_dashboard(), 1)
        root.addWidget(top_row)

        # -- Bottom area: plots (left) + event log (right) --
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self._build_plots_panel())
        splitter.addWidget(self._build_event_log_panel())
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        root.addWidget(splitter, 1)

    def _build_recording_controls(self) -> QGroupBox:
        group = QGroupBox("Recording")
        layout = QVBoxLayout(group)
        layout.setSpacing(6)

        self._rec_status_label = QLabel("● IDLE")
        self._rec_status_label.setStyleSheet("color: #8A9AB8; font-weight: 700; font-size: 12px;")
        self._rec_status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self._rec_status_label)

        self._elapsed_label = QLabel("00:00")
        self._elapsed_label.setStyleSheet("color: #E8EDF5; font-size: 18px; font-weight: 700;")
        self._elapsed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self._elapsed_label)

        self.start_button = QPushButton("▶  Start")
        self.start_button.setProperty("role", "primary")
        self.start_button.clicked.connect(self._on_start)
        layout.addWidget(self.start_button)

        self.stop_button = QPushButton("■  Stop")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self._on_stop)
        layout.addWidget(self.stop_button)

        self.reset_button = QPushButton("⟲  Reset")
        self.reset_button.clicked.connect(self._on_reset)
        layout.addWidget(self.reset_button)

        self.export_button = QPushButton("Export CSV…")
        self.export_button.clicked.connect(self._on_export)
        layout.addWidget(self.export_button)

        layout.addStretch(1)
        group.setFixedWidth(160)
        return group

    def _build_kpi_dashboard(self) -> QGroupBox:
        group = QGroupBox("KPI Dashboard")
        layout = QHBoxLayout(group)
        layout.setSpacing(8)
        for key in self._KPI_KEYS:
            card = _KPICard(key)
            self._kpi_cards[key] = card
            layout.addWidget(card)
        return group

    def _build_plots_panel(self) -> QWidget:
        container = QGroupBox("Live Plots")
        layout = QGridLayout(container)
        layout.setSpacing(6)

        pg.setConfigOptions(antialias=True, background="#0D1B35", foreground="#8A9AB8")

        positions = {
            "serial_latency": (0, 0),
            "modbus_cycle": (0, 1),
            "vision_time": (1, 0),
            "cycle_time": (1, 1),
        }
        titles = {
            "serial_latency": "Serial Latency (ms)",
            "modbus_cycle": "Modbus Cycle (ms)",
            "vision_time": "Vision Time (ms)",
            "cycle_time": "Cycle Time (s)",
        }
        for key, (row, col) in positions.items():
            pw = pg.PlotWidget(title=titles[key])
            pw.showGrid(x=True, y=True, alpha=0.15)
            pw.setLabel("bottom", "sample")
            pw.setMinimumHeight(140)
            color = _kpi_color(key)
            curve = pw.plot(pen=pg.mkPen(color, width=2))
            self._curves[key] = curve
            self._plots[key] = pw
            layout.addWidget(pw, row, col)

        return container

    def _build_event_log_panel(self) -> QGroupBox:
        group = QGroupBox("Failsafe Event Log")
        layout = QVBoxLayout(group)
        self._event_list = QListWidget()
        self._event_list.setStyleSheet(
            "QListWidget { background: #070E1C; color: #E8EDF5; border: none; font-size: 11px; }"
        )
        self._event_list.setWordWrap(True)
        layout.addWidget(self._event_list)

        self._clear_events_btn = QPushButton("Clear Log")
        self._clear_events_btn.clicked.connect(self._event_list.clear)
        layout.addWidget(self._clear_events_btn)
        return group

    # ----- recording slots -----

    def _on_start(self) -> None:
        self._recording = True
        self._record_start = time.monotonic()
        self._rec_status_label.setText("● RECORDING")
        self._rec_status_label.setStyleSheet("color: #FF0000; font-weight: 700; font-size: 12px;")
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.append_event("Recording started", "info")

    def _on_stop(self) -> None:
        self._recording = False
        self._rec_status_label.setText("● STOPPED")
        self._rec_status_label.setStyleSheet("color: #F5A623; font-weight: 700; font-size: 12px;")
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.append_event("Recording stopped", "info")

    def _on_reset(self) -> None:
        self._recording = False
        self._record_start = 0.0
        self._total_picks = 0
        self._success_picks = 0
        for buf in self._buffers.values():
            buf.clear()
        for card in self._kpi_cards.values():
            card.set_value("--")
        self._elapsed_label.setText("00:00")
        self._rec_status_label.setText("● IDLE")
        self._rec_status_label.setStyleSheet("color: #8A9AB8; font-weight: 700; font-size: 12px;")
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.append_event("Dashboard reset", "info")

    def _on_export(self) -> None:
        logger = ResearchLogger.instance()
        default_path = str(logger.session_path)
        path, _ = QFileDialog.getSaveFileName(self, "Export CSV", default_path, "CSV Files (*.csv)")
        if path:
            exported = logger.export_csv(path)
            self.append_event(f"Exported CSV to {exported}", "info")

    # ----- periodic refresh -----

    def _refresh(self) -> None:
        """Timer callback — update elapsed time and redraw plot curves."""
        if self._recording and self._record_start:
            elapsed = time.monotonic() - self._record_start
            mins, secs = divmod(int(elapsed), 60)
            self._elapsed_label.setText(f"{mins:02d}:{secs:02d}")

        for key, curve in self._curves.items():
            buf = self._buffers[key]
            if buf:
                curve.setData(list(buf))
