from __future__ import annotations

from pathlib import Path

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QKeySequence, QPixmap, QShortcut
from PyQt6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QSizePolicy,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ui.tab_control import ControlTab
from ui.tab_modbus import ModbusTab
from ui.tab_research import ResearchTab
from ui.tab_vision import VisionTab

try:
    from PyQt6.QtSvgWidgets import QSvgWidget
except Exception:
    QSvgWidget = None


class MainWindow(QMainWindow):
    """Main application window with global top bar and four primary tabs."""

    estop_requested = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("PAROL6 Control Application v1.1")
        self.setMinimumSize(1400, 900)

        self.control_tab = ControlTab()
        self.vision_tab = VisionTab()
        self.modbus_tab = ModbusTab()
        self.research_tab = ResearchTab()

        self._status_led = QLabel("●")
        self._status_text = QLabel("DISCONNECTED")
        self._port_info = QLabel("COM --")
        self._mode_badge = QLabel("NORMAL")
        self._estop_button = None

        self._build_ui()
        self._wire_shortcuts()

    def set_connection_status(self, connected: bool, description: str = "") -> None:
        """Update the top bar connection indicator."""
        color = "#22C97A" if connected else "#3A4E6B"
        text = "CONNECTED" if connected else "DISCONNECTED"
        self._status_led.setStyleSheet(f"color: {color}; font-size: 16px;")
        self._status_text.setText(text)
        self._port_info.setText(description or ("COM --" if not connected else "COM ?"))

    def set_safety_state(self, state_name: str) -> None:
        """Reflect the active failsafe state in the top bar and control tab."""
        state_map = {
            "normal": ("NORMAL", "#22C97A"),
            "estop": ("E-STOP", "#FF0000"),
            "workspace_error": ("WORKSPACE", "#E84040"),
            "ik_error": ("IK ERROR", "#E84040"),
            "serial_error": ("SERIAL", "#E84040"),
            "modbus_warning": ("MODBUS", "#F5A623"),
        }
        label, color = state_map.get(state_name, (state_name.upper(), "#4A90D9"))
        self._mode_badge.setText(label)
        self._mode_badge.setStyleSheet(
            "background-color: #162B52; border: 1px solid #243D6B; "
            f"border-radius: 4px; padding: 6px 10px; color: {color}; font-weight: 700;"
        )
        self.control_tab.set_safety_state(label, color)

    def append_log(self, message: str, level: str = "info") -> None:
        """Append one log line to the control tab response log."""
        self.control_tab.append_log(message, level)

    def _build_ui(self) -> None:
        container = QWidget()
        root_layout = QVBoxLayout(container)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)
        root_layout.addWidget(self._build_top_bar())
        root_layout.addWidget(self._build_tabs())
        self.setCentralWidget(container)
        self.set_connection_status(False)
        self.set_safety_state("normal")

    def _build_top_bar(self) -> QWidget:
        top_bar = QFrame()
        top_bar.setObjectName("topBar")
        top_bar.setStyleSheet(
            "QFrame#topBar {background-color: #0D1B35; border-bottom: 1px solid #243D6B;}"
        )
        layout = QHBoxLayout(top_bar)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(12)

        logo_section = QWidget()
        logo_layout = QHBoxLayout(logo_section)
        logo_layout.setContentsMargins(0, 0, 0, 0)
        logo_layout.setSpacing(10)
        logo_section.setMinimumWidth(220)

        logo_widget = self._build_logo_widget()
        logo_layout.addWidget(logo_widget)

        title = QLabel("PAROL6 v1.1")
        title.setStyleSheet("color: #E8EDF5; font-size: 13px; font-weight: 700;")
        subtitle = QLabel("Polman Bandung")
        subtitle.setStyleSheet("color: #8A9AB8; font-size: 10px;")

        title_column = QWidget()
        title_layout = QVBoxLayout(title_column)
        title_layout.setContentsMargins(0, 0, 0, 0)
        title_layout.setSpacing(0)
        title_layout.addWidget(title)
        title_layout.addWidget(subtitle)
        logo_layout.addWidget(title_column)
        layout.addWidget(logo_section)

        divider_left = self._build_divider()
        layout.addWidget(divider_left)

        status_section = QWidget()
        status_layout = QHBoxLayout(status_section)
        status_layout.setContentsMargins(0, 0, 0, 0)
        status_layout.setSpacing(8)
        self._status_led.setStyleSheet("color: #3A4E6B; font-size: 16px;")
        self._status_text.setStyleSheet("color: #E8EDF5; font-weight: 600;")
        self._port_info.setStyleSheet("color: #8A9AB8;")
        status_layout.addWidget(self._status_led)
        status_layout.addWidget(self._status_text)
        status_layout.addWidget(self._port_info)
        status_layout.addStretch(1)
        layout.addWidget(status_section, 1)

        layout.addWidget(self._build_divider())

        self._mode_badge.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        layout.addWidget(self._mode_badge)
        layout.addStretch(0)

        self._estop_button = self.control_tab.create_estop_button()
        self._estop_button.clicked.connect(lambda: self.estop_requested.emit("UI button"))
        layout.addWidget(self._estop_button)
        return top_bar

    def _build_logo_widget(self) -> QWidget:
        logo_path = Path(__file__).resolve().parents[1] / "assets" / "icons" / "polman.png"
        if logo_path.exists():
            pixmap = QPixmap(str(logo_path))
            if not pixmap.isNull():
                label = QLabel()
                label.setPixmap(
                    pixmap.scaled(
                        32,
                        32,
                        Qt.AspectRatioMode.KeepAspectRatio,
                        Qt.TransformationMode.SmoothTransformation,
                    )
                )
                label.setFixedSize(32, 32)
                return label
        fallback = QLabel("⬡⬡")
        fallback.setStyleSheet("color: #3B82F6; font-size: 18px; font-weight: 700;")
        fallback.setFixedWidth(32)
        return fallback

    def _build_tabs(self) -> QWidget:
        self.tabs = QTabWidget()
        self.tabs.addTab(self.control_tab, "Control")
        self.tabs.addTab(self.vision_tab, "Vision")
        self.tabs.addTab(self.modbus_tab, "Modbus")
        self.tabs.addTab(self.research_tab, "Research")
        return self.tabs

    def _wire_shortcuts(self) -> None:
        space_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Space), self)
        space_shortcut.activated.connect(lambda: self.estop_requested.emit("Keyboard Space"))
        f1_shortcut = QShortcut(QKeySequence(Qt.Key.Key_F1), self)
        f1_shortcut.activated.connect(lambda: self.estop_requested.emit("Keyboard F1"))
        self._space_shortcut = space_shortcut
        self._f1_shortcut = f1_shortcut

    @staticmethod
    def _build_divider() -> QWidget:
        divider = QFrame()
        divider.setFrameShape(QFrame.Shape.VLine)
        divider.setStyleSheet("color: #243D6B; background-color: #243D6B;")
        divider.setFixedWidth(1)
        return divider
