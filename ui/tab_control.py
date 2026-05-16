from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path

from PyQt6.QtCore import Qt, QUrl, QTimer, QRegularExpression
from PyQt6.QtGui import QColor, QFont, QSyntaxHighlighter, QTextCharFormat, QTextCursor
from PyQt6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QRadioButton,
    QScrollArea,
    QSlider,
    QSplitter,
    QSizePolicy,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QToolButton,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)
from program.program_model import ProgramCommand, ProgramModel
from backend.config_manager import ConfigManager

try:
    from backend.simulator_bridge import RobotCanvas
except Exception:
    RobotCanvas = None


COMMAND_LIBRARY: dict[str, dict[str, str]] = {
    "Begin": {
        "parameters": "-",
        "notes": "Program start",
        "hint": "Penanda awal program. Harus ada di baris pertama.",
        "template": "Begin()",
    },
    "MoveJoint": {
        "parameters": "j1,j2,j3,j4,j5,j6,t=4",
        "notes": "Move all joints in deg",
        "hint": "6 joint angles (deg). Opsional: v=%, a=%, t=detik, trap/poly.",
        "template": "MoveJoint({j1},{j2},{j3},{j4},{j5},{j6},t=4)",
    },
    "MovePose": {
        "parameters": "x,y,z,rx,ry,rz,t=4",
        "notes": "IK move via cartesian pose",
        "hint": "x,y,z (mm), rx,ry,rz (deg). Gerak di joint-space via IK.",
        "template": "MovePose({x},{y},{z},{rx},{ry},{rz},t=4)",
    },
    "MoveCart": {
        "parameters": "x,y,z,rx,ry,rz,t=4",
        "notes": "Cartesian linear move",
        "hint": "x,y,z (mm), rx,ry,rz (deg). Gerak linear di cartesian.",
        "template": "MoveCart({x},{y},{z},{rx},{ry},{rz},t=4)",
    },
    "MoveCartRelTRF": {
        "parameters": "dx,dy,dz,dr,dp,dyaw,t=4",
        "notes": "Move relative to tool frame",
        "hint": "dx,dy,dz (mm), dr,dp,dyaw (deg). Relatif terhadap tool.",
        "template": "MoveCartRelTRF(0,0,0,0,0,0,t=4)",
    },
    "SpeedJoint": {
        "parameters": "v=50, a=30",
        "notes": "Set joint speed/accel %",
        "hint": "v=kecepatan %, a=akselerasi %. Berlaku untuk command berikutnya.",
        "template": "SpeedJoint(v=50,a=30)",
    },
    "Home": {
        "parameters": "-",
        "notes": "Move robot to home",
        "hint": "Kirim robot ke posisi home.",
        "template": "Home()",
    },
    "Delay": {
        "parameters": "seconds",
        "notes": "Pause execution",
        "hint": "Delay dalam detik (contoh: Delay(1.5)).",
        "template": "Delay(1)",
    },
    "Loop": {
        "parameters": "-",
        "notes": "Loop back to Begin",
        "hint": "Kembali ke Begin() (infinite loop). Taruh di akhir program.",
        "template": "Loop()",
    },
    "End": {
        "parameters": "-",
        "notes": "Program end",
        "hint": "Penanda akhir program. Program berhenti di sini.",
        "template": "End()",
    },
    "Input": {
        "parameters": "pin, value",
        "notes": "Wait for digital input",
        "hint": "Menunggu input pin bernilai HIGH/LOW.",
        "template": "Input(1,HIGH)",
    },
    "Output": {
        "parameters": "pin, state",
        "notes": "Set digital output",
        "hint": "Set output pin ke HIGH atau LOW.",
        "template": "Output(1,HIGH)",
    },
    "Gripper": {
        "parameters": "position, speed, force",
        "notes": "Control gripper",
        "hint": "position 0-255, speed 0-255, force 100-1000.",
        "template": "Gripper(255,100,120)",
    },
    "Gripper_cal": {
        "parameters": "-",
        "notes": "Calibrate gripper",
        "hint": "Menjalankan kalibrasi gripper.",
        "template": "Gripper_cal()",
    },
    "vision": {
        "parameters": "-",
        "notes": "Vision-guided pick",
        "hint": "Menjalankan pick berbasis vision dengan setting aktif.",
        "template": "vision()",
    },
    "Dummy": {
        "parameters": "-",
        "notes": "No-op / testing",
        "hint": "Perintah dummy untuk testing. Tidak melakukan apa-apa.",
        "template": "Dummy()",
    },
}

COMMAND_TREE_STRUCTURE: list[tuple[str, list[str]]] = [
    ("Joint Space", ["MoveJoint", "MovePose", "SpeedJoint"]),
    ("Cartesian Space", ["MoveCart", "MoveCartRelTRF"]),
    ("Program Flow", ["Begin", "End", "Loop", "Delay"]),
    ("I/O", ["Input", "Output"]),
    ("Gripper", ["Gripper", "Gripper_cal"]),
    ("Vision", ["vision"]),
    ("Misc", ["Home", "Dummy"]),
]


class ProgramSyntaxHighlighter(QSyntaxHighlighter):
    """Syntax highlighter for PAROL6 program text editor."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._rules: list[tuple[QRegularExpression, QTextCharFormat]] = []

        # Keywords (command names) → green
        keyword_format = QTextCharFormat()
        keyword_format.setForeground(QColor("#4ADE80"))
        keyword_format.setFontWeight(QFont.Weight.Bold)
        keywords = list(COMMAND_LIBRARY.keys())
        for word in keywords:
            pattern = QRegularExpression(rf"\b{word}\b")
            self._rules.append((pattern, keyword_format))

        # Numbers → cyan
        number_format = QTextCharFormat()
        number_format.setForeground(QColor("#38BDF8"))
        self._rules.append((QRegularExpression(r"-?\b\d+(?:\.\d+)?\b"), number_format))

        # Parentheses → yellow
        paren_format = QTextCharFormat()
        paren_format.setForeground(QColor("#FBBF24"))
        self._rules.append((QRegularExpression(r"[()]"), paren_format))

        # Named params (key=) → light purple
        param_format = QTextCharFormat()
        param_format.setForeground(QColor("#A78BFA"))
        self._rules.append((QRegularExpression(r"\b\w+=(?=[^=])"), param_format))

    def highlightBlock(self, text: str) -> None:
        for pattern, fmt in self._rules:
            iterator = pattern.globalMatch(text)
            while iterator.hasNext():
                match = iterator.next()
                self.setFormat(match.capturedStart(), match.capturedLength(), fmt)


def _render_parameter_text(args: list[object], kwargs: dict[str, object]) -> str:
    parts = [ProgramCommand._render_value(value) for value in args]
    parts.extend(
        f"{key}={ProgramCommand._render_value(value)}" for key, value in kwargs.items()
    )
    return ", ".join(parts) if parts else "-"


def _normalize_command_row(command: str, parameters: str) -> tuple[str, str]:
    command_text = command.strip()
    parameters_text = (parameters or "-").strip() or "-"
    if not command_text:
        return "", parameters_text
    if "(" in command_text and command_text.endswith(")"):
        try:
            parsed = ProgramModel.parse_row(
                {"command": command_text, "parameters": "-", "notes": ""}
            )
        except Exception:
            return command_text, parameters_text
        return parsed.name, _render_parameter_text(parsed.args, parsed.kwargs)
    return command_text, parameters_text


class CollapsibleSection(QWidget):
    def __init__(self, title: str, content: QWidget, expanded: bool = True) -> None:
        super().__init__()
        self._content = content
        self._header = QToolButton()
        self._header.setProperty("role", "section-toggle")
        self._header.setText(title)
        self._header.setCheckable(True)
        self._header.setChecked(expanded)
        self._header.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self._header.setArrowType(
            Qt.ArrowType.DownArrow if expanded else Qt.ArrowType.RightArrow
        )
        self._header.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Fixed,
        )
        self._header.toggled.connect(self._set_expanded)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        layout.addWidget(self._header)
        layout.addWidget(self._content)

        self._content.setVisible(expanded)

    def _set_expanded(self, expanded: bool) -> None:
        self._header.setArrowType(
            Qt.ArrowType.DownArrow if expanded else Qt.ArrowType.RightArrow
        )
        self._content.setVisible(expanded)


class CommandDialog(QDialog):
    """Simple dialog for creating or editing one program command row."""

    def __init__(
        self,
        command: str = "",
        parameters: str = "",
        notes: str = "",
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Edit Command")
        self.setMinimumWidth(600)

        normalized_command, normalized_parameters = _normalize_command_row(command, parameters)
        self._last_default_parameters = ""
        self._last_default_notes = ""

        # -- Command list panel (left side) --
        self._command_list = QTableWidget(len(COMMAND_LIBRARY), 2)
        self._command_list.setHorizontalHeaderLabels(["Command", "Description"])
        self._command_list.verticalHeader().setVisible(False)
        self._command_list.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._command_list.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self._command_list.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._command_list.setMinimumWidth(240)
        self._command_list.setMaximumWidth(280)
        for row_idx, (cmd_name, spec) in enumerate(COMMAND_LIBRARY.items()):
            self._command_list.setItem(row_idx, 0, QTableWidgetItem(cmd_name))
            self._command_list.setItem(row_idx, 1, QTableWidgetItem(spec["notes"]))
        self._command_list.resizeColumnsToContents()
        self._command_list.cellClicked.connect(self._on_command_list_clicked)

        # -- Form panel (right side) --
        self.command_edit = QComboBox()
        self.command_edit.setEditable(True)
        self.command_edit.addItems(COMMAND_LIBRARY.keys())
        self.command_edit.setCurrentText(normalized_command or next(iter(COMMAND_LIBRARY)))
        self.parameters_edit = QLineEdit()
        self.parameters_hint = QLabel()
        self.parameters_hint.setProperty("role", "fieldHint")
        self.parameters_hint.setWordWrap(True)
        self.notes_edit = QLineEdit(notes)

        parameters_panel = QWidget()
        parameters_layout = QVBoxLayout(parameters_panel)
        parameters_layout.setContentsMargins(0, 0, 0, 0)
        parameters_layout.setSpacing(6)
        parameters_layout.addWidget(self.parameters_edit)
        parameters_layout.addWidget(self.parameters_hint)

        self.command_edit.currentTextChanged.connect(self._on_command_changed)
        self._apply_command_defaults(
            self.command_edit.currentText(),
            force=normalized_parameters in {"", "-"},
        )
        if normalized_parameters not in {"", "-"}:
            self.parameters_edit.setText(normalized_parameters)
        if notes:
            self.notes_edit.setText(notes)

        form_widget = QWidget()
        form_layout = QVBoxLayout(form_widget)
        form_layout.setContentsMargins(0, 0, 0, 0)
        form = QFormLayout()
        form.addRow("Command", self.command_edit)
        form.addRow("Parameters", parameters_panel)
        form.addRow("Notes", self.notes_edit)
        form_layout.addLayout(form)
        form_layout.addStretch(1)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        form_layout.addWidget(buttons)

        # -- Main layout: list | form --
        layout = QHBoxLayout(self)
        layout.addWidget(self._command_list)
        layout.addWidget(form_widget, 1)

        # Highlight current command in the list
        self._select_command_in_list(self.command_edit.currentText())

    def _on_command_list_clicked(self, row: int, _column: int) -> None:
        item = self._command_list.item(row, 0)
        if item:
            self.command_edit.setCurrentText(item.text())

    def _select_command_in_list(self, command_name: str) -> None:
        for row_idx in range(self._command_list.rowCount()):
            item = self._command_list.item(row_idx, 0)
            if item and item.text() == command_name.strip():
                self._command_list.selectRow(row_idx)
                return

    def _on_command_changed(self, command_name: str) -> None:
        self._apply_command_defaults(command_name, force=False)
        self._select_command_in_list(command_name)

    def _apply_command_defaults(self, command_name: str, force: bool) -> None:
        spec = COMMAND_LIBRARY.get(command_name.strip())
        if spec is None:
            self.parameters_hint.setText(
                "Gunakan format argumen Python sederhana, mis. x=0, y=0 atau j1=10."
            )
            self._last_default_parameters = ""
            self._last_default_notes = ""
            return

        previous_parameters = self._last_default_parameters
        previous_notes = self._last_default_notes
        current_parameters = self.parameters_edit.text().strip()
        current_notes = self.notes_edit.text().strip()
        default_parameters = "" if spec["parameters"] == "-" else spec["parameters"]
        default_notes = spec["notes"]

        if force or not current_parameters or current_parameters == previous_parameters:
            self.parameters_edit.setText(default_parameters)
        if force or not current_notes or current_notes == previous_notes:
            self.notes_edit.setText(default_notes)

        self.parameters_hint.setText(spec["hint"])
        self._last_default_parameters = default_parameters
        self._last_default_notes = default_notes

    def values(self) -> tuple[str, str, str]:
        """Return dialog field values as command, parameters, notes."""
        return (
            self.command_edit.currentText().strip(),
            self.parameters_edit.text().strip() or "-",
            self.notes_edit.text().strip(),
        )


class ControlTab(QWidget):
    """Main control tab with robot controls, program editor, and simulator view."""

    def __init__(self) -> None:
        super().__init__()
        self.response_log = QTextEdit()
        self.safety_label = QLabel("NORMAL")
        self.program_editor = QPlainTextEdit()
        self.program_table = QTableWidget(0, 4)  # kept for backward compat
        self.generated_script = QTextEdit()  # kept for backward compat
        self.current_line_value = QLabel("-")
        self.simulation_view: RobotCanvas | None = None
        self.simulation_view_rt: RobotCanvas | None = None
        self.simulation_content: QWidget | None = None
        self.sim_tab_widget: QTabWidget | None = None
        self.port_combo: QComboBox | None = None
        self.baud_combo: QComboBox | None = None
        self.connect_button: QPushButton | None = None
        self.disconnect_button: QPushButton | None = None
        self.enable_button: QPushButton | None = None
        self.disable_button: QPushButton | None = None
        self.home_button: QPushButton | None = None
        self.calibrate_button: QPushButton | None = None
        self.clear_error_button: QPushButton | None = None
        self.speed_slider: QSlider | None = None
        self.frame_combo: QComboBox | None = None
        self.joint_minus_buttons: list[QPushButton] = []
        self.joint_plus_buttons: list[QPushButton] = []
        self.joint_sliders: list[QSlider] = []
        self.joint_slider_value_labels: list[QLabel] = []
        self.cart_minus_buttons: dict[str, QPushButton] = {}
        self.cart_plus_buttons: dict[str, QPushButton] = {}
        self.cart_value_labels: dict[str, QLabel] = {}
        self.jog_source_label = QLabel("Jog source: NONE")
        self.input_state_labels: list[QLabel] = []
        self.output_state_labels: list[QLabel] = []
        self.output_low_buttons: list[QPushButton] = []
        self.output_high_buttons: list[QPushButton] = []
        self.simulation_status_label = QLabel("Joint state: --")
        self.preview_joint_labels: list[QLabel] = []
        self.realtime_joint_labels: list[QLabel] = []
        self.command_tree: QTreeWidget | None = None
        self.command_hint_label: QLabel | None = None
        self.commands_toggle_button: QPushButton | None = None
        self.command_palette_group: QGroupBox | None = None
        self._teaching_mode = "current"  # "current" or "custom"
        self._current_file_path: str = ""  # path of currently open .txt
        self._highlighter: ProgramSyntaxHighlighter | None = None
        self._highlight_extra_selection = None

        # --- Auto-repeat jog timer ---
        self._jog_repeat_timer = QTimer(self)
        self._jog_repeat_timer.setInterval(100)  # ms between repeats
        self._jog_repeat_callback = None
        self._jog_repeat_timer.timeout.connect(self._fire_jog_repeat)

        self._build_ui()

    def create_estop_button(self) -> QPushButton:
        """Create the shared E-Stop button instance used in the global top bar."""
        button = QPushButton("⬛ E-STOP")
        button.setProperty("role", "estop")
        return button

    def set_safety_state(self, label: str, color: str) -> None:
        """Update the safety state summary shown in the control tab."""
        self.safety_label.setText(label)
        self.safety_label.setStyleSheet(f"color: {color}; font-weight: 700;")

    def append_log(self, message: str, level: str = "info") -> None:
        """Append one formatted line to the response log."""
        colors = {
            "info": "#4ADE80",
            "warn": "#F5A623",
            "error": "#E84040",
            "estop": "#FF0000",
        }
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        color = colors.get(level, "#E8EDF5")
        self.response_log.append(
            f'<span style="color:#8A9AB8;">[{timestamp}]</span> '
            f'<span style="color:{color};">{message}</span>'
        )

    def _build_ui(self) -> None:
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        splitter = QSplitter()
        splitter.addWidget(self._build_left_panel())
        splitter.addWidget(self._build_center_panel())
        splitter.addWidget(self._build_right_panel())
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setStretchFactor(2, 2)
        splitter.setSizes([300, 500, 700])
        splitter.setCollapsible(0, False)
        splitter.setCollapsible(1, False)
        splitter.setCollapsible(2, False)
        layout.addWidget(splitter)

    def _build_left_panel(self) -> QWidget:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        layout.addWidget(CollapsibleSection("CONNECTION", self._build_connection_group()))
        layout.addWidget(CollapsibleSection("ROBOT ACTIONS", self._build_robot_actions_group()))
        layout.addWidget(CollapsibleSection("JOINT JOG", self._build_joint_jog_group()))
        layout.addWidget(CollapsibleSection("CARTESIAN JOG", self._build_cartesian_group()))
        layout.addWidget(CollapsibleSection("DIGITAL I/O", self._build_io_group()))
        layout.addWidget(CollapsibleSection("FAILSAFE", self._build_failsafe_group()))
        scroll.setWidget(panel)
        scroll.setMinimumWidth(340)
        return scroll

    def _build_center_panel(self) -> QWidget:
        panel = QWidget()
        panel.setMinimumWidth(200)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        center_splitter = QSplitter(Qt.Orientation.Horizontal)
        self.command_palette_group = self._build_command_palette()
        center_splitter.addWidget(self.command_palette_group)
        center_splitter.addWidget(self._build_program_editor_group())
        center_splitter.setStretchFactor(0, 0)
        center_splitter.setStretchFactor(1, 1)
        center_splitter.setSizes([220, 500])
        center_splitter.setCollapsible(0, True)
        center_splitter.setCollapsible(1, False)

        vertical_splitter = QSplitter(Qt.Orientation.Vertical)
        vertical_splitter.addWidget(center_splitter)
        self.response_log_section = CollapsibleSection(
            "RESPONSE LOG", self._build_response_log_group(), expanded=True
        )
        vertical_splitter.addWidget(self.response_log_section)
        vertical_splitter.setStretchFactor(0, 1)
        vertical_splitter.setStretchFactor(1, 0)
        vertical_splitter.setSizes([500, 160])
        vertical_splitter.setCollapsible(0, False)
        vertical_splitter.setCollapsible(1, True)
        vertical_splitter.setHandleWidth(6)
        vertical_splitter.setStyleSheet(
            "QSplitter::handle { background: #1E2A45; border-radius: 2px; }"
        )
        layout.addWidget(vertical_splitter, 1)
        return panel

    def _build_right_panel(self) -> QWidget:
        panel = QWidget()
        panel.setMinimumWidth(350)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        layout.addWidget(self._build_simulation_group(), 1)
        return panel

    def _build_connection_group(self) -> QWidget:
        group = self._create_section_frame()
        layout = QGridLayout(group)
        self._configure_grid_section(layout)
        self.port_combo = QComboBox()
        self.port_combo.addItems(["COM3", "COM4", "COM5", "MOCK"])
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200", "230400", "3000000"])
        self.baud_combo.setCurrentText("3000000")
        self.connect_button = QPushButton("Connect")
        self.connect_button.setProperty("role", "primary")
        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.setProperty("role", "danger")
        layout.addWidget(QLabel("COM Port"), 0, 0)
        layout.addWidget(self.port_combo, 0, 1)
        layout.addWidget(QLabel("Baudrate"), 1, 0)
        layout.addWidget(self.baud_combo, 1, 1)
        layout.addWidget(self.connect_button, 2, 0)
        layout.addWidget(self.disconnect_button, 2, 1)
        layout.setColumnStretch(1, 1)
        return group

    def _build_robot_actions_group(self) -> QWidget:
        group = self._create_section_frame()
        layout = QGridLayout(group)
        self._configure_grid_section(layout)
        self.enable_button = QPushButton("Enable")
        self.enable_button.setProperty("role", "primary")
        self.disable_button = QPushButton("Disable")
        self.disable_button.setProperty("role", "danger")
        self.home_button = QPushButton("Home")
        self.calibrate_button = QPushButton("Calibrate")
        self.clear_error_button = QPushButton("Clear Error")
        self.clear_error_button.setProperty("role", "ghost")
        buttons = [
            self.enable_button,
            self.disable_button,
            self.home_button,
            self.calibrate_button,
            self.clear_error_button,
        ]
        row = 0
        col = 0
        for button in buttons:
            layout.addWidget(button, row, col)
            col += 1
            if col == 2:
                col = 0
                row += 1
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)
        return group

    def _build_joint_jog_group(self) -> QWidget:
        group = self._create_section_frame()
        layout = QGridLayout(group)
        self._configure_grid_section(layout)
        _j_colors = ["#FF6B6B", "#4ECDC4", "#F5A623", "#A78BFA", "#38BDF8", "#FB7185"]
        for index, joint in enumerate(["J1", "J2", "J3", "J4", "J5", "J6"]):
            c = _j_colors[index]
            minus_button = QPushButton("-")
            minus_button.setProperty("role", "jog")
            plus_button = QPushButton("+")
            plus_button.setProperty("role", "jog")
            slider = QSlider()
            slider.setOrientation(Qt.Orientation.Horizontal)
            slider.setMinimum(-1800)
            slider.setMaximum(3600)
            slider.setValue(0)
            slider.setSingleStep(10)
            slider.setEnabled(False)  # disabled until Preview Mode ON
            value_label = QLabel("0.0°")
            value_label.setMinimumWidth(52)
            value_label.setStyleSheet(
                f"color: {c}; font-size: 11px; font-weight: 700;"
                "font-family: 'Consolas', 'Courier New', monospace;"
            )
            self.joint_minus_buttons.append(minus_button)
            self.joint_plus_buttons.append(plus_button)
            self.joint_sliders.append(slider)
            self.joint_slider_value_labels.append(value_label)
            lbl = QLabel(joint)
            lbl.setStyleSheet(f"color: {c}; font-weight: 700;")
            layout.addWidget(lbl, index, 0)
            layout.addWidget(minus_button, index, 1)
            layout.addWidget(slider, index, 2)
            layout.addWidget(plus_button, index, 3)
            layout.addWidget(value_label, index, 4)
        layout.setColumnStretch(2, 1)
        # Phase 9 – jog source indicator
        self.jog_source_label.setStyleSheet("color: #8A9AB8; font-size: 10px;")
        layout.addWidget(self.jog_source_label, 6, 0, 1, 5)
        return group

    def _build_cartesian_group(self) -> QWidget:
        group = self._create_section_frame()
        layout = QGridLayout(group)
        self._configure_grid_section(layout)
        axes = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
        for row, axis in enumerate(axes):
            minus_button = QPushButton("-")
            minus_button.setProperty("role", "jog")
            plus_button = QPushButton("+")
            plus_button.setProperty("role", "jog")
            value_label = QLabel("0.0")
            value_label.setMinimumWidth(62)
            value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            value_label.setStyleSheet(
                "color: #E8EDF5; font-size: 11px; font-weight: 600;"
                "font-family: 'Consolas', 'Courier New', monospace;"
            )
            self.cart_minus_buttons[axis] = minus_button
            self.cart_plus_buttons[axis] = plus_button
            self.cart_value_labels[axis] = value_label
            layout.addWidget(QLabel(axis), row, 0)
            layout.addWidget(minus_button, row, 1)
            layout.addWidget(value_label, row, 2)
            layout.addWidget(plus_button, row, 3)
        speed_label = QLabel("Speed %")
        self.speed_slider = QSlider()
        self.speed_slider.setOrientation(Qt.Orientation.Horizontal)
        self.speed_slider.setMinimum(1)
        self.speed_slider.setMaximum(100)
        self.speed_slider.setValue(30)
        self.frame_combo = QComboBox()
        self.frame_combo.addItems(["WRF", "TRF"])
        layout.addWidget(speed_label, len(axes), 0)
        layout.addWidget(self.speed_slider, len(axes), 1, 1, 3)
        layout.addWidget(QLabel("Frame"), len(axes) + 1, 0)
        layout.addWidget(self.frame_combo, len(axes) + 1, 1, 1, 3)
        layout.setColumnStretch(2, 1)
        return group

    def _build_io_group(self) -> QWidget:
        group = self._create_section_frame()
        layout = QGridLayout(group)
        self._configure_grid_section(layout)

        # Inputs (read-only)
        for i, name in enumerate(["INPUT 1", "INPUT 2"]):
            name_lbl = QLabel(name)
            state_lbl = QLabel("LOW")
            state_lbl.setStyleSheet(
                "color: #8A9AB8; font-weight: 700;"
                "font-family: 'Consolas', 'Courier New', monospace;"
            )
            state_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.input_state_labels.append(state_lbl)
            layout.addWidget(name_lbl, i, 0)
            layout.addWidget(state_lbl, i, 1, 1, 2)

        # Outputs (with LOW / HIGH buttons)
        for i, name in enumerate(["OUTPUT 1", "OUTPUT 2"]):
            row = 2 + i
            name_lbl = QLabel(name)
            state_lbl = QLabel("LOW")
            state_lbl.setStyleSheet(
                "color: #8A9AB8; font-weight: 700;"
                "font-family: 'Consolas', 'Courier New', monospace;"
            )
            state_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            state_lbl.setMinimumWidth(56)
            low_btn = QPushButton("LOW")
            high_btn = QPushButton("HIGH")
            high_btn.setProperty("role", "primary")
            self.output_state_labels.append(state_lbl)
            self.output_low_buttons.append(low_btn)
            self.output_high_buttons.append(high_btn)
            layout.addWidget(name_lbl, row, 0)
            layout.addWidget(state_lbl, row, 1)
            layout.addWidget(low_btn, row, 2)
            layout.addWidget(high_btn, row, 3)

        layout.setColumnStretch(1, 1)
        return group

    def update_io_state(self, inputs: list[int], outputs: list[int]) -> None:
        """Refresh INPUT/OUTPUT state labels from a runtime snapshot.

        inputs[0],inputs[1] map to INPUT 1, INPUT 2 (inout indices 0,1).
        outputs[2],outputs[3] map to OUTPUT 1, OUTPUT 2 (inout indices 2,3).
        """
        def _style(label: QLabel, value: int) -> None:
            if value:
                label.setText("HIGH")
                label.setStyleSheet(
                    "color: #4ADE80; font-weight: 700;"
                    "font-family: 'Consolas', 'Courier New', monospace;"
                )
            else:
                label.setText("LOW")
                label.setStyleSheet(
                    "color: #8A9AB8; font-weight: 700;"
                    "font-family: 'Consolas', 'Courier New', monospace;"
                )

        for i in range(min(2, len(self.input_state_labels))):
            if i < len(inputs):
                _style(self.input_state_labels[i], int(inputs[i]))
        for i in range(min(2, len(self.output_state_labels))):
            idx = 2 + i  # OUTPUT 1 -> inout[2], OUTPUT 2 -> inout[3]
            if idx < len(outputs):
                _style(self.output_state_labels[i], int(outputs[idx]))

    def _build_failsafe_group(self) -> QWidget:
        group = self._create_section_frame()
        layout = QGridLayout(group)
        self._configure_grid_section(layout)
        shortcut_label = QLabel("Shortcuts")
        shortcut_value = QLabel("Space / F1")
        state_title = QLabel("State")
        layout.addWidget(shortcut_label, 0, 0)
        layout.addWidget(shortcut_value, 0, 1)
        layout.addWidget(state_title, 1, 0)
        layout.addWidget(self.safety_label, 1, 1)
        layout.setColumnStretch(1, 1)
        return group

    def _build_response_log_group(self) -> QWidget:
        group = self._create_section_frame()
        layout = QVBoxLayout(group)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(10)
        self.response_log.setReadOnly(True)
        self.response_log.setMinimumHeight(80)
        layout.addWidget(self.response_log)
        return group

    def _build_command_palette(self) -> QGroupBox:
        group = QGroupBox("COMMANDS")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(6, 10, 6, 6)
        layout.setSpacing(6)

        # Teaching mode radio buttons
        teaching_frame = QFrame()
        teaching_layout = QVBoxLayout(teaching_frame)
        teaching_layout.setContentsMargins(4, 4, 4, 4)
        teaching_layout.setSpacing(4)
        self._radio_current = QRadioButton("Current Position")
        self._radio_custom = QRadioButton("Custom Position")
        self._radio_current.setChecked(True)
        self._radio_current.toggled.connect(self._on_teaching_mode_changed)
        teaching_layout.addWidget(self._radio_current)
        teaching_layout.addWidget(self._radio_custom)
        layout.addWidget(teaching_frame)

        # Command tree
        self.command_tree = QTreeWidget()
        self.command_tree.setHeaderLabels(["Commands"])
        self.command_tree.setMinimumWidth(180)
        self.command_tree.setIndentation(16)
        for category_name, commands in COMMAND_TREE_STRUCTURE:
            category_item = QTreeWidgetItem([category_name])
            category_item.setFlags(category_item.flags() & ~Qt.ItemFlag.ItemIsSelectable)
            for cmd_name in commands:
                child = QTreeWidgetItem([cmd_name])
                child.setData(0, Qt.ItemDataRole.UserRole, cmd_name)
                category_item.addChild(child)
            self.command_tree.addTopLevelItem(category_item)
            category_item.setExpanded(True)
        self.command_tree.itemClicked.connect(self._on_command_tree_clicked)
        layout.addWidget(self.command_tree, 1)

        # Hint label
        self.command_hint_label = QLabel("Klik command untuk insert ke program.")
        self.command_hint_label.setWordWrap(True)
        self.command_hint_label.setStyleSheet("color: #8A9AB8; font-size: 11px; padding: 4px;")
        layout.addWidget(self.command_hint_label)

        # Park button
        self.park_button = QPushButton("Park")
        self.park_button.setProperty("role", "ghost")
        self.park_button.clicked.connect(self._insert_park_program)
        layout.addWidget(self.park_button)

        group.setMaximumWidth(260)
        return group

    def _build_program_editor_group(self) -> QGroupBox:
        group = QGroupBox("PROGRAM EDITOR")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(6, 10, 6, 6)
        layout.setSpacing(6)

        # File toolbar
        file_toolbar = QHBoxLayout()
        self.commands_toggle_button = QPushButton("Commands")
        self.commands_toggle_button.setCheckable(True)
        self.commands_toggle_button.setChecked(True)
        self.commands_toggle_button.toggled.connect(self._toggle_commands_visibility)
        self.open_button = self._create_toolbar_button("Open", None, self._open_program)
        self.save_button = self._create_toolbar_button("Save", None, self._save_program)
        self.save_as_button = self._create_toolbar_button("Save As", None, self._save_as_program)
        
        file_toolbar.addWidget(self.commands_toggle_button)
        for btn in [self.open_button, self.save_button, self.save_as_button]:
            file_toolbar.addWidget(btn)
        file_toolbar.addStretch(1)
        layout.addLayout(file_toolbar)

        # Execution toolbar
        exec_toolbar = QHBoxLayout()
        self.run_button = self._create_toolbar_button("Start", "primary", self._run_program)
        self.pause_button = self._create_toolbar_button("Pause", None, self._pause_program)
        self.stop_button = self._create_toolbar_button("Stop", "danger", self._stop_program)
        self.step_button = self._create_toolbar_button("Step", None, self._step_program)
        for btn in [self.run_button, self.pause_button, self.stop_button, self.step_button]:
            exec_toolbar.addWidget(btn)
        exec_toolbar.addStretch(1)
        layout.addLayout(exec_toolbar)

        # Text editor with syntax highlighting
        self.program_editor.setFont(QFont("Consolas", 12))
        self.program_editor.setStyleSheet(
            "QPlainTextEdit { background-color: #0D1B35; color: #E8EDF5; "
            "border: 1px solid #243D6B; border-radius: 4px; padding: 6px; }"
        )
        self.program_editor.setPlainText("Begin()\n\nEnd()\n")
        self.program_editor.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self._highlighter = ProgramSyntaxHighlighter(self.program_editor.document())
        layout.addWidget(self.program_editor, 1)

        # Status row
        status_row = QHBoxLayout()
        status_row.addWidget(QLabel("Executing line:"))
        status_row.addWidget(self.current_line_value)
        self._file_label = QLabel("")
        self._file_label.setStyleSheet("color: #8A9AB8; font-size: 10px;")
        status_row.addStretch(1)
        status_row.addWidget(self._file_label)
        layout.addLayout(status_row)
        return group

    def _build_simulation_group(self) -> QFrame:
        group = self._create_section_frame()
        layout = QVBoxLayout(group)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(10)
        controls = QHBoxLayout()
        self.simulation_toggle_button = QPushButton("Hide Simulation")
        self.simulation_toggle_button.clicked.connect(self._toggle_simulation_visibility)
        controls.addWidget(self.simulation_toggle_button)

        self.preview_mode_button = QPushButton("Preview Mode: OFF")
        self.preview_mode_button.setCheckable(True)
        self.preview_mode_button.setProperty("role", "ghost")
        self.preview_mode_button.toggled.connect(self._on_preview_mode_toggled)
        controls.addWidget(self.preview_mode_button)

        controls.addStretch(1)
        layout.addLayout(controls)

        # --- Joint readout panel (Preview + Real-time side by side) ---
        joint_readout = QFrame()
        joint_readout.setStyleSheet(
            "QFrame { background: #111827; border-radius: 6px; padding: 6px; }"
        )
        jr_layout = QGridLayout(joint_readout)
        jr_layout.setContentsMargins(8, 6, 8, 6)
        jr_layout.setHorizontalSpacing(12)
        jr_layout.setVerticalSpacing(2)

        _j_colors = ["#FF6B6B", "#4ECDC4", "#F5A623", "#A78BFA", "#38BDF8", "#FB7185"]

        # Header row
        hdr_joint = QLabel("Joint")
        hdr_joint.setStyleSheet("color: #8A9AB8; font-size: 10px; font-weight: 700;")
        hdr_preview = QLabel("Preview (°)")
        hdr_preview.setStyleSheet("color: #F5A623; font-size: 10px; font-weight: 700;")
        hdr_realtime = QLabel("Real-time")
        hdr_realtime.setStyleSheet("color: #4ADE80; font-size: 10px; font-weight: 700;")
        jr_layout.addWidget(hdr_joint, 0, 0)
        jr_layout.addWidget(hdr_preview, 0, 1)
        jr_layout.addWidget(hdr_realtime, 0, 2)

        for i in range(6):
            c = _j_colors[i]
            name_lbl = QLabel(f"J{i + 1}")
            name_lbl.setStyleSheet(f"color: {c}; font-size: 11px; font-weight: 700;")
            preview_val = QLabel("0.00")
            preview_val.setStyleSheet(
                "color: #E8EDF5; font-size: 12px; font-weight: 600;"
                "font-family: 'Consolas', 'Courier New', monospace;"
            )
            realtime_val = QLabel("0.00")
            realtime_val.setStyleSheet(
                "color: #8A9AB8; font-size: 12px; font-weight: 600;"
                "font-family: 'Consolas', 'Courier New', monospace;"
            )
            jr_layout.addWidget(name_lbl, i + 1, 0)
            jr_layout.addWidget(preview_val, i + 1, 1)
            jr_layout.addWidget(realtime_val, i + 1, 2)
            self.preview_joint_labels.append(preview_val)
            self.realtime_joint_labels.append(realtime_val)

        jr_layout.setColumnStretch(1, 1)
        jr_layout.setColumnStretch(2, 1)

        # --- View angle shortcut buttons ---
        view_btn_row = QHBoxLayout()
        view_btn_row.setSpacing(4)
        view_angles = [
            ("Front",  20,    0),
            ("Back",   20,  180),
            ("Left",   20,  -90),
            ("Right",  20,   90),
            ("Top",    80,    0),
            ("Iso",    30,  -60),
        ]
        self._view_angle_buttons: list[QPushButton] = []
        for label, elev, azim in view_angles:
            btn = QPushButton(label)
            btn.setFixedHeight(24)
            btn.setStyleSheet(
                "QPushButton { font-size: 10px; padding: 2px 6px; }"
            )
            btn.clicked.connect(lambda checked, e=elev, a=azim: self._set_sim_view(e, a))
            view_btn_row.addWidget(btn)
            self._view_angle_buttons.append(btn)
        view_btn_row.addStretch(1)

        # --- Top part: joint readout + view buttons wrapped in a widget ---
        top_widget = QWidget()
        top_layout = QVBoxLayout(top_widget)
        top_layout.setContentsMargins(0, 0, 0, 0)
        top_layout.setSpacing(6)
        top_layout.addWidget(joint_readout)
        top_layout.addLayout(view_btn_row)

        # Two-tab simulator (Preview + Real-time) using matplotlib canvas
        self.sim_tab_widget = QTabWidget()
        placeholder = QLabel("3D simulator will be initialised after startup.")
        placeholder.setWordWrap(True)
        placeholder.setStyleSheet("color: #8A9AB8; padding: 20px;")
        self.sim_tab_widget.addTab(placeholder, "Preview")
        self.simulation_content = self.sim_tab_widget

        # --- Vertical splitter: drag handle between readout and 3D sim ---
        sim_splitter = QSplitter(Qt.Orientation.Vertical)
        sim_splitter.addWidget(top_widget)
        sim_splitter.addWidget(self.sim_tab_widget)
        sim_splitter.setStretchFactor(0, 0)   # readout: don't stretch
        sim_splitter.setStretchFactor(1, 1)   # sim: take remaining space
        sim_splitter.setSizes([140, 500])
        sim_splitter.setChildrenCollapsible(False)
        sim_splitter.setHandleWidth(6)
        sim_splitter.setStyleSheet(
            "QSplitter::handle { background: #1E2A45; border-radius: 2px; }"
        )
        layout.addWidget(sim_splitter, 1)
        return group

    def init_simulator_canvases(self, robot) -> None:
        """Create matplotlib RobotCanvas widgets and insert into tabs.

        Must be called after the robot model has been imported.
        """
        if RobotCanvas is None or self.sim_tab_widget is None:
            return
        # Remove placeholder tab(s)
        while self.sim_tab_widget.count():
            w = self.sim_tab_widget.widget(0)
            self.sim_tab_widget.removeTab(0)
            w.deleteLater()
        self.simulation_view = RobotCanvas(robot)
        self.simulation_view_rt = RobotCanvas(robot)
        self.sim_tab_widget.addTab(self.simulation_view, "Preview")
        self.sim_tab_widget.addTab(self.simulation_view_rt, "Real-time")

    def set_simulator_url(self, url: str) -> None:
        """No-op kept for backward compatibility."""
        pass

    def set_simulator_fallback(self, message: str) -> None:
        """Show a placeholder label when the simulator is unavailable."""
        if self.sim_tab_widget is not None:
            for i in range(self.sim_tab_widget.count()):
                self.sim_tab_widget.widget(i).hide()
            lbl = QLabel(message)
            lbl.setWordWrap(True)
            lbl.setStyleSheet("color: #8A9AB8; padding: 20px;")
            self.sim_tab_widget.addTab(lbl, "Info")
            self.sim_tab_widget.setCurrentWidget(lbl)

    def update_jog_source(self, source: str) -> None:
        """Update the jog source indicator label."""
        src = source.upper() if source else "NONE"
        colors = {"GUI": "#4ADE80", "MODBUS": "#F5A623", "NONE": "#8A9AB8"}
        color = colors.get(src, "#8A9AB8")
        self.jog_source_label.setText(f"Jog source: {src}")
        self.jog_source_label.setStyleSheet(f"color: {color}; font-size: 10px; font-weight: 600;")

    @staticmethod
    def _create_section_frame() -> QFrame:
        frame = QFrame()
        frame.setProperty("sectionBody", True)
        return frame

    @staticmethod
    def _configure_grid_section(layout: QGridLayout) -> None:
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(10)

    def _toggle_simulation_visibility(self) -> None:
        if self.simulation_content is None:
            return
        is_visible = self.simulation_content.isVisible()
        self.simulation_content.setVisible(not is_visible)

    def _toggle_commands_visibility(self, checked: bool) -> None:
        if self.command_palette_group is None:
            return
        self.command_palette_group.setVisible(checked)

    # --- Auto-repeat jog helpers ---
    def _start_jog_repeat(self, callback) -> None:
        """Begin auto-repeating *callback* while button is held."""
        self._jog_repeat_callback = callback
        callback()  # fire immediately on press
        self._jog_repeat_timer.start()

    def _stop_jog_repeat(self) -> None:
        """Stop auto-repeating when button is released."""
        self._jog_repeat_timer.stop()
        self._jog_repeat_callback = None

    def _fire_jog_repeat(self) -> None:
        """Called by timer."""
        if self._jog_repeat_callback is not None:
            self._jog_repeat_callback()
        if self.simulation_content is None:
            return
        is_visible = self.simulation_content.isVisible()
        self.simulation_toggle_button.setText(
            "Show Simulation" if is_visible else "Hide Simulation"
        )

    def _set_sim_view(self, elev: float, azim: float) -> None:
        """Apply a camera angle to both simulator canvases."""
        if self.simulation_view is not None:
            self.simulation_view.set_view_angle(elev, azim)
        if self.simulation_view_rt is not None:
            self.simulation_view_rt.set_view_angle(elev, azim)

    def _on_preview_mode_toggled(self, checked: bool) -> None:
        self.preview_mode_button.setText(
            "Preview Mode: ON" if checked else "Preview Mode: OFF"
        )
        color = "#F5A623" if checked else "#8A9AB8"
        self.preview_mode_button.setStyleSheet(
            f"color: {color}; font-weight: {'700' if checked else '400'};"
        )
        # Enable/disable sliders — sliders are only safe in preview mode
        for slider in self.joint_sliders:
            slider.setEnabled(checked)

    def is_preview_mode(self) -> bool:
        return self.preview_mode_button.isChecked()

    def update_simulation_status(self, positions: list[int]) -> None:
        """Update real-time joint labels from hardware feedback (steps/degrees)."""
        for i, val in enumerate(positions[:6]):
            if i < len(self.realtime_joint_labels):
                self.realtime_joint_labels[i].setText(f"{val:.2f}")

    def update_preview_joints(self, radians: list[float]) -> None:
        """Update preview joint labels and cartesian values from simulator FK."""
        import math
        import numpy as np
        for i, val in enumerate(radians[:6]):
            if i < len(self.preview_joint_labels):
                self.preview_joint_labels[i].setText(f"{math.degrees(val):.2f}")
        self.sync_sliders_from_radians(radians)
        self._update_cartesian_labels(radians)

    def _update_cartesian_labels(self, radians: list[float]) -> None:
        """Compute FK and update cartesian value labels (X/Y/Z in mm, Rx/Ry/Rz in °)."""
        if not self.cart_value_labels:
            return
        try:
            import numpy as np
            from spatialmath import SE3
            from tools.PAROL6_ROBOT import robot

            q = np.array(radians[:6])
            T = robot.fkine(q)

            # Position in mm
            x, y, z = T.t * 1000.0

            # Euler angles (RPY = Roll-Pitch-Yaw) in degrees, ZYX order
            rpy = T.rpy('deg', 'zyx')

            values = {
                "X": f"{x:.1f}",
                "Y": f"{y:.1f}",
                "Z": f"{z:.1f}",
                "Rx": f"{rpy[0]:.1f}",
                "Ry": f"{rpy[1]:.1f}",
                "Rz": f"{rpy[2]:.1f}",
            }
            for axis, lbl in self.cart_value_labels.items():
                lbl.setText(values.get(axis, "—"))
        except Exception:
            pass

    def sync_sliders_from_radians(self, radians: list[float]) -> None:
        """Sync slider positions from radians (called after any sim update)."""
        import math
        for i, val in enumerate(radians[:6]):
            if i < len(self.joint_sliders):
                deg = math.degrees(val)
                slider = self.joint_sliders[i]
                slider.blockSignals(True)
                slider.setValue(int(deg * 10))
                slider.blockSignals(False)
                if i < len(self.joint_slider_value_labels):
                    self.joint_slider_value_labels[i].setText(f"{deg:.1f}°")

    def program_rows(self) -> list[dict[str, str]]:
        """Return the current program text parsed into row-dict form."""
        return self._serialize_program()

    def selected_program_line(self) -> int:
        """Return the current cursor line in the text editor as 1-based."""
        cursor = self.program_editor.textCursor()
        return cursor.blockNumber() + 1

    def set_execution_line(self, line_number: int | None) -> None:
        """Highlight the currently executing line in the text editor."""
        line_count = self.program_editor.document().blockCount()
        if line_number is None or line_number <= 0 or line_number > line_count:
            self.current_line_value.setText("-")
            self._clear_line_highlight()
            return
        self.current_line_value.setText(str(line_number))
        self._highlight_line(line_number)

    def update_program_state(self, state: str) -> None:
        """Reflect program execution state in toolbar controls."""
        normalized = str(state).strip().lower()
        is_running = normalized == "running"
        is_paused = normalized == "paused"
        self.run_button.setEnabled(not is_running and not is_paused)
        self.step_button.setEnabled(not is_running and not is_paused)
        self.stop_button.setEnabled(is_running or is_paused)
        self.pause_button.setEnabled(is_running or is_paused)
        self.pause_button.setText("Resume" if is_paused else "Pause")
        self.program_editor.setReadOnly(is_running or is_paused)
        if normalized == "idle":
            self.current_line_value.setText("-")
            self._clear_line_highlight()
            self.program_editor.setReadOnly(False)

    def get_live_joint_degrees(self) -> list[float] | None:
        """Return live joint angles in degrees from the appropriate labels.

        In preview mode reads from preview_joint_labels (sim state);
        otherwise reads from realtime_joint_labels (hardware state).
        Returns None if data is not available yet.
        Called by teaching feature to capture current position.
        """
        is_preview = (
            hasattr(self, "preview_mode_button")
            and self.preview_mode_button.isChecked()
        )
        source_labels = (
            self.preview_joint_labels if is_preview else self.realtime_joint_labels
        )
        try:
            values = []
            for label in source_labels:
                text = label.text().strip()
                if text == "--":
                    return None
                values.append(float(text))
            if len(values) == 6:
                return values
        except (ValueError, IndexError):
            pass
        return None

    def get_live_cartesian(self) -> dict[str, float] | None:
        """Return live cartesian position from cart_value_labels."""
        try:
            result = {}
            for axis in ["X", "Y", "Z", "Rx", "Ry", "Rz"]:
                lbl = self.cart_value_labels.get(axis)
                if lbl is None:
                    return None
                result[axis] = float(lbl.text())
            return result
        except (ValueError, AttributeError):
            return None

    # --- Toolbar helpers ---

    def _create_toolbar_button(
        self,
        text: str,
        role: str | None,
        handler,
    ) -> QPushButton:
        button = QPushButton(text)
        if role:
            button.setProperty("role", role)
        button.clicked.connect(handler)
        return button

    # --- Teaching & command palette ---

    def _on_teaching_mode_changed(self, checked: bool) -> None:
        self._teaching_mode = "current" if checked else "custom"

    def _on_command_tree_clicked(self, item: QTreeWidgetItem, column: int) -> None:
        cmd_name = item.data(0, Qt.ItemDataRole.UserRole)
        if cmd_name is None:
            return
        spec = COMMAND_LIBRARY.get(cmd_name)
        if spec:
            self.command_hint_label.setText(spec["hint"])
        self._insert_command(cmd_name)

    def _insert_command(self, cmd_name: str) -> None:
        """Insert a command at the cursor in the text editor."""
        spec = COMMAND_LIBRARY.get(cmd_name, {})

        if cmd_name == "vision":
            lines_to_insert = self._build_vision_block()
        elif self._teaching_mode == "current":
            if cmd_name == "MoveJoint":
                joints = self.get_live_joint_degrees()
                if joints:
                    line = f"MoveJoint({','.join(f'{v:.3f}' for v in joints)},t=4)"
                else:
                    line = spec.get("template", f"{cmd_name}()")
            elif cmd_name in ("MovePose", "MoveCart"):
                cart = self.get_live_cartesian()
                if cart:
                    line = (f"{cmd_name}({cart['X']:.3f},{cart['Y']:.3f},{cart['Z']:.3f},"
                            f"{cart['Rx']:.3f},{cart['Ry']:.3f},{cart['Rz']:.3f},t=4)")
                else:
                    line = spec.get("template", f"{cmd_name}()")
            elif cmd_name == "MoveCartRelTRF":
                line = spec.get("template", f"{cmd_name}()")
            else:
                line = spec.get("template", f"{cmd_name}()")
            lines_to_insert = [line]
        else:
            lines_to_insert = [f"{cmd_name}()"]

        cursor = self.program_editor.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.EndOfBlock)
        cursor.insertText("\n" + "\n".join(lines_to_insert))
        self.program_editor.setTextCursor(cursor)
        self.program_editor.setFocus()
        if cmd_name == "vision" and len(lines_to_insert) > 1:
            self.append_log(f"Inserted: vision block ({len(lines_to_insert)} lines)")
        else:
            self.append_log(f"Inserted: {cmd_name}")

    def _build_vision_block(self) -> list[str]:
        """Build the vision command block: pre-pick orientation, vision(), post-pick descent.

        Pre-pick MoveJoint sets J6 to the calibrated grasping orientation so the
        gripper fingers align perpendicular to the selongsong cylinder axis.
        Post-pick MoveCartRelTRF descends along tool Z to actually grasp.
        """
        cfg = ConfigManager.instance()
        if not bool(cfg.get("vision.auto_insert_pre_post", True)):
            return ["vision()"]

        joints = cfg.get("vision.pre_pick_joints_deg",
                         [90.0, -144.683, 108.171, 2.222, 25.003, 180.0])
        descent_mm = float(cfg.get("vision.post_pick_descent_mm", 50.0))
        t_post = float(cfg.get("vision.post_pick_move_time_s", 2.0))

        joints_str = ",".join(f"{float(v):.3f}" for v in joints)
        return [
            f"MoveJoint({joints_str},t=4)",
            "vision()",
            f"MoveCartRelTRF(0,0,{descent_mm:.3f},0,0,0,t={t_post})",
        ]

    def _insert_park_program(self) -> None:
        """Insert a park program (like Commander's Park button)."""
        self.program_editor.setPlainText(
            "Begin()\n"
            "MoveJoint(90.0,-144.683,108.171,2.222,25.003,180.0,t=4)\n"
            "End()\n"
        )
        self.append_log("Park program loaded")

    # --- Line highlight ---

    def _highlight_line(self, line_number: int) -> None:
        """Highlight a specific line in the text editor."""
        block = self.program_editor.document().findBlockByLineNumber(line_number - 1)
        if not block.isValid():
            return
        cursor = QTextCursor(block)
        cursor.movePosition(QTextCursor.MoveOperation.EndOfBlock, QTextCursor.MoveMode.KeepAnchor)
        fmt = QTextCharFormat()
        fmt.setBackground(QColor("#1E3A5F"))
        selection = QTextEdit.ExtraSelection()
        selection.cursor = cursor
        selection.format = fmt
        self.program_editor.setExtraSelections([selection])

    def _clear_line_highlight(self) -> None:
        self.program_editor.setExtraSelections([])

    # --- Execution stubs (signals connected externally in main.py) ---

    def _run_program(self) -> None:
        self.current_line_value.setText("1")
        self.append_log("Run requested", "info")

    def _pause_program(self) -> None:
        self.append_log(
            "Pause/resume requested",
            "warn" if self.pause_button.text() == "Pause" else "info",
        )

    def _stop_program(self) -> None:
        self.current_line_value.setText("-")
        self._clear_line_highlight()
        self.append_log("Stop requested", "warn")

    def _step_program(self) -> None:
        line_count = self.program_editor.document().blockCount()
        if line_count == 0:
            return
        current = self.current_line_value.text()
        if current == "-":
            next_line = 1
        else:
            next_line = min(int(current) + 1, line_count)
        self.current_line_value.setText(str(next_line))
        self._highlight_line(next_line)
        self.append_log(f"Step requested to line {next_line}", "info")

    # --- File operations (.txt, Commander-compatible) ---

    def _open_program(self) -> None:
        programs_dir = Path(__file__).resolve().parents[1] / "program" / "Programs"
        programs_dir.mkdir(parents=True, exist_ok=True)
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Open Program",
            str(programs_dir),
            "Text Files (*.txt);;All Files (*)",
        )
        if not file_path:
            return
        text = Path(file_path).read_text(encoding="utf-8")
        self.program_editor.setPlainText(text)
        self._current_file_path = file_path
        self._file_label.setText(Path(file_path).name)
        self.append_log(f"Opened: {file_path}")

    def _save_program(self) -> None:
        if self._current_file_path:
            Path(self._current_file_path).write_text(
                self.program_editor.toPlainText(), encoding="utf-8"
            )
            self.append_log(f"Saved: {self._current_file_path}")
        else:
            self._save_as_program()

    def _save_as_program(self) -> None:
        programs_dir = Path(__file__).resolve().parents[1] / "program" / "Programs"
        programs_dir.mkdir(parents=True, exist_ok=True)
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Program As",
            str(programs_dir / "program.txt"),
            "Text Files (*.txt);;All Files (*)",
        )
        if not file_path:
            return
        Path(file_path).write_text(
            self.program_editor.toPlainText(), encoding="utf-8"
        )
        self._current_file_path = file_path
        self._file_label.setText(Path(file_path).name)
        self.append_log(f"Saved as: {file_path}")

    # --- Serialization (text → row dicts for executor compatibility) ---

    def _serialize_program(self) -> list[dict[str, str]]:
        """Parse the text editor content into row-dict form for the executor."""
        text = self.program_editor.toPlainText()
        rows: list[dict[str, str]] = []
        for raw_line in text.splitlines():
            line = raw_line.strip()
            if not line:
                continue
            rows.append({"command": line, "parameters": "-", "notes": ""})
        return rows
