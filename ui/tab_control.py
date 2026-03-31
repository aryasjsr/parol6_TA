from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path

from PyQt6.QtCore import Qt, QUrl
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
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSlider,
    QSplitter,
    QSizePolicy,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QToolButton,
    QVBoxLayout,
    QWidget,
)
from program.program_model import ProgramCommand, ProgramModel

try:
    from backend.simulator_bridge import RobotCanvas
except Exception:
    RobotCanvas = None


COMMAND_LIBRARY: dict[str, dict[str, str]] = {
    "start": {
        "parameters": "-",
        "notes": "Program boundary",
        "hint": "Penanda awal program.",
    },
    "MoveJoint": {
        "parameters": "j1=0, j2=0, j3=0, j4=0, j5=0, j6=0, speed_pct=30",
        "notes": "Move all joints in deg",
        "hint": "Gunakan derajat untuk j1..j6 dan speed_pct 0..100.",
    },
    "MoveCart": {
        "parameters": "x=0, y=0, z=200, r=0, p=0, yaw=0, speed_pct=30",
        "notes": "Move to absolute pose",
        "hint": "x,y,z dalam mm. r,p,yaw dalam deg.",
    },
    "MoveCartRelTRF": {
        "parameters": "dx=0, dy=0, dz=10, dr=0, dp=0, dyaw=0, speed_pct=30",
        "notes": "Move relative to tool frame",
        "hint": "dx,dy,dz dalam mm. dr,dp,dyaw dalam deg.",
    },
    "vision": {
        "parameters": "-",
        "notes": "Run vision-guided pick",
        "hint": "Menjalankan pick berbasis vision dengan setting aktif.",
    },
    "Home": {
        "parameters": "-",
        "notes": "Move robot to home",
        "hint": "Kirim robot ke posisi home.",
    },
    "Gripper": {
        "parameters": "state=open",
        "notes": "Control gripper state",
        "hint": "Gunakan state=open atau state=close.",
    },
    "Output": {
        "parameters": "n=1, state=ON",
        "notes": "Set digital output",
        "hint": "n adalah nomor output, state bisa ON/OFF.",
    },
    "Input": {
        "parameters": "n=1, value=1, timeout_s=10",
        "notes": "Wait for digital input",
        "hint": "Menunggu input bernilai 1/0 sampai timeout_s.",
    },
    "Delay": {
        "parameters": "ms=500",
        "notes": "Pause execution",
        "hint": "Delay dalam milidetik.",
    },
    "Loop": {
        "parameters": "count=2",
        "notes": "Repeat following block",
        "hint": "Jumlah pengulangan blok sampai EndLoop.",
    },
    "EndLoop": {
        "parameters": "-",
        "notes": "Close loop block",
        "hint": "Penutup untuk Loop().",
    },
    "end": {
        "parameters": "-",
        "notes": "Program boundary",
        "hint": "Penanda akhir program.",
    },
}


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
        self.program_table = QTableWidget(0, 4)
        self.generated_script = QTextEdit()
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
        self.jog_source_label = QLabel("Jog source: NONE")
        self.simulation_status_label = QLabel("Joint state: --")
        self.preview_joint_labels: list[QLabel] = []
        self.realtime_joint_labels: list[QLabel] = []
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
        layout.addWidget(CollapsibleSection("FAILSAFE", self._build_failsafe_group()))
        layout.addWidget(CollapsibleSection("RESPONSE LOG", self._build_response_log_group()))
        scroll.setWidget(panel)
        scroll.setMinimumWidth(340)
        return scroll

    def _build_center_panel(self) -> QWidget:
        panel = QWidget()
        panel.setMinimumWidth(200)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        layout.addWidget(self._build_program_group(), 3)
        layout.addWidget(self._build_generated_script_group(), 2)
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
            step_box = QDoubleSpinBox()
            step_box.setRange(-999.0, 999.0)
            step_box.setValue(1.0)
            self.cart_minus_buttons[axis] = minus_button
            self.cart_plus_buttons[axis] = plus_button
            layout.addWidget(QLabel(axis), row, 0)
            layout.addWidget(minus_button, row, 1)
            layout.addWidget(step_box, row, 2)
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
        self.response_log.setMinimumHeight(120)
        layout.addWidget(self.response_log)
        return group

    def _build_program_group(self) -> QGroupBox:
        group = QGroupBox("PROGRAM EDITOR")
        layout = QVBoxLayout(group)

        toolbar = QHBoxLayout()
        self.add_button = self._create_toolbar_button("Add", "primary", self._add_command)
        self.delete_button = self._create_toolbar_button(
            "Delete", "danger", self._delete_selected_command
        )
        self.move_up_button = self._create_toolbar_button("Move Up", None, self._move_selected_up)
        self.move_down_button = self._create_toolbar_button(
            "Move Down", None, self._move_selected_down
        )
        self.import_button = self._create_toolbar_button("Import JSON", None, self._import_program)
        self.export_button = self._create_toolbar_button("Export JSON", None, self._export_program)
        self.run_button = self._create_toolbar_button("Run", "primary", self._run_program)
        self.pause_button = self._create_toolbar_button("Pause", None, self._pause_program)
        self.stop_button = self._create_toolbar_button("Stop", "danger", self._stop_program)
        self.step_button = self._create_toolbar_button("Step", None, self._step_program)
        for button in [
            self.add_button,
            self.delete_button,
            self.move_up_button,
            self.move_down_button,
            self.import_button,
            self.export_button,
            self.run_button,
            self.pause_button,
            self.stop_button,
            self.step_button,
        ]:
            toolbar.addWidget(button)
        toolbar.addStretch(1)
        layout.addLayout(toolbar)

        self.program_table.setHorizontalHeaderLabels(["No", "Command", "Parameters", "Notes"])
        self.program_table.setAlternatingRowColors(True)
        self.program_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.program_table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.program_table.verticalHeader().setVisible(False)
        self.program_table.cellDoubleClicked.connect(self._edit_command)
        self._append_program_row("start", "-", "Program boundary")
        layout.addWidget(self.program_table, 1)

        status_row = QHBoxLayout()
        status_row.addWidget(QLabel("Current executing line:"))
        status_row.addWidget(self.current_line_value)
        status_row.addStretch(1)
        layout.addLayout(status_row)
        self._sync_generated_script()
        return group

    def _build_generated_script_group(self) -> QGroupBox:
        group = QGroupBox("GENERATED SCRIPT")
        layout = QVBoxLayout(group)
        self.generated_script.setReadOnly(True)
        layout.addWidget(self.generated_script)
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
            ("Front",   0,    0),
            ("Back",    0,  180),
            ("Left",    0,  -90),
            ("Right",   0,   90),
            ("Top",    90,    0),
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
        """Update preview joint labels from simulator FK (radians → degrees)."""
        import math
        for i, val in enumerate(radians[:6]):
            if i < len(self.preview_joint_labels):
                self.preview_joint_labels[i].setText(f"{math.degrees(val):.2f}")
        self.sync_sliders_from_radians(radians)

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
        """Return the current program table content in row-dict form."""
        return self._serialize_program()

    def selected_program_line(self) -> int:
        """Return the selected program line or the current line label as a 1-based integer."""
        row = self.program_table.currentRow()
        if row >= 0:
            return row + 1
        current = self.current_line_value.text().strip()
        if current.isdigit():
            return int(current)
        return 1

    def set_execution_line(self, line_number: int | None) -> None:
        """Update the highlighted execution line in the program table."""
        if line_number is None or line_number <= 0 or line_number > self.program_table.rowCount():
            self.current_line_value.setText("-")
            self.program_table.clearSelection()
            return
        self.current_line_value.setText(str(line_number))
        self.program_table.selectRow(line_number - 1)

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
        if normalized == "idle":
            self.current_line_value.setText("-")
            self.program_table.clearSelection()

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

    def _add_command(self) -> None:
        dialog = CommandDialog(parent=self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        command, parameters, notes = dialog.values()
        if not command:
            return
        self._append_program_row(command, parameters or "-", notes)
        self._sync_generated_script()
        self.append_log(f"Command added: {command}")

    def _edit_command(self, row: int, _: int) -> None:
        command = self.program_table.item(row, 1).text() if self.program_table.item(row, 1) else ""
        parameters = self.program_table.item(row, 2).text() if self.program_table.item(row, 2) else ""
        notes = self.program_table.item(row, 3).text() if self.program_table.item(row, 3) else ""
        dialog = CommandDialog(command, parameters, notes, self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        updated_command, updated_parameters, updated_notes = dialog.values()
        if not updated_command:
            return
        self.program_table.setItem(row, 1, QTableWidgetItem(updated_command))
        self.program_table.setItem(row, 2, QTableWidgetItem(updated_parameters or "-"))
        self.program_table.setItem(row, 3, QTableWidgetItem(updated_notes))
        self._sync_generated_script()
        self.append_log(f"Command updated on line {row + 1}")

    def _delete_selected_command(self) -> None:
        row = self.program_table.currentRow()
        if row < 0:
            return
        self.program_table.removeRow(row)
        self._renumber_rows()
        self._sync_generated_script()
        self.append_log(f"Command deleted from line {row + 1}", "warn")

    def _move_selected_up(self) -> None:
        row = self.program_table.currentRow()
        if row <= 0:
            return
        self._swap_rows(row, row - 1)
        self.program_table.selectRow(row - 1)
        self._sync_generated_script()

    def _move_selected_down(self) -> None:
        row = self.program_table.currentRow()
        if row < 0 or row >= self.program_table.rowCount() - 1:
            return
        self._swap_rows(row, row + 1)
        self.program_table.selectRow(row + 1)
        self._sync_generated_script()

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
        self.append_log("Stop requested", "warn")

    def _step_program(self) -> None:
        if self.program_table.rowCount() == 0:
            return
        current = self.current_line_value.text()
        if current == "-":
            next_line = 1
        else:
            next_line = min(int(current) + 1, self.program_table.rowCount())
        self.current_line_value.setText(str(next_line))
        self.program_table.selectRow(next_line - 1)
        self.append_log(f"Step requested to line {next_line}", "info")

    def _export_program(self) -> None:
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Export Program",
            str(Path.home() / "program.json"),
            "JSON Files (*.json)",
        )
        if not file_path:
            return
        ProgramModel.from_rows(self._serialize_program()).save_json(file_path)
        self.append_log(f"Program exported: {file_path}")

    def _import_program(self) -> None:
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Import Program",
            str(Path.home()),
            "JSON Files (*.json)",
        )
        if not file_path:
            return
        data = ProgramModel.load_json(file_path).to_rows()
        self.program_table.setRowCount(0)
        for row in data:
            self._append_program_row(
                row.get("command", ""),
                row.get("parameters", "-"),
                row.get("notes", ""),
            )
        self._sync_generated_script()
        self.append_log(f"Program imported: {file_path}")

    def _append_program_row(self, command: str, parameters: str, notes: str) -> None:
        normalized_command, normalized_parameters = _normalize_command_row(command, parameters)
        row = self.program_table.rowCount()
        self.program_table.insertRow(row)
        self.program_table.setItem(row, 0, QTableWidgetItem(str(row + 1)))
        self.program_table.setItem(row, 1, QTableWidgetItem(normalized_command))
        self.program_table.setItem(row, 2, QTableWidgetItem(normalized_parameters))
        self.program_table.setItem(row, 3, QTableWidgetItem(notes))

    def _swap_rows(self, first: int, second: int) -> None:
        values = []
        for row in (first, second):
            values.append([
                self.program_table.item(row, column).text()
                if self.program_table.item(row, column) is not None
                else ""
                for column in range(1, 4)
            ])
        for row, value_set in zip((first, second), reversed(values)):
            for column, value in enumerate(value_set, start=1):
                self.program_table.setItem(row, column, QTableWidgetItem(value))
        self._renumber_rows()

    def _renumber_rows(self) -> None:
        for row in range(self.program_table.rowCount()):
            self.program_table.setItem(row, 0, QTableWidgetItem(str(row + 1)))

    def _serialize_program(self) -> list[dict[str, str]]:
        rows: list[dict[str, str]] = []
        for row in range(self.program_table.rowCount()):
            rows.append(
                {
                    "command": self.program_table.item(row, 1).text()
                    if self.program_table.item(row, 1)
                    else "",
                    "parameters": self.program_table.item(row, 2).text()
                    if self.program_table.item(row, 2)
                    else "",
                    "notes": self.program_table.item(row, 3).text()
                    if self.program_table.item(row, 3)
                    else "",
                }
            )
        return rows

    def _sync_generated_script(self) -> None:
        rows = self._serialize_program()
        try:
            script = ProgramModel.from_rows(rows).to_script()
        except Exception:
            script_lines = []
            for row in rows:
                line = row["command"]
                if row["parameters"] and row["parameters"] != "-":
                    line = f"{line}  # {row['parameters']}"
                script_lines.append(line)
            script = "\n".join(script_lines).strip() + "\n"
        self.generated_script.setPlainText(script)
