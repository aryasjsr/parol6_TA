from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QToolButton,
    QVBoxLayout,
    QWidget,
)


class _CollapsibleSection(QWidget):
    """Collapsible section widget for the modbus tab."""

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


class ModbusTab(QWidget):
    """Modbus configuration and live monitoring panel for phase 5."""

    def __init__(self) -> None:
        super().__init__()
        self.ip_edit: QLineEdit | None = None
        self.port_spin: QSpinBox | None = None
        self.slave_id_spin: QSpinBox | None = None
        self.connect_button: QPushButton | None = None
        self.disconnect_button: QPushButton | None = None
        self.save_button: QPushButton | None = None
        self.status_value = QLabel("DISCONNECTED")
        self.address_table = QTableWidget(0, 5)
        self.monitor_table = QTableWidget(0, 2)
        self.add_row_button: QPushButton | None = None
        self.remove_row_button: QPushButton | None = None
        self.jog_speed_spin: QDoubleSpinBox | None = None
        self.jog_timeout_spin: QDoubleSpinBox | None = None
        self.jog_save_button: QPushButton | None = None
        self.jog_source_label = QLabel("Active source: NONE")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        layout.addWidget(_CollapsibleSection("MODBUS CONNECTION", self._build_connection_group()))
        layout.addWidget(_CollapsibleSection("MODBUS JOG", self._build_jog_group()))
        layout.addWidget(_CollapsibleSection("ADDRESS MAPPING", self._build_mapping_group()), 2)
        layout.addWidget(_CollapsibleSection("LIVE MONITOR", self._build_monitor_group()), 1)

    def _build_connection_group(self) -> QWidget:
        group = QWidget()
        layout = QGridLayout(group)
        self.ip_edit = QLineEdit("192.168.1.10")
        self.port_spin = QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(502)
        self.slave_id_spin = QSpinBox()
        self.slave_id_spin.setRange(1, 247)
        self.slave_id_spin.setValue(1)
        self.connect_button = QPushButton("Connect")
        self.connect_button.setProperty("role", "primary")
        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.setProperty("role", "danger")
        self.save_button = QPushButton("Save Config")

        layout.addWidget(QLabel("IP"), 0, 0)
        layout.addWidget(self.ip_edit, 0, 1)
        layout.addWidget(QLabel("Port"), 0, 2)
        layout.addWidget(self.port_spin, 0, 3)
        layout.addWidget(QLabel("Slave ID"), 1, 0)
        layout.addWidget(self.slave_id_spin, 1, 1)
        layout.addWidget(QLabel("Status"), 1, 2)
        layout.addWidget(self.status_value, 1, 3)
        layout.addWidget(self.connect_button, 2, 0, 1, 2)
        layout.addWidget(self.disconnect_button, 2, 2)
        layout.addWidget(self.save_button, 2, 3)
        return group

    def _build_mapping_group(self) -> QWidget:
        group = QWidget()
        layout = QVBoxLayout(group)

        toolbar = QHBoxLayout()
        self.add_row_button = QPushButton("Add Row")
        self.remove_row_button = QPushButton("Remove Row")
        toolbar.addWidget(self.add_row_button)
        toolbar.addWidget(self.remove_row_button)
        toolbar.addStretch(1)
        layout.addLayout(toolbar)

        self.address_table.setHorizontalHeaderLabels(["Name", "Type", "Address", "RW", "Description"])
        self.address_table.verticalHeader().setVisible(False)
        layout.addWidget(self.address_table, 1)
        return group

    def _build_monitor_group(self) -> QWidget:
        group = QWidget()
        layout = QVBoxLayout(group)
        self.monitor_table.setHorizontalHeaderLabels(["Signal", "Value"])
        self.monitor_table.verticalHeader().setVisible(False)
        layout.addWidget(self.monitor_table, 1)
        return group

    def _build_jog_group(self) -> QWidget:
        group = QWidget()
        layout = QGridLayout(group)
        self.jog_source_label.setStyleSheet("color: #8A9AB8; font-weight: 600;")
        layout.addWidget(self.jog_source_label, 0, 0, 1, 2)

        layout.addWidget(QLabel("Jog speed (%)"), 1, 0)
        self.jog_speed_spin = QDoubleSpinBox()
        self.jog_speed_spin.setRange(1, 100)
        self.jog_speed_spin.setValue(20)
        self.jog_speed_spin.setDecimals(0)
        layout.addWidget(self.jog_speed_spin, 1, 1)

        layout.addWidget(QLabel("Lock timeout (s)"), 2, 0)
        self.jog_timeout_spin = QDoubleSpinBox()
        self.jog_timeout_spin.setRange(1, 60)
        self.jog_timeout_spin.setValue(5.0)
        self.jog_timeout_spin.setDecimals(1)
        layout.addWidget(self.jog_timeout_spin, 2, 1)

        self.jog_save_button = QPushButton("Save Jog Config")
        layout.addWidget(self.jog_save_button, 3, 0, 1, 2)
        return group

    def update_jog_source(self, source: str) -> None:
        src = source.upper() if source else "NONE"
        colors = {"GUI": "#4ADE80", "MODBUS": "#F5A623", "NONE": "#8A9AB8"}
        color = colors.get(src, "#8A9AB8")
        self.jog_source_label.setText(f"Active source: {src}")
        self.jog_source_label.setStyleSheet(f"color: {color}; font-weight: 600;")

    def apply_config(self, config: dict) -> None:
        modbus = config.get("modbus", {})
        self.ip_edit.setText(str(modbus.get("ip", "192.168.1.10")))
        self.port_spin.setValue(int(modbus.get("port", 502)))
        self.slave_id_spin.setValue(int(modbus.get("slave_id", 1)))
        self.set_address_rows(modbus.get("addresses", []))
        if self.jog_speed_spin is not None:
            self.jog_speed_spin.setValue(float(modbus.get("jog_speed_pct", 20)))
        if self.jog_timeout_spin is not None:
            self.jog_timeout_spin.setValue(float(modbus.get("jog_lock_timeout_s", 5.0)))

    def build_config(self) -> tuple[str, int, int, list[dict[str, str | int]]]:
        return (
            self.ip_edit.text().strip(),
            int(self.port_spin.value()),
            int(self.slave_id_spin.value()),
            self.address_rows(),
        )

    def address_rows(self) -> list[dict[str, str | int]]:
        rows: list[dict[str, str | int]] = []
        for row in range(self.address_table.rowCount()):
            rows.append(
                {
                    "name": self._table_text(self.address_table, row, 0),
                    "type": self._table_text(self.address_table, row, 1) or "coil",
                    "address": int(self._table_text(self.address_table, row, 2) or 0),
                    "rw": self._table_text(self.address_table, row, 3) or "read",
                    "desc": self._table_text(self.address_table, row, 4),
                }
            )
        return rows

    def set_address_rows(self, rows: list[dict]) -> None:
        self.address_table.setRowCount(0)
        for row in rows:
            self.append_address_row(row)

    def append_address_row(self, row: dict | None = None) -> None:
        row_data = row or {"name": "signal", "type": "coil", "address": 0, "rw": "read", "desc": ""}
        row_index = self.address_table.rowCount()
        self.address_table.insertRow(row_index)
        self.address_table.setItem(row_index, 0, QTableWidgetItem(str(row_data.get("name", "signal"))))
        self.address_table.setItem(row_index, 1, QTableWidgetItem(str(row_data.get("type", "coil"))))
        self.address_table.setItem(row_index, 2, QTableWidgetItem(str(row_data.get("address", 0))))
        self.address_table.setItem(row_index, 3, QTableWidgetItem(str(row_data.get("rw", "read"))))
        self.address_table.setItem(row_index, 4, QTableWidgetItem(str(row_data.get("desc", ""))))

    def remove_selected_row(self) -> None:
        row = self.address_table.currentRow()
        if row >= 0:
            self.address_table.removeRow(row)

    def set_connection_status(self, connected: bool, description: str) -> None:
        self.status_value.setText(description if description else ("CONNECTED" if connected else "DISCONNECTED"))
        color = "#22C97A" if connected else "#E84040"
        self.status_value.setStyleSheet(f"color: {color}; font-weight: 700;")

    def update_monitor(self, snapshot: dict) -> None:
        self.monitor_table.setRowCount(0)
        for row_index, (name, value) in enumerate(snapshot.items()):
            self.monitor_table.insertRow(row_index)
            self.monitor_table.setItem(row_index, 0, QTableWidgetItem(str(name)))
            self.monitor_table.setItem(row_index, 1, QTableWidgetItem(str(value)))

    @staticmethod
    def _table_text(table: QTableWidget, row: int, column: int) -> str:
        item = table.item(row, column)
        return item.text().strip() if item is not None else ""
