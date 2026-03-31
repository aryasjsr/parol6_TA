from __future__ import annotations

import platform
import subprocess

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
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
    QSizePolicy,
    QSpinBox,
    QToolButton,
    QVBoxLayout,
    QWidget,
)


class _CollapsibleSection(QWidget):
    """Collapsible section widget for the vision config panel."""

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


class VisionTab(QWidget):
    """Vision UI for live feed, detection tuning, workspace config, and calibration."""

    confirm_pick_signal = pyqtSignal()
    skip_pick_signal = pyqtSignal()

    def __init__(self) -> None:
        super().__init__()
        self.live_feed_label = QLabel()
        self.status_value = QLabel("IDLE")
        self.object_status_value = QLabel("NO OBJECT")
        self.bbox_value = QLabel("-")
        self.centroid_value = QLabel("-")
        self.dimension_value = QLabel("-")
        self.pick_point_value = QLabel("-")
        self.orientation_value = QLabel("-")
        self.camera_source_combo: QComboBox | None = None
        self.start_camera_button: QPushButton | None = None
        self.stop_camera_button: QPushButton | None = None
        self.method_combo: QComboBox | None = None
        self.adaptive_block_size_spin: QSpinBox | None = None
        self.adaptive_c_spin: QSpinBox | None = None
        self.canny_threshold1_spin: QSpinBox | None = None
        self.canny_threshold2_spin: QSpinBox | None = None
        self.hsv_lower_h_spin: QSpinBox | None = None
        self.hsv_lower_s_spin: QSpinBox | None = None
        self.hsv_lower_v_spin: QSpinBox | None = None
        self.hsv_upper_h_spin: QSpinBox | None = None
        self.hsv_upper_s_spin: QSpinBox | None = None
        self.hsv_upper_v_spin: QSpinBox | None = None
        self.min_contour_area_spin: QSpinBox | None = None
        self.max_contour_area_spin: QSpinBox | None = None
        self.morph_kernel_size_spin: QSpinBox | None = None
        self.brightness_spin: QSpinBox | None = None
        self.contrast_spin: QDoubleSpinBox | None = None
        self.zoom_spin: QDoubleSpinBox | None = None
        self.tune_live_checkbox: QCheckBox | None = None
        self.apply_detection_button: QPushButton | None = None
        self.save_detection_button: QPushButton | None = None
        self.pick_inset_x_spin: QDoubleSpinBox | None = None
        self.pick_inset_y_spin: QDoubleSpinBox | None = None
        self.safe_pick_margin_pct_spin: QDoubleSpinBox | None = None
        self.marginal_timeout_spin: QDoubleSpinBox | None = None
        self.preview_pick_checkbox: QCheckBox | None = None
        self.save_pick_zone_button: QPushButton | None = None
        self.workspace_x_min_spin: QDoubleSpinBox | None = None
        self.workspace_x_max_spin: QDoubleSpinBox | None = None
        self.workspace_y_min_spin: QDoubleSpinBox | None = None
        self.workspace_y_max_spin: QDoubleSpinBox | None = None
        self.workspace_z_spin: QDoubleSpinBox | None = None
        self.workspace_margin_spin: QDoubleSpinBox | None = None
        self.save_workspace_button: QPushButton | None = None
        self.calibration_chessboard_x_spin: QSpinBox | None = None
        self.calibration_chessboard_y_spin: QSpinBox | None = None
        self.calibration_square_size_spin: QDoubleSpinBox | None = None
        self.snap_frame_button: QPushButton | None = None
        self.run_calibration_button: QPushButton | None = None
        self.save_calibration_button: QPushButton | None = None
        self.calibration_value = QLabel("fx=- fy=- cx=- cy=-")
        self.distortion_value = QLabel("-")
        self.snapshot_value = QLabel("0")
        # Model config widgets (4.8)
        self.model_path_edit: QLineEdit | None = None
        self.model_browse_button: QPushButton | None = None
        self.model_conf_spin: QDoubleSpinBox | None = None
        self.model_iou_spin: QDoubleSpinBox | None = None
        self.model_load_button: QPushButton | None = None
        self.model_save_button: QPushButton | None = None
        self.model_status_label = QLabel("NOT LOADED")
        # Safety badge (4.9)
        self.safety_badge = QLabel("")
        self._safety_badge_visible = False
        # MARGINAL confirmation (4.10)
        self._marginal_widget: QWidget | None = None
        self._marginal_countdown_label: QLabel | None = None
        self._marginal_confirm_btn: QPushButton | None = None
        self._marginal_skip_btn: QPushButton | None = None
        self._marginal_timer: QTimer | None = None
        self._marginal_remaining_s: int = 0
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        layout.addWidget(self._build_live_feed_group(), 3)
        layout.addWidget(self._build_side_panel_group(), 2)

    def _build_live_feed_group(self) -> QGroupBox:
        group = QGroupBox("LIVE FEED")
        layout = QVBoxLayout(group)
        controls = QHBoxLayout()
        self.camera_source_combo = QComboBox()
        self._populate_camera_list()
        self._refresh_camera_button = QPushButton("⟳")
        self._refresh_camera_button.setFixedWidth(32)
        self._refresh_camera_button.setToolTip("Refresh camera list")
        self._refresh_camera_button.clicked.connect(self._populate_camera_list)
        self.start_camera_button = QPushButton("Start Camera")
        self.start_camera_button.setProperty("role", "primary")
        self.stop_camera_button = QPushButton("Stop Camera")
        self.stop_camera_button.setProperty("role", "danger")
        controls.addWidget(QLabel("Source"))
        controls.addWidget(self.camera_source_combo, 1)
        controls.addWidget(self._refresh_camera_button)
        controls.addWidget(self.start_camera_button)
        controls.addWidget(self.stop_camera_button)
        layout.addLayout(controls)

        self.live_feed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.live_feed_label.setMinimumSize(640, 480)
        self.live_feed_label.setStyleSheet(
            "background-color: #070E1C; border: 1px solid #243D6B; color: #8A9AB8;"
        )
        self.live_feed_label.setText("Camera feed not started")
        layout.addWidget(self.live_feed_label, 1)

        # Safety badge overlay (4.9) — positioned over live feed
        self.safety_badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.safety_badge.setFixedHeight(28)
        self.safety_badge.setStyleSheet(
            "background: transparent; color: #8A9AB8; font-weight: bold; font-size: 13px;"
        )
        self.safety_badge.hide()
        layout.addWidget(self.safety_badge)

        # MARGINAL confirmation widget (4.10)
        self._marginal_widget = QWidget()
        marg_layout = QHBoxLayout(self._marginal_widget)
        marg_layout.setContentsMargins(8, 4, 8, 4)
        self._marginal_widget.setStyleSheet(
            "background-color: rgba(245, 166, 35, 40); border: 2px solid #F5A623; border-radius: 6px;"
        )
        self._marginal_countdown_label = QLabel("MARGINAL — Auto-skip in 10s")
        self._marginal_countdown_label.setStyleSheet("color: #F5A623; font-weight: bold; border: none;")
        self._marginal_confirm_btn = QPushButton("✓ Confirm Pick")
        self._marginal_confirm_btn.setStyleSheet(
            "background-color: #D4A017; color: #070E1C; font-weight: bold; padding: 4px 12px; border-radius: 4px; border: none;"
        )
        self._marginal_skip_btn = QPushButton("✗ Skip")
        self._marginal_skip_btn.setStyleSheet(
            "background-color: transparent; color: #F5A623; font-weight: bold; padding: 4px 12px; "
            "border: 1px solid #F5A623; border-radius: 4px;"
        )
        self._marginal_confirm_btn.clicked.connect(self._on_confirm_pick)
        self._marginal_skip_btn.clicked.connect(self._on_skip_pick)
        marg_layout.addWidget(self._marginal_countdown_label, 1)
        marg_layout.addWidget(self._marginal_confirm_btn)
        marg_layout.addWidget(self._marginal_skip_btn)
        self._marginal_widget.hide()
        layout.addWidget(self._marginal_widget)

        self._marginal_timer = QTimer(self)
        self._marginal_timer.setInterval(1000)
        self._marginal_timer.timeout.connect(self._marginal_tick)

        footer = QHBoxLayout()
        footer.addWidget(QLabel("Status"))
        footer.addWidget(self.status_value, 1)
        footer.addStretch(1)
        layout.addLayout(footer)
        return group

    # ---- Camera enumeration ----

    @staticmethod
    def _get_windows_camera_names() -> list[str]:
        """Query PnP camera device friendly names via PowerShell (Windows only)."""
        try:
            result = subprocess.run(
                ["powershell", "-Command",
                 "Get-PnpDevice -Class Camera,Image -Status OK "
                 "| Select-Object -ExpandProperty FriendlyName"],
                capture_output=True, text=True, timeout=5,
            )
            names = [n.strip() for n in result.stdout.strip().splitlines() if n.strip()]
            return names
        except Exception:
            return []

    def _populate_camera_list(self) -> None:
        """Auto-detect available cameras and populate the combo box."""
        prev_data = self.camera_source_combo.currentData()
        self.camera_source_combo.clear()

        # Probe OpenCV indices
        available_indices: list[int] = []
        try:
            import cv2
            for idx in range(8):
                cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW if platform.system() == "Windows" else cv2.CAP_ANY)
                if cap.isOpened():
                    available_indices.append(idx)
                    cap.release()
                else:
                    break  # stop at first gap
        except Exception:
            available_indices = [0]

        # Get friendly names on Windows
        device_names: list[str] = []
        if platform.system() == "Windows":
            device_names = self._get_windows_camera_names()

        # Add detected cameras
        for i, idx in enumerate(available_indices):
            if i < len(device_names):
                label = f"{device_names[i]}  (index {idx})"
            else:
                label = f"Camera {idx}"
            self.camera_source_combo.addItem(label, str(idx))

        # Always add MOCK at the end
        self.camera_source_combo.addItem("MOCK (Synthetic)", "MOCK")

        # Restore previous selection
        if prev_data is not None:
            restore_idx = self.camera_source_combo.findData(prev_data)
            if restore_idx >= 0:
                self.camera_source_combo.setCurrentIndex(restore_idx)

    def _build_side_panel_group(self) -> QGroupBox:
        group = QGroupBox("VISION CONFIG")
        layout = QVBoxLayout(group)
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QScrollArea.Shape.NoFrame)
        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(10)
        content_layout.addWidget(_CollapsibleSection("DETECTION SETUP", self._build_detection_group()))
        content_layout.addWidget(_CollapsibleSection("MODEL CONFIG", self._build_model_config_group(), expanded=False))
        content_layout.addWidget(_CollapsibleSection("OBJECT INFO", self._build_object_info_group()))
        content_layout.addWidget(_CollapsibleSection("SAFE PICK ZONE", self._build_pick_zone_group(), expanded=False))
        content_layout.addWidget(_CollapsibleSection("WORKSPACE", self._build_workspace_group(), expanded=False))
        content_layout.addWidget(_CollapsibleSection("CAMERA CALIBRATION", self._build_calibration_group(), expanded=False))
        content_layout.addStretch(1)
        scroll_area.setWidget(content)
        layout.addWidget(scroll_area)
        return group

    def _build_detection_group(self) -> QWidget:
        wrapper = QWidget()
        layout = QVBoxLayout(wrapper)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        # Method selector row
        method_row = QHBoxLayout()
        method_row.addWidget(QLabel("Method"))
        self.method_combo = QComboBox()
        self.method_combo.addItems(["Adaptive", "Canny", "HSV", "Model"])
        method_row.addWidget(self.method_combo, 1)
        layout.addLayout(method_row)

        # --- Adaptive params ---
        self._adaptive_widget = QWidget()
        ag = QGridLayout(self._adaptive_widget)
        ag.setContentsMargins(0, 0, 0, 0)
        self.adaptive_block_size_spin = self._make_spinbox(3, 99, 11)
        self.adaptive_c_spin = self._make_spinbox(-20, 20, 2)
        ag.addWidget(QLabel("Block size"), 0, 0)
        ag.addWidget(self.adaptive_block_size_spin, 0, 1)
        ag.addWidget(QLabel("C value"), 0, 2)
        ag.addWidget(self.adaptive_c_spin, 0, 3)
        layout.addWidget(self._adaptive_widget)

        # --- Canny params ---
        self._canny_widget = QWidget()
        cg = QGridLayout(self._canny_widget)
        cg.setContentsMargins(0, 0, 0, 0)
        self.canny_threshold1_spin = self._make_spinbox(0, 255, 50)
        self.canny_threshold2_spin = self._make_spinbox(0, 255, 150)
        cg.addWidget(QLabel("Threshold 1"), 0, 0)
        cg.addWidget(self.canny_threshold1_spin, 0, 1)
        cg.addWidget(QLabel("Threshold 2"), 0, 2)
        cg.addWidget(self.canny_threshold2_spin, 0, 3)
        layout.addWidget(self._canny_widget)

        # --- HSV params ---
        self._hsv_widget = QWidget()
        hg = QGridLayout(self._hsv_widget)
        hg.setContentsMargins(0, 0, 0, 0)
        self.hsv_lower_h_spin = self._make_spinbox(0, 180, 0)
        self.hsv_lower_s_spin = self._make_spinbox(0, 255, 0)
        self.hsv_lower_v_spin = self._make_spinbox(0, 255, 150)
        self.hsv_upper_h_spin = self._make_spinbox(0, 180, 180)
        self.hsv_upper_s_spin = self._make_spinbox(0, 255, 60)
        self.hsv_upper_v_spin = self._make_spinbox(0, 255, 255)
        hg.addWidget(QLabel("Lower H"), 0, 0)
        hg.addWidget(self.hsv_lower_h_spin, 0, 1)
        hg.addWidget(QLabel("S"), 0, 2)
        hg.addWidget(self.hsv_lower_s_spin, 0, 3)
        hg.addWidget(QLabel("V"), 0, 4)
        hg.addWidget(self.hsv_lower_v_spin, 0, 5)
        hg.addWidget(QLabel("Upper H"), 1, 0)
        hg.addWidget(self.hsv_upper_h_spin, 1, 1)
        hg.addWidget(QLabel("S"), 1, 2)
        hg.addWidget(self.hsv_upper_s_spin, 1, 3)
        hg.addWidget(QLabel("V"), 1, 4)
        hg.addWidget(self.hsv_upper_v_spin, 1, 5)
        layout.addWidget(self._hsv_widget)

        # --- Model params (inline, shown only when Model is selected) ---
        self._model_inline_widget = QWidget()
        mg = QGridLayout(self._model_inline_widget)
        mg.setContentsMargins(0, 0, 0, 0)
        self._model_inline_path = QLineEdit()
        self._model_inline_path.setPlaceholderText("assets/models/selongsong_yolov8n.onnx")
        self._model_inline_browse = QPushButton("Browse...")
        self._model_inline_browse.clicked.connect(self._browse_model_path)
        self._model_inline_conf = self._make_double_spinbox(0.0, 1.0, 0.5, 0.05)
        self._model_inline_iou = self._make_double_spinbox(0.0, 1.0, 0.45, 0.05)
        mg.addWidget(QLabel("Model path"), 0, 0)
        mg.addWidget(self._model_inline_path, 0, 1, 1, 2)
        mg.addWidget(self._model_inline_browse, 0, 3)
        mg.addWidget(QLabel("Conf threshold"), 1, 0)
        mg.addWidget(self._model_inline_conf, 1, 1)
        mg.addWidget(QLabel("IoU threshold"), 1, 2)
        mg.addWidget(self._model_inline_iou, 1, 3)
        layout.addWidget(self._model_inline_widget)

        # --- Common params (contour, morph — hidden for Model) ---
        self._common_params_widget = QWidget()
        cpg = QGridLayout(self._common_params_widget)
        cpg.setContentsMargins(0, 0, 0, 0)
        self.min_contour_area_spin = self._make_spinbox(10, 500000, 500)
        self.max_contour_area_spin = self._make_spinbox(100, 1000000, 50000)
        self.morph_kernel_size_spin = self._make_spinbox(1, 15, 3)
        cpg.addWidget(QLabel("Contour area"), 0, 0)
        cpg.addWidget(self.min_contour_area_spin, 0, 1)
        cpg.addWidget(self.max_contour_area_spin, 0, 2, 1, 2)
        cpg.addWidget(QLabel("Morph kernel"), 1, 0)
        cpg.addWidget(self.morph_kernel_size_spin, 1, 1)
        layout.addWidget(self._common_params_widget)

        # --- Image adjust (brightness, contrast, zoom — always visible) ---
        self._image_adjust_widget = QWidget()
        iag = QGridLayout(self._image_adjust_widget)
        iag.setContentsMargins(0, 0, 0, 0)
        self.brightness_spin = self._make_spinbox(-100, 100, 0)
        self.contrast_spin = self._make_double_spinbox(0.2, 3.0, 1.0, 0.1)
        self.zoom_spin = self._make_double_spinbox(1.0, 3.0, 1.0, 0.1)
        iag.addWidget(QLabel("Brightness"), 0, 0)
        iag.addWidget(self.brightness_spin, 0, 1)
        iag.addWidget(QLabel("Contrast"), 0, 2)
        iag.addWidget(self.contrast_spin, 0, 3)
        iag.addWidget(QLabel("Zoom"), 1, 0)
        iag.addWidget(self.zoom_spin, 1, 1)
        layout.addWidget(self._image_adjust_widget)

        # --- Tune / Apply / Save row ---
        self.tune_live_checkbox = QCheckBox("Tune Live")
        self.apply_detection_button = QPushButton("Apply Detection")
        self.save_detection_button = QPushButton("Save Detection")
        action_row = QHBoxLayout()
        action_row.addWidget(self.tune_live_checkbox)
        action_row.addStretch(1)
        action_row.addWidget(self.apply_detection_button)
        action_row.addWidget(self.save_detection_button)
        layout.addLayout(action_row)

        # Connect method combo to show/hide
        self.method_combo.currentTextChanged.connect(self._on_detection_method_changed)
        self._on_detection_method_changed(self.method_combo.currentText())

        return wrapper

    def _on_detection_method_changed(self, method: str) -> None:
        """Show only the config widgets relevant to the selected detection method."""
        m = method.lower()
        self._adaptive_widget.setVisible(m == "adaptive")
        self._canny_widget.setVisible(m == "canny")
        self._hsv_widget.setVisible(m == "hsv")
        self._model_inline_widget.setVisible(m == "model")
        # Common image params are relevant for non-model methods
        self._common_params_widget.setVisible(m != "model")

    def _build_object_info_group(self) -> QGroupBox:
        group = QGroupBox("OBJECT INFO")
        layout = QFormLayout(group)
        self.object_status_value.setWordWrap(True)
        self.bbox_value.setWordWrap(True)
        self.centroid_value.setWordWrap(True)
        self.dimension_value.setWordWrap(True)
        self.pick_point_value.setWordWrap(True)
        layout.addRow("Status", self.object_status_value)
        layout.addRow("Bounding box", self.bbox_value)
        layout.addRow("Centroid", self.centroid_value)
        layout.addRow("Dimensions", self.dimension_value)
        layout.addRow("Pick point", self.pick_point_value)
        layout.addRow("Orientation", self.orientation_value)
        return group

    def _build_pick_zone_group(self) -> QGroupBox:
        group = QGroupBox("SAFE PICK ZONE")
        layout = QGridLayout(group)
        self.pick_inset_x_spin = self._make_double_spinbox(0.0, 100.0, 10.0, 0.5)
        self.pick_inset_y_spin = self._make_double_spinbox(0.0, 100.0, 8.0, 0.5)
        self.safe_pick_margin_pct_spin = self._make_double_spinbox(0.0, 1.0, 0.15, 0.01)
        self.marginal_timeout_spin = self._make_double_spinbox(1.0, 30.0, 2.0, 0.5)
        self.preview_pick_checkbox = QCheckBox("Preview Pick Zone")
        self.preview_pick_checkbox.setChecked(True)
        self.save_pick_zone_button = QPushButton("Save Pick Zone")
        layout.addWidget(QLabel("Inset X (mm)"), 0, 0)
        layout.addWidget(self.pick_inset_x_spin, 0, 1)
        layout.addWidget(QLabel("Inset Y (mm)"), 1, 0)
        layout.addWidget(self.pick_inset_y_spin, 1, 1)
        layout.addWidget(QLabel("Safety margin (%)"), 2, 0)
        layout.addWidget(self.safe_pick_margin_pct_spin, 2, 1)
        layout.addWidget(QLabel("Marginal timeout (s)"), 3, 0)
        layout.addWidget(self.marginal_timeout_spin, 3, 1)
        layout.addWidget(self.preview_pick_checkbox, 4, 0, 1, 2)
        layout.addWidget(self.save_pick_zone_button, 5, 0, 1, 2)
        return group

    def _build_workspace_group(self) -> QGroupBox:
        group = QGroupBox("WORKSPACE")
        layout = QGridLayout(group)
        self.workspace_x_min_spin = self._make_double_spinbox(-1000.0, 1000.0, -150.0, 1.0)
        self.workspace_x_max_spin = self._make_double_spinbox(-1000.0, 1000.0, 150.0, 1.0)
        self.workspace_y_min_spin = self._make_double_spinbox(-1000.0, 1000.0, -100.0, 1.0)
        self.workspace_y_max_spin = self._make_double_spinbox(-1000.0, 1000.0, 100.0, 1.0)
        self.workspace_z_spin = self._make_double_spinbox(1.0, 2000.0, 200.0, 1.0)
        self.workspace_margin_spin = self._make_double_spinbox(0.0, 200.0, 10.0, 0.5)
        self.save_workspace_button = QPushButton("Save Workspace")
        layout.addWidget(QLabel("X min"), 0, 0)
        layout.addWidget(self.workspace_x_min_spin, 0, 1)
        layout.addWidget(QLabel("X max"), 0, 2)
        layout.addWidget(self.workspace_x_max_spin, 0, 3)
        layout.addWidget(QLabel("Y min"), 1, 0)
        layout.addWidget(self.workspace_y_min_spin, 1, 1)
        layout.addWidget(QLabel("Y max"), 1, 2)
        layout.addWidget(self.workspace_y_max_spin, 1, 3)
        layout.addWidget(QLabel("Z fixed"), 2, 0)
        layout.addWidget(self.workspace_z_spin, 2, 1)
        layout.addWidget(QLabel("Margin"), 2, 2)
        layout.addWidget(self.workspace_margin_spin, 2, 3)
        layout.addWidget(self.save_workspace_button, 3, 0, 1, 4)
        return group

    def _build_calibration_group(self) -> QGroupBox:
        group = QGroupBox("CAMERA CALIBRATION")
        layout = QGridLayout(group)
        self.calibration_chessboard_x_spin = self._make_spinbox(3, 20, 9)
        self.calibration_chessboard_y_spin = self._make_spinbox(3, 20, 6)
        self.calibration_square_size_spin = self._make_double_spinbox(1.0, 100.0, 25.0, 0.5)
        self.snap_frame_button = QPushButton("Snap Frame")
        self.run_calibration_button = QPushButton("Run Chessboard Calibration")
        self.save_calibration_button = QPushButton("Save Calibration")
        layout.addWidget(QLabel("Chessboard X"), 0, 0)
        layout.addWidget(self.calibration_chessboard_x_spin, 0, 1)
        layout.addWidget(QLabel("Chessboard Y"), 0, 2)
        layout.addWidget(self.calibration_chessboard_y_spin, 0, 3)
        layout.addWidget(QLabel("Square size (mm)"), 1, 0)
        layout.addWidget(self.calibration_square_size_spin, 1, 1)
        layout.addWidget(QLabel("Snapshots"), 1, 2)
        layout.addWidget(self.snapshot_value, 1, 3)
        layout.addWidget(self.snap_frame_button, 2, 0, 1, 2)
        layout.addWidget(self.run_calibration_button, 2, 2, 1, 2)
        layout.addWidget(QLabel("Intrinsic"), 3, 0)
        layout.addWidget(self.calibration_value, 3, 1, 1, 3)
        layout.addWidget(QLabel("Distortion"), 4, 0)
        layout.addWidget(self.distortion_value, 4, 1, 1, 3)
        layout.addWidget(self.save_calibration_button, 5, 0, 1, 4)
        return group

    def _build_model_config_group(self) -> QWidget:
        wrapper = QWidget()
        layout = QGridLayout(wrapper)
        layout.setContentsMargins(0, 0, 0, 0)
        # Point canonical attributes to the inline widgets created in detection group
        self.model_path_edit = self._model_inline_path
        self.model_browse_button = self._model_inline_browse
        self.model_conf_spin = self._model_inline_conf
        self.model_iou_spin = self._model_inline_iou
        self.model_load_button = QPushButton("Load Model")
        self.model_load_button.setProperty("role", "primary")
        self.model_save_button = QPushButton("Save Config")
        self.model_status_label.setStyleSheet("color: #8A9AB8; font-weight: bold;")

        layout.addWidget(self.model_load_button, 0, 0, 1, 2)
        layout.addWidget(QLabel("Status"), 0, 2)
        layout.addWidget(self.model_status_label, 0, 3)
        layout.addWidget(self.model_save_button, 1, 0, 1, 4)
        return wrapper

    def _browse_model_path(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Select ONNX Model", "", "ONNX Model (*.onnx);;All Files (*)"
        )
        if path:
            if self.model_path_edit is not None:
                self.model_path_edit.setText(path)

    def build_model_settings(self) -> dict:
        return {
            "model_path": self.model_path_edit.text().strip() if self.model_path_edit else "",
            "model_conf_threshold": float(self.model_conf_spin.value()) if self.model_conf_spin else 0.5,
            "model_iou_threshold": float(self.model_iou_spin.value()) if self.model_iou_spin else 0.45,
        }

    def update_model_status(self, status: str) -> None:
        color_map = {
            "LOADED": "#7AC922",
            "NOT LOADED": "#8A9AB8",
        }
        color = color_map.get(status, "#E84040")
        self.model_status_label.setText(status)
        self.model_status_label.setStyleSheet(f"color: {color}; font-weight: bold;")

    # ---- Safety badge (4.9) ----

    def update_safety_badge(self, status: str) -> None:
        badge_config = {
            "SAFE": ("● SAFE", "#7AC922", "rgba(122,201,34,30)"),
            "MARGINAL": ("● MARGINAL", "#F5A623", "rgba(245,166,35,30)"),
            "UNSAFE": ("● UNSAFE", "#E84040", "rgba(232,64,64,30)"),
            "UNKNOWN": ("● UNKNOWN", "#8A9AB8", "rgba(138,154,184,30)"),
        }
        text, color, bg = badge_config.get(status, ("", "#8A9AB8", "transparent"))
        if text:
            self.safety_badge.setText(text)
            self.safety_badge.setStyleSheet(
                f"background: {bg}; color: {color}; font-weight: bold; font-size: 13px; "
                f"border: 1px solid {color}; border-radius: 4px; padding: 2px 10px;"
            )
            self.safety_badge.show()
        else:
            self.safety_badge.hide()

    # ---- MARGINAL confirmation (4.10) ----

    def show_marginal_confirmation(self, timeout_s: int = 10) -> None:
        self._marginal_remaining_s = max(1, timeout_s)
        self._update_marginal_countdown()
        self._marginal_widget.show()
        self._marginal_timer.start()

    def hide_marginal_confirmation(self) -> None:
        self._marginal_timer.stop()
        self._marginal_widget.hide()

    def _on_confirm_pick(self) -> None:
        self.hide_marginal_confirmation()
        self.confirm_pick_signal.emit()

    def _on_skip_pick(self) -> None:
        self.hide_marginal_confirmation()
        self.skip_pick_signal.emit()

    def _marginal_tick(self) -> None:
        self._marginal_remaining_s -= 1
        if self._marginal_remaining_s <= 0:
            self.hide_marginal_confirmation()
            self.skip_pick_signal.emit()  # auto-skip on timeout
            return
        self._update_marginal_countdown()

    def _update_marginal_countdown(self) -> None:
        self._marginal_countdown_label.setText(
            f"MARGINAL — Auto-skip in {self._marginal_remaining_s}s"
        )

    def apply_config(self, config: dict) -> None:
        vision = config.get("vision", {})
        workspace = config.get("workspace", {})
        if self.camera_source_combo is not None:
            source_value = str(vision.get("video_source", vision.get("camera_index", 0)))
            idx = self.camera_source_combo.findData(source_value)
            if idx >= 0:
                self.camera_source_combo.setCurrentIndex(idx)
            else:
                self.camera_source_combo.addItem(f"Camera ({source_value})", source_value)
                self.camera_source_combo.setCurrentIndex(self.camera_source_combo.count() - 1)
        if self.method_combo is not None:
            method_map = {
                "adaptive": "Adaptive",
                "canny": "Canny",
                "hsv": "HSV",
                "model": "Model",
            }
            self.method_combo.setCurrentText(method_map.get(str(vision.get("detection_method", "adaptive")).lower(), "Adaptive"))
        self.adaptive_block_size_spin.setValue(int(vision.get("adaptive_block_size", 11)))
        self.adaptive_c_spin.setValue(int(vision.get("adaptive_c", 2)))
        self.canny_threshold1_spin.setValue(int(vision.get("canny_threshold1", 50)))
        self.canny_threshold2_spin.setValue(int(vision.get("canny_threshold2", 150)))
        hsv_lower = vision.get("hsv_lower", [0, 0, 150])
        hsv_upper = vision.get("hsv_upper", [180, 60, 255])
        self.hsv_lower_h_spin.setValue(int(hsv_lower[0]))
        self.hsv_lower_s_spin.setValue(int(hsv_lower[1]))
        self.hsv_lower_v_spin.setValue(int(hsv_lower[2]))
        self.hsv_upper_h_spin.setValue(int(hsv_upper[0]))
        self.hsv_upper_s_spin.setValue(int(hsv_upper[1]))
        self.hsv_upper_v_spin.setValue(int(hsv_upper[2]))
        self.min_contour_area_spin.setValue(int(vision.get("min_contour_area", 500)))
        self.max_contour_area_spin.setValue(int(vision.get("max_contour_area", 50000)))
        self.morph_kernel_size_spin.setValue(int(vision.get("morph_kernel_size", 3)))
        self.brightness_spin.setValue(int(vision.get("brightness", 0)))
        self.contrast_spin.setValue(float(vision.get("contrast", 1.0)))
        self.zoom_spin.setValue(float(vision.get("zoom", 1.0)))
        self.tune_live_checkbox.setChecked(bool(vision.get("tune_live", False)))
        self.pick_inset_x_spin.setValue(float(vision.get("pick_inset_x_mm", 10.0)))
        self.pick_inset_y_spin.setValue(float(vision.get("pick_inset_y_mm", 8.0)))
        self.preview_pick_checkbox.setChecked(bool(vision.get("preview_pick_zone", True)))
        if self.safe_pick_margin_pct_spin is not None:
            self.safe_pick_margin_pct_spin.setValue(float(vision.get("safe_pick_margin_pct", 0.15)))
        if self.marginal_timeout_spin is not None:
            self.marginal_timeout_spin.setValue(float(vision.get("marginal_confirm_timeout_s", 2.0)))
        self.workspace_x_min_spin.setValue(float(workspace.get("x_min_mm", -150.0)))
        self.workspace_x_max_spin.setValue(float(workspace.get("x_max_mm", 150.0)))
        self.workspace_y_min_spin.setValue(float(workspace.get("y_min_mm", -100.0)))
        self.workspace_y_max_spin.setValue(float(workspace.get("y_max_mm", 100.0)))
        self.workspace_z_spin.setValue(float(workspace.get("z_fixed_mm", 200.0)))
        self.workspace_margin_spin.setValue(float(workspace.get("margin_mm", 10.0)))
        calibration = vision.get("calibration", {})
        chessboard = calibration.get("chessboard_size", [9, 6])
        self.calibration_chessboard_x_spin.setValue(int(chessboard[0]))
        self.calibration_chessboard_y_spin.setValue(int(chessboard[1]))
        self.calibration_square_size_spin.setValue(float(calibration.get("square_size_mm", 25.0)))
        self.update_calibration(calibration)
        # Model config
        if self.model_path_edit is not None:
            self.model_path_edit.setText(str(vision.get("model_path", "")))
        if self.model_conf_spin is not None:
            self.model_conf_spin.setValue(float(vision.get("model_conf_threshold", 0.5)))
        if self.model_iou_spin is not None:
            self.model_iou_spin.setValue(float(vision.get("model_iou_threshold", 0.45)))

    def selected_camera_source(self) -> str:
        if self.camera_source_combo is None:
            return "0"
        data = self.camera_source_combo.currentData()
        if data is not None:
            return str(data)
        return self.camera_source_combo.currentText().strip()

    def build_detection_settings(self) -> dict:
        return {
            "video_source": self.selected_camera_source(),
            "detection_method": self.method_combo.currentText().strip().lower(),
            "adaptive_block_size": int(self.adaptive_block_size_spin.value()),
            "adaptive_c": int(self.adaptive_c_spin.value()),
            "canny_threshold1": int(self.canny_threshold1_spin.value()),
            "canny_threshold2": int(self.canny_threshold2_spin.value()),
            "hsv_lower": [
                int(self.hsv_lower_h_spin.value()),
                int(self.hsv_lower_s_spin.value()),
                int(self.hsv_lower_v_spin.value()),
            ],
            "hsv_upper": [
                int(self.hsv_upper_h_spin.value()),
                int(self.hsv_upper_s_spin.value()),
                int(self.hsv_upper_v_spin.value()),
            ],
            "min_contour_area": int(self.min_contour_area_spin.value()),
            "max_contour_area": int(self.max_contour_area_spin.value()),
            "morph_kernel_size": int(self.morph_kernel_size_spin.value()),
            "brightness": int(self.brightness_spin.value()),
            "contrast": float(self.contrast_spin.value()),
            "zoom": float(self.zoom_spin.value()),
            "tune_live": bool(self.tune_live_checkbox.isChecked()),
            "preview_pick_zone": bool(self.preview_pick_checkbox.isChecked()),
            "pick_inset_x_mm": float(self.pick_inset_x_spin.value()),
            "pick_inset_y_mm": float(self.pick_inset_y_spin.value()),
        }

    def build_pick_zone_settings(self) -> dict:
        return {
            "pick_inset_x_mm": float(self.pick_inset_x_spin.value()),
            "pick_inset_y_mm": float(self.pick_inset_y_spin.value()),
            "preview_pick_zone": bool(self.preview_pick_checkbox.isChecked()),
            "safe_pick_margin_pct": float(self.safe_pick_margin_pct_spin.value()) if self.safe_pick_margin_pct_spin else 0.15,
            "marginal_confirm_timeout_s": float(self.marginal_timeout_spin.value()) if self.marginal_timeout_spin else 2.0,
        }

    def build_workspace_settings(self) -> dict:
        return {
            "x_min_mm": float(self.workspace_x_min_spin.value()),
            "x_max_mm": float(self.workspace_x_max_spin.value()),
            "y_min_mm": float(self.workspace_y_min_spin.value()),
            "y_max_mm": float(self.workspace_y_max_spin.value()),
            "z_fixed_mm": float(self.workspace_z_spin.value()),
            "margin_mm": float(self.workspace_margin_spin.value()),
        }

    def calibration_request(self) -> tuple[tuple[int, int], float]:
        return (
            (int(self.calibration_chessboard_x_spin.value()), int(self.calibration_chessboard_y_spin.value())),
            float(self.calibration_square_size_spin.value()),
        )

    def update_frame(self, frame) -> None:
        if frame is None:
            self.live_feed_label.setText("No frame")
            return
        if len(frame.shape) != 3:
            self.live_feed_label.setText("Invalid frame")
            return
        rgb = frame[:, :, ::-1].copy()
        height, width, channels = rgb.shape
        image = QImage(rgb.data, width, height, channels * width, QImage.Format.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(image)
        scaled = pixmap.scaled(
            self.live_feed_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.live_feed_label.setPixmap(scaled)

    def update_detection(self, payload: dict) -> None:
        if not payload or payload.get("status") == "no_object":
            self.object_status_value.setText("NO OBJECT")
            self.bbox_value.setText("-")
            self.centroid_value.setText("-")
            self.dimension_value.setText("-")
            self.pick_point_value.setText("-")
            self.orientation_value.setText("-")
            return
        bbox = payload.get("bbox_px", [0, 0, 0, 0])
        centroid_px = payload.get("centroid_px", [0, 0])
        centroid_world = payload.get("centroid_world", [0.0, 0.0])
        pick_world = payload.get("pick_point_world", [0.0, 0.0])
        self.object_status_value.setText(payload.get("workspace_message", payload.get("workspace_status", "-")))
        self.bbox_value.setText(f"({bbox[0]}, {bbox[1]}) -> ({bbox[0] + bbox[2]}, {bbox[1] + bbox[3]})")
        self.centroid_value.setText(
            f"px=({centroid_px[0]}, {centroid_px[1]}) | world=({centroid_world[0]:.1f}, {centroid_world[1]:.1f}) mm"
        )
        self.dimension_value.setText(
            f"W={payload.get('width_mm', 0.0):.1f} mm | H={payload.get('height_mm', 0.0):.1f} mm"
        )
        self.pick_point_value.setText(f"({pick_world[0]:.1f}, {pick_world[1]:.1f}) mm")
        self.orientation_value.setText(f"{payload.get('orientation_deg', 0.0):.1f} deg")

    def update_status(self, level: str, message: str) -> None:
        palette = {
            "info": "#E8EDF5",
            "warn": "#F5A623",
            "error": "#E84040",
        }
        self.status_value.setText(message)
        self.status_value.setStyleSheet(f"color: {palette.get(level, '#E8EDF5')}; font-weight: 600;")

    def update_calibration(self, payload: dict) -> None:
        fx = float(payload.get("fx", 0.0) or 0.0)
        fy = float(payload.get("fy", 0.0) or 0.0)
        cx = float(payload.get("cx", 0.0) or 0.0)
        cy = float(payload.get("cy", 0.0) or 0.0)
        self.calibration_value.setText(f"fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}")
        distortion = payload.get("distortion", [])
        if isinstance(distortion, list):
            distortion_text = ", ".join(str(value) for value in distortion[:5]) or "-"
        else:
            distortion_text = str(distortion)
        self.distortion_value.setText(distortion_text)
        self.snapshot_value.setText(str(payload.get("snapshots_captured", 0)))

    @staticmethod
    def _make_spinbox(minimum: int, maximum: int, value: int) -> QSpinBox:
        widget = QSpinBox()
        widget.setRange(minimum, maximum)
        widget.setValue(value)
        return widget

    @staticmethod
    def _make_double_spinbox(
        minimum: float,
        maximum: float,
        value: float,
        step: float,
    ) -> QDoubleSpinBox:
        widget = QDoubleSpinBox()
        widget.setRange(minimum, maximum)
        widget.setDecimals(2)
        widget.setSingleStep(step)
        widget.setValue(value)
        return widget
