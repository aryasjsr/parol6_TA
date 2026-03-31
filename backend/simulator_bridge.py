"""Matplotlib-based 3D robot visualisation widget for PyQt6.

Embeds a ``FigureCanvasQTAgg`` that renders the PAROL6 6-DOF arm using
forward kinematics from *roboticstoolbox*.  No web server or browser
needed – the 3D plot lives directly inside the Qt widget tree.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 – registers '3d' projection
from spatialmath import SE3

if TYPE_CHECKING:
    from roboticstoolbox import DHRobot

logger = logging.getLogger(__name__)

# Axis limits (metres) – generous enough for PAROL6 workspace
_LIM = 0.45

# Distinct colours per joint position (J1 at base through EE)
# idx 0 = J1 (base, axis of rotation), idx 1 = J2, ... idx 5 = J6, idx 6 = EE
_JOINT_COLORS = [
    "#FF6B6B",  # J1 – red       (at base origin)
    "#4ECDC4",  # J2 – teal
    "#F5A623",  # J3 – orange
    "#A78BFA",  # J4 – purple
    "#38BDF8",  # J5 – sky-blue
    "#FB7185",  # J6 – pink
    "#22C97A",  # EE – green
]
_JOINT_LABELS = ["J1", "J2", "J3", "J4", "J5", "J6", "EE"]


class RobotCanvas(FigureCanvas):
    """A Qt-embeddable matplotlib canvas that draws a 6-DOF robot arm."""

    def __init__(self, robot: "DHRobot", parent=None, width: float = 8, height: float = 7, dpi: int = 100) -> None:
        self._robot = robot
        fig = Figure(figsize=(width, height), dpi=dpi)
        fig.patch.set_facecolor("#0B1120")
        fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
        super().__init__(fig)
        self.setParent(parent)

        self._ax: Axes3D = fig.add_subplot(111, projection="3d")
        self._ax.set_box_aspect([1.2, 1.2, 1])
        self._zoom = 1.0
        self._configure_axes()

        self._q = np.zeros(len(robot.links))
        self._line = None
        self._joint_scatters: list = []
        self._joint_labels: list = []
        self._joint_label_offsets: list = []  # z-offset per label
        self._draw_robot()

        # Enable scroll-wheel zoom
        self.mpl_connect("scroll_event", self._on_scroll)

    # ------------------------------------------------------------------
    def _configure_axes(self) -> None:
        ax = self._ax
        ax.set_facecolor("#0B1120")
        self._apply_zoom()
        ax.set_xlabel("X (m)", color="#8A9AB8", fontsize=8)
        ax.set_ylabel("Y (m)", color="#8A9AB8", fontsize=8)
        ax.set_zlabel("Z (m)", color="#8A9AB8", fontsize=8)
        ax.tick_params(colors="#5A6A88", labelsize=6)
        for spine in ax.xaxis.get_gridlines() + ax.yaxis.get_gridlines() + ax.zaxis.get_gridlines():
            spine.set_color("#1E2A45")
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor("#1E2A45")
        ax.yaxis.pane.set_edgecolor("#1E2A45")
        ax.zaxis.pane.set_edgecolor("#1E2A45")

    def _apply_zoom(self) -> None:
        """Set axis limits based on current zoom level."""
        lim = _LIM / self._zoom
        self._ax.set_xlim(-lim, lim)
        self._ax.set_ylim(-lim, lim)
        self._ax.set_zlim(-0.05 / self._zoom, lim)

    def _on_scroll(self, event) -> None:
        """Zoom in/out on mouse scroll wheel."""
        if event.button == "up":
            self._zoom = min(self._zoom * 1.15, 5.0)
        elif event.button == "down":
            self._zoom = max(self._zoom / 1.15, 0.3)
        self._apply_zoom()
        self.draw_idle()

    # ------------------------------------------------------------------
    def _fk_positions(self) -> np.ndarray:
        """Return (N+1, 3) array of joint positions via forward kinematics."""
        positions = [np.array([0.0, 0.0, 0.0])]
        T = SE3()
        for i, link in enumerate(self._robot.links):
            T = T * link.A(self._q[i])
            positions.append(T.t.copy())
        return np.array(positions)

    def _draw_robot(self) -> None:
        pts = self._fk_positions()
        xs, ys, zs = pts[:, 0], pts[:, 1], pts[:, 2]
        if self._line is None:
            # Draw link segments
            self._line, = self._ax.plot(xs, ys, zs, "-",
                                         color="#F5A623", linewidth=2.5,
                                         zorder=5)
            # Draw each joint as a distinct coloured marker with label
            for idx in range(len(pts)):
                c = _JOINT_COLORS[idx] if idx < len(_JOINT_COLORS) else "#FFFFFF"
                lbl = _JOINT_LABELS[idx] if idx < len(_JOINT_LABELS) else f"J{idx}"

                # Detect coincident joint (same position as previous)
                coincident = False
                if idx > 0:
                    dist = np.linalg.norm(pts[idx] - pts[idx - 1])
                    if dist < 1e-4:
                        coincident = True

                if idx == 0:
                    marker, sz = "^", 80
                elif coincident:
                    marker, sz = "D", 40  # diamond for coincident
                else:
                    marker, sz = "o", 60

                z_off = 0.025 if coincident else 0.015

                sc = self._ax.scatter(
                    [xs[idx]], [ys[idx]], [zs[idx]],
                    color=c, s=sz,
                    marker=marker,
                    edgecolors="#FFFFFF", linewidths=0.8,
                    zorder=6,
                )
                txt = self._ax.text(
                    xs[idx], ys[idx], zs[idx] + z_off,
                    lbl, color=c, fontsize=7, fontweight="bold",
                    ha="center", va="bottom", zorder=7,
                )
                self._joint_scatters.append(sc)
                self._joint_labels.append(txt)
                self._joint_label_offsets.append(z_off)
        else:
            self._line.set_data_3d(xs, ys, zs)
            for idx, (sc, txt) in enumerate(zip(self._joint_scatters, self._joint_labels)):
                sc._offsets3d = ([xs[idx]], [ys[idx]], [zs[idx]])
                z_off = self._joint_label_offsets[idx]
                txt.set_position((xs[idx], ys[idx]))
                txt.set_3d_properties(zs[idx] + z_off, zdir="z")
        self.draw_idle()

    # ------------------------------------------------------------------
    def update_joints(self, q) -> None:
        """Update the displayed joint configuration (radians)."""
        q_arr = np.asarray(q, dtype=float)
        if q_arr.shape[0] != len(self._robot.links):
            return
        self._q = q_arr
        self._draw_robot()

    def set_view_angle(self, elev: float, azim: float) -> None:
        """Set the camera elevation and azimuth angles (degrees)."""
        self._ax.view_init(elev=elev, azim=azim)
        self.draw_idle()
