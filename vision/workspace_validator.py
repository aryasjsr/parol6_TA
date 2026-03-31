from __future__ import annotations

import math

import cv2
import numpy as np

from backend.config_manager import ConfigManager


class WorkspaceValidator:
    class ValidationResult:
        VALID = "valid"
        OUT_OF_BOUNDS = "out"
        NEAR_BOUNDARY = "margin"

    def __init__(self, config: ConfigManager | None = None) -> None:
        self._config = config or ConfigManager.instance()

    def check(self, x_mm: float, y_mm: float) -> tuple[str, str]:
        workspace = self._config.get("workspace", {})
        x_min = float(workspace.get("x_min_mm", -150.0))
        x_max = float(workspace.get("x_max_mm", 150.0))
        y_min = float(workspace.get("y_min_mm", -100.0))
        y_max = float(workspace.get("y_max_mm", 100.0))
        margin = float(workspace.get("margin_mm", 10.0))

        if x_mm < x_min or x_mm > x_max or y_mm < y_min or y_mm > y_max:
            return self.ValidationResult.OUT_OF_BOUNDS, "OUT OF WORKSPACE"

        inner_x_min = x_min + margin
        inner_x_max = x_max - margin
        inner_y_min = y_min + margin
        inner_y_max = y_max - margin
        if x_mm < inner_x_min or x_mm > inner_x_max or y_mm < inner_y_min or y_mm > inner_y_max:
            return self.ValidationResult.NEAR_BOUNDARY, "NEAR BOUNDARY"

        return self.ValidationResult.VALID, "READY TO PICK"

    @staticmethod
    def pixel_to_world(u: float, v: float, intrinsic_matrix: np.ndarray, z_mm: float) -> tuple[float, float]:
        fx = float(intrinsic_matrix[0, 0])
        fy = float(intrinsic_matrix[1, 1])
        cx = float(intrinsic_matrix[0, 2])
        cy = float(intrinsic_matrix[1, 2])
        x_mm = (float(u) - cx) * float(z_mm) / fx
        y_mm = (float(v) - cy) * float(z_mm) / fy
        return x_mm, y_mm

    @staticmethod
    def world_to_pixel(x_mm: float, y_mm: float, intrinsic_matrix: np.ndarray, z_mm: float) -> tuple[int, int]:
        fx = float(intrinsic_matrix[0, 0])
        fy = float(intrinsic_matrix[1, 1])
        cx = float(intrinsic_matrix[0, 2])
        cy = float(intrinsic_matrix[1, 2])
        u = int(round(float(x_mm) * fx / float(z_mm) + cx))
        v = int(round(float(y_mm) * fy / float(z_mm) + cy))
        return u, v

    def draw_overlay(self, frame: np.ndarray, intrinsic_matrix: np.ndarray, z_mm: float) -> np.ndarray:
        overlay = frame.copy()
        workspace = self._config.get("workspace", {})
        x_min = float(workspace.get("x_min_mm", -150.0))
        x_max = float(workspace.get("x_max_mm", 150.0))
        y_min = float(workspace.get("y_min_mm", -100.0))
        y_max = float(workspace.get("y_max_mm", 100.0))
        margin = float(workspace.get("margin_mm", 10.0))

        outer_top_left = self.world_to_pixel(x_min, y_min, intrinsic_matrix, z_mm)
        outer_bottom_right = self.world_to_pixel(x_max, y_max, intrinsic_matrix, z_mm)
        inner_top_left = self.world_to_pixel(x_min + margin, y_min + margin, intrinsic_matrix, z_mm)
        inner_bottom_right = self.world_to_pixel(x_max - margin, y_max - margin, intrinsic_matrix, z_mm)

        self._fill_margin_zone(overlay, outer_top_left, outer_bottom_right, inner_top_left, inner_bottom_right)
        self._draw_dashed_rect(overlay, outer_top_left, outer_bottom_right, (217, 144, 73), 2)
        self._draw_dashed_rect(overlay, inner_top_left, inner_bottom_right, (75, 168, 200), 1)
        return overlay

    def _fill_margin_zone(
        self,
        frame: np.ndarray,
        outer_top_left: tuple[int, int],
        outer_bottom_right: tuple[int, int],
        inner_top_left: tuple[int, int],
        inner_bottom_right: tuple[int, int],
    ) -> None:
        shade = frame.copy()
        x1, y1 = self._normalize_rect(outer_top_left, outer_bottom_right)
        x2, y2 = self._normalize_rect(inner_top_left, inner_bottom_right)
        cv2.rectangle(shade, x1, y1, (75, 168, 200), 1)
        cv2.rectangle(shade, x1, y1, (75, 168, 200), -1)
        cv2.rectangle(shade, x2, y2, (0, 0, 0), -1)
        cv2.addWeighted(shade, 0.12, frame, 0.88, 0.0, dst=frame)

    @staticmethod
    def _normalize_rect(
        top_left: tuple[int, int],
        bottom_right: tuple[int, int],
    ) -> tuple[tuple[int, int], tuple[int, int]]:
        return (
            (min(top_left[0], bottom_right[0]), min(top_left[1], bottom_right[1])),
            (max(top_left[0], bottom_right[0]), max(top_left[1], bottom_right[1])),
        )

    def _draw_dashed_rect(
        self,
        frame: np.ndarray,
        top_left: tuple[int, int],
        bottom_right: tuple[int, int],
        color: tuple[int, int, int],
        thickness: int,
    ) -> None:
        start, end = self._normalize_rect(top_left, bottom_right)
        points = [
            (start[0], start[1]),
            (end[0], start[1]),
            (end[0], end[1]),
            (start[0], end[1]),
        ]
        for idx in range(4):
            self._draw_dashed_line(frame, points[idx], points[(idx + 1) % 4], color, thickness)

    @staticmethod
    def _draw_dashed_line(
        frame: np.ndarray,
        start: tuple[int, int],
        end: tuple[int, int],
        color: tuple[int, int, int],
        thickness: int,
        dash_px: int = 10,
        gap_px: int = 6,
    ) -> None:
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        distance = math.hypot(dx, dy)
        if distance <= 0:
            return
        step = dash_px + gap_px
        for offset in np.arange(0, distance, step):
            next_offset = min(offset + dash_px, distance)
            p1 = (
                int(round(start[0] + dx * offset / distance)),
                int(round(start[1] + dy * offset / distance)),
            )
            p2 = (
                int(round(start[0] + dx * next_offset / distance)),
                int(round(start[1] + dy * next_offset / distance)),
            )
            cv2.line(frame, p1, p2, color, thickness, lineType=cv2.LINE_AA)
