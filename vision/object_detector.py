from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import cv2
import numpy as np

from backend.config_manager import ConfigManager
from vision.workspace_validator import WorkspaceValidator


@dataclass(slots=True)
class DetectionResult:
    bbox_px: tuple[int, int, int, int]
    centroid_px: tuple[int, int]
    contour: np.ndarray
    orientation_deg: float
    centroid_world: tuple[float, float]
    width_mm: float
    height_mm: float
    pick_point_px: tuple[int, int]
    pick_point_world: tuple[float, float]
    workspace_status: str
    safe_zone_px: tuple[int, int, int, int]
    workspace_message: str = "NO OBJECT"

    def as_dict(self) -> dict:
        return {
            "bbox_px": list(self.bbox_px),
            "centroid_px": list(self.centroid_px),
            "orientation_deg": round(float(self.orientation_deg), 3),
            "centroid_world": [round(float(value), 3) for value in self.centroid_world],
            "width_mm": round(float(self.width_mm), 3),
            "height_mm": round(float(self.height_mm), 3),
            "pick_point_px": list(self.pick_point_px),
            "pick_point_world": [round(float(value), 3) for value in self.pick_point_world],
            "workspace_status": self.workspace_status,
            "workspace_message": self.workspace_message,
            "safe_zone_px": list(self.safe_zone_px),
        }


@dataclass(slots=True)
class DetectionBundle:
    """Result from model-based 2-class detection (selongsong + fixture)."""

    selongsong_box: np.ndarray | None  # [x1,y1,x2,y2] pixel, frame coords
    fixture_box: np.ndarray | None  # [x1,y1,x2,y2] pixel, frame coords
    pick_point_px: tuple[int, int] | None  # (u,v) safe pick point in pixel
    pick_point_world: tuple[float, float] | None  # Legacy name: (X_base_mm, Y_base_mm)
    pick_safety: str = "UNKNOWN"  # "SAFE" | "MARGINAL" | "UNSAFE" | "UNKNOWN"
    conf_selongsong: float = 0.0  # confidence score class 0
    conf_fixture: float = 0.0  # confidence score class 1 (0.0 if not found)

    def as_dict(self) -> dict:
        pick_point_base = (
            [round(v, 3) for v in self.pick_point_world]
            if self.pick_point_world is not None
            else None
        )
        return {
            "selongsong_box": self.selongsong_box.tolist() if self.selongsong_box is not None else None,
            "fixture_box": self.fixture_box.tolist() if self.fixture_box is not None else None,
            "pick_point_px": list(self.pick_point_px) if self.pick_point_px is not None else None,
            "pick_point_base": pick_point_base,
            "pick_point_world": pick_point_base,
            "pick_safety": self.pick_safety,
            "conf_selongsong": round(self.conf_selongsong, 3),
            "conf_fixture": round(self.conf_fixture, 3),
        }


class BaseDetector(ABC):
    """Abstract base class for all detectors."""

    def __init__(
        self,
        config: ConfigManager | None = None,
        workspace_validator: WorkspaceValidator | None = None,
    ) -> None:
        self._config = config or ConfigManager.instance()
        self._workspace_validator = workspace_validator or WorkspaceValidator(self._config)

    @abstractmethod
    def detect(
        self,
        frame: np.ndarray,
        intrinsic_matrix: np.ndarray,
        z_mm: float,
    ) -> tuple[DetectionResult | None, np.ndarray]:
        ...


class ObjectDetector(BaseDetector):
    """Contour-based detector (adaptive / canny / hsv)."""

    def __init__(
        self,
        config: ConfigManager | None = None,
        workspace_validator: WorkspaceValidator | None = None,
    ) -> None:
        super().__init__(config, workspace_validator)

    def detect(
        self,
        frame: np.ndarray,
        intrinsic_matrix: np.ndarray,
        z_mm: float,
    ) -> tuple[DetectionResult | None, np.ndarray]:
        method = str(self._config.get("vision.detection_method", "adaptive")).strip().lower()
        mask = self._build_mask(frame, method)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_area = float(self._config.get("vision.min_contour_area", 500))
        max_area = float(self._config.get("vision.max_contour_area", 50000))
        filtered = [contour for contour in contours if min_area <= cv2.contourArea(contour) <= max_area]
        if not filtered:
            return None, mask

        contour = max(filtered, key=cv2.contourArea)
        x, y, width_px, height_px = cv2.boundingRect(contour)
        moments = cv2.moments(contour)
        if not moments["m00"]:
            centroid_px = (x + width_px // 2, y + height_px // 2)
        else:
            centroid_px = (
                int(moments["m10"] / moments["m00"]),
                int(moments["m01"] / moments["m00"]),
            )
        orientation_deg = float(cv2.minAreaRect(contour)[2])

        fx = max(float(intrinsic_matrix[0, 0]), 1.0)
        fy = max(float(intrinsic_matrix[1, 1]), 1.0)
        width_mm = float(width_px) * float(z_mm) / fx
        height_mm = float(height_px) * float(z_mm) / fy

        pick_inset_x_mm = float(self._config.get("vision.pick_inset_x_mm", 10.0))
        pick_inset_y_mm = float(self._config.get("vision.pick_inset_y_mm", 8.0))
        inset_x_px = int(round(pick_inset_x_mm * fx / float(z_mm)))
        inset_y_px = int(round(pick_inset_y_mm * fy / float(z_mm)))

        safe_x1 = min(x + inset_x_px, x + max(width_px // 2, 1))
        safe_x2 = max(x + width_px - inset_x_px, safe_x1 + 1)
        safe_y1 = min(y + inset_y_px, y + max(height_px // 2, 1))
        safe_y2 = max(y + height_px - inset_y_px, safe_y1 + 1)
        if safe_x2 <= safe_x1:
            safe_x1 = x
            safe_x2 = x + width_px
        if safe_y2 <= safe_y1:
            safe_y1 = y
            safe_y2 = y + height_px

        pick_u = int(round((safe_x1 + safe_x2) / 2.0))
        pick_v = int(round((safe_y1 + safe_y2) / 2.0))
        centroid_world = self._workspace_validator.pixel_to_world(
            centroid_px[0],
            centroid_px[1],
            intrinsic_matrix,
            z_mm,
        )
        pick_point_world = self._workspace_validator.pixel_to_world(
            pick_u,
            pick_v,
            intrinsic_matrix,
            z_mm,
        )
        workspace_status, workspace_message = self._workspace_validator.check(
            pick_point_world[0],
            pick_point_world[1],
        )

        result = DetectionResult(
            bbox_px=(x, y, width_px, height_px),
            centroid_px=centroid_px,
            contour=contour,
            orientation_deg=orientation_deg,
            centroid_world=centroid_world,
            width_mm=width_mm,
            height_mm=height_mm,
            pick_point_px=(pick_u, pick_v),
            pick_point_world=pick_point_world,
            workspace_status=workspace_status,
            safe_zone_px=(safe_x1, safe_y1, safe_x2 - safe_x1, safe_y2 - safe_y1),
            workspace_message=workspace_message,
        )
        return result, mask

    def draw_detection_overlay(
        self,
        frame: np.ndarray,
        detection: DetectionResult,
        preview_pick_zone: bool = True,
    ) -> np.ndarray:
        overlay = frame.copy()
        x, y, width_px, height_px = detection.bbox_px
        safe_x, safe_y, safe_w, safe_h = detection.safe_zone_px
        bbox_color = {
            WorkspaceValidator.ValidationResult.VALID: (122, 201, 34),
            WorkspaceValidator.ValidationResult.NEAR_BOUNDARY: (35, 163, 245),
            WorkspaceValidator.ValidationResult.OUT_OF_BOUNDS: (64, 64, 232),
        }.get(detection.workspace_status, (122, 201, 34))
        cv2.rectangle(overlay, (x, y), (x + width_px, y + height_px), bbox_color, 2)

        dimension_text = f"W={detection.width_mm:.1f}mm H={detection.height_mm:.1f}mm"
        self._draw_outlined_text(overlay, dimension_text, (x, max(y - 10, 18)), (75, 168, 200))

        if preview_pick_zone:
            cv2.rectangle(overlay, (safe_x, safe_y), (safe_x + safe_w, safe_y + safe_h), (217, 144, 74), 2)
            self._draw_fixture_exclusion(overlay, detection)

        pick_u, pick_v = detection.pick_point_px
        self._draw_crosshair(overlay, (pick_u, pick_v), (75, 168, 200))
        self._draw_outlined_text(
            overlay,
            f"(X={detection.pick_point_world[0]:.1f}, Y={detection.pick_point_world[1]:.1f} mm)",
            (pick_u + 12, max(pick_v - 12, 18)),
            (255, 255, 255),
        )

        status_text = {
            WorkspaceValidator.ValidationResult.VALID: "VALID - READY TO PICK",
            WorkspaceValidator.ValidationResult.NEAR_BOUNDARY: "NEAR BOUNDARY",
            WorkspaceValidator.ValidationResult.OUT_OF_BOUNDS: "OUT OF WORKSPACE",
        }.get(detection.workspace_status, detection.workspace_message)
        status_color = (122, 201, 34) if detection.workspace_status == WorkspaceValidator.ValidationResult.VALID else (64, 64, 232)
        self._draw_outlined_text(overlay, status_text, (x, y + height_px + 20), status_color)
        return overlay

    def _build_mask(self, frame: np.ndarray, method: str) -> np.ndarray:
        kernel_size = int(self._config.get("vision.morph_kernel_size", 3))
        kernel_size = max(1, kernel_size)
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        min_area = float(self._config.get("vision.min_contour_area", 500))
        max_area = float(self._config.get("vision.max_contour_area", 50000))

        if method == "canny":
            threshold1 = int(self._config.get("vision.canny_threshold1", 50))
            threshold2 = int(self._config.get("vision.canny_threshold2", 150))
            mask = cv2.Canny(blurred, threshold1, threshold2)
            mask = cv2.dilate(mask, kernel, iterations=1)
        elif method == "hsv":
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower = np.array(self._config.get("vision.hsv_lower", [0, 0, 150]), dtype=np.uint8)
            upper = np.array(self._config.get("vision.hsv_upper", [180, 60, 255]), dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
        else:
            block_size = int(self._config.get("vision.adaptive_block_size", 11))
            if block_size % 2 == 0:
                block_size += 1
            if block_size < 3:
                block_size = 3
            constant = int(self._config.get("vision.adaptive_c", 2))
            candidates = []
            for threshold_type in (cv2.THRESH_BINARY, cv2.THRESH_BINARY_INV):
                candidate = cv2.adaptiveThreshold(
                    blurred,
                    255,
                    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                    threshold_type,
                    block_size,
                    constant,
                )
                candidate = cv2.morphologyEx(candidate, cv2.MORPH_OPEN, kernel, iterations=1)
                candidate = cv2.morphologyEx(candidate, cv2.MORPH_CLOSE, kernel, iterations=1)
                contours, _ = cv2.findContours(candidate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                in_range = [contour for contour in contours if min_area <= cv2.contourArea(contour) <= max_area]
                largest_area = max((cv2.contourArea(contour) for contour in contours), default=0.0)
                candidates.append((len(in_range), largest_area, candidate))
            candidates.sort(key=lambda item: (item[0], -abs(((min_area + max_area) / 2.0) - item[1])), reverse=True)
            mask = candidates[0][2]

        if method != "adaptive":
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        return mask

    @staticmethod
    def _draw_crosshair(frame: np.ndarray, center: tuple[int, int], color: tuple[int, int, int]) -> None:
        radius = 10
        cv2.line(frame, (center[0] - radius, center[1]), (center[0] + radius, center[1]), color, 2)
        cv2.line(frame, (center[0], center[1] - radius), (center[0], center[1] + radius), color, 2)
        cv2.circle(frame, center, 3, color, -1)

    def _draw_fixture_exclusion(self, frame: np.ndarray, detection: DetectionResult) -> None:
        x, y, width_px, height_px = detection.bbox_px
        safe_x, safe_y, safe_w, safe_h = detection.safe_zone_px
        zones = [
            ((x, y), (x + width_px, safe_y)),
            ((x, safe_y + safe_h), (x + width_px, y + height_px)),
            ((x, safe_y), (safe_x, safe_y + safe_h)),
            ((safe_x + safe_w, safe_y), (x + width_px, safe_y + safe_h)),
        ]
        for top_left, bottom_right in zones:
            if bottom_right[0] <= top_left[0] or bottom_right[1] <= top_left[1]:
                continue
            self._draw_dashed_rect(frame, top_left, bottom_right, (64, 64, 232), 1)

    @staticmethod
    def _draw_outlined_text(
        frame: np.ndarray,
        text: str,
        position: tuple[int, int],
        color: tuple[int, int, int],
    ) -> None:
        cv2.putText(frame, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (7, 14, 28), 3, cv2.LINE_AA)
        cv2.putText(frame, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    @classmethod
    def _draw_dashed_rect(
        cls,
        frame: np.ndarray,
        top_left: tuple[int, int],
        bottom_right: tuple[int, int],
        color: tuple[int, int, int],
        thickness: int,
    ) -> None:
        x1 = min(top_left[0], bottom_right[0])
        y1 = min(top_left[1], bottom_right[1])
        x2 = max(top_left[0], bottom_right[0])
        y2 = max(top_left[1], bottom_right[1])
        cls._draw_dashed_line(frame, (x1, y1), (x2, y1), color, thickness)
        cls._draw_dashed_line(frame, (x2, y1), (x2, y2), color, thickness)
        cls._draw_dashed_line(frame, (x2, y2), (x1, y2), color, thickness)
        cls._draw_dashed_line(frame, (x1, y2), (x1, y1), color, thickness)

    @staticmethod
    def _draw_dashed_line(
        frame: np.ndarray,
        start: tuple[int, int],
        end: tuple[int, int],
        color: tuple[int, int, int],
        thickness: int,
        dash_length: int = 8,
        gap_length: int = 4,
    ) -> None:
        delta = np.array(end, dtype=np.float64) - np.array(start, dtype=np.float64)
        distance = float(np.linalg.norm(delta))
        if distance <= 0:
            return
        direction = delta / distance
        offset = 0.0
        while offset < distance:
            dash_end = min(offset + dash_length, distance)
            p1 = tuple(np.round(np.array(start) + direction * offset).astype(int))
            p2 = tuple(np.round(np.array(start) + direction * dash_end).astype(int))
            cv2.line(frame, p1, p2, color, thickness, cv2.LINE_AA)
            offset += dash_length + gap_length
