"""Test B — end-to-end pick-accuracy validation (forward test).

This module is a *self-contained* validation harness used to measure how
accurately the full vision pipeline lands the tool on the intended pick point.
It does **not** modify or hook into the live detection path; it only *reuses*
the same production code (:class:`SafePickValidator` and
:class:`WorkspaceValidator.pixel_to_base`) so the numbers reflect the real
system, not a re-implementation.

The whole feature is gated behind ``vision.pick_accuracy_test.enabled`` in
``config.json``. When that flag is ``false`` (the default) the data-collection
entry points raise, so nothing here can run unless explicitly turned on.

Methodology (see thesis Test B):

* **Detected** — the pick point chosen by the live pipeline from the *detected*
  bounding boxes.
* **Ideal** — the pick point the *same* algorithm would choose from the
  ground-truth boxes measured manually (caliper). This isolates the detector's
  contribution from the safe-pick geometry.
* **B1 (selection error)** — distance between detected and ideal pick point,
  plus whether the SAFE/MARGINAL/UNSAFE decision matched. Computed in software.
* **B2 (landing error)** — uses the selongsong's leftmost tip ("ujung-1") as a
  common datum. The operator measures one caliper distance from that tip to the
  tool tip (``measured_distance_mm``); the program computes the distance from the
  same tip to the pick point (``reference_distance_mm``). The landing error is
  ``|measured - reference|`` (mm). One number to type, the rest is automatic.
  This is the headline thesis metric (1-D, along the selongsong axis).
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

from backend.config_manager import ConfigManager
from vision.safe_pick_validator import SafePickValidator
from vision.workspace_validator import WorkspaceValidator

Point2D = tuple[float, float]
Box = Sequence[float]  # [x1, y1, x2, y2] in pixel coords


def _as_tuple2(value: object) -> Point2D | None:
    if value is None:
        return None
    seq = list(value)  # type: ignore[arg-type]
    if len(seq) < 2:
        return None
    return (float(seq[0]), float(seq[1]))


def _distance(a: Point2D | None, b: Point2D | None) -> float | None:
    if a is None or b is None:
        return None
    return float(math.hypot(a[0] - b[0], a[1] - b[1]))


@dataclass(slots=True)
class PickTrial:
    """One recorded Test-B trial.

    The ``detected_*`` and ``ideal_*`` fields are normally filled by
    :meth:`PickAccuracyValidator.build_trial`.

    B2 landing error uses the selongsong's leftmost tip as a shared datum:

    * ``measured_distance_mm`` — operator's caliper reading, tip -> tool tip.
    * ``reference_distance_mm`` — program's tip -> pick point distance (mm).
    * landing error = ``|measured - reference|`` (see :attr:`landing_offset_mm`).

    ``ref_point_px`` / ``ref_point_base`` record where the program placed the
    tip datum. ``measured_offset_mm`` is the legacy 2-D (dx, dy) caliper reading,
    kept for backward compatibility with older logs.
    """

    trial_id: str
    object_label: str = ""
    orientation_deg: float = 0.0

    detected_pick_px: Point2D | None = None
    detected_pick_base: Point2D | None = None
    detected_status: str = "UNKNOWN"

    ideal_pick_px: Point2D | None = None
    ideal_pick_base: Point2D | None = None
    ideal_status: str = "UNKNOWN"

    # B2 datum = selongsong leftmost tip ("ujung-1")
    ref_point_px: Point2D | None = None
    ref_point_base: Point2D | None = None
    reference_distance_mm: float | None = None  # program: tip -> pick point
    measured_distance_mm: float | None = None   # operator caliper: tip -> tool

    expected_status: str = ""  # operator's logical expectation; falls back to ideal
    measured_offset_mm: Point2D | None = None  # legacy (dx, dy) caliper reading

    notes: str = ""

    # ---- derived metrics ----------------------------------------------------

    @property
    def selection_error_px(self) -> float | None:
        """B1 in pixels: detected vs ideal pick point."""
        return _distance(self.detected_pick_px, self.ideal_pick_px)

    @property
    def selection_error_mm(self) -> float | None:
        """B1 in mm (Base frame): detection-induced pick-point error."""
        return _distance(self.detected_pick_base, self.ideal_pick_base)

    @property
    def status_reference(self) -> str:
        return self.expected_status or self.ideal_status

    @property
    def status_match(self) -> bool:
        return self.detected_status == self.status_reference

    @property
    def landing_error_signed_mm(self) -> float | None:
        """B2 signed error along the selongsong axis: measured - reference (mm).

        Positive => tool landed *beyond* the pick point (farther from the tip);
        negative => tool fell *short* of the pick point.
        """
        if self.measured_distance_mm is None or self.reference_distance_mm is None:
            return None
        return float(self.measured_distance_mm - self.reference_distance_mm)

    @property
    def landing_offset_mm(self) -> float | None:
        """B2 landing-error magnitude (mm).

        Prefers the 1-D datum method (``|measured - reference|``); falls back to
        the legacy 2-D caliper magnitude for older logs.
        """
        signed = self.landing_error_signed_mm
        if signed is not None:
            return abs(signed)
        if self.measured_offset_mm is None:
            return None
        return float(math.hypot(self.measured_offset_mm[0], self.measured_offset_mm[1]))

    # ---- serialisation ------------------------------------------------------

    def to_dict(self) -> dict:
        data = asdict(self)
        for key in ("detected_pick_px", "detected_pick_base", "ideal_pick_px",
                    "ideal_pick_base", "ref_point_px", "ref_point_base",
                    "measured_offset_mm"):
            if data[key] is not None:
                data[key] = [round(float(v), 4) for v in data[key]]
        for key in ("reference_distance_mm", "measured_distance_mm"):
            if data[key] is not None:
                data[key] = round(float(data[key]), 4)
        data["metrics"] = {
            "selection_error_px": _round(self.selection_error_px),
            "selection_error_mm": _round(self.selection_error_mm),
            "status_match": self.status_match,
            "landing_offset_mm": _round(self.landing_offset_mm),
            "landing_error_signed_mm": _round(self.landing_error_signed_mm),
        }
        return data

    @classmethod
    def from_dict(cls, data: dict) -> "PickTrial":
        def _opt_float(value: object) -> float | None:
            return None if value is None else float(value)

        return cls(
            trial_id=str(data.get("trial_id", "")),
            object_label=str(data.get("object_label", "")),
            orientation_deg=float(data.get("orientation_deg", 0.0)),
            detected_pick_px=_as_tuple2(data.get("detected_pick_px")),
            detected_pick_base=_as_tuple2(data.get("detected_pick_base")),
            detected_status=str(data.get("detected_status", "UNKNOWN")),
            ideal_pick_px=_as_tuple2(data.get("ideal_pick_px")),
            ideal_pick_base=_as_tuple2(data.get("ideal_pick_base")),
            ideal_status=str(data.get("ideal_status", "UNKNOWN")),
            ref_point_px=_as_tuple2(data.get("ref_point_px")),
            ref_point_base=_as_tuple2(data.get("ref_point_base")),
            reference_distance_mm=_opt_float(data.get("reference_distance_mm")),
            measured_distance_mm=_opt_float(data.get("measured_distance_mm")),
            expected_status=str(data.get("expected_status", "")),
            measured_offset_mm=_as_tuple2(data.get("measured_offset_mm")),
            notes=str(data.get("notes", "")),
        )


@dataclass(slots=True)
class PickAccuracyReport:
    threshold_mm: float
    trial_count: int

    # B1 — pick-point selection (detection quality), mm
    selection_error_mean_mm: float | None
    selection_error_max_mm: float | None
    selection_error_rms_mm: float | None
    selection_error_std_mm: float | None
    status_accuracy_pct: float | None

    # B2 — physical landing accuracy, mm
    measured_trial_count: int
    landing_offset_mean_mm: float | None
    landing_offset_max_mm: float | None
    landing_offset_rms_mm: float | None
    landing_offset_std_mm: float | None
    landing_bias_mean_mm: float | None  # mean signed (measured - reference)
    success_count: int
    success_rate_pct: float | None

    def to_dict(self) -> dict:
        return asdict(self)


def _round(value: float | None, ndigits: int = 4) -> float | None:
    return None if value is None else round(float(value), ndigits)


def _stats(values: Sequence[float]) -> dict[str, float | None]:
    arr = np.asarray([v for v in values if v is not None], dtype=np.float64)
    if arr.size == 0:
        return {"mean": None, "max": None, "rms": None, "std": None}
    return {
        "mean": float(np.mean(arr)),
        "max": float(np.max(arr)),
        "rms": float(np.sqrt(np.mean(np.square(arr)))),
        "std": float(np.std(arr)),
    }


class PickAccuracyValidator:
    """Forward-test (Test B) harness for end-to-end pick accuracy.

    Construct it, then either drive it live with :meth:`evaluate_frame` or feed
    pre-recorded boxes to :meth:`build_trial`. Aggregate with :meth:`analyze`.

    The harness is inert unless ``vision.pick_accuracy_test.enabled`` is true.
    """

    CONFIG_PREFIX = "vision.pick_accuracy_test"

    def __init__(
        self,
        config: ConfigManager | None = None,
        workspace_validator: WorkspaceValidator | None = None,
        safe_pick_validator: SafePickValidator | None = None,
    ) -> None:
        self._config = config or ConfigManager.instance()
        self._workspace = workspace_validator or WorkspaceValidator(self._config)
        self._safe_pick = safe_pick_validator or SafePickValidator(self._config)

    # ---- on/off toggle ------------------------------------------------------

    def is_enabled(self) -> bool:
        return bool(self._config.get(f"{self.CONFIG_PREFIX}.enabled", False))

    def threshold_mm(self) -> float:
        return float(self._config.get(f"{self.CONFIG_PREFIX}.threshold_mm", 3.0))

    def log_path(self) -> Path:
        raw = str(self._config.get(f"{self.CONFIG_PREFIX}.log_path", "logs/pick_accuracy_trials.json"))
        path = Path(raw)
        if not path.is_absolute():
            path = Path(self._config.config_path).resolve().parent / path
        return path

    def _require_enabled(self) -> None:
        if not self.is_enabled():
            raise RuntimeError(
                "Pick-accuracy test (Test B) is disabled. "
                f"Set {self.CONFIG_PREFIX}.enabled = true in config.json to use it."
            )

    # ---- pick-point computation (reuses production code) --------------------

    def _safe_pick_base(
        self,
        selongsong_box: Box | None,
        fixture_box: Box | None,
        intrinsic_matrix: np.ndarray,
        distortion: np.ndarray | None,
        frame_size: tuple[int, int] | None,
    ) -> tuple[str, Point2D | None, Point2D | None]:
        """Run the production safe-pick + pixel→base on a pair of boxes."""
        if selongsong_box is None:
            return "UNKNOWN", None, None
        sel = np.asarray(selongsong_box, dtype=np.float64)
        fix = None if fixture_box is None else np.asarray(fixture_box, dtype=np.float64)
        status, pick_px = self._safe_pick.check_pair(sel, fix)
        if pick_px is None:
            return status, None, None
        pick_px_f = (float(pick_px[0]), float(pick_px[1]))
        pick_base: Point2D | None = None
        try:
            pick_base = self._workspace.pixel_to_base(
                pick_px_f[0], pick_px_f[1], intrinsic_matrix, distortion, frame_size,
            )
        except ValueError:
            pick_base = None
        return status, pick_px_f, pick_base

    def _left_edge_base(
        self,
        selongsong_box: Box | None,
        intrinsic_matrix: np.ndarray,
        distortion: np.ndarray | None,
        frame_size: tuple[int, int] | None,
    ) -> tuple[Point2D | None, Point2D | None]:
        """Selongsong leftmost tip ("ujung-1") in pixel + Base coords.

        The tip is the left edge at the box's vertical centre: ``(x1, (y1+y2)/2)``.
        """
        if selongsong_box is None:
            return None, None
        box = np.asarray(selongsong_box, dtype=np.float64)
        ref_px = (float(box[0]), 0.5 * (float(box[1]) + float(box[3])))
        ref_base: Point2D | None = None
        try:
            ref_base = self._workspace.pixel_to_base(
                ref_px[0], ref_px[1], intrinsic_matrix, distortion, frame_size,
            )
        except ValueError:
            ref_base = None
        return ref_px, ref_base

    def reference_distance(
        self,
        selongsong_box: Box | None,
        pick_base: object,
        intrinsic_matrix: np.ndarray,
        distortion: np.ndarray | None = None,
        frame_size: tuple[int, int] | None = None,
    ) -> float | None:
        """Program distance (mm) from the selongsong tip to ``pick_base``.

        Pure geometry helper for live previews; not gated by the enable flag.
        """
        _, ref_base = self._left_edge_base(
            selongsong_box, intrinsic_matrix, distortion, frame_size,
        )
        return _distance(ref_base, _as_tuple2(pick_base))

    def build_trial(
        self,
        trial_id: str,
        *,
        detected_selongsong_box: Box | None,
        detected_fixture_box: Box | None,
        ground_truth_selongsong_box: Box | None,
        ground_truth_fixture_box: Box | None,
        intrinsic_matrix: np.ndarray,
        distortion: np.ndarray | None = None,
        frame_size: tuple[int, int] | None = None,
        measured_distance_mm: float | None = None,
        measured_offset_mm: Point2D | None = None,
        object_label: str = "",
        orientation_deg: float = 0.0,
        expected_status: str = "",
        notes: str = "",
    ) -> PickTrial:
        """Build a trial from detected boxes (pipeline) + ground-truth boxes (caliper).

        ``measured_distance_mm`` (the B2 caliper reading, tip -> tool tip) may be
        supplied now or attached later by editing the returned trial. The program
        side (``reference_distance_mm``, tip -> pick point) is computed here from
        the *detected* selongsong box, so the landing error reflects the full
        end-to-end pipeline. ``measured_offset_mm`` is the legacy 2-D path.
        """
        self._require_enabled()
        det_status, det_px, det_base = self._safe_pick_base(
            detected_selongsong_box, detected_fixture_box,
            intrinsic_matrix, distortion, frame_size,
        )
        ideal_status, ideal_px, ideal_base = self._safe_pick_base(
            ground_truth_selongsong_box, ground_truth_fixture_box,
            intrinsic_matrix, distortion, frame_size,
        )
        ref_px, ref_base = self._left_edge_base(
            detected_selongsong_box, intrinsic_matrix, distortion, frame_size,
        )
        reference_distance_mm = _distance(ref_base, det_base)
        measured = None if measured_distance_mm is None else float(measured_distance_mm)
        return PickTrial(
            trial_id=trial_id,
            object_label=object_label,
            orientation_deg=float(orientation_deg),
            detected_pick_px=det_px,
            detected_pick_base=det_base,
            detected_status=det_status,
            ideal_pick_px=ideal_px,
            ideal_pick_base=ideal_base,
            ideal_status=ideal_status,
            ref_point_px=ref_px,
            ref_point_base=ref_base,
            reference_distance_mm=reference_distance_mm,
            measured_distance_mm=measured,
            expected_status=expected_status,
            measured_offset_mm=_as_tuple2(measured_offset_mm),
            notes=notes,
        )

    def evaluate_frame(
        self,
        frame: np.ndarray,
        intrinsic_matrix: np.ndarray,
        z_mm: float,
        distortion: np.ndarray | None = None,
        detector: object | None = None,
    ) -> tuple[Box | None, Box | None]:
        """Run the live model pipeline on *frame*, returning its detected boxes.

        Convenience for collecting the ``detected_*`` boxes that feed
        :meth:`build_trial`. ``ModelDetector`` is imported lazily so this module
        never hard-depends on onnxruntime.
        """
        self._require_enabled()
        if detector is None:
            from vision.model_detector import ModelDetector

            detector = ModelDetector(self._config, self._workspace)
        bundles = detector.detect_bundles(frame, intrinsic_matrix, z_mm, distortion)
        if not bundles:
            return None, None
        bundle = bundles[0]
        sel = None if bundle.selongsong_box is None else bundle.selongsong_box.tolist()
        fix = None if bundle.fixture_box is None else bundle.fixture_box.tolist()
        return sel, fix

    # ---- analysis -----------------------------------------------------------

    def analyze(self, trials: Sequence[PickTrial]) -> PickAccuracyReport:
        threshold = self.threshold_mm()
        selection = [t.selection_error_mm for t in trials if t.selection_error_mm is not None]
        sel_stats = _stats(selection)

        status_evaluable = [t for t in trials if t.detected_pick_px is not None or t.detected_status != "UNKNOWN"]
        status_accuracy = (
            100.0 * sum(1 for t in status_evaluable if t.status_match) / len(status_evaluable)
            if status_evaluable else None
        )

        measured = [t for t in trials if t.landing_offset_mm is not None]
        landing = [t.landing_offset_mm for t in measured]
        land_stats = _stats(landing)
        signed = [t.landing_error_signed_mm for t in measured if t.landing_error_signed_mm is not None]
        bias_mean = float(np.mean(signed)) if signed else None
        success_count = sum(1 for t in measured if (t.landing_offset_mm or math.inf) <= threshold)
        success_rate = 100.0 * success_count / len(measured) if measured else None

        return PickAccuracyReport(
            threshold_mm=threshold,
            trial_count=len(trials),
            selection_error_mean_mm=_round(sel_stats["mean"]),
            selection_error_max_mm=_round(sel_stats["max"]),
            selection_error_rms_mm=_round(sel_stats["rms"]),
            selection_error_std_mm=_round(sel_stats["std"]),
            status_accuracy_pct=_round(status_accuracy, 2),
            measured_trial_count=len(measured),
            landing_offset_mean_mm=_round(land_stats["mean"]),
            landing_offset_max_mm=_round(land_stats["max"]),
            landing_offset_rms_mm=_round(land_stats["rms"]),
            landing_offset_std_mm=_round(land_stats["std"]),
            landing_bias_mean_mm=_round(bias_mean),
            success_count=success_count,
            success_rate_pct=_round(success_rate, 2),
        )

    def format_report(self, report: PickAccuracyReport) -> str:
        def fmt(value: float | None, unit: str = " mm") -> str:
            return "n/a" if value is None else f"{value:.2f}{unit}"

        lines = [
            "===== Test B — Pick Accuracy Report =====",
            f"Trials recorded        : {report.trial_count}",
            f"Success threshold      : {report.threshold_mm:.2f} mm",
            "",
            "B1 — Pick-point selection (detection quality)",
            f"  mean error           : {fmt(report.selection_error_mean_mm)}",
            f"  max error            : {fmt(report.selection_error_max_mm)}",
            f"  RMS error            : {fmt(report.selection_error_rms_mm)}",
            f"  status decision acc. : {fmt(report.status_accuracy_pct, ' %')}",
            "",
            "B2 — Landing accuracy (end-to-end, tip datum)",
            f"  measured trials      : {report.measured_trial_count}",
            f"  mean |error|         : {fmt(report.landing_offset_mean_mm)}",
            f"  max |error|          : {fmt(report.landing_offset_max_mm)}",
            f"  RMS error            : {fmt(report.landing_offset_rms_mm)}",
            f"  std error            : {fmt(report.landing_offset_std_mm)}",
            f"  mean bias (meas-ref) : {fmt(report.landing_bias_mean_mm)}",
            f"  success              : {report.success_count}/{report.measured_trial_count}"
            f"  ({fmt(report.success_rate_pct, ' %')})",
            "=========================================",
        ]
        return "\n".join(lines)

    # ---- persistence --------------------------------------------------------

    def save_trials(self, trials: Sequence[PickTrial], path: str | Path | None = None) -> Path:
        target = Path(path) if path is not None else self.log_path()
        target.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "report": self.analyze(trials).to_dict(),
            "trials": [t.to_dict() for t in trials],
        }
        target.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        return target

    def load_trials(self, path: str | Path | None = None) -> list[PickTrial]:
        source = Path(path) if path is not None else self.log_path()
        if not source.exists():
            return []
        data = json.loads(source.read_text(encoding="utf-8"))
        return [PickTrial.from_dict(item) for item in data.get("trials", [])]

    def export_csv(self, trials: Sequence[PickTrial], path: str | Path) -> Path:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        fields = [
            "trial_id", "object_label", "orientation_deg", "detected_status",
            "ideal_status", "status_match", "selection_error_px",
            "selection_error_mm", "reference_distance_mm", "measured_distance_mm",
            "landing_error_signed_mm", "landing_offset_mm", "notes",
        ]
        with target.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for t in trials:
                writer.writerow({
                    "trial_id": t.trial_id,
                    "object_label": t.object_label,
                    "orientation_deg": round(t.orientation_deg, 2),
                    "detected_status": t.detected_status,
                    "ideal_status": t.ideal_status,
                    "status_match": t.status_match,
                    "selection_error_px": _round(t.selection_error_px, 2),
                    "selection_error_mm": _round(t.selection_error_mm, 3),
                    "reference_distance_mm": _round(t.reference_distance_mm, 3),
                    "measured_distance_mm": _round(t.measured_distance_mm, 3),
                    "landing_error_signed_mm": _round(t.landing_error_signed_mm, 3),
                    "landing_offset_mm": _round(t.landing_offset_mm, 3),
                    "notes": t.notes,
                })
        return target
