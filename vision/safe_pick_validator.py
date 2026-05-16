from __future__ import annotations

import numpy as np

from backend.config_manager import ConfigManager


class SafePickValidator:
    """Per-side overlap subtraction validator for model-detected pairs.

    Statuses:
        SAFE      – safe zone is large enough → pick immediately.
        MARGINAL  – safe zone exists but is narrow → BLOCK, wait for manual confirm.
        UNSAFE    – no safe zone → BLOCK, do not pick.
        UNKNOWN   – fixture not detected → fallback inset margin, pick with warning.
    """

    def __init__(self, config: ConfigManager | None = None) -> None:
        self._config = config or ConfigManager.instance()

    def check_pair(
        self,
        selongsong_box: np.ndarray,
        fixture_box: np.ndarray | None,
        margin_pct: float | None = None,
    ) -> tuple[str, tuple[int, int] | None]:
        """Validate safety of picking *selongsong_box* given *fixture_box*.

        Parameters
        ----------
        selongsong_box : array [x1, y1, x2, y2] in pixel coords.
        fixture_box    : array [x1, y1, x2, y2] or ``None`` if not detected.
        margin_pct     : safety margin as fraction of selongsong dimension.

        Returns
        -------
        (status, pick_point_px) where status ∈ {"SAFE", "MARGINAL", "UNSAFE", "UNKNOWN"}.
        pick_point_px is ``None`` when UNSAFE.
        """
        if margin_pct is None:
            margin_pct = float(self._config.get("vision.safe_pick_margin_pct", 0.15))

        s = np.asarray(selongsong_box, dtype=np.float64)
        s_w = s[2] - s[0]
        s_h = s[3] - s[1]
        s_cx = (s[0] + s[2]) / 2.0
        s_cy = (s[1] + s[3]) / 2.0

        if fixture_box is None:
            # UNKNOWN — no fixture detected; use center of selongsong
            return "UNKNOWN", (int(round(s_cx)), int(round(s_cy)))

        f = np.asarray(fixture_box, dtype=np.float64)
        sx1, sy1, sx2, sy2 = s
        fx1, fy1, fx2, fy2 = f

        # Build up to 4 candidate safe regions: parts of selongsong NOT
        # overlapping fixture. Each candidate is a rectangle that lies on the
        # selongsong but on one side of the fixture (LEFT, RIGHT, TOP, BOTTOM).
        candidates: list[tuple[float, float, float, float]] = []
        if fx1 > sx1:  # LEFT of fixture
            candidates.append((sx1, sy1, min(sx2, fx1), sy2))
        if fx2 < sx2:  # RIGHT of fixture
            candidates.append((max(sx1, fx2), sy1, sx2, sy2))
        if fy1 > sy1:  # TOP of fixture
            candidates.append((sx1, sy1, sx2, min(sy2, fy1)))
        if fy2 < sy2:  # BOTTOM of fixture
            candidates.append((sx1, max(sy1, fy2), sx2, sy2))

        # Drop degenerate candidates with zero/negative dimensions
        candidates = [
            c for c in candidates if (c[2] - c[0]) > 0 and (c[3] - c[1]) > 0
        ]

        if not candidates:
            # Selongsong fully covered by fixture on all sides
            return "UNSAFE", None

        # Pick candidate with the largest area = most room to grasp
        best = max(candidates, key=lambda c: (c[2] - c[0]) * (c[3] - c[1]))
        bx1, by1, bx2, by2 = best

        pick_u = int(round((bx1 + bx2) / 2.0))
        pick_v = int(round((by1 + by2) / 2.0))

        zone_w = bx2 - bx1
        zone_h = by2 - by1
        min_safe = min(s_w, s_h) * margin_pct

        if zone_w < min_safe or zone_h < min_safe:
            return "UNSAFE", None
        if zone_w < min_safe * 2 or zone_h < min_safe * 2:
            return "MARGINAL", (pick_u, pick_v)
        return "SAFE", (pick_u, pick_v)
