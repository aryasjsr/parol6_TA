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

        # Per-side overlap subtraction
        safe_x1 = max(s[0], f[2])
        safe_x2 = min(s[2], f[0])
        safe_y1 = max(s[1], f[3])
        safe_y2 = min(s[3], f[1])

        # If fixture covers all sides, fallback to margin from centroid
        if safe_x2 <= safe_x1 or safe_y2 <= safe_y1:
            margin_x = s_w * margin_pct
            margin_y = s_h * margin_pct
            safe_x1 = s_cx - margin_x
            safe_x2 = s_cx + margin_x
            safe_y1 = s_cy - margin_y
            safe_y2 = s_cy + margin_y

        pick_u = int(round((safe_x1 + safe_x2) / 2.0))
        pick_v = int(round((safe_y1 + safe_y2) / 2.0))

        zone_w = safe_x2 - safe_x1
        zone_h = safe_y2 - safe_y1
        min_safe = min(s_w, s_h) * margin_pct

        if zone_w < min_safe or zone_h < min_safe:
            return "UNSAFE", None
        if zone_w < min_safe * 2 or zone_h < min_safe * 2:
            return "MARGINAL", (pick_u, pick_v)
        return "SAFE", (pick_u, pick_v)
