from __future__ import annotations

import numpy as np

from backend.config_manager import ConfigManager


class SafePickValidator:
    """Select a pickup point only from the selongsong area left of fixture.

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
        left_offset_px: float | None = None,
    ) -> tuple[str, tuple[int, int] | None]:
        """Validate safety of picking *selongsong_box* given *fixture_box*.

        Parameters
        ----------
        selongsong_box : array [x1, y1, x2, y2] in pixel coords.
        fixture_box    : array [x1, y1, x2, y2] or ``None`` if not detected.
        margin_pct     : safety margin as fraction of selongsong dimension.
        left_offset_px : positive pixel offset from the zone center toward image-left.

        Returns
        -------
        (status, pick_point_px) where status ∈ {"SAFE", "MARGINAL", "UNSAFE", "UNKNOWN"}.
        pick_point_px is ``None`` when UNSAFE.
        """
        if margin_pct is None:
            margin_pct = float(self._config.get("vision.safe_pick_margin_pct", 0.25))
        if left_offset_px is None:
            left_offset_px = float(
                self._config.get("vision.safe_pick_left_offset_px", 0.0)
            )
        left_offset_px = max(0.0, float(left_offset_px))

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
        fx1 = float(f[0])

        # The process fixture can only be approached from the image-left side.
        # Do not switch to right/top/bottom even when those regions are larger.
        left_edge = min(float(sx2), fx1)
        if left_edge <= float(sx1):
            return "UNSAFE", None
        bx1, by1, bx2, by2 = float(sx1), float(sy1), left_edge, float(sy2)

        zone_w = bx2 - bx1
        zone_h = by2 - by1
        min_safe = min(s_w, s_h) * margin_pct

        if zone_w < min_safe or zone_h < min_safe:
            return "UNSAFE", None

        pick_center_u = (bx1 + bx2) / 2.0
        min_pick_u = bx1 + min_safe / 2.0
        max_pick_u = bx2 - min_safe / 2.0
        pick_u = int(round(np.clip(pick_center_u - left_offset_px, min_pick_u, max_pick_u)))
        pick_v = int(round((by1 + by2) / 2.0))

        if zone_w < min_safe * 2 or zone_h < min_safe * 2:
            return "MARGINAL", (pick_u, pick_v)
        return "SAFE", (pick_u, pick_v)
