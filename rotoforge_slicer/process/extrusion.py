"""Wire feed per mm of XY path. SPEC §5.3."""
from __future__ import annotations

import math


def wire_area_mm2(wire_diameter_mm: float) -> float:
    return math.pi * (wire_diameter_mm / 2.0) ** 2


def e_per_path_mm(mode: str, *,
                  feed_speed_mm_min: float | None = None,
                  traverse_mm_min: float | None = None,
                  bead_width_mm: float | None = None,
                  layer_height_mm: float | None = None,
                  wire_diameter_mm: float | None = None,
                  x_ratio: float = 1.0) -> float:
    """Wire mm fed per mm of XY path.

    screener : feed_speed_mm_min / traverse_mm_min  (volumetrically correct; lands Phi in band)
    x        : x_ratio                              (the historical ~1:1)
    volume   : bead_width * layer_height / wire_area
    """
    if mode == "screener":
        if not feed_speed_mm_min or not traverse_mm_min:
            raise ValueError("screener mode needs feed_speed_mm_min and traverse_mm_min")
        return feed_speed_mm_min / traverse_mm_min
    if mode == "x":
        return x_ratio
    if mode == "volume":
        if None in (bead_width_mm, layer_height_mm, wire_diameter_mm):
            raise ValueError("volume mode needs bead_width_mm, layer_height_mm, wire_diameter_mm")
        return bead_width_mm * layer_height_mm / wire_area_mm2(wire_diameter_mm)
    raise ValueError(f"unknown extrusion mode: {mode!r}")
