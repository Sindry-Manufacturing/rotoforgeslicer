"""Curvature / slew limit for tangential C-tracking. SPEC §4.3.

The wheel cannot bend faster than the C axis can slew:
    R_min = v / omega_max          (v in mm/s, omega_max in rad/s)
Within a single pass v and RPM are constant, so the whole pass must satisfy
R >= R_min(v) everywhere; otherwise the path is broken with an airborne reorient.
"""
from __future__ import annotations

import math


def r_min(v_mm_s: float, omega_max_deg_s: float) -> float:
    """Minimum followable radius at speed v. inf when slew limit is unset (<=0)."""
    if omega_max_deg_s is None or omega_max_deg_s <= 0:
        return math.inf
    return v_mm_s / math.radians(omega_max_deg_s)


def max_heading_rate_deg_per_mm(v_mm_s: float, omega_max_deg_s: float) -> float:
    if v_mm_s <= 0:
        return math.inf
    return omega_max_deg_s / v_mm_s


def split_on_curvature(path_xy, v_mm_s: float, omega_max_deg_s: float):
    """Split a polyline where local radius < r_min(v). SPEC §4.3.  [stub — M5]"""
    raise NotImplementedError("split_on_curvature: implement per SPEC §4.3")
