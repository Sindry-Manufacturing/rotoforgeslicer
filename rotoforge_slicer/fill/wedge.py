"""Heading <-> rotary A-axis mapping + the depositable wedge. SPEC §4.1.

Home heading is +Y (A = 0). Travel-direction headings are degrees CCW from +X.
Only headings within +/- wedge_half_angle of home are depositable (forward only).
"""
from __future__ import annotations

import math

from ..config import CAxisCfg


def heading_deg_from_vector(dx: float, dy: float) -> float:
    """Travel-direction heading in degrees CCW from +X, in (-180, 180]."""
    return math.degrees(math.atan2(dy, dx))


def heading_to_a_deg(theta_deg: float, cfg: CAxisCfg) -> float:
    """Map a travel-direction heading to the rotary A-axis angle."""
    return cfg.invert_sign * (theta_deg - cfg.home_heading_deg) + cfg.home_offset_deg


def in_wedge(a_deg: float, cfg: CAxisCfg, tol: float = 1e-9) -> bool:
    """True if an A-axis angle lies within the depositable +/- wedge about home."""
    return abs(a_deg - cfg.home_offset_deg) <= cfg.wedge_half_angle_deg + tol


def vector_in_wedge(dx: float, dy: float, cfg: CAxisCfg) -> bool:
    return in_wedge(heading_to_a_deg(heading_deg_from_vector(dx, dy), cfg), cfg)
