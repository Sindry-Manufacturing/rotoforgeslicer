"""Heading <-> rotary A-axis mapping, axis-range + winding helpers. SPEC §4.1 (D13).

The deposition head (wire feeder + wheel) rotates as one unit on the C axis, so the
wheel always points along the direction of travel — there is **no deposition wedge and
no privileged direction** (D13 supersedes the old +Y wedge). ``A`` is *always*
commanded equal to the travel heading (commanded drift ≈ 0).

Home heading is +Y, which is only the axis **zero reference** after homing — it has no
deposition meaning. Travel-direction headings are degrees CCW from +X. The only
per-axis limits are the slew rate (``fill.curvature``) and the usable **continuous
angular range** ``[a_min_deg, a_max_deg]`` (no full 360°): a pass's accumulated axis
angle must stay inside that range, else the path breaks with an airborne unwind.

(The module keeps its legacy filename; it is no longer about a wedge.)
"""
from __future__ import annotations

import math
from typing import List

from ..config import CAxisCfg


def heading_deg_from_vector(dx: float, dy: float) -> float:
    """Travel-direction heading in degrees CCW from +X, in (-180, 180]."""
    return math.degrees(math.atan2(dy, dx))


def heading_to_a_deg(theta_deg: float, cfg: CAxisCfg) -> float:
    """Map a travel-direction heading to the rotary A-axis angle (affine in theta)."""
    return cfg.invert_sign * (theta_deg - cfg.home_heading_deg) + cfg.home_offset_deg


def within_axis_range(a_deg: float, cfg: CAxisCfg, tol: float = 1e-9) -> bool:
    """True if an A-axis angle lies within the usable continuous range [a_min, a_max].

    This is the *only* hard heading limit under D13 — there is no wedge. It is checked
    against the winding-resolved (continuous) A, so a heading that maps to an
    out-of-range raw A may still be reachable at a different winding.
    """
    return cfg.a_min_deg - tol <= a_deg <= cfg.a_max_deg + tol


def unwrap_headings(headings_deg: List[float]) -> List[float]:
    """Make a heading sequence continuous by removing the ±360° atan2 branch-cut jumps.

    Each consecutive delta is wrapped into (-180, 180] and accumulated, so a path that
    turns smoothly through ±180° keeps a continuous (monotone-where-monotone) heading —
    the basis for tracking accumulated axis angle along a curve.
    """
    if not headings_deg:
        return []
    out = [headings_deg[0]]
    for h in headings_deg[1:]:
        d = (h - out[-1] + 180.0) % 360.0 - 180.0
        out.append(out[-1] + d)
    return out


def winding_shift(a_lo: float, a_hi: float, a_min: float, a_max: float,
                  tol: float = 1e-6) -> float:
    """A multiple of 360° that places the continuous A-band [a_lo, a_hi] inside the
    usable range [a_min, a_max].

    Picks the winding ``360*k`` whose shifted band best centres in the range (least
    transient motion). If the band is wider than the range (``a_hi-a_lo`` exceeds
    ``a_max-a_min``) no winding fits — the planner must have split the pass first; we
    then return the best-effort shift that anchors ``a_lo`` at ``a_min``.
    """
    k_lo = math.ceil((a_min - a_lo) / 360.0 - tol)
    k_hi = math.floor((a_max - a_hi) / 360.0 + tol)
    if k_lo <= k_hi:
        # centre the band in the range, then clamp the integer winding into [k_lo, k_hi]
        k_centre = round(((a_min + a_max) / 2.0 - (a_lo + a_hi) / 2.0) / 360.0)
        k = min(max(k_centre, k_lo), k_hi)
        return 360.0 * k
    return 360.0 * round((a_min - a_lo) / 360.0)
