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


def circumradius(p0, p1, p2, eps: float = 1e-12) -> float:
    """Radius of the circle through three points = local turn radius at ``p1``.

    inf for (near-)collinear points (a straight path bends infinitely gently)."""
    ax, ay = p0
    bx, by = p1
    cx, cy = p2
    a = math.hypot(bx - cx, by - cy)
    b = math.hypot(ax - cx, ay - cy)
    c = math.hypot(ax - bx, ay - by)
    area2 = abs((bx - ax) * (cy - ay) - (cx - ax) * (by - ay))  # 2 * triangle area
    if area2 <= eps or a <= eps or c <= eps:
        return math.inf
    return (a * b * c) / (2.0 * area2)


def min_radius(path_xy) -> float:
    """Tightest turn radius over a polyline's interior vertices (inf if < 3 points)."""
    pts = list(path_xy)
    if len(pts) < 3:
        return math.inf
    return min(circumradius(pts[i - 1], pts[i], pts[i + 1])
               for i in range(1, len(pts) - 1))


def heading_step_deg(p0, p1, p2) -> float:
    """|heading change| at vertex ``p1`` between segments p0→p1 and p1→p2, in
    (-180, 180] magnitude — the DISCRETE turn the C axis must make at the vertex."""
    t0 = math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0]))
    t1 = math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))
    return abs((t1 - t0 + 180.0) % 360.0 - 180.0)


def split_on_heading_step(path_xy, max_step_deg: float):
    """Split a polyline at vertices whose DISCRETE heading step exceeds
    ``max_step_deg`` (SPEC §4.3 / D13 corner rule).

    The circumradius proxy scales with segment length, so a dead-sharp corner
    between long legs (true turn radius 0) reads as a huge radius and slips past
    ``split_on_curvature`` — yet the emitter commands the new A on the NEXT G1 and
    the firmware interpolates the whole rotation across that in-contact segment:
    the wheel runs up to the full step off the travel tangent (drift violation —
    side-scrubbing). Any vertex turn sharper than ``max_step_deg`` must become an
    airborne lift-reorient-replunge. Densely-sampled smooth curves (streamlines,
    contour rings) step a few degrees per vertex and pass untouched; polygon
    corners split. ``max_step_deg <= 0`` disables the rule (legacy behavior).
    """
    pts = list(path_xy)
    if max_step_deg <= 0 or len(pts) < 3:
        return [pts] if len(pts) >= 2 else []
    out = []
    cur = [pts[0]]
    for i in range(1, len(pts) - 1):
        cur.append(pts[i])
        if heading_step_deg(pts[i - 1], pts[i], pts[i + 1]) > max_step_deg:
            out.append(cur)            # break AT the sharp vertex
            cur = [pts[i]]
    cur.append(pts[-1])
    out.append(cur)
    return [s for s in out if len(s) >= 2]


def split_on_curvature(path_xy, v_mm_s: float, omega_max_deg_s: float):
    """Split a polyline where the local radius < r_min(v). SPEC §4.3.

    Within a pass v is constant, so the whole pass must satisfy R >= R_min(v). Where a
    vertex turns tighter than that, break the path there (the planner inserts an
    airborne lift-reorient-replunge between the resulting sub-passes). Returns a list
    of polylines (each >= 2 points); a clean path returns ``[path]``.
    """
    pts = list(path_xy)
    Rmin = r_min(v_mm_s, omega_max_deg_s)
    if Rmin == math.inf or len(pts) < 3:
        return [pts] if len(pts) >= 2 else []

    out = []
    cur = [pts[0]]
    for i in range(1, len(pts) - 1):
        cur.append(pts[i])
        if circumradius(pts[i - 1], pts[i], pts[i + 1]) < Rmin:
            out.append(cur)            # break before the too-tight turn
            cur = [pts[i]]             # restart at the corner (lift-reorient-replunge)
    cur.append(pts[-1])
    out.append(cur)
    return [s for s in out if len(s) >= 2]
