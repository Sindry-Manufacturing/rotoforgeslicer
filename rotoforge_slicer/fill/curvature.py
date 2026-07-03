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


def vertex_step_ok(step_deg: float, l_next_mm: float, v_mm_s: float,
                   omega_max_deg_s: float, budget_deg_mm: float) -> bool:
    """Is a discrete per-vertex heading step physically acceptable in contact?

    The emitter puts the new A on the NEXT G1, and the firmware interpolates the
    rotation linearly across that in-contact segment. Two independent bounds:

    * **feasibility** — the axis must complete the turn within the segment:
      ``step ≤ ω · L_next / v`` (with ω uncalibrated the bound is off);
    * **scrub budget** — while interpolating, the wheel runs off-tangent by up to
      the step for up to the segment length, so the *product* ``step · L_next``
      bounds the off-tangent contact. Fine sampling of a legal curve (small step,
      tiny segment) passes; a polygon corner (large step, long leg) fails even
      though the circumradius proxy — which scales with leg length — reads it as
      a gentle turn.

    ``budget_deg_mm <= 0`` disables the scrub bound (legacy behavior).
    """
    if omega_max_deg_s > 0 and v_mm_s > 0 and \
            step_deg > omega_max_deg_s * l_next_mm / v_mm_s + 1e-9:
        return False
    if budget_deg_mm > 0 and step_deg * l_next_mm > budget_deg_mm + 1e-9:
        return False
    return True


def split_on_heading_step(path_xy, v_mm_s: float, omega_max_deg_s: float,
                          budget_deg_mm: float):
    """Split a polyline at vertices whose discrete heading step fails
    :func:`vertex_step_ok` (SPEC §4.3 / D13 corner rule) — the break becomes an
    airborne lift-reorient-replunge."""
    pts = list(path_xy)
    if len(pts) < 3:
        return [pts] if len(pts) >= 2 else []
    out = []
    cur = [pts[0]]
    for i in range(1, len(pts) - 1):
        cur.append(pts[i])
        step = heading_step_deg(pts[i - 1], pts[i], pts[i + 1])
        l_next = math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        if not vertex_step_ok(step, l_next, v_mm_s, omega_max_deg_s, budget_deg_mm):
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
