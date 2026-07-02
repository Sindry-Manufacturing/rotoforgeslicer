"""Contour / perimeter tracing fill (M17). SPEC §4.2, DECISIONS D13.

Concentric wall loops offset inward from the region boundary at the bead pitch.
Under D13 there is no wedge: **closed rings are legal deposition paths**, limited
only by the slew rate (``R ≥ v/ω_C``, split downstream by ``split_on_curvature``)
and the C axis's usable angular range (split by ``split_on_winding``). A ring's
~360° heading sweep fits a ≥360°-wide range **only if the pass starts at a
rotational extreme** — ``rotate_ring_to_extreme`` starts each ring at the segment
where its unwrapped axis angle is at its cyclic minimum, so the whole loop seats at
one winding when the range allows; on a narrower range the winding split breaks it
into arcs with airborne unwinds (the D13 annulus case: rings trace, breaking only
where winding/curvature requires).

On a **sub-360°** calibrated range some headings are unreachable at any winding —
the planner then deposits those arcs in REVERSE (``split_unreachable``; D13, no
privileged direction), so rings still trace on a real machine. Sharp polygon
corners are split into airborne reorients by the per-vertex heading-step rule
(``split_on_heading_step``) — a dead-sharp corner is not a followable turn.

Modes (SPEC §4.2 / ROADMAP M17, selected via ``fill.mode`` / ``fill.perimeter_loops``):

* ``outline``  — the outermost wall loop only (one ring set per boundary);
* ``contour``  — concentric rings all the way in (full contour fill);
* *perimeter walls* — ``fill.perimeter_loops = N`` with mode raster/streamline lays
  N wall loops, then hands the inset interior to the normal fill
  (``inset_interior`` — inset by the walls that actually FIT, so thin regions get
  infill instead of silent voids).

The first wall centreline sits ``bead_width/2`` inside the boundary; successive
walls step inward by the raster pitch. shapely does the offsetting (erosion shrinks
the exterior and grows holes, so hole walls come for free) and is imported lazily.

Known limitation: features tighter than ``R_min(v)`` shatter on the curvature split
and drop below ``min_deposit_len`` — sub-bead-scale rings vanish rather than emit an
invariant-violating pass.
"""
from __future__ import annotations

import math
from typing import List, Tuple

from ..config import CAxisCfg, Config
from .heading import heading_deg_from_vector, heading_to_a_deg, unwrap_headings
from .raster import raster_pitch

Ring = List[Tuple[float, float]]


def _rings_of(geom) -> List[Ring]:
    """Closed rings (exterior + holes) of a shapely (Multi)Polygon, as point lists
    with the closing point repeated (``pts[0] == pts[-1]``)."""
    from shapely.geometry import MultiPolygon, Polygon

    polys = []
    if isinstance(geom, Polygon):
        polys = [geom]
    elif isinstance(geom, MultiPolygon):
        polys = list(geom.geoms)
    rings: List[Ring] = []
    for p in polys:
        if p.is_empty:
            continue
        for ring in [p.exterior, *p.interiors]:
            pts = [(float(x), float(y)) for x, y in ring.coords]
            if len(pts) >= 4:                      # triangle + closure minimum
                rings.append(pts)
    return rings


def wall_depth_mm(cfg: Config, k: int) -> float:
    """Centreline depth of wall ``k`` (0 = outermost): bead/2 + k * pitch."""
    return cfg.process.bead_width_mm / 2.0 + k * raster_pitch(cfg)


def _rings_by_depth(region, cfg: Config, max_loops: int = 0) -> List[List[Ring]]:
    """Wall centreline rings per offset depth (outermost depth first); stops at the
    first depth whose erosion is empty or at ``max_loops``."""
    simplify = cfg.fill.contour_simplify_mm
    levels: List[List[Ring]] = []
    k = 0
    while True:
        if max_loops and k >= max_loops:
            break
        eroded = region.buffer(-wall_depth_mm(cfg, k))
        if simplify > 0:
            eroded = eroded.simplify(simplify, preserve_topology=True)
        ring_set = _rings_of(eroded)
        if not ring_set:
            break
        levels.append(ring_set)
        k += 1
    return levels


def contour_rings(region, cfg: Config, *, max_loops: int = 0) -> List[Ring]:
    """Concentric wall centreline rings for one region, **outermost first**.

    ``max_loops`` caps the number of offset depths (0 = keep offsetting until the
    region is consumed — full ``contour`` fill; 1 = ``outline``). Each eroded
    boundary is simplified by ``fill.contour_simplify_mm`` to keep vertex counts
    (and G-code size) sane without visibly changing the path.
    """
    return [r for level in _rings_by_depth(region, cfg, max_loops) for r in level]


def inset_interior(region, cfg: Config, loops: int):
    """The region left for raster/streamline infill inside ``loops`` wall loops.

    ``loops`` must be the number of wall depths that actually FIT (see
    ``perimeter_paths``) — insetting for merely-requested walls leaves silent
    voids in thin regions. The innermost wall centreline sits at
    ``wall_depth(loops-1)``; the infill boundary retreats **half** a further pitch,
    so the hatch (whose first line lands pitch/2 inside its boundary) sits exactly
    one pitch from the wall centreline — correct bead adjacency, no unfused seam.
    May be empty (walls consumed the region)."""
    if loops <= 0:
        return region
    return region.buffer(-(wall_depth_mm(cfg, loops - 1) + raster_pitch(cfg) / 2.0))


def _seats(lo: float, hi: float, a_min: float, a_max: float,
           tol: float = 1e-6) -> bool:
    """True if the continuous A-band [lo, hi] fits [a_min, a_max] at SOME whole-turn
    winding (mirror of ``toolpath.passplan._band_fits`` — the axis zero is physical,
    so a band can only shift by 360°k, never slide freely)."""
    return math.ceil((a_min - lo) / 360.0 - tol) <= math.floor((a_max - hi) / 360.0 + tol)


def rotate_ring_to_extreme(pts: Ring, c_axis: CAxisCfg) -> Ring:
    """Start a closed ring at a rotational extreme (D13 / M17 refinement).

    A full ring's heading sweep is ~360°, but its A-band can only be shifted by
    whole turns (the axis zero is physical) — so whether the loop seats at a single
    winding depends on WHERE the pass starts. This scans the ring's vertices for a
    start whose open-path band fits ``[a_min, a_max]`` at some winding and rotates
    the ring there; ``split_on_winding`` then keeps the whole loop as **one pass**.
    If no start seats (range narrower than the sweep), the ring is returned as-is
    and the planner degrades it: reachable arcs forward, unreachable arcs REVERSED
    (``split_unreachable``), broken with airborne unwinds — never an invalid pass.
    Non-closed input is returned unchanged.

    O(N): the per-start bands are sliding-window min/max over the doubled cyclic
    unwrapped-A array (a closed ring's total sweep is a whole number of turns, so
    the doubled array stays congruent mod 360 — seatability is shift-invariant).
    """
    if len(pts) < 4 or pts[0] != pts[-1]:
        return pts
    cycle = pts[:-1]                              # unique vertices
    n = len(cycle)
    headings = [heading_deg_from_vector(cycle[(i + 1) % n][0] - cycle[i][0],
                                        cycle[(i + 1) % n][1] - cycle[i][1])
                for i in range(n)]                # one per segment, cyclic
    u = unwrap_headings(headings)                 # continuous, from segment 0
    sweep = u[-1] + ((headings[0] - u[-1] + 180.0) % 360.0 - 180.0) - u[0]
    a_ext = [heading_to_a_deg(t, c_axis)
             for t in u + [ui + sweep for ui in u]]   # doubled cyclic unwrap

    from collections import deque

    lo_q: deque = deque()                         # indices, increasing a
    hi_q: deque = deque()                         # indices, decreasing a
    m_found = None
    j = 0                                         # window is [m, m+n)
    for m in range(n):
        while j < m + n:                          # grow the window to size n
            while lo_q and a_ext[lo_q[-1]] >= a_ext[j]:
                lo_q.pop()
            while hi_q and a_ext[hi_q[-1]] <= a_ext[j]:
                hi_q.pop()
            lo_q.append(j)
            hi_q.append(j)
            j += 1
        while lo_q[0] < m:
            lo_q.popleft()
        while hi_q[0] < m:
            hi_q.popleft()
        if _seats(a_ext[lo_q[0]], a_ext[hi_q[0]], c_axis.a_min_deg, c_axis.a_max_deg):
            m_found = m
            break
    if m_found is None:
        return pts                                # arcs + reversals downstream (safe)
    rotated = cycle[m_found:] + cycle[:m_found]
    return rotated + [rotated[0]]                 # re-close at the seatable start


def contour_paths(region, cfg: Config, mode: str = "contour") -> List[Ring]:
    """Wall centreline paths for one region, ready for the constraint pipeline,
    **innermost first** (deposit inner rings before the outer rings that will
    surround them — their lead-outs then cross only not-yet-deposited paths).

    ``mode``: ``"contour"`` (all rings) or ``"outline"`` (outermost only). Each
    ring starts at its rotational extreme; the caller still applies
    ``split_on_curvature`` + ``split_on_winding`` (exactly like streamlines).
    """
    if mode not in ("contour", "outline"):
        raise ValueError(f"contour mode must be 'contour' or 'outline', got {mode!r}")
    rings = contour_rings(region, cfg, max_loops=1 if mode == "outline" else 0)
    rings.reverse()                               # innermost first
    return [rotate_ring_to_extreme(r, cfg.c_axis) for r in rings]


def perimeter_paths(region, cfg: Config) -> Tuple[List[Ring], int]:
    """(wall loops for one region — innermost first, number of wall depths that
    actually FIT). Empty when the feature is off. The fitted count — not the
    requested ``fill.perimeter_loops`` — must drive ``inset_interior``, or thin
    regions where fewer walls fit would inset the infill for phantom walls and
    leave silent internal voids."""
    loops = cfg.fill.perimeter_loops
    if loops <= 0:
        return [], 0
    levels = _rings_by_depth(region, cfg, max_loops=loops)
    rings = [r for level in levels for r in level]
    rings.reverse()
    return [rotate_ring_to_extreme(r, cfg.c_axis) for r in rings], len(levels)
