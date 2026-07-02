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

Modes (SPEC §4.2 / ROADMAP M17, selected via ``fill.mode`` / ``fill.perimeter_loops``):

* ``outline``  — the outermost wall loop only (one ring set per boundary);
* ``contour``  — concentric rings all the way in (full contour fill);
* *perimeter walls* — ``fill.perimeter_loops = N`` with mode raster/streamline lays
  N wall loops, then hands the inset interior to the normal fill
  (``inset_interior``).

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


def contour_rings(region, cfg: Config, *, max_loops: int = 0) -> List[Ring]:
    """Concentric wall centreline rings for one region, **outermost first**.

    ``max_loops`` caps the number of offset depths (0 = keep offsetting until the
    region is consumed — full ``contour`` fill; 1 = ``outline``). Each eroded
    boundary is simplified by ``fill.contour_simplify_mm`` to keep vertex counts
    (and G-code size) sane without visibly changing the path.
    """
    simplify = cfg.fill.contour_simplify_mm
    rings: List[Ring] = []
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
        rings.extend(ring_set)
        k += 1
    return rings


def inset_interior(region, cfg: Config, loops: int):
    """The region left for raster/streamline infill inside ``loops`` wall loops.

    The innermost wall centreline sits at ``wall_depth(loops-1)``; the infill
    boundary retreats one further pitch so the hatch does not overlap the wall
    bead. May be empty (walls consumed the region)."""
    if loops <= 0:
        return region
    return region.buffer(-(wall_depth_mm(cfg, loops - 1) + raster_pitch(cfg)))


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
    If no start seats (range too narrow for the sweep), the ring is returned as-is
    and the winding split breaks it into arcs + airborne unwinds — always safe.
    Non-closed input is returned unchanged.
    """
    if len(pts) < 4 or pts[0] != pts[-1]:
        return pts
    cycle = pts[:-1]                              # unique vertices
    n = len(cycle)
    headings = [heading_deg_from_vector(cycle[(i + 1) % n][0] - cycle[i][0],
                                        cycle[(i + 1) % n][1] - cycle[i][1])
                for i in range(n)]                # one per segment, cyclic
    for m in range(n):
        a = [heading_to_a_deg(t, c_axis) for t in
             unwrap_headings(headings[m:] + headings[:m])]
        if _seats(min(a), max(a), c_axis.a_min_deg, c_axis.a_max_deg):
            rotated = cycle[m:] + cycle[:m]
            return rotated + [rotated[0]]         # re-close at the seatable start
    return pts                                    # will split into arcs (safe)


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


def perimeter_paths(region, cfg: Config) -> List[Ring]:
    """The ``fill.perimeter_loops`` wall loops for one region (innermost first),
    empty when the feature is off. The interior for infill is ``inset_interior``."""
    loops = cfg.fill.perimeter_loops
    if loops <= 0:
        return []
    rings = contour_rings(region, cfg, max_loops=loops)
    rings.reverse()
    return [rotate_ring_to_extreme(r, cfg.c_axis) for r in rings]
