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
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from ..config import CAxisCfg, Config
from .heading import heading_deg_from_vector, heading_to_a_deg, unwrap_headings
from .raster import raster_pitch

Ring = List[Tuple[float, float]]

#: PrusaSlicer's fixed seam RNG seed (SeamPlacer.cpp get_params), ported so
#: identical inputs always slice to identical G-code. Divergence from the
#: original: ONE stream per plan (plate), not per object — adding a part
#: reshuffles other parts' random seams; determinism per re-slice still holds.
SEAM_RNG_SEED = 1653710332


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


def seat_window(pts: Ring, c_axis: CAxisCfg) -> List[int]:
    """ALL start indices (into the ring's unique-vertex cycle) whose open-path
    A-band fits ``[a_min, a_max]`` at some whole-turn winding — the ring's
    **seat window**. Empty for open input or when nothing seats.

    O(N): sliding-window min/max (monotone deques) over the doubled cyclic
    unwrapped-A array (a closed ring's total sweep is a whole number of turns, so
    the doubled array stays congruent mod 360 — seatability is shift-invariant).

    Physics of the window width: the band from start ``m`` spans ``360° − δ``
    (δ = the heading step BEHIND vertex m; wider for non-convex backtracking), so
    the seat slack is ``W − span`` where W is the range width. At W = 360 the
    window is typically ONE vertex — pinned to the bearing where A meets the
    range stop; real seam freedom needs W > 360 (or a corner step δ > 360 − W,
    which can seat sharp-cornered rings even on sub-360 ranges).
    """
    if len(pts) < 4 or pts[0] != pts[-1]:
        return []
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
    window: List[int] = []
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
            window.append(m)
    return window


def _rotate_ring(pts: Ring, m: int) -> Ring:
    """Pure rotation of a closed ring to start at cycle index ``m``, re-closed."""
    cycle = pts[:-1]
    rotated = cycle[m:] + cycle[:m]
    return rotated + [rotated[0]]


def rotate_ring_to_extreme(pts: Ring, c_axis: CAxisCfg) -> Ring:
    """Start a closed ring at a rotational extreme (D13 / M17 refinement) — the
    FIRST seatable start in the ring's existing vertex order (= the ``extreme``
    seam policy, and the baseline every other policy is guarded against).

    A full ring's heading sweep is ~360°, but its A-band can only be shifted by
    whole turns (the axis zero is physical) — so whether the loop seats at a single
    winding depends on WHERE the pass starts. Rotating to a seatable start lets
    ``split_on_winding`` keep the whole loop as **one pass**. If no start seats
    (range narrower than the sweep), the ring is returned as-is and the planner
    degrades it: reachable arcs forward, unreachable arcs REVERSED
    (``split_unreachable``), broken with airborne unwinds — never an invalid pass.
    Non-closed input is returned unchanged.
    """
    window = seat_window(pts, c_axis)
    if not window:
        return pts                                # arcs + reversals downstream (safe)
    return _rotate_ring(pts, window[0])


# ---- seam placement (port #3) ---------------------------------------------------
#
# A Python port of PrusaSlicer's seam-placement architecture (src/libslic3r/
# GCode/SeamPlacer.cpp + SeamAligned/SeamRandom/SeamShells, (c) Prusa Research,
# AGPLv3 — policy structure ported with permission of the project license),
# with the scoring inputs replaced: we have no visibility/overhang concept; our
# hard input is the winding SEAT WINDOW (``seat_window``), and the guard that
# matters is DEPOSIT-LENGTH preservation (a start that lands a split cut within
# ``min_deposit_len`` of the seam silently drops bead — stacked vertically by
# the aligned policy, that is a part-scrapping channel).


@dataclass
class SeamContext:
    """Per-plan seam-policy state, created and rolled by ``plan_toolpath``.

    ``prev_seams`` / ``layer_seams`` hold ``(xy, ring_bbox)`` per placed ring —
    the bbox is the ring identity for aligned chains (the SeamShells
    bounding-box matching, reduced: no one-to-one claiming at our ring counts).
    ``last_xy`` is the within-layer chain position for ``nearest`` — a
    PLAN-ORDER chain (seam of ring k+1 nearest the seam of ring k), NOT
    PrusaSlicer's emit-time nozzle position: rings that split into arcs end far
    from their seam, and perimeter walls are planned before the infill they
    follow. ``rng`` is one deterministic stream per plan (``SEAM_RNG_SEED``).
    """

    policy: str
    one_pass: bool
    align_radius: float
    v_mm_min: float
    rng: random.Random = field(default_factory=lambda: random.Random(SEAM_RNG_SEED))
    prev_seams: List[tuple] = field(default_factory=list)
    layer_seams: List[tuple] = field(default_factory=list)
    last_xy: Optional[Tuple[float, float]] = None
    notes: List[str] = field(default_factory=list)
    _warned_window: bool = False

    @classmethod
    def from_cfg(cls, cfg: Config, *, v_mm_min: float) -> "SeamContext":
        return cls(policy=cfg.fill.seam_position,
                   one_pass=cfg.fill.seam_prefer_one_pass,
                   align_radius=cfg.fill.seam_align_radius_mm,
                   v_mm_min=v_mm_min)

    def next_layer(self) -> None:
        # a ring-less layer (thin neck, empty region) must not wipe the chain:
        # PrusaSlicer chains seam strings across such gaps, so do we
        if self.layer_seams:
            self.prev_seams = self.layer_seams
        self.layer_seams = []
        self.last_xy = None            # the first ring of a layer falls back to
        #                                the previous layer's last seam

    def record(self, xy: Tuple[float, float], bbox: tuple) -> None:
        self.layer_seams.append((xy, bbox))
        self.last_xy = xy

    def note_window_constrained(self, n_candidates: int) -> None:
        if not self._warned_window:
            self._warned_window = True
            self.notes.append(
                f"seam policy '{self.policy}' is constrained to a "
                f"{n_candidates}-vertex seat window by the axis range (one-pass "
                "rings pin the seam where A meets the range stop) — scattering "
                "needs a calibrated range wider than 360° or "
                "fill.seam_prefer_one_pass: false (which trades one extra "
                "airborne unwind + lead pair per ring)")


def _ring_bbox(cycle: List[Tuple[float, float]]) -> tuple:
    xs = [p[0] for p in cycle]
    ys = [p[1] for p in cycle]
    return (min(xs), min(ys), max(xs), max(ys))


def _bbox_distance(a: tuple, b: tuple) -> float:
    """Max corner-to-corner distance (the SeamGeometry bounding_box_distance
    port) — 0 for identical boxes, robust ring identity across layers."""
    return max(math.hypot(a[0] - b[0], a[1] - b[1]),
               math.hypot(a[2] - b[2], a[3] - b[3]))


def _dropped_len(subs: List[list], min_len: float) -> float:
    def plen(path):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1])
                   for a, b in zip(path, path[1:]))
    return sum(plen(s) for s in subs if plen(s) < min_len)


def _dry_run(ring: Ring, cfg: Config, v_mm_min: float):
    """(sub-paths, dropped-length) the real pipeline would produce for this
    start — the deposit-loss guard's oracle (shared code with _curved_passes,
    so it cannot drift). ``(None, inf)`` where reachability would raise."""
    from ..toolpath.passplan import curved_subpaths

    try:
        subs = curved_subpaths(ring, cfg, v_mm_min)
    except ValueError:
        return None, math.inf
    return subs, _dropped_len(subs, cfg.process.min_deposit_len_mm)


def _policy_candidate_order(cycle, candidates: List[int], baseline_m: Optional[int],
                            ctx: SeamContext) -> List[int]:
    """Candidates in policy-preference order (best first). Ties break to the
    lowest index (the PrusaSlicer nearest tiebreak)."""
    n = len(cycle)

    def by_distance_to(target):
        return sorted(candidates,
                      key=lambda m: (math.hypot(cycle[m][0] - target[0],
                                                cycle[m][1] - target[1]), m))

    if ctx.policy == "nearest":
        target = ctx.last_xy
        if target is None and ctx.prev_seams:
            target = ctx.prev_seams[-1][0]
        if target is None:
            return sorted(candidates, key=lambda m: (0 if m == baseline_m else 1, m))
        return by_distance_to(target)

    if ctx.policy == "aligned":
        bbox = _ring_bbox(cycle)
        target = None
        if ctx.prev_seams:
            xy, _ = min(ctx.prev_seams,
                        key=lambda s: _bbox_distance(s[1], bbox))
            # accept only if the CANDIDATE set can actually get near the target
            # (a narrow window 100 mm away is not an alignment, it's a teleport)
            if min(math.hypot(cycle[m][0] - xy[0], cycle[m][1] - xy[1])
                   for m in candidates) <= ctx.align_radius:
                target = xy
        if target is None:                        # chain birth: deterministic
            return sorted(candidates, key=lambda m: (0 if m == baseline_m else 1, m))
        return by_distance_to(target)

    # random: outgoing-arc-length-weighted draw without replacement (the
    # SeamRandom arc-length-uniform pick, restricted to vertices — inserting an
    # interpolated start would break the pure-rotation contract). Lazy: the
    # guard loop usually accepts the first draw, so the full permutation is
    # never materialized.
    def _random_order():
        pool = list(candidates)
        weights = [max(math.hypot(cycle[(m + 1) % n][0] - cycle[m][0],
                                  cycle[(m + 1) % n][1] - cycle[m][1]), 1e-9)
                   for m in pool]
        while pool:
            pick = ctx.rng.choices(range(len(pool)), weights=weights)[0]
            weights.pop(pick)
            yield pool.pop(pick)

    return _random_order()


def choose_seam_start(pts: Ring, cfg: Config, ctx: Optional[SeamContext]) -> Ring:
    """Rotate a closed ring to its policy-chosen seam (port #3).

    ``extreme`` (or no context) is exactly :func:`rotate_ring_to_extreme`. The
    other policies choose among CANDIDATE starts — the seat window when
    ``fill.seam_prefer_one_pass`` and the window actually yields one pass
    (dry-run-verified: a window blind to corner/curvature splits must not
    confiscate seam freedom for a one-pass that never materializes), otherwise
    every vertex (the ring splits regardless; the seam then places one of the
    cuts, costing ≥ 1 winding split vs a seated start).

    Every non-baseline choice is guarded against DEPOSIT LOSS: the rotated ring
    is dry-run through the real split chain and accepted only if it drops no
    more sub-min-length bead than the baseline (extreme) start would; otherwise
    the next-preferred candidate is tried, falling back to the baseline.
    """
    if ctx is None or ctx.policy not in ("nearest", "aligned", "random"):
        # extreme, or an unknown policy string (a foreign preset/project value
        # — plan_toolpath warns; library callers degrade safely, never scatter)
        return rotate_ring_to_extreme(pts, cfg.c_axis)
    if len(pts) < 4 or pts[0] != pts[-1]:
        return pts
    cycle = pts[:-1]
    n = len(cycle)
    window = seat_window(pts, cfg.c_axis)

    baseline_m = window[0] if window else None
    baseline = _rotate_ring(pts, baseline_m) if window else pts
    base_subs, base_lost = _dry_run(baseline, cfg, ctx.v_mm_min)

    candidates = list(window)
    window_restricted = True
    if not window or not ctx.one_pass or base_subs is None or len(base_subs) > 1:
        candidates = list(range(n))
        window_restricted = False
    elif len(candidates) <= 2:
        ctx.note_window_constrained(len(candidates))

    chosen = baseline
    chosen_m = baseline_m
    for m in _policy_candidate_order(cycle, candidates, baseline_m, ctx):
        if m == baseline_m:
            chosen, chosen_m = baseline, baseline_m
            break
        ring = _rotate_ring(pts, m)
        subs, lost = _dry_run(ring, cfg, ctx.v_mm_min)
        # a window-restricted candidate must also DELIVER the one pass the
        # window was kept for: the seat window is winding-only, but a sharp
        # corner AT the baseline seam becomes a mandatory split from any other
        # start — accepting it would confiscate seam freedom AND still pay the
        # extra unwind + lead pair the one-pass preference promises to avoid
        if subs is not None and lost <= base_lost + 1e-9 \
                and (not window_restricted or len(subs) <= len(base_subs)):
            chosen, chosen_m = ring, m
            break
    ctx.record(cycle[chosen_m] if chosen_m is not None else chosen[0],
               _ring_bbox(cycle))
    return chosen


def contour_paths(region, cfg: Config, mode: str = "contour",
                  seam_ctx: Optional[SeamContext] = None) -> List[Ring]:
    """Wall centreline paths for one region, ready for the constraint pipeline,
    **innermost first** (deposit inner rings before the outer rings that will
    surround them — their lead-outs then cross only not-yet-deposited paths).

    ``mode``: ``"contour"`` (all rings) or ``"outline"`` (outermost only). Each
    ring starts at its seam (``choose_seam_start`` — the rotational extreme
    without a ``seam_ctx``); the caller still applies ``split_on_curvature`` +
    ``split_on_winding`` (exactly like streamlines).
    """
    if mode not in ("contour", "outline"):
        raise ValueError(f"contour mode must be 'contour' or 'outline', got {mode!r}")
    rings = contour_rings(region, cfg, max_loops=1 if mode == "outline" else 0)
    rings.reverse()                               # innermost first
    return [choose_seam_start(r, cfg, seam_ctx) for r in rings]


def perimeter_paths(region, cfg: Config,
                    seam_ctx: Optional[SeamContext] = None) -> Tuple[List[Ring], int]:
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
    return [choose_seam_start(r, cfg, seam_ctx) for r in rings], len(levels)
