"""Segment fill into passes (constant v, RPM), order, lead-in/out, lifts. SPEC §4.5.

A *pass* is one forward deposition move at a single operating point: constant
traverse ``v`` and constant ``RPM`` (so revs/mm is constant within the pass — SPEC
§2.2/§4.5). RPM changes only between passes, airborne. M2 builds straight +Y passes
from the unidirectional raster; the process window (per-region v/RPM) lands in M3.

Pure data + plain tuples — no mesh library and no shapely here (the raster returns
tuples), so this stays planning-layer clean (SPEC §3.3).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

from ..config import Config
from ..fill.curvature import min_radius
from ..fill.raster import raster_lines, raster_pitch
from ..fill.heading import (
    heading_deg_from_vector, heading_to_a_deg, unwrap_headings, winding_shift,
)
from ..process.extrusion import e_per_path_mm
from ..process.screener import OperatingPoint


@dataclass
class Pass:
    """One forward deposition pass at a single operating point.

    A straight pass is two points (``[start, end]``); a curved streamline pass
    (M5) carries a full ``points`` polyline. The heading — and therefore the rotary
    A angle — is constant within each *segment* but may change between segments.
    """

    start: tuple          # (x, y) plunge point
    end: tuple            # (x, y) deposition end (lead-out begins here)
    z: float              # deposition Z
    a_deg: float          # rotary A for the FIRST segment's heading
    rpm: int
    traverse_mm_min: float
    e_per_path_mm: float
    points: Optional[list] = None   # full polyline; None => straight [start, end]

    def __post_init__(self):
        if self.points is None:
            self.points = [tuple(self.start), tuple(self.end)]

    @classmethod
    def curved(cls, points, *, z, rpm, traverse_mm_min, e_per_path_mm, c_axis):
        pts = [tuple(p) for p in points]
        h0 = heading_deg_from_vector(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1])
        return cls(start=pts[0], end=pts[-1], z=z,
                   a_deg=heading_to_a_deg(h0, c_axis), rpm=rpm,
                   traverse_mm_min=traverse_mm_min, e_per_path_mm=e_per_path_mm, points=pts)

    @property
    def is_curved(self) -> bool:
        return len(self.points) > 2

    @property
    def heading_deg(self) -> float:
        return heading_deg_from_vector(self.end[0] - self.start[0],
                                       self.end[1] - self.start[1])

    @property
    def length_mm(self) -> float:
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in self.segments())

    @property
    def e_total_mm(self) -> float:
        return self.e_per_path_mm * self.length_mm

    @property
    def min_radius_mm(self) -> float:
        return min_radius(self.points)

    def segments(self):
        return list(zip(self.points, self.points[1:]))

    def segment_headings_deg(self) -> List[float]:
        return [heading_deg_from_vector(b[0] - a[0], b[1] - a[1])
                for a, b in self.segments()]

    def segment_a_degs(self, c_axis) -> List[float]:
        """Naive per-segment A (each from its own heading; may wrap at ±180)."""
        return [heading_to_a_deg(h, c_axis) for h in self.segment_headings_deg()]

    def axis_angles(self, c_axis) -> List[float]:
        """Continuous, in-range rotary A per segment — winding-resolved (D13).

        The travel headings are unwrapped (no ±360 jumps), mapped to A, then shifted by
        a whole turn so the band sits inside the usable axis range ``[a_min, a_max]``.
        Within a pass A therefore evolves continuously and never wraps; this is what the
        emitter commands and what the winding-range validation checks. The planner has
        already split the path (``split_on_winding``) so the band fits one winding.
        """
        cont = unwrap_headings(self.segment_headings_deg())
        a_cont = [heading_to_a_deg(t, c_axis) for t in cont]
        shift = winding_shift(min(a_cont), max(a_cont),
                              c_axis.a_min_deg, c_axis.a_max_deg)
        return [a + shift for a in a_cont]


@dataclass
class LayerPlan:
    index: int
    z: float
    passes: List[Pass] = field(default_factory=list)

    @property
    def is_empty(self) -> bool:
        return not self.passes


@dataclass
class ToolpathPlan:
    layers: List[LayerPlan]
    rpm: int
    traverse_mm_min: float
    v_grind_floor_mm_min: float
    warnings: List[str] = field(default_factory=list)   # planner notes (e.g. a
    #                     seam policy constrained by the axis range) — surfaced
    #                     by the GUI summary and the CLI, never fatal

    @property
    def revs_per_mm(self) -> float:
        return self.rpm / self.traverse_mm_min if self.traverse_mm_min else 0.0

    @property
    def npasses(self) -> int:
        return sum(len(ly.passes) for ly in self.layers)

    @property
    def nonempty_layers(self) -> List[LayerPlan]:
        return [ly for ly in self.layers if not ly.is_empty]


def default_operating_point(cfg: Config) -> OperatingPoint:
    """Fallback operating point when no screener CSV is supplied (M2).

    Single-speed: traverse = the deposition feed primitive, RPM = the spindle floor
    (a documented placeholder until the screener sets per-region RPM in M3). Because
    there is one speed, the grind floor equals the traverse — every deposition move
    is exactly at the floor, which satisfies the contact invariant.
    """
    v = cfg.emit.feed_dep_mm_min
    rpm = cfg.spindle.rpm_min
    return OperatingPoint(
        revs_per_mm=rpm / v if v else 0.0,
        v_min_mm_min=v, v_max_mm_min=v,
        rpm=rpm, traverse_mm_min=v,
        feed_speed_mm_min=0.0, phi=0.0, torque_Nm=0.0, power_kW=0.0, t_az_c=0.0,
    )


def _e_per_path(cfg: Config, op: OperatingPoint, has_screener: bool) -> float:
    """Wire-mm per path-mm (SPEC §5.3). Screener if available, else x/volume."""
    mode = cfg.extrusion.mode
    if has_screener and mode == "screener":
        return e_per_path_mm("screener",
                             feed_speed_mm_min=op.feed_speed_mm_min,
                             traverse_mm_min=op.traverse_mm_min)
    if mode == "screener":          # configured for screener but none supplied
        mode = "volume"             # SPEC §5.3 fallback
    return e_per_path_mm(
        mode,
        bead_width_mm=cfg.process.bead_width_mm,
        layer_height_mm=cfg.process.layer_height_mm,
        wire_diameter_mm=cfg.process.wire_diameter_mm,
        x_ratio=cfg.extrusion.x_ratio,
    )


def order_passes_lead_away(passes: List[Pass]) -> List[Pass]:
    """Order passes so each leads AWAY from already-laid material (SPEC §4.6).

    Deposit the least-forward passes first (sorted by the lead-out edge's projection
    onto the heading), so the leading wire always advances into free space and never
    drives into a previously-deposited bead ahead. Ties (same forward extent — e.g. a
    convex part where every +Y line ends at the same Y) fall back to the perpendicular
    coordinate for a deterministic left-to-right sweep.
    """
    if not passes:
        return passes
    hx, hy = _heading_unit(passes[0].start, passes[0].end)

    def fwd(p):       # forward extent = lead-out edge projected on the heading
        return p.end[0] * hx + p.end[1] * hy

    def perp(p):      # perpendicular position (heading rotated -90 deg)
        return p.start[0] * hy - p.start[1] * hx

    return sorted(passes, key=lambda p: (round(fwd(p), 6), round(perp(p), 6)))


def _heading_unit(start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    n = math.hypot(dx, dy)
    return (dx / n, dy / n) if n else (0.0, 1.0)


def _polyline_len(pts) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def _band_fits(lo: float, hi: float, a_min: float, a_max: float,
               tol: float = 1e-6) -> bool:
    """True if the continuous A-band ``[lo, hi]`` can be shifted by a whole turn to sit
    entirely inside ``[a_min, a_max]``.

    A band wider than the range never fits. A band that *straddles the linear ±range
    seam* may not fit even when it is narrower than the range — the axis cannot sweep
    continuously past its stop, so that crossing needs an airborne unwind. The condition
    is exactly: some integer winding ``k`` has ``a_min ≤ lo+360k`` and ``hi+360k ≤ a_max``.
    """
    k_lo = math.ceil((a_min - lo) / 360.0 - tol)
    k_hi = math.floor((a_max - hi) / 360.0 + tol)
    return k_lo <= k_hi


def _reachable(a_deg: float, a_min: float, a_max: float, tol: float = 1e-6) -> bool:
    """True if the heading's A is reachable at SOME winding (``_band_fits`` of a
    single angle): sub-360° ranges leave a gap of headings no winding can reach."""
    return _band_fits(a_deg, a_deg, a_min, a_max, tol)


def split_unreachable(points, c_axis, tol: float = 1e-6):
    """Make every segment's heading reachable, reversing arcs where needed (D13).

    A sub-360° axis range cannot reach every heading at any winding — but there is
    **no privileged direction**, so an arc whose forward heading falls in the
    unreachable gap can be deposited in REVERSE (θ+180°, which is reachable
    whenever the range is ≥180° wide). Splits the polyline into maximal runs of
    same-reachability segments and flips the unreachable runs; the boundaries are
    airborne reorients (pass breaks). Raises only when a heading is unreachable in
    BOTH directions (range narrower than 180°) — genuinely impossible, reorient the
    part. On a ≥360° range every heading is reachable and this returns ``[points]``.
    """
    pts = [tuple(p) for p in points]
    if len(pts) < 2:
        return []
    a_min, a_max = c_axis.a_min_deg, c_axis.a_max_deg
    segs_a = [heading_to_a_deg(heading_deg_from_vector(b[0] - a[0], b[1] - a[1]), c_axis)
              for a, b in zip(pts, pts[1:])]
    ok = []
    for a in segs_a:
        fwd = _reachable(a, a_min, a_max, tol)
        if not fwd and not _reachable(a + 180.0, a_min, a_max, tol):
            raise ValueError(
                f"segment heading maps to A={a:.1f} deg, unreachable in either travel "
                f"direction within the axis range [{a_min}, {a_max}] — the range is "
                "narrower than 180 deg; widen it or reorient the part (D13)")
        ok.append(fwd)
    subs: List[list] = []
    start = 0
    for i in range(1, len(ok) + 1):
        if i == len(ok) or ok[i] != ok[start]:
            sub = pts[start:i + 1]                    # segments start..i-1
            subs.append(sub if ok[start] else list(reversed(sub)))
            start = i
    return subs


def split_on_winding(points, c_axis, tol: float = 1e-6):
    """Break a polyline where the accumulated axis angle can no longer be wound into the
    usable range (D13).

    The whole head rotates with the tangent, so a heading sweep drives the C axis along a
    continuous (unwrapped) A-band. Track that band; whenever including the next segment
    would make it impossible to seat the band in ``[a_min, a_max]`` at any single winding
    (see ``_band_fits``), cut there and start a new sub-path. Each returned sub-path can
    be wound into range (so ``Pass.axis_angles`` seats it); the cut between sub-paths is
    an **airborne unwind** (a pass boundary — the emitter lifts and reorients A between
    passes). A closed loop (~360° sweep) stays one pass only when the range can wind the
    whole turn; on a tighter range it splits into arcs + unwinds.
    """
    pts = [tuple(p) for p in points]
    if len(pts) < 3:
        return [pts]
    a_min, a_max = c_axis.a_min_deg, c_axis.a_max_deg
    cont = unwrap_headings([heading_deg_from_vector(b[0] - a[0], b[1] - a[1])
                            for a, b in zip(pts, pts[1:])])
    a_cont = [heading_to_a_deg(t, c_axis) for t in cont]   # one A per segment
    # A single heading whose A cannot be wound into range at ANY winding is physically
    # unreachable (range < 360 doesn't cover it) — not splittable. Fail early and clearly
    # rather than emit an out-of-range A that trips the emitter's range validator.
    for a in a_cont:
        if not _band_fits(a, a, a_min, a_max, tol):
            raise ValueError(
                f"segment heading maps to A={a:.1f} deg, unreachable within the axis range "
                f"[{a_min}, {a_max}] at any winding — widen the range or reorient the part (D13)")
    subs: List[list] = []
    cur = [pts[0], pts[1]]
    lo = hi = a_cont[0]
    for i in range(1, len(a_cont)):
        a = a_cont[i]
        nlo, nhi = min(lo, a), max(hi, a)
        if not _band_fits(nlo, nhi, a_min, a_max, tol):
            subs.append(cur)
            cur = [pts[i], pts[i + 1]]
            lo = hi = a
        else:
            cur.append(pts[i + 1])
            lo, hi = nlo, nhi
    subs.append(cur)
    return subs


def plan_axis_winding(passes, c_axis, tol: float = 1e-6):
    """Core winding manager (D13 / SPEC §4.1) — choose each pass's starting winding and
    insert airborne unwinds. **Not yet implemented.**

    Winding management is a core planner function under D13. For each pass this will:

    * pick the **starting winding** — the ``θ − home_heading ± 360k`` that lands the
      pass's first heading inside ``[c_axis.a_min_deg, c_axis.a_max_deg]`` while leaving
      the most rotation room for that pass's heading sweep;
    * track the **accumulated axis angle** across the pass and insert an **airborne
      unwind** (a lift + reorient at a pass boundary) wherever continuing would drive
      ``A`` past a stop;
    * prefer the **shortest legal rotation** between consecutive passes.

    Today ``split_on_winding`` only detects where a continuous A-band can no longer be
    seated, and ``Pass.axis_angles`` seats each sub-path via ``winding_shift``
    (start-agnostic). This function will supersede that ad-hoc seating with explicit,
    optimal starting-winding selection — closing the "closed-loop-in-one-pass" gap
    noted in DECISIONS D13.
    """
    raise NotImplementedError(
        "pass-planner winding management (starting-winding selection + airborne "
        "unwinds) is not implemented yet — SPEC §4.1 / DECISIONS D13")


def curved_subpaths(path, cfg: Config, v_mm_min: float) -> List[list]:
    """The D13 split chain for ONE curved path — reachability reversal (must run
    first: reversing a sub-path swaps which leg is "next" at every interior
    vertex), sharp-corner heading steps, the slew limit (SPEC §4.3), then the
    usable axis range (``split_on_winding``). Returns ALL sub-paths, including
    those below ``min_deposit_len`` — callers decide about drops. Shared by
    ``_curved_passes`` and the seam-placement deposit-loss guard
    (``fill.contour.choose_seam_start``), so the guard can never drift from the
    real pipeline. Raises ``ValueError`` where reachability does (a heading
    unreachable in either travel direction)."""
    from ..fill.curvature import split_on_curvature, split_on_heading_step

    v_s = v_mm_min / 60.0
    subs: List[list] = []
    for sub_r in split_unreachable(path, cfg.c_axis):
        for sub_s in split_on_heading_step(sub_r, v_s, cfg.c_axis.max_speed_deg_s,
                                           cfg.c_axis.max_scrub_deg_mm):
            for sub_c in split_on_curvature(sub_s, v_s, cfg.c_axis.max_speed_deg_s):
                subs.extend(split_on_winding(sub_c, cfg.c_axis))
    return subs


def _curved_passes(paths, layer_z, cfg: Config, op: OperatingPoint,
                   e_per_path: float) -> List[Pass]:
    """The D13 constraint pipeline for curved paths (streamlines, contour rings,
    perimeter walls) — ``curved_subpaths`` per path; sub-passes below
    ``min_deposit_len`` drop. Path order is preserved (callers order paths
    deliberately)."""
    min_len = cfg.process.min_deposit_len_mm
    out: List[Pass] = []
    for path in paths:
        for sub in curved_subpaths(path, cfg, op.traverse_mm_min):
            if _polyline_len(sub) >= min_len:
                out.append(Pass.curved(
                    sub, z=layer_z, rpm=op.rpm,
                    traverse_mm_min=op.traverse_mm_min,
                    e_per_path_mm=e_per_path, c_axis=cfg.c_axis))
    return out


def plan_layer(layer, cfg: Config, *, operating_point: OperatingPoint,
               e_per_path: float, heading_deg: float = 90.0,
               seam_ctx=None) -> LayerPlan:
    """Build one layer's passes — bidirectional raster, curved streamlines, or M17
    contour/outline rings, plus optional perimeter wall loops (D13).

    No wedge: every heading is depositable. Curved paths (streamlines AND closed
    contour rings) run the same constraint pipeline: slew split, winding split,
    min-length drop — the breaks are airborne reorients/unwinds. ``heading_deg`` is
    only the *base* hatch direction. With ``fill.perimeter_loops > 0`` the infill
    modes lay the hatch first, then the wall loops (infill lead-outs then cross only
    not-yet-deposited wall paths); the hatch region is inset past the walls.

    ``seam_ctx`` (a ``fill.contour.SeamContext``, normally created and rolled by
    ``plan_toolpath``) carries the seam-placement policy state; without it,
    non-``extreme`` ``fill.seam_position`` values degrade to the extreme start.
    """
    op = operating_point
    min_len = cfg.process.min_deposit_len_mm
    mode = cfg.fill.mode
    passes: List[Pass] = []

    if mode in ("contour", "outline"):
        from ..fill.contour import contour_paths

        for region in layer.regions:
            # innermost-first ring order from contour_paths; keep it (SPEC §4.6)
            passes.extend(_curved_passes(
                contour_paths(region, cfg, mode=mode, seam_ctx=seam_ctx),
                layer.z, cfg, op, e_per_path))
        return LayerPlan(index=layer.index, z=layer.z, passes=passes)

    # infill modes (raster | streamline), optionally wrapped by perimeter walls
    from ..fill.contour import inset_interior, perimeter_paths
    from ..fill.raster import best_heading_deg, dominant_heading_deg

    loops = cfg.fill.perimeter_loops
    wall_passes: List[Pass] = []
    for region in layer.regions:
        fitted = 0
        if loops > 0:
            walls, fitted = perimeter_paths(region, cfg, seam_ctx=seam_ctx)
            wall_passes.extend(_curved_passes(walls, layer.z, cfg, op, e_per_path))
        # inset by the walls that actually FIT — phantom walls would leave voids
        infill_region = inset_interior(region, cfg, fitted)
        if infill_region.is_empty:
            continue

        # Per-region heading (D13: no privileged direction — the choice is free):
        # * raster: SCORE candidate directions on the actual clipped hatch and take
        #   the winner (most kept bead, then fewest pieces among near-equals); the
        #   crosshatch delta is COMPOSED INTO the scored candidates, so the scored
        #   heading is exactly the laid heading; legacy +Y(+delta) is always a
        #   candidate, so coverage never regresses.
        # * streamline: bias along the region's LONG AXIS (+delta) — the guidance
        #   field bends toward boundaries, so aligning the bias with the dominant
        #   axis complements the curl (measured on real bracket geometry: about a
        #   third fewer passes, much less overlap, equal coverage; a straight-hatch
        #   score is a poor proxy for a boundary-following field).
        delta = heading_deg - 90.0
        if cfg.fill.auto_heading:
            if mode == "streamline":
                region_heading = (dominant_heading_deg(infill_region) + delta) % 360.0
            else:
                region_heading = best_heading_deg(infill_region, cfg, min_len,
                                                  delta_deg=delta)
        else:
            region_heading = heading_deg

        if mode == "streamline":
            from ..fill.streamline import streamline_fill

            region_passes = _curved_passes(
                streamline_fill(infill_region, cfg, heading_deg=region_heading),
                layer.z, cfg, op, e_per_path)
            # streamlines have no inherent order: lead away from laid material (§4.6)
            passes.extend(order_passes_lead_away(region_passes))
        else:  # raster
            pitch = raster_pitch(cfg)
            for start, end in raster_lines(infill_region, pitch,
                                           heading_deg=region_heading, min_len=min_len,
                                           bidirectional=cfg.fill.raster_bidirectional):
                # Reachability on a calibrated sub-360° axis range (D13): a line
                # whose heading no winding can reach deposits in REVERSE (its
                # +180° heading is reachable whenever the range is ≥180° wide);
                # unreachable both ways fails loud inside split_unreachable.
                a = heading_to_a_deg(
                    heading_deg_from_vector(end[0] - start[0], end[1] - start[1]),
                    cfg.c_axis)
                if not _reachable(a, cfg.c_axis.a_min_deg, cfg.c_axis.a_max_deg):
                    (sub,) = split_unreachable([start, end], cfg.c_axis)
                    start, end = sub[0], sub[1]
                    a = heading_to_a_deg(
                        heading_deg_from_vector(end[0] - start[0], end[1] - start[1]),
                        cfg.c_axis)
                passes.append(Pass(
                    start=start, end=end, z=layer.z, a_deg=a,
                    rpm=op.rpm, traverse_mm_min=op.traverse_mm_min,
                    e_per_path_mm=e_per_path))
            # raster_lines already returns lines left-to-right; bidirectional
            # alternation IS the deposit order (boustrophedon) — keep it (D13).

    passes.extend(wall_passes)      # walls after infill (lead-out crossing order)
    return LayerPlan(index=layer.index, z=layer.z, passes=passes)


def layer_heading_deg(cfg: Config, layer_index: int, base_deg: float = 90.0) -> float:
    """Per-layer deposition heading. With crosshatch on, alternate +/- the crosshatch
    angle about +Y between layers so adjacent layers cross (SPEC §4.2)."""
    if not cfg.fill.crosshatch:
        return base_deg
    theta = cfg.fill.crosshatch_angle_deg
    return base_deg + (theta if layer_index % 2 == 0 else -theta)


def plan_toolpath(model, cfg: Config, *,
                  operating_point: Optional[OperatingPoint] = None,
                  heading_deg: float = 90.0) -> ToolpathPlan:
    """Plan every layer into constant-(v, RPM) passes (raster or streamline; SPEC §4.2/§4.5)."""
    has_screener = operating_point is not None
    op = operating_point or default_operating_point(cfg)
    e_pp = _e_per_path(cfg, op, has_screener)

    # seam placement (port #3): one context per plan carries the policy state —
    # the deterministic RNG, the previous layer's seam points (aligned chains),
    # and the within-layer chain position (nearest)
    seam_ctx = None
    plan_warnings: List[str] = []
    seam_policy = cfg.fill.seam_position
    if seam_policy not in ("extreme", "nearest", "aligned", "random"):
        plan_warnings.append(
            f"unknown fill.seam_position {seam_policy!r}; using 'extreme' "
            "(valid: extreme, nearest, aligned, random)")
        seam_policy = "extreme"
    rings_possible = (cfg.fill.mode in ("contour", "outline")
                      or cfg.fill.perimeter_loops > 0)
    if rings_possible and seam_policy != "extreme":
        from ..fill.contour import SeamContext

        seam_ctx = SeamContext.from_cfg(cfg, v_mm_min=op.traverse_mm_min)

    layers = []
    for layer in model.layers:
        layers.append(plan_layer(
            layer, cfg, operating_point=op, e_per_path=e_pp,
            heading_deg=layer_heading_deg(cfg, layer.index, heading_deg),
            seam_ctx=seam_ctx))
        if seam_ctx is not None:
            seam_ctx.next_layer()
    if seam_ctx is not None:
        plan_warnings.extend(seam_ctx.notes)
    return ToolpathPlan(
        layers=layers, rpm=op.rpm, traverse_mm_min=op.traverse_mm_min,
        v_grind_floor_mm_min=op.v_grind_floor_mm_min,
        warnings=plan_warnings,
    )
