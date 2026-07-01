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


def plan_layer(layer, cfg: Config, *, operating_point: OperatingPoint,
               e_per_path: float, heading_deg: float = 90.0) -> LayerPlan:
    """Build one layer's passes — bidirectional raster, or curved streamlines (D13).

    No wedge: every heading is depositable. Curved paths are split first by the slew
    limit (``R >= R_min``, SPEC §4.3) and then by the usable axis range
    (``split_on_winding``), so every emitted pass is legal; the breaks are airborne
    reorients/unwinds. ``heading_deg`` is only the *base* hatch direction.
    """
    op = operating_point
    min_len = cfg.process.min_deposit_len_mm
    passes: List[Pass] = []

    if cfg.fill.mode == "streamline":
        from ..fill.curvature import split_on_curvature
        from ..fill.streamline import streamline_fill

        v_s = op.traverse_mm_min / 60.0
        for region in layer.regions:
            for path in streamline_fill(region, cfg, heading_deg=heading_deg):
                # slew limit first (SPEC §4.3), then winding range (D13)
                for sub_c in split_on_curvature(path, v_s, cfg.c_axis.max_speed_deg_s):
                    for sub in split_on_winding(sub_c, cfg.c_axis):
                        if _polyline_len(sub) >= min_len:
                            passes.append(Pass.curved(
                                sub, z=layer.z, rpm=op.rpm,
                                traverse_mm_min=op.traverse_mm_min,
                                e_per_path_mm=e_per_path, c_axis=cfg.c_axis))
        # streamlines have no inherent deposit order: lead away from laid material (§4.6)
        passes = order_passes_lead_away(passes)
    else:  # raster
        pitch = raster_pitch(cfg)
        for region in layer.regions:
            for start, end in raster_lines(region, pitch, heading_deg=heading_deg,
                                           min_len=min_len,
                                           bidirectional=cfg.fill.raster_bidirectional):
                passes.append(Pass(
                    start=start, end=end, z=layer.z,
                    a_deg=heading_to_a_deg(
                        heading_deg_from_vector(end[0] - start[0], end[1] - start[1]),
                        cfg.c_axis),
                    rpm=op.rpm, traverse_mm_min=op.traverse_mm_min,
                    e_per_path_mm=e_per_path))
        # raster_lines already returns lines left-to-right; bidirectional alternation IS
        # the deposit order (boustrophedon), so keep it rather than re-sorting (D13).

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

    layers = [
        plan_layer(layer, cfg, operating_point=op, e_per_path=e_pp,
                   heading_deg=layer_heading_deg(cfg, layer.index, heading_deg))
        for layer in model.layers
    ]
    return ToolpathPlan(
        layers=layers, rpm=op.rpm, traverse_mm_min=op.traverse_mm_min,
        v_grind_floor_mm_min=op.v_grind_floor_mm_min,
    )
