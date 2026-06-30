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
from ..fill.raster import raster_lines, raster_pitch
from ..fill.wedge import heading_deg_from_vector, heading_to_a_deg, in_wedge
from ..process.extrusion import e_per_path_mm
from ..process.screener import OperatingPoint


@dataclass
class Pass:
    """One forward deposition pass (a straight segment for M2)."""

    start: tuple          # (x, y) plunge point
    end: tuple            # (x, y) deposition end (lead-out begins here)
    z: float              # deposition Z
    a_deg: float          # rotary axis angle for the heading (constant for a straight pass)
    rpm: int
    traverse_mm_min: float
    e_per_path_mm: float

    @property
    def heading_deg(self) -> float:
        return heading_deg_from_vector(self.end[0] - self.start[0],
                                       self.end[1] - self.start[1])

    @property
    def length_mm(self) -> float:
        return math.hypot(self.end[0] - self.start[0], self.end[1] - self.start[1])

    @property
    def e_total_mm(self) -> float:
        return self.e_per_path_mm * self.length_mm


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


def plan_layer(layer, cfg: Config, *, operating_point: OperatingPoint,
               e_per_path: float, heading_deg: float = 90.0) -> LayerPlan:
    """Build the straight +Y passes for one sliced layer (SPEC §4.2/§4.5)."""
    a_deg = heading_to_a_deg(heading_deg, cfg.c_axis)
    if not in_wedge(a_deg, cfg.c_axis):
        raise ValueError(
            f"raster heading {heading_deg} deg -> A={a_deg:.1f} deg outside the "
            f"+/-{cfg.c_axis.wedge_half_angle_deg} deg wedge (SPEC §4.1)")

    pitch = raster_pitch(cfg)
    passes: List[Pass] = []
    for region in layer.regions:
        segs = raster_lines(region, pitch, heading_deg=heading_deg,
                            min_len=cfg.process.min_deposit_len_mm)
        for start, end in segs:
            passes.append(Pass(
                start=start, end=end, z=layer.z, a_deg=a_deg,
                rpm=operating_point.rpm,
                traverse_mm_min=operating_point.traverse_mm_min,
                e_per_path_mm=e_per_path,
            ))
    # Pass ordering (SPEC §4.6): lead away from already-laid material.
    passes = order_passes_lead_away(passes)
    return LayerPlan(index=layer.index, z=layer.z, passes=passes)


def plan_toolpath(model, cfg: Config, *,
                  operating_point: Optional[OperatingPoint] = None,
                  heading_deg: float = 90.0) -> ToolpathPlan:
    """Plan every layer of a SlicedModel into constant-(v, RPM) straight passes."""
    has_screener = operating_point is not None
    op = operating_point or default_operating_point(cfg)
    e_pp = _e_per_path(cfg, op, has_screener)

    layers = [
        plan_layer(layer, cfg, operating_point=op, e_per_path=e_pp,
                   heading_deg=heading_deg)
        for layer in model.layers
    ]
    return ToolpathPlan(
        layers=layers, rpm=op.rpm, traverse_mm_min=op.traverse_mm_min,
        v_grind_floor_mm_min=op.v_grind_floor_mm_min,
    )
