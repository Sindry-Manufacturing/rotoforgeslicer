"""Kinematic simulation of a planned toolpath. SPEC §9 (studio Preview mode).

Time-parameterizes the U2 tagged segments (``toolpath.segments``) with the same
feedrates the emitter commands, producing a timeline the GUI can scrub or play:

* deposition runs at the pass traverse (planar G1s, F == XY speed);
* the lead-in plunge uses the emitter's §6.2 compensation: F holds the XY speed at
  the traverse against the plunge **arc** length, and E feeds over that arc — while
  the single plunge G1 spans the straight chord (matters for curved passes);
* the lead-out runs at the Z feed applied to the full 3D move (as emitted);
* airborne travels / lifts / resets run at the travel / Z feed primitives, with a
  **slew floor** on travels: the reorientation to the next pass's A cannot beat
  ``ΔA / ω_C`` (a 180° bidirectional-raster flip dominates its tiny XY hop);
* the airborne spindle dwells (startup settle on the first M3, stabilization on
  every RPM hop — SPEC §4.5/§6.1) appear as zero-motion timeline events after the
  travel to the pass start, exactly where the emitter dwells.

Everything here is pure (no Qt / pyvista / matplotlib), so the timing, E
accumulation, and readout math are unit-tested headless. E accumulates on contact
moves only and is monotonic by construction (invariant 4); per-event RPM/traverse
are constant, mirroring invariant 5.
"""
from __future__ import annotations

import math
from bisect import bisect_right
from dataclasses import dataclass
from typing import List, Optional, Tuple

from ..config import CAxisCfg, Config
from ..toolpath.segments import SegmentKind, ToolpathSegment

Point3 = Tuple[float, float, float]

_CONTACT = frozenset({SegmentKind.DEPOSITION, SegmentKind.LEAD_IN, SegmentKind.LEAD_OUT})


def wheel_heading_deg(a_deg: float, c_axis: CAxisCfg) -> float:
    """Inverse of ``fill.heading.heading_to_a_deg``: the wheel's travel-direction
    heading (deg CCW from +X) recovered from a commanded A."""
    return c_axis.home_heading_deg + (a_deg - c_axis.home_offset_deg) / c_axis.invert_sign


@dataclass(frozen=True)
class TimelineEvent:
    """One constant-rate interval: a segment traversal, or a zero-motion dwell."""

    t0: float                      # seconds
    t1: float
    kind: str                      # SegmentKind value, or "dwell"
    pos0: Point3
    pos1: Point3
    a0: float                      # commanded A at t0 / t1 (lerped between)
    a1: float
    e0: float                      # cumulative wire E (mm) at t0 / t1
    e1: float
    rpm: int                       # spindle RPM in effect during the interval (0 = off)
    v_mm_min: float                # commanded feed (0 for a dwell)
    layer_index: Optional[int] = None
    pass_index: Optional[int] = None

    @property
    def duration(self) -> float:
        return self.t1 - self.t0

    @property
    def in_contact(self) -> bool:
        return self.kind in {k.value for k in _CONTACT}


@dataclass(frozen=True)
class SimState:
    """The machine state at one instant, for the head actor + readouts."""

    t: float
    x: float
    y: float
    z: float
    a_deg: float
    wheel_heading_deg: float
    kind: str
    in_contact: bool
    rpm: int
    v_mm_min: float
    revs_per_mm: float             # rpm / traverse while in contact, else 0
    e_mm: float
    layer_index: Optional[int]
    pass_index: Optional[int]


def _xy_len(seg: ToolpathSegment) -> float:
    return math.hypot(seg.end[0] - seg.start[0], seg.end[1] - seg.start[1])


def _plunge_arc_mm(pass_, cfg: Config) -> float:
    """The lead-in's arc length ALONG the pass polyline (emit/rrf.py step 4)."""
    return min(cfg.process.lead_in_len_mm, 0.5 * pass_.length_mm)


def _duration_s(seg: ToolpathSegment, pass_, cfg: Config) -> Tuple[float, float]:
    """(duration seconds, commanded feed mm/min) for one segment, mirroring the
    emitter's feed choices (see module docstring)."""
    if seg.kind is SegmentKind.LEAD_IN and pass_ is not None:
        # The emitted F realizes XY speed == traverse against the plunge ARC length
        # (the §6.2 compensation in emit/rrf.py step 4); the single plunge G1 itself
        # spans the straight CHORD, so its machine time is the chord 3D length at
        # that commanded F. For a straight pass chord == arc and this reduces to
        # xy_len / traverse; for a curved (streamline) pass they differ.
        v = pass_.traverse_mm_min
        plunge = _plunge_arc_mm(pass_, cfg)
        if plunge > 1e-9:
            f = v * math.hypot(plunge, cfg.process.approach_clearance_mm) / plunge
            return seg.length_mm / (f / 60.0), v
        return seg.length_mm / (cfg.emit.feed_z_mm_min / 60.0), cfg.emit.feed_z_mm_min
    if seg.kind is SegmentKind.DEPOSITION and pass_ is not None:
        v = pass_.traverse_mm_min
        xy = _xy_len(seg)
        if xy > 1e-9:
            return xy / (v / 60.0), v
        return seg.length_mm / (cfg.emit.feed_z_mm_min / 60.0), cfg.emit.feed_z_mm_min
    if seg.kind is SegmentKind.LEAD_OUT:
        v = cfg.emit.feed_z_mm_min          # emitted with f=f_z over the 3D move
        return seg.length_mm / (v / 60.0), v
    if seg.kind is SegmentKind.TRAVEL:
        v = cfg.emit.feed_travel_mm_min
        return seg.length_mm / (v / 60.0), v
    v = cfg.emit.feed_z_mm_min              # LIFTOFF / RESET (pure Z)
    return seg.length_mm / (v / 60.0), v


def build_timeline(segments: List[ToolpathSegment], plan, cfg: Config) -> List[TimelineEvent]:
    """Timeline events for a segment list, including the airborne spindle dwells."""
    pass_of = {}
    for lp in plan.layers:
        for i, p in enumerate(lp.passes):
            pass_of[(lp.index, i)] = p

    events: List[TimelineEvent] = []
    t = 0.0
    e = 0.0
    cur_a = 0.0                    # A homed at 0
    cur_rpm = 0                    # spindle off until the first M3

    for seg in segments:
        p = pass_of.get((seg.layer_index, seg.pass_index))
        a1 = seg.a_deg if seg.a_deg is not None else cur_a
        dur, v = _duration_s(seg, p, cfg)
        # Airborne reorientation floor: the travel G0 also carries the next pass's A
        # (emit/rrf.py step 1), and the C axis cannot slew faster than ω_C — a 180°
        # bidirectional-raster flip takes at least ΔA/ω_C regardless of the tiny XY
        # hop. (Exact RRF combined linear+rotary feed semantics are the SPEC §13
        # calibration item; this is the physical lower bound.) ω_C == 0 ⇒ uncalibrated,
        # no floor.
        if seg.kind is SegmentKind.TRAVEL and cfg.c_axis.max_speed_deg_s > 0:
            dur = max(dur, abs(a1 - cur_a) / cfg.c_axis.max_speed_deg_s)
        if seg.kind is SegmentKind.LEAD_IN and p is not None:
            de = p.e_per_path_mm * _plunge_arc_mm(p, cfg)   # arc, not chord (emitted E)
        elif seg.kind is SegmentKind.DEPOSITION and p is not None:
            de = p.e_per_path_mm * _xy_len(seg)
        else:
            de = 0.0
        if dur > 1e-12:
            events.append(TimelineEvent(
                t0=t, t1=t + dur, kind=seg.kind.value, pos0=seg.start, pos1=seg.end,
                a0=cur_a, a1=a1, e0=e, e1=e + de, rpm=cur_rpm, v_mm_min=v,
                layer_index=seg.layer_index, pass_index=seg.pass_index))
            t += dur
        e += de
        cur_a = a1

        # The emitter places M3 + dwell right after the airborne travel to a pass
        # whose RPM differs from the current spindle speed (SPEC §4.5/§6.1).
        if seg.kind is SegmentKind.TRAVEL and p is not None and p.rpm != cur_rpm:
            first = cur_rpm == 0
            settle_ms = (cfg.process.startup_settle_ms if first
                         else cfg.process.spindle_dwell_ms)
            cur_rpm = p.rpm
            if settle_ms > 0:
                dur = settle_ms / 1000.0
                events.append(TimelineEvent(
                    t0=t, t1=t + dur, kind="dwell", pos0=seg.end, pos1=seg.end,
                    a0=cur_a, a1=cur_a, e0=e, e1=e, rpm=cur_rpm, v_mm_min=0.0,
                    layer_index=seg.layer_index, pass_index=seg.pass_index))
                t += dur

    return events


def total_duration_s(events: List[TimelineEvent]) -> float:
    return events[-1].t1 if events else 0.0


def state_at(events: List[TimelineEvent], t: float, c_axis: CAxisCfg) -> Optional[SimState]:
    """Interpolated machine state at time ``t`` (clamped to the timeline)."""
    if not events:
        return None
    t = min(max(t, events[0].t0), events[-1].t1)
    starts = [ev.t0 for ev in events]
    ev = events[min(bisect_right(starts, t), len(events)) - 1]
    u = (t - ev.t0) / ev.duration if ev.duration > 0 else 1.0
    x = ev.pos0[0] + u * (ev.pos1[0] - ev.pos0[0])
    y = ev.pos0[1] + u * (ev.pos1[1] - ev.pos0[1])
    z = ev.pos0[2] + u * (ev.pos1[2] - ev.pos0[2])
    a = ev.a0 + u * (ev.a1 - ev.a0)
    contact = ev.in_contact
    return SimState(
        t=t, x=x, y=y, z=z, a_deg=a,
        wheel_heading_deg=wheel_heading_deg(a, c_axis),
        kind=ev.kind, in_contact=contact, rpm=ev.rpm, v_mm_min=ev.v_mm_min,
        revs_per_mm=(ev.rpm / ev.v_mm_min if contact and ev.v_mm_min > 0 else 0.0),
        e_mm=ev.e0 + u * (ev.e1 - ev.e0),
        layer_index=ev.layer_index, pass_index=ev.pass_index)
