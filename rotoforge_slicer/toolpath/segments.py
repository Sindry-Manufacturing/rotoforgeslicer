"""Tagged 3D toolpath segments for visualization (SPEC §9; U2).

The emitter (``emit/rrf.py``) turns a ``ToolpathPlan`` into RRF G-code following the
§6.1 motion sequence (airborne reposition → moving plunge → deposit → lead-out + lift).
``build_segments`` walks that **same** sequence and yields one tagged, fully-3D
``ToolpathSegment`` per commanded move, so a viewer draws exactly what the machine will
move — every segment's coordinates match the emitted G-code move-for-move (guarded by
``tests/test_segments.py``). Pure geometry: no E / F / spindle / validation — those live
in the emitter. Planning-layer clean (no mesh / Qt / matplotlib; SPEC §3.3).

Segment kinds, and the five viewer toggles they group into:

======================  =================================================  ============
kind                    move                                               toggle
======================  =================================================  ============
``DEPOSITION``          in-contact bead move (G1 at layer Z, E feeding)    deposition
``LEAD_IN``             moving plunge onto the surface (transition-in)     lead-in/out
``LEAD_OUT``            lead-out + moving lift, wire cut at the runout     lead-in/out
``LIFTOFF``             airborne Z **retract** (initial rise, lift, park)  liftoffs
``RESET``               airborne Z **approach** back down to the surface   resets
``TRAVEL``              airborne XY reposition to the next pass start      travels
======================  =================================================  ============

The LIFTOFF/RESET split is by Z direction — leaving the part vs returning to it; TRAVEL
is the in-plane airborne hop between them. Together the three airborne kinds tile the
journey from one pass's lead-out to the next pass's plunge.

The low-level move helpers (``split_polyline`` / ``unit_vec`` / ``seg_a`` /
``continuity_adjust``) intentionally mirror ``emit.rrf``'s private helpers so the two
producers agree; ``test_segments`` cross-checks the result against the real emitter, so
any future drift turns a test red rather than silently mis-drawing the path.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

from ..config import CAxisCfg, Config
from ..fill.heading import heading_deg_from_vector, heading_to_a_deg

Point3 = Tuple[float, float, float]


class SegmentKind(str, Enum):
    DEPOSITION = "deposition"
    LEAD_IN = "lead_in"
    LEAD_OUT = "lead_out"
    LIFTOFF = "liftoff"
    RESET = "reset"
    TRAVEL = "travel"


# Viewer toggle -> the segment kinds it shows. Five independent toggles (SPEC §9); every
# kind belongs to exactly one toggle (asserted in tests).
TOGGLE_KINDS = {
    "deposition": (SegmentKind.DEPOSITION,),
    "lead-in/out": (SegmentKind.LEAD_IN, SegmentKind.LEAD_OUT),
    "liftoffs": (SegmentKind.LIFTOFF,),
    "resets": (SegmentKind.RESET,),
    "travels": (SegmentKind.TRAVEL,),
}
TOGGLE_ORDER = ("deposition", "lead-in/out", "liftoffs", "resets", "travels")

# One distinct color per kind for the color-coded 3D viewer.
KIND_COLOR = {
    SegmentKind.DEPOSITION: "#1f5fd6",   # blue  — the bead
    SegmentKind.LEAD_IN: "#12a150",      # green — plunge on
    SegmentKind.LEAD_OUT: "#e0a000",     # amber — lead-out + wire cut
    SegmentKind.LIFTOFF: "#8e44ad",      # purple— airborne retract
    SegmentKind.RESET: "#16a2b8",        # teal  — airborne approach
    SegmentKind.TRAVEL: "#7f8c8d",       # gray  — airborne XY hop
}

_CONTACT_KINDS = frozenset(
    {SegmentKind.DEPOSITION, SegmentKind.LEAD_IN, SegmentKind.LEAD_OUT})


@dataclass(frozen=True)
class ToolpathSegment:
    """One tagged straight move in machine space (mm). ``a_deg`` is the commanded rotary
    A the emitter attaches to that move (None where the emitter commands no A — lifts,
    resets, lead-outs)."""

    kind: SegmentKind
    start: Point3
    end: Point3
    layer_index: Optional[int] = None
    pass_index: Optional[int] = None
    a_deg: Optional[float] = None

    @property
    def length_mm(self) -> float:
        return math.dist(self.start, self.end)

    @property
    def is_airborne(self) -> bool:
        """True for non-contact moves (travel / liftoff / reset)."""
        return self.kind not in _CONTACT_KINDS


# --- low-level move helpers (mirror emit.rrf; see module docstring) -------------------

def split_polyline(pts, dist: float):
    """(point at ``dist`` along the polyline, [that point, ...rest], index of the split
    segment). The index maps each deposit sub-segment back to its original segment's A."""
    if dist <= 1e-9:
        return pts[0], list(pts), 0
    acc = 0.0
    for i in range(len(pts) - 1):
        (ax, ay), (bx, by) = pts[i], pts[i + 1]
        seg = math.hypot(bx - ax, by - ay)
        if acc + seg >= dist:
            t = (dist - acc) / seg if seg > 0 else 0.0
            pp = (ax + t * (bx - ax), ay + t * (by - ay))
            return pp, [pp] + list(pts[i + 1:]), i
        acc += seg
    return pts[-1], [pts[-1]], len(pts) - 2


def plunge_split(pts, target_mm: float):
    """Split a pass polyline for the moving plunge: (pp, deposit_pts, seg0,
    plunge_len).

    A mid-segment split is only safe while the plunge stays WITHIN the first
    segment (the chord then equals the segment heading — no new heading vertex).
    When the target spans original vertices, snap to the interior vertex nearest
    the target (capped at half the path): the emitter-made junction then lands on
    validated geometry and the following in-contact segment keeps its full length
    — a mid-segment split there would leave an arbitrarily short remainder across
    which the firmware must interpolate the chord→segment A step (an unvalidated,
    possibly axis-infeasible in-contact turn).
    """
    if len(pts) >= 2:
        first_seg = math.hypot(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1])
        if target_mm <= first_seg + 1e-9 or len(pts) == 2:
            pp, deposit_pts, seg0 = split_polyline(pts, target_mm)
            return pp, deposit_pts, seg0, min(target_mm, first_seg)
    cum = [0.0]
    for a, b in zip(pts, pts[1:]):
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    half = 0.5 * cum[-1]
    best = None
    for i in range(1, len(pts) - 1):
        if cum[i] <= half + 1e-9 and (
                best is None or abs(cum[i] - target_mm) < abs(cum[best] - target_mm)):
            best = i
    if best is None:                              # no usable interior vertex
        pp, deposit_pts, seg0 = split_polyline(pts, target_mm)
        return pp, deposit_pts, seg0, target_mm
    return pts[best], list(pts[best:]), best, cum[best]


def unit_vec(p0, p1):
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    n = math.hypot(dx, dy)
    return (dx / n, dy / n) if n else (0.0, 1.0)


def seg_a(p0, p1, c_axis: CAxisCfg) -> float:
    """Raw rotary A for a segment's heading (no winding; may wrap near ±180)."""
    return heading_to_a_deg(heading_deg_from_vector(p1[0] - p0[0], p1[1] - p0[1]), c_axis)


def continuity_adjust(a_raw: float, a_prev: float) -> float:
    """Shift ``a_raw`` by whole turns to the value nearest ``a_prev`` (no ±360° jump)."""
    return a_raw + 360.0 * round((a_prev - a_raw) / 360.0)


# --- the builder ---------------------------------------------------------------------

def build_segments(plan, cfg: Config) -> List[ToolpathSegment]:
    """Tagged, fully-3D toolpath segments mirroring the emitter's §6.1 motion sequence.

    Coordinates match the emitted G-code move-for-move. Pure geometry — it never raises
    on an invariant violation (that is the emitter's job); a plan the emitter would
    reject still yields drawable segments for inspection.
    """
    proc = cfg.process
    c_axis = cfg.c_axis
    lift = proc.inter_pass_lift_mm
    lead_in = proc.lead_in_len_mm
    approach = proc.approach_clearance_mm
    lead_out = proc.lead_out_len_mm

    body = plan.nonempty_layers
    segs: List[ToolpathSegment] = []
    if not body:
        return segs

    x, y, z = 0.0, 0.0, 0.0        # position after G28 home

    def add(kind, nx, ny, nz, *, layer=None, pi=None, a=None):
        nonlocal x, y, z
        segs.append(ToolpathSegment(kind, (x, y, z), (nx, ny, nz),
                                    layer_index=layer, pass_index=pi, a_deg=a))
        x, y, z = nx, ny, nz

    # rise to the first layer's clearance before any contact (initial liftoff)
    first_lift_z = body[0].z + lift
    add(SegmentKind.LIFTOFF, x, y, first_lift_z, layer=body[0].index)
    cur_z = first_lift_z

    for ly in body:
        lift_z = ly.z + lift
        approach_z = ly.z + approach
        for pi, p in enumerate(ly.passes):
            pts = [tuple(q) for q in p.points]
            a_segs = p.axis_angles(c_axis)
            a_start = a_segs[0]
            start = pts[0]

            # 1. lift to this pass's safe Z if not already there (layer change / first pass)
            if cur_z != lift_z:
                add(SegmentKind.LIFTOFF, x, y, lift_z, layer=ly.index, pi=pi)
                cur_z = lift_z
            # 2. airborne XY reposition to the pass start (first heading set here)
            add(SegmentKind.TRAVEL, start[0], start[1], cur_z,
                layer=ly.index, pi=pi, a=a_start)
            # 3. airborne descent to a small clearance above the surface
            add(SegmentKind.RESET, x, y, approach_z, layer=ly.index, pi=pi)
            cur_z = approach_z
            # 4. moving plunge onto the surface (transition-in), descending to layer Z
            pp, deposit_pts, seg0, plunge = plunge_split(
                pts, min(lead_in, 0.5 * p.length_mm))
            a_pl = (continuity_adjust(seg_a(start, pp, c_axis), a_start)
                    if plunge > 1e-9 else a_start)
            add(SegmentKind.LEAD_IN, pp[0], pp[1], ly.z, layer=ly.index, pi=pi, a=a_pl)
            cur_z = ly.z
            # 5. depositing: one segment per polyline edge (skip zero-length, like the emitter)
            prev = deposit_pts[0]
            for m, nxt in enumerate(deposit_pts[1:]):
                if math.hypot(nxt[0] - prev[0], nxt[1] - prev[1]) > 1e-9:
                    add(SegmentKind.DEPOSITION, nxt[0], nxt[1], cur_z,
                        layer=ly.index, pi=pi, a=a_segs[seg0 + m])
                prev = nxt
            # 6. lead-out + moving lift; wire cut at the runout
            lx, lyu = unit_vec(deposit_pts[-2], deposit_pts[-1])
            end = deposit_pts[-1]
            add(SegmentKind.LEAD_OUT, end[0] + lead_out * lx, end[1] + lead_out * lyu,
                lift_z, layer=ly.index, pi=pi)
            cur_z = lift_z

    # park above the tallest deposited layer
    park_z = max(l.z for l in body) + lift
    add(SegmentKind.LIFTOFF, x, y, park_z, layer=body[-1].index)
    return segs
