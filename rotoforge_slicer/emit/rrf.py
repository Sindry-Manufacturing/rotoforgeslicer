"""RRF G-code emitter + hard validators. SPEC §6.

The emitter turns a planned toolpath into RepRapFirmware G-code following the §6.1
state-machine sequence (airborne reposition -> moving plunge -> deposit -> lead-out
+ lift), and PROVES the §6.3 invariants on every move it emits (axis range, contact /
no-grinding, monotonic E, no dwell in contact, single RPM+traverse per pass,
in-build-volume). The rotary A is commanded equal to the travel heading at every
segment (tangential tool, drift ≈ 0; D13), winding-resolved so it stays continuous and
inside the usable axis range. For a straight pass A is constant (ΔA=0) so F sets the XY
speed directly; the §6.2 combined-move feedrate compensation matters for curved fill.

The validator functions below are reused by the emitter and are independently
tested.
"""
from __future__ import annotations

import math
from typing import Iterable, List

from ..config import CAxisCfg, Config
from ..fill.curvature import r_min
from ..fill.wedge import heading_deg_from_vector, heading_to_a_deg, within_axis_range
from ..toolpath.statemachine import assert_contact_invariant  # noqa: F401 (re-exported)
from . import templates


def validate_axis_angle(a_deg: float, c_axis: CAxisCfg) -> None:
    """SPEC §6.3 (D13): every commanded A within the usable continuous axis range.

    There is no deposition wedge — the head rotates as a unit, so any heading deposits.
    The only hard heading limit is the axis travel range ``[a_min_deg, a_max_deg]``.
    """
    if not within_axis_range(a_deg, c_axis):
        raise ValueError(
            f"A={a_deg:.2f} deg outside usable axis range "
            f"[{c_axis.a_min_deg}, {c_axis.a_max_deg}] deg")


def validate_monotonic_e(e_values: Iterable[float], tol: float = 1e-9) -> None:
    """SPEC §6.3: E never decreases across the file."""
    prev = None
    for e in e_values:
        if prev is not None and e < prev - tol:
            raise ValueError(f"E not monotonic: {e} < {prev}")
        prev = e


def _fmt(v: float) -> str:
    """Compact coordinate: trim trailing zeros, avoid '-0'."""
    s = f"{v:.3f}".rstrip("0").rstrip(".")
    if s in ("", "-0"):
        return "0"
    return s


def _split_polyline(pts, dist):
    """(point at ``dist`` along the polyline, [that point, ...rest], index of the
    original segment the split point lands on). The returned index lets the emitter map
    each deposit sub-segment back to its original segment's validated axis angle."""
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


def _unit(p0, p1):
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    n = math.hypot(dx, dy)
    return (dx / n, dy / n) if n else (0.0, 1.0)


def _seg_a(p0, p1, c_axis) -> float:
    """Raw rotary A for a segment's heading (no winding; may wrap near ±180)."""
    return heading_to_a_deg(heading_deg_from_vector(p1[0] - p0[0], p1[1] - p0[1]), c_axis)


def _continuity_adjust(a_raw: float, a_prev: float) -> float:
    """Shift ``a_raw`` by whole turns to the value nearest ``a_prev`` — keeps commanded A
    continuous across a pass (no ±360° jump at the atan2 branch cut; D13)."""
    return a_raw + 360.0 * round((a_prev - a_raw) / 360.0)


def _compensated_feed(target_xy_mm_min: float, l_xy: float, l_3d: float) -> float:
    """F that realizes ``target_xy_mm_min`` of XY surface speed on a combined move.

    RRF applies F to the full 3D move length, so for a move with a Z (or, later, A)
    component F must be scaled by L_3d/L_xy to keep the XY speed on target
    (SPEC §6.2). For a pure-XY move L_3d==L_xy and F == target.
    """
    if l_xy <= 0:
        return target_xy_mm_min
    return target_xy_mm_min * (l_3d / l_xy)


class GCodeEmitter:
    """Emit RRF G-code from a planned toolpath and validate it. SPEC §6."""

    def __init__(self, cfg: Config):
        self.cfg = cfg

    # ---- public -------------------------------------------------------------

    def emit(self, plan) -> str:
        cfg = self.cfg
        A = cfg.machine.rotary_axis_letter
        dry = cfg.emit.dry_run
        f_travel = int(round(cfg.emit.feed_travel_mm_min))
        f_z = int(round(cfg.emit.feed_z_mm_min))
        lift = cfg.process.inter_pass_lift_mm
        lead_in = cfg.process.lead_in_len_mm
        approach = cfg.process.approach_clearance_mm
        lead_out = cfg.process.lead_out_len_mm
        bx, by, bz = cfg.machine.build_volume_mm
        # grind floor: compare unrounded mm/s on both sides (SPEC §6.3)
        floor_s = plan.v_grind_floor_mm_min / 60.0

        lines: List[str] = []
        coords: List[tuple] = []           # (x, y, z) of every move endpoint
        a_seq: List[float] = []            # every commanded A target (deposition + airborne)
        dwell_records = []                 # (dwell_z, layer_z) for every G4 (must be airborne)
        e_seq: List[float] = [0.0]         # cumulative E across the file
        e_cum = 0.0

        def move(code, x=None, y=None, z=None, a=None, e=None, f=None, cur_z=None):
            parts = [code]
            if x is not None:
                parts.append(f"X{_fmt(x)}")
            if y is not None:
                parts.append(f"Y{_fmt(y)}")
            if z is not None:
                parts.append(f"Z{_fmt(z)}")
            if a is not None:
                parts.append(f"{A}{_fmt(a)}")
                a_seq.append(a)
            if e is not None:
                parts.append(f"E{e:.5f}")
            if f is not None:
                parts.append(f"F{int(round(f))}")
            lines.append(" ".join(parts))
            zz = z if z is not None else cur_z
            coords.append((x, y, zz))

        # ---- header + preamble ----
        nlay = len(plan.nonempty_layers)
        lines.append("; Rotoforge Slicer — straight-fill + process window (SPEC §6); "
                     "afrb_yline parity deferred")
        lines.append(
            f"; passes={plan.npasses} layers={nlay} rpm={plan.rpm} "
            f"traverse={plan.traverse_mm_min:g}mm/min revs_per_mm={plan.revs_per_mm:.4f}")
        if dry:
            lines.append("; DRY RUN: spindle / heaters / fan / wire-E disabled")
        lines += templates.preamble(cfg) if not dry else self._dry_preamble()
        lines.append("G28               ; home")
        lines.append("G92 E0")

        body = plan.nonempty_layers
        if not body:
            lines += templates.postamble(cfg) if not dry else self._dry_postamble()
            return "\n".join(lines) + "\n"

        first_lift_z = body[0].z + lift
        # Rise to clearance before any contact. The spindle is brought to each pass's
        # RPM airborne inside the per-pass loop (SPEC §4.5: RPM changes only between
        # passes, airborne — the SuperPID can't be chased mid-move).
        move("G0", z=first_lift_z, f=f_z, cur_z=first_lift_z)
        cur_z = first_lift_z
        cur_rpm = None

        # ---- body: per layer, per pass ----
        for ly in body:
            lift_z = ly.z + lift
            approach_z = ly.z + approach
            for p in ly.passes:
                pts = p.points                 # polyline (straight = 2 points)
                v = p.traverse_mm_min          # this pass's traverse (operating point)
                v_s = v / 60.0                 # mm/s, for the contact check
                f_dep = int(round(v))

                # §6.3 (D13): every winding-resolved A within the usable axis range, A
                # continuous within the pass, and the whole pass within the curvature
                # limit at its single speed (§4.3).
                self._validate_pass_geometry(p, v, cfg)

                start = pts[0]
                # The winding-resolved, in-range, continuous A per ORIGINAL segment — the
                # exact sequence _validate_pass_geometry just proved legal. The emitter
                # commands these directly (no independent re-derivation), so the emitted A
                # is provably identical to the validated A.
                a_segs = p.axis_angles(cfg.c_axis)
                a_start = a_segs[0]

                # 1. airborne reposition (wheel up) to the pass start, first heading set
                if cur_z != lift_z:
                    move("G0", z=lift_z, f=f_z, cur_z=lift_z)
                    cur_z = lift_z
                move("G0", x=start[0], y=start[1], a=a_start, f=f_travel, cur_z=cur_z)

                # 2. RPM placement (SPEC §4.5/§6.1): set the spindle airborne ONLY when
                #    it changes between passes — never chase RPM mid-move. Long settle on
                #    the first spin-up; short stabilization dwell on later hops.
                if not dry and p.rpm != cur_rpm:
                    first = cur_rpm is None
                    lines.append(f"M3 S{p.rpm}        ; spindle to target (airborne)")
                    settle = cfg.process.startup_settle_ms if first else cfg.process.spindle_dwell_ms
                    if settle > 0:
                        tag = "startup settle" if first else "spindle stabilize"
                        lines.append(f"G4 P{settle}   ; airborne {tag}")
                        dwell_records.append((cur_z, ly.z))  # cur_z == lift_z -> airborne
                    cur_rpm = p.rpm

                # 3. airborne rapid descent to a small clearance above the surface
                move("G0", z=approach_z, f=f_z, cur_z=approach_z)
                cur_z = approach_z

                # 4. TRANSITION_IN: moving plunge over the first `plunge` mm of the path,
                #    descending to layer Z. F-compensated for the Z descent (§6.2/§4.4).
                plunge = min(lead_in, 0.5 * p.length_mm)
                pp, deposit_pts, seg0 = _split_polyline(pts, plunge)
                # The plunge is one short transition move (start -> pp), possibly spanning
                # several original segments; its A is the chord heading kept continuous
                # from a_start (in range — it lies within the pass's A band).
                a_pl = (_continuity_adjust(_seg_a(start, pp, cfg.c_axis), a_start)
                        if plunge > 1e-9 else a_start)
                de_plunge = p.e_per_path_mm * plunge
                f_plunge = _compensated_feed(v, max(plunge, 1e-9),
                                             math.hypot(max(plunge, 1e-9), approach))
                move("G1", x=pp[0], y=pp[1], z=ly.z, a=a_pl,
                     e=None if dry else de_plunge, f=f_plunge, cur_z=ly.z)
                cur_z = ly.z
                if not dry:
                    e_cum += de_plunge
                    e_seq.append(e_cum)
                    assert_contact_invariant(
                        in_contact=True, xy_speed_mm_s=v_s,
                        v_grind_floor_mm_s=floor_s, e_feeding=de_plunge > 0)

                # 5. DEPOSITING: one G1 per polyline segment. A is the travel heading
                #    (tangential tool, D13). Deposit sub-segment ``m`` maps to original
                #    segment ``seg0 + m``, so its A is taken straight from ``a_segs`` — the
                #    exact winding-resolved, continuous sequence _validate_pass_geometry
                #    proved legal (emitted A == validated A, no independent re-derivation).
                #    Each segment is planar (ΔZ=0) so F == the XY traverse (the rotary is
                #    slaved; the §6.2 combined-move feedrate is a calibration item, §13).
                prev = deposit_pts[0]
                for m, nxt in enumerate(deposit_pts[1:]):
                    seg_len = math.hypot(nxt[0] - prev[0], nxt[1] - prev[1])
                    if seg_len > 1e-9:
                        de = p.e_per_path_mm * seg_len
                        move("G1", x=nxt[0], y=nxt[1], a=a_segs[seg0 + m],
                             e=None if dry else de, f=f_dep, cur_z=cur_z)
                        if not dry:
                            e_cum += de
                            e_seq.append(e_cum)
                            assert_contact_invariant(
                                in_contact=True, xy_speed_mm_s=v_s,
                                v_grind_floor_mm_s=floor_s, e_feeding=de > 0)
                    prev = nxt

                # 6. TRANSITION_OUT: continue past the last point along the last heading
                #    through the lead-out while lifting; E stops; wire cut at the runout.
                lx, ly_u = _unit(deposit_pts[-2], deposit_pts[-1])
                end = deposit_pts[-1]
                move("G1", x=end[0] + lead_out * lx, y=end[1] + lead_out * ly_u,
                     z=lift_z, f=f_z, cur_z=lift_z)
                cur_z = lift_z  # wire cut at the lead-out (mechanical runout)

        # park + postamble
        park_z = max(l.z for l in body) + lift
        move("G0", z=park_z, f=f_z, cur_z=park_z)
        lines += templates.postamble(cfg) if not dry else self._dry_postamble()

        # ---- §6.3 whole-file validations ----
        validate_monotonic_e(e_seq)
        self._validate_constant_revs_per_mm(plan)
        self._validate_spindle_in_range(plan, cfg.spindle)
        self._validate_no_dwell_airborne(dwell_records, plan)
        self._validate_in_build_volume(coords, bx, by, bz)
        self._validate_a_in_axis_range(a_seq, cfg.c_axis)

        return "\n".join(lines) + "\n"

    # ---- preamble/postamble for dry-run -------------------------------------

    def _dry_preamble(self) -> List[str]:
        cfg = self.cfg
        out = ["; --- preamble (dry run) ---", "G21", "G90"]
        if cfg.gcode.use_relative_e:
            out.append("M83")
        out += ["M220 S100", "M221 S100", "M5  ; spindle off", "M106 P0 S0",
                "; process commands skipped: heaters / M3 / fan / E feed",
                "; --- end preamble ---"]
        return out

    def _dry_postamble(self) -> List[str]:
        return ["; --- postamble (dry run) ---", "M400", "M5  ; spindle off",
                "M106 P0 S0", "M84"]

    # ---- validators ---------------------------------------------------------

    @staticmethod
    def _validate_pass_geometry(p, v_mm_min: float, cfg: Config) -> None:
        """SPEC §6.3 (D13): the pass's winding-resolved A stays inside the usable axis
        range ``[a_min_deg, a_max_deg]`` and evolves **continuously** (no ±360° jump —
        the planner splits over-winding paths with ``split_on_winding``), and the whole
        pass holds ``R >= R_min(v)`` (§4.3). There is no wedge — every heading is
        depositable; the only heading limit is the axis travel range."""
        a_segs = p.axis_angles(cfg.c_axis)
        for a in a_segs:
            validate_axis_angle(a, cfg.c_axis)
        for a0, a1 in zip(a_segs, a_segs[1:]):
            # unwrap keeps consecutive A within (-180, 180]; a step that REACHES ±180 is a
            # hairpin cusp — a ≥180° in-contact swing that is never a valid single G1 and
            # must have been split. (Strict `> 180+eps` would miss the exact-180° reversal.)
            if abs(a1 - a0) > 180.0 - 1e-6:
                raise ValueError(
                    f"A axis swings {a1 - a0:.1f} deg between deposition segments — a ≥180°"
                    " in-contact reversal (cusp) is never a valid single move; split it "
                    "(SPEC §4.3 / D13)")
        r_floor = r_min(v_mm_min / 60.0, cfg.c_axis.max_speed_deg_s)
        if r_floor < math.inf and p.min_radius_mm < r_floor - 1e-9:
            raise ValueError(
                f"pass turn radius {p.min_radius_mm:.3f} mm < R_min {r_floor:.3f} mm at "
                f"v={v_mm_min:g} mm/min (SPEC §4.3/§6.3) — split tighter on curvature")

    @staticmethod
    def _validate_constant_revs_per_mm(plan, rel_tol: float = 1e-3) -> None:
        """SPEC §6.3 / acceptance 2: revs/mm is constant within every pass and equals
        the selected ray. A ``Pass`` holds scalar rpm+traverse (single-valued within a
        pass by construction); RPM may differ *between* passes, but every pass must sit
        on the plan's revs/mm ray."""
        ray = plan.revs_per_mm
        for ly in plan.layers:
            for p in ly.passes:
                if p.traverse_mm_min <= 0:
                    raise ValueError(f"pass has non-positive traverse {p.traverse_mm_min}")
                rpm_per_v = p.rpm / p.traverse_mm_min
                if ray > 0 and abs(rpm_per_v - ray) > rel_tol * ray:
                    raise ValueError(
                        f"pass revs/mm {rpm_per_v:.4f} != selected ray {ray:.4f} — RPM may "
                        "change between passes but must hold the constant revs/mm ray")

    @staticmethod
    def _validate_no_dwell_airborne(dwell_records, plan, clearance: float = 1e-6) -> None:
        """SPEC §6.3 / §2.2: every G4 dwell is airborne — strictly above the layer it is
        lifting from. Checking ``dwell_z > layer_z`` (rather than ``!= any deposition Z``)
        is robust to a lift height that happens to coincide with another layer's Z."""
        for dwell_z, layer_z in dwell_records:
            if dwell_z <= layer_z + clearance:
                raise ValueError(
                    f"G4 dwell at Z={dwell_z} is not airborne above layer Z={layer_z} "
                    "(all dwells must happen with the wheel lifted)")

    @staticmethod
    def _validate_spindle_in_range(plan, spindle) -> None:
        """SPEC §1.3/§6.3: every commanded RPM is within the SuperPID window."""
        for ly in plan.layers:
            for p in ly.passes:
                if not (spindle.rpm_min <= p.rpm <= spindle.rpm_max):
                    raise ValueError(
                        f"M3 S{p.rpm} outside the SuperPID window "
                        f"[{spindle.rpm_min},{spindle.rpm_max}] (SPEC §1.3)")

    @staticmethod
    def _validate_a_in_axis_range(a_values, c_axis, tol: float = 1e-6) -> None:
        """SPEC §6.3 (D13): every commanded A target — deposition AND airborne
        reorientation — lies within the usable continuous axis range
        ``[a_min_deg, a_max_deg]``. The head rotates as a unit so any heading deposits;
        this travel range is the only hard heading limit (no wedge). A pass whose heading
        sweep would exceed it is split with an airborne unwind (``split_on_winding``)."""
        lo, hi = c_axis.a_min_deg, c_axis.a_max_deg
        for a in a_values:
            if not within_axis_range(a, c_axis, tol):
                raise ValueError(
                    f"A={a:.2f} deg outside usable axis range [{lo},{hi}] — the wheel "
                    "cannot wrap past its stops; needs an airborne unwind (D13)")

    @staticmethod
    def _validate_in_build_volume(coords, bx, by, bz, tol: float = 1e-6) -> None:
        """SPEC §6.3: every emitted coordinate (toolpath + lead-outs) is in volume."""
        for (x, y, z) in coords:
            if x is not None and not (-tol <= x <= bx + tol):
                raise ValueError(f"X={x:.2f} outside build volume [0,{bx}] — part too large in X")
            if y is not None and not (-tol <= y <= by + tol):
                raise ValueError(
                    f"Y={y:.2f} outside build volume [0,{by}] — part + lead-out too large in Y")
            if z is not None and not (-tol <= z <= bz + tol):
                raise ValueError(f"Z={z:.2f} outside build volume [0,{bz}] — part too tall")
