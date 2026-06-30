"""RRF G-code emitter + hard validators. SPEC §6.

The emitter turns a planned toolpath into RepRapFirmware G-code following the §6.1
state-machine sequence (airborne reposition -> moving plunge -> deposit -> lead-out
+ lift), and PROVES the §6.3 invariants on every move it emits (wedge, contact /
no-grinding, monotonic E, no dwell in contact, single RPM+traverse per pass,
in-build-volume). M2 emits straight +Y passes; because A is constant during a
straight pass (ΔA=0), F sets the XY speed directly and the §6.2 combined-move
feedrate compensation is not needed until curved fill (M5).

The validator functions below are reused by the emitter and are independently
tested.
"""
from __future__ import annotations

import math
from typing import Iterable, List

from ..config import CAxisCfg, Config
from ..fill.wedge import in_wedge
from ..toolpath.statemachine import assert_contact_invariant  # noqa: F401 (re-exported)
from . import templates


def validate_heading(a_deg: float, c_axis: CAxisCfg) -> None:
    """SPEC §6.3: every deposition heading within the +/- wedge."""
    if not in_wedge(a_deg, c_axis):
        raise ValueError(
            f"heading A={a_deg:.2f} deg outside +/-{c_axis.wedge_half_angle_deg} deg wedge")


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
        dwell_at_dep_z = []                # z of any G4 (must be airborne)
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
            if e is not None:
                parts.append(f"E{e:.5f}")
            if f is not None:
                parts.append(f"F{int(round(f))}")
            lines.append(" ".join(parts))
            zz = z if z is not None else cur_z
            coords.append((x, y, zz))

        # ---- header + preamble ----
        nlay = len(plan.nonempty_layers)
        lines.append("; Rotoforge Slicer — M2 straight-fill (SPEC §6); afrb_yline parity deferred")
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
        # get to clearance, spin up + settle (airborne) before any contact
        move("G0", z=first_lift_z, f=f_z, cur_z=first_lift_z)
        cur_z = first_lift_z
        if not dry:
            lines.append(f"M3 S{plan.rpm}        ; spindle to target (airborne)")
            if cfg.process.startup_settle_ms > 0:
                lines.append(f"G4 P{cfg.process.startup_settle_ms}   ; airborne startup settle")
                dwell_at_dep_z.append(cur_z)

        # ---- body: per layer, per pass ----
        for ly in body:
            lift_z = ly.z + lift
            approach_z = ly.z + approach
            for p in ly.passes:
                x, y0 = p.start
                _, y1 = p.end
                a = p.a_deg
                v = p.traverse_mm_min          # this pass's traverse (operating point)
                v_s = v / 60.0                 # mm/s, for the contact check
                f_dep = int(round(v))
                length = p.length_mm
                # plunge consumes lead_in of the deposit length (keep a steady seg)
                plunge = min(lead_in, 0.5 * length)
                y_pl = y0 + plunge
                de_plunge = p.e_per_path_mm * plunge
                de_steady = p.e_per_path_mm * (y1 - y_pl)
                # F that keeps the plunge's XY surface speed at the traverse despite
                # its small Z descent (SPEC §6.2) — otherwise it would grind (§4.4).
                f_plunge = _compensated_feed(v, plunge, math.hypot(plunge, approach))

                # every deposition heading must be in the wedge (SPEC §6.3); also hold
                # the hard mechanical +/-wedge bound on the absolute A value.
                validate_heading(a, cfg.c_axis)
                if abs(a) > cfg.c_axis.wedge_half_angle_deg + 1e-9:
                    raise ValueError(
                        f"A={a:.2f} exceeds the mechanical +/-{cfg.c_axis.wedge_half_angle_deg} deg range")

                # 1. airborne reposition (wheel up) to the plunge start, heading set
                if cur_z != lift_z:
                    move("G0", z=lift_z, f=f_z, cur_z=lift_z)
                    cur_z = lift_z
                move("G0", x=x, y=y0, a=a, f=f_travel, cur_z=cur_z)
                # 2. airborne rapid descent to a small clearance above the surface
                move("G0", z=approach_z, f=f_z, cur_z=approach_z)
                cur_z = approach_z

                # 3. TRANSITION_IN: moving plunge — descend the last `approach` mm while
                #    moving forward `plunge` mm, E feeding. F-compensated so the realized
                #    XY speed == traverse (>= grind floor).
                move("G1", x=x, y=y_pl, z=ly.z, a=a,
                     e=None if dry else de_plunge, f=f_plunge, cur_z=ly.z)
                cur_z = ly.z
                if not dry:
                    e_cum += de_plunge
                    e_seq.append(e_cum)
                    # the plunge is an in-contact, wire-feeding move — prove it too
                    assert_contact_invariant(
                        in_contact=True, xy_speed_mm_s=v_s,
                        v_grind_floor_mm_s=floor_s, e_feeding=de_plunge > 0)

                # 4. DEPOSITING: steady forward move (pure +Y, ΔZ=ΔA=0 -> F == XY speed)
                move("G1", x=x, y=y1, a=a,
                     e=None if dry else de_steady, f=f_dep, cur_z=cur_z)
                if not dry:
                    e_cum += de_steady
                    e_seq.append(e_cum)
                    assert_contact_invariant(
                        in_contact=True, xy_speed_mm_s=v_s,
                        v_grind_floor_mm_s=floor_s, e_feeding=de_steady > 0)

                # 5. TRANSITION_OUT: continue fwd through lead-out while lifting; E stops
                move("G1", x=x, y=y1 + lead_out, z=lift_z, f=f_z, cur_z=lift_z)
                cur_z = lift_z  # wire cut at the lead-out (mechanical runout)

        # park + postamble
        park_z = max(l.z for l in body) + lift
        move("G0", z=park_z, f=f_z, cur_z=park_z)
        lines += templates.postamble(cfg) if not dry else self._dry_postamble()

        # ---- §6.3 whole-file validations ----
        validate_monotonic_e(e_seq)
        self._validate_pass_uniformity(plan)
        self._validate_no_dwell_in_contact(dwell_at_dep_z, plan)
        self._validate_in_build_volume(coords, bx, by, bz)

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
    def _validate_pass_uniformity(plan) -> None:
        """SPEC §6.3: a single RPM and a single traverse per pass (constant revs/mm)."""
        for ly in plan.layers:
            for p in ly.passes:
                if p.rpm != plan.rpm or p.traverse_mm_min != plan.traverse_mm_min:
                    raise ValueError(
                        "pass RPM/traverse differs from the plan's single operating "
                        f"point (rpm {p.rpm} vs {plan.rpm}, v {p.traverse_mm_min} vs "
                        f"{plan.traverse_mm_min}) — revs/mm would not be constant")

    def _validate_no_dwell_in_contact(self, dwell_zs, plan) -> None:
        """SPEC §6.3: no G4 dwell at a deposition Z (all dwells airborne)."""
        dep_zs = {round(ly.z, 6) for ly in plan.nonempty_layers}
        for z in dwell_zs:
            if round(z, 6) in dep_zs:
                raise ValueError(f"G4 dwell at deposition Z={z} (must be airborne)")

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
