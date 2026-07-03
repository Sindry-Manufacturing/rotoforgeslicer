"""Kinematic simulation timeline (studio Preview): timing, dwells, E, readouts.

Pure — no Qt / pyvista. The timeline must mirror the emitter's feed choices and
dwell placement, and the readouts must reproduce the process invariants (constant
revs/mm in contact, monotonic E, airborne dwells).
"""
import pytest

from rotoforge_slicer.config import CAxisCfg, Config
from rotoforge_slicer.studio.simulate import (
    build_timeline, state_at, total_duration_s, wheel_heading_deg,
)
from rotoforge_slicer.toolpath.passplan import LayerPlan, Pass, ToolpathPlan
from rotoforge_slicer.toolpath.segments import SegmentKind, build_segments


def _pass(x, rpm=5000, v=120.0):
    return Pass(start=(x, 100.0), end=(x, 130.0), z=0.06, a_deg=0.0, rpm=rpm,
                traverse_mm_min=v, e_per_path_mm=1.0)


def _plan(passes_by_layer, rpm=5000, v=120.0):
    layers = [LayerPlan(i, 0.06 + 0.12 * i, ps) for i, ps in enumerate(passes_by_layer)]
    return ToolpathPlan(layers=layers, rpm=rpm, traverse_mm_min=v,
                        v_grind_floor_mm_min=v)


def _timeline(plan, cfg=None):
    cfg = cfg or Config()
    return build_timeline(build_segments(plan, cfg), plan, cfg), cfg


def test_timeline_contiguous_and_monotonic():
    tl, _ = _timeline(_plan([[_pass(190), _pass(191)]]))
    assert tl and tl[0].t0 == 0.0
    for a, b in zip(tl, tl[1:]):
        assert b.t0 == pytest.approx(a.t1)                  # no gaps, no overlap
    assert total_duration_s(tl) == pytest.approx(tl[-1].t1)


def test_durations_match_emitted_feeds():
    cfg = Config()
    tl, _ = _timeline(_plan([[_pass(190)]]), cfg)
    by = {}
    for ev in tl:
        by.setdefault(ev.kind, []).append(ev)
    # deposition: XY length at the traverse (30 mm total incl. the 2 mm plunge at 120 mm/min)
    dep_t = sum(e.duration for e in by["deposition"])
    plunge = cfg.process.lead_in_len_mm
    assert dep_t == pytest.approx((30.0 - plunge) / (120.0 / 60.0))
    assert by["lead_in"][0].duration == pytest.approx(plunge / (120.0 / 60.0))
    # travel: XY hop at the travel feed
    for ev in by["travel"]:
        v = cfg.emit.feed_travel_mm_min
        assert ev.v_mm_min == v
    # lifts/resets at the Z feed
    for ev in by["liftoff"] + by["reset"]:
        assert ev.v_mm_min == cfg.emit.feed_z_mm_min


def test_startup_settle_dwell_once_for_constant_rpm():
    cfg = Config()
    tl, _ = _timeline(_plan([[_pass(190), _pass(191)]]), cfg)   # same RPM both passes
    dwells = [e for e in tl if e.kind == "dwell"]
    assert len(dwells) == 1                                  # first M3 only
    assert dwells[0].duration == pytest.approx(cfg.process.startup_settle_ms / 1000.0)
    assert dwells[0].pos0 == dwells[0].pos1                  # zero motion
    assert dwells[0].e0 == dwells[0].e1                      # no feeding while dwelling
    assert dwells[0].pos0[2] > 0.06                          # airborne (invariant 2)


def test_rpm_hop_adds_short_stabilization_dwell():
    # two passes on the same revs/mm ray at different RPM (like test_emit_m3)
    a = Pass(start=(190, 100), end=(190, 120), z=0.06, a_deg=0.0, rpm=10000,
             traverse_mm_min=100.0, e_per_path_mm=1.0)
    b = Pass(start=(191, 100), end=(191, 120), z=0.06, a_deg=0.0, rpm=12000,
             traverse_mm_min=120.0, e_per_path_mm=1.0)
    plan = ToolpathPlan([LayerPlan(0, 0.06, [a, b])], 10000, 100.0, 100.0)
    cfg = Config()
    tl = build_timeline(build_segments(plan, cfg), plan, cfg)
    dwells = [e for e in tl if e.kind == "dwell"]
    assert [d.duration for d in dwells] == [
        pytest.approx(cfg.process.startup_settle_ms / 1000.0),
        pytest.approx(cfg.process.spindle_dwell_ms / 1000.0)]
    assert dwells[1].rpm == 12000                            # dwell runs at the new RPM


def test_e_accumulates_monotonically_to_pass_totals():
    plan = _plan([[_pass(190), _pass(191)]])
    tl, _ = _timeline(plan)
    es = [e for ev in tl for e in (ev.e0, ev.e1)]
    assert all(b >= a - 1e-9 for a, b in zip(es, es[1:]))    # invariant 4
    expected = sum(p.e_total_mm for ly in plan.layers for p in ly.passes)
    assert tl[-1].e1 == pytest.approx(expected)


def test_state_during_deposition_holds_the_revs_per_mm_ray():
    plan = _plan([[_pass(190)]])
    tl, cfg = _timeline(plan)
    dep = next(e for e in tl if e.kind == "deposition")
    s = state_at(tl, (dep.t0 + dep.t1) / 2, cfg.c_axis)
    assert s.in_contact and s.kind == "deposition"
    assert s.z == pytest.approx(0.06)                        # on the layer
    assert s.rpm == 5000 and s.v_mm_min == pytest.approx(120.0)
    assert s.revs_per_mm == pytest.approx(plan.revs_per_mm)  # invariant 5 readout
    assert 100.0 < s.y < 130.0                               # midway along the line


def test_state_interpolates_position_and_clamps():
    plan = _plan([[_pass(190)]])
    tl, cfg = _timeline(plan)
    s0 = state_at(tl, -10.0, cfg.c_axis)
    assert (s0.x, s0.y, s0.z) == (0.0, 0.0, 0.0)             # clamped to the home start
    assert s0.rpm == 0                                       # spindle off before first M3
    send = state_at(tl, 1e9, cfg.c_axis)
    assert send.z == pytest.approx(0.06 + Config().process.inter_pass_lift_mm)  # parked
    assert (send.x, send.y, send.z) == pytest.approx(tl[-1].pos1)  # exactly timeline end
    assert send.t == pytest.approx(total_duration_s(tl))


def test_wheel_heading_roundtrip_with_sign_and_offset():
    from rotoforge_slicer.fill.heading import heading_to_a_deg

    for c in (CAxisCfg(), CAxisCfg(invert_sign=-1, home_offset_deg=30.0),
              CAxisCfg(home_heading_deg=60.0)):
        for theta in (0.0, 45.0, 90.0, 170.0, -120.0):
            a = heading_to_a_deg(theta, c)
            assert wheel_heading_deg(a, c) == pytest.approx(theta)


def test_curved_pass_heading_tracks_the_tangent():
    # D13 in the sim: during a curved deposition the recovered wheel heading follows
    # the per-segment travel heading (drift = 0 at segment resolution).
    cfg = Config()
    cfg.c_axis.max_speed_deg_s = 360.0
    pts = [(190, 100), (190, 106), (191.5, 112), (194, 117)]
    p = Pass.curved(pts, z=0.06, rpm=5000, traverse_mm_min=120.0,
                    e_per_path_mm=1.0, c_axis=cfg.c_axis)
    plan = ToolpathPlan([LayerPlan(0, 0.06, [p])], 5000, 120.0, 120.0)
    tl = build_timeline(build_segments(plan, cfg), plan, cfg)
    deps = [e for e in tl if e.kind == "deposition"]
    assert len(deps) >= 2
    a_ends = [e.a1 for e in deps]
    assert len(set(round(a, 3) for a in a_ends)) > 1         # A really sweeps the curve
    for e in deps:
        s = state_at(tl, e.t1 - 1e-6, cfg.c_axis)
        assert s.a_deg == pytest.approx(e.a1, abs=1e-3)


def test_curved_pass_e_uses_the_plunge_arc_not_the_chord():
    # review fix: the emitter feeds E over the lead-in ARC length along the polyline
    # (emit/rrf.py step 4); a curved pass whose lead-in bends must still reach the
    # exact emitted total e_per_path * arc_length — a chord-based sim under-counts.
    cfg = Config()
    cfg.c_axis.max_speed_deg_s = 360.0
    pts = [(190, 100), (190.4, 101), (191.5, 102), (193, 102.7), (195, 103)]  # bends fast
    p = Pass.curved(pts, z=0.06, rpm=5000, traverse_mm_min=120.0,
                    e_per_path_mm=1.0, c_axis=cfg.c_axis)
    plan = ToolpathPlan([LayerPlan(0, 0.06, [p])], 5000, 120.0, 120.0)
    tl = build_timeline(build_segments(plan, cfg), plan, cfg)
    lead_in = next(e for e in tl if e.kind == "lead_in")
    # the emitter's plunge_split is the single source of truth for the plunge arc
    # (it may snap a multi-segment plunge to an original vertex)
    from rotoforge_slicer.toolpath.segments import plunge_split

    _, _, _, plunge_arc = plunge_split(
        [tuple(q) for q in p.points], min(cfg.process.lead_in_len_mm, 0.5 * p.length_mm))
    assert lead_in.e1 - lead_in.e0 == pytest.approx(plunge_arc * 1.0)   # arc, not chord
    assert tl[-1].e1 == pytest.approx(p.e_total_mm)                     # exact total


def test_travel_duration_floors_at_the_a_slew_rate():
    # review fix: a bidirectional-raster transition is a tiny XY hop carrying a ~180°
    # A flip; the C axis cannot beat ΔA/ω_C, so the travel cannot take ~20 ms.
    cfg = Config()
    cfg.c_axis.max_speed_deg_s = 360.0                    # calibrated slew rate
    fwd = Pass(start=(190, 100), end=(190, 130), z=0.06, a_deg=0.0, rpm=5000,
               traverse_mm_min=120.0, e_per_path_mm=1.0)  # +Y -> A = 0
    back = Pass(start=(191, 130), end=(191, 100), z=0.06, a_deg=180.0, rpm=5000,
                traverse_mm_min=120.0, e_per_path_mm=1.0)  # -Y -> A = ±180
    plan = ToolpathPlan([LayerPlan(0, 0.06, [fwd, back])], 5000, 120.0, 120.0)
    tl = build_timeline(build_segments(plan, cfg), plan, cfg)
    travels = [e for e in tl if e.kind == "travel"]
    flip = travels[1]                                     # the reorienting hop
    assert abs(flip.a1 - flip.a0) == pytest.approx(180.0)
    # the ~4 mm hop takes ~0.1 s at F2500; the 180° flip floors it at ΔA/ω_C = 0.5 s
    assert flip.duration == pytest.approx(180.0 / 360.0)


def test_segment_kinds_all_reach_the_timeline():
    tl, _ = _timeline(_plan([[_pass(190), _pass(191)]]))
    kinds = {e.kind for e in tl}
    assert {k.value for k in SegmentKind} | {"dwell"} == kinds
