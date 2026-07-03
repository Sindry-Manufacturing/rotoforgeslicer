"""U2 tagged toolpath segments: the tag taxonomy, 3D coordinates, and parity with the
emitted G-code.

``build_segments`` walks the SAME §6.1 motion sequence as the emitter, so a viewer draws
exactly what the machine moves. These tests pin the taxonomy and cross-check the segment
coordinates against the real emitted G-code — the "do the drawn coordinates match what's
emitted" guard. No heavy deps: build_segments + the emitter are light.
"""
import re

import pytest

from rotoforge_slicer.config import Config
from rotoforge_slicer.emit.rrf import GCodeEmitter
from rotoforge_slicer.toolpath.passplan import LayerPlan, Pass, ToolpathPlan
from rotoforge_slicer.toolpath.segments import (
    TOGGLE_KINDS, TOGGLE_ORDER, SegmentKind, build_segments,
)


def _straight_pass(x, y0=100.0, y1=130.0):
    return Pass(start=(x, y0), end=(x, y1), z=0.06, a_deg=0.0, rpm=5000,
                traverse_mm_min=120.0, e_per_path_mm=1.0)


def _plan(layers):
    return ToolpathPlan(layers=layers, rpm=5000, traverse_mm_min=120.0,
                        v_grind_floor_mm_min=120.0)


def _positions_from_gcode(g):
    """Reconstruct every motion endpoint from the emitted body G-code the way a
    controller would: carry X/Y/Z forward across G0/G1 moves. Starts at the G28 home
    origin; only the body (after 'G92 E0') is motion (preamble/postamble have no moves)."""
    lines = g.splitlines()
    body = lines[lines.index("G92 E0") + 1:]
    x = y = z = 0.0
    out = []
    for ln in body:
        if not (ln.startswith("G0 ") or ln.startswith("G1 ")):
            continue
        for axis, val in re.findall(r"(?:^|\s)([XYZ])(-?\d+\.?\d*)", ln):
            v = float(val)
            if axis == "X":
                x = v
            elif axis == "Y":
                y = v
            else:
                z = v
        out.append((x, y, z))
    return out


def test_toggle_taxonomy_covers_every_kind_exactly_once():
    assert TOGGLE_ORDER == ("deposition", "lead-in/out", "liftoffs", "resets", "travels")
    seen = [k for name in TOGGLE_ORDER for k in TOGGLE_KINDS[name]]
    assert set(seen) == set(SegmentKind)          # all six kinds are reachable via a toggle
    assert len(seen) == len(set(seen))            # and none is in two toggles


def test_build_segments_tags_and_contiguity():
    segs = build_segments(
        _plan([LayerPlan(0, 0.06, [_straight_pass(190), _straight_pass(191)])]), Config())
    assert {s.kind for s in segs} == set(SegmentKind)   # a two-pass plan exercises all six
    assert segs[0].start == (0.0, 0.0, 0.0)             # from the home origin
    assert segs[0].kind is SegmentKind.LIFTOFF          # initial rise
    assert segs[-1].kind is SegmentKind.LIFTOFF         # final park
    for a, b in zip(segs, segs[1:]):
        assert a.end == b.start                         # one continuous chain


def test_build_segments_empty_plan_is_empty():
    assert build_segments(_plan([LayerPlan(0, 0.06, [])]), Config()) == []


def test_segment_coordinates_match_emitted_gcode_straight():
    # THE verify: every drawn endpoint equals the emitted G-code move-for-move, across two
    # layers with multiple passes (rounded to the emitter's 3-decimal coordinate format).
    cfg = Config()
    plan = _plan([
        LayerPlan(0, 0.06, [_straight_pass(190), _straight_pass(191.5)]),
        LayerPlan(1, 0.18, [_straight_pass(192)]),
    ])
    segs = build_segments(plan, cfg)
    emitted = _positions_from_gcode(GCodeEmitter(cfg).emit(plan))
    drawn = [s.end for s in segs]
    assert len(drawn) == len(emitted) > 0
    for (dx, dy, dz), (ex, ey, ez) in zip(drawn, emitted):
        assert dx == pytest.approx(ex, abs=1e-3)
        assert dy == pytest.approx(ey, abs=1e-3)
        assert dz == pytest.approx(ez, abs=1e-3)


def test_segment_coordinates_match_emitted_gcode_curved():
    # a curved pass exercises the per-segment deposition split + lead-out heading
    cfg = Config()
    cfg.c_axis.max_speed_deg_s = 360.0
    curved = Pass.curved([(190, 100), (190, 106), (191.5, 112), (194, 117)],
                         z=0.06, rpm=5000, traverse_mm_min=120.0,
                         e_per_path_mm=1.0, c_axis=cfg.c_axis)
    plan = _plan([LayerPlan(0, 0.06, [curved])])
    segs = build_segments(plan, cfg)
    emitted = _positions_from_gcode(GCodeEmitter(cfg).emit(plan))
    assert len(segs) == len(emitted) > 0
    for s, e in zip(segs, emitted):
        assert s.end[0] == pytest.approx(e[0], abs=1e-3)
        assert s.end[1] == pytest.approx(e[1], abs=1e-3)
        assert s.end[2] == pytest.approx(e[2], abs=1e-3)


def test_named_coordinates_line_up_with_the_machine():
    # spot-check the human-meaningful landmarks a reviewer would eyeball
    cfg = Config()
    lift = cfg.process.inter_pass_lift_mm
    p = _straight_pass(190, y0=100.0, y1=130.0)
    segs = build_segments(_plan([LayerPlan(0, 0.06, [p])]), cfg)

    travel = next(s for s in segs if s.kind is SegmentKind.TRAVEL)
    assert travel.end[:2] == pytest.approx((190.0, 100.0))          # flies to the pass start
    assert travel.end[2] == pytest.approx(0.06 + lift)              # at the lift height
    dep = next(s for s in segs if s.kind is SegmentKind.DEPOSITION)
    assert dep.end[:2] == pytest.approx((190.0, 130.0))             # bead ends at the pass end
    assert dep.end[2] == pytest.approx(0.06)                        # on the layer
    lead_out = next(s for s in segs if s.kind is SegmentKind.LEAD_OUT)
    assert lead_out.end[2] == pytest.approx(0.06 + lift)            # lifts to safe Z
    assert lead_out.end[1] == pytest.approx(130.0 + cfg.process.lead_out_len_mm)  # runs past


def test_deposition_a_matches_validated_axis_angles():
    # the drawn deposition A equals the emitter's winding-resolved Pass.axis_angles
    cfg = Config()
    cfg.c_axis.max_speed_deg_s = 360.0
    p = Pass.curved([(190, 100), (190, 106), (192, 111), (195, 114)],
                    z=0.06, rpm=5000, traverse_mm_min=120.0,
                    e_per_path_mm=1.0, c_axis=cfg.c_axis)
    segs = build_segments(_plan([LayerPlan(0, 0.06, [p])]), cfg)
    validated = {round(a, 6) for a in p.axis_angles(cfg.c_axis)}
    dep_a = [round(s.a_deg, 6) for s in segs if s.kind is SegmentKind.DEPOSITION]
    assert dep_a and all(a in validated for a in dep_a)


def test_plunge_split_snaps_multi_segment_plunges_to_a_vertex():
    # review fix (hardware): a mid-segment plunge split that SPANS vertices leaves
    # an arbitrarily short in-contact remainder across which the firmware must
    # interpolate the chord->segment A step. Within one segment the chord equals
    # the heading (safe, keep mid-split); across vertices, snap to the nearest one.
    from rotoforge_slicer.toolpath.segments import plunge_split

    straight = [(0.0, 0.0), (0.0, 30.0)]
    pp, dep, seg0, plunge = plunge_split(straight, 2.0)
    assert plunge == pytest.approx(2.0) and seg0 == 0        # mid-split, one segment
    assert pp == pytest.approx((0.0, 2.0))

    fine = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.1), (1.5, 0.25), (2.0, 0.5),
            (2.5, 0.9), (3.0, 1.4), (4.0, 2.6), (5.0, 4.0)]
    pp, dep, seg0, plunge = plunge_split(fine, 2.0)
    assert pp in fine                                        # landed ON a vertex
    assert dep[0] == pp and fine[seg0] == pp
    assert abs(plunge - 2.0) < 1.0                           # near the target


def test_curved_lead_in_junction_is_emitted_safely():
    # the review's repro: a legal 2mm-radius curved lead-in used to emit an A step
    # over a ~0.1mm in-contact remainder (axis-infeasible). With vertex snapping +
    # emit-time junction validation it must emit cleanly.
    import math

    cfg = Config()
    cfg.c_axis.max_speed_deg_s = 360.0
    pts = [(190 + 2 * math.sin(t), 100 + 2 * (1 - math.cos(t)))
           for t in [i * 0.15 for i in range(20)]]           # r=2mm arc, ~0.3mm steps
    from rotoforge_slicer.toolpath.passplan import Pass

    p = Pass.curved(pts, z=0.06, rpm=5000, traverse_mm_min=120.0,
                    e_per_path_mm=1.0, c_axis=cfg.c_axis)
    plan = _plan([LayerPlan(0, 0.06, [p])])
    g = GCodeEmitter(cfg).emit(plan)                         # must not raise
    assert "M84" in g


def test_segments_carry_layer_index_for_the_scrubber():
    plan = _plan([
        LayerPlan(0, 0.06, [_straight_pass(190)]),
        LayerPlan(1, 0.18, [_straight_pass(191)]),
    ])
    segs = build_segments(plan, Config())
    assert {s.layer_index for s in segs} == {0, 1}
    # depositions on the upper layer sit at the upper Z (the scrubber hides them at upto=0)
    dep1 = [s for s in segs if s.kind is SegmentKind.DEPOSITION and s.layer_index == 1]
    assert dep1 and all(s.end[2] == pytest.approx(0.18) for s in dep1)
