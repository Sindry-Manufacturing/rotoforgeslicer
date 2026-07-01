"""Pass planning: constant-(v, RPM) passes; bidirectional raster + winding (D13). SPEC §4.5."""
import math

import pytest

pytest.importorskip("shapely")
from shapely.geometry import Polygon  # noqa: E402

from rotoforge_slicer.config import Config  # noqa: E402
from rotoforge_slicer.geometry import Layer, SlicedModel  # noqa: E402
from rotoforge_slicer.toolpath.passplan import (  # noqa: E402
    default_operating_point,
    plan_toolpath,
)


def _model():
    layer = Layer(0, 0.06, [Polygon([(0, 0), (20, 0), (20, 12), (0, 12)])])
    return SlicedModel([layer], layer_height=0.12, z_min=0.0, z_max=0.12)


def test_default_operating_point_single_speed():
    cfg = Config()
    op = default_operating_point(cfg)
    assert op.traverse_mm_min == cfg.emit.feed_dep_mm_min == 120.0
    assert op.rpm == cfg.spindle.rpm_min == 5000
    assert op.v_grind_floor_mm_min == op.traverse_mm_min  # one speed -> floor == v


def test_unidirectional_raster_passes_forward_y_and_extruding():
    cfg = Config()
    cfg.fill.raster_bidirectional = False               # legacy one-way +Y sweep
    plan = plan_toolpath(_model(), cfg)
    assert plan.npasses > 0
    for ly in plan.layers:
        for p in ly.passes:
            assert p.a_deg == 0.0                          # +Y -> A0 (home)
            assert math.isclose(p.heading_deg, 90.0)       # forward +Y
            assert p.end[1] > p.start[1]
            assert p.length_mm >= cfg.process.min_deposit_len_mm
            assert p.e_total_mm > 0                         # wire feeding


def test_bidirectional_raster_alternates_heading_180():
    # D13: the default raster is bidirectional — adjacent lines run 180 deg apart, so the
    # head just turns airborne instead of flying back.
    cfg = Config()                                       # raster_bidirectional defaults True
    passes = [p for ly in plan_toolpath(_model(), cfg).layers for p in ly.passes]
    headings = [p.heading_deg for p in passes]
    assert any(abs(h - 90) < 1e-6 for h in headings)     # some +Y lines
    assert any(abs(abs(h) - 90) < 1e-6 and h < 0 for h in headings)   # some -Y lines
    for a, b in zip(passes, passes[1:]):                 # consecutive lines are 180 apart
        d = abs((b.heading_deg - a.heading_deg + 180) % 360 - 180)
        assert abs(d - 180) < 1e-6


def test_plan_revs_per_mm_constant():
    cfg = Config()
    plan = plan_toolpath(_model(), cfg)
    assert plan.rpm == 5000 and plan.traverse_mm_min == 120.0
    assert math.isclose(plan.revs_per_mm, 5000 / 120.0)
    # every pass shares the single operating point
    assert all(p.rpm == plan.rpm and p.traverse_mm_min == plan.traverse_mm_min
               for ly in plan.layers for p in ly.passes)


def test_plan_accepts_any_heading_no_wedge():
    # D13: no wedge -> any base heading plans fine (here +X, which pre-D13 was rejected).
    cfg = Config()
    plan = plan_toolpath(_model(), cfg, heading_deg=0.0)   # +X
    assert plan.npasses > 0
    for ly in plan.layers:
        for p in ly.passes:                                # +X lines (or 180 reversed)
            assert math.isclose(abs(p.heading_deg), 0.0) or math.isclose(abs(p.heading_deg), 180.0)


@pytest.mark.parametrize("mode", ["x", "volume"])
def test_e_per_path_mode_selection(mode):
    from rotoforge_slicer.process.extrusion import e_per_path_mm, wire_area_mm2

    cfg = Config()
    cfg.extrusion.mode = mode
    plan = plan_toolpath(_model(), cfg)
    e = plan.layers[0].passes[0].e_per_path_mm
    if mode == "x":
        assert math.isclose(e, cfg.extrusion.x_ratio)
    else:  # volume
        expected = (cfg.process.bead_width_mm * cfg.process.layer_height_mm
                    / wire_area_mm2(cfg.process.wire_diameter_mm))
        assert math.isclose(e, expected)


def test_screener_e_per_path_is_feed_over_traverse():
    """screener E coupling = feed_speed/traverse, distinct from x (1.0) / volume (~0.61)."""
    from rotoforge_slicer.process.screener import OperatingPoint

    cfg = Config()  # extrusion.mode defaults to "screener"
    op = OperatingPoint(
        revs_per_mm=150.0, v_min_mm_min=60.0, v_max_mm_min=140.0, rpm=15000,
        traverse_mm_min=100.0, feed_speed_mm_min=130.0, phi=1.3,
        torque_Nm=1.3, power_kW=0.9, t_az_c=460.0)
    plan = plan_toolpath(_model(), cfg, operating_point=op)
    e = plan.layers[0].passes[0].e_per_path_mm
    assert math.isclose(e, 130.0 / 100.0)            # 1.3 — screener-derived
    assert not math.isclose(e, cfg.extrusion.x_ratio)  # not the x-fallback


def test_screener_operating_point_drives_traverse_and_revs_per_mm():
    """A supplied operating point sets the pass traverse/RPM, not cfg.emit defaults."""
    from rotoforge_slicer.process.screener import OperatingPoint

    cfg = Config()
    op = OperatingPoint(
        revs_per_mm=80.0, v_min_mm_min=150.0, v_max_mm_min=250.0,
        rpm=16000, traverse_mm_min=200.0, feed_speed_mm_min=300.0,
        phi=1.0, torque_Nm=0.0, power_kW=0.0, t_az_c=0.0)
    plan = plan_toolpath(_model(), cfg, operating_point=op)
    assert plan.traverse_mm_min == 200.0 and plan.rpm == 16000
    assert plan.v_grind_floor_mm_min == 150.0
    assert all(p.traverse_mm_min == 200.0 for ly in plan.layers for p in ly.passes)


def test_crosshatch_alternates_heading_by_layer():
    from rotoforge_slicer.toolpath.passplan import layer_heading_deg

    cfg = Config()
    cfg.fill.crosshatch = True
    cfg.fill.crosshatch_angle_deg = 30.0
    assert layer_heading_deg(cfg, 0) == 120.0   # 90 + 30
    assert layer_heading_deg(cfg, 1) == 60.0    # 90 - 30
    cfg.fill.crosshatch = False
    assert layer_heading_deg(cfg, 0) == 90.0 and layer_heading_deg(cfg, 3) == 90.0


def test_pass_polyline_geometry():
    from rotoforge_slicer.config import CAxisCfg
    from rotoforge_slicer.toolpath.passplan import Pass

    c = CAxisCfg()
    p = Pass.curved([(0, 0), (0, 5), (2, 8)], z=0.1, rpm=5000,
                    traverse_mm_min=120.0, e_per_path_mm=1.0, c_axis=c)
    assert p.is_curved
    assert math.isclose(p.length_mm, 5.0 + math.hypot(2, 3))
    assert len(p.segments()) == 2
    assert len(p.segment_a_degs(c)) == 2
    # straight pass stays a 2-point polyline with a single heading
    s = Pass(start=(0, 0), end=(0, 10), z=0.1, a_deg=0.0, rpm=5000,
             traverse_mm_min=120.0, e_per_path_mm=1.0)
    assert not s.is_curved and s.points == [(0, 0), (0, 10)]


def test_streamline_mode_builds_curved_passes():
    pytest.importorskip("scipy")
    from shapely.geometry import Point

    cfg = Config()
    cfg.fill.mode = "streamline"
    cfg.fill.streamline_curl = 0.8
    region = Point(10, 15).buffer(12.0, quad_segs=48)
    model = SlicedModel([Layer(0, 0.06, [region])], layer_height=0.12, z_min=0.0, z_max=0.12)
    plan = plan_toolpath(model, cfg)
    assert plan.npasses > 0
    assert any(p.is_curved for ly in plan.layers for p in ly.passes)


def test_streamline_passes_emit_within_limit_and_axis_range():
    """End-to-end: a curved streamline plan emits with the curvature limit ACTIVE and
    every commanded A inside the usable axis range (SPEC §4.2/§4.3/§6.3; D13)."""
    pytest.importorskip("scipy")
    import re

    from shapely.geometry import Point

    from rotoforge_slicer.emit.rrf import GCodeEmitter

    cfg = Config()
    cfg.fill.mode = "streamline"
    cfg.fill.streamline_curl = 0.8
    cfg.c_axis.max_speed_deg_s = 360.0                  # limit active
    region = Point(190, 117).buffer(12.0, quad_segs=48)
    model = SlicedModel([Layer(0, 0.06, [region])], layer_height=0.12, z_min=0.0, z_max=0.12)
    plan = plan_toolpath(model, cfg)
    assert any(p.is_curved for ly in plan.layers for p in ly.passes)
    g = GCodeEmitter(cfg).emit(plan)                    # runs R>=R_min + axis-range + winding
    a = [float(m) for l in g.splitlines() for m in re.findall(r" A(-?\d+\.?\d*)", l)]
    assert a and all(cfg.c_axis.a_min_deg - 1e-6 <= v <= cfg.c_axis.a_max_deg + 1e-6
                     for v in a)


def test_crosshatch_emits_crossing_layers_in_axis_range():
    import re

    from shapely.geometry import Polygon

    from rotoforge_slicer.emit.rrf import GCodeEmitter

    cfg = Config()
    cfg.fill.crosshatch = True
    cfg.fill.crosshatch_angle_deg = 30.0
    cfg.fill.raster_bidirectional = False               # isolate the cross-LAYER crossing
    region = Polygon([(180, 105), (200, 105), (200, 130), (180, 130)])
    layers = [Layer(i, 0.06 + 0.12 * i, [region]) for i in range(2)]
    model = SlicedModel(layers, layer_height=0.12, z_min=0.0, z_max=0.24)
    g = GCodeEmitter(cfg).emit(plan_toolpath(model, cfg))
    a = [float(m) for l in g.splitlines() for m in re.findall(r" A(-?\d+\.?\d*)", l)]
    assert any(v > 5 for v in a) and any(v < -5 for v in a)   # adjacent layers cross (+30/-30)
    assert all(cfg.c_axis.a_min_deg - 1e-6 <= v <= cfg.c_axis.a_max_deg + 1e-6 for v in a)


def test_plan_holed_layer_splits_line_into_two_passes():
    from collections import Counter

    from shapely.geometry import Polygon

    # 20x20 square with a central hole spanning y in [8,12]; lines through the hole
    # split into [0,8] and [12,20] (each 8 mm >= min_deposit_len), so 2 passes/line.
    outer = Polygon([(0, 0), (20, 0), (20, 20), (0, 20)],
                    [[(7, 8), (13, 8), (13, 12), (7, 12)]])
    model = SlicedModel([Layer(0, 0.06, [outer])], layer_height=0.12, z_min=0.0, z_max=0.12)
    plan = plan_toolpath(model, Config())
    per_x = Counter(round(p.start[0], 3) for ly in plan.layers for p in ly.passes)
    assert max(per_x.values()) == 2   # a holed line -> two passes
    assert min(per_x.values()) == 1   # a clear line -> one pass
    assert all(p.e_total_mm > 0 for ly in plan.layers for p in ly.passes)


# ---- D13 winding management -------------------------------------------------

def _circle(n=24, r=10.0):
    """A convex closed loop: its heading sweeps ~360 deg around the loop."""
    return [(r * math.cos(i * math.tau / n), r * math.sin(i * math.tau / n))
            for i in range(n + 1)]


def test_closed_loop_one_pass_when_range_can_wind_the_whole_turn():
    # D13 closed contour: a ~360 deg heading sweep stays ONE pass only when the range can
    # wind the whole turn at one winding; a tighter range that cannot span the linear
    # ±seam breaks it into arcs with airborne unwinds. Parameterized by a_min/a_max.
    from rotoforge_slicer.config import CAxisCfg
    from rotoforge_slicer.toolpath.passplan import split_on_winding

    loop = _circle()
    wide = CAxisCfg(a_min_deg=-360.0, a_max_deg=360.0)     # winds the full turn -> 1 pass
    assert len(split_on_winding(loop, wide)) == 1
    phys = CAxisCfg(a_min_deg=-180.0, a_max_deg=180.0)     # can't span the seam -> arcs
    assert len(split_on_winding(loop, phys)) >= 2


def test_split_on_winding_rejects_an_unreachable_heading():
    # With a narrow range, a heading whose A cannot be wound into range at ANY winding is
    # physically unreachable (not splittable) — fail early and clearly, don't emit a bad A.
    from rotoforge_slicer.config import CAxisCfg
    from rotoforge_slicer.toolpath.passplan import split_on_winding

    narrow = CAxisCfg(a_min_deg=-30.0, a_max_deg=30.0)
    # +Y then down-right (heading ~ -45 deg -> A ~ -135, unreachable at +/-30)
    path = [(0.0, 0.0), (0.0, 5.0), (3.0, 2.0)]
    with pytest.raises(ValueError):
        split_on_winding(path, narrow)


def test_winding_accumulation_never_exceeds_the_range():
    # every split sub-path, once wound, has all its commanded A inside [a_min, a_max] —
    # the planner inserts an unwind (a break) exactly where the band would overrun.
    from rotoforge_slicer.config import CAxisCfg
    from rotoforge_slicer.toolpath.passplan import Pass, split_on_winding

    rng = CAxisCfg(a_min_deg=-180.0, a_max_deg=180.0)
    subs = split_on_winding(_circle(), rng)
    assert len(subs) >= 2
    for sub in subs:
        if len(sub) < 2:
            continue
        p = Pass.curved(sub, z=0.1, rpm=5000, traverse_mm_min=120.0,
                        e_per_path_mm=1.0, c_axis=rng)
        for a in p.axis_angles(rng):
            assert rng.a_min_deg - 1e-6 <= a <= rng.a_max_deg + 1e-6
