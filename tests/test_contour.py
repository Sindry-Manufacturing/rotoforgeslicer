"""M17 contour / perimeter tracing: rings, the rotational-extreme start, winding
splits, planner integration, and end-to-end emission (SPEC §4.2, D13).

The D13 acceptance shape: closed rings ARE legal; an annulus traces each ring,
breaking only where winding/curvature requires — never at a privileged tangent.
"""
import math

import pytest

shapely = pytest.importorskip("shapely")
from shapely.geometry import Point, Polygon  # noqa: E402

from rotoforge_slicer.config import CAxisCfg, Config  # noqa: E402
from rotoforge_slicer.emit.rrf import GCodeEmitter  # noqa: E402
from rotoforge_slicer.fill.contour import (  # noqa: E402
    contour_paths, contour_rings, inset_interior, perimeter_paths,
    rotate_ring_to_extreme, wall_depth_mm,
)
from rotoforge_slicer.fill.heading import (  # noqa: E402
    heading_deg_from_vector, heading_to_a_deg, unwrap_headings,
)
from rotoforge_slicer.geometry import Layer  # noqa: E402
from rotoforge_slicer.process.screener import OperatingPoint  # noqa: E402
from rotoforge_slicer.toolpath.passplan import (  # noqa: E402
    ToolpathPlan, plan_layer, split_on_winding,
)


def _cfg(**fill):
    cfg = Config()
    cfg.c_axis.max_speed_deg_s = 360.0        # calibrated slew
    for k, v in fill.items():
        setattr(cfg.fill, k, v)
    return cfg


def _op(cfg, v=120.0, rpm=5000):
    return OperatingPoint(revs_per_mm=rpm / v, v_min_mm_min=v, v_max_mm_min=v,
                          rpm=rpm, traverse_mm_min=v, feed_speed_mm_min=0.0,
                          phi=0.0, torque_Nm=0.0, power_kW=0.0, t_az_c=0.0)


def _square(cx=190.0, cy=110.0, half=15.0):
    return Polygon([(cx - half, cy - half), (cx + half, cy - half),
                    (cx + half, cy + half), (cx - half, cy + half)])


def test_wall_depths_and_ring_counts():
    cfg = _cfg()
    assert wall_depth_mm(cfg, 0) == pytest.approx(0.5)        # bead/2
    assert wall_depth_mm(cfg, 1) == pytest.approx(0.5 + 0.85)  # + pitch
    sq = _square(half=5.0)
    assert len(contour_rings(sq, cfg, max_loops=1)) == 1       # outline
    all_rings = contour_rings(sq, cfg)                         # full contour
    assert len(all_rings) > 3                                  # walks all the way in
    # outermost ring sits bead/2 inside the boundary
    xs = [p[0] for p in all_rings[0]]
    assert min(xs) == pytest.approx(185.5, abs=1e-6)


def test_rotate_ring_to_extreme_finds_a_seatable_start():
    # the A-band can only shift by whole turns (the axis zero is physical), so the
    # rotated ring's open-path band must fit [a_min, a_max] at SOME winding — that,
    # not "smallest heading first", is what makes the loop a single pass.
    cfg = CAxisCfg()
    circle = [(190 + 10 * math.cos(t), 110 + 10 * math.sin(t))
              for t in [i * 2 * math.pi / 72 for i in range(72)]]
    ring = circle + [circle[0]]
    rot = rotate_ring_to_extreme(ring, cfg)
    assert rot[0] == rot[-1] and len(rot) == len(ring)         # still closed, same size
    assert set(rot) == set(ring)                               # a pure rotation
    heads = [heading_deg_from_vector(b[0] - a[0], b[1] - a[1])
             for a, b in zip(rot, rot[1:])]
    a_cont = [heading_to_a_deg(t, cfg) for t in unwrap_headings(heads)]
    span = max(a_cont) - min(a_cont)
    assert span < 360.0                                        # closing turn off the end
    # the band seats at some whole-turn winding inside [-180, 180]
    assert any(cfg.a_min_deg - 1e-6 <= min(a_cont) + 360 * k
               and max(a_cont) + 360 * k <= cfg.a_max_deg + 1e-6
               for k in (-2, -1, 0, 1, 2))


def test_closed_ring_single_pass_arcs_or_clear_rejection():
    # D13/M17, three regimes for closed rings:
    cfg = _cfg()
    circle = [(190 + 10 * math.cos(t), 110 + 10 * math.sin(t))
              for t in [i * 2 * math.pi / 72 for i in range(72)]]
    ring = rotate_ring_to_extreme(circle + [circle[0]], cfg.c_axis)
    # 1. a 360°-wide range + seatable start: the whole loop is ONE pass
    assert len(split_on_winding(ring, cfg.c_axis)) == 1

    # 2. a sub-360° range leaves headings UNREACHABLE at any winding (an unwind
    #    resets accumulated angle; it cannot create new reachable headings) — the
    #    planner must reject clearly, not emit an impossible pass.
    narrow = CAxisCfg(a_min_deg=-90.0, a_max_deg=90.0)
    with pytest.raises(ValueError, match="unreachable"):
        split_on_winding(rotate_ring_to_extreme(ring, narrow), narrow)

    # 3. a NON-CONVEX ring whose total sweep exceeds the range width splits into
    #    arcs + airborne unwinds — every heading reachable, winding just re-set.
    wavy = [(190 + (10 + 3 * math.cos(3 * t)) * math.cos(t),
             110 + (10 + 3 * math.cos(3 * t)) * math.sin(t))
            for t in [i * 2 * math.pi / 144 for i in range(144)]]
    wavy_ring = rotate_ring_to_extreme(wavy + [wavy[0]], cfg.c_axis)
    subs = split_on_winding(wavy_ring, cfg.c_axis)
    assert len(subs) >= 2                                      # arcs + unwinds


def test_plan_layer_contour_annulus_traces_both_rings():
    cfg = _cfg(mode="contour")
    annulus = Point(190, 110).buffer(14.0).difference(Point(190, 110).buffer(7.0))
    layer = Layer(0, 0.06, [annulus])
    lp = plan_layer(layer, cfg, operating_point=_op(cfg), e_per_path=1.0)
    assert lp.passes                                           # rings became passes
    assert all(p.is_curved for p in lp.passes)
    # rings around BOTH boundaries: some passes hug r~13.5, some r~7.5
    def ring_radius(p):
        return sum(math.hypot(q[0] - 190, q[1] - 110) for q in p.points) / len(p.points)
    radii = sorted(ring_radius(p) for p in lp.passes)
    assert radii[0] < 9.0 and radii[-1] > 12.0
    # every pass's winding-resolved A is in range (the emitter will re-prove this)
    for p in lp.passes:
        for a in p.axis_angles(cfg.c_axis):
            assert cfg.c_axis.a_min_deg - 1e-6 <= a <= cfg.c_axis.a_max_deg + 1e-6


def test_contour_plan_emits_valid_gcode():
    cfg = _cfg(mode="contour")
    disc = Point(190, 110).buffer(12.0)
    lp = plan_layer(Layer(0, 0.06, [disc]), cfg, operating_point=_op(cfg), e_per_path=1.0)
    plan = ToolpathPlan([lp], 5000, 120.0, 120.0)
    g = GCodeEmitter(cfg).emit(plan)                           # §6.3 validators run
    assert "M84" in g and "G1" in g


def test_outline_mode_traces_boundary_only():
    cfg = _cfg(mode="outline")
    lp = plan_layer(Layer(0, 0.06, [_square(half=10.0)]), cfg,
                    operating_point=_op(cfg), e_per_path=1.0)
    assert lp.passes
    # all passes on the single outermost centreline (bead/2 = 0.5 inside)
    for p in lp.passes:
        for (x, y) in p.points:
            d = min(abs(x - 180.5), abs(x - 199.5), abs(y - 100.5), abs(y - 119.5))
            assert d < 0.3


def test_perimeter_loops_walls_plus_inset_raster():
    cfg = _cfg(mode="raster", perimeter_loops=2)
    sq = _square(half=12.0)
    lp = plan_layer(Layer(0, 0.06, [sq]), cfg, operating_point=_op(cfg), e_per_path=1.0)
    walls = [p for p in lp.passes if p.is_curved]
    lines = [p for p in lp.passes if not p.is_curved]
    assert walls and lines                                     # both kinds present
    assert lp.passes[-1] in walls                              # walls deposited last
    # raster is inset past the innermost wall: hatch x-extent well inside the region
    inset = inset_interior(sq, cfg, 2)
    ix0, _, ix1, _ = inset.bounds
    for p in lines:
        assert ix0 - 1e-6 <= p.start[0] <= ix1 + 1e-6


def test_inset_interior_consumed_by_walls_is_empty():
    cfg = _cfg()
    tiny = _square(half=1.5)                                   # 3 mm square
    assert inset_interior(tiny, cfg, 2).is_empty
    assert perimeter_paths(tiny, cfg) == [] if cfg.fill.perimeter_loops == 0 else True


def test_open_path_returned_unchanged_by_extreme_rotation():
    cfg = CAxisCfg()
    open_path = [(0, 0), (5, 0), (10, 3)]
    assert rotate_ring_to_extreme(open_path, cfg) == open_path
