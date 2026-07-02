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
    inset = inset_interior(sq, cfg, 2)
    ix0, _, ix1, _ = inset.bounds
    # square wall rings split at their corners (heading-step rule) into straight
    # sides; a raster hatch line is vertical AND inside the inset interior.
    def is_hatch(p):
        return (abs(p.start[0] - p.end[0]) < 1e-6
                and ix0 - 1e-6 <= p.start[0] <= ix1 + 1e-6)
    hatch = [p for p in lp.passes if is_hatch(p)]
    walls = [p for p in lp.passes if not is_hatch(p)]
    assert hatch and walls                                     # both kinds present
    assert any(abs(p.start[1] - p.end[1]) < 1e-6 for p in walls)  # horizontal sides
    assert not is_hatch(lp.passes[-1])                         # walls deposited last


def test_inset_interior_consumed_by_walls_is_empty():
    cfg = _cfg()
    tiny = _square(half=1.5)                                   # 3 mm square
    walls, fitted = perimeter_paths(tiny, _cfg(perimeter_loops=2))
    assert fitted >= 1 and walls
    assert inset_interior(tiny, cfg, 2).is_empty               # nothing left inside


def test_open_path_returned_unchanged_by_extreme_rotation():
    cfg = CAxisCfg()
    open_path = [(0, 0), (5, 0), (10, 3)]
    assert rotate_ring_to_extreme(open_path, cfg) == open_path


def test_sharp_corners_split_into_airborne_reorients():
    # review fix (hardware): a dead-sharp corner between long legs slips past the
    # circumradius proxy, but the firmware would interpolate the A step across the
    # whole next in-contact segment — off-tangent scrubbing. Corners sharper than
    # max_heading_step_deg must break the pass.
    from rotoforge_slicer.fill.curvature import split_on_heading_step

    square_ring = [(0, 0), (20, 0), (20, 20), (0, 20), (0, 0)]
    subs = split_on_heading_step(square_ring, 15.0)
    assert len(subs) == 4                                      # one pass per side
    assert split_on_heading_step(square_ring, 0.0) == [square_ring]  # disabled

    cfg = _cfg(mode="outline")
    lp = plan_layer(Layer(0, 0.06, [_square(half=10.0)]), cfg,
                    operating_point=_op(cfg), e_per_path=1.0)
    assert len(lp.passes) >= 4                                 # sides, not one ring
    for p in lp.passes:                                        # no step survives
        a = p.axis_angles(cfg.c_axis)
        assert all(abs(b - c) <= 15.0 + 1e-6 for b, c in zip(a, a[1:]))


def test_emitter_proves_the_heading_step_limit():
    # a synthetic 90-deg in-contact corner must be REJECTED by the emitter even if a
    # (buggy or foreign) planner produced it — the emitter proves, never trusts.
    from rotoforge_slicer.toolpath.passplan import LayerPlan, Pass

    cfg = _cfg()
    bad = Pass.curved([(190, 100), (190, 120), (210, 120)],   # 90-deg corner
                      z=0.06, rpm=5000, traverse_mm_min=120.0,
                      e_per_path_mm=1.0, c_axis=cfg.c_axis)
    plan = ToolpathPlan([LayerPlan(0, 0.06, [bad])], 5000, 120.0, 120.0)
    with pytest.raises(ValueError, match="steps"):
        GCodeEmitter(cfg).emit(plan)


def test_unreachable_headings_deposit_in_reverse_on_sub_360_range():
    # review fix: a real (calibrated, sub-360deg) axis range leaves some headings
    # unreachable at any winding; there is no privileged direction (D13), so those
    # arcs deposit in REVERSE instead of aborting the slice.
    from rotoforge_slicer.toolpath.passplan import split_unreachable

    c = CAxisCfg(a_min_deg=-170.0, a_max_deg=170.0)
    minus_y = [(190.0, 130.0), (190.0, 100.0)]                # heading -Y: A=±180
    subs = split_unreachable(minus_y, c)
    assert subs == [[(190.0, 100.0), (190.0, 130.0)]]         # reversed -> +Y, A=0

    # -Y reverses to +Y (A=0), reachable even on a narrow range — use a -X segment
    # instead: A=90 forward and A=-90 reversed are BOTH outside ±80.
    too_narrow = CAxisCfg(a_min_deg=-80.0, a_max_deg=80.0)    # < 180 deg wide
    minus_x = [(210.0, 100.0), (190.0, 100.0)]
    with pytest.raises(ValueError, match="either travel direction"):
        split_unreachable(minus_x, too_narrow)


def test_contour_slices_and_emits_on_a_calibrated_sub_360_range():
    # the M17 headline case on a REAL machine range: rings still trace (arcs, some
    # reversed, airborne unwinds between) and the emitter accepts the result.
    cfg = _cfg(mode="contour")
    cfg.c_axis.a_min_deg, cfg.c_axis.a_max_deg = -170.0, 170.0
    disc = Point(190, 110).buffer(12.0)
    lp = plan_layer(Layer(0, 0.06, [disc]), cfg, operating_point=_op(cfg),
                    e_per_path=1.0)
    assert len(lp.passes) >= 8                                # rings trace as arcs
    assert all(not (abs(p.points[0][0] - p.points[-1][0]) < 1e-9
                    and abs(p.points[0][1] - p.points[-1][1]) < 1e-9)
               for p in lp.passes)                            # no full ring survives
    plan = ToolpathPlan([lp], 5000, 120.0, 120.0)
    g = GCodeEmitter(cfg).emit(plan)                          # §6.3 validators pass
    assert "M84" in g


def test_thin_rib_gets_infill_when_requested_walls_do_not_fit():
    # review fix: inset by the walls that actually FIT — a 2.6 mm rib with
    # perimeter_loops=2 fits one wall; the interior must still get a hatch line
    # instead of a silent longitudinal void.
    from rotoforge_slicer.fill.contour import perimeter_paths

    cfg = _cfg(mode="raster", perimeter_loops=2)
    rib = Polygon([(180, 100), (210, 100), (210, 102.6), (180, 102.6)])
    walls, fitted = perimeter_paths(rib, cfg)
    assert fitted == 1 and walls                              # only one wall fits
    assert not inset_interior(rib, cfg, fitted).is_empty      # interior remains
    # hatch ALONG the rib (heading 0) so the core lines beat min_deposit_len; a
    # core hatch line is horizontal at the rib's mid-height (walls sit at ±0.5)
    lp = plan_layer(Layer(0, 0.06, [rib]), cfg, operating_point=_op(cfg),
                    e_per_path=1.0, heading_deg=0.0)
    assert any(abs(p.start[1] - p.end[1]) < 1e-6
               and abs(p.start[1] - 101.3) < 0.4 for p in lp.passes)


def test_wall_to_infill_spacing_is_one_pitch():
    # review fix: the infill boundary retreats HALF a pitch past the innermost wall
    # centreline, so the first hatch line (pitch/2 inside the boundary) sits exactly
    # one pitch from the wall — no unfused seam.
    from rotoforge_slicer.fill.raster import raster_pitch

    cfg = _cfg()
    pitch = raster_pitch(cfg)
    sq = _square(half=12.0)
    inner = inset_interior(sq, cfg, 1)
    wall_centre_depth = wall_depth_mm(cfg, 0)
    ix0 = inner.bounds[0]
    first_line_x = ix0 + pitch / 2.0
    wall_x = sq.bounds[0] + wall_centre_depth
    assert first_line_x - wall_x == pytest.approx(pitch, abs=1e-9)
