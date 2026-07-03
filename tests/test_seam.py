"""Seam placement (port #3, PrusaSlicer seam-engine port): the seat window, the
nearest/aligned/random policies, the deposit-loss guard, determinism, and
end-to-end emission. The extreme default must be byte-identical to the legacy
rotational-extreme behavior (D13 / M17)."""
import math

import pytest

shapely = pytest.importorskip("shapely")
from shapely.geometry import Point  # noqa: E402

from rotoforge_slicer.config import CAxisCfg, Config  # noqa: E402
from rotoforge_slicer.emit.rrf import GCodeEmitter  # noqa: E402
from rotoforge_slicer.fill.contour import (  # noqa: E402
    SeamContext, choose_seam_start, contour_paths, rotate_ring_to_extreme,
    seat_window,
)
from rotoforge_slicer.geometry import Layer  # noqa: E402
from rotoforge_slicer.process.screener import OperatingPoint  # noqa: E402
from rotoforge_slicer.toolpath.passplan import (  # noqa: E402
    ToolpathPlan, plan_toolpath, split_on_winding,
)


def _cfg(**fill):
    cfg = Config()
    cfg.c_axis.max_speed_deg_s = 360.0
    for k, v in fill.items():
        setattr(cfg.fill, k, v)
    return cfg


def _op(v=120.0, rpm=5000):
    # feed == traverse -> e_per_path_mm = 1.0 (plan_toolpath derives extrusion)
    return OperatingPoint(revs_per_mm=rpm / v, v_min_mm_min=v, v_max_mm_min=v,
                          rpm=rpm, traverse_mm_min=v, feed_speed_mm_min=v,
                          phi=0.0, torque_Nm=0.0, power_kW=0.0, t_az_c=0.0)


def _circle(cx=190.0, cy=110.0, r=10.0, n=72):
    pts = [(cx + r * math.cos(t), cy + r * math.sin(t))
           for t in [i * 2 * math.pi / n for i in range(n)]]
    return pts + [pts[0]]


def _ctx(cfg, **kw):
    ctx = SeamContext.from_cfg(cfg, v_mm_min=120.0)
    for k, v in kw.items():
        setattr(ctx, k, v)
    return ctx


class _Model:
    def __init__(self, layers):
        self.layers = layers


def _disc_layers(n_layers=3, r=10.0):
    disc = Point(190, 110).buffer(r)
    return _Model([Layer(i, 0.06 * (i + 1), [disc]) for i in range(n_layers)])


# ---- seat window ------------------------------------------------------------------

def test_seat_window_matches_extreme_and_seats_everywhere():
    c = CAxisCfg()                                 # ±180 -> W = 360
    ring = _circle()
    window = seat_window(ring, c)
    assert window, "a convex ring must seat somewhere at W=360"
    assert len(window) <= 2, "at W=360 the window is pinned to the range stop"
    # window[0] IS the legacy extreme start
    assert rotate_ring_to_extreme(ring, c)[0] == ring[:-1][window[0]]
    # every windowed start really yields a one-pass ring
    for m in window:
        cyc = ring[:-1]
        rot = cyc[m:] + cyc[:m] + [cyc[m]]
        assert len(split_on_winding(rot, c)) == 1


def test_seat_window_widens_with_the_axis_range():
    wide = CAxisCfg(a_min_deg=-220.0, a_max_deg=220.0)     # W = 440
    ring = _circle()
    window = seat_window(ring, wide)
    assert len(window) > len(ring) * 0.15, \
        "W-360=80 deg of slack must open a wide window (~22% of starts)"
    for m in window[:: max(1, len(window) // 8)]:
        cyc = ring[:-1]
        rot = cyc[m:] + cyc[:m] + [cyc[m]]
        assert len(split_on_winding(rot, wide)) == 1


def test_seat_window_open_input_empty():
    assert seat_window([(0, 0), (5, 0), (10, 3)], CAxisCfg()) == []


# ---- extreme default == legacy ------------------------------------------------------

def test_extreme_policy_is_exactly_legacy():
    cfg = _cfg(seam_position="extreme")
    ring = _circle()
    assert choose_seam_start(ring, cfg, None) == rotate_ring_to_extreme(ring, cfg.c_axis)
    # a non-extreme policy without a context also degrades to legacy
    cfg2 = _cfg(seam_position="random")
    assert choose_seam_start(ring, cfg2, None) == rotate_ring_to_extreme(ring, cfg2.c_axis)


# ---- random ---------------------------------------------------------------------------

def test_random_scatters_on_a_wide_range_and_stays_one_pass():
    cfg = _cfg(seam_position="random")
    cfg.c_axis.a_min_deg, cfg.c_axis.a_max_deg = -220.0, 220.0
    ctx = _ctx(cfg)
    starts = set()
    for _ in range(8):
        rot = choose_seam_start(_circle(), cfg, ctx)
        starts.add(rot[0])
        assert len(split_on_winding(rot, cfg.c_axis)) == 1     # one_pass honored
    assert len(starts) >= 3, "random must scatter within the wide seat window"


def test_random_at_w360_one_pass_degenerates_to_window_with_note():
    cfg = _cfg(seam_position="random")                          # W = 360
    ctx = _ctx(cfg)
    ring = _circle()
    window = seat_window(ring, cfg.c_axis)
    rot = choose_seam_start(ring, cfg, ctx)
    assert rot[0] == ring[:-1][window[0]] or rot[0] in [ring[:-1][m] for m in window]
    assert len(split_on_winding(rot, cfg.c_axis)) == 1
    assert ctx.notes and "seat window" in ctx.notes[0]

    # one_pass=False buys placement freedom at the cost of >= 1 winding split
    cfg2 = _cfg(seam_position="random", seam_prefer_one_pass=False)
    ctx2 = _ctx(cfg2)
    rots = [choose_seam_start(_circle(), cfg2, ctx2) for _ in range(6)]
    assert len({r[0] for r in rots}) >= 3, "off-window starts scatter"
    assert any(len(split_on_winding(r, cfg2.c_axis)) >= 2 for r in rots)


def test_random_never_loses_deposit_length_vs_extreme():
    """The PF1 guard: a policy start may not drop more sub-min-length bead than
    the extreme baseline. Wavy-but-coarse ring + off-window starts."""
    from rotoforge_slicer.toolpath.passplan import curved_subpaths

    cfg = _cfg(seam_position="random", seam_prefer_one_pass=False)
    min_len = cfg.process.min_deposit_len_mm

    def lost(ring):
        subs = curved_subpaths(ring, cfg, 120.0)
        return sum(sum(math.hypot(b[0] - a[0], b[1] - a[1])
                       for a, b in zip(s, s[1:]))
                   for s in subs
                   if sum(math.hypot(b[0] - a[0], b[1] - a[1])
                          for a, b in zip(s, s[1:])) < min_len)

    base_lost = lost(rotate_ring_to_extreme(_circle(), cfg.c_axis))
    ctx = _ctx(cfg)
    for _ in range(10):
        rot = choose_seam_start(_circle(), cfg, ctx)
        assert lost(rot) <= base_lost + 1e-9


# ---- nearest -------------------------------------------------------------------------

def test_nearest_picks_candidate_closest_to_previous_seam():
    cfg = _cfg(seam_position="nearest", seam_prefer_one_pass=False)
    cfg.c_axis.a_min_deg, cfg.c_axis.a_max_deg = -220.0, 220.0
    ctx = _ctx(cfg, last_xy=(200.0, 110.0))        # due east of the ring centre
    rot = choose_seam_start(_circle(), cfg, ctx)
    d = math.hypot(rot[0][0] - 200.0, rot[0][1] - 110.0)
    assert d < 1.5, f"nearest start should hug the target (got {d:.2f} mm away)"
    assert ctx.last_xy == rot[0]                    # chain advances


def test_nearest_falls_back_to_previous_layer_last_seam():
    cfg = _cfg(seam_position="nearest", seam_prefer_one_pass=False)
    cfg.c_axis.a_min_deg, cfg.c_axis.a_max_deg = -220.0, 220.0
    ctx = _ctx(cfg)
    ctx.prev_seams = [((180.0, 110.0), (180, 100, 200, 120))]   # west point
    ctx.last_xy = None                              # fresh layer
    rot = choose_seam_start(_circle(), cfg, ctx)
    assert math.hypot(rot[0][0] - 180.0, rot[0][1] - 110.0) < 1.5


# ---- aligned -------------------------------------------------------------------------

def test_aligned_chains_across_layers_and_is_deterministic():
    cfg = _cfg(mode="contour", seam_position="aligned", seam_prefer_one_pass=False)
    cfg.c_axis.a_min_deg, cfg.c_axis.a_max_deg = -220.0, 220.0
    plan = plan_toolpath(_disc_layers(4), cfg, operating_point=_op())
    plan2 = plan_toolpath(_disc_layers(4), cfg, operating_point=_op())
    # determinism: identical geometry in, identical passes out
    for la, lb in zip(plan.layers, plan2.layers):
        assert [p.points for p in la.passes] == [p.points for p in lb.passes]
    # alignment: the outermost ring's first point stays put layer over layer
    firsts = [ly.passes[-1].points[0] for ly in plan.layers if ly.passes]
    assert len(firsts) == 4
    for a, b in zip(firsts, firsts[1:]):
        assert math.hypot(a[0] - b[0], a[1] - b[1]) <= 2.0, \
            "aligned seams must form a coherent vertical chain"


def test_aligned_rejects_teleport_targets():
    cfg = _cfg(seam_position="aligned", seam_prefer_one_pass=False)
    cfg.c_axis.a_min_deg, cfg.c_axis.a_max_deg = -220.0, 220.0
    ctx = _ctx(cfg)
    # a previous seam 100 mm away: outside seam_align_radius_mm of any candidate
    ctx.prev_seams = [((300.0, 110.0), (295, 105, 305, 115))]
    rot = choose_seam_start(_circle(), cfg, ctx)
    window = seat_window(_circle(), cfg.c_axis)
    assert rot[0] == _circle()[:-1][window[0]], \
        "unreachable alignment target -> deterministic chain birth at the baseline"


# ---- end-to-end ----------------------------------------------------------------------

def test_full_plan_emits_valid_gcode_for_every_policy():
    for policy in ("extreme", "nearest", "aligned", "random"):
        for one_pass in (True, False):
            cfg = _cfg(mode="contour", seam_position=policy,
                       seam_prefer_one_pass=one_pass)
            plan = plan_toolpath(_disc_layers(2), cfg, operating_point=_op())
            g = GCodeEmitter(cfg).emit(plan)       # §6.3 validators prove it
            assert "M84" in g, f"{policy}/{one_pass}"


def test_plan_warnings_surface_when_window_constrained():
    cfg = _cfg(mode="contour", seam_position="random")   # W=360 + one_pass default
    plan = plan_toolpath(_disc_layers(1), cfg, operating_point=_op())
    assert plan.warnings and "seat window" in plan.warnings[0]
    # extreme (the default) never warns
    cfg2 = _cfg(mode="contour")
    assert plan_toolpath(_disc_layers(1), cfg2, operating_point=_op()).warnings == []


def test_one_pass_guard_rejects_windowed_starts_that_split():
    """Review fix: the seat window is winding-only — a sharp corner AT the
    baseline seam becomes a mandatory heading-step split from any other start.
    With one_pass on, a windowed candidate that dry-runs to MORE passes than the
    baseline must be rejected (the setting promises one pass per ring)."""
    from rotoforge_slicer.toolpath.passplan import curved_subpaths

    # teardrop: tip at 2r on +X, smooth major arc between the TANGENT points
    # (+/-60 deg) — the only corner is the 120-deg tip turn, at vertex 0.
    cx, cy, r = 190.0, 110.0, 10.0
    arc = [(cx + r * math.cos(t), cy + r * math.sin(t))
           for t in [math.radians(60 + 2.4 * i) for i in range(101)]]  # 60..300
    tip = (cx + 2 * r, cy)
    ring = [tip] + arc + [tip]
    cfg = _cfg(seam_position="nearest", seam_prefer_one_pass=True)
    # positions the seat window ON the tip: the tip start absorbs the corner
    # (1 pass); every other windowed start makes it an interior split (2 passes)
    cfg.c_axis.a_min_deg, cfg.c_axis.a_max_deg = -330.0, 110.0

    window = seat_window(ring, cfg.c_axis)
    base = rotate_ring_to_extreme(ring, cfg.c_axis)
    base_subs = curved_subpaths(base, cfg, 120.0)
    assert window[0] == 0 and len(window) > 5, "fixture: tip must lead the window"
    assert len(base_subs) == 1, "fixture: the tip baseline must be one pass"
    # target the far side of the ring: nearest wants a mid-arc start
    ctx = _ctx(cfg, last_xy=(cx - r, cy))
    rot = choose_seam_start(ring, cfg, ctx)
    assert len(curved_subpaths(rot, cfg, 120.0)) == 1, \
        "one_pass accepted a windowed start that splits the ring"


def test_unknown_seam_position_degrades_to_extreme_with_warning():
    cfg = _cfg(mode="contour", seam_position="rear")     # not a valid policy
    plan = plan_toolpath(_disc_layers(1), cfg, operating_point=_op())
    assert any("rear" in w and "extreme" in w for w in plan.warnings)
    # same passes as the extreme default (NOT the random fallthrough)
    cfg2 = _cfg(mode="contour", seam_position="extreme")
    plan2 = plan_toolpath(_disc_layers(1), cfg2, operating_point=_op())
    assert [p.points for ly in plan.layers for p in ly.passes] == \
           [p.points for ly in plan2.layers for p in ly.passes]
    # library callers constructing a bad ctx directly degrade too
    ctx = _ctx(_cfg(seam_position="random"))
    ctx.policy = "rear"
    ring = _circle()
    assert choose_seam_start(ring, _cfg(), ctx) == \
        rotate_ring_to_extreme(ring, CAxisCfg())


def test_ring_less_layer_does_not_wipe_the_seam_chain():
    cfg = _cfg(seam_position="aligned")
    ctx = _ctx(cfg)
    ctx.layer_seams = [((1.0, 2.0), (0, 0, 4, 4))]
    ctx.next_layer()
    assert ctx.prev_seams == [((1.0, 2.0), (0, 0, 4, 4))]
    ctx.next_layer()                               # a layer with no rings
    assert ctx.prev_seams == [((1.0, 2.0), (0, 0, 4, 4))], \
        "a ring-less layer must not reset the aligned chain"
    ctx.layer_seams = [((9.0, 9.0), (8, 8, 10, 10))]
    ctx.next_layer()
    assert ctx.prev_seams == [((9.0, 9.0), (8, 8, 10, 10))]


def test_sub360_range_still_emits_with_policies():
    cfg = _cfg(mode="contour", seam_position="random", seam_prefer_one_pass=False)
    cfg.c_axis.a_min_deg, cfg.c_axis.a_max_deg = -170.0, 170.0
    plan = plan_toolpath(_disc_layers(1), cfg, operating_point=_op())
    assert plan.npasses >= 4                       # arcs + reversals still trace
    g = GCodeEmitter(cfg).emit(ToolpathPlan(plan.layers, 5000, 120.0, 120.0))
    assert "M84" in g
