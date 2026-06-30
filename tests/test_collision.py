"""M4 2.5D swept-disc + leading-wire collision check + lead-away ordering. SPEC §4.6."""
import math
from pathlib import Path

import pytest

from rotoforge_slicer.config import Config
from rotoforge_slicer.toolpath.collision import (
    HeightField,
    assert_no_collisions,
    check_pass,
    disc_drop,
    replay_collision_check,
)
from rotoforge_slicer.toolpath.passplan import (
    LayerPlan,
    Pass,
    ToolpathPlan,
    order_passes_lead_away,
)

CFG = Path(__file__).resolve().parents[1] / "config" / "machine_duet3.yaml"


def _p(x, y0, y1, z=0.06, rpm=5000, v=120.0):
    return Pass(start=(x, y0), end=(x, y1), z=z, a_deg=0.0,
               rpm=rpm, traverse_mm_min=v, e_per_path_mm=1.0)


# ----------------------------- disc geometry -----------------------------

def test_disc_drop_profile():
    assert disc_drop(0.0, 25.0) == 0.0
    assert math.isclose(disc_drop(5.0, 25.0), 25.0 - math.sqrt(600.0), rel_tol=1e-9)
    assert math.isclose(disc_drop(5.0, 25.0), 0.5051, abs_tol=1e-3)  # ~0.5 mm (SPEC §4.6)
    assert disc_drop(30.0, 25.0) == math.inf                          # beyond the disc


# ----------------------------- height field -----------------------------

def test_height_field_deposit_and_query():
    f = HeightField(0, 0, 30, 30, 0.5)
    assert f.height_at(10, 6) == 0.0                 # bare bed
    f.deposit_segment((10, 2), (10, 12), 0.5, 0.06)
    assert math.isclose(f.height_at(10, 6), 0.06)    # on the bead
    assert f.height_at(20, 6) == 0.0                 # off the bead
    assert f.height_at(100, 100) == 0.0              # outside the window


def test_deposit_keeps_max_height():
    f = HeightField(0, 0, 30, 30, 0.5)
    f.deposit_segment((10, 2), (10, 12), 0.5, 0.50)
    f.deposit_segment((10, 2), (10, 12), 0.5, 0.06)  # a lower later pass
    assert math.isclose(f.height_at(10, 6), 0.50)    # height is the max, never lowered


# ----------------------------- collision detection -----------------------------
# Only material risen ABOVE the current layer is an obstruction; the deposit Z here
# is 0.18 so a 5 mm pre-existing wall is a genuine step-up.

def test_leading_wire_into_step_up_ahead_is_flagged():
    f = HeightField(0, 0, 40, 40, 0.5)
    f.deposit_segment((10, 17), (10, 28), 0.5, 5.0)         # 5 mm wall just ahead
    c = check_pass(f, _p(10, 2, 15.5, z=0.18), Config())    # wire reaches ~17.5 -> into it
    assert c is not None and c.kind == "wire"


def test_disc_body_into_far_step_up_is_flagged():
    # a wall beyond wire_lead (2mm) but within the disc radius (25mm) is caught by the DISC.
    f = HeightField(0, 0, 40, 40, 0.5)
    f.deposit_segment((10, 33), (10, 38), 0.5, 5.0)
    c = check_pass(f, _p(10, 2, 30, z=0.18), Config())
    assert c is not None and c.kind == "disc"


def test_plunge_into_wall_at_start_is_flagged():
    # SPEC §4.6/§4.4: the moving plunge descends at the START; a wall there is a crash.
    f = HeightField(0, 0, 40, 40, 0.5)
    f.deposit_segment((10, 2), (10, 6), 0.5, 5.0)           # wall over the pass START
    assert check_pass(f, _p(10, 2, 30, z=0.18), Config()) is not None


def test_wall_at_mid_pass_is_flagged():
    # the swept move is checked along its whole length, not just the end.
    f = HeightField(0, 0, 40, 40, 0.5)
    f.deposit_segment((10, 12), (10, 16), 0.5, 5.0)         # wall across the MIDDLE
    assert check_pass(f, _p(10, 2, 30, z=0.18), Config()) is not None


def test_same_layer_material_ahead_not_flagged():
    # the intended adjacent-bead OVERLAP (and coarse-cell bleed) sits at the current
    # layer Z and must NOT read as a collision.
    f = HeightField(0, 0, 40, 40, 0.5)
    f.deposit_segment((10, 17), (10, 28), 0.5, 0.18)        # same layer, not a step-up
    assert check_pass(f, _p(10, 2, 15.5, z=0.18), Config()) is None


def test_free_approach_is_clear():
    f = HeightField(0, 0, 30, 30, 0.5)
    assert check_pass(f, _p(10, 2, 12), Config()) is None   # nothing ahead


def test_adjacent_raster_passes_no_false_collision():
    # two parallel +Y passes one pitch (0.85) apart; depositing the first must not make
    # the second's approach read as a collision (the beads only graze-overlap laterally).
    plan = ToolpathPlan(layers=[LayerPlan(0, 0.06, [_p(10.0, 0, 12), _p(10.85, 0, 12)])],
                        rpm=5000, traverse_mm_min=120.0, v_grind_floor_mm_min=120.0)
    assert replay_collision_check(plan, Config()) == []


# ----------------------------- lead-away ordering -----------------------------

def test_order_lead_away_deposits_least_forward_first():
    a = _p(10, 2, 9)     # -Y region (ends at y=9)
    b = _p(10, 10, 18)   # +Y region (ends at y=18)
    ordered = order_passes_lead_away([b, a])
    assert [p.end[1] for p in ordered] == [9, 18]   # -Y first


def test_order_lead_away_convex_falls_back_to_perp():
    # a box: every +Y line ends at the same Y -> tie -> deterministic left-to-right.
    passes = [_p(3, 0, 12), _p(1, 0, 12), _p(2, 0, 12)]
    ordered = order_passes_lead_away(passes)
    assert [p.start[0] for p in ordered] == [1, 2, 3]


# ----------------------------- replay + pipeline gating -----------------------------

def _colliding_plan():
    # a tall bead is deposited first, then a low pass leads its wire straight into it.
    tall = Pass(start=(10, 18), end=(10, 28), z=5.0, a_deg=0.0,
                rpm=5000, traverse_mm_min=120.0, e_per_path_mm=1.0)
    low = Pass(start=(10, 2), end=(10, 15.5), z=0.06, a_deg=0.0,
               rpm=5000, traverse_mm_min=120.0, e_per_path_mm=1.0)
    return ToolpathPlan([LayerPlan(0, 0.06, [tall, low])], 5000, 120.0, 120.0)


def test_replay_detects_lead_into_tall_material():
    assert len(replay_collision_check(_colliding_plan(), Config())) >= 1


def test_assert_no_collisions_raises_and_escape_hatch():
    cfg = Config()
    with pytest.raises(ValueError):
        assert_no_collisions(_colliding_plan(), cfg)          # enabled -> build failure
    cfg.collision.enabled = False
    assert assert_no_collisions(_colliding_plan(), cfg) == []  # escape hatch -> no raise


# ----------------------------- end-to-end -----------------------------

def test_real_part_is_collision_free_and_pipeline_passes(tmp_path):
    trimesh = pytest.importorskip("trimesh")
    pytest.importorskip("shapely")
    pytest.importorskip("scipy")
    from rotoforge_slicer.pipeline import slice_geometry, slice_mesh
    from rotoforge_slicer.toolpath.passplan import plan_toolpath

    cfg = Config()
    ann = trimesh.creation.annulus(r_min=6.0, r_max=18.0, height=2.0, sections=64)
    stl = tmp_path / "ann.stl"
    ann.export(stl)
    model = slice_geometry(str(stl), cfg, place=True)
    plan = plan_toolpath(model, cfg)
    assert replay_collision_check(plan, cfg) == []     # multi-region/holed -> still clear

    # the full pipeline runs the check and does not raise on a clean part
    g = slice_mesh(str(stl), str(CFG), None, str(tmp_path / "ann.gcode"))
    assert "M84" in g
