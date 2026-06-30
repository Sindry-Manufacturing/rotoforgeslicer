"""M2 RRF emitter: SPEC §6.1 structure + §6.3 validations, end-to-end."""
import re
from pathlib import Path

import pytest

pytest.importorskip("shapely")
trimesh = pytest.importorskip("trimesh")
pytest.importorskip("scipy")

from rotoforge_slicer.config import load_config  # noqa: E402
from rotoforge_slicer.emit.rrf import GCodeEmitter  # noqa: E402
from rotoforge_slicer.pipeline import slice_geometry, slice_mesh  # noqa: E402
from rotoforge_slicer.toolpath.passplan import plan_toolpath  # noqa: E402

CFG = Path(__file__).resolve().parents[1] / "config" / "machine_duet3.yaml"


def _box_plan(tmp_path, cfg, *, place=True):
    stl = tmp_path / "box.stl"
    trimesh.creation.box(extents=(20, 12, 2)).export(stl)
    model = slice_geometry(str(stl), cfg, place=place)
    return plan_toolpath(model, cfg)


def test_emit_structure_and_monotonic_e(tmp_path):
    cfg = load_config(CFG)
    plan = _box_plan(tmp_path, cfg)
    g = GCodeEmitter(cfg).emit(plan)
    lines = g.splitlines()

    assert lines[0].startswith(";")
    assert any(l.startswith("G28") for l in lines)              # homing
    assert any(l.startswith("G92 E0") for l in lines)
    assert any(l.startswith(f"M3 S{plan.rpm}") for l in lines)   # spindle airborne
    assert any(l.startswith("G4 P") for l in lines)              # airborne startup settle
    assert g.strip().endswith("M84")

    # Relative E (M83): every E word is a per-segment delta and must be >= 0
    # (monotonic cumulative E, never a retraction). G92 E0 is the reset, not a move.
    es = [float(m) for l in lines if l.startswith("G1")
          for m in re.findall(r" E(-?\d+\.?\d*)", l)]
    assert es and all(d >= 0 for d in es)

    # every deposition heading is in the +/- wedge.
    avals = [float(m) for l in lines for m in re.findall(r" A(-?\d+\.?\d*)", l)]
    assert avals and max(abs(a) for a in avals) <= cfg.c_axis.wedge_half_angle_deg


def test_emit_dry_run_disables_spindle_and_extrusion(tmp_path):
    cfg = load_config(CFG)
    cfg.emit.dry_run = True
    plan = _box_plan(tmp_path, cfg)
    g = GCodeEmitter(cfg).emit(plan)
    lines = g.splitlines()
    assert "DRY RUN" in g
    assert not any(l.startswith("M3 S") for l in lines)          # no spindle
    assert not any(l.startswith("G1") and " E" in l for l in lines)  # no extrusion on moves


def test_emit_rejects_out_of_build_volume(tmp_path):
    cfg = load_config(CFG)
    plan = _box_plan(tmp_path, cfg, place=False)  # centred on origin -> negative coords
    with pytest.raises(ValueError):
        GCodeEmitter(cfg).emit(plan)


def test_emit_raises_grinding_when_below_floor(tmp_path):
    """A deposition speed below the grind floor must be rejected (SPEC §4.4)."""
    from rotoforge_slicer.toolpath.statemachine import GrindingError

    cfg = load_config(CFG)
    plan = _box_plan(tmp_path, cfg)
    plan.v_grind_floor_mm_min = plan.traverse_mm_min * 2.0  # floor now above traverse
    with pytest.raises(GrindingError):
        GCodeEmitter(cfg).emit(plan)


def test_emit_raises_grinding_when_not_feeding(tmp_path):
    """In-contact but feeding no wire is subtractive — must be rejected (SPEC §2.3)."""
    from rotoforge_slicer.toolpath.statemachine import GrindingError

    cfg = load_config(CFG)
    plan = _box_plan(tmp_path, cfg)
    plan.nonempty_layers[0].passes[0].e_per_path_mm = 0.0   # spinning in contact, no E
    with pytest.raises(GrindingError):
        GCodeEmitter(cfg).emit(plan)


def test_no_dwell_in_contact_validator_is_falsifiable(tmp_path):
    cfg = load_config(CFG)
    plan = _box_plan(tmp_path, cfg)
    em = GCodeEmitter(cfg)
    dep_z = plan.nonempty_layers[0].z
    with pytest.raises(ValueError):
        em._validate_no_dwell_in_contact([dep_z], plan)        # dwell at deposition Z
    em._validate_no_dwell_in_contact(                          # airborne dwell is fine
        [dep_z + cfg.process.inter_pass_lift_mm], plan)


def test_emit_holed_part_keeps_e_monotonic_and_in_wedge(tmp_path):
    cfg = load_config(CFG)
    ann = trimesh.creation.annulus(r_min=6.0, r_max=18.0, height=3.0, sections=64)
    stl = tmp_path / "ann.stl"
    ann.export(stl)
    g = slice_mesh(str(stl), str(CFG), None, str(tmp_path / "ann.gcode"))
    lines = g.splitlines()
    es = [float(m) for l in lines if l.startswith("G1")
          for m in re.findall(r" E(-?\d+\.?\d*)", l)]
    assert es and all(d >= 0 for d in es)                      # holed layers, still monotonic
    avals = [float(m) for l in lines for m in re.findall(r" A(-?\d+\.?\d*)", l)]
    assert avals and max(abs(a) for a in avals) <= cfg.c_axis.wedge_half_angle_deg


def test_slice_mesh_end_to_end_writes_file(tmp_path):
    stl = tmp_path / "box.stl"
    trimesh.creation.box(extents=(20, 12, 2)).export(stl)
    out = tmp_path / "box.gcode"
    g = slice_mesh(str(stl), str(CFG), None, str(out))
    assert out.exists()
    assert out.read_text(encoding="utf-8") == g
    assert "M84" in g and g.count("\n") > 100
