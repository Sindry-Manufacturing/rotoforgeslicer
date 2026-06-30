"""M3 process window: per-pass airborne RPM placement + constant-revs/mm. SPEC §4.5/§5.

The RPM-placement and validation tests build plans directly, so they need no heavy
deps; the end-to-end screener test pulls trimesh and is skipped where absent.
"""
import re
from pathlib import Path

import pytest

from rotoforge_slicer.config import Config
from rotoforge_slicer.emit.rrf import GCodeEmitter
from rotoforge_slicer.toolpath.passplan import LayerPlan, Pass, ToolpathPlan

CFG = Path(__file__).resolve().parents[1] / "config" / "machine_duet3.yaml"
FIXTURE = Path(__file__).resolve().parent / "fixtures" / "screener_sample.csv"


def _plan(rpm_b, v_b):
    """Two passes on the revs/mm=100 ray: A=(v100,rpm10000), B=(v_b,rpm_b)."""
    a = Pass(start=(190.0, 100.0), end=(190.0, 120.0), z=0.06, a_deg=0.0,
             rpm=10000, traverse_mm_min=100.0, e_per_path_mm=1.0)
    b = Pass(start=(191.0, 100.0), end=(191.0, 120.0), z=0.06, a_deg=0.0,
             rpm=rpm_b, traverse_mm_min=v_b, e_per_path_mm=1.0)
    return ToolpathPlan(layers=[LayerPlan(0, 0.06, [a, b])], rpm=10000,
                        traverse_mm_min=100.0, v_grind_floor_mm_min=100.0)


def test_rpm_hop_emitted_airborne_between_passes():
    # B at rpm 12000, v 120 -> 12000/120 = 100, same ray, different RPM.
    g = GCodeEmitter(Config()).emit(_plan(12000, 120.0))
    lines = g.splitlines()
    m3 = [l for l in lines if l.startswith("M3 S")]
    assert [m.split()[0:2] for m in m3] == [["M3", "S10000"], ["M3", "S12000"]]
    assert sum("startup settle" in l for l in lines) == 1      # long settle, first spin-up
    assert sum("spindle stabilize" in l for l in lines) == 1   # short dwell on the hop
    # the RPM hop is airborne: each M3 is preceded by a G0 (no G1 deposition between).
    for i, l in enumerate(lines):
        if l.startswith("M3 S"):
            assert any(lines[j].startswith("G0") for j in range(max(0, i - 3), i))


def test_constant_rpm_emits_single_m3():
    g = GCodeEmitter(Config()).emit(_plan(10000, 100.0))  # identical operating point
    assert sum(l.startswith("M3 S") for l in g.splitlines()) == 1   # set once, not per pass


def test_off_ray_pass_rejected():
    # B at rpm 15000, v 120 -> 125 revs/mm != the 100 ray -> must fail (acceptance 2).
    with pytest.raises(ValueError):
        GCodeEmitter(Config()).emit(_plan(15000, 120.0))


def test_validate_constant_revs_per_mm_is_falsifiable():
    em = GCodeEmitter(Config())
    em._validate_constant_revs_per_mm(_plan(12000, 120.0))      # on-ray: ok
    with pytest.raises(ValueError):
        em._validate_constant_revs_per_mm(_plan(15000, 120.0))  # off-ray: raises


def test_emit_rejects_rpm_outside_superpid():
    # 45000/300 = 150 holds the ray (constant-revs/mm passes) but exceeds the SuperPID
    # window [5000,30000] -> the §6.3 spindle validator must reject it (SPEC §1.3).
    p = Pass(start=(190.0, 100.0), end=(190.0, 120.0), z=0.06, a_deg=0.0,
             rpm=45000, traverse_mm_min=300.0, e_per_path_mm=1.0)
    plan = ToolpathPlan(layers=[LayerPlan(0, 0.06, [p])], rpm=45000,
                        traverse_mm_min=300.0, v_grind_floor_mm_min=300.0)
    with pytest.raises(ValueError):
        GCodeEmitter(Config()).emit(plan)


def test_end_to_end_screener_sets_rpm_and_feed(tmp_path):
    trimesh = pytest.importorskip("trimesh")
    pytest.importorskip("shapely")
    pytest.importorskip("scipy")
    from rotoforge_slicer.pipeline import slice_mesh

    stl = tmp_path / "box.stl"
    trimesh.creation.box(extents=(20, 12, 2)).export(stl)
    g = slice_mesh(str(stl), str(CFG), str(FIXTURE), str(tmp_path / "box.gcode"))

    assert "M3 S15000" in g            # screener-selected RPM (nv=150 ray, rep v=100)
    assert "revs_per_mm=150" in g      # header reflects the selected ray
    assert "traverse=100mm/min" in g
    es = [float(m) for l in g.splitlines() if l.startswith("G1")
          for m in re.findall(r" E(-?\d+\.?\d*)", l)]
    assert es and all(d >= 0 for d in es)   # screener E coupling, still monotonic
