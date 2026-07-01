"""M5 curved-pass emission: per-segment A + the §6.3 curvature limit. SPEC §4.3/§6.2."""
import math
import re

import pytest

from rotoforge_slicer.config import Config
from rotoforge_slicer.emit.rrf import GCodeEmitter
from rotoforge_slicer.fill.curvature import r_min
from rotoforge_slicer.toolpath.passplan import LayerPlan, Pass, ToolpathPlan


def _cfg():
    cfg = Config()
    cfg.c_axis.max_speed_deg_s = 360.0   # the calibrated machine value
    return cfg


def _curved_plan(cfg, points, v=120.0, rpm=5000):
    p = Pass.curved(points, z=0.06, rpm=rpm, traverse_mm_min=v,
                    e_per_path_mm=1.0, c_axis=cfg.c_axis)
    return ToolpathPlan([LayerPlan(0, 0.06, [p])], rpm, v, v)


def test_curved_pass_emits_per_segment_a_in_axis_range():
    # a gentle curve: headings (and so A) sweep per segment, all within the axis range.
    cfg = _cfg()
    pts = [(190, 100), (190, 106), (191.5, 112), (194, 117)]
    g = GCodeEmitter(cfg).emit(_curved_plan(cfg, pts))
    a_vals = [float(m) for l in g.splitlines() if l.startswith("G1")
              for m in re.findall(r" A(-?\d+\.?\d*)", l)]
    assert len(set(a_vals)) > 1                      # A really does change per segment
    assert all(cfg.c_axis.a_min_deg - 1e-6 <= a <= cfg.c_axis.a_max_deg + 1e-6
               for a in a_vals)


def test_emit_rejects_turn_tighter_than_curvature_limit():
    # the turn radius is below R_min at a fast traverse -> the §6.3 curvature validator
    # must reject it (SPEC §4.3), independent of the (now wedge-free) heading limits.
    cfg = _cfg()
    pts = [(190, 100), (190, 103), (191.93, 105.30)]   # ~40 deg turn, ~4 mm radius
    p0 = Pass.curved(pts, z=0.06, rpm=20000, traverse_mm_min=120.0,
                     e_per_path_mm=1.0, c_axis=cfg.c_axis)
    omega = cfg.c_axis.max_speed_deg_s * math.pi / 180.0
    v_mm_min = (p0.min_radius_mm * omega + 1.0) * 60.0   # forces R_min > pass radius
    with pytest.raises(ValueError):
        GCodeEmitter(cfg).emit(_curved_plan(cfg, pts, v=v_mm_min, rpm=20000))


def test_gentle_curve_within_limit_emits_ok():
    cfg = _cfg()
    pts = [(190, 100), (190, 106), (191.5, 112), (194, 117)]
    plan = _curved_plan(cfg, pts, v=120.0)
    assert plan.layers[0].passes[0].min_radius_mm >= r_min(
        120.0 / 60.0, cfg.c_axis.max_speed_deg_s)
    g = GCodeEmitter(cfg).emit(plan)
    assert "M84" in g


def test_tight_curve_emits_when_no_curvature_limit_set():
    # the default config has max_speed_deg_s=0 -> R_min=inf -> NO limit; a curved pass
    # (even a tight one) must emit, not be rejected. Pins the inf-guard (rrf.py).
    cfg = Config()
    assert cfg.c_axis.max_speed_deg_s == 0.0          # the uncalibrated default
    pts = [(190, 100), (190, 103), (191.93, 105.30)]  # ~4 mm radius (tight)
    p = Pass.curved(pts, z=0.06, rpm=cfg.spindle.rpm_min, traverse_mm_min=120.0,
                    e_per_path_mm=1.0, c_axis=cfg.c_axis)
    plan = ToolpathPlan([LayerPlan(0, 0.06, [p])], cfg.spindle.rpm_min, 120.0, 120.0)
    g = GCodeEmitter(cfg).emit(plan)                  # must NOT raise
    assert "M84" in g
