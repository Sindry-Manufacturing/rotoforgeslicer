from pathlib import Path

from rotoforge_slicer.config import load_config

CFG = Path(__file__).resolve().parents[1] / "config" / "machine_duet3.yaml"


def test_loads_default_config():
    cfg = load_config(CFG)
    assert cfg.machine.rotary_axis_letter == "A"
    assert cfg.machine.steps.e_per_mm == 46.73
    assert cfg.machine.steps.a_per_deg == 26.667
    assert tuple(cfg.machine.build_volume_mm) == (380, 235, 250)
    # D13: no wedge. The C-axis limits are the usable continuous range + the slew rate.
    assert not hasattr(cfg.c_axis, "wedge_half_angle_deg")
    assert cfg.c_axis.a_min_deg == -180 and cfg.c_axis.a_max_deg == 180
    assert cfg.c_axis.max_drift_deg == 0.0
    assert cfg.spindle.rpm_min == 5000 and cfg.spindle.rpm_max == 30000
    assert cfg.process.bead_width_mm == 1.0
    assert cfg.extrusion.mode == "screener"
