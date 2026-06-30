import pytest

from rotoforge_slicer.config import Config, CAxisCfg
from rotoforge_slicer.emit.templates import preamble, postamble
from rotoforge_slicer.emit.rrf import (
    GCodeEmitter, validate_heading, validate_monotonic_e,
)
from rotoforge_slicer.fill.wedge import heading_to_a_deg


def test_preamble_postamble_shape():
    cfg = Config()
    pre = preamble(cfg)
    post = postamble(cfg)
    assert pre[0].startswith(";") and "G21" in pre and "G90" in pre
    assert "M83" in pre                      # relative E by default
    assert any("M190" in ln for ln in pre)   # waits for bed
    assert any('M98 P"Hotshoe_300C.g"' == ln for ln in pre)
    assert "M5  ; spindle off" in post and "M84" in post


def test_validate_heading():
    c = CAxisCfg()
    validate_heading(heading_to_a_deg(90, c), c)     # +Y ok
    with pytest.raises(ValueError):
        validate_heading(heading_to_a_deg(0, c), c)   # +X -> A=-90, outside wedge


def test_validate_monotonic_e():
    validate_monotonic_e([0.0, 0.1, 0.1, 0.5])
    with pytest.raises(ValueError):
        validate_monotonic_e([0.0, 0.5, 0.4])


def test_mechanical_range_distinct_from_deposition_wedge():
    """D12: the ±180° mechanical travel limit is a SEPARATE, wider bound than the
    ±90° deposition wedge. A reorientation heading can be valid mechanically yet
    forbidden for deposition; only −Y (±180°) trips the mechanical stop."""
    c = CAxisCfg(wedge_half_angle_deg=90.0, a_min_deg=-180.0, a_max_deg=180.0)

    # in-range A targets (incl. the ±180° boundary = -Y, reachable airborne) pass
    GCodeEmitter._validate_a_in_mechanical_range([-180.0, -90.0, 0.0, 90.0, 180.0], c)

    # past the mechanical stop -> hard fail
    with pytest.raises(ValueError):
        GCodeEmitter._validate_a_in_mechanical_range([200.0], c)
    with pytest.raises(ValueError):
        GCodeEmitter._validate_a_in_mechanical_range([-181.0], c)

    # A=120° is OUTSIDE the deposition wedge (no -Y bead) but INSIDE the mechanical
    # range: the two checks must disagree here, proving they are distinct limits.
    with pytest.raises(ValueError):
        validate_heading(120.0, c)
    GCodeEmitter._validate_a_in_mechanical_range([120.0], c)  # does not raise
