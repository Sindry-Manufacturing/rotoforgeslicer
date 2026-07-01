import pytest

from rotoforge_slicer.config import Config, CAxisCfg
from rotoforge_slicer.emit.templates import preamble, postamble
from rotoforge_slicer.emit.rrf import (
    GCodeEmitter, validate_axis_angle, validate_monotonic_e,
)


def test_preamble_postamble_shape():
    cfg = Config()
    pre = preamble(cfg)
    post = postamble(cfg)
    assert pre[0].startswith(";") and "G21" in pre and "G90" in pre
    assert "M83" in pre                      # relative E by default
    assert any("M190" in ln for ln in pre)   # waits for bed
    assert any('M98 P"Hotshoe_300C.g"' == ln for ln in pre)
    assert "M5  ; spindle off" in post and "M84" in post


def test_validate_axis_angle():
    # D13: no wedge — the only hard heading limit is the usable axis range. +X (A=-90)
    # and -Y (A=±180) are perfectly fine; only past the stops fails.
    c = CAxisCfg(a_min_deg=-180.0, a_max_deg=180.0)
    validate_axis_angle(0.0, c)        # +Y home
    validate_axis_angle(-90.0, c)      # +X
    validate_axis_angle(180.0, c)      # -Y boundary, reachable airborne
    with pytest.raises(ValueError):
        validate_axis_angle(200.0, c)  # past the +stop
    with pytest.raises(ValueError):
        validate_axis_angle(-181.0, c)  # past the -stop


def test_validate_a_in_axis_range():
    c = CAxisCfg(a_min_deg=-180.0, a_max_deg=180.0)
    # every commanded A (deposition + airborne) inside the range passes
    GCodeEmitter._validate_a_in_axis_range([-180.0, -90.0, 0.0, 90.0, 180.0], c)
    with pytest.raises(ValueError):
        GCodeEmitter._validate_a_in_axis_range([200.0], c)
    with pytest.raises(ValueError):
        GCodeEmitter._validate_a_in_axis_range([-181.0], c)


def test_validate_monotonic_e():
    validate_monotonic_e([0.0, 0.1, 0.1, 0.5])
    with pytest.raises(ValueError):
        validate_monotonic_e([0.0, 0.5, 0.4])
