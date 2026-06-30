import pytest

from rotoforge_slicer.config import Config, CAxisCfg
from rotoforge_slicer.emit.templates import preamble, postamble
from rotoforge_slicer.emit.rrf import validate_heading, validate_monotonic_e
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
