import math

from rotoforge_slicer.fill.curvature import r_min


def test_r_min_unset_is_inf():
    assert r_min(10.0, 0.0) == math.inf


def test_r_min_value():
    assert math.isclose(r_min(10.0, 100.0), 10.0 / math.radians(100.0))
