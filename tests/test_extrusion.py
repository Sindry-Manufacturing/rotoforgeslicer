import math

from rotoforge_slicer.process.extrusion import e_per_path_mm, wire_area_mm2


def test_x_mode():
    assert e_per_path_mm("x", x_ratio=1.0) == 1.0


def test_volume_mode():
    v = e_per_path_mm("volume", bead_width_mm=1.0, layer_height_mm=0.12, wire_diameter_mm=0.5)
    assert math.isclose(v, 1.0 * 0.12 / wire_area_mm2(0.5))


def test_screener_mode():
    s = e_per_path_mm("screener", feed_speed_mm_min=149.5, traverse_mm_min=57.0)
    assert math.isclose(s, 149.5 / 57.0)
