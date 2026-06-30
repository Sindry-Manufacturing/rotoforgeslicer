import math

from rotoforge_slicer.fill.curvature import (
    circumradius,
    min_radius,
    r_min,
    split_on_curvature,
)


def test_r_min_unset_is_inf():
    assert r_min(10.0, 0.0) == math.inf


def test_r_min_value():
    assert math.isclose(r_min(10.0, 100.0), 10.0 / math.radians(100.0))


def test_circumradius_straight_is_inf():
    assert circumradius((0, 0), (1, 0), (2, 0)) == math.inf


def test_circumradius_unit_circle():
    # three points on the unit circle -> radius 1
    assert math.isclose(circumradius((1, 0), (0, 1), (-1, 0)), 1.0, rel_tol=1e-9)


def test_min_radius_of_corner():
    # a 90-degree corner has a finite (small) turn radius
    assert min_radius([(0, 0), (0, 5), (5, 5)]) < 5.0


def test_split_breaks_tight_corner_at_fast_v():
    path = [(0, 0), (0, 5), (5, 5)]            # ~3.5 mm corner radius
    subs = split_on_curvature(path, v_mm_s=100.0, omega_max_deg_s=360.0)  # R_min ~16 mm
    assert len(subs) == 2                       # broken at the corner


def test_split_keeps_corner_at_slow_v():
    path = [(0, 0), (0, 5), (5, 5)]
    subs = split_on_curvature(path, v_mm_s=2.0, omega_max_deg_s=360.0)    # R_min ~0.32 mm
    assert subs == [path]                       # followable at slow speed


def test_split_no_limit_when_omega_zero():
    path = [(0, 0), (0, 5), (5, 5)]
    assert split_on_curvature(path, 100.0, 0.0) == [path]   # no limit -> never split


def test_split_output_satisfies_r_min():
    # a zigzag of tight corners: after splitting, EVERY sub-path holds min_radius >= R_min.
    path = [(0, 0), (0, 3), (3, 3), (3, 6), (6, 6), (6, 9)]
    v, omega = 50.0, 360.0                      # R_min ~ 8 mm; the corners are far tighter
    floor = r_min(v, omega)
    subs = split_on_curvature(path, v, omega)
    assert len(subs) > 1
    for sub in subs:
        assert min_radius(sub) >= floor - 1e-9
