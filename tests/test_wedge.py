"""Heading <-> A-axis mapping, axis-range + winding helpers (fill/wedge.py, D13).

There is no deposition wedge anymore (D13): the head rotates as a unit, every heading
deposits, and A is always commanded equal to the travel heading. The only heading limit
is the usable continuous axis range; curves are bounded by slew + winding.
"""
import math

import pytest

from rotoforge_slicer.config import CAxisCfg
from rotoforge_slicer.fill.wedge import (
    heading_deg_from_vector, heading_to_a_deg, unwrap_headings, winding_shift,
    within_axis_range,
)

cfg = CAxisCfg()  # home +Y (90), offset 0, invert +1, range [-180, 180]


def test_plus_y_is_zero():
    assert abs(heading_to_a_deg(90, cfg)) < 1e-9


def test_heading_from_vector():
    assert heading_deg_from_vector(0, 1) == pytest.approx(90)    # +Y
    assert heading_deg_from_vector(1, 0) == pytest.approx(0)     # +X


def test_no_privileged_direction_every_heading_reaches_an_in_range_winding():
    # D13: no forbidden direction. -Y included; some winding of each heading is in range.
    for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, -1)]:
        a = heading_to_a_deg(heading_deg_from_vector(dx, dy), cfg)
        a_in = a + winding_shift(a, a, cfg.a_min_deg, cfg.a_max_deg)
        assert within_axis_range(a_in, cfg)


def test_within_axis_range_is_the_only_hard_limit():
    c = CAxisCfg(a_min_deg=-180.0, a_max_deg=180.0)
    assert within_axis_range(-180.0, c) and within_axis_range(180.0, c)
    assert within_axis_range(0.0, c)
    assert not within_axis_range(200.0, c)
    assert not within_axis_range(-181.0, c)


def test_unwrap_headings_removes_branch_cut_jumps():
    # a heading sweeping through the +180/-180 atan2 cut stays continuous (no 360 jump)
    cont = unwrap_headings([170.0, 178.0, -174.0, -160.0])
    for a, b in zip(cont, cont[1:]):
        assert abs(b - a) < 90.0
    assert cont[-1] == pytest.approx(200.0)   # 170 -> 178 -> 186 -> 200, continuous


def test_winding_shift_seats_band_in_range():
    # a continuous A-band [200, 260] (past the top stop) winds down by -360 into [-160,-100]
    shift = winding_shift(200.0, 260.0, -180.0, 180.0)
    assert shift == pytest.approx(-360.0)
    assert within_axis_range(200.0 + shift, cfg)
    assert within_axis_range(260.0 + shift, cfg)


def test_commanded_drift_is_zero_along_a_curve():
    # D13 per-instant rule: the wheel heading recovered from the commanded A equals the
    # travel heading at every segment -> commanded drift is structurally 0.
    pts = [(math.cos(t), math.sin(t)) for t in (0.0, 0.3, 0.6, 0.9, 1.2, 1.5)]
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        theta = heading_deg_from_vector(bx - ax, by - ay)
        a = heading_to_a_deg(theta, cfg)
        theta_wheel = cfg.home_heading_deg + (a - cfg.home_offset_deg) / cfg.invert_sign
        assert abs(theta_wheel - theta) < 1e-9
