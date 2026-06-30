import math

from rotoforge_slicer.config import CAxisCfg
from rotoforge_slicer.fill.wedge import (
    heading_deg_from_vector, heading_to_a_deg, in_wedge, vector_in_wedge,
)

cfg = CAxisCfg()  # home +Y (90), offset 0, invert +1, wedge 45


def test_plus_y_is_zero():
    assert abs(heading_to_a_deg(90, cfg)) < 1e-9


def test_wedge_edges_inclusive():
    assert in_wedge(heading_to_a_deg(45, cfg), cfg)    # +Y-45
    assert in_wedge(heading_to_a_deg(135, cfg), cfg)   # +Y+45
    assert not in_wedge(heading_to_a_deg(44, cfg), cfg)
    assert not in_wedge(heading_to_a_deg(136, cfg), cfg)


def test_vectors():
    assert abs(heading_deg_from_vector(0, 1) - 90) < 1e-9
    assert vector_in_wedge(0, 1, cfg)        # +Y depositable
    assert not vector_in_wedge(0, -1, cfg)   # -Y impossible
    assert not vector_in_wedge(1, 0, cfg)    # +X outside wedge


# The machine config widens the wedge to 90 (GUI-adjustable 0..180). At exactly
# 90 the depositable set is the closed +Y half-plane: every dy >= 0 heading is in
# (sideways +/-X land on the boundary), every dy < 0 heading is out. -Y stays
# impossible -- the wedge must never wrap around to admit a rearward heading.
w90 = CAxisCfg(wedge_half_angle_deg=90.0)


def test_wedge_90_is_the_plus_y_half_plane():
    assert vector_in_wedge(0, 1, w90)        # +Y
    assert vector_in_wedge(1, 0, w90)        # +X boundary now depositable
    assert vector_in_wedge(-1, 0, w90)       # -X boundary now depositable
    assert vector_in_wedge(1, 1, w90)        # +X+Y diagonal
    assert vector_in_wedge(-1, 1, w90)       # -X+Y diagonal


def test_wedge_90_still_forbids_rearward():
    assert not vector_in_wedge(0, -1, w90)   # -Y impossible
    assert not vector_in_wedge(1, -1, w90)   # down-right (dy<0) rejected
    assert not vector_in_wedge(-1, -1, w90)  # down-left  (dy<0) rejected
