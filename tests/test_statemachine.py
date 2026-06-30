import pytest

from rotoforge_slicer.toolpath.statemachine import (
    GrindingError, assert_contact_invariant,
)


def test_ok_when_moving_and_feeding():
    assert_contact_invariant(in_contact=True, xy_speed_mm_s=20,
                             v_grind_floor_mm_s=10, e_feeding=True)
    # airborne is always fine
    assert_contact_invariant(in_contact=False, xy_speed_mm_s=0,
                             v_grind_floor_mm_s=10, e_feeding=False)


def test_grinds_when_too_slow_or_not_feeding():
    with pytest.raises(GrindingError):
        assert_contact_invariant(in_contact=True, xy_speed_mm_s=5,
                                 v_grind_floor_mm_s=10, e_feeding=True)
    with pytest.raises(GrindingError):
        assert_contact_invariant(in_contact=True, xy_speed_mm_s=20,
                                 v_grind_floor_mm_s=10, e_feeding=False)
