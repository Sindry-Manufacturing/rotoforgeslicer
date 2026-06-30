"""Contact state machine + the grinding invariant. SPEC §4.4.

A spinning wheel in contact while not moving (or moving below the grind floor) or
not feeding wire is SUBTRACTIVE. The only legal in-contact condition is:
    in_contact  <=>  (xy_speed >= v_grind_floor)  AND  (E feeding)
All dwells must happen AIRBORNE.
"""
from __future__ import annotations

from enum import Enum


class ContactState(Enum):
    AIRBORNE = "airborne"
    TRANSITION_IN = "transition_in"    # moving plunge
    DEPOSITING = "depositing"
    TRANSITION_OUT = "transition_out"  # lead-out + moving lift + wire cut


class GrindingError(RuntimeError):
    """Raised when an in-contact move would grind material away."""


def assert_contact_invariant(*, in_contact: bool, xy_speed_mm_s: float,
                             v_grind_floor_mm_s: float, e_feeding: bool) -> None:
    if in_contact and not (xy_speed_mm_s >= v_grind_floor_mm_s and e_feeding):
        raise GrindingError(
            f"GRINDING: in-contact move with xy_speed={xy_speed_mm_s} "
            f"(floor={v_grind_floor_mm_s}), e_feeding={e_feeding}"
        )
