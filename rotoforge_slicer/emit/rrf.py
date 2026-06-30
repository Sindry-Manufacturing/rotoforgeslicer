"""RRF G-code emitter + hard validators. SPEC §6.

Validators are implemented (and tested); the emitter body is built in M2-M4.
"""
from __future__ import annotations

from typing import Iterable

from ..config import CAxisCfg, Config
from ..fill.wedge import in_wedge
from ..toolpath.statemachine import assert_contact_invariant  # noqa: F401 (re-exported)


def validate_heading(a_deg: float, c_axis: CAxisCfg) -> None:
    """SPEC §6.3: every deposition heading within the +/- wedge."""
    if not in_wedge(a_deg, c_axis):
        raise ValueError(
            f"heading A={a_deg:.2f} deg outside +/-{c_axis.wedge_half_angle_deg} deg wedge")


def validate_monotonic_e(e_values: Iterable[float], tol: float = 1e-9) -> None:
    """SPEC §6.3: E never decreases across the file."""
    prev = None
    for e in e_values:
        if prev is not None and e < prev - tol:
            raise ValueError(f"E not monotonic: {e} < {prev}")
        prev = e


class GCodeEmitter:
    """Emit RRF G-code from a planned toolpath. SPEC §6.  [body: M2-M4]"""

    def __init__(self, cfg: Config):
        self.cfg = cfg

    def emit(self, plan) -> str:
        raise NotImplementedError("GCodeEmitter.emit: implement per SPEC §6")
