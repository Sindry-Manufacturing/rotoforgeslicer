"""Slice helpers: layer Z heights, region cleanup. SPEC §3.1.  [stub — M1]"""
from __future__ import annotations


def layer_heights(z_min: float, z_max: float, layer_height: float) -> list[float]:
    """Z heights for planar layers (first layer at z_min + layer_height/2)."""
    hs = []
    z = z_min + layer_height / 2.0
    while z < z_max:
        hs.append(round(z, 6))
        z += layer_height
    return hs
