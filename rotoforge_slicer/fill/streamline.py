"""Curved fill via +Y-biased streamlines, clipped to region. SPEC §4.2.  [stub — M5]

Integrate a +Y-biased guidance field, trace streamlines, clip to the region with
pyclipr, then enforce wedge + monotonic-forward + min-length + curvature limits;
split where violated.
"""
from __future__ import annotations


def streamline_fill(region, cfg):
    raise NotImplementedError("streamline_fill: implement per SPEC §4.2/§4.3")
