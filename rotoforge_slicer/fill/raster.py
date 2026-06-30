"""Unidirectional +Y raster fill within the wedge. SPEC §4.2.  [stub — M2]

No bidirectional/boustrophedon (-Y deposition is impossible): every pass goes
+Y, then lift and travel (wheel up) back to the next line's start.
"""
from __future__ import annotations


def raster_fill(region, cfg):
    raise NotImplementedError("raster_fill: implement per SPEC §4.2")
