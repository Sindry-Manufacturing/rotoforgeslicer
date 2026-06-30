"""2.5D swept-disc + leading-wire collision / approach check. SPEC §4.6.  [stub — M4]

Maintain a height-field of deposited material; for each move sample the disc's
fore/aft lowest-point line plus the leading-wire point; flag violations; resolve
by reordering passes, increasing lift, or inserting an airborne reorient.
"""
from __future__ import annotations


class HeightMap:
    def __init__(self, *args, **kwargs):
        raise NotImplementedError("HeightMap: implement per SPEC §4.6")
