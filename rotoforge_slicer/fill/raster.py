"""Raster fill. SPEC §4.2 (D13).

The head rotates as a unit, so there is no privileged direction (D13). Raster is
**bidirectional** by default: adjacent lines alternate heading 180° (a boustrophedon),
so the wheel just turns 180° airborne at the end of each line instead of flying all the
way back — every line deposits, none is a wasted return. Set ``bidirectional=False``
for the legacy one-way sweep. The default heading is +Y (90 deg CCW from +X); a
non-default heading is supported via rotation so the cross-*layer* crosshatch reuses it.

shapely is imported lazily so the light core stays import-cheap (CLAUDE.md).
"""
from __future__ import annotations

import math
from typing import List, Tuple

Segment = Tuple[Tuple[float, float], Tuple[float, float]]


def raster_pitch(cfg) -> float:
    """Line spacing = bead_width * (1 - overlap) (SPEC §4.2)."""
    p = cfg.process.bead_width_mm * (1.0 - cfg.process.raster_overlap)
    if p <= 0:
        raise ValueError(f"raster pitch must be > 0 (bead_width/overlap), got {p}")
    return p


def _iter_lines(geom):
    """Yield shapely LineStrings from an intersection result (Line/Multi/GC)."""
    from shapely.geometry import LineString, MultiLineString, GeometryCollection

    if geom.is_empty:
        return
    if isinstance(geom, LineString):
        if geom.length > 0:
            yield geom
    elif isinstance(geom, (MultiLineString, GeometryCollection)):
        for g in geom.geoms:
            if isinstance(g, LineString) and g.length > 0:
                yield g


def raster_lines(region, pitch: float, heading_deg: float = 90.0,
                 min_len: float = 0.0, bidirectional: bool = False) -> List[Segment]:
    """Hatch ``region`` with parallel lines spaced ``pitch`` apart.

    Returns a list of ``((x0, y0), (x1, y1))`` segments in left-to-right order. Each is
    oriented forward along +heading (so +Y for the default); with ``bidirectional`` set,
    every other line is flipped so adjacent lines run 180° apart (boustrophedon — D13).
    Segments shorter than ``min_len`` are dropped. Holes are respected (the clip is
    against the region polygon, interiors included).
    """
    from shapely import affinity
    from shapely.geometry import LineString

    if pitch <= 0:
        raise ValueError(f"pitch must be > 0, got {pitch}")
    if region.is_empty:
        return []

    # Rotate the region so `heading_deg` aligns with +Y; hatch with vertical lines
    # (constant x); rotate the resulting segments back. delta=0 for the +Y default.
    delta = 90.0 - heading_deg
    center = region.centroid
    rreg = affinity.rotate(region, delta, origin=center) if delta else region

    minx, miny, maxx, maxy = rreg.bounds
    pad = 1.0  # extend cut lines past the bbox so the clip is clean
    segments: List[Segment] = []

    x = minx + pitch / 2.0
    # accumulate (x, piece) then sort for deterministic, left-to-right pass order
    while x < maxx:
        cut = LineString([(x, miny - pad), (x, maxy + pad)])
        for piece in _iter_lines(rreg.intersection(cut)):
            ys = [c[1] for c in piece.coords]
            y0, y1 = min(ys), max(ys)            # orient forward (+Y) in rotated frame
            seg = LineString([(x, y0), (x, y1)])
            if delta:
                seg = affinity.rotate(seg, -delta, origin=center)
            (sx0, sy0), (sx1, sy1) = seg.coords[0], seg.coords[-1]
            length = math.hypot(sx1 - sx0, sy1 - sy0)
            if length >= min_len:
                segments.append(((sx0, sy0), (sx1, sy1)))
        x += pitch

    if bidirectional:
        # flip every other line so adjacent lines run 180 deg apart (the head turns
        # airborne between them); the long fly-back of the one-way sweep is gone (D13).
        for k in range(1, len(segments), 2):
            (s, e) = segments[k]
            segments[k] = (e, s)
    return segments
