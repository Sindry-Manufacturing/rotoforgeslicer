"""Unidirectional +Y raster fill within the wedge. SPEC §4.2.

No bidirectional/boustrophedon (-Y deposition is impossible): every pass goes
forward along the heading, then lift and travel (wheel up) back to the next line's
start. The default heading is +Y (90 deg CCW from +X). A non-default heading is
supported via rotation so the cross-*layer* crosshatch (M5) can reuse this.

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
                 min_len: float = 0.0) -> List[Segment]:
    """Hatch ``region`` with parallel lines spaced ``pitch`` apart, each running
    forward along ``heading_deg``.

    Returns a list of ``((x0, y0), (x1, y1))`` segments, each oriented so that
    going start->end advances along +heading (so +Y for the default). Segments
    shorter than ``min_len`` are dropped. Holes are respected (the clip is against
    the region polygon, interiors included).
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
    return segments
