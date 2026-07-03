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


def dominant_heading_deg(region) -> float:
    """The region's dominant direction: the long-axis heading of its minimum
    rotated rectangle, in degrees CCW from +X, normalized to [0, 180)."""
    rect = region.minimum_rotated_rectangle
    coords = list(getattr(rect, "exterior", rect).coords)
    if len(coords) < 3:                      # degenerate (point/line) region
        return 90.0
    best_len, best_deg = -1.0, 90.0
    for (x0, y0), (x1, y1) in zip(coords, coords[1:]):
        length = math.hypot(x1 - x0, y1 - y0)
        if length > best_len:
            best_len = length
            best_deg = math.degrees(math.atan2(y1 - y0, x1 - x0)) % 180.0
    return best_deg


def best_heading_deg(region, cfg, min_len: float, delta_deg: float = 0.0) -> float:
    """The LAID hatch heading that fills this region best, chosen by SCORING
    candidate directions — the region's long axis, its perpendicular, +Y, and +X,
    each COMPOSED with ``delta_deg`` (the layer's crosshatch offset) so the scored
    heading is exactly the heading that will be laid. Under D13 every heading
    deposits identically, so the choice is free; measuring beats guessing.

    Scoring: keep the most bead (coverage), and among candidates within 5% of the
    best coverage prefer the FEWEST pieces — total hatch length is ~area/pitch in
    any direction, so raw kept-length differences between viable candidates are
    mostly boundary quantization noise, while the piece count is the real
    fragmentation signal. Legacy +Y (+delta) is always a candidate."""
    pitch = raster_pitch(cfg)
    dom = dominant_heading_deg(region)
    candidates = []
    for h in (dom, (dom + 90.0) % 180.0, 90.0, 0.0):
        h = (h + delta_deg) % 360.0
        if not any(abs((h - c + 90.0) % 180.0 - 90.0) < 2.0 for c in candidates):
            candidates.append(h)
    scored = []
    for h in candidates:
        segs = raster_lines(region, pitch, heading_deg=h, min_len=min_len)
        kept = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in segs)
        scored.append((h, kept, len(segs)))
    best_kept = max(kept for _, kept, _ in scored)
    if best_kept <= 0:
        if delta_deg:
            # a region that cannot fill at ANY crosshatch-tilted heading (e.g. a
            # rib narrower than min_len at ±30°) keeps its un-tilted best instead
            # of silently depositing nothing — coverage beats crosshatch.
            return best_heading_deg(region, cfg, min_len, delta_deg=0.0)
        return 90.0
    viable = [(h, kept, n) for h, kept, n in scored if kept >= 0.95 * best_kept]
    return min(viable, key=lambda t: (t[2], -t[1]))[0]


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
