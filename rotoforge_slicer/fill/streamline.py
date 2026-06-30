"""+Y-biased guidance-field streamlines, clipped to the region. SPEC §4.2.

Curved fill for regions where straight rasters waste motion: trace streamlines of a
guidance field that is biased toward the deposition heading (+Y home) but bends to
follow the region boundary, so passes parallel the part's contours. Each streamline's
heading is clamped into the depositable wedge; tight turns are broken later by the
curvature limit (``fill.curvature.split_on_curvature``) and monotonic-forward / length
are enforced here. Seeds reuse the raster's clipped line starts so holes and arbitrary
headings are handled.

numpy / scipy / shapely are imported lazily so the light core stays import-cheap.
"""
from __future__ import annotations

import math
from typing import List, Tuple

from .raster import raster_lines, raster_pitch

Point = Tuple[float, float]


def streamline_fill(region, cfg, heading_deg: float = 90.0) -> List[List[Point]]:
    """Return curved fill polylines (each forward-oriented, inside the region)."""
    import numpy as np
    import shapely
    from scipy import ndimage

    if region.is_empty:
        return []

    step = cfg.fill.streamline_step_mm
    curl = cfg.fill.streamline_curl
    home = cfg.c_axis.home_heading_deg
    wedge = cfg.c_axis.wedge_half_angle_deg
    min_len = cfg.process.min_deposit_len_mm
    brad = math.radians(heading_deg)
    bdir = (math.cos(brad), math.sin(brad))

    # ---- rasterise the region + distance field (inside, in mm) ----
    minx, miny, maxx, maxy = region.bounds
    res = max(step, 0.5)
    pad = 3.0 * res
    x0, y0 = minx - pad, miny - pad
    nx = int(math.ceil((maxx - minx + 2 * pad) / res)) + 1
    ny = int(math.ceil((maxy - miny + 2 * pad) / res)) + 1
    gx = x0 + np.arange(nx) * res
    gy = y0 + np.arange(ny) * res
    mx, my = np.meshgrid(gx, gy, indexing="ij")
    inside = shapely.contains_xy(region, mx.ravel(), my.ravel()).reshape(nx, ny)
    if not inside.any():
        return []
    dist = ndimage.distance_transform_edt(inside) * res        # mm to boundary
    ddx, ddy = np.gradient(dist, res)                          # inward gradient

    def _cell(x, y):
        i = min(max(int(round((x - x0) / res)), 0), nx - 1)
        j = min(max(int(round((y - y0) / res)), 0), ny - 1)
        return i, j

    def _inside(x, y) -> bool:
        i, j = _cell(x, y)
        return dist[i, j] > 0.5 * res

    def _direction(x, y):
        """Unit guidance direction: bias blended with the boundary tangent, clamped
        into the depositable wedge about home (SPEC §4.1)."""
        i, j = _cell(x, y)
        gxv, gyv = ddx[i, j], ddy[i, j]
        gn = math.hypot(gxv, gyv)
        if gn < 1e-9:
            dx, dy = bdir
        else:
            tx, ty = -gyv / gn, gxv / gn               # boundary-parallel (perp to grad)
            if tx * bdir[0] + ty * bdir[1] < 0:        # orient with the bias
                tx, ty = -tx, -ty
            dx, dy = bdir[0] + curl * tx, bdir[1] + curl * ty
        hd = math.degrees(math.atan2(dy, dx))
        off = max(-wedge, min(wedge, hd - home))       # clamp into the wedge
        hd = home + off
        return math.cos(math.radians(hd)), math.sin(math.radians(hd))

    # ---- trace one streamline per raster seed (the -bias clipped-line starts) ----
    seeds = [seg[0] for seg in raster_lines(region, raster_pitch(cfg), heading_deg)]
    span = (maxx - minx) + (maxy - miny)
    max_steps = int(span / step) + 50

    paths: List[List[Point]] = []
    for sx, sy in seeds:
        pts: List[Point] = [(sx, sy)]
        x, y = sx, sy
        for _ in range(max_steps):
            dx, dy = _direction(x, y)
            if dx * bdir[0] + dy * bdir[1] <= 0.05:   # monotonic forward only (SPEC §4.2)
                break
            nx_, ny_ = x + step * dx, y + step * dy
            if not _inside(nx_, ny_):
                break
            x, y = nx_, ny_
            pts.append((x, y))
        if _length(pts) >= min_len:
            paths.append(pts)
    return paths


def _length(pts) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
