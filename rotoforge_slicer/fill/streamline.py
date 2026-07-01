"""Guidance-field streamlines, clipped to the region. SPEC §4.2 (D13).

Curved fill for regions where straight rasters waste motion: trace streamlines of a
guidance field biased toward a base heading (+Y by default) but bending to follow the
region boundary, so passes parallel the part's contours. Under D13 there is **no
wedge**: the streamline heading is not clamped and curves need not stay "forward" —
tight turns and over-winding are broken downstream by the slew limit
(``split_on_curvature``) and the usable axis range (``split_on_winding``). Length is
enforced here; seeds reuse the raster's clipped line starts so holes and arbitrary
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
        """Unit guidance direction: base bias blended with the boundary tangent. No
        wedge clamp (D13) — the heading follows the field freely; slew/winding splits
        downstream keep each emitted pass legal."""
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
        n = math.hypot(dx, dy)
        return (dx / n, dy / n) if n else bdir

    # ---- trace one streamline per raster seed (the -bias clipped-line starts) ----
    seeds = [seg[0] for seg in raster_lines(region, raster_pitch(cfg), heading_deg)]
    span = (maxx - minx) + (maxy - miny)
    max_steps = int(span / step) + 50

    paths: List[List[Point]] = []
    for sx, sy in seeds:
        pts: List[Point] = [(sx, sy)]
        x, y = sx, sy
        pdx, pdy = bdir                                # previous step direction (seed = bias)
        for _ in range(max_steps):
            dx, dy = _direction(x, y)
            # No wedge / forward-only clamp (D13). Only stop on a hard self-reversal
            # (a ~180 deg cusp the axis can't track) — downstream split_on_curvature /
            # split_on_winding break the rest; min_len drops anything too short.
            if dx * pdx + dy * pdy <= -0.5:
                break
            nx_, ny_ = x + step * dx, y + step * dy
            if not _inside(nx_, ny_):
                break
            x, y = nx_, ny_
            pts.append((x, y))
            pdx, pdy = dx, dy
        if _length(pts) >= min_len:
            paths.append(pts)
    return paths


def _length(pts) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
