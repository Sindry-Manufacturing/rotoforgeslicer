"""2.5D swept-disc + leading-wire collision / approach check. SPEC §4.6.

The collision body is the 50 mm wheel disc plus the fragile leading wire — not a
point. We keep a 2.5D height field of deposited material (cell ~ bead/2) and, for
each planned pass *before* depositing it, sweep the contact along the whole move
(start→end, SPEC §4.6 "the line of disc-lowest-points along the heading") and check
that, at every sampled contact:

  - the **leading wire** (reaching ``wire_lead`` ahead of the contact, at the
    deposition plane) does not drive into material already at the current layer
    height or above — i.e. never plunge into / lead into a step-up or a laid bead; and
  - the **disc body** (which hugs the surface, rising ``R - sqrt(R²-d²)`` at fore
    offset ``d``) clears any taller existing feature by ``clearance``.

The companion rule lives in pass ordering (``passplan.order_passes_lead_away``):
deposit the least-forward passes first so the wire always leads into free space.
Resolution of a residual violation (lift / reorient) is a later refinement; M4
flags it. numpy is imported lazily so the light core stays import-cheap.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple


def disc_drop(d: float, radius: float) -> float:
    """How far the disc bottom rises above the contact at fore/aft offset ``d``.

    ``R - sqrt(R² - d²)`` — e.g. ~0.5 mm at d=5, R=25 (SPEC §4.6). Beyond the disc
    radius the body has lifted clear, so return +inf (no constraint)."""
    if abs(d) >= radius:
        return math.inf
    return radius - math.sqrt(radius * radius - d * d)


class HeightField:
    """Coarse 2.5D max-height grid of deposited material over an XY window."""

    def __init__(self, x0: float, y0: float, x1: float, y1: float, cell: float):
        import numpy as np

        self.cell = cell
        self.x0 = x0
        self.y0 = y0
        self.nx = max(1, int(math.ceil((x1 - x0) / cell)) + 1)
        self.ny = max(1, int(math.ceil((y1 - y0) / cell)) + 1)
        self.grid = np.zeros((self.nx, self.ny), dtype=float)

    def _clamp_ij(self, x: float, y: float):
        i = int(round((x - self.x0) / self.cell))
        j = int(round((y - self.y0) / self.cell))
        return min(max(i, 0), self.nx - 1), min(max(j, 0), self.ny - 1)

    def height_at(self, x: float, y: float) -> float:
        i = round((x - self.x0) / self.cell)
        j = round((y - self.y0) / self.cell)
        if not (0 <= i < self.nx and 0 <= j < self.ny):
            return 0.0  # outside the deposited window -> bare bed
        return float(self.grid[int(i), int(j)])

    def heights_at(self, xs, ys):
        """Vectorised height lookup; points outside the window read 0 (bare bed)."""
        import numpy as np

        xs = np.asarray(xs, dtype=float)
        ys = np.asarray(ys, dtype=float)
        i = np.round((xs - self.x0) / self.cell).astype(int)
        j = np.round((ys - self.y0) / self.cell).astype(int)
        inside = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny)
        h = self.grid[np.clip(i, 0, self.nx - 1), np.clip(j, 0, self.ny - 1)]
        return np.where(inside, h, 0.0)

    def deposit_segment(self, p0, p1, half_width: float, z: float) -> None:
        """Raise every cell within ``half_width`` of the segment to at least ``z``."""
        import numpy as np

        (ax, ay), (bx, by) = p0, p1
        lo_x, hi_x = min(ax, bx) - half_width, max(ax, bx) + half_width
        lo_y, hi_y = min(ay, by) - half_width, max(ay, by) + half_width
        i0, j0 = self._clamp_ij(lo_x, lo_y)
        i1, j1 = self._clamp_ij(hi_x, hi_y)
        ii = np.arange(i0, i1 + 1)
        jj = np.arange(j0, j1 + 1)
        cx = self.x0 + ii * self.cell
        cy = self.y0 + jj * self.cell
        gx, gy = np.meshgrid(cx, cy, indexing="ij")
        dist = _point_segment_distance(gx, gy, ax, ay, bx, by)
        mask = dist <= half_width
        block = self.grid[i0:i1 + 1, j0:j1 + 1]
        block[mask] = np.maximum(block[mask], z)


def _point_segment_distance(px, py, ax, ay, bx, by):
    import numpy as np

    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return np.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / L2
    t = np.clip(t, 0.0, 1.0)
    return np.hypot(px - (ax + t * dx), py - (ay + t * dy))


@dataclass
class Collision:
    pass_index: int
    z: float
    kind: str            # "wire" | "disc"
    at: Tuple[float, float]
    material_z: float
    detail: str


def _heading_unit(start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    n = math.hypot(dx, dy)
    return (dx / n, dy / n) if n else (0.0, 1.0)


def check_pass(field: HeightField, p, cfg, *, index: int = 0) -> Optional[Collision]:
    """Return a Collision if depositing ``p`` would drive the leading wire or the disc
    body into existing material, anywhere along the swept move, else None. Curved passes
    are checked segment by segment, each with its own heading (SPEC §4.6)."""
    for (s, e) in p.segments():
        c = _check_segment(field, s, e, p.z, cfg, index)
        if c is not None:
            return c
    return None


def _check_segment(field, s, e, z, cfg, index) -> Optional[Collision]:
    import numpy as np

    lh = cfg.process.layer_height_mm
    radius = cfg.process.wheel_diameter_mm / 2.0
    rim_half = cfg.process.bead_width_mm / 2.0
    clearance = cfg.collision.clearance_mm
    wire_lead = cfg.collision.wire_lead_mm
    # Only material risen ABOVE the current layer is an obstruction. In strict bottom-up
    # deposition everything is <= z, so this is 0 for normal parts; the adjacent bead
    # OVERLAP (at z) and the substrate (below z) are not crashes. ``tol`` keeps same-layer
    # material (and coarse-cell bleed) from flagging.
    tol = 0.5 * lh
    hx, hy = _heading_unit(s, e)
    (sx, sy), (ex, ey) = s, e

    # contact points swept along the segment (SPEC §4.6), ~ one per bead width
    seg_len = math.hypot(ex - sx, ey - sy)
    step = max(field.cell, cfg.process.bead_width_mm)
    nc = max(2, int(math.ceil(seg_len / step)) + 1)
    t = np.linspace(0.0, 1.0, nc)
    cx = sx + t * (ex - sx)
    cy = sy + t * (ey - sy)

    # --- leading wire (and the contact itself): a step-up taller than this layer ahead
    for ahead in (0.0, wire_lead):
        wx = cx + ahead * hx
        wy = cy + ahead * hy
        h = field.heights_at(wx, wy)
        bad = h > z + tol
        if bad.any():
            k = int(np.argmax(bad))
            return Collision(index, z, "wire", (float(wx[k]), float(wy[k])), float(h[k]),
                             f"leading wire into a Z={float(h[k]):.3f} step-up (deposit Z={z:.3f})")

    # --- disc body: fore offsets; flag taller material the rising disc body can't clear.
    dstep = max(field.cell, cfg.process.bead_width_mm)
    ds = np.arange(rim_half, radius + 1e-9, dstep)
    if ds.size:
        drop = np.array([disc_drop(float(d), radius) for d in ds])  # (D,)
        dx = cx[:, None] + ds[None, :] * hx                          # (C, D)
        dy = cy[:, None] + ds[None, :] * hy
        h = field.heights_at(dx.ravel(), dy.ravel()).reshape(dx.shape)
        disc_bottom = z + drop[None, :]
        bad = (h > z + tol) & (h > disc_bottom - clearance)
        if bad.any():
            ci, di = np.unravel_index(int(np.argmax(bad)), bad.shape)
            return Collision(index, z, "disc",
                             (float(dx[ci, di]), float(dy[ci, di])), float(h[ci, di]),
                             f"disc body (bottom Z={float(disc_bottom[0, di]):.3f}) into "
                             f"material Z={float(h[ci, di]):.3f} at {float(ds[di]):.1f}mm fore")
    return None


def replay_collision_check(plan, cfg) -> List[Collision]:
    """Replay the plan against a fresh height field (passes in plan order), checking
    each pass's swept approach *before* depositing its bead. Returns all collisions."""
    bounds = _plan_xy_bounds(plan)
    if bounds is None:
        return []
    radius = cfg.process.wheel_diameter_mm / 2.0
    margin = radius + cfg.collision.wire_lead_mm + 1.0
    x0, y0, x1, y1 = bounds
    cell = cfg.collision.cell_mm or cfg.process.bead_width_mm / 2.0
    field = HeightField(x0 - margin, y0 - margin, x1 + margin, y1 + margin, cell)
    half = cfg.process.bead_width_mm / 2.0

    out: List[Collision] = []
    idx = 0
    for ly in plan.layers:
        for p in ly.passes:
            c = check_pass(field, p, cfg, index=idx)
            if c is not None:
                out.append(c)
            for (s, e) in p.segments():        # deposit the whole polyline (curved-safe)
                field.deposit_segment(s, e, half, p.z)
            idx += 1
    return out


def assert_no_collisions(plan, cfg) -> List[Collision]:
    """Run the replay check (when enabled) and RAISE on any residual collision.

    Pass ordering already leads away from laid material; anything left needs lift /
    reorient (a later refinement), so it surfaces as a build failure rather than an
    unsafe path. ``collision.enabled = False`` is the escape hatch."""
    if not cfg.collision.enabled:
        return []
    cols = replay_collision_check(plan, cfg)
    if cols:
        c = cols[0]
        raise ValueError(
            f"{len(cols)} collision(s) detected (SPEC §4.6); first at pass "
            f"{c.pass_index} (layer Z={c.z:.3f}): {c.detail}")
    return cols


def _plan_xy_bounds(plan):
    # Every polyline vertex, not just the chord endpoints: a curved pass can bow well
    # outside its start->end chord, and material that lands outside the height-field
    # window is silently dropped (heights_at -> 0) and would weaken the check.
    xs: List[float] = []
    ys: List[float] = []
    for ly in plan.layers:
        for p in ly.passes:
            for (x, y) in p.points:
                xs.append(x)
                ys.append(y)
    if not xs:
        return None
    return min(xs), min(ys), max(xs), max(ys)
