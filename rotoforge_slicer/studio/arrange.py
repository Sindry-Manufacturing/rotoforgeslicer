"""Auto-arrange: multi-part plate layout. A Python port of PrusaSlicer's arrange
architecture (src/slic3r-arrange + slic3r-arrange-wrapper, (c) Prusa Research,
AGPLv3 — structure and scoring ported with permission of the project license).

The ported structure:

* :class:`ArrangeItem` — outline (convex hull) + **inflation** (half the part
  spacing) + priority + a fixed flag (already-placed obstacles), result written
  back as a translation;
* :class:`RectangleBed` — the plate with an **inset** (for us: the lead-out
  envelope that every pass's runout needs on all sides, SPEC §6.3);
* ``arrange(items, fixed, bed, ...)`` — first-fit-DECREASING selection (priority,
  then footprint area, like ``firstfit::SelectionStrategy``) over candidate
  placements scored by a **TM-style kernel** (``TMArrangeKernel``): *big* items
  (>2% of bed area) minimize the growth of the pile bounding box plus gravity
  toward the bed-center sink; *small* items snuggle toward the pile centroid.

Where PrusaSlicer generates candidate positions from no-fit polygons, this port
uses a coarse-to-fine grid with shapely collision tests — at Rotoforge part counts
(a handful of parts on a 380x235 plate) that is exact enough and dependency-free.
shapely is imported lazily (CLAUDE.md).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

BIG_ITEM_THRESHOLD = 0.02        # fraction of bed area (PrusaSlicer's BigItemTreshold)
UNARRANGED = None                # translation result for items that did not fit


@dataclass
class ArrangeItem:
    """One part to place: its footprint hull at the CURRENT position, an
    inflation margin (half the required part spacing), and a priority (higher
    packs first). ``fixed`` items are immovable obstacles. The result is
    ``translation`` — a delta to apply to the part's position."""

    outline: object                       # shapely Polygon (footprint hull)
    inflation_mm: float = 0.0
    priority: int = 0
    fixed: bool = False
    key: object = None                    # caller's handle (e.g. the ScenePart)
    translation: Optional[Tuple[float, float]] = field(default=None, init=False)

    def inflated(self):
        return self.outline.buffer(self.inflation_mm) if self.inflation_mm > 0 \
            else self.outline

    @property
    def area(self) -> float:
        return float(self.outline.area)


@dataclass
class RectangleBed:
    """The build plate, inset on all sides (the lead-out envelope — a placement
    is only valid if every pass's runout stays on the plate, SPEC §6.3)."""

    width_mm: float
    depth_mm: float
    inset_mm: float = 0.0

    @property
    def area(self) -> float:
        return self.width_mm * self.depth_mm

    @property
    def center(self) -> Tuple[float, float]:
        return self.width_mm / 2.0, self.depth_mm / 2.0

    def region(self):
        from shapely.geometry import box

        return box(self.inset_mm, self.inset_mm,
                   self.width_mm - self.inset_mm, self.depth_mm - self.inset_mm)


class _TMKernel:
    """PrusaSlicer's TMArrangeKernel scoring, reduced to what we need: big items
    minimize normalized pile-bbox growth + distance to the gravity sink (bed
    centre); small items minimize distance to the pile centroid."""

    def __init__(self, bed: RectangleBed):
        self.bed = bed
        self.norm = math.sqrt(bed.area)
        self.sink = bed.center
        self.pile_bb: Optional[Tuple[float, float, float, float]] = None
        self.pile_centroid: Optional[Tuple[float, float]] = None
        self._pile_items: List[Tuple[float, float, float]] = []   # (cx, cy, area)

    def is_big(self, item: ArrangeItem) -> bool:
        return item.area / self.bed.area > BIG_ITEM_THRESHOLD or self.pile_bb is None

    def score(self, item: ArrangeItem, placed_outline) -> float:
        minx, miny, maxx, maxy = placed_outline.bounds
        c = placed_outline.centroid
        if self.is_big(item):
            full = (minx, miny, maxx, maxy) if self.pile_bb is None else (
                min(minx, self.pile_bb[0]), min(miny, self.pile_bb[1]),
                max(maxx, self.pile_bb[2]), max(maxy, self.pile_bb[3]))
            bb_score = math.hypot(full[2] - full[0], full[3] - full[1]) / self.norm
            sink_d = math.hypot(c.x - self.sink[0], c.y - self.sink[1]) / self.norm
            return 0.65 * bb_score + 0.35 * sink_d      # TM big-item blend
        px, py = self.pile_centroid or self.sink
        return math.hypot(c.x - px, c.y - py) / self.norm

    def account(self, placed_outline) -> None:
        minx, miny, maxx, maxy = placed_outline.bounds
        self.pile_bb = (minx, miny, maxx, maxy) if self.pile_bb is None else (
            min(minx, self.pile_bb[0]), min(miny, self.pile_bb[1]),
            max(maxx, self.pile_bb[2]), max(maxy, self.pile_bb[3]))
        c = placed_outline.centroid
        self._pile_items.append((float(c.x), float(c.y), placed_outline.area))
        w = sum(a for _, _, a in self._pile_items)
        self.pile_centroid = (sum(x * a for x, _, a in self._pile_items) / w,
                              sum(y * a for _, y, a in self._pile_items) / w)


def arrange(items: List[ArrangeItem], fixed: List[ArrangeItem],
            bed: RectangleBed, *, coarse_cells: int = 40) -> List[ArrangeItem]:
    """First-fit-decreasing arrange over grid candidates (see module docstring).

    Mutates each movable item's ``translation`` (None = did not fit) and returns
    the items that could not be placed. Deterministic for identical input.
    """
    from shapely import affinity
    from shapely.strtree import STRtree

    kernel = _TMKernel(bed)
    region = bed.region()
    obstacles = []
    for f in fixed:
        obstacles.append(f.inflated())
        kernel.account(f.outline)

    order = sorted(range(len(items)),
                   key=lambda i: (-items[i].priority, -items[i].area))
    step = max(bed.width_mm, bed.depth_mm) / coarse_cells
    unplaced: List[ArrangeItem] = []

    for idx in order:
        item = items[idx]
        infl = item.inflated()
        ox, oy = infl.centroid.x, infl.centroid.y
        tree = STRtree(obstacles) if obstacles else None

        def feasible(dx, dy):
            cand = affinity.translate(infl, dx, dy)
            if not region.contains(cand):
                return None
            if tree is not None:
                for j in tree.query(cand):
                    if obstacles[j].intersects(cand):
                        return None
            return cand

        best = None      # (score, dx, dy)
        # coarse grid over the bed, centred candidate positions
        nx = int(bed.width_mm / step) + 1
        ny = int(bed.depth_mm / step) + 1
        for ix in range(nx):
            for iy in range(ny):
                dx = bed.inset_mm + ix * step - ox
                dy = bed.inset_mm + iy * step - oy
                if feasible(dx, dy) is not None:
                    cand = affinity.translate(item.outline, dx, dy)
                    s = kernel.score(item, cand)
                    if best is None or s < best[0]:
                        best = (s, dx, dy)
        if best is not None:
            # refine around the winner at quarter-steps (coarse-to-fine)
            for fine in (step / 4.0, step / 16.0):
                s0, bx, by = best
                for dx in (bx - fine, bx, bx + fine):
                    for dy in (by - fine, by, by + fine):
                        if (dx, dy) != (bx, by) and feasible(dx, dy) is not None:
                            cand = affinity.translate(item.outline, dx, dy)
                            s = kernel.score(item, cand)
                            if s < best[0]:
                                best = (s, dx, dy)
        if best is None:
            item.translation = UNARRANGED
            unplaced.append(item)
            continue
        _, dx, dy = best
        item.translation = (dx, dy)
        placed = affinity.translate(item.outline, dx, dy)
        obstacles.append(affinity.translate(infl, dx, dy))
        kernel.account(placed)

    return unplaced
