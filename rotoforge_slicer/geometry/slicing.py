"""Planar slicing: layer Z heights, region cleanup, sliced-model assembly. SPEC §3.1.

The rotary axis turns about Z, so slicing stays planar (flat Z layers) — this
module turns a mesh + layer height into per-layer shapely region polygons that the
fill/planning stages consume. It depends only on the ``GeometryBackend`` ABC and on
shapely (imported lazily so the light core stays import-cheap; CLAUDE.md).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple


def layer_heights(z_min: float, z_max: float, layer_height: float) -> List[float]:
    """Z heights for planar layers, sampled at each layer's mid-height.

    First layer centre at ``z_min + layer_height/2``; subsequent layers step by
    ``layer_height`` while strictly below ``z_max``.
    """
    if layer_height <= 0:
        raise ValueError(f"layer_height must be > 0, got {layer_height}")
    if z_max <= z_min:
        return []
    hs: List[float] = []
    z = z_min + layer_height / 2.0
    while z < z_max:
        hs.append(round(z, 6))
        z += layer_height
    return hs


def clean_polygons(geoms: Sequence, min_area: float = 1e-9) -> List:
    """Normalise a section's geometries into a flat list of valid shapely Polygons.

    Repairs invalid rings via ``buffer(0)``, explodes any MultiPolygons, drops
    empties and slivers below ``min_area`` (mm²). Interior holes are preserved.
    """
    from shapely.geometry import Polygon
    from shapely.geometry.base import BaseGeometry

    out: List[Polygon] = []

    def _emit(g, allow_repair: bool = True) -> None:
        if g is None or g.is_empty:
            return
        if isinstance(g, Polygon):
            if not g.is_valid and allow_repair:
                # buffer(0) yields areal geometry (Polygon/MultiPolygon); recurse
                # once (allow_repair=False) so a degenerate result can't re-loop.
                _emit(g.buffer(0), allow_repair=False)
                return
            if g.area >= min_area:
                out.append(g)
            return
        # MultiPolygon / GeometryCollection / etc.: explode; lines/points have no
        # .geoms and are ignored (they can appear from degenerate sections).
        parts = getattr(g, "geoms", None)
        if parts is not None:
            for part in parts:
                _emit(part, allow_repair)

    for geom in geoms:
        if isinstance(geom, BaseGeometry):
            _emit(geom)
    return out


@dataclass
class Layer:
    """One planar slice: its sampling Z and the solid regions at that Z.

    ``regions`` is a list of shapely Polygons (holes carried as interior rings).
    An empty list means the slicing plane missed the mesh at this height.
    """

    index: int
    z: float
    regions: List = field(default_factory=list)

    @property
    def is_empty(self) -> bool:
        return not self.regions

    @property
    def area(self) -> float:
        return float(sum(p.area for p in self.regions))

    @property
    def bounds(self) -> Optional[Tuple[float, float, float, float]]:
        """(minx, miny, maxx, maxy) over all regions, or None if empty."""
        if not self.regions:
            return None
        xs0, ys0, xs1, ys1 = zip(*(p.bounds for p in self.regions))
        return (min(xs0), min(ys0), max(xs1), max(ys1))

    def union(self):
        """Single (Multi)Polygon merging all regions (shapely)."""
        from shapely.ops import unary_union

        return unary_union(self.regions)


@dataclass
class SlicedModel:
    """The full stack of planar layers plus the mesh Z extent that produced them."""

    layers: List[Layer]
    layer_height: float
    z_min: float
    z_max: float

    def __len__(self) -> int:
        return len(self.layers)

    def __iter__(self):
        return iter(self.layers)

    def __getitem__(self, i) -> Layer:
        return self.layers[i]

    @property
    def nonempty_layers(self) -> List[Layer]:
        return [ly for ly in self.layers if not ly.is_empty]

    @property
    def total_area(self) -> float:
        return float(sum(ly.area for ly in self.layers))


def place_on_bed(model: "SlicedModel", cfg) -> "SlicedModel":
    """Translate a sliced model so it rests on the bed and is centred in XY.

    Drops the part bottom to Z=0 and centres its XY footprint on the build-plate
    centre (matching the reference tool's "Placed model bounds"). The +Y lead-out
    that follows every pass is reserved when centring, so a part that itself fits
    does not push its lead-outs off the plate (SPEC §6.3). shapely-only, so it stays
    backend-agnostic.
    """
    from shapely import affinity

    bnds = [ly.bounds for ly in model.layers if ly.bounds is not None]
    if not bnds:
        return model
    xmin = min(b[0] for b in bnds)
    ymin = min(b[1] for b in bnds)
    xmax = max(b[2] for b in bnds)
    ymax = max(b[3] for b in bnds)
    bx, by, _ = cfg.machine.build_volume_mm
    lead_out = cfg.process.lead_out_len_mm
    dx = bx / 2.0 - 0.5 * (xmin + xmax)
    # centre the [ymin, ymax + lead_out] envelope so +Y lead-outs stay on the plate
    dy = by / 2.0 - 0.5 * (ymin + ymax + lead_out)
    dz = -model.z_min
    layers = [
        Layer(index=ly.index, z=ly.z + dz,
              regions=[affinity.translate(p, dx, dy) for p in ly.regions])
        for ly in model.layers
    ]
    return SlicedModel(layers=layers, layer_height=model.layer_height,
                       z_min=model.z_min + dz, z_max=model.z_max + dz)


def slice_model(
    backend,
    mesh,
    layer_height: float,
    *,
    repair: bool = True,
    min_region_area: float = 1e-9,
) -> SlicedModel:
    """Repair (optional) + planar-slice a mesh into a :class:`SlicedModel`.

    Heights come from the mesh's own Z extent (``backend.bounds``) so callers need
    not know the mesh type. Each layer's polygons are cleaned via
    :func:`clean_polygons`.
    """
    if repair:
        mesh = backend.repair(mesh)
    (_, _, z_min), (_, _, z_max) = backend.bounds(mesh)
    heights = layer_heights(z_min, z_max, layer_height)
    raw = backend.slice(mesh, heights)
    layers = [
        Layer(index=i, z=z, regions=clean_polygons(raw[i], min_region_area))
        for i, z in enumerate(heights)
    ]
    return SlicedModel(layers=layers, layer_height=layer_height, z_min=z_min, z_max=z_max)
