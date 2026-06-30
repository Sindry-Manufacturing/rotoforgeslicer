"""Geometry backend interface. SPEC §3.3.

Planning/emission code depends ONLY on this ABC and on shapely polygons,
never on a specific mesh library.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Sequence, Tuple


class GeometryBackend(ABC):
    @abstractmethod
    def load(self, path: str):
        """Load a mesh from STL/3MF/etc."""

    @abstractmethod
    def repair(self, mesh):
        """Make watertight/manifold: fill holes, fix normals/winding/inversion."""

    @abstractmethod
    def bounds(self, mesh) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        """Axis-aligned bounding box as ((xmin, ymin, zmin), (xmax, ymax, zmax)).

        Planning code uses this to pick layer Z heights without depending on a
        specific mesh library (SPEC §3.3).
        """

    @abstractmethod
    def slice(self, mesh, z_heights: Sequence[float]) -> list:
        """Planar slice at each Z in ``z_heights`` (planar layers — the rotary
        axis turns about Z, so slicing stays flat; SPEC §0).

        Returns one entry per height: a list of shapely Polygons (each may carry
        interior holes) describing the solid regions at that Z. A height that
        misses the mesh yields an empty list.
        """
