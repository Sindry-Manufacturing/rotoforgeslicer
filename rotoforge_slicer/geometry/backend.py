"""Geometry backend interface. SPEC §3.3.

Planning/emission code depends ONLY on this ABC and on shapely polygons,
never on a specific mesh library.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Sequence


class GeometryBackend(ABC):
    @abstractmethod
    def load(self, path: str):
        """Load a mesh from STL/3MF/etc."""

    @abstractmethod
    def repair(self, mesh):
        """Make watertight/manifold: fill holes, fix normals/winding/inversion."""

    @abstractmethod
    def slice(self, mesh, z_heights: Sequence[float]) -> list:
        """Return one entry per height: a list of shapely (Multi)Polygon regions."""
