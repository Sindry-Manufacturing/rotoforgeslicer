"""Geometry backends (mesh load / repair / planar slice). SPEC §3.3.

Importing this package stays light: the backends and slicing helpers pull trimesh
and shapely lazily inside their functions (CLAUDE.md).
"""
from __future__ import annotations

from .backend import GeometryBackend
from .meshlib_backend import MeshLibBackend
from .slicing import (
    Layer,
    SlicedModel,
    clean_polygons,
    layer_heights,
    slice_model,
)
from .trimesh_backend import TrimeshBackend

__all__ = [
    "GeometryBackend",
    "TrimeshBackend",
    "MeshLibBackend",
    "Layer",
    "SlicedModel",
    "clean_polygons",
    "layer_heights",
    "slice_model",
]
