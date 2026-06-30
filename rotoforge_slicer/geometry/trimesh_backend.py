"""Default geometry backend: trimesh (MIT). SPEC §3.2/§3.3.

NOTE: verify the trimesh API against the installed version during M1. trimesh is
imported lazily so the package imports without it.
"""
from __future__ import annotations

from typing import Sequence

from .backend import GeometryBackend


class TrimeshBackend(GeometryBackend):
    def load(self, path: str):
        import trimesh
        return trimesh.load(path, force="mesh")

    def repair(self, mesh):
        import trimesh
        trimesh.repair.fix_inversion(mesh)
        trimesh.repair.fix_normals(mesh)
        trimesh.repair.fix_winding(mesh)
        trimesh.repair.fill_holes(mesh)
        mesh.process(validate=True)
        return mesh

    def slice(self, mesh, z_heights: Sequence[float]) -> list:
        # mesh.section_multiplane(plane_origin, plane_normal, heights)
        #   -> list[trimesh.path.Path2D | None]; Path2D.polygons_full -> shapely Polygons.
        import numpy as np
        origin = np.array([0.0, 0.0, 0.0])
        normal = np.array([0.0, 0.0, 1.0])
        sections = mesh.section_multiplane(
            plane_origin=origin, plane_normal=normal, heights=list(z_heights))
        layers: list = []
        for sec in sections:
            layers.append([] if sec is None else list(sec.polygons_full))
        return layers
