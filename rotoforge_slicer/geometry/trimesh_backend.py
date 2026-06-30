"""Default geometry backend: trimesh (MIT). SPEC §3.2/§3.3.

trimesh is imported lazily inside every method so ``import rotoforge_slicer`` and
the light core tests do not pull it (CLAUDE.md). Verified against trimesh 4.12:
``section_multiplane`` returns ``list[Path2D | None]`` and ``Path2D.polygons_full``
yields shapely ``Polygon`` objects (with interior rings for holes). Path assembly
needs ``scipy``; the slicing deps (``scipy``/``rtree``) are listed in §3.2.
"""
from __future__ import annotations

from typing import Sequence, Tuple

from .backend import GeometryBackend


class TrimeshBackend(GeometryBackend):
    def load(self, path: str):
        import trimesh

        obj = trimesh.load(path, force="mesh")
        # force="mesh" concatenates a Scene into one Trimesh, but guard anyway so
        # callers always get a slice-able body and a clear error otherwise.
        if isinstance(obj, trimesh.Scene):
            if not obj.geometry:
                raise ValueError(f"{path!r} loaded as an empty scene (no geometry).")
            obj = obj.dump(concatenate=True)
        if not isinstance(obj, trimesh.Trimesh):
            raise TypeError(
                f"{path!r} did not load as a triangular mesh (got {type(obj).__name__}).")
        if obj.is_empty or len(obj.faces) == 0:
            raise ValueError(f"{path!r} contains no faces.")
        return obj

    def repair(self, mesh):
        """Best-effort watertight/manifold repair (SPEC §3.2).

        Each step is independent and tolerant: a mesh that is already clean, or a
        repair that cannot help, must not abort slicing. The caller can inspect
        ``mesh.is_watertight`` afterwards.
        """
        import trimesh

        for fn in (
            trimesh.repair.fix_inversion,
            trimesh.repair.fix_normals,
            trimesh.repair.fix_winding,
            trimesh.repair.fill_holes,
        ):
            try:
                fn(mesh)
            except Exception:
                # Repair is advisory; never let a single fixer kill the pipeline.
                pass
        try:
            mesh.process(validate=True)
        except Exception:
            pass
        return mesh

    def bounds(self, mesh) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        b = mesh.bounds  # ((xmin,ymin,zmin),(xmax,ymax,zmax)) as a 2x3 array
        if b is None:
            raise ValueError("mesh has no bounds (empty geometry).")
        lo, hi = b[0], b[1]
        return (float(lo[0]), float(lo[1]), float(lo[2])), (float(hi[0]), float(hi[1]), float(hi[2]))

    def slice(self, mesh, z_heights: Sequence[float]) -> list:
        import numpy as np

        heights = [float(z) for z in z_heights]
        if not heights:
            return []
        sections = mesh.section_multiplane(
            plane_origin=np.array([0.0, 0.0, 0.0]),
            plane_normal=np.array([0.0, 0.0, 1.0]),
            heights=heights,
        )
        layers: list = []
        for sec in sections:
            if sec is None:
                layers.append([])
                continue
            # polygons_full carries each solid region with its holes as interiors.
            layers.append(list(getattr(sec, "polygons_full", []) or []))
        return layers
