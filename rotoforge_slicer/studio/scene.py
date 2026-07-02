"""Build-plate scene: parts, transforms, placement, multi-part slicing. SPEC §9 (M11).

Pure placement math lives here (numpy only — no trimesh / Qt / pyvista at import
time), so the transform / drop-to-bed / fit logic is unit-testable with stub meshes
that expose only ``.vertices``. The mesh-level operations (``placed_mesh`` /
``slice_scene``) pull trimesh lazily.

Conventions:

* A part's ``(x, y)`` is the plate position of its **pivot** — the part's own
  bounding-box centre. Transform order: centre-on-pivot → scale → Rx → Ry → Rz →
  translate. Rotations therefore tumble the part about its middle, like every
  desktop slicer.
* Parts always **rest on the bed**: the Z translation is not user-set but computed
  (``drop-to-bed``) after every transform change so the transformed minimum Z is 0.
* Placement replaces ``place_on_bed`` — ``slice_scene`` slices the placed parts
  as-is (no re-centring), so what you arrange is what the machine runs.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import List, Optional, Sequence, Tuple

import numpy as np


def _rot(axis: str, deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    m = np.eye(4)
    if axis == "x":
        m[1:3, 1:3] = [[c, -s], [s, c]]
    elif axis == "y":
        m[0, 0], m[0, 2], m[2, 0], m[2, 2] = c, s, -s, c
    else:
        m[0:2, 0:2] = [[c, -s], [s, c]]
    return m


def _translate(dx: float, dy: float, dz: float) -> np.ndarray:
    m = np.eye(4)
    m[:3, 3] = (dx, dy, dz)
    return m


@dataclass
class ScenePart:
    """One mesh on the build plate with its placement transform.

    ``mesh`` needs only ``.vertices`` (Nx3) for placement math; the slicing path
    additionally requires a real ``trimesh.Trimesh``.
    """

    name: str
    mesh: object
    x: float = 0.0                 # plate position of the pivot (bbox centre)
    y: float = 0.0
    rot_x_deg: float = 0.0
    rot_y_deg: float = 0.0
    rot_z_deg: float = 0.0
    scale: float = 1.0
    _dz: float = field(default=0.0, repr=False)   # drop-to-bed offset (computed)

    def __post_init__(self):
        self.refresh_drop()

    # ---- transform ----------------------------------------------------------

    @property
    def pivot(self) -> np.ndarray:
        v = np.asarray(self.mesh.vertices, dtype=float)
        return (v.min(axis=0) + v.max(axis=0)) / 2.0

    def matrix(self) -> np.ndarray:
        """4×4 placement matrix: centre-on-pivot → scale → Rx → Ry → Rz → translate."""
        s = np.diag([self.scale, self.scale, self.scale, 1.0])
        r = _rot("z", self.rot_z_deg) @ _rot("y", self.rot_y_deg) @ _rot("x", self.rot_x_deg)
        px, py, pz = self.pivot
        return _translate(self.x, self.y, self._dz) @ r @ s @ _translate(-px, -py, -pz)

    def transformed_vertices(self) -> np.ndarray:
        v = np.asarray(self.mesh.vertices, dtype=float)
        m = self.matrix()
        return v @ m[:3, :3].T + m[:3, 3]

    def refresh_drop(self) -> None:
        """Recompute the Z offset so the transformed part rests exactly on the bed."""
        self._dz = 0.0
        self._dz = -float(self.transformed_vertices()[:, 2].min())

    def set_transform(self, **kwargs) -> None:
        """Update any of x / y / rot_*_deg / scale, then re-drop to the bed."""
        for k, v in kwargs.items():
            if not hasattr(self, k) or k.startswith("_"):
                raise AttributeError(f"unknown transform field {k!r}")
            setattr(self, k, float(v))
        self.refresh_drop()

    # ---- queries -------------------------------------------------------------

    def bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        v = self.transformed_vertices()
        return v.min(axis=0), v.max(axis=0)

    def footprint(self) -> Tuple[float, float, float, float]:
        """(xmin, ymin, xmax, ymax) of the placed part."""
        lo, hi = self.bounds()
        return float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1])


def _footprints_overlap(a: ScenePart, b: ScenePart) -> bool:
    ax0, ay0, ax1, ay1 = a.footprint()
    bx0, by0, bx1, by1 = b.footprint()
    return ax0 < bx1 and bx0 < ax1 and ay0 < by1 and by0 < ay1


class SceneModel:
    """The set of placed parts plus placement operations and fit checks."""

    def __init__(self):
        self.parts: List[ScenePart] = []
        self._counter = 0

    # ---- part management -----------------------------------------------------

    def add(self, mesh, name: Optional[str] = None, *, cfg=None,
            at: Optional[Tuple[float, float]] = None) -> ScenePart:
        """Add a mesh, dropped to the bed, at ``at`` or the plate centre."""
        self._counter += 1
        if at is None and cfg is not None:
            bx, by, _ = cfg.machine.build_volume_mm
            at = (bx / 2.0, by / 2.0)
        part = ScenePart(name=name or f"part-{self._counter}", mesh=mesh,
                         x=at[0] if at else 0.0, y=at[1] if at else 0.0)
        self.parts.append(part)
        return part

    def duplicate(self, part: ScenePart) -> ScenePart:
        """Copy a part next to the original (offset by its footprint width + 5 mm)."""
        self._counter += 1
        x0, _, x1, _ = part.footprint()
        dup = replace(part, name=f"{part.name}-copy{self._counter}",
                      x=part.x + (x1 - x0) + 5.0)
        dup.refresh_drop()
        self.parts.append(dup)
        return dup

    def remove(self, part: ScenePart) -> None:
        self.parts.remove(part)

    # ---- fit checks ----------------------------------------------------------

    def issues(self, cfg) -> List[str]:
        """Human-readable placement problems: out-of-volume parts and overlapping
        footprints.

        The lead-out envelope is reserved on **all four sides**: every pass runs a
        ``lead_out_len_mm`` runout along its final heading (SPEC §6.3), and under D13
        headings are unrestricted — the default bidirectional raster leads out toward
        −Y on alternate lines and crosshatch tilts lead-outs into ±X. Conservative for
        a pure one-way fill, but a placement passing this check emits in any mode.
        """
        bx, by, bz = cfg.machine.build_volume_mm
        lead_out = cfg.process.lead_out_len_mm
        out: List[str] = []
        for p in self.parts:
            x0, y0, x1, y1 = p.footprint()
            _, hi = p.bounds()
            if (x0 - lead_out < 0 or y0 - lead_out < 0
                    or x1 + lead_out > bx or y1 + lead_out > by):
                out.append(f"{p.name}: outside the {bx:g}x{by:g} plate "
                           f"(footprint [{x0:.1f},{y0:.1f}]..[{x1:.1f},{y1:.1f}]"
                           f" with the {lead_out:g} mm lead-out envelope on all sides)")
            if float(hi[2]) > bz:
                out.append(f"{p.name}: taller than the {bz:g} mm build height")
        for i, a in enumerate(self.parts):
            for b in self.parts[i + 1:]:
                if _footprints_overlap(a, b):
                    out.append(f"{a.name} and {b.name}: footprints overlap")
        return out

    # ---- worker-thread safety ---------------------------------------------------

    def snapshot(self) -> "SceneModel":
        """Independent copy of the scene (meshes deep-copied where possible).

        The GUI hands a snapshot — never the live scene — to the slice worker, so
        transform edits / add / remove during a slice cannot race the worker
        (the live parts stay fully interactive).
        """
        snap = SceneModel()
        snap._counter = self._counter
        for p in self.parts:
            mesh = p.mesh.copy() if hasattr(p.mesh, "copy") else p.mesh
            snap.parts.append(replace(p, mesh=mesh))
        return snap

    # ---- slicing (trimesh, lazy) ----------------------------------------------

    def placed_mesh(self):
        """One trimesh with every part's placement baked in (for slicing/export).
        Works on copies — the scene's own meshes are never mutated."""
        import trimesh

        if not self.parts:
            raise ValueError("no parts on the plate")
        placed = []
        for p in self.parts:
            m = p.mesh.copy()
            m.apply_transform(p.matrix())
            placed.append(m)
        return placed[0] if len(placed) == 1 else trimesh.util.concatenate(placed)

    def slice_scene(self, cfg):
        """Slice the placed parts exactly as arranged (no re-centring).

        Each part is repaired individually on a COPY (repairing in place would
        invalidate the interactive drop-to-bed state, and repairing the concatenated
        body could weld separate parts), placement is baked in, and the combined
        mesh is re-anchored to Z=0 (repair may drop degenerate geometry and nudge
        the minimum) before the normal planar pipeline slices it.
        """
        import trimesh

        from ..geometry.slicing import slice_model
        from ..geometry.trimesh_backend import TrimeshBackend

        if not self.parts:
            raise ValueError("no parts on the plate")
        backend = TrimeshBackend()
        placed = []
        for p in self.parts:
            m = p.mesh.copy()
            backend.repair(m)                  # never mutate the scene's mesh
            m.apply_transform(p.matrix())
            placed.append(m)
        combined = placed[0] if len(placed) == 1 else trimesh.util.concatenate(placed)
        z_min = float(combined.bounds[0][2])
        if abs(z_min) > 1e-9:
            combined.apply_translation((0.0, 0.0, -z_min))
        return slice_model(backend, combined,
                           cfg.process.layer_height_mm, repair=False)
