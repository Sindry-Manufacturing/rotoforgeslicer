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


def euler_zyx_deg_from_matrix(r: np.ndarray) -> Tuple[float, float, float]:
    """(rot_x, rot_y, rot_z) in degrees such that Rz@Ry@Rx == ``r`` — the inverse of
    this module's rotation composition, so an arbitrary rotation (lay-flat, a
    world-frame 90° turn) can be written back into a part's transform fields."""
    r20 = min(1.0, max(-1.0, float(r[2, 0])))
    ry = math.asin(-r20)
    if abs(math.cos(ry)) > 1e-9:
        rx = math.atan2(r[2, 1], r[2, 2])
        rz = math.atan2(r[1, 0], r[0, 0])
    else:                                   # gimbal: ry = ±90°, fold everything into rx
        rz = 0.0
        sign = 1.0 if r20 < 0 else -1.0     # r20 = -sin(ry)
        rx = math.atan2(sign * r[0, 1], r[1, 1])
    return math.degrees(rx), math.degrees(ry), math.degrees(rz)


def _align_rotation(from_vec, to_vec) -> np.ndarray:
    """3×3 rotation taking unit vector ``from_vec`` onto ``to_vec`` (Rodrigues)."""
    a = np.asarray(from_vec, dtype=float)
    b = np.asarray(to_vec, dtype=float)
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v = np.cross(a, b)
    s = float(np.linalg.norm(v))
    c = float(np.dot(a, b))
    if s < 1e-12:
        if c > 0:
            return np.eye(3)
        # antiparallel: 180° about any axis perpendicular to a
        axis = np.cross(a, (1.0, 0.0, 0.0))
        if np.linalg.norm(axis) < 1e-9:
            axis = np.cross(a, (0.0, 1.0, 0.0))
        axis = axis / np.linalg.norm(axis)
        return 2.0 * np.outer(axis, axis) - np.eye(3)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k * ((1.0 - c) / (s * s))


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
    source_path: str = ""          # provenance only (project files embed the mesh)
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

    def rotation(self) -> np.ndarray:
        """The 3×3 rotation of the current transform (Rz@Ry@Rx, scale-free)."""
        r = (_rot("z", self.rot_z_deg) @ _rot("y", self.rot_y_deg)
             @ _rot("x", self.rot_x_deg))
        return r[:3, :3]

    def _apply_rotation(self, r3: np.ndarray) -> None:
        """Compose a world-frame rotation onto the part and re-drop."""
        rx, ry, rz = euler_zyx_deg_from_matrix(r3 @ self.rotation())
        self.set_transform(rot_x_deg=rx, rot_y_deg=ry, rot_z_deg=rz)

    def rotate_world(self, axis: str, deg: float) -> None:
        """Rotate the part about a WORLD axis (what the quick-rotate buttons mean:
        turning an already-tumbled part 90° about world Z must not re-tumble it,
        which naively incrementing the intrinsic euler fields would do)."""
        self._apply_rotation(_rot(axis, deg)[:3, :3])

    def lay_flat(self) -> None:
        """Rotate the part so its largest flat face rests on the bed.

        Convex-hull facets of the CURRENT orientation are clustered by outward
        normal; the cluster with the greatest total area is rotated to face −Z
        (then the part re-drops). The standard 'lay flat' of desktop slicers —
        deterministic, and safe for any watertight-ish mesh (hull only).
        """
        from scipy.spatial import ConvexHull

        v = self.transformed_vertices()
        hull = ConvexHull(v)
        # cluster facets by (rounded) outward normal, but align to the exact
        # AREA-WEIGHTED mean normal — aligning to the rounded key itself would tilt
        # the face by up to the rounding step.
        clusters: dict = {}
        for simplex, eq in zip(hull.simplices, hull.equations):
            a, b, c = v[simplex[0]], v[simplex[1]], v[simplex[2]]
            area = 0.5 * float(np.linalg.norm(np.cross(b - a, c - a)))
            key = tuple(round(float(x), 3) for x in eq[:3])
            acc = clusters.setdefault(key, [0.0, np.zeros(3)])
            acc[0] += area
            acc[1] += area * eq[:3]
        best_area, mean_n = max(clusters.values(), key=lambda kv: kv[0])
        self._apply_rotation(_align_rotation(mean_n, (0.0, 0.0, -1.0)))

    def size_mm(self) -> Tuple[float, float, float]:
        """Placed bounding-box dimensions (for the GUI's dimensions readout)."""
        lo, hi = self.bounds()
        return tuple(float(d) for d in (hi - lo))

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

    # ---- auto-arrange (PrusaSlicer-structure port; studio/arrange.py) -----------

    def arrange(self, cfg, spacing_mm: float = 30.0) -> List[str]:
        """Auto-place every part on the plate: convex-hull footprints inflated by
        half the spacing, bed inset by the lead-out envelope (a valid arrangement
        passes ``issues()`` by construction). Default spacing covers the 50 mm
        wheel body radius + margin, so the disc working one part clears its
        neighbours. Returns the names of parts that did not fit."""
        from shapely.geometry import MultiPoint

        from .arrange import ArrangeItem, RectangleBed, arrange as _arrange

        if not self.parts:
            return []
        bx, by, _ = cfg.machine.build_volume_mm
        bed = RectangleBed(bx, by, inset_mm=cfg.process.lead_out_len_mm)
        items = []
        for p in self.parts:
            hull = MultiPoint([tuple(q[:2]) for q in
                               p.transformed_vertices()]).convex_hull
            items.append(ArrangeItem(outline=hull, inflation_mm=spacing_mm / 2.0,
                                     key=p))
        unplaced = _arrange(items, [], bed)
        for it in items:
            if it.translation is not None:
                it.key.set_transform(x=it.key.x + it.translation[0],
                                     y=it.key.y + it.translation[1])
        return [it.key.name for it in unplaced]

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
