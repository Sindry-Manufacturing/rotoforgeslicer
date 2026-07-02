"""pyvista rendering for the studio: plate, parts, toolpath, simulated head. SPEC §9.

``BuildPlateScene`` wraps any pyvista plotter (the GUI passes a pyvistaqt
``QtInteractor``; tests pass an off-screen ``pv.Plotter``) and owns the actors:

* the build plate + volume outline + the +Y home reference arrow (axis zero, D13),
* one mesh actor per placed part (selection highlighted),
* the tagged toolpath as one polyline actor per segment kind, colored with the
  same ``KIND_COLOR`` map as the matplotlib viewer (U2),
* the simulated head: a wheel disc + the leading-wire heading arrow, posed from a
  ``simulate.SimState`` each frame (moved via actor transforms — no re-meshing).

pyvista / numpy are imported lazily so the light core stays import-cheap.
"""
from __future__ import annotations

from typing import Dict, List, Optional

PART_COLOR = "#7f9dbf"
PART_SELECTED_COLOR = "#e8862d"
PLATE_COLOR = "#3b4252"
HOME_COLOR = "#107C10"


class BuildPlateScene:
    """Owns and updates the studio's pyvista actors on one plotter."""

    def __init__(self, plotter=None, *, off_screen: bool = True):
        import pyvista as pv

        self.pv = pv
        self.plotter = plotter if plotter is not None else pv.Plotter(off_screen=off_screen)
        self._part_actors: Dict[int, object] = {}     # id(part) -> actor
        self._path_actors: List[object] = []
        self._head_actors: List[object] = []

    # ---- static scenery -------------------------------------------------------

    def draw_plate(self, cfg) -> None:
        """Build plate, volume outline, and the +Y home-heading reference arrow."""
        import math

        pv = self.pv
        bx, by, bz = cfg.machine.build_volume_mm
        plate = pv.Plane(center=(bx / 2, by / 2, -0.05), direction=(0, 0, 1),
                         i_size=bx, j_size=by, i_resolution=10, j_resolution=10)
        self.plotter.add_mesh(plate, color=PLATE_COLOR, opacity=0.55,
                              show_edges=True, edge_color="#5a6578", name="plate")
        box = pv.Box(bounds=(0, bx, 0, by, 0, bz))
        self.plotter.add_mesh(box, style="wireframe", color="#5a6578",
                              opacity=0.35, name="volume")
        # +Y home reference (the axis zero; D13 — no deposition meaning)
        h = math.radians(cfg.c_axis.home_heading_deg)
        arrow = pv.Arrow(start=(bx / 2, by / 2, 0.5),
                         direction=(math.cos(h), math.sin(h), 0.0),
                         scale=min(bx, by) * 0.12)
        self.plotter.add_mesh(arrow, color=HOME_COLOR, name="home-ref")
        self.plotter.show_axes()

    # ---- parts ---------------------------------------------------------------

    def _part_polydata(self, part):
        import numpy as np

        pv = self.pv
        verts = part.transformed_vertices()
        faces = getattr(part.mesh, "faces", None)
        if faces is None or len(faces) == 0:          # stub meshes: point cloud
            return pv.PolyData(verts)
        f = np.asarray(faces, dtype=np.int64)
        cells = np.hstack([np.full((len(f), 1), 3, dtype=np.int64), f])
        return pv.PolyData(verts, cells)

    def sync_parts(self, parts, selected=None) -> None:
        """Make the part actors match the scene: add / retransform / recolor / drop."""
        alive = set()
        for part in parts:
            key = id(part)
            alive.add(key)
            color = PART_SELECTED_COLOR if part is selected else PART_COLOR
            if key in self._part_actors:
                self.plotter.remove_actor(self._part_actors[key])
            self._part_actors[key] = self.plotter.add_mesh(
                self._part_polydata(part), color=color, smooth_shading=True,
                name=f"part-{key}")
        for key in list(self._part_actors):
            if key not in alive:
                self.plotter.remove_actor(self._part_actors.pop(key))

    # ---- toolpath --------------------------------------------------------------

    def show_toolpath(self, segments, enabled=None, upto_layer: Optional[int] = None) -> None:
        """Render tagged segments, one colored polyline actor per kind (U2 semantics:
        ``enabled`` filters by viewer-toggle name, ``upto_layer`` by the scrubber)."""
        import numpy as np

        from ..toolpath.segments import KIND_COLOR, TOGGLE_KINDS, TOGGLE_ORDER

        self.clear_toolpath()
        shown = set()
        for name in TOGGLE_ORDER:
            if enabled is None or name in set(enabled):
                shown.update(TOGGLE_KINDS[name])

        by_kind: Dict[object, list] = {}
        for s in segments:
            if s.kind not in shown:
                continue
            if upto_layer is not None and s.layer_index is not None \
                    and s.layer_index > upto_layer:
                continue
            by_kind.setdefault(s.kind, []).append(s)

        for kind, segs in by_kind.items():
            pts = np.empty((2 * len(segs), 3))
            for i, s in enumerate(segs):
                pts[2 * i] = s.start
                pts[2 * i + 1] = s.end
            lines = np.hstack([
                np.full((len(segs), 1), 2, dtype=np.int64),
                np.arange(2 * len(segs), dtype=np.int64).reshape(-1, 2)])
            poly = self.pv.PolyData(pts, lines=lines)
            width = 4 if kind.value == "deposition" else 2
            self._path_actors.append(self.plotter.add_mesh(
                poly, color=KIND_COLOR[kind], line_width=width,
                name=f"toolpath-{kind.value}"))

    def clear_toolpath(self) -> None:
        for a in self._path_actors:
            self.plotter.remove_actor(a)
        self._path_actors = []

    def show_collisions(self, collisions) -> None:
        """Red markers at collision points (SPEC §4.6 validation overlay)."""
        import numpy as np

        pts = np.array([(c.at[0], c.at[1], c.z) for c in collisions], dtype=float)
        if len(pts):
            self._path_actors.append(self.plotter.add_mesh(
                self.pv.PolyData(pts), color="#e01010", point_size=14,
                render_points_as_spheres=True, name="collisions"))

    # ---- simulated head ---------------------------------------------------------

    def ensure_head(self, cfg) -> None:
        """Create the head actors once: the wheel (the collision body, SPEC §1.5) and
        the leading-wire heading arrow. Posed per-frame in :meth:`update_head`.

        The wheel is a **vertical** disc whose plane contains the travel direction
        and Z (the axle is horizontal, perpendicular to travel). In the un-oriented
        source geometry the heading is +X, so the axle points +Y and the disc centre
        sits one radius above the origin — the rim touches the actor origin, which
        :meth:`update_head` places at the contact point. A pure Z rotation by the
        wheel heading then poses it correctly."""
        if self._head_actors:
            return
        pv = self.pv
        r = cfg.process.wheel_diameter_mm / 2.0
        disc = pv.Cylinder(center=(0, 0, r), direction=(0, 1, 0),
                           radius=r, height=1.2)
        arrow = pv.Arrow(start=(0, 0, 0), direction=(1, 0, 0), scale=r * 0.9)
        self._head_actors = [
            self.plotter.add_mesh(disc, color="#d8b13a", opacity=0.45, name="head-disc"),
            self.plotter.add_mesh(arrow, color="#c0392b", name="head-arrow"),
        ]

    def update_head(self, state, cfg) -> None:
        """Pose the head at a ``SimState``: the wheel's rim at the contact point,
        its plane and the wire arrow along the wheel heading (recovered from the
        commanded A — you can watch the C axis track the tangent and unwind
        airborne). Both source geometries are heading-+X at the origin, so one Z
        rotation orients them (the disc's centre offset is along Z, unaffected)."""
        self.ensure_head(cfg)
        disc, arrow = self._head_actors
        disc.SetPosition(state.x, state.y, state.z)
        disc.SetOrientation(0.0, 0.0, state.wheel_heading_deg)
        arrow.SetPosition(state.x, state.y, state.z)
        arrow.SetOrientation(0.0, 0.0, state.wheel_heading_deg)

    def clear_head(self) -> None:
        for a in self._head_actors:
            self.plotter.remove_actor(a)
        self._head_actors = []

    # ---- camera ------------------------------------------------------------------

    def reset_camera(self) -> None:
        self.plotter.reset_camera()
