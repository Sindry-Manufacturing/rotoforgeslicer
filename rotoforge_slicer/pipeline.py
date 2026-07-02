"""End-to-end orchestrator. SPEC §3.1, §11.

mesh -> slice -> place -> raster/streamline fill -> pass plan -> emit RRF G-code.
M1 wired the geometry stage; M2 adds straight-fill planning + the emitter. Pass
planning collision/curved fill (M4/M5) refine the plan further later.
"""
from __future__ import annotations

from pathlib import Path

from .config import Config, load_config


def slice_geometry(mesh_path: str, config: "str | Config", backend=None,
                   *, place: bool = False):
    """Load + repair + planar-slice a mesh into a SlicedModel (SPEC §3.1).

    ``config`` may be a path or a loaded :class:`Config`. With ``place=True`` the
    model is dropped to the bed and centred in XY (SPEC §6.3). trimesh/shapely are
    imported lazily here, not at package import.
    """
    cfg = config if isinstance(config, Config) else load_config(config)
    if backend is None:
        from .geometry.trimesh_backend import TrimeshBackend

        backend = TrimeshBackend()
    from .geometry.slicing import place_on_bed, slice_model

    mesh = backend.load(mesh_path)
    model = slice_model(backend, mesh, cfg.process.layer_height_mm)
    return place_on_bed(model, cfg) if place else model


def slice_mesh(mesh_path: str, config_path: str,
               screener_csv: str | None = None,
               out_path: str | None = None) -> str:
    """Full pipeline: mesh -> placed slices -> straight raster passes -> RRF G-code.

    Returns the G-code text; also writes it to ``out_path`` when given. A screener
    CSV (optional until M3) selects the operating point; otherwise a single-speed
    fallback is used (SPEC §5.3).
    """
    cfg = load_config(config_path)
    model = slice_geometry(mesh_path, cfg, place=True)

    op = None
    if screener_csv:
        from .process.screener import select_operating_point

        op = select_operating_point(
            screener_csv, mode=cfg.screener.revs_per_mm_mode,
            target=cfg.screener.revs_per_mm_target, tol=cfg.screener.revs_per_mm_tol,
            rpm_min=cfg.spindle.rpm_min, rpm_max=cfg.spindle.rpm_max,
            traverse_target=cfg.screener.traverse_target)

    from .emit.rrf import GCodeEmitter
    from .toolpath.passplan import plan_toolpath

    plan = plan_toolpath(model, cfg, operating_point=op)

    # 2.5D swept-disc + leading-wire collision check (SPEC §4.6); raises on any residual.
    from .toolpath.collision import assert_no_collisions

    assert_no_collisions(plan, cfg)

    gcode = GCodeEmitter(cfg).emit(plan)

    if out_path:
        Path(out_path).write_text(gcode, encoding="utf-8")
    return gcode
