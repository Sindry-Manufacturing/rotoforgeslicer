"""End-to-end orchestrator. SPEC §3.1, §11.

mesh -> slice -> wedge-constrained fill -> pass plan -> collision -> emit G-code.
Implemented across milestones M1-M5. M1 wires the geometry stage
(``slice_geometry``); the fill/plan/emit stages raise until their milestones land.
"""
from __future__ import annotations

from .config import Config, load_config


def slice_geometry(mesh_path: str, config: "str | Config", backend=None):
    """Milestone M1: load + repair + planar-slice a mesh into a SlicedModel.

    ``config`` may be a path to the YAML or an already-loaded :class:`Config`.
    ``backend`` defaults to the trimesh backend (SPEC §3.3). trimesh/shapely are
    pulled lazily here, not at package import.
    """
    cfg = config if isinstance(config, Config) else load_config(config)
    if backend is None:
        from .geometry.trimesh_backend import TrimeshBackend

        backend = TrimeshBackend()
    from .geometry.slicing import slice_model

    mesh = backend.load(mesh_path)
    return slice_model(backend, mesh, cfg.process.layer_height_mm)


def slice_mesh(mesh_path: str, config_path: str,
               screener_csv: str | None = None,
               out_path: str | None = None) -> str:
    cfg = load_config(config_path)
    # M1: geometry stage is live.
    model = slice_geometry(mesh_path, cfg)  # noqa: F841
    # M2-M5: fill -> pass plan -> collision -> emit are still stubbed.
    raise NotImplementedError(
        f"Sliced {len(model.nonempty_layers)}/{len(model)} non-empty layers (M1 done). "
        "Fill -> pass plan -> collision -> G-code emission land in M2-M5 (SPEC §11)."
    )
