"""End-to-end orchestrator. SPEC §3.1, §11.

mesh -> slice -> wedge-constrained fill -> pass plan -> collision -> emit G-code.
Implement across milestones M1-M5.
"""
from __future__ import annotations

from .config import load_config


def slice_mesh(mesh_path: str, config_path: str,
               screener_csv: str | None = None,
               out_path: str | None = None) -> str:
    cfg = load_config(config_path)  # noqa: F841  (validates config early)
    raise NotImplementedError(
        "pipeline.slice_mesh: wire geometry -> fill -> passplan -> collision -> emit "
        "per SPEC §3.1 (milestones M1-M5)."
    )
