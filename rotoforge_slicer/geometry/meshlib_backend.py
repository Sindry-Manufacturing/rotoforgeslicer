"""Optional meshlib geometry backend. SPEC §3.2/§3.3.  [stub]

meshlib offers higher-robustness repair (voxel remesh, tunnel fixing) but is
DUAL-LICENSED: free for non-commercial/education, a paid license is required for
commercial use. Rotoforge has a commercial entity behind it, so this backend is
NOT a default dependency — it is exposed behind the GeometryBackend interface so it
can be enabled only if a license is held (SPEC §3.2 line 139, §3.3 line 146).

Left as a stub on purpose; implement against ``meshlib.mrmeshpy`` when/if a license
is in place, keeping the heavy import lazy (CLAUDE.md) like TrimeshBackend.
"""
from __future__ import annotations

from typing import Sequence, Tuple

from .backend import GeometryBackend

_REF = "MeshLibBackend: optional dual-licensed meshlib backend, SPEC §3.2/§3.3"


class MeshLibBackend(GeometryBackend):
    def load(self, path: str):
        raise NotImplementedError(_REF)

    def repair(self, mesh):
        raise NotImplementedError(_REF)

    def bounds(self, mesh) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        raise NotImplementedError(_REF)

    def slice(self, mesh, z_heights: Sequence[float]) -> list:
        raise NotImplementedError(_REF)
