"""One-file reopenable project (.rfproj). SPEC §9.

A Python port of PrusaSlicer's 3MF project-container architecture
(src/libslic3r/Format/3mf.cpp, (c) Prusa Research, AGPLv3 — structure ported
with permission of the project license), as a zip of YAML + binary STL instead
of OPC XML. The ported structure:

* one archive = geometry + the FULL flat config snapshot + preset identity —
  "one geometry file + one flat global config file";
* meshes are **embedded** (the as-loaded, untransformed, un-repaired trimesh —
  transforms live in the manifest, never baked into vertices); source paths are
  provenance only, loading never needs them;
* restore order: geometry → config (defaults ← overlay) → preset
  reconciliation → UI (Plater.cpp:1481 order);
* robustness: write to a temp file then ``os.replace`` (delete the partial on
  any failure), a format-version write constant separate from the accept-up-to
  constant, substitution-not-abort on unknown config content.

Archive layout::

    project.yaml    format_version, generator, counter, ui{}, parts[]:
                    {name, mesh: meshes/NNN.stl, source_path, transform{6}}
    meshes/NNN.stl  binary STL, deduplicated (duplicated parts share one entry)
    config.yaml     config: {dotted.key: value} (full snapshot),
                    presets: {machine, material, process}, csv_source_path
    screener.csv    embedded process-window CSV (optional)

No Qt here; trimesh is imported inside functions (lazy-import rule) so the
module stays importable by the light core tests.
"""
from __future__ import annotations

import os
import zipfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import yaml

from ..presets import flatten_config
from .scene import SceneModel, ScenePart

FORMAT_VERSION = 1              # written into every project
FORMAT_VERSION_COMPATIBLE = 1   # highest version this build will open

_GENERATOR = "rotoforge-slicer"


@dataclass
class ProjectData:
    """Everything ``load_project`` restores; the studio wires it into the UI."""

    scene: SceneModel
    config_flat: Dict[str, object]
    selections: Dict[str, str]
    csv_bytes: Optional[bytes]
    csv_source_path: str
    ui: Dict[str, object] = field(default_factory=dict)
    warnings: List[str] = field(default_factory=list)


def save_project(path: "str | Path", scene: SceneModel, cfg, *,
                 csv_path: "str | None" = None,
                 selections: Optional[Dict[str, str]] = None,
                 ui: Optional[Dict[str, object]] = None) -> None:
    """Write the whole session — plate + config + preset identity + screener
    CSV — as one reopenable file. Atomic: a failed save leaves no partial file."""
    path = Path(path)
    flat = flatten_config(cfg)

    # dedupe meshes shared by duplicated parts (scene.duplicate shares the object)
    mesh_entries: Dict[int, str] = {}
    meshes: List[tuple] = []                        # (entry_name, mesh)
    parts_doc = []
    for p in scene.parts:
        key = id(p.mesh)
        if key not in mesh_entries:
            mesh_entries[key] = f"meshes/{len(meshes):03d}.stl"
            meshes.append((mesh_entries[key], p.mesh))
        parts_doc.append({
            "name": p.name,
            "mesh": mesh_entries[key],
            "source_path": getattr(p, "source_path", "") or "",
            "transform": {"x": p.x, "y": p.y,
                          "rot_x_deg": p.rot_x_deg, "rot_y_deg": p.rot_y_deg,
                          "rot_z_deg": p.rot_z_deg, "scale": p.scale},
        })

    manifest = {
        "format_version": FORMAT_VERSION,
        "generator": _GENERATOR,
        "counter": scene._counter,
        "ui": dict(ui or {}),
        "parts": parts_doc,
    }
    config_doc = {
        "config": flat,
        "presets": dict(selections or {}),
        # sticky provenance: the config's csv_path is the original source even
        # when the session works off an extracted temp copy
        "csv_source_path": str(flat.get("screener.csv_path") or csv_path or ""),
    }

    csv_bytes = None
    if csv_path and Path(csv_path).exists():
        csv_bytes = Path(csv_path).read_bytes()

    tmp = path.with_suffix(path.suffix + ".tmp")
    try:
        with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as z:
            z.writestr("project.yaml", yaml.safe_dump(manifest, sort_keys=False))
            z.writestr("config.yaml", yaml.safe_dump(config_doc, sort_keys=True))
            for entry, mesh in meshes:
                z.writestr(entry, _stl_bytes(mesh))
            if csv_bytes is not None:
                z.writestr("screener.csv", csv_bytes)
        os.replace(tmp, path)
    except BaseException:
        try:
            tmp.unlink()
        except OSError:
            pass
        raise


def load_project(path: "str | Path") -> ProjectData:
    """Read a project back. Geometry first, then config; per-part transforms are
    set explicitly (never via ``SceneModel.add`` defaults). Missing optional
    entries are tolerated (a model-only project is legal); a newer
    ``format_version`` refuses with a clear message."""
    path = Path(path)
    warnings: List[str] = []
    with zipfile.ZipFile(path, "r") as z:
        names = set(z.namelist())
        if "project.yaml" not in names:
            raise ValueError(f"{path.name}: not a Rotoforge project "
                             "(no project.yaml in the archive)")
        manifest = yaml.safe_load(z.read("project.yaml")) or {}
        if not isinstance(manifest, dict):
            raise ValueError(f"{path.name}: project.yaml is not a mapping")
        version = int(manifest.get("format_version") or 0)
        if version > FORMAT_VERSION_COMPATIBLE:
            raise ValueError(
                f"{path.name}: saved by a newer rotoforge-slicer "
                f"(format {version} > supported {FORMAT_VERSION_COMPATIBLE}) — "
                "update the application to open it")

        # geometry: one load per unique entry; duplicated parts re-share it
        mesh_cache: Dict[str, object] = {}
        scene = SceneModel()
        for pd in manifest.get("parts") or []:
            entry = pd.get("mesh") or ""
            if entry not in mesh_cache:
                if entry not in names:
                    raise ValueError(f"{path.name}: missing mesh entry {entry!r}")
                mesh_cache[entry] = _mesh_from_stl_bytes(z.read(entry), entry)
            t = pd.get("transform") or {}
            part = ScenePart(
                name=str(pd.get("name") or f"part-{len(scene.parts) + 1}"),
                mesh=mesh_cache[entry],
                x=float(t.get("x", 0.0)), y=float(t.get("y", 0.0)),
                rot_x_deg=float(t.get("rot_x_deg", 0.0)),
                rot_y_deg=float(t.get("rot_y_deg", 0.0)),
                rot_z_deg=float(t.get("rot_z_deg", 0.0)),
                scale=float(t.get("scale", 1.0)),
                source_path=str(pd.get("source_path") or ""))
            scene.parts.append(part)
        scene._counter = int(manifest.get("counter") or len(scene.parts))

        config_flat: Dict[str, object] = {}
        selections: Dict[str, str] = {}
        csv_source_path = ""
        if "config.yaml" in names:
            config_doc = yaml.safe_load(z.read("config.yaml")) or {}
            if not isinstance(config_doc, dict):
                warnings.append("config.yaml is not a mapping; ignored")
                config_doc = {}
            raw_flat = config_doc.get("config")
            config_flat = dict(raw_flat) if isinstance(raw_flat, dict) else {}
            raw_sel = config_doc.get("presets")
            selections = ({k: str(v) for k, v in raw_sel.items() if v}
                          if isinstance(raw_sel, dict) else {})
            csv_source_path = str(config_doc.get("csv_source_path") or "")
        else:
            warnings.append("project has no config.yaml (model-only project); "
                            "current settings kept")

        csv_bytes = z.read("screener.csv") if "screener.csv" in names else None

    return ProjectData(scene=scene, config_flat=config_flat,
                       selections=selections, csv_bytes=csv_bytes,
                       csv_source_path=csv_source_path,
                       ui=dict(manifest.get("ui") or {}), warnings=warnings)


# ---- mesh <-> STL bytes ----------------------------------------------------------

def _stl_bytes(mesh) -> bytes:
    """Binary STL of the pristine mesh (untransformed — transforms are manifest
    data; baking them here would double-apply on load)."""
    data = mesh.export(file_type="stl")
    return data if isinstance(data, bytes) else str(data).encode("utf-8")


def _mesh_from_stl_bytes(data: bytes, entry: str):
    """Load an embedded STL with the same guards as adding a mesh file
    (``TrimeshBackend.load``), so project meshes take the identical path."""
    from ..geometry.trimesh_backend import TrimeshBackend

    return TrimeshBackend().load_bytes(data, file_type="stl", label=entry)
