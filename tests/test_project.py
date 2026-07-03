"""Project save/load (.rfproj, the 3MF-architecture port): scene fidelity,
config snapshot, CSV embedding, version gate, atomic save. Needs trimesh for
mesh embedding (skipped where absent, same as the scene slicing tests)."""
import zipfile

import pytest
import yaml

trimesh = pytest.importorskip("trimesh")

from rotoforge_slicer.config import Config
from rotoforge_slicer.presets import flatten_config
from rotoforge_slicer.studio.project import (
    FORMAT_VERSION, FORMAT_VERSION_COMPATIBLE, load_project, save_project,
)
from rotoforge_slicer.studio.scene import SceneModel, ScenePart


def _box(extents=(10.0, 20.0, 5.0)):
    return trimesh.creation.box(extents=extents)


def _scene(cfg):
    scene = SceneModel()
    a = scene.add(_box(), name="bracket", cfg=cfg)
    a.source_path = r"C:\parts\bracket.stl"
    a.set_transform(x=100.0, y=60.0, rot_z_deg=33.3333333333333, scale=1.5)
    dup = scene.duplicate(a)                      # shares a's mesh object
    b = scene.add(_box((6.0, 6.0, 12.0)), name="pin", cfg=cfg)
    b.set_transform(x=200.0, y=150.0, rot_x_deg=90.0)
    return scene, (a, dup, b)


def test_round_trip_scene_config_and_ui(tmp_path):
    cfg = Config()
    cfg.process.layer_height_mm = 0.21
    cfg.fill.mode = "streamline"
    cfg.c_axis.a_max_deg = 171.25
    cfg.screener.csv_path = r"C:\data\window.csv"
    scene, parts = _scene(cfg)

    p = tmp_path / "job.rfproj"
    save_project(p, scene, cfg, selections={"process": "my proc"},
                 ui={"arrange_spacing_mm": 42.0})
    data = load_project(p)

    # scene: order, names, exact transforms, counter, provenance
    assert [q.name for q in data.scene.parts] == [q.name for q in scene.parts]
    for orig, back in zip(scene.parts, data.scene.parts):
        for f in ("x", "y", "rot_x_deg", "rot_y_deg", "rot_z_deg", "scale"):
            assert getattr(back, f) == getattr(orig, f), f
    assert data.scene._counter == scene._counter
    assert data.scene.parts[0].source_path == r"C:\parts\bracket.stl"

    # duplicated parts share ONE mesh entry and re-share one object on load
    with zipfile.ZipFile(p) as z:
        mesh_entries = [n for n in z.namelist() if n.startswith("meshes/")]
    assert len(mesh_entries) == 2, "3 parts, 2 unique meshes -> 2 entries"
    assert data.scene.parts[0].mesh is data.scene.parts[1].mesh

    # config snapshot: full flat dict, spot keys from several sections
    assert data.config_flat["process.layer_height_mm"] == 0.21
    assert data.config_flat["fill.mode"] == "streamline"
    assert data.config_flat["c_axis.a_max_deg"] == 171.25
    assert set(flatten_config(cfg)) == set(data.config_flat)
    assert data.selections == {"process": "my proc"}
    assert data.csv_source_path == r"C:\data\window.csv"
    assert data.ui["arrange_spacing_mm"] == 42.0


def test_mesh_embedding_keeps_placement_geometry(tmp_path):
    """Pivot = bbox centre of the stored mesh; the STL round trip may quantize
    to float32 but must keep the bbox (and a rotated part's placement) stable,
    and be exactly idempotent from the second save on."""
    cfg = Config()
    scene = SceneModel()
    mesh = _box()
    mesh.apply_transform(trimesh.transformations.rotation_matrix(
        0.3, (0.2, 0.5, 1.0)))                    # off-grid float32 vertices
    part = scene.add(mesh, name="rot", cfg=cfg)
    part.set_transform(x=90.0, y=70.0, rot_z_deg=12.5)

    p1 = tmp_path / "a.rfproj"
    save_project(p1, scene, cfg)
    back1 = load_project(p1)
    import numpy as np

    b0 = np.asarray(part.bounds())
    b1 = np.asarray(back1.scene.parts[0].bounds())
    assert np.allclose(b0, b1, atol=1e-4), "bbox/pivot must survive embedding"

    p2 = tmp_path / "b.rfproj"
    save_project(p2, back1.scene, cfg)
    back2 = load_project(p2)
    assert np.array_equal(
        np.asarray(back1.scene.parts[0].mesh.vertices),
        np.asarray(back2.scene.parts[0].mesh.vertices)), \
        "second round trip must be exact (no accumulating drift)"


def test_csv_embedded_and_missing_tolerated(tmp_path):
    cfg = Config()
    scene = SceneModel()
    scene.add(_box(), cfg=cfg)
    csv = tmp_path / "window.csv"
    csv.write_text("RPM,traverse\n1,2\n", encoding="utf-8")
    cfg.screener.csv_path = str(csv)

    p = tmp_path / "with.rfproj"
    save_project(p, scene, cfg, csv_path=str(csv))
    data = load_project(p)
    assert data.csv_bytes == csv.read_bytes()
    assert data.csv_source_path == str(csv)

    # CSV path recorded but file vanished by save time -> no embed, still saves
    cfg2 = Config()
    cfg2.screener.csv_path = str(tmp_path / "gone.csv")
    p2 = tmp_path / "without.rfproj"
    save_project(p2, scene, cfg2, csv_path=str(tmp_path / "gone.csv"))
    data2 = load_project(p2)
    assert data2.csv_bytes is None
    assert data2.csv_source_path == str(tmp_path / "gone.csv")


def test_version_gate_refuses_newer(tmp_path):
    cfg = Config()
    scene = SceneModel()
    scene.add(_box(), cfg=cfg)
    p = tmp_path / "new.rfproj"
    save_project(p, scene, cfg)
    assert FORMAT_VERSION <= FORMAT_VERSION_COMPATIBLE

    # rewrite the manifest as a future version
    with zipfile.ZipFile(p) as z:
        entries = {n: z.read(n) for n in z.namelist()}
    manifest = yaml.safe_load(entries["project.yaml"])
    manifest["format_version"] = FORMAT_VERSION_COMPATIBLE + 1
    entries["project.yaml"] = yaml.safe_dump(manifest).encode()
    with zipfile.ZipFile(p, "w") as z:
        for n, b in entries.items():
            z.writestr(n, b)
    with pytest.raises(ValueError, match="newer"):
        load_project(p)


def test_not_a_project_and_corrupt_zip_error_cleanly(tmp_path):
    plain = tmp_path / "plain.rfproj"
    with zipfile.ZipFile(plain, "w") as z:
        z.writestr("readme.txt", "hi")
    with pytest.raises(ValueError, match="not a Rotoforge project"):
        load_project(plain)

    garbage = tmp_path / "garbage.rfproj"
    garbage.write_bytes(b"\x00\x01\x02 nope")
    with pytest.raises(zipfile.BadZipFile):
        load_project(garbage)


def test_failed_save_leaves_no_partial_file(tmp_path):
    cfg = Config()
    scene = SceneModel()

    class _Boom:
        vertices = [[0, 0, 0]]

        def export(self, file_type):
            raise RuntimeError("export exploded")

    scene.parts.append(ScenePart(name="boom", mesh=_Boom()))
    p = tmp_path / "fail.rfproj"
    with pytest.raises(RuntimeError, match="export exploded"):
        save_project(p, scene, cfg)
    assert not p.exists()
    assert list(tmp_path.glob("*.tmp")) == []


def test_model_only_project_is_legal(tmp_path):
    """A project whose config.yaml is missing loads with a warning (the 3mf
    'model-only export' semantics)."""
    cfg = Config()
    scene = SceneModel()
    scene.add(_box(), cfg=cfg)
    p = tmp_path / "m.rfproj"
    save_project(p, scene, cfg)
    with zipfile.ZipFile(p) as z:
        entries = {n: z.read(n) for n in z.namelist() if n != "config.yaml"}
    with zipfile.ZipFile(p, "w") as z:
        for n, b in entries.items():
            z.writestr(n, b)
    data = load_project(p)
    assert data.config_flat == {} and data.selections == {}
    assert any("config" in w for w in data.warnings)
    assert len(data.scene.parts) == 1
