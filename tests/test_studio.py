"""Studio viewport + window: pyvista actors, toolpath rendering, head posing, and
offscreen Qt construction. Skipped cleanly where pyvista / pyvistaqt are absent.
"""
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

pv = pytest.importorskip("pyvista")

from rotoforge_slicer.config import Config  # noqa: E402
from rotoforge_slicer.studio.scene import SceneModel  # noqa: E402
from rotoforge_slicer.studio.simulate import build_timeline, state_at  # noqa: E402
from rotoforge_slicer.studio.viewport import BuildPlateScene  # noqa: E402
from rotoforge_slicer.toolpath.passplan import LayerPlan, Pass, ToolpathPlan  # noqa: E402
from rotoforge_slicer.toolpath.segments import build_segments  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


class _Tetra:
    """Minimal mesh stub with vertices AND faces so the viewport can polygonize it."""

    vertices = np.array([(0, 0, 0), (10, 0, 0), (0, 10, 0), (0, 0, 10)], dtype=float)
    faces = np.array([(0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3)])


def _plan():
    ps = [Pass(start=(190, 100), end=(190, 130), z=0.06, a_deg=0.0, rpm=5000,
               traverse_mm_min=120.0, e_per_path_mm=1.0),
          Pass(start=(191, 100), end=(191, 130), z=0.06, a_deg=0.0, rpm=5000,
               traverse_mm_min=120.0, e_per_path_mm=1.0)]
    return ToolpathPlan([LayerPlan(0, 0.06, ps)], 5000, 120.0, 120.0)


def _scene():
    return BuildPlateScene(off_screen=True)


def test_draw_plate_and_sync_parts():
    view = _scene()
    view.draw_plate(Config())
    n_static = len(view.plotter.renderer.actors)
    assert n_static >= 3                                     # plate + volume + home ref

    scene = SceneModel()
    a = scene.add(_Tetra(), cfg=Config())
    b = scene.add(_Tetra(), at=(60, 60))
    view.sync_parts(scene.parts, selected=b)
    assert len(view._part_actors) == 2
    scene.remove(a)
    view.sync_parts(scene.parts, selected=b)
    assert len(view._part_actors) == 1                       # stale actor dropped


def test_show_toolpath_kind_actors_toggles_and_scrubber():
    segs = build_segments(_plan(), Config())
    view = _scene()
    view.show_toolpath(segs)
    n_all = len(view._path_actors)
    assert n_all == len({s.kind for s in segs})              # one actor per kind

    view.show_toolpath(segs, enabled={"deposition"})
    assert len(view._path_actors) == 1                       # toggle filter

    view.show_toolpath(segs, enabled={"deposition"}, upto_layer=-1)
    assert len(view._path_actors) == 0                       # scrubber below all layers

    view.clear_toolpath()
    assert view._path_actors == []


def test_update_head_poses_disc_and_heading_arrow():
    cfg = Config()
    plan = _plan()
    tl = build_timeline(build_segments(plan, cfg), plan, cfg)
    dep = next(e for e in tl if e.kind == "deposition")
    state = state_at(tl, (dep.t0 + dep.t1) / 2, cfg.c_axis)

    view = _scene()
    view.update_head(state, cfg)
    disc, arrow = view._head_actors
    assert disc.GetPosition() == pytest.approx((state.x, state.y, state.z))
    assert arrow.GetOrientation()[2] == pytest.approx(state.wheel_heading_deg)  # +Y = 90°
    # review fix: the wheel is a VERTICAL disc (plane contains heading + Z), rim at
    # the contact point — its bounds span ~wheel_diameter in Z, upward from contact.
    assert disc.GetOrientation()[2] == pytest.approx(state.wheel_heading_deg)
    zb0, zb1 = disc.GetBounds()[4], disc.GetBounds()[5]
    assert zb1 - zb0 == pytest.approx(cfg.process.wheel_diameter_mm, abs=1.0)
    assert zb0 == pytest.approx(state.z, abs=0.5)
    view.clear_head()
    assert view._head_actors == []


def test_show_collisions_marks_points():
    from rotoforge_slicer.toolpath.collision import Collision

    view = _scene()
    view.show_collisions([Collision(0, 0.06, "wire", (12.0, 18.0), 5.0, "test")])
    assert len(view._path_actors) == 1


def _run_gl_subprocess(code, tmp_path, *args):
    """Run studio GUI code in a subprocess with an isolated preset data dir.
    NOT under QT_QPA_PLATFORM=offscreen: VTK's QtInteractor needs a real GL
    context and hard-crashes (no valid pixel format) on the offscreen platform.
    Windows are constructed but never shown; on truly headless machines (CI)
    the GL/display init fails inside VTK — skip there, don't fail."""
    env = dict(os.environ, ROTOFORGE_DATA_DIR=str(tmp_path / "data"),
               PYTHONPATH=str(ROOT))    # script files don't put the cwd on sys.path
    script = tmp_path / "studio_check.py"
    script.write_text(code, encoding="utf-8")
    r = subprocess.run([sys.executable, str(script), *map(str, args)],
                       cwd=str(ROOT), env=env,
                       capture_output=True, text=True, timeout=300)
    if r.returncode != 0 and any(
            key in (r.stderr or "") for key in
            ("pixel format", "OpenGL", "xcb", "DISPLAY", "display")):
        pytest.skip(f"no GL/display for QtInteractor here: {r.stderr[-200:]!r}")
    assert r.returncode == 0, f"stdout={r.stdout!r}\nstderr={r.stderr!r}"
    return r.stdout


def test_studio_window_constructs_offscreen(tmp_path):
    pytest.importorskip("pyvistaqt")
    pytest.importorskip("PySide6")
    code = (
        "from PySide6 import QtWidgets\n"
        "from rotoforge_slicer.studio.app import _build_studio_window\n"
        "app = QtWidgets.QApplication([])\n"
        "w = _build_studio_window()\n"
        "assert w.windowTitle() == 'Rotoforge Studio'\n"
        "assert w.tabs.count() == 2\n"
        "assert len(w.toggles) == 5\n"
        "assert w.part_list.count() == 0 and not w.btn_play.isEnabled()\n"
        "assert w.f_mode.count() == 4\n"          # raster/streamline/contour/outline
        "assert len(w.adv_widgets) >= 12\n"       # advanced parameters exposed
        "assert w.f_loops is not None and w.dims_lbl is not None\n"
        "assert w.layer_range is not None and not w.layer_range.isVisible()\n"
        "assert w.layer_range.clamp(7, 2, 0, 5) == (2, 5)\n"   # order + clamp
        "assert w.move_slider is not None and w.shells_cb is not None\n"
        "assert w.btn_arrange is not None and w.arr_spacing.value() == 30.0\n"
        "assert set(w.preset_combos) == {'machine', 'material', 'process'}\n"
        "for c in w.preset_combos.values():\n"
        "    assert c.currentText() == '- default -'\n"
        "assert w.btn_open_project is not None and w.btn_save_project is not None\n"
        "w.close()\n"
        "print('STUDIO_OK')\n"
    )
    out = _run_gl_subprocess(code, tmp_path)
    assert "STUDIO_OK" in out


def test_studio_presets_and_project_round_trip(tmp_path):
    """End-to-end through the real window: off-grid config values survive the
    lossy widget round trip (changed-only rule); a material edit survives a
    process preset switch (the _sync_bundle invariant); save project → open in
    a fresh window restores scene transforms, widget state, presets (clean
    reconciliation), the embedded CSV, and invalidates stale preview state."""
    pytest.importorskip("pyvistaqt")
    pytest.importorskip("PySide6")
    code = """
import sys
from pathlib import Path

import trimesh
from PySide6 import QtWidgets

from rotoforge_slicer.studio.app import _build_studio_window
from rotoforge_slicer.studio.project import save_project

tmp = Path(sys.argv[1])
app = QtWidgets.QApplication([])
w = _build_studio_window()

# --- off-grid cfg value survives widgets (changed-only write-back) ---
w.cfg.process.lead_in_len_mm = 0.3          # widget floor is 0.5
w._load_params_from_cfg()
assert w.adv_widgets["process.lead_in_len_mm"].value() == 0.5   # clamped DISPLAY
w._apply_params()
assert w.cfg.process.lead_in_len_mm == 0.3, "untouched widget must not clobber cfg"

# --- plate + edits ---
stl = tmp / "box.stl"
trimesh.creation.box(extents=(10.0, 20.0, 5.0)).export(stl)
w.add_mesh_file(str(stl))
assert w.scene.parts[0].source_path == str(stl)
w.scene.parts[0].set_transform(x=123.4, y=56.7, rot_z_deg=30.0, scale=1.25)
w.f_lh.setValue(0.2)
w.f_mode.setCurrentText("streamline")

csv = tmp / "window.csv"
csv.write_text("header\\n1,2\\n", encoding="utf-8")
w.csv_path = str(csv)                       # what _open_csv does, sans dialog
w.cfg.screener.csv_path = str(csv)
w.bundle.capture(w.cfg)

# --- material edit must survive a PROCESS preset activation (sync invariant) ---
w.cfg.process.bed_temp_c = 151.5            # what screener-dialog Apply does
w.bundle.capture(w.cfg)
w._sync_bundle()
w.bundle.collections["process"].save_current("proj proc")
w.bundle.save_selections()
w._refresh_preset_combos()
idx = w.preset_combos["process"].findData("proj proc")
assert idx >= 0
w._on_preset_activated("process", idx)      # re-select: discards process edits only
assert w.cfg.process.bed_temp_c == 151.5, "material dirty state lost on process switch"
assert w.cfg.process.layer_height_mm == 0.2
assert w.bundle.collections["material"].is_dirty()

proj = tmp / "job.rfproj"
save_project(proj, w.scene, w.cfg, csv_path=w.csv_path,
             selections=w.bundle.selections(),
             ui={"arrange_spacing_mm": 55.0})

# --- fresh window, open the project ---
w2 = _build_studio_window()
w2.preview = object()                       # stale artifacts must be dropped
w2.btn_save.setEnabled(True)
w2.open_project_file(str(proj))
assert w2.preview is None and not w2.btn_save.isEnabled()
assert len(w2.scene.parts) == 1
p = w2.scene.parts[0]
assert (p.x, p.y, p.rot_z_deg, p.scale) == (123.4, 56.7, 30.0, 1.25)
assert w2.selected is p
assert w2.cfg.process.layer_height_mm == 0.2
assert w2.f_lh.value() == 0.2
assert w2.cfg.fill.mode == "streamline" and w2.f_mode.currentText() == "streamline"
assert w2.cfg.process.lead_in_len_mm == 0.3, "off-grid value must survive save/load"
assert w2.cfg.process.bed_temp_c == 151.5
assert w2.arr_spacing.value() == 55.0
assert w2.bundle.collections["process"].selected == "proj proc"
assert not w2.bundle.collections["process"].is_dirty(), "should reconcile clean"
assert w2.csv_path and Path(w2.csv_path).read_bytes() == csv.read_bytes(), \\
    "embedded CSV must be restored"
assert "(embedded)" in w2.csv_lbl.text()
assert w2.cfg.screener.csv_path == str(csv), "provenance path stays in cfg"

# --- model-only project keeps current settings + presets ---
import zipfile
entries = {}
with zipfile.ZipFile(proj) as z:
    entries = {n: z.read(n) for n in z.namelist() if n != "config.yaml"}
mproj = tmp / "model_only.rfproj"
with zipfile.ZipFile(mproj, "w") as z:
    for n, b in entries.items():
        z.writestr(n, b)
before_lh = w2.cfg.process.layer_height_mm
before_sel = dict(w2.bundle.selections())
w2.open_project_file(str(mproj))
assert w2.cfg.process.layer_height_mm == before_lh, "model-only load reset the config"
assert w2.bundle.selections() == before_sel, "model-only load reset the presets"
assert len(w2.scene.parts) == 1

w.close(); w2.close()
print('ROUNDTRIP_OK')
"""
    out = _run_gl_subprocess(code, tmp_path, tmp_path)
    assert "ROUNDTRIP_OK" in out
