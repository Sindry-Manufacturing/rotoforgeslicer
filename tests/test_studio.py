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


def test_studio_window_constructs_offscreen():
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
        "w.close()\n"
        "print('STUDIO_OK')\n"
    )
    # NOT run under QT_QPA_PLATFORM=offscreen: VTK's QtInteractor needs a real GL
    # context and hard-crashes (no valid pixel format) on the offscreen platform.
    # The window is constructed but never shown. On truly headless machines (CI)
    # the GL/display init fails inside VTK — skip there, don't fail.
    r = subprocess.run([sys.executable, "-c", code], cwd=str(ROOT),
                       capture_output=True, text=True, timeout=180)
    if r.returncode != 0 and any(
            key in (r.stderr or "") for key in
            ("pixel format", "OpenGL", "xcb", "DISPLAY", "display")):
        pytest.skip(f"no GL/display for QtInteractor here: {r.stderr[-200:]!r}")
    assert r.returncode == 0, f"stdout={r.stdout!r}\nstderr={r.stderr!r}"
    assert "STUDIO_OK" in r.stdout
