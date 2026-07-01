"""M6 GUI: toolpath preview rendering, the slice-preview model, app construction. SPEC §9.

The matplotlib/model tests run headless (Agg); the Qt construction runs in an offscreen
subprocess so it never needs a display and cannot pollute the pytest process.
"""
import os
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("shapely")
matplotlib = pytest.importorskip("matplotlib")
matplotlib.use("Agg")
from shapely.geometry import Polygon  # noqa: E402

from rotoforge_slicer.config import Config, load_config  # noqa: E402
from rotoforge_slicer.geometry import Layer  # noqa: E402
from rotoforge_slicer.gui.preview import plot_toolpath_3d, plot_toolpath_layer  # noqa: E402
from rotoforge_slicer.toolpath.passplan import LayerPlan, Pass, ToolpathPlan  # noqa: E402
from rotoforge_slicer.toolpath.segments import SegmentKind, build_segments  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
CFG = ROOT / "config" / "machine_duet3.yaml"


def _pass(x):
    return Pass(start=(x, 0), end=(x, 30), z=0.06, a_deg=0.0, rpm=5000,
               traverse_mm_min=120.0, e_per_path_mm=1.0)


def test_plot_toolpath_layer_draws_vectors_leadouts_resets():
    import matplotlib.pyplot as plt

    layer = Layer(0, 0.06, [Polygon([(0, 0), (20, 0), (20, 30), (0, 30)])])
    lp = LayerPlan(0, 0.06, [_pass(5), _pass(10), _pass(15)])
    fig, ax = plt.subplots()
    plot_toolpath_layer(layer, lp, ax=ax, cfg=Config())
    assert ax.patches            # region fill
    assert ax.collections        # deposition quiver
    assert ax.lines              # lead-outs + resets
    assert "3 passes" in ax.get_title()
    plt.close(fig)


def test_plot_toolpath_layer_curved_pass_and_collision_overlay():
    import matplotlib.pyplot as plt

    from rotoforge_slicer.config import CAxisCfg
    from rotoforge_slicer.toolpath.collision import Collision

    region = Polygon([(0, 0), (20, 0), (20, 30), (0, 30)])
    curved = Pass.curved([(5, 0), (5, 10), (7, 20), (10, 30)], z=0.06, rpm=5000,
                         traverse_mm_min=120.0, e_per_path_mm=1.0, c_axis=CAxisCfg())
    lp = LayerPlan(0, 0.06, [curved])
    cols = [Collision(0, 0.06, "wire", (12.0, 18.0), 5.0, "test")]
    fig, ax = plt.subplots()
    plot_toolpath_layer(Layer(0, 0.06, [region]), lp, ax=ax, cfg=Config(), collisions=cols)
    assert len(ax.collections) >= 2          # curved heading quiver + collision scatter
    assert ax.lines                          # the polyline bow + lead-out
    plt.close(fig)


def test_plot_toolpath_layer_handles_empty_passes():
    import matplotlib.pyplot as plt

    layer = Layer(0, 0.06, [Polygon([(0, 0), (5, 0), (5, 5), (0, 5)])])
    fig, ax = plt.subplots()
    plot_toolpath_layer(layer, LayerPlan(0, 0.06, []), ax=ax, cfg=Config())  # must not raise
    plt.close(fig)


def _toolpath_plan_2layers():
    return ToolpathPlan(
        [LayerPlan(0, 0.06, [_pass(5), _pass(10)]),
         LayerPlan(1, 0.18, [_pass(7)])],
        rpm=5000, traverse_mm_min=120.0, v_grind_floor_mm_min=120.0)


def _drawn_count(ax):
    # Line3DCollection populates get_segments() from the 3D data only at draw time.
    ax.figure.canvas.draw()
    return sum(len(c.get_segments()) for c in ax.collections)


def test_plot_toolpath_3d_color_coded_one_collection_per_kind():
    import matplotlib.pyplot as plt

    segs = build_segments(_toolpath_plan_2layers(), Config())
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    plot_toolpath_3d(segs, ax=ax, cfg=Config())
    # every kind present -> one color-coded Line3DCollection each (all six here)
    assert len(ax.collections) == len({s.kind for s in segs})
    assert ax.get_zlabel() == "Z [mm]"
    plt.close(fig)


def test_plot_toolpath_3d_toggles_and_scrubber_filter():
    import matplotlib.pyplot as plt

    segs = build_segments(_toolpath_plan_2layers(), Config())

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    plot_toolpath_3d(segs, ax=ax, cfg=Config(), enabled={"deposition"})
    assert len(ax.collections) == 1                     # one kind shown
    n_all = _drawn_count(ax)
    plt.close(fig)

    # the layer scrubber (upto_layer=0) hides the upper layer's deposition
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    plot_toolpath_3d(segs, ax=ax, cfg=Config(), enabled={"deposition"}, upto_layer=0)
    assert 0 < _drawn_count(ax) < n_all
    plt.close(fig)

    # disabling everything draws nothing
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    plot_toolpath_3d(segs, ax=ax, cfg=Config(), enabled=set())
    assert not ax.collections
    plt.close(fig)


def test_build_preview_model(tmp_path):
    trimesh = pytest.importorskip("trimesh")
    pytest.importorskip("scipy")
    from rotoforge_slicer.gui.model import build_preview

    stl = tmp_path / "box.stl"
    trimesh.creation.box(extents=(20, 12, 3)).export(stl)
    progress = []
    pv = build_preview(str(stl), load_config(CFG), None,
                       progress=lambda f, m: progress.append(f))
    assert pv.layer_count > 0 and pv.nonempty_indices
    assert pv.gcode and "M84" in pv.gcode and pv.validation_error is None
    assert pv.collisions == []                       # a clean box
    assert pv.segments and any(s.kind is SegmentKind.DEPOSITION for s in pv.segments)
    layer, lp, cols = pv.layer(pv.nonempty_indices[0])
    assert lp.passes and cols == []
    assert any("passes:" in s for s in pv.summary_lines())
    assert progress and progress[-1] == 1.0          # progress reaches 100%


def test_gui_app_constructs_offscreen():
    pytest.importorskip("PySide6")
    code = (
        "from PySide6 import QtWidgets\n"
        "from rotoforge_slicer.gui.app import _build_main_window\n"
        "app = QtWidgets.QApplication([])\n"
        "w = _build_main_window()\n"
        "assert w.windowTitle() == 'Rotoforge Slicer'\n"
        "assert w.slider is not None and w.canvas is not None and w.ax is not None\n"
        "assert w.canvas3d is not None and w.ax3d is not None\n"
        "assert w.tabs.count() == 2\n"
        "assert len(w.toggles) == 5 and all(cb.isChecked() for cb in w.toggles.values())\n"
        "print('GUI_OK')\n"
    )
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    r = subprocess.run([sys.executable, "-c", code], cwd=str(ROOT),
                       capture_output=True, text=True, env=env)
    assert r.returncode == 0, r.stderr
    assert "GUI_OK" in r.stdout
