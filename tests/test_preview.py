"""M1 matplotlib layer preview. SPEC §9 (Qt-independent helpers).

matplotlib + shapely are declared deps but heavy; skip cleanly where absent.
"""
import pytest

shapely = pytest.importorskip("shapely")
matplotlib = pytest.importorskip("matplotlib")
matplotlib.use("Agg")  # headless

import math  # noqa: E402

from shapely.geometry import Polygon  # noqa: E402

from rotoforge_slicer.config import CAxisCfg, Config  # noqa: E402
from rotoforge_slicer.geometry import Layer  # noqa: E402
from rotoforge_slicer.gui.preview import _sample_indices, plot_layer, plot_slices  # noqa: E402


def _line_heading_deg(line):
    x0, x1 = line.get_xdata()
    y0, y1 = line.get_ydata()
    return math.degrees(math.atan2(y1 - y0, x1 - x0))


def _layer_with_hole(index=0, z=0.1):
    outer = [(0, 0), (10, 0), (10, 10), (0, 10)]
    hole = [(3, 3), (7, 3), (7, 7), (3, 7)]
    return Layer(index=index, z=z, regions=[Polygon(outer, [hole])])


def test_plot_layer_one_patch_per_region():
    import matplotlib.pyplot as plt

    layer = _layer_with_hole()
    fig, ax = plt.subplots()
    plot_layer(layer, ax=ax, show_wedge=False)
    assert len(ax.patches) == 1               # one region -> one (holed) patch
    assert ax.get_aspect() == 1.0             # equal aspect
    assert "Z=0.100" in ax.get_title()
    plt.close(fig)


def test_plot_layer_accepts_bare_region_list():
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots()
    regions = [Polygon([(0, 0), (5, 0), (5, 5), (0, 5)])]
    plot_layer(regions, ax=ax, show_wedge=False)
    assert len(ax.patches) == 1
    plt.close(fig)


def test_plot_layer_wedge_rays_at_home_plus_minus_half_default():
    import matplotlib.pyplot as plt

    layer = _layer_with_hole()
    fig, ax = plt.subplots()
    plot_layer(layer, ax=ax, cfg=Config(), show_wedge=True)
    # Two dashed wedge-boundary rays, at home ± half = 90 ± 45 = {45, 135} deg.
    assert len(ax.lines) == 2
    angles = sorted(_line_heading_deg(ln) for ln in ax.lines)
    assert angles == pytest.approx([45.0, 135.0])
    plt.close(fig)


def test_plot_layer_wedge_rays_follow_config_not_hardcoded():
    import matplotlib.pyplot as plt

    cfg = Config(c_axis=CAxisCfg(home_heading_deg=60.0, wedge_half_angle_deg=30.0))
    layer = _layer_with_hole()
    fig, ax = plt.subplots()
    plot_layer(layer, ax=ax, cfg=cfg, show_wedge=True)
    angles = sorted(_line_heading_deg(ln) for ln in ax.lines)
    assert angles == pytest.approx([30.0, 90.0])  # 60 ± 30, proves config flow
    plt.close(fig)


def test_plot_slices_renders_one_patch_per_chosen_layer():
    from rotoforge_slicer.geometry import SlicedModel

    layers = [_layer_with_hole(i, z=0.1 + 0.1 * i) for i in range(3)]
    model = SlicedModel(layers=layers, layer_height=0.1, z_min=0.0, z_max=0.3)
    fig = plot_slices(model, cfg=Config())
    drawn = [ax for ax in fig.axes if ax.patches]
    assert len(drawn) == 3  # each chosen layer actually rendered its region
    import matplotlib.pyplot as plt

    plt.close(fig)


def test_plot_slices_empty_model_branch():
    from rotoforge_slicer.geometry import SlicedModel

    model = SlicedModel(layers=[Layer(0, 0.0, [])], layer_height=0.1, z_min=0.0, z_max=0.1)
    fig = plot_slices(model)
    assert "no non-empty" in fig.axes[0].get_title()
    import matplotlib.pyplot as plt

    plt.close(fig)


def test_sample_indices_even_spread_includes_first_and_last():
    # The (max_layers, 2*max_layers) band that the old floor-division stride broke.
    idx = _sample_indices(23, 12)
    assert len(idx) == 12
    assert idx[0] == 0 and idx[-1] == 22       # top layer never dropped
    assert idx == sorted(idx) and len(set(idx)) == 12  # strictly increasing, unique
    assert idx[1] > 1                          # not clustered at the front
    # degenerate cases
    assert _sample_indices(3, 12) == [0, 1, 2]
    assert _sample_indices(0, 5) == []
    assert _sample_indices(5, 1) == [0]
