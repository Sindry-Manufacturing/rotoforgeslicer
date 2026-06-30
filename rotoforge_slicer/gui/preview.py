"""Matplotlib per-layer toolpath / region preview. SPEC §9.

M1 ships the Qt-independent plotting helpers (``plot_layer`` renders a sliced
layer's region polygons, holes included, with an optional ±45° deposition-wedge
overlay). The embedded Qt canvas (``make_preview_canvas``) is wired in M6 and will
reuse these helpers.

matplotlib is imported lazily so the light core stays import-cheap (CLAUDE.md).
These functions never call ``pyplot.show`` and work headless (Agg).
"""
from __future__ import annotations

import math
from typing import Optional


def _regions_of(layer_or_regions):
    """Accept either a geometry.Layer or a bare list of shapely Polygons."""
    return getattr(layer_or_regions, "regions", layer_or_regions)


def _polygon_path(poly):
    """matplotlib Path for a shapely Polygon, holes as opposite-wound subpaths."""
    import numpy as np
    from matplotlib.path import Path
    from shapely.geometry.polygon import orient

    poly = orient(poly, sign=1.0)  # exterior CCW, holes CW -> nonzero winding holes
    verts = []
    codes = []
    for ring in [poly.exterior, *poly.interiors]:
        coords = np.asarray(ring.coords)
        n = len(coords)
        if n < 3:
            continue
        verts.extend(coords)
        codes.append(Path.MOVETO)
        codes.extend([Path.LINETO] * (n - 2))
        codes.append(Path.CLOSEPOLY)
    return Path(verts, codes)


def plot_layer(
    layer_or_regions,
    ax=None,
    *,
    cfg=None,
    facecolor="#4C9BE8",
    edgecolor="#16456B",
    alpha: float = 0.5,
    show_wedge: bool = True,
    title: Optional[str] = None,
):
    """Draw a layer's solid regions (with holes) top-down onto a matplotlib Axes.

    Returns the Axes. If ``cfg`` is given and ``show_wedge`` is set, overlays the
    depositable ±wedge about the home heading (SPEC §4.1) at the layer centroid.
    """
    from matplotlib.patches import PathPatch

    regions = _regions_of(layer_or_regions)

    if ax is None:
        import matplotlib.pyplot as plt

        _, ax = plt.subplots()

    for poly in regions:
        path = _polygon_path(poly)
        ax.add_patch(PathPatch(path, facecolor=facecolor, edgecolor=edgecolor,
                               lw=1.0, alpha=alpha))

    if cfg is not None and show_wedge and regions:
        _draw_wedge(ax, regions, cfg)

    ax.set_aspect("equal", adjustable="datalim")
    ax.autoscale_view()
    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    if title is None:
        z = getattr(layer_or_regions, "z", None)
        idx = getattr(layer_or_regions, "index", None)
        if z is not None:
            title = f"Layer {idx} — Z={z:.3f} mm" if idx is not None else f"Z={z:.3f} mm"
    if title:
        ax.set_title(title)
    return ax


def _draw_wedge(ax, regions, cfg) -> None:
    """Overlay home heading + depositable wedge boundary rays at the regions' centre."""
    from shapely.ops import unary_union

    c = getattr(cfg, "c_axis", cfg)  # accept a full Config or a CAxisCfg
    union = unary_union(regions)
    cx, cy = union.centroid.x, union.centroid.y
    minx, miny, maxx, maxy = union.bounds
    span = max(maxx - minx, maxy - miny) or 1.0
    r = 0.45 * span

    home = c.home_heading_deg
    half = c.wedge_half_angle_deg
    # Home-heading arrow (e.g. +Y).
    ax.annotate(
        "", xy=(cx + r * math.cos(math.radians(home)), cy + r * math.sin(math.radians(home))),
        xytext=(cx, cy), arrowprops=dict(arrowstyle="->", color="#107C10", lw=1.5))
    # Wedge boundary rays (home ± half), travel-direction limits.
    for edge in (home - half, home + half):
        ax.plot([cx, cx + r * math.cos(math.radians(edge))],
                [cy, cy + r * math.sin(math.radians(edge))],
                color="#107C10", ls="--", lw=1.0, alpha=0.8)


def _sample_indices(count: int, n: int):
    """Indices of ``min(n, count)`` items spread evenly across ``range(count)``.

    Always includes the first and last index (so a tall part's top layer shows);
    indices are strictly increasing and unique. Avoids the floor-division stride
    that, for ``count`` in ``(n, 2n)``, would pick only the first ``n`` contiguous.
    """
    if count <= 0:
        return []
    m = min(n, count)
    if m <= 1:
        return [0]
    if m >= count:
        return list(range(count))
    return [round(i * (count - 1) / (m - 1)) for i in range(m)]


def plot_slices(model, max_layers: int = 12, *, cfg=None):
    """Grid preview of up to ``max_layers`` evenly-sampled non-empty layers.

    Returns the matplotlib Figure. Useful as a quick M1 sanity render.
    """
    import matplotlib.pyplot as plt

    layers = getattr(model, "nonempty_layers", None)
    if layers is None:
        layers = [ly for ly in model if not getattr(ly, "is_empty", False)]
    if not layers:
        fig, ax = plt.subplots()
        ax.set_title("no non-empty layers")
        return fig

    chosen = [layers[i] for i in _sample_indices(len(layers), max_layers)]
    n = len(chosen)

    cols = min(4, n)
    rows = math.ceil(n / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(3.2 * cols, 3.2 * rows), squeeze=False)
    for ax in axes.flat:
        ax.axis("off")
    for ax, layer in zip(axes.flat, chosen):
        ax.axis("on")
        plot_layer(layer, ax=ax, cfg=cfg)
    fig.tight_layout()
    return fig


def make_preview_canvas(parent=None):
    """Embedded Qt FigureCanvas for the GUI. SPEC §9.  [stub — M6]"""
    raise NotImplementedError("make_preview_canvas: implement per SPEC §9 (M6)")
