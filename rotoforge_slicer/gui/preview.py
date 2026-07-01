"""Matplotlib per-layer toolpath / region preview. SPEC §9.

M1 ships the Qt-independent plotting helpers (``plot_layer`` renders a sliced
layer's region polygons, holes included, with an optional +Y home-heading reference
arrow — D13 removed the deposition wedge, so there is no wedge fan to draw).
The embedded Qt canvas (``make_preview_canvas``) is wired in M6 and will reuse these
helpers.

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
    show_home: bool = True,
    title: Optional[str] = None,
):
    """Draw a layer's solid regions (with holes) top-down onto a matplotlib Axes.

    Returns the Axes. If ``cfg`` is given and ``show_home`` is set, overlays the +Y
    home-heading reference arrow (the axis zero; D13 — no wedge) at the layer centroid.
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

    if cfg is not None and show_home and regions:
        _draw_home_ref(ax, regions, cfg)

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


def _draw_home_ref(ax, regions, cfg) -> None:
    """Overlay the +Y home-heading reference arrow at the regions' centre.

    D13 removed the deposition wedge — every heading is depositable, so there is no
    wedge fan. The home heading is only the axis-zero reference; we draw it for
    orientation. (Break/unwind overlays are M11 work.)
    """
    from shapely.ops import unary_union

    c = getattr(cfg, "c_axis", cfg)  # accept a full Config or a CAxisCfg
    union = unary_union(regions)
    cx, cy = union.centroid.x, union.centroid.y
    minx, miny, maxx, maxy = union.bounds
    span = max(maxx - minx, maxy - miny) or 1.0
    r = 0.45 * span

    home = c.home_heading_deg
    ax.annotate(
        "A=0", xy=(cx + r * math.cos(math.radians(home)),
                   cy + r * math.sin(math.radians(home))),
        xytext=(cx, cy), color="#107C10",
        arrowprops=dict(arrowstyle="->", color="#107C10", lw=1.5))


def plot_toolpath_layer(layer, layer_plan, ax=None, *, cfg=None, show_home=True,
                        show_resets=True, collisions=None, title=None):
    """Draw one layer's planned toolpath: region fill + deposition vectors + lead-outs /
    wire-cuts + reset (airborne travel) moves + the +Y home reference (SPEC §9; D13).

    ``layer`` is a geometry.Layer (regions), ``layer_plan`` a passplan.LayerPlan (or a
    bare list of passes). Returns the Axes. ``collisions`` may be Collision records whose
    ``at`` points are flagged in red (validation overlay)."""
    import matplotlib.pyplot as plt

    if ax is None:
        _, ax = plt.subplots()

    plot_layer(layer, ax=ax, cfg=cfg, show_home=show_home,
               facecolor="#e9eef5", edgecolor="#9bb0c9", alpha=0.75, title="")

    passes = getattr(layer_plan, "passes", layer_plan) or []
    lead_out = getattr(getattr(cfg, "process", None), "lead_out_len_mm", 4.0)

    if passes:
        # A straight pass's quiver spans the whole line. A curved pass gets a short
        # heading arrow (its FIRST segment, matching p.a_deg — not a misleading chord)
        # plus its full polyline so the bow is visible.
        straight = [p for p in passes if not getattr(p, "is_curved", False)]
        curved = [p for p in passes if getattr(p, "is_curved", False)]
        if straight:
            ax.quiver([p.start[0] for p in straight], [p.start[1] for p in straight],
                      [p.end[0] - p.start[0] for p in straight],
                      [p.end[1] - p.start[1] for p in straight],
                      angles="xy", scale_units="xy", scale=1, color="#1f5fd6",
                      width=0.004, headwidth=3.2, headlength=4, zorder=4)
        if curved:
            ax.quiver([p.start[0] for p in curved], [p.start[1] for p in curved],
                      [p.points[1][0] - p.start[0] for p in curved],
                      [p.points[1][1] - p.start[1] for p in curved],
                      angles="xy", scale_units="xy", scale=1, color="#1f5fd6",
                      width=0.004, headwidth=3.2, headlength=4, zorder=4)
            for p in curved:
                ax.plot([q[0] for q in p.points], [q[1] for q in p.points],
                        color="#1f5fd6", lw=1.0, zorder=4)

    for i, p in enumerate(passes):
        (x1, y1) = p.end
        prev = p.points[-2] if len(p.points) >= 2 else p.start
        n = math.hypot(x1 - prev[0], y1 - prev[1]) or 1.0
        ux, uy = (x1 - prev[0]) / n, (y1 - prev[1]) / n
        lx, ly = x1 + lead_out * ux, y1 + lead_out * uy
        ax.plot([x1, lx], [y1, ly], color="#e0a000", lw=1.1, zorder=3)        # lead-out
        ax.plot([lx], [ly], marker="x", color="#c0392b", ms=4, zorder=5)      # wire cut
        if show_resets and i + 1 < len(passes):                               # reset/travel
            nx, ny = passes[i + 1].start
            ax.plot([lx, nx], [ly, ny], color="#888", ls=":", lw=0.5, zorder=2)

    if collisions:
        cx = [c.at[0] for c in collisions]
        cy = [c.at[1] for c in collisions]
        ax.scatter(cx, cy, s=70, facecolors="none", edgecolors="#e01010",
                   linewidths=1.6, zorder=6, label="collision")

    z = getattr(layer, "z", None)
    idx = getattr(layer, "index", None)
    if title is None and z is not None:
        title = f"layer {idx}  z={z:.2f} mm   |   {len(passes)} passes"
    if title:
        ax.set_title(title, fontsize=10)
    ax.set_aspect("equal", adjustable="datalim")
    ax.autoscale_view()
    return ax


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


def plot_toolpath_3d(segments, ax=None, *, cfg=None, enabled=None, upto_layer=None,
                     title=None, legend=True):
    """Draw tagged :class:`~rotoforge_slicer.toolpath.segments.ToolpathSegment` in 3D,
    color-coded by kind (SPEC §9; U2).

    ``segments``   list from ``toolpath.segments.build_segments``.
    ``enabled``    iterable of viewer-toggle names (``segments.TOGGLE_ORDER``); None = all.
    ``upto_layer`` layer index — show only segments on that layer and below (the layer
                   scrubber's cumulative build-up view); None shows the whole path.

    One ``Line3DCollection`` per shown kind (so toggling stays cheap). Returns the Axes3D.
    """
    from collections import defaultdict

    from mpl_toolkits.mplot3d.art3d import Line3DCollection

    from ..toolpath.segments import KIND_COLOR, TOGGLE_KINDS, TOGGLE_ORDER, SegmentKind

    if ax is None:
        import matplotlib.pyplot as plt
        import mpl_toolkits.mplot3d  # noqa: F401 (registers the 3d projection)

        ax = plt.figure().add_subplot(111, projection="3d")

    shown = set()
    for name in TOGGLE_ORDER:
        if enabled is None or name in set(enabled):
            shown.update(TOGGLE_KINDS[name])

    # deposition solid + thick; lead-in/out solid; airborne (travel/liftoff/reset) dashed thin
    style = {
        SegmentKind.DEPOSITION: (2.2, "-"),
        SegmentKind.LEAD_IN: (1.6, "-"),
        SegmentKind.LEAD_OUT: (1.6, "-"),
        SegmentKind.LIFTOFF: (0.9, "--"),
        SegmentKind.RESET: (0.9, "--"),
        SegmentKind.TRAVEL: (0.7, ":"),
    }

    by_kind = defaultdict(list)
    for s in segments:
        if s.kind not in shown:
            continue
        if upto_layer is not None and s.layer_index is not None and s.layer_index > upto_layer:
            continue
        by_kind[s.kind].append((s.start, s.end))

    xs, ys, zs = [], [], []
    for kind in SegmentKind:                    # stable draw order -> deterministic legend
        lines = by_kind.get(kind)
        if not lines:
            continue
        lw, ls = style[kind]
        ax.add_collection3d(Line3DCollection(
            lines, colors=KIND_COLOR[kind], linewidths=lw, linestyles=ls, label=kind.value))
        for a, b in lines:
            xs += (a[0], b[0])
            ys += (a[1], b[1])
            zs += (a[2], b[2])

    if xs:
        _set_equal_3d(ax, xs, ys, zs)           # mplot3d won't autoscale a Line3DCollection
    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    ax.set_zlabel("Z [mm]")
    if legend and by_kind:
        ax.legend(loc="upper left", fontsize=8)
    if title:
        ax.set_title(title, fontsize=10)
    return ax


def _set_equal_3d(ax, xs, ys, zs, pad: float = 0.05) -> None:
    """Undistorted (equal-scale) 3D limits with a small pad: center each axis on the
    data and share one half-range, so the toolpath keeps true proportions."""
    xmin, xmax, ymin, ymax, zmin, zmax = (
        min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))
    cx, cy, cz = (xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2
    half = max(xmax - xmin, ymax - ymin, zmax - zmin, 1.0) * (0.5 + pad)
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_zlim(cz - half, cz + half)


def make_preview_canvas(parent=None, *, projection=None):
    """An embedded Qt matplotlib canvas for the GUI preview (SPEC §9).

    ``projection="3d"`` returns a canvas with an ``Axes3D`` (the U2 toolpath viewer);
    the default is the 2D per-layer preview axes."""
    import matplotlib
    matplotlib.use("QtAgg")
    from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
    from matplotlib.figure import Figure

    canvas = FigureCanvasQTAgg(Figure(figsize=(6, 6)))
    if projection == "3d":
        import mpl_toolkits.mplot3d  # noqa: F401 (registers the 3d projection)

        canvas.figure.add_subplot(111, projection="3d")
    else:
        canvas.figure.add_subplot(111)
    return canvas
