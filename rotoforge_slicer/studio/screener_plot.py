"""The graphical process-window map: screener cells on the RPM x traverse plane.

Renders the FRAM screener grid the way SPEC §5.2 reasons about it: every tested
cell plotted at (traverse, RPM), stable cells green / unstable red, constant
revs/mm rays as lines through the origin, the selected ray highlighted with its
widest CONTIGUOUS stable run emphasized (that run is the selectable operating
window), and the chosen representative cell marked. matplotlib only (lazy import,
QtAgg canvas supplied by the GUI) — no Qt here, so it renders headless in tests.
"""
from __future__ import annotations

from typing import Optional

from ..process.screener import _nv, _trav, _truthy, distinct_rays, ray_run

STABLE_COLOR = "#12a150"
UNSTABLE_COLOR = "#d64545"
RAY_COLOR = "#b6c2d0"
SELECTED_RAY_COLOR = "#1f5fd6"
RUN_COLOR = "#1f5fd6"
CHOSEN_COLOR = "#e0a000"


def plot_screener_map(ax, rows, *, selected_nv: Optional[float] = None,
                      tol: float = 5.0, chosen_traverse: Optional[float] = None,
                      rpm_window=None) -> None:
    """Draw the process-window map onto a matplotlib Axes (cleared first).

    ``rows``            screener rows (``process.screener.load_rows``).
    ``selected_nv``     highlight this revs/mm ray + its contiguous stable run.
    ``chosen_traverse`` mark the representative cell nearest this traverse on the
                        selected ray (the operating point the slicer will use).
    ``rpm_window``      optional (rpm_min, rpm_max) SuperPID band, shaded.
    """
    ax.clear()
    if not rows:
        ax.set_title("no screener data")
        return

    stable = [r for r in rows if _truthy(r.get("pass", ""))]
    unstable = [r for r in rows if not _truthy(r.get("pass", ""))]
    if unstable:
        ax.scatter([_trav(r) for r in unstable], [float(r["rpm"]) for r in unstable],
                   s=22, marker="x", color=UNSTABLE_COLOR, alpha=0.7,
                   label="unstable", zorder=3)
    if stable:
        ax.scatter([_trav(r) for r in stable], [float(r["rpm"]) for r in stable],
                   s=26, marker="o", color=STABLE_COLOR, alpha=0.85,
                   label="stable", zorder=4)

    v_hi = max(_trav(r) for r in rows) * 1.08
    for nv in distinct_rays(rows, tol):
        sel = selected_nv is not None and abs(nv - selected_nv) <= tol
        ax.plot([0, v_hi], [0, nv * v_hi],
                color=SELECTED_RAY_COLOR if sel else RAY_COLOR,
                lw=2.0 if sel else 0.8, alpha=0.9 if sel else 0.5, zorder=2)

    if selected_nv is not None:
        run = ray_run(rows, selected_nv, tol)
        if run:
            vs = [_trav(r) for r in run]
            ax.plot(vs, [float(r["rpm"]) for r in run], color=RUN_COLOR, lw=4.0,
                    alpha=0.35, solid_capstyle="round", zorder=5,
                    label="stable window")
            if chosen_traverse is not None:
                rep = min(run, key=lambda r: abs(_trav(r) - chosen_traverse))
                ax.scatter([_trav(rep)], [float(rep["rpm"])], s=140, marker="*",
                           color=CHOSEN_COLOR, edgecolor="#7a5700", zorder=6,
                           label="operating point")

    if rpm_window:
        ax.axhspan(rpm_window[0], rpm_window[1], color="#8888ff", alpha=0.06, zorder=1)

    ax.set_xlabel("traverse [mm/min]")
    ax.set_ylabel("spindle RPM")
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.legend(loc="upper left", fontsize=8)
    ax.set_title("process window — constant revs/mm rays", fontsize=10)
