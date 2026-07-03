"""Graphical process window: ray helpers, user-chosen cells, the map plot, and
material profiles (SPEC §5/§9). Everything headless — Qt is exercised only via the
studio construction test."""
from pathlib import Path

import pytest

from rotoforge_slicer.config import Config
from rotoforge_slicer.process.screener import (
    distinct_rays, load_rows, nearest_stable_cell, ray_run,
    select_operating_point, widest_ray,
)
from rotoforge_slicer.studio.materials import (
    MaterialProfile, hotshoe_macro_name, hotshoe_temp_from_macro,
    load_profiles, save_profiles,
)

HEADER = ("rpm,traverse_mm_min,pass,n_over_v,feed_speed_mm_min,"
          "feed_ratio_phi,T_AZ_C,torque_Nm,power_kW")


def _csv(tmp_path, rows):
    p = tmp_path / "screener.csv"
    p.write_text(HEADER + "\n" + "\n".join(rows) + "\n")
    return str(p)


def _cells(nv, vs, ok=True):
    return [f"{int(nv * v)},{v},{1 if ok else 0},{nv},{v * 0.9:.1f},0.9,420,0.5,1.1"
            for v in vs]


def test_distinct_rays_cluster_and_widest():
    rows = [dict(zip(HEADER.split(","), c.split(",")))
            for c in _cells(100, [80, 100, 120]) + _cells(150, [90, 100, 110, 120, 130])
            + _cells(100.4, [140])]                      # clusters into the 100 ray
    rays = distinct_rays(rows, tol=5.0)
    assert len(rays) == 2                                # 100.4 clusters into 100
    assert rays[0] == pytest.approx(100.13, abs=0.2)     # cluster mean
    # ray 100 spans 80..140 (60 wide, all stable) vs ray 150's 90..130 (40 wide)
    assert widest_ray(rows, tol=5.0) == pytest.approx(rays[0])


def test_ray_run_breaks_on_unstable_cell(tmp_path):
    rows = load_rows(_csv(tmp_path,
                          _cells(150, [80, 90]) + _cells(150, [100], ok=False)
                          + _cells(150, [110, 120, 130])))
    run = ray_run(rows, 150.0, tol=5.0)
    vs = [float(r["traverse_mm_min"]) for r in run]
    assert vs == [110.0, 120.0, 130.0]                   # the widest CONTIGUOUS side


def test_select_operating_point_traverse_target_snaps_to_cell(tmp_path):
    csv = _csv(tmp_path, _cells(150, [80, 90, 100, 110, 120]))
    op = select_operating_point(csv, mode="manual", target=150.0, tol=5.0,
                                rpm_min=5000, rpm_max=30000, traverse_target=113.0)
    assert op.traverse_mm_min == 110.0                   # nearest measured cell
    assert op.rpm == 150 * 110
    mid = select_operating_point(csv, mode="manual", target=150.0, tol=5.0,
                                 rpm_min=5000, rpm_max=30000)
    assert mid.traverse_mm_min == 100.0                  # default: run midpoint


def test_plot_screener_map_headless(tmp_path):
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from rotoforge_slicer.studio.screener_plot import plot_screener_map

    rows = load_rows(_csv(tmp_path, _cells(150, [90, 100, 110])
                          + _cells(150, [120], ok=False) + _cells(100, [80, 100])))
    fig, ax = plt.subplots()
    plot_screener_map(ax, rows, selected_nv=150.0, tol=5.0, chosen_traverse=100.0,
                      rpm_window=(5000, 30000))
    assert len(ax.collections) >= 3          # stable + unstable + chosen-cell star
    assert len(ax.lines) >= 3                # two rays + the run band
    assert ax.get_xlabel().startswith("traverse")
    plot_screener_map(ax, [], selected_nv=None)          # empty-data branch
    assert "no screener" in ax.get_title()
    plt.close(fig)


def test_nearest_stable_cell_independent_axes(tmp_path):
    """The graphical screener's snap: RPM and traverse are independent targets
    landing on measured STABLE cells only (never interpolated)."""
    rows = load_rows(_csv(tmp_path,
                          _cells(100, [80, 100, 120])          # RPM 8k/10k/12k
                          + _cells(150, [90, 110])             # RPM 13.5k/16.5k
                          + _cells(150, [100], ok=False)))     # unstable: excluded
    # exact hit
    hit = nearest_stable_cell(rows, rpm=15000, traverse=100)
    assert float(hit["rpm"]) == 13500.0 and float(hit["traverse_mm_min"]) == 90.0
    # RPM-only snap ignores traverse entirely
    assert float(nearest_stable_cell(rows, rpm=16000)["rpm"]) == 16500.0
    # traverse-only snap
    assert float(nearest_stable_cell(rows, traverse=121)["traverse_mm_min"]) == 120.0
    # the unstable cell at (150 ray, v=100) is never selectable
    near_unstable = nearest_stable_cell(rows, rpm=15000, traverse=100)
    assert not (float(near_unstable["rpm"]) == 15000.0
                and float(near_unstable["traverse_mm_min"]) == 100.0)
    # cells off the previously-selected ray are reachable (independence)
    other_ray = nearest_stable_cell(rows, rpm=10000, traverse=100)
    assert float(other_ray["rpm"]) == 10000.0
    assert nearest_stable_cell([], rpm=1) is None


def test_independently_chosen_cell_pins_exactly(tmp_path):
    """WYSIWYG under independent selection: Apply pins the chosen cell's own
    revs/mm + traverse, and the pipeline reproduces exactly that cell."""
    csv = _csv(tmp_path, _cells(100, [80, 100, 120]) + _cells(150, [90, 110]))
    rows = load_rows(csv)
    cell = nearest_stable_cell(rows, rpm=13000, traverse=95)
    nv = float(cell["n_over_v"])
    op = select_operating_point(csv, mode="manual", target=nv, tol=5.0,
                                rpm_min=5000, rpm_max=30000,
                                traverse_target=float(cell["traverse_mm_min"]))
    assert op.traverse_mm_min == float(cell["traverse_mm_min"])
    assert op.rpm == int(round(float(cell["rpm"])))


def test_plot_axis_stays_at_measured_rpm_scale(tmp_path):
    """User report: the RPM axis ran to 1e6 — steep constant-revs/mm rays drawn
    to the traverse limit dragged the autoscale. Axes must stay at the measured
    data scale (the spindle tops out ~30k), rays clipped to the window."""
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from rotoforge_slicer.studio.screener_plot import plot_screener_map

    # steep ray: nv=300 at v<=100 -> RPM<=30k, but the ray extended to
    # v_hi=108 * nv... with a shallow ray forcing v_hi high, the old code drew
    # the steep ray to 300*216 = 64 800+; worse cases hit 1e6
    rows = load_rows(_csv(tmp_path, _cells(300, [80, 100])     # RPM 24k/30k
                          + _cells(50, [100, 200])))           # RPM 5k/10k
    fig, ax = plt.subplots()
    plot_screener_map(ax, rows, selected_nv=300.0, tol=5.0, chosen_traverse=100.0)
    max_rpm = 30000.0
    assert ax.get_ylim()[1] <= max_rpm * 1.2, \
        f"RPM axis blew up to {ax.get_ylim()[1]:g}"
    for line in ax.lines:                                      # rays clipped
        assert max(line.get_ydata()) <= max_rpm * 1.2
    plt.close(fig)


def test_plot_chosen_cell_marker(tmp_path):
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from rotoforge_slicer.studio.screener_plot import plot_screener_map

    rows = load_rows(_csv(tmp_path, _cells(150, [90, 100, 110])))
    cell = nearest_stable_cell(rows, rpm=15000, traverse=100)
    fig, ax = plt.subplots()
    plot_screener_map(ax, rows, selected_nv=150.0, tol=5.0, chosen_cell=cell)
    labels = [c.get_label() for c in ax.collections]
    assert "operating point" in labels
    plt.close(fig)


REAL_CSV = (Path(__file__).parent / "fixtures" /
            "fram_rim_jet_process_window_gridAl1100_30KRPM_300CW_60CB.csv")


@pytest.mark.skipif(not REAL_CSV.exists(), reason="real screener CSV not present")
def test_real_fram_export_loads_and_selects_independently():
    """The user's actual FRAM parameter-screener export (Al1100, 30k RPM grid,
    ~7400 cells): a rectangular RPM x traverse grid, NOT ray-structured data —
    the regime the independent selection exists for."""
    rows = load_rows(str(REAL_CSV))
    assert len(rows) > 1000
    cell = nearest_stable_cell(rows, rpm=12000, traverse=100)
    assert cell is not None and str(cell["pass"]).strip().upper() == "TRUE"
    # independent axes: asking for a different traverse at the same RPM moves
    # along the traverse axis without being dragged onto a revs/mm ray
    other = nearest_stable_cell(rows, rpm=float(cell["rpm"]), traverse=250)
    assert float(other["rpm"]) == pytest.approx(float(cell["rpm"]), rel=0.2)
    assert float(other["traverse_mm_min"]) != float(cell["traverse_mm_min"])
    # WYSIWYG: pinning the chosen cell reproduces exactly that cell
    op = select_operating_point(str(REAL_CSV), mode="manual",
                                target=float(cell["n_over_v"]), tol=5.0,
                                rpm_min=5000, rpm_max=30000,
                                traverse_target=float(cell["traverse_mm_min"]))
    assert op.traverse_mm_min == float(cell["traverse_mm_min"])


@pytest.mark.skipif(not REAL_CSV.exists(), reason="real screener CSV not present")
def test_real_fram_export_plot_axis_stays_sane():
    """User report reproduced with the REAL data: nv reaches ~3000 (30k RPM at
    v=10), so the old unclipped rays drove the RPM axis toward 1e6. The axis
    must stay at the measured spindle scale (~30k)."""
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from rotoforge_slicer.studio.screener_plot import plot_screener_map

    rows = load_rows(str(REAL_CSV))
    max_rpm = max(float(r["rpm"]) for r in rows)
    fig, ax = plt.subplots()
    plot_screener_map(ax, rows, selected_nv=None, tol=5.0)
    assert ax.get_ylim()[1] <= max_rpm * 1.2
    for line in ax.lines:
        assert max(line.get_ydata()) <= max_rpm * 1.2
    plt.close(fig)


def test_material_profiles_roundtrip_and_apply(tmp_path):
    path = tmp_path / "materials.yaml"
    profs = {"Al1100-O": MaterialProfile(
        name="Al1100-O", csv_path="al.csv", revs_per_mm=150.0,
        traverse_mm_min=100.0, bed_temp_c=110.0, hotshoe_temp_c=300.0)}
    save_profiles(profs, path)
    back = load_profiles(path)
    assert back["Al1100-O"] == profs["Al1100-O"]

    cfg = Config()
    back["Al1100-O"].apply_to_cfg(cfg)
    assert cfg.screener.revs_per_mm_mode == "manual"
    assert cfg.screener.revs_per_mm_target == 150.0
    assert cfg.screener.traverse_target == 100.0
    assert cfg.process.bed_temp_c == 110.0
    assert cfg.process.hotshoe_macro == "Hotshoe_300C.g"

    auto = MaterialProfile(name="x", revs_per_mm=0.0)    # 0 = auto ray
    auto.apply_to_cfg(cfg)
    assert cfg.screener.revs_per_mm_mode == "auto"


def test_traverse_target_outside_the_run_fails_loud(tmp_path):
    # review fix: a target outside the contiguous stable run means the selection is
    # stale (profile saved against different data) — snapping silently would run a
    # different operating point than the operator chose.
    csv = _csv(tmp_path, _cells(150, [80, 90, 100, 110, 120]))
    with pytest.raises(ValueError, match="outside the ray"):
        select_operating_point(csv, mode="manual", target=150.0, tol=5.0,
                               rpm_min=5000, rpm_max=30000, traverse_target=500.0)


def test_hotshoe_macro_reaches_the_emitted_preamble():
    # review fix: process.hotshoe_macro was dead config — the preamble emitted the
    # static YAML macro regardless of the material's hotshoe target.
    from rotoforge_slicer.emit.templates import preamble

    cfg = Config()
    MaterialProfile(name="hot", hotshoe_temp_c=450.0).apply_to_cfg(cfg)
    lines = preamble(cfg)
    assert 'M98 P"Hotshoe_450C.g"' in lines
    assert 'M98 P"Hotshoe_300C.g"' not in lines
    assert 'M98 P"CPAP_100pct.g"' in lines          # other macros untouched


def test_load_profiles_missing_file_is_empty(tmp_path):
    assert load_profiles(tmp_path / "nope.yaml") == {}


def test_hotshoe_macro_roundtrip():
    assert hotshoe_macro_name(300.0) == "Hotshoe_300C.g"
    assert hotshoe_temp_from_macro("Hotshoe_300C.g") == 300.0
    assert hotshoe_temp_from_macro("Hotshoe_247.5C.g") == 247.5
    assert hotshoe_temp_from_macro("Weird.g", default=300.0) == 300.0
