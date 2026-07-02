"""Graphical process window: ray helpers, user-chosen cells, the map plot, and
material profiles (SPEC §5/§9). Everything headless — Qt is exercised only via the
studio construction test."""
import pytest

from rotoforge_slicer.config import Config
from rotoforge_slicer.process.screener import (
    distinct_rays, load_rows, ray_run, select_operating_point, widest_ray,
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


def test_load_profiles_missing_file_is_empty(tmp_path):
    assert load_profiles(tmp_path / "nope.yaml") == {}


def test_hotshoe_macro_roundtrip():
    assert hotshoe_macro_name(300.0) == "Hotshoe_300C.g"
    assert hotshoe_temp_from_macro("Hotshoe_300C.g") == 300.0
    assert hotshoe_temp_from_macro("Hotshoe_247.5C.g") == 247.5
    assert hotshoe_temp_from_macro("Weird.g", default=300.0) == 300.0
