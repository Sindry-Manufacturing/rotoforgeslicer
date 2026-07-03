from pathlib import Path

import pytest

from rotoforge_slicer.process.screener import select_operating_point

# The REAL FRAM parameter-screener export (Al1100, 30 kRPM grid, ~7 400 cells) —
# a rectangular RPM x traverse grid, which is what production data looks like.
FIXTURE = (Path(__file__).resolve().parent / "fixtures" /
           "fram_rim_jet_process_window_gridAl1100_30KRPM_300CW_60CB.csv")

CSV = (
    "rpm,traverse_mm_min,pass,n_over_v,feed_speed_mm_min,feed_ratio_phi,torque_Nm,power_kW,T_AZ_C\n"
    "20000,100,1,200,100,1.00,1.2,0.80,420\n"
    "22000,110,1,200,110,1.00,1.3,0.85,430\n"
    "18000,90,1,200,90,1.00,1.1,0.75,410\n"
    "15000,300,0,50,300,1.00,1.9,1.10,500\n"   # not stable (pass=0)
)


def test_auto_selects_widest_revs_per_mm_ray(tmp_path):
    p = tmp_path / "window.csv"
    p.write_text(CSV)
    op = select_operating_point(str(p), mode="auto", tol=5)
    assert abs(op.revs_per_mm - 200) < 1e-9
    assert op.v_min_mm_min == 90 and op.v_max_mm_min == 110
    assert op.rpm_for(100) == 20000
    assert op.v_grind_floor_mm_min == 90


# ---- the real export (37 columns; only the needed ones are used) ----
# Expected values probed from the fixture itself: the auto pick is the widest
# CONTIGUOUS stable run over per-cell revs/mm candidates (nv ~ 30.31, traverse
# window ~ 623..1184 mm/min, representative midpoint cell v=904 -> RPM 22941).

@pytest.fixture(scope="module")
def auto_op():
    # auto selection walks ~3700 candidate rays over ~7400 cells (~seconds on
    # the real grid) — computed once for every test that reads it
    return select_operating_point(str(FIXTURE), mode="auto", tol=5)


def test_auto_picks_widest_contiguous_run(auto_op):
    op = auto_op
    assert op.revs_per_mm == pytest.approx(30.3106845)   # widest contiguous ray
    assert op.v_min_mm_min == pytest.approx(623.529412)
    assert op.v_max_mm_min == pytest.approx(1184.47059)
    assert op.rpm == 22941                               # rep cell at v=904 (midpoint)
    assert op.traverse_mm_min == pytest.approx(904.0)
    assert op.v_grind_floor_mm_min == pytest.approx(623.529412)


def test_manual_excludes_unstable_gap():
    # the nv~29.08 ray's stable cells span [395.6, 1237.1] mm/min, but unstable
    # (cold) cells break the low end — the contiguous run is [693.6, 1237.1];
    # the band must not cross the unstable gap.
    op = select_operating_point(str(FIXTURE), mode="manual", target=29.075798, tol=5)
    assert op.v_min_mm_min == pytest.approx(693.647, abs=0.01)
    assert op.v_max_mm_min == pytest.approx(1237.06, abs=0.01)
    assert op.v_min_mm_min > 500.0, \
        "the run must exclude the stable cells below the unstable gap (395.6..)"


def test_manual_no_match_raises():
    with pytest.raises(ValueError):
        select_operating_point(str(FIXTURE), mode="manual", target=1e6, tol=5)


def test_full_schema_parse_uses_only_needed_columns(auto_op):
    op = auto_op
    assert op.feed_speed_mm_min == pytest.approx(2373.2135)   # rep cell wire feed
    assert op.phi == pytest.approx(1.0)
    assert op.t_az_c > 0 and op.torque_Nm > 0
    assert "revs/mm=30.3" in op.summary()


# ---- contiguity actually changes the winner (vs the old max-min span) ----

GAP_CSV = (
    "rpm,traverse_mm_min,pass,n_over_v,feed_speed_mm_min,feed_ratio_phi,torque_Nm,power_kW,T_AZ_C\n"
    # ray nv=100: stable 50,60,70; UNSTABLE 80 breaks the run; stable outlier at 300.
    "5000,50,1,100,65,1.3,1.1,0.7,410\n"
    "6000,60,1,100,78,1.3,1.1,0.7,415\n"
    "7000,70,1,100,91,1.3,1.1,0.7,420\n"
    "8000,80,0,100,104,1.3,1.9,1.2,520\n"     # unstable
    "30000,300,1,100,390,1.3,1.3,0.9,470\n"   # stable, isolated outlier
    # ray nv=150: a clean contiguous run 60..140 (span 80).
    "9000,60,1,150,78,1.3,1.1,0.7,420\n"
    "12000,80,1,150,104,1.3,1.2,0.8,440\n"
    "15000,100,1,150,130,1.3,1.3,0.9,460\n"
    "18000,120,1,150,156,1.3,1.4,1.0,480\n"
    "21000,140,1,150,182,1.3,1.5,1.0,500\n"
)


def test_auto_picks_contiguous_not_widest_gapped(tmp_path):
    p = tmp_path / "gap.csv"
    p.write_text(GAP_CSV)
    op = select_operating_point(str(p), mode="auto", tol=5)
    # Old max-min span would pick nv=100 (50..300 = 250 via the outlier); the contiguous
    # rule picks nv=150 (its clean 60..140 run of 80 beats nv=100's broken run of 20).
    assert op.revs_per_mm == 150
    assert op.v_min_mm_min == 60 and op.v_max_mm_min == 140


# ---- RPM window (SPEC §1.3/§5.2 step 4) ----

def test_select_rejects_rpm_outside_superpid(tmp_path):
    hot = (
        "rpm,traverse_mm_min,pass,n_over_v,feed_speed_mm_min,feed_ratio_phi,torque_Nm,power_kW,T_AZ_C\n"
        "45000,300,1,150,390,1.3,1.3,0.9,500\n"   # 45000 > 30000
    )
    p = tmp_path / "hot.csv"
    p.write_text(hot)
    with pytest.raises(ValueError):
        select_operating_point(str(p), mode="auto", tol=5, rpm_min=5000, rpm_max=30000)
    # without bounds the cell is accepted as-is (no clamp/reject)
    assert select_operating_point(str(p), mode="auto", tol=5).rpm == 45000


def test_rpm_for_clamps_to_window():
    op = select_operating_point(str(FIXTURE), mode="auto", tol=5,
                                rpm_min=5000, rpm_max=30000)
    assert op.rpm_for(1000) == 30000   # 30.31*1000 = 30311 -> clamped down
    assert op.rpm_for(10) == 5000      # 30.31*10   = 303   -> clamped up


def test_missing_required_column_raises(tmp_path):
    bad = "rpm,traverse_mm_min,pass\n15000,100,1\n"   # no n_over_v / feed / etc.
    p = tmp_path / "bad.csv"
    p.write_text(bad)
    with pytest.raises(ValueError):
        select_operating_point(str(p), mode="auto", tol=5)
