from pathlib import Path

import pytest

from rotoforge_slicer.process.screener import select_operating_point

FIXTURE = Path(__file__).resolve().parent / "fixtures" / "screener_sample.csv"

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


# ---- full §5.1-schema fixture (37 columns; only the needed ones are used) ----

def test_auto_picks_widest_contiguous_run():
    op = select_operating_point(str(FIXTURE), mode="auto", tol=5)
    assert op.revs_per_mm == 150                      # the widest contiguous ray
    assert op.v_min_mm_min == 60 and op.v_max_mm_min == 140
    assert op.rpm == 15000                            # representative cell at v=100
    assert op.v_grind_floor_mm_min == 60


def test_manual_excludes_unstable_gap():
    # the nv=100 ray has an UNSTABLE cell at v=80, so the contiguous run is [50,70],
    # not the full stable span [50,90] — the band must not cross the unstable gap.
    op = select_operating_point(str(FIXTURE), mode="manual", target=100.0, tol=5)
    assert op.revs_per_mm == 100.0
    assert op.v_min_mm_min == 50 and op.v_max_mm_min == 70


def test_manual_no_match_raises():
    with pytest.raises(ValueError):
        select_operating_point(str(FIXTURE), mode="manual", target=999.0, tol=5)


def test_full_schema_parse_uses_only_needed_columns():
    op = select_operating_point(str(FIXTURE), mode="auto", tol=5)
    assert op.feed_speed_mm_min == 130 and op.phi == 1.3   # rep cell (v=100, feed=1.3v)
    assert op.t_az_c > 0 and op.torque_Nm > 0
    assert "revs/mm=150" in op.summary()


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
    assert op.rpm_for(1000) == 30000   # 150*1000 -> clamped down
    assert op.rpm_for(10) == 5000      # 150*10   -> clamped up


def test_missing_required_column_raises(tmp_path):
    bad = "rpm,traverse_mm_min,pass\n15000,100,1\n"   # no n_over_v / feed / etc.
    p = tmp_path / "bad.csv"
    p.write_text(bad)
    with pytest.raises(ValueError):
        select_operating_point(str(p), mode="auto", tol=5)
