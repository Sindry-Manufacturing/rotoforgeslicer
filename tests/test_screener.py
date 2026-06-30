from rotoforge_slicer.process.screener import select_operating_point

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
