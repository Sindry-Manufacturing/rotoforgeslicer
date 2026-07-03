"""Per-region automatic hatch heading (D13: no privileged direction) — the
over-segmentation fix: thin ribs fill lengthwise, and the scored raster pick can
never do worse than the legacy +Y because +Y is always a candidate."""
import math

import pytest

shapely = pytest.importorskip("shapely")
from shapely import affinity  # noqa: E402
from shapely.geometry import Polygon  # noqa: E402

from rotoforge_slicer.config import Config  # noqa: E402
from rotoforge_slicer.fill.raster import (  # noqa: E402
    best_heading_deg, dominant_heading_deg, raster_lines, raster_pitch,
)
from rotoforge_slicer.geometry import Layer  # noqa: E402
from rotoforge_slicer.process.screener import OperatingPoint  # noqa: E402
from rotoforge_slicer.toolpath.passplan import plan_layer  # noqa: E402


def _rib(angle_deg=0.0, length=40.0, width=3.0, at=(190.0, 110.0)):
    r = Polygon([(-length / 2, -width / 2), (length / 2, -width / 2),
                 (length / 2, width / 2), (-length / 2, width / 2)])
    r = affinity.rotate(r, angle_deg, origin=(0, 0))
    return affinity.translate(r, at[0], at[1])


def _op(v=120.0, rpm=5000):
    return OperatingPoint(revs_per_mm=rpm / v, v_min_mm_min=v, v_max_mm_min=v,
                          rpm=rpm, traverse_mm_min=v, feed_speed_mm_min=0.0,
                          phi=0.0, torque_Nm=0.0, power_kW=0.0, t_az_c=0.0)


def test_dominant_heading_follows_the_long_axis():
    assert dominant_heading_deg(_rib(0.0)) == pytest.approx(0.0, abs=1.0)
    assert dominant_heading_deg(_rib(30.0)) == pytest.approx(30.0, abs=1.0)
    assert dominant_heading_deg(_rib(120.0)) == pytest.approx(120.0, abs=1.0)


def test_best_heading_never_keeps_less_bead_than_legacy_plus_y():
    cfg = Config()
    pitch = raster_pitch(cfg)
    min_len = cfg.process.min_deposit_len_mm

    def kept(region, h):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1])
                   for a, b in raster_lines(region, pitch, h, min_len=min_len))

    for region in (_rib(0.0), _rib(30.0), _rib(90.0),
                   Polygon([(180, 100), (200, 100), (200, 120), (180, 120)])):
        h = best_heading_deg(region, cfg, min_len)
        assert kept(region, h) >= kept(region, 90.0) - 1e-6   # +Y is a candidate


def test_thin_rib_fills_lengthwise_with_few_long_passes():
    # the user-reported case: a rib perpendicular to +Y used to shatter into
    # rib-width crossings (or drop entirely, width < min_deposit_len)
    cfg = Config()
    cfg.fill.auto_heading = True
    layer = Layer(0, 0.06, [_rib(0.0, length=40.0, width=3.0)])
    lp = plan_layer(layer, cfg, operating_point=_op(), e_per_path=1.0)
    assert lp.passes                                          # legacy +Y: nothing!
    lens = sorted(p.length_mm for p in lp.passes)
    assert lens[len(lens) // 2] > 30.0                        # lengthwise passes

    cfg.fill.auto_heading = False
    lp_legacy = plan_layer(layer, cfg, operating_point=_op(), e_per_path=1.0)
    assert lp_legacy.passes == []                             # 3mm crossings all drop


def test_streamline_auto_heading_biases_along_the_rib():
    pytest.importorskip("scipy")
    cfg = Config()
    cfg.fill.mode = "streamline"
    cfg.fill.auto_heading = True
    cfg.c_axis.max_speed_deg_s = 360.0
    layer = Layer(0, 0.06, [_rib(0.0, length=40.0, width=4.0)])
    lp = plan_layer(layer, cfg, operating_point=_op(), e_per_path=1.0)
    assert lp.passes
    assert max(p.length_mm for p in lp.passes) > 20.0         # runs along the rib
