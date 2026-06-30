"""M5 +Y-biased streamline fill. SPEC §4.2."""
import math

import pytest

pytest.importorskip("shapely")
pytest.importorskip("scipy")
from shapely.geometry import Polygon  # noqa: E402

from rotoforge_slicer.config import Config  # noqa: E402
from rotoforge_slicer.fill.streamline import streamline_fill  # noqa: E402


def _seg_headings(path):
    return [math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
            for a, b in zip(path, path[1:])]


def _length(path):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


def test_streamlines_obey_wedge_and_forward():
    cfg = Config()
    region = Polygon([(0, 0), (20, 0), (20, 30), (0, 30)])
    paths = streamline_fill(region, cfg, heading_deg=90.0)
    assert paths
    home, wedge = cfg.c_axis.home_heading_deg, cfg.c_axis.wedge_half_angle_deg
    for path in paths:
        assert len(path) >= 2
        assert _length(path) >= cfg.process.min_deposit_len_mm
        assert path[-1][1] >= path[0][1]                       # net forward (+Y)
        for h in _seg_headings(path):
            assert home - wedge - 1e-6 <= h <= home + wedge + 1e-6   # in the wedge


def test_streamlines_curve_to_follow_a_disc():
    cfg = Config()
    cfg.fill.streamline_curl = 0.8
    region = Point_buffer(15.0)
    paths = streamline_fill(region, cfg, heading_deg=90.0)
    # at least some path bends (a heading that is not exactly +Y) — genuine curved fill
    assert any(any(abs(h - 90.0) > 5.0 for h in _seg_headings(p)) for p in paths)


def test_empty_region_returns_no_paths():
    assert streamline_fill(Polygon(), Config(), heading_deg=90.0) == []


def Point_buffer(r):
    from shapely.geometry import Point
    return Point(0, 0).buffer(r, quad_segs=48)
