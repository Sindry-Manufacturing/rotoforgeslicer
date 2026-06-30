"""M2 unidirectional +Y raster fill. SPEC §4.2."""
import math

import pytest

pytest.importorskip("shapely")
from shapely.geometry import Polygon  # noqa: E402

from rotoforge_slicer.config import Config  # noqa: E402
from rotoforge_slicer.fill.raster import raster_lines, raster_pitch  # noqa: E402


def test_raster_pitch_from_bead_and_overlap():
    cfg = Config()  # bead 1.0, overlap 0.15 -> 0.85
    assert math.isclose(raster_pitch(cfg), 0.85)


def test_raster_lines_box_are_forward_y():
    box = Polygon([(0, 0), (10, 0), (10, 12), (0, 12)])
    segs = raster_lines(box, pitch=1.0, heading_deg=90.0)
    assert len(segs) == 10  # x = 0.5, 1.5, ... 9.5
    for (x0, y0), (x1, y1) in segs:
        assert math.isclose(x0, x1)          # vertical (constant x)
        assert y1 > y0                        # oriented forward (+Y)
        assert math.isclose(y0, 0.0, abs_tol=1e-9)
        assert math.isclose(y1, 12.0, abs_tol=1e-9)


def test_raster_lines_split_by_hole():
    outer = Polygon([(0, 0), (10, 0), (10, 12), (0, 12)],
                    [[(3, 3), (7, 3), (7, 9), (3, 9)]])
    segs = raster_lines(outer, pitch=1.0, heading_deg=90.0)
    through_hole = [s for s in segs if abs(s[0][0] - 4.5) < 1e-6]
    assert len(through_hole) == 2  # the central hole splits the line into two


def test_raster_lines_min_len_drops_short():
    flat = Polygon([(0, 0), (10, 0), (10, 3), (0, 3)])  # only 3 mm in Y
    assert raster_lines(flat, pitch=1.0, heading_deg=90.0, min_len=6.0) == []


def test_raster_lines_heading_45_runs_along_heading():
    box = Polygon([(0, 0), (20, 0), (20, 20), (0, 20)])
    segs = raster_lines(box, pitch=2.0, heading_deg=45.0)
    assert segs
    for (x0, y0), (x1, y1) in segs:
        ang = math.degrees(math.atan2(y1 - y0, x1 - x0))
        assert math.isclose(ang, 45.0, abs_tol=1.0)  # forward along the heading
