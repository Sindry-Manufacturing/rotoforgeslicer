"""Auto-arrange (PrusaSlicer-structure port): packing validity, spacing, the
big-first selection, obstacles, overflow, determinism, and scene integration."""
import pytest

shapely = pytest.importorskip("shapely")
from shapely import affinity  # noqa: E402
from shapely.geometry import box  # noqa: E402

from rotoforge_slicer.config import Config  # noqa: E402
from rotoforge_slicer.studio.arrange import (  # noqa: E402
    ArrangeItem, RectangleBed, arrange,
)

BED = RectangleBed(380.0, 235.0, inset_mm=4.0)


def _item(w, h, inflation=15.0, priority=0):
    return ArrangeItem(outline=box(0, 0, w, h), inflation_mm=inflation,
                       priority=priority)


def _placed(it):
    return affinity.translate(it.outline, *it.translation)


def test_arrange_places_all_within_inset_bed_and_spacing():
    items = [_item(60, 40), _item(30, 30), _item(50, 20), _item(20, 20)]
    unplaced = arrange(items, [], BED)
    assert unplaced == []
    region = BED.region()
    placed = [_placed(it) for it in items]
    for p in placed:
        assert region.buffer(1e-6).contains(p.buffer(15.0))     # inflated in bed
    for i, a in enumerate(placed):
        for b in placed[i + 1:]:
            assert a.distance(b) >= 30.0 - 1e-6                 # full spacing kept


def test_big_items_pack_first_and_toward_the_sink():
    big = _item(100, 80)
    small = _item(12, 12)
    arrange([small, big], [], BED)
    cx, cy = BED.center
    bc = _placed(big).centroid
    sc = _placed(small).centroid
    # the big item claims the centre region (TM gravity sink)…
    assert abs(bc.x - cx) < 60 and abs(bc.y - cy) < 60
    # …and the small one nests near the pile rather than a far corner
    assert bc.distance(sc) < 150


def test_fixed_items_are_obstacles():
    fixed = ArrangeItem(outline=box(160, 90, 220, 145), inflation_mm=15.0,
                        fixed=True)
    item = _item(50, 40)
    arrange([item], [fixed], BED)
    assert _placed(item).distance(fixed.outline) >= 30.0 - 1e-6


def test_overflow_reports_unplaced():
    items = [_item(200, 150), _item(200, 150), _item(200, 150)]
    unplaced = arrange(items, [], BED)
    assert unplaced and all(it.translation is None for it in unplaced)
    placed = [it for it in items if it.translation is not None]
    assert placed                                               # at least one fits


def test_arrange_is_deterministic():
    def run():
        items = [_item(60, 40), _item(30, 30), _item(50, 20)]
        arrange(items, [], BED)
        return [it.translation for it in items]

    assert run() == run()


def test_scene_arrange_clears_all_placement_issues():
    trimesh = pytest.importorskip("trimesh")
    from rotoforge_slicer.studio.scene import SceneModel

    cfg = Config()
    scene = SceneModel()
    # dump five parts at the same spot — maximal overlap to untangle
    for i in range(5):
        scene.add(trimesh.creation.box(extents=(40, 25, 5)), name=f"p{i}",
                  at=(100.0, 100.0))
    assert scene.issues(cfg)                                    # overlapping now
    unplaced = scene.arrange(cfg, spacing_mm=30.0)
    assert unplaced == []
    assert scene.issues(cfg) == []                              # clean by construction
