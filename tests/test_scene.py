"""Studio scene model (M11): transforms, drop-to-bed, fit checks, multi-part slicing.

The placement math is pure (stub meshes with only ``.vertices``); the slicing test
pulls trimesh and is skipped where absent.
"""
import numpy as np
import pytest

from rotoforge_slicer.config import Config
from rotoforge_slicer.studio.scene import SceneModel, ScenePart


class _Stub:
    """Axis-aligned box mesh stub: vertices only (placement math needs no faces)."""

    def __init__(self, sx, sy, sz, origin=(0.0, 0.0, 0.0)):
        ox, oy, oz = origin
        self.vertices = np.array([(ox + x, oy + y, oz + z)
                                  for x in (0, sx) for y in (0, sy) for z in (0, sz)],
                                 dtype=float)


def test_add_centres_pivot_and_drops_to_bed():
    cfg = Config()
    bx, by, _ = cfg.machine.build_volume_mm
    scene = SceneModel()
    part = scene.add(_Stub(10, 20, 30, origin=(100, 100, 100)), cfg=cfg)
    lo, hi = part.bounds()
    assert lo[2] == pytest.approx(0.0)                       # rests on the bed
    assert (lo[0] + hi[0]) / 2 == pytest.approx(bx / 2)      # pivot at plate centre
    assert (lo[1] + hi[1]) / 2 == pytest.approx(by / 2)
    assert hi[2] - lo[2] == pytest.approx(30.0)


def test_rotation_tumbles_about_pivot_and_redrops():
    scene = SceneModel()
    part = scene.add(_Stub(10, 20, 30), at=(100, 100))
    part.set_transform(rot_x_deg=90.0)
    lo, hi = part.bounds()
    assert lo[2] == pytest.approx(0.0)                       # re-dropped after tumbling
    assert hi[2] - lo[2] == pytest.approx(20.0)              # y-extent is now the height
    assert hi[1] - lo[1] == pytest.approx(30.0)              # z-extent lies along Y
    assert (lo[0] + hi[0]) / 2 == pytest.approx(100.0)       # pivot stays put


def test_scale_and_move():
    scene = SceneModel()
    part = scene.add(_Stub(10, 10, 10), at=(50, 60))
    part.set_transform(scale=2.0, x=80.0)
    x0, y0, x1, y1 = part.footprint()
    assert x1 - x0 == pytest.approx(20.0)
    assert (x0 + x1) / 2 == pytest.approx(80.0)
    assert (y0 + y1) / 2 == pytest.approx(60.0)


def test_issues_out_of_volume_leadout_height_and_overlap():
    cfg = Config()
    bx, by, bz = cfg.machine.build_volume_mm
    scene = SceneModel()
    a = scene.add(_Stub(10, 10, 10), cfg=cfg)
    assert scene.issues(cfg) == []                           # centred part fits

    a.set_transform(x=2.0)                                   # footprint spills past x=0
    assert any("outside" in s for s in scene.issues(cfg))
    a.set_transform(x=bx / 2, y=by - 3.0)                    # +Y lead-out reserved (§6.3)
    assert any("lead-out" in s for s in scene.issues(cfg))
    # review fix: the envelope is reserved on ALL sides — the default bidirectional
    # raster leads out toward -Y too (footprint inside the plate, y0 < lead_out).
    a.set_transform(x=bx / 2, y=6.0)                         # footprint y in [1, 11]
    assert any("lead-out" in s for s in scene.issues(cfg))

    a.set_transform(x=bx / 2, y=by / 2)
    tall = scene.add(_Stub(10, 10, bz + 50), at=(60, 60))
    assert any("taller" in s for s in scene.issues(cfg))

    scene.remove(tall)
    b = scene.add(_Stub(10, 10, 10), at=(bx / 2 + 4, by / 2))  # footprints intersect
    assert any("overlap" in s for s in scene.issues(cfg))
    b.set_transform(x=bx / 2 + 40)
    assert scene.issues(cfg) == []


def test_duplicate_lands_beside_and_remove():
    scene = SceneModel()
    a = scene.add(_Stub(10, 10, 10), at=(100, 100))
    d = scene.duplicate(a)
    assert len(scene.parts) == 2 and d.name != a.name
    assert d.x == pytest.approx(a.x + 15.0)                  # width + 5 mm gap
    assert scene.issues(Config()) == []                      # no overlap
    scene.remove(a)
    assert scene.parts == [d]


def test_set_transform_rejects_unknown_field():
    part = ScenePart("p", _Stub(5, 5, 5))
    with pytest.raises(AttributeError):
        part.set_transform(bogus=1.0)


def test_rotate_world_is_world_frame():
    # after tumbling 90° about X, a WORLD-Z turn must spin the footprint, not re-tumble
    scene = SceneModel()
    part = scene.add(_Stub(10, 20, 30), at=(100, 100))
    part.set_transform(rot_x_deg=90.0)          # height becomes 20 (y-extent up)
    part.rotate_world("z", 90.0)
    lo, hi = part.bounds()
    assert hi[2] - lo[2] == pytest.approx(20.0)  # height unchanged by a world-Z turn
    x0, y0, x1, y1 = part.footprint()
    assert x1 - x0 == pytest.approx(30.0)        # footprint axes swapped by the spin
    assert y1 - y0 == pytest.approx(10.0)
    assert lo[2] == pytest.approx(0.0)           # still on the bed


def test_euler_roundtrip():
    import numpy as np

    from rotoforge_slicer.studio.scene import euler_zyx_deg_from_matrix

    part = ScenePart("p", _Stub(5, 5, 5))
    for angles in [(30, 40, 50), (0, 0, 0), (-120, 15, 179), (10, 90, 0)]:
        part.set_transform(rot_x_deg=angles[0], rot_y_deg=angles[1],
                           rot_z_deg=angles[2])
        r = part.rotation()
        rx, ry, rz = euler_zyx_deg_from_matrix(r)
        part.set_transform(rot_x_deg=rx, rot_y_deg=ry, rot_z_deg=rz)
        assert np.allclose(part.rotation(), r, atol=1e-9)   # same rotation recovered


def test_lay_flat_puts_largest_face_down():
    pytest.importorskip("scipy")
    scene = SceneModel()
    part = scene.add(_Stub(10, 20, 30), at=(100, 100))      # largest face 20x30 (±X)
    part.set_transform(rot_x_deg=33.0, rot_y_deg=21.0, rot_z_deg=57.0)  # tumble it
    part.lay_flat()
    lo, hi = part.bounds()
    assert hi[2] - lo[2] == pytest.approx(10.0, abs=1e-6)   # thinnest axis is vertical
    assert lo[2] == pytest.approx(0.0)                      # resting on the bed
    # lay-flat only levels the face; the tumble's yaw persists, so the FOOTPRINT is
    # the 20x30 face at an arbitrary Z rotation — its AABB stays within the diagonal.
    sx, sy, sz = part.size_mm()
    diag = (20.0**2 + 30.0**2) ** 0.5
    assert 20.0 - 1e-6 <= max(sx, sy) <= diag + 1e-6
    assert sx * sy >= 600.0 - 1e-6                          # covers at least the face


def test_snapshot_is_independent_of_the_live_scene():
    # review fix: the slice worker gets a snapshot, so live edits cannot race it.
    scene = SceneModel()
    a = scene.add(_Stub(10, 10, 10), at=(100, 100))
    snap = scene.snapshot()
    a.set_transform(x=200.0, rot_z_deg=45.0)
    scene.remove(a)
    assert len(snap.parts) == 1
    assert snap.parts[0].x == pytest.approx(100.0)           # frozen at snapshot time
    assert snap.parts[0].rot_z_deg == 0.0


def test_slice_scene_two_parts_merge_layers():
    trimesh = pytest.importorskip("trimesh")
    pytest.importorskip("shapely")
    pytest.importorskip("scipy")
    cfg = Config()
    scene = SceneModel()
    a = scene.add(trimesh.creation.box(extents=(20, 12, 3)), name="a", at=(100, 100))
    scene.add(trimesh.creation.box(extents=(20, 12, 3)), name="b", at=(160, 100))
    verts_before = a.mesh.vertices.copy()
    model = scene.slice_scene(cfg)
    # review fix: repair runs on copies — the scene's own meshes are never mutated
    # (an in-place repair would invalidate the interactive drop-to-bed state).
    assert (a.mesh.vertices == verts_before).all()
    assert model.nonempty_layers
    ly = model.nonempty_layers[0]
    assert len(ly.regions) == 2                              # both parts, one layer stack
    cxs = sorted(p.centroid.x for p in ly.regions)
    assert cxs[0] == pytest.approx(100, abs=0.5)             # placed as arranged
    assert cxs[1] == pytest.approx(160, abs=0.5)             # (no re-centring)
    assert model.z_min == pytest.approx(0.0, abs=1e-6)       # dropped to the bed
