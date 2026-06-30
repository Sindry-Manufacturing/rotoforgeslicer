"""M1 geometry: layer heights, region cleanup, and trimesh slicing. SPEC §3.1.

trimesh + shapely are declared deps (requirements.txt) but heavy; skip cleanly
where absent so the light core suite still passes.
"""
import math
from pathlib import Path

import pytest

from rotoforge_slicer.geometry import slicing

shapely = pytest.importorskip("shapely")
trimesh = pytest.importorskip("trimesh")
pytest.importorskip("scipy")  # trimesh path assembly needs it

from shapely.geometry import Polygon  # noqa: E402

from rotoforge_slicer.geometry import Layer, SlicedModel, TrimeshBackend, slice_model  # noqa: E402

CFG = Path(__file__).resolve().parents[1] / "config" / "machine_duet3.yaml"


# ----------------------------- layer_heights -----------------------------

def test_layer_heights_mid_layer_sampling():
    hs = slicing.layer_heights(0.0, 1.0, 0.25)
    assert hs == [0.125, 0.375, 0.625, 0.875]


def test_layer_heights_empty_when_no_extent():
    assert slicing.layer_heights(5.0, 5.0, 0.1) == []
    assert slicing.layer_heights(5.0, 1.0, 0.1) == []


def test_layer_heights_rejects_nonpositive_height():
    with pytest.raises(ValueError):
        slicing.layer_heights(0.0, 1.0, 0.0)


def test_layer_heights_boundary_and_negative():
    # A layer centre landing exactly on z_max is excluded (strict `z < z_max`).
    assert slicing.layer_heights(0.0, 0.5, 1.0) == []
    # Negative z_min must be handled (the real centred-box case).
    assert slicing.layer_heights(-2.5, 2.5, 1.0) == [-2.0, -1.0, 0.0, 1.0, 2.0]


# ----------------------------- clean_polygons -----------------------------

def test_clean_polygons_keeps_valid_and_drops_slivers():
    big = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])      # area 100
    sliver = Polygon([(0, 0), (0.1, 0), (0.1, 0.1), (0, 0.1)])  # area 0.01
    out = slicing.clean_polygons([big, sliver], min_area=1.0)
    assert len(out) == 1
    assert math.isclose(out[0].area, 100.0)


def test_clean_polygons_repairs_invalid():
    bowtie = Polygon([(0, 0), (2, 2), (2, 0), (0, 2)])  # self-intersecting -> invalid
    assert not bowtie.is_valid
    out = slicing.clean_polygons([bowtie])
    assert out and all(p.is_valid and not p.is_empty for p in out)


def test_clean_polygons_explodes_multipolygon():
    from shapely.geometry import MultiPolygon

    a = Polygon([(0, 0), (2, 0), (2, 2), (0, 2)])
    b = Polygon([(5, 5), (7, 5), (7, 7), (5, 7)])
    out = slicing.clean_polygons([MultiPolygon([a, b])])
    assert len(out) == 2  # disjoint parts split into separate Polygons
    assert all(isinstance(p, Polygon) for p in out)


def test_clean_polygons_recurses_geometrycollection_and_ignores_nonareal():
    from shapely.geometry import GeometryCollection, LineString, Point

    poly = Polygon([(0, 0), (2, 0), (2, 2), (0, 2)])
    gc = GeometryCollection([poly, LineString([(0, 0), (5, 5)]), Point(1, 1)])
    out = slicing.clean_polygons([gc])
    assert len(out) == 1  # the polygon is kept, the line/point are dropped
    assert math.isclose(out[0].area, 4.0)


def test_clean_polygons_preserves_holes():
    outer = [(0, 0), (10, 0), (10, 10), (0, 10)]
    hole = [(3, 3), (7, 3), (7, 7), (3, 7)]
    out = slicing.clean_polygons([Polygon(outer, [hole])])
    assert len(out) == 1
    assert len(out[0].interiors) == 1
    assert math.isclose(out[0].area, 100.0 - 16.0)


# ----------------------------- backend slicing -----------------------------

def test_backend_slices_box_to_constant_area():
    mesh = trimesh.creation.box(extents=(10, 20, 5))  # z in [-2.5, 2.5]
    backend = TrimeshBackend()
    (xmin, ymin, zmin), (xmax, ymax, zmax) = backend.bounds(mesh)
    assert (zmin, zmax) == (-2.5, 2.5)
    layers = backend.slice(mesh, [-1.0, 0.0, 1.0])
    assert len(layers) == 3
    for regions in layers:
        assert len(regions) == 1
        assert math.isclose(regions[0].area, 200.0, rel_tol=1e-6)


def test_backend_empty_layer_outside_mesh():
    mesh = trimesh.creation.box(extents=(10, 20, 5))
    layers = TrimeshBackend().slice(mesh, [10.0])  # plane above the box
    assert layers == [[]]


def test_backend_preserves_hole_for_annulus():
    # Pin tessellation so the faceting error is bounded and the tolerance can be
    # tight enough to catch a wrong-radius regression (a 128-gon is ~0.01% off).
    ann = trimesh.creation.annulus(r_min=2.0, r_max=5.0, height=4.0, sections=128)
    layers = TrimeshBackend().slice(ann, [0.0])
    polys = slicing.clean_polygons(layers[0])
    assert len(polys) == 1
    assert len(polys[0].interiors) == 1               # the central hole survives
    assert math.isclose(polys[0].area, math.pi * (25 - 4), rel_tol=0.01)


# ----------------------------- slice_model end-to-end -----------------------------

def test_slice_model_box():
    mesh = trimesh.creation.box(extents=(10, 20, 5))
    model = slice_model(TrimeshBackend(), mesh, layer_height=0.12)
    assert isinstance(model, SlicedModel)
    assert (model.z_min, model.z_max) == (-2.5, 2.5)
    assert len(model) == len(slicing.layer_heights(-2.5, 2.5, 0.12))
    assert model.nonempty_layers == model.layers  # box spans every interior layer
    for ly in model:
        assert isinstance(ly, Layer)
        assert math.isclose(ly.area, 200.0, rel_tol=1e-6)
        assert ly.bounds == pytest.approx((-5.0, -10.0, 5.0, 10.0))
    assert math.isclose(model.total_area, 200.0 * len(model), rel_tol=1e-6)


def test_load_and_slice_roundtrip_via_stl(tmp_path):
    mesh = trimesh.creation.box(extents=(8, 8, 3))
    stl = tmp_path / "box.stl"
    mesh.export(stl)
    backend = TrimeshBackend()
    loaded = backend.load(str(stl))
    assert loaded.is_watertight
    model = slice_model(backend, loaded, layer_height=0.5)
    assert len(model.nonempty_layers) == len(model) > 0
    for ly in model:
        assert math.isclose(ly.area, 64.0, rel_tol=1e-6)


def test_load_rejects_missing_or_empty(tmp_path):
    backend = TrimeshBackend()
    empty = tmp_path / "empty.stl"
    empty.write_text("")
    with pytest.raises(Exception):
        backend.load(str(empty))


# ----------------------------- empty-layer branches -----------------------------

def test_empty_layer_branches():
    p = Polygon([(0, 0), (2, 0), (2, 2), (0, 2)])
    layers = [Layer(0, 0.0, []), Layer(1, 0.1, [p]), Layer(2, 0.2, [])]
    model = SlicedModel(layers=layers, layer_height=0.1, z_min=0.0, z_max=0.2)

    assert len(model.nonempty_layers) == 1
    assert model.nonempty_layers == [layers[1]]
    # empty layers
    for empty in (layers[0], layers[2]):
        assert empty.is_empty is True
        assert empty.bounds is None
        assert empty.area == 0.0
        assert empty.union().is_empty
    # the lone non-empty layer
    assert layers[1].is_empty is False
    assert layers[1].bounds == pytest.approx((0.0, 0.0, 2.0, 2.0))
    assert math.isclose(layers[1].area, 4.0)
    assert math.isclose(model.total_area, 4.0)


# ----------------------------- mesh repair -----------------------------

def _broken_box():
    """A box with one face removed -> not watertight."""
    import numpy as np

    m = trimesh.creation.box(extents=(4, 4, 4))
    m.update_faces(np.arange(len(m.faces))[2:])  # drop the first face (2 triangles)
    m.remove_unreferenced_vertices()
    return m


def test_repair_closes_non_watertight_mesh():
    m = _broken_box()
    assert not m.is_watertight
    repaired = TrimeshBackend().repair(m)
    assert repaired.is_watertight  # fill_holes re-closed the missing face


def test_repair_is_nondestructive_on_clean_mesh():
    m = trimesh.creation.box(extents=(4, 4, 4))
    n_before = len(m.faces)
    repaired = TrimeshBackend().repair(m)
    assert repaired.is_watertight
    assert len(repaired.faces) == n_before  # nothing mangled


def test_slice_model_repair_flag_honored():
    """slice_model(repair=...) controls whether backend.repair runs."""

    class SpyBackend(TrimeshBackend):
        def __init__(self):
            self.repair_calls = 0

        def repair(self, mesh):
            self.repair_calls += 1
            return super().repair(mesh)

    mesh = trimesh.creation.box(extents=(6, 6, 4))

    spy_on = SpyBackend()
    slice_model(spy_on, mesh, layer_height=1.0, repair=True)
    assert spy_on.repair_calls == 1

    spy_off = SpyBackend()
    slice_model(spy_off, mesh, layer_height=1.0, repair=False)
    assert spy_off.repair_calls == 0


# ----------------------------- meshlib stub -----------------------------

def test_meshlib_backend_is_stub():
    from rotoforge_slicer.geometry import MeshLibBackend

    be = MeshLibBackend()  # instantiable: implements all 4 abstract methods
    for call in (lambda: be.load("x"), lambda: be.repair(None),
                 lambda: be.bounds(None), lambda: be.slice(None, [0.0])):
        with pytest.raises(NotImplementedError):
            call()


# ----------------------------- pipeline wiring -----------------------------

def test_pipeline_slice_geometry(tmp_path):
    from rotoforge_slicer.pipeline import slice_geometry

    stl = tmp_path / "box.stl"
    trimesh.creation.box(extents=(10, 10, 2)).export(stl)
    model = slice_geometry(str(stl), str(CFG))
    assert len(model) > 0
    assert all(math.isclose(ly.area, 100.0, rel_tol=1e-6) for ly in model.nonempty_layers)


def test_pipeline_slice_mesh_still_stubbed_after_geometry(tmp_path):
    from rotoforge_slicer.pipeline import slice_mesh

    stl = tmp_path / "box.stl"
    trimesh.creation.box(extents=(6, 6, 2)).export(stl)
    with pytest.raises(NotImplementedError):
        slice_mesh(str(stl), str(CFG))
