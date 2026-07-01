"""Guard the lazy-heavy-import rule (CLAUDE.md): importing the package and its
geometry / preview / pipeline modules must NOT pull trimesh, shapely, matplotlib,
PySide6, or pyclipr at module-import time.

Runs in a clean subprocess with those packages blocked at import, so it guards the
invariant even in a dep-complete environment (no importorskip — this test always
runs). A regression that adds a top-level heavy import would make the subprocess
fail to import and turn this test red.
"""
import subprocess
import sys
import textwrap
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

_CHILD = textwrap.dedent(
    """
    import sys
    HEAVY = {"trimesh", "shapely", "matplotlib", "PySide6", "pyclipr"}

    class Blocker:
        def find_spec(self, name, path=None, target=None):
            if name.split(".")[0] in HEAVY:
                raise ImportError("blocked heavy import: " + name)
            return None

    sys.meta_path.insert(0, Blocker())

    import rotoforge_slicer
    import rotoforge_slicer.geometry
    import rotoforge_slicer.geometry.backend
    import rotoforge_slicer.geometry.slicing
    import rotoforge_slicer.geometry.trimesh_backend
    import rotoforge_slicer.geometry.meshlib_backend
    import rotoforge_slicer.gui.preview
    import rotoforge_slicer.gui.app
    import rotoforge_slicer.gui.model
    import rotoforge_slicer.pipeline
    import rotoforge_slicer.fill.raster
    import rotoforge_slicer.fill.heading
    import rotoforge_slicer.fill.curvature
    import rotoforge_slicer.fill.streamline
    import rotoforge_slicer.toolpath.passplan
    import rotoforge_slicer.toolpath.collision
    import rotoforge_slicer.emit.rrf
    import rotoforge_slicer.emit.templates

    # Pure-python helpers must work with all heavy deps blocked.
    assert rotoforge_slicer.geometry.layer_heights(0.0, 1.0, 0.5) == [0.25, 0.75]
    assert rotoforge_slicer.gui.preview._sample_indices(3, 2) == [0, 2]

    leaked = sorted(m for m in HEAVY if m in sys.modules)
    assert not leaked, "heavy modules imported at module load: " + repr(leaked)
    print("LAZY_OK")
    """
)


def test_core_and_geometry_imports_stay_light():
    res = subprocess.run(
        [sys.executable, "-c", _CHILD],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
    )
    assert res.returncode == 0, f"stdout={res.stdout!r}\nstderr={res.stderr!r}"
    assert "LAZY_OK" in res.stdout
