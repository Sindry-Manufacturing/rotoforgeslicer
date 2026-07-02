# PyInstaller spec — Rotoforge Slicer. SPEC §8.
# Build per-OS (PyInstaller cannot cross-compile):
#   pyinstaller packaging/rotoforge_slicer.spec --noconfirm
# Produces a one-file executable in dist/ (no COLLECT step => onefile).
# Requires the runtime deps installed in the build environment.
#
# Paths are resolved against the repo ROOT (the spec lives in packaging/), because
# PyInstaller resolves a spec's relative script paths against the SPEC's directory,
# not the cwd.
import os

from PyInstaller.utils.hooks import collect_all, collect_submodules

ROOT = os.path.dirname(SPECPATH)            # SPECPATH = .../packaging ; ROOT = repo root

datas, binaries, hiddenimports = [], [], []
# collect_all grabs each heavy package whole — our lazy, in-function imports of these
# are invisible to PyInstaller's static analysis, so we cannot rely on it tracing them.
# pyvista/pyvistaqt/vtkmodules power the studio 3D viewport; VTK's DLLs are large
# but must ship or the default (studio) entry point dies at first import.
for pkg in ("trimesh", "shapely", "rtree", "matplotlib", "PySide6", "scipy",
            "networkx", "pyvista", "pyvistaqt", "vtkmodules"):
    try:
        d, b, h = collect_all(pkg)
        datas += d
        binaries += b
        hiddenimports += h
    except Exception as exc:  # pragma: no cover
        print(f"[spec] collect_all({pkg}) skipped: {exc}")

# Our own package is imported lazily throughout, so bundle every submodule explicitly.
hiddenimports += collect_submodules("rotoforge_slicer")
datas += [(os.path.join(ROOT, "config", "machine_duet3.yaml"), "config")]

block_cipher = None

a = Analysis(
    [os.path.join(ROOT, "packaging", "launch_gui.py")],
    pathex=[ROOT],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[os.path.join(ROOT, "packaging", "hooks")],
    runtime_hooks=[],
    excludes=[],
    cipher=block_cipher,
)
pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name="RotoforgeSlicer",
    debug=False,
    strip=False,
    upx=False,
    console=False,   # set True while debugging to see tracebacks
)
