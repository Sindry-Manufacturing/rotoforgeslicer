# PyInstaller spec — Rotoforge Slicer. SPEC §8.
# Build per-OS (PyInstaller cannot cross-compile):
#   pyinstaller packaging/rotoforge_slicer.spec --noconfirm
# Produces a one-file executable in dist/ (no COLLECT step => onefile).
# Requires the runtime deps installed in the build environment.

from PyInstaller.utils.hooks import collect_all

datas, binaries, hiddenimports = [], [], []
for pkg in ("trimesh", "shapely", "rtree", "pyclipr", "matplotlib", "PySide6"):
    try:
        d, b, h = collect_all(pkg)
        datas += d
        binaries += b
        hiddenimports += h
    except Exception as exc:  # pragma: no cover
        print(f"[spec] collect_all({pkg}) skipped: {exc}")

datas += [("config/machine_duet3.yaml", "config")]
hiddenimports += ["rotoforge_slicer"]

block_cipher = None

a = Analysis(
    ["rotoforge_slicer/gui/app.py"],
    pathex=["."],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=["packaging/hooks"],
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
