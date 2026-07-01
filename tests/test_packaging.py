"""M7 packaging contract: the frozen-app entry point and the PyInstaller spec. SPEC §8.

These don't build an exe (that's per-OS and slow — CI does it); they pin the wiring so
a refactor can't silently break the frozen entry point or the bundled-config path.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_launcher_entry_point_is_callable():
    # the frozen exe runs packaging/launch_gui.py, which imports this symbol
    from rotoforge_slicer.gui.app import main

    assert (ROOT / "packaging" / "launch_gui.py").exists()
    assert callable(main)


def test_spec_is_coherent():
    spec = (ROOT / "packaging" / "rotoforge_slicer.spec").read_text()
    assert "launch_gui.py" in spec                       # entry point
    assert "collect_submodules" in spec                  # bundles our lazy imports
    assert "machine_duet3.yaml" in spec                  # the config data file
    assert (ROOT / "config" / "machine_duet3.yaml").exists()
    assert (ROOT / "packaging" / "build_windows.bat").exists()
    assert (ROOT / "packaging" / "build_linux.sh").exists()


def test_default_config_finds_bundled_yaml(monkeypatch, tmp_path):
    # simulate a frozen app: sys._MEIPASS points at the bundle dir with config/ inside
    import sys

    from rotoforge_slicer.gui.app import _default_config

    (tmp_path / "config").mkdir()
    (tmp_path / "config" / "machine_duet3.yaml").write_text(
        "c_axis:\n  a_max_deg: 173\n")
    monkeypatch.setattr(sys, "_MEIPASS", str(tmp_path), raising=False)
    monkeypatch.chdir(tmp_path.parent)                   # so cwd/config doesn't shadow it
    cfg = _default_config()
    assert cfg.c_axis.a_max_deg == 173                   # loaded from the bundle
