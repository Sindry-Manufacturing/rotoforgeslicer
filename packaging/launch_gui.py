"""Frozen-app entry point (SPEC §8).

PyInstaller runs the entry script as ``__main__``, which has no parent package, so
``rotoforge_slicer/gui/app.py`` cannot be the entry point directly — its (lazy)
package-relative imports (``from ..config``, ``from .model``) would fail. This tiny
launcher imports the GUI by absolute path instead.
"""
from rotoforge_slicer.gui.app import main

if __name__ == "__main__":
    raise SystemExit(main())
