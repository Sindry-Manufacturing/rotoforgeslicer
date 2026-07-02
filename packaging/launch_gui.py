"""Frozen-app entry point (SPEC §8).

Launches the **studio** (3D build-plate GUI + kinematic simulation) by default;
``--classic`` opens the original M6 2D GUI instead. Mesh-file arguments are passed
through to whichever GUI starts.

PyInstaller runs the entry script as ``__main__``, which has no parent package, so
the GUI modules cannot be entry points directly — their (lazy) package-relative
imports (``from ..config``, ``from .model``) would fail. This tiny launcher imports
them by absolute path instead.
"""
import sys


def _main() -> int:
    argv = sys.argv[1:]
    if "--classic" in argv:
        from rotoforge_slicer.gui.app import main

        return main([a for a in argv if a != "--classic"])
    from rotoforge_slicer.studio.app import main

    return main(argv)


if __name__ == "__main__":
    raise SystemExit(_main())
