"""Rotoforge AFRB slicer. See docs/rotoforge_slicer_SPEC.md."""
from __future__ import annotations

__version__ = "0.1.0"

# Only light imports at package import time. geometry/, gui/ and pipeline pull
# heavy dependencies (trimesh, PySide6, matplotlib) and are imported lazily.
from .config import load_config, Config  # noqa: E402,F401

__all__ = ["__version__", "load_config", "Config"]
