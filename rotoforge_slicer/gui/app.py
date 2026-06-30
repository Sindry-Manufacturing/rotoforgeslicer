"""PySide6 main window. SPEC §9.  [stub — M6]

PySide6 is imported lazily so the package imports without a GUI stack.
"""
from __future__ import annotations


def main(argv=None) -> int:
    try:
        from PySide6 import QtWidgets  # noqa: F401
    except Exception as e:  # pragma: no cover
        raise SystemExit(f"PySide6 not available: {e}")
    raise NotImplementedError("GUI not yet implemented — build per SPEC §9")


if __name__ == "__main__":
    main()
