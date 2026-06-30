"""Headless CLI. SPEC §10."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from . import __version__


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="rotoforge-slice",
                                description="Rotoforge AFRB slicer")
    p.add_argument("mesh", nargs="?", help="input mesh (STL/3MF)")
    p.add_argument("-c", "--config", default="config/machine_duet3.yaml",
                   help="machine/process config YAML")
    p.add_argument("-s", "--screener", help="FRAM process-window CSV")
    p.add_argument("-o", "--output", help="output .gcode path")
    p.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    args = p.parse_args(argv)

    if not args.mesh:
        p.print_help()
        return 0

    out = args.output or str(Path(args.mesh).with_suffix(".gcode"))  # beside the mesh

    from .pipeline import slice_mesh  # lazy: pulls heavy deps
    try:
        gcode = slice_mesh(args.mesh, args.config, args.screener, out)
    except NotImplementedError as e:
        print(f"[not yet implemented] {e}", file=sys.stderr)
        return 2
    except (ValueError, FileNotFoundError) as e:
        print(f"[error] {e}", file=sys.stderr)
        return 1
    print(f"wrote {out} ({gcode.count(chr(10))} lines)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
