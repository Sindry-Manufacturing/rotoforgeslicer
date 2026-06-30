"""G-code preamble/postamble built from the config's macro lists. SPEC §6.1.

Prefer CALLING the user's tuned macros (Hotshoe_*, CPAP_*) over re-emitting tuned
values.
"""
from __future__ import annotations

from ..config import Config


def preamble(cfg: Config) -> list[str]:
    lines = ["; Rotoforge Slicer preamble", "G21", "G90"]
    if cfg.gcode.use_relative_e:
        lines.append("M83")
    lines += ["M220 S100", "M221 S100", "M5  ; spindle off"]
    if cfg.process.bed_temp_c:
        lines.append(f"M140 S{cfg.process.bed_temp_c:g}")
        lines.append(f"M190 S{cfg.process.bed_temp_c:g}")
    for macro in cfg.gcode.preamble_macros:
        lines.append(f'M98 P"{macro}"')
    lines.append("; --- end preamble ---")
    return lines


def postamble(cfg: Config) -> list[str]:
    lines = ["; --- postamble ---", "M5  ; spindle off"]
    for macro in cfg.gcode.postamble_macros:
        lines.append(f'M98 P"{macro}"')
    lines += ["M140 S0", "M84"]
    return lines
