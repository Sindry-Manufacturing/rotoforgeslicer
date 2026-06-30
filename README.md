# Rotoforge Slicer

Custom slicer / toolpath generator for the **Rotoforge** AFRB (additive friction
rotational bonding) friction wire-deposition machine. Converts a 3D mesh into
RepRapFirmware G-code driving X, Y, Z, the rotary wheel axis (firmware `A`,
functionally `C` about Z), and the wire feeder `E`.

Full design spec: **`docs/rotoforge_slicer_SPEC.md`**.

## Status

Scaffold + **M1 (geometry) complete.** **Implemented and tested:** config loading,
heading<->A-axis mapping and the +/-45 deg wedge check, the curvature/slew limit,
extrusion ratios, the contact-"grinding" invariant, screener operating-point
selection, the G-code preamble/postamble, and **M1: mesh load + repair + planar
`section_multiplane` slicing -> shapely region polygons (`geometry/`), plus a
matplotlib per-layer preview (`gui/preview.py`)**. **Stubbed** (next, per the spec):
raster/streamline fill, pass planning, collision, the G-code emitter body, and the GUI.

## Quickstart

```bash
python3 -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt -r requirements-dev.txt
pytest -q                  # core tests pass
rotoforge-slice --help     # CLI (pipeline stubbed)
```

## Layout

```
rotoforge_slicer/      package
  config.py            YAML -> dataclasses                 [done]
  pipeline.py          orchestrator                        [stub]
  cli.py               headless CLI                        [done/stub]
  geometry/            load + repair + planar slice         [M1 done]
  fill/                wedge, raster, streamline, curvature [wedge+curvature done]
  toolpath/            state machine, pass plan, collision  [invariant done]
  process/             screener CSV, extrusion              [done]
  emit/                RRF G-code emitter, templates        [templates+validators done]
  gui/                 PySide6 app + matplotlib preview     [stub]
config/                machine_duet3.yaml
docs/                  rotoforge_slicer_SPEC.md
packaging/             PyInstaller spec + per-OS build scripts
tests/                 pytest suite (core pieces green)
.github/workflows/     CI matrix build (Windows + Linux)
```

## Build one-click executables

PyInstaller can't cross-compile — build on each OS:

```bash
bash packaging/build_linux.sh         # -> dist/RotoforgeSlicer
packaging\build_windows.bat           # -> dist\RotoforgeSlicer.exe
```

`.github/workflows/build.yml` builds both as artifacts on tag push or manual run.

## Building it out

Scaffolded to be completed with Claude Code, milestone by milestone (SPEC §11):
M1 geometry -> M2 emitter parity against the existing `afrb_yline_*` G-code ->
M3 process window -> M4 contact/collision -> M5 curved fill -> M6 GUI -> M7 packaging.

## Calibrate before first print (SPEC §13)

- `c_axis.max_speed_deg_s` — measure (sets the curvature limit `R_min = v/omega`).
- RRF combined linear+rotary **feedrate** behavior (SPEC §6.2) — confirm on firmware.
- `c_axis.invert_sign` / `home_offset_deg` — calibrate to the physical wheel heading.
- `process.bead_width_mm` — verify effective bead vs the 1 mm rim under squeeze-out.
