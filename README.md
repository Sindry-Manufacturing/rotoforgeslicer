# Rotoforge Slicer

Custom slicer / toolpath generator for the **Rotoforge** AFRB (additive friction
rotational bonding) friction wire-deposition machine. Converts a 3D mesh into
RepRapFirmware G-code driving X, Y, Z, the rotary wheel axis (firmware `A`,
functionally `C` about Z), and the wire feeder `E`.

Full design spec: **`docs/rotoforge_slicer_SPEC.md`**.

## Status

**M1–M7 complete** — the full slicer: geometry, straight + curved fill, the process
window, contact/collision, the GUI, and one-click executables. A later **constraint-model
correction (D13)** removed the "deposition wedge" entirely: the head rotates as a unit, so
every heading deposits (no privileged direction), raster is bidirectional, and the only
C-axis limits are the slew rate and the usable angular range `[a_min_deg, a_max_deg]` with
winding management. **Implemented and tested:** config loading, heading<->A-axis mapping,
the axis-range + winding check (`within_axis_range` / `split_on_winding`), the
curvature/slew limit, extrusion ratios, the contact-"grinding" invariant; **M1**: mesh load + repair + planar
`section_multiplane` slicing -> shapely regions (`geometry/`) + matplotlib preview
(`gui/preview.py`); **M2**: bidirectional raster fill (`fill/raster.py`),
constant-(v,RPM) pass planning (`toolpath/passplan.py`), bed placement, and a
SPEC-compliant RRF emitter (`emit/rrf.py`) that proves the §6.3 invariants; **M3**: the
FRAM screener handshake (`process/screener.py`) — CSV -> widest-contiguous revs/mm ray
-> operating point -> per-pass **airborne RPM placement** + screener E coupling, with a
CLI operating-point read-out (`rotoforge-slice mesh.stl -s window.csv`); **M4**: a
2.5D height-map collision check (`toolpath/collision.py`) — swept 50 mm disc +
leading wire vs deposited material — and the lead-away pass ordering (§4.6); **M5**:
the curvature/slew limit (`fill/curvature.py`, now calibrated `max_speed_deg_s=360`),
+Y-biased boundary-following **streamline fill** (`fill/streamline.py`) with
per-pass curvature splitting, **cross-layer crosshatch**, and a polyline pass model
emitted as per-segment curved moves with the §6.3 `R ≥ R_min` proof. Set
`fill.mode: streamline` and/or `fill.crosshatch: true` in the config to enable; **M6**:
a PySide6 GUI (`gui/`) — open a mesh, tweak process fields, Slice off the UI thread,
scrub layers with a slider, inspect the toolpath (deposition vectors, lead-outs,
wire-cuts, resets, the +Y home reference, collisions) with mouse zoom/pan, and Save the
validated G-code, with **C-axis A-min/A-max fields** to set the usable angular range
before slicing. Launch with `rotoforge-slicer-gui [mesh.stl]`;
**M7**: one-file PyInstaller executables (`packaging/rotoforge_slicer.spec` +
`launch_gui.py` frozen entry point, bundling the lazy package submodules and the
`config/` YAML read back via `sys._MEIPASS`) built per-OS by `build_windows.bat` /
`build_linux.sh` and the `.github/` CI matrix — the 526 MB Windows onefile builds and
launches from a verified spec.

> **M2 parity note:** the SPEC's `afrb_yline_*` reference G-code and
> `afrb_playground_gui(2).py` generator are not in the repo. The only existing
> reference output is from an older prototype whose closed perimeters are now
> *consistent* with the corrected constraint model (D13: no wedge, closed contours
> allowed), though its 0–220° A range still differs from the calibrated
> `[a_min_deg, a_max_deg]`. **Bit-exact `afrb_yline_*` parity is deferred** until
> those reference files are provided.

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
  geometry/            load + repair + planar slice + place [M1 done]
  fill/                heading/axis-range, raster, streamline, curvature [done]
  toolpath/            state machine, pass plan, collision  [M4 done]
  process/             screener CSV, extrusion              [M3 done]
  emit/                RRF G-code emitter, templates        [M2 emitter done]
  gui/                 PySide6 app + matplotlib preview     [M6 done]
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

- `c_axis.max_speed_deg_s` — set to the measured 360 deg/s (sets `R_min = v/omega`).
- RRF combined linear+rotary **feedrate** behavior (SPEC §6.2) — confirm on firmware.
- `c_axis.invert_sign` / `home_offset_deg` — calibrate to the physical wheel heading.
- `process.bead_width_mm` — verify effective bead vs the 1 mm rim under squeeze-out.
