# Rotoforge Slicer — Claude Code project guide

Custom slicer / toolpath generator for the **Rotoforge** AFRB (additive friction
rotational bonding) friction wire-deposition machine. Emits RepRapFirmware G-code
for X, Y, Z, the rotary wheel axis (firmware `A`, functionally `C` about Z), and
the wire feeder `E`.

**The authoritative brief is `docs/rotoforge_slicer_SPEC.md` — read it before
implementing anything.** This file is the short version plus the rules that must
not drift across sessions.

## How to work here

- Build in **milestone order** (SPEC §11): M1 geometry → M2 emitter parity →
  M3 process window → M4 contact/collision → M5 curved fill → M6 GUI → M7
  packaging. Finish and test each before starting the next.
- `pytest -q` **must stay green.** Add tests for every new module. Never weaken an
  existing invariant or its test just to get to green.
- **Config-driven.** No machine/process magic numbers in code — read them from
  `config/machine_duet3.yaml` through `rotoforge_slicer.config`. The rotary axis
  letter is a config value (`A` today, `C` after the firmware rename); never
  hardcode it.
- **Lazy heavy imports.** Keep trimesh / PySide6 / matplotlib imported *inside*
  the functions that use them so `import rotoforge_slicer` and the core tests stay
  light. Planning and emission code depend only on the `GeometryBackend` ABC and
  shapely polygons — never on a specific mesh library.

## Non-negotiable invariants (these protect the hardware and the part)

Violating any of these grinds material away, crashes the head, or ruins the
build. The emitter must **prove** none are violated (SPEC §6.3):

1. **No grinding.** in-contact ⟺ (XY speed ≥ grind floor) AND (E feeding). A
   spinning wheel that is in contact while stopped, too slow, or not feeding is
   subtractive. Use `toolpath.statemachine.assert_contact_invariant`.
2. **All dwells airborne.** Startup settle (~10 s) and between-pass spindle
   stabilization happen with the wheel lifted — never in contact.
3. **Tangential tool, no privileged direction (D13).** Wheel heading = travel
   direction at all times (commanded drift ≈ 0); the C axis tracks the path tangent,
   so `A` is always the travel heading. +Y home is only the axis **zero** reference —
   it has no deposition meaning. There is **no wedge**: every heading deposits, raster
   may be **bidirectional**, and closed contours are allowed. Curves/loops are limited
   solely by the slew rate (`R ≥ v/ω_C`, invariant 6) and the C axis's usable
   continuous angular range `[a_min_deg, a_max_deg]` (no full 360°) — track accumulated
   axis angle and insert **airborne unwinds** when a sweep would exceed it. Use
   `fill.wedge` (heading↔A + `within_axis_range` + winding) and `toolpath.passplan`
   (`split_on_winding`). See `docs/DECISIONS.md` D13.
4. **Monotonic E.** Wire never retracts. Pass-to-pass separation is a mechanical
   cut at a lead-out, not negative E.
5. **Constant revs/mm per pass.** revs/mm = RPM / traverse = the screener's
   `n_over_v`. Within one pass, traverse **and** RPM are constant. RPM changes
   only between passes, airborne (the SuperPID can't be chased mid-move).
6. **Pass geometry obeys the curve limit.** R ≥ v/ω_C everywhere in a pass at the
   pass's single speed; otherwise break the path with an airborne reorient.
7. **Lift ≥ 10 mm between passes; the collision body is the 50 mm disc + the
   leading wire**, not a point. Lead away from existing tall material, never into
   it.

## The process window is the interface

The FRAM screener CSV (SPEC §5) governs deposition — do not invent process
physics. Each stable cell fully sets `(RPM, traverse, wire feed)` for a pass.
Extrusion coupling: `e_per_path_mm = feed_speed_mm_min / traverse_mm_min`.

## Commands

```bash
python3 -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt -r requirements-dev.txt
pytest -q                         # must pass
rotoforge-slice --help            # CLI
bash packaging/build_linux.sh     # one-file exe (Windows: packaging\build_windows.bat)
```

## Already implemented — build on these, don't rewrite

config loading, `fill/wedge.py`, `fill/curvature.py`, `process/extrusion.py`,
`process/screener.py`, `toolpath/statemachine.py` (the grinding invariant),
`emit/templates.py`, and the validators in `emit/rrf.py`. 15 tests green.

## Stubbed — implement per spec, in milestone order

geometry slicing, raster/streamline fill, pass planning, collision, the
`GCodeEmitter` body, and the GUI. Stubs raise `NotImplementedError` with a SPEC
section reference.

## Don't hardcode — calibrate on hardware (SPEC §13)

- `c_axis.max_speed_deg_s` sets the curve limit; it's `0` now ⇒ R_min = inf.
- RRF combined linear+rotary **feedrate** behavior (SPEC §6.2) — confirm before
  trusting constant revs/mm; default to per-segment F compensation.
- `c_axis.invert_sign` / `home_offset_deg` — calibrate to the physical wheel
  heading.
- `process.bead_width_mm` — verify the effective bead vs the 1 mm rim under
  squeeze-out.

## Gotchas

- PyInstaller's own working dirs are `build/` and `dist/` (gitignored); our
  source build files live in `packaging/` to avoid the name clash.
- `pyclipr` may compile from source on Linux — the build environment needs a C++
  toolchain (CI installs `build-essential`/`cmake`).
- M2 needs the real `afrb_yline_*.gcode` files committed to the repo to diff
  against; the emitter must reproduce them before any new features are trusted.
