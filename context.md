# Rotoforge Slicer â€” project context & status

*Snapshot: 2026-07-03, branch `feat/slicer-m1-m2`, HEAD `54c1b15`, 272 tests green,
`dist/RotoforgeSlicer.exe` (606 MB) rebuilt 2026-07-03 14:20 and launch-verified.*

A living orientation document: where the project is, what has been accomplished,
the current state, and the agreed next steps. Detailed history lives in
`docs/PROGRESS.md`; the authoritative spec is `docs/rotoforge_slicer_SPEC.md`;
decisions in `docs/DECISIONS.md`.

---

## 1. What this project is

A custom slicer + toolpath generator for the **Rotoforge AFRB** (additive friction
rotational bonding) wire-deposition machine. It converts meshes into
RepRapFirmware G-code driving X, Y, Z, the rotary wheel axis (`A`, functionally C
about Z), and the wire feeder `E` â€” under hard, hardware-protecting invariants
(CLAUDE.md): no grinding, airborne dwells, tangential tool with winding/slew/scrub
limits (D13 â€” **no wedge**), monotonic E, constant revs/mm per pass, curve limit,
collision body = 50 mm disc + leading wire. **The emitter proves every invariant
on emitted G-code** â€” nothing is trusted from planning.

## 2. Accomplished

### Core pipeline (M1â€“M7, all complete)
- **M1 geometry**: trimesh load/repair + planar slicing â†’ shapely regions
- **M2 emitter**: SPEC Â§6 RRF emitter proving the Â§6.3 invariants (afrb_yline
  bit-parity deferred â€” reference files absent)
- **M3 process window**: FRAM screener CSV â†’ widest contiguous revs/mm ray,
  airborne per-pass RPM placement
- **M4 collision**: 2.5D swept-disc + leading-wire height-field check
- **M5 curved fill**: slew/curvature limit, streamline fill, per-segment A
- **M6 GUI** (classic, `--classic` in the exe) and **M7 packaging** (one-file exe)

### Constraint model (D13) â€” the big correction
Tangential tool, **no deposition wedge**, no privileged direction. Limits are the
slew rate (`R â‰¥ v/Ï‰_C`), the usable continuous axis range `[a_min, a_max]` with
winding management + airborne unwinds, per-vertex **scrub budget**
(`max_scrub_deg_mm`: heading-step Ã— next-segment length, plus Ï‰-feasibility and an
unconditional <90Â° cap), and reverse-direction deposition for headings a sub-360Â°
range cannot reach.

### Rotoforge Studio (the current GUI; exe default)
- 3D build plate: multi-part load, **drag-to-move**, double-click select/move,
  **lay flat**, world-frame Â±90Â° turns, scale, drop-to-bed, live fit/overlap
  checks, camera presets, dimensions readout
- **Auto-arrange** (PrusaSlicer arrange port): first-fit-decreasing + TM-kernel
  scoring; spacing default 30 mm (clears the wheel body); bed inset = lead-out
  envelope, so arranged plates pass placement checks by construction
- **PrusaSlicer-style preview**: mesh hidden in Preview (optional ghost shells),
  vertical dual-handle layer-range slider, horizontal move slider revealing the
  top layer move-by-move, move-class toggles (deposition-only default)
- **Kinematic simulation**: play/pause/speed/scrub with the vertical 50 mm wheel +
  wire-heading arrow tracking commanded A; RPM / traverse / revs-per-mm / E
  readouts; emitter-faithful timings (arc-length plunge, slew-floored travels,
  airborne dwells)
- **Graphical process window**: screener map (stable cells, revs/mm rays, the
  contiguous stable window), cell-snapped traverse/RPM selection (WYSIWYG â€” Apply
  pins exactly what is displayed), bed + hotshoe temps (wired into the preamble),
  named per-material profiles (`config/materials.yaml`)
- ~17 advanced parameters exposed; fully resizable splitter layout

### Fill modes
- **Raster** (default) with **per-region auto heading** (scored candidates; legacy
  +Y always a candidate so coverage never regresses; crosshatch composed into the
  scoring) and bidirectional boustrophedon
- **Streamline** (long-axis auto bias â€” measured ~33% fewer passes on real
  bracket geometry at equal coverage)
- **M17 contour / outline / perimeter walls**: concentric rings via shapely
  erosion; closed rings deposit as ONE pass when the range can wind them
  (rotational-extreme start); arcs + unwinds / reversals otherwise; walls after
  infill; thin regions inset by walls that actually fit

### Verification culture
- **204 pytest tests**, all green; the lazy-import guard keeps the core light
- **Three multi-agent adversarial review campaigns** (~80 agents total) confirmed
  and fixed **31 real defects**, including three hardware-grade classes that
  passed all validators: corner scrubbing via the circumradius proxy, the
  unvalidated plunge-junction A step, and travel timing ignoring A rotation
- Real-part validation: the user's Y-axis bracket G-code + STL drove the
  over-segmentation diagnosis (fixed +Y heading = rib-width crossings) and its fix

### Strategic decision: PrusaSlicer by PORTING, not forking
Forking ~1M lines of C++ whose FFF physics violates our invariants was evaluated
and rejected; instead we port its subsystems from the user's source zip
(`C:\Users\Unit-006\Downloads\PrusaSlicer-master.zip`, GPL/AGPL approved).
**Ports #1 (auto-arrange), #2 (presets + projects), #3 (seam placement) are done.**

### Seam placement (Port #3, `fill/contour.py` + D14)
- `fill.seam_position: extreme | nearest | aligned | random` picks each closed
  ring's start within the **winding seat window**; deposit-loss-guarded against
  the extreme baseline (never drop more sub-6mm bead)
- The physics (D14): at the Â±180 placeholder range one-pass rings pin to the
  range stop â€” scattering needs a wider calibrated range or
  `seam_prefer_one_pass: false` (â‰¥ 1 extra unwind + lead pair per ring); a
  window-constrained policy warns in the GUI summary/CLI instead of no-oping

### Presets + projects (Port #2, `presets.py` + `studio/project.py`)
- **Machine / Material / Process presets** (PresetBundle architecture): explicit
  per-type key ownership (partition test), edited-overlay selection model with
  "(modified)" dirty labels, sparse YAML preset files under
  `config/presets/<type>/` (base `machine_duet3.yaml` stays authoritative for
  untouched keys), selections persisted in `config/studio_state.yaml`
  (`ROTOFORGE_DATA_DIR` overrides; `~/.rotoforge` when frozen)
- **`.rfproj` project files** (3MF-container architecture): zip of embedded
  binary-STL meshes + six-float transforms + full flat config snapshot + preset
  identity + embedded screener CSV (embedded copy wins on load); atomic save;
  version-gated; unknown/bad config content substitutes-and-reports, never aborts
- Studio: preset selector rows + Open/Save project; `_sync_bundle` invariant
  keeps widgets/cfg/overlays coherent; changed-only `_apply_params` so lossy
  widgets can't corrupt off-grid config values; project load reconciles presets
  clean/modified/external and invalidates stale preview/playback state

## 3. Current state

| Item | State |
|---|---|
| Branch / HEAD | `feat/slicer-m1-m2` @ `54c1b15`, working tree clean |
| Tests | 272 passing (`pytest -q` via AppData CPython 3.11) |
| Exe | `dist/RotoforgeSlicer.exe` (606 MB), studio default, `--classic` = old GUI |
| Deps | + pyvista 0.48 / pyvistaqt 0.12 (runtime); PyInstaller 6.21 builds it |
| Config | `config/machine_duet3.yaml`; Ï‰_C=360Â°/s measured; range Â±180Â° is a **placeholder** |

### Known limitations (honest)
- **Coverage on ribby parts**: ~50â€“70% on the bracket mid-layers â€” thin features
  below `min_deposit_len_mm` (6 mm, a hardware constraint) drop; no gap-fill yet
- **Streamline mode** has no bead-spacing enforcement (overlap/gaps unquantified)
- **Ring seams at a 360Â°-wide range pin to the range stop** â€” physics, not a bug
  (D14); port #3 delivers the policy machinery, and scattering becomes real when
  the axis range is calibrated wider than 360Â° (or via the explicit one-pass
  trade). Lead-out moves are still not collision-swept (pre-existing gap;
  follow-up task flagged)
- **M2 bit-parity** deferred (afrb_yline reference files still absent)
- RRF combined linear+rotary feed semantics remain a SPEC Â§13 calibration item
- M11 remainder: in-viewport drag/rotate gizmos, break/unwind heatmap

## 4. Next steps

1. **Hardware calibration** (SPEC Â§13): measure the real `a_min/a_max` range â€”
   now doubly important: it decides closed-loop-in-one-pass AND how much seam
   scattering the seat window allows (D14); verify RRF combined-feed behavior,
   calibrate `max_scrub_deg_mm` against observed bead quality, bead width under
   squeeze-out
2. **Coverage improvements**: gap-fill / thin-feature strategy for regions the
   6 mm minimum currently abandons
3. **M11 remainder**: transform gizmos, reorientation-break/unwind heatmap
4. Presets follow-ups (small): promote materials.yaml profiles to material
   presets in the screener dialog (`material_preset_from_profile` exists),
   remember last-used dirs in file dialogs
5. After any code change: rebuild the exe (`python -m PyInstaller
   packaging/rotoforge_slicer.spec --noconfirm`) â€” a onefile exe never picks up
   new code by itself

## 5. Session commit trail (newest first)

```
54c1b15  Seam placement: PrusaSlicer seam-engine port, seat-window constrained (port #3)
90a8030  context.md: snapshot after port #2
f34ff10  Presets + project save/load: PrusaSlicer PresetBundle/3MF port (port #2)
af6bd43  Auto-arrange: port of PrusaSlicer's arrange architecture
6fec75a  Review fixes: plunge-junction proof, <90Â° step cap, composed crosshatch, ...
49d1081  PrusaSlicer-style preview + resizable UI
94a2940  Fix over-segmentation: per-region auto heading + scrub budget
f2cb1b4  Review fixes: corner-scrub guard, reverse-arc reachability, hotshoe, drag, ...
283e5c0  Studio QoL: drag-to-move, lay flat, world turns, presets, advanced params
fddfe8f  Graphical process window: screener map, cell-snapped targets, materials
d12ec01  M17: contour/perimeter tracing with rotational-extreme ring starts
90023c0  Packaging: frozen exe launches the studio by default
8c086ad  Studio: 3D build-plate GUI + kinematic simulation
662e896  U2: tagged toolpath segments + 3D viewer
4a116c2  D13 finalize: fill/wedge.py -> fill/heading.py, plan_axis_winding stub
```
