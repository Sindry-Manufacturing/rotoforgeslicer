# Progress

Running status of the Rotoforge Slicer build. See `docs/rotoforge_slicer_SPEC.md`
for the milestone plan and `docs/DECISIONS.md` for decisions.

## Milestones

- **M1 — geometry** ✅ mesh load/repair + planar `section_multiplane` slicing → shapely regions.
- **M2 — straight fill + emitter** ✅ unidirectional +Y raster, constant-(v,RPM) pass planning, SPEC §6 RRF emitter proving the §6.3 invariants. (afrb_yline_* bit-parity deferred — reference files absent.)
- **M3 — process window** ✅ FRAM screener handshake, widest-contiguous revs/mm ray, per-pass airborne RPM placement.
- **M4 — contact & collision** ✅ 2.5D swept-disc + leading-wire height-field check, lead-away pass ordering.
- **M5 — curved fill** ✅ curvature/slew limit (`max_speed_deg_s=360`), +Y-biased streamline fill, cross-layer crosshatch, per-segment curved emission with `R ≥ R_min`.
- **M6 — GUI** ✅ PySide6 app: open mesh, tweak process fields + C-axis A-min/A-max angular range, slice off-thread, scrub layers, inspect toolpath, save validated G-code.
- **M7 — packaging** ✅ one-file PyInstaller exe (verified build + launch) + per-OS build scripts + CI matrix.

## Recent changes

- **Adversarial review of M17/screener/QoL — 13 defect classes fixed.** The big ones:
  **(1) corner scrubbing (hardware-critical):** sharp polygon corners slipped past the
  circumradius proxy while the firmware interpolates the A step across the whole next
  in-contact segment — new per-vertex rule `c_axis.max_heading_step_deg` (15°): the
  planner splits corners into airborne reorients (`split_on_heading_step`) and the
  emitter **proves** it; **(2) sub-360° ranges:** closed rings no longer abort — arcs
  with unreachable headings deposit in **reverse** (`split_unreachable`; D13, no
  privileged direction), so contour works on a truthfully calibrated machine (<180°
  ranges still fail loud); **(3) hotshoe was dead config:** the preamble now emits
  `process.hotshoe_macro`; **(4) drag-to-move actually works now:** fresh z-buffer
  world picks at the live event position + camera style disabled during grabs
  (pyvista's cached point-picker path returned stale/vertex-snapped points and the
  VTK abort call was a no-op); **(5) WYSIWYG screener:** Apply pins the displayed ray
  (auto-highlight could diverge from the pipeline's auto pick), all dialog state is
  local until Apply, profiles with missing CSVs load temps only, `traverse_target`
  outside the stable run fails loud; also: infill inset by walls that actually FIT
  (thin-rib voids), wall↔hatch spacing = one pitch, cfg deep-copied to the slice
  worker, planner/emitter A-tolerance unified, O(N) ring-seat scan, area-weighted
  lay-flat normals, changed-field-only transform writes. 190 tests green.
- **M17 — contour / perimeter tracing — landed (core).** New `fill/contour.py`:
  concentric wall centrelines via shapely erosion (bead/2 first, pitch steps; hole
  walls come free), simplified rings, and the D13 **rotational-extreme start** —
  each closed ring is scanned for a start whose A-band seats at ONE winding, so a
  full loop deposits as a **single pass** on a ≥360° range (verified: a disc slices
  to 13/13 closed one-pass rings). Sub-360° ranges reject clearly (headings
  unreachable at any winding — unwinds can't create reachable headings); non-convex
  rings sweeping past the range width split into arcs + airborne unwinds. Modes:
  `fill.mode: contour | outline`, plus `fill.perimeter_loops: N` walls around
  raster/streamline infill (infill inset past the walls, walls deposited last).
  Same constraint pipeline as streamlines (slew split → winding split → min-len).
- **Graphical process window + material profiles + parameter exposure.** The studio
  gains a "Process window / material…" dialog: the screener map drawn on the RPM ×
  traverse plane (stable/unstable cells, constant-revs/mm rays, the selected ray's
  contiguous stable run, the chosen cell), ray picking, a cell slider + RPM target
  that **snap to measured cells** (never interpolated physics), bed / hotshoe
  temperature targets (`Hotshoe_{T}C.g` macro naming), and named per-material
  profiles (`config/materials.yaml` via `studio/materials.py`).
  `select_operating_point` gains `traverse_target` (nearest-cell snap;
  `screener.traverse_target` in config); new public `distinct_rays` / `ray_run` /
  `widest_ray`. The studio also exposes ~16 **advanced parameters** (lead-in/out,
  clearances, feeds, slew, collision, streamline/contour knobs, dry-run) in a
  collapsible group.
- **Studio QoL (M11).** Direct **drag-to-move** (press a part grabs it — camera
  orbit suppressed via VTK observer abort; empty-plate drags still orbit),
  **lay flat** (largest convex-hull face down, area-weighted normals),
  **world-frame X/Y/Z +90° turns** (euler decomposition keeps the transform fields
  canonical), reset transform, live part-dimensions readout, and Top/Front/Right/
  Iso/Fit camera presets. 182 tests green.
- **Packaging: the frozen exe now opens the studio.** `packaging/launch_gui.py`
  defaults to `studio.app` (`--classic` reopens the M6 GUI); the spec collects
  `pyvista`/`pyvistaqt`/`vtkmodules` so the 3D viewport ships. Rebuilt
  `dist/RotoforgeSlicer.exe` (638 MB, was 551) verified by launch: default window
  title "Rotoforge Studio", `--classic` → "Rotoforge Slicer". (The previous exe was
  a stale Jun-30 M7 build — onefile artifacts never pick up new code.)
- **Studio — 3D build-plate GUI + kinematic simulation (M11 core) — landed.** New
  `studio/` package on the existing validated core (no core rewrite — the invariants
  stay untouched): `scene.py` (pure placement math: pivot-centred transforms,
  automatic drop-to-bed, fit / lead-out / overlap checks, **multi-part slicing** via
  per-part repair + trimesh concatenate, placement replacing `place_on_bed`),
  `simulate.py` (pure kinematic timeline mirroring the emitter's feeds + airborne
  spindle dwells; monotonic E; per-instant state with wheel-heading recovery),
  `viewport.py` (pyvista build plate / parts / color-coded tagged toolpath / posed
  head disc + heading arrow), `app.py` (pyvistaqt window: click-select, click-move,
  transform panel, off-thread slicing, move-class toggles + layer scrubber, play /
  pause / speed / scrub with RPM · traverse · revs-per-mm · E readouts).
  `gui/model.py` grew `preview_from_model` (pipeline tail for scene-sliced models).
  pyvista + pyvistaqt are now runtime deps. Launch: `python -m rotoforge_slicer.studio`.
  A multi-agent adversarial review confirmed and fixed 8 defects pre-commit: lead-in E
  over the plunge **arc** (not chord), a **slew floor** on reorienting travels
  (`ΔA/ω_C` — a 180° bidirectional flip can't take 20 ms), the lead-out envelope
  reserved on **all sides** (bidirectional raster leads out −Y), repair-on-**copy**
  (never mutate the scene's meshes), the worker slicing a scene **snapshot**,
  stage-boundary slice cancellation on close, **double-click** picking (orbit drags
  must not teleport parts) with an in-volume guard, and the head drawn as the real
  **vertical** wheel (rim at contact). 163 tests green.
- **U2 — tagged toolpath segments + 3D viewer — landed.** New `toolpath/segments.py`
  turns a plan into tagged, fully-3D `ToolpathSegment`s (deposition / lead-in / lead-out /
  liftoff / reset / travel), walking the emitter's §6.1 motion sequence so the drawn
  coordinates match the emitted G-code **move-for-move** (cross-checked in
  `tests/test_segments.py`, straight + curved). The GUI's shared viewport gains a **3D
  toolpath tab** (`preview.plot_toolpath_3d`, matplotlib mplot3d) beside the 2D layer view,
  sharing the layer scrubber, with **five independent color-coded toggles** for the move
  classes (liftoff = airborne Z retract, reset = airborne Z approach, travel = airborne XY
  hop). `build_preview` now carries `segments`. 138 tests green.
- **Constraint-model correction — tangential tool, NO wedge (D13) — landed.** Supersedes
  the entire "deposition wedge" framing (D3/D12). The head rotates as a unit, so every
  heading deposits identically; `A` always equals the travel heading (drift ≈ 0). The
  only C-axis limits are the **slew rate** (`R ≥ v/ω_C`) and the **usable continuous
  angular range** `[a_min_deg, a_max_deg]`. **Removed** `wedge_half_angle_deg` /
  `in_wedge` / `vector_in_wedge` / `validate_heading`. **Added:** `within_axis_range`,
  `unwrap_headings`, `winding_shift` (`fill/heading.py`); `split_on_winding` +
  `Pass.axis_angles` (winding-resolved, continuous, in-range A) in `passplan.py`;
  **bidirectional raster** (`fill.raster_bidirectional`); the emitter now commands a
  continuous winding-resolved A and validates `within_axis_range` + winding continuity;
  `max_drift_deg` config. Streamlines drop the wedge clamp (slew + winding split instead).
  Docs: CLAUDE.md invariant 3, SPEC §2 item 6 / §4.1 / §4.2 / §6.3 / §11 / §9, DECISIONS
  D13. GUI swaps the wedge field for A-min/A-max. 132 tests green.
  - **Earlier (now superseded):** the ROADMAP_ADDENDUM §1 ±90° wedge + ±180° mechanical
    limit (D12) — kept in DECISIONS for history.

## Queued (not started)

- **M11 remainder — placement polish:** the studio delivered the M11 core (3D viewport,
  multi-part placement, drop-to-bed, click-move). Still open: in-viewport **drag/rotate
  gizmos** and the live **reorientation-break / unwind + curvature-feasibility heatmap**
  (the wedge-coverage metric is gone — D13).
- **M17 remainder:** the contour core landed (see Recent changes) including the
  closed-loop-in-one-pass refinement. Still open: seam scattering (every ring starts
  near the same rotational extreme, so lead-outs cluster in one sector) and
  contour-direction interaction with the collision approach rule under tall builds.
