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

- **Screener dialog: independent RPM/traverse selection + sane axes (user
  feedback, validated on a real FRAM export).** The user's real parameter-
  screener export (Al1100, 30 kRPM grid, ~7 400 cells — now a test fixture) is
  a rectangular RPM × traverse grid, not ray-structured data: the ray-first
  selection flow fought it, and unclipped constant-revs/mm rays (nv up to
  3 000) drove the plot's RPM axis toward 1e6. Now: **RPM and traverse are
  independent targets** — each snap lands on the nearest measured STABLE cell
  (`screener.nearest_stable_cell`, axis-normalized; never interpolated
  physics), or click a cell on the map directly; the implied revs/mm ray +
  contiguous run stay highlighted for context; Apply pins exactly the
  displayed cell (WYSIWYG unchanged, verified end-to-end against the real
  CSV). Axes clamp to the measured data, rays clip to the window, and grid
  data (> 40 rays) draws only the selected ray. The real export then REPLACED
  the synthetic `screener_sample.csv` as THE screener fixture (user request):
  the §5 selection tests recalibrated against probed real values (auto pick
  nv≈30.31, run 623..1184 mm/min, rep cell RPM 22941 @ 904 mm/min; gap
  exclusion on the nv≈29.08 ray whose cold cells break the low end), and the
  end-to-end emit test asserts the real header. Auto selection on the 7 400-
  cell grid took 12.4 s (per-stable-row candidates × full-table rescans with
  fresh string parsing) — `load_rows` now parses numerics once and the
  candidate loop uses an nv-sorted bisect window + per-nv cache: 3.0 s,
  identical selection by construction. 278 tests green.
- **Seam placement landed (PrusaSlicer port #3; D14).** `fill.seam_position:
  extreme | nearest | aligned | random` chooses each closed ring's start,
  constrained by the **winding seat window** (`seat_window` — all starts whose
  A-band seats at some winding, the O(N) generalization of the extreme scan).
  The load-bearing physics (D14): at a 360°-wide range the window is ~one vertex
  pinned where A meets the range stop — one-pass rings cannot scatter; freedom
  comes from a wider calibrated range, or `seam_prefer_one_pass: false` which
  trades ≥ 1 winding split per ring. Policies (SeamPlacer/SeamAligned/SeamRandom
  /SeamShells architecture, scoring replaced by our seat/deposit constraints):
  `nearest` = plan-order chain; `aligned` = cross-layer chains matched by ring
  bounding-box with a candidate-reachable acceptance radius; `random` =
  arc-length-weighted, fixed-seed deterministic. Every non-baseline choice
  passes a **deposit-loss guard** (dry-run through the shared
  `passplan.curved_subpaths` split chain; never drop more sub-min-length bead
  than the extreme baseline — an aligned sliver drop would stack into a vertical
  unfused channel). Window-constrained policies surface a `ToolpathPlan.warnings`
  note in the GUI summary and CLI. Default `extreme` is byte-identical to the
  legacy behavior; studio gains a "Ring seam" combo + one-pass checkbox + align
  radius. The known M17 seam-clustering limitation is now ADDRESSABLE, not
  auto-fixed: scattering is a physics trade the config makes explicit.
  Pre-commit campaign (2 design critics reshaped the design: deposit-loss
  guard, one-pass-only-when-it-buys-one-pass, warnings channel, ring-bbox
  aligned identity; then find/verify — 4 defects fixed with regressions:
  the one-pass guard now also rejects windowed starts that SPLIT at a corner
  the baseline absorbs (teardrop fixture), unknown seam_position values warn
  and degrade to extreme instead of silently scattering as random, a ring-less
  layer no longer wipes the aligned/nearest chain, and the random candidate
  order is lazy). 272 tests green.
- **Presets + project save/load landed (PrusaSlicer port #2).** `presets.py` ports
  the Preset/PresetBundle architecture (Preset.cpp/PresetBundle.cpp): three preset
  types — **machine / material / process** — each owning an explicit static list of
  dotted config keys (partition test: every `Config` key claimed exactly once;
  `process.wheel_diameter_mm` is machine hardware), collections with the
  edited-overlay model (select discards edits, dirty = diff vs saved,
  `save_current` carries `inherits` as pure annotation), **sparse user preset
  files** (only diffs vs base — `machine_duet3.yaml` stays authoritative for
  untouched keys after recalibration), compose = ordered dict-apply over a deep
  copy of the base, `screener.csv_path` on the `profile_print_params_same`-style
  ignore list. `studio/project.py` ports the 3MF container (3mf.cpp) as a
  `.rfproj` zip: embedded binary-STL meshes (deduped for duplicated parts;
  untransformed — transforms live in the manifest), full flat config snapshot +
  selected preset names, embedded screener CSV (embedded copy WINS on load;
  source path kept as sticky provenance), atomic tmp+`os.replace` save,
  format-version write/accept-up-to constants, substitute-and-report (never
  abort) on unknown keys / bad values — coercion types come from the dataclass
  DEFAULTS (the yaml parses `110` as int where the field is float). Studio:
  three preset selector rows (canonical name in itemData, "(modified)" via
  setItemText, `activated`-only signal), Open/Save project buttons, project-load
  reconciliation (clean / modified / external "(project)" presets, port of
  `load_external_preset`), and the **`_sync_bundle` invariant** (widgets→cfg→
  overlays before every bundle read, capture after every direct cfg writer) so a
  preset switch can never drop screener-dialog state or another type's edits.
  `_apply_params` became **changed-only** with widget-rendered baselines: an
  out-of-range/off-grid config value (e.g. hand-tuned `lead_in 0.3` under the
  0.5 spinbox floor) survives open→save untouched. Pre-commit review: 3 design
  critics reshaped the design (sync invariant, sparse files, ignore list,
  changed-only write-back), then a 4-finder + 2-skeptic-per-finding campaign
  confirmed 12 defect classes — all fixed: preset/project values are
  **sanitized at load** (one bad hand-edited value can no longer brick the
  console-less frozen studio at launch), `TypeError` values substitute instead
  of aborting a project open, malformed `studio_state.yaml` degrades to
  defaults (and is written atomically), the embedded-CSV binding survives
  restarts and preset switches (content-hashed temp extraction, sticky
  provenance in the screener dialog), model-only projects keep current
  settings, machine-calibration drift in a project logs a hardware warning,
  `collision.enabled=false`/`dry_run` restored from presets/projects warn
  loudly, external "(project)" names are idempotent, case-only preset name
  collisions are rejected, non-leaf keys can't replace config sections, and a
  smaller-plate machine switch can't teleport parts. 255 tests green.
- **Direction decision: PrusaSlicer by PORTING, not forking.** Evaluated forking
  PrusaSlicer-master (~1M lines C++; MSVC present, deps superbuild required) to
  "add a Rotoforge mode": rejected — every pipeline stage encodes FFF physics that
  actively violates our invariants (retraction vs monotonic E, layer-height
  travels vs airborne rule, no rotary axis in any motion type, volumetric flow vs
  screener cells, no contact model), and the emitter-side invariant PROOFS would
  restart from zero inside a foreign codebase. Instead we port PrusaSlicer's
  subsystems into the validated stack, using the real source as reference (user
  approved GPL/AGPL). Ported so far: preview UX (earlier), and now **arrange**.
- **Auto-arrange landed (PrusaSlicer arrange port).** `studio/arrange.py` ports
  the `slic3r-arrange` architecture: ArrangeItem (hull + inflation + priority +
  fixed obstacles), RectangleBed with an inset (our lead-out envelope, so a valid
  arrangement passes `issues()` by construction), first-fit-decreasing selection,
  TM-kernel scoring (big items minimize pile-bbox growth + gravity to the bed
  sink; small items nest at the pile centroid; 2% big/small split). NFP candidate
  generation is replaced by a coarse-to-fine grid + shapely collision (exact
  enough at our part counts). Studio: Arrange button + spacing spin (default
  30 mm — clears the 50 mm wheel body); unplaced parts reported. 204 tests green.
- **Adversarial review of auto-heading/preview — 10 findings fixed.** The big one
  (hardware): the emitter's plunge split creates a chord→segment A junction the
  validators never saw; when the split landed mid-segment past original vertices,
  the firmware would interpolate the step across an arbitrarily short in-contact
  remainder — axis-infeasible, XY dragged below the grind floor. `plunge_split`
  now snaps multi-segment plunges to an original vertex (mid-segment splits stay
  only within the first segment, where chord == heading) and the emitter validates
  both junction steps. Also: `vertex_step_ok` gains an unconditional <90°
  forward-dominance cap (179°-over-1mm "skids" passed the pure product bound);
  reachability reversal now runs BEFORE the step check (reversal swaps which leg
  is "next"); straight raster lines reverse when their heading is unreachable on a
  calibrated sub-360° range; the crosshatch delta is composed INTO the scored
  heading candidates (scored == laid; a rib that can't fill at any tilted heading
  falls back to un-tilted — coverage beats crosshatch); heading scoring prefers
  fewest pieces among candidates within 5% coverage (kept-length is quantization
  noise between viable directions); switching to Prepare stops playback and
  abandons mid-gesture drags; the layer-range slider no longer deadlocks with
  coincident handles. 198 tests green.
- **Over-segmentation fixed with real-part data (user's Y-axis bracket gcode+STL).**
  Diagnosis: (1) the fixed +Y hatch/bias chops regions that don't align with +Y —
  thin ribs became rib-width crossings (their gcode: 14 286 passes, p50 13 mm, and
  a 40 mm rib perpendicular to +Y used to produce NOTHING — every 3 mm crossing
  dropped below min_deposit_len); (2) the 15° flat heading-step rule conflated
  corner sharpness with sampling density and shredded legally-tight streamlines
  (30% of bead dropped on a test layer). Fixes: **per-region auto heading**
  (`fill.auto_heading`, default on — raster SCORES candidate directions on the
  actual clipped hatch with legacy +Y always a candidate, so coverage never
  regresses; streamline biases along the region's long axis: measured ~33% fewer
  passes and far less overlap at equal coverage), and the step rule became a
  **scrub budget** `c_axis.max_scrub_deg_mm` (step × next-segment length, plus an
  ω-feasibility bound) — corners (90°×19 mm = 1710) split, sampled curves
  (14°×6 mm = 87) pass.
- **PrusaSlicer-style preview (GPL-ok wholesale copy of the UX).** Preview now
  HIDES the model (the mesh no longer occludes paths; a "model shells" toggle
  ghosts it back at 18%), with a **vertical dual-handle layer-range slider** beside
  the viewport (drag either handle, or the window between them) and a **horizontal
  move slider** revealing the top visible layer move-by-move. Move-class toggles
  default to deposition-only. Prepare shows the model, Preview shows the paths.
- **Resizable UI.** The right side is a vertical splitter (viewport / controls /
  log — every boundary draggable), the left panel scrolls and resizes via the main
  splitter, readouts word-wrap; nothing is clipped at small window sizes. 194
  tests green.
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
- **M17 remainder:** the contour core landed (see Recent changes); seam placement
  landed as port #3 (D14 — scattering is now a config choice; at the ±180
  placeholder range one-pass rings still pin to the range stop by physics).
  Still open: contour-direction interaction with the collision approach rule
  under tall builds, and lead-out moves are not collision-swept (pre-existing;
  the wire-lead probe covers only the first 2 mm of the takeoff).
