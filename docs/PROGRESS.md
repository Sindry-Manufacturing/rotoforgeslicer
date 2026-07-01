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

- **Constraint-model correction — tangential tool, NO wedge (D13) — landed.** Supersedes
  the entire "deposition wedge" framing (D3/D12). The head rotates as a unit, so every
  heading deposits identically; `A` always equals the travel heading (drift ≈ 0). The
  only C-axis limits are the **slew rate** (`R ≥ v/ω_C`) and the **usable continuous
  angular range** `[a_min_deg, a_max_deg]`. **Removed** `wedge_half_angle_deg` /
  `in_wedge` / `vector_in_wedge` / `validate_heading`. **Added:** `within_axis_range`,
  `unwrap_headings`, `winding_shift` (`fill/wedge.py`); `split_on_winding` +
  `Pass.axis_angles` (winding-resolved, continuous, in-range A) in `passplan.py`;
  **bidirectional raster** (`fill.raster_bidirectional`); the emitter now commands a
  continuous winding-resolved A and validates `within_axis_range` + winding continuity;
  `max_drift_deg` config. Streamlines drop the wedge clamp (slew + winding split instead).
  Docs: CLAUDE.md invariant 3, SPEC §2 item 6 / §4.1 / §4.2 / §6.3 / §11 / §9, DECISIONS
  D13. GUI swaps the wedge field for A-min/A-max. 132 tests green.
  - **Earlier (now superseded):** the ROADMAP_ADDENDUM §1 ±90° wedge + ±180° mechanical
    limit (D12) — kept in DECISIONS for history.

## Queued (not started)

- **M11 (revised) — interactive placement & orientation:** mouse-driven 3D build-plate
  viewport (pyvista/pyvistaqt) with transform gizmos; live **reorientation-break / unwind
  + curvature-feasibility** heatmap (the wedge-coverage metric is gone — D13), drop-to-bed,
  multi-part placement. Supersedes the original heatmap-only M11.
- **M17 (new) — contour / perimeter tracing fill:** concentric offsets, each ring clipped
  to **winding-range arcs** (`perimeter` / `contour` / `outline`). Includes the
  *closed-loop-in-one-pass* refinement — start a ring at a rotational extreme so its ~360°
  sweep aligns with the range (the current `split_on_winding` is start-agnostic).
