# Progress

Running status of the Rotoforge Slicer build. See `docs/rotoforge_slicer_SPEC.md`
for the milestone plan and `docs/DECISIONS.md` for decisions.

## Milestones

- **M1 — geometry** ✅ mesh load/repair + planar `section_multiplane` slicing → shapely regions.
- **M2 — straight fill + emitter** ✅ unidirectional +Y raster, constant-(v,RPM) pass planning, SPEC §6 RRF emitter proving the §6.3 invariants. (afrb_yline_* bit-parity deferred — reference files absent.)
- **M3 — process window** ✅ FRAM screener handshake, widest-contiguous revs/mm ray, per-pass airborne RPM placement.
- **M4 — contact & collision** ✅ 2.5D swept-disc + leading-wire height-field check, lead-away pass ordering.
- **M5 — curved fill** ✅ curvature/slew limit (`max_speed_deg_s=360`), +Y-biased streamline fill, cross-layer crosshatch, per-segment curved emission with `R ≥ R_min`.
- **M6 — GUI** ✅ PySide6 app: open mesh, tweak process fields + C-axis wedge, slice off-thread, scrub layers, inspect toolpath, save validated G-code.
- **M7 — packaging** ✅ one-file PyInstaller exe (verified build + launch) + per-OS build scripts + CI matrix.

## Recent changes

- **C-axis wedge update (ROADMAP_ADDENDUM §1) — landed.** The reachable **deposition
  wedge** is now ±90° from +Y (`wedge_half_angle_deg: 90`), with a separate ±180°
  **mechanical/firmware** travel limit (`a_min_deg`/`a_max_deg`). The emitter validates
  the two independently: deposition headings against the wedge, and *every* commanded A
  (deposition + airborne) against `[a_min_deg, a_max_deg]`. This also fixed a latent
  conflation where `emit/rrf.py` reused the wedge value as the "mechanical range."
  **Confirmed: the deposition wedge is 90, not 180.** Docs updated: CLAUDE.md invariant
  3, SPEC §2 item 6 / §4.1 / §4.2 / §6.3 / §11 config, DECISIONS D12. GUI exposes the
  wedge as a 0–180° field.
  - **Deferred:** pass-planner winding management (airborne unwind near the ±180° stops)
    — not needed at the current ±90° deposition wedge; revisit with M11/M17.

## Queued (not started)

- **M11 (revised) — interactive placement & orientation:** mouse-driven 3D build-plate
  viewport (pyvista/pyvistaqt) with transform gizmos, live wedge-coverage heatmap,
  drop-to-bed, multi-part placement. Supersedes the original heatmap-only M11.
- **M17 (new) — contour / perimeter tracing fill:** concentric offsets clipped to
  wedge-depositable arcs (`perimeter` / `contour` / `outline` modes).
