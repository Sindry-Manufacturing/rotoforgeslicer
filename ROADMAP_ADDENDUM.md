# Roadmap Addendum — wedge update, interactive placement, contour tracing

Supplements `docs/USABILITY_ROADMAP.md` and `docs/rotoforge_slicer_SPEC.md`.
Three changes: (1) the C-axis deposition wedge widened to ~±90° with a separate
±180° mechanical limit; (2) M11 expanded to interactive graphical placement; (3) a
new contour/perimeter tracing fill mode (M17).

---

## 1. C-axis: ±90° deposition wedge, ±180° mechanical limit  *(apply first)*

The reachable **deposition** wedge is now ~±90° from +Y home. Separately, the axis
is **mechanically/firmware-adjustable across ±180°** (continuous within that range;
still no full 360°). These are two different numbers and must stay separate.

> **Correctness check (do this first).** Ensure the *deposition wedge* is 90, **not**
> 180. Depositing at ±180° points −Y, which is impossible. If the wedge was set to
> 180 when the axis range was widened, that's a bug — it would emit −Y beads.

### Work order

**`config/machine_duet3.yaml`** — under `c_axis:`
```yaml
  wedge_half_angle_deg: 90     # DEPOSITION wedge: reachable deposition headings = +Y ±90. NOT the mechanical limit.
  a_min_deg: -180              # mechanical/firmware travel limit (airborne positioning); continuous within [-180, 180]
  a_max_deg: 180
```

**`rotoforge_slicer/config.py`** — add `a_min_deg: float = -180.0` and
`a_max_deg: float = 180.0` to `CAxisCfg`.

**Validation (`emit/rrf.py`)** — keep `validate_heading` checking deposition
headings against `wedge_half_angle_deg`; add a separate check that **every** A
target (deposition *and* airborne) lies within `[a_min_deg, a_max_deg]`.

**Winding management (`toolpath/passplan.py`)** — the axis cannot wrap past ±180°.
Track accumulated A; when a reorientation would exceed the mechanical limit, insert
an airborne **unwind** move. Prefer the shortest rotation between headings.

**`CLAUDE.md`** — replace invariant 3 with:
> 3. **±90° deposition wedge, +Y only.** Every deposition heading within ±90° of +Y
>    home (tangent dy ≥ 0); −Y impossible; no closed perimeters. The axis travels
>    ±180° mechanically for airborne reorientation, but **deposition stays within
>    the wedge — never ±180°.** Use `fill.wedge`.

**`docs/rotoforge_slicer_SPEC.md`** — update the illustrative "±45°" in §2 item 6
and §4.1 to "±90° (config `wedge_half_angle_deg`)". The streamline criteria in §4.2
("monotonic forward / no −Y reversal") are already wedge-parametric and need no
change beyond the value.

**`docs/DECISIONS.md`** — append:
> ## D12 — C-axis: ±90° deposition wedge + ±180° mechanical limit (supersedes D3's ±45°)
> **Decision.** Deposition wedge = ±90° from +Y (`wedge_half_angle_deg: 90`);
> mechanical/firmware travel = ±180° (`a_min_deg`/`a_max_deg`), continuous within
> range, no full 360°. The two are validated separately.
> **Context.** Hardware revision widened reachable deposition to ~±90°; the axis was
> made continuously adjustable to ±180° for airborne positioning.
> **Rationale.** Depositing beyond ±90° (toward −Y) is physically impossible, so the
> deposition wedge and the mechanical envelope are distinct limits. Separating them
> prevents emitting −Y beads while still using the full travel for reorientation.
> **Alternatives rejected.** A single ±180° limit used for both (would allow −Y
> deposition — wrong).
> **Status.** Active. D3 superseded (the ±45° value, not the config-driven approach).

### Design implications (no extra work, but the planner should exploit them)

- **Depositable set = upper half-plane** (any heading with dy ≥ 0), including pure
  +X and −X. The old "monotonic +Y, |dx/dy| ≤ 1" tightening is gone; the only
  in-wedge bound is `dy ≥ 0` plus the parametric ±90° check.
- **Closed perimeters still impossible** (−Y unreachable), but open paths can now
  wrap up to ~270° and turn up to ±90° from +Y. **Reorientation breaks drop
  sharply** — fill (M5) and contour tracing (M17) both benefit automatically since
  they already read the config wedge.
- **Raster** can now be oriented along any axis if useful, but solid fill stays
  unidirectional (the return leg is still airborne, not a −Y deposit).

---

## 2. M11 (revised) — Interactive placement & orientation

Supersedes the original M11 ("orientation heatmap + auto-orient"), folding manual
graphical manipulation in. The heatmap and auto-orient become aids inside an
interactive scene.

**Goal.** Reorient and position meshes on the build plate directly with the mouse,
with live wedge feedback, before slicing.

**Scope.**
- A real-time **3D build-plate viewport**: renders the 380×235×250 build volume,
  origin, the +Y home-heading axis, and the depositable wedge fan.
- **Mouse interaction:** orbit / pan / zoom camera; **transform gizmos** to rotate
  the mesh (full 3D to choose the down-face, with emphasis and angle-snap on
  Z-rotation since that maps to the wedge) and translate on the bed (X/Y);
  **drop-to-bed** (auto-Z so the part sits on the plate); rotate-by-increment.
- **Live wedge-coverage heatmap:** the M11 coverage metric recolors the mesh as you
  rotate, showing immediately how much falls outside the ±90° wedge in the current
  orientation, plus a single slice-ability score.
- **Auto-orient** button: proposes top candidates (minimize out-of-wedge area +
  reorientation breaks); accepting one snaps the gizmo.
- **Multi-part placement:** load several meshes, position each, basic overlap check
  between parts and against the bed bounds.
- **Bounds enforcement:** warn/prevent placing outside the build volume.

**Out of scope.** Automatic nesting/packing (future); non-planar orientation.

**Tech note.** Interactive 3D with picking and gizmos needs a true 3D viewport —
use **pyvista + pyvistaqt (VTK)**, promoting it from the optional `viz3d` extra to
a GUI dependency for this view. Keep the matplotlib per-layer preview for 2D layer
inspection. Add the VTK/pyvista hidden-imports/hooks to the PyInstaller spec (the
bundle grows — note it in gotchas).

**Modules.**
- `rotoforge_slicer/gui/scene3d.py` — pyvistaqt viewport, gizmos, picking.
- `rotoforge_slicer/geometry/placement.py`
  ```python
  @dataclass
  class Placement:                  # per-mesh transform applied BEFORE slicing
      rotation_deg: tuple           # (rx, ry, rz)
      translation_mm: tuple         # (tx, ty)  + auto drop-to-bed for z
  def apply(mesh, placement): ...
  def drop_to_bed(mesh, placement): ...
  ```
- wires to `geometry/orient.py` (`wedge_coverage`, `auto_orient`).

**Dependencies.** M1 geometry; pyvista/pyvistaqt. Pairs with M9/M10 in the flow.

**Acceptance.** User loads a mesh, rotates and positions it with the mouse, sees the
wedge heatmap update live, drops it to the bed, and slices the transformed
placement. Auto-orient reduces out-of-wedge area on a misaligned part. Multi-part
overlaps and out-of-bounds placements are flagged. The placement transform is what
gets sliced (not the raw mesh).

---

## 3. M17 (new) — Contour / perimeter tracing fill mode

A third path strategy alongside raster and streamline: follow the region's outline
and inward offsets as toolpaths. Independent of Phases A–C — schedule whenever, but
it pairs naturally with the wider wedge, so consider doing it early. (If "tracing"
was meant differently than contour-following, adjust scope.)

**Goal.** Offer contour-following toolpaths (walls / concentric fill / outline
trace) instead of always rastering or streamlining.

**Scope.**
- Generate **concentric contours**: offset the region boundary inward by the bead
  pitch repeatedly (pyclipr — already a dependency) to get nested rings.
- **Clip each contour to wedge-depositable arcs:** a closed contour can't be laid
  in one go (tangential constraint), so split it into the maximal arcs whose tangent
  stays in the wedge (dy ≥ 0), breaking where the tangent dips below horizontal;
  each arc is a pass with an airborne reorient between.
- **Modes** (a `fill_mode` field on config/profile + GUI selector):
  - `perimeter` — N outer offsets as walls; interior left empty or handed to
    raster/streamline.
  - `contour` — concentric arcs all the way in (full contour fill).
  - `outline` — boundary only.
- **Respect every invariant:** each arc obeys the curve limit at its single pass
  speed, stays in-wedge, meets min-length, gets lead-in/out, and goes through the
  collision/approach check (contour direction interacts with the approach rule).
- **Ordering:** inside-out or outside-in; offset spacing = bead pitch.

**Out of scope.** Bridging across breaks; closing loops (impossible); mixed
contour+raster within one region beyond `perimeter` walls + interior fill.

**Modules.** `rotoforge_slicer/fill/contour.py`
```python
def contour_fill(region, cfg, mode="contour") -> list[Pass]: ...
```
Selectable alongside `raster_fill` / `streamline_fill`; the pipeline picks by
`fill_mode`. Reuses pyclipr offsetting, `fill.wedge`, `fill.curvature`, and
`toolpath.passplan`.

**Dependencies.** Core fill machinery (M5) and pyclipr — all present. Benefits from
§1 (wider wedge → longer arcs per pass).

**Acceptance.** For an annulus, `contour` mode traces concentric arcs broken at the
−Y tangent points (no attempt to close the ring). For a curved wall, it follows the
wall in one or few passes. A convex contour yields arcs of up to ~180° at the ±90°
wedge (vs ~90° at ±45°). Every emitted arc passes the invariant harness. Switching
`fill_mode` between raster/streamline/contour changes the toolpath with no other
input.

---

### Apply / update checklist

- [ ] §1 config + `CAxisCfg` fields + dual validation + winding management.
- [ ] §1 doc edits: CLAUDE.md invariant 3, SPEC §2/§4.1 value, DECISIONS D12.
- [ ] Confirm the deposition wedge is 90, not 180.
- [ ] M11 (revised) in the roadmap (supersede original M11); add pyvista to GUI deps + PyInstaller hooks.
- [ ] M17 added to the roadmap.
- [ ] `docs/PROGRESS.md`: note the wedge change landed; M11 now interactive; M17 queued.
