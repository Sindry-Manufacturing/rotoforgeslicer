# Constraint Model Correction — tangential tool, no deposition wedge

**This supersedes the "deposition wedge" concept everywhere:** `ROADMAP_ADDENDUM.md`
§1, DECISIONS **D3** and **D12**, SPEC §2 / §4.1 / §4.2, and CLAUDE.md invariant 3.
Apply this before any further M11/M17 work — it reaches back into the M3–M5 core
(fill, pass planning, emitter validation), which encoded the wrong model.

---

## The correct model

The deposition head — **wire feeder and wheel together** — is mounted on the C
axis and rotates as one unit. Therefore:

1. **Tangential deposition, drift ≈ 0 (the only per-instant rule).** The wheel must
   point along the direction of travel at every moment. The C axis enforces this by
   tracking the path tangent continuously; A is *always* commanded equal to the
   travel heading, so commanded drift is structurally 0. Curves are deposited by the
   axis rotating in lockstep with the tangent.
2. **No privileged direction.** Because the whole head rotates, deposition quality
   is identical at every heading. There is **no forbidden direction** — +Y, −Y, and
   everything between are equally depositable. `+Y home is only the axis zero
   reference` after homing; it has no deposition meaning.
3. **Curves/closed contours are limited only by the C axis itself:**
   - **Slew rate** — `R ≥ v / ω_C` (unchanged, SPEC §4.3). The axis can't rotate
     faster than `ω_C`, so a curve too tight for the heading to keep up at speed `v`
     must slow (its own pass) or break.
   - **Usable continuous angular range** `[a_min, a_max]` (≈ ±180°, **no continuous
     360°**), set by head obstructions / subtended-angle limit. A deposition pass's
     accumulated axis angle must stay inside this range; when a path would drive A
     past a limit, insert an **airborne unwind** (lift, rotate A back across the
     range, resume).

**Closed contours** are therefore feasible: a convex loop is the heading sweeping
~360°, deposited in **one pass if the usable range spans ~360°** (start at a
rotational extreme), otherwise as **arcs with airborne unwinds** between them.
The break points come from the winding range and slew rate — **not** from any
"−Y tangent."

---

## What is removed / superseded

- The `wedge_half_angle_deg` parameter and the `in_wedge` / `vector_in_wedge`
  checks. **Deleted.**
- "+Y only", "−Y impossible", "no closed perimeters", "unidirectional raster only".
  **All false** under the correct model.
- D3 (±45° wedge) and D12 (±90° wedge / ±180° mechanical) → superseded by **D13**.

The config-driven *approach* (D3) was right; only the wedge *concept* was wrong.

---

## Work order

**`config/machine_duet3.yaml`** — under `c_axis:`
```yaml
  # REMOVE wedge_half_angle_deg (no deposition wedge).
  a_min_deg: -180          # usable CONTINUOUS angular range (head obstructions); no full 360
  a_max_deg: 180           # >>> set to the real measured range; decides closed-loop-in-one-pass
  max_speed_deg_s: 0.0     # ω_C, sets the slew/curvature limit (still must be measured)
  max_drift_deg: 0.0       # optional: allowed transient heading lag on curves (keep 0 unless tuned)
```

**`rotoforge_slicer/config.py`** — drop `wedge_half_angle_deg` from `CAxisCfg`;
keep `a_min_deg`/`a_max_deg`/`max_speed_deg_s`; add optional `max_drift_deg`.

**`rotoforge_slicer/fill/wedge.py`** — keep `heading_deg_from_vector` and
`heading_to_a_deg` (still needed to map tangent → axis angle). **Replace**
`in_wedge`/`vector_in_wedge` with `within_axis_range(a_deg, cfg)` checking
`a_min_deg ≤ a_deg ≤ a_max_deg`. (Rename the module to `fill/heading.py` if you
like — it's no longer about a wedge.)

**`rotoforge_slicer/emit/rrf.py`** — replace `validate_heading` (wedge) with
`validate_axis_angle` (within `[a_min,a_max]`). Add a winding-continuity check:
across a deposition pass, A evolves continuously (no ±360 jumps) and never exceeds
the range. Keep monotonic-E and the contact invariant unchanged.

**`rotoforge_slicer/toolpath/passplan.py`** — **winding management becomes core**:
track accumulated A along each pass; choose the pass's **starting winding** (among
`θ−home ± 360k` within range) to maximize available rotation room for that pass's
heading sweep; insert an airborne unwind when the sweep would exceed the range;
prefer the shortest legal rotation between passes.

**`rotoforge_slicer/fill/raster.py`** — allow **bidirectional** raster: alternate
the heading 180° between adjacent lines (A alternates between two in-range values,
e.g. 0 and ±180 — naturally staying within range). The inter-line reorient is
airborne (a 180° turn is far tighter than `R_min`, so no smooth U-turn); the win is
skipping the long fly-back of unidirectional raster.

**`rotoforge_slicer/fill/streamline.py`** and **`fill/contour.py`** — drop the
`dy ≥ 0` / wedge clipping. Clip instead by the **winding range + slew rate**;
closed contours are allowed (broken only where winding or curvature forces it).

**`CLAUDE.md`** — replace invariant 3 with:
> 3. **Tangential tool, no privileged direction.** Wheel heading = travel direction
>    at all times (drift ≈ 0); the C axis tracks the path tangent. +Y home is only
>    the axis zero. Curves and closed contours are limited solely by the slew rate
>    (`R ≥ v/ω_C`, invariant 6) and the C axis's usable continuous angular range
>    `[a_min,a_max]` (no full 360°) — track accumulated axis angle and insert
>    airborne unwinds. **No deposition wedge; raster may be bidirectional; closed
>    contours are allowed within the angular range.**

**`docs/rotoforge_slicer_SPEC.md`** — §2 and §4.1: replace the wedge wording with
the model above (drift≈0, axis-range + slew limits). §4.2: raster may be
bidirectional; streamline/contour may form closed loops within the angular range;
remove "no closed perimeters". §4.3 (curvature) is unchanged.

**`docs/DECISIONS.md`** — append:
> ## D13 — Tangential deposition, no wedge; limits are C-axis slew rate + angular range (supersedes D3, D12)
> **Decision.** No deposition "wedge" and no privileged direction. Per-instant rule:
> wheel heading = travel direction (drift ≈ 0), via the C axis tracking the tangent.
> +Y home is only the axis zero. The whole head (feeder + wheel) rotates together, so
> every heading deposits identically. Curves/closed contours are limited only by the
> C-axis max rate (`R ≥ v/ω_C`) and its usable continuous angular range
> (`a_min`/`a_max`, ~±180°, no full 360°), managed by accumulated-angle tracking +
> airborne unwinds.
> **Context.** Earlier model treated a ±45→±90 wedge with "+Y only / −Y impossible /
> unidirectional raster / no closed perimeters", from a misread of the head geometry.
> The head actually rotates as a unit, so there is no bad heading.
> **Rationale.** A fully-rotating tangential head has no forbidden direction; the real
> limits are rotation rate and total winding.
> **Consequences.** Bidirectional raster allowed; closed/curved contours allowed
> within the angular range; winding management is now a core planner function; the
> wedge config/validation is removed. Touches M3–M5 code.
> **Alternatives rejected.** Keeping a wedge / privileged-direction model (wrong).
> **Status.** Active. Supersedes D3 and D12.

---

## Revised milestones

**M11 (placement & orientation) — the metric changes meaning.** With no wedge,
orientation no longer affects "out-of-wedge area." It still matters for: the number
of **winding unwinds** and **curvature-forced breaks** a part needs, plus overhang /
layer adhesion. So the live heatmap/score becomes **reorientation-break + unwind
count and curvature feasibility** for the current orientation (color regions whose
paths would force breaks/unwinds or violate `R ≥ v/ω_C`). Auto-orient minimizes
breaks/unwinds (+ overhang). The interactive 3D scene (pyvista) is unchanged; drop
the wedge-fan overlay (show the +Y home reference and, optionally, predicted
break/unwind locations instead).

**M17 (contour tracing) — cleaner and more powerful.** Concentric contours offset
by bead pitch (pyclipr); each contour is a **full ring deposited in one pass when
its ~360° heading sweep fits the angular range** (start at a rotational extreme),
otherwise split into arcs with airborne unwinds. Break points are set by the winding
range + slew rate, not by tangent direction. Modes (`perimeter` / `contour` /
`outline`) unchanged. The annulus acceptance case becomes "traces each ring,
breaking only where winding/curvature requires," not "broken at −Y tangents."

---

## Tests to update (and the invariant harness)

- **Remove** the wedge tests.
- **Add:** `within_axis_range` accepts in-range A and rejects beyond `a_min/a_max`;
  commanded drift is 0 along sampled curves; winding accumulation never exceeds the
  range; an unwind is inserted exactly when a sweep would exceed it.
- **Bidirectional raster:** adjacent lines have headings 180° apart, both in range.
- **Closed contour:** a circle is planned as **one pass** when `a_max-a_min ≥ 360`,
  and as **arcs + unwinds** when the range is smaller — the harness should test both
  by parameterizing `a_min/a_max`.
- Update the **annulus** corpus case: it should now produce traced rings, **not** a
  rejection.

---

## Hardware unknowns to pin (was SPEC §13)

- **Usable continuous angular range** (`a_min`/`a_max`) after head obstructions —
  decides closed-loop-in-one-pass vs arcs+unwinds. *Newly the most consequential
  number.*
- `ω_C` (`max_speed_deg_s`) — the slew/curvature limit (still pending).
- A sign / home offset — calibrate to the physical head.
