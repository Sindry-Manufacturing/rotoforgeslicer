# Decisions

Architecture / process decision log for the Rotoforge Slicer. Newest last. Earlier
decisions (D1–D11) predate this file and live in `docs/rotoforge_slicer_SPEC.md` and
the milestone commit messages; this log starts at D12, the first decision recorded
here explicitly.

## D12 — C-axis: ±90° deposition wedge + ±180° mechanical limit (supersedes the original ±45° wedge value)

**Decision.** The depositable **deposition wedge** is ±90° from +Y home
(`c_axis.wedge_half_angle_deg: 90`); the **mechanical/firmware travel** limit is
±180° (`c_axis.a_min_deg`/`a_max_deg`), continuous within range, **no full 360°**.
The two are validated separately in the emitter (`emit/rrf.py`): `validate_heading`
enforces the deposition wedge on every deposition segment, and
`_validate_a_in_mechanical_range` enforces `[a_min_deg, a_max_deg]` on **every**
commanded A target (deposition *and* airborne).

**Context.** A hardware revision widened the reachable deposition to ~±90°, and the
axis was made continuously adjustable to ±180° for airborne positioning. The earlier
spec value was the ±45° wedge (the addendum labels that prior decision "D3").

**Rationale.** Depositing beyond ±90° (toward −Y) is physically impossible, so the
deposition wedge and the mechanical envelope are genuinely distinct limits.
Separating them lets the planner use the full ±180° travel for airborne reorientation
while a hard validator still prevents emitting a −Y bead. Before this change the
emitter conflated the two (it reused `wedge_half_angle_deg` as the "mechanical range"),
which the addendum's §1 correctness check explicitly flagged.

**Alternatives rejected.** A single ±180° limit used for both deposition and travel —
it would allow −Y deposition, which is wrong.

**Status.** Active. Supersedes the original ±45° wedge **value** (SPEC §2 item 6 /
§4.1) — not the config-driven approach, which is unchanged. The `CAxisCfg` dataclass
**default** stays 45° on purpose so the wedge-logic unit tests that construct
`CAxisCfg()` with no args keep asserting the original behavior; only the machine YAML
(and therefore real output) moved to 90°.

**Not yet implemented.** Pass-planner **winding management** (track accumulated A;
insert an airborne unwind when a reorientation would exceed ±180°) is deferred — at
the current ±90° deposition wedge with shortest-rotation airborne reorientation,
commanded A stays within ±90° and never approaches the ±180° stops, so no unwind is
needed yet. It becomes relevant with the wider arcs of M11/M17; the mechanical-range
validator already guards the invariant in the meantime.

> **SUPERSEDED by D13** — the whole "deposition wedge / −Y impossible" framing was a
> misread of the head geometry; see below.

## D13 — Tangential deposition, no wedge; limits are C-axis slew rate + angular range (supersedes D3, D12)

**Decision.** There is **no deposition "wedge" and no privileged direction**. The
per-instant rule is simply: wheel heading = travel direction (commanded drift ≈ 0),
enforced by the C axis tracking the path tangent, so `A` is always the travel heading.
+Y home is only the axis **zero** reference. The whole head (feeder + wheel) rotates as
a unit, so every heading deposits identically. Curves and closed contours are limited
**only** by the C-axis max rate (`R ≥ v/ω_C`, §4.3) and its usable **continuous angular
range** (`a_min_deg`/`a_max_deg`, ≈ ±180°, no full 360°), managed by accumulated-angle
tracking + airborne unwinds (`split_on_winding` / `Pass.axis_angles`). `wedge_half_angle_deg`
and `in_wedge`/`vector_in_wedge` are **removed**.

**Context.** The earlier model (D3 ±45° → D12 ±90° "wedge", "+Y only / −Y impossible /
unidirectional raster / no closed perimeters") came from misreading the head geometry.
The head actually rotates fully as a unit, so there is no bad heading.

**Rationale.** A fully-rotating tangential head has no forbidden direction; the real
limits are rotation rate and total winding (the axis can't wrap past its stops, and
can't sweep continuously across the linear ±range seam — that crossing needs an
airborne unwind).

**Consequences.** Bidirectional raster allowed (`fill.raster_bidirectional`, default on);
closed/curved contours allowed within the angular range; **winding management is now a
core planner function**; the wedge config + validation is removed; the emitter commands
a winding-resolved, continuous A and validates `within_axis_range`. Reaches back into
M3–M5 (fill, pass planning, emitter validation, tests). The `CAxisCfg` dataclass default
no longer carries a wedge; `max_drift_deg` is added (kept 0 unless tuned).

**Not yet implemented (M17).** *Closed-loop-in-one-pass* needs the planner to **start a
closed contour at a rotational extreme** so its ~360° sweep aligns with the range; the
current `split_on_winding` is start-agnostic, so on an exactly-±180° range a closed loop
splits into arcs + unwinds (safe, just not yet optimal). The contour fill itself
(`fill/contour.py`) is M17 and not started.

**Alternatives rejected.** Keeping a wedge / privileged-direction model (physically wrong).

**Status.** Active. Supersedes D3 and D12.

## D14 — Ring seam placement: policies constrained by the winding seat window (port #3)

**Context.** Every closed ring used to start at the FIRST seatable vertex
(`rotate_ring_to_extreme`), so all rings on all layers seamed at the same
config-determined bearing — a vertical stripe of lead-out tapers (the known M17
clustering limitation). PrusaSlicer's seam engine (SeamPlacer/SeamAligned/
SeamRandom/SeamShells, AGPLv3, ported with permission of the project license)
provides the policy architecture: nearest / aligned (cross-layer chains) /
random (deterministic seeded).

**The physics that bounds the feature.** A closed ring's open-path A-band spans
`360° − δ` (δ = the heading step behind the start vertex; non-convex backtracking
only widens the span), and the band shifts by whole turns only (the axis zero is
physical). Seat slack = `W − span` where `W = a_max − a_min`:

* `W = 360` (the ±180 placeholder): the seat window is typically ONE vertex,
  pinned where A meets the range stop. One-pass rings CANNOT scatter their seams
  — the stripe is physics. (A corner step δ > 360 − W can seat sharp-cornered
  rings even on sub-360 ranges — the window is then corner-adjacent.)
* `W > 360`: the window widens by the extra range — real placement freedom.
* Off-window starts cost ≥ 1 winding split (an airborne unwind + an extra
  lead-in/lead-out pair; "exactly one" only for convex rings) and leave a second
  seam near the stop bearing.

**Decision.** `fill.seam_position: extreme | nearest | aligned | random`
(default **extreme** = the legacy behavior, byte-identical) with
`fill.seam_prefer_one_pass` (default true) restricting policies to the seat
window — but only when the windowed start actually dry-runs to one pass; a
window that cannot deliver one pass (corner splits) does not confiscate seam
freedom. Every non-baseline choice passes a **deposit-loss guard**: the rotated
ring is dry-run through the real split chain (`passplan.curved_subpaths`, shared
code — the guard cannot drift) and rejected if it drops more sub-`min_deposit_len`
bead than the extreme baseline (an aligned policy stacking a dropped sliver
vertically would build an unfused channel). A window-constrained non-extreme
policy emits a `ToolpathPlan.warnings` note (GUI summary + CLI) instead of
silently no-oping. `nearest` is a PLAN-ORDER chain (previous ring's seam), not
PrusaSlicer's emit-time nozzle position; `aligned` matches rings across layers
by bounding-box distance (the SeamShells port, no one-to-one claiming at our
ring counts) and accepts a target only when the CANDIDATE set can reach it
(`fill.seam_align_radius_mm`); `random` is arc-length-weighted over candidate
vertices with ONE fixed-seed stream per plan (plate-global — a divergence from
PrusaSlicer's per-object streams; same input still slices to identical G-code).
Interior winding cuts of unseatable rings also move with the chosen start (the
start is one of the cuts); reachability-boundary cuts stay config-fixed.

**Alternatives rejected.** Interpolated (non-vertex) seam points (breaks the
pure-rotation contract the tests pin); rear policy (max-Y has no meaning under
D13's no-privileged-direction); collision-aware seam steering (lead-outs are not
collision-swept today — a separate, pre-existing gap).

**Status.** Active. Refines the M17 ring-start rule under D13.
