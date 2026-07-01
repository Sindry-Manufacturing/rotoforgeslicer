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
