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
