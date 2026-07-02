"""GUI slice-preview model: run the pipeline once, cache per-layer plans + validation
results so the GUI can scrub layers instantly. Thin wrapper over the same pipeline the
CLI uses (SPEC §9). Heavy deps are imported lazily inside ``build_preview``.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

from ..config import Config


@dataclass
class SlicePreview:
    """Everything the GUI needs from one slice, cached for scrubbing."""

    cfg: Config
    model: object                # geometry.SlicedModel (placed)
    plan: object                 # passplan.ToolpathPlan
    segments: list = field(default_factory=list)   # toolpath.segments.ToolpathSegment (U2)
    collisions: list = field(default_factory=list)
    collisions_by_layer: Dict[int, list] = field(default_factory=dict)
    gcode: Optional[str] = None          # None if §6.3 validation failed
    validation_error: Optional[str] = None
    operating_point: object = None
    source: str = ""

    @property
    def layer_count(self) -> int:
        return len(self.plan.layers)

    def layer(self, i: int):
        """(geometry Layer, LayerPlan, collisions) for layer index ``i``."""
        return self.model.layers[i], self.plan.layers[i], self.collisions_by_layer.get(i, [])

    @property
    def nonempty_indices(self) -> List[int]:
        return [i for i, lp in enumerate(self.plan.layers) if lp.passes]

    def summary_lines(self) -> List[str]:
        p = self.plan
        out = [
            f"layers: {len(p.nonempty_layers)}/{len(p.layers)} non-empty",
            f"passes: {p.npasses}",
            f"operating point: RPM {p.rpm}, traverse {p.traverse_mm_min:g} mm/min, "
            f"revs/mm {p.revs_per_mm:.3f}",
        ]
        if self.operating_point is not None:
            out.append(self.operating_point.summary())
        if self.collisions:
            out.append(f"COLLISIONS: {len(self.collisions)} (see red rings) — first: "
                       f"{self.collisions[0].detail}")
        else:
            out.append("collisions: none (SPEC §4.6)")
        if self.validation_error:
            out.append(f"VALIDATION FAILED (§6.3): {self.validation_error}")
        else:
            out.append("validation: all §6.3 checks pass; G-code ready to save")
        return out


def build_preview(mesh_path: str, cfg: Config, screener_csv: Optional[str] = None,
                  progress: Optional[Callable[[float, str], None]] = None) -> SlicePreview:
    """Slice -> place -> plan -> collision check -> emit, reporting progress. Unlike the
    CLI pipeline this does NOT raise on a collision (the GUI shows them); a §6.3 emitter
    error is caught and surfaced in ``validation_error`` instead of aborting."""
    def tick(frac, msg):
        if progress:
            progress(frac, msg)

    from ..geometry.slicing import place_on_bed, slice_model
    from ..geometry.trimesh_backend import TrimeshBackend

    tick(0.05, "loading mesh…")
    backend = TrimeshBackend()
    mesh = backend.load(mesh_path)

    tick(0.25, "slicing layers…")
    model = slice_model(backend, mesh, cfg.process.layer_height_mm)
    model = place_on_bed(model, cfg)
    return preview_from_model(model, cfg, screener_csv, progress=progress,
                              source=mesh_path)


def preview_from_model(model, cfg: Config, screener_csv: Optional[str] = None,
                       progress: Optional[Callable[[float, str], None]] = None,
                       source: str = "") -> SlicePreview:
    """The pipeline tail for an already-sliced, already-placed model: operating point
    -> plan -> collision check -> segments -> emit. The studio scene (multi-part GUI
    placement) enters here — its placement replaces ``place_on_bed``."""
    def tick(frac, msg):
        if progress:
            progress(frac, msg)

    op = None
    if screener_csv:
        from ..process.screener import select_operating_point

        tick(0.45, "selecting operating point…")
        op = select_operating_point(
            screener_csv, mode=cfg.screener.revs_per_mm_mode,
            target=cfg.screener.revs_per_mm_target, tol=cfg.screener.revs_per_mm_tol,
            rpm_min=cfg.spindle.rpm_min, rpm_max=cfg.spindle.rpm_max,
            traverse_target=cfg.screener.traverse_target)

    tick(0.55, "planning passes…")
    from ..toolpath.passplan import plan_toolpath

    plan = plan_toolpath(model, cfg, operating_point=op)

    collisions: list = []
    if cfg.collision.enabled:
        tick(0.8, "collision check…")
        from ..toolpath.collision import replay_collision_check

        collisions = replay_collision_check(plan, cfg)

    by_layer: Dict[int, list] = {}
    z_to_index = {round(ly.z, 6): i for i, ly in enumerate(model.layers)}
    for c in collisions:
        i = z_to_index.get(round(c.z, 6))
        if i is not None:
            by_layer.setdefault(i, []).append(c)

    # Tagged 3D toolpath segments for the viewer (U2). Pure geometry mirroring the
    # emitter's motion, so it is built even when §6.3 validation below fails — you can
    # still inspect the path that tripped the check.
    from ..toolpath.segments import build_segments

    segments = build_segments(plan, cfg)

    tick(0.92, "emitting G-code…")
    from ..emit.rrf import GCodeEmitter

    gcode, err = None, None
    try:
        gcode = GCodeEmitter(cfg).emit(plan)
    except ValueError as e:
        err = str(e)

    tick(1.0, "done")
    return SlicePreview(cfg=cfg, model=model, plan=plan, segments=segments,
                        collisions=collisions, collisions_by_layer=by_layer,
                        gcode=gcode, validation_error=err,
                        operating_point=op, source=source)
