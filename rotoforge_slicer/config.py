"""Configuration model + loader. SPEC §7.

Loads config/machine_duet3.yaml into validated dataclasses. Raises on unknown
keys so typos surface immediately.
"""
from __future__ import annotations

from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any, Mapping

import yaml


@dataclass
class StepsCfg:
    x: float = 80.0
    y: float = 80.0
    z: float = 400.0
    e_per_mm: float = 46.73
    a_per_deg: float = 26.667


@dataclass
class MachineCfg:
    name: str = "duet3"
    rotary_axis_letter: str = "A"
    build_volume_mm: tuple = (380.0, 235.0, 250.0)
    feedrate_mode: str = "per_segment_compensation"
    steps: StepsCfg = field(default_factory=StepsCfg)


@dataclass
class CAxisCfg:
    home_heading_deg: float = 90.0
    home_offset_deg: float = 0.0
    invert_sign: int = 1
    wedge_half_angle_deg: float = 45.0
    max_speed_deg_s: float = 0.0


@dataclass
class SpindleCfg:
    rpm_min: int = 5000
    rpm_max: int = 30000


@dataclass
class ProcessCfg:
    bead_width_mm: float = 1.0
    layer_height_mm: float = 0.12
    wire_diameter_mm: float = 0.50
    wheel_diameter_mm: float = 50.0   # the collision body (deposition is the ~1mm rim; SPEC §1.5)
    raster_overlap: float = 0.15
    min_deposit_len_mm: float = 6.0
    inter_pass_lift_mm: float = 10.0
    lead_in_len_mm: float = 2.0       # moving-plunge forward distance (SPEC §4.4)
    approach_clearance_mm: float = 0.5  # airborne gap above the surface before the moving plunge
    lead_out_len_mm: float = 4.0
    travel_z_mm: float = 10.0
    startup_settle_ms: int = 10000
    spindle_dwell_ms: int = 2000
    cpap_deposit: int = 255
    bed_temp_c: float = 110.0
    hotshoe_macro: str = "Hotshoe_300C.g"


@dataclass
class CollisionCfg:
    """2.5D swept-disc + leading-wire collision check (SPEC §4.6)."""
    enabled: bool = True
    cell_mm: float = 0.0       # height-field cell; 0 => bead_width/2 (fine enough vs pitch)
    clearance_mm: float = 0.3  # required gap between the disc body and existing material
    wire_lead_mm: float = 2.0  # how far the fragile leading wire reaches ahead of contact


@dataclass
class EmitCfg:
    """G-code feedrate primitives (SPEC §6). Defaults are the validated values
    seen in the existing tool's output (travel 2500, Z 300, deposition 120 mm/min).
    ``dry_run`` disables spindle/heaters/fan/E for a motion-only test file."""
    feed_travel_mm_min: float = 2500.0
    feed_z_mm_min: float = 300.0
    feed_dep_mm_min: float = 120.0     # also the default traverse when no screener
    dry_run: bool = False


@dataclass
class ExtrusionCfg:
    mode: str = "screener"   # screener | x | volume
    x_ratio: float = 1.0


@dataclass
class ScreenerCfg:
    csv_path: str = ""
    revs_per_mm_mode: str = "auto"   # auto | manual
    revs_per_mm_target: float = 0.0
    revs_per_mm_tol: float = 5.0


@dataclass
class GcodeCfg:
    preamble_macros: list = field(default_factory=lambda: ["Hotshoe_300C.g", "CPAP_100pct.g"])
    postamble_macros: list = field(default_factory=lambda: ["CPAP_OFF.g", "Hotshoe_OFF.g"])
    use_relative_e: bool = True


@dataclass
class Config:
    machine: MachineCfg = field(default_factory=MachineCfg)
    c_axis: CAxisCfg = field(default_factory=CAxisCfg)
    spindle: SpindleCfg = field(default_factory=SpindleCfg)
    process: ProcessCfg = field(default_factory=ProcessCfg)
    extrusion: ExtrusionCfg = field(default_factory=ExtrusionCfg)
    screener: ScreenerCfg = field(default_factory=ScreenerCfg)
    gcode: GcodeCfg = field(default_factory=GcodeCfg)
    emit: EmitCfg = field(default_factory=EmitCfg)
    collision: CollisionCfg = field(default_factory=CollisionCfg)


def _filter(dc_type, data: Mapping[str, Any] | None) -> dict:
    """Keep only keys that are fields of dc_type; raise on unknown keys."""
    if not data:
        return {}
    names = {f.name for f in fields(dc_type)}
    unknown = set(data) - names
    if unknown:
        raise ValueError(f"Unknown keys for {dc_type.__name__}: {sorted(unknown)}")
    return {k: v for k, v in data.items() if k in names}


def load_config(path: str | Path) -> Config:
    raw = yaml.safe_load(Path(path).read_text()) or {}

    m = dict(raw.get("machine") or {})
    steps = StepsCfg(**_filter(StepsCfg, m.pop("steps", None)))
    machine = MachineCfg(**_filter(MachineCfg, m), steps=steps)
    if isinstance(machine.build_volume_mm, list):
        machine.build_volume_mm = tuple(machine.build_volume_mm)

    return Config(
        machine=machine,
        c_axis=CAxisCfg(**_filter(CAxisCfg, raw.get("c_axis"))),
        spindle=SpindleCfg(**_filter(SpindleCfg, raw.get("spindle"))),
        process=ProcessCfg(**_filter(ProcessCfg, raw.get("process"))),
        extrusion=ExtrusionCfg(**_filter(ExtrusionCfg, raw.get("extrusion"))),
        screener=ScreenerCfg(**_filter(ScreenerCfg, raw.get("screener"))),
        gcode=GcodeCfg(**_filter(GcodeCfg, raw.get("gcode"))),
        emit=EmitCfg(**_filter(EmitCfg, raw.get("emit"))),
        collision=CollisionCfg(**_filter(CollisionCfg, raw.get("collision"))),
    )
