"""Per-material process profiles for the studio's graphical screener. SPEC §5/§9.

A *material profile* records one material's chosen operating window and thermal
targets: the screener CSV that characterized it, the selected revs/mm ray + the
representative traverse on it (both snap to measured cells — see
``process.screener``), and the bed / hotshoe temperature targets. Profiles are a
plain YAML mapping so they diff cleanly and can be hand-edited:

    Al1100-O:
      csv_path: screener_al1100.csv
      revs_per_mm: 150.0
      traverse_mm_min: 100.0
      bed_temp_c: 110.0
      hotshoe_temp_c: 300.0

Pure YAML + config plumbing — no Qt here, so profile logic is unit-tested headless.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict

DEFAULT_PROFILES_PATH = Path("config") / "materials.yaml"


@dataclass
class MaterialProfile:
    name: str
    csv_path: str = ""
    revs_per_mm: float = 0.0        # 0 = auto ray selection
    traverse_mm_min: float = 0.0    # 0 = run midpoint
    bed_temp_c: float = 110.0
    hotshoe_temp_c: float = 300.0

    def apply_to_cfg(self, cfg) -> None:
        """Write this profile's targets into a loaded Config (screener selection +
        thermal targets). The hotshoe target selects the matching tuned macro by
        name — the macro itself must exist on the Duet (SPEC §6.1)."""
        cfg.screener.csv_path = self.csv_path
        if self.revs_per_mm > 0:
            cfg.screener.revs_per_mm_mode = "manual"
            cfg.screener.revs_per_mm_target = self.revs_per_mm
        else:
            cfg.screener.revs_per_mm_mode = "auto"
            cfg.screener.revs_per_mm_target = 0.0
        cfg.screener.traverse_target = self.traverse_mm_min
        cfg.process.bed_temp_c = self.bed_temp_c
        cfg.process.hotshoe_macro = hotshoe_macro_name(self.hotshoe_temp_c)


def hotshoe_macro_name(temp_c: float) -> str:
    """The tuned-macro name for a hotshoe temperature target (SPEC §6.1 prefers
    calling the user's tuned macros over re-emitting raw heater codes)."""
    return f"Hotshoe_{temp_c:g}C.g"


def hotshoe_temp_from_macro(macro: str, default: float = 300.0) -> float:
    """Inverse of :func:`hotshoe_macro_name`; ``default`` for unrecognized names."""
    import re

    m = re.fullmatch(r"Hotshoe_(\d+(?:\.\d+)?)C\.g", macro.strip())
    return float(m.group(1)) if m else default


def load_profiles(path: "str | Path" = DEFAULT_PROFILES_PATH) -> Dict[str, MaterialProfile]:
    """Profiles from YAML (empty dict if the file doesn't exist yet)."""
    import yaml

    p = Path(path)
    if not p.exists():
        return {}
    raw = yaml.safe_load(p.read_text()) or {}
    out: Dict[str, MaterialProfile] = {}
    for name, fields in raw.items():
        known = {k: v for k, v in (fields or {}).items()
                 if k in MaterialProfile.__dataclass_fields__ and k != "name"}
        out[name] = MaterialProfile(name=name, **known)
    return out


def save_profiles(profiles: Dict[str, MaterialProfile],
                  path: "str | Path" = DEFAULT_PROFILES_PATH) -> None:
    import yaml

    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    raw = {}
    for name, prof in sorted(profiles.items()):
        d = asdict(prof)
        d.pop("name")
        raw[name] = d
    p.write_text(yaml.safe_dump(raw, sort_keys=True), encoding="utf-8")
