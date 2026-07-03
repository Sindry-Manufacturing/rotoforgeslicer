"""Machine / Material / Process preset layering. SPEC §7/§9.

A Python port of PrusaSlicer's preset architecture (src/libslic3r/Preset.cpp +
PresetBundle.cpp, (c) Prusa Research, AGPLv3 — structure ported with permission
of the project license), reduced to Rotoforge's three preset types:

* **machine** — the hardware: kinematics, C-axis limits, spindle range, macros.
* **material** — the characterized process window: screener selection + thermal
  targets (what ``studio.materials`` calls a profile).
* **process** — everything the operator tunes per job: bead/layer geometry,
  fill strategy, feeds, collision margins.

The ported structure and its semantics:

* Every dotted config key is owned by EXACTLY ONE preset type (the port of
  ``Preset::print_options()/filament_options()/printer_options()``); the key
  lists below are explicit so adding a ``Config`` field breaks the partition
  test until a human claims the key.
* A collection = the presets of one type + a selection + an **edited overlay**
  (the port of ``m_edited_preset``): selecting discards edits, dirtiness is the
  diff between the overlay and the saved preset (``select_preset`` /
  ``current_dirty_options`` semantics, Preset.cpp:1548/1415).
* User preset files are YAML, **sparse** (only keys differing from the base
  machine config), so ``config/machine_duet3.yaml`` — the calibration record —
  stays authoritative for keys a preset never touched. In memory a preset's
  values are always complete (base ← file). ``inherits`` is a pure annotation
  carried for provenance, exactly like PrusaSlicer user presets — it is never
  applied at load time.
* ``PresetBundle.full_config()`` composes the final ``Config`` by ordered
  dict-apply over a deep copy of the base (PresetBundle.cpp:754); the key
  partition makes the order cosmetic.
* Machine-local bookkeeping keys (``RECONCILE_IGNORE_KEYS``) are excluded from
  dirty/reconciliation comparisons (the port of ``profile_print_params_same``,
  Preset.cpp:879).

Plain YAML + dataclass plumbing — no Qt, no heavy imports; unit-tested headless.
"""
from __future__ import annotations

import copy
import math
import os
import sys
from dataclasses import dataclass, fields, is_dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Optional

import yaml

from .config import Config, load_config

DEFAULT_NAME = "- default -"

#: Preset-type key ownership (the port of Preset's static per-type option lists).
#: Explicit on purpose: a new Config field fails the partition test until claimed.
MACHINE_KEYS = (
    "machine.name", "machine.rotary_axis_letter", "machine.build_volume_mm",
    "machine.feedrate_mode", "machine.steps.x", "machine.steps.y",
    "machine.steps.z", "machine.steps.e_per_mm", "machine.steps.a_per_deg",
    "c_axis.home_heading_deg", "c_axis.home_offset_deg", "c_axis.invert_sign",
    "c_axis.a_min_deg", "c_axis.a_max_deg", "c_axis.max_speed_deg_s",
    "c_axis.max_drift_deg", "c_axis.max_scrub_deg_mm",
    "spindle.rpm_min", "spindle.rpm_max",
    "gcode.preamble_macros", "gcode.postamble_macros", "gcode.use_relative_e",
    # the 50 mm collision body is hardware, not a per-job tunable
    "process.wheel_diameter_mm",
)

MATERIAL_KEYS = (
    "screener.csv_path", "screener.revs_per_mm_mode",
    "screener.revs_per_mm_target", "screener.revs_per_mm_tol",
    "screener.traverse_target",
    "extrusion.mode", "extrusion.x_ratio",
    "process.bed_temp_c", "process.hotshoe_macro",
)

PROCESS_KEYS = (
    "process.bead_width_mm", "process.layer_height_mm",
    "process.wire_diameter_mm", "process.raster_overlap",
    "process.min_deposit_len_mm", "process.inter_pass_lift_mm",
    "process.lead_in_len_mm", "process.approach_clearance_mm",
    "process.lead_out_len_mm", "process.travel_z_mm",
    "process.startup_settle_ms", "process.spindle_dwell_ms",
    "process.cpap_deposit",
    "fill.mode", "fill.auto_heading", "fill.raster_bidirectional",
    "fill.crosshatch", "fill.crosshatch_angle_deg", "fill.streamline_step_mm",
    "fill.streamline_curl", "fill.perimeter_loops", "fill.contour_simplify_mm",
    "emit.feed_travel_mm_min", "emit.feed_z_mm_min", "emit.feed_dep_mm_min",
    "emit.dry_run",
    "collision.enabled", "collision.cell_mm", "collision.clearance_mm",
    "collision.wire_lead_mm",
)

KEYSETS: Dict[str, tuple] = {
    "machine": MACHINE_KEYS,
    "material": MATERIAL_KEYS,
    "process": PROCESS_KEYS,
}

#: Machine-local bookkeeping excluded from dirty / reconciliation comparisons
#: (the port of profile_print_params_same's ignore list): an absolute CSV path
#: differs between machines without the process window itself differing.
RECONCILE_IGNORE_KEYS = frozenset({"screener.csv_path"})


# ---- flat config plumbing -----------------------------------------------------

def flatten_config(cfg: Config) -> Dict[str, object]:
    """``Config`` -> flat ``{dotted.key: value}``. Tuples become lists (YAML-
    friendly); lists are copied so no caller ever aliases a live config list."""
    flat: Dict[str, object] = {}

    def walk(obj, prefix: str) -> None:
        for f in fields(obj):
            val = getattr(obj, f.name)
            key = f"{prefix}{f.name}"
            if is_dataclass(val):
                walk(val, key + ".")
            elif isinstance(val, tuple):
                flat[key] = list(val)
            elif isinstance(val, list):
                flat[key] = list(val)
            else:
                flat[key] = val

    walk(cfg, "")
    return flat


def _coerce(base_value, value):
    """Coerce ``value`` to the type of ``base_value``; raise ValueError (and
    ONLY ValueError — callers' substitute-and-report paths depend on it) if the
    value cannot faithfully represent itself in that type."""
    if isinstance(base_value, bool):                 # before int: bool <: int
        if isinstance(value, bool):
            return value
        raise ValueError(f"expected bool, got {value!r}")
    if isinstance(base_value, int):
        if isinstance(value, bool):
            raise ValueError(f"expected int, got {value!r}")
        try:
            f = float(value)
        except (TypeError, ValueError):
            raise ValueError(f"expected int, got {value!r}")
        if not f.is_integer():
            raise ValueError(f"expected int, got {value!r}")
        return int(f)
    if isinstance(base_value, float):
        if isinstance(value, bool):
            raise ValueError(f"expected float, got {value!r}")
        try:
            return float(value)
        except (TypeError, ValueError):
            raise ValueError(f"expected float, got {value!r}")
    if isinstance(base_value, str):
        if isinstance(value, str):
            return value
        raise ValueError(f"expected str, got {value!r}")
    if isinstance(base_value, tuple):                # fixed-shape (build volume)
        if not isinstance(value, (list, tuple)):
            raise ValueError(f"expected sequence, got {value!r}")
        if len(value) != len(base_value):
            raise ValueError(f"expected {len(base_value)} elements, got {value!r}")
        return tuple(_coerce(b, v) for b, v in zip(base_value, value))
    if isinstance(base_value, list):                 # homogeneous (macro lists)
        if not isinstance(value, (list, tuple)):
            raise ValueError(f"expected list, got {value!r}")
        if base_value:
            return [_coerce(base_value[0], v) for v in value]
        if any(isinstance(v, (list, tuple, dict)) for v in value):
            raise ValueError(f"expected a list of scalars, got {value!r}")
        return list(value)
    if is_dataclass(base_value):
        raise ValueError("not a leaf config key")    # never replace a section
    if type(value) is not type(base_value):
        raise ValueError(f"expected {type(base_value).__name__}, got {value!r}")
    return value


def sanitize_values(values: Mapping[str, object], base_values: Mapping[str, object],
                    ) -> "tuple[Dict[str, object], List[str]]":
    """Restrict + type-check foreign values (preset files, project snapshots)
    against the base: unknown keys are dropped, bad-typed values fall back to
    the base value; both are reported. Everything stored in a collection MUST
    pass through here, so ``full_config()`` can trust its overlays."""
    clean: Dict[str, object] = {}
    report: List[str] = []
    for key, value in values.items():
        if key not in base_values:
            report.append(f"unknown key {key!r} dropped")
            continue
        try:
            clean[key] = _coerce(base_values[key], value)
        except ValueError as e:
            report.append(f"bad value for {key!r} ({e}); kept base "
                          f"{base_values[key]!r}")
    return clean, report


def apply_flat(cfg: Config, flat: Mapping[str, object], *,
               on_unknown: str = "raise") -> List[str]:
    """Set dotted keys on a ``Config``.

    ``on_unknown='raise'`` (presets, trusted callers): unknown keys or
    un-coercible values raise ``ValueError``. ``on_unknown='warn'`` (project
    load, forward compatibility): the offending key keeps its current value and
    a report line is returned — the port of PrusaSlicer's ConfigSubstitution
    (substitute-and-report, never abort a project load)."""
    if on_unknown not in ("raise", "warn"):
        raise ValueError(f"on_unknown must be 'raise' or 'warn', got {on_unknown!r}")
    report: List[str] = []
    # type reference = the dataclass DEFAULTS: a yaml-loaded config may hold an
    # int where the field is float (yaml parses "110" as int), so the target's
    # current value is not a trustworthy type witness
    ref = Config()
    for key, value in flat.items():
        parts = key.split(".")
        obj = cfg
        robj = ref
        try:
            for part in parts[:-1]:
                obj = getattr(obj, part)
                robj = getattr(robj, part)
            current = getattr(obj, parts[-1])
            ref_value = getattr(robj, parts[-1])
        except AttributeError:
            if on_unknown == "raise":
                raise ValueError(f"unknown config key {key!r}")
            report.append(f"unknown key {key!r} skipped")
            continue
        try:
            setattr(obj, parts[-1], _coerce(ref_value, value))
        except ValueError as e:
            if on_unknown == "raise":
                raise ValueError(f"bad value for {key!r}: {e}")
            report.append(f"bad value for {key!r} ({e}); kept {current!r}")
    return report


def values_equal(a, b) -> bool:
    """Value comparison for dirty/reconciliation checks: floats compare with a
    tight relative tolerance (YAML round-trips), sequences elementwise."""
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b if isinstance(a, bool) and isinstance(b, bool) else False
    if isinstance(a, float) or isinstance(b, float):
        try:
            return math.isclose(float(a), float(b), rel_tol=1e-12, abs_tol=1e-12)
        except (TypeError, ValueError):
            return False
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(values_equal(x, y) for x, y in zip(a, b))
    return a == b


# ---- environment --------------------------------------------------------------

def data_dir() -> Path:
    """Where presets + studio state live: ``ROTOFORGE_DATA_DIR`` (tests) →
    ``<repo>/config`` in a source checkout (package-anchored, never the cwd) →
    ``~/.rotoforge`` for a frozen app. Same resolution family as
    ``studio.screener_panel.profiles_path``."""
    env = os.environ.get("ROTOFORGE_DATA_DIR")
    if env:
        return Path(env)
    if not getattr(sys, "frozen", False):
        repo_cfg = Path(__file__).resolve().parents[1] / "config"
        if repo_cfg.is_dir():
            return repo_cfg
    return Path.home() / ".rotoforge"


def base_config() -> Config:
    """The base machine config all preset math resolves against: the bundled /
    package ``machine_duet3.yaml``. Unlike the GUI's best-effort default, a
    PRESENT but unparsable yaml raises — silently regressing the calibration
    record to built-in defaults would bake wrong values (e.g. ω_C = 0 disables
    the slew-feasibility proof) into every preset saved afterwards."""
    candidates = []
    bundle = getattr(sys, "_MEIPASS", None)          # PyInstaller frozen data dir
    if bundle:
        candidates.append(Path(bundle) / "config" / "machine_duet3.yaml")
    candidates.append(Path(__file__).resolve().parents[1] / "config"
                      / "machine_duet3.yaml")
    for c in candidates:
        if c.exists():
            return load_config(c)                     # parse errors propagate
    return Config()


_WINDOWS_RESERVED = frozenset(
    {"CON", "PRN", "AUX", "NUL"}
    | {f"COM{i}" for i in range(1, 10)} | {f"LPT{i}" for i in range(1, 10)})


def validate_preset_name(name: str) -> str:
    """Preset names double as filenames — reject anything that can't be one."""
    name = name.strip()
    if not name:
        raise ValueError("preset name is empty")
    if name == DEFAULT_NAME:
        raise ValueError(f"{DEFAULT_NAME!r} is reserved")
    bad = set('/\\:*?"<>|') & set(name)
    if bad or any(ord(ch) < 32 for ch in name):
        raise ValueError(f"preset name may not contain {sorted(bad) or 'control chars'}")
    if name != name.rstrip(". "):
        raise ValueError("preset name may not end with a dot or space")
    if name.split(".")[0].upper() in _WINDOWS_RESERVED:
        raise ValueError(f"{name!r} is a reserved device name on Windows")
    return name


# ---- preset model ---------------------------------------------------------------

@dataclass
class Preset:
    """One named value set of one type. ``values`` is always COMPLETE for the
    type's key set (files may be sparse; loading resolves them over the base)."""

    name: str
    ptype: str
    values: Dict[str, object]
    inherits: str = ""            # provenance annotation only — never applied
    is_default: bool = False      # the synthetic "- default -" (base config)
    is_external: bool = False     # came from a project file; never on disk


class PresetCollection:
    """The presets of ONE type + selection + edited overlay (Preset.cpp:1548)."""

    def __init__(self, ptype: str, keys: tuple, base_flat: Mapping[str, object],
                 dir_path: Path):
        self.ptype = ptype
        self.keys = tuple(keys)
        self.base_values = {k: base_flat[k] for k in self.keys}
        self.dir_path = Path(dir_path)
        self.presets: Dict[str, Preset] = {
            DEFAULT_NAME: Preset(DEFAULT_NAME, ptype, dict(self.base_values),
                                 is_default=True)}
        self.selected = DEFAULT_NAME
        self.edited: Dict[str, object] = dict(self.base_values)
        self.warnings: List[str] = []
        self._load_dir()

    # ---- loading (errors accumulate; one bad file never kills startup) ------

    def _load_dir(self) -> None:
        if not self.dir_path.is_dir():
            return
        for f in sorted(self.dir_path.glob("*.yaml")):
            name = f.stem
            if name == DEFAULT_NAME:
                continue
            try:
                raw = yaml.safe_load(f.read_text(encoding="utf-8")) or {}
                if not isinstance(raw, dict):
                    raise ValueError("not a mapping")
                # every stored value is restricted AND type-checked here, so
                # full_config() can apply overlays in raise mode safely — one
                # bad hand-edited value must never brick the studio launch
                clean, report = sanitize_values(dict(raw.get("values") or {}),
                                                self.base_values)
                self.warnings.extend(
                    f"{self.ptype} preset {name!r}: {line}" for line in report)
                values = dict(self.base_values)
                values.update(clean)
                self.presets[name] = Preset(
                    name, self.ptype, values, inherits=str(raw.get("inherits") or ""))
            except Exception as e:
                self.warnings.append(f"{self.ptype} preset {name!r} unreadable: {e}")

    # ---- queries -------------------------------------------------------------

    def names(self) -> List[str]:
        user = sorted(n for n in self.presets if n != DEFAULT_NAME)
        return [DEFAULT_NAME] + user

    def dirty_keys(self) -> List[str]:
        saved = self.presets[self.selected].values
        return [k for k in self.keys if k not in RECONCILE_IGNORE_KEYS
                and not values_equal(self.edited[k], saved[k])]

    def is_dirty(self) -> bool:
        return bool(self.dirty_keys())

    # ---- selection / edit / save ----------------------------------------------

    def select(self, name: str) -> str:
        """Select a preset, DISCARDING the edited overlay (Preset.cpp:1548).
        Unknown names fall back to the default preset (the load_selections
        repair, PresetBundle.cpp:648)."""
        if name not in self.presets:
            self.warnings.append(
                f"{self.ptype} preset {name!r} not found; using {DEFAULT_NAME!r}")
            name = DEFAULT_NAME
        self.selected = name
        self.edited = dict(self.presets[name].values)
        return name

    def save_current(self, name: str) -> Preset:
        """Persist the edited overlay as a user preset. Files are SPARSE (diff
        vs base) so the machine yaml stays authoritative for untouched keys;
        ``inherits`` is carried from the source preset unchanged (empty when
        saving from the default) — Preset.cpp:1073 deliberately does not chain
        user presets."""
        name = validate_preset_name(name)
        for existing in self.presets:
            if existing.lower() == name.lower() and existing != name:
                raise ValueError(
                    f"{name!r} differs from existing preset {existing!r} only "
                    "by case — the filesystem would merge their files")
        source = self.presets[self.selected]
        inherits = "" if source.is_default else source.inherits
        sparse = {k: v for k, v in self.edited.items()
                  if not values_equal(v, self.base_values[k])}
        doc: Dict[str, object] = {"values": sparse}
        if inherits:
            doc["inherits"] = inherits
        self.dir_path.mkdir(parents=True, exist_ok=True)
        (self.dir_path / f"{name}.yaml").write_text(
            yaml.safe_dump(doc, sort_keys=True), encoding="utf-8")
        self.presets[name] = Preset(name, self.ptype, dict(self.edited),
                                    inherits=inherits)
        self.selected = name                      # clean by construction
        return self.presets[name]

    def delete(self, name: str) -> None:
        if name == DEFAULT_NAME:
            raise ValueError("cannot delete the default preset")
        preset = self.presets.pop(name, None)
        if preset is None:
            return
        if not preset.is_external:
            try:
                (self.dir_path / f"{name}.yaml").unlink()
            except FileNotFoundError:
                pass
        if self.selected == name:
            self.select(DEFAULT_NAME)

    def register_external(self, name: str, values: Mapping[str, object]) -> Preset:
        """A preset referenced by a project but absent here: kept in memory only
        (Preset.cpp is_external — never written to the preset dir). Same-name
        re-registration replaces."""
        clean, report = sanitize_values(values, self.base_values)
        self.warnings.extend(f"external {name!r}: {line}" for line in report)
        complete = dict(self.base_values)
        complete.update(clean)
        self.presets[name] = Preset(name, self.ptype, complete, is_external=True)
        return self.presets[name]


class PresetBundle:
    """The three collections + selection persistence + compose/capture
    (PresetBundle.cpp reduced to one machine, no vendor bundles)."""

    TYPES = ("machine", "material", "process")

    def __init__(self, data_dir_path: "str | Path | None" = None,
                 base: Optional[Config] = None):
        self.base = base if base is not None else base_config()
        base_flat = flatten_config(self.base)
        missing = [k for keys in KEYSETS.values() for k in keys
                   if k not in base_flat]
        if missing:
            raise ValueError(f"preset key lists name unknown config keys: {missing}")
        self.data_dir = Path(data_dir_path) if data_dir_path else data_dir()
        self.collections: Dict[str, PresetCollection] = {
            t: PresetCollection(t, KEYSETS[t], base_flat,
                                self.data_dir / "presets" / t)
            for t in self.TYPES}
        self.load_selections()

    # ---- compose / capture ---------------------------------------------------

    def full_config(self) -> Config:
        """Base ← machine ← material ← process edited overlays → a fresh Config
        (PresetBundle::full_config; the key partition makes order cosmetic)."""
        cfg = copy.deepcopy(self.base)
        for t in self.TYPES:
            apply_flat(cfg, self.collections[t].edited)
        return cfg

    def capture(self, cfg: Config) -> None:
        """Pull the live config INTO the edited overlays. PrusaSlicer's UI
        writes through to the edited preset; the studio's widgets write to a
        live Config instead, so every bundle read must be preceded by a
        capture of that config (the _sync_bundle invariant in studio.app)."""
        flat = flatten_config(cfg)
        for col in self.collections.values():
            col.edited = {k: flat[k] for k in col.keys}

    def selections(self) -> Dict[str, str]:
        return {t: self.collections[t].selected for t in self.TYPES}

    def warnings(self) -> List[str]:
        return [w for t in self.TYPES for w in self.collections[t].warnings]

    # ---- selection persistence (the port of AppConfig [presets]) -------------

    def _state_path(self) -> Path:
        return self.data_dir / "studio_state.yaml"

    def load_selections(self) -> None:
        try:
            raw = yaml.safe_load(self._state_path().read_text(encoding="utf-8")) or {}
        except FileNotFoundError:
            return
        except Exception as e:
            self.collections["machine"].warnings.append(
                f"studio_state.yaml unreadable: {e}")
            return
        # a truncated/hand-edited state file may parse to a scalar or list —
        # that must degrade to defaults, never crash the studio constructor
        stored = raw.get("presets") if isinstance(raw, dict) else None
        if not isinstance(stored, dict):
            if raw:
                self.collections["machine"].warnings.append(
                    "studio_state.yaml malformed; using default selections")
            return
        for t in self.TYPES:
            name = stored.get(t)
            if name:
                self.collections[t].select(str(name))

    def save_selections(self) -> None:
        # external presets exist only in their project file — never remember them
        stored = {t: (DEFAULT_NAME
                      if self.collections[t].presets[self.collections[t].selected].is_external
                      else self.collections[t].selected)
                  for t in self.TYPES}
        p = self._state_path()
        p.parent.mkdir(parents=True, exist_ok=True)
        tmp = p.with_suffix(".yaml.tmp")             # atomic: this file is written
        tmp.write_text(yaml.safe_dump({"presets": stored}, sort_keys=True),
                       encoding="utf-8")             # on every preset switch and
        os.replace(tmp, p)                           # a torn write bricks startup

    # ---- project reconciliation (the port of load_external_preset,
    #      Preset.cpp:899 — reduced: no system profiles, no renamed_from) -------

    def adopt_project(self, flat_snapshot: Mapping[str, object],
                      names: Optional[Mapping[str, str]]) -> Dict[str, str]:
        """Reconcile a project's full config snapshot against the collections.

        Per type: the snapshot restricted to the type's keys (missing keys fall
        back to base — defaults ← overlay semantics) either matches the named
        preset (→ select clean), differs from it (→ select it with the snapshot
        as the dirty overlay — the classic "project config appears as a
        modified preset"), or the name is unknown (→ in-memory external preset
        named "<name> (project)"). Returns {type: 'clean'|'modified'|'external'}.
        """
        base_flat = flatten_config(self.base)
        outcomes: Dict[str, str] = {}
        for t in self.TYPES:
            col = self.collections[t]
            raw_sub = {k: flat_snapshot[k] for k in col.keys if k in flat_snapshot}
            # snapshot values are foreign data: type-check them before they can
            # reach an overlay (a later full_config/save would otherwise choke)
            clean, report = sanitize_values(raw_sub, col.base_values)
            col.warnings.extend(f"project ({t}): {line}" for line in report)
            sub = dict(col.base_values)
            sub.update(clean)
            name = (names or {}).get(t) or ""
            if name and name in col.presets and not col.presets[name].is_external:
                saved = col.presets[name].values
                diff = [k for k in col.keys if k not in RECONCILE_IGNORE_KEYS
                        and not values_equal(sub[k], saved[k])]
                col.selected = name
                col.edited = sub
                outcomes[t] = "clean" if not diff else "modified"
            else:
                # idempotent suffix: re-saving a project while an external
                # preset is selected must not accrete "(project) (project)"
                if not name:
                    ext = "(project)"
                elif name.endswith(" (project)") or name == "(project)":
                    ext = name
                else:
                    ext = f"{name} (project)"
                col.register_external(ext, sub)
                col.selected = ext
                col.edited = dict(col.presets[ext].values)
                outcomes[t] = "external"
        return outcomes


# ---- materials bridge -----------------------------------------------------------

def material_preset_from_profile(profile, base: Optional[Config] = None) -> Preset:
    """Promote a ``studio.materials.MaterialProfile`` to a material preset
    (the profile's apply_to_cfg is the authoritative field mapping)."""
    cfg = copy.deepcopy(base if base is not None else base_config())
    profile.apply_to_cfg(cfg)
    flat = flatten_config(cfg)
    return Preset(name=profile.name, ptype="material",
                  values={k: flat[k] for k in MATERIAL_KEYS})
