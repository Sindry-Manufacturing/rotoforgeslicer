"""Preset layering (PrusaSlicer PresetBundle port): key partition, flat config
plumbing, collection select/edit/save semantics, bundle compose/capture,
selection persistence, project reconciliation. All headless (no Qt)."""
import pytest
import yaml

from rotoforge_slicer.config import Config
from rotoforge_slicer.presets import (
    DEFAULT_NAME, KEYSETS, MACHINE_KEYS, MATERIAL_KEYS, PROCESS_KEYS,
    RECONCILE_IGNORE_KEYS, PresetBundle, PresetCollection, apply_flat,
    flatten_config, material_preset_from_profile, validate_preset_name,
    values_equal,
)


# ---- key partition invariant ---------------------------------------------------

def test_key_partition_covers_config_exactly():
    """Every Config key is owned by EXACTLY one preset type. Adding a Config
    field must fail here until a human claims the key for a type."""
    all_keys = set(flatten_config(Config()))
    union = set(MACHINE_KEYS) | set(MATERIAL_KEYS) | set(PROCESS_KEYS)
    assert union == all_keys, (
        f"unclaimed: {sorted(all_keys - union)}; stale: {sorted(union - all_keys)}")
    assert len(MACHINE_KEYS) + len(MATERIAL_KEYS) + len(PROCESS_KEYS) == len(union), \
        "a key is claimed by more than one preset type"


def test_ignore_keys_are_real_keys():
    assert RECONCILE_IGNORE_KEYS <= set(flatten_config(Config()))


# ---- flatten / apply -------------------------------------------------------------

def test_flatten_apply_round_trip():
    cfg = Config()
    cfg.machine.build_volume_mm = (200.0, 100.0, 50.0)
    cfg.c_axis.a_max_deg = 123.456789012345
    cfg.gcode.preamble_macros = ["A.g", "B.g"]
    flat = flatten_config(cfg)
    assert flat["machine.build_volume_mm"] == [200.0, 100.0, 50.0]
    assert flat["machine.steps.e_per_mm"] == pytest.approx(46.73)

    out = Config()
    assert apply_flat(out, flat) == []
    assert out.machine.build_volume_mm == (200.0, 100.0, 50.0)   # tuple restored
    assert out.c_axis.a_max_deg == cfg.c_axis.a_max_deg           # full precision
    assert out.gcode.preamble_macros == ["A.g", "B.g"]
    assert out.gcode.preamble_macros is not flat["gcode.preamble_macros"], \
        "list values must be copied, never aliased"


def test_apply_flat_unknown_key_raise_vs_warn():
    cfg = Config()
    with pytest.raises(ValueError, match="unknown config key"):
        apply_flat(cfg, {"process.no_such_key": 1.0})
    report = apply_flat(cfg, {"process.no_such_key": 1.0,
                              "nosection.x": 2}, on_unknown="warn")
    assert len(report) == 2
    assert all("skipped" in line for line in report)


def test_apply_flat_bad_value_substitutes_and_reports():
    """The ConfigSubstitution port: a known key with an unusable value keeps the
    current value and is reported — a project from another version must open."""
    cfg = Config()
    before = cfg.process.layer_height_mm
    report = apply_flat(cfg, {"process.layer_height_mm": "not-a-number",
                              "fill.crosshatch": "yes",       # str for bool
                              "spindle.rpm_min": 2.5},        # non-integral int
                        on_unknown="warn")
    assert cfg.process.layer_height_mm == before
    assert cfg.fill.crosshatch is False
    assert cfg.spindle.rpm_min == 5000
    assert len(report) == 3
    with pytest.raises(ValueError, match="bad value"):
        apply_flat(Config(), {"process.layer_height_mm": "nope"})


def test_apply_flat_coerces_int_like_floats():
    cfg = Config()
    apply_flat(cfg, {"spindle.rpm_min": 6000.0, "process.startup_settle_ms": 5000.0})
    assert cfg.spindle.rpm_min == 6000 and isinstance(cfg.spindle.rpm_min, int)
    assert cfg.process.startup_settle_ms == 5000


def test_values_equal_semantics():
    assert values_equal(1.0, 1.0 + 1e-15)
    assert not values_equal(1.0, 1.0001)
    assert values_equal([1.0, 2.0], (1.0, 2.0))       # tuple/list normalized
    assert values_equal(True, True) and not values_equal(True, 1)
    assert not values_equal("a", "b")


# ---- preset name validation -----------------------------------------------------

@pytest.mark.parametrize("bad", ["", "  ", "a/b", "a\\b", "x:y", "dot.",
                                 "CON", "com3.yaml", DEFAULT_NAME, "q?"])
def test_validate_preset_name_rejects(bad):
    with pytest.raises(ValueError):
        validate_preset_name(bad)


def test_validate_preset_name_accepts_and_strips():
    assert validate_preset_name("  Al 1100-O  ") == "Al 1100-O"


# ---- collection semantics ---------------------------------------------------------

def _collection(tmp_path, ptype="process"):
    return PresetCollection(ptype, KEYSETS[ptype], flatten_config(Config()),
                            tmp_path / "presets" / ptype)


def test_collection_select_discards_edits_and_dirty_diff(tmp_path):
    col = _collection(tmp_path)
    assert col.selected == DEFAULT_NAME and not col.is_dirty()
    col.edited["process.layer_height_mm"] = 0.2
    assert col.dirty_keys() == ["process.layer_height_mm"]
    col.select(DEFAULT_NAME)                       # select DISCARDS edits
    assert not col.is_dirty()


def test_collection_save_is_sparse_and_reloads_complete(tmp_path):
    col = _collection(tmp_path)
    col.edited["process.layer_height_mm"] = 0.2
    col.edited["fill.mode"] = "contour"
    col.save_current("thick contour")
    assert col.selected == "thick contour" and not col.is_dirty()

    f = tmp_path / "presets" / "process" / "thick contour.yaml"
    raw = yaml.safe_load(f.read_text(encoding="utf-8"))
    assert set(raw["values"]) == {"process.layer_height_mm", "fill.mode"}, \
        "preset files are sparse: only keys differing from the base"

    fresh = _collection(tmp_path)                  # reload from disk
    assert "thick contour" in fresh.presets
    v = fresh.presets["thick contour"].values
    assert v["process.layer_height_mm"] == 0.2
    assert v["fill.mode"] == "contour"
    assert set(v) == set(KEYSETS["process"]), "in-memory values always complete"


def test_collection_base_stays_authoritative_for_untouched_keys(tmp_path):
    """Recalibrating the machine yaml must flow through presets that never
    touched the recalibrated key (files are sparse; memory resolves over base)."""
    col = _collection(tmp_path, "machine")
    col.edited["c_axis.a_max_deg"] = 170.0
    col.save_current("narrow range")

    recal = Config()
    recal.machine.steps.e_per_mm = 99.9            # "recalibrated"
    fresh = PresetCollection("machine", KEYSETS["machine"],
                             flatten_config(recal), tmp_path / "presets" / "machine")
    v = fresh.presets["narrow range"].values
    assert v["c_axis.a_max_deg"] == 170.0          # the preset's own key
    assert v["machine.steps.e_per_mm"] == 99.9     # follows the new base


def test_collection_delete_refuses_default_and_falls_back(tmp_path):
    col = _collection(tmp_path)
    with pytest.raises(ValueError):
        col.delete(DEFAULT_NAME)
    col.edited["fill.mode"] = "outline"
    col.save_current("x")
    col.delete("x")
    assert col.selected == DEFAULT_NAME
    assert not (tmp_path / "presets" / "process" / "x.yaml").exists()


def test_collection_inherits_annotation_carried_not_applied(tmp_path):
    col = _collection(tmp_path)
    col.edited["fill.mode"] = "contour"
    col.save_current("parent")                     # from default -> inherits ""
    assert col.presets["parent"].inherits == ""
    col.edited["fill.perimeter_loops"] = 3
    col.save_current("child")                      # from user preset "parent"
    assert col.presets["child"].inherits == ""     # user->user never chains
    # hand-written annotation survives load and is NOT applied
    f = tmp_path / "presets" / "process" / "annotated.yaml"
    f.write_text(yaml.safe_dump({"inherits": "parent",
                                 "values": {"fill.mode": "outline"}}),
                 encoding="utf-8")
    fresh = _collection(tmp_path)
    assert fresh.presets["annotated"].inherits == "parent"
    assert fresh.presets["annotated"].values["fill.perimeter_loops"] == 0, \
        "inherits is annotation only — parent values are never applied"


def test_collection_bad_file_and_foreign_keys_accumulate_warnings(tmp_path):
    d = tmp_path / "presets" / "process"
    d.mkdir(parents=True)
    (d / "broken.yaml").write_text("{ not yaml [", encoding="utf-8")
    (d / "foreign.yaml").write_text(
        yaml.safe_dump({"values": {"machine.name": "x", "fill.mode": "contour"}}),
        encoding="utf-8")
    col = _collection(tmp_path)
    assert "broken" not in col.presets
    assert col.presets["foreign"].values["fill.mode"] == "contour"
    assert any("broken" in w for w in col.warnings)
    assert any("foreign" in w and "machine.name" in w for w in col.warnings)


def test_collection_select_unknown_falls_back_to_default(tmp_path):
    col = _collection(tmp_path)
    assert col.select("no-such-preset") == DEFAULT_NAME
    assert any("no-such-preset" in w for w in col.warnings)


# ---- bundle: compose / capture / selections ---------------------------------------

def test_bundle_full_config_composes_and_capture_splits(tmp_path):
    b = PresetBundle(tmp_path, base=Config())
    cfg = b.full_config()
    assert cfg.process.layer_height_mm == Config().process.layer_height_mm

    cfg.process.layer_height_mm = 0.3              # process-owned
    cfg.process.bed_temp_c = 140.0                 # material-owned
    cfg.c_axis.a_max_deg = 90.0                    # machine-owned
    b.capture(cfg)
    assert b.collections["process"].dirty_keys() == ["process.layer_height_mm"]
    assert b.collections["material"].dirty_keys() == ["process.bed_temp_c"]
    assert b.collections["machine"].dirty_keys() == ["c_axis.a_max_deg"]

    out = b.full_config()
    assert out.process.layer_height_mm == 0.3
    assert out.process.bed_temp_c == 140.0
    assert out.c_axis.a_max_deg == 90.0
    # selecting ONE type discards only that type's dirty state
    b.collections["process"].select(DEFAULT_NAME)
    out2 = b.full_config()
    assert out2.process.layer_height_mm == Config().process.layer_height_mm
    assert out2.process.bed_temp_c == 140.0        # material edits survive
    assert out2.c_axis.a_max_deg == 90.0           # machine edits survive


def test_bundle_csv_path_never_dirties(tmp_path):
    b = PresetBundle(tmp_path, base=Config())
    cfg = b.full_config()
    cfg.screener.csv_path = r"C:\somewhere\window.csv"
    b.capture(cfg)
    assert not b.collections["material"].is_dirty(), \
        "screener.csv_path is machine-local bookkeeping (ignore list)"
    assert b.full_config().screener.csv_path == r"C:\somewhere\window.csv", \
        "…but it still rides the composed config"


def test_bundle_selections_persist_round_trip(tmp_path):
    b = PresetBundle(tmp_path, base=Config())
    cfg = b.full_config()
    cfg.fill.mode = "contour"
    b.capture(cfg)
    b.collections["process"].save_current("my process")
    b.save_selections()

    b2 = PresetBundle(tmp_path, base=Config())
    assert b2.collections["process"].selected == "my process"
    assert b2.full_config().fill.mode == "contour"


def test_bundle_selection_of_deleted_preset_falls_back(tmp_path):
    b = PresetBundle(tmp_path, base=Config())
    b.collections["material"].edited["process.bed_temp_c"] = 150.0
    b.collections["material"].save_current("gone")
    b.save_selections()
    (tmp_path / "presets" / "material" / "gone.yaml").unlink()

    b2 = PresetBundle(tmp_path, base=Config())
    assert b2.collections["material"].selected == DEFAULT_NAME


def test_bundle_never_remembers_external_selections(tmp_path):
    b = PresetBundle(tmp_path, base=Config())
    b.adopt_project(flatten_config(Config()), {"process": "nowhere"})
    assert b.collections["process"].selected == "nowhere (project)"
    b.save_selections()
    b2 = PresetBundle(tmp_path, base=Config())
    assert b2.collections["process"].selected == DEFAULT_NAME


# ---- bundle: project reconciliation ------------------------------------------------

def test_adopt_project_clean_modified_external(tmp_path):
    b = PresetBundle(tmp_path, base=Config())
    b.collections["process"].edited["fill.mode"] = "contour"
    b.collections["process"].save_current("known")

    snapshot = flatten_config(b.full_config())     # matches "known" exactly
    out = b.adopt_project(snapshot, {"process": "known",
                                     "machine": DEFAULT_NAME,
                                     "material": "missing-mat"})
    assert out["process"] == "clean"
    assert out["machine"] == "clean"
    assert out["material"] == "external"
    assert b.collections["material"].selected == "missing-mat (project)"
    assert b.collections["material"].presets["missing-mat (project)"].is_external

    snapshot2 = dict(snapshot)
    snapshot2["fill.perimeter_loops"] = 7          # differs from "known"
    out2 = b.adopt_project(snapshot2, {"process": "known"})
    assert out2["process"] == "modified"
    assert b.collections["process"].dirty_keys() == ["fill.perimeter_loops"]
    assert b.full_config().fill.perimeter_loops == 7


def test_adopt_project_ignores_csv_path_differences(tmp_path):
    b = PresetBundle(tmp_path, base=Config())
    b.collections["material"].save_current("mat")
    snapshot = flatten_config(b.full_config())
    snapshot["screener.csv_path"] = r"D:\other\machine.csv"
    out = b.adopt_project(snapshot, {"material": "mat"})
    assert out["material"] == "clean", \
        "a machine-local CSV path must not mark the material modified"
    assert b.full_config().screener.csv_path == r"D:\other\machine.csv"


def test_adopt_project_missing_snapshot_keys_fall_back_to_base(tmp_path):
    """Forward compat: an old project lacking newer keys keeps base values."""
    b = PresetBundle(tmp_path, base=Config())
    partial = {"process.layer_height_mm": 0.5}     # ancient tiny snapshot
    out = b.adopt_project(partial, None)
    assert out["process"] == "external"
    cfg = b.full_config()
    assert cfg.process.layer_height_mm == 0.5
    assert cfg.fill.mode == Config().fill.mode


# ---- review-campaign regressions ---------------------------------------------------

def test_bad_typed_preset_file_value_cannot_brick_startup(tmp_path):
    """A hand-edited preset with a wrong-typed value loads with a warning and
    the base value — and composing/selecting it NEVER raises (the studio's
    first full_config() runs unguarded at launch)."""
    d = tmp_path / "presets" / "process"
    d.mkdir(parents=True)
    d.joinpath("bad.yaml").write_text(
        yaml.safe_dump({"values": {"process.layer_height_mm": "fast",
                                   "fill.mode": "contour"}}), encoding="utf-8")
    b = PresetBundle(tmp_path, base=Config())
    col = b.collections["process"]
    assert any("bad" in w and "layer_height" in w for w in col.warnings)
    col.select("bad")
    cfg = b.full_config()                          # must not raise
    assert cfg.process.layer_height_mm == Config().process.layer_height_mm
    assert cfg.fill.mode == "contour"              # the good key still applies


def test_bad_persisted_selection_cannot_brick_startup(tmp_path):
    d = tmp_path / "presets" / "material"
    d.mkdir(parents=True)
    d.joinpath("hot.yaml").write_text(
        yaml.safe_dump({"values": {"process.bed_temp_c": ["not", "a", "float"]}}),
        encoding="utf-8")
    (tmp_path / "studio_state.yaml").write_text(
        yaml.safe_dump({"presets": {"material": "hot"}}), encoding="utf-8")
    b = PresetBundle(tmp_path, base=Config())      # selects "hot" at startup
    assert b.collections["material"].selected == "hot"
    assert b.full_config().process.bed_temp_c == Config().process.bed_temp_c


@pytest.mark.parametrize("content", ["pre", "- a\n- b\n",
                                     "presets: [machine, x]\n"])
def test_malformed_studio_state_degrades_to_defaults(tmp_path, content):
    (tmp_path / "studio_state.yaml").write_text(content, encoding="utf-8")
    b = PresetBundle(tmp_path, base=Config())      # must not raise
    assert all(b.collections[t].selected == DEFAULT_NAME for t in b.TYPES)


def test_apply_flat_warn_catches_type_errors_too():
    """float(list) raises TypeError, not ValueError — the never-abort project
    path must substitute it all the same."""
    cfg = Config()
    report = apply_flat(cfg, {"process.layer_height_mm": [1, 2],
                              "machine.build_volume_mm": [100.0, 50.0],   # len 2
                              "process": 5},       # non-leaf key
                        on_unknown="warn")
    assert cfg.process.layer_height_mm == Config().process.layer_height_mm
    assert cfg.machine.build_volume_mm == Config().machine.build_volume_mm
    assert not isinstance(cfg.process, int), "a section must never be replaced"
    assert len(report) == 3


def test_adopt_project_sanitizes_snapshot_values(tmp_path):
    b = PresetBundle(tmp_path, base=Config())
    snap = flatten_config(Config())
    snap["process.layer_height_mm"] = "corrupt"
    snap["c_axis.max_speed_deg_s"] = [360.0]
    out = b.adopt_project(snap, None)
    assert set(out.values()) == {"external"}
    cfg = b.full_config()                          # must not raise
    assert cfg.process.layer_height_mm == Config().process.layer_height_mm
    assert cfg.c_axis.max_speed_deg_s == Config().c_axis.max_speed_deg_s
    b.collections["process"].save_current("rescued")   # must not raise either
    assert PresetBundle(tmp_path, base=Config()).collections[
        "process"].presets["rescued"] is not None


def test_external_project_suffix_is_idempotent(tmp_path):
    b = PresetBundle(tmp_path, base=Config())
    snap = flatten_config(Config())
    b.adopt_project(snap, {"process": "lost"})
    name1 = b.collections["process"].selected
    b.adopt_project(snap, {"process": name1})      # save+reopen cycle
    assert b.collections["process"].selected == name1 == "lost (project)"


def test_save_current_rejects_case_only_collision(tmp_path):
    col = _collection(tmp_path)
    col.edited["fill.mode"] = "contour"
    col.save_current("Fast")
    with pytest.raises(ValueError, match="case"):
        col.save_current("fast")


# ---- materials bridge ---------------------------------------------------------------

def test_material_preset_from_profile():
    from rotoforge_slicer.studio.materials import MaterialProfile

    prof = MaterialProfile(name="Al", csv_path="al.csv", revs_per_mm=150.0,
                           traverse_mm_min=120.0, bed_temp_c=115.0,
                           hotshoe_temp_c=320.0)
    p = material_preset_from_profile(prof, base=Config())
    assert p.ptype == "material" and p.name == "Al"
    assert set(p.values) == set(MATERIAL_KEYS)
    assert p.values["screener.revs_per_mm_mode"] == "manual"
    assert p.values["screener.revs_per_mm_target"] == 150.0
    assert p.values["screener.traverse_target"] == 120.0
    assert p.values["process.bed_temp_c"] == 115.0
    assert p.values["process.hotshoe_macro"] == "Hotshoe_320C.g"
