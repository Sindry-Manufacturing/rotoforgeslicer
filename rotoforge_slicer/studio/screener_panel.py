"""The graphical process-window (screener) dialog. SPEC §5/§9.

Lets the user *see* the FRAM screener map (``screener_plot``) and choose the
operating cell the way the parameter screener works: **RPM and traverse are
selected independently** — type either target (each snaps to the nearest
measured STABLE cell, never interpolated physics, SPEC §5) or click a cell on
the map. The implied constant-revs/mm ray and its contiguous stable run are
highlighted for context. Bed and hotshoe temperature targets ride along, and
the whole selection saves/loads as a named per-material profile (``materials``).

Applying writes ``cfg.screener`` (mode/target/traverse_target) and the thermal
targets; the normal pipeline then selects exactly that cell on the next slice.
PySide6/matplotlib are imported inside the builder (lazy-import rule).
"""
from __future__ import annotations

from pathlib import Path

from ..process.screener import (
    _nv, _trav, load_rows, nearest_stable_cell, ray_run, widest_ray,
)
from .materials import (
    MaterialProfile, hotshoe_macro_name, hotshoe_temp_from_macro,
    load_profiles, save_profiles,
)


def profiles_path() -> Path:
    """Material profiles live next to the machine config in a source checkout
    (anchored to the PACKAGE location, never the process cwd — an unrelated
    ``config/`` folder in the working directory must not capture user profiles);
    a frozen app (read-only, temporary bundle dir) uses the user's home."""
    import sys

    if not getattr(sys, "frozen", False):
        repo_cfg = Path(__file__).resolve().parents[2] / "config"
        if repo_cfg.is_dir():
            return repo_cfg / "materials.yaml"
    return Path.home() / ".rotoforge" / "materials.yaml"


def open_screener_dialog(win) -> None:
    """Build and exec the process-window dialog against the studio main window
    (reads ``win.csv_path`` / ``win.cfg``, writes the selection back on Apply)."""
    from PySide6 import QtWidgets

    from ..gui.preview import make_preview_canvas
    from .screener_plot import plot_screener_map

    dlg = QtWidgets.QDialog(win)
    dlg.setWindowTitle("Process window — operating point & material")
    dlg.resize(980, 640)
    root = QtWidgets.QHBoxLayout(dlg)

    # ---- left: the map ----
    canvas = make_preview_canvas(dlg)
    ax = canvas.figure.axes[0]
    root.addWidget(canvas, stretch=1)

    # ---- right: selection controls ----
    side = QtWidgets.QVBoxLayout()
    root.addLayout(side)

    # ALL selection state is dialog-local until "Apply to slicer" — browsing a CSV
    # or a material profile must not touch the live config / main-window CSV.
    state = {"rows": [], "cell": None, "csv": win.csv_path}
    tol = win.cfg.screener.revs_per_mm_tol

    csv_lbl = QtWidgets.QLabel("no CSV loaded")
    csv_lbl.setWordWrap(True)
    btn_csv = QtWidgets.QPushButton("Load screener CSV…")
    side.addWidget(btn_csv)
    side.addWidget(csv_lbl)

    # RPM and traverse are INDEPENDENT targets (like the parameter screener);
    # each edit snaps the selection to the nearest measured stable cell, and
    # clicking the map picks a cell directly.
    hint = QtWidgets.QLabel("operating cell — set RPM and traverse\n"
                            "independently, or click a cell on the map\n"
                            "(always snaps to a measured stable cell)")
    side.addWidget(hint)
    rpm_spin = QtWidgets.QSpinBox()
    rpm_spin.setRange(0, win.cfg.spindle.rpm_max)
    rpm_spin.setSingleStep(500)
    trav_spin = QtWidgets.QDoubleSpinBox()
    trav_spin.setRange(0.0, 10000.0)
    trav_spin.setSingleStep(5.0)
    trav_spin.setDecimals(1)
    tgt_form = QtWidgets.QFormLayout()
    tgt_form.addRow("Spindle RPM", rpm_spin)
    tgt_form.addRow("Traverse (mm/min)", trav_spin)
    side.addLayout(tgt_form)
    cell_lbl = QtWidgets.QLabel("—")
    cell_lbl.setWordWrap(True)
    side.addWidget(cell_lbl)

    bed_spin = QtWidgets.QDoubleSpinBox()
    bed_spin.setRange(0, 200)
    bed_spin.setValue(win.cfg.process.bed_temp_c)
    hot_spin = QtWidgets.QDoubleSpinBox()
    hot_spin.setRange(0, 500)
    hot_spin.setValue(hotshoe_temp_from_macro(win.cfg.process.hotshoe_macro))
    form = QtWidgets.QFormLayout()
    form.addRow("Bed temp (°C)", bed_spin)
    form.addRow("Hotshoe temp (°C)", hot_spin)
    side.addLayout(form)

    side.addWidget(QtWidgets.QLabel("material profile"))
    mat_box = QtWidgets.QComboBox()
    mat_name = QtWidgets.QLineEdit()
    mat_name.setPlaceholderText("material name (e.g. Al1100-O)")
    btn_save = QtWidgets.QPushButton("Save profile")
    side.addWidget(mat_box)
    side.addWidget(mat_name)
    side.addWidget(btn_save)

    side.addStretch(1)
    btn_apply = QtWidgets.QPushButton("Apply to slicer")
    btn_close = QtWidgets.QPushButton("Close")
    side.addWidget(btn_apply)
    side.addWidget(btn_close)

    # ---- behaviour ----

    def selected_nv():
        cell = state["cell"]
        return _nv(cell) if cell else None

    def chosen_cell():
        return state["cell"]

    def replot():
        cell = state["cell"]
        plot_screener_map(
            ax, state["rows"], selected_nv=selected_nv(), tol=tol,
            chosen_cell=cell,
            rpm_window=(win.cfg.spindle.rpm_min, win.cfg.spindle.rpm_max))
        canvas.draw_idle()

    def select_cell(cell):
        """Make ``cell`` (a measured stable row, or None) the displayed
        selection: spins, readout, and map all show exactly this cell."""
        state["cell"] = cell
        if cell:
            rpm = int(round(float(cell["rpm"])))
            rpm_spin.blockSignals(True)
            rpm_spin.setValue(rpm)
            rpm_spin.blockSignals(False)
            trav_spin.blockSignals(True)
            trav_spin.setValue(_trav(cell))
            trav_spin.blockSignals(False)
            cell_lbl.setText(f"v = {_trav(cell):g} mm/min   RPM = {rpm}   "
                             f"revs/mm = {_nv(cell):g}   "
                             f"wire = {float(cell['feed_speed_mm_min']):g} mm/min   "
                             f"T_AZ = {float(cell['T_AZ_C']):g} °C")
        else:
            cell_lbl.setText("no stable cells in this CSV")
        replot()

    def on_target_edited():
        # each spin is an independent request; the selection lands on the
        # measured stable cell nearest the (RPM, traverse) pair
        if state["rows"]:
            select_cell(nearest_stable_cell(state["rows"], rpm=rpm_spin.value(),
                                            traverse=trav_spin.value()))

    def on_map_click(event):
        if event.inaxes is not ax or event.xdata is None or not state["rows"]:
            return
        select_cell(nearest_stable_cell(state["rows"], rpm=event.ydata,
                                        traverse=event.xdata))

    def load_csv(path=None):
        """Load a CSV into the DIALOG only; the main window's CSV changes on Apply."""
        if path is None:
            path, _ = QtWidgets.QFileDialog.getOpenFileName(
                dlg, "Open process-window CSV", "", "CSV (*.csv);;All files (*)")
            if not path:
                return
        try:
            state["rows"] = load_rows(path)
        except Exception as e:
            csv_lbl.setText(f"ERROR: {e}")
            return
        state["csv"] = path
        csv_lbl.setText(Path(path).name)
        travs = [_trav(r) for r in state["rows"]]
        trav_spin.blockSignals(True)
        trav_spin.setRange(0.0, (max(travs) * 1.5) if travs else 10000.0)
        trav_spin.blockSignals(False)
        # initial suggestion = what auto mode would pick (the widest contiguous
        # stable run's midpoint); the user adjusts RPM/traverse freely from there
        nv = widest_ray(state["rows"], tol)
        run = ray_run(state["rows"], nv, tol) if nv is not None else []
        select_cell(run[len(run) // 2] if run
                    else nearest_stable_cell(state["rows"]))

    def apply_to_cfg():
        """The single commit point: what the map DISPLAYS is what the slicer runs.

        The displayed cell is PINNED as a manual target (its own revs/mm +
        traverse) — the pipeline's auto search walks candidates differently
        than the dialog and could pick another cell, silently running at a
        different RPM/feed than displayed. WYSIWYG or nothing."""
        cell = chosen_cell()
        nv = selected_nv()
        if nv is None:
            win.cfg.screener.revs_per_mm_mode = "auto"      # no data loaded
            win.cfg.screener.revs_per_mm_target = 0.0
        else:
            win.cfg.screener.revs_per_mm_mode = "manual"
            win.cfg.screener.revs_per_mm_target = nv
        win.cfg.screener.traverse_target = _trav(cell) if cell else 0.0
        win.cfg.process.bed_temp_c = bed_spin.value()
        win.cfg.process.hotshoe_macro = hotshoe_macro_name(hot_spin.value())
        # the CSV path rides the config too (material presets / project files
        # round-trip it); the pipeline still receives win.csv_path explicitly.
        # A temp-extracted embedded CSV keeps its ORIGINAL source in the
        # config (sticky provenance), and an empty dialog never erases it.
        if state["csv"]:
            prov = getattr(win, "_csv_provenance", None)
            win.cfg.screener.csv_path = (
                prov if prov and state["csv"] == win.csv_path else state["csv"])
            win.csv_path = state["csv"]
            win._csv_provenance = win.cfg.screener.csv_path
            win.csv_lbl.setText(Path(state["csv"]).name)
        if cell:
            picked = (f"cell RPM={int(round(float(cell['rpm'])))}, "
                      f"v={_trav(cell):g} mm/min ({nv:g} revs/mm, pinned)")
        else:
            picked = "auto (no screener data)"
        win._log(f"process window applied: {picked}, "
                 f"bed {bed_spin.value():g} °C, "
                 f"hotshoe {hot_spin.value():g} °C ({win.cfg.process.hotshoe_macro})")

    def save_profile():
        name = mat_name.text().strip()
        if not name:
            csv_lbl.setText("enter a material name to save")
            return
        cell = chosen_cell()
        nv = selected_nv()          # save the DISPLAYED ray (pinned; WYSIWYG)
        profs = load_profiles(profiles_path())
        profs[name] = MaterialProfile(
            name=name, csv_path=state["csv"] or "",
            revs_per_mm=nv or 0.0,
            traverse_mm_min=_trav(cell) if cell else 0.0,
            bed_temp_c=bed_spin.value(), hotshoe_temp_c=hot_spin.value())
        save_profiles(profs, profiles_path())
        refresh_materials(select=name)
        win._log(f"material profile saved: {name} -> {profiles_path()}")

    def refresh_materials(select=None):
        mat_box.blockSignals(True)
        mat_box.clear()
        mat_box.addItem("— load a profile —", None)
        for name, prof in sorted(load_profiles(profiles_path()).items()):
            mat_box.addItem(name, prof)
        if select is not None:
            mat_box.setCurrentText(select)
        mat_box.blockSignals(False)

    def on_material(index):
        """Populate the DIALOG from a profile — nothing touches the live config
        until Apply. A profile whose CSV is missing loads only its thermal targets
        and says so loudly (silently pairing its ray/traverse with whatever CSV
        happens to be loaded would run the wrong material's window)."""
        prof = mat_box.itemData(index)
        if prof is None:
            return
        bed_spin.setValue(prof.bed_temp_c)
        hot_spin.setValue(prof.hotshoe_temp_c)
        mat_name.setText(prof.name)
        if not prof.csv_path or not Path(prof.csv_path).exists():
            csv_lbl.setText(f"PROFILE CSV MISSING: {prof.csv_path or '(none)'} — "
                            "temperatures loaded; re-pick the operating window")
            win._log(f"material profile {prof.name}: CSV missing "
                     f"({prof.csv_path or 'none'}); only temps loaded")
            return
        load_csv(prof.csv_path)
        if state["rows"] and (prof.revs_per_mm > 0 or prof.traverse_mm_min > 0):
            # reselect the profile's cell: rpm = revs/mm x traverse when both
            # are recorded; either target alone still snaps on its own axis
            rpm = (prof.revs_per_mm * prof.traverse_mm_min
                   if prof.revs_per_mm > 0 and prof.traverse_mm_min > 0 else None)
            select_cell(nearest_stable_cell(
                state["rows"], rpm=rpm,
                traverse=prof.traverse_mm_min if prof.traverse_mm_min > 0 else None))
        win._log(f"material profile loaded into the dialog: {prof.name} "
                 "(Apply to commit)")

    btn_csv.clicked.connect(lambda: load_csv())
    rpm_spin.editingFinished.connect(on_target_edited)
    trav_spin.editingFinished.connect(on_target_edited)
    canvas.mpl_connect("button_press_event", on_map_click)
    btn_save.clicked.connect(save_profile)
    mat_box.activated.connect(on_material)
    btn_apply.clicked.connect(apply_to_cfg)
    btn_close.clicked.connect(dlg.accept)

    refresh_materials()
    if win.csv_path:
        load_csv(win.csv_path)
    else:
        replot()
    dlg.exec()
