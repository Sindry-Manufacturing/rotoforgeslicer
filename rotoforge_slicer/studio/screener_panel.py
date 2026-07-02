"""The graphical process-window (screener) dialog. SPEC §5/§9.

Lets the user *see* the FRAM screener map (``screener_plot``) and choose the
operating window instead of typing revs/mm numbers: pick a constant-revs/mm ray,
slide along its contiguous stable run to choose the representative cell (the
traverse/RPM pair — selections always snap to measured cells, SPEC §5), set the
bed and hotshoe temperature targets, and save/apply the whole selection as a named
per-material profile (``materials``).

Applying writes ``cfg.screener`` (mode/target/traverse_target) and the thermal
targets; the normal pipeline then selects exactly that cell on the next slice.
PySide6/matplotlib are imported inside the builder (lazy-import rule).
"""
from __future__ import annotations

from pathlib import Path

from ..process.screener import (
    _trav, distinct_rays, load_rows, ray_run, widest_ray,
)
from .materials import (
    MaterialProfile, hotshoe_macro_name, hotshoe_temp_from_macro,
    load_profiles, save_profiles,
)


def profiles_path() -> Path:
    """Material profiles live next to the machine config when running from the
    repo; a frozen app (no writable ``config/``) falls back to the user's home."""
    repo_cfg = Path.cwd() / "config"
    if repo_cfg.is_dir():
        return repo_cfg / "materials.yaml"
    return Path.home() / ".rotoforge" / "materials.yaml"


def open_screener_dialog(win) -> None:
    """Build and exec the process-window dialog against the studio main window
    (reads ``win.csv_path`` / ``win.cfg``, writes the selection back on Apply)."""
    from PySide6 import QtCore, QtWidgets

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

    state = {"rows": [], "run": []}
    tol = win.cfg.screener.revs_per_mm_tol

    csv_lbl = QtWidgets.QLabel("no CSV loaded")
    csv_lbl.setWordWrap(True)
    btn_csv = QtWidgets.QPushButton("Load screener CSV…")
    side.addWidget(btn_csv)
    side.addWidget(csv_lbl)

    ray_box = QtWidgets.QComboBox()          # "auto" + one entry per stable ray
    side.addWidget(QtWidgets.QLabel("revs/mm ray (stability window)"))
    side.addWidget(ray_box)

    v_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
    v_lbl = QtWidgets.QLabel("—")
    side.addWidget(QtWidgets.QLabel("operating cell on the ray"))
    side.addWidget(v_slider)
    side.addWidget(v_lbl)

    rpm_spin = QtWidgets.QSpinBox()
    rpm_spin.setRange(win.cfg.spindle.rpm_min, win.cfg.spindle.rpm_max)
    rpm_spin.setSingleStep(500)
    side.addWidget(QtWidgets.QLabel("spindle RPM target (snaps to a cell)"))
    side.addWidget(rpm_spin)

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
        i = ray_box.currentIndex()
        if i <= 0:
            return widest_ray(state["rows"], tol)
        return float(ray_box.itemData(i))

    def chosen_cell():
        run = state["run"]
        if not run:
            return None
        return run[max(0, min(v_slider.value(), len(run) - 1))]

    def replot():
        cell = chosen_cell()
        plot_screener_map(
            ax, state["rows"], selected_nv=selected_nv(), tol=tol,
            chosen_traverse=_trav(cell) if cell else None,
            rpm_window=(win.cfg.spindle.rpm_min, win.cfg.spindle.rpm_max))
        canvas.draw_idle()

    def refresh_run():
        nv = selected_nv()
        state["run"] = ray_run(state["rows"], nv, tol) if nv is not None else []
        v_slider.blockSignals(True)
        v_slider.setRange(0, max(0, len(state["run"]) - 1))
        v_slider.setValue(len(state["run"]) // 2)
        v_slider.blockSignals(False)
        on_cell_changed()

    def on_cell_changed(*_):
        cell = chosen_cell()
        if cell:
            rpm = int(round(float(cell["rpm"])))
            v_lbl.setText(f"v = {_trav(cell):g} mm/min   RPM = {rpm}   "
                          f"wire = {float(cell['feed_speed_mm_min']):g} mm/min   "
                          f"T_AZ = {float(cell['T_AZ_C']):g} °C")
            rpm_spin.blockSignals(True)
            rpm_spin.setValue(rpm)
            rpm_spin.blockSignals(False)
        else:
            v_lbl.setText("no contiguous stable run on this ray")
        replot()

    def on_rpm_edited():
        run = state["run"]
        if run:
            i = min(range(len(run)),
                    key=lambda j: abs(float(run[j]["rpm"]) - rpm_spin.value()))
            v_slider.setValue(i)                 # snaps to the nearest measured cell

    def load_csv(path=None):
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
        win.csv_path = path
        win.csv_lbl.setText(Path(path).name)
        csv_lbl.setText(Path(path).name)
        ray_box.blockSignals(True)
        ray_box.clear()
        ray_box.addItem("auto (widest stable window)", None)
        for nv in distinct_rays(state["rows"], tol):
            ray_box.addItem(f"revs/mm = {nv:g}", nv)
        ray_box.blockSignals(False)
        refresh_run()

    def apply_to_cfg():
        cell = chosen_cell()
        nv = selected_nv()
        if ray_box.currentIndex() <= 0:
            win.cfg.screener.revs_per_mm_mode = "auto"
            win.cfg.screener.revs_per_mm_target = 0.0
        else:
            win.cfg.screener.revs_per_mm_mode = "manual"
            win.cfg.screener.revs_per_mm_target = nv or 0.0
        win.cfg.screener.traverse_target = _trav(cell) if cell else 0.0
        win.cfg.process.bed_temp_c = bed_spin.value()
        win.cfg.process.hotshoe_macro = hotshoe_macro_name(hot_spin.value())
        win._log(
            f"process window applied: ray="
            f"{'auto' if ray_box.currentIndex() <= 0 else f'{nv:g} revs/mm'}"
            f"{f', v={_trav(cell):g} mm/min' if cell else ''}, "
            f"bed {bed_spin.value():g} °C, "
            f"hotshoe {hot_spin.value():g} °C ({win.cfg.process.hotshoe_macro})")

    def save_profile():
        name = mat_name.text().strip()
        if not name:
            csv_lbl.setText("enter a material name to save")
            return
        cell = chosen_cell()
        nv = selected_nv() if ray_box.currentIndex() > 0 else 0.0
        profs = load_profiles(profiles_path())
        profs[name] = MaterialProfile(
            name=name, csv_path=win.csv_path or "",
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
        prof = mat_box.itemData(index)
        if prof is None:
            return
        prof.apply_to_cfg(win.cfg)
        bed_spin.setValue(prof.bed_temp_c)
        hot_spin.setValue(prof.hotshoe_temp_c)
        mat_name.setText(prof.name)
        if prof.csv_path and Path(prof.csv_path).exists():
            load_csv(prof.csv_path)
            if prof.revs_per_mm > 0:            # reselect the profile's ray + cell
                for i in range(1, ray_box.count()):
                    if abs(float(ray_box.itemData(i)) - prof.revs_per_mm) <= tol:
                        ray_box.setCurrentIndex(i)
                        break
                run = state["run"]
                if run and prof.traverse_mm_min > 0:
                    j = min(range(len(run)), key=lambda k: abs(
                        _trav(run[k]) - prof.traverse_mm_min))
                    v_slider.setValue(j)
        win._log(f"material profile loaded: {prof.name}")

    btn_csv.clicked.connect(lambda: load_csv())
    ray_box.currentIndexChanged.connect(lambda *_: refresh_run())
    v_slider.valueChanged.connect(on_cell_changed)
    rpm_spin.editingFinished.connect(on_rpm_edited)
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
