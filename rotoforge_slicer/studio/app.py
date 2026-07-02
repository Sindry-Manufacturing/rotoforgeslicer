"""Studio main window: 3D build plate + toolpath + kinematic simulation. SPEC §9.

Prepare mode: load meshes onto the simulated plate, click to select, click the plate
to move the selected part, tumble/scale via the transform panel; parts always drop
to the bed; fit/overlap issues are reported live. Slice runs the normal pipeline
(scene placement replaces ``place_on_bed``) off the UI thread.

Preview mode: the tagged toolpath (U2) rendered in the same viewport with the five
move-class toggles + a layer scrubber, and a kinematic playback (``simulate``) —
moving head, live wheel-heading arrow, contact state, RPM / traverse / revs-per-mm /
E readouts, airborne dwells included.

PySide6 / pyvista / trimesh are imported lazily; run with
``python -m rotoforge_slicer.studio``.
"""
from __future__ import annotations

import sys
from pathlib import Path

from .simulate import build_timeline, state_at, total_duration_s

SPEEDS = (("0.25x", 0.25), ("1x", 1.0), ("4x", 4.0), ("16x", 16.0), ("64x", 64.0))
_TICK_MS = 33


def _build_studio_window():
    """Construct (without showing) the studio window. Split out so a headless smoke
    test can build it under an offscreen Qt (same pattern as ``gui.app``)."""
    from PySide6 import QtCore, QtWidgets
    from pyvistaqt import QtInteractor

    from ..gui.app import _default_config
    from ..gui.model import preview_from_model
    from .scene import SceneModel
    from .viewport import BuildPlateScene

    class _Cancelled(Exception):
        pass

    class SliceWorker(QtCore.QObject):
        progress = QtCore.Signal(float, str)
        finished = QtCore.Signal(object)
        failed = QtCore.Signal(str)

        def __init__(self, scene, cfg, csv):
            super().__init__()
            # ``scene`` is a SceneModel.snapshot() — never the live scene, so the
            # Prepare UI can keep editing parts while this thread slices.
            self._scene, self._cfg, self._csv = scene, cfg, csv
            self.cancel = False              # checked at every pipeline stage tick

        def _tick(self, frac, msg):
            if self.cancel:
                raise _Cancelled()
            self.progress.emit(frac, msg)

        @QtCore.Slot()
        def run(self):
            try:
                self._tick(0.1, "slicing placed parts…")
                model = self._scene.slice_scene(self._cfg)
                pv = preview_from_model(model, self._cfg, self._csv,
                                        progress=self._tick, source="studio scene")
                self.finished.emit(pv)
            except _Cancelled:
                self.failed.emit("slice cancelled")
            except Exception as e:
                self.failed.emit(f"{type(e).__name__}: {e}")

    class StudioWindow(QtWidgets.QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("Rotoforge Studio")
            self.resize(1400, 860)
            self.cfg = _default_config()
            self.scene = SceneModel()
            self.selected = None
            self.preview = None
            self.timeline = []
            self.sim_t = 0.0
            self.playing = False
            self.csv_path = None
            self._thread = None
            self._worker = None
            self._build_ui()

        def closeEvent(self, event):
            # Cancel the slice at its next stage boundary, then wait for the worker
            # (the heavy per-stage calls have no interruption points, so the wait can
            # still take the remainder of the current stage — never kill the thread).
            if self._worker:
                self._worker.cancel = True
                self.statusBar().showMessage("finishing the current slice stage…")
            self._stop_thread()
            self.timer.stop()
            super().closeEvent(event)

        # ---- UI --------------------------------------------------------------

        def _build_ui(self):
            central = QtWidgets.QWidget()
            self.setCentralWidget(central)
            root = QtWidgets.QHBoxLayout(central)
            split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
            root.addWidget(split)

            # ---- left: parts + transform + process ----
            left = QtWidgets.QWidget()
            lyt = QtWidgets.QVBoxLayout(left)

            pbox = QtWidgets.QGroupBox("Parts")
            pl = QtWidgets.QVBoxLayout(pbox)
            self.part_list = QtWidgets.QListWidget()
            self.part_list.currentRowChanged.connect(self._on_select_row)
            pl.addWidget(self.part_list)
            prow = QtWidgets.QHBoxLayout()
            for text, slot in (("Add mesh…", self._add_mesh),
                               ("Duplicate", self._duplicate),
                               ("Remove", self._remove)):
                b = QtWidgets.QPushButton(text)
                b.clicked.connect(slot)
                prow.addWidget(b)
            pl.addLayout(prow)
            lyt.addWidget(pbox)

            tbox = QtWidgets.QGroupBox("Transform (selected part)")
            tf = QtWidgets.QFormLayout(tbox)
            bx, by, _ = self.cfg.machine.build_volume_mm
            self.t_x = self._dspin(0, 0, bx, 1.0, 1)
            self.t_y = self._dspin(0, 0, by, 1.0, 1)
            self.t_rx = self._dspin(0, -180, 180, 15.0, 1)
            self.t_ry = self._dspin(0, -180, 180, 15.0, 1)
            self.t_rz = self._dspin(0, -180, 180, 15.0, 1)
            self.t_s = self._dspin(1.0, 0.05, 20.0, 0.1, 2)
            for lbl, w in (("X (mm)", self.t_x), ("Y (mm)", self.t_y),
                           ("Rotate X (deg)", self.t_rx), ("Rotate Y (deg)", self.t_ry),
                           ("Rotate Z (deg)", self.t_rz), ("Scale", self.t_s)):
                tf.addRow(lbl, w)
            for w in (self.t_x, self.t_y, self.t_rx, self.t_ry, self.t_rz, self.t_s):
                w.valueChanged.connect(self._on_transform_edit)
            lyt.addWidget(tbox)

            form = QtWidgets.QFormLayout()
            p = self.cfg.process
            self.f_lh = self._dspin(p.layer_height_mm, 0.02, 1.0, 0.01, 3)
            self.f_bw = self._dspin(p.bead_width_mm, 0.2, 5.0, 0.1, 2)
            self.f_ov = self._dspin(p.raster_overlap, 0.0, 0.8, 0.05, 2)
            self.f_ml = self._dspin(p.min_deposit_len_mm, 1.0, 50.0, 0.5, 1)
            self.f_amin = self._dspin(self.cfg.c_axis.a_min_deg, -360.0, 0.0, 5.0, 0)
            self.f_amax = self._dspin(self.cfg.c_axis.a_max_deg, 0.0, 360.0, 5.0, 0)
            self.f_mode = QtWidgets.QComboBox()
            self.f_mode.addItems(["raster", "streamline"])
            self.f_mode.setCurrentText(self.cfg.fill.mode)
            self.f_cross = QtWidgets.QCheckBox("crosshatch (alternate heading per layer)")
            self.f_cross.setChecked(self.cfg.fill.crosshatch)
            form.addRow("Layer height (mm)", self.f_lh)
            form.addRow("Bead width (mm)", self.f_bw)
            form.addRow("Raster overlap", self.f_ov)
            form.addRow("Min deposit len (mm)", self.f_ml)
            form.addRow("C-axis A min (deg)", self.f_amin)
            form.addRow("C-axis A max (deg)", self.f_amax)
            form.addRow("Fill mode", self.f_mode)
            form.addRow(self.f_cross)
            box = QtWidgets.QGroupBox("Process")
            box.setLayout(form)
            lyt.addWidget(box)

            btn_csv = QtWidgets.QPushButton("Open process-window CSV…")
            btn_csv.clicked.connect(self._open_csv)
            self.csv_lbl = QtWidgets.QLabel("no CSV (single-speed fallback)")
            self.csv_lbl.setWordWrap(True)
            self.btn_slice = QtWidgets.QPushButton("Slice")
            self.btn_slice.clicked.connect(self._slice)
            self.btn_save = QtWidgets.QPushButton("Save G-code…")
            self.btn_save.clicked.connect(self._save)
            self.btn_save.setEnabled(False)
            self.issues_lbl = QtWidgets.QLabel("")
            self.issues_lbl.setWordWrap(True)
            self.issues_lbl.setStyleSheet("color: #c0392b;")
            for w in (btn_csv, self.csv_lbl, self.btn_slice, self.btn_save,
                      self.issues_lbl):
                lyt.addWidget(w)
            lyt.addStretch(1)
            left.setMaximumWidth(360)
            split.addWidget(left)

            # ---- right: 3D viewport + mode tabs + log ----
            right = QtWidgets.QWidget()
            rlyt = QtWidgets.QVBoxLayout(right)
            self.interactor = QtInteractor(right)
            self.view = BuildPlateScene(plotter=self.interactor)
            self.view.draw_plate(self.cfg)
            # Double-click only: single left-press also starts a camera-orbit drag,
            # which must not select or teleport parts.
            self.interactor.track_click_position(self._on_view_click, side="left",
                                                 double=True)
            rlyt.addWidget(self.interactor, stretch=1)

            self.tabs = QtWidgets.QTabWidget()
            self.tabs.setMaximumHeight(170)

            prep = QtWidgets.QWidget()
            pl2 = QtWidgets.QVBoxLayout(prep)
            hint = QtWidgets.QLabel(
                "Double-click a part to select it; double-click the plate to move the "
                "selected part there (drag orbits the camera). Parts always rest on "
                "the bed; rotations tumble about the part centre. Fit and overlap "
                "problems appear on the left.")
            hint.setWordWrap(True)
            pl2.addWidget(hint)
            pl2.addStretch(1)
            self.tabs.addTab(prep, "Prepare")

            prev = QtWidgets.QWidget()
            vl = QtWidgets.QVBoxLayout(prev)
            trow = QtWidgets.QHBoxLayout()
            trow.addWidget(QtWidgets.QLabel("Show:"))
            from ..toolpath.segments import TOGGLE_ORDER
            self.toggles = {}
            for name in TOGGLE_ORDER:
                cb = QtWidgets.QCheckBox(name)
                cb.setChecked(True)
                cb.stateChanged.connect(self._refresh_toolpath)
                self.toggles[name] = cb
                trow.addWidget(cb)
            trow.addStretch(1)
            trow.addWidget(QtWidgets.QLabel("Layers up to"))
            self.layer_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            self.layer_slider.setEnabled(False)
            self.layer_slider.valueChanged.connect(self._refresh_toolpath)
            self.layer_lbl = QtWidgets.QLabel("—")
            trow.addWidget(self.layer_slider, stretch=1)
            trow.addWidget(self.layer_lbl)
            vl.addLayout(trow)

            srow = QtWidgets.QHBoxLayout()
            self.btn_play = QtWidgets.QPushButton("▶ Play")
            self.btn_play.setEnabled(False)
            self.btn_play.clicked.connect(self._toggle_play)
            self.speed = QtWidgets.QComboBox()
            for label, _ in SPEEDS:
                self.speed.addItem(label)
            self.speed.setCurrentIndex(1)
            self.time_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            self.time_slider.setRange(0, 1000)
            self.time_slider.setEnabled(False)
            self.time_slider.valueChanged.connect(self._on_time_scrub)
            srow.addWidget(self.btn_play)
            srow.addWidget(QtWidgets.QLabel("Speed"))
            srow.addWidget(self.speed)
            srow.addWidget(self.time_slider, stretch=1)
            vl.addLayout(srow)

            self.readout = QtWidgets.QLabel("slice to enable the simulation")
            self.readout.setStyleSheet("font-family: Consolas, monospace;")
            vl.addWidget(self.readout)
            self.tabs.addTab(prev, "Preview")
            rlyt.addWidget(self.tabs)

            self.progress = QtWidgets.QProgressBar()
            self.progress.setRange(0, 100)
            rlyt.addWidget(self.progress)
            self.log = QtWidgets.QPlainTextEdit()
            self.log.setReadOnly(True)
            self.log.setMaximumHeight(110)
            rlyt.addWidget(self.log)
            split.addWidget(right)
            split.setStretchFactor(1, 1)

            self.timer = QtCore.QTimer(self)
            self.timer.setInterval(_TICK_MS)
            self.timer.timeout.connect(self._on_tick)

        def _dspin(self, val, lo, hi, step, dec):
            s = QtWidgets.QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setSingleStep(step)
            s.setDecimals(dec)
            s.setValue(val)
            return s

        def _log(self, msg):
            self.log.appendPlainText(msg)

        # ---- scene / prepare ---------------------------------------------------

        def _sync_scene(self):
            self.view.sync_parts(self.scene.parts, selected=self.selected)
            self.issues_lbl.setText("\n".join(self.scene.issues(self.cfg)))
            self.interactor.update()

        def _refresh_part_list(self):
            self.part_list.blockSignals(True)
            self.part_list.clear()
            for p in self.scene.parts:
                self.part_list.addItem(p.name)
            if self.selected in self.scene.parts:
                self.part_list.setCurrentRow(self.scene.parts.index(self.selected))
            self.part_list.blockSignals(False)

        def _add_mesh(self):
            fn, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, "Add mesh", "", "Meshes (*.stl *.3mf *.obj *.ply);;All files (*)")
            if fn:
                self.add_mesh_file(fn)

        def add_mesh_file(self, fn):
            from ..geometry.trimesh_backend import TrimeshBackend

            try:
                mesh = TrimeshBackend().load(fn)
            except Exception as e:
                self._log(f"ERROR loading {fn}: {e}")
                return
            part = self.scene.add(mesh, name=Path(fn).stem, cfg=self.cfg)
            self.selected = part
            self._refresh_part_list()
            self._load_transform_form()
            self._sync_scene()
            self.view.reset_camera()
            self._log(f"added {part.name}")

        def _duplicate(self):
            if self.selected:
                self.selected = self.scene.duplicate(self.selected)
                self._refresh_part_list()
                self._load_transform_form()
                self._sync_scene()

        def _remove(self):
            if self.selected:
                self.scene.remove(self.selected)
                self.selected = self.scene.parts[-1] if self.scene.parts else None
                self._refresh_part_list()
                self._load_transform_form()
                self._sync_scene()

        def _on_select_row(self, row):
            if 0 <= row < len(self.scene.parts):
                self.selected = self.scene.parts[row]
                self._load_transform_form()
                self._sync_scene()

        def _load_transform_form(self):
            p = self.selected
            for w, val in ((self.t_x, p.x if p else 0), (self.t_y, p.y if p else 0),
                           (self.t_rx, p.rot_x_deg if p else 0),
                           (self.t_ry, p.rot_y_deg if p else 0),
                           (self.t_rz, p.rot_z_deg if p else 0),
                           (self.t_s, p.scale if p else 1.0)):
                w.blockSignals(True)
                w.setValue(val)
                w.blockSignals(False)

        def _on_transform_edit(self, *_):
            if not self.selected:
                return
            self.selected.set_transform(
                x=self.t_x.value(), y=self.t_y.value(),
                rot_x_deg=self.t_rx.value(), rot_y_deg=self.t_ry.value(),
                rot_z_deg=self.t_rz.value(), scale=self.t_s.value())
            self._sync_scene()

        def _on_view_click(self, point):
            """Double-click a part -> select it; double-click the plate -> move the
            selection there. The world-point picker never returns None — a click on
            empty sky lands on the far clipping plane — so accept only points inside
            the build volume (with a little slack below the bed for plate hits)."""
            if point is None or self.tabs.currentIndex() != 0:
                return
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            bx, by, bz = self.cfg.machine.build_volume_mm
            if not (0 <= x <= bx and 0 <= y <= by and -1.0 <= z <= bz):
                return
            for part in self.scene.parts:
                x0, y0, x1, y1 = part.footprint()
                if x0 <= x <= x1 and y0 <= y <= y1:
                    self.selected = part
                    self._refresh_part_list()
                    self._load_transform_form()
                    self._sync_scene()
                    return
            if self.selected:
                self.selected.set_transform(x=x, y=y)
                self._load_transform_form()
                self._sync_scene()

        # ---- slicing -------------------------------------------------------------

        def _open_csv(self):
            fn, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, "Open process-window CSV", "", "CSV (*.csv);;All files (*)")
            if fn:
                self.csv_path = fn
                self.csv_lbl.setText(Path(fn).name)

        def _apply_params(self):
            p = self.cfg.process
            p.layer_height_mm = self.f_lh.value()
            p.bead_width_mm = self.f_bw.value()
            p.raster_overlap = self.f_ov.value()
            p.min_deposit_len_mm = self.f_ml.value()
            self.cfg.c_axis.a_min_deg = self.f_amin.value()
            self.cfg.c_axis.a_max_deg = self.f_amax.value()
            self.cfg.fill.mode = self.f_mode.currentText()
            self.cfg.fill.crosshatch = self.f_cross.isChecked()

        def _slice(self):
            if not self.scene.parts:
                self._log("Add a mesh first.")
                return
            issues = self.scene.issues(self.cfg)
            if issues:
                self._log("Placement issues:\n  " + "\n  ".join(issues))
            self._apply_params()
            self._set_playing(False)
            self.btn_slice.setEnabled(False)
            self.btn_save.setEnabled(False)
            self.progress.setValue(0)
            self._thread = QtCore.QThread()
            # snapshot: the worker slices a frozen copy; the live scene stays editable
            self._worker = SliceWorker(self.scene.snapshot(), self.cfg, self.csv_path)
            self._worker.moveToThread(self._thread)
            self._thread.started.connect(self._worker.run)
            self._worker.progress.connect(self._on_progress)
            self._worker.finished.connect(self._on_done)
            self._worker.failed.connect(self._on_failed)
            self._thread.start()

        def _on_progress(self, frac, msg):
            self.progress.setValue(int(frac * 100))
            self.statusBar().showMessage(msg)

        def _stop_thread(self):
            if self._thread:
                self._thread.quit()
                self._thread.wait()
                self._thread.deleteLater()
                if self._worker:
                    self._worker.deleteLater()
                self._thread = None
                self._worker = None

        def _on_failed(self, msg):
            self._stop_thread()
            self.btn_slice.setEnabled(True)
            self._log(f"ERROR: {msg}")

        def _on_done(self, preview):
            self._stop_thread()
            self.preview = preview
            self.btn_slice.setEnabled(True)
            self.btn_save.setEnabled(preview.gcode is not None)
            for line in preview.summary_lines():
                self._log("  " + line)
            self.timeline = build_timeline(preview.segments, preview.plan, self.cfg)
            self.sim_t = 0.0
            nlayers = len(preview.plan.layers)
            self.layer_slider.blockSignals(True)
            self.layer_slider.setRange(0, max(0, nlayers - 1))
            self.layer_slider.setValue(max(0, nlayers - 1))
            self.layer_slider.setEnabled(True)
            self.layer_slider.blockSignals(False)
            self.time_slider.setEnabled(bool(self.timeline))
            self.btn_play.setEnabled(bool(self.timeline))
            self.tabs.setCurrentIndex(1)
            self._refresh_toolpath()
            if preview.collisions:
                self.view.show_collisions(preview.collisions)
            self._update_sim_display()

        # ---- preview / simulation --------------------------------------------------

        def _refresh_toolpath(self, *_):
            if not self.preview:
                return
            enabled = {n for n, cb in self.toggles.items() if cb.isChecked()}
            upto = self.layer_slider.value() if self.layer_slider.isEnabled() else None
            self.view.show_toolpath(self.preview.segments, enabled=enabled,
                                    upto_layer=upto)
            if self.preview.collisions:
                self.view.show_collisions(self.preview.collisions)
            self.layer_lbl.setText(f"{upto}" if upto is not None else "—")
            self.interactor.update()

        def _set_playing(self, playing: bool):
            self.playing = playing and bool(self.timeline)
            self.btn_play.setText("⏸ Pause" if self.playing else "▶ Play")
            (self.timer.start if self.playing else self.timer.stop)()

        def _toggle_play(self):
            if not self.playing and self.timeline \
                    and self.sim_t >= total_duration_s(self.timeline):
                self.sim_t = 0.0                      # replay from the start
            self._set_playing(not self.playing)

        def _on_tick(self):
            speed = SPEEDS[self.speed.currentIndex()][1]
            self.sim_t += (_TICK_MS / 1000.0) * speed
            total = total_duration_s(self.timeline)
            if self.sim_t >= total:
                self.sim_t = total
                self._set_playing(False)
            self._update_sim_display()

        def _on_time_scrub(self, v):
            if self.timeline and self.time_slider.isEnabled():
                self.sim_t = (v / 1000.0) * total_duration_s(self.timeline)
                self._update_sim_display(scrubbed=True)

        def _update_sim_display(self, scrubbed: bool = False):
            if not self.timeline:
                return
            state = state_at(self.timeline, self.sim_t, self.cfg.c_axis)
            self.view.update_head(state, self.cfg)
            total = total_duration_s(self.timeline)
            if not scrubbed:
                self.time_slider.blockSignals(True)
                self.time_slider.setValue(int(1000 * state.t / total) if total else 0)
                self.time_slider.blockSignals(False)
            self.readout.setText(
                f"t {state.t:8.1f}/{total:.1f}s  {state.kind:<10}"
                f"{'CONTACT ' if state.in_contact else 'airborne'}  "
                f"X{state.x:7.2f} Y{state.y:7.2f} Z{state.z:6.2f}  "
                f"A{state.a_deg:7.1f}°  RPM {state.rpm:5d}  "
                f"v {state.v_mm_min:6.0f}mm/min  n/v {state.revs_per_mm:6.1f}  "
                f"E {state.e_mm:8.2f}mm  L{state.layer_index} P{state.pass_index}")
            self.interactor.update()

        # ---- export -----------------------------------------------------------------

        def _save(self):
            if not (self.preview and self.preview.gcode):
                return
            if self.preview.collisions:
                resp = QtWidgets.QMessageBox.warning(
                    self, "Collisions detected",
                    f"{len(self.preview.collisions)} collision(s) were detected "
                    f"(SPEC §4.6). The CLI would refuse to emit this path.\n\n"
                    "Save the G-code anyway?",
                    QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                    QtWidgets.QMessageBox.No)
                if resp != QtWidgets.QMessageBox.Yes:
                    return
            fn, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, "Save G-code", "studio.gcode", "G-code (*.gcode)")
            if fn:
                Path(fn).write_text(self.preview.gcode, encoding="utf-8")
                self._log(f"Saved {fn}")

    return StudioWindow()


def main(argv=None) -> int:
    try:
        from PySide6 import QtWidgets
        import pyvistaqt  # noqa: F401
    except Exception as e:  # pragma: no cover
        print(f"studio needs PySide6 + pyvista + pyvistaqt: {e}", file=sys.stderr)
        return 1
    args = list(sys.argv[1:] if argv is None else argv)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    win = _build_studio_window()
    win.show()
    for a in args:
        if a.lower().endswith((".stl", ".3mf", ".obj", ".ply")) and Path(a).exists():
            win.add_mesh_file(a)
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
