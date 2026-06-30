"""PySide6 main window for the Rotoforge slicer. SPEC §9.

A thin wrapper over the same pipeline the CLI uses: open a mesh + optional process
CSV, tweak the key process fields, Slice (off the UI thread), then scrub layers with a
slider and inspect the toolpath (deposition vectors, lead-outs/wire-cuts, resets, the
+/-45 wedge, and any collisions) with mouse zoom/pan. Save the validated G-code.

PySide6 / matplotlib are imported lazily so importing the package stays light.
"""
from __future__ import annotations

import sys
from pathlib import Path


def _default_config():
    """Best-effort load of the machine config; fall back to built-in defaults."""
    from ..config import Config, load_config

    here = Path(__file__).resolve()
    candidates = [Path.cwd() / "config" / "machine_duet3.yaml",
                  here.parents[2] / "config" / "machine_duet3.yaml"]
    bundle = getattr(sys, "_MEIPASS", None)            # PyInstaller frozen-app data dir
    if bundle:
        candidates.insert(0, Path(bundle) / "config" / "machine_duet3.yaml")
    for c in candidates:
        if c.exists():
            try:
                return load_config(c)
            except Exception:
                break
    return Config()


def _build_main_window():
    """Construct (without showing) the main window class. Returns the QMainWindow
    instance. Split out so a headless smoke test can build it under an offscreen Qt."""
    import matplotlib
    matplotlib.use("QtAgg")
    from matplotlib.backends.backend_qtagg import NavigationToolbar2QT
    from PySide6 import QtCore, QtWidgets

    from .model import build_preview
    from .preview import plot_toolpath_layer

    class SliceWorker(QtCore.QObject):
        progress = QtCore.Signal(float, str)
        finished = QtCore.Signal(object)
        failed = QtCore.Signal(str)

        def __init__(self, mesh, cfg, csv):
            super().__init__()
            self._mesh, self._cfg, self._csv = mesh, cfg, csv

        @QtCore.Slot()
        def run(self):
            try:
                pv = build_preview(self._mesh, self._cfg, self._csv,
                                   progress=lambda f, m: self.progress.emit(f, m))
                self.finished.emit(pv)
            except Exception as e:  # surface any pipeline error to the log
                self.failed.emit(f"{type(e).__name__}: {e}")

    class MainWindow(QtWidgets.QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("Rotoforge Slicer")
            self.resize(1180, 760)
            self.cfg = _default_config()
            self.mesh_path = None
            self.csv_path = None
            self.preview = None
            self._thread = None
            self._worker = None
            self._build_ui()

        def closeEvent(self, event):
            # never let the window be destroyed while the slice thread is running
            self._stop_thread()
            super().closeEvent(event)

        # ---- UI ----
        def _build_ui(self):
            from .preview import make_preview_canvas

            central = QtWidgets.QWidget()
            self.setCentralWidget(central)
            root = QtWidgets.QHBoxLayout(central)
            split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
            root.addWidget(split)

            # ---- left: inputs ----
            left = QtWidgets.QWidget()
            lyt = QtWidgets.QVBoxLayout(left)
            self.mesh_lbl = QtWidgets.QLabel("no mesh loaded")
            self.mesh_lbl.setWordWrap(True)
            btn_mesh = QtWidgets.QPushButton("Open mesh…")
            btn_mesh.clicked.connect(self._open_mesh)
            btn_csv = QtWidgets.QPushButton("Open process-window CSV…")
            btn_csv.clicked.connect(self._open_csv)
            self.csv_lbl = QtWidgets.QLabel("no CSV (single-speed fallback)")
            self.csv_lbl.setWordWrap(True)
            for w in (btn_mesh, self.mesh_lbl, btn_csv, self.csv_lbl):
                lyt.addWidget(w)

            form = QtWidgets.QFormLayout()
            p = self.cfg.process
            self.f_lh = self._dspin(p.layer_height_mm, 0.02, 1.0, 0.01, 3)
            self.f_bw = self._dspin(p.bead_width_mm, 0.2, 5.0, 0.1, 2)
            self.f_ov = self._dspin(p.raster_overlap, 0.0, 0.8, 0.05, 2)
            self.f_ml = self._dspin(p.min_deposit_len_mm, 1.0, 50.0, 0.5, 1)
            self.f_wedge = self._dspin(self.cfg.c_axis.wedge_half_angle_deg, 0.0, 180.0, 5.0, 0)
            self.f_mode = QtWidgets.QComboBox()
            self.f_mode.addItems(["raster", "streamline"])
            self.f_mode.setCurrentText(self.cfg.fill.mode)
            self.f_cross = QtWidgets.QCheckBox("crosshatch (alternate heading per layer)")
            self.f_cross.setChecked(self.cfg.fill.crosshatch)
            form.addRow("Layer height (mm)", self.f_lh)
            form.addRow("Bead width (mm)", self.f_bw)
            form.addRow("Raster overlap", self.f_ov)
            form.addRow("Min deposit len (mm)", self.f_ml)
            form.addRow("C-axis wedge ± (deg)", self.f_wedge)
            form.addRow("Fill mode", self.f_mode)
            form.addRow(self.f_cross)
            box = QtWidgets.QGroupBox("Process")
            box.setLayout(form)
            lyt.addWidget(box)

            self.btn_slice = QtWidgets.QPushButton("Slice")
            self.btn_slice.clicked.connect(self._slice)
            self.btn_save = QtWidgets.QPushButton("Save G-code…")
            self.btn_save.clicked.connect(self._save)
            self.btn_save.setEnabled(False)
            lyt.addWidget(self.btn_slice)
            lyt.addWidget(self.btn_save)
            lyt.addStretch(1)
            left.setMaximumWidth(330)
            split.addWidget(left)

            # ---- right: canvas + slider + log ----
            right = QtWidgets.QWidget()
            rlyt = QtWidgets.QVBoxLayout(right)
            self.canvas = make_preview_canvas(self)
            self.ax = self.canvas.figure.axes[0]
            rlyt.addWidget(NavigationToolbar2QT(self.canvas, right))
            rlyt.addWidget(self.canvas, stretch=1)

            srow = QtWidgets.QHBoxLayout()
            self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            self.slider.setEnabled(False)
            self.slider.valueChanged.connect(self._on_slider)
            self.layer_lbl = QtWidgets.QLabel("layer —")
            srow.addWidget(QtWidgets.QLabel("Layer"))
            srow.addWidget(self.slider, stretch=1)
            srow.addWidget(self.layer_lbl)
            rlyt.addLayout(srow)

            self.progress = QtWidgets.QProgressBar()
            self.progress.setRange(0, 100)
            rlyt.addWidget(self.progress)
            self.log = QtWidgets.QPlainTextEdit()
            self.log.setReadOnly(True)
            self.log.setMaximumHeight(150)
            rlyt.addWidget(self.log)
            split.addWidget(right)
            split.setStretchFactor(1, 1)

        def _dspin(self, val, lo, hi, step, dec):
            s = QtWidgets.QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setSingleStep(step)
            s.setDecimals(dec)
            s.setValue(val)
            return s

        def _log(self, msg):
            self.log.appendPlainText(msg)

        # ---- actions ----
        def _open_mesh(self):
            fn, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, "Open mesh", "", "Meshes (*.stl *.3mf *.obj *.ply);;All files (*)")
            if fn:
                self.mesh_path = fn
                self.mesh_lbl.setText(Path(fn).name)

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
            self.cfg.c_axis.wedge_half_angle_deg = self.f_wedge.value()
            self.cfg.fill.mode = self.f_mode.currentText()
            self.cfg.fill.crosshatch = self.f_cross.isChecked()

        def _slice(self):
            if not self.mesh_path:
                self._log("Open a mesh first.")
                return
            self._apply_params()
            self.btn_slice.setEnabled(False)
            self.btn_save.setEnabled(False)
            self.progress.setValue(0)
            self._log(f"Slicing {Path(self.mesh_path).name} "
                      f"({self.cfg.fill.mode}{', crosshatch' if self.cfg.fill.crosshatch else ''})…")
            self._thread = QtCore.QThread()
            self._worker = SliceWorker(self.mesh_path, self.cfg, self.csv_path)
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
                self._thread.wait()          # blocks until the worker's run() returns
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
            self.slider.setEnabled(True)
            self.slider.setRange(0, max(0, preview.layer_count - 1))
            for line in preview.summary_lines():
                self._log("  " + line)
            ne = preview.nonempty_indices
            self.slider.setValue(ne[len(ne) // 2] if ne else 0)
            self._on_slider(self.slider.value())

        def _on_slider(self, i):
            if not self.preview:
                return
            layer, lp, cols = self.preview.layer(i)
            self.ax.clear()
            plot_toolpath_layer(layer, lp, ax=self.ax, cfg=self.cfg, collisions=cols)
            self.layer_lbl.setText(f"layer {i}/{self.preview.layer_count - 1}  "
                                   f"z={layer.z:.2f}mm  {len(lp.passes)} passes")
            self.canvas.draw_idle()

        def _save(self):
            if not (self.preview and self.preview.gcode):
                return
            # the CLI pipeline refuses to emit a colliding plan (SPEC §4.6); the GUI lets
            # you inspect it, but require an explicit confirmation before writing one.
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
            default = str(Path(self.mesh_path).with_suffix(".gcode"))
            fn, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, "Save G-code", default, "G-code (*.gcode)")
            if fn:
                Path(fn).write_text(self.preview.gcode, encoding="utf-8")
                self._log(f"Saved {fn}")

    return MainWindow()


def main(argv=None) -> int:
    try:
        from PySide6 import QtCore, QtWidgets
    except Exception as e:  # pragma: no cover
        print(f"PySide6 not available: {e}", file=sys.stderr)
        return 1
    args = list(sys.argv[1:] if argv is None else argv)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    win = _build_main_window()
    win.show()
    # optional: "rotoforge-slicer-gui part.stl" opens the mesh and slices it immediately
    meshes = [a for a in args if a.lower().endswith((".stl", ".3mf", ".obj", ".ply"))]
    if meshes and Path(meshes[0]).exists():
        win.mesh_path = meshes[0]
        win.mesh_lbl.setText(Path(meshes[0]).name)
        QtCore.QTimer.singleShot(300, win._slice)
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
