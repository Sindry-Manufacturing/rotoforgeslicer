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

# Advanced process parameters (label, cfg path, lo, hi, step, decimals) — exposed in
# the collapsible Advanced group and written back on Slice. Values are config-driven
# per CLAUDE.md; this is the GUI onto them, not a second source of truth.
ADVANCED_FIELDS = (
    ("Lead-in (mm)", "process.lead_in_len_mm", 0.5, 20.0, 0.5, 1),
    ("Lead-out (mm)", "process.lead_out_len_mm", 0.5, 30.0, 0.5, 1),
    ("Approach clearance (mm)", "process.approach_clearance_mm", 0.1, 5.0, 0.1, 2),
    ("Inter-pass lift (mm)", "process.inter_pass_lift_mm", 1.0, 50.0, 1.0, 1),
    ("Wire diameter (mm)", "process.wire_diameter_mm", 0.1, 2.0, 0.05, 2),
    ("Travel feed (mm/min)", "emit.feed_travel_mm_min", 100.0, 10000.0, 50.0, 0),
    ("Z feed (mm/min)", "emit.feed_z_mm_min", 50.0, 2000.0, 10.0, 0),
    ("Deposition feed fallback (mm/min)", "emit.feed_dep_mm_min", 10.0, 2000.0, 10.0, 0),
    ("C-axis slew ω_C (deg/s)", "c_axis.max_speed_deg_s", 0.0, 2000.0, 10.0, 0),
    ("Corner scrub budget (deg·mm)", "c_axis.max_scrub_deg_mm", 0.0, 2000.0, 10.0, 0),
    ("Collision clearance (mm)", "collision.clearance_mm", 0.0, 5.0, 0.1, 2),
    ("Collision wire lead (mm)", "collision.wire_lead_mm", 0.0, 10.0, 0.5, 1),
    ("Crosshatch angle (deg)", "fill.crosshatch_angle_deg", 0.0, 90.0, 5.0, 1),
    ("Streamline step (mm)", "fill.streamline_step_mm", 0.1, 5.0, 0.1, 2),
    ("Streamline curl", "fill.streamline_curl", 0.0, 2.0, 0.1, 2),
    ("Contour simplify (mm)", "fill.contour_simplify_mm", 0.0, 1.0, 0.01, 2),
)


# Basic process-form fields (widget attr, cfg path). One table drives both the
# cfg -> widgets restore (_load_params_from_cfg) and the CHANGED-ONLY widgets ->
# cfg write-back (_apply_params): a value the user never touched must not be
# replaced by its range-clamped / decimals-quantized widget rendering.
BASIC_FIELDS = (
    ("f_lh", "process.layer_height_mm"),
    ("f_bw", "process.bead_width_mm"),
    ("f_ov", "process.raster_overlap"),
    ("f_ml", "process.min_deposit_len_mm"),
    ("f_amin", "c_axis.a_min_deg"),
    ("f_amax", "c_axis.a_max_deg"),
)


def _cfg_get(cfg, path: str):
    obj = cfg
    for part in path.split("."):
        obj = getattr(obj, part)
    return obj


def _cfg_set(cfg, path: str, value) -> None:
    parts = path.split(".")
    obj = cfg
    for part in parts[:-1]:
        obj = getattr(obj, part)
    setattr(obj, parts[-1], value)


def _build_studio_window():
    """Construct (without showing) the studio window. Split out so a headless smoke
    test can build it under an offscreen Qt (same pattern as ``gui.app``)."""
    from PySide6 import QtCore, QtWidgets
    from pyvistaqt import QtInteractor

    from ..gui.model import preview_from_model
    from ..presets import PresetBundle
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
            # PresetBundle port (rotoforge_slicer.presets): the machine /
            # material / process layering composes the working config; the
            # composed cfg stays THE live object every existing path reads.
            self.bundle = PresetBundle()
            self.cfg = self.bundle.full_config()
            self._param_baseline = {}     # cfg truth per widget-backed key
            self.scene = SceneModel()
            self.selected = None
            self.preview = None
            self.timeline = []
            self._layer_moves = {}
            self._move_index = []
            self.sim_t = 0.0
            self.playing = False
            self.csv_path = None
            self._csv_provenance = None   # original source of a temp-extracted CSV
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

            # ---- left: project + presets + parts + transform + process ----
            left = QtWidgets.QWidget()
            lyt = QtWidgets.QVBoxLayout(left)

            prj_row = QtWidgets.QHBoxLayout()
            self.btn_open_project = QtWidgets.QPushButton("Open project…")
            self.btn_open_project.clicked.connect(self._open_project)
            self.btn_save_project = QtWidgets.QPushButton("Save project…")
            self.btn_save_project.clicked.connect(self._save_project)
            prj_row.addWidget(self.btn_open_project)
            prj_row.addWidget(self.btn_save_project)
            lyt.addLayout(prj_row)

            # PrusaSlicer-style preset selectors. Canonical names ride in
            # itemData (item TEXT may carry a "(modified)" suffix); only the
            # user-gesture `activated` signal is connected, so programmatic
            # refreshes can never loop through the handler.
            prsbox = QtWidgets.QGroupBox("Presets")
            prsform = QtWidgets.QFormLayout(prsbox)
            self.preset_combos = {}
            for ptype, plabel in (("machine", "Machine"),
                                  ("material", "Material"),
                                  ("process", "Process")):
                prow_ = QtWidgets.QHBoxLayout()
                combo = QtWidgets.QComboBox()
                combo.activated.connect(
                    lambda idx, t=ptype: self._on_preset_activated(t, idx))
                b_save = QtWidgets.QPushButton("Save…")
                b_save.clicked.connect(lambda _=False, t=ptype: self._save_preset(t))
                b_del = QtWidgets.QPushButton("Del")
                b_del.clicked.connect(lambda _=False, t=ptype: self._delete_preset(t))
                prow_.addWidget(combo, stretch=1)
                prow_.addWidget(b_save)
                prow_.addWidget(b_del)
                self.preset_combos[ptype] = combo
                prsform.addRow(plabel, prow_)
            lyt.addWidget(prsbox)

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
            arow = QtWidgets.QHBoxLayout()          # PrusaSlicer-style auto-arrange
            self.btn_arrange = QtWidgets.QPushButton("Arrange")
            self.btn_arrange.clicked.connect(self._arrange)
            self.arr_spacing = self._dspin(30.0, 5.0, 120.0, 5.0, 0)
            self.arr_spacing.setToolTip(
                "part spacing (mm) — default clears the 50 mm wheel body")
            arow.addWidget(self.btn_arrange)
            arow.addWidget(QtWidgets.QLabel("spacing"))
            arow.addWidget(self.arr_spacing)
            pl.addLayout(arow)
            lyt.addWidget(pbox)

            tbox = QtWidgets.QGroupBox("Transform (selected part)")
            tv = QtWidgets.QVBoxLayout(tbox)
            tf = QtWidgets.QFormLayout()
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
            tv.addLayout(tf)
            # quick reorientation: world-frame 90° turns, lay-flat, reset (M11 QoL)
            qrow = QtWidgets.QHBoxLayout()
            for text, slot in (("X+90", lambda: self._rotate_world("x")),
                               ("Y+90", lambda: self._rotate_world("y")),
                               ("Z+90", lambda: self._rotate_world("z")),
                               ("Lay flat", self._lay_flat),
                               ("Reset", self._reset_transform)):
                b = QtWidgets.QPushButton(text)
                b.clicked.connect(slot)
                qrow.addWidget(b)
            tv.addLayout(qrow)
            self.dims_lbl = QtWidgets.QLabel("—")
            tv.addWidget(self.dims_lbl)
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
            self.f_mode.addItems(["raster", "streamline", "contour", "outline"])
            self.f_mode.setCurrentText(self.cfg.fill.mode)
            self.f_loops = QtWidgets.QSpinBox()
            self.f_loops.setRange(0, 20)
            self.f_loops.setValue(self.cfg.fill.perimeter_loops)
            self.f_cross = QtWidgets.QCheckBox("crosshatch (alternate heading per layer)")
            self.f_cross.setChecked(self.cfg.fill.crosshatch)
            form.addRow("Layer height (mm)", self.f_lh)
            form.addRow("Bead width (mm)", self.f_bw)
            form.addRow("Raster overlap", self.f_ov)
            form.addRow("Min deposit len (mm)", self.f_ml)
            form.addRow("C-axis A min (deg)", self.f_amin)
            form.addRow("C-axis A max (deg)", self.f_amax)
            form.addRow("Fill mode", self.f_mode)
            form.addRow("Perimeter loops (M17)", self.f_loops)
            form.addRow(self.f_cross)
            box = QtWidgets.QGroupBox("Process")
            box.setLayout(form)
            lyt.addWidget(box)

            # collapsible advanced parameters (table-driven; applied on Slice)
            self.adv_box = QtWidgets.QGroupBox("Advanced parameters")
            self.adv_box.setCheckable(True)
            self.adv_box.setChecked(False)
            adv_lyt = QtWidgets.QVBoxLayout(self.adv_box)
            adv_inner = QtWidgets.QWidget()
            adv_form = QtWidgets.QFormLayout(adv_inner)
            adv_form.setContentsMargins(0, 0, 0, 0)
            self.adv_widgets = {}
            for label, path, lo, hi, step, dec in ADVANCED_FIELDS:
                w = self._dspin(float(_cfg_get(self.cfg, path)), lo, hi, step, dec)
                self.adv_widgets[path] = w
                adv_form.addRow(label, w)
            self.adv_dry = QtWidgets.QCheckBox("dry run (no spindle/heat/E)")
            self.adv_dry.setChecked(self.cfg.emit.dry_run)
            adv_form.addRow(self.adv_dry)
            adv_lyt.addWidget(adv_inner)
            adv_inner.setVisible(False)
            self.adv_box.toggled.connect(adv_inner.setVisible)
            lyt.addWidget(self.adv_box)

            btn_screener = QtWidgets.QPushButton("Process window / material…")
            btn_screener.clicked.connect(self._open_screener)
            lyt.addWidget(btn_screener)
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
            scroll = QtWidgets.QScrollArea()
            scroll.setWidget(left)
            scroll.setWidgetResizable(True)
            scroll.setMinimumWidth(300)           # user-resizable via the splitter
            scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
            split.addWidget(scroll)

            # ---- right: viewport (+layer range slider) / tabs / log, all in a
            # vertical splitter so nothing is ever cut off — drag the boundaries.
            from .widgets import make_layer_range_slider

            right = QtWidgets.QSplitter(QtCore.Qt.Vertical)

            view_w = QtWidgets.QWidget()
            vwl = QtWidgets.QVBoxLayout(view_w)
            vwl.setContentsMargins(0, 0, 0, 0)
            vrow = QtWidgets.QHBoxLayout()          # camera presets (M11 QoL)
            for text, slot in (("Top", lambda: self._view_preset("xy")),
                               ("Front", lambda: self._view_preset("xz")),
                               ("Right", lambda: self._view_preset("yz")),
                               ("Iso", lambda: self._view_preset("iso")),
                               ("Fit", lambda: self._view_preset(None))):
                b = QtWidgets.QPushButton(text)
                b.setMaximumWidth(52)
                b.clicked.connect(slot)
                vrow.addWidget(b)
            vrow.addStretch(1)
            vwl.addLayout(vrow)

            hview = QtWidgets.QHBoxLayout()
            self.interactor = QtInteractor(view_w)
            self.view = BuildPlateScene(plotter=self.interactor)
            self.view.draw_plate(self.cfg)
            # Double-click only: single left-press also starts a camera-orbit drag,
            # which must not select or teleport parts.
            self.interactor.track_click_position(self._on_view_click, side="left",
                                                 double=True)
            # direct drag-to-move (press a part, drag it, release); orbit still owns
            # drags that start on empty plate
            self._drag = None
            self._install_drag_handlers()
            hview.addWidget(self.interactor, stretch=1)
            # PrusaSlicer-style vertical dual-handle layer range slider
            self.layer_range = make_layer_range_slider(view_w)
            self.layer_range.rangeChanged.connect(self._on_layer_range)
            self.layer_range.setVisible(False)
            hview.addWidget(self.layer_range)
            vwl.addLayout(hview, stretch=1)

            # PrusaSlicer-style horizontal move slider: scrubs the moves of the TOP
            # visible layer, progressively revealing its toolpath.
            self.move_row = QtWidgets.QWidget()
            mrow = QtWidgets.QHBoxLayout(self.move_row)
            mrow.setContentsMargins(0, 0, 0, 0)
            mrow.addWidget(QtWidgets.QLabel("Moves"))
            self.move_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            self.move_slider.valueChanged.connect(self._refresh_toolpath)
            self.move_lbl = QtWidgets.QLabel("—")
            mrow.addWidget(self.move_slider, stretch=1)
            mrow.addWidget(self.move_lbl)
            self.move_row.setVisible(False)
            vwl.addWidget(self.move_row)
            right.addWidget(view_w)

            self.tabs = QtWidgets.QTabWidget()
            # connected AFTER both tabs exist (below): addTab fires
            # currentChanged, and _on_mode_changed touches widgets (btn_play)
            # built later in this method

            prep = QtWidgets.QWidget()
            pl2 = QtWidgets.QVBoxLayout(prep)
            hint = QtWidgets.QLabel(
                "Drag a part to move it (drags starting on empty plate orbit the "
                "camera); double-click also selects / moves. X/Y/Z+90 rotate about "
                "world axes; Lay flat drops the largest face to the bed. Parts "
                "always rest on the bed; fit and overlap problems appear on the left.")
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
                # deposition-focused by default (PrusaSlicer semantics): the
                # auxiliary move classes clutter the view — flip them on as needed
                cb.setChecked(name == "deposition")
                cb.stateChanged.connect(self._refresh_toolpath)
                self.toggles[name] = cb
                trow.addWidget(cb)
            self.shells_cb = QtWidgets.QCheckBox("model shells")
            self.shells_cb.setChecked(False)      # Preview shows PATHS, not the mesh
            self.shells_cb.stateChanged.connect(lambda *_: self._on_mode_changed(
                self.tabs.currentIndex()))
            trow.addWidget(self.shells_cb)
            trow.addStretch(1)
            self.layer_lbl = QtWidgets.QLabel("—")
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
            self.readout.setWordWrap(True)
            vl.addWidget(self.readout)
            self.tabs.addTab(prev, "Preview")
            self.tabs.currentChanged.connect(self._on_mode_changed)
            right.addWidget(self.tabs)

            bottom = QtWidgets.QWidget()
            blyt = QtWidgets.QVBoxLayout(bottom)
            blyt.setContentsMargins(0, 0, 0, 0)
            self.progress = QtWidgets.QProgressBar()
            self.progress.setRange(0, 100)
            blyt.addWidget(self.progress)
            self.log = QtWidgets.QPlainTextEdit()
            self.log.setReadOnly(True)
            blyt.addWidget(self.log)
            right.addWidget(bottom)

            right.setStretchFactor(0, 1)          # the viewport gets spare space
            right.setSizes([560, 170, 130])       # initial; every boundary draggable
            split.addWidget(right)
            split.setStretchFactor(1, 1)
            split.setSizes([360, 1040])

            self.timer = QtCore.QTimer(self)
            self.timer.setInterval(_TICK_MS)
            self.timer.timeout.connect(self._on_tick)

            # widgets were built from cfg above; record the true cfg values as
            # the write-back baselines and populate the preset selectors
            self._load_params_from_cfg()
            self._restore_csv_from_cfg()  # a remembered material's CSV survives
            self._refresh_preset_combos()  # restarts (single-speed otherwise)
            for w in self.bundle.warnings():
                self._log(f"presets: {w}")
            self._warn_safety_flags()

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
            part.source_path = str(fn)          # provenance for project files
            self.selected = part
            self._refresh_part_list()
            self._load_transform_form()
            self._sync_scene()
            self.view.reset_camera()
            self._log(f"added {part.name}")

        def _arrange(self):
            if not self.scene.parts:
                return
            unplaced = self.scene.arrange(self.cfg, self.arr_spacing.value())
            self._load_transform_form()
            self._sync_scene()
            self.view.reset_camera()
            if unplaced:
                self._log(f"arrange: did not fit at {self.arr_spacing.value():g} mm "
                          f"spacing: {', '.join(unplaced)}")
            else:
                self._log(f"arranged {len(self.scene.parts)} part(s) at "
                          f"{self.arr_spacing.value():g} mm spacing")

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

        # ---- QoL: reorientation, view presets, drag-to-move (M11) ------------

        def _rotate_world(self, axis):
            if self.selected:
                self.selected.rotate_world(axis, 90.0)
                self._load_transform_form()
                self._sync_scene()

        def _lay_flat(self):
            if self.selected:
                try:
                    self.selected.lay_flat()
                except Exception as e:
                    self._log(f"lay flat failed: {e}")
                self._load_transform_form()
                self._sync_scene()

        def _reset_transform(self):
            if self.selected:
                self.selected.set_transform(rot_x_deg=0, rot_y_deg=0, rot_z_deg=0,
                                            scale=1.0)
                self._load_transform_form()
                self._sync_scene()

        def _view_preset(self, name):
            if name:
                self.interactor.camera_position = name
            self.view.reset_camera()
            self.interactor.update()

        def _install_drag_handlers(self):
            """Direct manipulation: press ON a part grabs it, dragging slides it on
            the plate (cheap actor offset), release commits the transform. While a
            part is grabbed the camera style is DISABLED (vtkInteractorObserver
            EnabledOff — our observers run at priority 10, before the style at 0, so
            the style never sees the grabbing press); a press on empty plate leaves
            the style alone and orbits normally. Picking is a fresh z-buffer
            vtkWorldPointPicker at the CURRENT event position on every event —
            pyvista's cached ``pick_mouse_position``/point-picker path returns stale
            or vertex-snapped points and must not be used here. Best-effort: any
            plumbing failure leaves the double-click select/move flow working."""
            try:
                iren = self.interactor.iren.interactor   # raw vtkRenderWindowInteractor
                iren.AddObserver("LeftButtonPressEvent", self._vtk_press, 10.0)
                iren.AddObserver("MouseMoveEvent", self._vtk_move, 10.0)
                iren.AddObserver("LeftButtonReleaseEvent", self._vtk_release, 10.0)
            except Exception as e:                        # pragma: no cover
                self._log(f"drag-to-move unavailable ({e}); double-click still works")

        def _world_pick_xy(self, iren):
            """World XY at the interactor's CURRENT event position via a fresh
            z-buffer world-point pick; None when the pick leaves the build volume
            (sky hits land on the far clipping plane, which the guard rejects)."""
            try:
                import vtk

                x, y = iren.GetEventPosition()
                picker = vtk.vtkWorldPointPicker()
                picker.Pick(x, y, 0, self.interactor.renderer)
                p = picker.GetPickPosition()
            except Exception:
                return None
            bx, by, bz = self.cfg.machine.build_volume_mm
            px, py, pz = float(p[0]), float(p[1]), float(p[2])
            if 0 <= px <= bx and 0 <= py <= by and -1.0 <= pz <= bz:
                return px, py
            return None

        def _camera_style_enabled(self, enabled: bool):
            try:
                style = self.interactor.iren.interactor.GetInteractorStyle()
                (style.EnabledOn if enabled else style.EnabledOff)()
            except Exception:                             # pragma: no cover
                pass

        def _vtk_press(self, obj, event):
            if self.tabs.currentIndex() != 0:
                return
            pick = self._world_pick_xy(obj)
            if pick is None:
                return                                    # let the camera have it
            for part in self.scene.parts:
                x0, y0, x1, y1 = part.footprint()
                if x0 <= pick[0] <= x1 and y0 <= pick[1] <= y1:
                    self.selected = part
                    self._refresh_part_list()
                    self._load_transform_form()
                    self._sync_scene()
                    self._drag = {"part": part, "ox": part.x - pick[0],
                                  "oy": part.y - pick[1], "x0": part.x, "y0": part.y,
                                  "tx": part.x, "ty": part.y}
                    self._camera_style_enabled(False)     # camera must not orbit
                    return

        def _vtk_move(self, obj, event):
            if not self._drag:
                return
            pick = self._world_pick_xy(obj)
            if pick is not None:
                d = self._drag
                d["tx"], d["ty"] = pick[0] + d["ox"], pick[1] + d["oy"]
                actor = self.view._part_actors.get(id(d["part"]))
                if actor is not None:                     # cheap live preview
                    actor.SetPosition(d["tx"] - d["x0"], d["ty"] - d["y0"], 0.0)
                    self.interactor.update()

        def _vtk_release(self, obj, event):
            if not self._drag:
                return
            d, self._drag = self._drag, None
            self._camera_style_enabled(True)
            d["part"].set_transform(x=d["tx"], y=d["ty"])  # commit + re-drop
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
            if p:
                sx, sy, sz = p.size_mm()
                self.dims_lbl.setText(f"{p.name}: {sx:.1f} × {sy:.1f} × {sz:.1f} mm")
            else:
                self.dims_lbl.setText("—")

        def _on_transform_edit(self, *_):
            if not self.selected:
                return
            # write back ONLY fields the user actually changed: the 0.1-precision
            # spinboxes would otherwise quantize a precise lay-flat / world-turn
            # orientation the first time any unrelated field is nudged.
            p = self.selected
            fields = {"x": self.t_x, "y": self.t_y, "rot_x_deg": self.t_rx,
                      "rot_y_deg": self.t_ry, "rot_z_deg": self.t_rz,
                      "scale": self.t_s}
            changed = {k: w.value() for k, w in fields.items()
                       if abs(w.value() - getattr(p, k)) > 0.5 * 10 ** -w.decimals()}
            if changed:
                p.set_transform(**changed)
                self._sync_scene()

        def _on_view_click(self, point):
            """Double-click a part -> select it; double-click the plate -> move the
            selection there. ``point`` from pyvista's tracker comes off its default
            POINT picker (snaps to dataset vertices — the plate mesh is coarse), so
            re-pick with the fresh z-buffer world picker at the event position."""
            if self.tabs.currentIndex() != 0:
                return
            pick = self._world_pick_xy(self.interactor.iren.interactor)
            if pick is None:
                return
            x, y = pick
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
                self._csv_provenance = fn
                # the config key rides along so material presets / project
                # files round-trip the CSV (the pipeline still gets the
                # explicit csv argument)
                self.cfg.screener.csv_path = fn
                self.csv_lbl.setText(Path(fn).name)
                self.bundle.capture(self.cfg)     # direct cfg write
                self._refresh_preset_combos()

        def _apply_params(self):
            """Widgets -> cfg, CHANGED-ONLY (the _on_transform_edit rationale):
            spinboxes clamp and quantize, so an untouched widget must never
            overwrite a config value it cannot represent (e.g. a hand-edited
            preset's out-of-range feed)."""
            def maybe(w, path):
                val = w.value()
                base = self._param_baseline.get(path)
                if base is not None and abs(val - base) <= 0.5 * 10 ** -w.decimals():
                    return
                cur = _cfg_get(self.cfg, path)
                _cfg_set(self.cfg, path, int(val) if w.decimals() == 0
                         and isinstance(cur, int) else val)
                self._param_baseline[path] = val

            for attr, path in BASIC_FIELDS:
                maybe(getattr(self, attr), path)
            for path, w in self.adv_widgets.items():
                maybe(w, path)
            if self.f_loops.value() != self._param_baseline.get("fill.perimeter_loops"):
                self.cfg.fill.perimeter_loops = self.f_loops.value()
                self._param_baseline["fill.perimeter_loops"] = self.f_loops.value()
            # the mode combo is changed-only too: a config mode outside the
            # combo's vocabulary (newer version) must not be silently replaced
            # by whatever the combo happens to display
            if self.f_mode.currentText() != self._param_baseline.get("fill.mode"):
                self.cfg.fill.mode = self.f_mode.currentText()
                self._param_baseline["fill.mode"] = self.f_mode.currentText()
            # boolean widgets are lossless — write through
            self.cfg.fill.crosshatch = self.f_cross.isChecked()
            self.cfg.emit.dry_run = self.adv_dry.isChecked()

        def _load_params_from_cfg(self):
            """Widgets <- cfg (the inverse of _apply_params), recording each
            TRUE cfg value as the write-back baseline. A widget that clamps or
            quantizes the incoming value is reported — the display is then an
            approximation, but the config value survives (changed-only rule)."""
            clamped = []

            def setv(w, path):
                val = float(_cfg_get(self.cfg, path))
                w.blockSignals(True)
                w.setValue(val)
                w.blockSignals(False)
                # baseline = the WIDGET's rendering of the loaded value: an
                # untouched widget then always compares equal to its baseline,
                # even when it had to clamp/quantize the true config value
                self._param_baseline[path] = w.value()
                if abs(w.value() - val) > 1e-9:
                    clamped.append(f"{path}={val:g} (shown {w.value():g})")

            for attr, path in BASIC_FIELDS:
                setv(getattr(self, attr), path)
            for path, w in self.adv_widgets.items():
                setv(w, path)
            self.f_loops.blockSignals(True)
            self.f_loops.setValue(self.cfg.fill.perimeter_loops)
            self.f_loops.blockSignals(False)
            self._param_baseline["fill.perimeter_loops"] = self.f_loops.value()
            self.f_mode.blockSignals(True)
            self.f_mode.setCurrentText(self.cfg.fill.mode)
            self.f_mode.blockSignals(False)
            self._param_baseline["fill.mode"] = self.f_mode.currentText()
            if self.f_mode.currentText() != self.cfg.fill.mode:
                clamped.append(f"fill.mode={self.cfg.fill.mode!r} not in the "
                               "mode selector")
            self.f_cross.setChecked(self.cfg.fill.crosshatch)
            self.adv_dry.setChecked(self.cfg.emit.dry_run)
            if clamped:
                self._log("WARNING: widget limits clamp config values (the "
                          "config keeps the true values): " + "; ".join(clamped))

        # ---- presets (PresetBundle port; see rotoforge_slicer.presets) --------

        def _sync_bundle(self):
            """THE bundle-read invariant: widgets -> cfg, then cfg -> the edited
            overlays. Must precede every bundle read (combo activate, preset
            Save/Delete, Slice, project save) — the studio's widgets write to a
            live Config, not through the presets like PrusaSlicer's tabs do."""
            self._apply_params()
            self.bundle.capture(self.cfg)

        def _refresh_preset_combos(self):
            for ptype, combo in self.preset_combos.items():
                col = self.bundle.collections[ptype]
                combo.blockSignals(True)
                combo.clear()
                for name in col.names():
                    combo.addItem(name, name)
                idx = combo.findData(col.selected)
                if idx >= 0:
                    if col.is_dirty():
                        combo.setItemText(idx, f"{col.selected} (modified)")
                    combo.setCurrentIndex(idx)
                combo.blockSignals(False)

        def _on_preset_activated(self, ptype, index):
            name = self.preset_combos[ptype].itemData(index)
            if name is None:
                return
            self._sync_bundle()               # other types keep their dirty state
            actual = self.bundle.collections[ptype].select(name)
            self.cfg = self.bundle.full_config()
            self.bundle.save_selections()
            self._load_params_from_cfg()
            self._restore_csv_from_cfg()
            if ptype == "machine":
                self._refresh_machine_ui()
            self._refresh_preset_combos()
            self._log(f"{ptype} preset: {actual}")
            self._warn_safety_flags()

        def _save_preset(self, ptype):
            self._sync_bundle()
            col = self.bundle.collections[ptype]
            suggestion = ("" if col.presets[col.selected].is_default
                          else col.selected)
            name, ok = QtWidgets.QInputDialog.getText(
                self, f"Save {ptype} preset", "Preset name:", text=suggestion)
            if not ok or not name.strip():
                return
            try:
                col.save_current(name)
            except (ValueError, OSError) as e:
                self._log(f"ERROR saving {ptype} preset: {e}")
                return
            self.bundle.save_selections()
            self._refresh_preset_combos()
            self._log(f"saved {ptype} preset {col.selected!r} -> "
                      f"{col.dir_path / (col.selected + '.yaml')}")

        def _delete_preset(self, ptype):
            col = self.bundle.collections[ptype]
            name = col.selected
            if col.presets[name].is_default:
                self._log("the default preset cannot be deleted")
                return
            resp = QtWidgets.QMessageBox.question(
                self, "Delete preset", f"Delete {ptype} preset {name!r}?")
            if resp != QtWidgets.QMessageBox.Yes:
                return
            col.delete(name)                  # selection falls back to default
            self.cfg = self.bundle.full_config()
            self.bundle.save_selections()
            self._load_params_from_cfg()
            self._restore_csv_from_cfg()
            if ptype == "machine":
                self._refresh_machine_ui()
            self._refresh_preset_combos()
            self._log(f"deleted {ptype} preset {name!r}")
            self._warn_safety_flags()

        def _restore_csv_from_cfg(self):
            """win.csv_path <- cfg.screener.csv_path after a preset changed the
            config underneath the UI. A temp-extracted embedded CSV stays bound
            as long as the config still names its recorded source — otherwise a
            preset switch after a project load would drop the working copy for
            a path that only exists on the machine the project came from."""
            csvp = self.cfg.screener.csv_path
            if (csvp and csvp == self._csv_provenance
                    and self.csv_path and Path(self.csv_path).exists()):
                return                                    # embedded copy stays
            if csvp and Path(csvp).exists():
                self.csv_path = csvp
                self._csv_provenance = csvp
                self.csv_lbl.setText(Path(csvp).name)
            elif csvp:
                self.csv_path = None
                self.csv_lbl.setText(f"CSV MISSING: {csvp}")
                self._log(f"WARNING: screener CSV not found: {csvp}")
            else:
                self.csv_path = None
                self.csv_lbl.setText("no CSV (single-speed fallback)")

        def _refresh_machine_ui(self):
            """Build-volume-derived UI (plate actor, transform ranges) after a
            machine preset / opened project changed cfg.machine."""
            bx, by, _ = self.cfg.machine.build_volume_mm
            for w, hi in ((self.t_x, bx), (self.t_y, by)):
                w.blockSignals(True)          # setRange clamps -> valueChanged
                # never clamp below an existing part position: the clamped
                # widget would read as "user-changed" on the next unrelated
                # edit and teleport the part (issues() flags oversize plates)
                w.setRange(0, max(hi, w.value()))
                w.blockSignals(False)
            self._load_transform_form()       # re-sync widgets to the part
            self.view.draw_plate(self.cfg)
            self._sync_scene()

        def _warn_safety_flags(self):
            """Presets/projects can restore flags with no prominent UI; say so
            loudly — a silently disabled collision check reads as 'no
            collisions' in the slice summary (SPEC §4.6)."""
            if not self.cfg.collision.enabled:
                self._log("WARNING: collision checking is DISABLED "
                          "(collision.enabled = false in this config)")
            if self.cfg.emit.dry_run:
                self._log("NOTE: dry run is ON (no spindle / heaters / E)")

        def _open_screener(self):
            from .screener_panel import open_screener_dialog

            open_screener_dialog(self)        # modal; Apply writes cfg directly
            self.bundle.capture(self.cfg)
            self._refresh_preset_combos()     # material may now be (modified)

        def _slice(self):
            if not self.scene.parts:
                self._log("Add a mesh first.")
                return
            issues = self.scene.issues(self.cfg)
            if issues:
                self._log("Placement issues:\n  " + "\n  ".join(issues))
            self._sync_bundle()
            self._refresh_preset_combos()     # edits may show as (modified)
            self._set_playing(False)
            self.btn_slice.setEnabled(False)
            self.btn_save.setEnabled(False)
            self.progress.setValue(0)
            import copy

            self._thread = QtCore.QThread()
            # snapshot BOTH the scene and the config: the worker slices frozen
            # copies, so live edits (transform panel, process-window dialog Apply)
            # cannot leak into an in-flight slice's plan or preamble.
            self._worker = SliceWorker(self.scene.snapshot(),
                                       copy.deepcopy(self.cfg), self.csv_path)
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
            # the preview carries the frozen cfg its plan was built with — time the
            # simulation against THAT, not the possibly-edited live cfg
            self.timeline = build_timeline(preview.segments, preview.plan, preview.cfg)
            self.sim_t = 0.0
            # per-layer move indices for the PrusaSlicer-style sliders: each segment
            # gets its ordinal within its layer (segments are in machine order)
            self._layer_moves = {}
            self._move_index = []
            for s in preview.segments:
                k = s.layer_index if s.layer_index is not None else -1
                self._move_index.append(self._layer_moves.get(k, 0))
                self._layer_moves[k] = self._layer_moves.get(k, 0) + 1
            max_layer = max((k for k in self._layer_moves if k >= 0), default=0)
            self.layer_range.setMaximum(max_layer)
            self.layer_range.setRange_(0, max_layer, emit=False)
            self._sync_move_slider()
            self.time_slider.setEnabled(bool(self.timeline))
            self.btn_play.setEnabled(bool(self.timeline))
            if self.tabs.currentIndex() == 1:
                self._on_mode_changed(1)          # already in Preview: refresh now
            else:
                self.tabs.setCurrentIndex(1)      # fires _on_mode_changed
            self._update_sim_display()

        # ---- preview / simulation --------------------------------------------------

        def _on_mode_changed(self, index):
            """Prepare shows the MODEL; Preview shows the TOOLPATH (PrusaSlicer
            semantics — the mesh must never occlude the paths; 'model shells'
            optionally ghosts it back in)."""
            preview_mode = index == 1 and self.preview is not None
            self.layer_range.setVisible(preview_mode)
            self.move_row.setVisible(preview_mode)
            if self._drag:                        # abandon a mid-gesture drag: the
                d, self._drag = self._drag, None  # grabbed part may be hidden now
                actor = self.view._part_actors.get(id(d["part"]))
                if actor is not None:
                    actor.SetPosition(0.0, 0.0, 0.0)
                self._camera_style_enabled(True)
            if preview_mode:
                self.view.set_parts_display(
                    "ghost" if self.shells_cb.isChecked() else "hidden")
                self._refresh_toolpath()
            else:
                self._set_playing(False)          # the timer would resurrect the
                self.view.set_parts_display("normal")  # head actors it just cleared
                self.view.clear_toolpath()
                self.view.clear_head()
            self.interactor.update()

        def _sync_move_slider(self):
            """Point the move slider at the TOP visible layer's move count."""
            hi = self.layer_range.high()
            n = self._layer_moves.get(hi, 0)
            self.move_slider.blockSignals(True)
            self.move_slider.setRange(0, max(0, n))
            self.move_slider.setValue(n)
            self.move_slider.blockSignals(False)

        def _on_layer_range(self, lo, hi):
            self._sync_move_slider()
            self._refresh_toolpath()

        def _refresh_toolpath(self, *_):
            if not self.preview or self.tabs.currentIndex() != 1:
                return
            lo, hi = self.layer_range.low(), self.layer_range.high()
            upto_move = self.move_slider.value()
            shown = []
            for s, mi in zip(self.preview.segments, self._move_index):
                k = s.layer_index if s.layer_index is not None else -1
                if k < lo or k > hi:
                    continue
                if k == hi and mi >= upto_move:
                    continue                      # top layer revealed move-by-move
                shown.append(s)
            enabled = {n for n, cb in self.toggles.items() if cb.isChecked()}
            self.view.show_toolpath(shown, enabled=enabled)
            if self.preview.collisions:
                self.view.show_collisions(self.preview.collisions)
            self.layer_lbl.setText(
                f"layers {lo}–{hi}   move {upto_move}/{self._layer_moves.get(hi, 0)}")
            self.move_lbl.setText(f"{upto_move}/{self._layer_moves.get(hi, 0)}")
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
            if not self.timeline or self.tabs.currentIndex() != 1:
                return                            # the head belongs to Preview only
            cfg = self.preview.cfg if self.preview else self.cfg
            state = state_at(self.timeline, self.sim_t, cfg.c_axis)
            self.view.update_head(state, cfg)
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

        # ---- project save/load (3MF-architecture port; studio/project.py) -----

        def _save_project(self):
            self._sync_bundle()               # widgets + cfg -> overlays first
            fn, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, "Save project", "studio.rfproj",
                "Rotoforge project (*.rfproj)")
            if not fn:
                return
            from .project import save_project

            try:
                save_project(fn, self.scene, self.cfg, csv_path=self.csv_path,
                             selections=self.bundle.selections(),
                             ui={"arrange_spacing_mm": self.arr_spacing.value()})
            except Exception as e:
                self._log(f"ERROR saving project: {e}")
                return
            self._refresh_preset_combos()
            self._log(f"Saved project {fn}")

        def _open_project(self):
            fn, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, "Open project", "",
                "Rotoforge project (*.rfproj);;All files (*)")
            if fn:
                self.open_project_file(fn)

        def open_project_file(self, fn):
            """Restore a saved project: geometry -> config -> preset
            reconciliation -> UI (the 3mf load order). Public — main() feeds
            CLI project paths here."""
            if self._thread:
                self._log("Cannot open a project while a slice is running.")
                return
            from ..presets import apply_flat, base_config
            from .project import load_project

            try:
                data = load_project(fn)
            except Exception as e:
                self._log(f"ERROR opening project {fn}: {e}")
                return

            # drop every artifact of the previous session: a stale preview /
            # running playback would pair the old toolpath with the new plate
            self._set_playing(False)
            self.preview = None
            self.timeline = []
            self.btn_save.setEnabled(False)
            self.btn_play.setEnabled(False)
            self.time_slider.setEnabled(False)
            self.layer_range.setVisible(False)
            self.move_row.setVisible(False)
            self._drag = None
            self.tabs.setCurrentIndex(0)

            for w in data.warnings:
                self._log(f"project: {w}")

            if data.config_flat:
                # config: defaults <- snapshot (missing keys keep the current
                # defaults; unknown/bad content substitutes and reports — a
                # project from another version must open, never abort)
                cfg = base_config()
                for line in apply_flat(cfg, data.config_flat, on_unknown="warn"):
                    self._log(f"project config: {line}")
                self.cfg = cfg
            else:
                self._log("model-only project: keeping the current settings "
                          "and presets")

            # scene swap — the old selection would be a ghost part
            self.scene = data.scene
            self.selected = self.scene.parts[-1] if self.scene.parts else None

            if data.config_flat:
                self._restore_project_csv(data, Path(fn).stem)
                n_before = {t: len(self.bundle.collections[t].warnings)
                            for t in ("machine", "material", "process")}
                outcomes = self.bundle.adopt_project(data.config_flat,
                                                     data.selections)
                self.bundle.save_selections()
                for t in ("machine", "material", "process"):
                    col = self.bundle.collections[t]
                    self._log(f"  {t} preset: {col.selected!r} ({outcomes[t]})")
                    for w in col.warnings[n_before[t]:]:
                        self._log(f"  {w}")
                self._warn_machine_drift(data.config_flat)

            if "arrange_spacing_mm" in data.ui:
                try:
                    self.arr_spacing.setValue(float(data.ui["arrange_spacing_mm"]))
                except (TypeError, ValueError):
                    pass

            self._load_params_from_cfg()
            self._refresh_machine_ui()
            self._refresh_preset_combos()
            self._refresh_part_list()
            self._load_transform_form()
            self._sync_scene()
            self.view.reset_camera()
            self._warn_safety_flags()
            self._log(f"Opened project {fn}: {len(self.scene.parts)} part(s)")

        def _warn_machine_drift(self, snapshot):
            """A project restores the FULL config — including this machine's
            calibration record (ω_C, steps, axis range). Adopting a stale
            calibration silently retargets the invariant proofs, so differences
            from the local base are called out (the Machine preset also shows
            (modified))."""
            from ..presets import (
                MACHINE_KEYS, RECONCILE_IGNORE_KEYS, flatten_config, values_equal,
            )

            base_flat = flatten_config(self.bundle.base)
            drift = [k for k in MACHINE_KEYS
                     if k in snapshot and k not in RECONCILE_IGNORE_KEYS
                     and not values_equal(snapshot[k], base_flat[k])]
            if drift:
                self._log("WARNING: this project's machine values differ from "
                          "the local calibration record (machine_duet3.yaml): "
                          + ", ".join(sorted(drift))
                          + " — verify before running on hardware; select the "
                          "machine default preset to use the local values")

        def _restore_project_csv(self, data, stem):
            """The EMBEDDED screener CSV is the ground truth the project was
            validated with — it always wins; the recorded source path stays in
            cfg as sticky provenance. A drifted source file is reported."""
            self.csv_path = None
            self._csv_provenance = None
            label = "no CSV (single-speed fallback)"
            src = data.csv_source_path or self.cfg.screener.csv_path
            if data.csv_bytes is not None:
                import hashlib
                import tempfile

                d = Path(tempfile.gettempdir()) / "rotoforge_projects"
                d.mkdir(parents=True, exist_ok=True)
                # content-addressed: same-stem projects / concurrent studio
                # instances must never overwrite each other's process window
                digest = hashlib.sha1(data.csv_bytes).hexdigest()[:12]
                tmp = d / f"{stem}_{digest}_screener.csv"
                tmp.write_bytes(data.csv_bytes)
                self.csv_path = str(tmp)
                self._csv_provenance = src or None
                label = f"{Path(src).name if src else tmp.name} (embedded)"
                if src and Path(src).exists() \
                        and Path(src).read_bytes() != data.csv_bytes:
                    self._log(f"WARNING: {src} has changed since this project "
                              "was saved; using the embedded copy")
            elif src and Path(src).exists():
                self.csv_path = src
                self._csv_provenance = src
                label = Path(src).name
            elif src:
                label = f"CSV MISSING: {src}"
                self._log("WARNING: screener CSV neither embedded nor found "
                          f"on disk: {src}")
            self.csv_lbl.setText(label)

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
        if not Path(a).exists():
            continue
        if a.lower().endswith(".rfproj"):
            win.open_project_file(a)
        elif a.lower().endswith((".stl", ".3mf", ".obj", ".ply")):
            win.add_mesh_file(a)
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
