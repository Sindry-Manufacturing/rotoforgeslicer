"""Custom Qt widgets for the studio. PySide6 imported lazily (CLAUDE.md).

``make_layer_range_slider`` builds the PrusaSlicer-style VERTICAL dual-handle layer
slider: the two handles bound the visible layer window [lo, hi]; dragging the body
between them slides the whole window. Emits ``rangeChanged(lo, hi)``.
"""
from __future__ import annotations


def make_layer_range_slider(parent=None):
    from PySide6 import QtCore, QtGui, QtWidgets

    class LayerRangeSlider(QtWidgets.QWidget):
        """Vertical two-handle range slider (values grow upward, like layers)."""

        rangeChanged = QtCore.Signal(int, int)

        HANDLE_H = 10          # px
        GROOVE_W = 6

        def __init__(self, parent=None):
            super().__init__(parent)
            self._min, self._max = 0, 0
            self._lo, self._hi = 0, 0
            self._drag = None            # "lo" | "hi" | ("window", grab_value)
            self.setMinimumWidth(26)
            self.setSizePolicy(QtWidgets.QSizePolicy.Fixed,
                               QtWidgets.QSizePolicy.Expanding)

        # ---- API ----------------------------------------------------------

        def setMaximum(self, m: int):
            self._max = max(0, int(m))
            self._lo = min(self._lo, self._max)
            self._hi = min(self._hi if self._hi else self._max, self._max)
            self.update()

        def setRange_(self, lo: int, hi: int, emit: bool = True):
            lo, hi = self.clamp(lo, hi, self._min, self._max)
            changed = (lo, hi) != (self._lo, self._hi)
            self._lo, self._hi = lo, hi
            self.update()
            if changed and emit:
                self.rangeChanged.emit(lo, hi)

        def low(self) -> int:
            return self._lo

        def high(self) -> int:
            return self._hi

        @staticmethod
        def clamp(lo: int, hi: int, mn: int, mx: int):
            """Pure range normalization (unit-testable): order, then clamp."""
            lo, hi = (int(min(lo, hi)), int(max(lo, hi)))
            return max(mn, min(lo, mx)), max(mn, min(hi, mx))

        # ---- geometry -------------------------------------------------------

        def _v2y(self, v: int) -> float:
            """Layer value -> widget y (top = max layer)."""
            h = self.height() - 2 * self.HANDLE_H
            if self._max <= self._min:
                return self.height() - self.HANDLE_H
            frac = (v - self._min) / (self._max - self._min)
            return self.height() - self.HANDLE_H - frac * h

        def _y2v(self, y: float) -> int:
            h = self.height() - 2 * self.HANDLE_H
            if h <= 0 or self._max <= self._min:
                return self._min
            frac = (self.height() - self.HANDLE_H - y) / h
            return round(self._min + max(0.0, min(1.0, frac)) * (self._max - self._min))

        # ---- painting -------------------------------------------------------

        def paintEvent(self, ev):
            p = QtGui.QPainter(self)
            p.setRenderHint(QtGui.QPainter.Antialiasing)
            cx = self.width() / 2
            groove = QtCore.QRectF(cx - self.GROOVE_W / 2, self.HANDLE_H,
                                   self.GROOVE_W, self.height() - 2 * self.HANDLE_H)
            p.fillRect(groove, QtGui.QColor("#3b4252"))
            y_lo, y_hi = self._v2y(self._lo), self._v2y(self._hi)
            p.fillRect(QtCore.QRectF(cx - self.GROOVE_W / 2, y_hi,
                                     self.GROOVE_W, y_lo - y_hi),
                       QtGui.QColor("#e0a000"))               # active window
            for y in (y_lo, y_hi):
                p.setBrush(QtGui.QColor("#e8862d"))
                p.setPen(QtGui.QPen(QtGui.QColor("#7a5700"), 1))
                p.drawRoundedRect(QtCore.QRectF(
                    2, y - self.HANDLE_H / 2, self.width() - 4, self.HANDLE_H), 3, 3)
            p.end()

        # ---- interaction -----------------------------------------------------

        def mousePressEvent(self, ev):
            y = ev.position().y()
            d_lo = abs(y - self._v2y(self._lo))
            d_hi = abs(y - self._v2y(self._hi))
            if min(d_lo, d_hi) > self.HANDLE_H * 1.5 \
                    and self._v2y(self._hi) < y < self._v2y(self._lo):
                self._drag = ("window", self._y2v(y))          # slide the window
            elif self._lo == self._hi:
                # coincident handles: pick by drag direction — pressing above the
                # handle grabs "hi" (can move up), below grabs "lo" (can move
                # down); always classifying as "lo" would deadlock at the bottom.
                self._drag = "hi" if y <= self._v2y(self._hi) else "lo"
            else:
                self._drag = "lo" if d_lo <= d_hi else "hi"
            self.mouseMoveEvent(ev)

        def mouseMoveEvent(self, ev):
            if self._drag is None:
                return
            v = self._y2v(ev.position().y())
            if self._drag == "lo":
                self.setRange_(min(v, self._hi), self._hi)
            elif self._drag == "hi":
                self.setRange_(self._lo, max(v, self._lo))
            else:
                _, grab = self._drag
                dv = v - grab
                span = self._hi - self._lo
                lo = max(self._min, min(self._lo + dv, self._max - span))
                self._drag = ("window", v)
                self.setRange_(lo, lo + span)

        def mouseReleaseEvent(self, ev):
            self._drag = None

    return LayerRangeSlider(parent)
