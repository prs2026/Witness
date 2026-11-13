"""Interactive viewer for CSV files produced by converter.py (PyQt6 + pyqtgraph).

Usage:
    python viewer.py S0000434.csv
    python viewer.py S0000434.csv P0000434.csv   # overlay on Witness uptime
    python viewer.py

With no filename, the viewer opens a file-selection dialog. Several CSVs (or
raw .BIN logs) can be open at once; drag traces from the list onto graphs to
overlay them. witnessOS embeds the same viewer as a tab (CsvViewerWidget).

The data side (loading, time axes, derivations, the memory guard) lives in
viewer_data.py. This module is only the user interface. What keeps big
files responsive:

* every trace is a numpy float64 array handed to pyqtgraph through
  qt_common.add_curve (clip-to-view + peak downsampling per pixel);
* a file's time offset moves its curves with ``curve.shift(offset)``
  (setPos), never rebuilding arrays; a new time column or unit rebuilds
  that file's x array once, with numpy;
* graphs are created lazily as they scroll into view (at most
  MAX_LIVE_PLOTS exist), and their x axes are linked so zooming one zooms
  all;
* loading, .BIN decoding, CSV export and FFTs run on worker threads
  (qt_common.run_in_thread) and report back through Qt signals.
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np
from PyQt6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

import csv_columns
import qt_common
import spectrum
import viewer_board as vb_file
import viewer_data as vd
from qt_common import add_curve, Debounce, ensure_app, run_in_thread, Throttle
# Compatibility: the data helpers were importable from viewer before the Qt
# port (Horizon main.py imports these four; tests and scripts use the rest).
from viewer_data import *  # noqa: F401,F403
from viewer_data import (  # noqa: F401
    classify_drop,
    AUTO_TIME, BINARY_COLUMNS, DROP_ZONE_HEIGHT, MIN_ZOOM_DRAG_PX, PAN_FRACTION,
    PLOT_COLORS, ROW_INDEX, SYNC_X, TEXT_READOUT_COLUMNS, TIME_UNITS, ZOOM_FACTOR,
    Dataset, GraphLayout, GraphRefs, LAUNCH_NOTE, MemoryBudgetError, Series, decimate_minmax,
    elide, graph_height_for, nice_ticks, tick_text, unique_label, value_text,
)


TRACES_MIME = "application/x-horizon-traces"
LEGEND_MIME = "application/x-horizon-legend"
AXIS_MIME = "application/x-horizon-axis"
PEN_STYLES = (QtCore.Qt.PenStyle.SolidLine, QtCore.Qt.PenStyle.DashLine,
              QtCore.Qt.PenStyle.DotLine, QtCore.Qt.PenStyle.DashDotLine,
              QtCore.Qt.PenStyle.DashDotDotLine)
AXIS_WIDTH = 64               # fixed left axis width: every graph's x lines up
MAX_LIVE_PLOTS = 16           # PlotWidgets kept alive; farther ones are freed. Every
                              # live plot re-clips on each zoom, so keep this small.
# Graphs redraw only once input stops: zoom/pan/scroll after SETTLE_MS of no
# wheel/drag/scroll events, the hover readout after HOVER_SETTLE_MS of a still mouse.
SETTLE_MS = 150
HOVER_SETTLE_MS = 80
MAX_COLUMNS = 4               # graphs side by side (Graphs per row)
MIN_RESIZE_HEIGHT = 90        # smallest graph a drag can make
MAX_RESIZE_HEIGHT = 2400
GRIP_HEIGHT = 7
DENSE_POINTS = 5000           # traces longer than this get a 1 px pen (2-3x faster)
LAZY_MARGIN = 0.5             # build graphs this many viewports above/below
REALIZE_BUDGET_S = 0.035      # GUI time spent creating graphs per event-loop turn
SERIES_CACHE_BYTES = 768 * 1024 ** 2
BUSY_SWITCH_INTERVAL_S = 0.0005
CSV_FILTER = "CSV / text logs (*.csv *.CSV *.txt *.tsv);;All files (*)"
BIN_FILTER = "Witness logs (*.bin *.BIN);;All files (*)"
DOCK_HEIGHT = 460             # a spectrogram docked in the board
COLORBAR_PAD = 95             # right-hand space a docked spectrogram's colour bar takes


def _refs_to_json(refs) -> bytes:
    return json.dumps([[int(i), str(c)] for i, c in refs]).encode("utf-8")


def _refs_from_json(data: bytes):
    return [(int(i), str(c)) for i, c in json.loads(bytes(data).decode("utf-8"))]


def _local_paths(mime: QtCore.QMimeData) -> list[Path]:
    if not mime.hasUrls():
        return []
    return [Path(url.toLocalFile()) for url in mime.urls() if url.isLocalFile()]


def _line_icon(color: str, style, width: int = 22, height: int = 12) -> QtGui.QIcon:
    pixmap = QtGui.QPixmap(width, height)
    pixmap.fill(QtCore.Qt.GlobalColor.transparent)
    painter = QtGui.QPainter(pixmap)
    pen = QtGui.QPen(QtGui.QColor(color), 2)
    pen.setStyle(style)
    painter.setPen(pen)
    painter.drawLine(1, height // 2, width - 1, height // 2)
    painter.end()
    return QtGui.QIcon(pixmap)


class ElidedLabel(QtWidgets.QLabel):
    """A one-line label that shortens its text with an ellipsis to fit."""

    def __init__(self, text: str = "", parent=None):
        super().__init__(parent)
        self._full = text
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored,
                           QtWidgets.QSizePolicy.Policy.Preferred)
        self.setText(text)

    def setText(self, text: str) -> None:  # noqa: N802 - Qt name
        self._full = text
        self.setToolTip(text)
        self._elide()

    def full_text(self) -> str:
        return self._full

    def resizeEvent(self, event) -> None:  # noqa: N802
        super().resizeEvent(event)
        self._elide()

    def _elide(self) -> None:
        width = max(10, self.width())
        shown = self.fontMetrics().elidedText(self._full, QtCore.Qt.TextElideMode.ElideRight,
                                              width)
        super().setText(shown)


class OffsetSpinBox(QtWidgets.QDoubleSpinBox):
    """Offset in seconds: shows values like 1.1 / -692 (no trailing zeros)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setRange(-1e9, 1e9)
        self.setDecimals(9)
        self.setSingleStep(0.1)
        self.setKeyboardTracking(True)
        self.setAccelerated(True)

    def textFromValue(self, value: float) -> str:  # noqa: N802
        return value_text(round(value, 9))


# ---------------------------------------------------------------------------
# Plot pieces
# ---------------------------------------------------------------------------

class ExactXViewBox(pg.ViewBox):
    """ViewBox that copies a linked view's x range exactly."""

    def linkedViewChanged(self, view, axis):  # noqa: N802
        # Every graph has the same width and axis width, so a linked x range
        # is copied exactly (pyqtgraph's default lines views up by screen
        # position, which is wrong for a plot built before it is laid out).
        if axis != pg.ViewBox.XAxis or self.linksBlocked or view is None:
            super().linkedViewChanged(view, axis)
            return
        low, high = view.viewRange()[0]
        view.blockLink(True)
        try:
            self.enableAutoRange(pg.ViewBox.XAxis, False)
            self.setXRange(low, high, padding=0)
        finally:
            view.blockLink(False)


class TimeViewBox(ExactXViewBox):
    """ViewBox with the viewer's mouse conventions.

    Drag across a plot: zoom x (every graph). Ctrl+wheel: zoom about the
    cursor. Shift+wheel: pan. Plain wheel scrolls the page. Shift/middle
    drag pans. Double-click resets the zoom. Right-click: menu. Zoom and
    pan are applied once the wheel/mouse stops (SETTLE_MS), not per event.
    """

    def __init__(self, owner: "CsvViewerWidget"):
        super().__init__(enableMenu=True)
        self.owner = owner
        self._pan_press_x = None
        self.setMouseEnabled(x=True, y=False)

    def _pan_shift(self) -> float:
        """How far the view has already moved since the drag began (a pause
        applies the pan mid-drag), so the drag keeps tracking the cursor."""
        pending = self.owner._drag_pan
        if pending is None or self.owner.x_view is None:
            return 0.0
        return self.owner.x_view[0] - pending[0][0]

    def mouseDragEvent(self, ev, axis=None):  # noqa: N802
        left = ev.button() == QtCore.Qt.MouseButton.LeftButton
        middle = ev.button() == QtCore.Qt.MouseButton.MiddleButton
        shift = bool(ev.modifiers() & QtCore.Qt.KeyboardModifier.ShiftModifier)
        if axis is None and (middle or (left and shift)):
            # Pan: applied when the mouse pauses or is released, not per move.
            ev.accept()
            press = self._pan_press_x if not ev.isStart() else None
            if press is None:
                press = self.mapToView(ev.buttonDownPos()).x()
                self._pan_press_x = press
            # Distance in the press-time view: map with the view as it was.
            here = self.mapToView(ev.pos()).x() - self._pan_shift()
            self.owner.drag_pan(press, here, ev.isFinish())
            if ev.isFinish():
                self._pan_press_x = None
            return
        if axis is not None or not left:
            super().mouseDragEvent(ev, axis)
            return
        ev.accept()
        down, here = ev.buttonDownPos(), ev.pos()
        if ev.isFinish():
            self.rbScaleBox.hide()
            if abs(here.x() - down.x()) >= MIN_ZOOM_DRAG_PX:
                x0 = self.mapToView(down).x()
                x1 = self.mapToView(here).x()
                self.owner.set_x_view((x0, x1))
            return
        height = self.boundingRect().height()
        self.updateScaleBox(QtCore.QPointF(down.x(), 0.0), QtCore.QPointF(here.x(), height))

    def wheelEvent(self, ev, axis=None):  # noqa: N802
        modifiers = ev.modifiers()
        delta = ev.delta()
        if not delta:
            ev.ignore()
            return
        # Queued, not applied: a fast wheel or a touchpad sends dozens of
        # events a second and each zoom redraws every graph.
        if modifiers & QtCore.Qt.KeyboardModifier.ControlModifier:
            self.owner.queue_wheel(zoom_steps=delta / 120.0, center=self.mapToView(ev.pos()).x())
            ev.accept()
        elif modifiers & QtCore.Qt.KeyboardModifier.ShiftModifier:
            self.owner.queue_wheel(pan_steps=-delta / 120.0)
            ev.accept()
        else:
            ev.ignore()          # the scroll area moves the page

    def mouseClickEvent(self, ev):  # noqa: N802
        if ev.button() == QtCore.Qt.MouseButton.LeftButton and ev.double():
            ev.accept()
            self.owner.reset_zoom()
            return
        super().mouseClickEvent(ev)


class AuxViewBox(ExactXViewBox):
    """The view box of an extra y axis: x-linked to the graph's main box,
    y autoranged on its own, and blind to the mouse (the main box has it)."""

    def __init__(self):
        super().__init__(enableMenu=False)
        self.setMouseEnabled(x=False, y=False)
        self.setAcceptedMouseButtons(QtCore.Qt.MouseButton.NoButton)
        self.setAcceptHoverEvents(False)

    def wheelEvent(self, ev, axis=None):  # noqa: N802
        ev.ignore()

    def mouseDragEvent(self, ev, axis=None):  # noqa: N802
        ev.ignore()

    def mouseClickEvent(self, ev):  # noqa: N802
        ev.ignore()


class LegendButton(QtWidgets.QToolButton):
    """A trace name in a graph header; drag it to another graph or the list."""

    def __init__(self, owner: "CsvViewerWidget", graph_index: int, ref, text: str,
                 icon: QtGui.QIcon | None = None, bold: bool = False, parent=None):
        super().__init__(parent)
        self.owner = owner
        self.graph_index = graph_index
        self.ref = ref
        self._press = None
        self.setAutoRaise(True)
        self.setText(text)
        self.setToolTip(f"{owner.series_label(ref)}: drag to another graph, the "
                        "drop area, or back to the trace list to remove it")
        if icon is not None:
            self.setIcon(icon)
            self.setToolButtonStyle(QtCore.Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        if bold:
            font = self.font()
            font.setBold(True)
            self.setFont(font)
        self.setContextMenuPolicy(QtCore.Qt.ContextMenuPolicy.CustomContextMenu)
        self.customContextMenuRequested.connect(self._menu)

    def _menu(self, pos) -> None:
        menu = QtWidgets.QMenu(self)
        menu.addAction("Spectrogram / FFT…", lambda: self.owner.open_spectrogram(self.ref))
        menu.addAction("Add spectrogram to board",
                       lambda: self.owner.add_board_spectrogram(self.ref, self.graph_index + 1))
        menu.addAction("Remove from graph", lambda: self.owner.drop(
            ("legend", self.graph_index, self.ref), ("list",)))
        menu.popup(self.mapToGlobal(pos))

    def mousePressEvent(self, event) -> None:  # noqa: N802
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self._press = event.position().toPoint()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:  # noqa: N802
        if self._press is None or (event.position().toPoint() - self._press).manhattanLength() \
                < QtWidgets.QApplication.startDragDistance():
            super().mouseMoveEvent(event)
            return
        self._press = None
        mime = QtCore.QMimeData()
        mime.setData(LEGEND_MIME, json.dumps(
            [self.graph_index, self.ref[0], self.ref[1]]).encode("utf-8"))
        drag = QtGui.QDrag(self)
        drag.setMimeData(mime)
        drag.exec(QtCore.Qt.DropAction.MoveAction)

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802
        self._press = None
        super().mouseReleaseEvent(event)


def _drop_payload(mime: QtCore.QMimeData):
    if mime.hasFormat(TRACES_MIME):
        return ("traces", _refs_from_json(mime.data(TRACES_MIME)))
    if mime.hasFormat(AXIS_MIME):
        source, group = json.loads(bytes(mime.data(AXIS_MIME)).decode())
        return ("axis", int(source), int(group))
    if mime.hasFormat(LEGEND_MIME):
        source, dataset_id, column = json.loads(bytes(mime.data(LEGEND_MIME)).decode())
        return ("legend", int(source), (int(dataset_id), str(column)))
    return None


class ResizeGrip(QtWidgets.QWidget):
    """Bottom edge of a graph: drag to change its height (the row's, when
    graphs are tiled), double-click to go back to the automatic height.

    While dragging only a guide line moves; the graph is resized (and
    redrawn) once, when the mouse is released.
    """

    def __init__(self, slot: "GraphSlot"):
        super().__init__(slot)
        self.slot = slot
        self.setCursor(QtCore.Qt.CursorShape.SizeVerCursor)
        self.setToolTip("Drag to resize this graph; double-click for automatic height")
        self._start = None
        self._guide = None

    def paintEvent(self, event) -> None:  # noqa: N802
        painter = QtGui.QPainter(self)
        painter.setPen(QtGui.QPen(QtGui.QColor("#b0b0b0"), 1))
        mid = self.height() // 2
        center = self.width() // 2
        for dx in (-12, -6, 0, 6, 12):
            painter.drawPoint(center + dx, mid)

    def mousePressEvent(self, event) -> None:  # noqa: N802
        if event.button() != QtCore.Qt.MouseButton.LeftButton:
            return
        self._start = (event.globalPosition().y(), self.slot.height())
        container = self.slot.owner.graph_container
        self._guide = QtWidgets.QRubberBand(QtWidgets.QRubberBand.Shape.Line, container)
        self._move_guide(event.globalPosition().y())
        self._guide.show()
        event.accept()

    def _target(self, global_y: float) -> int:
        start_y, start_height = self._start
        return int(max(MIN_RESIZE_HEIGHT, min(MAX_RESIZE_HEIGHT, start_height + global_y - start_y)))

    def _move_guide(self, global_y: float) -> None:
        container = self.slot.owner.graph_container
        top = self.slot.mapTo(container, QtCore.QPoint(0, 0))
        y = top.y() + self._target(global_y)
        self._guide.setGeometry(0, y - 1, container.width(), 3)

    def mouseMoveEvent(self, event) -> None:  # noqa: N802
        if self._start is not None:
            self._move_guide(event.globalPosition().y())

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802
        if self._start is None:
            return
        height = self._target(event.globalPosition().y())
        self._start = None
        if self._guide is not None:
            self._guide.hide()
            self._guide.deleteLater()
            self._guide = None
        self.slot.owner.set_graph_height(self.slot.index, height)

    def mouseDoubleClickEvent(self, event) -> None:  # noqa: N802
        self.slot.owner.set_graph_height(self.slot.index, None)


NEUTRAL_AXIS = "#555555"


class GroupAxis(pg.AxisItem):
    """A y axis of a graph (one group of traces). Drag it onto another axis of
    the graph to merge them, or to the edge / middle of a plot to move it;
    right-click for the menu."""

    def __init__(self, side: str, slot: "GraphSlot", number: int):
        super().__init__(side)
        self.slot = slot
        self.number = number
        self.setToolTip("Drag onto another axis to merge, to a plot edge to move; "
                        "right-click for more")

    def mouseDragEvent(self, ev):  # noqa: N802
        if ev.button() != QtCore.Qt.MouseButton.LeftButton or self.slot.plot is None:
            ev.ignore()
            return
        ev.accept()
        if not ev.isStart():
            return
        mime = QtCore.QMimeData()
        mime.setData(AXIS_MIME, json.dumps([self.slot.index, self.number]).encode("utf-8"))
        drag = QtGui.QDrag(self.slot.plot)
        drag.setMimeData(mime)
        drag.exec(QtCore.Qt.DropAction.MoveAction)

    def mouseClickEvent(self, ev):  # noqa: N802
        if ev.button() == QtCore.Qt.MouseButton.RightButton:
            ev.accept()
            self.slot.owner.axis_menu(self.slot.index, self.number, ev.screenPos().toPoint())
        else:
            super().mouseClickEvent(ev)


class DropHint(QtWidgets.QWidget):
    """Translucent overlay on a plot showing where a drag would land."""

    def __init__(self, parent):
        super().__init__(parent)
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.rect_f = QtCore.QRectF()
        self.text = ""
        self.hide()

    def show_target(self, rect: QtCore.QRectF, text: str) -> None:
        self.rect_f, self.text = rect, text
        self.setGeometry(self.parentWidget().rect())
        self.raise_()
        self.show()
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect_f, QtGui.QColor(21, 101, 192, 70))
        painter.setPen(QtGui.QPen(QtGui.QColor("#1565c0"), 2))
        painter.drawRect(self.rect_f)
        painter.setPen(QtGui.QColor("#0d3c75"))
        painter.drawText(self.rect_f, QtCore.Qt.AlignmentFlag.AlignCenter
                         | QtCore.Qt.TextFlag.TextWordWrap, self.text)
        painter.end()


def _autorange_y(view: pg.ViewBox, refs) -> None:
    """Y autorange over the visible x range, or a fixed 0..1 for status bits."""
    if refs and all(ref[1] in BINARY_COLUMNS for ref in refs):
        view.disableAutoRange(axis=pg.ViewBox.YAxis)
        view.setYRange(-0.1, 1.1, padding=0)
    else:
        view.enableAutoRange(axis=pg.ViewBox.YAxis)
        view.setAutoVisible(y=True)


def _map_between(source: pg.ViewBox, target: pg.ViewBox, y: float) -> float:
    """A y value of one view box expressed in another's coordinates (same screen height)."""
    low, high = source.viewRange()[1]
    t_low, t_high = target.viewRange()[1]
    if high == low:
        return (t_low + t_high) / 2
    return t_low + (y - low) / (high - low) * (t_high - t_low)


class GraphSlot(QtWidgets.QFrame):
    """One graph of the stack: a cheap placeholder until it scrolls into view,
    then a header (names, statistics, close) and a pyqtgraph plot."""

    def __init__(self, owner: "CsvViewerWidget", index: int, refs, height: int):
        super().__init__()
        self.owner = owner
        self.index = index
        self.refs = list(refs)
        self.plot: pg.PlotWidget | None = None
        self.vb: TimeViewBox | None = None
        self.curves: list = []           # [(ref, ShiftableCurve)]
        self.aux_views: dict = {}        # ref -> AuxViewBox (traces not on the first axis)
        self.axes: list = []             # GroupAxis per y axis, in group order
        self.group_views: list = []      # the ViewBox of each y axis
        self.hint: DropHint | None = None
        graph = owner.layout_model.graphs[index] if index < len(owner.layout_model.graphs) \
            else GraphRefs(self.refs)
        self.spec = [(group.side, list(group.refs)) for group in graph.axis_groups()]
        self.stats_label: QtWidgets.QLabel | None = None
        self.hover_line = self.hover_marks = self.hover_text = None
        self.setObjectName("graphSlot")
        self.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.setFixedHeight(height)
        self.setAcceptDrops(True)
        self._layout = QtWidgets.QVBoxLayout(self)
        self._layout.setContentsMargins(4, 2, 4, 2)
        self._layout.setSpacing(0)
        self._placeholder = QtWidgets.QLabel(
            ", ".join(owner.series_label(ref) for ref in self.refs))
        self._placeholder.setStyleSheet("color: #888888;")
        self._layout.addWidget(self._placeholder)
        self._layout.addStretch(1)
        self._layout.setContentsMargins(4, 2, 4, 2 + GRIP_HEIGHT)
        self.grip = ResizeGrip(self)

    def resizeEvent(self, event) -> None:  # noqa: N802
        super().resizeEvent(event)
        self.grip.setGeometry(0, self.height() - GRIP_HEIGHT, self.width(), GRIP_HEIGHT)
        self.grip.raise_()

    @property
    def realized(self) -> bool:
        return self.plot is not None

    @property
    def separate(self) -> bool:
        """More than one y axis."""
        return len(self.spec) > 1

    def view_for(self, ref):
        """The ViewBox a trace is drawn in (the main one unless on a later axis)."""
        return self.aux_views.get(ref, self.vb)

    def _build_axes(self) -> None:
        """One GroupAxis (and view box, for all but the first) per y axis, placed
        left or right of the plot, nearest the plot first."""
        item = self.plot.getPlotItem()
        for name in ("left", "right"):
            built_in = item.getAxis(name)
            item.layout.removeItem(built_in)
            built_in.hide()
        lefts, rights = [], []
        for number, (side, refs) in enumerate(self.spec):
            if number == 0:
                view = self.vb
            else:
                view = AuxViewBox()
                item.scene().addItem(view)
                view.setXLink(self.vb)
                view.setZValue(self.vb.zValue() - 1)

                def sync(_vb=None, view=view):
                    if self.vb is not None:
                        view.setGeometry(self.vb.sceneBoundingRect())
                self.vb.sigResized.connect(sync)
                sync()
            axis = GroupAxis(side, self, number)
            axis.setWidth(AXIS_WIDTH)
            axis.linkToView(view)
            self.group_views.append(view)
            self.axes.append(axis)
            (lefts if side == "left" else rights).append(axis)
            for ref in refs:
                if number > 0:
                    self.aux_views[ref] = view
        row_left = QtWidgets.QGraphicsLinearLayout(QtCore.Qt.Orientation.Horizontal)
        row_right = QtWidgets.QGraphicsLinearLayout(QtCore.Qt.Orientation.Horizontal)
        for row in (row_left, row_right):
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(0)
        row_left.addStretch(1)
        for axis in reversed(lefts):        # outermost first
            row_left.addItem(axis)
        for axis in rights:
            row_right.addItem(axis)
        row_right.addStretch(1)
        item.layout.addItem(row_left, 2, 0)
        item.layout.addItem(row_right, 2, 2)
        self._rows = (row_left, row_right)

    def _style_axes(self) -> None:
        """Label and colour each axis like its trace (neutral when it carries several)."""
        owner = self.owner
        for number, (axis, (side, refs)) in enumerate(zip(self.axes, self.spec)):
            if len(self.spec) < 2:
                break
            if len(refs) == 1:
                color = owner.series_style(self.index, self.refs.index(refs[0]),
                                           len(self.refs), refs[0])[0]
                text = owner.series_label(refs[0])
            else:
                color = NEUTRAL_AXIS
                text = f"{owner.series_label(refs[0])} +{len(refs) - 1}"
            axis.setLabel(text, color=color)
            axis.setPen(pg.mkPen(color))
            axis.setTextPen(pg.mkPen(color))

    def axis_summary(self, number: int) -> str:
        side, refs = self.spec[number]
        return ", ".join(self.owner.series_label(ref) for ref in refs)

    # ------------------------------------------------------------ build
    def realize(self) -> None:
        if self.plot is not None:
            return
        owner = self.owner
        while self._layout.count():
            item = self._layout.takeAt(0)
            if item.widget() is not None:
                item.widget().deleteLater()
        self._placeholder = None
        self._layout.addWidget(self._build_header())
        self.vb = TimeViewBox(owner)
        plot = qt_common.make_plot(view_box=self.vb, legend=False)
        plot.setAcceptDrops(False)
        plot.viewport().setAcceptDrops(False)
        plot.viewport().installEventFilter(owner)
        item = plot.getPlotItem()
        item.setLabel("bottom", owner.x_column())
        item.hideButtons()
        self.plot = plot
        self._layout.addWidget(plot, 1)
        self.hint = DropHint(plot.viewport())

        vb = self.vb
        vb.disableAutoRange(axis=pg.ViewBox.XAxis)
        separate = self.separate
        self._build_axes()
        # Each axis autoranges over its own traces (status bits: a fixed 0..1 axis).
        for view, (_side, refs) in zip(self.group_views, self.spec):
            _autorange_y(view, refs)
        x_column = owner.x_column()
        any_data = False
        for series_index, ref in enumerate(self.refs):
            xs, ys = owner.series(ref)
            color, style = owner.series_style(self.index, series_index, len(self.refs), ref)
            pen = pg.mkPen(color, width=1.0 if xs.size > DENSE_POINTS else 1.5, style=style)
            aux = self.aux_views.get(ref)
            if aux is not None:
                curve = qt_common.ShiftableCurve(xs, ys, pen=pen)
                aux.addItem(curve)
                curve.setClipToView(True)
                curve.setDownsampling(auto=True, method="peak")
            else:
                curve = add_curve(plot, xs, ys, pen=pen)
            curve.setSkipFiniteCheck(True)        # series arrays are finite
            dataset = owner.datasets.get(ref[0])
            if dataset is not None:
                curve.shift(vd.x_offset(dataset, x_column))
            self.curves.append((ref, curve))
            any_data = any_data or xs.size > 0
        if not any_data:
            message = pg.TextItem(f"No values for x = {x_column}", color="#777777",
                                  anchor=(0.5, 0.5))
            plot.addItem(message, ignoreBounds=True)
            full = owner.current_x_range()
            message.setPos((full[0] + full[1]) / 2, 0.5)
        self._style_axes()
        owner._pad_sides(self)
        # Hover crosshair, sample markers and the readout box.
        self.hover_line = pg.InfiniteLine(angle=90, movable=False,
                                          pen=pg.mkPen("#9e9e9e", style=QtCore.Qt.PenStyle.DashLine))
        self.hover_marks = pg.ScatterPlotItem(size=9, pen=pg.mkPen(width=2), brush="w")
        self.hover_text = pg.TextItem(anchor=(0, 0), color="#222222", fill=(255, 255, 244, 235),
                                      border="#9e9e9e")
        for hover_item in (self.hover_line, self.hover_marks, self.hover_text):
            hover_item.hide()
            plot.addItem(hover_item, ignoreBounds=True)
        self.hover_text.setZValue(100)
        self.hover_marks.setZValue(90)
        # Context menu: spectrogram of each trace, remove the graph.
        menu = vb.menu
        menu.addSeparator()
        for ref in self.refs:
            menu.addAction(f"Spectrogram / FFT: {owner.series_label(ref)}",
                           lambda r=ref: owner.open_spectrogram(r))
            menu.addAction(f"Add spectrogram to board: {owner.series_label(ref)}",
                           lambda r=ref: owner.add_board_spectrogram(r, self.index + 1))
        if len(self.refs) > 1:
            toggle = menu.addAction("Each trace on its own y axis")
            toggle.setCheckable(True)
            toggle.setChecked(separate and len(self.spec) == len(self.refs))
            toggle.triggered.connect(lambda on: owner.set_separate_axes(self.index, on))
        menu.addAction("Remove this graph", lambda: owner.remove_graph(self.index))
        owner._attach_view(self)
        plot.scene().sigMouseMoved.connect(self._mouse_moved)
        self.update_stats()
        self.grip.raise_()

    def _build_header(self) -> QtWidgets.QWidget:
        owner = self.owner
        header = QtWidgets.QWidget()
        row = QtWidgets.QHBoxLayout(header)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(2)
        if len(self.refs) == 1:
            ref = self.refs[0]
            name = LegendButton(owner, self.index, ref, owner.series_label(ref), bold=True)
            name.setMaximumWidth(360)
            row.addWidget(name)
            self.stats_label = QtWidgets.QLabel("")
            self.stats_label.setStyleSheet("color: #444444;")
            self.stats_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight
                                          | QtCore.Qt.AlignmentFlag.AlignVCenter)
            self.stats_label.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored,
                                           QtWidgets.QSizePolicy.Policy.Preferred)
            row.addWidget(self.stats_label, 1)
        else:
            for series_index, ref in enumerate(self.refs):
                color, style = owner.series_style(self.index, series_index, len(self.refs), ref)
                button = LegendButton(owner, self.index, ref, owner.series_label(ref),
                                      icon=_line_icon(color, style))
                button.setMaximumWidth(260)
                button.setStyleSheet(f"color: {color};")
                row.addWidget(button)
            row.addStretch(1)
            own = QtWidgets.QToolButton()
            own.setText("Own axes")
            own.setCheckable(True)
            own.setChecked(self.separate)
            own.setAutoRaise(True)
            own.setToolTip("Draw each trace on its own y axis, each autoranged separately")
            own.clicked.connect(lambda on: owner.set_separate_axes(self.index, on))
            row.addWidget(own)
        close = QtWidgets.QToolButton()
        close.setText("✕")
        close.setAutoRaise(True)
        close.setToolTip("Remove this graph")
        close.clicked.connect(lambda: owner.remove_graph(self.index))
        row.addWidget(close)
        return header

    def dispose(self) -> None:
        """Free the plot (kept as a placeholder slot)."""
        if self.plot is None:
            return
        self.owner._detach_view(self)
        plot, self.plot = self.plot, None
        self.vb = None
        self.curves = []
        self.aux_views = {}
        self.axes = []
        self.group_views = []
        self.hint = None
        self.stats_label = None
        self.hover_line = self.hover_marks = self.hover_text = None
        while self._layout.count():
            item = self._layout.takeAt(0)
            if item.widget() is not None:
                item.widget().deleteLater()
        plot.deleteLater()
        self._placeholder = QtWidgets.QLabel(
            ", ".join(self.owner.series_label(ref) for ref in self.refs))
        self._placeholder.setStyleSheet("color: #888888;")
        self._layout.addWidget(self._placeholder)
        self._layout.addStretch(1)

    # ------------------------------------------------------------ stats
    def update_stats(self) -> None:
        if self.stats_label is None or not self.curves:
            return
        owner = self.owner
        ref = self.refs[0]
        xs, ys = owner.series(ref)
        view = owner.x_view
        if view is not None:
            dataset = owner.datasets.get(ref[0])
            offset = vd.x_offset(dataset, owner.x_column()) if dataset else 0.0
            stats = vd.view_stats(xs, ys, view[0] - offset, view[1] - offset)
        else:
            stats = vd.view_stats(xs, ys)
        self.stats_label.setText("" if stats is None else vd.stats_text(stats, view is not None))

    # ------------------------------------------------------------ hover
    def _mouse_moved(self, pos) -> None:
        self.owner._hover_request(self, pos)

    def x_at_scene(self, pos) -> float | None:
        if self.vb is None or not self.vb.sceneBoundingRect().contains(pos):
            return None
        return float(self.vb.mapSceneToView(pos).x())

    # ------------------------------------------------------------ drops
    def drop_target(self, pos):
        """Where a drop at ``pos`` (slot coordinates) lands on the axes, or None
        (header, margins, unbuilt plot: the plain overlay)."""
        if self.plot is None or self.vb is None:
            return None
        point = self.plot.mapFrom(self, QtCore.QPoint(int(pos.x()), int(pos.y())))
        scene = self.plot.mapToScene(point)
        rect = self.vb.sceneBoundingRect()
        if not self.plot.rect().contains(point) or not rect.top() <= scene.y() <= rect.bottom():
            return None
        spans = []
        for number, axis in enumerate(self.axes):
            # The layout cell, not boundingRect(): that includes grid lines over the plot.
            cell = axis.parentItem().mapRectToScene(axis.geometry())
            spans.append((cell.left(), cell.right(), number))
        lefts = sum(side == "left" for side, _refs in self.spec)
        return classify_drop(scene.x(), spans, rect.left(), rect.right(),
                             lefts, len(self.spec) - lefts)

    def _show_hint(self, target) -> None:
        if self.hint is None:
            return
        if target is None:
            self.hint.hide()
            return
        top = self.plot.mapFromScene(self.vb.sceneBoundingRect().topLeft()).y()
        bottom = self.plot.mapFromScene(self.vb.sceneBoundingRect().bottomLeft()).y()
        x0 = self.plot.mapFromScene(QtCore.QPointF(target.x0, 0)).x()
        x1 = self.plot.mapFromScene(QtCore.QPointF(target.x1, 0)).x()
        if target.kind == "group":
            text = "same axis:\n" + self.axis_summary(target.value)
        else:
            text = f"new axis\n({target.value})"
        self.hint.show_target(QtCore.QRectF(x0, top, x1 - x0, bottom - top), text)

    def dragEnterEvent(self, event) -> None:  # noqa: N802
        if _drop_payload(event.mimeData()) is not None:
            event.acceptProposedAction()
            self.setStyleSheet("#graphSlot { border: 3px solid #1565c0; }")
        else:
            event.ignore()

    def dragMoveEvent(self, event) -> None:  # noqa: N802
        if _drop_payload(event.mimeData()) is not None:
            event.acceptProposedAction()
            self._show_hint(self.drop_target(event.position()))
        else:
            event.ignore()

    def dragLeaveEvent(self, event) -> None:  # noqa: N802
        self.setStyleSheet("")
        self._show_hint(None)

    def dropEvent(self, event) -> None:  # noqa: N802
        self.setStyleSheet("")
        payload = _drop_payload(event.mimeData())
        target = self.drop_target(event.position())
        self._show_hint(None)
        if payload is None:
            event.ignore()
            return
        event.acceptProposedAction()
        index = self.index
        where = ("graph", index) if target is None else ("graph", index, target.axis)
        QtCore.QTimer.singleShot(0, lambda: self.owner.drop(payload, where))


class DropZone(QtWidgets.QLabel):
    """The area below the last graph: drop traces here for a new graph."""

    def __init__(self, owner: "CsvViewerWidget"):
        super().__init__()
        self.owner = owner
        self.setAcceptDrops(True)
        self.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.setMinimumHeight(DROP_ZONE_HEIGHT - 16)
        self.set_empty(True)
        self._style(False)

    def set_empty(self, empty: bool) -> None:
        self.setText("Drag traces here from the list to plot them" if empty
                     else "Drop a trace here for a new graph")

    def _style(self, active: bool) -> None:
        self.setStyleSheet(
            "QLabel { border: 1px dashed %s; color: #777777; background: %s; }"
            % (("#1565c0", "#e3f2fd") if active else ("#bdbdbd", "transparent")))

    def dragEnterEvent(self, event) -> None:  # noqa: N802
        if _drop_payload(event.mimeData()) is not None:
            event.acceptProposedAction()
            self._style(True)
        else:
            event.ignore()

    def dragMoveEvent(self, event) -> None:  # noqa: N802
        if _drop_payload(event.mimeData()) is not None:
            event.acceptProposedAction()
        else:
            event.ignore()

    def dragLeaveEvent(self, event) -> None:  # noqa: N802
        self._style(False)

    def dropEvent(self, event) -> None:  # noqa: N802
        self._style(False)
        payload = _drop_payload(event.mimeData())
        if payload is None:
            event.ignore()
            return
        event.acceptProposedAction()
        QtCore.QTimer.singleShot(0, lambda: self.owner.drop(payload, ("new",)))


class TraceTree(QtWidgets.QTreeWidget):
    """CSVs and their columns. Drag columns onto graphs; drop a graph's
    legend entry here to remove it from that graph."""

    def __init__(self, owner: "CsvViewerWidget"):
        super().__init__()
        self.owner = owner
        self.setHeaderHidden(True)
        self.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.ExtendedSelection)
        self.setDragEnabled(True)
        self.setAcceptDrops(True)
        self.setDragDropMode(QtWidgets.QAbstractItemView.DragDropMode.DragDrop)
        self.setDefaultDropAction(QtCore.Qt.DropAction.CopyAction)
        self.setUniformRowHeights(True)

    def mimeTypes(self):  # noqa: N802
        return [TRACES_MIME]

    def mimeData(self, items):  # noqa: N802
        refs = [ref for ref in (self.owner._item_ref(item) for item in items) if ref]
        mime = QtCore.QMimeData()
        mime.setData(TRACES_MIME, _refs_to_json(refs))
        return mime

    def startDrag(self, actions) -> None:  # noqa: N802
        refs = [ref for ref in (self.owner._item_ref(item) for item in self.selectedItems())
                if ref]
        if not refs:
            return
        drag = QtGui.QDrag(self)
        drag.setMimeData(self.mimeData(self.selectedItems()))
        drag.exec(QtCore.Qt.DropAction.CopyAction)

    def dragEnterEvent(self, event) -> None:  # noqa: N802
        if event.mimeData().hasFormat(LEGEND_MIME):
            event.acceptProposedAction()
        else:
            event.ignore()

    def dragMoveEvent(self, event) -> None:  # noqa: N802
        if event.mimeData().hasFormat(LEGEND_MIME):
            event.acceptProposedAction()
        else:
            event.ignore()

    def dropEvent(self, event) -> None:  # noqa: N802
        payload = _drop_payload(event.mimeData())
        if payload is None or payload[0] != "legend":
            event.ignore()
            return
        event.acceptProposedAction()
        QtCore.QTimer.singleShot(0, lambda: self.owner.drop(payload, ("list",)))


class ColumnPickerDialog(QtWidgets.QDialog):
    """Offered when a file would exceed the memory budget: pick columns (or
    thin rows) so it fits, rather than allocating it all."""

    def __init__(self, error: MemoryBudgetError, parent=None):
        super().__init__(parent)
        self.error = error
        estimate = error.estimate
        self.setWindowTitle(f"{error.path.name} is too large")
        layout = QtWidgets.QVBoxLayout(self)
        intro = QtWidgets.QLabel(str(error))
        intro.setWordWrap(True)
        layout.addWidget(intro)
        self.list = QtWidgets.QListWidget()
        time_like = set(vd.TIME_COLUMN_PREFERENCE) | vd.SYNC_SKIP_COLUMNS | {
            "uptime", "fc_uptime", "status", "fc_status"}
        for name in estimate.columns:
            item = QtWidgets.QListWidgetItem(name)
            item.setFlags(item.flags() | QtCore.Qt.ItemFlag.ItemIsUserCheckable)
            checked = name in time_like or "time" in name.lower()
            item.setCheckState(QtCore.Qt.CheckState.Checked if checked
                               else QtCore.Qt.CheckState.Unchecked)
            self.list.addItem(item)
        layout.addWidget(self.list, 1)
        self.estimate_label = QtWidgets.QLabel()
        layout.addWidget(self.estimate_label)
        buttons = QtWidgets.QDialogButtonBox()
        self.load_button = buttons.addButton("Load selected columns",
                                             QtWidgets.QDialogButtonBox.ButtonRole.AcceptRole)
        self.thin_button = None
        if estimate.thinnable:
            self.thin_button = buttons.addButton("Load every Nth row",
                                                 QtWidgets.QDialogButtonBox.ButtonRole.ActionRole)
            self.thin_button.clicked.connect(self._thin)
        buttons.addButton(QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
        self.choice = None
        self.list.itemChanged.connect(self._update)
        self._update()
        self.resize(460, 520)

    def selected(self) -> list[str]:
        return [self.list.item(i).text() for i in range(self.list.count())
                if self.list.item(i).checkState() == QtCore.Qt.CheckState.Checked]

    def _update(self, *_args) -> None:
        columns = self.selected()
        need = self.error.estimate.bytes_for(columns)
        fits = need <= self.error.budget and bool(columns)
        self.estimate_label.setText(
            f"{len(columns)} column(s): about {vd.format_bytes(need)} of "
            f"{vd.format_bytes(self.error.budget)}" + ("" if fits else " (too much)"))
        self.load_button.setEnabled(fits)

    def _thin(self) -> None:
        self.choice = ("thin", None)
        super().accept()

    def accept(self) -> None:
        if self.choice is None:
            self.choice = ("columns", self.selected())
        super().accept()


class LoadRequest:
    __slots__ = ("path", "columns", "thin")

    def __init__(self, path, columns=None, thin=False):
        self.path = Path(path)
        self.columns = columns
        self.thin = thin


# ---------------------------------------------------------------------------
# The viewer
# ---------------------------------------------------------------------------

class CsvViewerWidget(QtWidgets.QWidget):
    """The CSV viewer as an embeddable widget (Summary and Plots tabs).

    Several CSVs can be open at once. The trace list groups numeric columns
    by CSV; drag a trace onto a graph to overlay it, or onto the area below
    the last graph for a new one. Drag a graph's legend entry onto another
    graph to move it, or back onto the list to remove it. Every CSV has its
    own time axis (column, units, offset), so logs of the same session line
    up on one shared time axis.

    ``on_title`` is called with a window title when the files change.
    """

    show_dialogs = True          # tests turn message boxes off; messages are kept

    def __init__(self, parent=None, files=(), *, on_title=None):
        super().__init__(parent)
        ensure_app()
        self.on_title = on_title
        self.datasets: dict[int, Dataset] = {}
        self._next_dataset_id = 0
        self.layout_model = GraphLayout()
        self.slots: list[GraphSlot] = []
        self.x_view: tuple[float, float] | None = None
        self._full_x: tuple[float, float] | None = None
        self._link_root: TimeViewBox | None = None
        self._series_cache: collections.OrderedDict = collections.OrderedDict()
        self._order_cache: dict = {}
        self._cache_bytes = 0
        self._syncing = False
        self._closing = False
        self.messages: list[tuple[str, str, str]] = []
        self.last_save_message = ""
        self.memory_budget: int | None = None      # None: viewer_data default
        self._load_queue: list[LoadRequest] = []
        self._load_worker = None
        self._load_request: LoadRequest | None = None
        self._load_cancel = threading.Event()
        self._load_errors: list[str] = []
        self._save_worker = None
        self._switch_saved: float | None = None
        self._spectrum_windows: list[SpectrogramWindow] = []
        self._hover_pending = None
        self._hover_visible = False
        self.graph_columns = 1
        self.docks: list[SpectrogramDock] = []   # spectrograms docked in the board
        self.sync_transition: tuple | None = None   # (from, to) state the logs are synced on
        self.overlay_own_axes = False    # new overlays get a y axis each
        self._height_overrides: dict[tuple, int] = {}   # graph refs -> height set by dragging
        self._wheel_zoom = 0.0          # queued Ctrl+wheel steps (+ = in)
        self._wheel_pan = 0.0           # queued Shift+wheel steps (+ = later)
        self._wheel_center = None

        self._build_ui()
        self.setAcceptDrops(True)

        self._realize_timer = QtCore.QTimer(self)
        self._realize_timer.setSingleShot(True)
        self._realize_timer.setInterval(0)
        self._realize_timer.timeout.connect(self._realize_visible)
        self._hover_throttle = Debounce(self._hover_tick, HOVER_SETTLE_MS, self)
        self._view_throttle = Throttle(self._view_changed_tick, 120, self)
        self._wheel_throttle = Debounce(self._apply_wheel, SETTLE_MS, self)
        self._scroll_settle = Debounce(self._schedule_realize, SETTLE_MS, self)
        self._pan_settle = Debounce(self._apply_drag_pan, SETTLE_MS, self)
        self._drag_pan = None           # (view at press, view x at press, latest view x)

        files = [files] if isinstance(files, (str, Path)) else list(files or ())
        if files:
            QtCore.QTimer.singleShot(0, lambda: self.add_files(files))
        self.refresh_all()

    # ================================================================ UI
    def _build_ui(self) -> None:
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(6, 6, 6, 6)

        bar = QtWidgets.QHBoxLayout()
        outer.addLayout(bar)
        open_button = QtWidgets.QPushButton("Open CSV…")
        open_button.clicked.connect(self.choose_csv)
        bin_button = QtWidgets.QPushButton("Open .BIN…")
        bin_button.clicked.connect(self.choose_bin)
        add_button = QtWidgets.QPushButton("Add CSV…")
        add_button.clicked.connect(self.choose_add_csv)
        self.hex_check = QtWidgets.QCheckBox("Hex columns as numbers")
        self.hex_check.setChecked(True)
        self.hex_check.setToolTip("Applies to files opened from now on")
        for widget in (open_button, bin_button, add_button, self.hex_check):
            bar.addWidget(widget)
        self.file_label = ElidedLabel("No file loaded")
        bar.addWidget(self.file_label, 1)
        self.load_label = QtWidgets.QLabel("")
        self.load_bar = QtWidgets.QProgressBar()
        self.load_bar.setRange(0, 1000)
        self.load_bar.setFixedWidth(140)
        self.load_bar.setTextVisible(False)
        self.load_cancel_button = QtWidgets.QPushButton("Cancel")
        self.load_cancel_button.clicked.connect(self.cancel_loads)
        for widget in (self.load_label, self.load_bar, self.load_cancel_button):
            bar.addWidget(widget)
            widget.hide()

        self.notebook = QtWidgets.QTabWidget()
        outer.addWidget(self.notebook, 1)
        self.summary_tab = self._build_summary_tab()
        self.plot_tab = self._build_plot_tab()
        self.notebook.addTab(self.summary_tab, "Summary")
        self.notebook.addTab(self.plot_tab, "Plots")

    def _build_summary_tab(self) -> QtWidgets.QWidget:
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        page = QtWidgets.QWidget()
        scroll.setWidget(page)
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(18, 18, 18, 18)
        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("Flight Summary")
        font = title.font()
        font.setPointSize(22)
        font.setBold(True)
        title.setFont(font)
        header.addWidget(title)
        header.addStretch(1)
        header.addWidget(QtWidgets.QLabel("CSV:"))
        self.summary_combo = QtWidgets.QComboBox()
        self.summary_combo.setMinimumWidth(220)
        self.summary_combo.currentIndexChanged.connect(lambda _i: self.show_summary())
        header.addWidget(self.summary_combo)
        layout.addLayout(header)
        layout.addWidget(QtWidgets.QLabel(
            "Summary values calculated from the complete converted CSV."))
        layout.addSpacing(14)

        cards = QtWidgets.QHBoxLayout()
        self.summary_values: dict[str, QtWidgets.QLabel] = {}
        for key, text in (("max_altitude", "Max altitude (Kalman, else baro)"),
                          ("launch_time", "Launch detected at"),
                          ("max_acceleration", "Max acceleration")):
            card = QtWidgets.QGroupBox(text)
            card_layout = QtWidgets.QVBoxLayout(card)
            value = QtWidgets.QLabel("--")
            value.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            value.setWordWrap(True)
            value_font = value.font()
            value_font.setPointSize(13)
            value_font.setBold(True)
            value.setFont(value_font)
            card_layout.addWidget(value)
            cards.addWidget(card, 1)
            self.summary_values[key] = value
        layout.addLayout(cards)
        state_card = QtWidgets.QGroupBox("Flight state (status byte 1 bits 2:0)")
        state_layout = QtWidgets.QVBoxLayout(state_card)
        state = QtWidgets.QLabel("--")
        state.setWordWrap(True)
        state_font = state.font()
        state_font.setPointSize(12)
        state_font.setBold(True)
        state.setFont(state_font)
        state_layout.addWidget(state)
        self.summary_values["flight_state"] = state
        layout.addSpacing(10)
        layout.addWidget(state_card)
        self.summary_details_label = QtWidgets.QLabel("Open a CSV file to begin.")
        self.summary_details_label.setWordWrap(True)
        self.summary_details_label.setTextInteractionFlags(
            QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addSpacing(18)
        layout.addWidget(self.summary_details_label)
        note = QtWidgets.QLabel(LAUNCH_NOTE)
        note.setWordWrap(True)
        note.setStyleSheet("color: #555555;")
        layout.addWidget(note)
        layout.addStretch(1)
        return scroll

    def _build_plot_tab(self) -> QtWidgets.QWidget:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)
        layout.setContentsMargins(4, 4, 4, 4)
        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("X axis:"))
        self.x_axis_combo = QtWidgets.QComboBox()
        self.x_axis_combo.setMinimumWidth(220)
        self.x_axis_combo.activated.connect(lambda _i: self.on_axis_changed())
        controls.addWidget(self.x_axis_combo)
        controls.addSpacing(10)
        controls.addWidget(QtWidgets.QLabel("Packet type:"))
        self.packet_filter_combo = QtWidgets.QComboBox()
        self.packet_filter_combo.addItem("All")
        self.packet_filter_combo.activated.connect(lambda _i: self.on_axis_changed())
        controls.addWidget(self.packet_filter_combo)
        controls.addSpacing(10)
        controls.addWidget(QtWidgets.QLabel("Graphs per row:"))
        self.columns_spin = QtWidgets.QSpinBox()
        self.columns_spin.setRange(1, MAX_COLUMNS)
        self.columns_spin.setValue(self.graph_columns)
        self.columns_spin.setToolTip("Tile graphs side by side")
        self.columns_spin.valueChanged.connect(self.set_columns)
        controls.addWidget(self.columns_spin)
        self.own_axes_check = QtWidgets.QCheckBox("Overlays on own y axes")
        self.own_axes_check.setToolTip(
            "Traces dropped onto a graph get their own autoranging y axis each "
            "(also switchable per graph: Own axes in its header)")
        self.own_axes_check.toggled.connect(self._own_axes_default_changed)
        controls.addWidget(self.own_axes_check)
        auto_heights = QtWidgets.QPushButton("Auto heights")
        auto_heights.setToolTip("Undo graph resizing (drag a graph's bottom edge to resize it)")
        auto_heights.clicked.connect(self.reset_graph_heights)
        controls.addWidget(auto_heights)
        for text, tip, slot in (
                ("Save board…", "Save the graphs, axes and spectrograms as a data board file",
                 self.choose_save_board),
                ("Load board…", "Rebuild a saved data board on the open log(s), by column name",
                 self.choose_load_board)):
            button = QtWidgets.QPushButton(text)
            button.setToolTip(tip)
            button.clicked.connect(slot)
            controls.addWidget(button)
        controls.addStretch(1)
        layout.addLayout(controls)

        status = QtWidgets.QHBoxLayout()
        self.summary_label = ElidedLabel("")
        status.addWidget(self.summary_label, 1)
        for text, slot in (("Zoom out", self.zoom_out), ("Reset zoom", self.reset_zoom),
                           ("Clear graphs", self.clear_graphs)):
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(slot)
            status.addWidget(button)
        layout.addLayout(status)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        layout.addWidget(splitter, 1)
        splitter.addWidget(self._build_trace_panel())

        self.graph_scroll = QtWidgets.QScrollArea()
        self.graph_scroll.setWidgetResizable(True)
        self.graph_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.graph_container = QtWidgets.QWidget()
        # Graphs in a grid: ``graph_columns`` per row (1 = the classic stack).
        self.graph_layout = QtWidgets.QGridLayout(self.graph_container)
        self.graph_layout.setContentsMargins(0, 0, 0, 0)
        self.graph_layout.setSpacing(4)
        self.drop_zone = DropZone(self)
        self._place_slots()
        self.graph_scroll.setWidget(self.graph_container)
        # Scrolling the page builds newly visible graphs once scrolling stops.
        self.graph_scroll.verticalScrollBar().valueChanged.connect(
            lambda _v: self._scroll_settle.request())
        self.graph_scroll.viewport().installEventFilter(self)
        splitter.addWidget(self.graph_scroll)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([250, 900])
        return tab

    def _build_trace_panel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        panel.setMinimumWidth(200)
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(2, 2, 2, 2)
        title = QtWidgets.QLabel("Traces")
        font = title.font()
        font.setBold(True)
        font.setPointSize(font.pointSize() + 1)
        title.setFont(font)
        layout.addWidget(title)
        self.help_label = QtWidgets.QLabel(
            "Drag traces onto a graph's y axis to share it, onto its left/right edge or "
            "middle for a new y axis, or below the last graph for a new graph. "
            "Double-click adds a graph. Drag an axis onto another to merge them. Drag a "
            "legend entry to another graph, or back here to remove it.\nZoom (all graphs): drag across a plot, or "
            "Ctrl+wheel; Shift+wheel pans; double-click a plot to reset. Hover a plot "
            "for exact values. Drop files anywhere to open them.")
        self.help_label.setWordWrap(True)
        self.help_label.setStyleSheet("color: #555555;")
        layout.addWidget(self.help_label)

        buttons = QtWidgets.QHBoxLayout()
        for text, slot in (("Plot CSV", self.plot_selected_dataset),
                           ("Save CSV", self.save_selected_as_csv),
                           ("Remove CSV", self.remove_selected_dataset)):
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(slot)
            buttons.addWidget(button)
            if text == "Save CSV":
                self.save_csv_button = button
        layout.addLayout(buttons)
        spectrum_button = QtWidgets.QPushButton("Spectrogram / FFT of selected trace…")
        spectrum_button.clicked.connect(self.spectrogram_selected)
        layout.addWidget(spectrum_button)
        board_button = QtWidgets.QPushButton("Add spectrogram of selected trace to board")
        board_button.setToolTip("Dock a spectrogram between the graphs, on the same time axis")
        board_button.clicked.connect(self.board_spectrogram_selected)
        layout.addWidget(board_button)

        self.time_axis_box = QtWidgets.QGroupBox("Time axis")
        grid = QtWidgets.QGridLayout(self.time_axis_box)
        grid.addWidget(QtWidgets.QLabel("Column"), 0, 0)
        self.time_column_combo = QtWidgets.QComboBox()
        self.time_column_combo.activated.connect(lambda _i: self._time_controls_changed())
        grid.addWidget(self.time_column_combo, 0, 1)
        grid.addWidget(QtWidgets.QLabel("Units"), 1, 0)
        self.time_unit_combo = QtWidgets.QComboBox()
        self.time_unit_combo.addItems(list(TIME_UNITS))
        self.time_unit_combo.activated.connect(lambda _i: self._time_controls_changed())
        grid.addWidget(self.time_unit_combo, 1, 1)
        grid.addWidget(QtWidgets.QLabel("Offset s"), 2, 0)
        self.offset_spin = OffsetSpinBox()
        self.offset_spin.valueChanged.connect(lambda _v: self.apply_offset())
        grid.addWidget(self.offset_spin, 2, 1)
        nudges = QtWidgets.QHBoxLayout()
        for text, delta in (("−1", -1.0), ("−0.1", -0.1), ("+0.1", 0.1), ("+1", 1.0)):
            button = QtWidgets.QPushButton(text)
            button.setMinimumWidth(36)
            button.clicked.connect(lambda _c=False, d=delta: self.nudge_offset(d))
            nudges.addWidget(button)
        grid.addLayout(nudges, 3, 0, 1, 2)
        self.time_axis_note = QtWidgets.QLabel("")
        self.time_axis_note.setWordWrap(True)
        self.time_axis_note.setStyleSheet("color: #555555;")
        grid.addWidget(self.time_axis_note, 4, 0, 1, 2)
        grid.setColumnStretch(1, 1)
        layout.addWidget(self.time_axis_box)
        sync_row = QtWidgets.QHBoxLayout()
        sync_row.addWidget(QtWidgets.QLabel("Sync logs on"))
        self.sync_combo = QtWidgets.QComboBox()
        self.sync_combo.setToolTip(
            "Shift every open Witness log's time offset so the first rise into the chosen "
            "flight state is t = 0 (from the log's state, else its session's .TXT log or "
            "another open log of the session). The offsets stay editable above.")
        self.sync_combo.addItem("(not synced)", None)
        for label, pair in vd.SYNC_TRANSITIONS.items():
            self.sync_combo.addItem(label, pair)
        self.sync_combo.activated.connect(lambda _i: self._sync_choice_changed())
        sync_row.addWidget(self.sync_combo, 1)
        layout.addLayout(sync_row)

        self.trace_tree = TraceTree(self)
        self.trace_tree.itemSelectionChanged.connect(self._sync_time_axis_controls)
        self.trace_tree.itemDoubleClicked.connect(self._tree_double)
        self.trace_tree.setContextMenuPolicy(QtCore.Qt.ContextMenuPolicy.CustomContextMenu)
        self.trace_tree.customContextMenuRequested.connect(self._tree_menu)
        layout.addWidget(self.trace_tree, 1)
        return panel

    # ========================================================= lifecycle
    def shutdown(self) -> None:
        """Stop timers, background work and spectrogram windows (idempotent)."""
        self._closing = True
        for timer in (self._realize_timer,):
            timer.stop()
        self._hover_throttle.stop()
        self._view_throttle.stop()
        self._wheel_throttle.stop()
        self._scroll_settle.stop()
        self._pan_settle.stop()
        self.cancel_loads()
        for worker in (self._load_worker, self._save_worker):
            self._disconnect_worker(worker)
        self._load_worker = self._save_worker = None
        for window in list(self._spectrum_windows):
            try:
                window.shutdown()
                window.close()
            except RuntimeError:
                pass
        self._spectrum_windows = []
        for dock in self.docks:
            dock.dispose()
        self.docks = []
        self._background_busy(False)

    def closeEvent(self, event) -> None:  # noqa: N802
        self.shutdown()
        super().closeEvent(event)

    @staticmethod
    def _disconnect_worker(worker) -> None:
        if worker is None:
            return
        for signal in (worker.signals.done, worker.signals.failed, worker.signals.progress):
            try:
                signal.disconnect()
            except (TypeError, RuntimeError):
                pass

    def _background_busy(self, busy: bool) -> None:
        # While a worker parses or decodes, every GUI-thread callback waits up
        # to a whole GIL switch interval (5 ms); a short interval keeps the
        # interface smooth. Restored as soon as the background work ends.
        if busy and self._switch_saved is None:
            self._switch_saved = sys.getswitchinterval()
            sys.setswitchinterval(BUSY_SWITCH_INTERVAL_S)
        elif not busy and self._switch_saved is not None:
            sys.setswitchinterval(self._switch_saved)
            self._switch_saved = None

    def _message(self, kind: str, title: str, text: str) -> None:
        """Report to the user without blocking (non-modal box), and keep it."""
        self.messages.append((kind, title, text))
        if not self.show_dialogs or self._closing:
            return
        icon = (QtWidgets.QMessageBox.Icon.Critical if kind == "error"
                else QtWidgets.QMessageBox.Icon.Information)
        box = QtWidgets.QMessageBox(icon, title, text, parent=self)
        box.setAttribute(QtCore.Qt.WidgetAttribute.WA_DeleteOnClose)
        box.open()

    def _set_title(self) -> None:
        if self.on_title is not None:
            self.on_title(vd.title_for(self.datasets.values()))

    # ============================================================== files
    def choose_csv(self) -> None:
        names, _ = QtWidgets.QFileDialog.getOpenFileNames(
            self, "Open telemetry CSV(s)", "", CSV_FILTER)
        if names:
            self.open_files(names)

    def choose_add_csv(self) -> None:
        names, _ = QtWidgets.QFileDialog.getOpenFileNames(
            self, "Add telemetry CSV(s)", "", CSV_FILTER)
        if names:
            self.add_files(names)

    def choose_bin(self) -> None:
        """Open raw Witness logs (.bin): decoded in the background and added
        alongside the files already loaded."""
        names, _ = QtWidgets.QFileDialog.getOpenFileNames(
            self, "Open raw Witness log(s) (.bin)", "", BIN_FILTER)
        if names:
            self.add_files(names)

    def open_files(self, paths) -> None:
        """Replace everything with these files (read in the background)."""
        self.close_all()
        self.add_files(paths)

    def add_files(self, paths) -> None:
        """Read files on a worker thread; each is added when it finishes.

        A progress bar and Cancel button show while files load; the
        interface stays responsive however large they are.
        """
        if isinstance(paths, (str, Path)):
            paths = [paths]
        self._load_queue.extend(LoadRequest(path) for path in paths)
        if self._load_worker is None:
            self._start_next_load()

    def close_all(self) -> None:
        """Cancel loads and close every file."""
        self.cancel_loads()
        self.clear_datasets()

    @property
    def loading(self) -> bool:
        return self._load_worker is not None or bool(self._load_queue)

    def cancel_loads(self) -> None:
        """Stop the file being read and drop the queued ones."""
        self._load_queue.clear()
        self._load_cancel.set()

    def wait_for_loads(self, timeout: float = 60.0) -> bool:
        """Pump Qt until queued loads finish (tests and scripts)."""
        return qt_common.process_events_until(lambda: not self.loading, timeout)

    def _start_next_load(self) -> None:
        if self._closing:
            return
        if not self._load_queue:
            self._load_request = None
            self._show_load_progress(None)
            if self._save_worker is None:
                self._background_busy(False)
            if self._load_errors:
                errors, self._load_errors = self._load_errors, []
                self._message("error", "Open CSV", "Could not open:\n\n" + "\n\n".join(errors))
            return
        request = self._load_queue.pop(0)
        self._load_request = request
        self._load_cancel = cancel = threading.Event()
        hex_numbers = self.hex_check.isChecked()
        budget = self.memory_budget

        def work(progress):
            return vd.load_dataset(request.path, progress=progress, cancel=cancel,
                                   hex_as_numbers=hex_numbers, columns=request.columns,
                                   memory_budget=budget, thin=request.thin)

        self._background_busy(True)
        self._show_load_progress(request.path, 0.0)
        self._load_worker = run_in_thread(
            work, on_done=self._load_done, on_error=self._load_failed,
            on_progress=lambda fraction, p=request.path: self._show_load_progress(p, fraction))

    def _load_done(self, dataset) -> None:
        self._load_worker = None
        if self._closing:
            return
        self._add_loaded(dataset)
        self._start_next_load()

    def _load_failed(self, exc) -> None:
        self._load_worker = None
        request = self._load_request
        if self._closing:
            return
        if isinstance(exc, csv_columns.LoadCancelled):
            pass
        elif isinstance(exc, MemoryBudgetError):
            self.messages.append(("memory", "Open CSV", str(exc)))
            self.memory_prompt(exc, request)
        else:
            text = str(exc) if isinstance(exc, (OSError, ValueError, MemoryError)) \
                else f"{type(exc).__name__}: {exc}"
            path = request.path if request is not None else "?"
            self._load_errors.append(f"{path}:\n{text}")
        self._start_next_load()

    def memory_prompt(self, error: MemoryBudgetError, request: LoadRequest | None) -> None:
        """A file is over the memory budget: offer columns (or thinning)."""
        if not self.show_dialogs:
            self._load_errors.append(f"{error.path}:\n{error}")
            return
        dialog = ColumnPickerDialog(error, self)
        dialog.setAttribute(QtCore.Qt.WidgetAttribute.WA_DeleteOnClose)

        def chosen(_result=None):
            if dialog.result() != QtWidgets.QDialog.DialogCode.Accepted or dialog.choice is None:
                return
            kind, columns = dialog.choice
            self._load_queue.append(LoadRequest(error.path, columns=columns,
                                                thin=kind == "thin"))
            if self._load_worker is None:
                self._start_next_load()

        dialog.finished.connect(chosen)
        dialog.open()

    def _show_load_progress(self, path, fraction: float = 0.0, verb: str = "Loading") -> None:
        widgets = (self.load_label, self.load_bar, self.load_cancel_button)
        if path is None:
            if self._save_worker is None:
                for widget in widgets:
                    widget.hide()
            return
        for widget in widgets:
            widget.show()
        pending = len(self._load_queue)
        self.load_label.setText(f"{verb} {Path(path).name}" + (f" (+{pending})" if pending else ""))
        try:
            self.load_bar.setValue(int(max(0.0, min(1.0, float(fraction))) * 1000))
        except (TypeError, ValueError):
            pass

    def add_csv(self, path, report: bool = True) -> bool:
        """Read one file now, on this thread, and add it (scripts and tests;
        the interface itself always loads in the background)."""
        try:
            dataset = vd.load_dataset(path, hex_as_numbers=self.hex_check.isChecked(),
                                      memory_budget=self.memory_budget)
        except (OSError, ValueError) as exc:
            if not report:
                raise
            self._message("error", "Open CSV", f"Could not open {path}:\n{exc}")
            return False
        self._add_loaded(dataset)
        return True

    def load_csv(self, path) -> bool:
        """Replace everything with this one file (blocking; see add_csv)."""
        self.close_all()
        return self.add_csv(path)

    def _add_loaded(self, dataset: Dataset) -> None:
        dataset.label = unique_label(dataset.path.name,
                                     (d.label for d in self.datasets.values()))
        dataset_id = self._next_dataset_id
        self._next_dataset_id += 1
        first = not self.datasets
        self.datasets[dataset_id] = dataset
        self._insert_tree(dataset_id, dataset)
        self._refresh_axis_choices(keep=not first)
        if first and not self.layout_model.graphs:
            self.plot_dataset(dataset_id)
        self._refresh_summary_choices(select=dataset_id)
        if self.sync_transition is not None:
            self.sync_logs(self.sync_transition, report=False)
        if first:
            self.notebook.setCurrentWidget(self.summary_tab)
            self.graph_scroll.verticalScrollBar().setValue(0)
        self.rebuild_graphs()
        self.refresh_all()

    def clear_datasets(self) -> None:
        self.datasets.clear()
        self.x_view = None
        self.layout_model.clear()
        self._clear_docks()
        self._drop_cached()
        self.trace_tree.clear()
        self._refresh_axis_choices(keep=False)
        self._refresh_summary_choices()
        self.rebuild_graphs()
        self.refresh_all()

    def remove_dataset(self, dataset_id: int) -> None:
        if dataset_id not in self.datasets:
            return
        for window in list(self._spectrum_windows):
            if window.ref[0] == dataset_id:
                window.close()
        del self.datasets[dataset_id]
        self.layout_model.remove_dataset(dataset_id)
        for dock in [d for d in self.docks if d.ref[0] == dataset_id]:
            self.remove_board_spectrogram(dock, rebuild=False)
        self._drop_cached(dataset_id)
        item = self._tree_item(dataset_id)
        if item is not None:
            self.trace_tree.takeTopLevelItem(self.trace_tree.indexOfTopLevelItem(item))
        self._refresh_axis_choices(keep=True)
        self._refresh_summary_choices()
        self.rebuild_graphs()
        self.refresh_all()

    def refresh_all(self) -> None:
        self.file_label.setText(vd.file_label_text(self.datasets.values()))
        self._sync_time_axis_controls()
        self._update_status()
        self._set_title()

    def remove_selected_dataset(self) -> None:
        dataset_id = self._selected_dataset_id()
        if dataset_id is not None:
            self.remove_dataset(dataset_id)

    def plot_dataset(self, dataset_id: int) -> None:
        """One graph per numeric column of a CSV (the classic view)."""
        dataset = self.datasets.get(dataset_id)
        if dataset is not None:
            self.layout_model.add([(dataset_id, column) for column in
                                   vd.default_plot_columns(dataset, self.x_column())])

    def plot_selected_dataset(self) -> None:
        dataset_id = self._selected_dataset_id()
        if dataset_id is not None:
            self.plot_dataset(dataset_id)
            self.rebuild_graphs()

    def clear_graphs(self) -> None:
        self.layout_model.clear()
        self.rebuild_graphs()

    def save_selected_as_csv(self) -> None:
        """Write the selected decoded .bin as a CSV (converter.py output)."""
        dataset = self.datasets.get(self._selected_dataset_id())
        if dataset is None:
            return
        if not vd.is_bin_path(dataset.path):
            self._message("info", "Save CSV", f"{dataset.label} is already a CSV file.")
            return
        name, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save decoded log as CSV", str(dataset.path.with_suffix(".csv")),
            "CSV files (*.csv)")
        if name:
            self.save_bin_as_csv(dataset.path, Path(name))

    def save_bin_as_csv(self, source: Path, target: Path) -> None:
        """Decode ``source`` again and stream it to ``target`` (worker thread)."""
        def done(message):
            self._save_worker = None
            self.last_save_message = message
            self._save_finished()
            self._message("info", "Save CSV", message)

        def failed(exc):
            self._save_worker = None
            self.last_save_message = f"Could not save {target}:\n{exc}"
            self._save_finished()
            self._message("error", "Save CSV", self.last_save_message)

        self._background_busy(True)
        self._show_load_progress(target, 0.0, verb="Saving")
        self.load_bar.setRange(0, 0)          # busy indicator
        self._save_worker = run_in_thread(vd.save_bin_as_csv, Path(source), Path(target),
                                          on_done=done, on_error=failed)

    def _save_finished(self) -> None:
        self.load_bar.setRange(0, 1000)
        if self._load_worker is None:
            self._show_load_progress(None)
            self._background_busy(False)

    # -------------------------------------------------- drag & drop files
    def dragEnterEvent(self, event) -> None:  # noqa: N802
        if _local_paths(event.mimeData()):
            event.acceptProposedAction()
        else:
            event.ignore()

    def dragMoveEvent(self, event) -> None:  # noqa: N802
        if _local_paths(event.mimeData()):
            event.acceptProposedAction()
        else:
            event.ignore()

    def dropEvent(self, event) -> None:  # noqa: N802
        paths = _local_paths(event.mimeData())
        if not paths:
            event.ignore()
            return
        event.acceptProposedAction()
        self.add_files(paths)

    # ======================================================= trace list
    def _insert_tree(self, dataset_id: int, dataset: Dataset) -> None:
        node = QtWidgets.QTreeWidgetItem([vd.dataset_text(dataset)])
        node.setData(0, QtCore.Qt.ItemDataRole.UserRole, ("dataset", dataset_id))
        node.setFlags(node.flags() & ~QtCore.Qt.ItemFlag.ItemIsDragEnabled
                      & ~QtCore.Qt.ItemFlag.ItemIsDropEnabled)
        for column in dataset.numeric_columns:
            child = QtWidgets.QTreeWidgetItem([column])
            child.setData(0, QtCore.Qt.ItemDataRole.UserRole, ("trace", dataset_id, column))
            child.setFlags(child.flags() & ~QtCore.Qt.ItemFlag.ItemIsDropEnabled)
            node.addChild(child)
        self.trace_tree.addTopLevelItem(node)
        node.setExpanded(True)

    def _tree_item(self, dataset_id: int) -> QtWidgets.QTreeWidgetItem | None:
        for index in range(self.trace_tree.topLevelItemCount()):
            item = self.trace_tree.topLevelItem(index)
            data = item.data(0, QtCore.Qt.ItemDataRole.UserRole)
            if data and data[1] == dataset_id:
                return item
        return None

    def _item_ref(self, item):
        if item is None:
            return None
        data = item.data(0, QtCore.Qt.ItemDataRole.UserRole)
        if data and data[0] == "trace" and data[1] in self.datasets:
            return (data[1], data[2])
        return None

    def select_dataset(self, dataset_id: int) -> None:
        item = self._tree_item(dataset_id)
        if item is not None:
            self.trace_tree.setCurrentItem(item)
            self.trace_tree.clearSelection()
            item.setSelected(True)

    def _selected_dataset_id(self) -> int | None:
        items = self.trace_tree.selectedItems() or [self.trace_tree.currentItem()]
        for item in items:
            if item is None:
                continue
            data = item.data(0, QtCore.Qt.ItemDataRole.UserRole)
            if data and data[1] in self.datasets:
                return data[1]
        return next(iter(self.datasets), None)

    def _selected_ref(self):
        for item in self.trace_tree.selectedItems() or [self.trace_tree.currentItem()]:
            ref = self._item_ref(item)
            if ref is not None:
                return ref
        return None

    def _tree_double(self, item, _column=0) -> None:
        ref = self._item_ref(item)
        if ref is not None:
            self.layout_model.add([ref])
            self.rebuild_graphs()
            QtCore.QTimer.singleShot(0, lambda: self.graph_scroll.verticalScrollBar().setValue(
                self.graph_scroll.verticalScrollBar().maximum()))

    def _tree_menu(self, pos) -> None:
        item = self.trace_tree.itemAt(pos)
        if item is None:
            return
        if not item.isSelected():
            self.trace_tree.clearSelection()
            item.setSelected(True)
        menu = QtWidgets.QMenu(self)
        ref = self._item_ref(item)
        if ref is not None:
            menu.addAction("Spectrogram / FFT…", lambda: self.open_spectrogram(ref))
            menu.addAction("Add spectrogram to board",
                           lambda: self.add_board_spectrogram(ref))
            menu.addAction("Add graph", lambda: (self.layout_model.add([ref]),
                                                 self.rebuild_graphs()))
        else:
            menu.addAction("Plot CSV", self.plot_selected_dataset)
            menu.addAction("Remove CSV", self.remove_selected_dataset)
        menu.popup(self.trace_tree.viewport().mapToGlobal(pos))

    def _refresh_axis_choices(self, keep: bool) -> None:
        choices = vd.axis_choices(self.datasets.values())
        current = self.x_axis_combo.currentText()
        self.x_axis_combo.blockSignals(True)
        self.x_axis_combo.clear()
        self.x_axis_combo.addItems(choices)
        self.x_axis_combo.setCurrentText(current if keep and current in choices
                                         else vd.preferred_axis(choices))
        self.x_axis_combo.blockSignals(False)
        packet_types = sorted({kind for dataset in self.datasets.values()
                               for kind in dataset.packet_types})
        current = self.packet_filter_combo.currentText()
        self.packet_filter_combo.blockSignals(True)
        self.packet_filter_combo.clear()
        self.packet_filter_combo.addItems(["All", *packet_types])
        self.packet_filter_combo.setCurrentText(current if current in packet_types else "All")
        self.packet_filter_combo.blockSignals(False)

    def x_column(self) -> str:
        return self.x_axis_combo.currentText()

    def packet_filter(self) -> str:
        return self.packet_filter_combo.currentText() or "All"

    def on_axis_changed(self) -> None:
        """A new x axis or packet filter: rebuild every trace."""
        self._drop_cached()
        self.x_view = None
        self.rebuild_graphs()
        self._refresh_docks()

    # ===================================================== per-file time
    def set_time_axis(self, dataset_id: int, column: str | None = None,
                      unit: str | None = None, offset: float | None = None) -> None:
        """Change one CSV's time column, units and/or offset (seconds).

        An offset only moves that file's curves (curve.shift); a new column
        or unit rebuilds that file's x array once. Raises ValueError for an
        unknown column or unit.
        """
        dataset = self.datasets.get(dataset_id)
        if dataset is None:
            return
        before = (dataset.time_column, dataset.time_unit)
        dataset.set_time_axis(column, unit, offset)
        x_column = self.x_column()
        if (dataset.time_column, dataset.time_unit) != before:
            self._drop_cached(dataset_id)
            if x_column == SYNC_X:
                for slot in self.live_slots():
                    for ref, curve in slot.curves:
                        if ref[0] == dataset_id:
                            xs, ys = self.series(ref)
                            curve.setData(xs, ys)
        for slot in self.live_slots():
            for ref, curve in slot.curves:
                if ref[0] == dataset_id:
                    curve.shift(vd.x_offset(dataset, x_column))
        item = self._tree_item(dataset_id)
        if item is not None:
            item.setText(0, vd.dataset_text(dataset))
        self._update_full_x()
        if self._selected_dataset_id() == dataset_id:
            self._sync_time_axis_controls()
        self._update_status()
        self._refresh_docks(dataset_id)

    def _sync_choice_changed(self) -> None:
        pair = self.sync_combo.currentData()
        self.sync_logs(tuple(pair) if pair else None)

    def sync_logs(self, transition, report: bool = True) -> dict:
        """Slide every open log so its first rise into a flight state is t = 0.

        ``transition`` is (from state, to state) as in vd.SYNC_TRANSITIONS, or
        None to stop syncing. Each log's per-file offset is set (so it shows
        in, and can be edited from, the Time axis box); logs where the
        transition is not found keep their offset and are reported. Returns
        {"synced": [labels], "skipped": [labels]}.
        """
        result = {"synced": [], "skipped": []}
        label = vd.sync_label(transition) if transition else None
        self.sync_transition = tuple(transition) if label else None
        self.sync_combo.setCurrentIndex(self.sync_combo.findText(label) if label else 0)
        if not label:
            return result
        times = vd.find_sync_times(self.datasets, *self.sync_transition)
        for dataset_id in sorted(self.datasets):
            dataset = self.datasets[dataset_id]
            moment = times.get(dataset_id)
            if moment is None:
                result["skipped"].append(dataset.label)
                continue
            self.set_time_axis(dataset_id, offset=round(-float(moment), 9))
            result["synced"].append(dataset.label)
        if result["skipped"] and report:
            self._message("warning", "Sync logs",
                          f"No {label} found in: " + ", ".join(result["skipped"])
                          + ". Their offsets were left as they were.")
        return result

    def apply_offset(self) -> None:
        """Use the offset typed (or spun) for the selected CSV."""
        if self._syncing:
            return
        dataset_id = self._selected_dataset_id()
        dataset = self.datasets.get(dataset_id)
        if dataset is None:
            return
        offset = round(float(self.offset_spin.value()), 9)
        if not math.isfinite(offset) or offset == dataset.offset:
            return
        self.set_time_axis(dataset_id, offset=offset)

    def nudge_offset(self, delta: float) -> None:
        dataset_id = self._selected_dataset_id()
        dataset = self.datasets.get(dataset_id)
        if dataset is not None:
            self.set_time_axis(dataset_id, offset=round(dataset.offset + delta, 9))

    def _time_controls_changed(self) -> None:
        if self._syncing:
            return
        dataset_id = self._selected_dataset_id()
        dataset = self.datasets.get(dataset_id)
        if dataset is None:
            return
        column = self.time_column_combo.currentText()
        unit = self.time_unit_combo.currentText()
        if column and column != dataset.time_column:
            self.set_time_axis(dataset_id, column=column)   # brings the units its name implies
        elif unit in TIME_UNITS and unit != dataset.time_unit and column != AUTO_TIME:
            self.set_time_axis(dataset_id, unit=unit)

    def _sync_time_axis_controls(self) -> None:
        """Show the selected CSV's time axis in the controls."""
        self._syncing = True
        try:
            dataset = self.datasets.get(self._selected_dataset_id())
            if dataset is None:
                self.time_axis_box.setTitle("Time axis")
                self.time_column_combo.clear()
                self.time_unit_combo.setCurrentIndex(-1)
                self.offset_spin.setValue(0.0)
                self.time_axis_note.setText("")
                return
            self.time_axis_box.setTitle(self.fontMetrics().elidedText(
                f"Time axis: {dataset.label}", QtCore.Qt.TextElideMode.ElideMiddle, 230))
            choices = dataset.time_choices()
            if [self.time_column_combo.itemText(i)
                    for i in range(self.time_column_combo.count())] != choices:
                self.time_column_combo.clear()
                self.time_column_combo.addItems(choices)
            self.time_column_combo.setCurrentText(dataset.time_column)
            auto = dataset.time_column == AUTO_TIME
            self.time_unit_combo.setCurrentText("s" if auto else dataset.time_unit)
            self.time_unit_combo.setEnabled(not auto)
            self.offset_spin.setValue(dataset.offset)
            self.time_axis_note.setText(vd.time_axis_note(dataset))
        finally:
            self._syncing = False

    # =========================================================== summary
    def _refresh_summary_choices(self, select: int | None = None) -> None:
        current = self.summary_combo.currentData()
        self.summary_combo.blockSignals(True)
        self.summary_combo.clear()
        for dataset_id, dataset in self.datasets.items():
            self.summary_combo.addItem(dataset.label, dataset_id)
        target = select if select in self.datasets else current
        index = self.summary_combo.findData(target) if target is not None else -1
        self.summary_combo.setCurrentIndex(index if index >= 0 else
                                           (0 if self.datasets else -1))
        self.summary_combo.blockSignals(False)
        self.show_summary()

    def show_summary(self) -> None:
        dataset = self.datasets.get(self.summary_combo.currentData())
        if dataset is None:
            for label in self.summary_values.values():
                label.setText("--")
            self.summary_details_label.setText("Open a CSV file to begin.")
            return
        for key, label in self.summary_values.items():
            label.setText(dataset.summary[key])
        self.summary_details_label.setText(vd.summary_details(dataset))

    # ============================================================ traces
    def _order(self, dataset_id: int, dataset: Dataset):
        key = (dataset_id, self.x_column(), self.packet_filter())
        order = self._order_cache.get(key)
        if order is None:
            order = vd.row_order(dataset, key[1], key[2])
            self._order_cache[key] = order
        return order

    def series(self, ref) -> tuple[np.ndarray, np.ndarray]:
        """One trace's finite, x-sorted (x, y) arrays, before the file's offset."""
        dataset_id, column = ref
        key = (dataset_id, self.x_column(), self.packet_filter(), column)
        cached = self._series_cache.get(key)
        if cached is not None:
            self._series_cache.move_to_end(key)
            return cached
        dataset = self.datasets.get(dataset_id)
        if dataset is None:
            return np.empty(0), np.empty(0)
        xs, ys = vd.series_arrays(dataset, column, self._order(dataset_id, dataset))
        nbytes = sum(a.nbytes for a in (xs, ys)
                     if a is not dataset.numeric_data.get(column) and a is not dataset.times())
        self._series_cache[key] = (xs, ys)
        self._cache_bytes += nbytes
        while self._cache_bytes > SERIES_CACHE_BYTES and len(self._series_cache) > 1:
            old_key, (old_x, old_y) = self._series_cache.popitem(last=False)
            self._cache_bytes -= old_x.nbytes + old_y.nbytes
        self._cache_bytes = max(0, self._cache_bytes)
        return xs, ys

    def series_points(self, ref) -> Series:
        """One trace on the shared x axis (offset applied; a copy, for scripts)."""
        xs, ys = self.series(ref)
        dataset = self.datasets.get(ref[0])
        offset = vd.x_offset(dataset, self.x_column()) if dataset is not None else 0.0
        return Series(xs + offset, ys)

    def _drop_cached(self, dataset_id: int | None = None) -> None:
        if dataset_id is None:
            self._series_cache.clear()
            self._order_cache.clear()
            self._cache_bytes = 0
            return
        for key in [k for k in self._series_cache if k[0] == dataset_id]:
            self._series_cache.pop(key)
        for key in [k for k in self._order_cache if k[0] == dataset_id]:
            self._order_cache.pop(key)

    def series_label(self, ref) -> str:
        dataset = self.datasets.get(ref[0])
        if dataset is None or len(self.datasets) == 1:
            return ref[1]
        return f"{dataset.label}: {ref[1]}"

    @staticmethod
    def series_style(graph_index: int, series_index: int, count: int, ref):
        """(colour, pen style): per-graph colour when alone, else one colour
        per trace and one dash pattern per CSV."""
        if count == 1:
            return PLOT_COLORS[graph_index % len(PLOT_COLORS)], PEN_STYLES[0]
        return (PLOT_COLORS[series_index % len(PLOT_COLORS)],
                PEN_STYLES[ref[0] % len(PEN_STYLES)])

    # ============================================================ graphs
    def live_slots(self) -> list[GraphSlot]:
        return [slot for slot in self.slots if slot.plot is not None]

    def rebuild_graphs(self) -> None:
        """Recreate the graph stack from the layout (plots are built lazily)."""
        self._clear_hover()
        for slot in self.slots:
            slot.dispose()
            self.graph_layout.removeWidget(slot)
            slot.deleteLater()
        self.slots = []
        self._link_root = None
        self._update_full_x(apply=False)
        for index, refs in enumerate(self.layout_model.graphs):
            self.slots.append(GraphSlot(self, index, refs, self._graph_height()))
        self._place_slots()
        self._apply_heights()
        self.drop_zone.set_empty(not self.slots and not self.docks)
        self._pad_docks()
        self._update_status()
        self._schedule_realize()

    def _graph_height(self) -> int:
        """Automatic height: rows (not graphs) share the view when tiled."""
        view_height = self.graph_scroll.viewport().height() if hasattr(self, "graph_scroll") else 0
        rows = -(-len(self.layout_model.graphs) // max(1, self.graph_columns))
        return graph_height_for(rows, view_height - DROP_ZONE_HEIGHT)

    def _slot_rows(self) -> list[list[GraphSlot]]:
        """Graph slots grouped into grid rows; a docked spectrogram ends a row."""
        columns = max(1, self.graph_columns)
        count = len(self.slots)
        cuts = sorted({min(max(dock.after, 0), count) for dock in self.docks} | {count})
        rows, start = [], 0
        for cut in cuts:
            for first in range(start, cut, columns):
                rows.append(self.slots[first:min(first + columns, cut)])
            start = cut
        return rows

    def _place_slots(self) -> None:
        """Put the graphs in the grid, ``graph_columns`` per row, the docked
        spectrograms between them, then the drop zone."""
        grid = self.graph_layout
        for widget in [*self.slots, *self.docks, self.drop_zone]:
            grid.removeWidget(widget)
        for row in range(grid.rowCount()):
            grid.setRowStretch(row, 0)
        columns = max(1, self.graph_columns)
        count = len(self.slots)
        docks = sorted(self.docks, key=lambda dock: dock.after)    # stable: ties keep order
        row, placed = 0, 0
        cuts = sorted({min(max(dock.after, 0), count) for dock in docks} | {count})
        for cut in cuts:
            for index in range(placed, cut):
                grid.addWidget(self.slots[index], row + (index - placed) // columns,
                               (index - placed) % columns)
            row += -(-(cut - placed) // columns)
            placed = cut
            for dock in docks:
                if min(max(dock.after, 0), count) == cut:
                    grid.addWidget(dock, row, 0, 1, columns)
                    row += 1
        grid.addWidget(self.drop_zone, row, 0, 1, columns)
        grid.setRowStretch(row + 1, 1)
        for column in range(MAX_COLUMNS):
            grid.setColumnStretch(column, 1 if column < columns else 0)

    def _slot_height(self, slot: "GraphSlot", auto: int) -> int:
        return self._height_overrides.get(tuple(slot.refs), auto)

    def _apply_heights(self) -> None:
        """Each row takes its tallest graph's height, so tiles line up."""
        auto = self._graph_height()
        for row in self._slot_rows():
            height = max(self._slot_height(slot, auto) for slot in row)
            for slot in row:
                if slot.height() != height:
                    slot.setFixedHeight(height)

    def set_columns(self, columns: int) -> None:
        """Tile graphs ``columns`` per row (1 to MAX_COLUMNS)."""
        columns = max(1, min(MAX_COLUMNS, int(columns)))
        if self.columns_spin.value() != columns:
            self.columns_spin.setValue(columns)     # re-enters through valueChanged
            return
        if columns == self.graph_columns and self.slots:
            return
        self.graph_columns = columns
        self._place_slots()
        self._apply_heights()
        self._schedule_realize()

    def set_graph_height(self, index: int, height: int | None) -> None:
        """Height of graph ``index`` (and its row when tiled); None = automatic."""
        if not 0 <= index < len(self.slots):
            return
        row = next((r for r in self._slot_rows() if self.slots[index] in r), [])
        for slot in row:
            key = tuple(slot.refs)
            if height is None:
                self._height_overrides.pop(key, None)
            else:
                self._height_overrides[key] = int(max(MIN_RESIZE_HEIGHT, min(MAX_RESIZE_HEIGHT, height)))
        self._apply_heights()
        self._schedule_realize()

    def reset_graph_heights(self) -> None:
        self._height_overrides.clear()
        self._apply_heights()
        self._schedule_realize()

    def _schedule_realize(self) -> None:
        if not self._closing and not self._realize_timer.isActive():
            self._realize_timer.start()

    def _visible_band(self) -> tuple[int, int]:
        top = self.graph_scroll.verticalScrollBar().value()
        height = max(1, self.graph_scroll.viewport().height())
        margin = int(height * LAZY_MARGIN)
        return top - margin, top + height + margin

    def _realize_visible(self) -> None:
        """Create the plots of graphs in (or near) view, a few per event-loop
        turn so the interface never stalls, and free far-away ones."""
        if self._closing or not self.slots:
            return
        top, bottom = self._visible_band()
        start = time.perf_counter()
        wanted = [slot for slot in self.slots
                  if slot.y() + slot.height() >= top and slot.y() <= bottom]
        for slot in wanted:
            if slot.plot is None:
                slot.realize()
                if time.perf_counter() - start > REALIZE_BUDGET_S:
                    self._realize_timer.start()
                    break
        live = self.live_slots()
        if len(live) > MAX_LIVE_PLOTS:
            center = (top + bottom) / 2
            far = sorted((slot for slot in live if slot not in wanted),
                         key=lambda slot: -abs(slot.y() + slot.height() / 2 - center))
            for slot in far[:len(live) - MAX_LIVE_PLOTS]:
                slot.dispose()

    def realize_visible_now(self) -> None:
        """Build every graph in view right away (tests)."""
        QtWidgets.QApplication.processEvents()
        for _ in range(len(self.slots) + 1):
            self._realize_visible()
            if not self._realize_timer.isActive():
                break
            self._realize_timer.stop()

    def eventFilter(self, obj, event) -> bool:  # noqa: N802
        kind = event.type()
        if obj is self.graph_scroll.viewport() and kind == QtCore.QEvent.Type.Resize:
            self._apply_heights()
            self._schedule_realize()
        elif kind == QtCore.QEvent.Type.Leave:
            self._clear_hover()
        return False

    def _attach_view(self, slot: GraphSlot) -> None:
        """Link a new plot's x axis to the others and give it the shared range."""
        vb = slot.vb
        self._apply_limits(vb)
        low, high = self.current_x_range()
        vb.setXRange(low, high, padding=0)
        if self._link_root is None:
            self._link_root = vb
        else:
            vb.setXLink(self._link_root)
        vb.sigXRangeChanged.connect(self._x_range_changed)
        self._relink_docks()

    def _detach_view(self, slot: GraphSlot) -> None:
        vb = slot.vb
        if vb is None:
            return
        try:
            vb.sigXRangeChanged.disconnect(self._x_range_changed)
        except (TypeError, RuntimeError):
            pass
        if vb is self._link_root:
            others = [s.vb for s in self.live_slots() if s is not slot and s.vb is not None]
            self._link_root = others[0] if others else None
            if self._link_root is not None:
                self._link_root.setXLink(None)
                for other in others[1:]:
                    other.setXLink(self._link_root)
        else:
            vb.setXLink(None)
        self._relink_docks()

    def _apply_limits(self, vb: TimeViewBox) -> None:
        full = self.current_x_range(full=True)
        span = full[1] - full[0]
        pad = span * 0.02
        # A floor on the zoom span: deep zoom never reaches float resolution.
        min_span = max(span * 1e-9, max(abs(full[0]), abs(full[1]), 1.0) * 1e-12)
        vb.setLimits(xMin=full[0] - pad, xMax=full[1] + pad, minXRange=min_span)

    def _update_full_x(self, apply: bool = True) -> None:
        """The union of every plotted trace's x range (offsets included)."""
        lows, highs = [], []
        x_column = self.x_column()
        for refs in self.layout_model.graphs:
            for ref in refs:
                xs, _ys = self.series(ref)
                if xs.size:
                    dataset = self.datasets.get(ref[0])
                    offset = vd.x_offset(dataset, x_column) if dataset else 0.0
                    lows.append(float(xs[0]) + offset)
                    highs.append(float(xs[-1]) + offset)
        self._full_x = (min(lows), max(highs)) if lows else None
        if self.x_view is not None and self._full_x is not None:
            low = max(self.x_view[0], self._full_x[0])
            high = min(self.x_view[1], self._full_x[1])
            self.x_view = (low, high) if high > low else None
        elif self._full_x is None:
            self.x_view = None
        if apply:
            for slot in self.live_slots():
                self._apply_limits(slot.vb)
            self._apply_x_range()

    def current_x_range(self, full: bool = False) -> tuple[float, float]:
        """The x range every graph shows: the zoom, else all plotted data."""
        x_range = self._full_x if full else (self.x_view or self._full_x)
        if x_range is None:
            return 0.0, 1.0
        low, high = x_range
        if high <= low:
            low, high = low - 0.5, high + 0.5
        return float(low), float(high)

    def x_range(self) -> tuple[float, float] | None:
        return self.x_view or self._full_x

    def _apply_x_range(self) -> None:
        root = self._link_root
        if root is not None:
            low, high = self.current_x_range()
            root.setXRange(low, high, padding=0)      # linked plots follow
        self._view_throttle.request()

    def _x_range_changed(self, vb, x_range) -> None:
        """A plot's x range moved (zoom, pan, link): remember it as the zoom."""
        if self._closing or vb is not self._link_root:
            return
        low, high = float(x_range[0]), float(x_range[1])
        full = self._full_x
        if full is None:
            return
        tolerance = (full[1] - full[0]) * 1e-6
        if low <= full[0] + tolerance and high >= full[1] - tolerance:
            self.x_view = None
        else:
            self.x_view = (max(low, full[0]), min(high, full[1]))
        self._view_throttle.request()

    def _view_changed_tick(self) -> None:
        self._update_status()
        for slot in self.live_slots():
            slot.update_stats()

    def set_x_view(self, view: tuple[float, float] | None) -> None:
        """Zoom every graph to x in [low, high]; None shows everything."""
        full = self._full_x
        if view is not None and full is not None:
            low, high = sorted(view)
            low, high = max(low, full[0]), min(high, full[1])
            if high - low <= 0 or (low <= full[0] and high >= full[1]):
                view = None
            else:
                view = (low, high)
        self.x_view = view
        self._apply_x_range()

    def zoom(self, factor: float, center: float | None = None) -> None:
        full = self._full_x
        if full is None:
            return
        view = self.x_view or full
        if center is None:
            center = (view[0] + view[1]) / 2
        self.set_x_view(vd.zoom_range(view, full, center, factor))

    def queue_wheel(self, zoom_steps: float = 0.0, pan_steps: float = 0.0,
                    center: float | None = None) -> None:
        """Accumulate wheel input; _apply_wheel applies it once per frame."""
        self._wheel_zoom += zoom_steps
        self._wheel_pan += pan_steps
        if center is not None:
            self._wheel_center = center
        self._wheel_throttle.request()

    def _apply_wheel(self) -> None:
        zoom, pan, center = self._wheel_zoom, self._wheel_pan, self._wheel_center
        self._wheel_zoom = self._wheel_pan = 0.0
        self._wheel_center = None
        full = self._full_x
        if full is None:
            return
        if zoom:
            # 120 units = one notch = ZOOM_FACTOR; touchpad fractions scale.
            factor = ZOOM_FACTOR ** max(-20.0, min(20.0, zoom))
            view = self.x_view or full
            if center is None:
                center = (view[0] + view[1]) / 2
            self.set_x_view(vd.zoom_range(view, full, center, factor))
        if pan and self.x_view is not None:
            view = self.x_view
            step = (view[1] - view[0]) * PAN_FRACTION * pan
            self.set_x_view(vd.pan_range(view, full, step))

    def drag_pan(self, press_x: float, here_x: float, finished: bool) -> None:
        """Shift/middle-drag pan: applied when the mouse pauses or is released."""
        if self._drag_pan is None:
            if self._full_x is None or self.x_view is None:
                return
            self._drag_pan = (self.x_view, press_x, here_x)
        view, start, _ = self._drag_pan
        self._drag_pan = (view, start, here_x)
        if finished:
            self._pan_settle.stop()
            self._apply_drag_pan()
            self._drag_pan = None
        else:
            self._pan_settle.request()

    def _apply_drag_pan(self) -> None:
        if self._drag_pan is None or self._full_x is None:
            return
        view, start, here = self._drag_pan
        # The view is moved by the drag distance measured in the press-time view.
        self.set_x_view(vd.pan_range(view, self._full_x, start - here))

    def flush_wheel(self) -> None:
        """Apply queued wheel input now (tests)."""
        self._wheel_throttle.flush()

    def zoom_out(self) -> None:
        self.zoom(2.0)

    def reset_zoom(self) -> None:
        self.set_x_view(None)

    def pan(self, direction: int) -> None:
        view, full = self.x_view, self._full_x
        if view is not None and full is not None:
            step = (view[1] - view[0]) * PAN_FRACTION * direction
            self.set_x_view(vd.pan_range(view, full, step))

    def _update_status(self) -> None:
        if not self.datasets:
            self.summary_label.setText("")
            return
        zoom_text = ""
        if self.x_view is not None:
            zoom_text = (f" | zoom {value_text(round(self.x_view[0], 6))} to "
                         f"{value_text(round(self.x_view[1], 6))}")
        graphs = self.layout_model.graphs
        self.summary_label.setText(
            f"{len(self.datasets)} CSV(s) | {len(graphs)} graph(s), "
            f"{sum(len(refs) for refs in graphs)} trace(s) | x = {self.x_column()}"
            + zoom_text)

    def remove_graph(self, index: int) -> None:
        self.layout_model.remove_graph(index)
        for dock in self.docks:
            if dock.after > index:
                dock.after -= 1
        QtCore.QTimer.singleShot(0, self.rebuild_graphs)

    def drop(self, payload, target) -> None:
        """Apply a finished drag (also used by tests)."""
        if target is None or payload is None:
            return
        if payload[0] == "traces":
            refs = [ref for ref in payload[1] if ref[0] in self.datasets]
            if target[0] == "graph":
                self.layout_model.add(refs, target[1], separate=self.overlay_own_axes,
                                      axis=target[2] if len(target) > 2 else None)
            elif target[0] == "new":
                self.layout_model.add(refs)
            else:
                return
        elif payload[0] == "legend":
            source, ref = payload[1], payload[2]
            if target[0] == "graph":
                self.layout_model.move(source, ref, target[1], separate=self.overlay_own_axes,
                                       axis=target[2] if len(target) > 2 else None)
            elif target[0] == "new":
                self.layout_model.move(source, ref, None)
            elif target[0] == "list":
                self.layout_model.remove_series(source, ref)
        elif payload[0] == "axis":
            if target[0] == "graph":
                self.layout_model.move_axis(payload[1], payload[2], target[1],
                                            target[2] if len(target) > 2 else None)
            elif target[0] == "new":
                groups = self.layout_model.graphs[payload[1]].axis_groups() \
                    if payload[1] < len(self.layout_model.graphs) else []
                if payload[2] < len(groups):
                    for ref in list(groups[payload[2]].refs):
                        self.layout_model.move(payload[1], ref, None)
            else:
                return
        self.rebuild_graphs()

    # ============================================================= hover
    def _hover_request(self, slot: GraphSlot, pos) -> None:
        self._hover_pending = (slot, QtCore.QPointF(pos))
        self._hover_throttle.request()

    def _hover_tick(self) -> None:
        pending, self._hover_pending = self._hover_pending, None
        if pending is None:
            return
        slot, pos = pending
        if slot.plot is None or slot not in self.slots:
            return
        x = slot.x_at_scene(pos)
        if x is None:
            self._clear_hover()
            return
        self.show_hover(slot.index, x)

    def readout_rows(self, graph_index: int, x_value: float) -> list[tuple[str, str]]:
        """(text, colour) rows: the cursor x, then each trace's nearest sample,
        then text columns of generic CSVs at the nearest row."""
        refs = self.layout_model.graphs[graph_index]
        x_column = self.x_column()
        rows = [(f"{x_column}: {x_value:.6g}", "#222222")]
        for series_index, ref in enumerate(refs):
            xs, ys = self.series(ref)
            dataset = self.datasets.get(ref[0])
            offset = vd.x_offset(dataset, x_column) if dataset else 0.0
            # A log that has ended (or not started) here is left out rather
            # than showing its last sample from somewhere else.
            if not xs.size or not xs[0] + offset <= x_value <= xs[-1] + offset:
                continue
            index = vd.nearest_index(xs, x_value - offset)
            color, _style = self.series_style(graph_index, series_index, len(refs), ref)
            rows.append((f"{self.series_label(ref)} = "
                         f"{vd.cell_text(dataset, ref[1], float(ys[index]))}"
                         f"  @ {value_text(float(xs[index]) + offset)}", color))
        for dataset_id in dict.fromkeys(ref[0] for ref in refs):
            dataset = self.datasets.get(dataset_id)
            if dataset is None or not dataset.text_columns:
                continue
            row = self.nearest_row(dataset_id, x_value)
            if row is None:
                continue
            for name, column in list(dataset.text_columns.items())[:TEXT_READOUT_COLUMNS]:
                value = column[row]
                if value:
                    prefix = "" if len(self.datasets) == 1 else f"{dataset.label}: "
                    rows.append((f"{prefix}{name}: {value}", "#666666"))
        return rows

    def hover_readout(self, graph_index: int, x_value: float) -> list[str]:
        """Text of the hover box for a graph at an x value."""
        return [text for text, _color in self.readout_rows(graph_index, x_value)]

    def nearest_row(self, dataset_id: int, x_value: float) -> int | None:
        """Row of a CSV nearest to ``x_value`` on the current x axis."""
        dataset = self.datasets.get(dataset_id)
        if dataset is None:
            return None
        xs, rows = self._order(dataset_id, dataset)
        index = vd.nearest_index(xs, x_value - vd.x_offset(dataset, self.x_column()))
        if index is None:
            return None
        return int(index if rows is None else rows[index])

    def show_hover(self, graph_index: int, x_value: float) -> None:
        """Crosshair + nearest samples on every live graph, readout box on one."""
        x_column = self.x_column()
        for slot in self.live_slots():
            slot.hover_line.setPos(x_value)
            slot.hover_line.show()
            spots = []
            for series_index, (ref, _curve) in enumerate(slot.curves):
                xs, ys = self.series(ref)
                dataset = self.datasets.get(ref[0])
                offset = vd.x_offset(dataset, x_column) if dataset else 0.0
                if not xs.size or not xs[0] + offset <= x_value <= xs[-1] + offset:
                    continue
                index = vd.nearest_index(xs, x_value - offset)
                color, _style = self.series_style(slot.index, series_index,
                                                  len(slot.refs), ref)
                y = float(ys[index])
                aux = slot.aux_views.get(ref)
                if aux is not None:
                    y = _map_between(aux, slot.vb, y)
                spots.append({"pos": (float(xs[index]) + offset, y),
                              "pen": pg.mkPen(color, width=2)})
            slot.hover_marks.setData(spots)
            slot.hover_marks.show()
            if slot.index == graph_index:
                rows = self.readout_rows(graph_index, x_value)
                html = "<br>".join(
                    f'<span style="color:{color}">{_html_escape(text)}</span>'
                    for text, color in rows)
                slot.hover_text.setHtml(f'<div style="font-size:9pt">{html}</div>')
                (low, high), (y_low, y_high) = slot.vb.viewRange()
                right_side = x_value > (low + high) / 2
                slot.hover_text.setAnchor((1.05, 0) if right_side else (-0.05, 0))
                slot.hover_text.setPos(x_value, y_high)
                slot.hover_text.show()
            else:
                slot.hover_text.hide()
        self._hover_visible = True

    def _clear_hover(self) -> None:
        if not self._hover_visible:
            return
        self._hover_visible = False
        for slot in self.live_slots():
            for item in (slot.hover_line, slot.hover_marks, slot.hover_text):
                item.hide()

    clear_hover = _clear_hover

    # ===================================================== separate axes
    def _own_axes_default_changed(self, on: bool) -> None:
        self.overlay_own_axes = bool(on)

    def set_separate_axes(self, index: int, separate: bool = True) -> None:
        """Draw each trace of graph ``index`` on its own autoranging y axis (or
        back on one shared axis)."""
        if not 0 <= index < len(self.layout_model.graphs) \
                or self.layout_model.is_separate(index) == bool(separate):
            return
        self.layout_model.set_separate(index, separate)
        QtCore.QTimer.singleShot(0, self.rebuild_graphs)

    def axis_menu(self, graph_index: int, number: int, global_pos) -> None:
        """Right-click menu of a y axis."""
        if not 0 <= graph_index < len(self.layout_model.graphs):
            return
        graph = self.layout_model.graphs[graph_index]
        groups = graph.axis_groups()
        if not 0 <= number < len(groups):
            return
        menu = QtWidgets.QMenu(self)
        other = "right" if groups[number].side == "left" else "left"
        menu.addAction(f"Move axis to {other}",
                       lambda: self.move_axis_side(graph_index, number, other))
        for target, group in enumerate(groups):
            if target != number:
                names = ", ".join(self.series_label(ref) for ref in group.refs)
                menu.addAction(f"Merge into the {group.side} axis: {names}"[:90],
                               lambda t=target: self.merge_axes(graph_index, number, t))
        if len(groups[number].refs) > 1:
            menu.addAction("Split into one axis per trace",
                           lambda: self.split_axis(graph_index, number))
        menu.popup(global_pos)

    def _graph_changed(self) -> None:
        QtCore.QTimer.singleShot(0, self.rebuild_graphs)

    def move_axis_side(self, graph_index: int, number: int, side: str) -> None:
        if 0 <= graph_index < len(self.layout_model.graphs):
            self.layout_model.graphs[graph_index].move_group(number, side)
            self._graph_changed()

    def merge_axes(self, graph_index: int, source: int, target: int) -> None:
        if 0 <= graph_index < len(self.layout_model.graphs):
            self.layout_model.graphs[graph_index].merge(source, target)
            self._graph_changed()

    def split_axis(self, graph_index: int, number: int) -> None:
        """Give each trace of an axis its own axis (on that axis' side)."""
        if not 0 <= graph_index < len(self.layout_model.graphs):
            return
        graph = self.layout_model.graphs[graph_index]
        groups = graph.axis_groups()
        if 0 <= number < len(groups):
            for ref in list(groups[number].refs)[1:]:
                graph.assign(ref, side=groups[number].side)
            self._graph_changed()

    def _side_counts(self) -> tuple[int, int]:
        """Most axes any graph has on the left, on the right."""
        lefts = rights = 0
        for graph in self.layout_model.graphs:
            groups = graph.axis_groups()
            left = sum(group.side == "left" for group in groups)
            lefts, rights = max(lefts, left), max(rights, len(groups) - left)
        return lefts, rights

    def left_total(self) -> int:
        """Left-hand width reserved in every graph (at least one y axis)."""
        return max(1, self._side_counts()[0]) * AXIS_WIDTH

    def right_total(self) -> int:
        """Right-hand width reserved in every graph (y axes, or the spectrogram colour bar)."""
        return max(self._side_counts()[1] * AXIS_WIDTH, COLORBAR_PAD if self.docks else 0)

    def _pad_docks(self) -> None:
        for dock in self.docks:
            dock.window.set_pads(self.left_total(), self.right_total() - COLORBAR_PAD)

    def _pad_sides(self, slot: GraphSlot) -> None:
        """Reserve the same width left and right in every graph, so their time
        axes line up whatever number of y axes each has (and with the
        colour bar of docked spectrograms)."""
        layout = slot.plot.getPlotItem().layout
        layout.setColumnFixedWidth(0, self.left_total())
        layout.setColumnFixedWidth(2, self.right_total())

    # ===================================================== board spectrograms
    def add_board_spectrogram(self, ref, after: int | None = None):
        """Dock a spectrogram of ``ref`` in the board, ``after`` graphs down
        (default: at the bottom)."""
        if ref is None or ref[0] not in self.datasets:
            return None
        count = len(self.layout_model.graphs)
        dock = SpectrogramDock(self, ref, count if after is None else max(0, min(after, count)))
        self.docks.append(dock)
        self._after_docks_changed()
        return dock

    def board_spectrogram_selected(self):
        ref = self._selected_ref()
        if ref is None:
            self._message("info", "Spectrogram", "Select a trace (a column under a CSV) in "
                          "the list first, or right-click a trace or graph.")
            return None
        return self.add_board_spectrogram(ref)

    def remove_board_spectrogram(self, dock: SpectrogramDock, rebuild: bool = True) -> None:
        if dock in self.docks:
            self.docks.remove(dock)
            self.graph_layout.removeWidget(dock)
            dock.dispose()
            if rebuild:
                self._after_docks_changed()

    def move_board_spectrogram(self, dock: SpectrogramDock, step: int) -> None:
        """Move a docked spectrogram up (-1) or down (+1) past one graph."""
        count = len(self.layout_model.graphs)
        dock.after = max(0, min(count, dock.after + step))
        self._after_docks_changed()

    def _clear_docks(self) -> None:
        for dock in self.docks:
            self.graph_layout.removeWidget(dock)
            dock.dispose()
        self.docks = []

    def _after_docks_changed(self) -> None:
        """Re-place the grid, re-pad the plots' right edge and re-link the time axes."""
        self._place_slots()
        self._apply_heights()
        for slot in self.live_slots():
            self._pad_sides(slot)
        self._pad_docks()
        self._relink_docks()
        self.drop_zone.set_empty(not self.slots and not self.docks)
        self._schedule_realize()

    def _relink_docks(self) -> None:
        """Link every docked spectrogram's time axis to the graphs' (time x axis only)."""
        root = self._link_root if self.x_column() == SYNC_X else None
        for dock in self.docks:
            view = dock.window.p1.vb
            if view.linkedView(pg.ViewBox.XAxis) is not root:
                view.setXLink(root)

    def _refresh_docks(self, dataset_id: int | None = None) -> None:
        """A time axis or packet filter changed: recompute the docked spectrograms."""
        for dock in self.docks:
            if dataset_id is None or dock.ref[0] == dataset_id:
                dock.window.refresh_source()
        self._relink_docks()

    # =========================================================== boards
    def board_dict(self) -> dict:
        """The current dashboard as a data board (see viewer_board)."""
        ids = sorted(self.datasets)
        index_of = {dataset_id: i for i, dataset_id in enumerate(ids)}
        sources = [{"label": self.datasets[i].label,
                    "time_column": self.datasets[i].time_column,
                    "time_unit": self.datasets[i].time_unit,
                    "offset": float(self.datasets[i].offset)} for i in ids]

        def trace(ref):
            return {"source": index_of[ref[0]], "column": ref[1]}

        panels = []
        docks = sorted(self.docks, key=lambda dock: dock.after)

        def emit_docks(upto: int) -> None:
            while docks and docks[0].after <= upto:
                dock = docks.pop(0)
                panels.append({"type": "spectrogram", "trace": trace(dock.ref),
                               **dock.window.board_settings()})

        for position, refs in enumerate(self.layout_model.graphs):
            emit_docks(position)
            panels.append({"type": "plot",
                           "height": self._height_overrides.get(tuple(refs)),
                           "traces": [trace(ref) for ref in refs],
                           "axes": [{"side": group.side,
                                     "traces": [trace(ref) for ref in group.refs]}
                                    for group in refs.axis_groups()]})
        emit_docks(len(self.layout_model.graphs))
        return {"format": vb_file.BOARD_FORMAT, "version": vb_file.BOARD_VERSION,
                "x_axis": self.x_column() or None, "packet_filter": self.packet_filter(),
                "sync": list(self.sync_transition) if self.sync_transition else None,
                "graphs_per_row": self.graph_columns, "sources": sources, "panels": panels}

    def save_board(self, path) -> None:
        vb_file.write_board(path, self.board_dict())

    def load_board(self, source, apply_time: bool = True) -> "vb_file.BoardReport":
        """Rebuild a board on the open files (path or board dict).

        Columns are matched by name; those the open logs lack are listed in
        the report and skipped. Source 0 of the board goes to the first open
        file, and so on. Raises BoardError for an unusable board file.
        """
        board = vb_file.validate(source) if isinstance(source, dict) \
            else vb_file.read_board(source)
        report = vb_file.BoardReport()
        ids = sorted(self.datasets)
        if not ids:
            report.notes.append("No log is open: open one, then load the board.")
            return report
        targets = {i: ids[i] for i in range(min(len(ids), len(board["sources"]) or 1))}
        for i in range(len(board["sources"])):
            if i not in targets:
                report.notes.append(f"Board file {i + 1} ({board['sources'][i]['label']}) "
                                    "has no matching open log: its traces were skipped.")
        if apply_time:
            for i, entry in enumerate(board["sources"]):
                dataset = self.datasets.get(targets.get(i))
                if dataset is None:
                    continue
                column = entry["time_column"]
                try:
                    if column and column in dataset.time_choices():
                        self.set_time_axis(targets[i], column=column,
                                           unit=entry["time_unit"] or None,
                                           offset=entry["offset"])
                    else:
                        if column:
                            report.notes.append(f"{dataset.label}: time column {column!r} "
                                                "not found; time axis unchanged.")
                        self.set_time_axis(targets[i], offset=entry["offset"])
                except ValueError as exc:
                    report.notes.append(f"{dataset.label}: time axis not applied ({exc}).")
        if apply_time and board["sync"]:
            outcome = self.sync_logs(board["sync"], report=False)
            if outcome["skipped"]:
                report.notes.append(
                    f"No {vd.sync_label(board['sync'])} found in: "
                    + ", ".join(outcome["skipped"]) + "; their offsets were kept.")
        elif apply_time:
            self.sync_logs(None)
        for name, combo in (("x_axis", self.x_axis_combo),
                            ("packet_filter", self.packet_filter_combo)):
            wanted = board[name]
            if wanted is None:
                continue
            if combo.findText(wanted) >= 0:
                combo.setCurrentText(wanted)
            elif name == "x_axis":
                report.notes.append(f"x axis {wanted!r} not found; kept {self.x_column()!r}.")
        self._drop_cached()
        self.x_view = None

        def resolve(trace, quiet=False):
            dataset_id = targets.get(trace["source"])
            if dataset_id is None:
                return None
            if trace["column"] not in self.datasets[dataset_id].numeric_data:
                if not quiet:
                    report.missing.append(self.series_label((dataset_id, trace["column"])))
                return None
            return (dataset_id, trace["column"])

        self.layout_model.clear()
        self._height_overrides.clear()
        self._clear_docks()
        pending_docks = []
        for number, panel in enumerate(board["panels"], 1):
            if panel["type"] == "plot":
                refs = [ref for ref in (resolve(t) for t in panel["traces"]) if ref is not None]
                if not refs:
                    report.notes.append(f"Graph {number} skipped: none of its columns are here.")
                    continue
                graph = GraphRefs(dict.fromkeys(refs))
                groups = []
                for entry in panel["axes"]:
                    found = [ref for ref in (resolve(t, quiet=True) for t in entry["traces"])
                             if ref is not None]
                    if found:
                        groups.append((entry["side"], found))
                if groups:
                    graph.set_groups(groups)
                self.layout_model.graphs.append(graph)
                if panel["height"]:
                    self._height_overrides[tuple(graph)] = panel["height"]
                report.plots += 1
            else:
                ref = resolve(panel["trace"])
                if ref is None:
                    report.notes.append(f"Spectrogram {number} skipped: its column is not here.")
                    continue
                pending_docks.append((ref, len(self.layout_model.graphs), panel))
                report.spectrograms += 1
        self.columns_spin.setValue(max(1, min(MAX_COLUMNS, board["graphs_per_row"])))
        self.graph_columns = self.columns_spin.value()
        self.rebuild_graphs()
        for ref, after, panel in pending_docks:
            dock = SpectrogramDock(self, ref, after)
            dock.window.apply_board_settings(panel)
            self.docks.append(dock)
        self._after_docks_changed()
        report.missing = list(dict.fromkeys(report.missing))
        if report.missing or report.notes:
            self._message("warning", "Data board", report.text())
        return report

    def choose_save_board(self) -> None:
        name, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save data board", "", vb_file.BOARD_FILTER)
        if not name:
            return
        path = Path(name)
        if not path.name.endswith(".json"):
            path = path.with_name(path.name + vb_file.BOARD_SUFFIX)
        try:
            self.save_board(path)
        except (OSError, ValueError) as exc:
            self._message("error", "Save data board", f"Could not save {path.name}: {exc}")

    def choose_load_board(self) -> None:
        name, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load data board", "", vb_file.BOARD_FILTER)
        if name:
            self.open_board(name)

    def open_board(self, path):
        """Load a board file; problems with the file are reported, not raised."""
        try:
            return self.load_board(path)
        except vb_file.BoardError as exc:
            self._message("error", "Load data board", str(exc))
            return None

    # ======================================================= spectrogram
    def spectrum_source(self, ref):
        return vd.spectrum_source(self.datasets.get(ref[0]), ref[1], self.packet_filter())

    def current_time_zoom(self) -> tuple[float, float] | None:
        """The shared zoom, when the x axis is time (else None)."""
        return self.x_view if self.x_column() == SYNC_X else None

    def open_spectrogram(self, ref) -> "SpectrogramWindow | None":
        if ref is None or ref[0] not in self.datasets:
            return None
        window = SpectrogramWindow(self, ref)
        self._spectrum_windows.append(window)
        window.destroyed.connect(lambda _o=None, w=window: self._forget_window(w))
        window.show()
        return window

    def _forget_window(self, window) -> None:
        if window in self._spectrum_windows:
            self._spectrum_windows.remove(window)

    def spectrogram_selected(self):
        ref = self._selected_ref()
        if ref is None:
            self._message("info", "Spectrogram", "Select a trace (a column under a CSV) in "
                          "the list first, or right-click a trace or graph.")
            return None
        return self.open_spectrogram(ref)


def _html_escape(text: str) -> str:
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


# ---------------------------------------------------------------------------
# Spectrogram / FFT window
# ---------------------------------------------------------------------------

PAD_COLUMN = 12               # unused PlotItem layout column holding a spectrogram's right pad


class SpectrogramWindow(QtWidgets.QWidget):
    """Spectrogram (waterfall) of one trace over a time window.

    Time runs along x, frequency up y and power (dB) is the colour of a
    pyqtgraph ImageItem with a ColorBarItem; the Welch average spectrum is
    plotted underneath. The FFT and the display image are computed on a
    worker thread; the maths is in spectrum.py.
    """

    def __init__(self, panel: CsvViewerWidget, ref, embedded: bool = False):
        # Embedded: a child widget of a board dock (its time axis follows the
        # board's graphs); otherwise a window of its own.
        if embedded:
            super().__init__(panel)
        else:
            super().__init__(panel, QtCore.Qt.WindowType.Window)
            self.setAttribute(QtCore.Qt.WidgetAttribute.WA_DeleteOnClose)
        self.embedded = embedded
        self.panel = panel
        self.ref = ref
        self.label = panel.series_label(ref)
        if not embedded:
            self.setWindowTitle(f"Spectrogram - {self.label}")
            self.resize(1000, 720)
        self.t, self.y = panel.spectrum_source(ref)
        self.result: vd.SpectrogramResult | None = None
        self.error = ""
        self._worker = None
        self._pending = False
        self._closed = False
        self._build()
        zoom = None if embedded else panel.current_time_zoom()
        self.set_window(*(zoom or self.full_range()))
        QtCore.QTimer.singleShot(0, self.compute)

    @property
    def analysis(self):
        return None if self.result is None else self.result.analysis

    # ---------------------------------------------------------------- UI
    def _build(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        if self.embedded:
            layout.setContentsMargins(0, 0, 0, 0)
        row1 = QtWidgets.QHBoxLayout()
        name = QtWidgets.QLabel(f"Trace: {self.label}")
        font = name.font()
        font.setBold(True)
        name.setFont(font)
        row1.addWidget(name)
        row1.addSpacing(12)
        row1.addWidget(QtWidgets.QLabel("Start s"))
        self.start_edit = QtWidgets.QLineEdit()
        self.start_edit.setMaximumWidth(110)
        row1.addWidget(self.start_edit)
        row1.addWidget(QtWidgets.QLabel("End s"))
        self.end_edit = QtWidgets.QLineEdit()
        self.end_edit.setMaximumWidth(110)
        row1.addWidget(self.end_edit)
        for edit in (self.start_edit, self.end_edit):
            edit.returnPressed.connect(self.compute)
        zoom_button = QtWidgets.QPushButton("Use current zoom")
        zoom_button.clicked.connect(self.use_zoom)
        full_button = QtWidgets.QPushButton("Whole trace")
        full_button.clicked.connect(self.use_full)
        row1.addWidget(zoom_button)
        row1.addWidget(full_button)
        row1.addStretch(1)
        layout.addLayout(row1)

        row2 = QtWidgets.QHBoxLayout()
        row2.addWidget(QtWidgets.QLabel("FFT length"))
        self.nfft_combo = QtWidgets.QComboBox()
        self.nfft_combo.addItems([str(n) for n in spectrum.FFT_LENGTHS])
        self.nfft_combo.setCurrentText("1024")
        row2.addWidget(self.nfft_combo)
        row2.addWidget(QtWidgets.QLabel("Overlap %"))
        self.overlap_spin = QtWidgets.QSpinBox()
        self.overlap_spin.setRange(0, 95)
        self.overlap_spin.setSingleStep(5)
        self.overlap_spin.setValue(50)
        row2.addWidget(self.overlap_spin)
        row2.addWidget(QtWidgets.QLabel("Window"))
        self.window_combo = QtWidgets.QComboBox()
        self.window_combo.addItems(list(spectrum.WINDOWS))
        row2.addWidget(self.window_combo)
        row2.addWidget(QtWidgets.QLabel("Detrend"))
        self.detrend_combo = QtWidgets.QComboBox()
        self.detrend_combo.addItems(list(spectrum.DETRENDS))
        row2.addWidget(self.detrend_combo)
        self.welch_check = QtWidgets.QCheckBox("Average spectrum (Welch)")
        self.welch_check.setChecked(not self.embedded)
        self.welch_check.toggled.connect(lambda _c: self.render())
        row2.addWidget(self.welch_check)
        self.compute_button = QtWidgets.QPushButton("Compute")
        self.compute_button.clicked.connect(self.compute)
        row2.addWidget(self.compute_button)
        row2.addStretch(1)
        layout.addLayout(row2)

        row3 = QtWidgets.QHBoxLayout()
        row3.addWidget(QtWidgets.QLabel("Colour range dB  min"))
        self.db_min_edit = QtWidgets.QLineEdit()
        self.db_min_edit.setMaximumWidth(80)
        row3.addWidget(self.db_min_edit)
        row3.addWidget(QtWidgets.QLabel("max"))
        self.db_max_edit = QtWidgets.QLineEdit()
        self.db_max_edit.setMaximumWidth(80)
        row3.addWidget(self.db_max_edit)
        for edit in (self.db_min_edit, self.db_max_edit):
            edit.returnPressed.connect(self.apply_colours)
        apply_button = QtWidgets.QPushButton("Apply")
        apply_button.clicked.connect(self.apply_colours)
        row3.addWidget(apply_button)
        self.auto_range_check = QtWidgets.QCheckBox("Auto range")
        self.auto_range_check.setChecked(True)
        self.auto_range_check.toggled.connect(lambda _c: self.render())
        row3.addWidget(self.auto_range_check)
        row3.addStretch(1)
        layout.addLayout(row3)

        self.status_label = QtWidgets.QLabel("")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("color: #444444;")
        layout.addWidget(self.status_label)

        self.graphics = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphics, 1)
        if self.embedded:
            self.graphics.ci.layout.setContentsMargins(0, 0, 0, 0)   # line up with the graphs
            self.p1 = self.graphics.addPlot(row=0, col=0, viewBox=ExactXViewBox())
            self.p1.getAxis("left").setWidth(AXIS_WIDTH)
        else:
            self.p1 = self.graphics.addPlot(row=0, col=0)
        self.p1.setLabel("bottom", "time [s] (shared axis)")
        self.p1.setLabel("left", "frequency [Hz]")
        self.p1.setMouseEnabled(x=True, y=True)
        self.image = pg.ImageItem()
        self.p1.addItem(self.image)
        self.colorbar = pg.ColorBarItem(values=(-100, 0), colorMap=pg.colormap.get("viridis"),
                                        label="dB", interactive=True)
        self.colorbar.setImageItem(self.image, insert_in=self.p1)
        self.colorbar.sigLevelsChangeFinished.connect(self._colorbar_moved)
        self.message = pg.TextItem("Computing…", color="#777777", anchor=(0.5, 0.5))
        self.p1.addItem(self.message, ignoreBounds=True)
        cross = pg.mkPen("#ffffff", style=QtCore.Qt.PenStyle.DashLine)
        self.v_line = pg.InfiniteLine(angle=90, movable=False, pen=cross)
        self.h_line = pg.InfiniteLine(angle=0, movable=False, pen=cross)
        for line in (self.v_line, self.h_line):
            line.hide()
            self.p1.addItem(line, ignoreBounds=True)
        self.p2 = self.graphics.addPlot(row=1, col=0)
        self.p2.setLabel("bottom", "frequency [Hz]")
        self.p2.setLabel("left", "Welch dB")
        self.p2.showGrid(x=True, y=True, alpha=0.25)
        self.welch_curve = add_curve(self.p2, np.empty(0), np.empty(0),
                                     pen=pg.mkPen("#1565c0", width=1.5))
        self.welch_line = pg.InfiniteLine(angle=90, movable=False,
                                          pen=pg.mkPen("#9e9e9e", style=QtCore.Qt.PenStyle.DashLine))
        self.welch_line.hide()
        self.p2.addItem(self.welch_line, ignoreBounds=True)
        self.graphics.ci.layout.setRowStretchFactor(0, 7)
        self.graphics.ci.layout.setRowStretchFactor(1, 3)
        self.graphics.scene().sigMouseMoved.connect(self._mouse_moved)

        self.hover_label = QtWidgets.QLabel("Hover the plot for time, frequency and dB.")
        layout.addWidget(self.hover_label)

    def shutdown(self) -> None:
        self._closed = True
        self._pending = False
        CsvViewerWidget._disconnect_worker(self._worker)
        self._worker = None

    def closeEvent(self, event) -> None:  # noqa: N802
        self.shutdown()
        super().closeEvent(event)

    # ------------------------------------------------------------ window
    def full_range(self) -> tuple[float, float]:
        if self.t.size == 0:
            return 0.0, 0.0
        return float(self.t.min()), float(self.t.max())

    def set_window(self, start: float, end: float) -> None:
        self.start_edit.setText(value_text(round(start, 6)))
        self.end_edit.setText(value_text(round(end, 6)))

    def use_zoom(self) -> None:
        zoom = self.panel.current_time_zoom()
        if zoom is None:
            self.status_label.setText("No time zoom to use: set the x axis to time and drag "
                                      "across a plot to zoom first.")
            return
        self.set_window(*zoom)
        self.compute()

    def use_full(self) -> None:
        self.set_window(*self.full_range())
        self.compute()

    def set_pads(self, left: int, right: int) -> None:
        """Match the graphs: left axis width, and empty space right of the
        colour bar, so the time axis starts and ends where the graphs' do."""
        self.p1.getAxis("left").setWidth(int(left))
        self.p1.layout.setColumnFixedWidth(PAD_COLUMN, max(0, int(right)))

    def refresh_source(self) -> None:
        """Re-read the trace (new time axis, offset or packet filter) and recompute."""
        self.t, self.y = self.panel.spectrum_source(self.ref)
        self.use_full()

    def board_settings(self) -> dict:
        """The settings a data board remembers (see viewer_board)."""
        full = self.full_range()
        try:
            window = [float(self.start_edit.text()), float(self.end_edit.text())]
        except ValueError:
            window = list(full)
        same = (abs(window[0] - round(full[0], 6)) < 1e-6
                and abs(window[1] - round(full[1], 6)) < 1e-6)
        if self.auto_range_check.isChecked():
            colours = "auto"
        else:
            try:
                colours = [float(self.db_min_edit.text()), float(self.db_max_edit.text())]
            except ValueError:
                colours = "auto"
        return {"nfft": int(self.nfft_combo.currentText()),
                "overlap": int(self.overlap_spin.value()),
                "window": self.window_combo.currentText(),
                "detrend": self.detrend_combo.currentText(),
                "welch": self.welch_check.isChecked(),
                "colour_range": colours,
                "time_window": "full" if same else window}

    def apply_board_settings(self, settings: dict) -> None:
        for combo, value in ((self.nfft_combo, str(settings["nfft"])),
                             (self.window_combo, settings["window"]),
                             (self.detrend_combo, settings["detrend"])):
            if combo.findText(value) >= 0:
                combo.setCurrentText(value)
        self.overlap_spin.setValue(max(0, min(95, int(settings["overlap"]))))
        self.welch_check.setChecked(bool(settings["welch"]))
        colours = settings["colour_range"]
        if colours != "auto":
            self.db_min_edit.setText(value_text(colours[0]))
            self.db_max_edit.setText(value_text(colours[1]))
        self.auto_range_check.setChecked(colours == "auto")
        window = settings["time_window"]
        self.set_window(*(self.full_range() if window == "full" else window))

    def settings(self) -> dict:
        return vd.spectrum_settings(self.start_edit.text(), self.end_edit.text(),
                                    self.nfft_combo.currentText(), self.overlap_spin.value(),
                                    self.window_combo.currentText(),
                                    self.detrend_combo.currentText())

    # ----------------------------------------------------------- compute
    @property
    def computing(self) -> bool:
        return self._worker is not None

    def compute(self) -> None:
        """Run the FFT for the current settings on a worker thread."""
        if self._closed:
            return
        try:
            params = self.settings()
        except ValueError as exc:
            self.status_label.setText(f"Check the settings: {exc}")
            return
        if self._worker is not None:
            self._pending = True          # rerun with the newest settings after
            return
        self.compute_button.setEnabled(False)
        self.status_label.setText("Computing…")
        self._worker = run_in_thread(vd.compute_spectrogram, self.t, self.y, params,
                                     on_done=self._done, on_error=self._failed)

    def _finished(self) -> None:
        self._worker = None
        self.compute_button.setEnabled(True)
        self.render()
        if self._pending:
            self._pending = False
            self.compute()

    def _done(self, result) -> None:
        if self._closed:
            return
        self.result, self.error = result, ""
        self.status_label.setText(self.status_text())
        self._finished()

    def _failed(self, exc) -> None:
        if self._closed:
            return
        self.result = None
        self.error = str(exc) if isinstance(exc, (ValueError, MemoryError)) and str(exc) \
            else f"{type(exc).__name__}: {exc}"
        self.status_label.setText(f"Could not compute: {self.error}")
        self._finished()

    def wait(self, timeout: float = 30.0) -> bool:
        """Pump Qt until the computation finishes (tests and scripts)."""
        return qt_common.process_events_until(
            lambda: self._worker is None and not self._pending, timeout)

    def status_text(self) -> str:
        a = self.analysis
        return self.error if a is None else vd.spectrum_status_text(a)

    # ------------------------------------------------------------ render
    def db_range(self) -> tuple[float, float]:
        result = self.result
        if self.auto_range_check.isChecked() and result is not None:
            low, high = result.auto_range
            self.db_min_edit.setText(value_text(low))
            self.db_max_edit.setText(value_text(high))
            return float(low), float(high)
        try:
            low, high = float(self.db_min_edit.text()), float(self.db_max_edit.text())
            if math.isfinite(low) and math.isfinite(high) and high > low:
                return low, high
        except ValueError:
            pass
        return tuple(map(float, result.auto_range)) if result is not None else (-100.0, 0.0)

    def apply_colours(self) -> None:
        self.auto_range_check.blockSignals(True)
        self.auto_range_check.setChecked(False)
        self.auto_range_check.blockSignals(False)
        self.render()

    def _colorbar_moved(self, bar) -> None:
        low, high = bar.levels()
        self.db_min_edit.setText(value_text(round(low, 1)))
        self.db_max_edit.setText(value_text(round(high, 1)))
        self.auto_range_check.blockSignals(True)
        self.auto_range_check.setChecked(False)
        self.auto_range_check.blockSignals(False)

    def render(self) -> None:
        self.p2.setVisible(self.welch_check.isChecked())
        result = self.result
        if result is None:
            self.image.clear()
            self.message.setText(self.error or "Computing…")
            self.message.show()
            return
        self.message.hide()
        a = result.analysis
        low, high = self.db_range()
        fmax = float(a.freqs[-1])
        self.image.setImage(result.image, autoLevels=False, levels=(low, high))
        self.image.setRect(QtCore.QRectF(a.t_start, 0.0, a.t_end - a.t_start, fmax))
        self.colorbar.setLevels((low, high))
        if self.embedded:
            # The board's graphs own the time zoom; only the data fixes y.
            self.p1.setLimits(yMin=0.0, yMax=fmax)
            self.p1.setYRange(0.0, fmax, padding=0)
            if self.p1.vb.linkedView(pg.ViewBox.XAxis) is None:
                self.p1.setXRange(a.t_start, a.t_end, padding=0)
        else:
            self.p1.setLimits(xMin=a.t_start, xMax=a.t_end, yMin=0.0, yMax=fmax)
            self.p1.setRange(xRange=(a.t_start, a.t_end), yRange=(0.0, fmax), padding=0)
        welch = np.where(a.welch_db > -299, a.welch_db, np.nan)
        self.welch_curve.setData(a.freqs, welch)
        self.p2.setLimits(xMin=0.0, xMax=fmax)
        self.p2.setXRange(0.0, fmax, padding=0)
        self.p2.setTitle(f"average spectrum (Welch, {a.result.segments:,} segments)",
                         size="9pt")

    # ------------------------------------------------------------- hover
    def readout_at(self, t: float, f: float) -> str | None:
        """Hover text at a spectrogram point: time, frequency and dB."""
        a = self.analysis
        if a is None:
            return None
        hit = a.value_at(t, f)
        if hit is None:
            return None
        return f"t = {hit[0]:.6g} s    f = {hit[1]:.6g} Hz    {hit[2]:.1f} dB"

    def welch_readout_at(self, f: float) -> str | None:
        a = self.analysis
        if a is None or not 0 <= f <= a.freqs[-1]:
            return None
        df = a.freqs[1] - a.freqs[0] if a.freqs.size > 1 else 1.0
        index = min(a.freqs.size - 1, int(round(f / df)))
        return (f"f = {a.freqs[index]:.6g} Hz    {a.welch_db[index]:.1f} dB "
                "(average spectrum)")

    def _mouse_moved(self, pos) -> None:
        text = None
        for line in (self.v_line, self.h_line, self.welch_line):
            line.hide()
        if self.p1.vb.sceneBoundingRect().contains(pos):
            point = self.p1.vb.mapSceneToView(pos)
            text = self.readout_at(point.x(), point.y())
            if text is not None:
                self.v_line.setPos(point.x())
                self.h_line.setPos(point.y())
                self.v_line.show()
                self.h_line.show()
        elif self.p2.isVisible() and self.p2.vb.sceneBoundingRect().contains(pos):
            point = self.p2.vb.mapSceneToView(pos)
            text = self.welch_readout_at(point.x())
            if text is not None:
                self.welch_line.setPos(point.x())
                self.welch_line.show()
        if text is not None:
            self.hover_label.setText(text)


class SpectrogramDock(QtWidgets.QFrame):
    """A spectrogram docked in the board between the graphs. Its time axis is
    x-linked to the graphs (when the x axis is time), so zooming either moves
    both. ``after`` is how many graphs sit above it."""

    def __init__(self, owner: "CsvViewerWidget", ref, after: int):
        super().__init__()
        self.owner = owner
        self.ref = ref
        self.after = after
        self.setObjectName("spectrogramDock")
        self.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.setFixedHeight(DOCK_HEIGHT)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(0)
        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel(f"Spectrogram: {owner.series_label(ref)}")
        font = title.font()
        font.setBold(True)
        title.setFont(font)
        header.addWidget(title)
        header.addStretch(1)
        for text, tip, step in (("\u25b2", "Move up the board", -1),
                                ("\u25bc", "Move down the board", 1)):
            button = QtWidgets.QToolButton()
            button.setText(text)
            button.setAutoRaise(True)
            button.setToolTip(tip)
            button.clicked.connect(lambda _c=False, d=step: owner.move_board_spectrogram(self, d))
            header.addWidget(button)
        close = QtWidgets.QToolButton()
        close.setText("\u2715")
        close.setAutoRaise(True)
        close.setToolTip("Remove this spectrogram from the board")
        close.clicked.connect(lambda: owner.remove_board_spectrogram(self))
        header.addWidget(close)
        layout.addLayout(header)
        self.window = SpectrogramWindow(owner, ref, embedded=True)
        layout.addWidget(self.window, 1)

    def dispose(self) -> None:
        self.window.shutdown()
        self.window.p1.vb.setXLink(None)
        self.setParent(None)
        self.deleteLater()


# ---------------------------------------------------------------------------
# Standalone window
# ---------------------------------------------------------------------------

class CsvViewerWindow(QtWidgets.QMainWindow):
    """Standalone window around CsvViewerWidget (python viewer.py)."""

    def __init__(self, files=(), prompt_on_start: bool = False):
        super().__init__()
        self.setWindowTitle("Horizon CSV Viewer")
        screen = QtGui.QGuiApplication.primaryScreen()
        if screen is not None:
            geometry = screen.availableGeometry()
            self.resize(min(1200, geometry.width() - 40), min(850, geometry.height() - 80))
        self.setMinimumSize(560, 380)
        self.viewer = CsvViewerWidget(self, files, on_title=self.setWindowTitle)
        self.setCentralWidget(self.viewer)
        if prompt_on_start and not files:
            QtCore.QTimer.singleShot(0, self.viewer.choose_csv)

    def closeEvent(self, event) -> None:  # noqa: N802
        self.viewer.shutdown()
        super().closeEvent(event)


def parse_arguments(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Graph and overlay numeric columns of converted Horizon CSV files.")
    parser.add_argument("csv_files", nargs="*", type=Path,
                        help="converted CSV file(s) or raw .BIN logs; several are "
                             "overlaid on Witness uptime")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_arguments(argv)
    app = ensure_app()
    import stall_watchdog
    stall_watchdog.install(Path(__file__).resolve().parent / "logs")
    window = CsvViewerWindow(args.csv_files, prompt_on_start=not args.csv_files)
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
