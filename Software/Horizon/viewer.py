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
import viewer_data as vd
from qt_common import add_curve, ensure_app, run_in_thread, Throttle
# Compatibility: the data helpers were importable from viewer before the Qt
# port (Horizon main.py imports these four; tests and scripts use the rest).
from viewer_data import *  # noqa: F401,F403
from viewer_data import (  # noqa: F401
    AUTO_TIME, BINARY_COLUMNS, DROP_ZONE_HEIGHT, MIN_ZOOM_DRAG_PX, PAN_FRACTION,
    PLOT_COLORS, ROW_INDEX, SYNC_X, TEXT_READOUT_COLUMNS, TIME_UNITS, ZOOM_FACTOR,
    Dataset, GraphLayout, LAUNCH_NOTE, MemoryBudgetError, Series, decimate_minmax,
    elide, graph_height_for, nice_ticks, tick_text, unique_label, value_text,
)


TRACES_MIME = "application/x-horizon-traces"
LEGEND_MIME = "application/x-horizon-legend"
PEN_STYLES = (QtCore.Qt.PenStyle.SolidLine, QtCore.Qt.PenStyle.DashLine,
              QtCore.Qt.PenStyle.DotLine, QtCore.Qt.PenStyle.DashDotLine,
              QtCore.Qt.PenStyle.DashDotDotLine)
AXIS_WIDTH = 64               # fixed left axis width: every graph's x lines up
MAX_LIVE_PLOTS = 40           # PlotWidgets kept alive; farther ones are freed
LAZY_MARGIN = 0.75            # build graphs this many viewports above/below
REALIZE_BUDGET_S = 0.035      # GUI time spent creating graphs per event-loop turn
SERIES_CACHE_BYTES = 768 * 1024 ** 2
BUSY_SWITCH_INTERVAL_S = 0.0005
CSV_FILTER = "CSV / text logs (*.csv *.CSV *.txt *.tsv);;All files (*)"
BIN_FILTER = "Witness logs (*.bin *.BIN);;All files (*)"


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

class TimeViewBox(pg.ViewBox):
    """ViewBox with the viewer's mouse conventions.

    Drag across a plot: zoom x (every graph). Ctrl+wheel: zoom about the
    cursor. Shift+wheel: pan. Plain wheel scrolls the page. Shift/middle
    drag pans. Double-click resets the zoom. Right-click: menu.
    """

    def __init__(self, owner: "CsvViewerWidget"):
        super().__init__(enableMenu=True)
        self.owner = owner
        self.setMouseEnabled(x=True, y=False)

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

    def mouseDragEvent(self, ev, axis=None):  # noqa: N802
        left = ev.button() == QtCore.Qt.MouseButton.LeftButton
        shift = bool(ev.modifiers() & QtCore.Qt.KeyboardModifier.ShiftModifier)
        if axis is not None or not left or shift:
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
        if modifiers & QtCore.Qt.KeyboardModifier.ControlModifier:
            center = self.mapToView(ev.pos()).x()
            self.owner.zoom(ZOOM_FACTOR if delta > 0 else 1 / ZOOM_FACTOR, center)
            ev.accept()
        elif modifiers & QtCore.Qt.KeyboardModifier.ShiftModifier:
            self.owner.pan(-1 if delta > 0 else 1)
            ev.accept()
        else:
            ev.ignore()          # the scroll area moves the page

    def mouseClickEvent(self, ev):  # noqa: N802
        if ev.button() == QtCore.Qt.MouseButton.LeftButton and ev.double():
            ev.accept()
            self.owner.reset_zoom()
            return
        super().mouseClickEvent(ev)


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
    if mime.hasFormat(LEGEND_MIME):
        source, dataset_id, column = json.loads(bytes(mime.data(LEGEND_MIME)).decode())
        return ("legend", int(source), (int(dataset_id), str(column)))
    return None


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

    @property
    def realized(self) -> bool:
        return self.plot is not None

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
        item.getAxis("left").setWidth(AXIS_WIDTH)
        item.setLabel("bottom", owner.x_column())
        item.hideButtons()
        self.plot = plot
        self._layout.addWidget(plot, 1)

        vb = self.vb
        vb.disableAutoRange(axis=pg.ViewBox.XAxis)
        if self.refs and all(ref[1] in BINARY_COLUMNS for ref in self.refs):
            vb.disableAutoRange(axis=pg.ViewBox.YAxis)
            vb.setYRange(-0.1, 1.1, padding=0)
        else:
            vb.enableAutoRange(axis=pg.ViewBox.YAxis)
            vb.setAutoVisible(y=True)
        x_column = owner.x_column()
        any_data = False
        for series_index, ref in enumerate(self.refs):
            xs, ys = owner.series(ref)
            color, style = owner.series_style(self.index, series_index, len(self.refs), ref)
            pen = pg.mkPen(color, width=1.5, style=style)
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
        menu.addAction("Remove this graph", lambda: owner.remove_graph(self.index))
        owner._attach_view(self)
        plot.scene().sigMouseMoved.connect(self._mouse_moved)
        self.update_stats()

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
    def dragEnterEvent(self, event) -> None:  # noqa: N802
        if _drop_payload(event.mimeData()) is not None:
            event.acceptProposedAction()
            self.setStyleSheet("#graphSlot { border: 3px solid #1565c0; }")
        else:
            event.ignore()

    def dragMoveEvent(self, event) -> None:  # noqa: N802
        if _drop_payload(event.mimeData()) is not None:
            event.acceptProposedAction()
        else:
            event.ignore()

    def dragLeaveEvent(self, event) -> None:  # noqa: N802
        self.setStyleSheet("")

    def dropEvent(self, event) -> None:  # noqa: N802
        self.setStyleSheet("")
        payload = _drop_payload(event.mimeData())
        if payload is None:
            event.ignore()
            return
        event.acceptProposedAction()
        index = self.index
        QtCore.QTimer.singleShot(0, lambda: self.owner.drop(payload, ("graph", index)))


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

        self._build_ui()
        self.setAcceptDrops(True)

        self._realize_timer = QtCore.QTimer(self)
        self._realize_timer.setSingleShot(True)
        self._realize_timer.setInterval(0)
        self._realize_timer.timeout.connect(self._realize_visible)
        self._hover_throttle = Throttle(self._hover_tick, 25, self)
        self._view_throttle = Throttle(self._view_changed_tick, 120, self)

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
        self.graph_layout = QtWidgets.QVBoxLayout(self.graph_container)
        self.graph_layout.setContentsMargins(0, 0, 0, 0)
        self.graph_layout.setSpacing(4)
        self.drop_zone = DropZone(self)
        self.graph_layout.addWidget(self.drop_zone)
        self.graph_layout.addStretch(1)
        self.graph_scroll.setWidget(self.graph_container)
        self.graph_scroll.verticalScrollBar().valueChanged.connect(
            lambda _v: self._schedule_realize())
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
            "Drag traces onto a graph to overlay them, or below the last graph for a "
            "new one. Double-click adds a graph. Drag a legend entry to another graph, "
            "or back here to remove it.\nZoom (all graphs): drag across a plot, or "
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
        if first:
            self.notebook.setCurrentWidget(self.summary_tab)
            self.graph_scroll.verticalScrollBar().setValue(0)
        self.rebuild_graphs()
        self.refresh_all()

    def clear_datasets(self) -> None:
        self.datasets.clear()
        self.x_view = None
        self.layout_model.clear()
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
        height = self._graph_height()
        for index, refs in enumerate(self.layout_model.graphs):
            slot = GraphSlot(self, index, refs, height)
            self.graph_layout.insertWidget(index, slot)
            self.slots.append(slot)
        self.drop_zone.set_empty(not self.slots)
        self._update_status()
        self._schedule_realize()

    def _graph_height(self) -> int:
        view_height = self.graph_scroll.viewport().height()
        return graph_height_for(len(self.layout_model.graphs), view_height - DROP_ZONE_HEIGHT)

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
            height = self._graph_height()
            for slot in self.slots:
                if slot.height() != height:
                    slot.setFixedHeight(height)
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
        QtCore.QTimer.singleShot(0, self.rebuild_graphs)

    def drop(self, payload, target) -> None:
        """Apply a finished drag (also used by tests)."""
        if target is None or payload is None:
            return
        if payload[0] == "traces":
            refs = [ref for ref in payload[1] if ref[0] in self.datasets]
            if target[0] == "graph":
                self.layout_model.add(refs, target[1])
            elif target[0] == "new":
                self.layout_model.add(refs)
            else:
                return
        elif payload[0] == "legend":
            source, ref = payload[1], payload[2]
            if target[0] == "graph":
                self.layout_model.move(source, ref, target[1])
            elif target[0] == "new":
                self.layout_model.move(source, ref, None)
            elif target[0] == "list":
                self.layout_model.remove_series(source, ref)
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
                spots.append({"pos": (float(xs[index]) + offset, float(ys[index])),
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

class SpectrogramWindow(QtWidgets.QWidget):
    """Spectrogram (waterfall) of one trace over a time window.

    Time runs along x, frequency up y and power (dB) is the colour of a
    pyqtgraph ImageItem with a ColorBarItem; the Welch average spectrum is
    plotted underneath. The FFT and the display image are computed on a
    worker thread; the maths is in spectrum.py.
    """

    def __init__(self, panel: CsvViewerWidget, ref):
        super().__init__(panel, QtCore.Qt.WindowType.Window)
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_DeleteOnClose)
        self.panel = panel
        self.ref = ref
        self.label = panel.series_label(ref)
        self.setWindowTitle(f"Spectrogram - {self.label}")
        self.resize(1000, 720)
        self.t, self.y = panel.spectrum_source(ref)
        self.result: vd.SpectrogramResult | None = None
        self.error = ""
        self._worker = None
        self._pending = False
        self._closed = False
        self._build()
        zoom = panel.current_time_zoom()
        self.set_window(*(zoom or self.full_range()))
        QtCore.QTimer.singleShot(0, self.compute)

    @property
    def analysis(self):
        return None if self.result is None else self.result.analysis

    # ---------------------------------------------------------------- UI
    def _build(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
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
        self.welch_check.setChecked(True)
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
    window = CsvViewerWindow(args.csv_files, prompt_on_start=not args.csv_files)
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
