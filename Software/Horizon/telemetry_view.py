"""Telemetry field tree and diagnostics console for Horizon's input tabs (Qt).

The rows' state lives in telemetry_model.TelemetryTable; this widget renders
it in a QTreeWidget (source > packet > field) with value, receive time, age
and delivery/validity columns, coloured by data age, above a bounded log.
"""

from __future__ import annotations

import datetime as dt

from PyQt6 import QtCore, QtGui, QtWidgets

from qt_widgets import LogConsole
from telemetry_model import (  # noqa: F401 - re-exported
    FEATHERWEIGHT_FIELDS, WITNESS_FIELDS, TelemetryTable, flatten_fields,
)

COLUMNS = (("Source / packet / field", 320), ("Latest value", 240), ("Received at (UTC)", 200),
           ("Data age", 90), ("Delivery / validity", 145))
VALUE, RECEIVED, AGE, QUALITY = 1, 2, 3, 4
TAG_COLORS = {"fresh": "#f7f7f7", "yellow": "#fff2a8", "orange": "#ffd27f",
              "stale": "#ff8a80", "invalid": "#ff8a80", "retained": "#e4e4e4"}


def configure_tree(tree: QtWidgets.QTreeWidget, columns) -> None:
    tree.setColumnCount(len(columns))
    tree.setHeaderLabels([title for title, _width in columns])
    for index, (_title, width) in enumerate(columns):
        tree.setColumnWidth(index, width)
    tree.setUniformRowHeights(True)
    tree.setAlternatingRowColors(False)
    tree.header().setStretchLastSection(True)
    tree.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAsNeeded)


def paint_row(item: QtWidgets.QTreeWidgetItem, tag: str) -> None:
    color = TAG_COLORS.get(tag)
    brush = QtGui.QBrush(QtGui.QColor(color)) if color else QtGui.QBrush()
    for column in range(item.columnCount()):
        item.setBackground(column, brush)
    item.setData(0, QtCore.Qt.ItemDataRole.UserRole, tag)


class TelemetryView(QtWidgets.QWidget):
    def __init__(self, parent=None, *, schema=None):
        super().__init__(parent)
        self.model = TelemetryTable(schema)
        self.schema = self.model.schema
        self._sources, self._groups, self._rows = {}, {}, {}
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        splitter.setChildrenCollapsible(False)
        self.tree = QtWidgets.QTreeWidget()
        configure_tree(self.tree, COLUMNS)
        splitter.addWidget(self.tree)
        box = QtWidgets.QGroupBox("Received packets and diagnostics")
        box_layout = QtWidgets.QVBoxLayout(box)
        box_layout.setContentsMargins(4, 4, 4, 4)
        self.console = LogConsole(max_lines=1000)
        box_layout.addWidget(self.console)
        splitter.addWidget(box)
        splitter.setStretchFactor(0, 4)
        splitter.setStretchFactor(1, 1)
        layout.addWidget(splitter)

    # ------------------------------------------------------------ rows
    def clear(self):
        self.tree.clear()
        self._sources.clear()
        self._groups.clear()
        self._rows.clear()
        self.model.clear()

    def _item(self, row):
        key = (row.source, row.packet, row.field)
        item = self._rows.get(key)
        if item is not None:
            return item
        parent = self._sources.get(row.source)
        if parent is None:
            parent = self._sources[row.source] = QtWidgets.QTreeWidgetItem(self.tree, [row.source])
            parent.setExpanded(True)
        group = self._groups.get((row.source, row.packet))
        if group is None:
            group = self._groups[(row.source, row.packet)] = QtWidgets.QTreeWidgetItem(parent, [row.packet])
            group.setExpanded(True)
        item = self._rows[key] = QtWidgets.QTreeWidgetItem(group, [row.field, "null", "--", "--", "--"])
        return item

    def _show(self, row):
        item = self._item(row)
        for column, text in ((VALUE, row.value), (RECEIVED, row.received), (AGE, row.age),
                             (QUALITY, row.quality)):
            if item.text(column) != text:
                item.setText(column, str(text))
        if item.data(0, QtCore.Qt.ItemDataRole.UserRole) != row.tag:
            paint_row(item, row.tag)

    def add_source(self, source):
        for row in self.model.add_source(source):
            self._show(row)

    def update_record(self, record, *, witness=False):
        self.tree.setUpdatesEnabled(False)
        try:
            for row in self.model.update_record(record, witness=witness):
                self._show(row)
        finally:
            self.tree.setUpdatesEnabled(True)
        if len(self.model.rows) >= self.model.max_rows and not self.model.limit_reported:
            self.model.limit_reported = True
            self.append(["Display field limit reached; further data remains in the input log."])

    def update_ages(self):
        for row in self.model.update_ages():
            self._show(row)

    # Accessors used by tests and other tabs.
    def value(self, key):
        return self._rows[key].text(VALUE)

    def column(self, key, column):
        return self._rows[key].text({"value": VALUE, "received": RECEIVED, "age": AGE,
                                     "quality": QUALITY}[column])

    def tag(self, key):
        return self.model.rows[key].tag

    @property
    def _received(self):
        return self.model.received

    # ---------------------------------------------------------- console
    def append(self, lines):
        if not lines:
            return
        stamp = dt.datetime.now().strftime("%H:%M:%S")
        self.console.append_lines(f"[{stamp}] {line}" for line in lines)
