"""Small Qt helpers shared by Horizon's views: Cell bindings, consoles, layouts.

Views bind widgets to the observable Cells of the logic modules:
    bind_label(core.status_text, label)
    bind_line_edit(core.test_file, edit)
    bind_check(core.auto_connect, checkbox, on_user=core._auto_changed)
"""

from __future__ import annotations

from typing import Callable, Iterable

from PyQt6 import QtCore, QtGui, QtWidgets

from observable import Cell


def mono_font(size: int = 9) -> QtGui.QFont:
    """Monospace font (created on demand: QFont needs a QGuiApplication)."""
    font = QtGui.QFont("Consolas")
    font.setStyleHint(QtGui.QFont.StyleHint.Monospace)
    font.setPointSize(size)
    return font


def _owned(widget: QtWidgets.QWidget, unsubscribe: Callable[[], None]) -> None:
    """Drop a Cell subscription when its widget is destroyed."""
    widget.destroyed.connect(lambda *_: unsubscribe())


def bind_label(cell: Cell, label: QtWidgets.QLabel, fmt: Callable = str) -> None:
    def show(value):
        label.setText(fmt(value))
    _owned(label, cell.subscribe(show, call_now=True))


def bind_line_edit(cell: Cell, edit: QtWidgets.QLineEdit) -> None:
    def show(value):
        text = "" if value is None else str(value)
        if edit.text() != text:
            edit.setText(text)
    edit.textChanged.connect(cell.set)
    _owned(edit, cell.subscribe(show, call_now=True))


def bind_check(cell: Cell, box: QtWidgets.QAbstractButton, on_user: Callable | None = None) -> None:
    """Checkbox <-> boolean Cell. on_user runs after a user click only."""
    def show(value):
        if box.isChecked() != bool(value):
            box.blockSignals(True)
            box.setChecked(bool(value))
            box.blockSignals(False)

    def clicked(checked):
        cell.set(bool(checked))
        if on_user is not None:
            on_user()
    box.clicked.connect(clicked)
    _owned(box, cell.subscribe(show, call_now=True))


def bind_combo(cell: Cell, combo: QtWidgets.QComboBox, choices: Cell | Iterable[str] | None = None) -> None:
    """Combo text <-> Cell; choices may be a Cell holding the item list."""
    def set_items(items):
        current = combo.currentText()
        combo.blockSignals(True)
        combo.clear()
        combo.addItems([str(item) for item in items])
        wanted = str(cell.get()) if cell.get() is not None else current
        index = combo.findText(wanted)
        if index >= 0:
            combo.setCurrentIndex(index)
        elif combo.isEditable():
            combo.setEditText(wanted)
        combo.blockSignals(False)

    def show(value):
        text = "" if value is None else str(value)
        if combo.currentText() == text:
            return
        combo.blockSignals(True)
        index = combo.findText(text)
        if index >= 0:
            combo.setCurrentIndex(index)
        elif combo.isEditable() or not text:
            combo.setEditText(text) if combo.isEditable() else combo.setCurrentIndex(-1)
        combo.blockSignals(False)

    if isinstance(choices, Cell):
        _owned(combo, choices.subscribe(set_items, call_now=True))
    elif choices is not None:
        set_items(choices)
    combo.currentTextChanged.connect(cell.set)
    _owned(combo, cell.subscribe(show, call_now=True))


def bind_enabled(cell: Cell, widget: QtWidgets.QWidget, invert: bool = False) -> None:
    _owned(widget, cell.subscribe(lambda value: widget.setEnabled(bool(value) != invert), call_now=True))


class LogConsole(QtWidgets.QPlainTextEdit):
    """Read-only, bounded, monospace log that follows its end."""

    def __init__(self, parent=None, *, max_lines: int = 1000):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setFont(mono_font())
        self.setMaximumBlockCount(max_lines)
        self.setLineWrapMode(QtWidgets.QPlainTextEdit.LineWrapMode.WidgetWidth)

    def append_lines(self, lines: Iterable[str]) -> None:
        lines = list(lines)
        if not lines:
            return
        self.appendPlainText("\n".join(lines))
        bar = self.verticalScrollBar()
        bar.setValue(bar.maximum())

    def line_count(self) -> int:
        return self.blockCount()


class RetilingGrid(QtWidgets.QWidget):
    """Lays its tiles out in as many equal columns as the width allows."""

    def __init__(self, parent=None, *, min_tile_width: int = 290):
        super().__init__(parent)
        self.min_tile_width = min_tile_width
        self.tiles: list[QtWidgets.QWidget] = []
        self.columns = 0
        self.grid = QtWidgets.QGridLayout(self)
        self.grid.setContentsMargins(4, 4, 4, 4)
        self.grid.setSpacing(6)

    def add_tile(self, widget: QtWidgets.QWidget) -> None:
        self.tiles.append(widget)
        self.columns = 0
        self._retile(self.width())

    def resizeEvent(self, event):  # noqa: N802 - Qt API
        super().resizeEvent(event)
        self._retile(event.size().width())

    def _retile(self, width: int) -> None:
        if not self.tiles:
            return
        tile_width = max(self.min_tile_width, max(t.sizeHint().width() for t in self.tiles) + 8)
        columns = max(1, min(len(self.tiles), max(1, width) // tile_width))
        if columns == self.columns:
            return
        self.columns = columns
        for tile in self.tiles:
            self.grid.removeWidget(tile)
        for column in range(len(self.tiles)):
            self.grid.setColumnStretch(column, 1 if column < columns else 0)
        for index, tile in enumerate(self.tiles):
            self.grid.addWidget(tile, index // columns, index % columns, QtCore.Qt.AlignmentFlag.AlignTop)
        self.grid.setRowStretch(len(self.tiles) // columns + 1, 1)


def scroll_area(widget: QtWidgets.QWidget, *, horizontal: bool = False) -> QtWidgets.QScrollArea:
    area = QtWidgets.QScrollArea()
    area.setWidgetResizable(True)
    area.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
    if not horizontal:
        area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAsNeeded)
    area.setWidget(widget)
    return area


def hbox(*widgets, spacing: int = 6, margins=(0, 0, 0, 0), stretch_last: bool = False) -> QtWidgets.QHBoxLayout:
    layout = QtWidgets.QHBoxLayout()
    layout.setSpacing(spacing)
    layout.setContentsMargins(*margins)
    for widget in widgets:
        if widget is None:
            layout.addStretch(1)
        elif isinstance(widget, QtWidgets.QLayout):
            layout.addLayout(widget)
        else:
            layout.addWidget(widget)
    if stretch_last:
        layout.addStretch(1)
    return layout


def wrap_label(text: str = "", *, muted: bool = False) -> QtWidgets.QLabel:
    label = QtWidgets.QLabel(text)
    label.setWordWrap(True)
    label.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Preferred)
    label.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
    if muted:
        label.setStyleSheet("color: #555;")
    return label


def set_background(widget: QtWidgets.QWidget, color: str | None, foreground: str | None = None) -> None:
    """Solid background (and optional text color) via a style sheet; None clears."""
    style = []
    if color:
        style.append(f"background-color: {color};")
    if foreground:
        style.append(f"color: {foreground};")
    widget.setStyleSheet(" ".join(style))


AGE_COLORS = ((2.0, None), (5.0, "#fff2a8"), (10.0, "#ffd27f"), (float("inf"), "#ff8a80"))


def age_color(age: float | None) -> str | None:
    if age is None:
        return None
    for limit, colour in AGE_COLORS:
        if age < limit:
            return colour
    return AGE_COLORS[-1][1]


def light_axes(plot_item, color: str = "#444444") -> None:
    """Dark axis lines/labels for a plot drawn on a light background (any system theme)."""
    for name in ("left", "bottom", "right", "top"):
        axis = plot_item.getAxis(name)
        axis.setPen(color)
        axis.setTextPen(color)
        if axis.label is not None:
            axis.label.setDefaultTextColor(QtGui.QColor(color))
