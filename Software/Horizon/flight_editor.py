"""Form-based editor for Flight View JSON (Qt); changes stay in a separate draft.

Applying is explicit and resets plot history/source bindings, never receivers
or logs. Saving writes the draft, not the live layout. The draft logic is in
flight_layout_draft.LayoutDraft; this widget is a tree + form over it.
"""

from __future__ import annotations

from PyQt6 import QtCore, QtGui, QtWidgets

from flight_layout_draft import (  # noqa: F401 - re-exported
    DEFAULT_LAYOUT, FIELDS, GROUPS, HINT, WITNESS_EDITOR_FIELDS, LayoutDraft,
)
from qt_widgets import bind_check, bind_combo, bind_label, bind_line_edit, wrap_label


def ask_yes_no(parent, title, text) -> bool:
    """Confirmation dialog (tests patch this)."""
    answer = QtWidgets.QMessageBox.question(
        parent, title, text,
        QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
        QtWidgets.QMessageBox.StandardButton.No)
    return answer == QtWidgets.QMessageBox.StandardButton.Yes


class FlightLayoutEditor(QtWidgets.QWidget):
    def __init__(self, parent=None, flight=None):
        super().__init__(parent)
        self.flight = flight
        self.draft = LayoutDraft(flight.data.layout.document)
        self.status = self.draft.status
        self.widgets = {}
        self._items = {}
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        toolbar = QtWidgets.QHBoxLayout()
        for label, slot in (("Use current view", self.use_current), ("Load JSON", self.load_file),
                            ("Defaults", self.defaults), ("Apply to Flight View", self.apply),
                            ("Save JSON as...", self.save_file)):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(lambda _c=False, s=slot: s())
            toolbar.addWidget(button)
        toolbar.addStretch(1)
        layout.addLayout(toolbar)
        status = wrap_label()
        bind_label(self.status, status)
        layout.addWidget(status)
        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        splitter.setChildrenCollapsible(False)
        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        self.tree = QtWidgets.QTreeWidget()
        self.tree.setHeaderHidden(True)
        self.tree.currentItemChanged.connect(self._select)
        left_layout.addWidget(self.tree, 1)
        buttons = QtWidgets.QGridLayout()
        for index, (label, slot) in enumerate((
                ("Add graph", lambda: self.add_row("plots")), ("Add readout", lambda: self.add_row("readouts")),
                ("Add trace", self.add_trace), ("Remove", self.remove),
                ("Move up", lambda: self.move(-1)), ("Move down", lambda: self.move(1)))):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(lambda _c=False, s=slot: s())
            buttons.addWidget(button, index // 2, index % 2)
        left_layout.addLayout(buttons)
        splitter.addWidget(left)
        self.form_host = QtWidgets.QWidget()
        self.form_scroll = QtWidgets.QScrollArea()
        self.form_scroll.setWidgetResizable(True)
        self.form_scroll.setWidget(self.form_host)
        splitter.addWidget(self.form_scroll)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)
        layout.addWidget(splitter, 1)
        self.rebuild("layout")

    # Tk-era attribute names.
    @property
    def document(self):
        return self.draft.document

    @property
    def selected(self):
        return self.draft.selected

    @property
    def variables(self):
        return self.draft.variables

    @property
    def dirty(self):
        return self.draft.dirty

    @property
    def path(self):
        return self.draft.path

    def commit(self):
        return self.draft.commit()

    # ----------------------------------------------------------------- tree
    def rebuild(self, select=None):
        if select is not None and select != self.draft.selected:
            self.draft.select_after_edit(select)
        self.tree.blockSignals(True)
        self.tree.clear()
        self._items = {}
        for key, parent, text in self.draft.tree():
            parent_item = self._items.get(parent, self.tree.invisibleRootItem())
            item = QtWidgets.QTreeWidgetItem(parent_item, [text])
            item.setData(0, QtCore.Qt.ItemDataRole.UserRole, key)
            item.setExpanded(True)
            self._items[key] = item
        self.tree.setCurrentItem(self._items[self.draft.selected])
        self.tree.blockSignals(False)
        self.show_form()

    def selected_key(self):
        item = self.tree.currentItem()
        return item.data(0, QtCore.Qt.ItemDataRole.UserRole) if item is not None else None

    def select_item(self, key):
        """Select ``key`` in the tree as a user click would."""
        self.tree.setCurrentItem(self._items[key])

    def _select(self, item, _previous=None):
        key = item.data(0, QtCore.Qt.ItemDataRole.UserRole) if item is not None else None
        if key is None or key == self.draft.selected:
            return
        if not self.draft.select(key):
            self.tree.blockSignals(True)
            self.tree.setCurrentItem(self._items[self.draft.selected])
            self.tree.blockSignals(False)
            return
        self.show_form()

    # ----------------------------------------------------------------- form
    def show_form(self):
        self.form_host = QtWidgets.QWidget()
        form = QtWidgets.QFormLayout(self.form_host)
        form.setFieldGrowthPolicy(QtWidgets.QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        title = QtWidgets.QLabel(self.draft.title)
        font = title.font()
        font.setPointSize(12)
        font.setBold(True)
        title.setFont(font)
        form.addRow(title)
        self.widgets = {}
        self.draft.loading = True
        for field in self.draft.fields:
            cell = self.draft.variables[field.key]
            if field.kind == "bool":
                widget = QtWidgets.QCheckBox()
                bind_check(cell, widget)
            elif field.kind == "choice":
                widget = QtWidgets.QComboBox()
                widget.setEditable(True)
                bind_combo(cell, widget, field.choices or [])
                if field.key in ("input", "packet"):
                    widget.activated.connect(lambda _i: self.suggestions())
            else:
                widget = QtWidgets.QLineEdit()
                bind_line_edit(cell, widget)
            self.widgets[field.key] = widget
            form.addRow(field.title, widget)
        self.draft.loading = False
        if "color" in self.widgets:
            pick = QtWidgets.QPushButton("Choose color...")
            pick.clicked.connect(self.pick_color)
            form.addRow("", pick)
        form.addRow(wrap_label(HINT, muted=True))
        update = QtWidgets.QPushButton("Update draft item")
        update.clicked.connect(self.update_item)
        form.addRow(update)
        self.form_scroll.setWidget(self.form_host)
        self.form_scroll.verticalScrollBar().setValue(0)

    def choices(self, key):
        widget = self.widgets.get(key)
        if isinstance(widget, QtWidgets.QComboBox):
            return [widget.itemText(i) for i in range(widget.count())]
        return self.draft.choices(key)

    def suggestions(self):
        self.draft.suggestions()
        for key in ("packet", "field"):
            widget = self.widgets.get(key)
            if widget is None:
                continue
            text = widget.currentText()
            widget.blockSignals(True)
            widget.clear()
            widget.addItems(self.draft.choices(key) or [])
            widget.setEditText(text)
            widget.blockSignals(False)

    def pick_color(self):
        current = QtGui.QColor(str(self.draft.variables["color"].get()))
        color = QtWidgets.QColorDialog.getColor(current, self, "Trace color")
        if color.isValid():
            self.draft.variables["color"].set(color.name())

    # -------------------------------------------------------------- editing
    def update_item(self):
        if self.draft.update_item():
            self.rebuild()

    def add_row(self, section):
        if self.draft.add_row(section) is not None:
            self.rebuild()

    def add_trace(self):
        if self.draft.add_trace() is not None:
            self.rebuild()

    def remove(self):
        if not self.draft.selected or self.draft.selected == "layout":
            return
        if not ask_yes_no(self, "Remove item", "Remove the selected item from the draft?"):
            return
        self.draft.remove()
        self.rebuild()

    def move(self, direction):
        if self.draft.move(direction):
            self.rebuild()

    def replace_draft(self, document, path=None):
        if self.draft.dirty and not ask_yes_no(self, "Replace draft", "Discard unsaved editor changes?"):
            return False
        self.draft.replace(document, path)
        self.rebuild()
        return True

    def use_current(self):
        return self.replace_draft(self.flight.data.layout.document)

    def defaults(self):
        return self.load_file(DEFAULT_LAYOUT)

    def load_file(self, filename=None):
        if filename is None:
            filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load layout", "", "Layout JSON (*.json)")
        if not filename:
            return False
        try:
            document = self.draft.load_document(filename)
        except (OSError, ValueError) as exc:
            self.status.set(f"Load failed: {exc}")
            return False
        return self.replace_draft(document, filename)

    def validated(self):
        return self.draft.validated()

    def apply(self):
        layout = self.draft.validated()
        if layout is None:
            return False
        if not ask_yes_no(self, "Apply layout", "Apply this draft to Flight View? Graph history and source "
                          "bindings will reset. Logging and connections continue."):
            return False
        from pathlib import Path
        self.flight.apply_layout(layout, self.draft.path or Path("Unsaved editor layout"))
        self.status.set("Applied to Flight View. Save JSON to keep this setup for next time.")
        return True

    def save_file(self, filename=None):
        if self.draft.validated() is None:
            return False
        if filename is None:
            filename, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, "Save layout", "flight_view_custom.json", "Layout JSON (*.json)")
        if not filename:
            return False
        return self.draft.save(filename)
