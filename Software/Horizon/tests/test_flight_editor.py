"""Layout editor tests: the GUI-free draft model and the Qt editor (offscreen)."""

from copy import deepcopy
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from flight_data import DEFAULT_LAYOUT, load_layout  # noqa: E402
from flight_layout_draft import LayoutDraft  # noqa: E402


class LayoutDraftTests(unittest.TestCase):
    """No Qt: the editor's logic on its own."""

    def setUp(self):
        self.draft = LayoutDraft(load_layout(DEFAULT_LAYOUT).document)

    def test_tree_form_and_commit(self):
        keys = [key for key, _parent, _text in self.draft.tree()]
        self.assertEqual(keys[:3], ["layout", "plots", "plots:0"])
        self.assertIn("readouts:0:0", keys)
        self.assertEqual([f.key for f in self.draft.fields][:2], ["title", "window_seconds"])
        self.draft.variables["window_seconds"].set("45")
        self.assertTrue(self.draft.dirty)
        self.assertTrue(self.draft.select("plots:0:0"))
        self.assertEqual(self.draft.document["window_seconds"], 45.0)
        self.assertIn("state", self.draft.choices("packet"))
        self.assertFalse(self.draft.select("plots"))         # section headers are not editable

    def test_bad_value_blocks_selection(self):
        self.draft.variables["columns"].set("x")
        self.assertFalse(self.draft.select("plots:0"))
        self.assertEqual(self.draft.selected, "layout")
        self.assertIn("correct", self.draft.status.get())


class FlightEditorTests(unittest.TestCase):
    def setUp(self):
        from horizon_qt_support import make_horizon
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.app = make_horizon(self.directory.name)
        self.addCleanup(self.app.close)
        self.editor = self.app.layout_editor

    def test_draft_isolated_until_confirmed_apply(self):
        editor, flight = self.editor, self.app.flight
        original = deepcopy(flight.data.layout.document)
        editor.widgets["window_seconds"].setText("60")
        self.assertEqual(editor.variables["window_seconds"].get(), "60")
        self.assertTrue(editor.commit())
        self.assertEqual(flight.data.layout.document, original)
        histories = flight.data.histories
        with patch("flight_editor.ask_yes_no", return_value=False):
            self.assertFalse(editor.apply())
        self.assertIs(flight.data.histories, histories)
        with patch("flight_editor.ask_yes_no", return_value=True):
            self.assertTrue(editor.apply())
        self.assertEqual(flight.data.layout.window_seconds, 60)
        self.assertIsNot(flight.data.histories, histories)
        # Subsequent draft edits still cannot mutate the live layout.
        editor.variables["title"].set("Another draft")
        editor.commit()
        self.assertNotEqual(flight.data.layout.title, "Another draft")

    def test_save_load_custom_trace_round_trip(self):
        editor = self.editor
        original = deepcopy(self.app.flight.data.layout.document)
        editor.rebuild("plots:0:0")
        for key, value in {"packet": "custom_a, custom_b", "field": "custom.voltage",
                           "label": "My trace", "scale": "0.001", "offset": "2",
                           "color": "#123456", "source": "COM22",
                           "match": '{"unit_type":"TRACKER"}'}.items():
            editor.variables[key].set(value)
        self.assertEqual(editor.widgets["label"].text(), "My trace")
        self.assertEqual(editor.widgets["packet"].currentText(), "custom_a, custom_b")
        filename = Path(self.directory.name) / "custom.json"
        self.assertTrue(editor.save_file(filename))
        layout = load_layout(filename)
        trace = layout.plots[0].traces[0]
        self.assertEqual(trace.packet, ("custom_a", "custom_b"))
        self.assertEqual(trace.match, {"unit_type": "TRACKER"})
        self.assertEqual((trace.scale, trace.offset, trace.color), (.001, 2, "#123456"))
        self.assertFalse(editor.dirty)
        self.assertEqual(self.app.flight.data.layout.document, original)
        self.assertTrue(editor.load_file(filename))
        self.assertEqual(editor.document, layout.document)

    def test_validation_blocks_apply_save_and_bad_selection(self):
        editor = self.editor
        editor.variables["columns"].set("3")
        filename = Path(self.directory.name) / "invalid.json"
        with patch("flight_editor.ask_yes_no") as confirm:
            self.assertFalse(editor.apply())
            confirm.assert_not_called()
        self.assertFalse(editor.save_file(filename))
        self.assertFalse(filename.exists())
        self.assertIn("columns", editor.status.get())
        editor.variables["columns"].set("not a number")
        editor.select_item("plots:0")
        self.assertEqual(editor.selected, "layout")
        self.assertEqual(editor.selected_key(), "layout")
        editor.variables["columns"].set("2")
        editor.commit()
        editor.rebuild("readouts:0")
        editor.variables["group"].set("unknown section")
        self.assertFalse(editor.commit())
        self.assertIn("position", editor.status.get())

    def test_add_move_remove_and_suggestions(self):
        editor = self.editor
        original_titles = [row["title"] for row in editor.document["plots"]]
        editor.rebuild("plots:0")
        editor.move(1)
        self.assertEqual(editor.document["plots"][1]["title"], original_titles[0])
        self.assertEqual(editor.selected_key(), "plots:1")
        editor.add_trace()
        self.assertEqual(editor.variables["field"].get(), "kalman_altitude")
        editor.variables["input"].set("aim")
        editor.variables["packet"].set("pressure_payload/v1")
        editor.suggestions()
        self.assertIn("_derived.vertical_velocity_mps", editor.choices("field"))
        with patch("flight_editor.ask_yes_no", return_value=True):
            editor.remove()
        editor.add_row("readouts")
        self.assertEqual(editor.document["readouts"][-1]["group"], "power")
        with patch("flight_editor.ask_yes_no", return_value=True):
            editor.remove()
        self.assertIsNotNone(editor.validated())

    def test_replace_confirmation_and_load_failure(self):
        editor = self.editor
        self.app.flight.data.set_window(30)
        editor.variables["title"].set("Unsaved")
        with patch("flight_editor.ask_yes_no", return_value=False):
            editor.use_current()
        self.assertEqual(editor.variables["title"].get(), "Unsaved")
        with patch("flight_editor.ask_yes_no", return_value=True):
            editor.use_current()
        self.assertEqual(float(editor.variables["window_seconds"].get()), 30)
        self.assertFalse(editor.dirty)
        before = deepcopy(editor.document)
        self.assertFalse(editor.load_file(Path(self.directory.name) / "missing.json"))
        self.assertEqual(editor.document, before)
        self.assertIn("Load failed", editor.status.get())

    def test_all_trace_controls_reachable_on_small_screen(self):
        from PyQt6 import QtWidgets
        from horizon_qt_support import pump
        editor = self.editor
        self.app.resize(1000, 650)
        self.app.show()
        self.app.notebook.setCurrentWidget(editor)
        editor.rebuild("plots:0:0")
        pump(0.05)
        bar = editor.form_scroll.verticalScrollBar()
        bar.setValue(bar.maximum())
        pump(0.05)
        update = next(button for button in editor.form_host.findChildren(QtWidgets.QPushButton)
                      if button.text() == "Update draft item")
        viewport = editor.form_scroll.viewport()
        bottom = update.mapTo(viewport, update.rect().bottomLeft()).y()
        self.assertLessEqual(bottom, viewport.height())
        self.assertGreaterEqual(bottom, 0)


if __name__ == "__main__":
    unittest.main()
