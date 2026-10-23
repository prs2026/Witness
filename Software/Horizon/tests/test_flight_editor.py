"""Layout editor tests with hidden Tk windows and no device connections."""

from copy import deepcopy
from pathlib import Path
import tempfile
import tkinter as tk
from tkinter import ttk
import unittest
from unittest.mock import patch

from flight_data import load_layout
from horizon import HorizonGroundStation


class FlightEditorTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        try:
            self.app = HorizonGroundStation(log_dir=Path(self.directory.name), auto_connect=False)
        except tk.TclError as exc:
            self.skipTest(f"Tk display unavailable: {exc}")
        self.app.withdraw()
        self.addCleanup(self.app.close)
        self.editor = self.app.layout_editor

    def test_draft_isolated_until_confirmed_apply(self):
        editor, flight = self.editor, self.app.flight
        original = deepcopy(flight.data.layout.document)
        editor.variables["window_seconds"].set("60")
        self.assertTrue(editor.commit())
        self.assertEqual(flight.data.layout.document, original)
        histories = flight.data.histories
        with patch("flight_editor.messagebox.askyesno", return_value=False):
            self.assertFalse(editor.apply())
        self.assertIs(flight.data.histories, histories)
        with patch("flight_editor.messagebox.askyesno", return_value=True):
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
        with patch("flight_editor.messagebox.askyesno") as confirm:
            self.assertFalse(editor.apply())
            confirm.assert_not_called()
        self.assertFalse(editor.save_file(filename))
        self.assertFalse(filename.exists())
        self.assertIn("columns", editor.status.get())
        editor.variables["columns"].set("not a number")
        editor.tree.selection_set("plots:0")
        editor._select()
        self.assertEqual(editor.selected, "layout")
        self.assertEqual(editor.tree.selection(), ("layout",))
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
        editor.add_trace()
        self.assertEqual(editor.variables["field"].get(), "barometric_altitude")
        editor.variables["input"].set("aim")
        editor.variables["packet"].set("pressure_payload/v1")
        editor.suggestions()
        self.assertIn("_derived.vertical_velocity_mps", editor.widgets["field"]["values"])
        with patch("flight_editor.messagebox.askyesno", return_value=True):
            editor.remove()
        editor.add_row("readouts")
        self.assertEqual(editor.document["readouts"][-1]["group"], "power")
        with patch("flight_editor.messagebox.askyesno", return_value=True):
            editor.remove()
        self.assertIsNotNone(editor.validated())

    def test_replace_confirmation_and_load_failure(self):
        editor = self.editor
        self.app.flight.data.set_window(30)
        editor.variables["title"].set("Unsaved")
        with patch("flight_editor.messagebox.askyesno", return_value=False):
            editor.use_current()
        self.assertEqual(editor.variables["title"].get(), "Unsaved")
        with patch("flight_editor.messagebox.askyesno", return_value=True):
            editor.use_current()
        self.assertEqual(float(editor.variables["window_seconds"].get()), 30)
        self.assertFalse(editor.dirty)
        before = deepcopy(editor.document)
        self.assertFalse(editor.load_file(Path(self.directory.name) / "missing.json"))
        self.assertEqual(editor.document, before)
        self.assertIn("Load failed", editor.status.get())

    def test_all_trace_controls_reachable_on_small_screen(self):
        editor = self.editor
        self.app.attributes("-alpha", 0)
        self.app.deiconify()
        self.app.geometry("1000x650")
        self.app.notebook.select(editor)
        editor.rebuild("plots:0:0")
        self.app.update()
        editor.canvas.yview_moveto(1)
        self.app.update_idletasks()
        self.assertAlmostEqual(editor.canvas.yview()[1], 1, places=2)
        update_button = next(widget for widget in editor.form.winfo_children()
                             if isinstance(widget, ttk.Button)
                             if widget.cget("text") == "Update draft item")
        self.assertLessEqual(update_button.winfo_rooty() + update_button.winfo_height(),
                             editor.canvas.winfo_rooty() + editor.canvas.winfo_height())


if __name__ == "__main__":
    unittest.main()
