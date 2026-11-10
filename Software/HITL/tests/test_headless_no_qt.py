"""--headless must never import Qt (scripts and the bench run it)."""

import os
import subprocess
import sys
import unittest

HITL_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SCRIPT = r"""
import contextlib, io, sys
sys.path.insert(0, {hitl!r})
import hitl_gui
with contextlib.redirect_stdout(io.StringIO()):
    code = hitl_gui.main(["--headless", "--dry-run", "--no-save"])
qt = sorted(m for m in sys.modules
            if m.split(".")[0] in ("PyQt6", "pyqtgraph", "qt_common", "hitl_qt"))
print("CODE", code)
print("QT", ",".join(qt))
"""


class HeadlessImportTests(unittest.TestCase):
    def test_headless_dry_run_does_not_import_qt(self):
        out = subprocess.run(
            [sys.executable, "-c", SCRIPT.format(hitl=HITL_DIR)],
            capture_output=True, text=True, timeout=120, cwd=HITL_DIR)
        self.assertEqual(out.returncode, 0, out.stderr)
        lines = dict(line.split(" ", 1) if " " in line else (line, "")
                     for line in out.stdout.splitlines())
        self.assertEqual(lines.get("CODE"), "0", out.stdout)
        self.assertEqual(lines.get("QT"), "", f"Qt imported: {lines.get('QT')}")

    def test_script_is_a_deprecated_alias_and_gui_moved_to_witnessos(self):
        script = os.path.join(HITL_DIR, "hitl_gui.py")
        out = subprocess.run(
            [sys.executable, script, "--headless", "--dry-run", "--no-save"],
            capture_output=True, text=True, timeout=120, cwd=HITL_DIR)
        self.assertEqual(out.returncode, 0, out.stderr)
        self.assertIn("deprecated", out.stderr)
        self.assertIn("witnessos.py --hitl-headless", out.stderr)
        self.assertIn("Expected:", out.stdout)
        out = subprocess.run([sys.executable, script], capture_output=True,
                             text=True, timeout=120, cwd=HITL_DIR)
        self.assertNotEqual(out.returncode, 0)
        self.assertIn("witnessOS", out.stderr)


if __name__ == "__main__":
    unittest.main()
