"""witnessos.py command line: --hitl-headless runs the HITL runner without Qt."""

import os
import subprocess
import sys
import unittest

WOS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SCRIPT = r"""
import contextlib, io, sys
sys.path.insert(0, {wos!r})
import witnessos
out = io.StringIO()
with contextlib.redirect_stdout(out):
    code = witnessos.main(["--hitl-headless", "--dry-run", "--no-save",
                           "--preset", "Pad abort - no launch"])
qt = sorted(m for m in sys.modules
            if m.split(".")[0] in ("PyQt6", "pyqtgraph", "qt_common",
                                   "wos_app", "wos_hitl_tab"))
print("CODE", code)
print("QT", ",".join(qt))
print("OUT", "Pad abort" in out.getvalue())
"""


class CliTests(unittest.TestCase):
    def test_hitl_headless_dry_run_does_not_import_qt(self):
        out = subprocess.run(
            [sys.executable, "-c", SCRIPT.format(wos=WOS_DIR)],
            capture_output=True, text=True, timeout=120, cwd=WOS_DIR)
        self.assertEqual(out.returncode, 0, out.stderr)
        lines = dict(line.split(" ", 1) if " " in line else (line, "")
                     for line in out.stdout.splitlines())
        self.assertEqual(lines.get("CODE"), "0", out.stdout)
        self.assertEqual(lines.get("QT"), "", f"Qt imported: {lines.get('QT')}")
        self.assertEqual(lines.get("OUT"), "True")

    def test_hitl_headless_bad_preset_exits_2(self):
        out = subprocess.run(
            [sys.executable, os.path.join(WOS_DIR, "witnessos.py"),
             "--hitl-headless", "--dry-run", "--no-save", "--preset", "nope"],
            capture_output=True, text=True, timeout=120, cwd=WOS_DIR)
        self.assertEqual(out.returncode, 2, out.stderr)


if __name__ == "__main__":
    unittest.main()
