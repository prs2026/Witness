"""witnessOS: the Witness bench tools in one tabbed window (PyQt6).

    python witnessos.py                  # all three tabs
    python witnessos.py flight.csv       # open a converted CSV in the viewer
    python witnessos.py a.csv b.csv      # overlay several CSVs
    python witnessos.py --tab monitor    # start on a given tab
    python witnessos.py --no-feed        # do not start the Horizon feed
    python witnessos.py --no-auto-connect  # monitor: no automatic USB opens
    python witnessos.py --hitl-headless [HITL args]   # bench automation, no GUI

Tabs:
  HITL bench       wos_hitl_tab.py (HitlWidget) on Software/HITL's session,
                   simulation and trajectory modules
  CSV viewer       Software/Horizon/viewer.py (CsvViewerWidget)
  Witness monitor  wos_monitor_tab.py: Horizon's monitor (main.MonitorPanel:
                   telemetry readout, graphs, console, commands and Horizon
                   JSON feed) plus mirroring of the HITL tab's connection

``--hitl-headless`` runs one HITL simulation without a window and takes the
same arguments as the former ``hitl_gui.py --headless`` (``--hitl-headless
--help`` lists them). It never imports Qt: this module imports the window
(wos_app.py) only when the GUI starts.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import wos_paths  # noqa: F401  (must precede the tool imports)

APP_NAME = "witnessOS"
TABS = ("hitl", "viewer", "monitor")
TAB_TITLES = {"hitl": "HITL bench", "viewer": "CSV viewer",
              "monitor": "Witness monitor"}
TAB_ALIASES = {"relay": "monitor"}
HITL_HEADLESS = "--hitl-headless"


def __getattr__(name):
    # witnessos.WitnessOS etc. without importing Qt for the headless path.
    if name in ("WitnessOS", "run"):
        import wos_app
        return getattr(wos_app, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("csv_files", nargs="*", type=Path,
                    help="converted CSV(s) to open (overlaid) in the viewer")
    ap.add_argument("--tab", choices=TABS + tuple(TAB_ALIASES),
                    default="hitl", help="tab to show first")
    ap.add_argument("--no-feed", action="store_true",
                    help="do not start the Horizon Witness feed at launch")
    ap.add_argument("--no-auto-connect", action="store_true",
                    help="monitor: do not open Witness USB ports "
                    "automatically")
    ap.add_argument(HITL_HEADLESS, action="store_true",
                    help="run one HITL simulation without the GUI; the "
                    "following arguments are the HITL runner's "
                    f"({HITL_HEADLESS} --help lists them)")
    return ap


def run_hitl_headless(argv: list[str]) -> int:
    """The HITL headless runner (Qt-free)."""
    import hitl_gui
    return hitl_gui.main(["--headless", *argv])


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if HITL_HEADLESS in argv:
        i = argv.index(HITL_HEADLESS)
        return run_hitl_headless(argv[:i] + argv[i + 1:])
    args = build_parser().parse_args(argv)
    import wos_app   # PyQt6 is imported only for the GUI
    return wos_app.run(args)


if __name__ == "__main__":
    sys.exit(main())
