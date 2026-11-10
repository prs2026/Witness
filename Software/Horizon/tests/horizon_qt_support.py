"""Shared helpers for Horizon's Qt GUI tests (headless, offscreen platform)."""

import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PyQt6 import QtCore  # noqa: E402

import qt_common  # noqa: E402

app = qt_common.ensure_app()


def pump(seconds: float = 0.0) -> None:
    """Run the Qt event loop for ``seconds`` (at least one pass)."""
    deadline = time.monotonic() + seconds
    while True:
        app.processEvents()
        if time.monotonic() >= deadline:
            break
        QtCore.QThread.msleep(5)


def pump_until(predicate, timeout: float = 5.0) -> bool:
    return qt_common.process_events_until(predicate, timeout)


def make_monitor(log_dir, **options):
    """A MonitorPanel with the feed on an ephemeral port and logs in log_dir."""
    from pathlib import Path
    from unittest.mock import patch
    import main
    options.setdefault("auto_connect", False)
    with patch("monitor_core.DEFAULT_WITNESS_FEED_PORT", 0), patch("monitor_core.LOG_DIR", Path(log_dir)):
        return main.MonitorPanel(**options)


def make_horizon(log_dir, **options):
    """Horizon main window with no broker, no auto-connect, offline tiles, no GL."""
    from pathlib import Path
    from horizon import HorizonGroundStation
    from map_tiles import TileCache
    options.setdefault("auto_connect", False)
    options.setdefault("broker_enabled", False)
    options.setdefault("use_gl", False)
    options.setdefault("tile_cache", TileCache(Path(log_dir) / "tiles", fetch=None))
    return HorizonGroundStation(log_dir=Path(log_dir), **options)
