"""Horizon Unified Ground Station (PyQt6 + pyqtgraph). Run with: python horizon.py.

Witness arrives from the Witness monitor's JSON feed (the "Witness monitor"
tab of witnessOS, 127.0.0.1:8766), Featherweight directly from its COM port,
and AIM from the broker used by the vendor AIM BASE app. All three inputs are
receive-only and have independent connection lifecycles.

Usage:
    python -m pip install -r requirements.txt
    python ../witnessOS/witnessos.py --tab monitor   # Witness ports, decoder, commands, feed
    python horizon.py    # Auto-connects to Witness, AIM MQTT and known Featherweight USB

Either app can start first. Defaults are in ground_settings.py; the Witness
feed uses 127.0.0.1:8766, not either of Witness's raw TCP inputs. Select
generic Featherweight adapters once here; after a valid packet their USB
identity is saved in auto_connect_featherweight.json. Named Featherweight
devices are discovered automatically. For AIM, leave the vendor AIM BASE
application running and publish to localhost:1883. Horizon starts an
embedded, local-only MQTT broker by default; disable it with the checkbox in
the AIM tab or --no-broker to use an external broker. The broker stops when
Horizon closes; retained messages are not persisted. Broker lifecycle/errors
are logged to logs/mqtt_broker_*.log. Each input has an Auto-connect
checkbox: turning it off keeps a healthy connection but stops future retries.
Disconnect also disables that input's auto-connect. No network range is
scanned. Horizon does not open the AIM USB device. Logs go in logs/: Witness
JSONL, Featherweight raw BIN plus diagnostic/packet JSONL, and AIM MQTT JSONL.

Windows > Visualizer (F9) opens a second window with the 3D orientation view
and the top-down map side by side (resizable splitter). The map is an
offline metric grid; with "OSM tiles" on it also draws OpenStreetMap tiles
cached under tile_cache/ (use --offline-map to never fetch). Tracks are
collected while the window is closed.

Flight View opens first. The Attitude tab draws the Witness quaternion as a 3D
rocket (live from the feed, or replaying a converter/HITL CSV); see README.md.
Use the Layout Editor tab to edit a separate draft, then Apply to update the
dashboard or Save JSON to reuse it. Applying resets graph history, not
receiver connections or logs. Start with a layout:
python horizon.py --flight-layout path/to/layout.json
See flight_view.py's module help for the layout schema and source selection.

Keys: F9 Visualizer, F11 fullscreen (Escape leaves), Ctrl+P pause Flight View,
Ctrl+1..6 switch tabs, Ctrl+Q quit.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from PyQt6 import QtGui, QtWidgets

from qt_common import ensure_app
from aim_tab import AimMqttTab
from attitude_view import AttitudeTab
from flight_data import DEFAULT_LAYOUT
from flight_editor import FlightLayoutEditor
from flight_view import FlightView
from ground_settings import (
    DEFAULT_AIM_MQTT_BASE_PATH, DEFAULT_AIM_MQTT_HOST, DEFAULT_AIM_MQTT_PORT, DEFAULT_AIM_MQTT_UUID,
    DEFAULT_MQTT_BROKER_ENABLED, DEFAULT_MQTT_BROKER_PORT,
)
from ground_tabs import FeatherweightTab, WitnessTab
from map_data import MapData
from map_tiles import TileCache
from pad_reference import PadReference
from visualizer import VisualizerWindow

LOG_DIR = Path(__file__).resolve().parent / "logs"
TAB_TITLES = ("Flight View", "Attitude", "Witness", "Featherweight GPS", "AIM XTRA MQTT", "Layout Editor")


class HorizonGroundStation(QtWidgets.QMainWindow):
    def __init__(self, *, log_dir=LOG_DIR, auto_connect=True, flight_layout=DEFAULT_LAYOUT,
                 broker_enabled=DEFAULT_MQTT_BROKER_ENABLED, broker_port=DEFAULT_MQTT_BROKER_PORT,
                 map_online=True, tile_cache=None, use_gl=None):
        ensure_app()
        super().__init__()
        # Pad reference and ground track are shared by Flight View (AGL altitudes,
        # distance) and the Visualizer map; both are fed on receiver threads.
        self.pad = PadReference()
        self.map_data = MapData(self.pad)
        self.tiles = tile_cache if tile_cache is not None else TileCache(online=map_online)
        self.use_gl = use_gl
        self.visualizer = None
        self._closed = False
        self.setWindowTitle("Horizon Unified Ground Station")
        screen = QtGui.QGuiApplication.primaryScreen()
        if screen is not None:
            area = screen.availableGeometry()
            self.resize(min(1440, area.width() - 40), min(900, area.height() - 100))
        self.notebook = QtWidgets.QTabWidget()
        self.notebook.setDocumentMode(True)
        self.setCentralWidget(self.notebook)
        self.flight = FlightView(layout_path=flight_layout, pad=self.pad)
        self.attitude = AttitudeTab(use_gl=use_gl)
        self.witness = WitnessTab(log_dir=log_dir, on_record=self._witness_record)
        self.featherweight = FeatherweightTab(log_dir=log_dir, on_record=self._featherweight_record)
        self.aim = AimMqttTab(log_dir=log_dir, host=DEFAULT_AIM_MQTT_HOST, port=DEFAULT_AIM_MQTT_PORT,
                              client_uuid=DEFAULT_AIM_MQTT_UUID, base_path=DEFAULT_AIM_MQTT_BASE_PATH,
                              on_sample=self.flight.data.accept_aim, broker_enabled=broker_enabled,
                              broker_port=broker_port)
        self.layout_editor = FlightLayoutEditor(flight=self.flight)
        for tab, title in zip((self.flight, self.attitude, self.witness, self.featherweight, self.aim,
                               self.layout_editor), TAB_TITLES):
            self.notebook.addTab(tab, title)
        for index in range(len(TAB_TITLES)):
            shortcut = QtGui.QShortcut(QtGui.QKeySequence(f"Ctrl+{index + 1}"), self)
            shortcut.activated.connect(lambda i=index: self.notebook.setCurrentIndex(i))
        for tab in (self.witness, self.featherweight, self.aim):
            tab.set_auto_connect(auto_connect)
        windows = self.menuBar().addMenu("&Windows")
        action = windows.addAction("Visualizer (orientation + map)")
        action.setShortcut(QtGui.QKeySequence("F9"))
        action.triggered.connect(self.open_visualizer)
        quit_action = windows.addAction("Quit")
        quit_action.setShortcut(QtGui.QKeySequence("Ctrl+Q"))
        quit_action.triggered.connect(self.close)

    def tab_titles(self):
        return [self.notebook.tabText(i) for i in range(self.notebook.count())]

    # Receiver callbacks (receiver threads; the models are thread-safe).
    def _witness_record(self, record):
        # Pad first, so this record's AGL altitude already uses a new pad fix.
        self.pad.accept_record("witness", record)
        self.map_data.accept_record("witness", record)
        self.flight.data.accept_record("witness", record)
        self.attitude.accept_record(record)

    def _featherweight_record(self, record):
        self.pad.accept_record("featherweight", record)
        self.map_data.accept_record("featherweight", record)
        self.flight.data.accept_record("featherweight", record)

    def open_visualizer(self):
        if self.visualizer is not None and self.visualizer.is_open():
            self.visualizer.showNormal()
            self.visualizer.raise_()
            self.visualizer.activateWindow()
            return self.visualizer
        # The Attitude tab's LiveAttitude already ingests every Witness record.
        self.visualizer = VisualizerWindow(live=self.attitude.live, map_data=self.map_data,
                                           tiles=self.tiles, use_gl=self.use_gl)
        self.visualizer.show()
        return self.visualizer

    def shutdown(self):
        if self._closed:
            return
        self._closed = True
        for tab in (self.witness, self.featherweight, self.aim):
            tab.close()
        if self.visualizer is not None and self.visualizer.is_open():
            self.visualizer.close()
        self.flight.close()
        self.attitude.close()
        self.tiles.close()

    def closeEvent(self, event):  # noqa: N802 - Qt API
        self.shutdown()
        super().closeEvent(event)

    def close(self):
        self.shutdown()
        return super().close()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Horizon Unified Ground Station")
    parser.add_argument("--flight-layout", default=DEFAULT_LAYOUT, type=Path,
                        help="Flight View JSON layout (default: flight_view_default.json next to horizon.py)")
    parser.add_argument("--no-auto-connect", action="store_true", help="start with all automatic connections off")
    parser.add_argument("--no-broker", action="store_true", help="do not start the embedded localhost MQTT broker")
    parser.add_argument("--offline-map", action="store_true",
                        help="never download OpenStreetMap tiles (cached tiles and the grid still show)")
    parser.add_argument("--no-gl", action="store_true", help="draw attitude as a 2D projection (no OpenGL)")
    args = parser.parse_args(argv)
    app = ensure_app()
    window = HorizonGroundStation(flight_layout=args.flight_layout, auto_connect=not args.no_auto_connect,
                                  broker_enabled=DEFAULT_MQTT_BROKER_ENABLED and not args.no_broker,
                                  map_online=not args.offline_map, use_gl=False if args.no_gl else None)
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
