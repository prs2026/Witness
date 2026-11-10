"""Horizon Unified Ground Station. Run with: python horizon.py.

Witness arrives from the running main.py application, Featherweight directly
from its COM port, and AIM from the broker used by the vendor AIM BASE app.
All three inputs are receive-only and have independent connection lifecycles.

Usage:
    python -m pip install -r requirements.txt
    python main.py       # Witness ports, decoder, commands, graphs and raw logs
    python horizon.py    # Auto-connects to Witness, AIM MQTT and known Featherweight USB

Restart an already-running main.py once after upgrading to enable its feed.
Either app can subsequently start first. Defaults are in ground_settings.py;
the Witness feed uses 127.0.0.1:8766, not either of Witness's raw TCP inputs.
Select generic Featherweight adapters once here (not in main.py); after a valid
packet their USB identity is saved in auto_connect_featherweight.json. Named
Featherweight devices are discovered automatically. For AIM, leave the vendor
AIM BASE application running and publish to localhost:1883. Horizon starts an
embedded, local-only MQTT broker by default; disable it with the checkbox in
the AIM tab or --no-broker to use an external broker. No separate broker app is
needed. The broker stops when Horizon closes; retained messages are not persisted.
Install requirements.txt after upgrading. Broker lifecycle/errors are logged
to logs/mqtt_broker_*.log. Each input has an Auto-connect
checkbox: turning it off keeps a healthy connection but stops future retries.
Disconnect also disables that input's auto-connect. No network range is scanned.
Horizon does not open the AIM USB device. Logs go in logs/: Witness JSONL,
Featherweight raw BIN plus diagnostic/packet JSONL, and AIM MQTT JSONL.

Windows > Visualizer (or F9) opens a second window: an Orientation tab (live
Witness quaternion as a 3D rocket with tilt/roll/heading) and a top-down Map
of the rocket relative to the pad. The map is an offline grid; with "OSM
tiles" on it also draws OpenStreetMap tiles cached under tile_cache/ (use
--offline-map to never fetch). Tracks are collected while the window is closed.

Flight View opens first. The Attitude tab draws the Witness quaternion as a 3D
rocket (live from the feed, or replaying a converter/HITL CSV); see README.md.
Use the Layout Editor tab to edit a separate draft,
then Apply to update the dashboard or Save JSON to reuse it. Applying resets
graph history, not receiver connections or logs. You can also load a layout or
start with: python horizon.py --flight-layout path/to/layout.json
See flight_view.py's module help for the layout schema and source selection.
"""

import argparse
from pathlib import Path
import tkinter as tk
from tkinter import ttk

from aim_tab import AimMqttTab
from attitude_view import AttitudeTab
from flight_data import DEFAULT_LAYOUT
from flight_view import FlightView
from flight_editor import FlightLayoutEditor
from ground_tabs import FeatherweightTab, WitnessTab
from map_data import MapData
from map_tiles import TileCache
from pad_reference import PadReference
from visualizer import VisualizerWindow
from ground_settings import (
    DEFAULT_AIM_MQTT_HOST, DEFAULT_AIM_MQTT_PORT,
    DEFAULT_AIM_MQTT_UUID, DEFAULT_AIM_MQTT_BASE_PATH,
    DEFAULT_MQTT_BROKER_ENABLED, DEFAULT_MQTT_BROKER_PORT,
)

LOG_DIR = Path(__file__).resolve().parent / "logs"


class HorizonGroundStation(tk.Tk):
    def __init__(self, *, log_dir=LOG_DIR, auto_connect=True, flight_layout=DEFAULT_LAYOUT,
                 broker_enabled=DEFAULT_MQTT_BROKER_ENABLED, broker_port=DEFAULT_MQTT_BROKER_PORT,
                 map_online=True, tile_cache=None):
        super().__init__()
        # Pad reference and ground track are shared by Flight View (AGL altitudes,
        # distance) and the Visualizer map; both are fed on receiver threads.
        self.pad = PadReference()
        self.map_data = MapData(self.pad)
        self.tiles = tile_cache if tile_cache is not None else TileCache(online=map_online)
        self.visualizer = None
        self.title("Horizon Unified Ground Station")
        self.geometry(f"{min(1440, self.winfo_screenwidth() - 40)}x{min(900, self.winfo_screenheight() - 100)}")
        self.minsize(min(1000, self.winfo_screenwidth() - 40), min(650, self.winfo_screenheight() - 100))
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True, padx=4, pady=(2, 4))
        self.flight = FlightView(self.notebook, layout_path=flight_layout, pad=self.pad)
        self.attitude = AttitudeTab(self.notebook)
        self.witness = WitnessTab(self.notebook, log_dir=log_dir, on_record=self._witness_record)
        self.featherweight = FeatherweightTab(self.notebook, log_dir=log_dir,
                                             on_record=self._featherweight_record)
        self.aim = AimMqttTab(self.notebook, log_dir=log_dir, host=DEFAULT_AIM_MQTT_HOST,
                              port=DEFAULT_AIM_MQTT_PORT, client_uuid=DEFAULT_AIM_MQTT_UUID,
                              base_path=DEFAULT_AIM_MQTT_BASE_PATH, on_sample=self.flight.data.accept_aim,
                              broker_enabled=broker_enabled, broker_port=broker_port)
        self.layout_editor = FlightLayoutEditor(self.notebook, self.flight)
        for tab, title in ((self.flight, "Flight View"), (self.attitude, "Attitude"), (self.witness, "Witness"),
                           (self.featherweight, "Featherweight GPS"), (self.aim, "AIM XTRA MQTT"),
                           (self.layout_editor, "Layout Editor")):
            self.notebook.add(tab, text=title)
        for tab in (self.witness, self.featherweight, self.aim):
            tab.set_auto_connect(auto_connect)
        menubar = tk.Menu(self)
        windows = tk.Menu(menubar, tearoff=False)
        windows.add_command(label="Visualizer (orientation + map)", accelerator="F9",
                            command=self.open_visualizer)
        menubar.add_cascade(label="Windows", menu=windows)
        self.configure(menu=menubar)
        self.bind("<F9>", lambda _event: self.open_visualizer())
        self.protocol("WM_DELETE_WINDOW", self.close)

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
        if self.visualizer is not None and self.visualizer.winfo_exists():
            self.visualizer.deiconify()
            self.visualizer.lift()
            return self.visualizer
        # The Attitude tab's LiveAttitude already ingests every Witness record.
        self.visualizer = VisualizerWindow(self, live=self.attitude.live, map_data=self.map_data,
                                           tiles=self.tiles)
        return self.visualizer

    def close(self):
        for tab in (self.witness, self.featherweight, self.aim):
            tab.close()
        if self.visualizer is not None and self.visualizer.winfo_exists():
            self.visualizer.close()
        self.flight.close()
        self.attitude.close()
        self.tiles.close()
        self.destroy()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Horizon Unified Ground Station")
    parser.add_argument("--flight-layout", default=DEFAULT_LAYOUT, type=Path,
                        help="Flight View JSON layout (default: flight_view_default.json next to horizon.py)")
    parser.add_argument("--no-auto-connect", action="store_true", help="start with all automatic connections off")
    parser.add_argument("--no-broker", action="store_true", help="do not start the embedded localhost MQTT broker")
    parser.add_argument("--offline-map", action="store_true",
                        help="never download OpenStreetMap tiles (cached tiles and the grid still show)")
    args = parser.parse_args()
    HorizonGroundStation(flight_layout=args.flight_layout, auto_connect=not args.no_auto_connect,
                         broker_enabled=DEFAULT_MQTT_BROKER_ENABLED and not args.no_broker,
                         map_online=not args.offline_map).mainloop()
