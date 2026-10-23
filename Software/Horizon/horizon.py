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
AIM BASE application and its MQTT broker running. Each input has an Auto-connect
checkbox: turning it off keeps a healthy connection but stops future retries.
Disconnect also disables that input's auto-connect. No network range is scanned.
Horizon does not open the AIM USB device. Logs go in logs/: Witness JSONL,
Featherweight raw BIN plus diagnostic/packet JSONL, and AIM MQTT JSONL.

Flight View opens first. Edit flight_view_default.json, use Load layout, or
start with: python horizon.py --flight-layout path/to/layout.json
See flight_view.py's module help for the layout schema and source selection.
"""

import argparse
from pathlib import Path
import tkinter as tk
from tkinter import ttk

from aim_tab import AimMqttTab
from flight_data import DEFAULT_LAYOUT
from flight_view import FlightView
from ground_tabs import FeatherweightTab, WitnessTab
from ground_settings import (
    DEFAULT_AIM_MQTT_HOST, DEFAULT_AIM_MQTT_PORT,
    DEFAULT_AIM_MQTT_UUID, DEFAULT_AIM_MQTT_BASE_PATH,
)

LOG_DIR = Path(__file__).resolve().parent / "logs"


class HorizonGroundStation(tk.Tk):
    def __init__(self, *, log_dir=LOG_DIR, auto_connect=True, flight_layout=DEFAULT_LAYOUT):
        super().__init__()
        self.title("Horizon Unified Ground Station")
        self.geometry(f"{min(1440, self.winfo_screenwidth() - 40)}x{min(900, self.winfo_screenheight() - 100)}")
        self.minsize(min(1000, self.winfo_screenwidth() - 40), min(650, self.winfo_screenheight() - 100))
        ttk.Label(self, text="Horizon Unified Ground Station", font=("Segoe UI", 15, "bold"),
                  padding=(12, 8)).pack(anchor="w")
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.flight = FlightView(self.notebook, layout_path=flight_layout)
        self.witness = WitnessTab(self.notebook, log_dir=log_dir,
                                 on_record=lambda record: self.flight.data.accept_record("witness", record))
        self.featherweight = FeatherweightTab(self.notebook, log_dir=log_dir,
                                             on_record=lambda record: self.flight.data.accept_record("featherweight", record))
        self.aim = AimMqttTab(self.notebook, log_dir=log_dir, host=DEFAULT_AIM_MQTT_HOST,
                              port=DEFAULT_AIM_MQTT_PORT, client_uuid=DEFAULT_AIM_MQTT_UUID,
                              base_path=DEFAULT_AIM_MQTT_BASE_PATH, on_sample=self.flight.data.accept_aim)
        for tab, title in ((self.flight, "Flight View"), (self.witness, "Witness"), (self.featherweight, "Featherweight GPS"),
                           (self.aim, "AIM XTRA MQTT")):
            self.notebook.add(tab, text=title)
        for tab in (self.witness, self.featherweight, self.aim):
            tab.set_auto_connect(auto_connect)
        self.protocol("WM_DELETE_WINDOW", self.close)

    def close(self):
        for tab in (self.witness, self.featherweight, self.aim):
            tab.close()
        self.flight.close()
        self.destroy()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Horizon Unified Ground Station")
    parser.add_argument("--flight-layout", default=DEFAULT_LAYOUT, type=Path,
                        help="Flight View JSON layout (default: flight_view_default.json next to horizon.py)")
    parser.add_argument("--no-auto-connect", action="store_true", help="start with all automatic connections off")
    args = parser.parse_args()
    HorizonGroundStation(flight_layout=args.flight_layout, auto_connect=not args.no_auto_connect).mainloop()
