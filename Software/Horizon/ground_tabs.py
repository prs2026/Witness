"""Witness relay and Featherweight serial tabs for Horizon Unified."""

import tkinter as tk
from tkinter import messagebox, ttk

from featherweight import FEATHERWEIGHT_BAUD
from ground_receivers import FeatherweightReceiver, WitnessReceiver
from ground_settings import WITNESS_FEED_HOST, WITNESS_FEED_PORT
from telemetry_view import TelemetryView

WITNESS_FIELDS = {
    "sensors": ("status", "uptime", "filtered_accel[0]", "filtered_accel[1]", "filtered_accel[2]",
                "gyro[0]", "gyro[1]", "gyro[2]", "sensor_trailing_u16"),
    "state": ("status", "uptime", "orientation[0]", "orientation[1]", "orientation[2]", "orientation[3]",
              "position[0]", "position[1]", "position[2]", "velocity[0]", "velocity[1]", "velocity[2]",
              "latitude", "longitude", "gps_altitude", "barometric_altitude", "witness_battery_voltage"),
    "camera": ("fc_status", "fc_uptime", "status", "uptime", "current_sense[0]", "current_sense[1]",
               "current_sense[2]", "iris_battery_voltage"),
    "heartbeat": ("status", "uptime"),
    "command": ("status", "uptime", "command_value"),
    "ground_station_status": ("rssi", "uptime", *(f"reserved[{i}]" for i in range(6))),
    "witness_debug": ("status", "uptime", "mcu_temperature", "witness_battery_voltage",
                      *(f"{name}[{i}]" for name, count in (("filtered_accel", 3), ("orientation", 4),
                        ("position", 3), ("velocity", 3), ("low_accel", 3), ("high_accel", 3), ("gyro", 3))
                        for i in range(count)), "lsm6_temperature", "pressure", "ms5607_temperature",
                      "barometric_altitude", "latitude", "longitude", "gps_time", "gps_altitude"),
    "iris_debug": ("status", "uptime", "mcu_temperature", "iris_battery_voltage",
                   "current_sense[0]", "current_sense[1]", "current_sense[2]"),
}
FEATHERWEIGHT_FIELDS = {"GPS_STAT": (
    "packet_type", "timestamp", "packet_length", "radio_crc_status", "text_crc_valid",
    "text_crc_expected", "text_crc_computed", "unit_type", "unit_id", "altitude_ft", "latitude_deg",
    "longitude_deg", "horizontal_velocity_fps", "heading_deg", "vertical_velocity_fps", "fix_type",
    "satellites_total", "satellites_over_24db", "satellites_over_32db", "satellites_over_40db",
    *(f"satellite_details[{i}]" for i in range(5)),
)}


class ReceiverTab(ttk.Frame):
    def __init__(self, parent, log_dir, *, witness=False, schema=None, on_record=None):
        super().__init__(parent, padding=8)
        self.log_dir, self.witness = log_dir, witness
        self.receiver = None
        self.on_record = on_record
        self.entries = []
        self.columnconfigure(0, weight=1)
        self.rowconfigure(4, weight=1)
        self.controls = ttk.Frame(self)
        self.controls.grid(row=0, column=0, sticky="ew")
        self.status = tk.StringVar(value="Disconnected")
        self.stats = tk.StringVar(value="Packets: 0   Bytes: 0   Errors: 0")
        self.log_status = tk.StringVar()
        for row, variable in ((1, self.status), (2, self.stats), (5, self.log_status)):
            label = ttk.Label(self, textvariable=variable, wraplength=800)
            label.grid(row=row, column=0, sticky="ew", pady=3)
            label.bind("<Configure>", lambda event, widget=label: widget.configure(
                wraplength=max(120, event.width)))
        self.note = ttk.Label(self, wraplength=850)
        self.note.grid(row=3, column=0, sticky="ew", pady=(0, 6))
        self.view = TelemetryView(self, schema=schema)
        self.view.grid(row=4, column=0, sticky="nsew")
        self._after_id = self.after(100, self._poll)

    def toggle_connection(self):
        if self.receiver is not None:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        if self.receiver is not None:
            return
        try:
            receiver = self.make_receiver()
            receiver.start()
        except (ValueError, RuntimeError, OSError) as exc:
            messagebox.showerror("Input connection", str(exc), parent=self)
            return
        self.receiver = receiver
        self.view.clear()
        self.prepare_view()
        self.status.set("Connecting...")
        self.log_status.set(f"Log: {receiver.log_path}")
        for entry in self.entries:
            entry.configure(state="disabled")
        self.connect_button.configure(text="Disconnect")

    def prepare_view(self):
        pass

    def _consume(self, receiver):
        updates, stats, diagnostics = receiver.drain()
        status, messages, byte_count, errors = stats
        self.status.set(status)
        self.stats.set(f"Packets: {messages}   Bytes: {byte_count}   Errors: {errors}")
        for record in updates:
            try:
                self.view.update_record(record, witness=self.witness)
            except (ValueError, TypeError, OverflowError, OSError) as exc:
                diagnostics.append(f"DISPLAY_ERROR: {exc}")
        self.view.append(diagnostics)

    def _poll(self):
        try:
            if self.receiver is not None:
                self._consume(self.receiver)
            self.view.update_ages()
        finally:
            self._after_id = self.after(100, self._poll)

    def disconnect(self):
        receiver, self.receiver = self.receiver, None
        if receiver is not None:
            self._consume(receiver)
            # Wait for the old reader to release its COM port/log files before
            # allowing another connection (serial reads time out after 0.2 s).
            receiver.stop(wait=True)
        self.status.set("Disconnected - latest values retained")
        self.connect_button.configure(text="Connect")
        for entry in self.entries:
            entry.configure(state="readonly" if isinstance(entry, ttk.Combobox) else "normal")

    def close(self):
        if self._after_id is not None:
            self.after_cancel(self._after_id)
            self._after_id = None
        self.disconnect()


class WitnessTab(ReceiverTab):
    def __init__(self, parent, *, log_dir, on_record=None):
        super().__init__(parent, log_dir, witness=True, schema=WITNESS_FIELDS, on_record=on_record)
        self.host = tk.StringVar(value=WITNESS_FEED_HOST)
        self.port = tk.StringVar(value=str(WITNESS_FEED_PORT))
        for column, (label, variable, width) in enumerate((
                ("Witness app host:", self.host, 16), ("Feed port:", self.port, 7))):
            ttk.Label(self.controls, text=label).grid(row=0, column=column * 2, padx=(0, 4))
            entry = ttk.Entry(self.controls, textvariable=variable, width=width)
            entry.grid(row=0, column=column * 2 + 1, padx=(0, 12))
            self.entries.append(entry)
        self.connect_button = ttk.Button(self.controls, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=4)
        self.note.configure(text="Receive only: start main.py to supply decoded Witness telemetry. "
                            "Auto-reconnects; snapshots retain original age. Units: g, dps, m, m/s, V, A, C, Pa; "
                            "uptime in ms. Live graphs are on Flight View; commands remain in main.py.")

    def make_receiver(self):
        return WitnessReceiver(self.host.get().strip(), int(self.port.get()), self.log_dir, on_record=self.on_record)


class FeatherweightTab(ReceiverTab):
    def __init__(self, parent, *, log_dir, on_record=None):
        super().__init__(parent, log_dir, schema=FEATHERWEIGHT_FIELDS, on_record=on_record)
        ttk.Label(self.controls, text="Featherweight COM port:").grid(row=0, column=0, padx=(0, 4))
        self.port_combo = ttk.Combobox(self.controls, width=14, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=(0, 5))
        self.entries.append(self.port_combo)
        ttk.Button(self.controls, text="Refresh", command=self.refresh_ports).grid(row=0, column=2)
        self.connect_button = ttk.Button(self.controls, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3, padx=12)
        self.note.configure(text=f"{FEATHERWEIGHT_BAUD} baud, 8-N-1, receive only. "
                            "Altitude: ft ASL; velocity: ft/s; coordinates/heading: degrees. "
                            "Fix: 0 = no fix, 2 = 2D, 3 = 3D. Bad text CRC is shown in red. "
                            "Binary FWT chatter without a tracker is ignored; raw bytes are logged.")
        self.refresh_ports()
        self.view.add_source("(no port)")

    def refresh_ports(self):
        try:
            from serial.tools import list_ports
            ports = [port.device for port in list_ports.comports()]
        except ImportError:
            ports = []
        current = self.port_combo.get()
        if self.receiver is not None:
            return
        self.port_combo["values"] = ["(no port)", *ports]
        self.port_combo.set(current if current in ports else "(no port)")

    def make_receiver(self):
        return FeatherweightReceiver(self.port_combo.get(), self.log_dir, on_record=self.on_record)

    def prepare_view(self):
        self.view.add_source(self.port_combo.get())
