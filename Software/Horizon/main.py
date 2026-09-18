"""Small serial monitor for the AMA communications protocol."""

from __future__ import annotations

import datetime as _dt
import struct
import queue
import threading
import time
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk


ROOT = Path(__file__).resolve().parent
LOG_DIR = ROOT / "logs"


def number(data: bytes) -> int:
    return int.from_bytes(data, byteorder="big", signed=False)


def signed_number(data: bytes) -> int:
    return int.from_bytes(data, byteorder="big", signed=True)


def signed_int24(data: bytes) -> int:
    value = int.from_bytes(data, byteorder="big", signed=False)
    return value - (1 << 24) if value & (1 << 23) else value


def float32(data: bytes) -> float:
    return struct.unpack(">f", data)[0]


def float_vector(data: bytes, count: int) -> list[float]:
    return [round(float32(data[i:i + 4]), 6) for i in range(0, count * 4, 4)]


# comms.h canonical field scales.
FILTERED_ACCEL_G_PER_COUNT = 0.000040
GYRO_DPS_PER_LSB = 0.070
BATTERY_V_PER_COUNT = 0.001
CURRENT_A_PER_COUNT = 0.1
GPS_DEGREES_PER_COUNT = 0.0001
LSM6_TEMPERATURE_OFFSET_C = 25.0
LSM6_TEMPERATURE_C_PER_COUNT = 1.0 / 256.0
MS5607_TEMPERATURE_C_PER_COUNT = 0.01


def parse_packet(packet_id: int, payload: bytes):
    if packet_id == 0x01 and len(payload) == 21:
        return {"packet": "sensors", "status": number(payload[0:2]), "uptime": number(payload[2:6]),
                "filtered_accel": [round(signed_int24(payload[i:i + 3]) * FILTERED_ACCEL_G_PER_COUNT, 6)
                          for i in (6, 9, 12)],
                "gyro": [round(signed_number(payload[i:i + 2]) * GYRO_DPS_PER_LSB, 3)
                         for i in (15, 17, 19)]}
    if packet_id == 0x02 and len(payload) == 62:
        return {"packet": "state", "status": number(payload[0:2]), "uptime": number(payload[2:6]),
                "orientation": float_vector(payload[6:22], 4),
                "position": float_vector(payload[22:34], 3),
                "velocity": float_vector(payload[34:46], 3),
                "latitude": round(signed_int24(payload[46:49]) * GPS_DEGREES_PER_COUNT, 4),
                "longitude": round(signed_int24(payload[49:52]) * GPS_DEGREES_PER_COUNT, 4),
                "gps_altitude": round(float32(payload[52:56]), 6),
                "barometric_altitude": round(float32(payload[56:60]), 6),
                "witness_battery_voltage": round(number(payload[60:62]) * BATTERY_V_PER_COUNT, 3)}
    if packet_id == 0x03 and len(payload) == 17:
        result = {"packet": "camera", "fc_status": number(payload[0:2]), "fc_uptime": number(payload[2:6]),
                "status": number(payload[6:8]), "uptime": number(payload[8:12]),
                "current_sense": [round(payload[i] * CURRENT_A_PER_COUNT, 1) for i in (12, 13, 14)],
                "iris_battery_voltage": round(number(payload[15:17]) * BATTERY_V_PER_COUNT, 3)}
        return result
    if packet_id == 0xFF and len(payload) == 6:
        return {"packet": "heartbeat", "status": number(payload[0:2]), "uptime": number(payload[2:6])}
    if packet_id == 0x05 and len(payload) == 8:
        return {"packet": "command", "status": number(payload[0:2]), "uptime": number(payload[2:6]),
                "command_value": number(payload[6:8])}
    return None


class PacketDecoder:
    """Decode the single ID-prefixed serial protocol from comms.h."""
    payload_lengths = {0x01: 21, 0x02: 62, 0x03: 17, 0xFF: 6, 0x05: 8}

    def __init__(self, on_packet, on_error=None):
        self.buffer = bytearray()
        self.on_packet = on_packet
        self.on_error = on_error or (lambda _message: None)

    def feed(self, data: bytes):
        self.buffer.extend(data)
        while self.buffer:
            try:
                packet_id = self.buffer[0]
                if packet_id not in self.payload_lengths:
                    self.on_error(f"unknown packet ID 0x{packet_id:02x}; discarded 1 byte")
                    del self.buffer[0]
                    continue

                payload_length = self.payload_lengths[packet_id]
                if len(self.buffer) < 1 + payload_length:
                    return
                frame = bytes(self.buffer[1:1 + payload_length])
                del self.buffer[:1 + payload_length]
                parsed = parse_packet(packet_id, frame)
                if parsed:
                    self.on_packet(parsed)
            except Exception as exc:
                bad = bytes(self.buffer[: min(len(self.buffer), 32)])
                self.on_error(f"decoder rejected packet: {exc}; buffer={bad.hex(' ')}")
                # Always make progress after malformed input.
                del self.buffer[0]


class SerialMonitor(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Horizon Communications Monitor")
        self.geometry("920x720")
        self.serials = {}
        self.stop_event = threading.Event()
        self.events = queue.Queue()
        self.log_files = {}
        self.debug_log_file = None
        self.values = {}
        self.check_vars = {}
        self.history = {}
        self.last_packet_time = {}
        self.age_labels = {}
        self.max_history = 300
        self.build_ui()
        self.refresh_ports()
        self.after(50, self.process_events)
        self.after(100, self.draw_graphs)
        self.after(100, self.update_age_indicators)
        self.protocol("WM_DELETE_WINDOW", self.close)

    def build_ui(self):
        controls = ttk.Frame(self, padding=8)
        controls.pack(fill="x")
        ttk.Label(controls, text="Port A:").pack(side="left")
        self.port_a_combo = ttk.Combobox(controls, width=13, state="readonly")
        self.port_a_combo.pack(side="left", padx=(4, 8))
        ttk.Label(controls, text="Port B:").pack(side="left")
        self.port_b_combo = ttk.Combobox(controls, width=13, state="readonly")
        self.port_b_combo.pack(side="left", padx=(4, 4))
        ttk.Button(controls, text="Refresh", command=self.refresh_ports).pack(side="left")
        ttk.Label(controls, text="Baud:").pack(side="left", padx=(14, 4))
        self.baud_combo = ttk.Combobox(controls, width=9, values=("115200", "57600", "38400", "9600"))
        self.baud_combo.set("115200")
        self.baud_combo.pack(side="left")
        self.connect_button = ttk.Button(controls, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(side="left", padx=(14, 0))
        self.status_label = ttk.Label(controls, text="Disconnected", width=34, anchor="w")
        self.status_label.pack(side="left", padx=12)
        main_pane = ttk.PanedWindow(self, orient=tk.VERTICAL)
        main_pane.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        notebook = ttk.Notebook(main_pane)
        self.telemetry_frame = ttk.Frame(notebook, padding=8)
        notebook.add(self.telemetry_frame, text="Telemetry")
        self.graph_frame = ttk.Frame(notebook, padding=8)
        notebook.add(self.graph_frame, text="Graphs")
        self.make_sections(self.telemetry_frame, "packet")
        graph_controls = ttk.Frame(self.graph_frame)
        graph_controls.pack(fill="x", pady=(0, 5))
        ttk.Button(graph_controls, text="Clear graph history", command=self.clear_history).pack(side="left")
        ttk.Label(graph_controls, text="Select fields using the checkboxes on the Telemetry tab.").pack(side="left", padx=10)
        self.graph_canvas = tk.Canvas(self.graph_frame, background="white", height=500,
                                      highlightthickness=1, highlightbackground="#b0b0b0")
        self.graph_canvas.pack(fill="both", expand=True)
        self.graph_canvas.bind("<Configure>", lambda _event: self.draw_graphs())

        console_frame = ttk.LabelFrame(main_pane, text="Console", padding=5)
        self.console = scrolledtext.ScrolledText(
            console_frame, height=9, wrap="word", state="disabled",
            font=("Consolas", 9)
        )
        self.console.pack(fill="both", expand=True)
        main_pane.add(notebook, weight=4)
        main_pane.add(console_frame, weight=1)

    def make_sections(self, parent, prefix):
        # Field lists mirror comms.h exactly, in wire order, per packet type.
        specs = {
            "sensors": [
                ("status", "Status"), ("uptime", "Uptime"),
                ("filtered_accel", "Filtered accel [g; x, y, z]"),
                ("gyro", "Gyro [dps; x, y, z]"),
            ],
        "state": [
                ("status", "Status"), ("uptime", "Uptime"),
                ("orientation", "Orientation [x, y, z, w]"),
                ("position", "Position [x, y, z]"),
                ("velocity", "Velocity [x, y, z]"),
                ("latitude", "Latitude"), ("longitude", "Longitude"),
                ("gps_altitude", "GPS altitude"),
                ("barometric_altitude", "Barometric altitude (MS5607)"),
                ("witness_battery_voltage", "Witness battery voltage [V]"),
            ],
            "camera": [
                ("fc_status", "FC status"), ("fc_uptime", "FC uptime"),
                ("status", "Camera status"), ("uptime", "Camera uptime"),
                ("current_sense", "Current [ch1, ch2, ch3]"),
                ("iris_battery_voltage", "Iris battery voltage [V]"),
            ],
            "heartbeat": [("status", "Status"), ("uptime", "Uptime")],
            "command": [
                ("status", "Status"), ("uptime", "Uptime"),
                ("command_value", "Command value"),
            ],
        }
        for row, (packet, fields) in enumerate(specs.items()):
            section = ttk.LabelFrame(parent, text=packet.capitalize(), padding=6)
            section.grid(row=row // 2, column=row % 2, sticky="nsew", padx=5, pady=5)
            parent.columnconfigure(row % 2, weight=1)
            header = ttk.Frame(section)
            header.pack(fill="x", pady=(0, 5))
            ttk.Label(header, text="Data age:").pack(side="left")
            age_key = f"{prefix}.{packet}"
            age_label = tk.Label(header, text="--", width=10, anchor="center",
                                 relief="sunken", bd=1)
            age_label.pack(side="left", padx=(5, 0))
            self.age_labels[age_key] = age_label
            for field, label in fields:
                key = f"{prefix}.{packet}.{field}"
                self.values[key] = tk.StringVar(value="null")
                self.check_vars[key] = tk.BooleanVar(value=False)
                row_frame = ttk.Frame(section)
                row_frame.pack(fill="x", anchor="w", pady=(0, 3))
                ttk.Checkbutton(row_frame, variable=self.check_vars[key]).pack(side="left")
                ttk.Label(row_frame, text=label + ":").pack(side="left")
                # Keep sections dimensionally stable as values change; long
                # vectors and debug values wrap inside this fixed area.
                ttk.Label(section, textvariable=self.values[key], width=32,
                          wraplength=275, justify="left", anchor="w").pack(
                              anchor="w", padx=(30, 0))

    def refresh_ports(self):
        try:
            from serial.tools import list_ports
            ports = [p.device for p in list_ports.comports()]
        except ImportError:
            ports = []
        for combo in (self.port_a_combo, self.port_b_combo):
            combo["values"] = ports
        if ports and not self.port_a_combo.get():
            self.port_a_combo.set(ports[0])
        if len(ports) > 1 and not self.port_b_combo.get():
            self.port_b_combo.set(ports[1])

    def toggle_connection(self):
        self.disconnect() if self.serials else self.connect()

    def connect(self):
        ports = [port for port in (self.port_a_combo.get(), self.port_b_combo.get()) if port]
        if not ports:
            messagebox.showerror("Serial monitor", "Select at least one serial port first.")
            return
        if len(set(ports)) != len(ports):
            messagebox.showerror("Serial monitor", "Port A and Port B must be different.")
            return
        try:
            import serial
        except ImportError:
            messagebox.showerror("Serial monitor", "pyserial is required: pip install pyserial")
            return
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.debug_log_file = (LOG_DIR / f"debug_{stamp}.log").open("a", encoding="utf-8")
        self.stop_event.clear()
        try:
            for port in ports:
                serial_port = serial.Serial(port, int(self.baud_combo.get()), timeout=0.2)
                self.serials[port] = serial_port
                safe_port = "".join(char if char.isalnum() else "_" for char in port)
                self.log_files[port] = (LOG_DIR / f"serial_{safe_port}_{stamp}.bin").open("ab")
                threading.Thread(target=self.read_serial, args=(port, serial_port), daemon=True).start()
        except Exception as exc:
            self.disconnect()
            messagebox.showerror("Serial monitor", f"Could not open serial port: {exc}")
            return
        self.connect_button.configure(text="Disconnect")
        self.status_label.configure(text=f"Connected: {', '.join(ports)}")

    def read_serial(self, port_name, serial_port):
        decoder = PacketDecoder(
            lambda p: self.events.put(("packet", (port_name, p))),
            lambda message: self.events.put(("decoder_error", (port_name, message))),
        )
        while not self.stop_event.is_set() and self.serials.get(port_name) is serial_port:
            try:
                data = serial_port.read(serial_port.in_waiting or 1)
                if data:
                    log_file = self.log_files.get(port_name)
                    if log_file is not None:
                        log_file.write(data)
                        log_file.flush()
                    self.events.put(("bytes", (port_name, bytes(data))))
                    decoder.feed(data)
            except Exception as exc:
                self.events.put(("error", (port_name, str(exc))))
                break

    def process_events(self):
        while True:
            try:
                kind, payload = self.events.get_nowait()
                if kind == "error":
                    port_name, message = payload
                    self.status_label.configure(text=f"Serial error on {port_name}: {message}")
                    self.console_write(f"ERROR[{port_name}] {message}")
                elif kind == "decoder_error":
                    port_name, message = payload
                    self.console_write(f"BAD_PACKET[{port_name}] {message}")
                elif kind == "bytes":
                    port_name, data = payload
                    self.console_write(f"RX[{port_name}]  {data.hex(' ')}")
                else:
                    port_name, packet = payload
                    self.update_values("packet", packet)
                    self.console_write(f"PACKET[{port_name}] {self.format_packet(packet)}")
            except queue.Empty:
                break
            except Exception as exc:
                # A malformed packet or display value must never terminate the
                # Tk event pump. Keep the failure visible in the debug log.
                try:
                    self.console_write(f"GUI_ERROR {exc}")
                except Exception:
                    pass
        self.after(50, self.process_events)

    def update_age_indicators(self):
        now = time.monotonic()
        for key, label in self.age_labels.items():
            received = self.last_packet_time.get(key)
            if received is None:
                label.configure(text="--", background="#f0f0f0")
                continue
            age = max(0.0, now - received)
            if age < 2.0:
                background = "#f0f0f0"
            elif age < 5.0:
                background = "#fff2a8"
            elif age < 10.0:
                background = "#ffd27f"
            else:
                background = "#ff8a80"
            label.configure(text=f"{age:.1f}s", background=background)
        self.after(100, self.update_age_indicators)

    @staticmethod
    def format_packet(packet):
        fields = ", ".join(
            f"{name}={value}" for name, value in packet.items() if name != "packet"
        )
        return f"{packet['packet']}: {fields}"

    def console_write(self, line):
        timestamp = _dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        formatted = f"[{timestamp}] {line}"
        if self.debug_log_file is not None:
            self.debug_log_file.write(formatted + "\n")
            self.debug_log_file.flush()
        self.console.configure(state="normal")
        self.console.insert("end", formatted + "\n")
        self.console.see("end")
        self.console.configure(state="disabled")

    def update_values(self, kind, packet):
        packet_key = f"{kind}.{packet['packet']}"
        self.last_packet_time[packet_key] = time.monotonic()
        for field, value in packet.items():
            if field != "packet":
                key = f"{kind}.{packet['packet']}.{field}"
                if key in self.values:
                    self.values[key].set("null" if value is None else str(value))
                    if isinstance(value, (int, float, list, tuple)):
                        samples = self.history.setdefault(key, [])
                        samples.append((_dt.datetime.now().timestamp(), value))
                        del samples[:-self.max_history]
        self.draw_graphs()

    def clear_history(self):
        self.history.clear()
        self.draw_graphs()

    def draw_graphs(self):
        if not hasattr(self, "graph_canvas"):
            return
        canvas = self.graph_canvas
        canvas.delete("all")
        selected = [key for key, variable in self.check_vars.items() if variable.get()]
        width = max(canvas.winfo_width(), 500)
        graph_height = 170
        if not selected:
            canvas.create_text(width // 2, 40, text="No fields selected", fill="#555555")
            return
        canvas.configure(scrollregion=(0, 0, width, graph_height * len(selected)))
        colors = ("#1565c0", "#c62828", "#2e7d32", "#6a1b9a", "#ef6c00")
        for graph_index, key in enumerate(selected):
            top = graph_index * graph_height
            samples = self.history.get(key, [])
            canvas.create_rectangle(5, top + 5, width - 5, top + graph_height - 5,
                                    outline="#cccccc")
            canvas.create_text(14, top + 16, anchor="w", text=key, fill="#222222")
            if not samples:
                canvas.create_text(width // 2, top + 85, text="Waiting for data", fill="#777777")
                continue
            times = [sample[0] for sample in samples]
            values = [sample[1] for sample in samples]
            components = len(values[-1]) if isinstance(values[-1], (list, tuple)) else 1
            flat = [float(component) for value in values
                    for component in (value if isinstance(value, (list, tuple)) else [value])]
            minimum, maximum = min(flat), max(flat)
            if minimum == maximum:
                minimum -= 0.5
                maximum += 0.5
            # Use half-number increments for the value axis. This keeps the
            # grid predictable instead of basing it directly on raw extrema.
            value_step = max(0.5, ((maximum - minimum) / 4.0 / 0.5).__ceil__() * 0.5)
            minimum = (minimum // value_step) * value_step
            maximum = ((maximum + value_step - 1e-12) // value_step) * value_step
            if maximum <= minimum:
                maximum = minimum + value_step
            left, right = 70, width - 15
            top_plot, bottom_plot = top + 30, top + graph_height - 22
            time_span = max(times[-1] - times[0], 0.001)
            time_step = max(0.5, ((time_span / 4.0 / 0.5).__ceil__()) * 0.5)

            # Horizontal value grid and labels.
            value = minimum
            while value <= maximum + value_step * 0.01:
                y = bottom_plot - (value - minimum) / (maximum - minimum) * (bottom_plot - top_plot)
                canvas.create_line(left, y, right, y, fill="#e1e1e1")
                canvas.create_text(left - 5, y, anchor="e", text=f"{value:.4g}", fill="#555555")
                value += value_step

            # Vertical time grid and wall-clock labels.
            elapsed = 0.0
            while elapsed <= time_span + time_step * 0.01:
                x = left + min(elapsed / time_span, 1.0) * (right - left)
                canvas.create_line(x, top_plot, x, bottom_plot, fill="#e1e1e1")
                tick_time = _dt.datetime.fromtimestamp(times[0] + elapsed)
                tick_label = tick_time.strftime("%H:%M:%S.%f")[:-3]
                canvas.create_text(x, bottom_plot + 14, anchor="n", text=tick_label, fill="#555555")
                elapsed += time_step

            for component in range(components):
                points = []
                for timestamp, value in samples:
                    numeric = value[component] if isinstance(value, (list, tuple)) else value
                    x = left + (timestamp - times[0]) / time_span * (right - left)
                    y = bottom_plot - (float(numeric) - minimum) / (maximum - minimum) * (bottom_plot - top_plot)
                    points.extend((x, y))
                if len(points) >= 4:
                    canvas.create_line(*points, fill=colors[component % len(colors)], width=2)
            start_label = _dt.datetime.fromtimestamp(times[0]).strftime("%H:%M:%S.%f")[:-3]
            end_label = _dt.datetime.fromtimestamp(times[-1]).strftime("%H:%M:%S.%f")[:-3]
            # Ensure the exact endpoints remain visible even when the final
            # timestamp does not land on a grid interval.
            canvas.create_text(left, bottom_plot + 30, anchor="w", text=start_label, fill="#555555")
            canvas.create_text(right, bottom_plot + 30, anchor="e", text=end_label, fill="#555555")

    def disconnect(self):
        self.stop_event.set()
        for serial_port in self.serials.values():
            try:
                serial_port.close()
            except Exception:
                pass
        self.serials.clear()
        for log_file in self.log_files.values():
            try:
                log_file.close()
            except Exception:
                pass
        self.log_files.clear()
        if self.debug_log_file is not None:
            self.debug_log_file.close()
            self.debug_log_file = None
        self.connect_button.configure(text="Connect")
        self.status_label.configure(text="Disconnected")

    def close(self):
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    SerialMonitor().mainloop()
