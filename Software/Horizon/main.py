"""Small serial monitor for the AMA communications protocol."""

from __future__ import annotations

import datetime as _dt
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


def vector(data: bytes, width: int, count: int) -> list[int]:
    return [number(data[i:i + width]) for i in range(0, width * count, width)]


def scaled_vector(data: bytes, width: int, count: int, scale: float, digits: int) -> list[float]:
    return [round(signed_number(data[i:i + width]) * scale, digits)
            for i in range(0, width * count, width)]


# LSM6DSV320X sensitivities from the ST datasheet:
# low-g accelerometer at +/-16 g: 0.488 mg/LSB = 0.000488 g/LSB
# gyroscope at +/-2000 dps: 70 mdps/LSB = 0.070 dps/LSB
ACCEL_G_PER_LSB = 0.000488
GYRO_DPS_PER_LSB = 0.070


def parse_lora(packet_id: int, payload: bytes):
    if packet_id == 0x01 and len(payload) in (18, 20):
        return {"packet": "sensors", "status": number(payload[0:2]), "uptime": number(payload[2:6]),
                "accel": scaled_vector(payload[6:12], 2, 3, ACCEL_G_PER_LSB, 6),
                "gyro": scaled_vector(payload[12:18], 2, 3, GYRO_DPS_PER_LSB, 3),
                "crc": number(payload[18:20]) if len(payload) == 20 else None}
    if packet_id == 0x02 and len(payload) in (47, 48, 49, 50):
        # Field order/offsets per comms.h IRIS_PACKET_STATE_* macros:
        # status(2) uptime(4) orientation(12) position(9) velocity(6)
        # latitude(3) longitude(3) gps_altitude(4) barometric_altitude(4) crc(2)
        # Note: there is no "gps_time" field in this packet.
        result = {"packet": "state", "status": number(payload[0:2]), "uptime": number(payload[2:6]),
                "orientation": vector(payload[6:18], 3, 4), "position": vector(payload[18:27], 3, 3),
                "velocity": vector(payload[27:33], 2, 3), "latitude": number(payload[33:36]),
                "longitude": number(payload[36:39]), "gps_altitude": number(payload[39:43]),
                "barometric_altitude": number(payload[43:47])}
        has_battery = len(payload) in (48, 50)
        result["witness_battery_voltage"] = number(payload[47:48]) if has_battery else None
        result["crc"] = number(payload[48:50]) if len(payload) == 50 else (
            number(payload[47:49]) if len(payload) == 49 else None)
        return result
    if packet_id == 0x03 and len(payload) in (15, 16, 17, 18):
        result = {"packet": "camera", "fc_status": number(payload[0:2]), "fc_uptime": number(payload[2:6]),
                "status": number(payload[6:8]), "uptime": number(payload[8:12]),
                "current_sense": vector(payload[12:15], 1, 3)}
        has_battery = len(payload) in (16, 18)
        result["iris_battery_voltage"] = number(payload[15:16]) if has_battery else None
        result["crc"] = number(payload[16:18]) if len(payload) == 18 else (
            number(payload[15:17]) if len(payload) == 17 else None)
        return result
    if packet_id == 0xFF and len(payload) == 6:
        return {"packet": "heartbeat", "status": number(payload[0:2]), "uptime": number(payload[2:6])}
    if packet_id == 0x05 and len(payload) in (8, 10):
        return {"packet": "command", "status": number(payload[0:2]), "uptime": number(payload[2:6]),
                "command_value": number(payload[6:8]),
                "crc": number(payload[8:10]) if len(payload) == 10 else None}
    return None


def parse_can(packet_id: int, payload: bytes):
    """Decode CAN payloads when a serial bridge prefixes them with packet ID."""
    if packet_id == 0x01 and len(payload) == 18:
        return {"packet": "sensors", "status": number(payload[0:2]), "uptime": number(payload[2:6]),
                "accel": scaled_vector(payload[6:12], 2, 3, ACCEL_G_PER_LSB, 6),
                "gyro": scaled_vector(payload[12:18], 2, 3, GYRO_DPS_PER_LSB, 3)}
    if packet_id == 0x02 and len(payload) in (60, 62):
        # Same field order as the LoRa state packet but with the CAN
        # (uncompressed) component widths from comms.h. Again, no gps_time.
        result = {"packet": "state", "status": number(payload[0:2]), "uptime": number(payload[2:6]),
                "orientation": vector(payload[6:22], 4, 4), "position": vector(payload[22:34], 4, 3),
                "velocity": vector(payload[34:46], 4, 3), "latitude": number(payload[46:49]),
                "longitude": number(payload[49:52]), "gps_altitude": number(payload[52:56]),
                "barometric_altitude": number(payload[56:60])}
        result["witness_battery_voltage"] = number(payload[60:62]) if len(payload) == 62 else None
        return result
    if packet_id == 0x03 and len(payload) in (9, 11):
        result = {"packet": "camera", "status": number(payload[0:2]),
                "uptime": number(payload[2:6]), "current_sense": vector(payload[6:9], 1, 3)}
        # CAN camera layouts in older headers stop at current sense; accept
        # the newer optional two-byte Iris battery field when present.
        if len(payload) == 11:
            result["iris_battery_voltage"] = number(payload[9:11])
        else:
            result["iris_battery_voltage"] = None
        return result
    if packet_id == 0xFF and len(payload) == 6:
        return {"packet": "heartbeat", "status": number(payload[0:2]), "uptime": number(payload[2:6])}
    if packet_id == 0x05 and len(payload) == 8:
        return {"packet": "command", "status": number(payload[0:2]), "uptime": number(payload[2:6]),
                "command_value": number(payload[6:8])}
    return None


class PacketDecoder:
    """Decode ID-prefixed serial frames; unknown bytes are discarded."""
    lengths = {0x01: (20, 18), 0x02: (50, 60), 0x03: (18, 15), 0xFF: (6, 6), 0x05: (10, 8)}
    no_crc_lengths = {0x01: (18,), 0x02: (48, 47), 0x03: (16, 15), 0xFF: (6,), 0x05: (8,)}

    def __init__(self, on_lora, on_can, on_error=None):
        self.buffer = bytearray()
        self.on_lora, self.on_can = on_lora, on_can
        self.on_error = on_error or (lambda _message: None)

    def feed(self, data: bytes):
        self.buffer.extend(data)
        while self.buffer:
            try:
                packet_id = self.buffer[0]
                if packet_id not in self.lengths:
                    self.on_error(f"unknown packet ID 0x{packet_id:02x}; discarded 1 byte")
                    del self.buffer[0]
                    continue

                lora_len, can_len = self.lengths[packet_id]
                no_crc_candidates = self.no_crc_lengths[packet_id]
                # The observed serial bridge omits application CRC bytes.
                # Decode that form whenever the following byte is another
                # packet ID, or when exactly one complete no-CRC frame exists.
                for no_crc_len in no_crc_candidates:
                    if len(self.buffer) < 1 + no_crc_len:
                        continue
                    following = self.buffer[1 + no_crc_len] if len(self.buffer) > 1 + no_crc_len else None
                    if following in self.lengths or len(self.buffer) == 1 + no_crc_len:
                        frame = bytes(self.buffer[1:1 + no_crc_len])
                        del self.buffer[:1 + no_crc_len]
                        parsed = parse_lora(packet_id, frame)
                        if parsed:
                            self.on_lora(parsed)
                            can_parsed = parse_can(packet_id, frame)
                            if can_parsed:
                                self.on_can(can_parsed)
                        break
                else:
                    no_crc_len = None
                if no_crc_len is not None:
                    continue

                # Accept an ID-prefixed CAN state payload for debugging.
                if packet_id == 0x02 and len(self.buffer) >= 1 + can_len:
                    frame = bytes(self.buffer[1:1 + can_len])
                    del self.buffer[:1 + can_len]
                    parsed = parse_can(packet_id, frame)
                    if parsed:
                        self.on_can(parsed)
                    continue
                if len(self.buffer) < 1 + lora_len:
                    return
                frame = bytes(self.buffer[1:1 + lora_len])
                del self.buffer[:1 + lora_len]
                parsed = parse_lora(packet_id, frame)
                if parsed:
                    self.on_lora(parsed)
                    can_parsed = parse_can(packet_id, frame[:-2]) if packet_id != 0xFF else parse_can(packet_id, frame)
                    if can_parsed:
                        self.on_can(can_parsed)
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
        self.serial = None
        self.stop_event = threading.Event()
        self.events = queue.Queue()
        self.log_file = None
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
        ttk.Label(controls, text="Serial port:").pack(side="left")
        self.port_combo = ttk.Combobox(controls, width=18, state="readonly")
        self.port_combo.pack(side="left", padx=(6, 4))
        ttk.Button(controls, text="Refresh", command=self.refresh_ports).pack(side="left")
        ttk.Label(controls, text="Baud:").pack(side="left", padx=(14, 4))
        self.baud_combo = ttk.Combobox(controls, width=9, values=("115200", "57600", "38400", "9600"))
        self.baud_combo.set("115200")
        self.baud_combo.pack(side="left")
        self.connect_button = ttk.Button(controls, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(side="left", padx=(14, 0))
        self.status_label = ttk.Label(controls, text="Disconnected")
        self.status_label.pack(side="left", padx=12)
        main_pane = ttk.PanedWindow(self, orient=tk.VERTICAL)
        main_pane.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        notebook = ttk.Notebook(main_pane)
        self.lora_frame, self.can_frame = ttk.Frame(notebook, padding=8), ttk.Frame(notebook, padding=8)
        notebook.add(self.lora_frame, text="LoRa telemetry")
        notebook.add(self.can_frame, text="CAN debug")
        self.graph_frame = ttk.Frame(notebook, padding=8)
        notebook.add(self.graph_frame, text="Graphs")
        self.make_sections(self.lora_frame, "lora")
        self.make_sections(self.can_frame, "can")
        graph_controls = ttk.Frame(self.graph_frame)
        graph_controls.pack(fill="x", pady=(0, 5))
        ttk.Button(graph_controls, text="Clear graph history", command=self.clear_history).pack(side="left")
        ttk.Label(graph_controls, text="Select fields using the checkboxes on the telemetry tabs.").pack(side="left", padx=10)
        self.graph_canvas = tk.Canvas(self.graph_frame, background="white", height=500,
                                      highlightthickness=1, highlightbackground="#b0b0b0")
        self.graph_canvas.pack(fill="both", expand=True)
        self.graph_canvas.bind("<Configure>", lambda _event: self.draw_graphs())

        console_frame = ttk.LabelFrame(main_pane, text="Console", padding=5)
        self.console = scrolledtext.ScrolledText(
            console_frame, height=9, wrap="none", state="disabled",
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
                ("accel", "Accel [g; x, y, z]"), ("gyro", "Gyro [dps; x, y, z]"),
                ("crc", "CRC"),
            ],
        "state": [
                ("status", "Status"), ("uptime", "Uptime"),
                ("orientation", "Orientation [w, x, y, z]"),
                ("position", "Position [x, y, z]"),
                ("velocity", "Velocity [x, y, z]"),
                ("latitude", "Latitude"), ("longitude", "Longitude"),
                ("gps_altitude", "GPS altitude"),
                ("barometric_altitude", "Barometric altitude (MS5607)"),
                ("witness_battery_voltage", "Witness battery voltage"),
                ("crc", "CRC"),
            ],
            "camera": [
                ("fc_status", "FC status"), ("fc_uptime", "FC uptime"),
                ("status", "Camera status"), ("uptime", "Camera uptime"),
                ("current_sense", "Current [ch1, ch2, ch3]"),
                ("iris_battery_voltage", "Iris battery voltage"), ("crc", "CRC"),
            ],
            "heartbeat": [("status", "Status"), ("uptime", "Uptime")],
            "command": [
                ("status", "Status"), ("uptime", "Uptime"),
                ("command_value", "Command value"), ("crc", "CRC"),
            ],
        }
        # CAN has the same logical battery fields when the extended payload
        # widths from comms.h are used.
        if prefix == "can":
            specs["camera"] = [
                ("status", "Camera status"), ("uptime", "Camera uptime"),
                ("current_sense", "Current [ch1, ch2, ch3]"),
                ("iris_battery_voltage", "Iris battery voltage"),
            ]

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
                ttk.Label(section, textvariable=self.values[key]).pack(anchor="w", padx=(30, 0))

    def refresh_ports(self):
        try:
            from serial.tools import list_ports
            ports = [p.device for p in list_ports.comports()]
        except ImportError:
            ports = []
        self.port_combo["values"] = ports
        if ports and not self.port_combo.get():
            self.port_combo.set(ports[0])

    def toggle_connection(self):
        self.disconnect() if self.serial is not None else self.connect()

    def connect(self):
        port = self.port_combo.get()
        if not port:
            messagebox.showerror("Serial monitor", "Select a serial port first.")
            return
        try:
            import serial
            self.serial = serial.Serial(port, int(self.baud_combo.get()), timeout=0.2)
        except ImportError:
            messagebox.showerror("Serial monitor", "pyserial is required: pip install pyserial")
            return
        except Exception as exc:
            messagebox.showerror("Serial monitor", f"Could not open {port}: {exc}")
            return
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = (LOG_DIR / f"serial_{stamp}.bin").open("ab")
        self.debug_log_file = (LOG_DIR / f"debug_{stamp}.log").open("a", encoding="utf-8")
        self.stop_event.clear()
        threading.Thread(target=self.read_serial, daemon=True).start()
        self.connect_button.configure(text="Disconnect")
        self.status_label.configure(text=f"Connected; logging {self.log_file.name}")

    def read_serial(self):
        decoder = PacketDecoder(
            lambda p: self.events.put(("lora", p)),
            lambda p: self.events.put(("can", p)),
            lambda message: self.events.put(("decoder_error", message)),
        )
        while not self.stop_event.is_set() and self.serial is not None:
            try:
                data = self.serial.read(self.serial.in_waiting or 1)
                if data:
                    self.log_file.write(data)
                    self.log_file.flush()
                    self.events.put(("bytes", bytes(data)))
                    decoder.feed(data)
            except Exception as exc:
                self.events.put(("error", str(exc)))
                break

    def process_events(self):
        while True:
            try:
                kind, payload = self.events.get_nowait()
                if kind == "error":
                    self.status_label.configure(text=f"Serial error: {payload}")
                    self.console_write(f"ERROR {payload}")
                    self.disconnect()
                elif kind == "decoder_error":
                    self.console_write(f"BAD_PACKET {payload}")
                elif kind == "bytes":
                    self.console_write(f"RX  {payload.hex(' ')}")
                else:
                    self.update_values(kind, payload)
                    self.console_write(f"{kind.upper():4} {self.format_packet(payload)}")
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
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None
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
