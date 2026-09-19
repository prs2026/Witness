"""Small serial monitor for the AMA communications protocol."""

from __future__ import annotations

import datetime as _dt
import socket
import struct
import queue
import threading
import time
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk


ROOT = Path(__file__).resolve().parent
LOG_DIR = ROOT / "logs"

# Default network endpoints shown by the GUI at startup.
DEFAULT_TCP_1_HOST = "prs2026hitl"
DEFAULT_TCP_1_PORT = "7100"
DEFAULT_TCP_2_HOST = "prs2026hitl"
DEFAULT_TCP_2_PORT = "7101"


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
LSM6_LOW_ACCEL_G_PER_COUNT = 0.000488
LSM6_HIGH_ACCEL_G_PER_COUNT = 0.010417


def parse_packet(packet_id: int, payload: bytes):
    if packet_id == 0x01 and len(payload) in (21, 23):
        result = {"packet": "sensors", "status": number(payload[0:2]), "uptime": number(payload[2:6]),
                "filtered_accel": [round(signed_int24(payload[i:i + 3]) * FILTERED_ACCEL_G_PER_COUNT, 6)
                          for i in (6, 9, 12)],
                "gyro": [round(signed_number(payload[i:i + 2]) * GYRO_DPS_PER_LSB, 3)
                         for i in (15, 17, 19)]}
        # The live stream currently contains two additional bytes on sensor
        # frames even though the checked-in comms.h totals 21 payload bytes.
        # Preserve them numerically until the producer/header discrepancy is
        # resolved, rather than rejecting an otherwise valid sensor sample.
        if len(payload) == 23:
            result["sensor_trailing_u16"] = number(payload[21:23])
        return result
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
    if packet_id == 0xE0 and len(payload) == 101:
        return {
            "packet": "witness_debug", "status": number(payload[0:2]),
            "uptime": number(payload[2:6]), "mcu_temperature": signed_number(payload[6:7]),
            "witness_battery_voltage": round(number(payload[7:9]) * BATTERY_V_PER_COUNT, 3),
            "filtered_accel": [round(signed_int24(payload[i:i + 3]) * FILTERED_ACCEL_G_PER_COUNT, 6)
                               for i in (9, 12, 15)],
            "orientation": float_vector(payload[18:34], 4),
            "position": float_vector(payload[34:46], 3),
            "velocity": float_vector(payload[46:58], 3),
            "low_accel": [round(signed_number(payload[i:i + 2]) * LSM6_LOW_ACCEL_G_PER_COUNT, 6)
                          for i in (58, 60, 62)],
            "lsm6_temperature": round(LSM6_TEMPERATURE_OFFSET_C +
                                       signed_number(payload[64:66]) * LSM6_TEMPERATURE_C_PER_COUNT, 3),
            "high_accel": [round(signed_number(payload[i:i + 2]) * LSM6_HIGH_ACCEL_G_PER_COUNT, 6)
                           for i in (66, 68, 70)],
            "gyro": [round(signed_number(payload[i:i + 2]) * GYRO_DPS_PER_LSB, 3)
                     for i in (72, 74, 76)],
            "pressure": number(payload[78:81]),
            "ms5607_temperature": round(signed_number(payload[81:83]) * MS5607_TEMPERATURE_C_PER_COUNT, 2),
            "barometric_altitude": round(float32(payload[83:87]), 6),
            "latitude": round(signed_int24(payload[87:90]) * GPS_DEGREES_PER_COUNT, 4),
            "longitude": round(signed_int24(payload[90:93]) * GPS_DEGREES_PER_COUNT, 4),
            "gps_time": number(payload[93:97]),
            "gps_altitude": round(float32(payload[97:101]), 6),
        }
    if packet_id == 0xE1 and len(payload) == 12:
        return {
            "packet": "iris_debug", "status": number(payload[0:2]),
            "uptime": number(payload[2:6]), "mcu_temperature": signed_number(payload[6:7]),
            "iris_battery_voltage": round(number(payload[7:9]) * BATTERY_V_PER_COUNT, 3),
            "current_sense": [round(payload[i] * CURRENT_A_PER_COUNT, 1) for i in (9, 10, 11)],
        }
    return None


class PacketDecoder:
    """Decode the single ID-prefixed serial protocol from comms.h."""
    # Sensor frames in the supplied capture are 25 bytes total: ID + 23-byte
    # payload + EOF.  Keep the 21-byte header-defined form as a fallback so
    # older producers remain readable while the firmware/header are reconciled.
    payload_lengths = {0x01: (23, 21), 0x02: (62,), 0x03: (17,), 0xFF: (6,),
                       0x05: (8,), 0xE0: (101,), 0xE1: (12,)}
    eof = 0x0A

    def __init__(self, on_packet, on_error=None):
        self.buffer = bytearray()
        self.on_packet = on_packet
        self.on_error = on_error or (lambda _message: None)

    def find_valid_frame(self, start=0):
        """Return the next offset whose fixed-length frame has a valid EOF."""
        for offset in range(start, len(self.buffer)):
            packet_id = self.buffer[offset]
            payload_lengths = self.payload_lengths.get(packet_id)
            if payload_lengths is None:
                continue
            for payload_length in payload_lengths:
                eof_index = offset + 1 + payload_length
                if eof_index < len(self.buffer) and self.buffer[eof_index] == self.eof:
                    return offset
        return None

    def frame_payload_length(self, packet_id: int):
        """Return the matching payload length, or None if more bytes are needed."""
        payload_lengths = self.payload_lengths.get(packet_id)
        if payload_lengths is None:
            return None
        for payload_length in payload_lengths:
            eof_index = 1 + payload_length
            if len(self.buffer) > eof_index and self.buffer[eof_index] == self.eof:
                return payload_length
        return None

    def feed(self, data: bytes):
        self.buffer.extend(data)
        while self.buffer:
            try:
                packet_id = self.buffer[0]
                if packet_id not in self.payload_lengths:
                    next_frame = self.find_valid_frame(1)
                    discard_count = next_frame if next_frame is not None else 1
                    self.on_error(
                        f"unknown packet ID 0x{packet_id:02x}; discarded {discard_count} byte(s)"
                    )
                    del self.buffer[:discard_count]
                    continue

                payload_length = self.frame_payload_length(packet_id)
                if payload_length is None:
                    # Do not reject a split frame. For the sensor packet this
                    # also allows us to wait for the longer observed form
                    # before deciding that its EOF is bad.
                    maximum_length = max(self.payload_lengths[packet_id])
                    if len(self.buffer) < 1 + maximum_length + 1:
                        return
                    payload_length = maximum_length
                frame_length = 1 + payload_length + 1
                if len(self.buffer) < frame_length:
                    return
                frame = bytes(self.buffer[1:1 + payload_length])
                eof_byte = self.buffer[1 + payload_length]
                del self.buffer[:frame_length]
                if eof_byte != self.eof:
                    next_frame = self.find_valid_frame(1)
                    self.on_error(
                        f"bad EOF for 0x{packet_id:02x}: expected 0x0a, got 0x{eof_byte:02x}; "
                        f"resync={'offset ' + str(next_frame) if next_frame is not None else 'discard 1 byte'}"
                    )
                    if next_frame is not None:
                        del self.buffer[:next_frame]
                    else:
                        # The current frame is invalid, but leave later bytes
                        # buffered so a split TCP read can complete a frame.
                        del self.buffer[:1]
                    continue
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
        self.tcp_sockets = {}
        self.stop_event = threading.Event()
        self.events = queue.Queue()
        self.log_files = {}
        self.debug_log_file = None
        self.values = {}
        self.check_vars = {}
        self.history = {}
        self.last_packet_time = {}
        self.age_labels = {}
        self.section_groups = {}
        self.section_column_counts = {}
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
        tcp_controls = ttk.Frame(self, padding=(8, 0, 8, 8))
        tcp_controls.pack(fill="x")
        self.tcp_entries = []
        for index, (host_default, port_default) in enumerate(
                ((DEFAULT_TCP_1_HOST, DEFAULT_TCP_1_PORT),
                 (DEFAULT_TCP_2_HOST, DEFAULT_TCP_2_PORT)), start=1):
            ttk.Label(tcp_controls, text=f"TCP {index} host:").pack(side="left",
                                                                      padx=(0 if index == 1 else 14, 4))
            host_entry = ttk.Entry(tcp_controls, width=16)
            host_entry.insert(0, host_default)
            host_entry.pack(side="left")
            ttk.Label(tcp_controls, text="port:").pack(side="left", padx=(4, 2))
            port_entry = ttk.Entry(tcp_controls, width=7)
            port_entry.insert(0, port_default)
            port_entry.pack(side="left")
            self.tcp_entries.append((host_entry, port_entry))
        main_pane = ttk.PanedWindow(self, orient=tk.VERTICAL)
        main_pane.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        notebook = ttk.Notebook(main_pane)
        telemetry_page = ttk.Frame(notebook)
        notebook.add(telemetry_page, text="Telemetry")
        telemetry_canvas = tk.Canvas(telemetry_page, highlightthickness=0)
        telemetry_scrollbar = ttk.Scrollbar(telemetry_page, orient="vertical",
                                            command=telemetry_canvas.yview)
        self.telemetry_frame = ttk.Frame(telemetry_canvas, padding=8)
        telemetry_window = telemetry_canvas.create_window(
            (0, 0), window=self.telemetry_frame, anchor="nw"
        )
        telemetry_canvas.configure(yscrollcommand=telemetry_scrollbar.set)
        telemetry_canvas.pack(side="left", fill="both", expand=True)
        telemetry_scrollbar.pack(side="right", fill="y")
        self.telemetry_frame.bind(
            "<Configure>",
            lambda _event: telemetry_canvas.configure(
                scrollregion=telemetry_canvas.bbox("all")
            ),
        )
        telemetry_canvas.bind(
            "<Configure>",
            lambda event: telemetry_canvas.itemconfigure(
                telemetry_window, width=event.width
            ),
        )
        telemetry_canvas.bind("<Enter>", lambda _event: telemetry_canvas.bind_all(
            "<MouseWheel>", lambda event: telemetry_canvas.yview_scroll(
                int(-event.delta / 120), "units"
            )
        ))
        telemetry_canvas.bind("<Leave>", lambda _event: telemetry_canvas.unbind_all("<MouseWheel>"))
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
        command_frame = ttk.Frame(console_frame, padding=(0, 6, 0, 0))
        command_frame.pack(fill="x")
        ttk.Label(command_frame, text="Send hex to:").pack(side="left")
        self.command_port_combo = ttk.Combobox(command_frame, width=13, state="readonly")
        self.command_port_combo.pack(side="left", padx=(5, 8))
        ttk.Label(command_frame, text="Command:").pack(side="left")
        self.command_entry = ttk.Entry(command_frame)
        self.command_entry.pack(side="left", fill="x", expand=True, padx=5)
        self.command_entry.bind("<Return>", lambda _event: self.send_hex_command())
        self.send_command_button = ttk.Button(
            command_frame, text="Send", command=self.send_hex_command
        )
        self.send_command_button.pack(side="left")
        main_pane.add(notebook, weight=4)
        main_pane.add(console_frame, weight=1)

    def make_sections(self, parent, prefix):
        # Field lists mirror comms.h exactly, in wire order, per packet type.
        specs = {
            "sensors": [
                ("status", "Status"), ("uptime", "Uptime"),
                ("filtered_accel", "Filtered accel [g; x, y, z]"),
                ("gyro", "Gyro [dps; x, y, z]"),
                ("sensor_trailing_u16", "Sensor trailing bytes [u16]"),
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
            "witness_debug": [
                ("status", "Status"), ("uptime", "Uptime"),
                ("mcu_temperature", "MCU temperature [C]"),
                ("witness_battery_voltage", "Witness battery voltage [V]"),
                ("filtered_accel", "Filtered accel [g; x, y, z]"),
                ("orientation", "Orientation [x, y, z, w]"),
                ("position", "Position [m; x, y, z]"),
                ("velocity", "Velocity [m/s; x, y, z]"),
                ("low_accel", "LSM6 low accel [g; x, y, z]"),
                ("lsm6_temperature", "LSM6 temperature [C]"),
                ("high_accel", "LSM6 high accel [g; x, y, z]"),
                ("gyro", "Gyro [dps; x, y, z]"),
                ("pressure", "MS5607 pressure [Pa]"),
                ("ms5607_temperature", "MS5607 temperature [C]"),
                ("barometric_altitude", "Barometric altitude [m]"),
                ("latitude", "Latitude [deg]"), ("longitude", "Longitude [deg]"),
                ("gps_time", "GPS time [Unix s]"), ("gps_altitude", "GPS altitude [m]"),
            ],
            "iris_debug": [
                ("status", "Status"), ("uptime", "Uptime"),
                ("mcu_temperature", "MCU temperature [C]"),
                ("iris_battery_voltage", "Iris battery voltage [V]"),
                ("current_sense", "Current [A; ch1, ch2, ch3]"),
            ],
        }
        sections = []
        for row, (packet, fields) in enumerate(specs.items()):
            section = ttk.LabelFrame(parent, text=packet.capitalize(), padding=6)
            section.grid(row=row // 3, column=row % 3, sticky="nsew", padx=4, pady=4)
            sections.append(section)
            section.columnconfigure(0, weight=1)
            header = ttk.Frame(section)
            header.grid(row=0, column=0, sticky="ew", pady=(0, 5))
            ttk.Label(header, text="Data age:").pack(side="left")
            age_key = f"{prefix}.{packet}"
            age_label = tk.Label(header, text="--", width=10, anchor="center",
                                 relief="sunken", bd=1)
            age_label.pack(side="left", padx=(5, 0))
            self.age_labels[age_key] = age_label
            for field_index, (field, label) in enumerate(fields, start=1):
                key = f"{prefix}.{packet}.{field}"
                self.values[key] = tk.StringVar(value="null")
                self.check_vars[key] = tk.BooleanVar(value=False)
                row_frame = ttk.Frame(section)
                row_frame.grid(row=field_index, column=0,
                               sticky="ew", pady=(0, 2))
                row_frame.columnconfigure(2, weight=1)
                ttk.Checkbutton(row_frame, variable=self.check_vars[key]).grid(
                    row=0, column=0, sticky="w")
                ttk.Label(row_frame, text=label + ":", width=20, wraplength=145,
                          justify="left", anchor="w").grid(row=0, column=1,
                                                             sticky="w", padx=(2, 4))
                # Fixed-width value column: resizing the window changes the
                # section tiling, not the size of the displayed value cells.
                ttk.Label(row_frame, textvariable=self.values[key], width=18,
                          wraplength=135, justify="left", anchor="w").grid(
                              row=0, column=2, sticky="w")
        self.section_groups[parent] = sections
        parent.bind("<Configure>", lambda _event, frame=parent: self.retile_sections(frame))
        self.retile_sections(parent)

    def retile_sections(self, parent):
        sections = self.section_groups.get(parent, [])
        if not sections:
            return
        # Value columns remain fixed-width; only the number of tiles changes.
        tile_width = 290
        columns = max(1, min(len(sections), parent.winfo_width() // tile_width))
        if columns == 0:
            columns = 1
        if self.section_column_counts.get(parent) == columns:
            return
        self.section_column_counts[parent] = columns
        for column in range(len(sections)):
            parent.columnconfigure(column, weight=1 if column < columns else 0,
                                   minsize=0)
        for index, section in enumerate(sections):
            section.grid_configure(row=index // columns, column=index % columns)

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
        self.refresh_command_targets()

    def refresh_command_targets(self):
        targets = list(self.serials) + list(self.tcp_sockets)
        self.command_port_combo["values"] = targets
        if self.command_port_combo.get() not in targets:
            self.command_port_combo.set(targets[0] if targets else "")
        state = "normal" if targets else "disabled"
        self.command_entry.configure(state=state)
        self.send_command_button.configure(state=state)

    def send_hex_command(self):
        target_name = self.command_port_combo.get()
        serial_port = self.serials.get(target_name)
        tcp_socket = self.tcp_sockets.get(target_name)
        if serial_port is None and tcp_socket is None:
            self.console_write("TX_ERROR no connected serial/TCP target selected")
            return
        text = self.command_entry.get().strip().replace(",", " ")
        text = text.replace("0x", "").replace("0X", "")
        if not text:
            self.console_write(f"TX_ERROR[{target_name}] command is empty")
            return
        try:
            command = bytes.fromhex(text)
        except ValueError as exc:
            self.console_write(f"TX_ERROR[{target_name}] invalid hex command: {exc}")
            return
        try:
            if serial_port is not None:
                serial_port.write(command)
            else:
                tcp_socket.sendall(command)
            self.console_write(f"TX[{target_name}]  {command.hex(' ')}")
            self.command_entry.delete(0, "end")
        except Exception as exc:
            self.console_write(f"TX_ERROR[{target_name}] {exc}")

    def toggle_connection(self):
        self.disconnect() if (self.serials or self.tcp_sockets) else self.connect()

    def connect(self):
        ports = [port for port in (self.port_a_combo.get(), self.port_b_combo.get()) if port]
        tcp_endpoints = []
        for index, (host_entry, port_entry) in enumerate(self.tcp_entries, start=1):
            host = host_entry.get().strip()
            port_text = port_entry.get().strip()
            if not host:
                continue
            try:
                port = int(port_text)
            except ValueError:
                messagebox.showerror("Horizon monitor", f"TCP {index} port must be a number.")
                return
            if not 1 <= port <= 65535:
                messagebox.showerror("Horizon monitor", f"TCP {index} port must be 1-65535.")
                return
            tcp_endpoints.append((index, host, port))
        if len({(host, port) for _, host, port in tcp_endpoints}) != len(tcp_endpoints):
            messagebox.showerror("Horizon monitor", "TCP endpoints must be different.")
            return
        if not ports and not tcp_endpoints:
            messagebox.showerror("Horizon monitor", "Select a serial port or enter a TCP host.")
            return
        if len(set(ports)) != len(ports):
            messagebox.showerror("Serial monitor", "Port A and Port B must be different.")
            return
        try:
            if ports:
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
            self.refresh_command_targets()
            for index, tcp_host, tcp_port in tcp_endpoints:
                tcp_socket = socket.create_connection((tcp_host, tcp_port), timeout=3.0)
                tcp_socket.settimeout(0.2)
                tcp_source = f"TCP_{tcp_host}_{tcp_port}"
                self.tcp_sockets[tcp_source] = tcp_socket
                safe_source = "".join(char if char.isalnum() else "_" for char in tcp_source)
                self.log_files[tcp_source] = (LOG_DIR / f"{safe_source}_{stamp}.bin").open("ab")
                threading.Thread(target=self.read_tcp, args=(tcp_source, tcp_socket), daemon=True).start()
            self.refresh_command_targets()
        except Exception as exc:
            self.disconnect()
            messagebox.showerror("Serial monitor", f"Could not open serial port: {exc}")
            return
        self.connect_button.configure(text="Disconnect")
        sources = ports + [f"TCP {host}:{port}" for _, host, port in tcp_endpoints]
        self.status_label.configure(text=f"Connected: {', '.join(sources)}")

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

    def read_tcp(self, source_name, tcp_socket):
        decoder = PacketDecoder(
            lambda p: self.events.put(("packet", (source_name, p))),
            lambda message: self.events.put(("decoder_error", (source_name, message))),
        )
        while not self.stop_event.is_set() and self.tcp_sockets.get(source_name) is tcp_socket:
            try:
                data = tcp_socket.recv(4096)
                if not data:
                    self.events.put(("error", (source_name, "TCP server closed the connection")))
                    break
                log_file = self.log_files.get(source_name)
                if log_file is not None:
                    log_file.write(data)
                    log_file.flush()
                self.events.put(("bytes", (source_name, bytes(data))))
                decoder.feed(data)
            except socket.timeout:
                continue
            except Exception as exc:
                self.events.put(("error", (source_name, str(exc))))
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
        graph_height = 275
        if not selected:
            canvas.create_text(width // 2, 40, text="No fields selected", fill="#555555")
            return
        canvas.configure(scrollregion=(0, 0, width, graph_height * len(selected)))
        colors = ("#1565c0", "#c62828", "#2e7d32", "#6a1b9a", "#ef6c00")
        for graph_index, key in enumerate(selected):
            top = graph_index * graph_height
            samples = self.history.get(key, [])
            canvas.create_rectangle(5, top + 5, width - 5, top + graph_height - 45,
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
            top_plot, bottom_plot = top + 30, top + graph_height - 95
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
            latest = samples[-1][1] if samples else "--"
            canvas.create_text(left, top + graph_height - 22, anchor="w",
                               text=f"Latest: {latest}", fill="#222222",
                               width=max(200, right - left))

    def disconnect(self):
        self.stop_event.set()
        for serial_port in self.serials.values():
            try:
                serial_port.close()
            except Exception:
                pass
        self.serials.clear()
        self.refresh_command_targets()
        for tcp_socket in self.tcp_sockets.values():
            try:
                tcp_socket.close()
            except Exception:
                pass
        self.tcp_sockets.clear()
        self.refresh_command_targets()
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
