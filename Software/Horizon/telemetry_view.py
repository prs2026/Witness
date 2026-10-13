"""Scrollable, fixed-column telemetry and diagnostics view for standalone tabs."""

import datetime as dt
import json
import time
from tkinter import scrolledtext, ttk
import tkinter as tk


def flatten_fields(packet, max_fields=512):
    fields = {}
    def visit(value, path, depth):
        if depth > 16 or len(fields) >= max_fields:
            raise ValueError("Telemetry field/depth limit exceeded")
        if isinstance(value, dict) and value:
            for key, item in value.items():
                key = key.replace("\\", "\\\\").replace(".", "\\.")
                key = key.replace("[", "\\[").replace("]", "\\]")
                visit(item, f"{path}.{key}" if path else key, depth + 1)
        elif isinstance(value, list) and value:
            for index, item in enumerate(value):
                visit(item, f"{path}[{index}]", depth + 1)
        else:
            fields[path] = value
    for key, value in packet.items():
        if key != "packet":
            visit(value, key, 0)
    return fields


class TelemetryView(ttk.Frame):
    def __init__(self, parent, *, schema=None):
        super().__init__(parent)
        self.schema = schema or {}
        self._sources, self._groups, self._rows, self._received = {}, {}, {}, {}
        self._line_count = 0
        self._limit_reported = False
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        panes = ttk.PanedWindow(self, orient=tk.VERTICAL)
        panes.grid(row=0, column=0, sticky="nsew")
        fields = ttk.Frame(panes)
        fields.columnconfigure(0, weight=1)
        fields.rowconfigure(0, weight=1)
        self.tree = ttk.Treeview(fields, columns=("value", "received", "age", "quality"),
                                 show="tree headings", height=16)
        for column, title, width in (
            ("#0", "Source / packet / field", 320), ("value", "Latest value", 240),
            ("received", "Received at (UTC)", 200), ("age", "Data age", 90),
            ("quality", "Delivery / validity", 145),
        ):
            self.tree.heading(column, text=title)
            self.tree.column(column, width=width, minwidth=60, stretch=False)
        for name, color in (("fresh", "#f7f7f7"), ("yellow", "#fff2a8"),
                            ("orange", "#ffd27f"), ("stale", "#ff8a80"),
                            ("invalid", "#ff8a80")):
            self.tree.tag_configure(name, background=color)
        self.tree.grid(row=0, column=0, sticky="nsew")
        vertical = ttk.Scrollbar(fields, orient="vertical", command=self.tree.yview)
        horizontal = ttk.Scrollbar(fields, orient="horizontal", command=self.tree.xview)
        vertical.grid(row=0, column=1, sticky="ns")
        horizontal.grid(row=1, column=0, sticky="ew")
        self.tree.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)
        panes.add(fields, weight=4)
        diagnostics = ttk.LabelFrame(panes, text="Received packets and diagnostics", padding=4)
        self.console = scrolledtext.ScrolledText(diagnostics, height=7, state="disabled",
                                                wrap="word", font=("Consolas", 9))
        self.console.pack(fill="both", expand=True)
        panes.add(diagnostics, weight=1)

    def clear(self):
        self.tree.delete(*self.tree.get_children())
        self._sources.clear()
        self._groups.clear()
        self._rows.clear()
        self._received.clear()
        self._limit_reported = False

    def _row(self, source, packet, field):
        key = (source, packet, field)
        if key in self._rows:
            return self._rows[key]
        if len(self._rows) >= 8192:
            if not self._limit_reported:
                self.append(["Display field limit reached; further data remains in the input log."])
                self._limit_reported = True
            return None
        if source not in self._sources:
            self._sources[source] = self.tree.insert("", "end", text=source, open=True)
        group = (source, packet)
        if group not in self._groups:
            self._groups[group] = self.tree.insert(self._sources[source], "end", text=packet, open=True)
        row = self.tree.insert(self._groups[group], "end", text=field,
                               values=("null", "--", "--", "--"))
        self._rows[key] = row
        return row

    def add_source(self, source):
        for packet, fields in self.schema.items():
            for field in fields:
                self._row(source, packet, field)

    def update_record(self, record, *, witness=False):
        packet = record["packet"]
        fields = flatten_fields(packet)
        if witness:
            for status_field in ("status", "fc_status"):
                status = packet.get(status_field)
                if not isinstance(status, int):
                    continue
                for bit in range(8):
                    fields[f"{status_field}.byte0.bit{bit}"] = (status >> (8 + bit)) & 1
                camera = status_field == "status" and packet["packet"] in ("camera", "iris_debug")
                if not camera:
                    state = status & 7
                    fields[f"{status_field}.flight_state"] = state
                    names = ("Pad idle", "Boost", "Coast", "Descent", "Landed")
                    fields[f"{status_field}.flight_state_name"] = names[state] if state < 5 else "Reserved/invalid"
        source = record["source"]
        if source not in self._sources:
            self.add_source(source)
        received_at = dt.datetime.fromtimestamp(record["received_at"], dt.timezone.utc).isoformat(
            timespec="milliseconds").replace("T", " ").replace("+00:00", "")
        received_mono = record.get("received_mono", time.monotonic() - max(
            0, time.time() - record["received_at"]))
        invalid = packet.get("text_crc_valid") is False
        quality = "BAD TEXT CRC" if invalid else "Snapshot" if record.get("replay") else "Live"
        for field, value in fields.items():
            row = self._row(source, packet["packet"], field)
            if row is None:
                continue
            if witness and field in ("status", "fc_status") and isinstance(value, int):
                display = f"0b{value:016b}"
            else:
                display = value if isinstance(value, str) else json.dumps(value, ensure_ascii=True)
            self.tree.item(row, values=(display, received_at, "--", quality))
            self._received[row] = received_mono, invalid

    def update_ages(self):
        now = time.monotonic()
        for row, (received, invalid) in self._received.items():
            age = max(0, now - received)
            tag = ("invalid" if invalid else "fresh" if age < 2 else "yellow" if age < 5
                   else "orange" if age < 10 else "stale")
            self.tree.set(row, "age", f"{age:.1f}s")
            self.tree.item(row, tags=(tag,))

    def append(self, lines):
        if not lines:
            return
        stamp = dt.datetime.now().strftime("%H:%M:%S")
        self.console.configure(state="normal")
        self.console.insert("end", "\n".join(f"[{stamp}] {line}" for line in lines) + "\n")
        self._line_count += len(lines)
        if self._line_count > 1000:
            self.console.delete("1.0", f"{self._line_count - 999}.0")
            self._line_count = 1000
        self.console.see("end")
        self.console.configure(state="disabled")
