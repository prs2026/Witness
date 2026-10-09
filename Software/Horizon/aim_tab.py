"""Independent AIM XTRA MQTT tab for the Horizon Tk interface."""

from __future__ import annotations

import json
from pathlib import Path
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

from aim_mqtt import AIM_TOPIC_FIELDS, AimMqttReceiver, AimSettings


class AimMqttTab(ttk.Frame):
    def __init__(self, parent, *, log_dir: Path, host, port, client_uuid, base_path):
        super().__init__(parent, padding=8)
        self.log_dir = log_dir
        self.receiver = None
        self._groups = {}
        self._rows = {}
        self._received = {}
        self._raw_lines = 0
        self._after_id = None
        self._limit_reported = False
        self.host = tk.StringVar(value=host)
        self.port = tk.StringVar(value=str(port))
        self.client_uuid = tk.StringVar(value=client_uuid)
        self.base_path = tk.StringVar(value=base_path)
        self.status = tk.StringVar(value="Disconnected")
        self.stats = tk.StringVar(value="Messages: 0   Bytes: 0   Errors: 0")
        self.log_status = tk.StringVar(value="")
        self._build_ui()
        self._reset_fields()
        self._after_id = self.after(100, self._poll)

    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(4, weight=1)
        controls = ttk.Frame(self)
        controls.grid(row=0, column=0, sticky="ew")
        controls.columnconfigure(1, weight=1)
        self.entries = []
        for column, (label, variable, width) in enumerate((
            ("MQTT host:", self.host, 20),
            ("Port:", self.port, 6),
            ("Subscriber UUID:", self.client_uuid, 10),
        )):
            ttk.Label(controls, text=label).grid(row=0, column=column * 2, padx=(0, 4))
            entry = ttk.Entry(controls, textvariable=variable, width=width)
            entry.grid(row=0, column=column * 2 + 1, sticky="ew", padx=(0, 10))
            self.entries.append(entry)
        self.connect_button = ttk.Button(controls, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=6)

        topic_controls = ttk.Frame(self)
        topic_controls.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        topic_controls.columnconfigure(1, weight=1)
        ttk.Label(topic_controls, text="Base path:").grid(row=0, column=0, padx=(0, 4))
        entry = ttk.Entry(topic_controls, textvariable=self.base_path)
        entry.grid(row=0, column=1, sticky="ew")
        self.entries.append(entry)
        ttk.Label(topic_controls, text="/#  (receive only)").grid(row=0, column=2)

        information = ttk.Frame(self)
        information.grid(row=2, column=0, sticky="ew", pady=5)
        information.columnconfigure(0, weight=1)
        status_label = ttk.Label(information, textvariable=self.status, wraplength=650)
        status_label.grid(row=0, column=0, sticky="ew")
        information.bind("<Configure>", lambda event: status_label.configure(
            wraplength=max(150, event.width)))
        ttk.Label(information, textvariable=self.stats).grid(row=1, column=0, sticky="w")
        ttk.Label(
            self,
            text="Values and timestamps are shown as published. Retained messages may be old. "
                 "Drag column dividers to resize; scroll to see all fields.",
            wraplength=750,
        ).grid(row=3, column=0, sticky="ew", pady=(0, 5))

        pane = ttk.PanedWindow(self, orient=tk.VERTICAL)
        pane.grid(row=4, column=0, sticky="nsew")
        fields_frame = ttk.Frame(pane)
        fields_frame.columnconfigure(0, weight=1)
        fields_frame.rowconfigure(0, weight=1)
        self.tree = ttk.Treeview(
            fields_frame, columns=("value", "received", "age", "delivery"),
            show="tree headings", height=14,
        )
        for column, title, width in (
            ("#0", "Topic / field", 340), ("value", "Latest value", 200),
            ("received", "Received at (UTC)", 205), ("age", "Receive age", 95),
            ("delivery", "Delivery", 80),
        ):
            self.tree.heading(column, text=title)
            self.tree.column(column, width=width, minwidth=60, stretch=False)
        self.tree.tag_configure("fresh", background="#f7f7f7")
        self.tree.tag_configure("yellow", background="#fff2a8")
        self.tree.tag_configure("orange", background="#ffd27f")
        self.tree.tag_configure("stale", background="#ff8a80")
        self.tree.tag_configure("retained", background="#e4e4e4")
        self.tree.grid(row=0, column=0, sticky="nsew")
        vertical = ttk.Scrollbar(fields_frame, orient="vertical", command=self.tree.yview)
        vertical.grid(row=0, column=1, sticky="ns")
        horizontal = ttk.Scrollbar(fields_frame, orient="horizontal", command=self.tree.xview)
        horizontal.grid(row=1, column=0, sticky="ew")
        self.tree.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)
        # The Telemetry page uses a global wheel binding. Handle wheel events
        # locally here so switching tabs cannot scroll the hidden page instead.
        self.tree.bind("<MouseWheel>", self._scroll_fields)
        pane.add(fields_frame, weight=4)

        raw_frame = ttk.LabelFrame(pane, text="MQTT messages and diagnostics", padding=4)
        self.raw = scrolledtext.ScrolledText(
            raw_frame, height=6, wrap="word", state="disabled", font=("Consolas", 9),
        )
        self.raw.pack(fill="both", expand=True)
        pane.add(raw_frame, weight=1)
        log_label = ttk.Label(self, textvariable=self.log_status, wraplength=700)
        log_label.grid(row=5, column=0, sticky="ew", pady=(4, 0))
        self.bind("<Configure>", lambda event: log_label.configure(
            wraplength=max(150, event.width - 16)))

    def _scroll_fields(self, event):
        self.tree.yview_scroll(-int(event.delta / 120), "units")
        return "break"

    def _reset_fields(self):
        self.tree.delete(*self.tree.get_children())
        self._groups.clear()
        self._rows.clear()
        self._received.clear()
        self._limit_reported = False
        for topic, fields in AIM_TOPIC_FIELDS.items():
            for field in fields:
                self._ensure_row(topic, field)

    def _ensure_row(self, topic, field):
        key = (topic, field)
        if key not in self._rows:
            # Keep a badly behaved publisher from creating unlimited Tk rows.
            if len(self._rows) >= 4096:
                if not self._limit_reported:
                    self._append_lines(["Display field limit reached; further fields remain in the log."])
                    self._limit_reported = True
                return None
            if topic not in self._groups:
                self._groups[topic] = self.tree.insert("", "end", text=topic, open=True)
            self._rows[key] = self.tree.insert(
                self._groups[topic], "end", text=field,
                values=("null", "--", "--", "--"),
            )
        return self._rows[key]

    def toggle_connection(self):
        if self.receiver is not None:
            self.disconnect()
            return
        try:
            settings = AimSettings(
                self.host.get().strip(), int(self.port.get()),
                self.client_uuid.get().strip(), self.base_path.get().strip().rstrip("/"),
            )
            receiver = AimMqttReceiver(settings, self.log_dir)
            receiver.start()
        except (ValueError, RuntimeError, OSError) as exc:
            messagebox.showerror("AIM XTRA MQTT", str(exc), parent=self)
            return
        self.receiver = receiver
        self._reset_fields()
        self.status.set(f"Connecting to {settings.host}:{settings.port}...")
        self.stats.set("Messages: 0   Bytes: 0   Errors: 0")
        self.log_status.set(f"Log: {receiver.log_path}")
        for entry in self.entries:
            entry.configure(state="disabled")
        self.connect_button.configure(text="Disconnect")

    def disconnect(self):
        receiver, self.receiver = self.receiver, None
        if receiver is not None:
            self._consume(receiver)
            receiver.stop()
        self.status.set("Disconnected — latest values retained")
        self.connect_button.configure(text="Connect")
        for entry in self.entries:
            entry.configure(state="normal")

    def _consume(self, receiver):
        updates, stats, diagnostics = receiver.drain()
        status, messages, byte_count, errors = stats
        self.status.set(status)
        self.stats.set(f"Messages: {messages}   Bytes: {byte_count}   Errors: {errors}")
        for topic, fields in updates.items():
            for field, (value, received_at, received_mono, retained) in fields.items():
                row = self._ensure_row(topic, field)
                if row is None:
                    continue
                display = value if isinstance(value, str) else json.dumps(value, ensure_ascii=True)
                self.tree.item(row, values=(
                    display, received_at.replace("T", " ").replace("+00:00", ""),
                    "0.0s", "Retained" if retained else "Live",
                ))
                self._received[row] = (received_mono, retained)
        self._append_lines(diagnostics)

    def _append_lines(self, lines):
        if not lines:
            return
        self.raw.configure(state="normal")
        self.raw.insert("end", "\n".join(lines) + "\n")
        self._raw_lines += len(lines)
        if self._raw_lines > 1000:
            excess = self._raw_lines - 1000
            self.raw.delete("1.0", f"{excess + 1}.0")
            self._raw_lines = 1000
        self.raw.see("end")
        self.raw.configure(state="disabled")

    def _poll(self):
        try:
            if self.receiver is not None:
                self._consume(self.receiver)
            now = time.monotonic()
            for row, (received, retained) in self._received.items():
                age = max(0, now - received)
                tag = ("fresh" if age < 2 else "yellow" if age < 5
                       else "orange" if age < 10 else "stale")
                self.tree.set(row, "age", f"{age:.1f}s")
                self.tree.item(row, tags=("retained" if retained else tag,))
        finally:
            self._after_id = self.after(100, self._poll)

    def close(self):
        if self._after_id is not None:
            self.after_cancel(self._after_id)
            self._after_id = None
        self.disconnect()
