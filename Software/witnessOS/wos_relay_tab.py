"""witnessOS "Witness relay" tab: GUI for wos_relay.RelayEngine."""

from __future__ import annotations

import queue
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

import wos_relay as wr

SRC_TCP = "tcp"
SRC_SERIAL = "serial"
SRC_HITL = "hitl"
POLL_MS = 250
MAX_LOG_LINES = 2000


def serial_ports() -> list[str] | None:
    """Available serial port names, or None when pyserial is missing."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    return sorted(p.device for p in list_ports.comports())


class RelayTab(ttk.Frame):
    def __init__(self, master, hitl_session=None, *, engine=None,
                 default_host: str = "prs2026hitl", default_port: int = 7100,
                 start_feed: bool = True):
        super().__init__(master, padding=6)
        self.engine = engine or wr.RelayEngine()
        self.hitl_session = hitl_session
        self._closing = False
        self._selected_kind = None

        # --- input
        box = ttk.LabelFrame(self, text="Witness input", padding=6)
        box.pack(fill="x")
        row = ttk.Frame(box)
        row.pack(fill="x")
        self.source_var = tk.StringVar(value=SRC_HITL if hitl_session
                                       else SRC_TCP)
        ttk.Radiobutton(row, text="Raw TCP", value=SRC_TCP,
                        variable=self.source_var,
                        command=self._refresh_controls).pack(side="left")
        ttk.Radiobutton(row, text="USB serial", value=SRC_SERIAL,
                        variable=self.source_var,
                        command=self._refresh_controls).pack(side="left",
                                                             padx=(8, 0))
        self.hitl_radio = ttk.Radiobutton(
            row, text="Share HITL tab connection", value=SRC_HITL,
            variable=self.source_var, command=self._refresh_controls)
        self.hitl_radio.pack(side="left", padx=(8, 0))
        if hitl_session is None:
            self.hitl_radio.state(["disabled"])

        row = ttk.Frame(box)
        row.pack(fill="x", pady=(4, 0))
        ttk.Label(row, text="Host").pack(side="left")
        self.host_var = tk.StringVar(value=default_host)
        self.host_entry = ttk.Entry(row, textvariable=self.host_var, width=16)
        self.host_entry.pack(side="left")
        ttk.Label(row, text="Port").pack(side="left", padx=(6, 0))
        self.port_var = tk.StringVar(value=str(default_port))
        self.port_entry = ttk.Entry(row, textvariable=self.port_var, width=6)
        self.port_entry.pack(side="left")
        ttk.Label(row, text="Serial").pack(side="left", padx=(10, 0))
        self.serial_var = tk.StringVar()
        self.serial_combo = ttk.Combobox(row, textvariable=self.serial_var,
                                         width=12)
        self.serial_combo.pack(side="left")
        self.serial_refresh = ttk.Button(row, text="↻", width=3,
                                         command=self.refresh_serial)
        self.serial_refresh.pack(side="left", padx=(2, 0))

        row = ttk.Frame(box)
        row.pack(fill="x", pady=(4, 0))
        self.connect_btn = ttk.Button(row, text="Connect",
                                      command=self.toggle_connect)
        self.connect_btn.pack(side="left")
        self.record_var = tk.BooleanVar(value=self.engine.record_raw)
        ttk.Checkbutton(row, text="Record raw .bin to Horizon/logs",
                        variable=self.record_var,
                        command=self._record_changed).pack(side="left",
                                                           padx=8)
        self.input_label = ttk.Label(row, text="Disconnected")
        self.input_label.pack(side="left", padx=8)
        self.note = ttk.Label(box, foreground="#555", justify="left",
                              wraplength=420, text=(
                                  "Witness's raw TCP port serves one client "
                                  "at a time. While the HITL tab is "
                                  "connected, use 'Share HITL tab "
                                  "connection'. The relay never transmits "
                                  "to Witness."))
        self.note.pack(fill="x", pady=(4, 0))

        # --- feed
        fbox = ttk.LabelFrame(self, text="Horizon Witness feed "
                              "(horizon.witness.v1 JSON, 127.0.0.1)",
                              padding=6)
        fbox.pack(fill="x", pady=(6, 0))
        row = ttk.Frame(fbox)
        row.pack(fill="x")
        ttk.Label(row, text="Port").pack(side="left")
        self.feed_port_var = tk.StringVar(value=str(self.engine.feed_port))
        ttk.Entry(row, textvariable=self.feed_port_var,
                  width=6).pack(side="left")
        self.feed_btn = ttk.Button(row, text="Start feed",
                                   command=self.toggle_feed)
        self.feed_btn.pack(side="left", padx=6)
        self.feed_label = ttk.Label(fbox, text="Feed stopped", justify="left",
                                    wraplength=420)
        self.feed_label.pack(fill="x", pady=(4, 0))
        self.counters_label = ttk.Label(self, text="", font=("Consolas", 9),
                                        justify="left", wraplength=420)
        self.counters_label.pack(fill="x", pady=(6, 0))
        self.bind("<Configure>", self._wrap)

        # --- packet tables and log (draggable splits)
        vsplit = ttk.PanedWindow(self, orient="vertical")
        vsplit.pack(fill="both", expand=True, pady=(6, 0))
        tables = ttk.PanedWindow(vsplit, orient="horizontal")
        vsplit.add(tables, weight=3)

        left = ttk.Frame(tables)
        tables.add(left, weight=1)
        cols = ("type", "count", "rate", "age")
        self.packets = ttk.Treeview(left, columns=cols, show="headings",
                                    height=6, selectmode="browse")
        for c, text, w in zip(cols, ("Packet", "Count", "Hz", "Age s"),
                              (100, 60, 45, 50)):
            self.packets.heading(c, text=text)
            self.packets.column(c, width=w, minwidth=40,
                                anchor="w" if c == "type" else "e")
        psb = ttk.Scrollbar(left, orient="vertical",
                            command=self.packets.yview)
        self.packets.configure(yscrollcommand=psb.set)
        self.packets.pack(side="left", fill="both", expand=True)
        psb.pack(side="right", fill="y")
        self.packets.bind("<<TreeviewSelect>>", self._on_select)

        right = ttk.Frame(tables)
        tables.add(right, weight=2)
        self.fields = ttk.Treeview(right, columns=("field", "value"),
                                   show="headings", height=6)
        self.fields.heading("field", text="Field")
        self.fields.heading("value", text="Latest value")
        self.fields.column("field", width=120, minwidth=60, anchor="w")
        self.fields.column("value", width=160, minwidth=60, anchor="w")
        fsb = ttk.Scrollbar(right, orient="vertical",
                            command=self.fields.yview)
        fxsb = ttk.Scrollbar(right, orient="horizontal",
                             command=self.fields.xview)
        self.fields.configure(yscrollcommand=fsb.set,
                              xscrollcommand=fxsb.set)
        fxsb.pack(side="bottom", fill="x")
        self.fields.pack(side="left", fill="both", expand=True)
        fsb.pack(side="right", fill="y")

        self.logbox = ScrolledText(vsplit, height=5, width=30,
                                   font=("Consolas", 9), state="disabled")
        vsplit.add(self.logbox, weight=1)

        self.refresh_serial()
        self._refresh_controls()
        if start_feed:
            self.toggle_feed()
        self._poll_job = self.after(POLL_MS, self.poll)

    # ------------------------------------------------------------ actions
    def refresh_serial(self):
        ports = serial_ports()
        if ports is None:
            self.serial_combo["values"] = ()
            self.serial_var.set("(pyserial not installed)")
            self._serial_ok = False
        else:
            self.serial_combo["values"] = ports
            if self.serial_var.get() not in ports:
                self.serial_var.set(ports[0] if ports else "")
            self._serial_ok = True
        self._refresh_controls()

    def _record_changed(self):
        self.engine.record_raw = bool(self.record_var.get())

    def toggle_connect(self):
        eng = self.engine
        if eng.source is not None:
            eng.disconnect()
            self._refresh_controls()
            return
        eng.reset_stats()
        kind = self.source_var.get()
        try:
            if kind == SRC_HITL:
                eng.share_session(self.hitl_session)
            elif kind == SRC_SERIAL:
                eng.connect_serial(self.serial_var.get().strip())
            else:
                host = self.host_var.get().strip()
                port = int(self.port_var.get())
                if (self.hitl_session is not None
                        and self.hitl_session.connected
                        and getattr(self.hitl_session, "_host", None) == host):
                    self.append_log("[relay] the HITL tab is connected to "
                                    f"{host}; Witness serves one TCP client, "
                                    "so use 'Share HITL tab connection'")
                    return
                eng.connect_tcp(host, port)
        except (OSError, ValueError, RuntimeError, ImportError) as exc:
            self.append_log(f"[relay] connect failed: {exc}")
        self._refresh_controls()

    def toggle_feed(self):
        eng = self.engine
        if eng.publisher is not None:
            eng.stop_feed()
        else:
            try:
                port = int(self.feed_port_var.get())
            except ValueError:
                self.append_log("[relay] invalid feed port")
                return
            eng.start_feed(port)
        self._refresh_controls()

    def close(self):
        self._closing = True
        if self._poll_job is not None:
            self.after_cancel(self._poll_job)
            self._poll_job = None
        self.engine.close()

    # ------------------------------------------------------------ display
    def _wrap(self, event):
        if event.widget is self:
            width = max(200, event.width - 30)
            for label in (self.note, self.feed_label, self.counters_label):
                label.configure(wraplength=width)

    def _refresh_controls(self):
        eng = self.engine
        active = eng.source is not None
        kind = self.source_var.get()
        for widget, on in ((self.host_entry, kind == SRC_TCP),
                           (self.port_entry, kind == SRC_TCP),
                           (self.serial_combo,
                            kind == SRC_SERIAL and self._serial_ok),
                           (self.serial_refresh, kind == SRC_SERIAL)):
            widget.state(["!disabled"] if on and not active
                         else ["disabled"])
        self.connect_btn.config(text="Disconnect" if active else "Connect")
        self.feed_btn.config(text="Stop feed" if eng.publisher is not None
                             else "Start feed")

    def append_log(self, line: str):
        lb = self.logbox
        lb.config(state="normal")
        lb.insert("end", line + "\n")
        if int(lb.index("end-1c").split(".")[0]) > MAX_LOG_LINES:
            lb.delete("1.0", "500.0")
        lb.see("end")
        lb.config(state="disabled")

    def _on_select(self, _event=None):
        sel = self.packets.selection()
        if sel:
            self._selected_kind = sel[0]
            self._update_fields(force=True)

    def _update_fields(self, snap=None, force=False):
        snap = snap or self.engine.snapshot()
        kind = self._selected_kind
        packets = snap["packets"]
        if kind not in packets:
            kind = max(packets, key=lambda k: packets[k]["count"],
                       default=None)
        if kind is None:
            self.fields.delete(*self.fields.get_children())
            return
        rows = wr.flatten_packet(packets[kind]["latest"])
        existing = self.fields.get_children()
        if len(existing) != len(rows) or force:
            self.fields.delete(*existing)
            for name, value in rows:
                self.fields.insert("", "end", iid=name, values=(name, value))
        else:
            for name, value in rows:
                if self.fields.exists(name):
                    self.fields.item(name, values=(name, value))

    def poll(self):
        if self._closing:
            return
        try:
            for _ in range(200):
                try:
                    kind, message = self.engine.events.get_nowait()
                except queue.Empty:
                    break
                prefix = {"error": "[relay] ERROR ", "decode": "[decode] "
                          }.get(kind, "[relay] ")
                self.append_log(prefix + message)
            if self.winfo_ismapped():
                self._update_display()
        except tk.TclError:
            return
        self._poll_job = self.after(POLL_MS, self.poll)

    def _update_display(self):
        snap = self.engine.snapshot()
        feed = snap["feed"]
        if feed.get("running"):
            text = (f"Serving on {feed['host']}:{feed['port']} - "
                    f"{feed['sequence']:,} records published")
            if feed.get("dropped"):
                text += f", {feed['dropped']:,} dropped (slow client)"
        elif feed.get("error"):
            text = f"Feed stopped: {feed['error']}"
        else:
            text = "Feed stopped"
        self.feed_label.config(text=text)
        if snap["source"] is None:
            self.input_label.config(text="Disconnected")
        else:
            state = "connected" if snap["connected"] else "waiting"
            self.input_label.config(text=f"{snap['source']} ({state})")
        self.counters_label.config(text=(
            f"rx {snap['rx_bytes']:,} B   decoded "
            f"{sum(p['count'] for p in snap['packets'].values()):,}   "
            f"decode errors {snap['decode_errors']:,}   "
            f"host-echo frames {snap['host_packets']:,}"))
        packets = snap["packets"]
        for kind in sorted(packets):
            p = packets[kind]
            values = (kind, f"{p['count']:,}", f"{p['rate_hz']:.1f}",
                      f"{p['age_s']:.1f}")
            if self.packets.exists(kind):
                self.packets.item(kind, values=values)
            else:
                self.packets.insert("", "end", iid=kind, values=values)
        for iid in self.packets.get_children():
            if iid not in packets:
                self.packets.delete(iid)
        self._update_fields(snap)
        self._refresh_controls()
