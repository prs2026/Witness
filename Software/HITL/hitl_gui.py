"""Witness hardware-in-the-loop simulator: Tkinter GUI and headless CLI.

GUI:       python hitl_gui.py
Headless:  python hitl_gui.py --headless --preset "Nominal 10 g" \
               --host prs2026hitl --port 7100

The tool only ever transmits HITL ENTER (0x4849), HITL EXIT (0x4858) and 0x0C
HITL sensor frames. See README.md for the safety rules.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
import queue
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import hitl_protocol as hp  # noqa: E402
import hitl_session as hs  # noqa: E402
import sim_model as sm  # noqa: E402

SETTINGS_PATH = os.path.join(HERE, "hitl_settings.json")
DEFAULT_HOST = "prs2026hitl"
DEFAULT_PORT = 7100
DEFAULT_TOLERANCE_S = 1.0


def load_settings(path: str = SETTINGS_PATH) -> dict:
    settings = {"host": DEFAULT_HOST, "port": DEFAULT_PORT,
                "preset": "Nominal 10 g", "tolerance_s": DEFAULT_TOLERANCE_S}
    try:
        with open(path) as f:
            data = json.load(f)
        if isinstance(data, dict):
            settings.update({k: data[k] for k in settings if k in data})
    except (OSError, ValueError):
        pass
    return settings


def save_settings(settings: dict, path: str = SETTINGS_PATH):
    try:
        with open(path, "w") as f:
            json.dump(settings, f, indent=2)
    except OSError:
        pass


def parse_overrides(items) -> dict:
    """--set key=value pairs -> typed SimParams overrides."""
    types = {f.name: f.type for f in dataclasses.fields(sm.SimParams)}
    out = {}
    for item in items or []:
        key, _, raw = item.partition("=")
        key = key.strip()
        if key not in types:
            raise SystemExit(f"unknown parameter {key!r}")
        out[key] = coerce(types[key], raw.strip())
    return out


def coerce(type_name, raw: str):
    t = str(type_name)
    if "None" in t and raw.lower() in ("", "none"):
        return None
    if "bool" in t:
        return raw.lower() in ("1", "true", "yes", "on")
    if "int" in t and "float" not in t:
        return int(float(raw))
    return float(raw)


# ---------------------------------------------------------------------------
# Headless
# ---------------------------------------------------------------------------

def run_headless(args) -> int:
    try:
        params = sm.preset_params(args.preset)
    except KeyError as exc:
        print(exc.args[0], file=sys.stderr)
        return 2
    params = dataclasses.replace(params, **parse_overrides(args.set))
    runs_dir = None if args.no_save else (args.runs_dir or hs.DEFAULT_RUNS_DIR)
    session = hs.HitlSession(runs_dir=runs_dir)
    run = session.prepare_run(params, preset=args.preset,
                              tolerance_s=args.tolerance,
                              time_scale=args.time_scale)
    summary = sm.summarize(run.samples)
    print(f"Preset {args.preset!r}: {summary['duration_s']:.1f} s sim, "
          f"apogee {summary['apogee_agl_m']:.0f} m AGL at "
          f"{summary['apogee_t_s']:.1f} s")
    try:
        session.connect(args.host, args.port, timeout=args.connect_timeout)
    except OSError as exc:
        print(f"connect to {args.host}:{args.port} failed: {exc}",
              file=sys.stderr)
        return 2

    def drain():
        while True:
            try:
                kind, payload = session.events.get_nowait()
            except queue.Empty:
                return
            if args.verbose and kind in ("log", "text"):
                print(payload)
            elif kind == "warning":
                print("WARNING:", payload)

    # Give Witness a moment to report its current state before starting.
    deadline = time.monotonic() + args.settle
    while time.monotonic() < deadline and session.view.flight_state is None:
        time.sleep(0.05)
    drain()
    try:
        session.start_run(run, abort_on_no_hitl=True)
        while not session.wait_run(0.2):
            drain()
            if not session.connected:
                break
    except KeyboardInterrupt:
        session.stop_run("interrupted")
    finally:
        session.wait_run(5)
        drain()
        session.disconnect(send_exit=False)
    result = run.result or hs.evaluate(run)
    print(hs.format_table(result))
    if run.run_dir:
        print(f"Run saved to {run.run_dir}")
    return 0 if result["passed"] else 1


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

STATUS_LAMPS = (
    ("HITL active", lambda s: s["hitl_active"], "good"),
    ("GPS locked", lambda s: s["gps_locked"], "good"),
    ("Low-g", lambda s: s["low_g_ready"], "good"),
    ("High-g", lambda s: s["high_g_ready"], "good"),
    ("Gyro", lambda s: s["gyro_ready"], "good"),
    ("IMU temp", lambda s: s["imu_temp_ready"], "good"),
    ("Baro", lambda s: s["baro_ready"], "good"),
    ("Heartbeat", lambda s: s["heartbeat"], "good"),
    ("NAND fail", lambda s: s["nand_init_failed"], "bad"),
    ("Flash log fail", lambda s: s["flash_log_failed"], "bad"),
)


class HitlApp:
    def __init__(self, root, settings: dict | None = None,
                 settings_path: str = SETTINGS_PATH,
                 runs_dir: str | None = hs.DEFAULT_RUNS_DIR):
        import tkinter as tk
        from tkinter import ttk
        from tkinter.scrolledtext import ScrolledText
        self.tk = tk
        self.root = root
        self.settings_path = settings_path
        self.settings = settings or load_settings(settings_path)
        self.session = hs.HitlSession(runs_dir=runs_dir)
        self._shown_run = None
        self._closing = False
        root.title("Witness HITL")
        root.geometry("1280x860")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        # --- connection bar
        bar = ttk.Frame(root, padding=4)
        bar.pack(side="top", fill="x")
        ttk.Label(bar, text="Host").pack(side="left")
        self.host_var = tk.StringVar(value=self.settings["host"])
        ttk.Entry(bar, textvariable=self.host_var, width=16).pack(side="left")
        ttk.Label(bar, text="Port").pack(side="left", padx=(6, 0))
        self.port_var = tk.StringVar(value=str(self.settings["port"]))
        ttk.Entry(bar, textvariable=self.port_var, width=6).pack(side="left")
        self.connect_btn = ttk.Button(bar, text="Connect",
                                      command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=4)
        self.conn_label = ttk.Label(bar, text="Disconnected", width=14)
        self.conn_label.pack(side="left")
        self.bytes_label = ttk.Label(bar, text="rx 0 / tx 0 B")
        self.bytes_label.pack(side="left", padx=8)
        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y",
                                                   padx=6)
        self.btn_enter = ttk.Button(bar, text="Enter HITL",
                                    command=self.enter_hitl)
        self.btn_start = ttk.Button(bar, text="Start sim",
                                    command=self.start_sim)
        self.btn_stop = ttk.Button(bar, text="Stop", command=self.stop_sim)
        self.btn_exit = ttk.Button(bar, text="Exit HITL",
                                   command=self.exit_hitl)
        for b in (self.btn_enter, self.btn_start, self.btn_stop,
                  self.btn_exit):
            b.pack(side="left", padx=2)
        ttk.Label(bar, text="Tolerance s").pack(side="left", padx=(8, 0))
        self.tol_var = tk.StringVar(value=str(self.settings["tolerance_s"]))
        ttk.Entry(bar, textvariable=self.tol_var, width=5).pack(side="left")

        self.banner = tk.Label(root, text="", bg="#b00020", fg="white",
                               font=("Segoe UI", 11, "bold"))

        main = ttk.PanedWindow(root, orient="horizontal")
        main.pack(fill="both", expand=True)

        # --- left: preset + parameters
        left = ttk.Frame(main, padding=4)
        main.add(left, weight=0)
        ttk.Label(left, text="Preset").pack(anchor="w")
        self.preset_var = tk.StringVar(value=self.settings["preset"]
                                       if self.settings["preset"] in
                                       sm.PRESETS else "Nominal 10 g")
        cb = ttk.Combobox(left, textvariable=self.preset_var,
                          values=list(sm.PRESETS), state="readonly", width=28)
        cb.pack(anchor="w", fill="x")
        cb.bind("<<ComboboxSelected>>", lambda e: self.load_preset())
        pf_outer = ttk.LabelFrame(left, text="Parameters", padding=2)
        pf_outer.pack(fill="both", expand=True, pady=4)
        canvas = tk.Canvas(pf_outer, width=270, highlightthickness=0)
        sb = ttk.Scrollbar(pf_outer, orient="vertical", command=canvas.yview)
        pf = ttk.Frame(canvas)
        pf.bind("<Configure>", lambda e: canvas.configure(
            scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=pf, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.param_vars = {}
        self.param_types = {}
        for r, f in enumerate(dataclasses.fields(sm.SimParams)):
            ttk.Label(pf, text=f.name).grid(row=r, column=0, sticky="w")
            t = str(f.type)
            self.param_types[f.name] = t
            if "bool" in t:
                var = tk.BooleanVar()
                ttk.Checkbutton(pf, variable=var).grid(row=r, column=1,
                                                       sticky="w")
            else:
                var = tk.StringVar()
                ttk.Entry(pf, textvariable=var, width=10).grid(
                    row=r, column=1, sticky="w")
            self.param_vars[f.name] = var
        ttk.Button(left, text="Predict transitions",
                   command=self.predict).pack(fill="x")
        self.predict_label = ttk.Label(left, text="", justify="left",
                                       wraplength=280)
        self.predict_label.pack(anchor="w", fill="x")

        # --- right: status, plot, table, log
        right = ttk.Frame(main, padding=4)
        main.add(right, weight=1)
        st = ttk.Frame(right)
        st.pack(fill="x")
        self.state_label = tk.Label(st, text="State: ?", width=18,
                                    font=("Segoe UI", 16, "bold"),
                                    anchor="w")
        self.state_label.pack(side="left")
        lamps = ttk.Frame(st)
        lamps.pack(side="left", padx=6)
        self.lamps = []
        for i, (name, _fn, _kind) in enumerate(STATUS_LAMPS):
            c = tk.Canvas(lamps, width=14, height=14, highlightthickness=0)
            oval = c.create_oval(2, 2, 12, 12, fill="#888")
            c.grid(row=i // 5, column=(i % 5) * 2)
            ttk.Label(lamps, text=name).grid(row=i // 5, column=(i % 5) * 2 + 1,
                                            sticky="w", padx=(0, 8))
            self.lamps.append((c, oval))
        self.info_label = ttk.Label(right, text="", font=("Consolas", 10))
        self.info_label.pack(fill="x")

        self.plot = tk.Canvas(right, height=300, bg="white")
        self.plot.pack(fill="both", expand=True, pady=4)

        lower = ttk.Frame(right)
        lower.pack(fill="both", expand=True)
        cols = ("state", "expected", "witness", "delta", "result")
        self.table = ttk.Treeview(lower, columns=cols, show="headings",
                                  height=5)
        for c, w in zip(cols, (90, 90, 90, 80, 90)):
            self.table.heading(c, text=c.capitalize())
            self.table.column(c, width=w, anchor="center")
        self.table.pack(side="left", fill="y")
        self.logbox = ScrolledText(lower, height=12, font=("Consolas", 9),
                                   state="disabled")
        self.logbox.pack(side="left", fill="both", expand=True, padx=(4, 0))

        self.load_preset()
        self._refresh_buttons()
        self.root.after(100, self.poll)
        self.root.after(250, self.redraw_plot)

    # ------------------------------------------------------------ params
    def load_preset(self):
        p = sm.preset_params(self.preset_var.get())
        for name, value in p.to_dict().items():
            var = self.param_vars[name]
            if "bool" in self.param_types[name]:
                var.set(bool(value))
            else:
                var.set("" if value is None else str(value))
        self.predict()

    def read_params(self) -> sm.SimParams:
        values = {}
        for name, var in self.param_vars.items():
            t = self.param_types[name]
            if "bool" in t:
                values[name] = bool(var.get())
            else:
                try:
                    values[name] = coerce(t, var.get().strip())
                except ValueError:
                    raise ValueError(f"bad value for {name}: {var.get()!r}")
        return sm.SimParams(**values)

    def predict(self):
        try:
            params = self.read_params()
        except ValueError as exc:
            self.predict_label.config(text=str(exc))
            return None
        samples = sm.generate(params)
        exp = sm.predict_transitions(samples)
        s = sm.summarize(samples)
        text = (f"Duration {s['duration_s']:.1f} s, apogee "
                f"{s['apogee_agl_m']:.0f} m AGL at {s['apogee_t_s']:.1f} s\n"
                + "\n".join(f"{hp.FLIGHT_STATES[k]}: {v:.2f} s"
                            for k, v in sorted(exp.items(),
                                               key=lambda kv: kv[1]))
                + ("" if exp else "No transitions expected"))
        self.predict_label.config(text=text)
        if self.session.run is None or not self.session.run.running:
            self._fill_table({k: v for k, v in exp.items()}, {})
        return params

    # ----------------------------------------------------------- actions
    def toggle_connect(self):
        if self.session.connected:
            self.session.disconnect()
            return
        host = self.host_var.get().strip()
        try:
            port = int(self.port_var.get())
        except ValueError:
            self.append_log("[tool] invalid port")
            return
        self.settings.update(host=host, port=port)
        save_settings(self.settings, self.settings_path)
        try:
            self.session.connect(host, port, timeout=5)
        except OSError as exc:
            self.append_log(f"[tool] connect to {host}:{port} failed: {exc}")
        self._refresh_buttons()

    def enter_hitl(self):
        self._safe(lambda: self.session.send_command(hp.CMD_HITL_ENTER))

    def exit_hitl(self):
        if self.session.run is not None and self.session.run.running:
            self.session.stop_run("Exit HITL pressed")
        else:
            self._safe(lambda: self.session.send_command(hp.CMD_HITL_EXIT))

    def start_sim(self):
        try:
            params = self.read_params()
            tol = float(self.tol_var.get())
        except ValueError as exc:
            self.append_log(f"[tool] {exc}")
            return
        self.settings.update(preset=self.preset_var.get(), tolerance_s=tol)
        save_settings(self.settings, self.settings_path)
        run = self.session.prepare_run(params, preset=self.preset_var.get(),
                                       tolerance_s=tol)
        self.hide_banner()
        self._safe(lambda: self.session.start_run(run))
        self._refresh_buttons()

    def stop_sim(self):
        self.session.stop_run("stopped by user")
        self._refresh_buttons()

    def _safe(self, fn):
        try:
            fn()
        except (OSError, RuntimeError, ValueError) as exc:
            self.append_log(f"[tool] {exc}")

    def on_close(self):
        self._closing = True
        try:
            if self.session.connected:
                self.session.disconnect()  # stops run / sends EXIT
        finally:
            self.root.destroy()

    # ----------------------------------------------------------- display
    def show_banner(self, text):
        self.banner.config(text=text)
        if not self.banner.winfo_ismapped():
            self.banner.pack(side="top", fill="x", after=self.root.
                             winfo_children()[0])

    def hide_banner(self):
        self.banner.pack_forget()

    def append_log(self, line: str):
        lb = self.logbox
        lb.config(state="normal")
        lb.insert("end", line + "\n")
        if int(lb.index("end-1c").split(".")[0]) > 3000:
            lb.delete("1.0", "500.0")
        lb.see("end")
        lb.config(state="disabled")

    def _refresh_buttons(self):
        conn = self.session.connected
        running = self.session.run is not None and self.session.run.running
        self.connect_btn.config(text="Disconnect" if conn else "Connect")
        self.conn_label.config(text="Connected" if conn else "Disconnected")
        for b, ok in ((self.btn_enter, conn and not running),
                      (self.btn_start, conn and not running),
                      (self.btn_stop, running),
                      (self.btn_exit, conn)):
            b.state(["!disabled"] if ok else ["disabled"])

    def _fill_table(self, expected: dict, observed: dict, tol=None):
        self.table.delete(*self.table.get_children())
        for st in hs.TRANSITION_STATES:
            e = expected.get(st)
            o = observed.get(st)
            if e is None and o is None:
                continue
            d = None if e is None or o is None else o - e
            if tol is None:
                res = ""
            elif e is None:
                res = "UNEXPECTED"
            elif o is None:
                res = "pending"
            else:
                res = "OK" if abs(d) <= tol else ("LATE" if d > 0 else "EARLY")
            fmt = (lambda v: "-" if v is None else f"{v:.2f}")
            self.table.insert("", "end", values=(
                hp.FLIGHT_STATES[st], fmt(e), fmt(o), fmt(d), res))

    def poll(self):
        if self._closing:
            return
        s = self.session
        for _ in range(500):
            try:
                kind, payload = s.events.get_nowait()
            except queue.Empty:
                break
            if kind in ("log", "text"):
                self.append_log(payload)
            elif kind == "warning":
                self.show_banner("WARNING: " + payload)
            elif kind == "run" and payload == "finished":
                run = s.run
                if run and run.result:
                    for line in hs.format_table(run.result).splitlines():
                        self.append_log("[result] " + line)
                    if not run.result["passed"]:
                        self.show_banner("Run FAILED: " + "; ".join(
                            run.result["problems"] or
                            [r["state"] + " " + r["result"]
                             for r in run.result["rows"]
                             if r["result"] != "OK"]))
        v = s.view
        state = ("?" if v.flight_state is None
                 else hp.FLIGHT_STATES[v.flight_state])
        self.state_label.config(text=f"State: {state}")
        status = v.last_status
        stale = s.now() - v.last_status_time > 3.0
        for (c, oval), (_n, fn, kind) in zip(self.lamps, STATUS_LAMPS):
            if status is None or stale:
                color = "#888"
            elif fn(status):
                color = "#1a9c3a" if kind == "good" else "#d02020"
            else:
                color = "#555" if kind == "good" else "#c8c8c8"
            c.itemconfig(oval, fill=color)
        fmt = (lambda x, f: "-" if x is None else format(x, f))
        run = s.run
        sim = (f"sim {run.sim_ms / 1000:7.2f} s  sent {run.frames_sent}"
               f"  late {run.late_ticks}" if run else "")
        self.info_label.config(text=(
            f"sats {fmt(v.gps_sats, 'd')}  lock {'Y' if v.gps_locked else 'N'}"
            f"  baro {fmt(v.baro_alt_m, '.1f')} m  gps {fmt(v.gps_alt_m, '.1f')}"
            f" m  accX {fmt(v.accel_x_g, '.2f')} g  batt "
            f"{fmt(v.battery_mv, 'd')} mV  uptime "
            f"{fmt(None if v.uptime_ms is None else v.uptime_ms / 1000, '.1f')}"
            f" s  echo {v.echo_frames}  nmea {v.nmea_lines}   {sim}"))
        self.bytes_label.config(text=f"rx {v.rx_bytes} / tx {v.tx_bytes} B")
        if run is not None:
            self._fill_table(run.expected, run.observed, run.tolerance_s)
        self._refresh_buttons()
        self.root.after(100, self.poll)

    def redraw_plot(self):
        if self._closing:
            return
        run = self.session.run
        c = self.plot
        c.delete("all")
        w = max(c.winfo_width(), 200)
        h = max(c.winfo_height(), 160)
        if run is not None:
            t_end = max(run.samples[-1].t_ms / 1000 if run.samples else 1, 1)
            half = h // 2
            alts = [a for _t, a in run.plot_true] + \
                [a for _t, a in run.plot_baro]
            accs = [a for _t, a in run.plot_accel] + \
                [a for _t, a in run.plot_wacc]
            self._draw_series(c, 0, half, w, t_end, alts, "Altitude MSL (m)",
                              [(run.plot_true, "#1f5fbf", "sim true"),
                               (run.plot_baro, "#d2691e", "Witness baro")])
            self._draw_series(c, half, h - half, w, t_end, accs,
                              "X accel (g)",
                              [(run.plot_accel, "#1f5fbf", "sim low-g X"),
                               (run.plot_wacc, "#d2691e", "Witness accel X")])
            x = 50 + (w - 60) * min(run.sim_ms / 1000 / t_end, 1)
            c.create_line(x, 0, x, h, fill="#bbb", dash=(2, 2))
        else:
            c.create_text(w / 2, h / 2, text="Start a sim to plot",
                          fill="#888")
        self.root.after(250, self.redraw_plot)

    @staticmethod
    def _draw_series(c, y0, hh, w, t_end, values, title, series):
        left, right, top, bot = 50, w - 10, y0 + 16, y0 + hh - 16
        c.create_text(left, y0 + 8, text=title, anchor="w",
                      font=("Segoe UI", 9, "bold"))
        c.create_rectangle(left, top, right, bot, outline="#ccc")
        if not values:
            return
        lo, hi = min(values), max(values)
        if hi - lo < 1e-6:
            lo, hi = lo - 1, hi + 1
        pad = (hi - lo) * 0.05
        lo, hi = lo - pad, hi + pad
        c.create_text(left - 4, top, text=f"{hi:.0f}" if hi > 50
                      else f"{hi:.1f}", anchor="e", font=("Segoe UI", 8))
        c.create_text(left - 4, bot, text=f"{lo:.0f}" if hi > 50
                      else f"{lo:.1f}", anchor="e", font=("Segoe UI", 8))
        c.create_text(right, bot + 8, text=f"{t_end:.0f} s", anchor="e",
                      font=("Segoe UI", 8))
        lx = left + 150
        for pts, color, label in series:
            c.create_text(lx, y0 + 8, text=label, fill=color, anchor="w",
                          font=("Segoe UI", 9))
            lx += 120
            if len(pts) < 2:
                continue
            step = max(1, len(pts) // 1500)
            coords = []
            for t, v in pts[::step]:
                coords.append(left + (right - left) * min(t / t_end, 1))
                coords.append(bot - (bot - top) * (v - lo) / (hi - lo))
            c.create_line(*coords, fill=color, width=1.5)


def build_parser():
    s = load_settings()
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--headless", action="store_true",
                    help="run one simulation without the GUI")
    ap.add_argument("--host", default=s["host"])
    ap.add_argument("--port", type=int, default=s["port"])
    ap.add_argument("--preset", default="Nominal 10 g")
    ap.add_argument("--set", action="append", metavar="KEY=VALUE",
                    help="override a model parameter (repeatable)")
    ap.add_argument("--tolerance", type=float, default=s["tolerance_s"],
                    help="max |Witness - expected| transition delta, s")
    ap.add_argument("--time-scale", type=int, default=1,
                    help=argparse.SUPPRESS)  # fake-Witness tests only
    ap.add_argument("--settle", type=float, default=1.5,
                    help="seconds to wait for telemetry before starting")
    ap.add_argument("--connect-timeout", type=float, default=5.0)
    ap.add_argument("--runs-dir", default=None)
    ap.add_argument("--no-save", action="store_true")
    ap.add_argument("--list-presets", action="store_true")
    ap.add_argument("-v", "--verbose", action="store_true")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.list_presets:
        for name in sm.PRESETS:
            exp = sm.predict_transitions(sm.generate(sm.preset_params(name)))
            print(f"{name:<24}" + ", ".join(
                f"{hp.FLIGHT_STATES[k]} {v:.2f}s" for k, v in exp.items()))
        return 0
    if args.headless:
        return run_headless(args)
    import tkinter as tk
    root = tk.Tk()
    HitlApp(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
