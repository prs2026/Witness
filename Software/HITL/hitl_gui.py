"""Witness hardware-in-the-loop simulator: Tkinter GUI and headless CLI.

GUI:       python hitl_gui.py
Headless:  python hitl_gui.py --headless --preset "Nominal 10 g" \
               --host prs2026hitl --port 7100
CSV:       python hitl_gui.py --headless --csv trajectories/x.csv

The tool only ever transmits HITL ENTER (0x4849), HITL EXIT (0x4858), Witness
debug start (0x00E0, CSV runs only) and 0x0C HITL sensor frames. See README.md
for the safety rules.
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

import attitude as att  # noqa: E402
import hitl_protocol as hp  # noqa: E402
import hitl_session as hs  # noqa: E402
import sim_model as sm  # noqa: E402
import trajectory_csv as tc  # noqa: E402

SETTINGS_PATH = os.path.join(HERE, "hitl_settings.json")
DEFAULT_HOST = "prs2026hitl"
DEFAULT_PORT = 7100
DEFAULT_TOLERANCE_S = 1.0
TRAJECTORY_DIR = os.path.join(HERE, "trajectories")
CSV_SOURCE = "CSV trajectory"


def load_settings(path: str = SETTINGS_PATH) -> dict:
    settings = {"host": DEFAULT_HOST, "port": DEFAULT_PORT,
                "preset": "Nominal 10 g", "tolerance_s": DEFAULT_TOLERANCE_S,
                "csv_path": "", "att_final_tilt_deg": att.DEFAULT_THRESHOLDS[
                    "final_tilt_max_deg"],
                # FC flash logs fetched through the bench Pi after each run.
                "fetch_fc_logs": True, "pi_host": "", "pi_user": "",
                "pi_key": "", "fetch_restart_hub": True}
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


def parse_overrides(items, cls=sm.SimParams) -> dict:
    """--set key=value pairs -> typed overrides for a params dataclass."""
    types = {f.name: f.type for f in dataclasses.fields(cls)}
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
    if t == "str":
        return raw
    if "bool" in t:
        return raw.lower() in ("1", "true", "yes", "on")
    if "int" in t and "float" not in t:
        return int(float(raw))
    return float(raw)


# ---------------------------------------------------------------------------
# Headless
# ---------------------------------------------------------------------------

def attitude_thresholds(args) -> dict:
    return {"final_tilt_max_deg": args.att_final_tilt,
            "final_att_max_deg": args.att_final_error,
            "flight_tilt_max_deg": args.att_flight_tilt}


def fc_log_config(settings: dict, enabled: bool = True) -> dict:
    """FC log fetch settings; env WITNESS_PI_USER / _KEY / _HOST fill gaps."""
    return {
        "enabled": enabled and bool(settings.get("fetch_fc_logs", True)),
        "pi_host": settings.get("pi_host") or os.environ.get("WITNESS_PI_HOST", ""),
        "pi_user": settings.get("pi_user") or os.environ.get("WITNESS_PI_USER", ""),
        "pi_key": settings.get("pi_key") or os.environ.get("WITNESS_PI_KEY", ""),
        "restart": bool(settings.get("fetch_restart_hub", True)),
    }


def run_headless(args) -> int:
    scale = max(int(args.time_scale), int(args.speed or 1))
    if scale != 1 and not hs.is_loopback(args.host):
        print("--speed is only allowed against the local fake Witness "
              "(127.0.0.1); hardware runs are real time", file=sys.stderr)
        return 2
    if args.csv:
        try:
            params = tc.CsvParams(csv_path=args.csv,
                                  **parse_overrides(args.set, tc.CsvParams))
            label = "CSV " + os.path.basename(args.csv)
        except (OSError, ValueError, TypeError) as exc:
            print(f"bad CSV run arguments: {exc}", file=sys.stderr)
            return 2
    else:
        try:
            params = sm.preset_params(args.preset)
        except KeyError as exc:
            print(exc.args[0], file=sys.stderr)
            return 2
        params = dataclasses.replace(params, **parse_overrides(args.set))
        label = args.preset
    runs_dir = None if args.no_save else (args.runs_dir or hs.DEFAULT_RUNS_DIR)
    session = hs.HitlSession(runs_dir=runs_dir)
    session.fc_log_config = fc_log_config(load_settings(), not args.no_fetch_logs)
    try:
        run = session.prepare_run(params, preset=label,
                                  tolerance_s=args.tolerance,
                                  time_scale=scale,
                                  att_thresholds=attitude_thresholds(args),
                                  att_latency_ms=args.att_latency_ms)
    except (OSError, ValueError) as exc:
        print(f"cannot build run: {exc}", file=sys.stderr)
        return 2
    summary = sm.summarize(run.samples)
    print(f"{'Source' if args.csv else 'Preset'} {label!r}: "
          f"{summary['duration_s']:.1f} s sim, "
          f"apogee {summary['apogee_agl_m']:.0f} m AGL at "
          f"{summary['apogee_t_s']:.1f} s")
    if run.csv_report:
        print(tc.format_report(run.csv_report))
    print("Expected: " + (", ".join(
        f"{hp.FLIGHT_STATES[k]} {v:.2f} s"
        for k, v in sorted(run.expected.items(), key=lambda kv: kv[1]))
        or "no transitions"))
    if args.dry_run:
        return 0
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
        try:
            session.start_run(run, abort_on_no_hitl=True)
        except ValueError as exc:
            print(str(exc), file=sys.stderr)
            session.disconnect()
            return 2
        while not session.wait_run(0.2):
            drain()
            # The FC log fetch closes the connection on purpose after the run.
            if not session.connected and run.streaming:
                break
    except KeyboardInterrupt:
        session.stop_run("interrupted")
    finally:
        session.wait_run(600 if session.fc_log_config.get("enabled") else 5)
        drain()
        session.disconnect(send_exit=False)
    result = run.result or hs.evaluate(run)
    print(hs.format_table(result))
    if run.imu_report:
        print(f"960 Hz IMU: {run.imu_frames_sent} 0x0E frames sent "
              f"({run.imu_frames_sent * 8} samples)")
    for key, title in (("comparison", "Witness 0xE0 (10 Hz) vs truth"),
                       ("comparison_flash", "Flash state packets (50 Hz) vs truth")):
        if result.get(key):
            print(title + ":")
            for phase, d in result[key].items():
                att_txt = (f", attitude {d['attitude_rms_deg']:.2f} deg rms"
                           if d.get("attitude_rms_deg") is not None else "")
                print(f"  {phase:<9} up {d['up_rms_m']:.1f} m rms (max {d['up_max_m']:.1f}), "
                      f"horizontal {d['horizontal_rms_m']:.1f} m rms, "
                      f"speed {d['speed_rms_mps']:.1f} m/s rms{att_txt}")
    fc = result.get("fc_logs")
    if fc:
        print(f"FC logs: {', '.join(fc['files']) if fc.get('ok') else 'failed: ' + str(fc.get('error'))}")
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
    """The HITL bench GUI.

    Standalone (``parent`` is None) it owns ``root``: window title, size and
    close button. Embedded (witnessOS passes a ``parent`` frame) it only
    builds its widgets inside ``parent`` and the host calls ``shutdown()``.
    """

    def __init__(self, root, settings: dict | None = None,
                 settings_path: str = SETTINGS_PATH,
                 runs_dir: str | None = hs.DEFAULT_RUNS_DIR, parent=None):
        import tkinter as tk
        from tkinter import ttk
        from tkinter.scrolledtext import ScrolledText
        self.tk = tk
        self.root = root
        self.standalone = parent is None
        self.container = root if parent is None else parent
        self.settings_path = settings_path
        self.settings = settings or load_settings(settings_path)
        self.session = hs.HitlSession(runs_dir=runs_dir)
        self.session.fc_log_config = fc_log_config(self.settings) \
            if hasattr(self, "settings") else fc_log_config(load_settings())
        self._shown_run = None
        self._closing = False
        self._plot_sig = None
        self._plot_job = None
        if self.standalone:
            root.title("Witness HITL")
            sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
            root.geometry(f"{min(1280, sw - 40)}x{min(860, sh - 80)}")
            root.minsize(560, 380)
            root.protocol("WM_DELETE_WINDOW", self.on_close)
        top = self.container

        # --- connection bar (two rows so the window can be narrow)
        self.bar = bar_box = ttk.Frame(top, padding=4)
        bar_box.pack(side="top", fill="x")
        bar = ttk.Frame(bar_box)
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
        bar2 = ttk.Frame(bar_box)
        bar2.pack(side="top", fill="x", pady=(4, 0))
        self.btn_enter = ttk.Button(bar2, text="Enter HITL",
                                    command=self.enter_hitl)
        self.btn_start = ttk.Button(bar2, text="Start sim",
                                    command=self.start_sim)
        self.btn_stop = ttk.Button(bar2, text="Stop", command=self.stop_sim)
        self.btn_exit = ttk.Button(bar2, text="Exit HITL",
                                   command=self.exit_hitl)
        for b in (self.btn_enter, self.btn_start, self.btn_stop,
                  self.btn_exit):
            b.pack(side="left", padx=2)
        ttk.Label(bar2, text="Tolerance s").pack(side="left", padx=(8, 0))
        self.tol_var = tk.StringVar(value=str(self.settings["tolerance_s"]))
        ttk.Entry(bar2, textvariable=self.tol_var, width=5).pack(side="left")

        self.banner = tk.Label(top, text="", bg="#b00020", fg="white",
                               font=("Segoe UI", 11, "bold"), anchor="w",
                               justify="left", wraplength=600)
        self.banner.bind("<Configure>", lambda e: self.banner.configure(
            wraplength=max(200, e.width - 8)))

        main = ttk.PanedWindow(top, orient="horizontal")
        main.pack(fill="both", expand=True)
        self.main_pane = main

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
        csv_row = ttk.Frame(left)
        csv_row.pack(fill="x", pady=(4, 0))
        ttk.Button(csv_row, text="Load CSV…",
                   command=self.load_csv).pack(side="left")
        self.csv_label = ttk.Label(csv_row, text="(preset source)",
                                   width=26)
        self.csv_label.pack(side="left", padx=4)
        att_row = ttk.Frame(left)
        att_row.pack(fill="x")
        ttk.Label(att_row, text="Final tilt max deg").pack(side="left")
        self.att_tilt_var = tk.StringVar(
            value=str(self.settings.get("att_final_tilt_deg", 5.0)))
        ttk.Entry(att_row, textvariable=self.att_tilt_var,
                  width=6).pack(side="left", padx=4)
        self.csv_path = None
        # Packed bottom-up first so the scrolling parameter list, not the
        # Predict button or its text, gives way when the window is short.
        self.predict_label = ttk.Label(left, text="", justify="left",
                                       wraplength=280)
        self.predict_label.pack(side="bottom", anchor="w", fill="x")
        ttk.Button(left, text="Predict transitions",
                   command=self.predict).pack(side="bottom", fill="x")
        left.bind("<Configure>", lambda e: self.predict_label.configure(
            wraplength=max(120, e.width - 10)))
        pf_outer = ttk.LabelFrame(left, text="Parameters", padding=2)
        pf_outer.pack(fill="both", expand=True, pady=4)
        canvas = tk.Canvas(pf_outer, width=250, height=120,
                           highlightthickness=0)
        sb = ttk.Scrollbar(pf_outer, orient="vertical", command=canvas.yview)
        pf = ttk.Frame(canvas)
        pf.bind("<Configure>", lambda e: canvas.configure(
            scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=pf, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.param_canvas = canvas
        self.param_frame = pf
        self.param_cls = None
        self.param_vars = {}
        self.param_types = {}
        self._build_param_grid(sm.SimParams)

        # --- right: status, plot, table, log
        right = ttk.Frame(main, padding=4)
        main.add(right, weight=1)
        st = ttk.Frame(right)
        st.pack(fill="x")
        self.state_label = tk.Label(st, text="State: ?", width=14,
                                    font=("Segoe UI", 16, "bold"),
                                    anchor="w")
        self.state_label.pack(side="left", anchor="n")
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
        self.info_label = ttk.Label(right, text="", font=("Consolas", 10),
                                    anchor="w", justify="left",
                                    wraplength=600)
        self.info_label.pack(fill="x")
        right.bind("<Configure>", lambda e: self.info_label.configure(
            wraplength=max(200, e.width - 10)))

        # Plot and table/log share a draggable vertical split.
        vsplit = ttk.PanedWindow(right, orient="vertical")
        vsplit.pack(fill="both", expand=True, pady=(4, 0))
        self.plot = tk.Canvas(vsplit, height=220, width=320, bg="white",
                              highlightthickness=0)
        vsplit.add(self.plot, weight=3)
        self.plot.bind("<Configure>", lambda e: self._schedule_plot())

        lower = ttk.Frame(vsplit)
        vsplit.add(lower, weight=2)
        cols = ("state", "expected", "witness", "delta", "result")
        self.table = ttk.Treeview(lower, columns=cols, show="headings",
                                  height=5)
        for c, w in zip(cols, (80, 70, 70, 60, 80)):
            self.table.heading(c, text=c.capitalize())
            self.table.column(c, width=w, minwidth=50, anchor="center",
                              stretch=False)
        self.table.pack(side="left", fill="y")
        self.logbox = ScrolledText(lower, height=6, width=24,
                                   font=("Consolas", 9), state="disabled")
        self.logbox.pack(side="left", fill="both", expand=True, padx=(4, 0))

        self.load_preset()
        self._refresh_buttons()
        self._poll_id = self.root.after(100, self.poll)
        self._redraw_id = self.root.after(250, self.redraw_plot)

    # ------------------------------------------------------------ params
    def _build_param_grid(self, cls):
        """(Re)build the parameter entries for SimParams or CsvParams."""
        if self.param_cls is cls:
            return
        tk = self.tk
        from tkinter import ttk
        for child in self.param_frame.winfo_children():
            child.destroy()
        self.param_cls = cls
        self.param_vars = {}
        self.param_types = {}
        r = 0
        for f in dataclasses.fields(cls):
            if f.name == "csv_path":
                continue
            ttk.Label(self.param_frame, text=f.name).grid(row=r, column=0,
                                                          sticky="w")
            t = str(f.type)
            self.param_types[f.name] = t
            if "bool" in t:
                var = tk.BooleanVar()
                ttk.Checkbutton(self.param_frame, variable=var).grid(
                    row=r, column=1, sticky="w")
            elif f.name == "after_apogee_attitude":
                var = tk.StringVar()
                ttk.Combobox(self.param_frame, textvariable=var,
                             values=tc.ATTITUDE_MODES, state="readonly",
                             width=9).grid(row=r, column=1, sticky="w")
            else:
                var = tk.StringVar()
                ttk.Entry(self.param_frame, textvariable=var, width=10).grid(
                    row=r, column=1, sticky="w")
            self.param_vars[f.name] = var
            r += 1

    def _fill_params(self, values: dict):
        for name, value in values.items():
            var = self.param_vars.get(name)
            if var is None:
                continue
            if "bool" in self.param_types[name]:
                var.set(bool(value))
            else:
                var.set("" if value is None else str(value))

    def load_preset(self):
        self.csv_path = None
        self.csv_label.config(text="(preset source)")
        self._build_param_grid(sm.SimParams)
        p = sm.preset_params(self.preset_var.get())
        self._fill_params(p.to_dict())
        self.predict()

    def load_csv(self, path: str | None = None):
        """Switch the source to a trajectory CSV (file dialog if no path)."""
        if path is None:
            from tkinter import filedialog
            last = self.settings.get("csv_path") or ""
            path = filedialog.askopenfilename(
                title="Load trajectory CSV",
                initialdir=os.path.dirname(last) if last else TRAJECTORY_DIR,
                filetypes=(("CSV", "*.csv"), ("All files", "*.*")))
            if not path:
                return
        was_csv = self.param_cls is tc.CsvParams
        keep = {}
        if was_csv:
            try:
                keep = self.read_params().to_dict()
            except ValueError:
                keep = {}
        self._build_param_grid(tc.CsvParams)
        self._fill_params(keep or tc.CsvParams().to_dict())
        self.csv_path = path
        self.csv_label.config(text=os.path.basename(path))
        self.settings["csv_path"] = path
        save_settings(self.settings, self.settings_path)
        self.predict()

    def read_params(self):
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
        if self.param_cls is tc.CsvParams:
            return tc.CsvParams(csv_path=self.csv_path or "", **values)
        return sm.SimParams(**values)

    def _att_thresholds(self) -> dict:
        raw = self.att_tilt_var.get().strip()
        th = {"final_tilt_max_deg": None if raw.lower() in ("", "none")
              else float(raw)}
        return th

    def predict(self):
        try:
            params = self.read_params()
        except ValueError as exc:
            self.predict_label.config(text=str(exc))
            return None
        report = ""
        if isinstance(params, tc.CsvParams):
            try:
                build = tc.build(params)
            except (OSError, ValueError) as exc:
                self.predict_label.config(text=f"CSV error: {exc}")
                return None
            samples = build.samples
            exp = hs.expected_from_track(samples,
                                         sm.predict_state_track(samples))
            report = "\n" + tc.format_report(build.report)
        else:
            samples = sm.generate(params)
            exp = sm.predict_transitions(samples)
        s = sm.summarize(samples)
        text = (f"Duration {s['duration_s']:.1f} s, apogee "
                f"{s['apogee_agl_m']:.0f} m AGL at {s['apogee_t_s']:.1f} s\n"
                + "\n".join(f"{hp.FLIGHT_STATES[k]}: {v:.2f} s"
                            for k, v in sorted(exp.items(),
                                               key=lambda kv: kv[1]))
                + ("" if exp else "No transitions expected") + report)
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
            th = self._att_thresholds()
        except ValueError as exc:
            self.append_log(f"[tool] {exc}")
            return
        self.settings.update(preset=self.preset_var.get(), tolerance_s=tol,
                             att_final_tilt_deg=th["final_tilt_max_deg"])
        save_settings(self.settings, self.settings_path)
        label = (CSV_SOURCE + " " + os.path.basename(self.csv_path)
                 if self.csv_path else self.preset_var.get())
        try:
            run = self.session.prepare_run(params, preset=label,
                                           tolerance_s=tol,
                                           att_thresholds=th)
        except (OSError, ValueError) as exc:
            self.append_log(f"[tool] cannot build run: {exc}")
            return
        if run.csv_report:
            for line in tc.format_report(run.csv_report).splitlines():
                self.append_log("[csv] " + line)
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

    def shutdown(self):
        """Stop timers and disconnect (stops any run / sends EXIT)."""
        self._closing = True
        for job in (self._poll_id, self._redraw_id, self._plot_job):
            if job is not None:
                try:
                    self.root.after_cancel(job)
                except Exception:
                    pass
        if self.session.connected:
            self.session.disconnect()

    def on_close(self):
        try:
            self.shutdown()
        finally:
            if self.standalone:
                self.root.destroy()

    # ----------------------------------------------------------- display
    def show_banner(self, text):
        self.banner.config(text=text)
        if not self.banner.winfo_ismapped():
            self.banner.pack(side="top", fill="x", after=self.bar)

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
        self._poll_id = self.root.after(100, self.poll)

    def _schedule_plot(self):
        """Redraw soon after a resize (debounced), independent of the timer."""
        if self._closing:
            return
        if self._plot_job is not None:
            self.root.after_cancel(self._plot_job)
        self._plot_job = self.root.after(60, self._redraw_now)

    def _redraw_now(self):
        self._plot_job = None
        self._plot_sig = None
        self._draw_plot()

    def redraw_plot(self):
        """Periodic redraw; skipped while hidden or when nothing changed."""
        if self._closing:
            return
        try:
            self._draw_plot()
        finally:
            self._redraw_id = self.root.after(250, self.redraw_plot)

    def _draw_plot(self):
        c = self.plot
        if self._closing or not c.winfo_ismapped():
            return
        run = self.session.run
        w = max(c.winfo_width(), 120)
        h = max(c.winfo_height(), 80)
        sig = (w, h, id(run)) + (() if run is None else (
            run.sim_ms, len(run.plot_baro), len(run.plot_wacc),
            len(run.plot_tilt_w), len(run.plot_tilt_err),
            len(run.plot_att_err)))
        if sig == self._plot_sig:
            return
        self._plot_sig = sig
        c.delete("all")
        if run is None:
            c.create_text(w / 2, h / 2, text="Start a sim to plot",
                          fill="#888")
            return
        t_end = max(run.samples[-1].t_ms / 1000 if run.samples else 1, 1)
        panels = 3 if run.truth_quats is not None else 2
        ph = h / panels
        now = run.sim_ms / 1000
        specs = [
            ("Altitude MSL (m)",
             [(run.plot_true, "#1f5fbf", "sim true"),
              (run.plot_baro, "#d2691e", "Witness baro")]),
            ("X accel (g)",
             [(run.plot_accel, "#1f5fbf", "sim low-g X"),
              (run.plot_wacc, "#d2691e", "Witness accel X")]),
        ]
        if panels == 3:
            truth = [pt for pt in run.plot_tilt_truth if pt[0] <= now]
            title = "Tilt / attitude error (deg)"
            if run.att_records and not run.plot_tilt_w:
                title += " - Witness attitude not available (all zero)"
            specs.append((title, [(truth, "#1f5fbf", "truth tilt"),
                                  (run.plot_tilt_w, "#d2691e", "Witness tilt"),
                                  (run.plot_tilt_err, "#2e8b57", "tilt err"),
                                  (run.plot_att_err, "#b00020", "att err")]))
        for i, (title, series) in enumerate(specs):
            y0 = round(i * ph)
            y1 = round((i + 1) * ph)
            self._draw_series(c, y0, y1 - y0, w, t_end, title, series,
                              cursor_t=min(now, t_end))

    @staticmethod
    def _draw_series(c, y0, hh, w, t_end, title, series, cursor_t=None):
        """One panel: title + wrapped legend, nice y/x ticks, clipped data."""
        import tkinter.font as tkfont
        font = tkfont.nametofont("TkDefaultFont")
        ls = font.metrics("linespace")
        values = [v for pts, _c, _l in series for _t, v in pts]
        if values:
            lo, hi = min(values), max(values)
        else:
            lo, hi = 0.0, 1.0
        if hi - lo < 1e-6:
            lo, hi = lo - 1, hi + 1
        pad = (hi - lo) * 0.05
        lo, hi = lo - pad, hi + pad

        # Header: title then legend entries, wrapping onto extra lines.
        x, line = 6, 0
        items = [(title, "#222", True)] + [(lab, col, False)
                                          for _p, col, lab in series]
        placed = []
        for text, color, bold in items:
            tw = font.measure(text) + (14 if not bold else 6)
            if x + tw > w - 6 and x > 6:
                x, line = 6, line + 1
            placed.append((x, line, text, color, bold))
            x += tw + 10
        header_h = (line + 1) * ls + 4
        plot_h_est = hh - header_h - ls - 8
        y_ticks = _nice_ticks(lo, hi, max(2, int(plot_h_est // (ls * 1.8))))
        lo, hi = min(lo, y_ticks[0]), max(hi, y_ticks[-1])
        y_step = y_ticks[1] - y_ticks[0] if len(y_ticks) > 1 else 1.0
        labels = [_tick_label(v, y_step) for v in y_ticks]
        left = max(36, max(font.measure(s) for s in labels) + 10)
        right = w - 10
        top = y0 + header_h
        bot = y0 + hh - ls - 6
        for px, ln, text, color, bold in placed:
            ty = y0 + 2 + ln * ls + ls / 2
            if bold:
                c.create_text(px, ty, text=text, anchor="w", fill=color,
                              font=("Segoe UI", 9, "bold"))
            else:
                c.create_line(px, ty, px + 10, ty, fill=color, width=2)
                c.create_text(px + 14, ty, text=text, anchor="w", fill=color,
                              font=("Segoe UI", 9))
        if bot - top < 12 or right - left < 30:
            return  # too small to plot legibly; header still shown
        c.create_rectangle(left, top, right, bot, outline="#bbb")

        def ys(v):
            return bot - (bot - top) * (v - lo) / (hi - lo)

        def xs(t):
            return left + (right - left) * min(max(t / t_end, 0.0), 1.0)
        last_label_y = None
        for v, s in zip(y_ticks, labels):
            y = ys(v)
            if y < top - 0.5 or y > bot + 0.5:
                continue
            c.create_line(left, y, right, y, fill="#eee")
            if last_label_y is None or abs(last_label_y - y) >= ls:
                c.create_text(left - 4, y, text=s, anchor="e",
                              font=("Segoe UI", 8), fill="#555")
                last_label_y = y
        x_ticks = [t for t in _nice_ticks(
            0, t_end, max(2, int((right - left) // 70))) if 0 <= t <= t_end]
        x_step = x_ticks[1] - x_ticks[0] if len(x_ticks) > 1 else t_end
        last_right = -1e9
        for t in x_ticks:
            x = xs(t)
            c.create_line(x, top, x, bot, fill="#f2f2f2")
            s = _tick_label(t, x_step) + (" s" if t == x_ticks[-1] else "")
            half = font.measure(s) / 2
            if x - half > last_right + 6 and x + half <= w:
                c.create_text(x, bot + 2, text=s, anchor="n",
                              font=("Segoe UI", 8), fill="#555")
                last_right = x + half
        for pts, color, _label in series:
            if len(pts) < 2:
                continue
            step = max(1, len(pts) // max(200, 2 * int(right - left)))
            coords = []
            for t, v in pts[::step]:
                coords.append(xs(t))
                coords.append(min(max(ys(v), top), bot))
            c.create_line(*coords, fill=color, width=1.5)
        if cursor_t is not None:
            x = xs(cursor_t)
            c.create_line(x, top, x, bot, fill="#999", dash=(2, 2))


def _nice_ticks(lo: float, hi: float, target: int = 5) -> list:
    """Round tick values covering [lo, hi] with about ``target`` intervals."""
    import math
    span = hi - lo
    if not math.isfinite(span) or span <= 0:
        return [lo, lo + 1.0]
    rough = span / max(1, target)
    mag = 10.0 ** math.floor(math.log10(rough))
    step = next(m for m in (1, 2, 5, 10) if rough / mag <= m) * mag
    i0, i1 = math.floor(lo / step + 1e-9), math.ceil(hi / step - 1e-9)
    if i1 - i0 > 200:
        i1 = i0 + 200
    return [i * step for i in range(i0, i1 + 1)]


def _tick_label(v: float, step: float) -> str:
    import math
    if abs(v) < step * 1e-6:
        return "0"
    if step >= 1:
        return f"{v:.0f}"
    decimals = min(6, max(0, -math.floor(math.log10(step))))
    return f"{v:.{decimals}f}"


def build_parser():
    s = load_settings()
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--headless", action="store_true",
                    help="run one simulation without the GUI")
    ap.add_argument("--host", default=s["host"])
    ap.add_argument("--port", type=int, default=s["port"])
    ap.add_argument("--preset", default="Nominal 10 g")
    ap.add_argument("--csv", metavar="PATH",
                    help="drive the run from a trajectory CSV instead of a "
                    "preset (--set then applies to CSV parameters)")
    ap.add_argument("--set", action="append", metavar="KEY=VALUE",
                    help="override a model parameter (repeatable)")
    ap.add_argument("--tolerance", type=float, default=s["tolerance_s"],
                    help="max |Witness - expected| transition delta, s")
    ap.add_argument("--time-scale", type=int, default=1,
                    help=argparse.SUPPRESS)  # fake-Witness tests only
    ap.add_argument("--speed", type=int, default=1, metavar="N",
                    help="send N sim ticks per 10 ms; ONLY for the local fake "
                    "Witness (refused for non-loopback hosts)")
    ap.add_argument("--dry-run", action="store_true",
                    help="build the frames, print the prediction and exit "
                    "without connecting")
    ap.add_argument("--att-final-tilt", type=float,
                    default=att.DEFAULT_THRESHOLDS["final_tilt_max_deg"],
                    metavar="DEG", help="max final tilt error (CSV runs)")
    ap.add_argument("--att-final-error", type=float, default=None,
                    metavar="DEG", help="max final attitude error with the "
                    "heading offset removed (CSV runs; default off)")
    ap.add_argument("--att-flight-tilt", type=float, default=None,
                    metavar="DEG", help="max tilt error during Boost..Descent "
                    "(CSV runs; default off)")
    ap.add_argument("--att-latency-ms", type=int, default=0,
                    help="compare Witness attitude with the truth this many "
                    "ms before the last frame sent")
    ap.add_argument("--settle", type=float, default=1.5,
                    help="seconds to wait for telemetry before starting")
    ap.add_argument("--connect-timeout", type=float, default=5.0)
    ap.add_argument("--runs-dir", default=None)
    ap.add_argument("--no-save", action="store_true")
    ap.add_argument("--no-fetch-logs", action="store_true",
                    help="do not download the FC flash logs through the bench Pi "
                         "after the run (default: fetch, then power-cycle Pi USB hub 2)")
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
