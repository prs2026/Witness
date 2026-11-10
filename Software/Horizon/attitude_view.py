"""Horizon Attitude tab: 3D wireframe rocket, readouts and strip chart.

Live input is the Witness decoded feed (the records Flight View receives);
0xE0 debug quaternions are preferred while fresh, else the 1 Hz 0x02 state
quaternion. Offline, load a converter.py CSV or a HITL/generic attitude CSV
(see attitude_data.load_attitude_csv) and replay it with play/pause/scrub.
Math and conventions live in attitude_math.py. Drawing is plain Tk canvas.
"""

import math
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import attitude_math as am
from attitude_data import AttitudeTrack, LiveAttitude, load_attitude_csv, state_name

BG = "#ffffff"
TEXT = "#171717"
MUTED = "#555555"
GRID = "#d0d0d0"
BAD = "#990000"
WARN = "#704000"
OK = "#0a5a0a"
ROCKET_NEAR = "#1a1a1a"
ROCKET_FAR = "#8a8a8a"
MARK = "#c04000"
GHOST = "#2f6fb0"
AXIS_COLORS = ("#b00020", "#007a33", "#0040c0")   # body x, y, z
Q_COLORS = ("#b00020", "#007a33", "#0040c0", "#6a3d9a")
STALE_S = 3.0
LIVE_WINDOWS = (30, 60, 120, 300, 600)
SPEEDS = ("0.25", "0.5", "1", "2", "5", "10")
VIEWS = {  # compass azimuth / elevation of the camera position
    "Iso": (135.0, math.degrees(math.asin(1 / math.sqrt(3)))),
    "From East": (90.0, 0.0),
    "From North": (0.0, 0.0),
    "From Above": (0.0, 90.0),
}


CONVENTIONS = (
    "Quaternion: Hamilton, x,y,z,w (scalar last), rotates BODY into WORLD. "
    "WORLD = ENU (+X east, +Y north, +Z up). BODY +X = nose; upright on the pad "
    "body +X points up. 0,0,0,0 = attitude not available.\n\n"
    "Tilt from vertical: angle between the nose and world up.\n"
    "Nose azimuth: compass direction of the nose, 0 = north, 90 = east "
    "(undefined when vertical).\n"
    "Spin about nose: twist about the nose, right-handed, 0 for the pad's "
    "shortest-arc alignment (body +Y north, +Z west).\n"
    "ZYX yaw/pitch/roll: intrinsic Z-Y'-X'' of body relative to ENU, each "
    "right-handed. Yaw from east toward north; pitch > 0 = nose down; upright = "
    "pitch -90 (gimbal lock, roll folded into yaw).\n\n"
    "Heading is unobservable on Witness (no magnetometer): azimuth, spin and yaw "
    "include an arbitrary heading set at initialisation. Truth comparison removes "
    "a constant heading offset (auto at the first valid pair, or Align heading here).\n\n"
    "Orange fin and stripe = body +Y. Body triad xb/yb/zb. Truth = blue dashed ghost."
)


def rocket_segments():
    """Wireframe in BODY coordinates (+X nose).

    Returns ``(segments, fins)``: segments are ``(p1, p2, role)`` lines and fins
    are ``(corner_points, role)`` polygons.
    """
    segments, fins = [], []
    radius, tail, shoulder, tip = 0.07, -0.6, 0.32, 0.65
    n = 12
    ring = [(math.cos(2 * math.pi * i / n), math.sin(2 * math.pi * i / n)) for i in range(n)]
    for x in (tail, shoulder):
        for (c1, s1), (c2, s2) in zip(ring, ring[1:] + ring[:1]):
            segments.append(((x, radius * c1, radius * s1), (x, radius * c2, radius * s2), "body"))
    for c, s in ring[1::3]:
        segments.append(((tail, radius * c, radius * s), (shoulder, radius * c, radius * s), "body"))
    for c, s in ring[::2]:
        segments.append(((shoulder, radius * c, radius * s), (tip, 0.0, 0.0), "nose"))
    # Four fins on body +Y, +Z, -Y, -Z. The +Y fin and a +Y stripe are marked
    # so roll is visible.
    for index, (dy, dz) in enumerate(((1, 0), (0, 1), (-1, 0), (0, -1))):
        role = "mark" if index == 0 else "fin"
        span = 0.2
        outline = [(tail, radius), (tail + 0.26, radius), (tail + 0.1, radius + span), (tail - 0.03, radius + span)]
        points = [(x, dy * r, dz * r) for x, r in outline]
        fins.append((points, role))
    segments.append(((tail, radius, 0.0), (shoulder, radius, 0.0), "mark"))
    return segments, fins


ROCKET, FINS = rocket_segments()
FIN_FILL = {"fin": "#c8c8c8", "mark": "#f2a36b"}


def fmt(value, digits=1, unit="°"):
    if value is None or not isinstance(value, (int, float)) or not math.isfinite(value):
        return "--"
    return f"{value:.{digits}f}{unit}"


def make_screen(canvas, view_az, view_el, *, fill=0.42, minimum=200):
    """Projection for a canvas: returns (screen(p) -> (x, y, depth), w, h, cx, cy)."""
    w = max(canvas.winfo_width(), minimum)
    h = max(canvas.winfo_height(), minimum)
    scale = min(w, h) * fill
    cx, cy = w / 2, h / 2
    basis = am.camera_basis(view_az, view_el)

    def screen(p):
        sx, sy, depth = am.project(p, basis)
        return cx + scale * sx, cy - scale * sy, depth
    return screen, w, h, cx, cy


def draw_world(canvas, screen, ground=-0.75):
    """Ground grid (0.25 spacing) and world ENU axes."""
    def line(a, b, **kw):
        (x1, y1, _), (x2, y2, _) = screen(a), screen(b)
        canvas.create_line(x1, y1, x2, y2, **kw)
    for v in [i * 0.25 - 1.0 for i in range(9)]:
        line((v, -1, ground), (v, 1, ground), fill=GRID)
        line((-1, v, ground), (1, v, ground), fill=GRID)
    for axis, label in (((1, 0, 0), "E"), ((0, 1, 0), "N"), ((0, 0, 1), "U")):
        line((0, 0, 0), tuple(0.95 * a for a in axis), fill="#909090", width=1, arrow="last")
        x, y, _ = screen(tuple(1.05 * a for a in axis))
        canvas.create_text(x, y, text=label, fill=MUTED, font=("Segoe UI", 10, "bold"))


def draw_body_axes(canvas, screen, q):
    """Body triad xb (nose) / yb / zb rotated by q."""
    for axis, color, label, length in zip(((1, 0, 0), (0, 1, 0), (0, 0, 1)), AXIS_COLORS,
                                          ("xb", "yb", "zb"), (0.85, 0.45, 0.45)):
        x1, y1, _ = screen((0, 0, 0))
        x2, y2, _ = screen(am.rotate(q, tuple(length * a for a in axis)))
        canvas.create_line(x1, y1, x2, y2, fill=color, width=3, arrow="last")
        lx, ly, _ = screen(am.rotate(q, tuple((length + 0.08) * a for a in axis)))
        canvas.create_text(lx, ly, text=label, fill=color, font=("Segoe UI", 9, "bold"))


def draw_rocket(canvas, screen, q, ghost=False):
    """Depth-sorted wireframe rocket; screen(p) -> (x, y, depth)."""
    items = []
    for a, b, role in ROCKET:
        pa, pb = screen(am.rotate(q, a)), screen(am.rotate(q, b))
        items.append(((pa[2] + pb[2]) / 2, pa, pb, role))
    for points, role in FINS:
        projected = [screen(am.rotate(q, p)) for p in points]
        items.append((sum(p[2] for p in projected) / len(projected), projected, None, role))
    items.sort(key=lambda item: item[0])
    depths = [item[0] for item in items]
    mid = (depths[0] + depths[-1]) / 2 if depths else 0.0
    for depth, pa, pb, role in items:
        if pb is None:   # fin polygon
            coords = [v for p in pa for v in p[:2]]
            if ghost:
                canvas.create_polygon(*coords, fill="", outline=GHOST, dash=(4, 3))
            else:
                canvas.create_polygon(*coords, fill=FIN_FILL[role],
                                      outline=MARK if role == "mark" else ROCKET_NEAR, width=1.5)
            continue
        if ghost:
            kw = {"fill": GHOST, "width": 1, "dash": (4, 3)}
        elif role == "mark":
            kw = {"fill": MARK, "width": 3}
        else:
            kw = {"fill": ROCKET_NEAR if depth >= mid else ROCKET_FAR,
                  "width": 2 if role in ("fin", "nose") else 1.5}
        canvas.create_line(pa[0], pa[1], pb[0], pb[1], capstyle="round", **kw)


class AttitudeTab(ttk.Frame):
    """Attitude visualization; feed it with accept_record() from the Witness tab."""

    def __init__(self, parent, *, refresh_ms=100):
        super().__init__(parent, padding=6)
        self.refresh_ms = refresh_ms
        self.live = LiveAttitude()
        self.track = None
        self.mode = "live"
        self.play_time = 0.0
        self.playing = False
        self._last_tick = time.monotonic()
        self.manual_offset = None
        self.view_az, self.view_el = VIEWS["Iso"]
        self._drag = None
        self._scrubbing = False
        self.last_sample = None
        self._build()
        self._after_id = self.after(self.refresh_ms, self._tick)

    # ------------------------------------------------------------- layout
    def _build(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)
        bar = ttk.Frame(self)
        bar.grid(row=0, column=0, columnspan=2, sticky="ew")
        self.live_button = ttk.Button(bar, text="Live feed", command=self.use_live)
        self.live_button.pack(side="left")
        ttk.Button(bar, text="Load CSV...", command=self.choose_file).pack(side="left", padx=(4, 10))
        self.show_truth = tk.BooleanVar(value=True)
        self.align = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="Truth ghost", variable=self.show_truth,
                        command=self.refresh).pack(side="left", padx=2)
        ttk.Checkbutton(bar, text="Heading-align estimate", variable=self.align,
                        command=self.refresh).pack(side="left", padx=2)
        ttk.Button(bar, text="Align heading here", command=self.align_here).pack(side="left", padx=2)
        ttk.Button(bar, text="Auto align", command=self.auto_align).pack(side="left", padx=2)
        ttk.Button(bar, text="Clear", command=self.clear).pack(side="left", padx=(10, 2))
        ttk.Label(bar, text="Live window s").pack(side="left", padx=(10, 2))
        self.window = tk.StringVar(value="60")
        ttk.Combobox(bar, textvariable=self.window, values=[str(v) for v in LIVE_WINDOWS],
                     width=5, state="readonly").pack(side="left")
        views = ttk.Frame(self)
        views.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(3, 0))
        ttk.Label(views, text="Camera:").pack(side="left")
        for name in VIEWS:
            ttk.Button(views, text=name, width=len(name) + 1,
                       command=lambda name=name: self.set_view(name)).pack(side="left", padx=1)
        ttk.Label(views, text="(drag the view to orbit)", foreground=MUTED).pack(side="left", padx=6)
        ttk.Button(views, text="Conventions...", command=self.show_conventions).pack(side="right")

        self.canvas = tk.Canvas(self, bg=BG, highlightthickness=1, highlightbackground="#999999")
        self.canvas.grid(row=2, column=0, sticky="nsew", pady=4)
        self.canvas.bind("<ButtonPress-1>", self._drag_start)
        self.canvas.bind("<B1-Motion>", self._drag_move)

        panel = ttk.Frame(self, padding=(8, 0, 0, 0))
        panel.grid(row=2, column=1, sticky="ns", pady=4)
        self.values = {}
        self.value_labels = {}
        rows = (("mode", "Mode"), ("source", "Source"), ("age", "Sample age"),
                ("state", "Flight state"), ("hitl", "HITL"), ("gps", "GPS lock"), None,
                ("qx", "q x"), ("qy", "q y"), ("qz", "q z"), ("qw", "q w"), ("norm", "|q|"), None,
                ("tilt", "Tilt from vertical"), ("azimuth", "Nose azimuth"), ("spin", "Spin about nose"), None,
                ("yaw", "ZYX yaw"), ("pitch", "ZYX pitch"), ("roll", "ZYX roll"), None,
                ("truth_tilt", "Truth tilt"), ("tilt_err", "Tilt error"), ("att_err", "Attitude error"),
                ("offset", "Heading offset"), None, ("note", "Note"))
        mono = ("Consolas", 10)
        r = 0
        for item in rows:
            if item is None:
                ttk.Separator(panel).grid(row=r, column=0, columnspan=2, sticky="ew", pady=2)
                r += 1
                continue
            key, title = item
            ttk.Label(panel, text=title).grid(row=r, column=0, sticky="nw", padx=(0, 6))
            var = tk.StringVar(value="--")
            label = tk.Label(panel, textvariable=var, font=mono, anchor="w", justify="left",
                             fg=TEXT, width=24, wraplength=220)
            label.grid(row=r, column=1, sticky="w")
            self.values[key], self.value_labels[key] = var, label
            r += 1

        self.chart = tk.Canvas(self, bg=BG, height=160, highlightthickness=1, highlightbackground="#999999")
        self.chart.grid(row=3, column=0, columnspan=2, sticky="ew")

        replay = ttk.Frame(self)
        replay.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(4, 0))
        replay.columnconfigure(3, weight=1)
        self.play_button = ttk.Button(replay, text="Play", width=7, command=self.toggle_play)
        self.play_button.grid(row=0, column=0)
        ttk.Label(replay, text="Speed").grid(row=0, column=1, padx=(8, 2))
        self.speed = tk.StringVar(value="1")
        ttk.Combobox(replay, textvariable=self.speed, values=SPEEDS, width=5,
                     state="readonly").grid(row=0, column=2)
        self.scrub = tk.DoubleVar(value=0.0)
        self.scale = ttk.Scale(replay, from_=0.0, to=1.0, variable=self.scrub, command=self._scrubbed)
        self.scale.grid(row=0, column=3, sticky="ew", padx=8)
        self.time_label = ttk.Label(replay, text="live", width=34)
        self.time_label.grid(row=0, column=4)
        self._set_replay_enabled(False)

    def show_conventions(self):
        messagebox.showinfo("Attitude conventions", CONVENTIONS, parent=self)

    def _set_replay_enabled(self, enabled):
        state = "!disabled" if enabled else "disabled"
        self.play_button.state([state])
        self.scale.state([state])

    # ----------------------------------------------------------- inputs
    def accept_record(self, record):
        """Witness decoded-feed record. Never raises into the receiver loop."""
        try:
            return self.live.accept_record(record)
        except (TypeError, ValueError, KeyError, AttributeError):
            return False

    def choose_file(self):
        path = filedialog.askopenfilename(parent=self, title="Load attitude CSV",
                                          filetypes=(("CSV", "*.csv"), ("All files", "*.*")))
        if not path:
            return
        try:
            self.load_file(path)
        except (OSError, ValueError, UnicodeDecodeError) as exc:
            messagebox.showerror("Attitude CSV", str(exc), parent=self)

    def load_file(self, path):
        self.set_track(load_attitude_csv(path))
        return self.track

    def set_track(self, track):
        if not isinstance(track, AttitudeTrack) or not len(track):
            raise ValueError("empty attitude track")
        self.track = track
        self.mode = "file"
        self.manual_offset = None
        self.playing = False
        self.play_button.configure(text="Play")
        self.play_time = track.start
        self.scale.configure(from_=track.start, to=max(track.end, track.start + 1e-6))
        self.scrub.set(track.start)
        self._set_replay_enabled(True)
        self.refresh()

    def use_live(self):
        self.mode = "live"
        self.playing = False
        self.play_button.configure(text="Play")
        self._set_replay_enabled(False)
        self.refresh()

    def clear(self):
        if self.mode == "live":
            self.live.clear()
        self.manual_offset = None
        self.refresh()

    def set_view(self, name):
        self.view_az, self.view_el = VIEWS[name]
        self.refresh()

    def _drag_start(self, event):
        self._drag = (event.x, event.y, self.view_az, self.view_el)

    def _drag_move(self, event):
        if self._drag is None:
            return
        x0, y0, az, el = self._drag
        self.view_az = (az - (event.x - x0) * 0.5) % 360.0
        self.view_el = max(-89.0, min(90.0, el + (event.y - y0) * 0.5))
        self.refresh()

    # ----------------------------------------------------------- replay
    def toggle_play(self):
        if self.mode != "file" or self.track is None:
            return
        if not self.playing and self.play_time >= self.track.end:
            self.play_time = self.track.start
        self.playing = not self.playing
        self._last_tick = time.monotonic()
        self.play_button.configure(text="Pause" if self.playing else "Play")

    def _scrubbed(self, value):
        if self.mode != "file" or self._scrubbing:
            return
        self.play_time = float(value)
        self.refresh()

    def seek(self, t):
        if self.track is None:
            return
        self.play_time = max(self.track.start, min(self.track.end, float(t)))
        self._scrubbing = True
        try:
            self.scrub.set(self.play_time)
        finally:
            self._scrubbing = False
        self.refresh()

    def _tick(self):
        now = time.monotonic()
        try:
            if self.playing and self.track is not None:
                try:
                    speed = float(self.speed.get())
                except ValueError:
                    speed = 1.0
                self.play_time += (now - self._last_tick) * speed
                if self.play_time >= self.track.end:
                    self.play_time = self.track.end
                    self.playing = False
                    self.play_button.configure(text="Play")
                self.seek(self.play_time)
            elif self.winfo_ismapped():
                self.refresh()
        finally:
            self._last_tick = now
            self._after_id = self.after(self.refresh_ms, self._tick)

    def close(self):
        if self._after_id is not None:
            try:
                self.after_cancel(self._after_id)
            except tk.TclError:
                pass
            self._after_id = None

    # ---------------------------------------------------------- heading
    def heading_offset(self):
        """(offset_deg or None, how) for the loaded truth comparison."""
        if self.track is None or not self.track.has_truth:
            return None, ""
        if self.manual_offset is not None:
            return self.manual_offset, "manual"
        offset, at = self.track.first_heading_offset()
        return offset, ("" if at is None else f"auto @ {at:.2f} s")

    def align_here(self):
        sample = self.current_sample()
        if sample is None or sample.q is None or sample.truth is None:
            self.values["note"].set("Align needs valid estimate and truth here")
            return
        self.manual_offset = am.heading_offset_deg(sample.q, sample.truth)
        self.refresh()

    def auto_align(self):
        self.manual_offset = None
        self.refresh()

    # ---------------------------------------------------------- refresh
    def current_sample(self, now=None):
        if self.mode == "file" and self.track is not None:
            return self.track.sample_at(self.play_time)
        return self.live.current(now)

    def refresh(self, now=None):
        now = time.monotonic() if now is None else now
        sample = self.current_sample(now)
        self.last_sample = sample
        offset, how = self.heading_offset()
        self._readouts(sample, now, offset, how)
        self._draw_scene(sample, offset)
        self._draw_chart(now)
        if self.mode == "file" and self.track is not None:
            self.time_label.configure(text=f"t = {self.play_time:.2f} s  [{self.track.start:.2f} .. "
                                           f"{self.track.end:.2f}]")
        else:
            self.time_label.configure(text="live")

    def _set(self, key, text, color=TEXT):
        self.values[key].set(text)
        self.value_labels[key].configure(fg=color)

    def _readouts(self, s, now, offset, how):
        if self.mode == "file" and self.track is not None:
            mode = f"File: {self.track.name}"
        else:
            mode = "Live (Witness feed)"
        self._set("mode", mode)
        for key in ("qx", "qy", "qz", "qw", "norm", "tilt", "azimuth", "spin", "yaw", "pitch", "roll",
                    "truth_tilt", "tilt_err", "att_err", "offset", "note"):
            self._set(key, "--")
        if s is None:
            self._set("source", "--")
            self._set("age", "no attitude samples" if self.mode == "live" else "--")
            self._set("state", "--")
            self._set("hitl", "--")
            self._set("gps", "--")
            self._set("note", "Waiting for Witness 0x02/0xE0" if self.mode == "live" else "--")
            return
        device = f" ({s.device})" if s.device else ""
        self._set("source", f"{s.source}{device}" + (" interp" if s.interpolated else ""))
        if self.mode == "live":
            age = now - s.time
            text = f"{age:.1f} s" + (" SNAPSHOT" if s.snapshot else "")
            self._set("age", text + (" STALE" if age > STALE_S else ""),
                      BAD if age > STALE_S or s.snapshot else TEXT)
            state, gps, hitl = self.live.status
        else:
            age = self.play_time - s.time
            self._set("age", f"file +{max(0.0, age):.2f} s")
            state, gps, hitl = s.flight_state, s.gps_locked, s.hitl
        self._set("state", state_name(state))
        if hitl == 1:
            self._set("hitl", "ACTIVE - SIMULATED", BAD)
        else:
            self._set("hitl", "off" if hitl == 0 else "--")
        self._set("gps", {0: "0", 1: "1"}.get(gps, "--"))
        if s.raw is not None:
            for key, value in zip(("qx", "qy", "qz", "qw"), s.raw):
                self._set(key, f"{value:+.5f}" if math.isfinite(value) else "invalid")
        if s.q is None:
            self._set("note", f"Attitude not available ({s.reason})", BAD)
            if s.raw_norm is not None:
                self._set("norm", f"{s.raw_norm:.4f}")
        else:
            if s.raw_norm is not None:
                bad = am.norm_bad(s.raw_norm)
                self._set("norm", f"{s.raw_norm:.4f}" + ("  CHECK NORM" if bad else ""), BAD if bad else TEXT)
            angles = am.attitude_angles(s.q)
            self._set("tilt", fmt(angles["tilt"]))
            vertical = angles["tilt"] < 1.0
            self._set("azimuth", fmt(angles["azimuth"]) + (" (~vertical)" if vertical and angles["azimuth"] is not None else ""),
                      MUTED if vertical else TEXT)
            self._set("spin", fmt(angles["spin"]))
            self._set("yaw", fmt(angles["yaw"]) + (" (yaw+roll)" if angles["gimbal"] else ""))
            self._set("pitch", fmt(angles["pitch"]))
            self._set("roll", fmt(angles["roll"]) + (" gimbal lock" if angles["gimbal"] else ""),
                      MUTED if angles["gimbal"] else TEXT)
        if self.track is not None and self.mode == "file" and self.track.has_truth:
            if s.truth is not None:
                self._set("truth_tilt", fmt(am.tilt_deg(s.truth)))
            if offset is not None:
                self._set("offset", f"{offset:+.1f}° {how}")
            if s.truth is not None and s.q is not None and offset is not None:
                tilt_err, att_err = am.attitude_errors(s.q, s.truth, offset)
                self._set("tilt_err", fmt(tilt_err, 2))
                self._set("att_err", fmt(att_err, 2))
            elif s.truth is None:
                self._set("note", f"Truth not available ({s.truth_reason})", WARN)

    # ------------------------------------------------------------ scene
    def _draw_scene(self, s, offset):
        c = self.canvas
        c.delete("all")
        screen, w, h, cx, cy = make_screen(c, self.view_az, self.view_el)
        draw_world(c, screen)
        c.create_text(8, 8, anchor="nw", fill=MUTED, font=("Segoe UI", 9),
                      text=f"camera az {self.view_az:.0f}° el {self.view_el:.0f}°  (world ENU, grid 0.25)")

        truth = s.truth if s is not None and self.mode == "file" and self.show_truth.get() else None
        if truth is not None:
            self._draw_rocket(screen, truth, ghost=True)
        if s is None or s.q is None:
            message = "ATTITUDE NOT AVAILABLE" if s is not None else "NO ATTITUDE DATA"
            detail = s.reason if s is not None else "waiting for Witness 0x02 / 0xE0 or load a CSV"
            c.create_text(cx, cy - 12, text=message, fill=BAD, font=("Segoe UI", 18, "bold"))
            c.create_text(cx, cy + 16, text=detail, fill=MUTED, font=("Segoe UI", 10))
            return
        q = s.q
        aligned = False
        if offset is not None and self.align.get() and self.mode == "file":
            q = am.apply_heading(q, offset)
            aligned = abs(offset) > 1e-9
        self._draw_rocket(screen, q)
        draw_body_axes(c, screen, q)
        legend = "estimate (black, orange fin = +Y)"
        if truth is not None:
            legend += "   truth (blue dashed)"
        if aligned:
            legend += f"   estimate rotated {offset:+.1f}° about U"
        c.create_text(8, h - 8, anchor="sw", fill=MUTED, font=("Segoe UI", 9), text=legend)

    def _draw_rocket(self, screen, q, ghost=False):
        draw_rocket(self.canvas, screen, q, ghost=ghost)

    # ------------------------------------------------------------ chart
    def _series(self, now):
        """Return (samples, t_start, t_end, cursor, max_gap) for the chart."""
        if self.mode == "file" and self.track is not None:
            return self.track.samples, self.track.start, max(self.track.end, self.track.start + 1e-6), \
                self.play_time, self.track.max_gap
        try:
            window = float(self.window.get())
        except ValueError:
            window = 60.0
        return self.live.history_list(), now - window, now, None, 2.5

    def _draw_chart(self, now):
        c = self.chart
        c.delete("all")
        w = max(c.winfo_width(), 300)
        h = max(c.winfo_height(), 150)
        samples, t0, t1, cursor, max_gap = self._series(now)
        left, right = 46, w - 120
        visible = [s for s in samples if t0 <= s.time <= t1]
        stride = max(1, len(visible) // max(1, int(w - 166)))
        visible = visible[::stride]
        gap = max_gap * stride
        tilts = [am.tilt_deg(x) for s in visible for x in (s.q, s.truth) if x is not None]
        tilt_hi = 90.0 if not tilts or max(tilts) <= 90.0 else 180.0
        panels = ((6, h * 0.5 - 4, -1.0, 1.0, "q"), (h * 0.5 + 8, h - 18, 0.0, tilt_hi, "tilt °"))
        span = max(t1 - t0, 1e-6)

        def xpos(t):
            return left + (t - t0) / span * (right - left)

        for top, bottom, lo, hi, title in panels:
            c.create_rectangle(left, top, right, bottom, outline="#999999")
            for frac in (0.0, 0.5, 1.0):
                y = bottom - frac * (bottom - top)
                c.create_line(left, y, right, y, fill=GRID)
                value = lo + frac * (hi - lo)
                c.create_text(left - 4, y, anchor="e", fill=MUTED, font=("Segoe UI", 8),
                              text=f"{value:g}")
            c.create_text(4, top, anchor="nw", fill=TEXT, font=("Segoe UI", 8, "bold"), text=title)
        if self.mode == "file":
            c.create_text(left, h - 2, anchor="sw", fill=MUTED, font=("Segoe UI", 8), text=f"{t0:.1f} s")
            c.create_text(right, h - 2, anchor="se", fill=MUTED, font=("Segoe UI", 8), text=f"{t1:.1f} s")
        else:
            c.create_text(left, h - 2, anchor="sw", fill=MUTED, font=("Segoe UI", 8), text=f"-{span:.0f} s")
            c.create_text(right, h - 2, anchor="se", fill=MUTED, font=("Segoe UI", 8), text="now (host time)")

        def ymap(value, top, bottom, lo, hi):
            value = max(lo, min(hi, value))
            return bottom - (value - lo) / (hi - lo) * (bottom - top)

        def polylines(getter, panel, **kw):
            top, bottom, lo, hi, _ = panel
            run, last_t = [], None
            for s in visible:
                value = getter(s)
                if value is None or (last_t is not None and s.time - last_t > gap):
                    if len(run) >= 4:
                        c.create_line(*run, **kw)
                    run = []
                if value is not None:
                    run += [xpos(s.time), ymap(value, top, bottom, lo, hi)]
                last_t = s.time
            if len(run) >= 4:
                c.create_line(*run, **kw)
            elif len(run) == 2:
                c.create_oval(run[0] - 1.5, run[1] - 1.5, run[0] + 1.5, run[1] + 1.5,
                              outline=kw.get("fill"), fill=kw.get("fill"))

        has_truth = self.mode == "file" and self.track is not None and self.track.has_truth
        for i, color in enumerate(Q_COLORS):
            if has_truth:
                polylines(lambda s, i=i: s.truth[i] if s.truth is not None else None, panels[0],
                          fill=color, width=1, dash=(3, 3))
            polylines(lambda s, i=i: s.q[i] if s.q is not None else None, panels[0], fill=color, width=1.5)
        if has_truth:
            polylines(lambda s: am.tilt_deg(s.truth) if s.truth is not None else None, panels[1],
                      fill=GHOST, width=1, dash=(3, 3))
        polylines(lambda s: am.tilt_deg(s.q) if s.q is not None else None, panels[1], fill=TEXT, width=1.5)
        for i, (label, color) in enumerate(zip("xyzw", Q_COLORS)):
            c.create_text(right + 8 + 26 * i, 10, anchor="nw", fill=color, font=("Segoe UI", 9, "bold"),
                          text=f"q{label}")
        c.create_text(right + 8, h * 0.5 + 10, anchor="nw", fill=TEXT, font=("Segoe UI", 9), text="tilt est")
        if has_truth:
            c.create_text(right + 8, 30, anchor="nw", fill=MUTED, font=("Segoe UI", 8), text="dashed = truth")
            c.create_text(right + 8, h * 0.5 + 26, anchor="nw", fill=GHOST, font=("Segoe UI", 9), text="tilt truth")
        if cursor is not None:
            x = xpos(cursor)
            c.create_line(x, 4, x, h - 16, fill=BAD, width=1)
