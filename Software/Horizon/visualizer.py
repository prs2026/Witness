"""Horizon Visualizer: a separate window with an Orientation tab and a Map tab.

Open it from Horizon's menu (Windows > Visualizer, or F9). Both tabs use the
live Witness/Featherweight records Flight View receives; data is collected
while the window is closed, so opening it later still shows the track.

Orientation: the Witness quaternion (fresh 0xE0, else 0x02; see attitude_data)
drawn as the same 3D wireframe rocket as the Attitude tab, with large tilt,
roll and heading readouts. All-zero quaternions are explained: not yet
initialised on the pad, or LOST in flight after gyro saturation (the last valid
attitude is then drawn dashed).

Map: top-down, north up, centred on the pad (pad_reference.py chooses the
source; "Set pad here" overrides it). Always works offline as a metric grid;
when "OSM tiles" is on, OpenStreetMap tiles are fetched on a worker thread and
cached on disk (map_tiles.py), so pre-caching at home makes them available at
the launch site. Range rings, north arrow, scale bar, track per source, and
range / bearing / altitude above the pad for each source.

Redraws are timer driven (REFRESH_MS, 25 Hz max) and only when something
changed, so a fast feed never blocks the UI.
"""

import math
import time
import tkinter as tk
from tkinter import messagebox, ttk

import attitude_math as am
from attitude_data import LiveAttitude, state_name
from attitude_view import (BAD, BG, GHOST, GRID, MUTED, OK, TEXT, VIEWS, WARN, draw_body_axes,
                           draw_rocket, draw_world, fmt, make_screen)
from geodesy import (TILE_SIZE, compass_point, enu_to_geodetic, format_distance, geodetic_to_enu,
                     lat_lon_to_world_px, metres_per_pixel, nice_ring_step)
from map_data import SOURCE_LABELS, MapData
from map_tiles import ATTRIBUTION, MAX_PRECACHE_TILES, TileCache, precache_plan
from pad_reference import SOURCE_LABELS as PAD_LABELS

REFRESH_MS = 40          # <= 25 Hz redraw, and only when dirty
READOUT_MS = 250
STALE_S = 3.0
IN_FLIGHT = (1, 2, 3)
TRACK_STYLE = {          # colour, line dash, marker
    "kf": ("#171717", None, "dot"),
    "wgps": ("#0050b0", None, "square"),
    "fw": ("#008030", (6, 3), "circle"),
}
PAD_COLOR = "#c00000"
MIN_MAP_ZOOM, MAX_MAP_ZOOM, DEFAULT_ZOOM = 8, 19, 16
TILE_CACHE_IMAGES = 160


# ------------------------------------------------------------ orientation

def attitude_condition(sample, last_valid, flight_state, hitl=None):
    """(code, title, detail) for the current attitude sample.

    code: none (no data), ok, lost (zero quaternion in flight after a valid
    one: gyro saturation), unavailable (zero in flight, never valid here),
    init (zero on the pad: waiting for the stationary window).
    """
    if sample is None:
        return "none", "NO ATTITUDE DATA", "waiting for Witness 0x02 / 0xE0"
    if sample.q is not None:
        return "ok", "", ""
    if flight_state in IN_FLIGHT or flight_state == 4:
        if last_valid is not None:
            return ("lost", "ATTITUDE LOST",
                    "Gyro saturated in flight (>= 1950 dps): Witness stops its attitude for the rest "
                    "of the flight and its Kalman filter ignores the IMU. Last valid attitude dashed.")
        return ("unavailable", "ATTITUDE NOT AVAILABLE",
                "Zero quaternion in flight: no stationary window before launch, or lost "
                "before Horizon received a valid attitude.")
    detail = f"Waiting for a 1 s stationary window on the pad ({sample.reason})."
    if hitl == 1:
        detail += " HITL entry/exit resets the estimator."
    return "init", "ATTITUDE NOT INITIALISED", detail


class OrientationTab(ttk.Frame):
    def __init__(self, parent, *, live=None, refresh_ms=REFRESH_MS):
        super().__init__(parent, padding=6)
        self.live = live if live is not None else LiveAttitude()
        self.refresh_ms = refresh_ms
        self.view_az, self.view_el = VIEWS["Iso"]
        self.last_valid = None
        self._drag = None
        self._drawn_key = None
        self._readout_at = 0.0
        self.condition = ("none", "", "")
        self._build()
        self._after_id = self.after(self.refresh_ms, self._tick)

    def _build(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        bar = ttk.Frame(self)
        bar.grid(row=0, column=0, columnspan=2, sticky="ew")
        ttk.Label(bar, text="Camera:").pack(side="left")
        for name in VIEWS:
            ttk.Button(bar, text=name, width=len(name) + 1,
                       command=lambda name=name: self.set_view(name)).pack(side="left", padx=1)
        ttk.Label(bar, text="drag to orbit. Live Witness quaternion: fresh 0xE0, else 0x02.",
                  foreground=MUTED).pack(side="left", padx=8)
        self.canvas = tk.Canvas(self, bg=BG, highlightthickness=1, highlightbackground="#999999",
                                width=520, height=420)
        self.canvas.grid(row=1, column=0, sticky="nsew", pady=4)
        self.canvas.bind("<ButtonPress-1>", self._drag_start)
        self.canvas.bind("<B1-Motion>", self._drag_move)
        self.canvas.bind("<Configure>", lambda _e: self._invalidate())
        panel = ttk.Frame(self, padding=(10, 0, 0, 0))
        panel.grid(row=1, column=1, sticky="ns", pady=4)
        self.values, self.labels = {}, {}
        big = ("Segoe UI", 20, "bold")
        r = 0
        for key, title in (("tilt", "Tilt from vertical"), ("roll", "Roll (spin about nose)"),
                           ("heading", "Heading (nose azimuth)")):
            ttk.Label(panel, text=title).grid(row=r, column=0, sticky="w")
            var = tk.StringVar(value="--")
            label = tk.Label(panel, textvariable=var, font=big, fg=TEXT, anchor="w", width=9)
            label.grid(row=r + 1, column=0, sticky="w", pady=(0, 6))
            self.values[key], self.labels[key] = var, label
            r += 2
        ttk.Label(panel, text="Heading and roll are relative to the arbitrary heading set at\n"
                              "initialisation (no magnetometer). Tilt is absolute.",
                  foreground=MUTED, justify="left").grid(row=r, column=0, sticky="w", pady=(0, 8))
        r += 1
        small = ("Consolas", 10)
        for key, title in (("status", "Attitude"), ("source", "Source"), ("age", "Sample age"),
                           ("state", "Flight state"), ("hitl", "HITL"), ("quat", "q (x, y, z, w)"),
                           ("norm", "|q|")):
            ttk.Label(panel, text=title).grid(row=r, column=0, sticky="w")
            var = tk.StringVar(value="--")
            label = tk.Label(panel, textvariable=var, font=small, fg=TEXT, anchor="w", justify="left",
                             wraplength=260, width=34)
            label.grid(row=r + 1, column=0, sticky="w", pady=(0, 3))
            self.values[key], self.labels[key] = var, label
            r += 2

    def _set(self, key, text, color=TEXT):
        if self.values[key].get() != text:
            self.values[key].set(text)
        self.labels[key].configure(fg=color)

    def set_view(self, name):
        self.view_az, self.view_el = VIEWS[name]
        self._invalidate()

    def _drag_start(self, event):
        self._drag = (event.x, event.y, self.view_az, self.view_el)

    def _drag_move(self, event):
        if self._drag is None:
            return
        x0, y0, az, el = self._drag
        self.view_az = (az - (event.x - x0) * 0.5) % 360.0
        self.view_el = max(-89.0, min(90.0, el + (event.y - y0) * 0.5))
        self._invalidate()

    def _invalidate(self):
        self._drawn_key = None

    def _tick(self):
        try:
            if self.winfo_ismapped():
                self.refresh()
        finally:
            self._after_id = self.after(self.refresh_ms, self._tick)

    def refresh(self, now=None):
        now = time.monotonic() if now is None else now
        sample = self.live.current(now)
        state, _gps, hitl = self.live.status
        if sample is not None and sample.q is not None:
            self.last_valid = sample
        elif state == 0 and sample is not None and sample.q is None and self.last_valid is not None \
                and self.last_valid.flight_state not in (0, None):
            self.last_valid = None      # back on the pad: a new flight
        self.condition = attitude_condition(sample, self.last_valid, state, hitl)
        key = (id(sample), self.condition[0], id(self.last_valid), self.view_az, self.view_el)
        if key != self._drawn_key:
            self._drawn_key = key
            self._draw(sample)
        if now - self._readout_at >= READOUT_MS / 1000 or key != getattr(self, "_readout_key", None):
            self._readout_at, self._readout_key = now, key
            self._readouts(sample, now, state, hitl)

    def _readouts(self, s, now, state, hitl):
        code, title, detail = self.condition
        shown = s.q if s is not None and s.q is not None else None
        angles = am.attitude_angles(shown) if shown is not None else None
        if angles is not None:
            vertical = angles["tilt"] < 1.0
            self._set("tilt", fmt(angles["tilt"]))
            self._set("roll", fmt(angles["spin"]))
            self._set("heading", fmt(angles["azimuth"]) + (" ~vert" if vertical else ""),
                      MUTED if vertical else TEXT)
        else:
            for key in ("tilt", "roll", "heading"):
                self._set(key, "--", MUTED)
        if code == "ok":
            self._set("status", "valid", OK)
        else:
            self._set("status", f"{title}: {detail}", BAD if code in ("lost", "unavailable") else WARN)
        if s is None:
            for key in ("source", "age", "quat", "norm"):
                self._set(key, "--")
        else:
            self._set("source", s.source + (f" ({s.device})" if s.device else ""))
            age = now - s.time
            self._set("age", f"{age:.1f} s" + (" SNAPSHOT" if s.snapshot else "") + (" STALE" if age > STALE_S else ""),
                      BAD if age > STALE_S or s.snapshot else TEXT)
            if s.raw is not None:
                self._set("quat", ", ".join(f"{v:+.4f}" if math.isfinite(v) else "nan" for v in s.raw))
            if s.raw_norm is not None:
                bad = s.q is not None and am.norm_bad(s.raw_norm)
                self._set("norm", f"{s.raw_norm:.4f}" + ("  CHECK NORM" if bad else ""), BAD if bad else TEXT)
        self._set("state", state_name(state))
        self._set("hitl", "ACTIVE - SIMULATED" if hitl == 1 else "off" if hitl == 0 else "--",
                  BAD if hitl == 1 else TEXT)

    def _draw(self, s):
        c = self.canvas
        c.delete("all")
        screen, w, h, cx, cy = make_screen(c, self.view_az, self.view_el)
        draw_world(c, screen)
        code, title, detail = self.condition
        if code == "ok":
            draw_rocket(c, screen, s.q)
            draw_body_axes(c, screen, s.q)
            return
        if code == "lost" and self.last_valid is not None:
            draw_rocket(c, screen, self.last_valid.q, ghost=True)
        c.create_text(cx, 28, text=title, fill=BAD, font=("Segoe UI", 18, "bold"))
        c.create_text(cx, 54, text=detail, fill=MUTED, font=("Segoe UI", 10), width=max(200, w - 40))

    def close(self):
        if self._after_id is not None:
            try:
                self.after_cancel(self._after_id)
            except tk.TclError:
                pass
            self._after_id = None


# -------------------------------------------------------------------- map

class MapTab(ttk.Frame):
    def __init__(self, parent, *, data=None, tiles=None, refresh_ms=REFRESH_MS):
        super().__init__(parent, padding=6)
        self.data = data if data is not None else MapData()
        self._own_tiles = tiles is None
        self.tiles = tiles if tiles is not None else TileCache()
        self.refresh_ms = refresh_ms
        self.zoom = DEFAULT_ZOOM
        self.pan = (0.0, 0.0)            # map centre, ENU metres from the reference
        self.set_pad_mode = False
        self._images = {}                # (z, x, y) -> PhotoImage, insertion ordered (LRU)
        self._drag = None
        self._drawn_key = None
        self._readout_at = 0.0
        self.last_snapshot = None
        self._build()
        self._after_id = self.after(self.refresh_ms, self._tick)

    # -------------------------------------------------------------- UI
    def _build(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        bar = ttk.Frame(self)
        bar.grid(row=0, column=0, columnspan=2, sticky="ew")
        ttk.Button(bar, text="-", width=3, command=lambda: self.set_zoom(self.zoom - 1)).pack(side="left")
        ttk.Button(bar, text="+", width=3, command=lambda: self.set_zoom(self.zoom + 1)).pack(side="left")
        self.zoom_label = ttk.Label(bar, width=8)
        self.zoom_label.pack(side="left", padx=(2, 8))
        ttk.Button(bar, text="Centre on pad", command=self.centre).pack(side="left")
        self.follow = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="Follow rocket", variable=self.follow,
                        command=self._invalidate).pack(side="left", padx=6)
        self.pad_button = ttk.Button(bar, text="Set pad here...", command=self.toggle_set_pad)
        self.pad_button.pack(side="left", padx=(8, 2))
        ttk.Button(bar, text="Auto pad", command=self.clear_manual_pad).pack(side="left", padx=2)
        ttk.Button(bar, text="Clear track", command=self.clear_track).pack(side="left", padx=(8, 2))
        self.online = tk.BooleanVar(value=self.tiles.online)
        ttk.Checkbutton(bar, text="OSM tiles", variable=self.online,
                        command=self._online_changed).pack(side="left", padx=(12, 2))
        ttk.Button(bar, text="Pre-cache area...", command=self.precache_dialog).pack(side="left", padx=2)
        self.canvas = tk.Canvas(self, bg="#f4f4ef", highlightthickness=1, highlightbackground="#999999",
                                width=620, height=480, cursor="fleur")
        self.canvas.grid(row=1, column=0, sticky="nsew", pady=4)
        self.canvas.bind("<ButtonPress-1>", self._press)
        self.canvas.bind("<B1-Motion>", self._motion)
        self.canvas.bind("<ButtonRelease-1>", self._release)
        self.canvas.bind("<MouseWheel>", self._wheel)
        self.canvas.bind("<Button-4>", lambda e: self.set_zoom(self.zoom + 1))
        self.canvas.bind("<Button-5>", lambda e: self.set_zoom(self.zoom - 1))
        self.canvas.bind("<Configure>", lambda _e: self._invalidate())
        panel = ttk.Frame(self, padding=(10, 0, 0, 0))
        panel.grid(row=1, column=1, sticky="ns", pady=4)
        self.values, self.labels = {}, {}
        rows = [("pad", "Pad"), ("pad_pos", "Pad position"), ("pad_alt", "Pad altitude"), None,
                ("state", "Flight state")]
        for name in ("kf", "wgps", "fw"):
            rows += [None, (name, SOURCE_LABELS[name])]
        rows += [None, ("tiles", "Map tiles")]
        r = 0
        for item in rows:
            if item is None:
                ttk.Separator(panel).grid(row=r, column=0, sticky="ew", pady=3)
                r += 1
                continue
            key, title = item
            if key in TRACK_STYLE:
                head = tk.Label(panel, text="● " + title, fg=TRACK_STYLE[key][0],
                                font=("Segoe UI", 9, "bold"), anchor="w")
                head.grid(row=r, column=0, sticky="w")
            else:
                ttk.Label(panel, text=title).grid(row=r, column=0, sticky="w")
            var = tk.StringVar(value="--")
            label = tk.Label(panel, textvariable=var, font=("Consolas", 10), fg=TEXT, anchor="w",
                             justify="left", width=36, wraplength=250)
            label.grid(row=r + 1, column=0, sticky="w")
            self.values[key], self.labels[key] = var, label
            r += 2

    def _set(self, key, text, color=TEXT):
        if self.values[key].get() != text:
            self.values[key].set(text)
        self.labels[key].configure(fg=color)

    # ---------------------------------------------------------- actions
    def set_zoom(self, zoom):
        self.zoom = max(MIN_MAP_ZOOM, min(MAX_MAP_ZOOM, int(zoom)))
        self._invalidate()

    def centre(self):
        self.pan = (0.0, 0.0)
        self.follow.set(False)
        self._invalidate()

    def clear_track(self):
        self.data.clear()
        self._invalidate()

    def toggle_set_pad(self):
        self.set_pad_mode = not self.set_pad_mode
        self.pad_button.configure(text="Click map (Esc)" if self.set_pad_mode else "Set pad here...")
        self.canvas.configure(cursor="crosshair" if self.set_pad_mode else "fleur")
        self.canvas.bind("<Escape>", lambda _e: self.toggle_set_pad() if self.set_pad_mode else None)
        self.canvas.focus_set()

    def clear_manual_pad(self):
        self.data.pad.clear_manual()
        self._invalidate()

    def _online_changed(self):
        self.tiles.set_online(self.online.get())
        self._invalidate()

    def precache_dialog(self):
        ref = self.data.reference()
        if ref is None:
            messagebox.showinfo("Pre-cache map", "The pad position is not known yet. Wait for a pad "
                                "source (Featherweight GS, Aperture GPS) or use Set pad here.", parent=self)
            return
        if not self.online.get():
            messagebox.showinfo("Pre-cache map", "Turn on OSM tiles first (needs internet).", parent=self)
            return
        radius, zooms = 2000.0, range(12, 18)
        plan = precache_plan(ref[0], ref[1], radius, zooms)
        if len(plan) > MAX_PRECACHE_TILES:
            radius, plan = 1500.0, precache_plan(ref[0], ref[1], 1500.0, zooms)
        if not messagebox.askyesno(
                "Pre-cache map",
                f"Download up to {min(len(plan), MAX_PRECACHE_TILES)} OpenStreetMap tiles covering "
                f"{radius / 1000:.1f} km around the pad (zoom {zooms.start}-{zooms.stop - 1}) into "
                f"{self.tiles.root}?\nAlready-cached tiles are skipped. Downloads run slowly in the "
                "background (tile usage policy).", parent=self):
            return
        queued = self.tiles.precache(plan)
        self._set("tiles", f"pre-cache: {queued} tiles queued")

    # ------------------------------------------------------------ mouse
    def _press(self, event):
        if self.set_pad_mode:
            self.set_pad_at(event.x, event.y)
            return
        self._drag = (event.x, event.y, self.pan)

    def _motion(self, event):
        if self._drag is None:
            return
        x0, y0, (pe, pn) = self._drag
        mpp = self._mpp()
        self.follow.set(False)
        self.pan = (pe - (event.x - x0) * mpp, pn + (event.y - y0) * mpp)
        self._invalidate()

    def _release(self, _event):
        self._drag = None

    def _wheel(self, event):
        self.set_zoom(self.zoom + (1 if event.delta > 0 else -1))

    def set_pad_at(self, x, y):
        ref = self.data.reference()
        if ref is None:
            messagebox.showinfo("Set pad", "No position reference yet: wait for a GPS fix first.", parent=self)
        else:
            e, n = self.screen_to_enu(x, y)
            lat, lon, _ = enu_to_geodetic(e, n, 0.0, ref[0], ref[1], 0.0)
            self.data.pad.set_manual(lat, lon)
            self.pan = (0.0, 0.0)
        self.toggle_set_pad()
        self._invalidate()

    # --------------------------------------------------------- geometry
    def _ref_lat(self):
        ref = self.data.reference()
        return ref[0] if ref else 0.0

    def _mpp(self):
        return metres_per_pixel(self._ref_lat(), self.zoom)

    def _size(self):
        return max(self.canvas.winfo_width(), 300), max(self.canvas.winfo_height(), 240)

    def enu_to_screen(self, e, n, mpp=None, size=None):
        mpp = mpp or self._mpp()
        w, h = size or self._size()
        return w / 2 + (e - self.pan[0]) / mpp, h / 2 - (n - self.pan[1]) / mpp

    def screen_to_enu(self, x, y):
        mpp = self._mpp()
        w, h = self._size()
        return self.pan[0] + (x - w / 2) * mpp, self.pan[1] - (y - h / 2) * mpp

    # ----------------------------------------------------------- redraw
    def _invalidate(self):
        self._drawn_key = None

    def _tick(self):
        try:
            if self.winfo_ismapped():
                self.refresh()
        finally:
            self._after_id = self.after(self.refresh_ms, self._tick)

    def refresh(self, now=None):
        now = time.monotonic() if now is None else now
        if self.tiles.drain_completed():
            self._drawn_key = None
        key = (self.data.version, self.data.pad.version, self.zoom, self.pan, self.follow.get(),
               self.online.get(), self._size())
        if key != self._drawn_key:
            snap = self.data.snapshot()
            self.last_snapshot = snap
            if self.follow.get():
                primary = self.primary(snap, now)
                if primary is not None:
                    self.pan = (primary.east, primary.north)
                    key = key[:3] + (self.pan,) + key[4:]
            self._drawn_key = key
            self._draw(snap)
        if now - self._readout_at >= READOUT_MS / 1000 and self.last_snapshot is not None:
            self._readout_at = now
            self._readouts(self.last_snapshot, now)

    @staticmethod
    def primary(snap, now=None):
        """Freshest-preferred rocket point: Kalman, else Witness GPS, else Featherweight."""
        now = time.monotonic() if now is None else now
        latest = snap["latest"]
        for name in ("kf", "wgps", "fw"):
            item = latest.get(name)
            if item is not None and now - item["point"].t <= STALE_S:
                return item["point"]
        items = [latest[name]["point"] for name in ("kf", "wgps", "fw") if name in latest]
        return max(items, key=lambda p: p.t) if items else None

    def _draw(self, snap):
        c = self.canvas
        c.delete("all")
        w, h = self._size()
        ref = snap["reference"]
        mpp = metres_per_pixel(ref[0] if ref else 0.0, self.zoom)
        self.zoom_label.configure(text=f"zoom {self.zoom}")
        # Offline, already-cached tiles are still drawn; online, missing ones are queued.
        tiles_drawn = self._draw_tiles(ref, w, h) if ref is not None else 0
        self._draw_grid(w, h, mpp)
        to_screen = lambda e, n: self.enu_to_screen(e, n, mpp, (w, h))
        # Range rings around the pad.
        px, py = to_screen(0.0, 0.0)
        span = min(w, h) * mpp / 2
        step = nice_ring_step(span, 3)
        for i in range(1, 9):
            radius = i * step / mpp
            if radius > max(w, h) * 1.5:
                break
            c.create_oval(px - radius, py - radius, px + radius, py + radius, outline="#c06060",
                          dash=(3, 4))
            c.create_text(px + radius * 0.7071 + 3, py - radius * 0.7071 - 3, anchor="sw",
                          text=format_distance(i * step), fill="#a04040", font=("Segoe UI", 8))
        # Witness launch site, when it differs from the pad.
        site = self.data.pad.witness_launch_site()
        if site is not None and ref is not None and snap["pad"] is not None and snap["pad"].source != "witness":
            se, sn, _ = geodetic_to_enu(site.lat, site.lon, 0.0, ref[0], ref[1], 0.0)
            sx, sy = to_screen(se, sn)
            c.create_polygon(sx, sy - 6, sx + 6, sy, sx, sy + 6, sx - 6, sy, outline="#c06000", fill="",
                             width=2)
            c.create_text(sx + 8, sy, anchor="w", text="Witness pad", fill="#c06000", font=("Segoe UI", 8))
        # Tracks.
        for name in ("fw", "wgps", "kf"):
            points = snap["tracks"].get(name, [])
            if not points:
                continue
            color, dash, marker = TRACK_STYLE[name]
            stride = max(1, len(points) // 1500)
            coords = []
            for p in points[::stride] + [points[-1]]:
                coords += to_screen(p.east, p.north)
            if len(coords) >= 4:
                c.create_line(*coords, fill=color, width=2 if name == "kf" else 1.5, dash=dash or "")
            if marker == "square":
                for p in points[-60::max(1, stride)]:
                    x, y = to_screen(p.east, p.north)
                    c.create_rectangle(x - 2, y - 2, x + 2, y + 2, outline=color, fill=color)
            x, y = to_screen(points[-1].east, points[-1].north)
            if marker == "circle":
                c.create_oval(x - 6, y - 6, x + 6, y + 6, outline=color, width=2)
            elif marker == "square":
                c.create_rectangle(x - 5, y - 5, x + 5, y + 5, outline=color, width=2)
            else:
                c.create_oval(x - 5, y - 5, x + 5, y + 5, fill=color, outline="white")
            agl = points[-1].agl
            c.create_text(x + 9, y - 9, anchor="sw", fill=color, font=("Segoe UI", 9, "bold"),
                          text={"kf": "KF", "wgps": "W GPS", "fw": "FW"}[name]
                          + (f" {agl:.0f} m" if agl is not None else ""))
        # Pad marker.
        c.create_polygon(px, py - 9, px + 8, py + 6, px - 8, py + 6, fill=PAD_COLOR, outline="white")
        c.create_text(px, py + 9, anchor="n", text="PAD", fill=PAD_COLOR, font=("Segoe UI", 8, "bold"))
        self._draw_north_arrow(w)
        self._draw_scale_bar(h, mpp)
        if tiles_drawn:
            c.create_text(w - 4, h - 3, anchor="se", text=ATTRIBUTION, fill="#333333",
                          font=("Segoe UI", 8))
        if ref is None:
            c.create_text(w / 2, 24, text="PAD POSITION UNKNOWN - grid is relative to the Witness "
                          "Kalman origin; GPS tracks appear once a pad source or fix arrives.",
                          fill=WARN, font=("Segoe UI", 10, "bold"), width=w - 40)
        elif snap["pad"] is None:
            c.create_text(w / 2, 24, text=f"No pad source yet - centred on {ref[2]}", fill=WARN,
                          font=("Segoe UI", 10, "bold"))
        if self.set_pad_mode:
            c.create_text(w / 2, h - 24, text="Click the pad location (Esc cancels)", fill=PAD_COLOR,
                          font=("Segoe UI", 11, "bold"))

    def _draw_grid(self, w, h, mpp):
        c = self.canvas
        step = nice_ring_step(max(w, h) * mpp, 8)
        e0, n1 = self.screen_to_enu_at(0, 0, mpp, w, h)
        e1, n0 = self.screen_to_enu_at(w, h, mpp, w, h)
        e = math.floor(e0 / step) * step
        while e <= e1:
            x, _ = self.enu_to_screen(e, 0, mpp, (w, h))
            c.create_line(x, 0, x, h, fill=GRID if e else "#a0a0a0")
            e += step
        n = math.floor(n0 / step) * step
        while n <= n1:
            _, y = self.enu_to_screen(0, n, mpp, (w, h))
            c.create_line(0, y, w, y, fill=GRID if n else "#a0a0a0")
            n += step

    def screen_to_enu_at(self, x, y, mpp, w, h):
        return self.pan[0] + (x - w / 2) * mpp, self.pan[1] - (y - h / 2) * mpp

    def _draw_tiles(self, ref, w, h):
        c = self.canvas
        z = self.zoom
        rx, ry = lat_lon_to_world_px(ref[0], ref[1], z)
        mpp = metres_per_pixel(ref[0], z)
        # world pixel at the canvas centre
        wx, wy = rx + self.pan[0] / mpp, ry - self.pan[1] / mpp
        left, top = wx - w / 2, wy - h / 2
        n = 1 << z
        drawn = 0
        for ty in range(max(0, int(top // TILE_SIZE)), min(n - 1, int((top + h) // TILE_SIZE)) + 1):
            for tx in range(int(left // TILE_SIZE), int((left + w) // TILE_SIZE) + 1):
                key = (z, tx % n, ty)
                image = self._image(key)
                if image is not None:
                    c.create_image(tx * TILE_SIZE - left, ty * TILE_SIZE - top, image=image, anchor="nw")
                    drawn += 1
        return drawn

    def _image(self, key):
        image = self._images.pop(key, None)
        if image is None:
            path = self.tiles.request(*key) if self.online.get() else self.tiles.cached(*key)
            if path is None:
                return None
            try:
                image = tk.PhotoImage(master=self.canvas, file=str(path))
            except tk.TclError:
                return None
            while len(self._images) >= TILE_CACHE_IMAGES:
                self._images.pop(next(iter(self._images)))
        self._images[key] = image
        return image

    def _draw_north_arrow(self, w):
        c = self.canvas
        x, y = w - 30, 46
        c.create_polygon(x, y - 26, x + 9, y + 4, x, y - 3, x - 9, y + 4, fill="#171717", outline="white")
        c.create_text(x, y + 8, anchor="n", text="N", fill="#171717", font=("Segoe UI", 11, "bold"))

    def _draw_scale_bar(self, h, mpp):
        c = self.canvas
        length = nice_ring_step(120 * mpp, 1)
        pixels = length / mpp
        x, y = 12, h - 14
        c.create_rectangle(x - 3, y - 16, x + pixels + 40, y + 6, fill="white", outline="", stipple="gray50")
        c.create_line(x, y, x + pixels, y, fill="#171717", width=3)
        for xx in (x, x + pixels):
            c.create_line(xx, y - 5, xx, y + 3, fill="#171717", width=2)
        c.create_text(x + pixels + 4, y, anchor="w", text=format_distance(length), fill="#171717",
                      font=("Segoe UI", 9))

    def _readouts(self, snap, now):
        pad = snap["pad"]
        if pad is None:
            ref = snap["reference"]
            self._set("pad", "unknown" + (f" (centred on {ref[2]})" if ref else ""), WARN)
            self._set("pad_pos", "--")
        else:
            self._set("pad", PAD_LABELS.get(pad.source, pad.source) + (f", {pad.detail}" if pad.detail else ""),
                      WARN if pad.source in ("tracker", "manual") else TEXT)
            self._set("pad_pos", f"{pad.lat:.6f}, {pad.lon:.6f}")
        alt = snap["pad_alt"]
        self._set("pad_alt", f"{alt[0]:.1f} m MSL ({alt[1]})" if alt else "unknown (GPS AGL unavailable)",
                  TEXT if alt else WARN)
        state, gps, hitl = snap["status"]
        self._set("state", state_name(state) + (f", GPS lock {gps}" if gps is not None else "")
                  + (", HITL SIMULATED" if hitl == 1 else ""), BAD if hitl == 1 else TEXT)
        for name in ("kf", "wgps", "fw"):
            item = snap["latest"].get(name)
            if item is None:
                self._set(name, "no data", MUTED)
                continue
            p = item["point"]
            age = now - p.t
            agl = f"{p.agl:.1f} m AGL" if p.agl is not None else "alt AGL --"
            text = (f"{format_distance(item['range'])} @ {item['bearing']:.0f}° "
                    f"{compass_point(item['bearing'])}\n{agl}   age {age:.1f} s")
            if name == "kf" and snap["reference"] is not None and not snap["kf_placed"]:
                text += "\norigin assumed at pad"
            self._set(name, text, BAD if age > STALE_S else TEXT)
        self._set("tiles", self.tiles.status())

    def close(self):
        if self._after_id is not None:
            try:
                self.after_cancel(self._after_id)
            except tk.TclError:
                pass
            self._after_id = None
        self._images.clear()
        if self._own_tiles:
            self.tiles.close()


# ------------------------------------------------------------------ window

class VisualizerWindow(tk.Toplevel):
    def __init__(self, master, *, live=None, map_data=None, tiles=None):
        super().__init__(master)
        self.title("Horizon Visualizer")
        self.geometry(f"{min(1100, self.winfo_screenwidth() - 60)}x{min(760, self.winfo_screenheight() - 120)}")
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True, padx=4, pady=4)
        self.orientation = OrientationTab(self.notebook, live=live)
        self.map = MapTab(self.notebook, data=map_data, tiles=tiles)
        self.notebook.add(self.orientation, text="Orientation")
        self.notebook.add(self.map, text="Map")
        self.protocol("WM_DELETE_WINDOW", self.close)

    def close(self):
        self.orientation.close()
        self.map.close()
        self.destroy()
