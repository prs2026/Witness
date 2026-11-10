"""Horizon Visualizer: orientation and map side by side in one window (Qt).

Open it from Horizon's menu (Windows > Visualizer, or F9). The 3D orientation
view (left) and the top-down map (right) sit in a horizontal splitter, both
visible at once and resizable. Both use the live Witness/Featherweight records
Flight View receives; data is collected while the window is closed, so
opening it later still shows the track.

Orientation: the Witness quaternion (fresh 0xE0, else 0x02; see attitude_data)
drawn with pyqtgraph.opengl as a 3D rocket with axes (drag to orbit), or as a
2D projection where OpenGL is unavailable. The AIM Xtra orientation (AIM BASE
MQTT) is overlaid on the same view as a translucent purple rocket. Large tilt, roll and heading
readouts. All-zero quaternions are explained: not yet initialised on the pad,
or LOST in flight after gyro saturation (the last valid attitude is then drawn
as a ghost).

Map: a pyqtgraph plot in local ENU metres centred on the pad, north up, locked
aspect (drag to pan, wheel to zoom). pad_reference.py chooses the pad
(Featherweight GS, then Aperture 0xF0 GPS, ...); "Set pad here" overrides it.
It always works offline as a metric grid; with "OSM tiles" on, OpenStreetMap
tiles are fetched by map_tiles.py's rate-limited worker (its User-Agent and
tile policy), cached on disk, decoded on a worker thread and shown as
ImageItems. "Pre-cache area" downloads the launch area at home. Range rings,
north arrow, scale bar, a track per source (Witness Kalman, Witness GPS,
Featherweight tracker, AIM Xtra GPS) and range / bearing / altitude
above the pad for each source.

Redraws are timer driven (REFRESH_MS, 25 Hz max) and only when something
changed, so a fast feed never blocks the UI.
"""

from __future__ import annotations

import math
import time

import numpy as np
from PyQt6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

import attitude_math as am
from attitude_data import LiveAimAttitude, LiveAttitude, state_name
from attitude_scene import (AIM_ALPHA, AIM_COLOR, AIM_STALE_ALPHA, BAD, MUTED, OK, STALE_ALPHA,  # noqa: F401
                            TEXT, VIEWS, WARN,
                            attitude_condition, fmt)
from attitude_view import RocketScene, readout_panel, set_colored
from geodesy import (TILE_SIZE, compass_point, enu_to_geodetic, format_distance, geodetic_to_enu,
                     lat_lon_to_world_px, metres_per_pixel, nice_ring_step)
from map_data import SOURCE_LABELS, MapData
from map_tiles import ATTRIBUTION, MAX_PRECACHE_TILES, TileCache, precache_plan
from observable import Cell
from pad_reference import SOURCE_LABELS as PAD_LABELS
from qt_common import run_in_thread
from qt_widgets import bind_check, light_axes

REFRESH_MS = 40          # <= 25 Hz redraw, and only when dirty
READOUT_MS = 250
STALE_S = 3.0
TRACK_STYLE = {          # colour, dashed, marker symbol
    "kf": ("#171717", False, "o"),
    "wgps": ("#0050b0", False, "s"),
    "fw": ("#008030", True, "o"),
    "aim": (AIM_COLOR, True, "t"),
}
TRACK_TAGS = {"kf": "KF", "wgps": "W GPS", "fw": "FW", "aim": "AIM"}
TRACK_ORDER = ("aim", "fw", "wgps", "kf")      # drawing order: KF on top
PAD_COLOR = "#c00000"
MIN_MAP_ZOOM, MAX_MAP_ZOOM, DEFAULT_ZOOM = 8, 19, 16
TILE_ITEMS = 160
MAX_VISIBLE_TILES = 120
DEFAULT_VIEW_PX = 600
# Auto range: the view's half-extent (pad to the nearest edge) steps through
# these so it never jitters; the farthest rocket stays inside ~80% of it.
AUTO_RANGE_STEPS_M = (500, 750, 1000, 1500, 2000, 3000, 5000, 7500, 10000, 15000, 20000)
AUTO_RANGE_MARGIN = 1.25


def auto_range_radius(snap):
    """Half-extent (m) showing every rocket's latest position: 500 m to 20 km."""
    far = max((item["range"] for item in snap["latest"].values()), default=0.0)
    need = far * AUTO_RANGE_MARGIN
    for step in AUTO_RANGE_STEPS_M:
        if need <= step:
            return float(step)
    return float(AUTO_RANGE_STEPS_M[-1])


def ask_yes_no(parent, title, text) -> bool:
    """Confirmation dialog (tests patch this)."""
    answer = QtWidgets.QMessageBox.question(
        parent, title, text,
        QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
        QtWidgets.QMessageBox.StandardButton.No)
    return answer == QtWidgets.QMessageBox.StandardButton.Yes


def inform(parent, title, text):
    """Information dialog (tests patch this)."""
    QtWidgets.QMessageBox.information(parent, title, text)


def load_tile_rgba(path):
    """PNG -> (h, w, 4) uint8 RGBA, rows flipped so row 0 is the south edge.

    Runs on a worker thread (QImage is safe off the GUI thread).
    """
    image = QtGui.QImage(str(path))
    if image.isNull():
        raise ValueError(f"unreadable tile {path}")
    image = image.convertToFormat(QtGui.QImage.Format.Format_RGBA8888)
    width, height = image.width(), image.height()
    pointer = image.constBits()
    pointer.setsize(image.sizeInBytes())
    array = np.frombuffer(pointer, np.uint8).reshape(height, image.bytesPerLine() // 4, 4)[:, :width]
    return np.ascontiguousarray(array[::-1])


def tile_rect_enu(z, x, y, ref_lat, ref_lon):
    """(east_left, north_bottom, size_m) of tile (z, x, y) around the reference."""
    rx, ry = lat_lon_to_world_px(ref_lat, ref_lon, z)
    mpp = metres_per_pixel(ref_lat, z)
    east = (x * TILE_SIZE - rx) * mpp
    north_top = (ry - y * TILE_SIZE) * mpp
    size = TILE_SIZE * mpp
    return east, north_top - size, size


# ------------------------------------------------------------ orientation

class OrientationPanel(QtWidgets.QWidget):
    def __init__(self, parent=None, *, live=None, aim=None, refresh_ms=REFRESH_MS, use_gl=None):
        super().__init__(parent)
        self.live = live if live is not None else LiveAttitude()
        self.aim = aim if aim is not None else LiveAimAttitude()
        self.aim_shown = None
        self.aim_stale = False
        self.witness_stale = False
        self.last_valid = None
        self._drawn_key = None
        self._readout_key = None
        self._readout_at = 0.0
        self.condition = ("none", "", "")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        bar = QtWidgets.QHBoxLayout()
        bar.addWidget(QtWidgets.QLabel("Camera:"))
        for name in VIEWS:
            button = QtWidgets.QPushButton(name)
            button.clicked.connect(lambda _c=False, n=name: self.set_view(n))
            bar.addWidget(button)
        bar.addStretch(1)
        layout.addLayout(bar)
        hint = QtWidgets.QLabel("Drag to orbit. Solid rocket: Witness quaternion (fresh 0xE0, else 0x02). "
                                f"<span style='color:{AIM_COLOR}'>Translucent purple rocket: AIM Xtra.</span>")
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color: {MUTED};")
        layout.addWidget(hint)
        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        splitter.setChildrenCollapsible(False)
        self.scene = RocketScene(use_gl=use_gl)
        self.scene.viewChanged.connect(lambda *_: self._invalidate())
        splitter.addWidget(self.scene)
        rows = (("tilt", "Tilt from vertical"), ("roll", "Roll (spin about nose)"),
                ("heading", "Heading (nose azimuth)"), None,
                ("status", "Attitude"), ("source", "Source"), ("age", "Sample age"),
                ("state", "Flight state"), ("hitl", "HITL"), ("quat", "q (x, y, z, w)"), ("norm", "|q|"), None,
                ("aim_tilt", "AIM tilt"), ("aim_roll", "AIM roll"), ("aim_heading", "AIM heading"),
                ("aim_age", "AIM sample age"))
        panel, self.values, self.labels = readout_panel(rows, big_keys=("tilt", "roll", "heading"))
        note = QtWidgets.QLabel("Witness heading and roll are relative to the arbitrary heading set at "
                                "initialisation (no magnetometer). Tilt is absolute. AIM's body frame and "
                                "heading reference relative to Witness are unverified, so compare tilt first.")
        note.setWordWrap(True)
        note.setStyleSheet(f"color: {MUTED};")
        panel.layout().addWidget(note, panel.layout().rowCount(), 0, 1, 2)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(panel)
        splitter.addWidget(scroll)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        # Stretch factors alone leave the readouts' size hint in charge of the
        # first layout; give the 3D view about three quarters of the height.
        self.scene.setMinimumHeight(240)
        splitter.setSizes([3000, 1000])
        layout.addWidget(splitter, 1)
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(refresh_ms)

    def _set(self, key, text, color=TEXT):
        set_colored(self.values, self.labels, key, text, color)

    def set_view(self, name):
        self.scene.set_view(name)
        self._invalidate()

    def _invalidate(self):
        self._drawn_key = None

    def _tick(self):
        if self.isVisible():
            self.refresh()

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
        aim = self.aim.current(now)
        # Old samples stay on screen, only fainter.
        self.aim_shown = aim.q if aim is not None and aim.q is not None else None
        self.aim_stale = aim is not None and now - aim.time > STALE_S
        self.witness_stale = sample is not None and (sample.snapshot or now - sample.time > STALE_S)
        key = (id(sample), self.condition[0], id(self.last_valid), id(aim), self.aim_shown is None,
               self.aim_stale, self.witness_stale)
        if key != self._drawn_key:
            self._drawn_key = key
            self._draw(sample)
        if now - self._readout_at >= READOUT_MS / 1000 or key != self._readout_key:
            self._readout_at, self._readout_key = now, key
            self._readouts(sample, now, state, hitl)

    def _draw(self, s):
        code, title, detail = self.condition
        aim = self.aim_shown
        aim_alpha = AIM_STALE_ALPHA if self.aim_stale else AIM_ALPHA
        alpha = STALE_ALPHA if self.witness_stale else 1.0
        notes = []
        if aim is not None:
            notes.append("AIM Xtra: translucent purple rocket" + (" (stale, faded)" if self.aim_stale else ""))
        if code == "ok" and self.witness_stale:
            notes.append("Witness sample stale (faded)")
        aim_legend = "\n".join(notes)
        if code == "ok":
            self.scene.show_attitude(s.q, overlay=aim, legend=aim_legend, alpha=alpha, overlay_alpha=aim_alpha)
        elif code == "lost" and self.last_valid is not None:
            legend = "last valid attitude (blue ghost)" + ("\n" + aim_legend if aim_legend else "")
            self.scene.show_attitude(None, ghost=self.last_valid.q, overlay=aim, message=(title, detail),
                                     legend=legend, overlay_alpha=aim_alpha)
        else:
            self.scene.show_attitude(None, overlay=aim, message=(title, detail), legend=aim_legend,
                                     overlay_alpha=aim_alpha)

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
        self._aim_readouts(now)

    def _aim_readouts(self, now):
        aim = self.aim.current(now)
        angles = am.attitude_angles(aim.q) if aim is not None and aim.q is not None else None
        if angles is None:
            for key in ("aim_tilt", "aim_roll", "aim_heading"):
                self._set(key, "--", MUTED)
        else:
            self._set("aim_tilt", fmt(angles["tilt"]), AIM_COLOR)
            self._set("aim_roll", fmt(angles["spin"]), AIM_COLOR)
            self._set("aim_heading", fmt(angles["azimuth"]), AIM_COLOR)
        if aim is None:
            self._set("aim_age", "no AIM orientation", MUTED)
        else:
            age = now - aim.time
            stale = age > STALE_S
            self._set("aim_age", f"{age:.1f} s" + (" STALE (faded)" if stale else "")
                      + (f" ({aim.reason})" if aim.q is None and aim.reason else ""), BAD if stale else TEXT)

    def close(self):
        self._timer.stop()
        return super().close()


# -------------------------------------------------------------------- map

class MapPanel(QtWidgets.QWidget):
    def __init__(self, parent=None, *, data=None, tiles=None, refresh_ms=REFRESH_MS):
        super().__init__(parent)
        self.data = data if data is not None else MapData()
        self._own_tiles = tiles is None
        self.tiles = tiles if tiles is not None else TileCache()
        self.zoom = DEFAULT_ZOOM
        self.set_pad_mode = False
        self.follow = Cell(False)
        self.auto_range = Cell(True)
        self.auto_radius = None
        self.online = Cell(self.tiles.online)
        self.last_snapshot = None
        self.tiles_drawn = 0
        self.redraws = 0
        self._drawn_key = None
        self._readout_at = 0.0
        self._tile_items = {}            # (z, x, y) -> ImageItem (insertion ordered, LRU)
        self._loading = set()
        self._tile_ref = None
        self._tiles_dirty = True
        self._applying_range = False
        self._build()
        self.set_zoom(DEFAULT_ZOOM)
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(refresh_ms)

    # -------------------------------------------------------------- UI
    def _build(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        bar = QtWidgets.QHBoxLayout()
        minus, plus = QtWidgets.QPushButton("-"), QtWidgets.QPushButton("+")
        for button, step in ((minus, -1), (plus, 1)):
            button.setMaximumWidth(32)
            button.clicked.connect(lambda _c=False, s=step: self._manual_zoom(self.zoom + s))
            bar.addWidget(button)
        self.zoom_label = QtWidgets.QLabel()
        bar.addWidget(self.zoom_label)
        centre = QtWidgets.QPushButton("Centre on pad")
        centre.clicked.connect(self.centre)
        auto = QtWidgets.QCheckBox("Auto range")
        auto.setToolTip("Scale the map to the farthest rocket (500 m to 20 km). "
                        "Zooming or panning by hand turns it off.")
        bind_check(self.auto_range, auto, on_user=self._invalidate)
        follow = QtWidgets.QCheckBox("Follow rocket")
        bind_check(self.follow, follow, on_user=self._invalidate)
        self.pad_button = QtWidgets.QPushButton("Set pad here...")
        self.pad_button.clicked.connect(self.toggle_set_pad)
        auto_pad = QtWidgets.QPushButton("Auto pad")
        auto_pad.clicked.connect(self.clear_manual_pad)
        clear = QtWidgets.QPushButton("Clear track")
        clear.clicked.connect(self.clear_track)
        online = QtWidgets.QCheckBox("OSM tiles")
        bind_check(self.online, online, on_user=self._online_changed)
        precache = QtWidgets.QPushButton("Pre-cache area...")
        precache.clicked.connect(self.precache_dialog)
        for widget in (centre, auto, follow, self.pad_button, auto_pad, clear, online, precache):
            bar.addWidget(widget)
        bar.addStretch(1)
        bar_widget = QtWidgets.QWidget()
        bar_widget.setLayout(bar)
        bar_scroll = QtWidgets.QScrollArea()
        bar_scroll.setWidget(bar_widget)
        bar_scroll.setWidgetResizable(True)
        bar_scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        bar_scroll.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        bar_scroll.setFixedHeight(bar_widget.sizeHint().height() + 14)
        layout.addWidget(bar_scroll)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        splitter.setChildrenCollapsible(False)
        self.plot = pg.PlotWidget()
        self.plot.setBackground("w")
        item = self.plot.getPlotItem()
        item.setAspectLocked(True)
        light_axes(item)
        item.showGrid(x=True, y=True, alpha=0.25)
        item.setLabel("bottom", "East of pad [m]")
        item.setLabel("left", "North of pad [m]")
        item.hideButtons()
        item.setMenuEnabled(False)
        self.view = item.getViewBox()
        self.view.sigRangeChanged.connect(self._range_changed)
        self.view.sigRangeChangedManually.connect(self._manual_range)
        self.plot.scene().sigMouseClicked.connect(self._clicked)
        self._rings = pg.PlotCurveItem(pen=pg.mkPen("#c06060", width=1, style=QtCore.Qt.PenStyle.DashLine),
                                       connect="finite")
        item.addItem(self._rings)
        self._ring_labels = []
        self._tracks = {}
        for name in TRACK_ORDER:
            color, dashed, symbol = TRACK_STYLE[name]
            line = pg.PlotDataItem(pen=pg.mkPen(color, width=2 if name == "kf" else 1.5,
                                                style=QtCore.Qt.PenStyle.DashLine if dashed
                                                else QtCore.Qt.PenStyle.SolidLine))
            marker = pg.ScatterPlotItem(size=11, symbol=symbol, pen=pg.mkPen(color, width=2),
                                        brush=pg.mkBrush(color if name == "kf" else None))
            label = pg.TextItem("", color=color, anchor=(0, 1))
            for graphic in (line, marker, label):
                item.addItem(graphic)
            self._tracks[name] = (line, marker, label)
        self._site = pg.ScatterPlotItem(size=12, symbol="d", pen=pg.mkPen("#c06000", width=2), brush=None)
        self._site_label = pg.TextItem("Witness pad", color="#c06000", anchor=(0, 0.5))
        self._pad = pg.ScatterPlotItem([0.0], [0.0], size=16, symbol="t1", pen=pg.mkPen("w"),
                                       brush=pg.mkBrush(PAD_COLOR))
        self._pad_label = pg.TextItem("PAD", color=PAD_COLOR, anchor=(0.5, 0))
        for graphic in (self._site, self._site_label, self._pad, self._pad_label):
            item.addItem(graphic)
        self._site.setVisible(False)
        self._site_label.setVisible(False)
        self.scale_bar = None
        # Screen-fixed overlays: north arrow, warnings, attribution.
        self.north_arrow = QtWidgets.QLabel("▲", self.plot)
        self.north_label = QtWidgets.QLabel("N", self.plot)
        for label in (self.north_arrow, self.north_label):
            label.setStyleSheet("color: #171717; font-weight: bold; font-size: 14pt; background: transparent;")
            label.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.warning = QtWidgets.QLabel(self.plot)
        self.warning.setWordWrap(True)
        self.warning.setStyleSheet(f"color: {WARN}; font-weight: bold; background: rgba(255,255,255,170);")
        self.warning.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.attribution = QtWidgets.QLabel(ATTRIBUTION, self.plot)
        self.attribution.setStyleSheet("color: #333; background: rgba(255,255,255,170); font-size: 8pt;")
        self.attribution.hide()
        self.plot.installEventFilter(self)
        splitter.addWidget(self.plot)

        rows = [("pad", "Pad"), ("pad_pos", "Pad position"), ("pad_alt", "Pad altitude"), None,
                ("state", "Flight state")]
        for name in ("kf", "wgps", "fw", "aim"):
            rows += [None, (name, "● " + SOURCE_LABELS[name])]
        rows += [None, ("tiles", "Map tiles")]
        panel, self.values, self.labels = readout_panel(rows)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(panel)
        splitter.addWidget(scroll)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        layout.addWidget(splitter, 1)
        escape = QtGui.QShortcut(QtGui.QKeySequence("Escape"), self)
        escape.setContext(QtCore.Qt.ShortcutContext.WidgetWithChildrenShortcut)
        escape.activated.connect(lambda: self.toggle_set_pad() if self.set_pad_mode else None)

    def eventFilter(self, obj, event):  # noqa: N802 - Qt API: keep overlays placed
        if obj is self.plot and event.type() == QtCore.QEvent.Type.Resize:
            self._place_overlays()
            if self.auto_range.get():
                self.auto_radius = None      # refit the range to the new size
                self._invalidate()
        return super().eventFilter(obj, event)

    def _place_overlays(self):
        w, h = self.plot.width(), self.plot.height()
        self.north_arrow.adjustSize()
        self.north_label.adjustSize()
        self.north_arrow.move(w - 40, 8)
        self.north_label.move(w - 38, 8 + self.north_arrow.height() - 4)
        self.warning.setGeometry(70, 6, max(80, w - 130), 48)
        self.attribution.adjustSize()
        self.attribution.move(60, h - self.attribution.height() - 30)

    def _set(self, key, text, color=TEXT):
        set_colored(self.values, self.labels, key, text, color)

    # --------------------------------------------------------- geometry
    def _ref_lat(self):
        ref = self.data.reference()
        return ref[0] if ref else 0.0

    def _view_pixels(self):
        width = self.view.width() if self.view.width() > 10 else DEFAULT_VIEW_PX
        height = self.view.height() if self.view.height() > 10 else DEFAULT_VIEW_PX * 0.75
        return width, height

    @property
    def pan(self):
        (x0, x1), (y0, y1) = self.view.viewRange()
        return ((x0 + x1) / 2, (y0 + y1) / 2)

    def mpp(self):
        """Metres per screen pixel of the current view."""
        (x0, x1), _ = self.view.viewRange()
        return (x1 - x0) / self._view_pixels()[0]

    def _set_view(self, centre, mpp):
        w, h = self._view_pixels()
        self._applying_range = True
        try:
            self.view.setRange(xRange=(centre[0] - w * mpp / 2, centre[0] + w * mpp / 2),
                               yRange=(centre[1] - h * mpp / 2, centre[1] + h * mpp / 2), padding=0)
        finally:
            self._applying_range = False
        self._range_changed()

    def set_zoom(self, zoom):
        self.zoom = max(MIN_MAP_ZOOM, min(MAX_MAP_ZOOM, int(zoom)))
        self._set_view(self.pan if self.last_snapshot is not None else (0.0, 0.0),
                       metres_per_pixel(self._ref_lat(), self.zoom))

    def _range_changed(self, *_):
        """User zoom/pan: pick the tile zoom matching the view's metres/pixel."""
        base = metres_per_pixel(self._ref_lat(), 0)
        mpp = max(1e-6, self.mpp())
        self.zoom = max(MIN_MAP_ZOOM, min(MAX_MAP_ZOOM, int(round(math.log2(base / mpp)))))
        self.zoom_label.setText(f"zoom {self.zoom}" + (f", auto {format_distance(self.auto_radius)}"
                                                     if self.auto_range.get() and self.auto_radius else ""))
        self._tiles_dirty = True
        self._invalidate()

    def _manual_range(self, *_):
        """Mouse wheel / drag on the map: the user takes over the range."""
        if self.auto_range.get():
            self.auto_range.set(False)
            self._invalidate()

    def _manual_zoom(self, zoom):
        self.auto_range.set(False)
        self.set_zoom(zoom)

    def apply_auto_range(self, snap, now=None):
        """Fit the view to the farthest rocket (centred on the pad, or the rocket when following)."""
        radius = auto_range_radius(snap)
        centre = (0.0, 0.0)
        if self.follow.get():
            primary = self.primary(snap, now)
            if primary is not None:
                centre = (primary.east, primary.north)
        w, h = self._view_pixels()
        mpp = 2.0 * radius / max(1.0, min(w, h))
        if (radius != self.auto_radius or abs(centre[0] - self.pan[0]) > 1e-6
                or abs(centre[1] - self.pan[1]) > 1e-6 or abs(mpp - self.mpp()) > 0.01 * mpp):
            self.auto_radius = radius
            self._set_view(centre, mpp)

    def centre(self):
        self.follow.set(False)
        self._set_view((0.0, 0.0), self.mpp())

    def enu_at_scene(self, scene_pos):
        point = self.view.mapSceneToView(scene_pos)
        return point.x(), point.y()

    # ---------------------------------------------------------- actions
    def clear_track(self):
        self.data.clear()
        self._invalidate()

    def toggle_set_pad(self):
        self.set_pad_mode = not self.set_pad_mode
        self.pad_button.setText("Click map (Esc)" if self.set_pad_mode else "Set pad here...")
        self.plot.setCursor(QtCore.Qt.CursorShape.CrossCursor if self.set_pad_mode
                            else QtCore.Qt.CursorShape.ArrowCursor)
        self._invalidate()

    def clear_manual_pad(self):
        self.data.pad.clear_manual()
        self._invalidate()

    def _online_changed(self):
        self.tiles.set_online(bool(self.online.get()))
        self._tiles_dirty = True
        self._invalidate()

    def _clicked(self, event):
        if not self.set_pad_mode or event.button() != QtCore.Qt.MouseButton.LeftButton:
            return
        self.set_pad_at_enu(*self.enu_at_scene(event.scenePos()))
        event.accept()

    def set_pad_at_enu(self, east, north):
        """Manual pad at this map point (ENU metres from the current reference)."""
        ref = self.data.reference()
        if ref is None:
            inform(self, "Set pad", "No position reference yet: wait for a GPS fix first.")
        else:
            lat, lon, _ = enu_to_geodetic(east, north, 0.0, ref[0], ref[1], 0.0)
            self.data.pad.set_manual(lat, lon)
            self._set_view((0.0, 0.0), self.mpp())
        if self.set_pad_mode:
            self.toggle_set_pad()
        self._tiles_dirty = True
        self._invalidate()

    def precache_dialog(self):
        ref = self.data.reference()
        if ref is None:
            inform(self, "Pre-cache map", "The pad position is not known yet. Wait for a pad source "
                   "(Featherweight GS, Aperture GPS) or use Set pad here.")
            return False
        if not self.online.get():
            inform(self, "Pre-cache map", "Turn on OSM tiles first (needs internet).")
            return False
        radius, zooms = 2000.0, range(12, 18)
        plan = precache_plan(ref[0], ref[1], radius, zooms)
        if len(plan) > MAX_PRECACHE_TILES:
            radius, plan = 1500.0, precache_plan(ref[0], ref[1], 1500.0, zooms)
        if not ask_yes_no(
                self, "Pre-cache map",
                f"Download up to {min(len(plan), MAX_PRECACHE_TILES)} OpenStreetMap tiles covering "
                f"{radius / 1000:.1f} km around the pad (zoom {zooms.start}-{zooms.stop - 1}) into "
                f"{self.tiles.root}?\nAlready-cached tiles are skipped. Downloads run slowly in the "
                "background (tile usage policy)."):
            return False
        queued = self.tiles.precache(plan)
        self._set("tiles", f"pre-cache: {queued} tiles queued")
        return True

    # ----------------------------------------------------------- redraw
    def _invalidate(self):
        self._drawn_key = None

    def _tick(self):
        if self.isVisible():
            self.refresh()

    @staticmethod
    def primary(snap, now=None):
        """Freshest-preferred rocket point: Kalman, Witness GPS, Featherweight, then AIM."""
        now = time.monotonic() if now is None else now
        latest = snap["latest"]
        for name in ("kf", "wgps", "fw", "aim"):
            item = latest.get(name)
            if item is not None and now - item["point"].t <= STALE_S:
                return item["point"]
        items = [latest[name]["point"] for name in ("kf", "wgps", "fw", "aim") if name in latest]
        return max(items, key=lambda p: p.t) if items else None

    def refresh(self, now=None):
        now = time.monotonic() if now is None else now
        completed = self.tiles.drain_completed()
        if completed:
            self._tiles_dirty = True
            for key in completed:
                stale = self._tile_items.pop(key, None)   # refreshed on disk: reload
                if stale is not None:
                    self.plot.removeItem(stale)
        key = (self.data.version, self.data.pad.version, self.follow.get(), self.online.get(), self.set_pad_mode,
               self.auto_range.get())
        if key != self._drawn_key:
            snap = self.data.snapshot()
            self.last_snapshot = snap
            if self.auto_range.get():
                self.apply_auto_range(snap, now)
            elif self.follow.get():
                primary = self.primary(snap, now)
                if primary is not None and (abs(primary.east - self.pan[0]) > 1e-6
                                            or abs(primary.north - self.pan[1]) > 1e-6):
                    self._set_view((primary.east, primary.north), self.mpp())
            self._drawn_key = key
            self._draw(snap)
        if self._tiles_dirty:
            self._tiles_dirty = False
            self._update_tiles()
        if self.last_snapshot is not None and now - self._readout_at >= READOUT_MS / 1000:
            self._readout_at = now
            self._readouts(self.last_snapshot, now)

    def _draw(self, snap):
        self.redraws += 1
        ref = snap["reference"]
        if ref is not None and self._tile_ref != tuple(ref[:2]):
            self._clear_tiles()
            self._tile_ref = tuple(ref[:2])
            self._tiles_dirty = True
        self._draw_rings()
        site = self.data.pad.witness_launch_site()
        show_site = site is not None and ref is not None and snap["pad"] is not None and snap["pad"].source != "witness"
        if show_site:
            se, sn, _ = geodetic_to_enu(site.lat, site.lon, 0.0, ref[0], ref[1], 0.0)
            self._site.setData([se], [sn])
            self._site_label.setPos(se + self.mpp() * 8, sn)
        self._site.setVisible(show_site)
        self._site_label.setVisible(show_site)
        for name, (line, marker, label) in self._tracks.items():
            points = snap["tracks"].get(name, [])
            if not points:
                line.setData([], [])
                marker.setData([], [])
                label.setText("")
                continue
            east = np.fromiter((p.east for p in points), float, len(points))
            north = np.fromiter((p.north for p in points), float, len(points))
            line.setData(east, north)
            if name == "wgps":
                marker.setData(east[-60:], north[-60:], size=[5] * (min(60, len(points)) - 1) + [11])
            else:
                marker.setData([east[-1]], [north[-1]])
            agl = points[-1].agl
            label.setText(TRACK_TAGS[name] + (f" {agl:.0f} m" if agl is not None else ""))
            label.setPos(east[-1], north[-1])
        self._pad_label.setPos(0.0, 0.0)
        if ref is None:
            warning = ("PAD POSITION UNKNOWN - grid is relative to the Witness Kalman origin; "
                       "GPS tracks appear once a pad source or fix arrives.")
        elif snap["pad"] is None:
            warning = f"No pad source yet - centred on {ref[2]}"
        else:
            warning = ""
        if self.set_pad_mode:
            warning = (warning + "\n" if warning else "") + "Click the pad location (Esc cancels)"
        self.warning.setText(warning)
        self.warning.setVisible(bool(warning))
        self._place_overlays()

    def _draw_rings(self):
        (x0, x1), (y0, y1) = self.view.viewRange()
        span = min(x1 - x0, y1 - y0) / 2
        step = nice_ring_step(max(span, 1.0), 3)
        far = max(abs(x0), abs(x1), abs(y0), abs(y1)) * 1.5
        theta = np.linspace(0, 2 * math.pi, 97)
        xs, ys = [], []
        for label in self._ring_labels:
            self.plot.removeItem(label)
        self._ring_labels = []
        for i in range(1, 9):
            radius = i * step
            if radius > far:
                break
            xs += [*(radius * np.cos(theta)), math.nan]
            ys += [*(radius * np.sin(theta)), math.nan]
            text = pg.TextItem(format_distance(radius), color="#a04040", anchor=(0, 1))
            text.setPos(radius * 0.7071, radius * 0.7071)
            self.plot.addItem(text)
            self._ring_labels.append(text)
        self._rings.setData(np.array(xs, float), np.array(ys, float))
        self._ring_step = step
        self._update_scale_bar()

    def _update_scale_bar(self):
        length = nice_ring_step(120 * max(self.mpp(), 1e-6), 1)
        if self.scale_bar is None:
            self.scale_bar = pg.ScaleBar(size=length, suffix="m", offset=(-14, -14),
                                         pen=pg.mkPen("#171717"), brush=pg.mkBrush("#171717"))
            self.scale_bar.setParentItem(self.view)
            self.scale_bar.length = None
        if self.scale_bar.length != length:
            self.scale_bar.length = length
            self.scale_bar.size = length
            self.scale_bar.text.setText(format_distance(length))
            self.scale_bar.updateBar()

    # ------------------------------------------------------------- tiles
    def _clear_tiles(self):
        for item in self._tile_items.values():
            self.plot.removeItem(item)
        self._tile_items.clear()
        self.tiles_drawn = 0

    def visible_tiles(self):
        """Tile keys (z, x, y) covering the current view at the current zoom."""
        ref = self.data.reference()
        if ref is None:
            return []
        z = self.zoom
        rx, ry = lat_lon_to_world_px(ref[0], ref[1], z)
        mpp = metres_per_pixel(ref[0], z)
        (x0, x1), (y0, y1) = self.view.viewRange()
        left, right = rx + x0 / mpp, rx + x1 / mpp
        top, bottom = ry - y1 / mpp, ry - y0 / mpp
        n = 1 << z
        keys = []
        for ty in range(max(0, int(top // TILE_SIZE)), min(n - 1, int(bottom // TILE_SIZE)) + 1):
            for tx in range(int(left // TILE_SIZE), int(right // TILE_SIZE) + 1):
                keys.append((z, tx % n, ty, tx))
        return keys[:MAX_VISIBLE_TILES]

    def _update_tiles(self):
        ref = self.data.reference()
        visible = self.visible_tiles() if ref is not None else []
        wanted = {key[:3] for key in visible}
        for key, item in self._tile_items.items():
            item.setVisible(key in wanted)
        for z, x, y, raw_x in visible:
            key = (z, x, y)
            if key in self._tile_items or key in self._loading:
                if key in self._tile_items:
                    self._tile_items[key] = self._tile_items.pop(key)   # LRU touch
                continue
            path = self.tiles.request(*key) if self.online.get() else self.tiles.cached(*key)
            if path is None:
                continue
            self._loading.add(key)
            run_in_thread(load_tile_rgba, path,
                          on_done=lambda array, key=key, raw_x=raw_x, ref=tuple(ref[:2]):
                              self._tile_loaded(key, raw_x, ref, array),
                          on_error=lambda _exc, key=key: self._loading.discard(key))
        self._count_tiles()

    def _tile_loaded(self, key, raw_x, ref, array):
        self._loading.discard(key)
        if ref != self._tile_ref or key in self._tile_items:
            return
        item = pg.ImageItem(array)
        east, north, size = tile_rect_enu(key[0], raw_x, key[2], *ref)
        item.setRect(QtCore.QRectF(east, north, size, size))
        item.setZValue(-100)
        self.plot.addItem(item)
        self._tile_items[key] = item
        while len(self._tile_items) > TILE_ITEMS:
            old = next(iter(self._tile_items))
            self.plot.removeItem(self._tile_items.pop(old))
        wanted = {k[:3] for k in self.visible_tiles()}
        item.setVisible(key in wanted)
        self._count_tiles()

    def _count_tiles(self):
        self.tiles_drawn = sum(item.isVisible() for item in self._tile_items.values())
        self.attribution.setVisible(self.tiles_drawn > 0)
        self._place_overlays()

    # ---------------------------------------------------------- readouts
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
        for name in ("kf", "wgps", "fw", "aim"):
            item = snap["latest"].get(name)
            if item is None:
                self._set(name, "no data", MUTED)
                continue
            p = item["point"]
            age = now - p.t
            if name == "aim":
                agl = f"GPS alt {p.alt_raw:.1f} (AIM units)" if p.alt_raw is not None else "GPS alt --"
            else:
                agl = f"{p.agl:.1f} m AGL" if p.agl is not None else "alt AGL --"
            text = (f"{format_distance(item['range'])} @ {item['bearing']:.0f}° "
                    f"{compass_point(item['bearing'])}\n{agl}   age {age:.1f} s")
            if name == "kf" and snap["reference"] is not None and not snap["kf_placed"]:
                text += "\norigin assumed at pad"
            self._set(name, text, BAD if age > STALE_S else TEXT)
        self._set("tiles", self.tiles.status() + (f", {self.tiles_drawn} shown" if self.tiles_drawn else ""))

    def texts(self):
        """Visible map labels (tests and diagnostics)."""
        out = [self.north_label.text(), self._pad_label.textItem.toPlainText()]
        out += [label.textItem.toPlainText() for _line, _marker, label in self._tracks.values()]
        out += [label.textItem.toPlainText() for label in self._ring_labels]
        if self.attribution.isVisibleTo(self):
            out.append(self.attribution.text())
        if self.warning.isVisibleTo(self):
            out.append(self.warning.text())
        return [text for text in out if text]

    def close(self):
        self._timer.stop()
        self._clear_tiles()
        if self._own_tiles:
            self.tiles.close()
        return super().close()


# Tk-era names.
OrientationTab = OrientationPanel
MapTab = MapPanel


# ------------------------------------------------------------------ window

class VisualizerPanel(QtWidgets.QWidget):
    """Orientation (left) and map (right) side by side: Horizon's "Map &
    Attitude" tab, and the body of the pop-out VisualizerWindow."""

    def __init__(self, parent=None, *, live=None, aim=None, map_data=None, tiles=None, use_gl=None,
                 on_pop_out=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        if on_pop_out is not None:
            bar = QtWidgets.QHBoxLayout()
            hint = QtWidgets.QLabel("Map of every rocket relative to the pad (right) and the "
                                    "Witness / AIM attitude (left). Drag the divider to resize.")
            hint.setStyleSheet(f"color: {MUTED};")
            pop = QtWidgets.QPushButton("Open in separate window (F9)")
            pop.clicked.connect(on_pop_out)
            bar.addWidget(hint, 1)
            bar.addWidget(pop)
            layout.addLayout(bar)
        self.splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        self.splitter.setChildrenCollapsible(False)
        self.orientation = OrientationPanel(live=live, aim=aim, use_gl=use_gl)
        self.map = MapPanel(data=map_data, tiles=tiles)
        self.orientation.setMinimumWidth(260)
        self.map.setMinimumWidth(300)
        self.splitter.addWidget(self.orientation)
        self.splitter.addWidget(self.map)
        self.splitter.setStretchFactor(0, 2)
        self.splitter.setStretchFactor(1, 3)
        layout.addWidget(self.splitter, 1)
        self.splitter.setSizes([400, 600])
        self._closed = False

    def close(self):
        if not self._closed:
            self._closed = True
            self.orientation.close()
            self.map.close()
        return super().close()


class VisualizerWindow(QtWidgets.QWidget):
    """Top-level window: orientation (left) and map (right) side by side."""

    closed = QtCore.pyqtSignal()

    def __init__(self, parent=None, *, live=None, aim=None, map_data=None, tiles=None, use_gl=None):
        super().__init__(parent, QtCore.Qt.WindowType.Window)
        self.setWindowTitle("Horizon Visualizer")
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_DeleteOnClose, False)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.panel = VisualizerPanel(live=live, aim=aim, map_data=map_data, tiles=tiles, use_gl=use_gl)
        self.splitter, self.orientation, self.map = self.panel.splitter, self.panel.orientation, self.panel.map
        layout.addWidget(self.panel)
        self._closed = False
        screen = QtGui.QGuiApplication.primaryScreen()
        area = screen.availableGeometry() if screen is not None else QtCore.QRect(0, 0, 1400, 900)
        self.resize(min(1400, area.width() - 60), min(820, area.height() - 120))
        self.splitter.setSizes([self.width() * 2 // 5, self.width() * 3 // 5])
        close = QtGui.QShortcut(QtGui.QKeySequence("Ctrl+W"), self)
        close.activated.connect(self.close)

    def is_open(self):
        return not self._closed

    def closeEvent(self, event):  # noqa: N802 - Qt API
        self._shutdown()
        super().closeEvent(event)

    def _shutdown(self):
        if self._closed:
            return
        self._closed = True
        self.panel.close()
        self.closed.emit()

    def close(self):
        self._shutdown()
        return super().close()
