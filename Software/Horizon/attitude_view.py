"""Horizon Attitude tab (Qt): 3D rocket, readouts and a pyqtgraph strip chart.

Live input is the Witness decoded feed (the records Flight View receives);
0xE0 debug quaternions are preferred while fresh, else the 1 Hz 0x02 state
quaternion. Offline, load a converter.py CSV or a HITL/generic attitude CSV
(see attitude_data.load_attitude_csv) and replay it with play/pause/scrub
(Space toggles play). Math and conventions live in attitude_math.py, geometry
in attitude_scene.py.

RocketScene draws the rocket with pyqtgraph.opengl (drag to orbit, wheel to
zoom). Where OpenGL is unavailable (headless tests, some remote desktops) it
falls back to an orthographic 2D pyqtgraph projection of the same wireframe
(drag to orbit), so nothing crashes and the views stay usable.
"""

from __future__ import annotations

import math
import os
import time

import numpy as np
from PyQt6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

import attitude_math as am
from attitude_data import AttitudeTrack, LiveAttitude, load_attitude_csv, state_name
from attitude_scene import (  # noqa: F401 - re-exported
    AIM_ALPHA, AIM_COLOR, AXIS_COLORS, BAD, BG, CONVENTIONS, FIN_FILL, FIN_ROLES, GHOST, GRID, MARK, MUTED, OK, Q_COLORS,
    ROCKET_FAR, ROCKET_NEAR, TEXT, VIEWS, WARN, body_axes_world, chart_series, fmt, ground_grid,
    projector, rocket_segments, rocket_world,
)
from observable import Cell
from qt_common import message_error
from qt_widgets import light_axes, mono_font

STALE_S = 3.0
LIVE_WINDOWS = (30, 60, 120, 300, 600)
SPEEDS = ("0.25", "0.5", "1", "2", "5", "10")
SCRUB_STEPS = 10000

_GL_STATE = None


def gl_available() -> bool:
    """True when a working OpenGL context can be created (checked once)."""
    global _GL_STATE
    if _GL_STATE is not None:
        return _GL_STATE
    _GL_STATE = False
    if os.environ.get("HORIZON_NO_GL"):
        return False
    app = QtWidgets.QApplication.instance()
    if app is None or app.platformName() in ("offscreen", "minimal", "vnc"):
        return False
    try:
        import pyqtgraph.opengl  # noqa: F401
        from PyQt6.QtGui import QOffscreenSurface, QOpenGLContext
        context = QOpenGLContext()
        if not context.create():
            return False
        surface = QOffscreenSurface()
        surface.setFormat(context.format())
        surface.create()
        ok = context.makeCurrent(surface)
        if ok:
            context.doneCurrent()
        _GL_STATE = bool(ok)
    except Exception:  # noqa: BLE001 - any GL failure means "use the 2D view"
        _GL_STATE = False
    return _GL_STATE


def _faded(color, alpha=1.0):
    """QColor with alpha (2D drawing)."""
    c = QtGui.QColor(color)
    c.setAlphaF(max(0.0, min(1.0, alpha)))
    return c


def _rgba(color, alpha=1.0):
    c = QtGui.QColor(color)
    return (c.redF(), c.greenF(), c.blueF(), alpha)


class _Overlay(QtWidgets.QLabel):
    """Centered message over the scene (title + detail)."""

    def __init__(self, parent):
        super().__init__(parent)
        self.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignTop)
        self.setWordWrap(True)
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.setStyleSheet("background: transparent;")
        self.hide()

    def show_message(self, title, detail):
        if not title:
            self.hide()
            return
        self.setText(f"<div style='color:{BAD}; font-size:16pt; font-weight:bold'>{title}</div>"
                     f"<div style='color:{MUTED}; font-size:9pt'>{detail}</div>")
        self.show()
        self.raise_()


class RocketScene(QtWidgets.QWidget):
    """The rocket at a quaternion, with world axes, ground grid and body triad."""

    viewChanged = QtCore.pyqtSignal(float, float)

    def __init__(self, parent=None, *, use_gl=None):
        super().__init__(parent)
        self.view_az, self.view_el = VIEWS["Iso"]
        self.use_gl = gl_available() if use_gl is None else bool(use_gl)
        self.message = ("", "")
        self.legend_text = ""
        self.state = None
        self._drag = None
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.setMinimumSize(200, 180)
        if self.use_gl:
            try:
                self._build_gl(layout)
            except Exception:  # noqa: BLE001 - fall back if the GL widget fails
                self.use_gl = False
        if not self.use_gl:
            self._build_2d(layout)
        self.caption = QtWidgets.QLabel(self)
        self.caption.setStyleSheet(f"color: {MUTED}; background: transparent;")
        self.caption.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.overlay = _Overlay(self)
        self.set_view_angles(*VIEWS["Iso"])

    # ----------------------------------------------------------------- GL
    def _build_gl(self, layout):
        import pyqtgraph.opengl as gl
        view = gl.GLViewWidget()
        view.setBackgroundColor(BG)
        self.gl_view = view
        grid = gl.GLGridItem(color=QtGui.QColor("#b8b8b8"))
        grid.setSize(2, 2)
        grid.setSpacing(0.25, 0.25)
        grid.translate(0, 0, -0.75)
        view.addItem(grid)
        world = np.array([[0, 0, 0], [0.95, 0, 0], [0, 0, 0], [0, 0.95, 0], [0, 0, 0], [0, 0, 0.95]])
        view.addItem(gl.GLLinePlotItem(pos=world, mode="lines", color=_rgba("#909090"), width=1.5,
                                       antialias=True))
        for label, pos in (("E", (1.05, 0, 0)), ("N", (0, 1.05, 0)), ("U", (0, 0, 1.05))):
            view.addItem(gl.GLTextItem(pos=pos, text=label, color=QtGui.QColor(MUTED)))
        # Translucent blending so a stale sample can be drawn faded (alpha < 1).
        self._gl_body = gl.GLLinePlotItem(mode="lines", width=2, antialias=True, glOptions="translucent")
        self._gl_mark = gl.GLLinePlotItem(mode="lines", width=4, antialias=True, color=_rgba(MARK),
                                          glOptions="translucent")
        self._gl_ghost = gl.GLLinePlotItem(mode="lines", width=1.5, antialias=True, color=_rgba(GHOST, 0.8))
        self._gl_fins = [gl.GLMeshItem(smooth=False, drawEdges=True, glOptions="translucent",
                                       edgeColor=_rgba(MARK if role == "mark" else ROCKET_NEAR),
                                       color=_rgba(FIN_FILL[role])) for role in FIN_ROLES]
        self._gl_axes = [gl.GLLinePlotItem(mode="lines", width=3, color=_rgba(color), glOptions="translucent")
                         for color in AXIS_COLORS]
        # AIM Xtra overlay: the same rocket, translucent, in AIM_COLOR.
        self._gl_aim = gl.GLLinePlotItem(mode="lines", width=2, antialias=True,
                                         color=_rgba(AIM_COLOR, min(1.0, AIM_ALPHA * 2)), glOptions="translucent")
        self._gl_aim_fins = [gl.GLMeshItem(smooth=False, drawEdges=True, glOptions="translucent",
                                           edgeColor=_rgba(AIM_COLOR, min(1.0, AIM_ALPHA * 2)),
                                           color=_rgba(AIM_COLOR, AIM_ALPHA)) for _role in FIN_ROLES]
        for item in (self._gl_body, self._gl_mark, self._gl_ghost, *self._gl_fins, *self._gl_axes,
                     self._gl_aim, *self._gl_aim_fins):
            view.addItem(item)
            item.setVisible(False)
        layout.addWidget(view)

    def _gl_show(self, q, ghost, overlay=None, alpha=1.0, overlay_alpha=AIM_ALPHA):
        body_color = _rgba(ROCKET_NEAR, alpha)
        for item in (self._gl_body, self._gl_mark, *self._gl_fins, *self._gl_axes, self._gl_ghost,
                     self._gl_aim, *self._gl_aim_fins):
            item.setVisible(False)
        if overlay is not None:
            segments, _roles, fins = rocket_world(overlay)
            edge = _rgba(AIM_COLOR, min(1.0, overlay_alpha * 2))
            self._gl_aim.setData(pos=segments.reshape(-1, 3), color=edge)
            self._gl_aim.setVisible(True)
            for item, quad in zip(self._gl_aim_fins, fins):
                item.opts["edgeColor"] = edge
                item.setColor(_rgba(AIM_COLOR, overlay_alpha))
                item.setMeshData(vertexes=quad, faces=np.array([[0, 1, 2], [0, 2, 3]]))
                item.setVisible(True)
        if ghost is not None:
            segments, _roles, _fins = rocket_world(ghost)
            self._gl_ghost.setData(pos=segments.reshape(-1, 3))
            self._gl_ghost.setVisible(True)
        if q is None:
            return
        segments, roles, fins = rocket_world(q)
        self._gl_body.setData(pos=segments[roles != "mark"].reshape(-1, 3), color=body_color)
        self._gl_mark.setData(pos=segments[roles == "mark"].reshape(-1, 3), color=_rgba(MARK, alpha))
        for item, quad, role in zip(self._gl_fins, fins, FIN_ROLES):
            item.opts["edgeColor"] = _rgba(MARK if role == "mark" else ROCKET_NEAR, alpha)
            item.setColor(_rgba(FIN_FILL[role], alpha))
            item.setMeshData(vertexes=quad, faces=np.array([[0, 1, 2], [0, 2, 3]]))
        for item, axis, color in zip(self._gl_axes, body_axes_world(q), AXIS_COLORS):
            item.setData(pos=np.array([[0, 0, 0], axis]), color=_rgba(color, alpha))
        for item in (self._gl_body, self._gl_mark, *self._gl_fins, *self._gl_axes):
            item.setVisible(True)

    # ----------------------------------------------------------------- 2D
    def _build_2d(self, layout):
        self.plot = pg.PlotWidget()
        self.plot.setBackground(BG)
        item = self.plot.getPlotItem()
        item.hideAxis("left")
        item.hideAxis("bottom")
        item.hideButtons()
        item.setMenuEnabled(False)
        item.setMouseEnabled(x=False, y=False)
        item.setAspectLocked(True)
        item.setRange(xRange=(-1.25, 1.25), yRange=(-1.25, 1.25), padding=0)
        self.plot.viewport().installEventFilter(self)
        self._items = []
        layout.addWidget(self.plot)

    def _line(self, points_xy, color, width=1.0, style=QtCore.Qt.PenStyle.SolidLine, pairs=True):
        x, y = points_xy
        curve = pg.PlotCurveItem(x=np.asarray(x, float), y=np.asarray(y, float),
                                 pen=pg.mkPen(color, width=width, style=style),
                                 connect="pairs" if pairs else "all")
        self.plot.addItem(curve)
        self._items.append(curve)
        return curve

    def _text(self, text, pos, color, bold=False, size=9):
        item = pg.TextItem(text, color=color, anchor=(0.5, 0.5))
        font = QtGui.QFont()
        font.setPointSize(size)
        font.setBold(bold)
        item.setFont(font)
        item.setPos(*pos)
        self.plot.addItem(item)
        self._items.append(item)
        return item

    def _draw_2d(self, q, ghost, overlay=None, alpha=1.0, overlay_alpha=AIM_ALPHA):
        for item in self._items:
            self.plot.removeItem(item)
        self._items = []
        project = projector(self.view_az, self.view_el)
        grid = ground_grid()
        gx, gy, _ = project(grid)
        self._line((gx.ravel(), gy.ravel()), GRID)
        for axis, label in (((1, 0, 0), "E"), ((0, 1, 0), "N"), ((0, 0, 1), "U")):
            ax, ay, _ = project(np.array([[0, 0, 0], [0.95 * a for a in axis]]))
            self._line((ax, ay), "#909090", 1.0)
            lx, ly, _ = project(np.array([1.05 * a for a in axis]))
            self._text(label, (float(lx), float(ly)), MUTED, bold=True, size=10)
        if ghost is not None:
            segments, _roles, fins = rocket_world(ghost)
            sx, sy, _ = project(segments)
            self._line((sx.ravel(), sy.ravel()), GHOST, 1.0, QtCore.Qt.PenStyle.DashLine)
            for quad in fins:
                fx, fy, _ = project(np.vstack([quad, quad[:1]]))
                self._line((fx, fy), GHOST, 1.0, QtCore.Qt.PenStyle.DashLine, pairs=False)
        if overlay is not None:
            segments, _roles, fins = rocket_world(overlay)
            line = _faded(AIM_COLOR, min(1.0, overlay_alpha * 2))
            fill = _faded(AIM_COLOR, overlay_alpha)
            sx, sy, _ = project(segments)
            self._line((sx.ravel(), sy.ravel()), line, 2.0)
            for quad in fins:
                fx, fy, _ = project(quad)
                polygon = QtWidgets.QGraphicsPolygonItem(QtGui.QPolygonF(
                    [QtCore.QPointF(float(a), float(b)) for a, b in zip(fx, fy)]))
                polygon.setBrush(pg.mkBrush(fill))
                polygon.setPen(pg.mkPen(line, width=1.0))
                self.plot.addItem(polygon)
                self._items.append(polygon)
        if q is None:
            return
        segments, roles, fins = rocket_world(q)
        sx, sy, depth = project(segments)
        mid_depth = depth.mean(axis=1)
        middle = (mid_depth.min() + mid_depth.max()) / 2
        fin_depth = [project(quad)[2].mean() for quad in fins]
        # Painter's order: far lines, fins far-to-near, near lines, marks.
        far = (roles != "mark") & (mid_depth < middle)
        near = (roles != "mark") & (mid_depth >= middle)
        self._line((sx[far].ravel(), sy[far].ravel()), _faded(ROCKET_FAR, alpha), 1.5)
        for index in np.argsort(fin_depth):
            quad, role = fins[index], FIN_ROLES[index]
            fx, fy, _ = project(np.vstack([quad, quad[:1]]))
            polygon = QtWidgets.QGraphicsPolygonItem(QtGui.QPolygonF(
                [QtCore.QPointF(float(a), float(b)) for a, b in zip(fx, fy)]))
            polygon.setBrush(pg.mkBrush(_faded(FIN_FILL[role], alpha)))
            polygon.setPen(pg.mkPen(_faded(MARK if role == "mark" else ROCKET_NEAR, alpha), width=1.5))
            self.plot.addItem(polygon)
            self._items.append(polygon)
        self._line((sx[near].ravel(), sy[near].ravel()), _faded(ROCKET_NEAR, alpha), 2.0)
        marks = roles == "mark"
        self._line((sx[marks].ravel(), sy[marks].ravel()), _faded(MARK, alpha), 3.0)
        for axis, color, label in zip(body_axes_world(q), AXIS_COLORS, ("xb", "yb", "zb")):
            ax, ay, _ = project(np.array([[0, 0, 0], axis]))
            self._line((ax, ay), _faded(color, alpha), 3.0)
            tip = axis * (1 + 0.08 / max(1e-9, float(np.linalg.norm(axis))))
            tx, ty, _ = project(tip)
            self._text(label, (float(tx), float(ty)), color, bold=True)

    def eventFilter(self, obj, event):  # noqa: N802 - Qt API (2D orbit)
        kind = event.type()
        if kind == QtCore.QEvent.Type.MouseButtonPress and event.button() == QtCore.Qt.MouseButton.LeftButton:
            self._drag = (event.position().x(), event.position().y(), self.view_az, self.view_el)
            return True
        if kind == QtCore.QEvent.Type.MouseMove and self._drag is not None:
            x0, y0, az, el = self._drag
            self.set_view_angles((az - (event.position().x() - x0) * 0.5) % 360.0,
                                 max(-89.0, min(90.0, el + (event.position().y() - y0) * 0.5)))
            return True
        if kind == QtCore.QEvent.Type.MouseButtonRelease:
            self._drag = None
        return super().eventFilter(obj, event)

    # ---------------------------------------------------------------- API
    def set_view_angles(self, az, el):
        self.view_az, self.view_el = float(az), float(el)
        if self.use_gl:
            self.gl_view.setCameraPosition(distance=2.6, elevation=self.view_el, azimuth=90.0 - self.view_az)
        elif self.state is not None:
            self._draw_2d(*self.state)
        self._update_caption()
        self.viewChanged.emit(self.view_az, self.view_el)

    def set_view(self, name):
        self.set_view_angles(*VIEWS[name])

    def show_attitude(self, q, *, ghost=None, overlay=None, message=("", ""), legend="",
                      alpha=1.0, overlay_alpha=AIM_ALPHA):
        """Draw estimate q (None hides it), an optional ghost, an optional
        translucent overlay rocket (AIM Xtra) and a message. alpha and
        overlay_alpha fade the rockets (stale samples are drawn fainter)."""
        self.state = (q, ghost, overlay, alpha, overlay_alpha)
        self.message = message
        self.legend_text = legend
        if self.use_gl:
            self._gl_show(q, ghost, overlay, alpha, overlay_alpha)
        else:
            self._draw_2d(q, ghost, overlay, alpha, overlay_alpha)
        self.overlay.show_message(*message)
        self._update_caption()
        self._place_overlays()

    def _update_caption(self):
        mode = "OpenGL" if self.use_gl else "2D projection (OpenGL unavailable)"
        text = f"camera az {self.view_az:.0f}° el {self.view_el:.0f}° (world ENU, grid 0.25) - {mode}"
        if self.legend_text:
            text += "\n" + self.legend_text
        self.caption.setText(text)
        self.caption.adjustSize()

    def _place_overlays(self):
        self.caption.move(6, max(0, self.height() - self.caption.height() - 4))
        self.overlay.setGeometry(10, 10, max(50, self.width() - 20), max(40, self.height() // 2))

    def resizeEvent(self, event):  # noqa: N802 - Qt API
        super().resizeEvent(event)
        self._place_overlays()

    def item_count(self):
        """Drawn primitives (2D mode) or visible GL items."""
        if self.use_gl:
            return sum(item.visible() for item in self.gl_view.items)
        return len(self._items)

    def texts(self):
        out = [self.caption.text()]
        if self.overlay.isVisibleTo(self):
            out += [self.message[0], self.message[1]]
        if not self.use_gl:
            out += [item.textItem.toPlainText() for item in self._items if isinstance(item, pg.TextItem)]
        return out


def readout_panel(rows, *, big_keys=(), width=None):
    """Grid of title/value labels bound to Cells. Returns (widget, cells, labels)."""
    widget = QtWidgets.QWidget()
    widget.setObjectName("readouts")
    # Light panel whatever the system theme: status colours are tuned for white.
    widget.setStyleSheet(f"#readouts {{ background: {BG}; }} QLabel {{ color: {TEXT}; background: transparent; }}")
    grid = QtWidgets.QGridLayout(widget)
    grid.setContentsMargins(6, 4, 4, 4)
    grid.setVerticalSpacing(2)
    cells, labels = {}, {}
    r = 0
    for item in rows:
        if item is None:
            line = QtWidgets.QFrame()
            line.setFrameShape(QtWidgets.QFrame.Shape.HLine)
            grid.addWidget(line, r, 0, 1, 2)
            r += 1
            continue
        key, title = item
        cell = Cell("--")
        value = QtWidgets.QLabel("--")
        value.setWordWrap(True)
        value.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        if key in big_keys:
            font = value.font()
            font.setPointSize(18)
            font.setBold(True)
            value.setFont(font)
            grid.addWidget(QtWidgets.QLabel(title), r, 0, 1, 2)
            grid.addWidget(value, r + 1, 0, 1, 2)
            r += 2
        else:
            value.setFont(mono_font(10))
            grid.addWidget(QtWidgets.QLabel(title), r, 0, QtCore.Qt.AlignmentFlag.AlignTop)
            grid.addWidget(value, r, 1)
            r += 1
        cell.subscribe(value.setText)
        cells[key], labels[key] = cell, value
    grid.setColumnStretch(1, 1)
    grid.setRowStretch(r, 1)
    return widget, cells, labels


def set_colored(cells, labels, key, text, color=TEXT):
    cells[key].set(text)
    if labels[key].property("fg") != color:
        labels[key].setProperty("fg", color)
        labels[key].setStyleSheet(f"color: {color};")


class AttitudeTab(QtWidgets.QWidget):
    """Attitude visualization; feed it with accept_record() from the Witness feed."""

    def __init__(self, parent=None, *, refresh_ms=100, use_gl=None):
        super().__init__(parent)
        self.refresh_ms = refresh_ms
        self.live = LiveAttitude()
        self.track = None
        self.mode = "live"
        self.play_time = 0.0
        self.playing = False
        self._last_tick = time.monotonic()
        self.manual_offset = None
        self._scrubbing = False
        self._chart_key = None
        self.show_truth = Cell(True)
        self.align = Cell(True)
        self.window = Cell("60")
        self.speed = Cell("1")
        self._build(use_gl)
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(self.refresh_ms)

    # ------------------------------------------------------------- layout
    def _build(self, use_gl):
        from qt_widgets import bind_check, bind_combo
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        bar = QtWidgets.QHBoxLayout()
        self.live_button = QtWidgets.QPushButton("Live feed")
        self.live_button.clicked.connect(self.use_live)
        load = QtWidgets.QPushButton("Load CSV...")
        load.setShortcut(QtGui.QKeySequence("Ctrl+O"))
        load.clicked.connect(self.choose_file)
        truth = QtWidgets.QCheckBox("Truth ghost")
        bind_check(self.show_truth, truth, on_user=self.refresh)
        align = QtWidgets.QCheckBox("Heading-align estimate")
        bind_check(self.align, align, on_user=self.refresh)
        bar_widgets = [self.live_button, load, truth, align]
        for text, slot in (("Align heading here", self.align_here), ("Auto align", self.auto_align),
                           ("Clear", self.clear)):
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(slot)
            bar_widgets.append(button)
        window = QtWidgets.QComboBox()
        bind_combo(self.window, window, [str(v) for v in LIVE_WINDOWS])
        bar_widgets += [QtWidgets.QLabel("Live window s"), window]
        for widget in bar_widgets:
            bar.addWidget(widget)
        bar.addStretch(1)
        layout.addLayout(bar)
        views = QtWidgets.QHBoxLayout()
        views.addWidget(QtWidgets.QLabel("Camera:"))
        for name in VIEWS:
            button = QtWidgets.QPushButton(name)
            button.clicked.connect(lambda _c=False, n=name: self.set_view(n))
            views.addWidget(button)
        hint = QtWidgets.QLabel("(drag the view to orbit)")
        hint.setStyleSheet(f"color: {MUTED};")
        views.addWidget(hint)
        views.addStretch(1)
        conventions = QtWidgets.QPushButton("Conventions...")
        conventions.clicked.connect(self.show_conventions)
        views.addWidget(conventions)
        layout.addLayout(views)

        vertical = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        vertical.setChildrenCollapsible(False)
        top = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        top.setChildrenCollapsible(False)
        self.scene = RocketScene(use_gl=use_gl)
        top.addWidget(self.scene)
        rows = (("mode", "Mode"), ("source", "Source"), ("age", "Sample age"),
                ("state", "Flight state"), ("hitl", "HITL"), ("gps", "GPS lock"), None,
                ("qx", "q x"), ("qy", "q y"), ("qz", "q z"), ("qw", "q w"), ("norm", "|q|"), None,
                ("tilt", "Tilt from vertical"), ("azimuth", "Nose azimuth"), ("spin", "Spin about nose"), None,
                ("yaw", "ZYX yaw"), ("pitch", "ZYX pitch"), ("roll", "ZYX roll"), None,
                ("truth_tilt", "Truth tilt"), ("tilt_err", "Tilt error"), ("att_err", "Attitude error"),
                ("offset", "Heading offset"), None, ("note", "Note"))
        panel, self.values, self.value_labels = readout_panel(rows)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(panel)
        scroll.setMinimumWidth(220)
        top.addWidget(scroll)
        top.setStretchFactor(0, 3)
        top.setStretchFactor(1, 1)
        top.setSizes([750, 300])
        vertical.addWidget(top)

        self.chart = pg.GraphicsLayoutWidget()
        self.chart.setBackground(BG)
        self.chart.setMinimumHeight(140)
        self.q_plot = self.chart.addPlot(row=0, col=0)
        self.tilt_plot = self.chart.addPlot(row=1, col=0)
        self.tilt_plot.setXLink(self.q_plot)
        self.q_plot.setLabel("left", "q")
        self.tilt_plot.setLabel("left", "tilt °")
        self.q_plot.setYRange(-1, 1)
        legend = self.q_plot.addLegend(offset=(-10, 5), colCount=4, labelTextColor=MUTED)
        self.q_curves, self.truth_curves = [], []
        for label, color in zip("xyzw", Q_COLORS):
            self.q_curves.append(self.q_plot.plot(pen=pg.mkPen(color, width=1.5), name=f"q{label}",
                                                  connect="finite"))
            self.truth_curves.append(self.q_plot.plot(pen=pg.mkPen(color, width=1,
                                                                   style=QtCore.Qt.PenStyle.DashLine),
                                                      connect="finite"))
        del legend
        self.tilt_curve = self.tilt_plot.plot(pen=pg.mkPen(TEXT, width=1.5), connect="finite")
        self.truth_tilt_curve = self.tilt_plot.plot(pen=pg.mkPen(GHOST, width=1, style=QtCore.Qt.PenStyle.DashLine),
                                                    connect="finite")
        for plot in (self.q_plot, self.tilt_plot):
            light_axes(plot)
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.setMouseEnabled(x=False, y=False)
            plot.hideButtons()
        self.cursor = pg.InfiniteLine(angle=90, pen=pg.mkPen(BAD))
        self.tilt_plot.addItem(self.cursor)
        vertical.addWidget(self.chart)
        vertical.setStretchFactor(0, 3)
        vertical.setStretchFactor(1, 1)
        layout.addWidget(vertical, 1)

        replay = QtWidgets.QHBoxLayout()
        self.play_button = QtWidgets.QPushButton("Play")
        self.play_button.setToolTip("Play / pause (Space)")
        self.play_button.clicked.connect(self.toggle_play)
        speed = QtWidgets.QComboBox()
        bind_combo(self.speed, speed, SPEEDS)
        self.scale = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.scale.setRange(0, SCRUB_STEPS)
        self.scale.valueChanged.connect(self._scrubbed)
        self.time_label = QtWidgets.QLabel("live")
        self.time_label.setMinimumWidth(200)
        for widget in (self.play_button, QtWidgets.QLabel("Speed"), speed):
            replay.addWidget(widget)
        replay.addWidget(self.scale, 1)
        replay.addWidget(self.time_label)
        layout.addLayout(replay)
        space = QtGui.QShortcut(QtGui.QKeySequence("Space"), self)
        space.setContext(QtCore.Qt.ShortcutContext.WidgetWithChildrenShortcut)
        space.activated.connect(self.toggle_play)
        self._set_replay_enabled(False)

    @property
    def canvas(self):  # Tk-era name for the 3D view
        return self.scene

    def show_conventions(self):
        QtWidgets.QMessageBox.information(self, "Attitude conventions", CONVENTIONS)

    def _set_replay_enabled(self, enabled):
        self.play_button.setEnabled(enabled)
        self.scale.setEnabled(enabled)

    # ------------------------------------------------------------- inputs
    def accept_record(self, record):
        """Witness decoded-feed record. Never raises into the receiver loop."""
        try:
            return self.live.accept_record(record)
        except (TypeError, ValueError, KeyError, AttributeError):
            return False

    def choose_file(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load attitude CSV", "", "CSV (*.csv);;All files (*)")
        if not path:
            return
        try:
            self.load_file(path)
        except (OSError, ValueError, UnicodeDecodeError) as exc:
            message_error(self, "Attitude CSV", str(exc))

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
        self.play_button.setText("Play")
        self.play_time = track.start
        self._chart_key = None
        self._set_replay_enabled(True)
        self._set_scrub(track.start)
        self.refresh()

    def use_live(self):
        self.mode = "live"
        self.playing = False
        self.play_button.setText("Play")
        self._chart_key = None
        self._set_replay_enabled(False)
        self.refresh()

    def clear(self):
        if self.mode == "live":
            self.live.clear()
        self.manual_offset = None
        self._chart_key = None
        self.refresh()

    def set_view(self, name):
        self.scene.set_view(name)
        self.refresh()

    # ------------------------------------------------------------- replay
    def toggle_play(self):
        if self.mode != "file" or self.track is None:
            return
        if not self.playing and self.play_time >= self.track.end:
            self.play_time = self.track.start
        self.playing = not self.playing
        self._last_tick = time.monotonic()
        self.play_button.setText("Pause" if self.playing else "Play")

    def _set_scrub(self, t):
        if self.track is None:
            return
        span = max(self.track.end - self.track.start, 1e-6)
        self._scrubbing = True
        try:
            self.scale.setValue(int(round((t - self.track.start) / span * SCRUB_STEPS)))
        finally:
            self._scrubbing = False

    def _scrubbed(self, value):
        if self.mode != "file" or self._scrubbing or self.track is None:
            return
        span = max(self.track.end - self.track.start, 1e-6)
        self.play_time = self.track.start + value / SCRUB_STEPS * span
        self.refresh()

    def seek(self, t):
        if self.track is None:
            return
        self.play_time = max(self.track.start, min(self.track.end, float(t)))
        self._set_scrub(self.play_time)
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
                    self.play_button.setText("Play")
                self.seek(self.play_time)
            elif self.isVisible():
                self.refresh()
        finally:
            self._last_tick = now

    def close(self):
        self._timer.stop()
        return super().close()

    # ------------------------------------------------------------ heading
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

    # ------------------------------------------------------------ refresh
    def current_sample(self, now=None):
        if self.mode == "file" and self.track is not None:
            return self.track.sample_at(self.play_time)
        return self.live.current(now)

    def refresh(self, now=None):
        now = time.monotonic() if now is None else now
        sample = self.current_sample(now)
        offset, how = self.heading_offset()
        self._readouts(sample, now, offset, how)
        self._draw_scene(sample, offset)
        self._draw_chart(now)
        if self.mode == "file" and self.track is not None:
            self.time_label.setText(f"t = {self.play_time:.2f} s  [{self.track.start:.2f} .. {self.track.end:.2f}]")
        else:
            self.time_label.setText("live")

    def _set(self, key, text, color=TEXT):
        set_colored(self.values, self.value_labels, key, text, color)

    def _readouts(self, s, now, offset, how):
        self._set("mode", f"File: {self.track.name}" if self.mode == "file" and self.track is not None
                  else "Live (Witness feed)")
        for key in ("qx", "qy", "qz", "qw", "norm", "tilt", "azimuth", "spin", "yaw", "pitch", "roll",
                    "truth_tilt", "tilt_err", "att_err", "offset", "note"):
            self._set(key, "--")
        if s is None:
            self._set("source", "--")
            self._set("age", "no attitude samples" if self.mode == "live" else "--")
            for key in ("state", "hitl", "gps"):
                self._set(key, "--")
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
            self._set("age", f"file +{max(0.0, self.play_time - s.time):.2f} s")
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
            self._set("azimuth", fmt(angles["azimuth"]) + (" (~vertical)" if vertical and angles["azimuth"] is not None
                                                          else ""), MUTED if vertical else TEXT)
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

    def _draw_scene(self, s, offset):
        truth = s.truth if s is not None and self.mode == "file" and self.show_truth.get() else None
        if s is None or s.q is None:
            message = ("ATTITUDE NOT AVAILABLE" if s is not None else "NO ATTITUDE DATA",
                       s.reason if s is not None else "waiting for Witness 0x02 / 0xE0 or load a CSV")
            self.scene.show_attitude(None, ghost=truth, message=message)
            return
        q = s.q
        aligned = False
        if offset is not None and self.align.get() and self.mode == "file":
            q = am.apply_heading(q, offset)
            aligned = abs(offset) > 1e-9
        legend = "estimate (black, orange fin = +Y)"
        if truth is not None:
            legend += "   truth (blue dashed)"
        if aligned:
            legend += f"   estimate rotated {offset:+.1f}° about U"
        self.scene.show_attitude(q, ghost=truth, legend=legend)

    def _series_bounds(self, now):
        if self.mode == "file" and self.track is not None:
            return (self.track.samples, self.track.start, max(self.track.end, self.track.start + 1e-6),
                    self.play_time, self.track.max_gap)
        try:
            window = float(self.window.get())
        except ValueError:
            window = 60.0
        return self.live.history_list(), now - window, now, None, 2.5

    def _draw_chart(self, now):
        samples, t0, t1, cursor, max_gap = self._series_bounds(now)
        has_truth = self.mode == "file" and self.track is not None and self.track.has_truth
        key = (self.mode, id(self.track), len(samples), samples[-1].time if samples else None,
               self.window.get(), int(now) if self.mode == "live" else 0)
        if key != self._chart_key:
            self._chart_key = key
            t, q, tilt, tq, tt = chart_series(samples, t0, t1, max_gap, truth=has_truth)
            x = t - now if self.mode == "live" else t
            for i, curve in enumerate(self.q_curves):
                curve.setData(x, q[:, i] if len(q) else q.reshape(-1))
            for i, curve in enumerate(self.truth_curves):
                curve.setData(x, tq[:, i]) if has_truth else curve.setData([], [])
            self.tilt_curve.setData(x, tilt)
            if has_truth:
                self.truth_tilt_curve.setData(x, tt)
            else:
                self.truth_tilt_curve.setData([], [])
            finite = tilt[np.isfinite(tilt)]
            high = 90.0 if not len(finite) or finite.max() <= 90.0 else 180.0
            self.tilt_plot.setYRange(0, high)
            if self.mode == "live":
                self.q_plot.setXRange(t0 - now, 0, padding=0)
                self.tilt_plot.setLabel("bottom", "seconds before now (host time)")
            else:
                self.q_plot.setXRange(t0, t1, padding=0)
                self.tilt_plot.setLabel("bottom", "file time [s]")
        self.cursor.setVisible(cursor is not None)
        if cursor is not None:
            self.cursor.setValue(cursor)

    def chart_point_count(self):
        return sum(len(curve.xData) if curve.xData is not None else 0 for curve in self.q_curves)
