"""Shared PyQt6 / pyqtgraph foundation for Horizon, HITL and witnessOS.

Every GUI in this repository is built on PyQt6 with pyqtgraph for plots.
Import from here instead of configuring Qt or pyqtgraph per tool, so all
windows share one QApplication, one theme and the same plotting defaults.

    from qt_common import ensure_app, make_plot, Throttle, run_in_thread

Rules the tools follow (they are what keeps big logs from freezing the UI):
- Never draw raw millions of points by hand: give pyqtgraph numpy arrays and
  let make_plot()'s clip-to-view + peak downsampling reduce them per pixel.
- Moving a curve in time (a per-file offset) is item.setPos(dx, 0), not a
  rebuild of the data arrays.
- Live data arrives on worker threads; it is queued and drawn by a Throttle
  (a coalescing QTimer, <= ~30 Hz), never one redraw per packet.
- Slow work (file loading, FFTs, tile downloads, IMU synthesis) runs through
  run_in_thread() and reports back on the GUI thread through Qt signals.
- Tests run headless with QT_QPA_PLATFORM=offscreen (set by ensure_app when
  no display is available, or by the test module before importing Qt).
"""

from __future__ import annotations

import os
import sys
import traceback
from typing import Any, Callable

from PyQt6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

APP_NAME = "Witness tools"
_APP = None

# One-time pyqtgraph configuration. OpenGL plotting stays off (driver
# dependent); downsampling + clip-to-view do the heavy lifting instead.
pg.setConfigOptions(antialias=False, useOpenGL=False, imageAxisOrder="row-major",
                    background=None, foreground=None)

PLOT_COLORS = (
    "#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd",
    "#8c564b", "#e377c2", "#17becf", "#bcbd22", "#7f7f7f",
)


def ensure_app(argv: list[str] | None = None) -> QtWidgets.QApplication:
    """Return the process's QApplication, creating it (and the theme) once."""
    app = QtWidgets.QApplication.instance()
    if app is not None:
        return app  # type: ignore[return-value]
    if os.environ.get("QT_QPA_PLATFORM") is None and sys.platform.startswith("linux") \
            and not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        os.environ["QT_QPA_PLATFORM"] = "offscreen"
    QtCore.QCoreApplication.setAttribute(
        QtCore.Qt.ApplicationAttribute.AA_ShareOpenGLContexts, True)
    app = QtWidgets.QApplication(list(argv if argv is not None else sys.argv[:1]))
    # Keep a reference: PyQt destroys a QApplication whose Python wrapper is
    # garbage collected, which crashes callers that discard the return value.
    global _APP
    _APP = app
    app.setApplicationName(APP_NAME)
    app.setStyle("Fusion")
    apply_light_theme(app)
    apply_plot_theme(app)
    return app


def light_palette() -> QtGui.QPalette:
    """White backgrounds and black text, whatever the OS theme."""
    roles = QtGui.QPalette.ColorRole
    palette = QtGui.QPalette()
    colors = {
        roles.Window: "#f5f5f5", roles.WindowText: "#000000", roles.Base: "#ffffff",
        roles.AlternateBase: "#f0f0f0", roles.Text: "#000000", roles.Button: "#ececec",
        roles.ButtonText: "#000000", roles.ToolTipBase: "#ffffdc", roles.ToolTipText: "#000000",
        roles.PlaceholderText: "#707070", roles.Highlight: "#2a6fd6", roles.HighlightedText: "#ffffff",
        roles.Link: "#0050b0", roles.BrightText: "#c00000", roles.Light: "#ffffff",
        roles.Midlight: "#e6e6e6", roles.Mid: "#b8b8b8", roles.Dark: "#a0a0a0", roles.Shadow: "#707070",
    }
    for role, value in colors.items():
        palette.setColor(role, QtGui.QColor(value))
    disabled = QtGui.QPalette.ColorGroup.Disabled
    for role in (roles.WindowText, roles.Text, roles.ButtonText):
        palette.setColor(disabled, role, QtGui.QColor("#8a8a8a"))
    return palette


def apply_light_theme(app: QtWidgets.QApplication) -> None:
    """Force light mode (white background, black text) on every platform."""
    try:
        app.styleHints().setColorScheme(QtCore.Qt.ColorScheme.Light)
    except AttributeError:  # pragma: no cover - Qt < 6.8
        pass
    app.setPalette(light_palette())


def is_dark(app: QtWidgets.QApplication | None = None) -> bool:
    """Always False: the tools force light mode (kept for callers)."""
    return False


def apply_plot_theme(app: QtWidgets.QApplication | None = None) -> None:
    """pyqtgraph plots: white background, black axes and text."""
    pg.setConfigOption("background", "w")
    pg.setConfigOption("foreground", "k")


def color(index: int) -> str:
    return PLOT_COLORS[index % len(PLOT_COLORS)]


def make_plot(parent: QtWidgets.QWidget | None = None, *, title: str | None = None,
              x_label: str | None = None, y_label: str | None = None,
              legend: bool = True, x_time: bool = False,
              view_box: pg.ViewBox | None = None) -> pg.PlotWidget:
    """A PlotWidget configured for large data: clip-to-view, peak downsample.

    Curves added with plot.plot(x, y, ...) inherit these settings; pass numpy
    arrays (float64) so pyqtgraph never copies Python lists. ``view_box`` is
    an optional ViewBox (subclass) for custom mouse handling.
    """
    plot = pg.PlotWidget(parent=parent, title=title, viewBox=view_box)
    item = plot.getPlotItem()
    item.setClipToView(True)
    item.setDownsampling(ds=None, auto=True, mode="peak")
    item.showGrid(x=True, y=True, alpha=0.25)
    if legend:
        item.addLegend(offset=(10, 10))
    if x_label:
        item.setLabel("bottom", x_label)
    if y_label:
        item.setLabel("left", y_label)
    if x_time:
        item.setAxisItems({"bottom": pg.DateAxisItem(orientation="bottom")})
    plot.setMenuEnabled(True)
    return plot


class _ShiftedView:
    """A ViewBox seen from a curve moved by setPos(dx, 0): viewRect() in the
    curve's own x coordinates (everything else is the real ViewBox)."""
    __slots__ = ("_view", "_dx")

    def __init__(self, view, dx: float):
        self._view = view
        self._dx = dx

    def viewRect(self):
        return self._view.viewRect().translated(-self._dx, 0.0)

    def __getattr__(self, name):
        return getattr(self._view, name)


class ShiftableCurve(pg.PlotDataItem):
    """PlotDataItem whose clip-to-view and y auto-range honour setPos().

    pyqtgraph clips a curve to ``ViewBox.viewRect()`` and passes the visible
    x range to ``dataBounds`` in *view* coordinates, while the data are in
    the item's coordinates; a curve moved with setPos(dx, 0) is then clipped
    to the wrong window. This subclass shifts both by -dx, so moving a curve
    in time is just ``shift(dx)``: no new arrays, no replot of the data.
    """

    _clip_dx = 0.0

    def shift(self, dx: float) -> None:
        """Move the curve along x by ``dx`` (absolute, not cumulative)."""
        dx = float(dx)
        if dx == self.pos().x():
            return
        self.setPos(dx, 0.0)
        self.viewRangeChanged()       # re-clip the cached arrays to the view

    def getViewBox(self):
        view = super().getViewBox()
        if self._clip_dx and view is not None:
            return _ShiftedView(view, self._clip_dx)
        return view

    def _getDisplayDataset(self):
        self._clip_dx = float(self.pos().x())
        try:
            return super()._getDisplayDataset()
        finally:
            self._clip_dx = 0.0

    def dataBounds(self, ax, frac=1.0, orthoRange=None):
        dx = float(self.pos().x())
        if ax == 1 and orthoRange is not None and dx:
            orthoRange = (orthoRange[0] - dx, orthoRange[1] - dx)
        return super().dataBounds(ax, frac, orthoRange)


def add_curve(plot: pg.PlotWidget, x, y, *, name: str | None = None,
              index: int = 0, width: float = 1.0, pen=None) -> pg.PlotDataItem:
    """Add a downsampled, clipped curve (numpy x/y) with a palette color.

    The curve is a ShiftableCurve, so ``curve.shift(dx)`` (or setPos) moves
    it in time with clipping still correct.
    """
    curve = ShiftableCurve(x, y, name=name, pen=pen or pg.mkPen(color(index), width=width))
    plot.addItem(curve)
    curve.setClipToView(True)
    curve.setDownsampling(auto=True, method="peak")
    curve.setSkipFiniteCheck(False)
    return curve


class Throttle(QtCore.QObject):
    """Coalesce many update requests into at most one call per interval.

    request() may be called from the GUI thread as often as data arrives; the
    callback runs on the next timer tick only if something was requested.
    """

    def __init__(self, callback: Callable[[], None], interval_ms: int = 40,
                 parent: QtCore.QObject | None = None):
        super().__init__(parent)
        self._callback = callback
        self._pending = False
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(max(1, int(interval_ms)))
        self._timer.timeout.connect(self._tick)
        self._timer.start()

    def request(self) -> None:
        self._pending = True

    def flush(self) -> None:
        self._pending = True
        self._tick()

    def stop(self) -> None:
        self._timer.stop()

    def _tick(self) -> None:
        if not self._pending:
            return
        self._pending = False
        try:
            self._callback()
        except Exception:  # keep the timer alive; report like Qt would
            traceback.print_exc()


class _WorkerSignals(QtCore.QObject):
    done = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(object)
    progress = QtCore.pyqtSignal(object)


class Worker(QtCore.QRunnable):
    """A QRunnable whose result/error/progress are delivered as Qt signals.

    fn receives a ``progress`` callable as keyword ``progress`` if it accepts
    one, and a ``cancelled`` callable returning True after cancel().
    """

    def __init__(self, fn: Callable[..., Any], *args, **kwargs):
        super().__init__()
        self.fn, self.args, self.kwargs = fn, args, kwargs
        self.signals = _WorkerSignals()
        self._cancelled = False
        self.setAutoDelete(True)

    def cancel(self) -> None:
        self._cancelled = True

    def cancelled(self) -> bool:
        return self._cancelled

    def run(self) -> None:  # pragma: no cover - exercised through run_in_thread
        try:
            result = self.fn(*self.args, **self.kwargs)
        except BaseException as exc:  # noqa: BLE001 - forwarded to the GUI
            if not self._cancelled:
                self.signals.failed.emit(exc)
            return
        if not self._cancelled:
            self.signals.done.emit(result)


def run_in_thread(fn: Callable[..., Any], *args,
                  on_done: Callable[[Any], None] | None = None,
                  on_error: Callable[[BaseException], None] | None = None,
                  on_progress: Callable[[Any], None] | None = None,
                  **kwargs) -> Worker:
    """Run fn(*args, **kwargs) on Qt's global thread pool.

    Callbacks run on the GUI thread. If fn takes ``progress=`` it is given a
    thread-safe emitter; if it takes ``cancelled=`` it gets worker.cancelled.
    """
    import inspect
    worker = Worker(fn, *args, **kwargs)
    try:
        params = inspect.signature(fn).parameters
    except (TypeError, ValueError):
        params = {}
    if "progress" in params and "progress" not in kwargs:
        worker.kwargs["progress"] = worker.signals.progress.emit
    if "cancelled" in params and "cancelled" not in kwargs:
        worker.kwargs["cancelled"] = worker.cancelled
    if on_done:
        worker.signals.done.connect(on_done)
    if on_error:
        worker.signals.failed.connect(on_error)
    if on_progress:
        worker.signals.progress.connect(on_progress)
    QtCore.QThreadPool.globalInstance().start(worker)
    return worker


class GuiQueue(QtCore.QObject):
    """Thread-safe hand-off from worker threads to the GUI thread.

    put() can be called from any thread; items are delivered in order to
    ``handler(list_of_items)`` on the GUI thread, batched per timer tick.
    """

    def __init__(self, handler: Callable[[list], None], interval_ms: int = 30,
                 parent: QtCore.QObject | None = None):
        super().__init__(parent)
        import collections
        import threading
        self._handler = handler
        self._items: collections.deque = collections.deque()
        self._lock = threading.Lock()
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(max(1, int(interval_ms)))
        self._timer.timeout.connect(self.drain)
        self._timer.start()

    def put(self, item) -> None:
        with self._lock:
            self._items.append(item)

    def drain(self) -> None:
        with self._lock:
            if not self._items:
                return
            batch = list(self._items)
            self._items.clear()
        try:
            self._handler(batch)
        except Exception:
            traceback.print_exc()

    def stop(self) -> None:
        self._timer.stop()


def process_events_until(predicate: Callable[[], bool], timeout_s: float = 5.0) -> bool:
    """Test helper: pump the Qt event loop until predicate() or timeout."""
    import time
    app = ensure_app()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        app.processEvents(QtCore.QEventLoop.ProcessEventsFlag.AllEvents, 20)
        if predicate():
            return True
        QtCore.QThread.msleep(5)
    app.processEvents()
    return bool(predicate())


def message_error(parent: QtWidgets.QWidget | None, title: str, text: str) -> None:
    QtWidgets.QMessageBox.critical(parent, title, text)


def run_app(window: QtWidgets.QWidget) -> int:
    """Show a top-level window and run the event loop."""
    app = ensure_app()
    window.show()
    return app.exec()
