"""Resize-tolerant Tk building blocks shared by the witnessOS tabs.

ScrollableFrame
    A frame whose content stretches to fill the window when there is room
    and scrolls (vertical and/or horizontal scrollbars, shown only when
    needed) when the window is smaller than the content's requested size.
    Nothing inside is ever clipped out of reach.

WheelRouter
    One application-wide mouse-wheel handler. The wheel scrolls the
    innermost registered scroller under the pointer that can still move in
    that direction, so nested scroll areas (a parameter list inside a
    scrolling tab) behave as expected. Text, Treeview and Listbox widgets
    keep their own native wheel handling.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

NATIVE_WHEEL_CLASSES = frozenset(("Text", "Treeview", "Listbox"))


def fit_inner(canvas_w: int, canvas_h: int, req_w: int,
              req_h: int) -> tuple[int, int, bool, bool]:
    """Size of the scrolled content and which scrollbars are needed.

    The content is never smaller than the visible area (so it fills large
    windows) and never smaller than it asks for (so it is not squashed and
    clipped in small ones).
    """
    canvas_w = max(1, int(canvas_w))
    canvas_h = max(1, int(canvas_h))
    width = max(canvas_w, int(req_w))
    height = max(canvas_h, int(req_h))
    return width, height, req_w > canvas_w, req_h > canvas_h


def can_scroll(view: tuple[float, float], direction: int) -> bool:
    """True if a (first, last) x/yview can still move in ``direction``."""
    first, last = float(view[0]), float(view[1])
    if direction < 0:
        return first > 1e-3
    return last < 1.0 - 1e-3


class ScrollableFrame(ttk.Frame):
    """Put content in ``.inner``; see the module docstring."""

    POLL_MS = 300

    def __init__(self, master, router: "WheelRouter | None" = None, **kwargs):
        super().__init__(master, **kwargs)
        background = ttk.Style(self).lookup("TFrame", "background") or None
        self.canvas = tk.Canvas(self, highlightthickness=0, borderwidth=0,
                                width=200, height=150)
        if background:
            self.canvas.configure(background=background)
        self.vbar = ttk.Scrollbar(self, orient="vertical",
                                  command=self.canvas.yview)
        self.hbar = ttk.Scrollbar(self, orient="horizontal",
                                  command=self.canvas.xview)
        self.canvas.configure(yscrollcommand=self.vbar.set,
                              xscrollcommand=self.hbar.set)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.vbar.grid(row=0, column=1, sticky="ns")
        self.hbar.grid(row=1, column=0, sticky="ew")
        self.vbar.grid_remove()
        self.hbar.grid_remove()
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)
        self.inner = ttk.Frame(self.canvas)
        self._window = self.canvas.create_window((0, 0), window=self.inner,
                                                 anchor="nw")
        self._state = None
        self._job = None
        self._poll_job = None
        self.canvas.bind("<Configure>", lambda _e: self.relayout_soon())
        self.bind("<Destroy>", self._on_destroy, add="+")
        if router is not None:
            router.register(self.canvas)
        self._poll()

    def relayout_soon(self):
        if self._job is None:
            self._job = self.after_idle(self.relayout)

    def relayout(self):
        """Fit the content to the visible area; show/hide the scrollbars."""
        self._job = None
        if not self.winfo_exists():
            return
        cw, ch = self.canvas.winfo_width(), self.canvas.winfo_height()
        rw, rh = self.inner.winfo_reqwidth(), self.inner.winfo_reqheight()
        state = (cw, ch, rw, rh)
        if state == self._state:
            return
        self._state = state
        width, height, need_x, need_y = fit_inner(cw, ch, rw, rh)
        self.canvas.itemconfigure(self._window, width=width, height=height)
        self.canvas.configure(scrollregion=(0, 0, width, height))
        for bar, need in ((self.hbar, need_x), (self.vbar, need_y)):
            if need and not bar.winfo_ismapped():
                bar.grid()
            elif not need and bar.winfo_ismapped():
                bar.grid_remove()
        if not need_x:
            self.canvas.xview_moveto(0)
        if not need_y:
            self.canvas.yview_moveto(0)

    def _poll(self):
        # The content's requested size can change without any <Configure>
        # on the canvas (a rebuilt parameter grid, a new banner), so check.
        self._poll_job = None
        try:
            if self.winfo_exists():
                if (self.inner.winfo_reqwidth(), self.inner.winfo_reqheight()) \
                        != (self._state or (0, 0, -1, -1))[2:]:
                    self.relayout()
                self._poll_job = self.after(self.POLL_MS, self._poll)
        except tk.TclError:
            pass

    def _on_destroy(self, event):
        if event.widget is not self:
            return
        for job in (self._job, self._poll_job):
            if job is not None:
                try:
                    self.after_cancel(job)
                except tk.TclError:
                    pass


class WheelRouter:
    """Route the mouse wheel to the right scroller (see module docstring)."""

    def __init__(self, root):
        self.root = root
        self._targets: dict[str, tk.Widget] = {}
        root.bind_all("<MouseWheel>", self._on_wheel, add="+")
        root.bind_all("<Shift-MouseWheel>",
                      lambda e: self._on_wheel(e, horizontal=True), add="+")
        root.bind_all("<Button-4>", lambda e: self._on_wheel(e, -1), add="+")
        root.bind_all("<Button-5>", lambda e: self._on_wheel(e, 1), add="+")

    def register(self, widget) -> None:
        """Let the wheel scroll ``widget`` (anything with x/yview_scroll)."""
        self._targets[str(widget)] = widget

    def target_for(self, widget, direction: int, horizontal: bool = False):
        """The registered scroller that should take this wheel step."""
        while widget is not None:
            try:
                cls = widget.winfo_class()
            except tk.TclError:
                return None
            if cls in NATIVE_WHEEL_CLASSES:
                return None
            target = self._targets.get(str(widget))
            if target is not None:
                try:
                    view = target.xview() if horizontal else target.yview()
                except tk.TclError:
                    view = (0.0, 1.0)
                if can_scroll(view, direction):
                    return target
            widget = getattr(widget, "master", None)
        return None

    def _on_wheel(self, event, direction: int | None = None,
                  horizontal: bool = False):
        if direction is None:
            if not getattr(event, "delta", 0):
                return None
            direction = -1 if event.delta > 0 else 1
            amount = max(1, abs(int(event.delta / 120)))
        else:
            amount = 1
        try:
            widget = self.root.winfo_containing(event.x_root, event.y_root)
        except (KeyError, tk.TclError):
            widget = None
        if widget is None:
            return None
        target = self.target_for(widget, direction, horizontal)
        if target is None:
            return None
        if horizontal:
            target.xview_scroll(direction * amount, "units")
        else:
            target.yview_scroll(direction * amount, "units")
        return "break"
