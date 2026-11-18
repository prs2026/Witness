"""A tiny observable value: the GUI-free state behind Qt widgets.

Logic modules (monitor_core, auto_connect, flight_layout_draft, ...) keep their
user-visible state in Cells; Qt widgets subscribe and render it, and push user
edits back with set(). ``get``/``set`` mirror the old tkinter Variable API, so
callers that read ``monitor.values[key].get()`` keep working.

Cells are not thread-safe: set them on the GUI thread (worker threads hand data
over through queues first).
"""

from __future__ import annotations

from typing import Any, Callable


class _Unsubscribe:
    """The callable returned by Cell.subscribe.

    A callable object rather than a closure on purpose: Qt ``destroyed`` slots call it
    while Python's cyclic GC may be clearing the garbage, and a cleared closure (a
    function with its globals gone) crashes the interpreter when called, whereas a
    cleared instance is harmless (see qt_widgets._OnDestroyed).
    """

    __slots__ = ("cell", "listener")

    def __init__(self, cell, listener):
        self.cell = cell
        self.listener = listener

    def __call__(self) -> None:
        try:
            self.cell._listeners.remove(self.listener)
        except (ValueError, AttributeError):   # already gone (interpreter exit, or cleared by GC)
            pass


class Cell:
    __slots__ = ("_value", "_listeners", "__weakref__")

    def __init__(self, value: Any = None):
        self._value = value
        self._listeners: list[Callable[[Any], None]] = []

    def get(self) -> Any:
        return self._value

    def set(self, value: Any) -> None:
        if value == self._value and type(value) is type(self._value):
            return
        self._value = value
        for listener in tuple(self._listeners):
            listener(value)

    def subscribe(self, listener: Callable[[Any], None], *, call_now: bool = False) -> Callable[[], None]:
        """Call listener(value) on every change; returns an unsubscribe function."""
        self._listeners.append(listener)
        if call_now:
            listener(self._value)

        return _Unsubscribe(self, listener)

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"Cell({self._value!r})"
