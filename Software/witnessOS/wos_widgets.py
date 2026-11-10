"""Qt building blocks shared by the witnessOS tabs.

scroll_tab(widget)
    Wraps a tab's panel in a QScrollArea that stretches the panel to fill
    the window when there is room and scrolls (scrollbars shown only when
    needed) when the window is smaller than the panel's minimum size, so
    nothing is ever clipped out of reach. Nested scroll areas (the HITL
    parameter list, the log) keep their own wheel handling: Qt routes the
    wheel to the innermost scrollable widget under the pointer.
"""

from __future__ import annotations

from PyQt6 import QtCore, QtWidgets


def scroll_tab(widget: QtWidgets.QWidget) -> QtWidgets.QScrollArea:
    area = QtWidgets.QScrollArea()
    area.setWidgetResizable(True)
    area.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
    area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAsNeeded)
    area.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAsNeeded)
    area.setWidget(widget)
    return area
