"""What the Flight View dashboard shows, computed without any GUI.

flight_view.FlightView (Qt + pyqtgraph) renders the structures returned here:
banner text/colour, the flight overview and state panel, the GPS / power
readouts, each graph's legend entries and the plot series (numpy arrays with
NaN gaps). Colours are dark on white; pale custom colours are darkened for
readability at render time without rewriting saved JSON.
"""

from __future__ import annotations

from functools import lru_cache
import math

import numpy as np

from flight_data import graph_range

BG = "#ffffff"
PANEL = "#ffffff"
GRID = "#b8b8b8"
BORDER = "#666666"
TEXT = "#171717"
MUTED = "#444444"
WARN = "#704000"
BAD = "#990000"
STATES = ("PAD IDLE", "BOOST", "COAST", "DESCENT", "LANDED")
OLD_MARKERS = ("AGE", "STALE", "INVALID", "SNAPSHOT")


@lru_cache(maxsize=256)
def readable_color(color):
    """Darken pale custom/legacy colors to at least 7:1 against white.

    Apply only while rendering: saved JSON colors are not rewritten. Preserve
    the requested hue where possible and use the same color for text and lines.
    """
    rgb = [int(color[i:i + 2], 16) for i in (1, 3, 5)]
    while True:
        channels = [v / 255 for v in rgb]
        linear = [v / 12.92 if v <= .04045 else ((v + .055) / 1.055) ** 2.4 for v in channels]
        luminance = sum(v * weight for v, weight in zip(linear, (.2126, .7152, .0722)))
        if 1.05 / (luminance + .05) >= 7:
            return "#" + "".join(f"{v:02x}" for v in rgb)
        rgb = [int(v * .9) for v in rgb]


def value_text(trace, history):
    value = history.latest
    if not trace.enabled:
        return "N/A"
    if value is None:
        return "--"
    if trace.format == "state":
        return STATES[int(value)] if value in range(5) else "INVALID STATE"
    if trace.format == "continuity":
        return {0: "OPEN", 1: "CONTINUITY"}.get(value, "UNKNOWN")
    if trace.format == "flag":
        return {0: "0", 1: "1"}.get(value, "?")
    if trace.format == "hitl":
        return {0: "HITL OFF - real sensors", 1: "HITL ACTIVE - SIMULATED SENSORS"}.get(value, "HITL ?")
    if "voltage" in trace.field.lower():
        return f"{value:.3f}"
    return f"{value:.5g}"


def hitl_trace(row, data):
    """Return (trace, history) for a state row's optional HITL trace."""
    for trace, history in zip(row.traces[1:], data[1:]):
        if trace.format == "hitl":
            return trace, history
    return None


def hitl_line(row, data, end):
    """Text and color for the HITL line under the flight state.

    HITL active is shown in the error color whatever the data age, so a
    simulated run cannot be mistaken for flight data even after it stops.
    """
    found = hitl_trace(row, data)
    if found is None:
        return None
    trace, history = found
    if not trace.enabled:
        return "HITL: N/A", MUTED
    if history.latest == 1:
        return "HITL ACTIVE - SIMULATED SENSORS", BAD
    if history.latest is None:
        return "HITL: --", MUTED
    status, _color = quality_text(history, end)
    return f"HITL off - real sensors ({status})", MUTED


def quality_text(history, end):
    if history.received is None:
        if history.quality == "INVALID":
            return "INVALID / NO DATA", BAD
        return "N/A" if history.quality == "UNAVAILABLE" else "NO DATA", MUTED
    age = max(0, end - history.received)
    if history.quality != "LIVE":
        return f"{history.quality} {age:.1f}s", BAD if history.quality == "INVALID" else WARN
    return (f"STALE {age:.1f}s", BAD) if age >= 10 else (f"AGE {age:.1f}s", WARN) if age >= 2 else (f"LIVE {age:.1f}s", MUTED)


def _marker(history, end):
    status, color = quality_text(history, end)
    return ("!", color) if status.startswith(OLD_MARKERS) else ("", None)


def short_title(title):
    return title.replace("Featherweight", "FW").replace("Witness", "W").replace("computer", "CPU")


# ------------------------------------------------------------- panels

def banner(layout, histories, end, *, paused=False, load_error=None):
    """(text, background) for the status banner above the dashboard."""
    flat = [(trace, history) for row, data in zip(layout.rows, histories)
            for trace, history in zip(row.traces, data) if trace.enabled]
    fresh = sum(h.quality == "LIVE" and h.received is not None and end - h.received < 2 for _, h in flat)
    simulated = any(h.source.startswith("TEST_") for _, h in flat)
    hitl = any(trace.format == "hitl" and h.latest == 1 for trace, h in flat)
    state = "PAUSED - DISPLAY FROZEN; receiving/logging continues" if paused else (
        f"LIVE - {fresh}/{len(flat)} readings fresh" if fresh else "WAITING / STALE - no fresh live readings")
    warning = ("HITL - SIMULATED SENSORS | " if hitl else "") + ("TEST DATA | " if simulated else "")
    if load_error:
        warning = "LAYOUT ERROR (fallback active) | " + warning
    background = ("#fff2a8" if paused or simulated or hitl or load_error
                  else "#d8efdf" if fresh else "#ffd5cf")
    return f"{warning}{state} | {layout.window_seconds:g}s", background


def flight_panel(layout, histories, end):
    """Left panel: flight metrics and the Witness state.

    Returns {"metrics": [(title, [(text, color, status, status_color)])],
             "state": None or {value, color, status, status_color, source, hitl}}.
    """
    rows = list(zip(layout.rows, histories))
    metrics = []
    for row, data in rows:
        if row.group != "flight":
            continue
        cells = []
        for trace, history in zip(row.traces, data):
            unit = trace.unit or row.unit
            status, status_color = quality_text(history, end)
            cells.append((f"{trace.label}: {value_text(trace, history)} {unit}".rstrip(),
                          readable_color(trace.color), status, status_color))
        metrics.append((row.title, cells))
    state = None
    states = [(row, data) for row, data in rows if row.group == "state"]
    if states:
        row, data = states[0]
        trace, history = row.traces[0], data[0]
        status, status_color = quality_text(history, end)
        state = {"value": value_text(trace, history), "color": readable_color(trace.color),
                 "status": status, "status_color": status_color,
                 "source": history.source or "Waiting for Witness",
                 "hitl": hitl_line(row, data, end)}
    return {"metrics": metrics, "state": state}


def status_rows(layout, histories, end, group):
    """GPS or power readouts: [(title, [(text, color)])] in layout order.

    Two-value GPS rows read "W GPS sats 7  lock 1" on one line; power pairs
    keep independent values and stale/invalid "!" markers (never combined).
    """
    out = []
    for row, data in zip(layout.rows, histories):
        if row.group != group:
            continue
        title = short_title(row.title)
        if len(data) == 2 and group == "gps":
            parts, alert = [], None
            for trace, history in zip(row.traces, data):
                marker, color = _marker(history, end)
                alert = alert or color
                parts.append(f"{trace.label} {value_text(trace, history)}{marker}")
            out.append((title, [("  ".join(parts), alert or readable_color(row.traces[0].color))]))
            continue
        cells = []
        for trace, history in zip(row.traces, data):
            marker, color = _marker(history, end)
            unit = trace.unit or row.unit
            if len(data) == 2:
                text = f"{value_text(trace, history)}{marker} {unit}"
            else:
                text = value_text(trace, history)
                if trace.format == "number" and trace.enabled:
                    text += " " + unit
                if marker:
                    text += " !"
            cells.append((text, color or readable_color(trace.color)))
        if len(data) == 1:
            trace, history = row.traces[0], data[0]
            status, status_color = quality_text(history, end)
            note = trace.note if not trace.enabled else status
            cells.append((note, status_color))
        out.append((title, cells))
    return out


def legend(plot, histories, end):
    """[(text, color)] per trace: "<label> <latest value>" with "!" when old/invalid."""
    items = []
    for trace, history in zip(plot.traces, histories):
        _status, color = quality_text(history, end)
        stale = history.received is not None and end - history.received >= 2
        marker = "!" if stale or history.quality in ("INVALID", "SNAPSHOT") else ""
        items.append((f"{trace.label} {value_text(trace, history)}{marker}",
                      color if marker else readable_color(trace.color)))
    return items


def trace_arrays(points, start, end, gap):
    """Visible samples as float arrays with NaN breaks at invalid values and gaps."""
    xs, ys = [], []
    previous = None
    for stamp, value in points:
        if stamp < start or stamp > end:
            continue
        if value is None or (previous is not None and stamp - previous > gap):
            if xs and not math.isnan(ys[-1]):
                xs.append(stamp if value is None else previous)
                ys.append(math.nan)
        if value is not None:
            xs.append(stamp)
            ys.append(value)
        previous = stamp
    return np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)


def axis_ranges(plot, series, layout, pixels=300):
    """{"left"/"right": (low, high, step)} and whether any sample is clipped."""
    ranges, clipped = {}, False
    for axis in ("left", "right"):
        values = np.concatenate([y[np.isfinite(y)] for trace, (_x, y) in zip(plot.traces, series)
                                 if trace.axis == axis] or [np.empty(0)])
        limits = plot.y_range if axis == "left" and not layout.auto_scale else None
        low, high, step = graph_range(values.tolist() if limits is None and len(values) < 4096
                                      else ([float(values.min()), float(values.max())] if len(values) else []),
                                      pixels, limits)
        ranges[axis] = (low, high, step)
        if len(values):
            clipped |= bool(np.any((values < low) | (values > high)))
    return ranges, clipped
