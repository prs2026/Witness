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
STALE_S = 3.0              # readouts older than this are greyed (audit H10)
STALE_GREY = "#8a8a8a"
SENSOR_BITS = ((9, "low-g"), (10, "gyro"), (11, "IMU temp"), (13, "high-g"), (14, "baro"))
STRIP_COLORS = {"bad": ("#b00000", "#ffffff"), "warn": ("#ffd24d", "#171717"), "ok": ("#d8efdf", "#0a3a1a"),
                "muted": ("#e4e4e4", "#444444"),
                "bench": ("#d9c9f2", "#2a0a4a")}
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


# Display-only unit conversion. Histories, derived fields and layout JSON stay
# SI ("m", "m/s", "km"); they are converted to imperial only when drawn or printed.
FT_M = 0.3048
MI_M = 5280 * FT_M
DISPLAY_UNITS = {"m": ("ft", 1 / FT_M), "m/s": ("ft/s", 1 / FT_M), "km": ("mi", 1000 / MI_M)}


def display_unit(unit):
    """(label, factor) a layout unit is shown as; unknown units pass through."""
    return DISPLAY_UNITS.get(unit, (unit, 1.0))


def unit_label(unit):
    return display_unit(unit)[0]


def trace_unit(row, trace):
    """Layout unit of one trace: its override, else its axis's (right_unit or row unit)."""
    if trace.unit:
        return trace.unit
    if getattr(trace, "axis", "left") == "right" and getattr(row, "right_unit", None):
        return row.right_unit
    return row.unit


def display_factor(row, trace):
    return display_unit(trace_unit(row, trace))[1]


def value_text(trace, history, unit=None):
    """Readout text; `unit` is the row's layout unit (a trace unit override wins)."""
    value = history.latest
    if type(value) in (int, float) and (trace.unit or unit):
        value = value * display_unit(trace.unit or unit)[1]
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


def dim(color, history, end):
    """Grey a readout whose data is old, invalid, a snapshot or missing."""
    if history.received is None or history.quality != "LIVE" or end - history.received >= STALE_S:
        return STALE_GREY
    return color


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

def source_text(status):
    """"W COM7" style text for the sources Flight View currently follows."""
    if not status:
        return ""
    parts = [f"{kind[0].upper()}: {source}" for kind, source in sorted(status["sources"].items())]
    note = status.get("switch")
    if note:
        parts.append(f"switched {note[1]} -> {note[2]}")
    return " | ".join(parts)


def banner(layout, histories, end, *, paused=False, load_error=None, status=None):
    """(text, background) for the status banner above the dashboard.

    With ``status`` (FlightData.status_snapshot) green LIVE means the essential
    telemetry is fresh: Witness state + position/altitude packets within 3 s,
    not "any one of 45 traces" (audit H10).
    """
    flat = [(trace, history) for row, data in zip(layout.rows, histories)
            for trace, history in zip(row.traces, data) if trace.enabled]
    fresh = sum(h.quality == "LIVE" and h.received is not None and end - h.received < 2 for _, h in flat)
    simulated = any(h.source.startswith("TEST_") for _, h in flat)
    hitl = any(trace.format == "hitl" and h.latest == 1 for trace, h in flat)
    state = "PAUSED - DISPLAY FROZEN; receiving/logging continues" if paused else (
        f"LIVE - {fresh}/{len(flat)} readings fresh" if fresh else "WAITING / STALE - no fresh live readings")
    partial = False
    if status is not None and not paused:
        ages = {k: max(0.0, end - t) for k, t in status["essential"].items()}
        witness = ages.get("witness")
        others = [a for k, a in ages.items() if k != "witness"]
        if witness is not None and witness < STALE_S:
            fresh = max(fresh, 1)
            state = f"LIVE - Witness state/position {witness:.1f}s old | {fresh}/{len(flat)} readings fresh"
        elif others and min(others) < STALE_S:
            partial = True
            lag = f"STALE {witness:.0f}s" if witness is not None else "NO DATA"
            state = f"WITNESS {lag} - other sources live"
            fresh = 0
        else:
            fresh = 0
            lag = f"last Witness packet {witness:.0f}s ago" if witness is not None else "no Witness state received"
            state = f"WAITING / STALE - {lag}"
        src = source_text(status)
        if src:
            state += f" | {src}"
    warning = ("HITL - SIMULATED SENSORS | " if hitl else "") + ("TEST DATA | " if simulated else "")
    if load_error:
        warning = "LAYOUT ERROR (fallback active) | " + warning
    background = ("#fff2a8" if paused or simulated or hitl or load_error or partial
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
            unit = unit_label(trace.unit or row.unit)
            status, status_color = quality_text(history, end)
            cells.append((f"{trace.label}: {value_text(trace, history, row.unit)} {unit}".rstrip(),
                          dim(readable_color(trace.color), history, end), status, status_color))
        metrics.append((row.title, cells))
    state = None
    states = [(row, data) for row, data in rows if row.group == "state"]
    if states:
        row, data = states[0]
        trace, history = row.traces[0], data[0]
        status, status_color = quality_text(history, end)
        state = {"value": value_text(trace, history), "color": dim(readable_color(trace.color), history, end),
                 "status": status, "status_color": status_color,
                 "source": history.source or "Waiting for Witness",
                 "hitl": hitl_line(row, data, end)}
    return {"metrics": metrics, "state": state}


def bit_chip(status, end):
    """(text, kind, tooltip) of the built-in self test chip, or None before a BIT summary arrived."""
    model = status.get("bit") if status else None
    return model.status(end) if model is not None else None


def status_strip(status, end):
    """[(text, kind)] chips for the Witness status word, kind: bad/warn/ok/muted.

    Red: NOT LOGGING (log slots full), logging/flash failed, maintenance image.
    Amber: resumed after reset, HITL, no GPS fix, sensors not ready, old status.
    Green chips go grey when the status word is older than 3 s; red ones never do.
    """
    vehicle = status.get("vehicle") if status else None
    if not vehicle:
        return [("NO WITNESS STATUS RECEIVED", "muted")]
    bits = vehicle["bits"]
    age = max(0.0, end - vehicle["received"])
    old = age >= STALE_S or vehicle.get("snapshot")
    chips = []
    bad_log = False
    if bits.get("status.log_slots_full"):
        chips.append(("NOT LOGGING - LOG SLOTS FULL (erase/clear to log)", "bad"))
        bad_log = True
    if bits.get("status.logging_failed"):
        chips.append(("LOGGING FAILED (NAND write)", "bad"))
        bad_log = True
    if bits.get("status.flash_init_failed"):
        chips.append(("FLASH INIT FAILED - NOT LOGGING", "bad"))
        bad_log = True
    if bits.get("status.maintenance"):
        chips.append(("MAINTENANCE IMAGE - NOT FLIGHT READY", "bad"))
    if not bad_log:
        chips.append(("LOGGING OK", "muted" if old else "ok"))
    if bits.get("status.resumed"):
        chips.append(("RESUMED AFTER RESET", "warn"))
    if bits.get("status.hitl_active"):
        chips.append(("HITL ACTIVE - SIMULATED SENSORS", "warn"))
    if bits.get("status.gps_locked"):
        chips.append(("GPS LOCKED", "muted" if old else "ok"))
    else:
        held = vehicle.get("position")
        extra = f" - last known position {max(0.0, end - held[2]):.0f}s old" if held else ""
        chips.append((f"GPS NO FIX{extra}", "warn"))
    value = vehicle["status"]
    missing = [name for bit, name in SENSOR_BITS if not (value >> bit) & 1]
    chips.append(("SENSORS READY", "muted" if old else "ok") if not missing
                 else (f"SENSOR NOT READY: {', '.join(missing)}", "warn"))
    if status.get("relative_agl"):
        chips.append(("GPS ALT RELATIVE TO FIRST SAMPLE (no pad altitude)", "muted"))
    shown = bit_chip(status, end)
    if shown is not None:
        chips.append((shown[0], shown[1]))
    if old:
        chips.append((f"STATUS {age:.0f}s OLD" + (" (snapshot)" if vehicle.get("snapshot") else ""), "warn"))
    return chips


def landed_line(status, end):
    """(text, kind) for the 0x09 landed position, or None before one arrived."""
    landed = status.get("landed") if status else None
    if not landed:
        return None
    age = max(0.0, end - landed["received"])
    flags = (" PING-REPLY" if landed.get("ping_reply") else "") + (" STALE-FIX" if landed.get("stale") else "")
    if not landed.get("position_valid") or landed.get("latitude") is None:
        return (f"LANDED PACKET: NO VALID FIX (rx {age:.0f}s ago){flags}", "warn")
    fix = landed.get("fix_age")
    fix_text = f", fix {fix:.0f}s old" if isinstance(fix, (int, float)) else ""
    return (f"LANDED FIX {landed['latitude']:.6f}, {landed['longitude']:.6f} alt "
            f"{landed['gps_altitude'] / FT_M:.0f} ft MSL (rx {age:.0f}s ago{fix_text}){flags}",
            "warn" if landed.get("stale") else "ok")


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
                parts.append(f"{trace.label} {value_text(trace, history, row.unit)}{marker}")
            out.append((title, [("  ".join(parts), alert or readable_color(row.traces[0].color))]))
            continue
        cells = []
        for trace, history in zip(row.traces, data):
            marker, color = _marker(history, end)
            unit = unit_label(trace.unit or row.unit)
            if len(data) == 2:
                text = f"{value_text(trace, history, row.unit)}{marker} {unit}"
            else:
                text = value_text(trace, history, row.unit)
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
        items.append((f"{trace.label} {value_text(trace, history, trace_unit(plot, trace))}{marker}",
                      color if marker else readable_color(trace.color)))
    return items


def trace_arrays(points, start, end, gap):
    """Visible samples as float arrays with NaN breaks at invalid values and gaps.

    ``points`` is a flight_data.PointSnapshot/PointBuffer (numpy, fast) or any
    iterable of (time, value|None). Vectorised: no per-point Python loop.
    A break (NaN) is emitted only after a valid sample, before an invalid sample
    (at its time) or before a sample that follows a gap (at the previous time).
    """
    if hasattr(points, "arrays"):
        t, v = points.arrays()
    else:
        items = list(points)
        t = np.fromiter((p[0] for p in items), float, len(items))
        v = np.fromiter((math.nan if p[1] is None else p[1] for p in items), float, len(items))
    mask = (t >= start) & (t <= end)
    t, v = t[mask], v[mask]
    n = len(t)
    if n == 0:
        return np.empty(0), np.empty(0)
    bad = np.isnan(v)
    prev_ok = np.zeros(n, bool)
    prev_ok[1:] = ~bad[:-1]
    gapped = np.zeros(n, bool)
    gapped[1:] = (t[1:] - t[:-1]) > gap
    brk = prev_ok & (bad | gapped)
    brk_x = np.where(bad, t, np.concatenate(([t[0]], t[:-1])))
    keep = ~bad
    count = brk.astype(np.intp) + keep.astype(np.intp)
    ends = np.cumsum(count)
    xs, ys = np.empty(int(ends[-1])), np.empty(int(ends[-1]))
    xs[ends[keep] - 1], ys[ends[keep] - 1] = t[keep], v[keep]
    first = ends[brk] - count[brk]
    xs[first], ys[first] = brk_x[brk], math.nan
    return xs, ys


def scale_series(plot, series):
    """Display-unit copies of trace_arrays results (m -> ft, m/s -> ft/s, km -> mi)."""
    out = []
    for trace, (x, y) in zip(plot.traces, series):
        factor = display_factor(plot, trace)
        out.append((x, y * factor) if factor != 1.0 else (x, y))
    return out


def axis_ranges(plot, series, layout, pixels=300):
    """{"left"/"right": (low, high, step)} and whether any sample is clipped.

    `series` is in display units (scale_series); fixed y_range limits are given in
    the layout unit and converted the same way.
    """
    ranges, clipped = {}, False
    for axis in ("left", "right"):
        values = np.concatenate([y[np.isfinite(y)] for trace, (_x, y) in zip(plot.traces, series)
                                 if trace.axis == axis] or [np.empty(0)])
        limits = plot.y_range if axis == "left" and not layout.auto_scale else None
        if limits is not None:
            factor = display_unit(plot.unit)[1]
            limits = (limits[0] * factor, limits[1] * factor)
        low, high, step = graph_range(values.tolist() if limits is None and len(values) < 4096
                                      else ([float(values.min()), float(values.max())] if len(values) else []),
                                      pixels, limits)
        ranges[axis] = (low, high, step)
        if len(values):
            clipped |= bool(np.any((values < low) | (values > high)))
    return ranges, clipped
