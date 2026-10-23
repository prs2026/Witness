"""Validated Flight View layouts and thread-safe, bounded telemetry histories.

No Tk or device access here. Receiver callbacks run before latest-value UI
coalescing, preserving short peaks. All plots use host receive time, not uptime.
"""

from collections import deque
from dataclasses import dataclass, field as dataclass_field, replace
import json
import math
from pathlib import Path
import re
import threading
import time

from telemetry_view import flatten_fields
from aim_orientation import AimAngularRateEstimator
from aim_altitude import AimAltitudeEstimator

DEFAULT_LAYOUT = Path(__file__).resolve().parent / "flight_view_default.json"
COLORS = ("#004b80", "#803900", "#006020", "#5b267d", "#990000", "#555000")


def _number(value, name, low, high):
    if type(value) not in (int, float) or not low <= value <= high or not math.isfinite(value):
        raise ValueError(f"{name} must be a number between {low} and {high}")
    return value


def _text(value, name, limit=200):
    if not isinstance(value, str) or not value.strip() or len(value) > limit:
        raise ValueError(f"{name} must be nonempty text, at most {limit} characters")
    return value


def _keys(value, allowed, name):
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be an object")
    unknown = set(value) - set(allowed.split())
    if unknown:
        raise ValueError(f"{name}: unknown setting(s): {', '.join(sorted(unknown))}")


@dataclass(frozen=True)
class Trace:
    label: str
    input: str
    packet: str | tuple[str, ...]
    field: str
    source: str = "*"
    color: str = COLORS[0]
    scale: float = 1
    offset: float = 0
    match: dict = dataclass_field(default_factory=dict)
    unit: str | None = None
    enabled: bool = True
    note: str = ""
    format: str = "number"
    axis: str = "left"


@dataclass(frozen=True)
class Plot:
    title: str
    unit: str
    traces: tuple
    y_range: tuple | None = None
    group: str = ""
    right_unit: str | None = None


@dataclass(frozen=True)
class Layout:
    title: str
    window_seconds: float
    refresh_ms: int
    columns: int
    plot_height: int
    max_points: int
    gap_seconds: float
    plots: tuple
    document: dict
    readouts: tuple = ()
    auto_scale: bool = True

    @property
    def rows(self):
        return self.plots + self.readouts


def validate_layout(document):
    _keys(document, "version description title window_seconds refresh_ms columns plot_height max_points gap_seconds plots readouts auto_scale", "Layout")
    auto_scale = document.get("auto_scale", True)
    if type(auto_scale) is not bool:
        raise ValueError("auto_scale must be true or false")
    if type(document.get("version")) is not int or document["version"] != 1:
        raise ValueError("Layout version must be 1")
    title = _text(document.get("title", "Flight View"), "title")
    window = _number(document.get("window_seconds", 120), "window_seconds", 5, 3600)
    gap = _number(document.get("gap_seconds", 3), "gap_seconds", 0.1, 120)
    integers = {}
    for key, default, low, high in (("refresh_ms", 250, 100, 2000), ("columns", 2, 1, 2),
                                   ("plot_height", 220, 140, 600), ("max_points", 6000, 100, 20000)):
        value = document.get(key, default)
        _number(value, key, low, high)
        if type(value) is not int:
            raise ValueError(f"{key} must be an integer")
        integers[key] = value
    rows = document.get("plots")
    if not isinstance(rows, list) or not 1 <= len(rows) <= 6:
        raise ValueError("The fixed dashboard supports 1 to 6 plots; remove extra plots so all remain visible")
    if math.ceil(len(rows) / integers["columns"]) > 3:
        raise ValueError("The fixed dashboard supports at most 3 graph rows; use columns: 2")
    readouts = document.get("readouts", [])
    if not isinstance(readouts, list) or len(readouts) > 20:
        raise ValueError("readouts must contain at most 20 rows")
    plots, groups = [], {}
    for index, row in enumerate(rows + readouts):
        name = f"Plot {index + 1}"
        is_readout = index >= len(rows)
        _keys(row, "title unit traces y_range right_unit" + (" group" if is_readout else ""), name)
        right_unit = row.get("right_unit")
        if right_unit is not None:
            _text(right_unit, "right_unit", 30)
            if is_readout:
                raise ValueError("right_unit is only available for graphs")
        group = row.get("group", "")
        if is_readout:
            if group not in ("flight", "state", "gps", "power"):
                raise ValueError("Readout group must be flight, state, gps, or power")
            groups[group] = groups.get(group, 0) + 1
            if groups[group] > {"flight": 3, "state": 1, "gps": 6, "power": 10}[group]:
                raise ValueError(f"Too many {group} readouts for the fixed dashboard")
        plot_title = _text(row.get("title"), name + " title")
        unit = _text(row.get("unit", "value"), name + " unit", 60)
        limits = row.get("y_range")
        if limits is not None:
            if not isinstance(limits, list) or len(limits) != 2:
                raise ValueError(f"{name}: y_range must be null (auto) or [minimum, maximum]")
            for limit in limits:
                _number(limit, "y_range", -1e15, 1e15)
            if limits[0] >= limits[1]:
                raise ValueError(f"{name}: y_range minimum must be below maximum")
        traces = row.get("traces")
        if not isinstance(traces, list) or not 1 <= len(traces) <= 8:
            raise ValueError(f"{name}: expected 1 to 8 traces")
        if is_readout and len(traces) > (3 if group == "flight" else 2 if group == "power" else 1):
            raise ValueError("Flight readouts allow 3 values, power allows 2, other readouts allow 1")
        parsed = []
        for ti, trace in enumerate(traces):
            _keys(trace, "label input packet field source color scale offset match unit enabled note format axis", name + " trace")
            axis = trace.get("axis", "left")
            if axis not in ("left", "right") or (axis == "right" and (is_readout or right_unit is None)):
                raise ValueError("axis must be left, or right on a graph with right_unit")
            kind = trace.get("input")
            if kind not in ("witness", "featherweight", "aim"):
                raise ValueError(f"{name}: input must be witness, featherweight, or aim")
            color = trace.get("color", COLORS[ti % len(COLORS)])
            if not isinstance(color, str) or not re.fullmatch(r"#[0-9a-fA-F]{6}", color):
                raise ValueError(f"{name}: color must be #RRGGBB")
            match = trace.get("match", {})
            if not isinstance(match, dict) or len(match) > 16:
                raise ValueError(f"{name}: match must be a small field/value object")
            for key, value in match.items():
                _text(key, "match field")
                if not isinstance(value, (str, int, float, bool)) or (isinstance(value, float) and not math.isfinite(value)):
                    raise ValueError("match values must be finite numbers, strings or booleans")
            packet = trace.get("packet")
            if isinstance(packet, list):
                if not 1 <= len(packet) <= 8:
                    raise ValueError("packet list must contain 1..8 names")
                packet = tuple(_text(item, "packet/topic") for item in packet)
            else:
                packet = _text(packet, "packet/topic")
            enabled = trace.get("enabled", True)
            if type(enabled) is not bool:
                raise ValueError("enabled must be true or false")
            unit = trace.get("unit")
            if unit is not None:
                _text(unit, "trace unit", 30)
                if not is_readout and unit != (right_unit if axis == "right" else row.get("unit", "value")):
                    raise ValueError("Graph traces must use their axis unit")
            note = trace.get("note", "")
            if not isinstance(note, str) or len(note) > 80:
                raise ValueError("note must be text of at most 80 characters")
            display_format = trace.get("format", "number")
            if display_format not in ("number", "state", "continuity"):
                raise ValueError("format must be number, state, or continuity")
            parsed.append(Trace(
                _text(trace.get("label"), "trace label", 80), kind,
                packet, _text(trace.get("field"), "field"),
                _text(trace.get("source", "*"), "source"), color,
                _number(trace.get("scale", 1), "scale", -1e12, 1e12),
                _number(trace.get("offset", 0), "offset", -1e15, 1e15), match,
                unit, enabled, note, display_format, axis))
        plots.append(Plot(plot_title, _text(row.get("unit", "value"), name + " unit", 60),
                          tuple(parsed), tuple(limits) if limits else None, group, right_unit))
    if sum(len(plot.traces) for plot in plots) * integers["max_points"] > 500000:
        raise ValueError("Layout exceeds 500,000 total history points; reduce max_points or traces")
    return Layout(title, window, **integers, gap_seconds=gap, plots=tuple(plots[:len(rows)]),
                  document=document, readouts=tuple(plots[len(rows):]), auto_scale=auto_scale)


def load_layout(path):
    with Path(path).open("rb") as stream:
        data = stream.read(262145)
    if len(data) > 262144:
        raise ValueError("Flight layout exceeds 256 KiB")
    try:
        document = json.loads(data)
    except (ValueError, UnicodeError, RecursionError) as exc:
        raise ValueError(f"Invalid layout JSON: {exc}") from exc
    return validate_layout(document)


@dataclass
class History:
    points: deque
    latest: float | None = None
    received: float | None = None
    source: str = ""
    quality: str = "NO DATA"
    live: bool = False
    seen: float | None = None


class FlightData:
    def __init__(self, layout):
        self._lock = threading.Lock()
        self.configure(layout)

    def configure(self, layout):
        with self._lock:
            self.layout = layout
            self.selected = {}
            self.origins = {}
            self.aim_rates = AimAngularRateEstimator()
            self.aim_altitude = AimAltitudeEstimator()
            self.histories = [[History(deque(maxlen=layout.max_points if not plot.group else 1),
                                       quality="NO DATA" if trace.enabled else "UNAVAILABLE")
                               for trace in plot.traces] for plot in layout.rows]
            self.routes = {}
            for pi, plot in enumerate(layout.rows):
                for ti, trace in enumerate(plot.traces):
                    if trace.enabled:
                        for packet in trace.packet if isinstance(trace.packet, tuple) else (trace.packet,):
                            self.routes.setdefault((trace.input, packet), []).append((trace, self.histories[pi][ti]))

    def clear(self):
        self.configure(self.layout)

    def set_window(self, seconds):
        """Change only the view span; preserve data, source bindings and origins."""
        seconds = _number(seconds, "window_seconds", 5, 3600)
        with self._lock:
            self.layout = replace(self.layout, window_seconds=seconds,
                                  document={**self.layout.document, "window_seconds": seconds})

    def reset_pad(self):
        """Next valid live sample sets each source's origin. Other history stays."""
        with self._lock:
            self.origins.clear()
            for plot, histories in zip(self.layout.rows, self.histories):
                for trace, history in zip(plot.traces, histories):
                    if trace.enabled and trace.field == "_derived.horizontal_distance_m":
                        history.points.clear()
                        history.latest = history.received = None
                        history.live = False
                        history.quality = "NO DATA"

    def accept_record(self, kind, record):
        packet = record["packet"]
        fields = flatten_fields(packet)
        if kind == "witness":
            for key in ("status", "fc_status"):
                value = packet.get(key)
                if type(value) is int:
                    fields[f"{key}.flight_state"] = value & 7
                    for bit in range(8):
                        fields[f"{key}.byte0.bit{bit}"] = (value >> (8 + bit)) & 1
        received = record.get("received_mono")
        if received is None:
            received = time.monotonic() - max(0, time.time() - record["received_at"])
        valid = packet.get("text_crc_valid") is not False
        if kind == "featherweight":
            valid = valid and packet.get("radio_crc_status", "CRC_OK") == "CRC_OK"
            if packet.get("fix_type", 3) != 3:
                for key in ("altitude_ft", "latitude_deg", "longitude_deg", "vertical_velocity_fps", "horizontal_velocity_fps"):
                    if key in fields:
                        fields[key] = None  # Satellite/fix counts themselves remain useful without a fix.
        self.accept(kind, record["source"], packet["packet"], fields, received,
                    snapshot=bool(record.get("replay")), valid=valid)

    def accept_aim(self, source, topic, fields, received, retained):
        self.accept("aim", source, topic, fields, received, snapshot=retained)

    def accept(self, kind, source, packet, fields, received, *, snapshot=False, valid=True):
        if not isinstance(received, (int, float)) or not math.isfinite(received):
            return
        with self._lock:
            if kind == "aim" and packet == "orientation_payload/v1":
                fields = {**fields, **self.aim_rates.update(source, fields, received,
                          retained=snapshot, valid=valid, max_gap=self.layout.gap_seconds)}
            if kind == "aim" and packet == "pressure_payload/v1":
                fields = {**fields, **self.aim_altitude.update(source, fields, received,
                          retained=snapshot, valid=valid, max_gap=self.layout.gap_seconds)}
            if kind == "aim" and packet == "gnss_payload/v1":
                sats = fields.get("satellites_used", 4)
                if type(sats) not in (int, float) or not 4 <= sats <= 256:
                    fields = {**fields, "gnss_coordinate.altitude": None}
            fields = self._derive_distance(kind, source, fields, received, snapshot, valid)
            for trace, history in self.routes.get((kind, packet), ()):
                if trace.field not in fields or any(fields.get(key) != value for key, value in trace.match.items()):
                    continue
                if trace.source != "*" and trace.source != source:
                    continue
                # Auto means one stable source per INPUT, not interleaved COM ports
                # or live/test sources. Clear/reload releases this binding.
                if trace.source == "*" and self.selected.get(kind, source) != source:
                    continue
                value = fields[trace.field]
                numeric = type(value) in (int, float, bool)
                try:
                    value = float(value) * trace.scale + trace.offset if numeric else None
                    if value is not None and (not math.isfinite(value) or abs(value) > 1e15):
                        value = None
                except (ValueError, OverflowError):
                    value = None
                if not valid:
                    value = None
                if snapshot and history.live:
                    continue  # Cached/retained messages never refresh a live trace.
                if history.seen is not None and received < history.seen:
                    continue
                if not snapshot and value is not None and trace.source == "*":
                    self.selected.setdefault(kind, source)
                if history.source != source:
                    history.points.clear()
                    history.latest = history.received = None
                history.source, history.seen = source, received
                # Hold the last good display value and its ORIGINAL age. An
                # invalid sample still breaks the plotted line and sets a flag;
                # it must not turn a held number into a fresh measurement.
                if value is not None:
                    history.received, history.latest = received, value
                history.quality = "SNAPSHOT" if snapshot else "INVALID" if value is None else "LIVE"
                if not snapshot:
                    history.live = True
                    history.points.append((received, value))
                    # Keep history when zooming in, still bounded by max_points
                    # and the maximum selectable time span.
                    cutoff = received - 3600 - self.layout.gap_seconds
                    while history.points and history.points[0][0] < cutoff:
                        history.points.popleft()

    def _derive_distance(self, kind, source, fields, received, snapshot, valid):
        """Horizontal distance from first live position, reset by Set pad.

        Witness uses estimator X/Y meters; tracker GPS uses great-circle
        distance (Earth radius 6371000 m). Never initialize from retained data.
        """
        if kind == "witness":
            keys, mode = ("position[0]", "position[1]"), "local"
        elif kind == "featherweight":
            keys, mode = ("latitude_deg", "longitude_deg"), "gps"
        else:
            keys, mode = ("gnss_coordinate.latitude", "gnss_coordinate.longitude"), "gps"
        if not all(key in fields for key in keys):
            return fields
        fields = dict(fields, **{"_derived.horizontal_distance_m": None})
        x, y = (fields[key] for key in keys)
        if not valid or snapshot or not all(type(v) in (int, float) and abs(v) < 1e15 and math.isfinite(v) for v in (x, y)):
            return fields
        if mode == "gps" and (not -90 <= x <= 90 or not -180 <= y <= 180):
            return fields
        if kind == "aim":
            sats = fields.get("satellites_used", 4)
            if type(sats) not in (int, float) or not 4 <= sats <= 256 or not math.isfinite(sats):
                return fields
        key = (kind, source, fields.get("unit_type", ""), fields.get("unit_id", ""))
        if key not in self.origins:
            if len(self.origins) >= 128:
                return fields
            self.origins[key] = (x, y, received)
        ox, oy, last = self.origins[key]
        if received < last:
            return fields
        self.origins[key] = (ox, oy, received)
        if mode == "local":
            distance = math.hypot(x - ox, y - oy)
        else:
            lat, lon, olat, olon = map(math.radians, (x, y, ox, oy))
            hav = math.sin((lat - olat) / 2)**2 + math.cos(lat) * math.cos(olat) * math.sin((lon - olon) / 2)**2
            distance = 6371000 * 2 * math.asin(math.sqrt(max(0, min(1, hav))))
        fields["_derived.horizontal_distance_m"] = distance
        return fields

    def snapshot(self):
        with self._lock:
            return [[History(tuple(h.points), h.latest, h.received, h.source, h.quality, h.live, h.seen)
                     for h in row] for row in self.histories]


def nice_step(span, count=5):
    raw = max(span / count, 1e-12)
    magnitude = 10 ** math.floor(math.log10(raw))
    return next(step * magnitude for step in (1, 2, 2.5, 5, 10) if step * magnitude >= raw)


def graph_range(values, pixels, limits=None):
    """Tight data bounds, independent of tick spacing and absolute DC offset."""
    if limits is not None:
        low, high = limits
    elif values:
        low, high = min(values), max(values)
        padding = max((high - low) * .08, math.ulp(max(abs(low), abs(high))) * 4, 1e-9)
        if low == high:
            padding = max(.01, padding)
        low, high = low - padding, high + padding
    else:
        low, high = 0, 1
    # Never round limits out to whole tick steps: on short panels this used to
    # expand -60..0 to -100..100 and crush small changes around large offsets.
    return low, high, nice_step(high - low, max(2, int(pixels / 35)))


def plot_segments(points, start, end, pixels, gap):
    """Bound drawing cost with first/min/max/last per pixel; never hide peaks."""
    segments, segment, bucket = [], [], []
    previous = None
    column = None
    def flush_bucket():
        if bucket:
            indices = sorted({0, len(bucket) - 1,
                              min(range(len(bucket)), key=lambda i: bucket[i][1]),
                              max(range(len(bucket)), key=lambda i: bucket[i][1])})
            segment.extend(bucket[i] for i in indices)
            bucket.clear()
    for stamp, value in points:
        if stamp < start or stamp > end:
            continue
        if value is None or (previous is not None and stamp - previous > gap):
            flush_bucket()
            if segment:
                segments.append(segment)
            segment, column = [], None
        if value is not None:
            current = int((stamp - start) / (end - start) * max(1, pixels))
            if current != column:
                flush_bucket()
                column = current
            bucket.append((stamp, value))
        previous = stamp
    flush_bucket()
    if segment:
        segments.append(segment)
    return segments
