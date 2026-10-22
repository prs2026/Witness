"""Fixed-viewport dashboard renderer. No scrolling and no content-sized widgets."""

import datetime as dt
from functools import lru_cache
import math
from tkinter import font as tkfont

from flight_data import nice_step, plot_segments

BG = "#ffffff"
PANEL = "#ffffff"
GRID = "#b8b8b8"
BORDER = "#666666"
TEXT = "#171717"
MUTED = "#444444"
WARN = "#704000"
BAD = "#990000"
STATES = ("PAD IDLE", "BOOST", "COAST", "DESCENT", "LANDED")


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
    return f"{value:.5g}"


def quality_text(history, end):
    if history.received is None:
        return "N/A" if history.quality == "UNAVAILABLE" else "NO DATA", MUTED
    age = max(0, end - history.received)
    if history.quality != "LIVE":
        return f"{history.quality} {age:.1f}s", BAD if history.quality == "INVALID" else WARN
    return (f"STALE {age:.1f}s", BAD) if age >= 10 else (f"AGE {age:.1f}s", WARN) if age >= 2 else (f"LIVE {age:.1f}s", MUTED)


class DashboardRenderer:
    def __init__(self, canvas):
        self.canvas = canvas
        self.fonts = {}
        self.panel_bounds = {}
        self.text_boxes = []

    def font(self, size, bold=False):
        key = size, bold
        if key not in self.fonts:
            self.fonts[key] = tkfont.Font(root=self.canvas, family="Segoe UI", size=-size,
                                         weight="bold" if bold else "normal")
        return self.fonts[key]

    def text(self, box, text, size=14, color=TEXT, bold=False, align="left"):
        """Fit a single line INSIDE its assigned box; ellipsize only as a last resort."""
        x0, y0, x1, y1 = box
        width, height = max(1, x1 - x0 - 4), max(1, y1 - y0 - 2)
        text = str(text).replace("\n", " ")
        size = max(6, int(min(size, height * 0.8)))
        font = self.font(size, bold)
        while size > 6 and (font.measure(text) > width or font.metrics("linespace") > height):
            size -= 1
            font = self.font(size, bold)
        if font.measure(text) > width:
            while text and font.measure(text + "...") > width:
                text = text[:-1]
            text += "..."
        anchor = {"left": "w", "right": "e", "center": "center"}[align]
        x = x0 + 2 if align == "left" else x1 - 2 if align == "right" else (x0 + x1) / 2
        item = self.canvas.create_text(x, (y0 + y1) / 2, text=text, font=font,
                                       fill=readable_color(color), anchor=anchor)
        self.text_boxes.append((item, box))
        return item

    def panel(self, name, box):
        self.panel_bounds[name] = box
        self.canvas.create_rectangle(*box, fill=PANEL, outline=BORDER, width=1)

    def draw(self, layout, histories, end, wall_offset, *, width=None, height=None):
        c = self.canvas
        c.delete("all")
        self.panel_bounds, self.text_boxes = {}, []
        width = width or c.winfo_width()
        height = height or c.winfo_height()
        if width < 100 or height < 100:
            return
        gap, pad = 8, 5
        available = width - 2 * pad - 2 * gap
        left_w, center_w = available * 0.30, available * 0.51
        left = (pad, pad, pad + left_w, height - pad)
        cx = left[2] + gap
        right_x = cx + center_w + gap
        right = (right_x, pad, width - pad, height - pad)
        self.panel("flight", left)
        rows = list(zip(layout.rows, histories))
        self.flight(left, [(row, data) for row, data in rows if row.group in ("flight", "state")], end)
        split = pad + (height - 2 * pad) * 0.46
        gps_box = (right[0], right[1], right[2], split - gap / 2)
        power_box = (right[0], split + gap / 2, right[2], right[3])
        self.panel("gps", gps_box)
        self.panel("power", power_box)
        self.status_panel(gps_box, "GPS / SATELLITES", [(p, h) for p, h in rows if p.group == "gps"], end)
        self.status_panel(power_box, "POWER / CONTINUITY", [(p, h) for p, h in rows if p.group == "power"], end)
        # Default sketch: two columns, two tall rows and a shorter bottom row.
        columns = min(layout.columns, len(layout.plots))
        row_count = math.ceil(len(layout.plots) / columns)
        weights = (0.37, 0.37, 0.26) if row_count == 3 else [1 / row_count] * row_count
        usable_h = height - 2 * pad - (row_count - 1) * gap
        cell_w = (center_w - (columns - 1) * gap) / columns
        tops = [pad + usable_h * sum(weights[:r]) + r * gap for r in range(row_count)]
        for index, (plot, data) in enumerate(zip(layout.plots, histories)):
            row, col = divmod(index, columns)
            box = (cx + col * (cell_w + gap), tops[row], cx + col * (cell_w + gap) + cell_w,
                   tops[row] + usable_h * weights[row])
            self.panel(f"plot_{index}", box)
            self.graph(box, plot, data, layout, end, wall_offset)

    def flight(self, box, rows, end):
        x0, y0, x1, y1 = box
        x0, x1 = x0 + 10, x1 - 10
        w, h = x1 - x0, y1 - y0
        self.text((x0, y0 + 8, x1, y0 + h * .065), "FLIGHT OVERVIEW", 21, bold=True)
        self.text((x0, y0 + h * .065, x1, y0 + h * .10), "W = Witness    A = AIM    F = Featherweight", 12, MUTED)
        metrics = [(p, data) for p, data in rows if p.group == "flight"]
        for index, (row, data) in enumerate(metrics):
            top = y0 + h * (.12 + index * .155)
            self.text((x0, top, x1, top + h * .043), row.title, 18, bold=True)
            for ti, (trace, history) in enumerate(zip(row.traces, data)):
                cell_x = x0 + ti * w / len(data)
                cell_end = cell_x + w / len(data) - 4
                unit = trace.unit or row.unit
                self.text((cell_x, top + h * .047, cell_end, top + h * .094),
                          f"{trace.label}: {value_text(trace, history)}", 26, trace.color, True)
                status, color = quality_text(history, end)
                self.text((cell_x, top + h * .095, cell_end, top + h * .125), f"{unit} | {status}", 11, color)
        states = [(row, data) for row, data in rows if row.group == "state"]
        self.canvas.create_line(x0, y0 + h * .60, x1, y0 + h * .60, fill=GRID)
        self.text((x0, y0 + h * .62, x1, y0 + h * .67), "WITNESS FLIGHT STATE", 18, bold=True)
        if states:
            row, data = states[0]
            trace, history = row.traces[0], data[0]
            state, color = quality_text(history, end)
            self.text((x0, y0 + h * .68, x1, y0 + h * .78), value_text(trace, history), 46, trace.color, True)
            self.text((x0, y0 + h * .79, x1, y0 + h * .825), state, 16, color, True)
            self.text((x0, y0 + h * .83, x1, y0 + h * .865), history.source or "Waiting for Witness", 12, MUTED)
        else:
            self.text((x0, y0 + h * .68, x1, y0 + h * .78), "NOT CONFIGURED", 24, MUTED)
        self.text((x0, y0 + h * .90, x1, y0 + h * .93), "Distance: from first live position / Set pad", 12, MUTED)
        self.text((x0, y0 + h * .935, x1, y0 + h * .965), "Altitude: pressure reference / GPS ASL", 12, MUTED)
        self.text((x0, y0 + h * .965, x1, y1 - 4), "AIM raw units are not converted to SI", 11, WARN)

    def status_panel(self, box, title, rows, end):
        x0, y0, x1, y1 = box
        x0, x1 = x0 + 9, x1 - 9
        h = y1 - y0
        heading = min(32, h * .13)
        self.text((x0, y0 + 5, x1, y0 + heading), title, 17, bold=True)
        if not rows:
            self.text((x0, y0 + heading + 5, x1, y0 + heading + 30), "No readouts configured", 12, MUTED)
            return
        row_h = (h - heading - 10) / len(rows)
        for index, (row, data) in enumerate(rows):
            top = y0 + heading + index * row_h
            trace, history = row.traces[0], data[0]
            unit = trace.unit or row.unit
            value = value_text(trace, history)
            if trace.format == "number" and trace.enabled:
                value += " " + unit
            self.text((x0, top + 2, x1, top + row_h * .34), row.title, 14, MUTED)
            self.text((x0, top + row_h * .34, x1, top + row_h * .69), value, 21, trace.color, True)
            status, color = quality_text(history, end)
            note = trace.note if not trace.enabled else status
            self.text((x0, top + row_h * .69, x1, top + row_h - 3), note, 10, color)
            if index < len(rows) - 1:
                self.canvas.create_line(x0, top + row_h, x1, top + row_h, fill=GRID)

    def graph(self, box, plot, histories, layout, end, wall_offset):
        x0, y0, x1, y1 = box
        w, h = x1 - x0, y1 - y0
        self.text((x0 + 7, y0 + 3, x1 - 7, y0 + 25), f"{plot.title} [{plot.unit}]", 15, bold=True)
        legend_rows = math.ceil(len(histories) / 3)
        for i, (trace, history) in enumerate(zip(plot.traces, histories)):
            row, col = divmod(i, 3)
            columns = min(3, len(histories))
            lx = x0 + 7 + col * (w - 14) / columns
            _, color = quality_text(history, end)
            stale = history.received is not None and end - history.received >= 2
            marker = " !" if stale or history.quality in ("INVALID", "SNAPSHOT") else ""
            self.text((lx, y0 + 26 + row * 15, lx + (w - 14) / columns - 2, y0 + 40 + row * 15),
                      f"{trace.label} {value_text(trace, history)}{marker}", 11,
                      color if marker else trace.color)
        left, right = x0 + min(49, w * .22), x1 - 10
        top, bottom = y0 + 31 + legend_rows * 15, y1 - 24
        if right <= left or bottom - top < 8:
            return
        start = end - layout.window_seconds
        visible = [[(t, v) for t, v in hist.points if start <= t <= end] for hist in histories]
        values = [v for points in visible for _, v in points if v is not None]
        low, high = plot.y_range or (min(values, default=0), max(values, default=1))
        if plot.y_range is None:
            padding = max((high - low) * .08, abs(high) * .01, .01)
            low, high = low - padding, high + padding
            step = nice_step(high - low, 3)
            low, high = math.floor(low / step) * step, math.ceil(high / step) * step
        else:
            step = nice_step(high - low, 3)
        xm = lambda stamp: left + (stamp - start) / layout.window_seconds * (right - left)
        ym = lambda value: bottom - (max(low, min(high, value)) - low) / (high - low) * (bottom - top)
        for index in range(12):
            value = (math.ceil(low / step) + index) * step
            if value > high + step * 1e-6:
                break
            y = ym(value)
            self.canvas.create_line(left, y, right, y, fill=GRID)
            self.text((x0 + 2, y - 7, left - 4, y + 7), f"{value:.3g}", 10, MUTED, align="right")
        xstep = nice_step(layout.window_seconds, max(1, int((right - left) / 80)))
        first = math.ceil((start + wall_offset) / xstep) * xstep
        for index in range(20):
            stamp = first + index * xstep - wall_offset
            if stamp > end:
                break
            x = xm(stamp)
            self.canvas.create_line(x, top, x, bottom, fill=GRID)
            label = dt.datetime.fromtimestamp(stamp + wall_offset, dt.timezone.utc).strftime("%H:%M:%S")
            tx = max(left + 25, min(right - 25, x))
            self.text((tx - 27, bottom + 3, tx + 27, y1 - 3), label, 9, MUTED, align="center")
        clipped = False
        for trace, points in zip(plot.traces, visible):
            color = readable_color(trace.color)
            clipped |= any(v is not None and not low <= v <= high for _, v in points)
            for segment in plot_segments(points, start, end, right - left, layout.gap_seconds):
                coords = [coordinate for stamp, value in segment for coordinate in (xm(stamp), ym(value))]
                if len(coords) >= 4:
                    self.canvas.create_line(*coords, fill=color, width=2)
                elif coords:
                    x, y = coords
                    self.canvas.create_oval(x - 1, y - 1, x + 1, y + 1, fill=color, outline="")
        if not values or clipped:
            self.text((left + 3, top + 3, right - 3, min(top + 22, bottom)),
                      "OUT OF RANGE" if clipped else "NO LIVE SAMPLES", 11, BAD if clipped else MUTED,
                      align="center")
