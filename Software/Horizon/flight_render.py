"""Fixed-viewport dashboard renderer. No scrolling and no content-sized widgets."""

import datetime as dt
from functools import lru_cache
import math
from tkinter import font as tkfont

from flight_data import graph_range, nice_step, plot_segments

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
    if "voltage" in trace.field.lower():
        return f"{value:.3f}"
    return f"{value:.5g}"


def quality_text(history, end):
    if history.received is None:
        if history.quality == "INVALID":
            return "INVALID / NO DATA", BAD
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
        self.min_font = 12
        self.compact = True

    def font(self, size, bold=False):
        key = size, bold
        if key not in self.fonts:
            self.fonts[key] = tkfont.Font(root=self.canvas, family="Segoe UI", size=-size,
                                         weight="bold" if bold else "normal")
        return self.fonts[key]

    def text(self, box, text, size=14, color=TEXT, bold=False, align="left"):
        """Keep a readable pixel floor; shorten labels rather than shrinking to 6px."""
        x0, y0, x1, y1 = box
        width, height = max(1, x1 - x0 - 4), max(1, y1 - y0)
        text = str(text).replace("\n", " ")
        size = max(self.min_font, int(min(size, height * 0.8)))
        font = self.font(size, bold)
        while size > self.min_font and (font.measure(text) > width or font.metrics("linespace") > height):
            size -= 1
            font = self.font(size, bold)
        if font.metrics("linespace") > height:
            return None  # Ancillary text cannot overlap a neighboring row.
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
        self.compact = width < 1450 or height < 700
        self.min_font = max(12, min(16, int(min(width / 120, height / 60))))
        gap, pad = 8, 5
        available = width - 2 * pad - 2 * gap
        left_w = available * (.26 if self.compact else .28)
        right_w = max(190, min(300, available * .19))
        center_w = available - left_w - right_w
        left = (pad, pad, pad + left_w, height - pad)
        cx = left[2] + gap
        right_x = cx + center_w + gap
        right = (right_x, pad, width - pad, height - pad)
        self.panel("flight", left)
        rows = list(zip(layout.rows, histories))
        self.flight(left, [(row, data) for row, data in rows if row.group in ("flight", "state")], end)
        gps_count = sum(row.group == "gps" for row in layout.rows)
        split = (pad + 38 + max(20, self.min_font + 8) * max(1, gps_count) if self.compact
                 else pad + (height - 2 * pad) * .40)
        gps_box = (right[0], right[1], right[2], split - gap / 2)
        power_box = (right[0], split + gap / 2, right[2], right[3])
        self.panel("gps", gps_box)
        self.panel("power", power_box)
        self.status_panel(gps_box, "GPS / SATELLITES", [(p, h) for p, h in rows if p.group == "gps"], end)
        self.status_panel(power_box, "POWER", [(p, h) for p, h in rows if p.group == "power"], end)
        # Equal graph heights; the narrower status column gives graphs more width.
        columns = min(layout.columns, len(layout.plots))
        row_count = math.ceil(len(layout.plots) / columns)
        weights = [1 / row_count] * row_count
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
        if self.compact:
            self.compact_flight(box, rows, end)
            return
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
        self.text((x0, y0 + h * .965, x1, y1 - 4), "A baro / Vz: estimates; GPS units raw", 11, WARN)

    def compact_flight(self, box, rows, end):
        x0, y0, x1, y1 = box
        x0, x1 = x0 + 8, x1 - 8
        h = y1 - y0
        self.text((x0, y0 + 2, x1, y0 + 25), "FLIGHT OVERVIEW", 19, bold=True)
        self.text((x0, y0 + 25, x1, y0 + 45), "W Witness / A AIM / F FW", 12, MUTED)
        metrics = [(row, data) for row, data in rows if row.group == "flight"]
        for i, (row, data) in enumerate(metrics):
            top = y0 + h * (.105 + i * .19)
            self.text((x0, top, x1, top + 20), row.title, 16, bold=True)
            line_h = max(19, min(26, (h * .19 - 23) / 3))
            for ti, (trace, history) in enumerate(zip(row.traces, data)):
                y = top + 21 + ti * line_h
                cut = x0 + (x1 - x0) * .70
                unit = trace.unit or row.unit
                self.text((x0, y, cut, y + line_h),
                          f"{trace.label}: {value_text(trace, history)} {unit}", 17, trace.color, True)
                status, color = quality_text(history, end)
                short = (f"!{max(0, end-history.received):.0f}s" if status.startswith(("AGE", "STALE"))
                         else "NO DATA" if status == "NO DATA" else status.split()[0])
                self.text((cut, y, x1, y + line_h), short, 12, color, align="right")
        top = y0 + h * .70
        self.canvas.create_line(x0, top, x1, top, fill=GRID)
        self.text((x0, top + 3, x1, top + 24), "WITNESS STATE", 16, bold=True)
        states = [(row, data) for row, data in rows if row.group == "state"]
        if states:
            row, data = states[0]
            trace, history = row.traces[0], data[0]
            self.text((x0, top + 25, x1, top + 65), value_text(trace, history), 34, trace.color, True)
            status, color = quality_text(history, end)
            self.text((x0, top + 66, x1, top + 86), status, 14, color)
            self.text((x0, top + 87, x1, top + 107), history.source or "Waiting for Witness", 12, MUTED)
        else:
            self.text((x0, top + 25, x1, top + 65), "NOT CONFIGURED", 19, MUTED)
        self.text((x0, y1 - 24, x1, y1 - 3), "A baro / Vz estimated; GPS raw", 12, WARN)

    def status_panel(self, box, title, rows, end):
        x0, y0, x1, y1 = box
        x0, x1 = x0 + 9, x1 - 9
        h = y1 - y0
        heading = 30 if self.compact else 32
        self.text((x0, y0 + 5, x1, y0 + heading), title, 17, bold=True)
        if not rows:
            self.text((x0, y0 + heading + 5, x1, y0 + heading + 30), "No readouts configured", 12, MUTED)
            return
        row_h = (h - heading - 10) / len(rows)
        for index, (row, data) in enumerate(rows):
            top = y0 + heading + index * row_h
            if len(data) == 2:
                # Paired power supplies share one row, but keep independent
                # values and stale/invalid markers (never combine timestamps).
                self.text((x0, top, x1, top + row_h * .42), row.title, 14, MUTED)
                for ti, (trace, history) in enumerate(zip(row.traces, data)):
                    a = x0 + (x1 - x0) * ti / 2
                    b = x0 + (x1 - x0) * (ti + 1) / 2 - 2
                    status, color = quality_text(history, end)
                    marker = "!" if status.startswith(("AGE", "STALE", "INVALID", "SNAPSHOT")) else ""
                    unit = trace.unit or row.unit
                    self.text((a, top + row_h * .42, b, top + row_h - 3),
                              f"{value_text(trace, history)}{marker} {unit}", 14,
                              color if marker else trace.color, True)
                if index < len(rows) - 1:
                    self.canvas.create_line(x0, top + row_h, x1, top + row_h, fill=GRID)
                continue
            trace, history = row.traces[0], data[0]
            unit = trace.unit or row.unit
            value = value_text(trace, history)
            if trace.format == "number" and trace.enabled:
                value += " " + unit
            if self.compact or row_h < 3 * (self.font(self.min_font).metrics("linespace") + 3):
                # Two lines per row, with quality alongside the value. Eight
                # power readings stay legible on a 1000x650 application window.
                label = row.title.replace("Featherweight", "FW").replace("Witness", "W").replace("computer", "CPU")
                if title == "GPS / SATELLITES" or row_h < 2 * self.font(self.min_font).metrics("linespace") + 2:
                    cut = x0 + (x1 - x0) * .65
                    status, color = quality_text(history, end)
                    marker = " !" if status.startswith(("AGE", "STALE", "INVALID", "SNAPSHOT")) else ""
                    self.text((x0, top, cut, top + row_h), label, 14, MUTED)
                    self.text((cut, top, x1, top + row_h), value + marker, 14,
                              color if marker else trace.color, True, align="right")
                    continue
                self.text((x0, top, x1, top + row_h / 2), label, 14, MUTED)
                cut = x0 + (x1 - x0) * .55
                self.text((x0, top + row_h / 2, cut, top + row_h), value, 16, trace.color, True)
                status, color = quality_text(history, end)
                self.text((cut, top + row_h / 2, x1, top + row_h), status, 12, color, align="right")
                if index < len(rows) - 1:
                    self.canvas.create_line(x0, top + row_h, x1, top + row_h, fill=GRID)
                continue
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
        units = plot.unit + (f" | {plot.right_unit}" if plot.right_unit else "")
        self.text((x0 + 5, y0 + 2, x1 - 5, y0 + 25), f"{plot.title} [{units}]", 15, bold=True)
        columns = min(2 if self.compact else 3, len(histories))
        legend_h = self.min_font + 7
        legend_rows = math.ceil(len(histories) / columns)
        for i, (trace, history) in enumerate(zip(plot.traces, histories)):
            row, col = divmod(i, columns)
            lx = x0 + 5 + col * (w - 10) / columns
            _, color = quality_text(history, end)
            stale = history.received is not None and end - history.received >= 2
            marker = "!" if stale or history.quality in ("INVALID", "SNAPSHOT") else ""
            self.text((lx, y0 + 26 + row * legend_h, lx + (w - 10) / columns - 2,
                       y0 + 26 + (row + 1) * legend_h),
                      f"{trace.label} {value_text(trace, history)}{marker}", self.min_font,
                      color if marker else trace.color)
        margin = max(40, self.min_font * 3.5)
        left, right = x0 + margin, x1 - (margin if plot.right_unit else 10)
        top, bottom = y0 + 30 + legend_rows * legend_h, y1 - (self.min_font + 12)
        if right <= left or bottom - top < 8:
            return
        start = end - layout.window_seconds
        visible = [[(t, v) for t, v in hist.points if start <= t <= end] for hist in histories]
        values = [v for points in visible for _, v in points if v is not None]
        ranges = {}
        for axis in ("left", "right"):
            axis_values = [v for trace, points in zip(plot.traces, visible) if trace.axis == axis
                           for _, v in points if v is not None]
            limits = plot.y_range if axis == "left" and not layout.auto_scale else None
            ranges[axis] = graph_range(axis_values, bottom - top, limits)
        xm = lambda stamp: left + (stamp - start) / layout.window_seconds * (right - left)
        def ym(value, axis):
            low, high, _ = ranges[axis]
            return bottom - (max(low, min(high, value)) - low) / (high - low) * (bottom - top)
        for axis in ("left", "right") if plot.right_unit else ("left",):
            low, high, step = ranges[axis]
            for index in range(12):
                value = (math.ceil(low / step) + index) * step
                if value > high + step * 1e-6:
                    break
                y = ym(value, axis)
                if axis == "left":
                    self.canvas.create_line(left, y, right, y, fill=GRID)
                    label_box = (x0 + 1, y - 10, left - 3, y + 10)
                else:
                    label_box = (right + 3, y - 10, x1 - 1, y + 10)
                precision = max(0, min(9, -math.floor(math.log10(step)) + 1))
                label = f"{value:.{precision}f}" if abs(value) < 1e5 else f"{value:.8g}"
                self.text(label_box, label, self.min_font,
                          BAD if axis == "right" else MUTED, align="right" if axis == "left" else "left")
        label_w = self.font(self.min_font).measure("00:00:00") + 8
        xstep = nice_step(layout.window_seconds, max(1, int((right - left) / (label_w + 8))))
        first = math.ceil((start + wall_offset) / xstep) * xstep
        last_label_end = -math.inf
        for index in range(20):
            stamp = first + index * xstep - wall_offset
            if stamp > end:
                break
            x = xm(stamp)
            self.canvas.create_line(x, top, x, bottom, fill=GRID)
            label = dt.datetime.fromtimestamp(stamp + wall_offset, dt.timezone.utc).strftime("%H:%M:%S")
            tx = max(left + label_w / 2, min(right - label_w / 2, x))
            if tx - label_w / 2 >= last_label_end + 4:
                self.text((tx - label_w / 2, bottom + 3, tx + label_w / 2, y1 - 2),
                          label, self.min_font, MUTED, align="center")
                last_label_end = tx + label_w / 2
        clipped = False
        for trace, points in zip(plot.traces, visible):
            color = readable_color(trace.color)
            low, high, _ = ranges[trace.axis]
            clipped |= any(v is not None and not low <= v <= high for _, v in points)
            for segment in plot_segments(points, start, end, right - left, layout.gap_seconds):
                coords = [coordinate for stamp, value in segment for coordinate in (xm(stamp), ym(value, trace.axis))]
                if len(coords) >= 4:
                    self.canvas.create_line(*coords, fill=color, width=2,
                                            dash=(6, 3) if trace.input == "aim" else (),
                                            tags=("trace", trace.input, trace.axis))
                elif coords:
                    x, y = coords
                    self.canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill=color, outline="",
                                            tags=("trace", trace.input, trace.axis))
        if not values or clipped:
            self.text((left + 3, top + 2, right - 3, min(top + 24, bottom)),
                      "OUT OF RANGE" if clipped else "NO LIVE SAMPLES", self.min_font,
                      BAD if clipped else MUTED, align="center")
