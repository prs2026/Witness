"""Data board files for the CSV viewer: schema, validation and file I/O.

A data board is the viewer's plot dashboard (which columns sit in which
graph and on which y axis, the spectrograms docked between the graphs, the x
axis and each file's time axis) saved as JSON so the same view can be
rebuilt on another log. Columns are matched by name when a board is loaded.
No Qt here: the viewer builds and applies the dictionaries.

Schema, version 2 (version 1 files still load: a plot's ``axes`` was the
string "shared" or "separate", meaning one axis for all traces or one axis
each)::

    {"format": "horizon-csv-board", "version": 1,
     "x_axis": "time [s] (per-file time axes)", "packet_filter": "All",
     "graphs_per_row": 1, "sync": null | [from_state, to_state],
     "sources": [{"label": "S0000434.csv", "time_column": "...",
                  "time_unit": "s", "offset": 0.0}],
     "panels": [
       {"type": "plot", "height": null,
        "traces": [{"source": 0, "column": "alt_m"}, ...],
        "axes": [{"side": "left" | "right",
                  "traces": [{"source": 0, "column": "alt_m"}]}, ...]},
       {"type": "spectrogram", "trace": {"source": 0, "column": "ax"},
        "nfft": 1024, "overlap": 50, "window": "Hann", "detrend": "Mean",
        "welch": false, "colour_range": "auto" | [low_db, high_db],
        "time_window": "full" | [start_s, end_s]}]}

``source`` is the position of a file in the open-file order; the board is
applied to the files open when it is loaded, first file to source 0. Each
plot's ``axes`` lists its y axes (each autoranged over its own traces); every
trace of ``traces`` should be on exactly one; any that are not join the
first axis.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path

BOARD_FORMAT = "horizon-csv-board"
BOARD_VERSION = 2
BOARD_FILTER = "Data boards (*.board.json *.json);;All files (*)"
BOARD_SUFFIX = ".board.json"


class BoardError(ValueError):
    """A board file that cannot be used (not a board, newer version, malformed)."""


@dataclass
class BoardReport:
    """What loading a board did: what was applied and what had to be skipped."""
    missing: list[str] = field(default_factory=list)   # "file: column" not in the new log
    notes: list[str] = field(default_factory=list)     # other things not applied
    plots: int = 0
    spectrograms: int = 0

    @property
    def ok(self) -> bool:
        return not self.missing and not self.notes

    def text(self) -> str:
        lines = [f"Board loaded: {self.plots} graph(s), {self.spectrograms} spectrogram(s)."]
        if self.missing:
            lines.append("Not in this log, skipped: " + ", ".join(self.missing))
        lines.extend(self.notes)
        return "\n".join(lines)


def _number(value, name: str, default=None) -> float:
    if value is None and default is not None:
        return default
    if isinstance(value, bool) or not isinstance(value, (int, float)) \
            or not math.isfinite(value):
        raise BoardError(f"{name} must be a finite number")
    return float(value)


def _trace(raw, where: str) -> dict:
    if not isinstance(raw, dict) or not isinstance(raw.get("column"), str) or not raw["column"]:
        raise BoardError(f"{where}: a trace needs a column name")
    source = raw.get("source", 0)
    if isinstance(source, bool) or not isinstance(source, int) or source < 0:
        raise BoardError(f"{where}: bad source index")
    return {"source": source, "column": raw["column"]}


def _pair(raw, name: str):
    if (not isinstance(raw, (list, tuple)) or len(raw) != 2):
        raise BoardError(f"{name} must be [low, high]")
    low, high = _number(raw[0], name), _number(raw[1], name)
    if high <= low:
        raise BoardError(f"{name}: high must exceed low")
    return [low, high]


def _axes(raw, traces: list, where: str) -> list:
    """A plot's y axes as [{"side", "traces"}], from a version 2 list or the
    version 1 words "shared" / "separate"."""
    if isinstance(raw, list):
        groups = []
        for entry in raw:
            if not isinstance(entry, dict) or entry.get("side") not in ("left", "right"):
                raise BoardError(f"{where}: an axis needs a side (left or right)")
            groups.append({"side": entry["side"],
                           "traces": [_trace(t, where) for t in entry.get("traces") or []]})
        return groups
    if raw == "separate":                       # version 1: one axis per trace
        return [{"side": "left" if number == 0 else "right", "traces": [trace]}
                for number, trace in enumerate(traces)]
    return [{"side": "left", "traces": list(traces)}] if traces else []


def validate(data) -> dict:
    """The board as a normalised dictionary; BoardError if it is unusable."""
    if not isinstance(data, dict) or data.get("format") != BOARD_FORMAT:
        raise BoardError("this is not a Horizon data board file")
    version = data.get("version")
    if isinstance(version, bool) or not isinstance(version, int) or version < 1:
        raise BoardError("the board has no valid version")
    if version > BOARD_VERSION:
        raise BoardError(f"the board is version {version}; this viewer reads up to "
                         f"version {BOARD_VERSION}. Update Horizon to open it.")
    sources = []
    for raw in data.get("sources") or []:
        if not isinstance(raw, dict):
            raise BoardError("bad source entry")
        unit = raw.get("time_unit")
        sources.append({
            "label": str(raw.get("label", "")),
            "time_column": raw["time_column"] if isinstance(raw.get("time_column"), str) else None,
            "time_unit": unit if isinstance(unit, str) else None,
            "offset": _number(raw.get("offset"), "offset", 0.0)})
    panels = []
    for number, raw in enumerate(data.get("panels") or [], 1):
        where = f"panel {number}"
        if not isinstance(raw, dict):
            raise BoardError(f"{where}: not an object")
        kind = raw.get("type")
        if kind == "plot":
            traces = [_trace(t, where) for t in raw.get("traces") or []]
            height = raw.get("height")
            if height is not None:
                height = int(_number(height, "height"))
            panels.append({"type": "plot", "traces": traces, "height": height,
                           "axes": _axes(raw.get("axes"), traces, where)})
        elif kind == "spectrogram":
            colour = raw.get("colour_range", "auto")
            window = raw.get("time_window", "full")
            panels.append({
                "type": "spectrogram", "trace": _trace(raw.get("trace"), where),
                "nfft": int(_number(raw.get("nfft"), "nfft", 1024)),
                "overlap": int(_number(raw.get("overlap"), "overlap", 50)),
                "window": str(raw.get("window", "Hann")),
                "detrend": str(raw.get("detrend", "Mean")),
                "welch": bool(raw.get("welch", False)),
                "colour_range": "auto" if colour == "auto" else _pair(colour, "colour_range"),
                "time_window": "full" if window == "full" else _pair(window, "time_window")})
        else:
            raise BoardError(f"{where}: unknown panel type {kind!r}")
    for panel in panels:
        traces = panel["traces"] if panel["type"] == "plot" else [panel["trace"]]
        for trace in traces:
            if sources and trace["source"] >= len(sources):
                raise BoardError("a trace names a source the board does not list")
    columns = data.get("graphs_per_row", 1)
    sync = data.get("sync")
    if sync is not None:
        if (not isinstance(sync, (list, tuple)) or len(sync) != 2
                or not all(isinstance(v, int) and not isinstance(v, bool) for v in sync)):
            raise BoardError("sync must be [from_state, to_state]")
        sync = [int(sync[0]), int(sync[1])]
    return {"sync": sync, "format": BOARD_FORMAT, "version": version,
            "x_axis": data["x_axis"] if isinstance(data.get("x_axis"), str) else None,
            "packet_filter": data["packet_filter"]
            if isinstance(data.get("packet_filter"), str) else None,
            "graphs_per_row": columns if isinstance(columns, int) and columns >= 1 else 1,
            "sources": sources, "panels": panels}


def read_board(path) -> dict:
    """Load and validate a board file (BoardError for any problem with it)."""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError) as exc:
        raise BoardError(f"cannot read {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise BoardError(f"{Path(path).name} is not valid JSON: {exc}") from exc
    return validate(data)


def write_board(path, board: dict) -> None:
    """Write a board (validated first) atomically."""
    validate(board)
    path = Path(path)
    temp = path.with_name(path.name + ".tmp")
    temp.write_text(json.dumps(board, indent=2) + "\n", encoding="utf-8")
    os.replace(temp, path)
