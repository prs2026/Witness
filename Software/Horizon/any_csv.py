"""Load arbitrary CSV files as numpy columns (other flight computers, GoPro...).

converter.py's CSVs go through csv_columns.load_table. Everything else, such
as Blue Raven LR/HR exports, GoPro telemetry CSVs, semicolon or tab separated
logs and headerless dumps, goes through ``load`` here. It returns a
csv_columns.Table (a ``GenericTable``), so the viewer handles both the same way.

What ``sniff`` works out from the first 512 KB and the last 256 KB of the file:

* the encoding (a UTF-8 BOM is skipped; UTF-16 is decoded) and the delimiter
  (comma, semicolon, tab or pipe; semicolon/tab files may use decimal commas);
* leading comment lines (``#``, ``//``, ``%``) and metadata lines with fewer
  fields than the table, which GoPro telemetry exports often have;
* whether there is a header at all (an all-numeric first row gets
  ``col1..colN``) and whether a units row follows it;
* unique column names: duplicates become ``LT_AGL1``, ``LT_AGL1#2``...,
  runs of whitespace collapse, and blank names become ``colN``;
* a kind per column: ``num``, ``hex`` (``8c5``, ``0x1F``), ``tod`` (time of
  day, ``12:26:22.422``), ``datetime`` (ISO 8601) or ``text``.

Hex columns become numbers (or text with ``hex_as_numbers=False``). Times of
day become seconds since midnight, and ISO datetimes become seconds since
midnight of the column's first date. Separate Year/Month/Day columns, or an
ISO date column, combine with a time-of-day column into a ``datetime``
column. Text columns with up to MAX_TEXT_CATEGORIES distinct values are kept
for the hover readout; empty cells and NaN/NA/null are missing.

The bulk parse uses the same chunked numpy ``loadtxt`` as csv_columns, so a
200k-row file loads in well under a second. Odd files, such as ragged rows,
comment lines in the data or junk in a numeric column, fall back to a
``csv.reader`` pass that gives the same columns.
"""

from __future__ import annotations

import csv
import io
import math
import os
import re
from array import array
from collections import Counter
from dataclasses import dataclass, field, replace
from datetime import date, datetime, timedelta

import numpy as np

import csv_columns
from csv_columns import LoadCancelled, Table, TextColumn

DELIMITERS = (";", "\t", ",", "|")          # tie-break order (see _pick_delimiter)
DELIMITER_NAMES = {",": "comma", ";": "semicolon", "\t": "tab", "|": "pipe"}
COMMENT_PREFIXES = ("#", "//", "%")
MISSING = frozenset(("", "nan", "-nan", "na", "n/a", "#n/a", "null", "none", "-", "--", "?"))
HEAD_BYTES = 512 * 1024
TAIL_BYTES = 256 * 1024
MAX_SAMPLE_LINES = 4000
MAX_TEXT_CATEGORIES = 4096
MAX_HEX_DIGITS = 13                          # 52 bits: exact as float64
BAD_NUMERIC_FRACTION = 0.01                  # robust path: more junk -> not numeric

TOD_RE = re.compile(r"^(\d{1,3}):([0-5]\d):([0-5]\d(?:[.,]\d+)?)$")
ISO_RE = re.compile(
    r"^(\d{4})[-/](\d{1,2})[-/](\d{1,2})"
    r"(?:[T ](\d{1,2}):(\d{2})(?::(\d{2}(?:[.,]\d+)?))?)?"
    r"\s*(Z|[+-]\d{2}:?\d{2})?$", re.IGNORECASE)
PLAIN_ISO_RE = re.compile(r"^\d{4}-\d{2}-\d{2}(?:[T ]\d{2}:\d{2}(?::\d{2}(?:\.\d+)?)?)?$")
HEX_RE = re.compile(r"^(0x)?[0-9a-f]+$", re.IGNORECASE)
UNIT_RE = re.compile(r"^(.*?)[\s_]*[(\[]([^()\[\]]*)[)\]]\s*$")

TIME_UNIT_ALIASES = {
    "s": "s", "sec": "s", "secs": "s", "second": "s", "seconds": "s",
    "ms": "ms", "msec": "ms", "msecs": "ms", "millis": "ms", "millisecond": "ms",
    "milliseconds": "ms",
    "us": "us", "µs": "us", "μs": "us", "usec": "us", "micros": "us",
    "microsecond": "us", "microseconds": "us",
    "ns": "ns", "nsec": "ns", "nanoseconds": "ns",
}


# --------------------------------------------------------------- scalars

def is_missing(text: str) -> bool:
    return text.strip().lower() in MISSING


def parse_number(text: str, decimal_comma: bool = False) -> float:
    """float(text), NaN when missing, not a number or not finite."""
    text = text.strip()
    if decimal_comma:
        text = text.replace(",", ".")
    try:
        value = float(text)
    except ValueError:
        return math.nan
    return value if math.isfinite(value) else math.nan


def _is_number(text: str, decimal_comma: bool) -> bool:
    if decimal_comma:
        text = text.replace(",", ".")
    try:
        float(text)
    except ValueError:
        return False
    return True


def parse_time_of_day(text: str) -> float:
    """``HH:MM:SS[.fff]`` -> seconds since midnight (NaN if it is not one)."""
    match = TOD_RE.match(text.strip())
    if match is None:
        return math.nan
    hours, minutes, seconds = match.groups()
    return int(hours) * 3600 + int(minutes) * 60 + float(seconds.replace(",", "."))


def times_of_day(values) -> np.ndarray:
    """parse_time_of_day for a whole array, vectorized for the usual case.

    Values of one length with the colons in one place (``12:26:22.422``)
    are decoded from their bytes in numpy; anything else goes through
    parse_time_of_day one value at a time.
    """
    values = np.char.strip(np.asarray(values, dtype=str))
    out = np.full(values.size, np.nan)
    if values.size == 0:
        return out
    done = np.zeros(values.size, bool)
    lengths = np.char.str_len(values)
    for length in np.unique(lengths).tolist():
        rows = np.flatnonzero(lengths == length)
        if length < 7:
            continue
        try:
            raw = values[rows].astype(f"S{length}")
        except UnicodeEncodeError:
            continue
        b = raw.view(np.uint8).reshape(-1, length).astype(np.int64)
        colon = np.argmax(b == 58, axis=1)            # hour digits before it
        for h in np.unique(colon).tolist():
            if not 1 <= h <= 3 or length < h + 6 or (length > h + 6 and length < h + 8):
                continue
            sub = np.flatnonzero(colon == h)
            d = b[sub] - 48
            ok = (b[sub, h + 3] == 58)
            digit_cols = list(range(h)) + [h + 1, h + 2, h + 4, h + 5] + list(
                range(h + 7, length))
            ok &= np.all((d[:, digit_cols] >= 0) & (d[:, digit_cols] <= 9), axis=1)
            ok &= (d[:, h + 1] <= 5) & (d[:, h + 4] <= 5)
            if length > h + 6:
                ok &= np.isin(b[sub, h + 6], (46, 44))      # '.' or ','
            hours = sum(d[:, k] * 10 ** (h - 1 - k) for k in range(h))
            value = (hours * 3600 + (d[:, h + 1] * 10 + d[:, h + 2]) * 60
                     + d[:, h + 4] * 10 + d[:, h + 5]).astype(np.float64)
            if length > h + 7:
                places = length - h - 7
                fraction = sum(d[:, k] * 10 ** (length - 1 - k) for k in range(h + 7, length))
                value = value + fraction / 10.0 ** places
            target = rows[sub[ok]]
            out[target] = value[ok]
            done[target] = True
    for i in np.flatnonzero(~done).tolist():
        text = values[i]
        out[i] = math.nan if is_missing(text) else parse_time_of_day(text)
    return out


def format_time_of_day(seconds: float) -> str:
    """Seconds since midnight -> ``HH:MM:SS.fff`` (``d+N`` past midnight)."""
    if not math.isfinite(seconds):
        return ""
    days, rest = divmod(seconds, 86400.0)
    hours, rest = divmod(rest, 3600.0)
    minutes, secs = divmod(rest, 60.0)
    text = f"{int(hours):02d}:{int(minutes):02d}:{secs:06.3f}"
    return text if days == 0 else f"d{int(days):+d} {text}"


def parse_hex(text: str) -> float:
    text = text.strip()
    if not HEX_RE.match(text):
        return math.nan
    digits = text[2:] if text[:2].lower() == "0x" else text
    if len(digits) > MAX_HEX_DIGITS:
        return math.nan
    return float(int(digits, 16))


def _iso_datetime(text: str) -> datetime | None:
    """An ISO 8601 date or datetime as a naive UTC datetime (None if not)."""
    text = text.strip()
    match = ISO_RE.match(text)
    if match is None:
        return None
    year, month, day, hour, minute, second, zone = match.groups()
    try:
        seconds = float((second or "0").replace(",", "."))
        whole = int(seconds)
        value = datetime(int(year), int(month), int(day), int(hour or 0), int(minute or 0),
                         whole, int(round((seconds - whole) * 1e6)) if seconds != whole else 0)
    except ValueError:
        return None
    if zone and zone.upper() != "Z":
        sign = 1 if zone[0] == "+" else -1
        digits = zone[1:].replace(":", "")
        offset = sign * (int(digits[:2]) * 3600 + int(digits[2:]) * 60)
        value -= timedelta(seconds=offset)          # to UTC
    return value


def _midnight(value: datetime) -> datetime:
    return datetime(value.year, value.month, value.day)


def iso_seconds(values, reference: datetime | None) -> np.ndarray:
    """ISO datetimes -> seconds since ``reference`` (NaN where unparseable)."""
    values = [str(v).strip() for v in values]
    out = np.full(len(values), np.nan)
    if reference is None:
        return out
    ref = np.datetime64(reference, "us")
    # Fast path: numpy parses plain ISO strings (no zone, '-' separators) in C.
    plain = [v if PLAIN_ISO_RE.match(v) else "NaT" for v in values]
    try:
        stamps = np.array(plain, dtype="datetime64[us]")
        good = ~np.isnat(stamps)
        out[good] = (stamps[good] - ref).astype(np.int64) / 1e6
        todo = [i for i, (v, p) in enumerate(zip(values, plain))
                if p == "NaT" and not is_missing(v)]
    except ValueError:
        todo = range(len(values))
    for i in todo:
        parsed = _iso_datetime(values[i])
        if parsed is not None:
            out[i] = (parsed - reference).total_seconds()
    return out


# ----------------------------------------------------------- names/units

def split_unit(name: str) -> tuple[str, str]:
    """``"Flight_Time_(s)"`` -> (``"Flight_Time"``, ``"s"``); no unit -> ""."""
    match = UNIT_RE.match(name.strip())
    if match is None or not match.group(1).strip():
        return name.strip(), ""
    return match.group(1).strip(), match.group(2).strip()


def name_key(name: str) -> str:
    """A column name without its unit, lower case, words joined by ``_``."""
    base, _unit = split_unit(name.split("#")[0] if "#" in name else name)
    return re.sub(r"[^0-9a-z]+", "_", base.lower()).strip("_")


def time_unit_of(name: str) -> str | None:
    """Time unit given in a header (``time (ms)``, ``t [s]``), else None."""
    _base, unit = split_unit(name)
    return TIME_UNIT_ALIASES.get(unit.lower().replace(" ", ""))


def unique_names(names) -> list[str]:
    """Clean header names, made unique as ``name``, ``name#2``, ``name#3``..."""
    out: list[str] = []
    taken: set[str] = set()
    counts: dict[str, int] = {}
    for index, raw in enumerate(names):
        name = " ".join(str(raw).replace("﻿", "").split()) or f"col{index + 1}"
        if name in taken:
            count = counts.get(name, 1)
            while True:
                count += 1
                candidate = f"{name}#{count}"
                if candidate not in taken:
                    break
            counts[name] = count
            name = candidate
        taken.add(name)
        out.append(name)
    return out


# Auto-picked time columns, most preferred first: (name key, unit when the
# header gives none). Seconds unless a unit is given or implied.
TIME_COLUMN_NAMES = (
    ("flight_time", "s"), ("time", "s"), ("t", "s"), ("timestamp", "s"),
    ("time_s", "s"), ("elapsed", "s"), ("elapsed_time", "s"), ("seconds", "s"),
    ("sec", "s"), ("time_sec", "s"),
    ("milliseconds", "ms"), ("millis", "ms"), ("ms", "ms"), ("time_ms", "ms"),
    ("timestamp_ms", "ms"), ("cts", "ms"),
    ("microseconds", "us"), ("micros", "us"), ("us", "us"), ("time_us", "us"),
    ("timestamp_us", "us"),
    ("datetime", "s"), ("date_time", "s"), ("date", "s"),
)


def pick_time_column(columns, kinds: dict | None = None):
    """(column, unit) of the likeliest time axis among ``columns``, or None.

    Recognises Flight_Time_(s), time, t, timestamp, Milliseconds/ms, GoPro's
    cts (ms), date/datetime and similar, then any other name starting with
    "time" or containing "timestamp".
    """
    kinds = kinds or {}
    columns = list(columns)
    keys = {column: name_key(column) for column in columns}

    def unit_for(column, default):
        if kinds.get(column) in ("tod", "datetime"):
            return "s"
        return time_unit_of(column) or default

    for key, default in TIME_COLUMN_NAMES:
        for column in columns:
            if keys[column] == key:
                return column, unit_for(column, default)
    for column in columns:
        key = keys[column]
        if key.startswith("time") or "timestamp" in key or key.endswith("_time"):
            unit = "ms" if key.endswith("_ms") else "us" if key.endswith("_us") else "s"
            return column, unit_for(column, unit)
    return None


# ----------------------------------------------------------------- sniff

@dataclass
class Sniff:
    """How a CSV is laid out (see the module docstring)."""
    encoding: str
    delimiter: str
    data_offset: int                 # byte offset of the first data line
    data_line: int                   # physical line number of the first data line
    skipped: int                     # comment/metadata lines before the header
    has_header: bool
    units_row: bool
    raw_header: list[str]
    names: list[str]
    kinds: list[str]
    decimal_comma: bool = False
    avg_line: float = 100.0          # bytes per data line (sampled)
    missing_tokens: bool = False     # NA / null / - cells csv_columns would call text
    tod_reference: dict = field(default_factory=dict)     # datetime column -> midnight

    @property
    def is_plain(self) -> bool:
        """True for a CSV csv_columns.load_table reads exactly as intended."""
        if (self.delimiter != "," or self.encoding != "utf-8" or self.skipped
                or not self.has_header or self.units_row or self.decimal_comma):
            return False
        if self.missing_tokens or [name.strip() for name in self.raw_header] != self.names:
            return False
        # Converted Witness CSVs keep csv_columns' handling, so a CSV and the
        # .BIN it came from load identically (packet_id "0x02" stays text).
        if {"packet_type", "packet_id"} & set(self.names):
            return True
        return all(kind == "num" for kind in self.kinds)

    def describe(self) -> str:
        parts = [f"{DELIMITER_NAMES.get(self.delimiter, repr(self.delimiter))}-separated"]
        if self.decimal_comma:
            parts.append("decimal commas")
        if self.encoding != "utf-8":
            parts.append(self.encoding)
        if self.skipped:
            parts.append(f"{self.skipped} line(s) before the header skipped")
        if not self.has_header:
            parts.append("no header (col1..)")
        if self.units_row:
            parts.append("units row")
        renamed = sum(1 for raw, name in zip(self.raw_header, self.names)
                      if raw.strip() != name)
        if self.has_header and renamed:
            parts.append(f"{renamed} column name(s) made unique/cleaned")
        return ", ".join(parts)


def _open_bytes(path: str):
    """(binary file, size, encoding); UTF-16 files are decoded to UTF-8 first."""
    with open(path, "rb") as f:
        start = f.read(4)
    if start[:2] in (b"\xff\xfe", b"\xfe\xff"):
        with open(path, encoding="utf-16", errors="replace") as f:
            data = f.read().encode("utf-8")
        return io.BytesIO(data), len(data), "utf-16"
    return open(path, "rb"), os.path.getsize(path), "utf-8"


def _decode(line: bytes) -> str:
    return line.decode("utf-8", "replace").rstrip("\r")


def _fields(line: str, delimiter: str) -> list[str]:
    return next(csv.reader([line], delimiter=delimiter), [])


def _is_comment(line: str) -> bool:
    stripped = line.strip()
    return not stripped or stripped.startswith(COMMENT_PREFIXES)


def _pick_delimiter(lines: list[str]) -> str:
    best, best_score = ",", -1.0
    for delimiter in DELIMITERS:
        counts = [len(_fields(line, delimiter)) for line in lines]
        if not counts:
            continue
        # The table's own width shows in the later lines (metadata lines
        # come first), so judge consistency on the second half.
        tail = counts[len(counts) // 2:] or counts
        modal, hits = Counter(tail).most_common(1)[0]
        if modal < 2:
            continue
        score = hits / len(tail)
        if score > best_score + 0.02:   # ties keep the earlier (preferred) one
            best, best_score = delimiter, score
    return best


def _data_like(cells: list[str], decimal_comma: bool) -> bool:
    """A row of numbers / times (a headerless file's first row)."""
    values = [cell.strip() for cell in cells if not is_missing(cell)]
    return bool(values) and all(
        _is_number(v, decimal_comma) or TOD_RE.match(v) or ISO_RE.match(v) for v in values)


def classify(name: str, values: list[str], decimal_comma: bool = False) -> str:
    """Kind of one column from sampled values: num, hex, tod, datetime or text."""
    if name in csv_columns.TEXT_COLUMNS:          # packet_type / packet_id ids
        return "text"
    values = [v.strip() for v in values if not is_missing(v)]
    if not values:
        return "num"
    hinted = "hex" in "".join(name.lower().split())
    if all(_is_number(v, decimal_comma) for v in values):
        if hinted and all(HEX_RE.match(v) for v in values):
            return "hex"
        return "num"

    def short_hex(v):
        if not HEX_RE.match(v):
            return False
        digits = v[2:] if v[:2].lower() == "0x" else v
        return len(digits) <= MAX_HEX_DIGITS

    if all(short_hex(v) for v in values):
        prefixed = all(v[:2].lower() == "0x" for v in values)
        with_digit = sum(any(c.isdigit() for c in v) for v in values) / len(values)
        if hinted or prefixed or with_digit >= 0.5:
            return "hex"
    if all(TOD_RE.match(v) for v in values):
        return "tod"
    if all(ISO_RE.match(v) for v in values):
        return "datetime"
    return "text"


def sniff(path) -> Sniff:
    """Work out a CSV's encoding, delimiter, header, units and column kinds."""
    path = str(path)
    f, size, encoding = _open_bytes(path)
    with f:
        head = f.read(HEAD_BYTES)
        tail = b""
        if size > HEAD_BYTES + TAIL_BYTES:
            f.seek(size - TAIL_BYTES)
            tail = f.read()
        elif size > HEAD_BYTES:
            tail = f.read()
    bom = 3 if head.startswith(b"\xef\xbb\xbf") else 0
    raw_lines = head[bom:].split(b"\n")
    if len(head) == HEAD_BYTES and len(raw_lines) > 1:
        raw_lines = raw_lines[:-1]                # may be cut mid-line
    lines = [_decode(line) for line in raw_lines[:MAX_SAMPLE_LINES]]
    if not any(line.strip() for line in lines):
        raise ValueError("the file is empty")

    candidates = [i for i, line in enumerate(lines) if not _is_comment(line)]
    if not candidates:
        raise ValueError("the file has only comment lines")
    sample = [lines[i] for i in candidates[:400]]
    delimiter = _pick_delimiter(sample)
    counts = {i: len(_fields(lines[i], delimiter)) for i in candidates}
    # The table's width: the commonest field count (ties: the wider), so
    # ragged rows do not hide the header.
    tally = Counter(counts.values())
    modal = max(tally, key=lambda width: (tally[width], width))
    header_line = next(i for i in candidates if counts[i] >= modal)
    skipped = sum(1 for i in range(header_line) if lines[i].strip())

    first = _fields(lines[header_line], delimiter)
    body = [i for i in candidates if i > header_line]
    # Decimal commas: only possible when the comma is not the delimiter.
    decimal_comma = False
    if delimiter != ",":
        probe = [cell.strip() for i in body[:200] for cell in _fields(lines[i], delimiter)]
        commas = sum(1 for v in probe if re.fullmatch(r"-?\d+,\d+", v))
        dots = sum(1 for v in probe if re.fullmatch(r"-?\d*\.\d+", v))
        decimal_comma = commas > 0 and commas >= dots

    # A header with fewer fields than the data (data rows ending in a
    # delimiter) looks like a metadata line; take it back as the header.
    earlier = [i for i in candidates if i < header_line]
    if (earlier and _data_like(first, decimal_comma) and counts[earlier[-1]] >= 2
            and counts[earlier[-1]] >= modal - 1
            and not _data_like(_fields(lines[earlier[-1]], delimiter), decimal_comma)):
        header_line = earlier[-1]
        first = _fields(lines[header_line], delimiter)
        skipped = sum(1 for i in range(header_line) if lines[i].strip())
        body = [i for i in candidates if i > header_line]
    has_header = not _data_like(first, decimal_comma)
    width = max(modal, len(first))
    units: list[str] = []
    data_line = header_line + 1 if has_header else header_line
    if has_header and body:
        second = _fields(lines[body[0]], delimiter)
        after = [_fields(lines[i], delimiter) for i in body[1:21]]
        texty = [j for j, cell in enumerate(second)
                 if not is_missing(cell) and not _is_number(cell.strip(), decimal_comma)
                 and len(cell.strip()) <= 16 and not TOD_RE.match(cell.strip())
                 and not ISO_RE.match(cell.strip())]
        numeric_below = [j for j in texty if after and sum(
            1 for row in after if j < len(row) and _is_number(row[j].strip(), decimal_comma))
            >= len(after) / 2]
        if texty and len(numeric_below) >= max(1, len(texty) / 2) and not any(
                _is_number(c.strip(), decimal_comma) for c in second if not is_missing(c)):
            units = second
            data_line = body[0] + 1

    raw_header = list(first) if has_header else []
    raw_header += [""] * (width - len(raw_header))
    names = raw_header if has_header else [f"col{i + 1}" for i in range(width)]
    if units:
        names = [f"{name.strip()} [{units[j].strip()}]" if j < len(units) and units[j].strip()
                 else name for j, name in enumerate(names)]
    names = unique_names(names)

    # Sample values per column: the head after the header, plus the tail so
    # values that only appear later (hex flags after launch) are classified.
    rows = [_fields(lines[i], delimiter) for i in candidates if i >= data_line]
    if tail:
        tail_lines = [_decode(line) for line in tail.split(b"\n")[1:]]
        rows += [_fields(line, delimiter) for line in tail_lines if not _is_comment(line)]
    columns = [[row[j] for row in rows if j < len(row)] for j in range(width)]
    kinds = [classify(name, values, decimal_comma) for name, values in zip(names, columns)]
    references = {}
    for name, kind, values in zip(names, kinds, columns):
        if kind == "datetime":
            stamp = next((d for d in map(_iso_datetime, values) if d is not None), None)
            references[name] = None if stamp is None else _midnight(stamp)

    data_offset = bom + sum(len(raw_lines[i]) + 1 for i in range(data_line))
    data_lines = raw_lines[data_line:]
    avg_line = (sum(len(line) + 1 for line in data_lines) / len(data_lines)
                if data_lines else 100.0)
    tokens = MISSING - {"", "nan", "-nan"}
    missing_tokens = any(v.strip().lower() in tokens for values in columns for v in values)
    return Sniff(avg_line=max(1.0, avg_line), missing_tokens=missing_tokens, encoding=encoding, delimiter=delimiter, data_offset=data_offset,
                 data_line=data_line, skipped=skipped, has_header=has_header,
                 units_row=bool(units), raw_header=raw_header, names=names, kinds=kinds,
                 decimal_comma=decimal_comma, tod_reference=references)


# ------------------------------------------------------------------ load

@dataclass
class GenericTable(Table):
    """A Table plus what the generic loader learned about the file."""
    kinds: dict = field(default_factory=dict)       # column -> hex / tod / datetime
    notes: list = field(default_factory=list)
    format: str = ""
    dropped: list = field(default_factory=list)     # text columns not kept


class _Columns:
    """Accumulates converted column pieces chunk by chunk."""

    def __init__(self, sn: Sniff, hex_as_numbers: bool):
        self.sn = sn
        self.hex_as_numbers = hex_as_numbers
        self.numeric: dict[int, list[np.ndarray]] = {}
        self.text_codes: dict[int, list[np.ndarray]] = {}
        self.text_lookup: dict[int, dict[str, int]] = {}
        self.dropped: set[int] = set()

    def kind(self, j: int) -> str:
        kind = self.sn.kinds[j]
        return "text" if kind == "hex" and not self.hex_as_numbers else kind

    def add_numeric(self, j: int, values: np.ndarray) -> None:
        values = np.asarray(values, dtype=np.float64)
        values[~np.isfinite(values)] = np.nan
        self.numeric.setdefault(j, []).append(values)

    def add_strings(self, j: int, values) -> None:
        """One chunk of a non-numeric column (numpy str array or list)."""
        kind = self.kind(j)
        values = np.asarray(values, dtype=str)
        if kind == "tod":
            self.numeric.setdefault(j, []).append(times_of_day(values))
        elif kind == "hex":
            uniques, inverse = np.unique(values, return_inverse=True)
            converted = np.array([math.nan if is_missing(u) else parse_hex(u)
                                  for u in uniques.tolist()], dtype=np.float64)
            self.numeric.setdefault(j, []).append(converted[inverse])
        elif kind == "datetime":
            reference = self.sn.tod_reference.get(self.sn.names[j])
            self.numeric.setdefault(j, []).append(iso_seconds(values.tolist(), reference))
        else:
            if j in self.dropped:
                return
            lookup = self.text_lookup.setdefault(j, {"": 0})
            uniques, inverse = np.unique(values, return_inverse=True)
            remap = np.array([lookup.setdefault("" if is_missing(u) else u.strip(), len(lookup))
                              for u in uniques.tolist()] or [0], dtype=np.int64)
            if len(lookup) > MAX_TEXT_CATEGORIES:
                self.dropped.add(j)
                self.text_codes.pop(j, None)
                return
            self.text_codes.setdefault(j, []).append(remap[inverse].astype(np.uint16))

    def finish(self, path, stride, total_rows, notes) -> GenericTable:
        sn = self.sn
        columns: dict[str, np.ndarray] = {}
        kinds: dict[str, str] = {}
        for j, name in enumerate(sn.names):
            parts = self.numeric.pop(j, None)
            if parts is None:
                continue
            values = np.concatenate(parts) if parts else np.empty(0)
            if values.size and not np.all(np.isnan(values)):
                columns[name] = values
                if self.kind(j) != "num":
                    kinds[name] = self.kind(j)
        text = {}
        for j, parts in self.text_codes.items():
            if j not in self.dropped:
                text[sn.names[j]] = TextColumn(list(self.text_lookup[j]),
                                               np.concatenate(parts))
        dropped = [sn.names[j] for j in sorted(self.dropped)]
        table = GenericTable(path=str(path), header=list(sn.names), columns=columns,
                             text=text, stride=stride, total_rows=total_rows,
                             kinds=kinds, notes=list(notes), format=sn.describe(),
                             dropped=dropped)
        add_combined_datetime(table)
        return table


def _fill_empty(chunk: bytes, delimiter: bytes) -> bytes:
    """Empty fields -> ``nan`` so numpy can parse them (byte level)."""
    double = delimiter + delimiter
    filled = delimiter + b"nan" + delimiter
    for _ in range(2):
        chunk = chunk.replace(double, filled)
    chunk = (chunk.replace(delimiter + b"\n", delimiter + b"nan\n")
             .replace(b"\n" + delimiter, b"\nnan" + delimiter))
    if chunk.startswith(delimiter):
        chunk = b"nan" + chunk
    return chunk


def load(path, progress=None, cancel=None, max_bytes: int = csv_columns.MAX_TABLE_BYTES,
         hex_as_numbers: bool = True, sniffed: Sniff | None = None,
         columns=None) -> GenericTable:
    """Read any CSV into numpy columns (safe to run off the UI thread).

    ``progress(fraction)`` reports progress; setting the ``cancel``
    threading.Event raises LoadCancelled. ``columns`` (a set of names)
    limits the numeric columns read; the others are skipped, never
    allocated. Raises ValueError for an empty file or one with nothing to
    plot, OSError for I/O errors.
    """
    path = str(path)
    sn = sniffed or sniff(path)
    if columns is not None:
        wanted = set(columns)
        sn = replace(sn, kinds=[kind if kind == "text" or name in wanted else "skip"
                                for name, kind in zip(sn.names, sn.kinds)])
    f, size, _encoding = _open_bytes(path)
    f.close()
    estimated_rows = max(1, int((size - sn.data_offset) / sn.avg_line))
    n_numeric = max(1, sum(kind not in ("text", "skip") for kind in sn.kinds))
    stride = max(1, math.ceil(estimated_rows * n_numeric * 8 / max_bytes))
    notes = []
    try:
        table = _load_fast(path, sn, stride, size, progress, cancel, hex_as_numbers)
    except ValueError:
        table, bad = _load_robust(path, sn, stride, size, progress, cancel, hex_as_numbers)
        notes.append("irregular rows: read with the csv module")
        if bad:
            notes.append("not numeric (junk values): " + ", ".join(bad))
        table.notes = notes + table.notes
    if stride > 1:
        table.notes.append(f"every {stride}th row kept (memory cap)")
    if table.dropped:
        table.notes.append("text columns with too many values left out: "
                           + ", ".join(table.dropped))
    if progress is not None:
        progress(1.0)
    return table


def _load_fast(path, sn, stride, size, progress, cancel, hex_as_numbers) -> GenericTable:
    acc = _Columns(sn, hex_as_numbers)
    num_index = [j for j, kind in enumerate(sn.kinds) if acc.kind(j) == "num"]
    str_index = [j for j, kind in enumerate(sn.kinds) if acc.kind(j) not in ("num", "skip")]
    delimiter = sn.delimiter.encode()
    total_rows = 0
    phase = 0
    f, _size, _encoding = _open_bytes(path)
    with f:
        f.seek(sn.data_offset)
        done = sn.data_offset
        rest = b""
        while True:
            if cancel is not None and cancel.is_set():
                raise LoadCancelled()
            block = f.read(csv_columns.CHUNK_BYTES)
            if not block and not rest:
                break
            data = rest + block
            if block:
                cut = data.rfind(b"\n") + 1
                if cut == 0:
                    rest = data
                    continue
                data, rest = data[:cut], data[cut:]
            else:
                rest = b""
                if not data.endswith(b"\n"):
                    data += b"\n"
            done += len(block)
            data = data.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
            # Blank lines are skipped by loadtxt; drop them first so the
            # numeric and text passes see the same rows.
            while b"\n\n" in data:
                data = data.replace(b"\n\n", b"\n")
            if data.startswith(b"\n"):
                data = data[1:]
            total_rows += data.count(b"\n")
            if stride > 1:
                data, phase = csv_columns._thin(data, stride, phase)
            if sn.decimal_comma:
                data = data.replace(b",", b".")
            data = _fill_empty(data, delimiter)
            if data.strip():
                common = dict(delimiter=sn.delimiter, quotechar='"', comments=None,
                              ndmin=2, encoding="utf-8")
                if num_index:
                    values = np.loadtxt(io.BytesIO(data), usecols=num_index,
                                        dtype=np.float64, **common)
                    for k, j in enumerate(num_index):
                        acc.add_numeric(j, values[:, k].copy())
                    del values
                if str_index:
                    values = np.loadtxt(io.BytesIO(data), usecols=str_index, dtype=str,
                                        **common)
                    for k, j in enumerate(str_index):
                        acc.add_strings(j, values[:, k])
                    del values
            if progress is not None and size:
                progress(min(0.99, done / size))
    return acc.finish(path, stride, total_rows, [])


def _load_robust(path, sn, stride, size, progress, cancel, hex_as_numbers):
    """csv.reader pass for ragged or odd files (same columns as _load_fast)."""
    acc = _Columns(sn, hex_as_numbers)
    width = len(sn.names)
    kinds = [acc.kind(j) for j in range(width)]
    numbers = {j: array("d") for j in range(width) if kinds[j] == "num"}
    strings = {j: [] for j in range(width) if kinds[j] not in ("num", "skip")}
    bad = Counter()
    present = Counter()
    nan = math.nan
    total_rows = 0
    f, _size, _encoding = _open_bytes(path)
    with f:
        f.seek(sn.data_offset)
        text = io.TextIOWrapper(f, encoding="utf-8", errors="replace", newline="")
        reader = csv.reader(text, delimiter=sn.delimiter)
        kept = 0
        for row in reader:
            if not any(cell.strip() for cell in row) or row[0].lstrip().startswith(
                    COMMENT_PREFIXES):
                continue
            total_rows += 1
            if (total_rows - 1) % stride:
                continue
            kept += 1
            if kept % 20000 == 0:
                if cancel is not None and cancel.is_set():
                    raise LoadCancelled()
                if progress is not None and size:
                    progress(min(0.99, f.tell() / size))
            count = len(row)
            for j, values in numbers.items():
                cell = row[j] if j < count else ""
                if is_missing(cell):
                    values.append(nan)
                    continue
                present[j] += 1
                value = parse_number(cell, sn.decimal_comma)
                if math.isnan(value) and cell.strip().lower() not in ("nan", "inf", "-inf"):
                    bad[j] += 1
                values.append(value)
            for j, values in strings.items():
                values.append(row[j].strip() if j < count else "")
    demoted = []
    for j, values in numbers.items():
        if bad[j] and bad[j] > BAD_NUMERIC_FRACTION * present[j]:
            demoted.append(sn.names[j])
            continue
        acc.add_numeric(j, np.frombuffer(values, dtype=np.float64).copy())
    for j, values in strings.items():
        acc.add_strings(j, values if values else np.empty(0, dtype=str))
    return acc.finish(path, stride, total_rows, []), demoted


YEAR_KEYS = ("year", "yr", "yyyy")
MONTH_KEYS = ("month", "mon", "mm")
DAY_KEYS = ("day", "dd")


def add_combined_datetime(table: GenericTable) -> str | None:
    """Combine Year/Month/Day (or an ISO date column) with a time of day.

    Adds a ``datetime`` column, in seconds since midnight of the first date,
    placed after the time-of-day column, so files that cross midnight stay
    continuous. Returns its name, or None.
    """
    tod = next((name for name, kind in table.kinds.items() if kind == "tod"), None)
    if tod is None:
        return None
    keys = {name_key(name): name for name in table.columns}
    year = next((keys[k] for k in YEAR_KEYS if k in keys), None)
    month = next((keys[k] for k in MONTH_KEYS if k in keys), None)
    day = next((keys[k] for k in DAY_KEYS if k in keys), None)
    seconds = table.columns[tod]
    if year and month and day:
        y, m, d = (table.columns[name] for name in (year, month, day))
        ok = ~(np.isnan(y) | np.isnan(m) | np.isnan(d))
        if not ok.any():
            return None
        first = int(np.flatnonzero(ok)[0])
        try:
            ref = date(int(y[first]), int(m[first]), int(d[first]))
        except ValueError:
            return None
        days = np.full(seconds.size, np.nan)
        stamp = ((y[ok].astype(np.int64) - 1970).astype("datetime64[Y]")
                 + (m[ok].astype(np.int64) - 1).astype("timedelta64[M]"))
        stamp = stamp.astype("datetime64[D]") + (d[ok].astype(np.int64) - 1).astype(
            "timedelta64[D]")
        days[ok] = (stamp - np.datetime64(ref, "D")).astype(np.int64)
    else:
        dated = next((name for name, kind in table.kinds.items() if kind == "datetime"),
                     None)
        if dated is None:
            return None
        values = table.columns[dated]
        # A date-only column is whole days since its first midnight.
        finite = values[~np.isnan(values)]
        if finite.size == 0 or np.any(np.mod(finite, 86400.0) != 0):
            return None
        days = values / 86400.0
    combined = days * 86400.0 + seconds
    if np.all(np.isnan(combined)):
        return None
    name = unique_names(list(table.header) + ["datetime"])[-1]
    columns = {}
    for key, values in table.columns.items():
        columns[key] = values
        if key == tod:
            columns[name] = combined
    table.columns = columns
    header = list(table.header)
    header.insert(header.index(tod) + 1, name)
    table.header = header
    table.kinds[name] = "datetime"
    return name
