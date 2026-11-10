"""Fast, column-wise loading of converted CSVs (numpy), for the viewer.

A converted flash log can have millions of rows (I*.csv IMU exports are
960 Hz). Reading it as one Python dict of strings per row costs about ten
times the file size in memory and seconds of CPU, so this module keeps only:

* every numeric column as one float64 numpy array (NaN = empty), and
* the few text columns the viewer needs (``packet_type``, ``packet_id``).

The fast path fills empty fields with ``nan`` at the byte level and hands
each ~4 MB chunk to numpy's C ``loadtxt``, so the parse never holds the GIL
for long (a UI thread stays responsive) and progress can be reported. Odd
files (ragged rows, text in a column that looked numeric) fall back to a
``csv.reader`` pass with the same result. Very large files are thinned to
every Nth row so the arrays stay under ``max_bytes``.
"""

from __future__ import annotations

import csv
import io
import math
import os
from array import array
from dataclasses import dataclass, field

import numpy as np

TEXT_COLUMNS = ("packet_type", "packet_id")
CHUNK_BYTES = 4 * 1024 * 1024
SAMPLE_BYTES = 512 * 1024
MAX_TABLE_BYTES = 1024 ** 3          # numeric arrays; above this, thin rows


class TextColumn:
    """A text column stored as categories + small integer codes.

    Columns such as ``packet_type`` repeat a handful of values over millions
    of rows; codes take 2 bytes per row instead of a Python or unicode
    string. ``column == "camera"`` gives a boolean array.
    """
    __slots__ = ("categories", "codes")

    def __init__(self, categories, codes):
        self.categories = list(categories)
        self.codes = np.asarray(codes, dtype=np.uint16)

    @classmethod
    def from_values(cls, values) -> "TextColumn":
        lookup: dict[str, int] = {}
        codes = np.fromiter((lookup.setdefault(str(v).strip(), len(lookup)) for v in values),
                            dtype=np.uint16)
        return cls(list(lookup), codes)

    def __len__(self) -> int:
        return int(self.codes.size)

    def __eq__(self, value):  # elementwise, like a numpy array
        if value not in self.categories:
            return np.zeros(self.codes.size, bool)
        return self.codes == self.categories.index(value)

    __hash__ = None

    def isin(self, values) -> np.ndarray:
        wanted = [i for i, c in enumerate(self.categories) if c in set(values)]
        return np.isin(self.codes, wanted)

    def map(self, fn) -> "TextColumn":
        """Apply ``fn`` to every value (via the categories), merging duplicates."""
        mapped = [fn(c) for c in self.categories]
        merged: dict[str, int] = {}
        remap = np.array([merged.setdefault(m, len(merged)) for m in mapped] or [0],
                         dtype=np.uint16)
        return TextColumn(list(merged), remap[self.codes] if self.categories else self.codes)

    def unique(self) -> list[str]:
        used = np.unique(self.codes)
        return [self.categories[i] for i in used.tolist()]

    def append(self, value: str, count: int) -> "TextColumn":
        categories = list(self.categories)
        if value not in categories:
            categories.append(value)
        extra = np.full(count, categories.index(value), dtype=np.uint16)
        return TextColumn(categories, np.concatenate([self.codes, extra]))

    def __getitem__(self, index):
        if isinstance(index, (int, np.integer)):
            return self.categories[int(self.codes[index])]
        return TextColumn(self.categories, self.codes[index])


class LoadCancelled(Exception):
    """Raised when ``cancel`` is set during a load."""


@dataclass
class Table:
    """A CSV as columns. ``columns`` are float64 arrays of equal length."""
    path: str
    header: list[str]
    columns: dict[str, np.ndarray]
    text: dict[str, TextColumn] = field(default_factory=dict)
    stride: int = 1                  # 1 = every row; N = every Nth row kept
    total_rows: int = 0              # data rows in the file

    @property
    def row_count(self) -> int:
        for values in self.columns.values():
            return len(values)
        for values in self.text.values():
            return len(values)
        return 0

    @property
    def nbytes(self) -> int:
        return sum(values.nbytes for values in self.columns.values())


def _classify(header: list[str], sample_rows: list[list[str]]) -> list[bool]:
    """True for columns whose sampled values are all numbers or empty."""
    numeric = []
    for index in range(len(header)):
        ok = True
        for row in sample_rows:
            value = row[index] if index < len(row) else ""
            if value:
                try:
                    float(value)
                except ValueError:
                    ok = False
                    break
        numeric.append(ok)
    return numeric


def _fill_empty(chunk: bytes) -> bytes:
    """Empty CSV fields -> ``nan`` so numpy can parse them (byte level)."""
    for _ in range(2):  # ",,," needs two passes
        chunk = chunk.replace(b",,", b",nan,")
    chunk = chunk.replace(b",\n", b",nan\n").replace(b"\n,", b"\nnan,")
    if chunk.startswith(b","):
        chunk = b"nan" + chunk
    if chunk.endswith(b","):
        chunk += b"nan"
    return chunk


def _thin(chunk: bytes, stride: int, phase: int) -> tuple[bytes, int]:
    """Keep every ``stride``-th line; returns the kept lines and next phase."""
    lines = chunk.split(b"\n")
    if lines and lines[-1] == b"":
        lines.pop()
    kept = lines[(-phase) % stride::stride]
    return b"\n".join(kept) + (b"\n" if kept else b""), (phase + len(lines)) % stride


def _read_head(path: str) -> tuple[list[str], list[list[str]], int, float]:
    with open(path, "rb") as f:
        head = f.read(SAMPLE_BYTES)
    text = head.decode("utf-8-sig", "replace").replace("\r\n", "\n")
    lines = text.split("\n")
    if len(head) == SAMPLE_BYTES and len(lines) > 1:
        lines = lines[:-1]  # last line may be cut
    rows = list(csv.reader(lines))
    if not rows or not rows[0]:
        raise ValueError("the CSV has no header row")
    header = [name.strip() for name in rows[0]]
    sample = [row for row in rows[1:] if row]
    header_bytes = len(lines[0].encode("utf-8")) + 1
    avg_line = (max(1, len(head) - header_bytes) / len(sample)) if sample else 100.0
    return header, sample, header_bytes, avg_line


def load_table(path, progress=None, cancel=None,
               max_bytes: int = MAX_TABLE_BYTES) -> Table:
    """Load a converted CSV as numpy columns.

    ``progress(fraction)`` is called as the file is read; setting the
    ``cancel`` threading.Event raises LoadCancelled. Raises ValueError for a
    file without a header or numeric columns, OSError for I/O errors.
    """
    path = str(path)
    size = os.path.getsize(path)
    header, sample, _header_bytes, avg_line = _read_head(path)
    is_numeric = _classify(header, sample)
    estimated_rows = max(1, int(size / max(avg_line, 1.0)))
    n_numeric = sum(is_numeric)
    stride = max(1, math.ceil(estimated_rows * max(n_numeric, 1) * 8 / max_bytes))
    try:
        table = _load_fast(path, header, is_numeric, stride, size, progress, cancel)
    except ValueError:
        table = _load_robust(path, header, stride, size, progress, cancel)
    # Columns that never held a number are not numeric columns.
    table.columns = {name: values for name, values in table.columns.items()
                     if values.size and not np.all(np.isnan(values))}
    if progress is not None:
        progress(1.0)
    return table


def _load_fast(path, header, is_numeric, stride, size, progress, cancel) -> Table:
    numeric_index = [i for i, ok in enumerate(is_numeric) if ok]
    text_index = [header.index(name) for name in TEXT_COLUMNS
                  if name in header and not is_numeric[header.index(name)]]
    # Each chunk is split into per-column pieces straight away, so the peak
    # is about one copy of the data (not chunk blocks + a 2-D block + columns).
    column_parts: list[list[np.ndarray]] = [[] for _ in numeric_index]
    text_parts: dict[int, list[np.ndarray]] = {i: [] for i in text_index}
    text_lookup: dict[int, dict[str, int]] = {i: {} for i in text_index}
    total_rows = 0
    phase = 0
    with open(path, "rb") as f:
        first = f.readline()            # header
        done = len(first)
        rest = b""
        while True:
            if cancel is not None and cancel.is_set():
                raise LoadCancelled()
            block = f.read(CHUNK_BYTES)
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
            data = data.replace(b"\r\n", b"\n")
            total_rows += data.count(b"\n") - data.count(b"\n\n")
            if stride > 1:
                data, phase = _thin(data, stride, phase)
            data = _fill_empty(data)
            if data.strip():
                if numeric_index:
                    block = np.loadtxt(
                        io.BytesIO(data), delimiter=",", usecols=numeric_index,
                        dtype=np.float64, quotechar='"', ndmin=2, encoding="utf-8")
                    for k, parts in enumerate(column_parts):
                        parts.append(block[:, k].copy())
                    del block
                for i in text_index:
                    values = np.loadtxt(io.BytesIO(data), delimiter=",", usecols=[i],
                                        dtype=str, quotechar='"', ndmin=1,
                                        encoding="utf-8")
                    uniques, inverse = np.unique(values, return_inverse=True)
                    lookup = text_lookup[i]
                    remap = np.array([lookup.setdefault(
                        "" if u.strip() == "nan" else u.strip(), len(lookup))
                        for u in uniques.tolist()], dtype=np.uint16)
                    text_parts[i].append(remap[inverse])
            if progress is not None and size:
                progress(min(0.99, done / size))
    columns = {}
    for k, i in enumerate(numeric_index):
        parts = column_parts[k]
        columns[header[i]] = np.concatenate(parts) if parts else np.empty(0)
        column_parts[k] = []          # free the pieces as we go
    text = {header[i]: TextColumn(list(text_lookup[i]),
                                  np.concatenate(parts) if parts else np.empty(0))
            for i, parts in text_parts.items()}
    return Table(path=path, header=header, columns=columns, text=text,
                 stride=stride, total_rows=total_rows)


def _load_robust(path, header, stride, size, progress, cancel) -> Table:
    """csv.reader fallback: same output as _load_fast for any CSV."""
    n = len(header)
    values = [array("d") for _ in range(n)]
    has_text = [False] * n
    keep_text = {header.index(name) for name in TEXT_COLUMNS if name in header}
    texts: dict[int, list[str]] = {i: [] for i in keep_text}
    nan = math.nan
    total_rows = 0
    with open(path, newline="", encoding="utf-8-sig", errors="replace") as f:
        reader = csv.reader(f)
        next(reader, None)
        appends = [column.append for column in values]
        for row_index, row in enumerate(reader):
            if not row:
                continue
            total_rows += 1
            if row_index % stride:
                continue
            if row_index % 20000 == 0:
                if cancel is not None and cancel.is_set():
                    raise LoadCancelled()
                if progress is not None and size:
                    progress(min(0.99, f.tell() / size))
            width = len(row)
            for i in range(n):
                value = row[i].strip() if i < width else ""
                if i in keep_text:
                    texts[i].append(value)
                if value:
                    try:
                        appends[i](float(value))
                        continue
                    except ValueError:
                        has_text[i] = True
                appends[i](nan)
    columns = {header[i]: np.frombuffer(values[i], dtype=np.float64).copy()
               for i in range(n) if not has_text[i]}
    text = {header[i]: TextColumn.from_values(texts[i]) for i in keep_text
            if has_text[i] or header[i] not in columns}
    for i in keep_text:
        if header[i] in columns and has_text[i]:
            columns.pop(header[i], None)
    return Table(path=path, header=header, columns=columns, text=text,
                 stride=stride, total_rows=total_rows)


def table_from_rows(path, rows, base_columns=(), progress=None, cancel=None,
                    size: int = 0) -> Table:
    """Build a Table from an iterable of row dicts (converter.iter_* output).

    Rows are consumed one at a time and never kept, so a decoded log of
    millions of records costs only its numeric arrays. Columns are ordered
    like converter.write_csv's CSV header (``base_columns`` first, then in
    first-seen order, ``raw_packet_hex`` last); a column is numeric when all
    its non-empty values are numbers, as when the CSV is loaded.
    """
    nan = math.nan
    numeric: dict[str, array] = {}
    is_text: set[str] = set()
    text_codes: dict[str, array] = {}
    text_lookup: dict[str, dict[str, int]] = {}
    order: list[str] = []
    seen: set[str] = set()
    count = 0
    for row in rows:
        for name, value in row.items():
            if name not in seen:
                seen.add(name)
                order.append(name)
            if name in TEXT_COLUMNS:
                lookup = text_lookup.setdefault(name, {"": 0})
                codes = text_codes.setdefault(name, array("H"))
                if len(codes) < count:
                    codes.extend([0] * (count - len(codes)))
                codes.append(lookup.setdefault(str(value).strip(), len(lookup)))
                continue
            if name in is_text:
                continue
            if isinstance(value, (int, float)):
                number = float(value)
            elif value is None or value == "":
                continue
            else:
                try:
                    number = float(value)
                except (TypeError, ValueError):
                    is_text.add(name)
                    numeric.pop(name, None)
                    continue
            values = numeric.get(name)
            if values is None:
                values = numeric[name] = array("d")
            if len(values) < count:
                values.extend([nan] * (count - len(values)))
            values.append(number)
        count += 1
        if count % 20000 == 0:
            if cancel is not None and cancel.is_set():
                raise LoadCancelled()
            if progress is not None and size:
                progress(min(0.99, float(row.get("byte_offset", 0)) / size))
    header = list(base_columns) + [name for name in order
                                   if name not in base_columns and name != "raw_packet_hex"]
    if "raw_packet_hex" in seen:
        header.append("raw_packet_hex")
    columns = {}
    for name in header:
        values = numeric.get(name)
        if values is None:
            continue
        if len(values) < count:
            values.extend([nan] * (count - len(values)))
        columns[name] = np.frombuffer(values, dtype=np.float64).copy()
    text = {}
    for name, codes in text_codes.items():
        if len(codes) < count:
            codes.extend([0] * (count - len(codes)))
        text[name] = TextColumn(list(text_lookup[name]),
                                np.frombuffer(codes, dtype=np.uint16).copy())
    if progress is not None:
        progress(1.0)
    return Table(path=str(path), header=header, columns=columns, text=text,
                 stride=1, total_rows=count)
