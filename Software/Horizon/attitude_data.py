"""Attitude samples for the Horizon Attitude tab: live feed selection and CSV
replay tracks. Pure Python (no Tk) so it can be unit tested directly.

Live: Witness decoded-feed records (the same dicts Flight View receives) with
``packet["packet"]`` of ``"witness_debug"`` (0xE0, 10 Hz when enabled) or
``"state"`` (0x02, 1 Hz) and ``packet["orientation"] = [x, y, z, w]``. The 0xE0
quaternion is preferred while it is fresh (``E0_FRESH_S``); otherwise 0x02.

File: see :func:`load_attitude_csv` for the accepted column names.
"""

from bisect import bisect_right
from collections import deque
import csv
from dataclasses import dataclass, replace
import math
from pathlib import Path
import threading
import time

import attitude_math as am

E0_FRESH_S = 2.5          # 0xE0 arrives at 10 Hz over USB, 1 Hz over LoRa.
LIVE_HISTORY = 6000       # samples kept for the strip chart (10 min at 10 Hz)
WITNESS_STATUS_PACKETS = frozenset(("sensors", "state", "command", "heartbeat", "witness_debug"))
FLIGHT_STATES = ("PAD IDLE", "BOOST", "COAST", "DESCENT", "LANDED")

# Column names, matched case-insensitively after stripping whitespace.
TIME_COLUMNS_S = ("time_s", "time", "t", "t_s", "time_seconds", "sim_time_s", "host_time_s", "timestamp_s")
TIME_COLUMNS_MS = ("time_ms", "t_ms", "uptime_ms", "uptime", "sim_time_ms")
_COMPONENTS = ("x", "y", "z", "w")


def _quat_sets(prefixes, extra=()):
    sets = []
    for prefix in prefixes:
        sets.append(tuple(f"{prefix}q{c}" for c in _COMPONENTS))
        sets.append(tuple(f"{prefix}q_{c}" for c in _COMPONENTS))
        sets.append(tuple(f"{prefix}quat_{c}" for c in _COMPONENTS))
        sets.append(tuple(f"{prefix}orientation_{c}" for c in _COMPONENTS))
    sets.extend(extra)
    return tuple(sets)


ESTIMATE_COLUMN_SETS = _quat_sets(("witness_", "est_", "estimate_", "wit_", ""))
TRUTH_COLUMN_SETS = _quat_sets(("truth_", "true_", "sim_", "ref_"))


def status_fields(status):
    """(flight_state, gps_locked, hitl_active) from Witness status uint16."""
    if type(status) is not int:
        return None, None, None
    return status & 7, (status >> 3) & 1, (status >> 4) & 1


def state_name(state):
    if state is None:
        return "--"
    return FLIGHT_STATES[state] if 0 <= state < len(FLIGHT_STATES) else f"RESERVED ({state})"


@dataclass(frozen=True)
class AttitudeSample:
    time: float                     # seconds: host monotonic (live) or file time
    q: tuple | None                 # normalised estimate, None if unusable
    raw: tuple | None = None        # components as received
    raw_norm: float | None = None
    reason: str = ""                # why q is None ("not available", ...)
    source: str = ""                # "0xE0", "0x02", "file"
    device: str = ""                # feed source name (COM7, TCP...) or file name
    truth: tuple | None = None
    truth_reason: str = "missing"
    flight_state: int | None = None
    gps_locked: int | None = None
    hitl: int | None = None
    snapshot: bool = False          # relay replay of an old record on reconnect
    interpolated: bool = False


def make_sample(t, raw, source, *, truth_raw=None, **fields):
    q, raw_norm, reason = am.checked(raw)
    truth, _, truth_reason = am.checked(truth_raw) if truth_raw is not None else (None, None, "missing")
    raw_tuple = None
    if raw is not None:
        try:
            raw_tuple = tuple(float(c) if c is not None else math.nan for c in raw)
        except (TypeError, ValueError):
            raw_tuple = None
    return AttitudeSample(t, q, raw_tuple, raw_norm, reason, source, truth=truth,
                          truth_reason=truth_reason, **fields)


# ---------------------------------------------------------------- live feed

class LiveAttitude:
    """Tracks the latest 0x02/0xE0 attitude and Witness status from the feed.

    accept_record() runs on the receiver worker thread (it must not touch Tk);
    the GUI reads through current()/history_list()/status under the lock.
    """

    def __init__(self, history=LIVE_HISTORY):
        self._lock = threading.Lock()
        self.latest = {}
        self.history = deque(maxlen=history)
        self.status = (None, None, None)
        self.status_time = None
        self.records = 0

    def clear(self):
        with self._lock:
            self.latest.clear()
            self.history.clear()
            self.status = (None, None, None)
            self.status_time = None

    def history_list(self):
        with self._lock:
            return list(self.history)

    @staticmethod
    def received_time(record):
        received = record.get("received_mono")
        if isinstance(received, (int, float)) and math.isfinite(received):
            return float(received)
        at = record.get("received_at")
        if isinstance(at, (int, float)) and math.isfinite(at):
            return time.monotonic() - max(0.0, time.time() - at)
        return time.monotonic()

    def accept_record(self, record):
        """Ingest one decoded-feed record. Returns True if it carried attitude."""
        with self._lock:
            return self._accept(record)

    def _accept(self, record):
        packet = record.get("packet") if isinstance(record, dict) else None
        if not isinstance(packet, dict):
            return False
        kind = packet.get("packet")
        if kind not in WITNESS_STATUS_PACKETS:
            return False
        received = self.received_time(record)
        snapshot = bool(record.get("replay"))
        state, gps, hitl = status_fields(packet.get("status"))
        if state is not None and (self.status_time is None or received >= self.status_time):
            self.status, self.status_time = (state, gps, hitl), received
        if kind not in ("state", "witness_debug") or "orientation" not in packet:
            return False
        source = "0xE0" if kind == "witness_debug" else "0x02"
        sample = make_sample(received, packet.get("orientation"), source,
                             device=str(record.get("source", "")), flight_state=state,
                             gps_locked=gps, hitl=hitl, snapshot=snapshot)
        previous = self.latest.get(source)
        if previous is not None and previous.time > received:
            return True   # out of order (e.g. snapshot after live): keep newer
        self.latest[source] = sample
        self.records += 1
        if not snapshot:
            e0 = self.latest.get("0xE0")
            e0_fresh = e0 is not None and not e0.snapshot and received - e0.time <= E0_FRESH_S
            if source == "0xE0" or not e0_fresh:
                self.history.append(sample)
        return True

    def current(self, now=None):
        """Preferred sample: fresh 0xE0, else whichever of 0x02/0xE0 is newest."""
        now = time.monotonic() if now is None else now
        with self._lock:
            e0, s2 = self.latest.get("0xE0"), self.latest.get("0x02")
        if e0 is not None and not e0.snapshot and now - e0.time <= E0_FRESH_S:
            return e0
        candidates = [s for s in (s2, e0) if s is not None]
        return max(candidates, key=lambda s: s.time) if candidates else None


# --------------------------------------------------------------- file track

def _find(columns, names):
    lookup = {c.strip().lower(): c for c in columns if c is not None}
    for name in names:
        if name in lookup:
            return lookup[name]
    return None


def _find_set(columns, sets):
    lookup = {c.strip().lower(): c for c in columns if c is not None}
    for group in sets:
        if all(name in lookup for name in group):
            return tuple(lookup[name] for name in group)
    return None


def _float(text):
    if text is None:
        return None
    text = str(text).strip()
    if text == "":
        return None
    try:
        return float(text)
    except ValueError:
        return None


def _int(text):
    value = _float(text)
    if value is None or not math.isfinite(value):
        return None
    return int(value)


class AttitudeTrack:
    """Time-ordered samples loaded from a CSV, with interpolated lookup."""

    def __init__(self, samples, *, name="", columns=None, warnings=()):
        self.samples = list(samples)
        self.times = [s.time for s in self.samples]
        self.name = name
        self.columns = columns or {}
        self.warnings = list(warnings)
        self.has_truth = any(s.truth is not None for s in self.samples)
        gaps = sorted(b - a for a, b in zip(self.times, self.times[1:]) if b > a)
        self.median_dt = gaps[len(gaps) // 2] if gaps else 0.1
        # Interpolate across normal sample spacing, not across dropouts.
        self.max_gap = max(0.25, 3.0 * self.median_dt)

    @property
    def start(self):
        return self.times[0] if self.times else 0.0

    @property
    def end(self):
        return self.times[-1] if self.times else 0.0

    def __len__(self):
        return len(self.samples)

    def sample_at(self, t):
        """Sample at time t: slerp between neighbours, else hold the previous."""
        if not self.samples:
            return None
        i = bisect_right(self.times, t) - 1
        if i < 0:
            return self.samples[0]
        a = self.samples[i]
        if i + 1 >= len(self.samples) or t <= a.time:
            return a
        b = self.samples[i + 1]
        gap = b.time - a.time
        if gap <= 0 or gap > self.max_gap:
            return a
        f = (t - a.time) / gap
        changes = {"interpolated": True}
        if a.q is not None and b.q is not None:
            changes["q"] = am.slerp(a.q, b.q, f)
        if a.truth is not None and b.truth is not None:
            changes["truth"] = am.slerp(a.truth, b.truth, f)
        return replace(a, **changes)

    def first_heading_offset(self):
        """Heading offset at the first sample where estimate and truth are valid."""
        for s in self.samples:
            if s.q is not None and s.truth is not None:
                return am.heading_offset_deg(s.q, s.truth), s.time
        return None, None


def load_attitude_csv(path):
    """Load a converter.py CSV or a generic/HITL attitude CSV.

    Accepted columns (case-insensitive):

    * time, seconds: ``time_s``, ``time``, ``t``, ``t_s``, ``time_seconds``
      (converter), ``sim_time_s``, ``host_time_s``, ``timestamp_s``; or
      milliseconds: ``time_ms``, ``t_ms``, ``uptime_ms``, ``uptime``,
      ``sim_time_ms``. Without one, the row number x 0.1 s is used.
    * estimate (Witness) quaternion, first complete set wins, ``P`` one of
      ``witness_``, ``est_``, ``estimate_``, ``wit_`` or no prefix:
      ``Pqx,Pqy,Pqz,Pqw`` / ``Pq_x..`` / ``Pquat_x..`` / ``Porientation_x..``
      (the converter writes ``orientation_x,_y,_z,_w``).
    * optional truth quaternion, same patterns with ``truth_``, ``true_``,
      ``sim_`` or ``ref_`` prefix, e.g. ``truth_qx,truth_qy,truth_qz,truth_qw``.
    * optional status: ``flight_state``, ``hitl_active``, ``gps_locked`` or a
      raw ``status`` uint16; optional ``packet_id`` / ``source`` label.

    Quaternions are x,y,z,w Hamilton body->ENU (comms.h); 0,0,0,0 means not
    available. Converter CSVs: rows without orientation are skipped and, if
    any 0xE0 rows carry orientation, only those are used (else 0x02 rows).
    """
    path = Path(path)
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        columns = reader.fieldnames or []
        rows = list(reader)
    est_cols = _find_set(columns, ESTIMATE_COLUMN_SETS)
    truth_cols = _find_set(columns, TRUTH_COLUMN_SETS)
    if est_cols is None and truth_cols is None:
        raise ValueError(f"{path.name}: no quaternion columns found (expected e.g. "
                         "time_s,witness_qx,witness_qy,witness_qz,witness_qw or the "
                         "converter's orientation_x..orientation_w)")
    time_col = _find(columns, TIME_COLUMNS_S)
    time_scale = 1.0
    if time_col is None:
        time_col = _find(columns, TIME_COLUMNS_MS)
        time_scale = 0.001
    packet_col = _find(columns, ("packet_id",))
    source_col = _find(columns, ("source", "packet_source"))
    state_col = _find(columns, ("flight_state", "witness_flight_state"))
    hitl_col = _find(columns, ("hitl_active", "hitl"))
    gps_col = _find(columns, ("gps_locked",))
    status_col = _find(columns, ("status",))
    warnings = []

    def cells(row, cols):
        if cols is None:
            return None
        values = [_float(row.get(c)) for c in cols]
        return None if all(v is None for v in values) else values

    usable = [(row, cells(row, est_cols), cells(row, truth_cols)) for row in rows]
    usable = [item for item in usable if item[1] is not None or item[2] is not None]
    if packet_col is not None:
        e0 = [item for item in usable if str(item[0].get(packet_col, "")).strip().upper() == "0XE0"]
        if e0 and len(e0) < len(usable):
            warnings.append(f"using {len(e0)} 0xE0 rows; {len(usable) - len(e0)} 0x02 rows ignored")
            usable = e0

    samples = []
    offset = 0.0
    previous = None
    for index, (row, est, truth) in enumerate(usable):
        t = _float(row.get(time_col)) if time_col else None
        if t is None or not math.isfinite(t):
            t = (previous - offset + 0.1) if previous is not None else index * 0.1
        else:
            t *= time_scale
        t += offset
        if previous is not None and t < previous:
            # Time went backwards (device reboot, looped replay): continue on.
            jump = previous - t
            offset += jump + 0.1
            t += jump + 0.1
            if not any(w.startswith("time went backwards") for w in warnings):
                warnings.append("time went backwards; later samples shifted to keep order")
        previous = t
        state, gps, hitl = (None, None, None)
        if status_col is not None:
            state, gps, hitl = status_fields(_int(row.get(status_col)))
        if state_col is not None and _int(row.get(state_col)) is not None:
            state = _int(row.get(state_col))
        if hitl_col is not None and _int(row.get(hitl_col)) is not None:
            hitl = _int(row.get(hitl_col))
        if gps_col is not None and _int(row.get(gps_col)) is not None:
            gps = _int(row.get(gps_col))
        if packet_col is not None and str(row.get(packet_col, "")).strip():
            source = str(row[packet_col]).strip()
            source = "0x" + source[2:].upper() if source.lower().startswith("0x") else source
        elif source_col is not None and str(row.get(source_col, "")).strip():
            source = str(row[source_col]).strip()
        else:
            source = "file"
        samples.append(make_sample(t, est, source, truth_raw=truth, device=path.name,
                                   flight_state=state, gps_locked=gps, hitl=hitl))
    if not samples:
        raise ValueError(f"{path.name}: no rows with quaternion values")
    names = {"time": time_col or "(row number)", "estimate": est_cols, "truth": truth_cols}
    return AttitudeTrack(samples, name=path.name, columns=names, warnings=warnings)
