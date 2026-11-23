r"""Convert flight logs to Rerun recordings (.rrd) for analysis in the Rerun viewer.

    ..\Software\Horizon\.venv\Scripts\python.exe to_rerun.py flightreal "flight logs\aimflight.csv" ^
        "flight logs\BlRv_*.csv" "flight logs\horizon_*.jsonl" "flight logs\aim_mqtt_*.jsonl" ^
        --liftoff "2026-11-21 13:47:32.6"
    rerun flightreal.rrd aimflight.rrd ...          # or open the .rrd files in the viewer
    ... -o flight.rrd                                # everything in one file instead

Inputs (same readers as witness_to_mcap.py and logs_to_mcap.py):
  * Witness: a folder (or I/S/P *.BIN files) of one session; the three files are synchronised on
    liftoff as in witness_to_mcap.py.
  * AIM onboard CSV export, Blue Raven HR/LR CSV, Horizon JSONL (Witness, Featherweight), AIM MQTT JSONL.

Each input becomes one .rrd next to it (all with the same recording id, so opening them together
in Rerun shows one recording), or with -o one combined .rrd. Inside:
  * Timelines: flight_time (seconds from liftoff, when the logger's liftoff is known),
    wall_clock (for loggers on real time) and logger_clock (seconds on the logger's own clock).
  * Every numeric field is a scalar series at <logger topic>/<field>, e.g. witness/imu/gyro_x_dps,
    aim_onboard/pressure/pressure, raven/SN2878/lr/baro_altitude_agl_feet.
  * GPS positions are GeoPoints at <topic>/position (Rerun's map view).
  * Witness and AIM orientation quaternions drive a box at <topic>/attitude (3D view).
  * Flight-state changes and BIT verdicts are text log entries at <topic>/events.
Liftoff per logger: --liftoff puts Witness and the AIM onboard log there (as in the MCAP scripts)
and is also the flight_time zero for the wall-clock logs (Horizon, AIM MQTT, Raven). Without it,
Witness, AIM onboard and Raven use their own detected liftoff; Horizon/AIM MQTT get no flight_time.
Needs the packages in requirements.txt (Logs/.venv).
"""

from __future__ import annotations

import argparse
import collections
import glob
import math
import re
import sys
from pathlib import Path

import numpy as np
import rerun as rr

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import logs_to_mcap as logs          # noqa: E402
import witness_to_mcap as witness    # noqa: E402

RERUN_VERSION = "0.43.2"             # API used here; other versions may differ
APP_ID = "witness_flight"
WALL_NS = int(1e17)                  # times after ~1973 are taken as wall clock

# Bookkeeping fields that are not worth plotting.
SKIP_FIELDS = {
    "packet_index", "byte_offset", "frame_length", "timestamp_us", "time_seconds", "sequence",
    "gap_before_bytes", "device_timestamp_us", "segment", "record_version", "packet_length",
    "t_s", "_sequence", "_sent_at", "_replay", "text_crc_expected", "text_crc_computed", "id_a",
}
EVENT_FIELDS = ("flight_state_name", "state", "verdict_name", "resume_flight_state_name")
QUATERNIONS = (("orientation_x", "orientation_y", "orientation_z", "orientation_w"),
               ("qx", "qy", "qz", "qw"), ("x", "y", "z", "w"))


def entity(*parts) -> str:
    out = []
    for p in parts:
        for seg in str(p).strip("/").split("/"):
            seg = re.sub(r"[^A-Za-z0-9_]+", "_", seg).strip("_")
            if seg:
                out.append(seg.lower())
    return "/".join(out)


def numeric(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


# ------------------------------------------------------------------ sources
class Source:
    """One logger: messages [(time_ns, topic, msg)] on one timeline, and where liftoff is on it."""

    def __init__(self, name, messages, liftoff_ns, out_path):
        self.name, self.messages, self.liftoff_ns, self.out_path = name, messages, liftoff_ns, out_path


def load_sources(inputs, liftoff_s):
    sources = []
    witness_inputs, others = [], []
    for pattern in inputs:
        for m in sorted(glob.glob(pattern)) or [pattern]:
            p = Path(m)
            if p.is_dir() or (p.suffix.upper() == ".BIN" and p.name[:1].upper() in witness.KIND):
                witness_inputs.append(p)
            elif p.is_file():
                others.append(p)
            else:
                print(f"skipped {p}: not found", file=sys.stderr)
    # Witness: each folder is a session; loose BIN files together form one session.
    groups = [[p] for p in witness_inputs if p.is_dir()]
    loose = [p for p in witness_inputs if not p.is_dir()]
    if loose:
        groups.append(loose)
    for group in groups:
        files = witness.find_witness_files(group)
        if not files:
            print(f"skipped {group[0]}: no I/S/P *.BIN files", file=sys.stderr)
            continue
        msgs, lift = witness.load_witness(files, liftoff_s)
        name = group[0].name if group[0].is_dir() else files[0].stem[1:]
        out = group[0].with_suffix(".rrd") if group[0].is_dir() else files[0].with_name(name + ".rrd")
        sources.append(Source(name, msgs, None if lift is None else int(round(lift * 1e9)), out))
    for f in others:
        reader = logs.reader_for(f)
        if reader is None:
            print(f"skipped {f.name}: not a known log", file=sys.stderr)
            continue
        raw = logs.read_file(f)
        lift_ns = None
        if reader is logs.read_aim_csv:
            if liftoff_s is not None:
                raw = logs.shift_to_liftoff(f.name, raw, liftoff_s)
                lift_ns = int(round(liftoff_s * 1e9))
            else:
                states = sorted((m[0], m[3].get("state")) for m in raw if m[2] == "/aim_onboard/state")
                lift_ns = next((t for (t, s), (_, p) in zip(states[1:], states) if s == "Boost" and p == "Pad"), None)
        elif reader is logs.read_raven:
            # Flight_Time_(s) is 0 at the Raven's liftoff.
            for t, _p, _topic, msg in raw:
                ft = msg.get("Flight_Time_(s)")
                if numeric(ft) and ft >= 0:
                    lift_ns = t - int(round(ft * 1e9))
                    break
        elif liftoff_s is not None and liftoff_s > 1e8:
            lift_ns = int(round(liftoff_s * 1e9))         # wall-clock logs (Horizon, AIM MQTT)
        msgs = [(t, topic, msg) for t, _p, topic, msg in raw if not topic.endswith("/fix")]
        sources.append(Source(f.stem, msgs, lift_ns, f.with_suffix(".rrd")))
        print(f"{f.name}: {len(msgs)} messages"
              + ("" if lift_ns is None else f", liftoff at {lift_ns / 1e9:.3f} s on its timeline"))
    return sources


# ------------------------------------------------------------------ writing
def time_columns(t_ns: np.ndarray, liftoff_ns):
    cols = []
    if liftoff_ns is not None:
        cols.append(rr.TimeColumn("flight_time", duration=(t_ns - liftoff_ns) / 1e9))
    if len(t_ns) and t_ns.min() > WALL_NS:
        cols.append(rr.TimeColumn("wall_clock", timestamp=t_ns.astype("datetime64[ns]")))
    else:
        cols.append(rr.TimeColumn("logger_clock", duration=t_ns / 1e9))
    return cols


def set_time(rec, t_ns, liftoff_ns):
    if liftoff_ns is not None:
        rec.set_time("flight_time", duration=(t_ns - liftoff_ns) / 1e9)
    if t_ns > WALL_NS:
        rec.set_time("wall_clock", timestamp=np.datetime64(int(t_ns), "ns"))
    else:
        rec.set_time("logger_clock", duration=t_ns / 1e9)


def write_source(rec, src: Source) -> int:
    series = collections.defaultdict(lambda: ([], []))        # (entity) -> (times, values)
    positions = collections.defaultdict(lambda: ([], []))     # entity -> (times, [lat, lon])
    attitude = collections.defaultdict(lambda: ([], []))      # entity -> (times, [x, y, z, w])
    events = []                                                # (t, entity, text)
    last_event = {}
    for t, topic, msg in src.messages:
        base = entity(topic)
        for k, v in msg.items():
            if k in SKIP_FIELDS or k.startswith("_"):
                continue
            if numeric(v):
                times, values = series[f"{base}/{entity(k)}"]
                times.append(t)
                values.append(float(v))
            elif k in EVENT_FIELDS and isinstance(v, str) and v:
                key = (base, k)
                if last_event.get(key) != v:
                    last_event[key] = v
                    events.append((t, f"{base}/events", f"{k} = {v}"))
        fix = logs.location_fix(t, msg)
        if fix is not None:
            times, pts = positions[f"{base}/position"]
            times.append(t)
            pts.append((fix["latitude"], fix["longitude"]))
        for q in QUATERNIONS:
            if all(numeric(msg.get(c)) for c in q):
                quat = np.array([msg[c] for c in q], dtype=float)
                norm = np.linalg.norm(quat)
                if norm > 0.5:
                    times, quats = attitude[f"{base}/attitude"]
                    times.append(t)
                    quats.append(quat / norm)
                break

    count = 0
    for path, (times, values) in series.items():
        t = np.asarray(times, dtype=np.int64)
        rr.send_columns(path, indexes=time_columns(t, src.liftoff_ns),
                        columns=rr.Scalars.columns(scalars=np.asarray(values)), recording=rec)
        count += len(values)
    for path, (times, pts) in positions.items():
        t = np.asarray(times, dtype=np.int64)
        rr.send_columns(path, indexes=time_columns(t, src.liftoff_ns),
                        columns=rr.GeoPoints.columns(positions=np.asarray(pts), radii=[3.0] * len(pts)),
                        recording=rec)
    for path, (times, quats) in attitude.items():
        t = np.asarray(times, dtype=np.int64)
        rec.log(path, rr.Boxes3D(half_sizes=[[0.05, 0.05, 0.4]]), static=True)
        rr.send_columns(path, indexes=time_columns(t, src.liftoff_ns),
                        columns=rr.Transform3D.columns(quaternion=np.asarray(quats)), recording=rec)
    for t, path, text in events:
        set_time(rec, t, src.liftoff_ns)
        rec.log(path, rr.TextLog(text, level="INFO"))
    print(f"  {src.name}: {len(series)} series ({count} values), {len(positions)} position tracks, "
          f"{len(attitude)} attitude tracks, {len(events)} events")
    return count


def new_recording(path: Path, recording_id: str):
    rec = rr.RecordingStream(APP_ID, recording_id=recording_id)
    rec.save(str(path))
    return rec


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inputs", nargs="+", help="Witness folders/BINs and log files (wildcards ok)")
    ap.add_argument("--liftoff", help="liftoff time: '2026-11-21 13:47:32.6' (local unless it has a zone) "
                                      "or seconds; see the help text above")
    ap.add_argument("-o", "--output", type=Path, help="write everything into this one .rrd")
    ap.add_argument("--recording-id", default=None,
                    help="Rerun recording id (default: from the liftoff time or the first input)")
    args = ap.parse_args()
    if rr.__version__ != RERUN_VERSION:
        print(f"note: written for rerun-sdk {RERUN_VERSION}, found {rr.__version__}", file=sys.stderr)

    liftoff_s = witness.parse_liftoff(args.liftoff) if args.liftoff else None
    sources = load_sources(args.inputs, liftoff_s)
    if not sources:
        print("nothing to convert", file=sys.stderr)
        return 2
    recording_id = args.recording_id or (f"flight_{int(liftoff_s)}" if liftoff_s else f"flight_{sources[0].name}")

    if args.output:
        rec = new_recording(args.output, recording_id)
        for src in sources:
            write_source(rec, src)
        rec.flush()
        print(f"Wrote {args.output}")
    else:
        for src in sources:
            out = src.out_path
            rec = new_recording(out, recording_id)
            write_source(rec, src)
            rec.flush()
            print(f"Wrote {out.resolve()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
