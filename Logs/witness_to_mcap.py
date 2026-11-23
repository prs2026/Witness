r"""Convert Witness flight logs (I*.BIN, S*.BIN, P*.BIN) to one MCAP file for Foxglove.

    .\.venv\Scripts\python.exe witness_to_mcap.py C:\flight\I0000123.BIN C:\flight\S0000123.BIN C:\flight\P0000123.BIN
    .\.venv\Scripts\python.exe witness_to_mcap.py C:\flight             # every I/S/P *.BIN in the folder
    .\.venv\Scripts\python.exe witness_to_mcap.py C:\flight -o flight.mcap
    .\.venv\Scripts\python.exe witness_to_mcap.py C:\flight --liftoff "2026-11-21 13:47:32.6"

Decoding is converter.py's (same rows as its CSVs). Each file kind and packet type
becomes its own JSON topic, e.g. /witness/imu, /witness/sensors,
/witness/packets/state, /witness/packets/kalman. Message time is the row's
time_seconds (device uptime, so the three files of one session line up). The offset is 0 by
default, so the timeline starts at uptime 0; --utc-offset SECONDS shifts it, or --liftoff TIME puts
each file's own Pad idle -> Boost transition at TIME (a local date/time, or seconds such as 2410.01
to match the AIM clock). The three files' clocks differ, so each file is aligned on its own liftoff:
without --liftoff, all three are put on the I*.BIN file's uptime at liftoff. A file without a
liftoff (in-flight resets can let the P*.BIN ring overwrite the launch) is aligned on the last
in-flight reset it shares with an aligned file, else on the end of the logs. U*.BIN (raw
GPS) is skipped. Needs the `mcap` package (pip install mcap).
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import sys
from pathlib import Path

from mcap.writer import Writer

# converter.py (the Witness log decoder) lives in Software/Horizon; this script may sit elsewhere.
for _dir in (Path(__file__).resolve().parent, Path(__file__).resolve().parent.parent / "Software" / "Horizon"):
    if (_dir / "converter.py").is_file():
        sys.path.insert(0, str(_dir))
        break
import converter  # noqa: E402

KIND = {"I": "imu", "S": "sensors", "P": "packets"}


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, (bytes, bytearray)):
        return value.hex()
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def json_type(value) -> str:
    if isinstance(value, bool):
        return "boolean"
    if isinstance(value, (int, float)):
        return "number"
    if isinstance(value, list):
        return "array"
    return "string"


def topic_for(kind: str, row: dict) -> str:
    if kind == "packets":
        return f"/witness/packets/{row.get('packet_type') or 'unknown'}"
    return f"/witness/{kind}"


def rows_of(path: Path):
    data = path.read_bytes()
    _desc, decoder = converter.select_decoder(path, data)
    return list(decoder(data, converter.DecodeStats()))


BOOST = 1
MAX_SPAN_S = 6 * 3600.0          # rows further than this from a file's median time are corrupt
STATE_ROWS = ("state", "flash_sample", "imu_sample")     # rows whose flight_state is Witness's own


def last_resume(rows):
    """Uptime (s) of the last in-flight reset marker (resumed row) in a file, or None."""
    times = [row.get("time_seconds") for row in rows if row.get("resumed")]
    times = [t for t in times if isinstance(t, (int, float))]
    return max(times) if times else None


def liftoff_time(rows):
    """Uptime (s) of the first Pad idle -> Boost transition in a file's state rows, or None."""
    previous = None
    for row in rows:
        if row.get("packet_type") not in STATE_ROWS:
            continue
        state, t = row.get("flight_state"), row.get("time_seconds")
        if state == BOOST and previous == 0 and isinstance(t, (int, float)):
            return float(t)
        if state is not None:
            previous = state
    return None


def parse_liftoff(text: str) -> float:
    """Seconds (Unix time for a date) from '2026-11-21 13:47:32.6' (local unless it has a zone) or '2410.01'."""
    try:
        return float(text)
    except ValueError:
        pass
    when = dt.datetime.fromisoformat(text.strip().replace("Z", "+00:00"))
    return when.timestamp()          # a naive time is local time


def to_ns(seconds: float) -> int:
    return max(0, int(round(seconds * 1e9)))


def find_witness_files(inputs) -> list:
    """I/S/P *.BIN files from files and folders."""
    files = []
    for p in inputs:
        p = Path(p)
        files += sorted(f for f in p.glob("*.BIN") if f.name[:1].upper() in KIND) if p.is_dir() else [p]
    return [f for f in files if f.name[:1].upper() in KIND]


def load_witness(files, target=None, utc_offset=0.0):
    """Decode and synchronise Witness I/S/P files. Returns ([(time_ns, topic, msg)], liftoff_s):
    liftoff_s is where liftoff sits on the returned timeline (None if no file has one)."""
    loaded = []              # (file, kind, rows, [(t, topic, msg)], liftoff uptime or None)
    for f in files:
        kind = KIND[f.name[0].upper()]
        rows = rows_of(f)
        file_messages = []
        for row in rows:
            t = row.get("time_seconds")
            if not isinstance(t, (int, float)) or not math.isfinite(t) or t < 0:
                continue
            msg = {k: clean(v) for k, v in row.items() if k != "raw_packet_hex"}
            file_messages.append((t, topic_for(kind, row), msg))
        # A few corrupt records can carry absurd times (years away); keep the flight's span only.
        if file_messages:
            middle = sorted(m[0] for m in file_messages)[len(file_messages) // 2]
            good = [m for m in file_messages if abs(m[0] - middle) <= MAX_SPAN_S]
            if len(good) < len(file_messages):
                print(f"{f.name}: dropped {len(file_messages) - len(good)} rows with times more than "
                      f"{MAX_SPAN_S / 3600:.0f} h from the rest", file=sys.stderr)
            file_messages = good
        lift = liftoff_time(rows)
        loaded.append((f, kind, rows, file_messages, lift))

    # Synchronise: the files' clocks differ, so each file's own first Pad idle -> Boost is put at
    # the same instant: --liftoff if given, else the liftoff of the I*.BIN file (or the first file
    # that has one). A file without a liftoff (the P*.BIN packet ring can overwrite the launch after
    # in-flight resets) is aligned on the last in-flight reset (resume marker) it shares with a
    # file that has a liftoff; failing that, on the end of the logs. --utc-offset is added after.
    lifts = {f.name[0].upper(): lift for f, _k, _r, _m, lift in loaded if lift is not None}
    if target is None and lifts:
        target = lifts.get("I", next(iter(lifts.values())))
    offset_of = {}
    for f, _k, _r, _m, lift in loaded:
        if target is not None and lift is not None:
            offset_of[f] = target - lift
            print(f"{f.name}: liftoff at its uptime {lift:.3f} s")
    refs = [(f, rows, msgs) for f, _k, rows, msgs, _l in loaded if f in offset_of]
    messages = []            # (time_ns, topic, dict)
    for f, kind, rows, file_messages, lift in loaded:
        if f not in offset_of:
            offset_of[f] = 0.0
            if refs:
                mine = last_resume(rows)
                pairs = [(rf, last_resume(rrows)) for rf, rrows, _ in refs]
                pairs = [(rf, t) for rf, t in pairs if t is not None]
                if mine is not None and pairs:
                    rf, t = pairs[0]
                    offset_of[f] = offset_of[rf] + t - mine
                    how = f"its last in-flight reset (uptime {mine:.1f} s) = {rf.name}'s ({t:.1f} s)"
                else:
                    rf, _rows, rmsgs = refs[0]
                    end_ref = max(m[0] for m in rmsgs) if rmsgs else 0.0
                    end_me = max(m[0] for m in file_messages) if file_messages else 0.0
                    offset_of[f] = offset_of[rf] + end_ref - end_me
                    how = f"the end of the log, matched to the end of {rf.name}"
                print(f"{f.name}: no Pad idle -> Boost in this file; aligned on {how} "
                      "(approximate: reset gaps without GPS time are estimated)", file=sys.stderr)
        offset = offset_of[f] + (utc_offset or 0.0)
        messages += [(to_ns(t + offset), topic, msg) for t, topic, msg in file_messages if t + offset >= 0]
        print(f"{f.name}: {len(file_messages)} of {len(rows)} rows ({kind})")
    return messages, target


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inputs", nargs="+", type=Path, help="I/S/P *.BIN files or folders")
    ap.add_argument("-o", "--output", type=Path, help="output .mcap (default: next to the first input)")
    when = ap.add_mutually_exclusive_group()
    when.add_argument("--utc-offset", type=float, default=0.0,
                      help="seconds to add to Witness uptime (default 0: the timeline is Witness uptime)")
    when.add_argument("--liftoff",
                      help="time to put Witness's Pad idle -> Boost transition at: a date and time "
                           "('2026-11-21 13:47:32.6', local time unless it ends in Z or +hh:mm) or a "
                           "number of seconds (e.g. 2410.01 to match the AIM clock)")
    args = ap.parse_args()

    files = find_witness_files(args.inputs)
    if not files:
        print("no I*/S*/P*.BIN files given", file=sys.stderr)
        return 2
    out = args.output or files[0].with_name(files[0].stem[1:] + ".mcap")

    target = parse_liftoff(args.liftoff) if args.liftoff else None
    messages, target = load_witness(files, target, args.utc_offset)
    if args.liftoff and target > 1e8:
        print(f"liftoff = {dt.datetime.fromtimestamp(target).isoformat(sep=' ')} local time")
    if messages:
        span = (max(m[0] for m in messages) - min(m[0] for m in messages)) / 1e9
        print(f"timeline span {span:.1f} s")
    messages.sort(key=lambda m: m[0])

    # One JSON schema per topic, from the fields seen on it.
    fields: dict[str, dict] = {}
    for _t, topic, msg in messages:
        props = fields.setdefault(topic, {})
        for k, v in msg.items():
            if v is not None and k not in props:
                props[k] = {"type": json_type(v)}

    with open(out, "wb") as stream:
        writer = Writer(stream)
        writer.start(profile="", library="witness_to_mcap")
        channels = {}
        for topic, props in fields.items():
            schema = writer.register_schema(
                name=topic.strip("/").replace("/", "."), encoding="jsonschema",
                data=json.dumps({"type": "object", "properties": props}).encode())
            channels[topic] = writer.register_channel(topic=topic, message_encoding="json", schema_id=schema)
        for seq, (t, topic, msg) in enumerate(messages):
            writer.add_message(channel_id=channels[topic], log_time=t, publish_time=t,
                               data=json.dumps(msg).encode(), sequence=seq)
        writer.finish()
    print(f"Wrote {len(messages)} messages on {len(channels)} topics to {out}")
    for topic in sorted(channels):
        print("  ", topic)
    return 0


if __name__ == "__main__":
    sys.exit(main())
