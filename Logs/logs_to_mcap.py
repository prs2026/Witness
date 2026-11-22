r"""Convert Horizon, AIM (MQTT and onboard CSV) and Blue Raven logs to MCAP files for Foxglove.

    .\.venv\Scripts\python.exe logs_to_mcap.py logs\horizon_witness_20261121_134500_123456.jsonl
    .\.venv\Scripts\python.exe logs_to_mcap.py "flight logs\BlRv_*.csv" logs\aim_mqtt_*.jsonl   # one .mcap each
    .\.venv\Scripts\python.exe logs_to_mcap.py logs\horizon_witness_*.jsonl logs\aim_mqtt_*.jsonl -o flight.mcap

Each input file becomes its own MCAP next to it (same name, .mcap); open several in Foxglove
together to see them on one timeline (all times are absolute). -o combines every input into one
file instead. File types are detected from the content:

* Horizon JSONL (horizon_witness_*, horizon_featherweight_*): one topic per source and packet,
  e.g. /horizon/COM24/state, /horizon/COM5/BATT_BLE. Time = host receive time (received_at).
* AIM MQTT JSONL (aim_mqtt_*): /aim/<payload>, e.g. /aim/pressure_payload/v1. Log time = host
  receive time; publish time = AIM's own timestamp when it has one.
* Blue Raven CSV (BlRv_*_HR_*.csv / _LR_*.csv): /raven/<SN>/hr and /raven/<SN>/lr. Time =
  the Raven's date and time columns, taken as this PC's local time zone (the CSV has no zone).
  Repeated column names (the per-channel deployment flags) get _2, _3, ... suffixes.

* AIM onboard flight log exported to CSV (column groups, each with its own "time" column):
  /aim_onboard/<group>, e.g. /aim_onboard/gyro, /aim_onboard/gps, /aim_onboard/state. Time = the AIM's
  own clock (offset 0), so it is not on the Horizon/AIM-MQTT/Raven timeline, unless --liftoff TIME
  puts the AIM's Pad -> Boost state change at TIME (a local date/time, or seconds).

Positions (Witness state, ground station, AIM GNSS, Featherweight GPS) are also published as
foxglove.LocationFix on <topic>/fix for the Map panel. Status/event lines are skipped.
Needs the `mcap` package.
"""

from __future__ import annotations

import argparse
import collections
import csv
import datetime as dt
import glob
import json
import math
import re
import struct
import sys
from pathlib import Path

from mcap.writer import Writer

LOCATION_FIX_SCHEMA = {
    "type": "object", "title": "foxglove.LocationFix",
    "properties": {
        "timestamp": {"type": "object", "properties": {"sec": {"type": "integer"}, "nsec": {"type": "integer"}}},
        "frame_id": {"type": "string"}, "latitude": {"type": "number"}, "longitude": {"type": "number"},
        "altitude": {"type": "number"}, "position_covariance": {"type": "array", "items": {"type": "number"}},
        "position_covariance_type": {"type": "integer"},
    },
}


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
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
    if isinstance(value, dict):
        return "object"
    return "string"


def safe(name) -> str:
    return re.sub(r"[^A-Za-z0-9_./-]+", "_", str(name)).strip("_") or "unknown"


def to_ns(seconds: float) -> int:
    return int(round(seconds * 1e9))


def location_fix(t_ns: int, msg: dict):
    """A LocationFix from a message with usable lat/lon (any of the field spellings), else None."""
    lat = msg.get("latitude", msg.get("gnss_coordinate.latitude", msg.get("latitude_deg")))
    lon = msg.get("longitude", msg.get("gnss_coordinate.longitude", msg.get("longitude_deg")))
    alt = msg.get("gps_altitude", msg.get("gnss_coordinate.altitude", msg.get("altitude")))
    if alt is None and isinstance(msg.get("altitude_ft"), (int, float)):
        alt = msg["altitude_ft"] * 0.3048          # Featherweight reports feet
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in (lat, lon)):
        return None
    if not (-90 <= lat <= 90 and -180 <= lon <= 180) or (lat == 0 and lon == 0):
        return None
    return {"timestamp": {"sec": t_ns // 1_000_000_000, "nsec": t_ns % 1_000_000_000}, "frame_id": "wgs84",
            "latitude": lat, "longitude": lon,
            "altitude": alt if isinstance(alt, (int, float)) and not isinstance(alt, bool) else 0.0,
            "position_covariance": [0.0] * 9, "position_covariance_type": 0}


# ------------------------------------------------------------------ readers -> (log_ns, publish_ns, topic, msg)
def read_jsonl(path: Path):
    with open(path, encoding="utf-8", errors="replace") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            if rec.get("event") == "status":
                continue
            if "topic" in rec and "fields" in rec:                       # AIM MQTT
                received = rec.get("received_at")
                try:
                    t = dt.datetime.fromisoformat(received).timestamp()
                except (TypeError, ValueError):
                    continue
                fields = dict(rec.get("fields") or {})
                topic = "/aim/" + safe("/".join(str(rec["topic"]).split("/")[3:]) or rec["topic"])
                stamp = fields.get("timestamp")
                publish = int(stamp) * 1000 if isinstance(stamp, (int, float)) and stamp > 1e15 else to_ns(t)
                yield to_ns(t), publish, topic, fields
            elif isinstance(rec.get("packet"), dict):                     # Horizon witness/featherweight
                received = rec.get("received_at")
                if not isinstance(received, (int, float)):
                    continue
                packet = rec["packet"]
                name = packet.get("packet") or packet.get("packet_type") or "unknown"
                topic = f"/horizon/{safe(rec.get('source', 'source'))}/{safe(name)}"
                msg = dict(packet)
                for key in ("replay", "sequence", "sent_at"):
                    if key in rec:
                        msg["_" + key] = rec[key]
                yield to_ns(received), to_ns(received), topic, msg


def number(text: str):
    try:
        value = float(text)
    except ValueError:
        return text
    return int(value) if value.is_integer() and "." not in text and "e" not in text.lower() else value


def read_raven(path: Path):
    m = re.search(r"SN(\d+)_(HR|LR)", path.name, re.I)
    topic = f"/raven/SN{m.group(1)}/{m.group(2).lower()}" if m else f"/raven/{safe(path.stem)}"
    with open(path, newline="", encoding="utf-8", errors="replace") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        names, seen = [], {}
        for h in header:
            h = h.strip()
            seen[h] = seen.get(h, 0) + 1
            names.append(h if seen[h] == 1 else f"{h}_{seen[h]}")
        for row in reader:
            if len(row) < 5:
                continue
            rec = dict(zip(names, row))
            try:
                hms = rec["Time"]
                when = dt.datetime.strptime(f"{rec['Year']}-{rec['Month']}-{rec['Day']} {hms}",
                                            "%Y-%m-%d %H:%M:%S.%f" if "." in hms else "%Y-%m-%d %H:%M:%S")
            except (KeyError, ValueError):
                continue
            t = to_ns(when.timestamp())                     # local time zone of this PC
            msg = {k: number(v.strip()) for k, v in rec.items() if k not in ("Year", "Month", "Day", "Time")}
            yield t, t, topic, msg


# AIM onboard flight-log CSV export: column groups, each starting with its own "time" column
# (seconds on the AIM's clock). Topic per group, named from its first column.
AIM_GROUP_TOPICS = {
    "lat_x_accel": "lateral_accel", "gyro_x": "gyro", "lat": "gps", "acceleration": "acceleration",
    "pressure": "pressure", "mag_x": "mag", "eject_bat_voltage": "eject_battery",
    "comp_bat_voltage": "computer_battery", "state": "state", "qx": "orientation",
    "temperature": "temperature", "gps_time_of_week": "gps_time",
}
AIM_GPS_NAMES = {"lat": "latitude", "long": "longitude", "gps_msl": "altitude"}


def field_name(text: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", text.strip().lower()).strip("_") or "field"


def read_aim_csv(path: Path):
    """AIM onboard log exported to CSV. Time = the AIM's own clock (offset 0)."""
    with open(path, newline="", encoding="utf-8", errors="replace") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        starts = [i for i, h in enumerate(header) if h.strip().lower() == "time"]
        groups = []
        for n, start in enumerate(starts):
            end = starts[n + 1] if n + 1 < len(starts) else len(header)
            names = [field_name(h) for h in header[start + 1:end]]
            first = names[0] if names else f"group_{n}"
            topic = AIM_GROUP_TOPICS.get(first) or (first[:-8] if first.endswith("_voltage") else first)
            if first.endswith("_voltage") and first not in AIM_GROUP_TOPICS:
                topic = f"channel_{first[:-8]}_voltage"
            if topic == "gps":
                names = [AIM_GPS_NAMES.get(x, x) for x in names]
            groups.append((start, names, f"/aim_onboard/{topic}"))
        # Once a group has no more samples the export writes empty cells for it, a variable number
        # of them, so later columns shift. Walk each row instead, skipping empty cells: a group whose
        # next time would go backwards (or jump > 120 s) has ended for good, and the cell belongs to
        # a later group.
        last = [None] * len(groups)
        ended = [False] * len(groups)
        for row in reader:
            cells = [c.strip() for c in row]
            p = 0
            for g, (_start, names, topic) in enumerate(groups):
                if ended[g]:
                    continue
                while p < len(cells) and cells[p] == "":
                    p += 1                        # padding of groups that have ended
                if p >= len(cells):
                    break
                try:
                    t = float(cells[p])
                except ValueError:
                    ended[g] = True
                    continue
                if last[g] is not None and not (last[g] - 1e-3 <= t <= last[g] + 120.0):
                    ended[g] = True               # the cell belongs to a later group
                    continue
                last[g] = t
                msg = {"t_s": t}
                for k, name in enumerate(names):
                    value = cells[p + 1 + k] if p + 1 + k < len(cells) else ""
                    if value != "":
                        msg[name] = number(value)
                p += 1 + len(names)
                yield to_ns(t), to_ns(t), topic, msg


def reader_for(path: Path):
    head = path.read_bytes()[:4096].decode("utf-8", errors="replace")
    first_line = head.splitlines()[0] if head else ""
    if first_line.lower().startswith("time,") and first_line.lower().count("time,") > 1:
        return read_aim_csv
    if path.suffix.lower() == ".csv" and head.startswith("Year,Month,Day,Time"):
        return read_raven
    if head.lstrip().startswith("{"):
        return read_jsonl
    return None


# ------------------------------------------------------------------ writing
def write_mcap(messages, out: Path) -> None:
    messages.sort(key=lambda m: m[0])
    props: dict[str, dict] = {}
    for _l, _p, topic, msg in messages:
        if topic.endswith("/fix"):
            continue
        p = props.setdefault(topic, {})
        for k, v in msg.items():
            if v is not None and k not in p:
                p[k] = {"type": json_type(v)}
    with open(out, "wb") as stream:
        writer = Writer(stream)
        writer.start(profile="", library="logs_to_mcap")
        channels = {}
        for _l, _p, topic, _m in messages:
            if topic in channels:
                continue
            if topic.endswith("/fix"):
                schema = writer.register_schema(name="foxglove.LocationFix", encoding="jsonschema",
                                                data=json.dumps(LOCATION_FIX_SCHEMA).encode())
            else:
                schema = writer.register_schema(name=topic.strip("/").replace("/", "."), encoding="jsonschema",
                                                data=json.dumps({"type": "object", "properties": props[topic]}).encode())
            channels[topic] = writer.register_channel(topic=topic, message_encoding="json", schema_id=schema)
        for seq, (log_ns, pub_ns, topic, msg) in enumerate(messages):
            writer.add_message(channel_id=channels[topic], log_time=log_ns, publish_time=pub_ns,
                               data=json.dumps(msg).encode(), sequence=seq)
        writer.finish()
    print(f"  -> {out} ({len(messages)} messages, {len(channels)} topics: {', '.join(sorted(channels))})")


def read_file(f: Path):
    """[(log_ns, publish_ns, topic, msg)] of one input file, or None if it is not a known log."""
    reader = reader_for(f)
    if reader is None:
        return None
    messages = []
    for log_ns, pub_ns, topic, msg in reader(f):
        msg = clean(msg)
        messages.append((log_ns, pub_ns, topic, msg))
        fix = location_fix(log_ns, msg)
        if fix is not None:
            messages.append((log_ns, pub_ns, topic + "/fix", fix))
    return messages


def parse_liftoff(text: str) -> float:
    """Seconds (Unix time for a date) from '2026-11-21 13:47:32.6' (local unless it has a zone) or '2410.01'."""
    try:
        return float(text)
    except ValueError:
        return dt.datetime.fromisoformat(text.strip().replace("Z", "+00:00")).timestamp()


def shift_to_liftoff(name, messages, target):
    """Move an AIM onboard log so its first Pad -> Boost state change is at target (seconds)."""
    states = sorted((m[0], m[3].get("state")) for m in messages if m[2] == "/aim_onboard/state")
    lift = next((t for (t, s), (_, prev) in zip(states[1:], states) if s == "Boost" and prev == "Pad"), None)
    if lift is None:
        print(f"{name}: no Pad -> Boost state change; --liftoff not applied", file=sys.stderr)
        return messages
    shift = to_ns(target) - lift
    print(f"{name}: AIM liftoff at {lift / 1e9:.3f} s on its own clock")
    out = []
    for log_ns, pub_ns, topic, msg in messages:
        if topic.endswith("/fix"):
            t = log_ns + shift
            msg = dict(msg, timestamp={"sec": t // 1_000_000_000, "nsec": t % 1_000_000_000})
        out.append((max(0, log_ns + shift), max(0, pub_ns + shift), topic, msg))
    return out


# ------------------------------------------------------------------ main
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inputs", nargs="+", help="Horizon/AIM .jsonl and Blue Raven .csv files (wildcards ok)")
    ap.add_argument("--liftoff",
                    help="AIM onboard CSV only: time to put the AIM's Pad -> Boost transition at, a date and "
                         "time ('2026-11-21 13:47:32.6', local unless it ends in Z or +hh:mm) or seconds")
    ap.add_argument("-o", "--output", type=Path,
                    help="combine every input into this one .mcap (default: one .mcap next to each input)")
    args = ap.parse_args()

    files = []
    for pattern in args.inputs:
        matches = sorted(glob.glob(pattern)) or [pattern]
        files += [Path(m) for m in matches]
    combined, written = [], 0
    for f in files:
        if not f.is_file():
            print(f"skipped {f}: not a file", file=sys.stderr)
            continue
        messages = read_file(f)
        if messages is not None and args.liftoff and reader_for(f) is read_aim_csv:
            messages = shift_to_liftoff(f.name, messages, parse_liftoff(args.liftoff))
        if messages is None:
            print(f"skipped {f.name}: not a Horizon/AIM JSONL or Blue Raven CSV", file=sys.stderr)
            continue
        print(f"{f.name}: {len(messages)} messages")
        if not messages:
            continue
        if args.output:
            combined += messages
        else:
            write_mcap(messages, f.with_suffix(".mcap"))
            written += 1
    if args.output and combined:
        write_mcap(combined, args.output)
        written += 1
    if not written:
        print("nothing to write", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
