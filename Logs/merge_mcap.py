r"""Merge several MCAP files into one, in time order, for upload and analysis (e.g. Foxglove).

    .\.venv\Scripts\python.exe merge_mcap.py "flight logs\*.mcap" -o "flight logs\flight_all.mcap"
    .\.venv\Scripts\python.exe merge_mcap.py a.mcap b.mcap c.mcap -o merged.mcap --prefix

Topics and schemas are copied as they are. Two inputs with the same topic share one channel
unless --prefix is given, which puts each file's topics under /<file name>/... instead.
--shift FILE=SECONDS moves one input in time (repeatable; FILE is the file name or its stem).
Messages are written in log-time order and zstd-compressed. The output file is never read as an
input, even when a wildcard matches it.
"""

from __future__ import annotations

import argparse
import glob
import heapq
import re
import sys
from pathlib import Path

from mcap.reader import make_reader
from mcap.writer import Writer


def safe(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("_") or "file"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inputs", nargs="+", help=".mcap files (wildcards ok)")
    ap.add_argument("-o", "--output", type=Path, required=True, help="merged .mcap to write")
    ap.add_argument("--prefix", action="store_true", help="put each file's topics under /<file name>/")
    ap.add_argument("--shift", action="append", default=[], metavar="FILE=SECONDS",
                    help="move one input in time by SECONDS (repeatable)")
    args = ap.parse_args()

    out = args.output.resolve()
    files = []
    for pattern in args.inputs:
        for m in sorted(glob.glob(pattern)) or [pattern]:
            p = Path(m)
            if p.resolve() != out and p not in files:
                files.append(p)
    missing = [f for f in files if not f.is_file()]
    if missing or not files:
        print("not found: " + ", ".join(map(str, missing)) if missing else "no inputs", file=sys.stderr)
        return 2
    shifts = {}
    for item in args.shift:
        name, _, seconds = item.partition("=")
        shifts[name] = int(round(float(seconds) * 1e9))

    handles = [open(f, "rb") for f in files]
    try:
        streams = []
        for i, (f, h) in enumerate(zip(files, handles)):
            shift = shifts.get(f.name, shifts.get(f.stem, 0))
            prefix = f"/{safe(f.stem)}" if args.prefix else ""

            def stream(i=i, h=h, shift=shift, prefix=prefix):
                for schema, channel, message in make_reader(h).iter_messages(log_time_order=True):
                    yield message.log_time + shift, i, schema, channel, message, shift, prefix
            streams.append(stream())

        counts = [0] * len(files)
        with open(out, "wb") as sink:
            writer = Writer(sink)
            writer.start(profile="", library="merge_mcap")
            schema_ids, channel_ids = {}, {}
            seq = 0
            for log_time, i, schema, channel, message, shift, prefix in heapq.merge(*streams, key=lambda x: x[:2]):
                if log_time < 0:
                    continue
                skey = (schema.name, schema.encoding, bytes(schema.data)) if schema else None
                if skey is not None and skey not in schema_ids:
                    schema_ids[skey] = writer.register_schema(name=schema.name, encoding=schema.encoding,
                                                              data=bytes(schema.data))
                topic = prefix + (channel.topic if channel.topic.startswith("/") or not prefix
                                  else "/" + channel.topic)
                ckey = (topic, channel.message_encoding, skey)
                if ckey not in channel_ids:
                    channel_ids[ckey] = writer.register_channel(
                        topic=topic, message_encoding=channel.message_encoding,
                        schema_id=schema_ids[skey] if skey is not None else 0,
                        metadata=dict(channel.metadata or {}))
                writer.add_message(channel_id=channel_ids[ckey], log_time=log_time,
                                   publish_time=max(0, message.publish_time + shift),
                                   data=message.data, sequence=seq)
                seq += 1
                counts[i] += 1
            writer.finish()
    finally:
        for h in handles:
            h.close()

    for f, n in zip(files, counts):
        print(f"{f.name}: {n} messages")
    print(f"Wrote {sum(counts)} messages on {len(channel_ids)} topics to {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
