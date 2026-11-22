import csv, json, sys
from mcap.writer import Writer

# usage: python csv2mcap.py in.csv out.mcap time_col topic device_name session_key T0_epoch_seconds
inp, out, tcol, topic, device, session, t0 = sys.argv[1:8]
t0 = float(t0)

with open(inp) as f, open(out, "wb") as o:
    rows = list(csv.DictReader(f))
    cols = [c for c in rows[0] if c != tcol]
    w = Writer(o)
    w.start()
    # Metadata Foxglove reads on import: device and session grouping
    w.add_metadata("foxglove", {"deviceName": device, "sessionKey": session})
    schema = w.register_schema(
        name="row", encoding="jsonschema",
        data=json.dumps({"type": "object",
                         "properties": {c: {"type": "number"} for c in cols}}).encode())
    ch = w.register_channel(topic=topic, message_encoding="json", schema_id=schema)
    for r in rows:
        ns = int((t0 + float(r[tcol])) * 1e9)
        w.add_message(ch, log_time=ns, publish_time=ns,
                      data=json.dumps({c: float(r[c]) for c in cols if r[c] != ""}).encode())
    w.finish()