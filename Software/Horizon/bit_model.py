"""Built-in self test (BIT) model: wire decoding, the check table and the status shown to the operator.

No GUI and no transport. Wire layouts are in comms.h (IRIS_PACKET_ID_BIT_SUMMARY 0x0B,
IRIS_PACKET_ID_BIT_DETAIL 0x10) and Software/BIT_PLAN_2026-11-18.md; the check table below is the
firmware's (Witness/softwarev1/main/bit_types.h, kChecks) and tests/test_bit.py compares the two.

Horizon never runs BIT by itself (owner decision, plan Q8): this module only decodes and
summarises; the monitor sends 0x5442 only from the "Run BIT" button.
"""

from __future__ import annotations

import threading

SUMMARY_PAYLOAD_LENGTH = 12
DETAIL_PAYLOAD_LENGTH = 99
DETAIL_RECORDS = 24
DETAIL_HEADER = 3
DETAIL_RECORD_LENGTH = 4
DETAIL_UNUSED_ID = 0xFF
SUMMARY_VERSION = 1

VERDICT_NOT_RUN, VERDICT_RUNNING, VERDICT_GO, VERDICT_GO_WARN, VERDICT_NO_GO = 0, 1, 2, 3, 4
VERDICT_NAMES = {0: "NOT RUN", 1: "RUNNING", 2: "GO", 3: "GO (warnings)", 4: "NO-GO"}
RUN_KINDS = {0: "startup", 1: "commanded", 2: "passive"}
RESULT_PASS, RESULT_FAIL, RESULT_SKIPPED, RESULT_NA, RESULT_NOT_YET, RESULT_ERROR = range(6)
RESULT_NAMES = {0: "pass", 1: "FAIL", 2: "skipped", 3: "n/a", 4: "not yet", 5: "ERROR"}
CLASS_NAMES = {0: "info", 1: "degraded", 2: "blocking"}
GROUP_NAMES = {
    0: "platform", 1: "supervision", 2: "IMU", 3: "barometer", 4: "attitude", 5: "NAND", 6: "log session",
    7: "GPS", 8: "radio", 9: "CAN", 10: "Iris", 11: "power", 12: "clocks", 13: "ground", 14: "reserved",
    15: "camera",
}
STALE_S = 300.0            # plan 6.3: STALE after 5 minutes without a refresh

# (check id = group << 4 | number, highest class it can report, short name): bit_types.h kChecks.
CHECKS = (
    (0x01, "Degraded", "reset reason"),
    (0x02, "Degraded", "image identity"),
    (0x03, "Blocking", "heap"),
    (0x04, "Blocking", "stack margins"),
    (0x05, "Degraded", "mcu temperature"),
    (0x06, "Info", "image integrity"),
    (0x07, "Degraded", "heap structure"),
    (0x11, "Blocking", "task census"),
    (0x12, "Degraded", "task watchdog"),
    (0x13, "Degraded", "nvs"),
    (0x14, "Blocking", "hitl inactive"),
    (0x15, "Blocking", "flight configuration"),
    (0x16, "Degraded", "stale requests"),
    (0x21, "Blocking", "imu alive"),
    (0x22, "Blocking", "imu rate and loss"),
    (0x23, "Blocking", "gravity and noise"),
    (0x24, "Degraded", "gyro bias and noise"),
    (0x25, "Degraded", "high-g consistency"),
    (0x26, "Info", "imu temperature"),
    (0x27, "Degraded", "imu self-test"),
    (0x28, "Degraded", "body orientation"),
    (0x31, "Blocking", "baro ready"),
    (0x32, "Blocking", "baro plausible"),
    (0x33, "Degraded", "baro noise"),
    (0x34, "Degraded", "baro rate"),
    (0x35, "Info", "baro vs gps"),
    (0x41, "Degraded", "attitude valid"),
    (0x42, "Degraded", "position filter"),
    (0x43, "Blocking", "launch readiness"),
    (0x44, "Info", "pad quiet"),
    (0x45, "Info", "state sanity"),
    (0x46, "Info", "calibration"),
    (0x47, "Degraded", "pad heading set"),
    (0x51, "Blocking", "nand chip"),
    (0x52, "Blocking", "nand write test"),
    (0x53, "Degraded", "nand bad blocks"),
    (0x54, "Blocking", "nand writer health"),
    (0x61, "Blocking", "log session"),
    (0x62, "Blocking", "log free space"),
    (0x63, "Blocking", "log end-to-end"),
    (0x64, "Degraded", "log queues and drops"),
    (0x65, "Blocking", "logger heartbeat"),
    (0x66, "Degraded", "prelaunch ring"),
    (0x67, "Info", "post-flight log"),
    (0x71, "Degraded", "gps receiver"),
    (0x72, "Degraded", "gps rate"),
    (0x74, "Degraded", "gps fix"),
    (0x75, "Info", "utc valid"),
    (0x81, "Blocking", "radio ready"),
    (0x82, "Degraded", "radio device errors"),
    (0x83, "Blocking", "radio power"),
    (0x85, "Degraded", "radio tx path"),
    (0x86, "Degraded", "radio rx path"),
    (0xB1, "Blocking", "battery voltage"),
    (0xB2, "Degraded", "battery adc"),
    (0xB3, "Degraded", "3v3 rail"),
    (0xB4, "Info", "brownout detector"),
    (0xC1, "Degraded", "imu clock"),
    (0xC4, "Info", "time of day"),
)
CHECK_NAMES = {cid: name for cid, _cls, name in CHECKS}
CHECK_COUNT = len(CHECKS)
DETAIL_PAGE_COUNT = (CHECK_COUNT + DETAIL_RECORDS - 1) // DETAIL_RECORDS


def check_label(check_id: int) -> str:
    """"8.3 radio power" for a check ID (an unknown ID keeps its number)."""
    name = CHECK_NAMES.get(check_id)
    base = f"{check_id >> 4}.{check_id & 0x0F}"
    return f"{base} {name}" if name else f"{base} (unknown check)"


def group_names(mask: int) -> list[str]:
    return [GROUP_NAMES[g] for g in range(16) if mask & (1 << g)]


def decode_summary(payload) -> dict | None:
    """0x0B payload (12 data bytes) -> packet dict, None for a wrong length.

    Another version is still decoded (the frame length is fixed, so the stream stays in step);
    "version" lets the display say the layout is unknown.
    """
    if len(payload) != SUMMARY_PAYLOAD_LENGTH:
        return None
    d = bytes(payload)
    verdict = d[0] & 0x07
    return {"packet": "bit_summary", "version": d[0] >> 4, "verdict": verdict,
            "verdict_name": VERDICT_NAMES.get(verdict, f"?{verdict}"), "bench": bool(d[0] & 0x08),
            "run_kind": d[1] & 0x03, "run_kind_name": RUN_KINDS.get(d[1] & 0x03, "?"),
            "flight_state": (d[1] >> 2) & 0x07, "core_done": bool(d[1] & 0x20), "settled": bool(d[1] & 0x40),
            "active_ran": bool(d[1] & 0x80),
            "blocking_mask": (d[2] << 8) | d[3], "degraded_mask": (d[4] << 8) | d[5],
            "not_evaluated_mask": (d[6] << 8) | d[7], "first_fail_id": d[8], "sequence": d[9],
            "age_s": (d[10] << 8) | d[11]}


def decode_detail(payload) -> dict | None:
    """0x10 payload (99 data bytes: page, page count, sequence, 24 x 4-byte records)."""
    if len(payload) != DETAIL_PAYLOAD_LENGTH:
        return None
    d = bytes(payload)
    records = []
    for r in range(DETAIL_RECORDS):
        rec = d[DETAIL_HEADER + r * DETAIL_RECORD_LENGTH:DETAIL_HEADER + (r + 1) * DETAIL_RECORD_LENGTH]
        if rec[0] == DETAIL_UNUSED_ID:
            continue
        records.append({"id": rec[0], "result": rec[1] & 0x07, "class": (rec[1] >> 3) & 0x03,
                        "bench": bool(rec[1] & 0x20), "value": (rec[2] << 8) | rec[3]})
    return {"packet": "bit_detail", "page": d[0], "page_count": d[1], "sequence": d[2], "records": records}


def ack_verdict(detail: int) -> tuple[str, int]:
    """Verdict name and number of blocking-failed groups from a BIT run ack detail (verdict << 8 | groups)."""
    return VERDICT_NAMES.get(detail >> 8, f"?{detail >> 8}"), detail & 0xFF


class BitModel:
    """Latest summary and detail pages seen, and what to show for them.

    Thread safe (the monitor feeds it on the GUI thread, a worker reads missing_pages()). Times are
    monotonic seconds passed in by the caller, so tests need no clock.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self.summary = None
        self.received = None
        self.snapshot = False
        self.pages = {}             # page -> detail packet, all of run self.pages_sequence
        self.pages_sequence = None

    def clear(self):
        with self._lock:
            self.summary = None
            self.received = None
            self.snapshot = False
            self.pages = {}
            self.pages_sequence = None

    def feed(self, packet, received, snapshot=False):
        """Take a decoded bit_summary / bit_detail packet; anything else is ignored. True if used."""
        kind = packet.get("packet") if isinstance(packet, dict) else None
        with self._lock:
            if kind == "bit_summary":
                self.summary, self.received, self.snapshot = dict(packet), received, snapshot
                return True
            if kind == "bit_detail":
                if packet.get("sequence") != self.pages_sequence:
                    self.pages, self.pages_sequence = {}, packet.get("sequence")
                self.pages[packet.get("page")] = dict(packet)
                return True
        return False

    def _held(self):
        """(summary, pages of the summary's run)."""
        with self._lock:
            summary = self.summary
            pages = dict(self.pages) if summary is not None and self.pages_sequence == summary["sequence"] else {}
        return summary, pages

    def failing(self):
        """[(id, label, class name, result name, value)] of the checks that failed in the held detail pages
        of the current run, blocking first. Empty when no detail has arrived for this run."""
        _summary, pages = self._held()
        out = []
        for page in pages.values():
            for rec in page["records"]:
                if rec["result"] in (RESULT_FAIL, RESULT_ERROR):
                    out.append((rec["id"], check_label(rec["id"]), CLASS_NAMES.get(rec["class"], "?"),
                                RESULT_NAMES[rec["result"]], rec["value"]))
        out.sort(key=lambda r: (r[2] != "blocking", r[0]))
        return out

    def missing_pages(self):
        """Detail pages not held for the current run (all of them while none is)."""
        _summary, pages = self._held()
        return [p for p in range(DETAIL_PAGE_COUNT) if p not in pages]

    def status(self, now):
        """(text, kind, tooltip) for the compact BIT readout, or None before any summary arrived.

        kind: ok / warn / bad / bench / muted (flight_render.STRIP_COLORS). A NO-GO whose only blocking
        failures are bench conditions (HITL active, radio power below -3 dBm: the firmware sets the
        summary's bench flag) is "bench", not "bad".
        """
        with self._lock:
            s, received, snapshot = self.summary, self.received, self.snapshot
        if s is None:
            return None
        age = s["age_s"] + max(0.0, now - received)
        stale = snapshot or age >= STALE_S or now - received >= STALE_S
        if s["version"] != SUMMARY_VERSION:
            return (f"BIT ? (packet version {s['version']})", "warn",
                    f"Horizon decodes BIT summary version {SUMMARY_VERSION}; Witness sent {s['version']}.")
        verdict = s["verdict"]
        lines = [f"Run {s['sequence']} ({s['run_kind_name']}), last evaluated {age:.0f} s ago"
                 + (" (snapshot)" if snapshot else "")]
        blocking, degraded = group_names(s["blocking_mask"]), group_names(s["degraded_mask"])
        if blocking:
            lines.append("Blocking groups: " + ", ".join(blocking))
        if degraded:
            lines.append("Degraded groups: " + ", ".join(degraded))
        if s["not_evaluated_mask"]:
            lines.append("Not evaluated yet: " + ", ".join(group_names(s["not_evaluated_mask"])))
        fails = self.failing()
        for _cid, label, cls, result, value in fails[:12]:
            lines.append(f"  {label}: {cls} {result} (value {value})")
        if len(fails) > 12:
            lines.append(f"  ... {len(fails) - 12} more")
        lines.append("core checks done" if s["core_done"] else "core checks still running")
        lines.append("settled" if s["settled"] else "still settling")
        if verdict in (VERDICT_GO_WARN, VERDICT_NO_GO) and not fails:
            lines.append("Check names not received yet: use 'BIT detail'.")
        first = check_label(s["first_fail_id"]) if s["first_fail_id"] else ""
        if verdict == VERDICT_NOT_RUN:
            text, kind = "BIT not run", "muted"
        elif verdict == VERDICT_RUNNING:
            text, kind = "BIT running", "muted"
        elif verdict == VERDICT_GO:
            text, kind = "BIT GO", "ok"
        elif verdict == VERDICT_GO_WARN:
            more = (len(fails) if fails else bin(s["degraded_mask"]).count("1")) - 1
            text = "BIT GO with warnings" + (f": {first}" if first else "") + (f" (+{more} more)" if first and more > 0 else "")
            kind = "warn"
        elif verdict == VERDICT_NO_GO:
            text = "BIT NO-GO" + (" (bench)" if s["bench"] else "") + (f": {first}" if first else "")
            kind = "bench" if s["bench"] else "bad"
            if s["bench"]:
                lines.append("Bench condition: every blocking failure is HITL active or low radio power.")
        else:
            text, kind = f"BIT ? verdict {verdict}", "warn"
        if verdict in (VERDICT_GO, VERDICT_GO_WARN, VERDICT_NO_GO) and not s["core_done"]:
            text += " (core pending)"
            kind = "warn" if kind == "ok" else kind
        if stale and verdict != VERDICT_NOT_RUN:
            text += " - STALE"
            kind = "warn" if kind in ("ok", "muted") else kind
        return text, kind, "\n".join(lines)
