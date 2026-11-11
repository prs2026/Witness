"""GPS assistance for the Witness receiver: RINEX broadcast ephemeris ->
SkyTraq "Set GPS Ephemeris" (0x41) messages -> chunked 0x05 command frames.

Research summary (SkyTraq AN0037, Phoenix binary messages): the PX1120S has
no almanac-upload message. It does accept GPS ephemeris over UART, 0x41 "Set
GPS Ephemeris" (87-byte payload: SV id, then subframes 1-3, each a reserved
byte plus the 24 data bits of words 2-10), and answers ACK 83 41 / NACK 84 41.
Ephemeris is strictly better than almanac for time to first fix: with it the
receiver computes positions as soon as it has code phase and time, instead of
decoding subframes 1-3 from each satellite (18-30 s at best). The u-blox M8
would take UBX-MGA-GPS-ALM/EPH; that path is not implemented (firmware refuses
it with a log line).

Firmware path (Software/Witness/softwarev1/main/gps_assist.h): the blob is
records of [u16 length][SkyTraq body]; it is sent as 0x05 command frames
BEGIN (0x4741) / DATA (0x4744, 4 bytes each at a byte offset) / COMMIT
(0x4745), which work over USB, the radio and CAN. Witness verifies a CRC-16,
then writes each record to the receiver and logs ACK/NACK counts.

The same ephemerides drive gps-sdr-sim (HITL RF simulation): shift_to() moves
a RINEX file to the scenario start time (like gps-sdr-sim -T, but in one place
so the uploaded and transmitted ephemerides are identical) and write_rinex2()
writes the file gps-sdr-sim reads.
"""

from __future__ import annotations

import argparse
import dataclasses
import datetime as dt
import math
import re
import struct
import sys
import time
from pathlib import Path

# Command opcodes (comms.h IRIS_COMMAND_GPS_ASSIST_*).
CMD_BEGIN, CMD_DATA, CMD_COMMIT, CMD_STATUS = 0x4741, 0x4744, 0x4745, 0x4751
FORMAT_SKYTRAQ = 1
MAX_BLOB = 4096
CHUNK = 4
SKYTRAQ_SET_GPS_EPHEMERIS = 0x41
EPHEMERIS_BODY_LENGTH = 87
GPS_EPOCH = dt.datetime(1980, 1, 6)
SECONDS_IN_WEEK = 604800
GPS_UTC_LEAP_S = 18          # GPS - UTC since 2017-01-01
PI = 3.1415926535898         # IS-GPS-200 value (as gps-sdr-sim)


# ----------------------------------------------------------------- GPS time
def gps_from_datetime(t: dt.datetime) -> tuple[int, float]:
    """(week, seconds of week) for a GPS-time calendar date (no leap seconds)."""
    delta = (t - GPS_EPOCH).total_seconds()
    week = int(delta // SECONDS_IN_WEEK)
    return week, delta - week * SECONDS_IN_WEEK


def datetime_from_gps(week: int, sec: float) -> dt.datetime:
    return GPS_EPOCH + dt.timedelta(seconds=week * SECONDS_IN_WEEK + sec)


def gps_now(offset_s: float = 0.0) -> dt.datetime:
    """Current GPS time (UTC + 18 s) as a naive datetime."""
    utc = dt.datetime.now(dt.timezone.utc).replace(tzinfo=None)
    return utc + dt.timedelta(seconds=GPS_UTC_LEAP_S + offset_s)


# ------------------------------------------------------------------ RINEX
@dataclasses.dataclass
class Ephemeris:
    prn: int
    toc: dt.datetime            # clock reference, GPS time
    af0: float
    af1: float
    af2: float
    iode: float
    crs: float
    deltan: float
    m0: float
    cuc: float
    ecc: float
    cus: float
    sqrta: float
    toe_sec: float
    cic: float
    omg0: float
    cis: float
    inc0: float
    crc: float
    aop: float
    omgdot: float
    idot: float
    code_l2: float
    week: int                   # GPS week of toe (continuous)
    l2p: float
    accuracy: float
    health: float
    tgd: float
    iodc: float
    tx_time: float = 0.0
    fit: float = 4.0

    @property
    def toe(self) -> dt.datetime:
        return datetime_from_gps(self.week, self.toe_sec)


@dataclasses.dataclass
class NavFile:
    ephemerides: list
    header: dict                # 'ion_alpha', 'ion_beta', 'delta_utc', 'leap'


def _num(text: str) -> float:
    text = text.strip().replace("D", "E").replace("d", "e")
    return float(text) if text else 0.0


def _fields(line: str, start: int, count: int = 4, width: int = 19) -> list:
    return [_num(line[start + i * width:start + (i + 1) * width]) for i in range(count)]


def parse_rinex_nav(path) -> NavFile:
    """GPS records from a RINEX 2 (.yyn) or RINEX 3 (mixed or GPS) nav file."""
    lines = Path(path).read_text(encoding="ascii", errors="replace").splitlines()
    version = _num(lines[0][:9]) if lines else 0.0
    header: dict = {}
    body = 0
    for index, line in enumerate(lines):
        label = line[60:].strip()
        if label == "END OF HEADER":
            body = index + 1
            break
        if label == "ION ALPHA":
            header["ion_alpha"] = _fields(line, 2, 4, 12)
        elif label == "ION BETA":
            header["ion_beta"] = _fields(line, 2, 4, 12)
        elif label.startswith("DELTA-UTC"):
            header["delta_utc"] = (_num(line[3:22]), _num(line[22:41]),
                                   int(_num(line[41:50])), int(_num(line[50:59])))
        elif label == "IONOSPHERIC CORR" and line[:4] in ("GPSA", "GPSB"):
            header["ion_alpha" if line[:4] == "GPSA" else "ion_beta"] = _fields(line, 5, 4, 12)
        elif label == "TIME SYSTEM CORR" and line[:4] == "GPUT":
            header["delta_utc"] = (_num(line[5:22]), _num(line[22:38]),
                                   int(_num(line[38:45])), int(_num(line[45:50])))
        elif label == "LEAP SECONDS":
            header["leap"] = int(_num(line[:6]))
    ephemerides = []
    i = body
    v3 = version >= 3.0
    while i < len(lines):
        line = lines[i]
        if not line.strip():
            i += 1
            continue
        if v3:
            system = line[0]
            if system != "G":
                # Skip this record: GLONASS/SBAS 4 lines, others 8.
                i += 4 if system in ("R", "S") else 8
                continue
            prn = int(line[1:3])
            y, mo, d, h, mi, s = (int(line[4:8]), int(line[9:11]), int(line[12:14]),
                                  int(line[15:17]), int(line[18:20]), int(line[21:23]))
            first = _fields(line, 23, 3)
            start = 4
        else:
            prn = int(line[0:2])
            yy = int(line[3:5])
            y = yy + (2000 if yy < 80 else 1900)
            mo, d, h, mi = int(line[6:8]), int(line[9:11]), int(line[12:14]), int(line[15:17])
            s = int(round(_num(line[17:22])))
            first = _fields(line, 22, 3)
            start = 3
        rows = [_fields(lines[i + k], start) for k in range(1, 8) if i + k < len(lines)]
        if len(rows) < 7:
            break
        toc = dt.datetime(y, mo, d, h, mi) + dt.timedelta(seconds=s)
        r = rows
        ephemerides.append(Ephemeris(
            prn=prn, toc=toc, af0=first[0], af1=first[1], af2=first[2],
            iode=r[0][0], crs=r[0][1], deltan=r[0][2], m0=r[0][3],
            cuc=r[1][0], ecc=r[1][1], cus=r[1][2], sqrta=r[1][3],
            toe_sec=r[2][0], cic=r[2][1], omg0=r[2][2], cis=r[2][3],
            inc0=r[3][0], crc=r[3][1], aop=r[3][2], omgdot=r[3][3],
            idot=r[4][0], code_l2=r[4][1], week=int(r[4][2]), l2p=r[4][3],
            accuracy=r[5][0], health=r[5][1], tgd=r[5][2], iodc=r[5][3],
            tx_time=r[6][0], fit=r[6][1]))
        i += 8
    if not ephemerides:
        raise ValueError(f"{path}: no GPS ephemeris records found")
    return NavFile(ephemerides, header)


def shift_to(nav: NavFile, start: dt.datetime) -> tuple[NavFile, dt.datetime]:
    """Move every ephemeris so the earliest clock time is the hour of `start`
    (GPS time); returns the shifted file and the shifted earliest toc.

    The orbits are the file's own; only their reference times move (as
    gps-sdr-sim -T does), so a receiver that is told the scenario time sees a
    consistent constellation. Moving to the hour keeps the scenario start
    within an hour after the first ephemeris set, which gps-sdr-sim requires.
    """
    first = min(e.toc for e in nav.ephemerides)
    target = start.replace(minute=0, second=0, microsecond=0)
    delta = target - first
    base = scenario_iode_base(target)
    ranks = _toc_ranks(nav.ephemerides)
    shifted = []
    for e in nav.ephemerides:
        toe = e.toe + delta
        week, toe_sec = gps_from_datetime(toe)
        # New issue of data for every scenario: a receiver that still holds
        # ephemerides from an earlier scenario (same orbits, other times)
        # sees a changed IODE and replaces them instead of keeping the stale
        # copy. IODC's low 8 bits must equal IODE (IS-GPS-200).
        iode = (base + ranks[id(e)]) % 256
        shifted.append(dataclasses.replace(e, toc=e.toc + delta, week=week, toe_sec=round(toe_sec, 3),
                                           iode=float(iode), iodc=float(iode)))
    header = dict(nav.header)
    if "delta_utc" in header:
        a0, a1, _tot, _wnt = header["delta_utc"]
        week, sec = gps_from_datetime(target)
        header["delta_utc"] = (a0, a1, int(sec // 4096) * 4096, week)
    return NavFile(shifted, header), target


IODE_SETS_PER_HOUR = 16   # >= ephemeris sets per satellite in a daily file (13)


def scenario_iode_base(target: dt.datetime) -> int:
    """First IODE of a scenario starting at `target` (whole hour, GPS time).

    Scenario hour h, ephemeris set k gets IODE (16 h + k) mod 256, so two
    scenarios created less than 16 hours apart never reuse an IODE for
    different data (the most 8 bits allow with up to 16 sets per satellite).
    For older leftovers in the receiver, cold-start it (0x4752)."""
    hours = int((target - GPS_EPOCH).total_seconds() // 3600)
    return (hours * IODE_SETS_PER_HOUR) % 256


def _toc_ranks(ephemerides) -> dict:
    """{id(record): index of its clock time among that satellite's records}."""
    by_prn: dict = {}
    for e in ephemerides:
        by_prn.setdefault(e.prn, []).append(e)
    ranks = {}
    for records in by_prn.values():
        for rank, e in enumerate(sorted(records, key=lambda r: r.toc)):
            ranks[id(e)] = min(rank, IODE_SETS_PER_HOUR - 1)
    return ranks


def _d(value: float, width: int = 19, digits: int = 12) -> str:
    """Fixed-width exponent number, e.g. ' 4.691267386080D-04' (RINEX 2)."""
    return format(float(value), f" .{digits}E").replace("E", "D").rjust(width)


def write_rinex2(nav: NavFile, path) -> Path:
    """Write a RINEX 2.10 GPS nav file (the format gps-sdr-sim reads)."""
    out = ["     2.10           N: GPS NAV DATA                         RINEX VERSION / TYPE"]
    out.append(f"{'Witness HITL':<20}{'gps_assist.py':<20}{dt.datetime.now(dt.timezone.utc).replace(tzinfo=None):%d-%b-%y %H:%M}".ljust(60)[:60]
               + "PGM / RUN BY / DATE")
    h = nav.header
    if "ion_alpha" in h:
        out.append(("  " + "".join(_d(v, 12, 4) for v in h["ion_alpha"])).ljust(60) + "ION ALPHA")
    if "ion_beta" in h:
        out.append(("  " + "".join(_d(v, 12, 4) for v in h["ion_beta"])).ljust(60) + "ION BETA")
    if "delta_utc" in h:
        a0, a1, tot, wnt = h["delta_utc"]
        out.append(f"   {_d(a0, 19, 12)}{_d(a1, 19, 12)}{tot:9d}{wnt:9d}".ljust(60) + "DELTA-UTC: A0,A1,T,W")
    if "leap" in h:
        out.append(f"{h['leap']:6d}".ljust(60) + "LEAP SECONDS")
    out.append(" " * 60 + "END OF HEADER")
    for e in sorted(nav.ephemerides, key=lambda e: (e.toc, e.prn)):
        t = e.toc
        out.append(f"{e.prn:2d} {t.year % 100:02d} {t.month:2d} {t.day:2d} {t.hour:2d} {t.minute:2d}"
                   f"{t.second + t.microsecond / 1e6:5.1f}{_d(e.af0)}{_d(e.af1)}{_d(e.af2)}")
        for row in ((e.iode, e.crs, e.deltan, e.m0), (e.cuc, e.ecc, e.cus, e.sqrta),
                    (e.toe_sec, e.cic, e.omg0, e.cis), (e.inc0, e.crc, e.aop, e.omgdot),
                    (e.idot, e.code_l2, float(e.week), e.l2p), (e.accuracy, e.health, e.tgd, e.iodc),
                    (e.tx_time, e.fit, 0.0, 0.0)):
            out.append("   " + "".join(_d(v) for v in row))
    path = Path(path)
    path.write_text("\n".join(out) + "\n", encoding="ascii")
    return path


def ephemeris_sets(nav: NavFile) -> list:
    """Group records into sets as gps-sdr-sim does (a new set once a clock time
    is more than an hour after the set's first)."""
    sets, start = [], None
    for e in nav.ephemerides:      # file order, as gps-sdr-sim reads it
        if start is None or (e.toc - start).total_seconds() > 3600:
            sets.append({})
            start = e.toc
        sets[-1][e.prn] = e
    return sets


def select_for(nav: NavFile, when: dt.datetime) -> dict:
    """{prn: Ephemeris} of the set gps-sdr-sim transmits at `when`: the first
    set with a clock time within an hour of it."""
    for group in ephemeris_sets(nav):
        if any(-3600 <= (when - e.toc).total_seconds() < 3600 for e in group.values()):
            # Unhealthy SVs too: gps-sdr-sim transmits them with their health
            # bits, and the receiver applies the same flag to the upload.
            return dict(group)
    raise ValueError(f"no ephemeris set within an hour of {when:%Y-%m-%d %H:%M} (GPS)")


# ------------------------------------------------ subframes (IS-GPS-200)
def _tc(value: float, scale: float, bits: int) -> int:
    """Scaled to an integer (truncated toward zero, as gps-sdr-sim) as `bits`-bit
    two's complement."""
    return int(value / scale) & ((1 << bits) - 1)


def subframes(e: Ephemeris) -> list:
    """Subframes 1-3 as 10 words each: the 24 data bits of every word (parity
    omitted), same scaling as gps-sdr-sim eph2sbf. Word 1 is the TLM."""
    p = lambda n: 2.0 ** n  # noqa: E731
    toe = int(e.toe_sec / 16)
    toc_sec = gps_from_datetime(e.toc)[1]
    toc = int(toc_sec / 16)
    wn = e.week % 1024
    iodc, iode = int(e.iodc), int(e.iode)
    tow = int((e.toe_sec // 6)) % 100800       # HOW time of week (6 s units)
    m0 = _tc(e.m0 / PI, p(-31), 32)
    ecc = int(e.ecc / p(-33)) & 0xFFFFFFFF
    sqrta = int(e.sqrta / p(-19)) & 0xFFFFFFFF
    omg0 = _tc(e.omg0 / PI, p(-31), 32)
    inc0 = _tc(e.inc0 / PI, p(-31), 32)
    aop = _tc(e.aop / PI, p(-31), 32)
    tlm = 0x8B0000

    def how(subframe):
        return ((tow & 0x1FFFF) << 7) | ((subframe & 7) << 2)

    sf1 = [tlm, how(1),
           ((wn & 0x3FF) << 14) | ((int(e.code_l2) & 3) << 12) | ((0 & 0xF) << 8)
           | ((int(e.health) & 0x3F) << 2) | ((iodc >> 8) & 3),
           0, 0, 0,
           _tc(e.tgd, p(-31), 8),
           ((iodc & 0xFF) << 16) | (toc & 0xFFFF),
           (_tc(e.af2, p(-55), 8) << 16) | _tc(e.af1, p(-43), 16),
           _tc(e.af0, p(-31), 22) << 2]
    sf2 = [tlm, how(2),
           ((iode & 0xFF) << 16) | _tc(e.crs, p(-5), 16),
           (_tc(e.deltan / PI, p(-43), 16) << 8) | ((m0 >> 24) & 0xFF),
           m0 & 0xFFFFFF,
           (_tc(e.cuc, p(-29), 16) << 8) | ((ecc >> 24) & 0xFF),
           ecc & 0xFFFFFF,
           (_tc(e.cus, p(-29), 16) << 8) | ((sqrta >> 24) & 0xFF),
           sqrta & 0xFFFFFF,
           (toe & 0xFFFF) << 8]
    sf3 = [tlm, how(3),
           (_tc(e.cic, p(-29), 16) << 8) | ((omg0 >> 24) & 0xFF),
           omg0 & 0xFFFFFF,
           (_tc(e.cis, p(-29), 16) << 8) | ((inc0 >> 24) & 0xFF),
           inc0 & 0xFFFFFF,
           (_tc(e.crc, p(-5), 16) << 8) | ((aop >> 24) & 0xFF),
           aop & 0xFFFFFF,
           _tc(e.omgdot / PI, p(-43), 24),
           ((iode & 0xFF) << 16) | (_tc(e.idot / PI, p(-43), 14) << 2)]
    return [sf1, sf2, sf3]


def skytraq_set_ephemeris(e: Ephemeris) -> bytes:
    """0x41 body: ID, SV id (u16), then per subframe a reserved 0 and the 24
    data bits of words 2-10 (big-endian), 87 bytes."""
    body = bytearray((SKYTRAQ_SET_GPS_EPHEMERIS,)) + struct.pack(">H", e.prn)
    for words in subframes(e):
        body.append(0)
        for word in words[1:]:
            body += (word & 0xFFFFFF).to_bytes(3, "big")
    assert len(body) == EPHEMERIS_BODY_LENGTH
    return bytes(body)


def decode_skytraq_ephemeris(body: bytes) -> dict:
    """Inverse of the subframe packing for the fields a test can check."""
    if len(body) != EPHEMERIS_BODY_LENGTH or body[0] != SKYTRAQ_SET_GPS_EPHEMERIS:
        raise ValueError("not a 0x41 Set GPS Ephemeris body")
    words = []
    for sf in range(3):
        base = 3 + sf * 28 + 1
        words.append([0] + [int.from_bytes(body[base + 3 * k:base + 3 * k + 3], "big") for k in range(9)])

    def signed(value, bits):
        return value - (1 << bits) if value & (1 << (bits - 1)) else value
    sf1, sf2, sf3 = words
    sqrta = ((sf2[7] & 0xFF) << 24) | sf2[8]
    ecc = ((sf2[5] & 0xFF) << 24) | sf2[6]
    m0 = signed(((sf2[3] & 0xFF) << 24) | sf2[4], 32)
    return {
        "prn": struct.unpack(">H", body[1:3])[0],
        "week": (sf1[2] >> 14) & 0x3FF,
        "iodc": (((sf1[2] & 3) << 8) | (sf1[7] >> 16)),
        "toc": (sf1[7] & 0xFFFF) * 16,
        "af0": signed(sf1[9] >> 2, 22) * 2.0 ** -31,
        "iode": sf2[2] >> 16,
        "sqrta": sqrta * 2.0 ** -19,
        "ecc": ecc * 2.0 ** -33,
        "m0": m0 * 2.0 ** -31 * PI,
        "toe": (sf2[9] >> 8) * 16,
        "omg0": signed(((sf3[2] & 0xFF) << 24) | sf3[3], 32) * 2.0 ** -31 * PI,
        "idot": signed((sf3[9] >> 2) & 0x3FFF, 14) * 2.0 ** -43 * PI,
        "subframe_ids": [(w[1] >> 2) & 7 for w in words],
    }


# ---------------------------------------------------------- blob / frames
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_blob(bodies) -> bytes:
    blob = b"".join(struct.pack(">H", len(b)) + bytes(b) for b in bodies)
    if not blob:
        raise ValueError("no assistance records")
    if len(blob) > MAX_BLOB:
        raise ValueError(f"assistance data is {len(blob)} bytes; the limit is {MAX_BLOB}")
    return blob


def command_frame(opcode: int, data: bytes = b"") -> bytes:
    data = bytes(data).ljust(6, b"\0")[:6]
    return bytes((0x05,)) + data + struct.pack(">H", opcode) + b"\x0A"


def assist_frames(blob: bytes, transfer_id: int = 1) -> list:
    """BEGIN, DATA..., COMMIT frames for one upload."""
    frames = [command_frame(CMD_BEGIN, struct.pack(">HHBB", len(blob), crc16(blob),
                                                   FORMAT_SKYTRAQ, transfer_id & 0xFF))]
    for offset in range(0, len(blob), CHUNK):
        frames.append(command_frame(CMD_DATA, struct.pack(">H", offset) + blob[offset:offset + CHUNK]))
    frames.append(command_frame(CMD_COMMIT, bytes((transfer_id & 0xFF,))))
    return frames


def status_frame() -> bytes:
    return command_frame(CMD_STATUS)


def ephemeris_upload(nav: NavFile, when: dt.datetime | None = None) -> tuple[bytes, list]:
    """(blob, PRNs) for the ephemerides valid at `when` (GPS time; default now)."""
    selected = select_for(nav, when or gps_now())
    prns = sorted(selected)
    return build_blob(skytraq_set_ephemeris(selected[p]) for p in prns), prns


def send_frames(write, frames, pace_s: float = 0.004) -> None:
    """Write frames with a small gap (USB/ser2net); `write(bytes)`."""
    for frame in frames:
        write(frame)
        if pace_s:
            time.sleep(pace_s)


# ------------------------------------------------------------------- CLI
def _open_link(target: str):
    """'host:port' (ser2net) or a serial port name -> (write, close)."""
    if re.fullmatch(r"[\w.\-]+:\d+", target):
        import socket
        host, port = target.rsplit(":", 1)
        sock = socket.create_connection((host, int(port)), timeout=5)
        return sock.sendall, sock.close
    import serial  # pyserial
    port = serial.Serial(target, 115200, timeout=1)
    return port.write, port.close


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("rinex", help="RINEX navigation file (v2 .yyn or v3 mixed)")
    parser.add_argument("--target", help="serial port or host:port of Witness USB; omit for a dry run")
    parser.add_argument("--shift-to-now", action="store_true",
                        help="move the ephemerides to the current hour (matches a gps-sdr-sim scenario)")
    parser.add_argument("--time", help="GPS time YYYY-mm-ddTHH:MM:SS to select ephemerides for")
    args = parser.parse_args(argv)
    nav = parse_rinex_nav(args.rinex)
    when = dt.datetime.fromisoformat(args.time) if args.time else gps_now()
    if args.shift_to_now:
        nav, _ = shift_to(nav, when)
    blob, prns = ephemeris_upload(nav, when)
    frames = assist_frames(blob)
    print(f"{len(prns)} ephemerides (PRN {', '.join(map(str, prns))}), {len(blob)} bytes, {len(frames)} frames")
    if not args.target:
        return 0
    write, close = _open_link(args.target)
    try:
        send_frames(write, frames)
        write(status_frame())
    finally:
        close()
    print("sent; Witness logs 'GPS assist done: N/N acknowledged' when the receiver has them")
    return 0


if __name__ == "__main__":
    sys.exit(main())
