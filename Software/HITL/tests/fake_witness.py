"""Local fake Witness for end-to-end tests (127.0.0.1 only).

Behaves like the Witness USB stream behind ser2net: echoes every received
byte, accepts HITL ENTER only in Pad idle, consumes 0x0C frames, runs the
reference flight-state logic on the decoded values, auto-exits HITL after
1000 ms without a 0x0C frame, and emits 0x01 (10 Hz), 0x02 and 0xFF (1 Hz)
frames interleaved with ESP-IDF style log text, NMEA lines and junk bytes.

Telemetry encoding here is written independently of hitl_protocol's decoder
so the tests cross-check the two. The fake uses the 0x0C sim time as its state
machine clock so the tests can run faster than real time.

Attitude (optional, on by default): after ``init_after_s`` of HITL input on
the pad the fake aligns its quaternion to the measured specific force by the
shortest arc (body +X up -> world +Z, like Witness) and adds an arbitrary
heading ``heading_offset_deg``; afterwards it integrates the received gyro,
q <- q * exp(omega dt), Hamilton, BODY -> ENU WORLD. The quaternion is
reported (x, y, z, w float32) in 0x02 at 1 Hz and in 0xE0 at 10 Hz once the
0x00E0 debug-start command has been received. Before initialisation the
quaternion fields are all zero, as on Witness.
"""

from __future__ import annotations

import math
import os
import socket
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import sim_model  # noqa: E402

STATE_NAMES = ("pad idle", "boost", "coast", "descent", "landed")
ENTER = b"\x05\x00\x00\x00\x00\x00\x00\x48\x49\x0a"
EXIT = b"\x05\x00\x00\x00\x00\x00\x00\x48\x58\x0a"


def be_i24(v: int) -> bytes:
    return (int(v) & 0xFFFFFF).to_bytes(3, "big")


def frame_status(b0: int, state: int, gps: bool, hitl: bool) -> bytes:
    return bytes((b0 & 0xFF, (state & 7) | (0x08 if gps else 0)
                  | (0x10 if hitl else 0)))


def make_heartbeat(status: bytes, uptime: int) -> bytes:
    return b"\xff" + status + struct.pack(">I", uptime) + b"\x0a"


def make_sensors(status: bytes, uptime: int, acc_g, gyro_dps) -> bytes:
    body = b"".join(be_i24(round(a / 0.00004)) for a in acc_g)
    body += b"".join(struct.pack(">h", max(-32768, min(32767,
                                                       round(g / 0.07))))
                     for g in gyro_dps)
    return b"\x01" + status + struct.pack(">I", uptime) + body + b"\x0a"


def make_state(status: bytes, uptime: int, lat, lon, gps_alt, baro_alt,
               batt, sats) -> bytes:
    body = bytes(40)  # quaternion, position, velocity (zero)
    body += be_i24(round(lat * 1e4)) + be_i24(round(lon * 1e4))
    body += struct.pack(">ffHB", gps_alt, baro_alt, int(batt), int(sats))
    out = b"\x02" + status + struct.pack(">I", uptime) + body + b"\x0a"
    assert len(out) == 65
    return out


def make_state_q(status: bytes, uptime: int, quat, lat, lon, gps_alt,
                 baro_alt, batt, sats) -> bytes:
    body = struct.pack(">ffff", *quat) + bytes(24)  # position, velocity 0
    body += be_i24(round(lat * 1e4)) + be_i24(round(lon * 1e4))
    body += struct.pack(">ffHB", gps_alt, baro_alt, int(batt), int(sats))
    out = b"\x02" + status + struct.pack(">I", uptime) + body + b"\x0a"
    assert len(out) == 65
    return out


def make_debug(status: bytes, uptime: int, quat, last, baro_alt) -> bytes:
    """0xE0 Witness debug packet, 103 bytes (GROUND_STATION_PROTOCOL.md)."""
    i16 = (lambda v: struct.pack(">h", max(-32768, min(32767,  # noqa: E731
                                                       round(v)))))
    out = b"\xe0" + status + struct.pack(">I", uptime)
    out += struct.pack(">bH", 30, int(last["batt"]))
    out += b"".join(be_i24(round(a / 0.00004)) for a in last["low_g"])
    out += struct.pack(">ffff", *quat)                 # offset 19
    out += bytes(24)                                   # position, velocity
    out += b"".join(i16(a / 0.000488) for a in last["low_g"])     # 59
    out += i16(0)                                      # IMU temp
    out += b"".join(i16(a / 0.010417) for a in last.get(
        "high_g", last["low_g"]))                      # 67
    out += b"".join(i16(g / 0.07) for g in last["gyro_dps"])      # 73
    out += int(last["pressure_pa"]).to_bytes(3, "big")             # 79
    out += i16(2000) + struct.pack(">f", baro_alt)     # 82, 84
    out += be_i24(round(last["lat"] * 1e4)) + be_i24(round(last["lon"] * 1e4))
    out += struct.pack(">If", 0, last["gps_alt"]) + b"\x0a"
    assert len(out) == 103, len(out)
    return out


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _align(acc, heading_deg):
    """Shortest arc taking the measured up direction (body) to world +Z."""
    n = math.sqrt(sum(a * a for a in acc))
    u = [a / n for a in acc]
    # cross(u, z) = (u_y, -u_x, 0), w = 1 + u_z
    q = (u[1], -u[0], 0.0, 1.0 + u[2])
    m = math.sqrt(sum(c * c for c in q))
    q = tuple(c / m for c in q)
    h = math.radians(heading_deg) / 2
    return _qmul((0.0, 0.0, math.sin(h), math.cos(h)), q)


def _integrate(q, gyro_dps, dt):
    w = [math.radians(g) for g in gyro_dps]
    ang = math.sqrt(sum(c * c for c in w)) * dt
    if ang < 1e-12:
        return q
    k = math.sin(ang / 2) / (ang / dt)
    dq = (w[0] * k, w[1] * k, w[2] * k, math.cos(ang / 2))
    q = _qmul(q, dq)
    m = math.sqrt(sum(c * c for c in q))
    return tuple(c / m for c in q)


def parse_hitl(f: bytes) -> dict:
    """Independent 0x0C parser (frame offsets = payload offsets + 1)."""
    h = lambda o: struct.unpack_from(">h", f, 1 + o)[0]  # noqa: E731
    return {
        "sim_time_ms": struct.unpack_from(">I", f, 3)[0],
        "low_g": [h(o) * 0.000488 for o in (6, 8, 10)],
        "high_g": [h(o) * 0.010417 for o in (12, 14, 16)],
        "gyro_dps": [h(o) * 0.070 for o in (18, 20, 22)],
        "pressure_pa": int.from_bytes(f[27:30], "big"),
        "lat": int.from_bytes(f[32:35], "big", signed=True) / 1e4,
        "lon": int.from_bytes(f[35:38], "big", signed=True) / 1e4,
        "gps_alt": struct.unpack_from(">f", f, 38)[0],
        "sats": f[42],
        "fix": f[43],
        "batt": struct.unpack_from(">H", f, 44)[0],
    }


class FakeWitness:
    def __init__(self, set_hitl_bit: bool = True, start_state: int = 0,
                 attitude: bool = True, heading_offset_deg: float = 37.0,
                 init_after_s: float = 2.0):
        self.set_hitl_bit = set_hitl_bit
        self.start_state = start_state
        self.attitude = attitude
        self.heading_offset_deg = heading_offset_deg
        self.init_after_s = init_after_s
        self.debug_enabled = False
        self.quat = (0.0, 0.0, 0.0, 0.0)  # latest reported attitude
        self.debug_packets = 0
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(1)
        self.port = self.srv.getsockname()[1]
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self.commands = []       # every command value received
        self.imu_frames = 0      # 0x0E frames received
        self.hitl_frames = 0
        self.transitions = []    # (sim s, from, to)
        self.hitl_entries = 0
        # Uptime starts so that its bytes contain 0x0A (embedded EOF bytes).
        self._uptime0 = 0x000A0A00

    def start(self):
        self._thread.start()
        return self

    def close(self):
        self._stop.set()
        try:
            self.srv.close()
        except OSError:
            pass
        self._thread.join(timeout=3)

    # ------------------------------------------------------------------
    def _serve(self):
        self.srv.settimeout(0.1)
        while not self._stop.is_set():
            try:
                conn, _ = self.srv.accept()
            except (socket.timeout, OSError):
                continue
            try:
                self._client(conn)
            finally:
                conn.close()

    def _client(self, conn):
        conn.settimeout(0.005)
        lock = threading.Lock()
        t_start = time.monotonic()
        st = {"state": self.start_state, "hitl": False, "last_hitl": 0.0,
              "fsm": sim_model.ReferenceStateMachine(), "last": None,
              "hb": 0, "q": None, "q_first_ms": None, "q_last_ms": None,
              "acc_sum": [0.0, 0.0, 0.0], "acc_n": 0}
        self.debug_enabled = False
        self.quat = (0.0, 0.0, 0.0, 0.0)
        st["fsm"].state = self.start_state
        st["fsm"]._observed_state = self.start_state

        def uptime():
            return (self._uptime0 + int((time.monotonic() - t_start) * 1000)) \
                & 0xFFFFFFFF

        def send(data):
            with lock:
                try:
                    conn.sendall(data)
                except OSError:
                    pass

        def log(msg):
            send(f"\x1b[0;32mI ({uptime() & 0xFFFFFF}) {msg}\x1b[0m\r\n"
                 .encode())

        buf = bytearray()
        next_sensor = next_slow = time.monotonic()
        send(b"\x00\x13junk")  # leading garbage before any frame
        log("main: fake witness ready")
        while not self._stop.is_set():
            try:
                data = conn.recv(65536)
                if not data:
                    return
                send(data)  # USB echo, byte for byte
                buf += data
            except socket.timeout:
                pass
            except OSError:
                return
            # Parse host input.
            while buf:
                if buf[0] == 0x05 and len(buf) >= 10:
                    if buf[9] == 0x0A:
                        cmd = struct.unpack_from(">H", buf, 7)[0]
                        self.commands.append(cmd)
                        del buf[:10]
                        self._command(cmd, st, log)
                        continue
                elif buf[0] == 0x0C and len(buf) >= 47:
                    if buf[46] == 0x0A:
                        self._hitl(parse_hitl(bytes(buf[:47])), st, log)
                        del buf[:47]
                        continue
                elif buf[0] == 0x0E and len(buf) >= 150:
                    if buf[149] == 0x0A:
                        self.imu_frames += 1
                        del buf[:150]
                        continue
                elif buf[0] in (0x05, 0x0C, 0x0E):
                    break  # incomplete
                del buf[0]
            now = time.monotonic()
            if st["hitl"] and now - st["last_hitl"] > 1.0:
                st["hitl"] = False
                log("hitl: no input for 1000 ms, leaving HITL")
            status = frame_status(0x6E | (st["hb"] & 1), st["state"],
                                  bool(st["last"] and st["last"]["fix"] >= 1
                                       and st["hitl"]),
                                  st["hitl"] and self.set_hitl_bit)
            last = st["last"] or {"low_g": [1, 0, 0], "gyro_dps": [0, 0, 0],
                                  "pressure_pa": 85000, "lat": 0, "lon": 0,
                                  "gps_alt": 0.0, "sats": 0, "batt": 7400}
            quat = st["q"] or (0.0, 0.0, 0.0, 0.0)
            if now >= next_sensor:
                next_sensor += 0.1
                send(make_sensors(status, uptime(), last["low_g"],
                                  last["gyro_dps"]))
                if self.debug_enabled:
                    self.debug_packets += 1
                    send(make_debug(status, uptime(), quat, last,
                                    sim_model.hp.pressure_altitude_m(
                                        last["pressure_pa"])))
            if now >= next_slow:
                next_slow += 1.0
                st["hb"] += 1
                baro = sim_model.hp.pressure_altitude_m(last["pressure_pa"])
                send(make_heartbeat(status, uptime()))
                send(b"$GPGGA,000000.00,,,,,0,00,99.99,,,,,,*66\r\n")
                send(make_state_q(status, uptime(), quat, last["lat"],
                                  last["lon"], last["gps_alt"], baro,
                                  last["batt"], last["sats"]))
                send(b"\xff\x01\x02")  # truncated junk candidate

    def _command(self, cmd, st, log):
        if cmd == 0x4849:
            if st["state"] == 0:
                st["hitl"] = True
                st["last_hitl"] = time.monotonic()
                self.hitl_entries += 1
                log("hitl: HITL mode entered")
            else:
                log("hitl: ENTER rejected, not in pad idle")
        elif cmd == 0x4858:
            if st["hitl"]:
                log("hitl: HITL mode exited")
            st["hitl"] = False
        elif cmd == 0x00E0:
            self.debug_enabled = True
            log("sensors: Witness debug packets enabled at 10 Hz")
        else:
            log(f"cmd: unexpected command 0x{cmd:04X}")

    def _hitl(self, d, st, log):
        if not st["hitl"]:
            return
        self.hitl_frames += 1
        st["last_hitl"] = time.monotonic()
        st["last"] = d
        if self.attitude:
            self._attitude(d, st)
        fsm = st["fsm"]
        before = fsm.state
        fsm.update((d["sim_time_ms"] + 10) * 1000, d["high_g"][0],
                   d["low_g"][0], d["low_g"], d["gyro_dps"],
                   sim_model.hp.pressure_altitude_m(d["pressure_pa"]))
        if fsm.state != before:
            st["state"] = fsm.state
            self.transitions.append((d["sim_time_ms"] / 1000, before,
                                     fsm.state))
            log(f"flight_state: {STATE_NAMES[before]} -> "
                f"{STATE_NAMES[fsm.state]}")


def _attitude_update(self, d, st):
    ms = d["sim_time_ms"]
    if st["q"] is None:
        if st["state"] != 0:
            return
        if st["q_first_ms"] is None:
            st["q_first_ms"] = ms
        for i in range(3):
            st["acc_sum"][i] += d["low_g"][i]
        st["acc_n"] += 1
        if (ms - st["q_first_ms"]) / 1000 >= self.init_after_s:
            st["q"] = _align([a / st["acc_n"] for a in st["acc_sum"]],
                             self.heading_offset_deg)
            st["q_last_ms"] = ms
    else:
        dt = (ms - st["q_last_ms"]) / 1000
        st["q_last_ms"] = ms
        if 0 < dt < 1.0:
            st["q"] = _integrate(st["q"], d["gyro_dps"], dt)
    if st["q"] is not None:
        self.quat = st["q"]


FakeWitness._attitude = _attitude_update


if __name__ == "__main__":
    fw = FakeWitness().start()
    print(f"fake Witness listening on 127.0.0.1:{fw.port} (Ctrl+C to stop)")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        fw.close()
