"""Local fake Witness for end-to-end tests (127.0.0.1 only).

Behaves like the Witness USB stream behind ser2net: echoes every received
byte, accepts HITL ENTER only in Pad idle, consumes 0x0C frames, runs the
reference flight-state logic on the decoded values, auto-exits HITL after
1000 ms without a 0x0C frame, and emits 0x01 (10 Hz), 0x02 and 0xFF (1 Hz)
frames interleaved with ESP-IDF style log text, NMEA lines and junk bytes.

Telemetry encoding here is written independently of hitl_protocol's decoder
so the tests cross-check the two. The fake uses the 0x0C sim time as its state
machine clock so the tests can run faster than real time.
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
    def __init__(self, set_hitl_bit: bool = True, start_state: int = 0):
        self.set_hitl_bit = set_hitl_bit
        self.start_state = start_state
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(1)
        self.port = self.srv.getsockname()[1]
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self.commands = []       # every command value received
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
              "hb": 0}
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
                elif buf[0] in (0x05, 0x0C):
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
            if now >= next_sensor:
                next_sensor += 0.1
                send(make_sensors(status, uptime(), last["low_g"],
                                  last["gyro_dps"]))
            if now >= next_slow:
                next_slow += 1.0
                st["hb"] += 1
                baro = sim_model.hp.pressure_altitude_m(last["pressure_pa"])
                send(make_heartbeat(status, uptime()))
                send(b"$GPGGA,000000.00,,,,,0,00,99.99,,,,,,*66\r\n")
                send(make_state(status, uptime(), last["lat"], last["lon"],
                                last["gps_alt"], baro, last["batt"],
                                last["sats"]))
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
        else:
            log(f"cmd: unexpected command 0x{cmd:04X}")

    def _hitl(self, d, st, log):
        if not st["hitl"]:
            return
        self.hitl_frames += 1
        st["last_hitl"] = time.monotonic()
        st["last"] = d
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


if __name__ == "__main__":
    fw = FakeWitness().start()
    print(f"fake Witness listening on 127.0.0.1:{fw.port} (Ctrl+C to stop)")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        fw.close()
