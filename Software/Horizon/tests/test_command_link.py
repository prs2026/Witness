"""CommandLink against a simulated Witness on a lossy link (no hardware)."""
import os
import random
import struct
import sys
import threading
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import command_link as cl  # noqa: E402
import gps_assist as ga  # noqa: E402
import witness_protocol as wp  # noqa: E402


class FakeWitness:
    """Mirrors main/gps_assist.cpp Assembler + usb_serial_echo acks."""

    def __init__(self, link, loss=0.0, seed=1):
        self.link = link
        self.loss = loss
        self.rng = random.Random(seed)
        self.outage = 0            # drop the next N frames both ways
        self.writes = 0
        self.data_writes = 0
        self.reset()
        self.committed = None

    def reset(self):
        self.state, self.tid, self.length, self.crc, self.buf, self.have = 0, 0, 0, 0, b"", set()

    def _lost(self):
        if self.outage > 0:
            self.outage -= 1
            return True
        return self.rng.random() < self.loss

    def _reply(self, packet_id, payload):
        if self._lost():
            return
        self.link.feed(wp.parse_packet(packet_id, payload))

    def _ack(self, opcode, data, result, detail=0):
        self._reply(0x06, struct.pack(">H", opcode) + data + bytes((result,)) + struct.pack(">H", detail))

    def expected(self):
        return (self.length + ga.CHUNK - 1) // ga.CHUNK

    def write(self, frame):
        self.writes += 1
        if len(frame) != 10 or frame[0] != 0x05:
            return                 # not a canonical command: Witness does not ack it
        if self._lost():
            return
        data, opcode = frame[1:7], (frame[7] << 8) | frame[8]
        if opcode == ga.CMD_BEGIN:
            length, crc, fmt, tid = struct.unpack(">HHBB", data)
            same = self.state == 1 and (tid, length, crc) == (self.tid, self.length, self.crc)
            if not same:
                self.state, self.tid, self.length, self.crc = 1, tid, length, crc
                self.buf, self.have = bytearray(length), set()
            self._ack(opcode, data, 0)
        elif opcode == ga.CMD_DATA:
            self.data_writes += 1
            if self.state == 1:
                off = (data[0] << 8) | data[1]
                n = min(ga.CHUNK, self.length - off)
                self.buf[off:off + n] = data[2:2 + n]
                self.have.add(off // ga.CHUNK)
        elif opcode == ga.CMD_STATUS:
            exp = self.expected()
            start = next((c for c in range(exp) if c not in self.have), exp)
            bits = bytearray(64)
            for i in range(512):
                c = start + i
                if c >= exp or c in self.have:
                    bits[i // 8] |= 1 << (i % 8)
            self._reply(0x07, bytes((self.tid, self.state)) + struct.pack(
                ">HHHH", self.length, exp, len(self.have), start) + bytes(bits))
        elif opcode == ga.CMD_COMMIT:
            if self.state == 2 and data[0] == self.tid:
                self._ack(opcode, data, 0, 1)
            elif self.state != 1 or data[0] != self.tid:
                self._ack(opcode, data, 1)
            elif len(self.have) < self.expected():
                self._ack(opcode, data, 5, len(self.have))
            else:
                self.state = 2
                self.committed = bytes(self.buf)
                self._ack(opcode, data, 0)
        else:
            self._ack(opcode, data, 0)


def blob(n=600, seed=3):
    r = random.Random(seed)
    return bytes(r.randrange(256) for _ in range(n))


class Reliable(unittest.TestCase):
    def test_parse_roundtrip(self):
        link = cl.CommandLink()
        w = FakeWitness(link)
        frame = wp.build_pad_heading_packet("+Y", 12.5)
        ack = link.send_reliable(w.write, frame, timeout=0.05)
        self.assertIsNotNone(ack)
        self.assertEqual(ack["result"], 0)
        self.assertEqual(ack["command_value"], 0x4844)

    def test_retries_through_loss(self):
        link = cl.CommandLink()
        w = FakeWitness(link)
        w.outage = 3       # first command and the first ack attempt are lost
        ack = link.send_reliable(w.write, ga.command_frame(0x1234, b"\x01"), timeout=0.02, attempts=5)
        self.assertIsNotNone(ack)
        self.assertGreater(w.writes, 1)

    def test_gives_up(self):
        link = cl.CommandLink()
        w = FakeWitness(link, loss=1.0)
        self.assertIsNone(link.send_reliable(w.write, ga.command_frame(0x1234), timeout=0.01, attempts=3))
        self.assertEqual(w.writes, 3)

    def test_stale_ack_for_other_data_ignored(self):
        link = cl.CommandLink()
        link.feed({"packet": "command_ack", "command_value": 0x1234, "command_data": b"\x09" + bytes(5),
                   "result": 0})
        w = FakeWitness(link, loss=1.0)
        self.assertIsNone(link.send_reliable(w.write, ga.command_frame(0x1234, b"\x01"), timeout=0.01,
                                             attempts=1))


class Upload(unittest.TestCase):
    def run_upload(self, w, link, data, **kw):
        kw.setdefault("pace_s", 0)
        kw.setdefault("timeout", 0.02)
        kw.setdefault("resume_deadline_s", 30)
        return link.upload_assist(w.write, data, transfer_id=4, **kw)

    def test_clean(self):
        link = cl.CommandLink()
        w = FakeWitness(link)
        data = blob()
        ok, msg = self.run_upload(w, link, data)
        self.assertTrue(ok, msg)
        self.assertEqual(w.committed, data)
        self.assertEqual(w.data_writes, 150)

    def test_lossy_resends_only_missing(self):
        link = cl.CommandLink()
        w = FakeWitness(link, loss=0.2, seed=7)
        data = blob(2768)          # ~ a full ephemeris set: 692 chunks, beyond one 512-bit window
        ok, msg = self.run_upload(w, link, data)
        self.assertTrue(ok, msg)
        self.assertEqual(w.committed, data)
        self.assertLess(w.data_writes, 692 * 1.6)

    def test_outage_then_resume(self):
        link = cl.CommandLink()
        w = FakeWitness(link)
        data = blob()
        orig = w.write
        count = [0]

        def write(frame):
            count[0] += 1
            if count[0] == 60:
                w.outage = 40      # link drops mid-stream, then comes back
            orig(frame)
        notes = []
        ok, msg = link.upload_assist(write, data, transfer_id=4, pace_s=0, timeout=0.02,
                                     resume_deadline_s=30, progress=lambda r, e, n: notes.append(n))
        self.assertTrue(ok, msg)
        self.assertEqual(w.committed, data)
        self.assertLess(w.data_writes, 150 + 60)    # resumed, not restarted

    def test_witness_reboot_restarts(self):
        link = cl.CommandLink()
        w = FakeWitness(link)
        data = blob()
        orig = w.write
        count = [0]

        def write(frame):
            count[0] += 1
            if count[0] == 80:
                w.reset()
            orig(frame)
        ok, msg = link.upload_assist(write, data, transfer_id=4, pace_s=0, timeout=0.02, resume_deadline_s=30)
        self.assertTrue(ok, msg)
        self.assertEqual(w.committed, data)

    def test_cancel(self):
        link = cl.CommandLink()
        w = FakeWitness(link, loss=1.0)
        ev = threading.Event()
        ev.set()
        ok, msg = self.run_upload(w, link, blob(), cancel=ev)
        self.assertFalse(ok)


if __name__ == "__main__":
    unittest.main()
