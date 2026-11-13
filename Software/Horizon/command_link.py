"""Acknowledged commands and resumable GPS-assist uploads (comms.h 0x06 / 0x07).

Witness answers every canonical 0x05 command with a 0x06 acknowledgement on
the link it arrived on (USB, or LoRa through Aperture), echoing the opcode
and the six data bytes, and answers a GPS-assist STATUS with a 0x07 transfer
status carrying a bitmap of the chunks it holds. This module matches those
replies to what was sent:

- send_reliable(): write a command, wait for its ack, resend until it comes
  (a lost uplink or a lost ack both just cause a resend; every command
  Witness defines is safe to repeat).
- upload_assist(): BEGIN, stream all DATA chunks, then STATUS -> resend only
  the missing chunks -> STATUS ... until Witness has them all, then COMMIT.
  If the link goes quiet the upload waits and keeps asking (up to
  resume_deadline_s); BEGIN with the same id/length/CRC resumes the transfer
  on Witness, so nothing already delivered is sent again.

No GUI and no transport: callers pass write(bytes) and feed decoded packets
(witness_protocol.parse_packet dicts) in with feed().
"""
from __future__ import annotations

import random
import struct
import threading
import time

import gps_assist as ga

ACK_OK = 0
ACK_INCOMPLETE = 5


def missing_chunks(status: dict) -> list:
    """Chunk numbers a 0x07 status reports missing (bit clear) in its window."""
    start = status.get("window_start", 0)
    bitmap = bytes.fromhex(status.get("bitmap", ""))
    return [start + i for i in range(len(bitmap) * 8) if not (bitmap[i // 8] >> (i % 8)) & 1]


class CommandLink:
    def __init__(self):
        self._cv = threading.Condition()
        self._acks: list[dict] = []
        self._status: list[dict] = []

    # ------------------------------------------------------------- input
    def feed(self, packet: dict) -> None:
        """Hand every decoded packet in; acks and transfer status are kept."""
        kind = packet.get("packet")
        if kind not in ("command_ack", "transfer_status"):
            return
        with self._cv:
            (self._acks if kind == "command_ack" else self._status).append(packet)
            del self._acks[:-64]
            del self._status[:-16]
            self._cv.notify_all()

    def _wait(self, pred, timeout: float):
        deadline = time.monotonic() + timeout
        with self._cv:
            while True:
                for item in self._acks + self._status:
                    if pred(item):
                        return item
                left = deadline - time.monotonic()
                if left <= 0:
                    return None
                self._cv.wait(left)

    def _clear(self, pred):
        with self._cv:
            self._acks = [a for a in self._acks if not pred(a)]
            self._status = [s for s in self._status if not pred(s)]

    # ------------------------------------------------------ single command
    @staticmethod
    def is_acked_command(frame: bytes) -> bool:
        """Canonical 0x05 commands Witness acknowledges with 0x06: all but
        assist DATA (no reply) and STATUS (answered with 0x07 instead)."""
        return (len(frame) == 10 and frame[0] == 0x05 and frame[9] == 0x0A
                and ((frame[7] << 8) | frame[8]) not in (ga.CMD_DATA, ga.CMD_STATUS))

    @staticmethod
    def _ack_match(frame: bytes):
        opcode = (frame[7] << 8) | frame[8]
        data = bytes(frame[1:7]).hex(" ")
        return lambda p: (p.get("packet") == "command_ack" and p.get("command_value") == opcode
                          and p.get("command_data") == data)

    def forget(self, frame: bytes) -> None:
        """Drop stale acks for this command (call before writing it yourself)."""
        self._clear(self._ack_match(frame))

    def send_reliable(self, write, frame: bytes, timeout: float = 1.0, attempts: int = 5,
                      cancel: threading.Event | None = None, first_sent: bool = False):
        """Write a 10-byte 0x05 command until Witness acknowledges it.
        Returns the ack dict (check ack['result'] == 0 for success) or None.
        first_sent: the caller already wrote it once (after forget())."""
        if len(frame) != 10 or frame[0] != 0x05:
            raise ValueError("send_reliable takes a canonical 10-byte 0x05 command frame")
        match = self._ack_match(frame)
        if not first_sent:
            self._clear(match)
        for attempt in range(attempts):
            if cancel is not None and cancel.is_set():
                return None
            if attempt or not first_sent:
                write(frame)
            ack = self._wait(match, timeout)
            if ack is not None:
                self._clear(match)
                return ack
        return None

    def request_status(self, write, timeout: float = 1.5, attempts: int = 3,
                       cancel: threading.Event | None = None, transfer_id: int | None = None):
        """STATUS (window from the first missing chunk) -> 0x07 dict or None."""
        frame = ga.command_frame(ga.CMD_STATUS, bytes((0xFF, 0xFF)))

        def match(p):
            return p.get("packet") == "transfer_status" and (
                transfer_id is None or p.get("transfer_id") == transfer_id)

        self._clear(match)
        for _ in range(attempts):
            if cancel is not None and cancel.is_set():
                return None
            write(frame)
            st = self._wait(match, timeout)
            if st is not None:
                self._clear(match)
                return st
        return None

    # ---------------------------------------------------- resumable upload
    def upload_assist(self, write, blob: bytes, transfer_id: int = 1, pace_s: float = 0.06,
                      timeout: float = 1.5, resume_deadline_s: float = 600.0,
                      progress=None, cancel: threading.Event | None = None):
        """Deliver a GPS-assist blob reliably. Returns (ok, message).
        progress(received, expected, note) is called as it goes."""
        transfer_id &= 0xFF
        crc = ga.crc16(blob)
        begin = ga.command_frame(ga.CMD_BEGIN, struct.pack(">HHBB", len(blob), crc, ga.FORMAT_SKYTRAQ,
                                                           transfer_id))
        chunks = {}
        for offset in range(0, len(blob), ga.CHUNK):
            chunks[offset // ga.CHUNK] = ga.command_frame(
                ga.CMD_DATA, struct.pack(">H", offset) + blob[offset:offset + ga.CHUNK])
        commit = ga.command_frame(ga.CMD_COMMIT, bytes((transfer_id,)))
        expected = len(chunks)
        report = progress or (lambda *_a: None)
        deadline = time.monotonic() + resume_deadline_s

        def cancelled():
            return cancel is not None and cancel.is_set()

        def begin_until_acked(note):
            while not cancelled() and time.monotonic() < deadline:
                ack = self.send_reliable(write, begin, timeout=timeout, attempts=3, cancel=cancel)
                if ack is not None:
                    return ack.get("result") == ACK_OK
                report(None, expected, f"{note}: no answer, link down? retrying")
                time.sleep(1.0)
            return False

        if not begin_until_acked("BEGIN"):
            return False, "Witness did not acknowledge BEGIN (link down, or not in a state to accept it)"
        to_send = list(range(expected))
        rounds = 0
        rng = random.Random(transfer_id)
        while not cancelled():
            # Over the radio Witness cannot hear while it transmits telemetry
            # (~170 ms per packet), so a short resend round at a fixed rhythm
            # can keep landing in the same deaf window: send the last few
            # missing chunks twice and leave a random gap before STATUS.
            repeat = 2 if pace_s and len(to_send) <= 8 else 1
            for chunk in to_send:
                for _ in range(repeat):
                    if cancelled():
                        return False, "cancelled"
                    write(chunks[chunk])
                    if pace_s:
                        time.sleep(pace_s)
            if pace_s:
                time.sleep(rng.uniform(0.0, 0.4))
            rounds += 1
            st = self.request_status(write, timeout=timeout, cancel=cancel, transfer_id=transfer_id)
            if st is None:
                # Link lost: keep asking; when it answers again, BEGIN (same
                # transfer) resumes and only the missing chunks go out.
                if time.monotonic() > deadline:
                    return False, f"link lost; gave up after {resume_deadline_s:.0f} s"
                report(None, expected, "no status reply: waiting for the link")
                time.sleep(1.0)
                if not begin_until_acked("resume"):
                    return False, "link did not come back in time"
                to_send = []
                continue
            received = st.get("chunks_received", 0)
            report(received, expected, f"round {rounds}")
            if st.get("transfer_id") != transfer_id or st.get("state") != "receiving":
                if st.get("state") == "complete" and st.get("transfer_id") == transfer_id:
                    return True, "already complete"
                # Witness lost the transfer (reboot?): start over.
                if not begin_until_acked("restart"):
                    return False, "could not restart the transfer"
                to_send = list(range(expected))
                continue
            if received >= expected:
                break
            to_send = [c for c in missing_chunks(st) if c < expected]
            if not to_send:
                to_send = [c for c in range(st.get("window_start", 0), expected)]
        if cancelled():
            return False, "cancelled"
        ack = self.send_reliable(write, commit, timeout=timeout, attempts=5, cancel=cancel)
        if ack is None:
            return False, "COMMIT not acknowledged"
        if ack.get("result") == ACK_OK:
            report(expected, expected, "committed")
            return True, f"{expected} chunks delivered in {rounds} round(s); Witness passed them to the GPS"
        return False, f"COMMIT refused: {ack.get('result_name', ack.get('result'))} (detail {ack.get('detail')})"
