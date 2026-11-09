"""Pick NMEA 0183 sentences out of the mixed Witness USB byte stream.

Witness mirrors its GPS UART onto USB, interleaved with binary telemetry
frames and ESP-IDF log text. A UART chunk boundary can put a telemetry frame
in the middle of a sentence, so every candidate is checked against its
``*hh`` XOR checksum; damaged sentences are counted, not logged as valid.
"""

from __future__ import annotations

import re

MAX_SENTENCE_LENGTH = 100  # NMEA 0183 allows 82; leave room for proprietary talkers.
_SENTENCE = re.compile(rb"^\$([A-Z][A-Z0-9]{1,4}),?([\x20-\x7e]*)\*([0-9A-Fa-f]{2})$")


def nmea_checksum(body: bytes) -> int:
    value = 0
    for byte in body:
        value ^= byte
    return value


class NmeaExtractor:
    """Streaming extractor: feed raw bytes, get complete sentences back.

    ``feed`` returns ``[(sentence_text, checksum_ok), ...]`` for every
    ``$...*hh`` line completed by this chunk. Sentences may span any number
    of chunks. Non-printable bytes inside a candidate end it as damaged.
    """

    def __init__(self):
        self._buffer = bytearray()
        self._active = False
        self.valid = 0
        self.bad = 0

    def feed(self, data: bytes) -> list[tuple[str, bool]]:
        out = []
        for byte in data:
            if byte == 0x24:  # '$' always starts a new candidate
                if self._active and len(self._buffer) > 1:
                    self._finish(out, damaged=True)
                self._buffer = bytearray(b"$")
                self._active = True
                continue
            if not self._active:
                continue
            if byte in (0x0D, 0x0A):
                self._finish(out)
                continue
            if byte < 0x20 or byte > 0x7E or len(self._buffer) >= MAX_SENTENCE_LENGTH:
                self._finish(out, damaged=True)
                continue
            self._buffer.append(byte)
        return out

    def _finish(self, out, damaged=False):
        line = bytes(self._buffer)
        self._buffer = bytearray()
        self._active = False
        match = _SENTENCE.match(line)
        if not match:
            if len(line) > 6:  # ignore a bare '$' or a short fragment
                self.bad += 1
            return
        star = line.rindex(b"*")
        ok = not damaged and nmea_checksum(line[1:star]) == int(line[star + 1:], 16)
        if ok:
            self.valid += 1
        else:
            self.bad += 1
        out.append((line.decode("ascii"), ok))


def sentence_type(sentence: str) -> str:
    """'GNGGA' -> 'GGA'; the talker prefix (GP, GN, GL, ...) is dropped."""
    head = sentence[1:].split(",", 1)[0]
    return head[2:] if len(head) == 5 else head


def summarize_gga(sentence: str) -> str | None:
    """Short readout for a valid GGA: fix quality, satellites, altitude."""
    fields = sentence.split("*", 1)[0].split(",")
    if len(fields) < 10 or sentence_type(sentence) != "GGA":
        return None
    quality = fields[6] or "0"
    satellites = fields[7] or "0"
    altitude = fields[9] or "-"
    utc = fields[1][:6]
    utc = f"{utc[0:2]}:{utc[2:4]}:{utc[4:6]}" if len(utc) == 6 else "--:--:--"
    return f"GGA {utc} UTC, fix {quality}, sats {int(satellites)}, alt {altitude} m"
