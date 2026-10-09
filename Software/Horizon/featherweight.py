"""Featherweight GPS text-packet parsing and stream deframing.

The tracker also emits binary packets beginning with ``FWT``.  The user-facing
protocol explicitly says those packets are not intended for host parsing, so
the stream parser ignores everything until the next ``@`` text packet.
"""

from __future__ import annotations

import re
from typing import Any, Callable


FEATHERWEIGHT_BAUD = 115200
MAX_TEXT_PACKET_BYTES = 4096
TEXT_PACKET_HEADER = re.compile(
    rb"^@[ \t]+[A-Z][A-Z0-9_]{1,23}[ \t]+\d+[ \t]+\d{4}"
    rb"[ \t]+\d{1,2}[ \t]+\d{1,2}[ \t]+"
)


class FeatherweightParseError(ValueError):
    """Raised when a Featherweight text packet is incomplete or malformed."""


def crc16_buypass(data: bytes) -> int:
    """Return CRC-16/BUYPASS (poly=0x8005, init=0, refin/refout=false)."""

    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x8005) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _find_token(tokens: list[str], label: str) -> int:
    wanted = label.casefold()
    for index, token in enumerate(tokens):
        if token.casefold() == wanted:
            return index
    raise FeatherweightParseError(f"missing {label!r} field")


def _as_int(value: str, field: str) -> int:
    try:
        return int(value, 10)
    except ValueError as exc:
        raise FeatherweightParseError(f"invalid {field}: {value!r}") from exc


def _as_float(value: str, field: str) -> float:
    try:
        return float(value)
    except ValueError as exc:
        raise FeatherweightParseError(f"invalid {field}: {value!r}") from exc


def _parse_gps_status(payload: list[str], result: dict[str, Any]) -> None:
    alt_index = _find_token(payload, "Alt")
    lat_index = _find_token(payload, "lt")
    lon_index = _find_token(payload, "ln")
    velocity_index = _find_token(payload, "Vel")
    fix_index = _find_token(payload, "Fix")
    satellite_index = _find_token(payload, "#")

    prefix = payload[:alt_index]
    if not prefix:
        raise FeatherweightParseError("GPS_STAT is missing its radio CRC status")

    result["radio_crc_status"] = prefix[0]
    result["unit_type"] = prefix[1] if len(prefix) >= 2 else ""
    result["unit_id"] = " ".join(prefix[2:]) if len(prefix) >= 3 else ""

    try:
        result["altitude_ft"] = _as_float(payload[alt_index + 1], "altitude")
        result["latitude_deg"] = _as_float(payload[lat_index + 1], "latitude")
        result["longitude_deg"] = _as_float(payload[lon_index + 1], "longitude")
        result["horizontal_velocity_fps"] = _as_float(
            payload[velocity_index + 1], "horizontal velocity"
        )
        result["heading_deg"] = _as_float(payload[velocity_index + 2], "heading")
        result["vertical_velocity_fps"] = _as_float(
            payload[velocity_index + 3], "vertical velocity"
        )
        result["fix_type"] = _as_int(payload[fix_index + 1], "fix type")
        result["satellites_total"] = _as_int(
            payload[satellite_index + 1], "satellite count"
        )
        result["satellites_over_24db"] = _as_int(
            payload[satellite_index + 2], "satellites over 24 dB"
        )
        result["satellites_over_32db"] = _as_int(
            payload[satellite_index + 3], "satellites over 32 dB"
        )
        result["satellites_over_40db"] = _as_int(
            payload[satellite_index + 4], "satellites over 40 dB"
        )
    except IndexError as exc:
        raise FeatherweightParseError("GPS_STAT ended in the middle of a field") from exc

    result["satellite_details"] = payload[satellite_index + 5 :]


def parse_featherweight_line(line: str) -> dict[str, Any]:
    """Parse one ``@`` Featherweight text packet into a field dictionary.

    A packet is returned even when its final text CRC is bad.  The caller can
    inspect ``text_crc_valid`` and retain/display the data for diagnostics.
    """

    clean_line = line.strip("\r\n")
    if not clean_line.startswith("@"):
        raise FeatherweightParseError("text packet does not begin with '@'")

    crc_match = re.search(r"\sCRC:\s*([0-9A-Fa-f]{1,4})\s*$", clean_line)
    if crc_match is None:
        raise FeatherweightParseError("text packet is missing its CRC field")

    crc_source = clean_line[: crc_match.start()].rstrip()
    expected_crc = int(crc_match.group(1), 16)
    computed_crc = crc16_buypass(crc_source.encode("ascii", errors="strict"))
    tokens = clean_line.split()
    if len(tokens) < 9 or tokens[0] != "@":
        raise FeatherweightParseError("text packet has an incomplete common header")

    try:
        crc_token_index = tokens.index("CRC:")
    except ValueError as exc:
        raise FeatherweightParseError("text packet has an invalid CRC field") from exc

    packet_type = tokens[1]
    packet_length = _as_int(tokens[2], "packet length")
    year = _as_int(tokens[3], "year")
    month = _as_int(tokens[4], "month")
    day = _as_int(tokens[5], "day")
    packet_time = tokens[6]
    payload = tokens[7:crc_token_index]

    result: dict[str, Any] = {
        "packet_type": packet_type,
        "packet_length": packet_length,
        "year": year,
        "month": month,
        "day": day,
        "time": packet_time,
        "timestamp": f"{year:04d}-{month:02d}-{day:02d} {packet_time}",
        "payload_text": " ".join(payload),
        "text_crc_expected": expected_crc,
        "text_crc_computed": computed_crc,
        "text_crc_valid": computed_crc == expected_crc,
        "raw": clean_line,
    }

    if packet_type.casefold() == "gps_stat":
        _parse_gps_status(payload, result)

    return result


class FeatherweightStreamParser:
    """Extract Featherweight ``@`` text packets from an arbitrary byte stream."""

    def __init__(
        self,
        on_packet: Callable[[dict[str, Any]], None],
        on_error: Callable[[str], None] | None = None,
    ) -> None:
        self.on_packet = on_packet
        self.on_error = on_error or (lambda _message: None)
        self.buffer = bytearray()

    @staticmethod
    def _find_text_start(buffer: bytearray, offset: int = 0) -> int:
        """Find ``@`` followed by text-field whitespace.

        A bare trailing ``@`` is retained as a possible split sync marker.  An
        ``@`` followed by binary data is not a text sync marker and is ignored.
        """

        index = buffer.find(b"@", offset)
        while index >= 0:
            if index + 1 == len(buffer) or buffer[index + 1] in (ord(" "), ord("\t")):
                return index
            index = buffer.find(b"@", index + 1)
        return -1

    def feed(self, data: bytes) -> None:
        if not data:
            return
        self.buffer.extend(data)

        while self.buffer:
            start = self._find_text_start(self.buffer)
            if start < 0:
                # Includes the documented binary FWT packets, which are ignored.
                self.buffer.clear()
                return
            if start:
                del self.buffer[:start]
            if len(self.buffer) == 1:
                # The sync byte arrived at the end of this serial read.
                return

            carriage_return = self.buffer.find(b"\r")
            line_feed = self.buffer.find(b"\n")
            terminators = [index for index in (carriage_return, line_feed) if index >= 0]
            end = min(terminators) if terminators else -1

            next_start = self._find_text_start(self.buffer, 1)
            if next_start >= 0 and (end < 0 or next_start < end):
                candidate = bytes(self.buffer[:next_start])
                if TEXT_PACKET_HEADER.match(candidate):
                    self.on_error(
                        "discarded incomplete Featherweight text packet "
                        f"({next_start} byte(s))"
                    )
                del self.buffer[:next_start]
                continue

            if end < 0:
                if len(self.buffer) > MAX_TEXT_PACKET_BYTES:
                    if TEXT_PACKET_HEADER.match(bytes(self.buffer)):
                        self.on_error("Featherweight text packet exceeded the size limit")
                    self.buffer.clear()
                return

            raw_line = bytes(self.buffer[:end])
            consume = end + 1
            if (
                self.buffer[end] == ord("\r")
                and len(self.buffer) > consume
                and self.buffer[consume] == ord("\n")
            ):
                consume += 1
            del self.buffer[:consume]

            # Binary packets can contain both 0x40 and line-ending byte values.
            # Only a documented text header makes the candidate actionable.
            if not TEXT_PACKET_HEADER.match(raw_line):
                continue

            try:
                line = raw_line.decode("ascii")
                packet = parse_featherweight_line(line)
            except (UnicodeDecodeError, FeatherweightParseError) as exc:
                preview = raw_line[:120].decode("ascii", errors="replace")
                self.on_error(f"{exc}; line={preview!r}")
                continue

            self.on_packet(packet)
