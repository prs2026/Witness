"""Launch-pad reference: which position and MSL altitude Horizon treats as "the pad".

Fed (on receiver threads) with the same decoded records Flight View receives.
Sources, highest priority first; the first one that has produced a valid
position is used, so the pad does not hop between sources as packets arrive:

  manual        "Set pad here" on the map (position only; altitude from the next source)
  featherweight Featherweight ground station GPS_STAT (unit_type BASE), fix 2D/3D
  aperture      Aperture's own LC86G fix in the 0xF0 ground-station status
  witness       mean of Witness GPS fixes received while it reports Pad Idle + GPS lock;
                frozen from the first packet in any other state (Boost onwards) until
                Witness reboots (its uptime restarts) or the operator clears the pad
  tracker       first valid Featherweight tracker fix (last resort)

Altitude (m MSL) uses the same order, but needs a 3D fix for Featherweight, and
skips sources that have no altitude. Featherweight altitudes are feet ASL.
"""

from dataclasses import dataclass
import math
import threading
import time

from geodesy import valid_lat_lon

FT_TO_M = 0.3048
PAD_SOURCES = ("manual", "featherweight", "aperture", "witness", "tracker")
SOURCE_LABELS = {
    "manual": "manual (Set pad here)",
    "featherweight": "Featherweight ground station",
    "aperture": "Aperture LC86G GPS (0xF0)",
    "witness": "Witness GPS on pad (mean)",
    "tracker": "Featherweight tracker first fix",
}
WITNESS_PAD_FIXES = 120


@dataclass(frozen=True)
class PadFix:
    source: str
    lat: float
    lon: float
    alt_msl: float | None
    received: float
    detail: str = ""


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def record_time(record):
    received = record.get("received_mono")
    if _finite(received):
        return float(received)
    at = record.get("received_at")
    if _finite(at):
        return time.monotonic() - max(0.0, time.time() - at)
    return time.monotonic()


class PadReference:
    """Thread-safe pad selector. accept_record() never raises on bad data."""

    def __init__(self):
        self._lock = threading.Lock()
        self._fixes = {}
        self._witness = []
        # Witness left Pad Idle: its pad estimate is frozen (see _accept_witness).
        self.witness_frozen = False
        self._witness_uptime = None
        self.version = 0     # bumps whenever the selected pad could have changed

    # ------------------------------------------------------------ inputs
    def accept_record(self, kind, record):
        try:
            packet = record.get("packet") if isinstance(record, dict) else None
            if not isinstance(packet, dict) or record.get("replay"):
                return False
            received = record_time(record)
            if kind == "featherweight":
                return self._accept_featherweight(packet, received)
            if kind == "witness":
                return self._accept_witness(packet, received)
        except (TypeError, ValueError, KeyError, AttributeError, OverflowError):
            pass
        return False

    def _store(self, fix):
        with self._lock:
            self._fixes[fix.source] = fix
            self.version += 1
        return True

    def _accept_featherweight(self, packet, received):
        if packet.get("packet") != "GPS_STAT" or packet.get("text_crc_valid") is False:
            return False
        if packet.get("radio_crc_status", "CRC_OK") != "CRC_OK":
            return False
        unit = packet.get("unit_type")
        lat, lon = packet.get("latitude_deg"), packet.get("longitude_deg")
        fix_type = packet.get("fix_type", 0)
        if unit not in ("BASE", "TRACKER") or type(fix_type) is not int or fix_type < 2:
            return False
        if not valid_lat_lon(lat, lon):
            return False
        alt = packet.get("altitude_ft")
        alt = alt * FT_TO_M if fix_type >= 3 and _finite(alt) else None
        if unit == "BASE":
            return self._store(PadFix("featherweight", lat, lon, alt, received, f"fix {fix_type}D"))
        with self._lock:
            if "tracker" in self._fixes:
                return False
        return self._store(PadFix("tracker", lat, lon, alt, received, "first fix"))

    def _accept_witness(self, packet, received):
        kind = packet.get("packet")
        if kind == "ground_station_status":
            lat, lon = packet.get("latitude"), packet.get("longitude")
            if not valid_lat_lon(lat, lon):
                return False
            alt = packet.get("gps_altitude")
            quality, age = packet.get("gps_fix_quality"), packet.get("gps_fix_age_ms")
            detail = f"q{quality} {packet.get('gps_satellites')} sats"
            if _finite(packet.get("gps_hdop")):
                detail += f" HDOP {packet['gps_hdop']:.1f}"
            detail += " fix age " + (f"{age / 1000:.1f} s" if _finite(age) else ">65 s")
            return self._store(PadFix("aperture", lat, lon, alt if _finite(alt) else None, received, detail))
        if kind not in ("state", "witness_debug"):
            return False
        status = packet.get("status")
        if type(status) is not int:
            return False
        uptime = packet.get("uptime")
        with self._lock:
            if type(uptime) is int:
                if self._witness_uptime is not None and uptime + 5000 < self._witness_uptime:
                    # Witness rebooted: a new session may start a new pad.
                    self.witness_frozen = False
                    self._witness.clear()
                self._witness_uptime = uptime
            if status & 0x07 != 0:
                # Boost or later: the pad is where it was; never follow the
                # vehicle (nor a landing site after the flight).
                self.witness_frozen = True
                return False
            if self.witness_frozen:
                return False
        if not status & 0x08:
            return False   # Only Pad Idle with the GPS-locked bit describes the pad.
        lat, lon, alt = packet.get("latitude"), packet.get("longitude"), packet.get("gps_altitude")
        if not valid_lat_lon(lat, lon):
            return False
        with self._lock:
            self._witness.append((lat, lon, alt if _finite(alt) else None))
            del self._witness[:-WITNESS_PAD_FIXES]
            samples = list(self._witness)
        alts = [a for _, _, a in samples if a is not None]
        fix = PadFix("witness", sum(s[0] for s in samples) / len(samples),
                     sum(s[1] for s in samples) / len(samples),
                     sum(alts) / len(alts) if alts else None, received, f"{len(samples)} fixes")
        return self._store(fix)

    # ----------------------------------------------------------- control
    def set_manual(self, lat, lon):
        if not valid_lat_lon(lat, lon, reject_null_island=False):
            raise ValueError("pad latitude/longitude out of range")
        return self._store(PadFix("manual", float(lat), float(lon), None, time.monotonic(), "operator"))

    def clear_manual(self):
        """"Auto pad": drop the manual pad and let Witness define it again."""
        with self._lock:
            self._fixes.pop("manual", None)
            self.witness_frozen = False
            self.version += 1

    def clear(self):
        with self._lock:
            self._fixes.clear()
            self._witness.clear()
            self.witness_frozen = False
            self._witness_uptime = None
            self.version += 1

    # ------------------------------------------------------------ output
    def fixes(self):
        with self._lock:
            return dict(self._fixes)

    def position(self):
        """Selected PadFix (lat/lon), or None before any source has a fix."""
        fixes = self.fixes()
        return next((fixes[name] for name in PAD_SOURCES if name in fixes), None)

    def altitude_msl(self):
        """(altitude m MSL, source) of the highest-priority source with an altitude."""
        fixes = self.fixes()
        for name in PAD_SOURCES:
            fix = fixes.get(name)
            if fix is not None and fix.alt_msl is not None:
                return fix.alt_msl, name
        return None

    def witness_launch_site(self):
        """Mean Witness pad fix: the Kalman ENU origin when Witness had GPS on the pad."""
        return self.fixes().get("witness")
