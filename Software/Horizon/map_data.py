"""Rocket ground tracks for the Visualizer map, relative to the launch pad.

accept_record() runs on receiver threads (no GUI). snapshot() is called by the
map's UI timer and returns positions already converted to pad-relative ENU
metres; conversions are cached per point and redone only when the pad or the
Witness launch site changes.

Sources (distinguished on the map):
  kf       Witness position Kalman filter ENU (0x02/0xE0 position), origin = the
           vehicle's own pad. Placed with the Witness launch site (mean Witness
           GPS on the pad) when known, else assumed to be at the selected pad.
  wgps     Witness GPS lat/lon (0x02/0xE0, int24 1e-4 deg ~ 11 m), only while the
           status GPS-locked bit is set.
  fw       Featherweight tracker GPS_STAT (unit_type TRACKER) with a 2D/3D fix.
Altitudes are metres above the pad: Kalman up for kf, GPS MSL minus pad MSL
for the GPS sources (None until a pad altitude is known).
"""

from collections import deque
from dataclasses import dataclass
import math
import threading

from geodesy import enu_range_bearing, enu_to_geodetic, geodetic_to_enu, valid_lat_lon
from pad_reference import FT_TO_M, PadReference, record_time

TRACK_POINTS = 5000
SOURCES = ("kf", "wgps", "fw")
SOURCE_LABELS = {"kf": "Witness Kalman (ENU)", "wgps": "Witness GPS", "fw": "Featherweight tracker"}


@dataclass(frozen=True)
class TrackPoint:
    t: float
    east: float
    north: float
    agl: float | None
    lat: float | None = None
    lon: float | None = None


def _finite(*values):
    return all(type(v) in (int, float) and math.isfinite(v) for v in values)


class MapData:
    def __init__(self, pad=None, *, points=TRACK_POINTS):
        self.pad = pad if pad is not None else PadReference()
        self._lock = threading.Lock()
        self._raw = {name: deque(maxlen=points) for name in SOURCES}
        self._points = points
        self._cache = {}
        self._seq = 0
        self.status = (None, None, None)     # flight state, GPS lock, HITL
        self.version = 0

    def clear(self):
        with self._lock:
            for track in self._raw.values():
                track.clear()
            self._cache.clear()
            self.version += 1

    # ------------------------------------------------------------- input
    def accept_record(self, kind, record):
        try:
            packet = record.get("packet") if isinstance(record, dict) else None
            if not isinstance(packet, dict) or record.get("replay"):
                return False
            t = record_time(record)
            if kind == "witness":
                return self._witness(packet, t)
            if kind == "featherweight":
                return self._featherweight(packet, t)
        except (TypeError, ValueError, KeyError, AttributeError, OverflowError):
            pass
        return False

    def _append(self, name, item):
        with self._lock:
            track = self._raw[name]
            if track and item[0] < track[-1][1]:
                return False      # out of order
            self._seq += 1
            track.append((self._seq, *item))
            self.version += 1
        return True

    def _witness(self, packet, t):
        if packet.get("packet") not in ("state", "witness_debug"):
            return False
        status = packet.get("status")
        locked = None
        if type(status) is int:
            locked = (status >> 3) & 1
            with self._lock:
                self.status = (status & 7, locked, (status >> 4) & 1)
        added = False
        position = packet.get("position")
        quat = packet.get("orientation")
        if isinstance(position, list) and len(position) == 3 and _finite(*position):
            initialised = any(position) or (isinstance(quat, list) and any(quat))
            if initialised:
                added = self._append("kf", (t, *map(float, position)))
        lat, lon, alt = packet.get("latitude"), packet.get("longitude"), packet.get("gps_altitude")
        if locked == 1 and valid_lat_lon(lat, lon):
            added = self._append("wgps", (t, lat, lon, alt if _finite(alt) else None)) or added
        return added

    def _featherweight(self, packet, t):
        if packet.get("packet") != "GPS_STAT" or packet.get("unit_type") != "TRACKER":
            return False
        if packet.get("text_crc_valid") is False or packet.get("radio_crc_status", "CRC_OK") != "CRC_OK":
            return False
        fix = packet.get("fix_type")
        lat, lon = packet.get("latitude_deg"), packet.get("longitude_deg")
        if type(fix) is not int or fix < 2 or not valid_lat_lon(lat, lon):
            return False
        alt = packet.get("altitude_ft")
        alt = alt * FT_TO_M if fix >= 3 and _finite(alt) else None
        return self._append("fw", (t, lat, lon, alt))

    # ---------------------------------------------------------- reference
    def reference(self):
        """(lat, lon, label) the map is centred on, or None (KF-only, pad-relative)."""
        pad = self.pad.position()
        if pad is not None:
            return pad.lat, pad.lon, pad.source
        site = self.pad.witness_launch_site()
        if site is not None:
            return site.lat, site.lon, "witness"
        with self._lock:
            for name in ("wgps", "fw"):
                if self._raw[name]:
                    _, _, lat, lon, _ = self._raw[name][0]
                    return lat, lon, f"first {name} fix"
        return None

    def snapshot(self):
        ref = self.reference()
        pad_alt = self.pad.altitude_msl()
        site = self.pad.witness_launch_site()
        kf_origin = (site.lat, site.lon) if site is not None else (tuple(ref[:2]) if ref else None)
        key = (tuple(ref[:2]) if ref else None, pad_alt[0] if pad_alt else None, kf_origin)
        with self._lock:
            raw = {name: list(track) for name, track in self._raw.items()}
            status, version = self.status, self.version
        tracks = {}
        for name, items in raw.items():
            cached_key, converted = self._cache.get(name, (None, {}))
            if cached_key != key:
                converted = {}
            keep = {}
            for item in items:
                seq = item[0]
                point = converted.get(seq)
                if point is None and seq not in converted:
                    point = self._convert(name, item[1:], ref, pad_alt, kf_origin)
                keep[seq] = point
            self._cache[name] = (key, keep)
            tracks[name] = [p for p in keep.values() if p is not None]
        latest = {}
        for name, points in tracks.items():
            if points:
                p = points[-1]
                rng, bearing = enu_range_bearing(p.east, p.north)
                latest[name] = {"point": p, "range": rng, "bearing": bearing}
        return {"reference": ref, "pad": self.pad.position(), "pad_alt": pad_alt, "tracks": tracks,
                "latest": latest, "status": status, "version": version,
                "kf_placed": site is not None}

    @staticmethod
    def _convert(name, item, ref, pad_alt, kf_origin):
        if name == "kf":
            t, e, n, u = item
            if ref is None or kf_origin is None or kf_origin == tuple(ref[:2]):
                return TrackPoint(t, e, n, u)
            lat, lon, _ = enu_to_geodetic(e, n, 0.0, *kf_origin)
            pe, pn, _ = geodetic_to_enu(lat, lon, 0.0, ref[0], ref[1], 0.0)
            return TrackPoint(t, pe, pn, u, lat, lon)
        t, lat, lon, alt = item
        if ref is None:
            return None
        e, n, _ = geodetic_to_enu(lat, lon, 0.0, ref[0], ref[1], 0.0)
        agl = alt - pad_alt[0] if alt is not None and pad_alt is not None else None
        return TrackPoint(t, e, n, agl, lat, lon)
