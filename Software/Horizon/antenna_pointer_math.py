"""Pure maths for a G-code antenna pointer (no Qt, no I/O, no serial).

Convention: X = azimuth in degrees clockwise from TRUE north, Y = elevation in
degrees above the horizon. Controller "millimetres" are treated as degrees (set
$100/$101 steps-per-unit to steps per degree). Design: ANTENNA_POINTER_DESIGN.md.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from geodesy import enu_range_bearing, enu_to_geodetic, geodetic_to_enu

GRBL_JOG_CANCEL = b"\x85"
GRBL_FEED_HOLD = b"!"
GRBL_STATUS_QUERY = b"?"


@dataclass(frozen=True)
class LookAngles:
    azimuth: float      # deg true, 0..360
    elevation: float    # deg above horizon, -90..90
    range_m: float      # slant range


def look_angles(gs_lat, gs_lon, gs_alt, lat, lon, alt):
    """Azimuth/elevation/slant range from the ground station to a target (WGS-84)."""
    e, n, u = geodetic_to_enu(lat, lon, alt, gs_lat, gs_lon, gs_alt)
    horizontal, az = enu_range_bearing(e, n)
    el = math.degrees(math.atan2(u, horizontal))
    return LookAngles(az, el, math.sqrt(e * e + n * n + u * u))


def predict_lla(lat, lon, alt, v_east, v_north, v_up, dt):
    """Constant-velocity extrapolation of a fix by dt seconds (dt may be 0)."""
    if dt == 0:
        return lat, lon, alt
    return enu_to_geodetic(v_east * dt, v_north * dt, v_up * dt, lat, lon, alt)


def wrap360(angle):
    return angle % 360.0


def wrap180(angle):
    return (angle + 180.0) % 360.0 - 180.0


def mount_command(look: LookAngles, *, az_offset=0.0, el_offset=0.0, declination=0.0):
    """True look angles -> mount (az, el). Offsets are calibration: the mount reads
    (true + offset). `declination` (east positive) converts true to magnetic for a
    mount whose zero was aligned with a compass: magnetic = true - declination."""
    return wrap360(look.azimuth + az_offset - declination), look.elevation + el_offset


def choose_azimuth(target_az, current_az, az_min=-180.0, az_max=180.0):
    """Equivalent of target_az (mod 360) inside [az_min, az_max], nearest current_az.

    Returns (azimuth, in_range). If no equivalent lies in the limits (travel < 360)
    the angularly nearest limit is returned with in_range False. current_az is
    the controller's reported axis position (an unwrapped value).
    """
    if az_max < az_min:
        raise ValueError("az_max < az_min")
    base = wrap360(target_az)
    k_lo = math.ceil((az_min - base) / 360.0 - 1e-12)
    k_hi = math.floor((az_max - base) / 360.0 + 1e-12)
    candidates = [base + 360.0 * k for k in range(k_lo, k_hi + 1)]
    if candidates:
        return min(candidates, key=lambda c: abs(c - current_az)), True
    d_min, d_max = abs(wrap180(base - az_min)), abs(wrap180(base - az_max))
    return (az_min if d_min <= d_max else az_max), False


def in_keyhole(elevation, limit=85.0):
    """True when azimuth is ill-conditioned (near zenith); hold azimuth there."""
    return elevation >= limit


def clamp(value, low, high):
    return max(low, min(high, value))


def rate_limit(current, target, max_rate, dt):
    """Move current toward target by at most max_rate*dt (units/s, s)."""
    step = max(0.0, max_rate) * max(0.0, dt)
    return current + clamp(target - current, -step, step)


def feed_for_move(delta_az, delta_el, dt, *, min_feed=6.0, max_feed=3600.0, margin=1.3):
    """Feed (deg/min) so the move takes about dt seconds, with a small margin.

    GRBL applies feed to the vector length, so az and el are combined.
    """
    distance = math.hypot(delta_az, delta_el)
    if distance < 1e-9:
        return min_feed
    return clamp(distance / max(dt, 1e-3) * 60.0 * margin, min_feed, max_feed)


def format_jog(az, el, feed):
    """GRBL 1.1 / grblHAL / FluidNC absolute jog (cancelable with 0x85)."""
    return f"$J=G21G90X{az:.3f}Y{el:.3f}F{feed:.0f}"


def format_move(az, el, feed):
    """Plain G1 move (also valid for Marlin, which has no jog command)."""
    return f"G1 X{az:.3f} Y{el:.3f} F{feed:.0f}"


def parse_grbl_status(line):
    """Parse a '<Idle|MPos:1.000,2.000,0.000|FS:0,0>' report -> (state, pos) or None."""
    line = line.strip()
    if not (line.startswith("<") and line.endswith(">")):
        return None
    fields = line[1:-1].split("|")
    pos = {}
    for field in fields[1:]:
        key, _, value = field.partition(":")
        if key in ("MPos", "WPos"):
            try:
                x, y, z = (float(v) for v in value.split(",")[:3])
            except ValueError:
                return None
            pos = {"x": x, "y": y, "z": z, "kind": key}
    return (fields[0].split(":")[0], pos) if pos else None
