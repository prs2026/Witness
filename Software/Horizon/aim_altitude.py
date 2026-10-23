"""AIM pressure altitude and interval-average vertical velocity estimates.

Captured pressure_payload/v1 values (~99,000 near ground) are interpreted as
Pa, not hPa. Standard-atmosphere hydrostatic layers use 101325 Pa at sea level;
this is pressure altitude, NOT AIM's calibrated/fused altitude or height AGL.
GPS altitude remains in publisher units until that MQTT unit is verified.
Velocity is delta altitude / delta publisher timestamp (Unix microseconds),
positive upwards. No extrapolation, GPS/baro mixing or differentiation of
retained, duplicate, invalid or out-of-order samples.
"""

import math

BARO_ALTITUDE = "_derived.barometric_altitude_m"
BARO_VELOCITY = "_derived.vertical_velocity_mps"


def pressure_altitude(pressure):
    """Invert standard hydrostatic lapse-rate layers; pressure in Pa, height m."""
    if type(pressure) not in (int, float) or not 100 <= pressure <= 120000:
        return None
    height, temperature, base_pressure = 0.0, 288.15, 101325.0
    gravity_over_r = 9.80665 / 287.05287
    for top, lapse in ((11000, -.0065), (20000, 0), (32000, .001), (47000, .0028), (51000, 0)):
        top_temperature = temperature + lapse * (top - height)
        top_pressure = base_pressure * (math.exp(-gravity_over_r * (top - height) / temperature)
            if lapse == 0 else (top_temperature / temperature) ** (-gravity_over_r / lapse))
        if pressure >= top_pressure:
            return height + (-temperature / gravity_over_r * math.log(pressure / base_pressure)
                if lapse == 0 else temperature / lapse * ((pressure / base_pressure) ** (-lapse / gravity_over_r) - 1))
        height, temperature, base_pressure = top, top_temperature, top_pressure
    return None


class AimAltitudeEstimator:
    def __init__(self):
        self.previous = {}
        self.last_received = {}

    def update(self, source, fields, received, *, retained=False, valid=True, max_gap=3):
        if retained or "pressure" not in fields:
            return {}
        previous = self.previous.get(source)
        if received < self.last_received.get(source, -math.inf):
            return {}
        if source not in self.last_received and len(self.last_received) >= 128:
            oldest = next(iter(self.last_received))
            self.last_received.pop(oldest)
            self.previous.pop(oldest, None)
        self.last_received[source] = received
        altitude = pressure_altitude(fields["pressure"]) if valid else None
        stamp = fields.get("timestamp")
        if type(stamp) is not int or not 0 <= stamp <= 10**18:
            self.previous.pop(source, None)
            return {BARO_ALTITUDE: altitude, BARO_VELOCITY: None}
        if previous is not None and received - previous[2] > max_gap:
            previous = None
        if previous is not None:
            seconds = (stamp - previous[0]) / 1_000_000
            if seconds < .001:
                return {}
        if altitude is None:
            self.previous.pop(source, None)
            return {BARO_ALTITUDE: None, BARO_VELOCITY: None}
        if source not in self.previous and len(self.previous) >= 128:
            self.previous.pop(next(iter(self.previous)))
        self.previous[source] = stamp, altitude, received
        velocity = None if previous is None or seconds > max_gap else (altitude - previous[1]) / seconds
        return {BARO_ALTITUDE: altitude, BARO_VELOCITY: velocity}
