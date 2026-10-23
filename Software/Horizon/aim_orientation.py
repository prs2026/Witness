"""Estimated AIM angular rate from successive orientation MQTT samples.

Hamilton quaternions in (w, x, y, z) order are normalized before computing
conjugate(previous) * current. Its shortest rotation vector divided by dt is
the interval-average rate in degrees/second. This assumes body-to-world
orientation; AIM's frame convention/mounting relative to Witness is unverified.
It is NOT a raw gyro measurement: fusion corrections, quantization and motion
over 180 degrees between samples cannot be distinguished from actual rotation.

See https://ahrs.readthedocs.io/en/latest/quaternion/quaternionarray.angular_velocities.html
for the quaternion-difference relationship. Here the full axis-angle logarithm
is used instead of a small-angle approximation. No smoothing or extrapolation.

Captured AIM BASE timestamps are integer Unix microseconds. Use their deltas,
never MQTT receive spacing (several orientations can arrive in the same burst).
Duplicate/out-of-order stamps do not refresh the estimate. After a reception
gap or invalid sample, two fresh orientations are required to produce rates.
"""

import math


GYRO_FIELDS = tuple(f"_derived.gyro_dps[{i}]" for i in range(3))


def rotation_rate(previous, current, seconds):
    """Shortest relative rotation / dt; inputs must be unit quaternions."""
    # q and -q describe exactly the same orientation.
    if sum(a * b for a, b in zip(previous, current)) < 0:
        current = tuple(-v for v in current)
    w, x, y, z = previous
    a, b, c, d = current
    scalar = w*a + x*b + y*c + z*d
    vector = (w*b - x*a - y*d + z*c,
              w*c + x*d - y*a - z*b,
              w*d - x*c + y*b - z*a)
    length = math.hypot(*vector)
    if length < 1e-12:
        return (0.0, 0.0, 0.0)
    angle = 2 * math.atan2(length, max(0.0, scalar))
    factor = math.degrees(angle) / (length * seconds)
    return tuple(v * factor for v in vector)


class AimAngularRateEstimator:
    """Per-source state. Called under FlightData's lock; no GUI/device access."""

    def __init__(self):
        self.previous = {}

    def update(self, source, fields, received, *, retained=False, valid=True, max_gap=3):
        if retained or not any(key in fields for key in ('w', 'x', 'y', 'z')):
            return {}
        missing = dict.fromkeys(GYRO_FIELDS)
        previous = self.previous.get(source)
        if previous is not None and received < previous[2]:
            return {}
        stamp = fields.get('timestamp')
        quaternion = tuple(fields.get(key) for key in ('w', 'x', 'y', 'z'))
        if (not valid or type(stamp) is not int or not 0 <= stamp <= 10**18
                or not all(type(v) in (int, float) and abs(v) <= 1e100 and math.isfinite(v)
                           for v in quaternion)):
            self.previous.pop(source, None)
            return missing
        norm = math.hypot(*quaternion)
        if norm < 1e-12:
            self.previous.pop(source, None)
            return missing
        quaternion = tuple(v / norm for v in quaternion)
        if previous is not None and received - previous[2] > max_gap:
            previous = None
        if previous is not None:
            seconds = (stamp - previous[0]) / 1_000_000
            if seconds < .001:
                # Ignore repeated/backwards/sub-ms timestamps; keep the original
                # baseline so a subsequent usable interval can still be measured.
                return {}
        if source not in self.previous and len(self.previous) >= 128:
            self.previous.pop(next(iter(self.previous)))
        self.previous[source] = (stamp, quaternion, received)
        if previous is None or seconds > max_gap:
            return missing
        return dict(zip(GYRO_FIELDS, rotation_rate(previous[1], quaternion, seconds)))
