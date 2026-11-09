"""Deterministic 1-D vertical flight model and firmware reference state machine.

The model runs at 100 Hz and produces what the Witness sensors would see:
specific force along the rocket's long axis (+X; +1 g at rest on the pad),
small Y/Z noise, gyro noise (optional roll spin, conical sway under canopy),
pressure from an ISA atmosphere, temperatures, GPS and battery.

``ReferenceStateMachine`` mirrors Witness ``flight_state_machine.cpp`` so the
tool can predict when Witness should report each transition. Prediction is fed
the quantised values that actually go on the wire (encode -> decode).
"""

from __future__ import annotations

import dataclasses
import math
import random
from dataclasses import dataclass

import hitl_protocol as hp


def meters_per_degree(lat_deg: float) -> tuple[float, float]:
    """WGS-84 metres per degree of latitude and of longitude at lat_deg.

    Real GPS reports WGS-84 coordinates and Witness converts them with these
    same formulas, so simulated fixes must use them too (a 111,320 m/deg
    sphere put a 0.37% north scale error into HITL runs).
    """
    phi = math.radians(lat_deg)
    lat_m = 111132.92 - 559.82 * math.cos(2 * phi) + 1.175 * math.cos(4 * phi)
    lon_m = 111412.84 * math.cos(phi) - 93.5 * math.cos(3 * phi)
    return lat_m, max(lon_m, 1e-6)

G0 = 9.80665
DT_S = 0.01
TICK_MS = 10

# ISA troposphere
_T0 = 288.15
_LAPSE = 0.0065
_R = 287.05287
_EXP = G0 / (_R * _LAPSE)  # 5.25588


def isa_pressure_pa(alt_msl_m: float, sea_level_pa: float = 101325.0) -> float:
    return sea_level_pa * (1.0 - _LAPSE * alt_msl_m / _T0) ** _EXP


def isa_density(alt_msl_m: float) -> float:
    t = _T0 - _LAPSE * alt_msl_m
    return isa_pressure_pa(alt_msl_m) / (_R * t)


@dataclass
class SimParams:
    """All model parameters. Times in s, altitudes in m, rates in m/s."""
    seed: int = 1
    pad_time_s: float = 10.0
    ignite: bool = True
    site_alt_msl_m: float = 1401.0
    lat_deg: float = 32.9903
    lon_deg: float = -106.9750
    sea_level_pressure_pa: float = 101325.0
    ground_temp_c: float = 25.0
    thrust_accel_g: float = 14.0       # average thrust / mass
    burn_time_s: float = 2.0
    mass_kg: float = 5.0
    drag_cd: float = 0.5
    diameter_m: float = 0.10
    drogue_delay_s: float = 1.0        # after apogee
    drogue_rate_mps: float = 25.0
    main_alt_agl_m: float = 200.0
    main_rate_mps: float = 6.0
    chute_tau_s: float = 0.6           # canopy velocity time constant
    descent_sway_dps: float = 15.0     # conical sway under canopy
    spin_dps: float = 0.0              # roll about X during boost/coast
    accel_noise_g: float = 0.003
    gyro_noise_dps: float = 0.15
    baro_noise_pa: float = 1.5
    gps_sats: int = 10
    gps_fix: bool = True
    gps_drift_m: float = 0.5           # random-walk sigma per second
    gps_dropout_start_s: float | None = None
    gps_dropout_end_s: float | None = None
    battery_mv: float = 7800.0
    battery_drain_mv_per_min: float = 4.0
    imu_temp_c: float = 32.0
    post_landing_s: float = 40.0
    max_duration_s: float = 900.0
    no_launch_duration_s: float = 60.0  # run length when ignite is False

    @classmethod
    def from_dict(cls, data: dict) -> "SimParams":
        names = {f.name for f in dataclasses.fields(cls)}
        return cls(**{k: v for k, v in data.items() if k in names})

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)


PRESETS = {
    "Nominal 10 g": {},
    "Long burn": {"thrust_accel_g": 11.5, "burn_time_s": 6.0,
                  "mass_kg": 12.0, "diameter_m": 0.13},
    "High-g short burn": {"thrust_accel_g": 40.0, "burn_time_s": 0.6,
                          "spin_dps": 120.0},
    "GPS dropout": {"gps_dropout_start_s": 11.0, "gps_dropout_end_s": 40.0},
    "No GPS fix": {"gps_fix": False, "gps_sats": 3},
    "Sea-level site": {"site_alt_msl_m": 5.0, "lat_deg": 37.0,
                       "lon_deg": -122.0},
    "Pad abort - no launch": {"ignite": False},
    "Quick hop (test)": {"pad_time_s": 2.0, "thrust_accel_g": 15.0,
                         "burn_time_s": 0.8, "drogue_delay_s": 0.5,
                         "drogue_rate_mps": 40.0, "main_alt_agl_m": 0.0,
                         "post_landing_s": 35.0},
}
# Accept the em-dash spelling used in specs.
PRESET_ALIASES = {"Pad abort — no launch": "Pad abort - no launch",
                  "Pad abort": "Pad abort - no launch"}


def preset_params(name: str) -> SimParams:
    name = PRESET_ALIASES.get(name, name)
    if name not in PRESETS:
        raise KeyError(f"unknown preset {name!r}; choose from "
                       + ", ".join(PRESETS))
    return SimParams(**PRESETS[name])


@dataclass
class SimSample:
    t_ms: int
    phase: str
    alt_msl_m: float
    vel_mps: float
    hitl: hp.HitlInput

    @property
    def low_g_x(self) -> float:
        return self.hitl.low_g[0]


class FlightSim:
    """100 Hz deterministic flight model; iterate with ``step()``."""

    def __init__(self, params: SimParams):
        self.p = params
        self.rng = random.Random(params.seed)
        self.t = 0.0
        self.tick = 0
        self.alt = params.site_alt_msl_m
        self.vel = 0.0
        self.phase = "pad"
        self.burn_start = None
        self.apogee_t = None
        self.apogee_alt = None
        self.landed_t = None
        self.roll = 0.0
        self.gps_n = 0.0
        self.gps_e = 0.0
        area = math.pi * (params.diameter_m / 2) ** 2
        self._k_drag = 0.5 * params.drag_cd * area / params.mass_kg

    def done(self) -> bool:
        p = self.p
        if self.t >= p.max_duration_s:
            return True
        if not p.ignite:
            return self.t >= p.no_launch_duration_s
        return (self.landed_t is not None
                and self.t >= self.landed_t + p.post_landing_s)

    def _noise(self, sigma: float) -> float:
        return self.rng.gauss(0.0, sigma) if sigma > 0 else 0.0

    def step(self) -> SimSample:
        p = self.p
        t = self.t
        ground = p.site_alt_msl_m
        specific_x = 1.0  # g
        gyro = [0.0, 0.0, 0.0]

        if self.phase == "pad" and p.ignite and t >= p.pad_time_s:
            self.phase = "boost"
            self.burn_start = t

        if self.phase in ("boost", "coast", "drogue", "main"):
            drag = self._k_drag * isa_density(self.alt) * self.vel * abs(
                self.vel)  # m/s^2, opposes velocity sign via vel*|vel|
            if self.phase == "boost":
                tb = t - self.burn_start
                ramp = min(1.0, tb / 0.05, max(0.0, (p.burn_time_s - tb) / 0.05))
                thrust = p.thrust_accel_g * G0 * ramp
                if tb >= p.burn_time_s:
                    self.phase = "coast"
                    thrust = 0.0
                accel = thrust - drag - G0
                specific_x = (thrust - drag) / G0
                if self.alt <= ground and accel < 0:  # still on the rail
                    accel = 0.0
                    specific_x = 1.0
            elif self.phase == "coast":
                accel = -drag - G0
                specific_x = -drag / G0
                if self.vel <= 0 and self.apogee_t is None:
                    self.apogee_t = t
                    self.apogee_alt = self.alt
                if self.apogee_t is not None and t >= self.apogee_t + \
                        p.drogue_delay_s:
                    self.phase = "drogue"
            else:
                target = -(p.drogue_rate_mps if self.phase == "drogue"
                           else p.main_rate_mps)
                if (self.phase == "drogue" and p.main_alt_agl_m > 0
                        and self.alt - ground <= p.main_alt_agl_m):
                    self.phase = "main"
                    target = -p.main_rate_mps
                accel = (target - self.vel) / max(p.chute_tau_s, 0.05)
                specific_x = (accel + G0) / G0
                ang = 2 * math.pi * 0.4 * t
                gyro[1] = p.descent_sway_dps * math.cos(ang)
                gyro[2] = p.descent_sway_dps * math.sin(ang)
            if self.phase in ("boost", "coast"):
                gyro[0] = p.spin_dps
            self.vel += accel * DT_S
            self.alt += self.vel * DT_S
            if self.alt <= ground and self.phase in ("drogue", "main", "coast"):
                self.alt = ground
                self.vel = 0.0
                self.phase = "landed"
                self.landed_t = t
                specific_x = 1.0
                gyro = [0.0, 0.0, 0.0]
            elif self.alt < ground:
                self.alt = ground
                self.vel = max(self.vel, 0.0)

        # Sensor outputs.
        ln = p.accel_noise_g
        low = (specific_x + self._noise(ln), self._noise(ln), self._noise(ln))
        hi_n = max(ln, 0.01)
        high = (specific_x + self._noise(hi_n), self._noise(hi_n),
                self._noise(hi_n))
        gn = p.gyro_noise_dps
        gyr = tuple(g + self._noise(gn) for g in gyro)
        pressure = isa_pressure_pa(self.alt, p.sea_level_pressure_pa) + \
            self._noise(p.baro_noise_pa)
        air_temp = p.ground_temp_c - _LAPSE * (self.alt - ground)
        baro_temp = 0.7 * p.imu_temp_c + 0.3 * air_temp

        # GPS: slight random-walk drift, optional dropout / no fix.
        step_sigma = p.gps_drift_m * math.sqrt(DT_S)
        self.gps_n += self._noise(step_sigma)
        self.gps_e += self._noise(step_sigma)
        dropout = (p.gps_dropout_start_s is not None
                   and t >= p.gps_dropout_start_s
                   and (p.gps_dropout_end_s is None
                        or t < p.gps_dropout_end_s))
        has_fix = p.gps_fix and not dropout
        if has_fix:
            m_lat, m_lon = meters_per_degree(p.lat_deg)
            lat = p.lat_deg + self.gps_n / m_lat
            lon = p.lon_deg + self.gps_e / m_lon
            gps_alt = self.alt + self._noise(1.0)
            sats, fix = p.gps_sats, 1
        else:
            lat = lon = gps_alt = 0.0
            sats, fix = (0 if dropout else p.gps_sats), 0

        battery = p.battery_mv - p.battery_drain_mv_per_min * t / 60.0
        if self.phase == "boost":
            battery -= 60.0

        sample = SimSample(
            t_ms=self.tick * TICK_MS, phase=self.phase, alt_msl_m=self.alt,
            vel_mps=self.vel,
            hitl=hp.HitlInput(
                sim_time_ms=self.tick * TICK_MS, low_g=low, high_g=high,
                gyro_dps=gyr, imu_temp_c=p.imu_temp_c, pressure_pa=pressure,
                baro_temp_c=baro_temp, lat_deg=lat, lon_deg=lon,
                gps_alt_m=gps_alt, satellites=sats, fix_quality=fix,
                battery_mv=battery))
        self.tick += 1
        self.t = self.tick * DT_S
        return sample


def generate(params: SimParams) -> list:
    sim = FlightSim(params)
    out = []
    while not sim.done():
        out.append(sim.step())
    return out


# ---------------------------------------------------------------------------
# Reference implementation of flight_state_machine.cpp
# ---------------------------------------------------------------------------

class ReferenceStateMachine:
    LAUNCH_G = 10.0
    LAUNCH_MS = 300
    BURNOUT_G = 1.0
    BURNOUT_MS = 500
    DESCENT_FRACTION = 0.95
    DESCENT_MS = 500
    LAND_GYRO_DPS = 10.0
    LAND_MIN_G = 0.8
    LAND_MAX_G = 1.2
    LAND_SPAN_M = 10.0
    LAND_MS = 30_000

    def __init__(self):
        self.state = hp.PAD_IDLE
        self._observed_state = hp.PAD_IDLE
        self._cond_active = False
        self._cond_start = 0
        self._land_active = False
        self._land_start = 0
        self._land_min = 0.0
        self._land_max = 0.0
        self._max_valid = False
        self._max_alt = 0.0
        self.transitions = []  # (timestamp_us, from, to)

    def _set(self, state, ts):
        if state != self.state:
            self.transitions.append((ts, self.state, state))
        self.state = state

    def _held(self, cond, ts, ms):
        if not cond:
            self._cond_active = False
            return False
        if not self._cond_active:
            self._cond_active = True
            self._cond_start = ts
            return False
        return ts - self._cond_start >= ms * 1000

    def _landing(self, accel, gyro, alt, ts):
        am = math.sqrt(sum(a * a for a in accel))
        gm = math.sqrt(sum(g * g for g in gyro))
        if not (self.LAND_MIN_G <= am <= self.LAND_MAX_G
                and gm < self.LAND_GYRO_DPS):
            self._land_active = False
            return False
        if not self._land_active:
            self._land_min = self._land_max = alt
        else:
            self._land_min = min(self._land_min, alt)
            self._land_max = max(self._land_max, alt)
        if self._land_max - self._land_min > self.LAND_SPAN_M:
            self._land_active = False
            self._land_min = self._land_max = alt
            return False
        if not self._land_active:
            self._land_active = True
            self._land_start = ts
            return False
        return ts - self._land_start >= self.LAND_MS * 1000

    def update(self, ts_us, axial_g, low_g_x, accel_g, gyro_dps, baro_alt_m):
        """One observation. accel_g/gyro_dps are low-g and gyro xyz vectors."""
        if ts_us == 0:
            return self.state
        cur = self.state
        if cur != self._observed_state:
            self._observed_state = cur
            self._cond_active = False
            if cur in (hp.PAD_IDLE, hp.BOOST):
                self._max_valid = True
                self._max_alt = baro_alt_m
            if cur in (hp.PAD_IDLE, hp.LANDED):
                self._land_active = False
        if not self._max_valid or baro_alt_m > self._max_alt:
            self._max_alt = baro_alt_m
            self._max_valid = True
            if cur == hp.COAST:
                self._cond_active = False
        if cur in (hp.BOOST, hp.COAST, hp.DESCENT) and self._landing(
                accel_g, gyro_dps, baro_alt_m, ts_us):
            self._set(hp.LANDED, ts_us)
            return self.state
        if cur == hp.PAD_IDLE:
            if self._held(axial_g > self.LAUNCH_G, ts_us, self.LAUNCH_MS):
                self._set(hp.BOOST, ts_us)
        elif cur == hp.BOOST:
            if self._held(low_g_x < self.BURNOUT_G, ts_us, self.BURNOUT_MS):
                self._set(hp.COAST, ts_us)
        elif cur == hp.COAST:
            thr = self._max_alt - abs(self._max_alt) * (
                1.0 - self.DESCENT_FRACTION)
            if self._held(self._max_valid and baro_alt_m < thr, ts_us,
                          self.DESCENT_MS):
                self._set(hp.DESCENT, ts_us)
        return self.state

    def update_from_hitl(self, ts_us: int, decoded: dict):
        """Feed a decoded 0x0C frame (as Witness would interpret it)."""
        return self.update(
            ts_us, decoded["high_g"][0], decoded["low_g"][0],
            decoded["low_g"], decoded["gyro_dps"],
            hp.pressure_altitude_m(decoded["pressure_pa"]))


def predict_transitions(samples) -> dict:
    """Expected sim time (s) of each state entry, from the quantised frames.

    Returns {state: seconds}; states never reached are absent.
    """
    fsm = ReferenceStateMachine()
    for s in samples:
        frame = hp.encode_hitl(s.hitl)
        # Firmware ignores timestamp 0; offset by one tick like a real clock.
        fsm.update_from_hitl((s.t_ms + TICK_MS) * 1000, hp.decode_hitl(frame))
    return {to: (ts / 1e6) - TICK_MS / 1000.0
            for ts, _frm, to in fsm.transitions}


def predict_state_track(samples) -> list:
    """Expected Witness flight state after each sample (same feed as above)."""
    fsm = ReferenceStateMachine()
    out = []
    for s in samples:
        frame = hp.encode_hitl(s.hitl)
        out.append(fsm.update_from_hitl((s.t_ms + TICK_MS) * 1000,
                                        hp.decode_hitl(frame)))
    return out


def summarize(samples) -> dict:
    apogee = max(samples, key=lambda s: s.alt_msl_m) if samples else None
    site = samples[0].alt_msl_m if samples else 0.0
    return {
        "duration_s": len(samples) * DT_S,
        "apogee_msl_m": apogee.alt_msl_m if apogee else None,
        "apogee_agl_m": (apogee.alt_msl_m - site) if apogee else None,
        "apogee_t_s": apogee.t_ms / 1000 if apogee else None,
        "max_low_g_x": max((s.hitl.low_g[0] for s in samples), default=0),
        "max_high_g_x": max((s.hitl.high_g[0] for s in samples), default=0),
    }
