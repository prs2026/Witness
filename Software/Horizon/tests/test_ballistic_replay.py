"""Verify the ready-to-replay ballistic fixture against independent equations."""

import math
from pathlib import Path
import unittest

from main import PacketDecoder
from witness_test_source import load_scenario


SOURCE = Path(__file__).resolve().parents[1] / "test_data" / "ballistic_10g.json"
G = 9.80665
LAUNCH = 5.0
BURNOUT = 7.0
APOGEE = 27.0
BURNOUT_SPEED = 20 * G
BURNOUT_ALTITUDE = 20 * G
MAX_ALTITUDE = 220 * G
TOUCHDOWN = APOGEE + math.sqrt(2 * MAX_ALTITUDE / G)


class BallisticReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scenario = load_scenario(SOURCE)
        cls.packets, cls.errors = [], []
        decoder = PacketDecoder(cls.packets.append, cls.errors.append)
        for frame in cls.scenario.frames:
            # Each chunk contains several packets; transport boundaries are arbitrary.
            for offset in range(0, len(frame.data), 7):
                decoder.feed(frame.data[offset:offset + 7])
        cls.remaining = bytes(decoder.buffer)
        cls.groups = {}
        for packet in cls.packets:
            cls.groups.setdefault(packet["uptime"], {})[packet["packet"]] = packet

    def test_complete_stream_and_state_transitions(self):
        self.assertEqual(self.errors, [])
        self.assertEqual(self.remaining, b"")
        self.assertEqual(self.scenario.duration_ms, 53000)
        self.assertEqual(len(self.scenario.frames), 531)
        self.assertEqual(len(self.packets), 1646)
        self.assertEqual({p["packet"] for p in self.packets}, {"sensors", "state", "witness_debug", "heartbeat"})
        transitions = []
        previous = None
        for stamp, packets in self.groups.items():
            states = {p["status"] & 7 for p in packets.values()}
            self.assertEqual(len(states), 1)
            state = states.pop()
            if state != previous:
                transitions.append((stamp, state))
                previous = state
            self.assertEqual(packets["state"]["status"] >> 8, 0x6E | ((stamp // 1000) % 2))
            self.assertEqual("heartbeat" in packets, stamp % 1000 == 0)
        self.assertEqual(transitions, [(0, 0), (5000, 1), (7000, 2), (27000, 3), (47977, 4)])

    def test_trajectory_matches_constant_gravity_and_no_drag(self):
        for stamp, packets in self.groups.items():
            t = stamp / 1000
            if t < LAUNCH or t >= TOUCHDOWN:
                altitude, velocity = 0.0, 0.0
            elif t < BURNOUT:
                elapsed = t - LAUNCH
                altitude, velocity = 0.5 * 10 * G * elapsed**2, 10 * G * elapsed
            else:
                elapsed = t - BURNOUT
                altitude = BURNOUT_ALTITUDE + BURNOUT_SPEED * elapsed - 0.5 * G * elapsed**2
                velocity = BURNOUT_SPEED - G * elapsed
            state, debug = packets["state"], packets["witness_debug"]
            for packet in (state, debug):
                self.assertAlmostEqual(packet["barometric_altitude"], altitude, delta=0.0002)
                self.assertEqual(packet["gps_altitude"], packet["barometric_altitude"])
                self.assertEqual(packet["position"], [0.0, 0.0, packet["barometric_altitude"]])
                self.assertAlmostEqual(packet["velocity"][2], velocity, delta=0.00002)
                self.assertEqual(packet["velocity"][:2], [0.0, 0.0])
                self.assertEqual(packet["orientation"], [0.0, 0.0, 0.0, 1.0])
            pressure = 101325 * (1 - altitude / 44330)**5.255
            self.assertAlmostEqual(debug["pressure"], pressure, delta=0.51)
            if BURNOUT <= t < TOUCHDOWN:
                energy = 0.5 * state["velocity"][2]**2 + G * state["position"][2]
                self.assertAlmostEqual(energy, G * MAX_ALTITUDE, delta=0.01)
        apogee = self.groups[27000]["state"]
        self.assertEqual(apogee["velocity"][2], 0.0)
        self.assertAlmostEqual(apogee["barometric_altitude"], 2157.463, delta=0.0002)
        self.assertEqual(max(g["state"]["barometric_altitude"] for g in self.groups.values()),
                         apogee["barometric_altitude"])
        self.assertLess(self.groups[47900]["state"]["velocity"][2], -200)

    def test_accelerometers_report_specific_force_not_kinematic_acceleration(self):
        for stamp, packets in self.groups.items():
            t = stamp / 1000
            specific_force = 11 if LAUNCH <= t < BURNOUT else 0 if BURNOUT <= t < TOUCHDOWN else 1
            sensors, debug = packets["sensors"], packets["witness_debug"]
            self.assertEqual(sensors["filtered_accel"], [0.0, 0.0, specific_force])
            self.assertEqual(sensors["filtered_accel"], debug["filtered_accel"])
            self.assertEqual(sensors["gyro"], [0.0, 0.0, 0.0])
            self.assertEqual(debug["gyro"], [0.0, 0.0, 0.0])
            self.assertAlmostEqual(debug["low_accel"][2], specific_force, delta=0.000245)
            self.assertAlmostEqual(debug["high_accel"][2], specific_force, delta=0.005209)


if __name__ == "__main__":
    unittest.main()
