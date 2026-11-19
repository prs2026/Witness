"""A board reset during a HITL run: later states come from the bench sensors.

After a warm reset HITL is off and the board runs on its real sensors, which
read the bench altitude, not the simulated site (bench 2026-11-17: restored pad
reference at the simulated 609 m, bench barometer about 195 m, AGL about
-418 m, so a false Descent and Landed). The run must not score or fail on
those states, and must say so.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import hitl_protocol as hp  # noqa: E402
import hitl_session as hs  # noqa: E402


def status(state, hitl=True, resumed=False):
    return {"flight_state": state, "hitl_active": hitl, "gps_locked": True,
            "resumed_after_reset": resumed}


class ResetInRunTests(unittest.TestCase):
    def _run(self):
        run = hs.RunState(params=None, preset="t", samples=[], frames=[],
                          expected={hp.BOOST: 10.0, hp.COAST: 12.0, hp.DESCENT: 30.0,
                                    hp.LANDED: 60.0},
                          tolerance_s=1.0)
        run.running = run.streaming = True
        return run

    def _feed(self, session, run, sim_s, st, uptime, **kw):
        session._sim_s = lambda: sim_s
        session._on_status(status(st, **kw), uptime)
        session._accept_state(st, sim_s, "status")

    def test_reset_marks_later_states_not_scored(self):
        session = hs.HitlSession(runs_dir=None)
        run = self._run()
        session.run = run
        for t, st in ((9.0, hp.PAD_IDLE), (10.1, hp.BOOST), (12.1, hp.COAST)):
            self._feed(session, run, t, st, int(t * 1000) + 5000)
        self.assertIsNone(run.reset_sim_s)
        # Reset in Coast: uptime restarts, HITL off, resumed bit set; then the
        # bench barometer drives a false Descent and Landed.
        for t, st, up in ((15.0, hp.COAST, 400), (17.5, hp.DESCENT, 2900), (50.0, hp.LANDED, 35400)):
            self._feed(session, run, t, st, up, hitl=False, resumed=True)
        self.assertAlmostEqual(run.reset_sim_s, 15.0)
        run.hitl_seen = True
        result = hs.evaluate(run)
        by = {r["state"]: r["result"] for r in result["rows"]}
        self.assertEqual(by["Boost"], "OK")
        self.assertEqual(by["Coast"], "OK")
        self.assertEqual(by["Descent"], "AFTER RESET")
        self.assertEqual(by["Landed"], "AFTER RESET")
        self.assertTrue(result["passed"], result["problems"])
        self.assertFalse(run.hitl_dropped)

    def test_resumed_bit_already_set_at_run_start_is_not_a_reset(self):
        session = hs.HitlSession(runs_dir=None)
        run = self._run()
        session.run = run
        for t in (1.0, 2.0, 3.0):
            self._feed(session, run, t, hp.PAD_IDLE, int(t * 1000) + 90_000, resumed=True)
        self.assertIsNone(run.reset_sim_s)


def sensor_frame(uptime_ms, accel_y_bytes=b"\x00\x0a\x00", state=0):
    """A 0x01 sensors frame (23 bytes) with the given uptime; the accel Y bytes are free."""
    f = bytearray(23)
    f[0] = 0x01
    f[1] = 0x02
    f[2] = 0x10 | (state << 0)
    f[3:7] = uptime_ms.to_bytes(4, "big")
    f[10:13] = accel_y_bytes
    f[22] = 0x0A
    return bytes(f)


# A BIT summary frame (0x0B) as Witness sent it on the bench (2026-11-18): its data bytes hold a 0x01.
BIT_SUMMARY = bytes.fromhex("0b 14 60 01 40 08 96 00 40 61 01 00 00 0a")


class BitFramesAreNotSensorFramesTests(unittest.TestCase):
    """The false 'board reset at sim 5.4 s': BIT frames were scanned as other frames."""

    def test_summary_then_sensors_is_two_frames_with_the_real_uptime(self):
        # Sensor frame whose byte 11 is 0x0A: the old decoder started a "sensors" frame at the 0x01
        # inside the summary and ended it on that byte, reading a wild uptime.
        raw = BIT_SUMMARY + sensor_frame(3_681_540)
        events = hp.StreamDecoder().feed(raw)
        packets = [v for k, v in events if k == "packet"]
        self.assertEqual([p["name"] for p in packets], ["bit_summary", "sensors"])
        self.assertEqual(packets[1]["uptime_ms"], 3_681_540)
        self.assertEqual(packets[0]["verdict"], 4)   # NO-GO

    def test_detail_frame_is_one_frame(self):
        body = bytearray(99)
        body[0], body[1], body[2] = 0, 3, 7
        for i in range(24):   # records: id, flags, value; plenty of 0x01 and 0x02 bytes
            body[3 + 4 * i:7 + 4 * i] = bytes((0x11, 0x01, 0x00, 0x02))
        frame = bytes((0x10,)) + bytes(body) + b"\x0a"
        self.assertEqual(len(frame), 101)
        events = hp.StreamDecoder().feed(frame + sensor_frame(5000))
        self.assertEqual([v["name"] for k, v in events if k == "packet"], ["bit_detail", "sensors"])


class ResetDetectorTests(unittest.TestCase):
    def test_continuous_uptime_is_no_reset(self):
        d = hs.ResetDetector()
        for i in range(50):
            self.assertIsNone(d.feed(5000 + i * 100, False, i * 0.1))

    def test_stray_high_uptime_is_ignored(self):
        d = hs.ResetDetector()
        out = []
        for i, up in enumerate((5000, 5100, 167_995_136, 5300, 5400, 167_995_136, 5600, 5700)):
            out.append(d.feed(up, False, i * 0.1))
        self.assertEqual(out, [None] * 8)

    def test_stray_low_uptime_is_ignored(self):
        d = hs.ResetDetector()
        out = [d.feed(up, False, i * 0.1) for i, up in enumerate((60_000, 60_100, 0, 60_300, 60_400))]
        self.assertEqual(out, [None] * 5)

    def test_real_reset_is_found_at_its_first_packet(self):
        d = hs.ResetDetector()
        for i in range(5):
            self.assertIsNone(d.feed(60_000 + i * 100, False, 5.0 + i * 0.1))
        self.assertIsNone(d.feed(430, False, 5.5))          # first packet after the reset
        self.assertEqual(d.feed(530, False, 5.6), 5.5)      # the second one confirms it
        self.assertIsNone(d.feed(630, False, 5.7))          # reported once

    def test_forward_jump_is_not_a_reset(self):
        d = hs.ResetDetector()
        d.feed(1000, False, 0.0)
        self.assertIsNone(d.feed(500_000, False, 0.1))
        self.assertIsNone(d.feed(500_100, False, 0.2))

    def test_resumed_bit_needs_two_packets(self):
        d = hs.ResetDetector()
        d.feed(10_000, False, 0.0)
        self.assertIsNone(d.feed(10_100, True, 0.1))        # glitch
        self.assertIsNone(d.feed(10_200, False, 0.2))
        self.assertIsNone(d.feed(10_300, True, 0.3))
        self.assertEqual(d.feed(10_400, True, 0.4), 0.3)    # sticks: a resume

    def test_resumed_bit_set_from_the_start_is_not_a_reset(self):
        d = hs.ResetDetector()
        for i in range(5):
            self.assertIsNone(d.feed(90_000 + i * 100, True, i * 0.1))

    def test_session_does_not_flag_a_stray_uptime(self):
        session = hs.HitlSession(runs_dir=None)
        run = hs.RunState(params=None, preset="t", samples=[], frames=[], expected={}, tolerance_s=1.0)
        run.running = run.streaming = True
        session.run = run
        for i, up in enumerate((3_681_440, 167_995_136, 3_681_540, 167_995_136, 3_681_640, 3_681_740)):
            session._sim_s = lambda i=i: 5.0 + i * 0.1
            session._on_status(status(hp.PAD_IDLE), up)
        self.assertIsNone(run.reset_sim_s)


if __name__ == "__main__":
    unittest.main()
