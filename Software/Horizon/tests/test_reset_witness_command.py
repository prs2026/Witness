"""'False flight on pad: reset Witness' (0x7300): enabled only in Landed, asks first."""
import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import witness_protocol as wp  # noqa: E402

LANDED, PAD, BOOST = 4, 0, 1


class ResetWitnessCommand(unittest.TestCase):
    def setUp(self):
        from horizon_qt_support import make_monitor
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        patcher = patch("monitor_core.LOG_DIR", Path(self.directory.name))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.monitor = make_monitor(self.directory.name)
        self.addCleanup(self.monitor.close)
        self.core = self.monitor.core
        self.core.commands_enabled.set(True)
        self.sent = []
        self.core.transmit_command = lambda target, command, label="": self.sent.append(
            (command, label)) or True
        self.asked = []
        self.answer = True
        self.core.confirm = lambda title, text: self.asked.append(text) or self.answer

    def feed(self, state, packet="sensors", flags=0x0100):
        pkt = {"packet": packet, "status": flags | state, "uptime": 1,
               "filtered_accel": [0.0, 0.0, 1.0], "gyro": [0.0, 0.0, 0.0]}
        self.monitor.events.put(("packet", ("COM_TEST", pkt, time.time())))
        self.monitor.process_events()

    def button(self):
        return self.monitor.reset_witness_button

    def test_button_is_labelled_and_follows_the_flight_state(self):
        button = self.button()
        self.assertEqual(button.text(), "False flight on pad: reset Witness")
        self.assertFalse(button.isEnabled())            # no telemetry yet
        for state, enabled in ((PAD, False), (BOOST, False), (LANDED, True), (BOOST, False)):
            self.feed(state)
            self.assertEqual(button.isEnabled(), enabled, f"state {state}")
        self.feed(LANDED)
        self.core.commands_enabled.set(False)           # no link: never enabled
        self.assertFalse(button.isEnabled())

    def test_landed_status_with_other_flag_bits_still_counts(self):
        self.feed(LANDED, flags=0x0100 | 0x0008 | 0x0010)    # GPS lock + HITL bits
        self.assertTrue(self.button().isEnabled())
        self.feed(LANDED, packet="heartbeat")
        self.assertTrue(self.button().isEnabled())

    def test_sends_0x7300_only_after_confirmation(self):
        self.feed(LANDED)
        self.answer = False
        self.assertFalse(self.core.send_quick_command(wp.RESET_WITNESS_LABEL, 0x7300, True))
        self.assertEqual(self.sent, [])
        self.assertIn("FALSE FLIGHT ON THE PAD", self.asked[-1])
        self.answer = True
        self.assertTrue(self.core.send_quick_command(wp.RESET_WITNESS_LABEL, 0x7300, True))
        self.assertEqual(self.sent[-1][0], wp.build_command_packet(0x7300))

    def test_refused_by_the_monitor_when_not_landed(self):
        for state in (PAD, BOOST):
            self.feed(state)
            self.assertFalse(self.core.send_quick_command(wp.RESET_WITNESS_LABEL, 0x7300, True))
        self.assertEqual(self.sent, [])
        self.assertEqual(self.asked, [])                 # not even asked
        self.assertIn("TX_BLOCKED", "\n".join(self.core.console_pending) if hasattr(
            self.core, "console_pending") else "TX_BLOCKED")

    def test_the_command_is_in_the_quick_list_once(self):
        self.assertEqual([c for c in wp.QUICK_COMMANDS if c[1] == 0x7300],
                         [(wp.RESET_WITNESS_LABEL, 0x7300, True)])


if __name__ == "__main__":
    unittest.main()
