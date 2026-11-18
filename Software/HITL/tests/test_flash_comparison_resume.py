"""Flash-log comparison after a reset in flight (converted P*.csv with segments)."""

import csv
import os
import sys
import tempfile
import types
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import hitl_outputs  # noqa: E402


class FlashComparisonResumeTest(unittest.TestCase):
    def test_rows_after_a_reset_are_not_compared(self):
        with tempfile.TemporaryDirectory() as tmp:
            fc_dir = os.path.join(tmp, "fc_logs")
            os.makedirs(fc_dir)
            fields = ["log_source", "uptime", "hitl_active", "flight_state", "segment"]
            with open(os.path.join(fc_dir, "P0000001.csv"), "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=fields)
                w.writeheader()
                for i in range(3):      # before the reset: no segment column value
                    w.writerow({"log_source": "witness_state", "uptime": 1000 + 20 * i,
                                "hitl_active": "1", "flight_state": "2"})
                w.writerow({"log_source": "log_marker", "uptime": "", "segment": "1"})
                for i in range(2):      # device uptime restarted
                    w.writerow({"log_source": "witness_state", "uptime": 5000 + 20 * i,
                                "hitl_active": "1", "flight_state": "2", "segment": "1"})
            run = types.SimpleNamespace(
                run_dir=tmp, state_track=None,
                debug_records=[(10.0 + i, {"uptime_ms": 1000 + 1000 * i}) for i in range(3)],
                samples=[types.SimpleNamespace(t_ms=1_000_000)])
            seen = []
            originals = (hitl_outputs._compare_row, hitl_outputs._write_rows, hitl_outputs.summarize)
            hitl_outputs._compare_row = lambda *a: seen.append(a[3]) or {"uptime_ms": a[3]}
            hitl_outputs._write_rows = lambda path, rows: None
            hitl_outputs.summarize = lambda rows: {"n": len(rows)}
            try:
                messages = []
                result = hitl_outputs.write_flash_comparison(run, fc_dir, messages.append)
            finally:
                hitl_outputs._compare_row, hitl_outputs._write_rows, hitl_outputs.summarize = originals
            self.assertEqual(result, {"n": 3})
            self.assertEqual(seen, [1000, 1020, 1040])
            self.assertTrue(any("2 state packet(s) after a reset" in m for m in messages))


if __name__ == "__main__":
    unittest.main()
