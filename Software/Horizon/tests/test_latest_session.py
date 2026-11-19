"""witness_download.latest_session_files: resumed sessions and a restarted counter."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import witness_download as wd  # noqa: E402


def files(*names):
    return [{"name": n, "size": 100} for n in names]


class LatestSession(unittest.TestCase):
    def test_a_resumed_session_is_still_one_generation(self):
        # A reset in flight resumes in the same slot: the same files keep growing.
        listing = files("I0000367.BIN", "P0000367.BIN", "S0000367.BIN", "U0000367.BIN",
                        "I0000366.BIN", "P0000366.BIN")
        newest, wanted = wd.latest_session_files(listing)
        self.assertEqual(newest, 0x367)
        self.assertEqual(sorted(f["name"] for f in wanted),
                         ["I0000367.BIN", "P0000367.BIN", "S0000367.BIN", "U0000367.BIN"])

    def test_counter_restarted_after_a_full_erase(self):
        # Session numbers restart at 1 after 0x6869; every older file is gone with them.
        newest, wanted = wd.latest_session_files(files("I0000001.BIN", "P0000001.BIN", "P0000002.BIN"))
        self.assertEqual(newest, 2)
        self.assertEqual([f["name"] for f in wanted], ["P0000002.BIN"])

    def test_clear_keeps_counting(self):
        newest, _wanted = wd.latest_session_files(files("P0000870.BIN", "P0000871.BIN", "P0000872.BIN"))
        self.assertEqual(newest, 0x872)

    def test_lower_case_names_label_files_and_junk_are_ignored(self):
        newest, wanted = wd.latest_session_files(files("p00000ab.bin", "LABEL", "readme.txt", "P00000AA.BIN"))
        self.assertEqual(newest, 0xAB)
        self.assertEqual([f["name"] for f in wanted], ["p00000ab.bin"])

    def test_empty_listing(self):
        self.assertEqual(wd.latest_session_files([]), (-1, []))
        self.assertEqual(wd.latest_session_files(files("LABEL")), (-1, []))


if __name__ == "__main__":
    unittest.main()
