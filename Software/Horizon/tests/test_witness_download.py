"""Tests for the Pi-side FAT16 reader used by witness_download.py."""

import io
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import witness_pi_agent as agent  # noqa: E402

SECTOR = 512
SECTORS_PER_CLUSTER = 8
RESERVED = 1
FAT_SECTORS = 16
ROOT_ENTRIES = 512
CLUSTER = SECTOR * SECTORS_PER_CLUSTER


def build_image(files):
    """files: list of (8.3 name, bytes, cluster list or None for contiguous)."""
    root_offset = (RESERVED + FAT_SECTORS) * SECTOR
    data_offset = root_offset + ROOT_ENTRIES * 32
    total_clusters = 64
    image = bytearray(data_offset + total_clusters * CLUSTER)
    boot = bytearray(SECTOR)
    struct.pack_into("<HBHBHHBH", boot, 11, SECTOR, SECTORS_PER_CLUSTER, RESERVED, 1,
                     ROOT_ENTRIES, 0, 0xF8, FAT_SECTORS)
    boot[43:54] = b"WITNESSLOGS"
    boot[510:512] = b"\x55\xaa"
    image[0:SECTOR] = boot
    fat = bytearray(FAT_SECTORS * SECTOR)
    struct.pack_into("<HH", fat, 0, 0xFFF8, 0xFFFF)
    next_free = 2
    entry_offset = root_offset
    # A volume label entry and a deleted entry first; both must be skipped.
    image[entry_offset:entry_offset + 11] = b"WITNESSLOGS"
    image[entry_offset + 11] = 0x08
    entry_offset += 32
    image[entry_offset] = 0xE5
    image[entry_offset + 11] = 0x20
    entry_offset += 32
    for name, content, chain in files:
        count = max(1, -(-len(content) // CLUSTER))
        if chain is None:
            chain = list(range(next_free, next_free + count))
        next_free = max(next_free, max(chain) + 1)
        for i, cluster in enumerate(chain):
            following = chain[i + 1] if i + 1 < len(chain) else 0xFFFF
            struct.pack_into("<H", fat, cluster * 2, following)
            piece = content[i * CLUSTER:(i + 1) * CLUSTER]
            start = data_offset + (cluster - 2) * CLUSTER
            image[start:start + len(piece)] = piece
        base, ext = name.split(".")
        entry = bytearray(32)
        entry[0:8] = base.ljust(8).encode()
        entry[8:11] = ext.ljust(3).encode()
        entry[11] = 0x21  # read-only, archive
        struct.pack_into("<H", entry, 26, chain[0])
        struct.pack_into("<I", entry, 28, len(content))
        image[entry_offset:entry_offset + 32] = entry
        entry_offset += 32
    image[RESERVED * SECTOR:RESERVED * SECTOR + len(fat)] = fat
    return bytes(image)


class Fat16ReaderTest(unittest.TestCase):
    def setUp(self):
        self.imu = bytes((i * 7) & 0xFF for i in range(3 * CLUSTER + 100))
        self.text = b"I (1) sensors: hello\n" * 50
        # Fragmented chain, out of order, to exercise run splitting.
        self.gps = bytes((i * 13 + 1) & 0xFF for i in range(2 * CLUSTER + 5))
        self.image = build_image([
            ("I0000012.BIN", self.imu, None),
            ("S0000012.TXT", self.text, None),
            ("U0000012.BIN", self.gps, [40, 30, 31]),
            ("S0000011.BIN", b"", [50]),
        ])
        self.fat = agent.Fat16(io.BytesIO(self.image))

    def test_lists_files_and_skips_label_and_deleted(self):
        entries = [(name, size) for name, _cluster, size in self.fat.entries()]
        self.assertEqual(entries, [
            ("I0000012.BIN", len(self.imu)),
            ("S0000012.TXT", len(self.text)),
            ("U0000012.BIN", len(self.gps)),
            ("S0000011.BIN", 0),
        ])

    def test_streams_contiguous_and_fragmented_files(self):
        for name, expected in (("I0000012.BIN", self.imu), ("s0000012.txt", self.text),
                               ("U0000012.BIN", self.gps), ("S0000011.BIN", b"")):
            out = io.BytesIO()
            self.fat.stream(name, out, chunk_clusters=2)
            self.assertEqual(out.getvalue(), expected, name)

    def test_missing_file_and_bad_volume(self):
        with self.assertRaises(agent.FatError):
            self.fat.stream("X.BIN", io.BytesIO())
        with self.assertRaises(agent.FatError):
            agent.Fat16(io.BytesIO(bytes(4096)))


if __name__ == "__main__":
    unittest.main()
