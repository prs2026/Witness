"""Reads the Witness read-only log disk on the bench Pi, without mounting it.

witness_download.py pipes this file to ``python3 -`` over SSH, so nothing is
installed on the Pi. It only needs read access to the block device (see
``witness_download.py setup``). Standard library only.

Commands (argv after ``-``):
    find            print the Witness disk device path, or exit 3
    list            print the root directory as JSON lines {"name","size"}
    cat NAME        write the file's bytes to stdout

The disk is the FAT16 view Witness synthesizes in mass-storage mode (label
WITNESSLOGS, no partition table, 8.3 names in the root directory).
"""

from __future__ import annotations

import glob
import json
import os
import struct
import sys

# Witness MSC: VID 303a PID 4012, product "Witness Read-Only Logs".
BY_ID_PATTERNS = ("/dev/disk/by-id/usb-Witness_*",)
MODEL_NAMES = ("READ-ONLY LOGS", "Read-Only Logs")


class FatError(Exception):
    pass


class Fat16:
    def __init__(self, device):
        self.device = device
        boot = self._read(0, 512)
        if boot[510:512] != b"\x55\xaa":
            raise FatError("no FAT boot signature")
        (self.bytes_per_sector, self.sectors_per_cluster, reserved, fat_count,
         root_entries, total16, _media, fat_sectors) = struct.unpack_from(
            "<HBHBHHBH", boot, 11)
        if self.bytes_per_sector == 0 or self.sectors_per_cluster == 0 or fat_sectors == 0:
            raise FatError("not a FAT12/16 volume")
        self.fat_offset = reserved * self.bytes_per_sector
        self.root_offset = self.fat_offset + fat_count * fat_sectors * self.bytes_per_sector
        self.root_bytes = root_entries * 32
        self.data_offset = self.root_offset + self.root_bytes
        self.cluster_bytes = self.bytes_per_sector * self.sectors_per_cluster
        self.fat = self._read(self.fat_offset, fat_sectors * self.bytes_per_sector)

    def _read(self, offset, length):
        self.device.seek(offset)
        data = bytearray()
        while len(data) < length:
            chunk = self.device.read(length - len(data))
            if not chunk:
                raise FatError(f"short read at {offset + len(data)}")
            data += chunk
        return bytes(data)

    def entries(self):
        root = self._read(self.root_offset, self.root_bytes)
        for i in range(0, len(root), 32):
            entry = root[i:i + 32]
            first = entry[0]
            if first == 0x00:
                break
            attributes = entry[11]
            if first == 0xE5 or attributes == 0x0F or attributes & 0x18:
                continue  # deleted, long-name part, volume label or directory
            base = entry[0:8].decode("ascii", "replace").rstrip()
            ext = entry[8:11].decode("ascii", "replace").rstrip()
            name = f"{base}.{ext}" if ext else base
            cluster = struct.unpack_from("<H", entry, 26)[0]
            size = struct.unpack_from("<I", entry, 28)[0]
            yield name, cluster, size

    def find(self, name):
        for entry_name, cluster, size in self.entries():
            if entry_name.upper() == name.upper():
                return cluster, size
        raise FatError(f"{name} not found")

    def clusters(self, first):
        cluster = first
        seen = 0
        limit = len(self.fat) // 2
        while 2 <= cluster < 0xFFF8:
            yield cluster
            seen += 1
            if seen > limit:
                raise FatError("cluster chain loops")
            cluster = struct.unpack_from("<H", self.fat, cluster * 2)[0]

    def stream(self, name, out, chunk_clusters=64):
        """Writes the file to out, reading contiguous cluster runs together."""
        first, size = self.find(name)
        remaining = size
        run_start = None
        run_length = 0

        def flush():
            nonlocal remaining
            if run_start is None or remaining <= 0:
                return
            offset = self.data_offset + (run_start - 2) * self.cluster_bytes
            length = min(run_length * self.cluster_bytes, remaining)
            out.write(self._read(offset, length))
            remaining -= length

        for cluster in self.clusters(first):
            if run_start is not None and cluster == run_start + run_length and run_length < chunk_clusters:
                run_length += 1
                continue
            flush()
            run_start, run_length = cluster, 1
        flush()
        if remaining > 0:
            raise FatError(f"{name}: cluster chain ends {remaining} bytes early")


def find_device():
    for pattern in BY_ID_PATTERNS:
        for path in sorted(glob.glob(pattern)):
            if "-part" not in path:
                return os.path.realpath(path)
    for block in sorted(glob.glob("/sys/block/sd*")):
        try:
            with open(os.path.join(block, "device", "model")) as model_file:
                model = model_file.read().strip()
        except OSError:
            continue
        if model in MODEL_NAMES:
            return "/dev/" + os.path.basename(block)
    return None


def main(argv):
    if not argv:
        print(__doc__, file=sys.stderr)
        return 2
    device_path = find_device()
    if device_path is None:
        print("Witness log disk not found (is Witness in export mode?)", file=sys.stderr)
        return 3
    if argv[0] == "find":
        print(device_path)
        return 0
    try:
        device = open(device_path, "rb", buffering=0)
    except PermissionError:
        print(f"no read permission on {device_path}; run: witness_download.py setup",
              file=sys.stderr)
        return 4
    with device:
        fat = Fat16(device)
        if argv[0] == "list":
            for name, _cluster, size in fat.entries():
                print(json.dumps({"name": name, "size": size}))
            return 0
        if argv[0] == "cat" and len(argv) == 2:
            fat.stream(argv[1], sys.stdout.buffer)
            sys.stdout.buffer.flush()
            return 0
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except FatError as error:
        print(f"FAT error: {error}", file=sys.stderr)
        sys.exit(5)
