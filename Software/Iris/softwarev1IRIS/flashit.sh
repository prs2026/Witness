#!/usr/bin/env sh
set -eu
cd "$(dirname "$0")"
python3 -m esptool -p "${1:-rfc2217://prs2026hitl:7000}" -b 460800 --before usb-reset --after hard-reset --chip esp32c3 write-flash --flash-mode dio --flash-freq 80m --flash-size 4MB 0x0 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/softwarev1.bin
