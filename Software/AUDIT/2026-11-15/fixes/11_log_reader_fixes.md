# Fix report 11: flight-log readers after the logger fixes (fix 01)

Scope: readers of the on-NAND log (the `.BIN` files of the maintenance-image USB disk) updated for the resumed-flight marker, restarting clocks, 32-bit wrap, unknown record types and missing pages. Firmware not touched. Nothing committed. Only synthetic logs exercise the new paths: no log with a resume, an unknown source or a gap exists in the repo or on disk, and nothing was run on hardware.

## Readers found (before any change)

Searched Software/Horizon, witnessOS, HITL and Witness (incl. tools/).

| Reader / consumer | What it does | Changed |
|---|---|---|
| `Horizon/converter.py: iter_packet_log` (P*.BIN), `iter_imu_log` (I*.BIN), `iter_flash_log` (S*.BIN WL v1/v2), `select_decoder`, `main` | The only decoder of the three NAND streams. Everything else calls it | yes (rewritten decoders, see below) |
| `Horizon/viewer_data.py: load_bin_dataset`, `_estimate_bin`, `save_bin_as_csv` | Viewer "Open .BIN" and "save as CSV": call `converter.select_decoder` | `load_bin_dataset` + `Dataset.decode_notes` and the file-summary details line show resume/skip/gap notes. `_estimate_bin`, `save_bin_as_csv` need nothing |
| `Horizon/witness_download.py: convert` | Runs `converter.py` on each downloaded file | prints the converter's resume/skip/gap lines |
| `HITL/hitl_outputs.py: write_flash_comparison` (via `fetch_fc_logs`, `regenerate`) | Reads the converted `P*.csv`, maps device `uptime` to sim time with one offset | yes: rows with a `segment` after a reset are not compared (their uptime restarted), and the count is logged |
| `HITL/hitl_report.py: kalman_rows` | `converter.iter_packet_log` for the 0xE2 rows; uses `timestamp_us` with a device-uptime fit | yes: rows of resumed segments are skipped (they are not on the fitted clock) |
| `Horizon/viewer.py`, `csv_columns.py`, `any_csv.py`, `flight_data.py`, `attitude_data.py` | Read converter CSVs / live data; no direct .BIN parsing | no (new columns are plain numeric/text columns; verified via the viewer test) |
| `Horizon/viewer_data.py: _wl_transitions` / `add_paired_wl_flight_state` | Merges S*.TXT state transitions into WL v1 rows by device uptime | no (WL v1 only; see Not done) |
| witnessOS (`witnessos.py`, `wos_*.py`) | HITL tab only starts the HITL fetch (`hitl_outputs`); no log parsing | no |
| Witness/softwarev1 (`tools/imu_calibrate.py`, `kalman_replay.cpp`) | No NAND-log reading | no |
| `HITL/kalman_replay.py`, `hitl_gui.py`, `hitl_session.py` | Use `comparison_flash.csv` only | no |

## Record layouts confirmed in the firmware (flash_logger.h/.cpp)

- Page: 12-byte header (magic `WLPG`, type, version, u16 length, u32 sequence with a 12-bit session tag in the top bits) + up to **2036** payload bytes (`kPagePayloadSize`). The export (`scan_file`/`load_export_page`/`read_file_data`) returns the payloads concatenated with no page headers and no gap placeholders; unreadable/untagged pages are skipped (`continue`). So a reader cannot see a missing page directly.
- Packet record (`log_packet`): `[u32 us BE][u8 source][u8 length][frame]`, length <= 103. Sources 1-5 carry a whole wire frame ending in 0x0A (`kPacketSourceState/CanRx/CanTx/RadioRx/Kalman`). Source 6 (`kPacketSourceLogMarker`), written once in `start()` after a resume: frame = `[u8 kind=1 (resumed)][u8 flight state][u32 BE session generation]`, 6 bytes, no 0x0A terminator, timestamp = the new boot's esp_timer. **Matches the fix report exactly** (`[u32 us][6][len 6][kind 1][state][u32 generation]`).
- IMU record (`make_imu_record`/`log_imu_record`): 24 bytes BE `u32 us, 9 x int16, u8 status (state in bits 0-2), u8 sequence` (the sequence is assigned before the Descent 1-in-10 decimation, so it steps by 1, and by 10 in Descent). No framing: the byte stream is whole records, but pages split records (2036 mod 24 = 20).
- WL v2 (`serialize_record`): 24 bytes BE `"WL", 2, status0, status1 (state), reserved, u32 sequence, u32 uptime ms, ...`; v1 is 48 bytes.
- Differences from the fix report: none in the marker. Two statements there were not true of the old reader: "readers that skip unknown sources by their length byte are unaffected" (the old `iter_packet_log` stopped at the first unknown source and discarded the rest of the file), and "the reader skips" gaps (the export hides them; the IMU and WL streams then end up misaligned, see below). The report's "IMU u8 sequence and WL u32 sequence restart at 0" is right (clock and sequences restart after boot); it does not say that the packet stream has no sequence at all, so there is nothing to compare there.

## What changed in `converter.py`

- `DecodeStats` keeps `discarded` and gains `gap_events`, `unknown_records`, `unknown_sources`, `segments`, `resumes` (list of dicts), `time_wraps`; `describe_stats()` turns them into report lines, printed by `main()` after the existing four lines (old logs print nothing extra).
- `_Clock` (unwrap of one segment, the previous algorithm moved unchanged plus `restarted()`), `_Axis` (chains segments).
- `iter_packet_log` is built on `_walk_packet_records`: source 6 kind 1 -> a `log_resumed` row (`packet_id` LOG, `log_source` log_marker, `resumed`=1, `resume_flight_state`(+`_name`), `resume_generation`, `resume_gap_s`, `resume_gap_source`) and a segment boundary; a source-6 record of any other kind is skipped and counted (`6:kind N`); any unknown source 0/7-255 is skipped by its length byte and counted. It is believed only when the next records are intact (end of data, or a known/marker record within 4 records); otherwise the decoder re-synchronises.
- Re-synchronisation (`_packet_resync`): scans for the next offset where a source 1-6 record starts and two of the next three records are known/marker records (or the data ends). Bytes skipped go to `discarded`, `gap_events` counts them, the first row after has `gap_before_bytes`. With no such offset the rest is discarded, as before (the old truncated-tail behaviour and test are unchanged).
- `iter_imu_log`: no marker exists in this stream. A pair of records is consistent when the timestamp step is -50..+250 ms and the sequence step 1..16 (real logs: 1 ms / 10 ms and 1 / 10). When the next record does not fit, the four records from there are checked; if they do not chain either but a position 1..47 bytes further does, that shift is skipped (a missing page leaves an offset of 20 bytes per 2036 plus the torn records). Otherwise it is a real discontinuity (dropped records, reset) and decoding goes on exactly as before. A step back of more than 2 s that is not a wrap starts a new segment: `resumed`=1 on that row, `missing_before` = 0 there.
- `iter_flash_log` (WL): a record is accepted when it parses and the next record also starts with the magic+version; otherwise the decoder searches for the next position where three records in a row start with it, and skips to it (a record cut by a missing page is dropped). An uptime step back of more than 2 s is a new segment. With nothing to find it steps by the record size as before.
- `main()` prints the new report lines. `import re` added.

## Time axis across a resume

Per segment the u32 microsecond counter is unwrapped (a step back of > 2^31 that continues forward within 300 s is a wrap; the existing "straggler" handling is unchanged). The first segment is exactly the old axis. At a reset the restarted clock's zero is placed at `last_axis_time + gap`, where `last_axis_time` is the latest time seen so far, so a record at device time `t` after the reset is at `last + gap + t` (the `log_resumed` row sits at its own device time, typically seconds after boot). `gap` is:

- packet log, GPS UTC present on both sides (Kalman 0xE2 `gps_time`, last pre-reset value and first post-reset value): `UTC(clock zero after) - UTC(last pre-reset record)`, accepted only within -2 s..12 h (clamped at 0), `resume_gap_source` = `gps_utc`, `time_after_reset_estimated` = 0. The packet list is read once more (`resume_gaps_from_gps`) only when a marker is found.
- otherwise nominal 1.0 s: `resume_gap_source` = `nominal`, `time_after_reset_estimated` = 1 on every row of the segment. Always the case for I*.BIN and S*.BIN (they are converted alone). A marker with nothing before it gives `no_previous_data` and no offset.

The real reset duration (plus the up to 1 s of lost RAM buffer) is unknown, so after a nominal resume `time_seconds` is a lower bound. `timestamp_us` is the continuous axis; the raw device clock is `device_timestamp_us`. Sequence restarts are not reported as dropped packets (`missing_before` = 0 on the reset record; WL and the packet log have no drop counter). The new columns (`segment`, `time_after_reset_estimated`, `device_timestamp_us`, `resumed`, `resume_*`, `gap_before_bytes`) appear only on rows after a reset / gap, so the header of a normal log is unchanged.

Viewer: `Dataset.decode_notes` shows the report lines in the file summary; the viewer prefers `timestamp_us` as the time column, which is the continuous axis.

## Backward compatibility

Corpus: all 100 real `.BIN` files in the repo (33 I*, 31 P*, 36 S*; 5 Horizon/logs S* and 95 from HITL/runs/*/fc_logs and witnessOS/*/fc_logs; 1.8 GB of CSV) were converted with a copy of the working-tree `converter.py` taken before any edit and with the new one. All 100 CSVs are byte-identical (sha256) and the CLI output differs only in the output directory I used. None of them produced a resume, unknown-record or gap line. (The first comparison pass was also done with `cmp` and showed no CSV difference. My scratch copies of the outputs were deleted afterwards; the recipe is in this section.)

## Tests

- New `Horizon/tests/test_log_resume.py`, 27 tests, bytes built per the firmware layouts: normal session (packets, IMU, WL: no new columns); resume marker (marker row fields, monotonic time, nominal and GPS-derived gap, stale GPS falls back, two resets, marker first in file, unknown marker kind, IMU and WL restarts, sequence restart not "missing"); unknown sources (skipped, counted, no desync, zero-length at the end); u32 wrap (packets, IMU, restart after a wrap, the indistinguishable case); missing page (IMU, IMU plus reset, packets, packets plus resume, WL, truncated tail); CSV header; viewer `load_bin_dataset` and the converter CLI on a resumed log.
- New `HITL/tests/test_flash_comparison_resume.py`, 1 test.
- Counts: Horizon 431 -> 458 pass; HITL 94 -> 95 run, same single error (`test_report_from_a_saved_run_layout`, no matplotlib); witnessOS 19 pass (unchanged). The existing `test_packet_log.py` and `test_converter_wrap.py` pass unchanged.

## Not done / limits

- No real resumed log was available (the firmware is not flashed); the GPS gap path uses the field offsets from the 0xE2 layout and has only synthetic data. If the Kalman log's `gps_time` holds the last UTC rather than advancing, the sanity window rejects most cases and the nominal gap is used, but a stale value that happens to fit would be taken.
- I*.BIN and S*.BIN gaps are always nominal even when a P*.BIN of the same session has a GPS gap; the files are converted independently, so after a reset the axes of the three files of one session can differ by the gap error. Sharing the gap through a sibling P file was not built.
- A reset in the last 5 minutes before the 71.6-minute counter wrap is read as a wrap (in the IMU/WL restart rule); the packet stream is not affected because it has the marker.
- An IMU page missing at an exact record boundary (a partial last page of a chunk) cannot be seen except as a jump in `missing_before`/time; so a missing page in a stream whose pages are all whole records is not "realigned" (nothing to realign).
- `HITL hitl_outputs.write_flash_comparison` skips rows after a reset instead of re-aligning them to sim time (the sim-time offset across the unknown gap is not known); `hitl_report` skips the Kalman rows after a reset.
- WL v1 paired state transitions from `S*.TXT` (`viewer_data._wl_transitions`) are matched by device uptime and are not segment-aware (v2 logs carry the state themselves). The text log and the raw GPS `U*.BIN` ring are not decoded by any reader.
- A run of 5 or more consecutive records of unknown sources with no known record near them would be treated as damage and re-synchronised over (documented in code).
- Not measured: decode speed on large files (the packet decoder now makes two passes only when a marker exists; otherwise one pass with a function call per record).
