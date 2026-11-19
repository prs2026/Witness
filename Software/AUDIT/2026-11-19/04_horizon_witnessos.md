# Audit 04: Horizon and witnessOS ground software (2026-11-19)

Scope: commits 0d3a403 (imperial units, gyro readout, tile cache, 2 Hz telemetry) and a06d14d (Wi-Fi / OTA box),
plus a crash / freeze / logging review of Horizon and the witnessOS monitor. Read-only; no hardware touched.
Paths are under Software/Horizon unless noted (witnessOS embeds Horizon's `main.MonitorPanel` and `monitor_core.py`).

## Findings

### High

**H1. Witness tab shows raw SI values under an "ft" label (CONFIRMED)**
- `ground_tabs.py:113-115` (WitnessTab note) says "Units: ... ft, ft/s (shown converted from m, m/s)", "position/velocity ... (ft, ft/s)",
  "gps_altitude is ft MSL".
- The tab fills its table via `telemetry_model.py:179-216` (`TelemetryTable.update_record`), which does `json.dumps(value)` of the decoded
  feed value. The feed is SI by design (flight_data, jsonl logs, `ground_receivers.py:71,115`). Nothing there calls `format_display_value`
  or `display_scale`; their only users are `monitor_core.update_values` and `main.GraphPlot`.
- Scenario: operator reads `kalman_altitude` 1000 on the Witness tab, believes 1000 ft, true value is 1000 m (3281 ft). This is exactly the
  "SI value with an ft label" case.
- Fix: either convert in `update_record` for `DISPLAY_LENGTH_FIELDS` (element-wise for lists, display string only) or revert the note to
  "m, m/s (SI; Flight View and the monitor show ft)". The one-line note change is the safe pre-flight fix.

### Medium

**M1. Debug log PACKET lines now carry feet with no unit (CONFIRMED)**
- `monitor_core.py:1467` writes `PACKET[port] <format_packet(packet)>` to the console and the on-disk debug log;
  `witness_protocol.py:660-665` builds that text with `format_display_value`, so `position`, `velocity`, `gps_altitude`, `kalman_altitude`
  are feet / ft/s but the line has no unit (`kalman_altitude=3280.84`). Older debug logs hold metres in the same format.
- Raw `.bin`, the JSONL feed log and NMEA CSVs stay SI (checked), so no data is lost; the risk is misreading post-flight text logs
  or mixing old and new logs.
- Fix: have `format_packet` append the unit (`kalman_altitude=3280.84 ft`) or keep the debug log SI and convert only the on-screen console.

**M2. OSM tile worker thread dies on a non-OSError network failure and never restarts (PLAUSIBLE)**
- `map_tiles.py` `fetch_now` catches `HTTPError`, `OSError`, `ValueError` only. `http.client.IncompleteRead` / `BadStatusLine`
  (`HTTPException`, not `OSError`) from a truncated tile response propagates out of `_run`; the thread ends, but `self._thread` stays non-None
  (`_enqueue`), so no new worker is started and the key stays in `_queued`.
- Impact: map tiles stop loading for the rest of the session (grid still works, telemetry unaffected). Likely on flaky mobile data at a field.
- Fix: catch `Exception` in `fetch_now` (treat as offline backoff), and wrap the `_run` loop body in try/except.

### Low

**L1. Secrets/addresses in public source (CONFIRMED)**: `wifi_ota_box.py` NOTE (lines 26-29) embeds the Witness AP password and AP
address as user-facing text; the repo is public. If that password is the one burned into firmware, rotate it or move the text to local
config (`witness_ota.py` has the same defaults). Not flight-affecting.

**L2. Wi-Fi ON gate ignores telemetry age (CONFIRMED, mitigated)**: `monitor_core.py:517-524` `witness_flight_state()` uses the newest status
word ever seen with no age limit, and `None` (no telemetry) enables the button. If the link drops on the pad and the rocket launches, the
button stays enabled. Safe in practice: firmware refuses Wi-Fi ON outside Pad Idle/Landed
(`Software/Witness/softwarev1/main/usb_serial_echo.cpp` `handle_wifi`, ACK_REFUSED) and the GUI asks for confirmation first
(`monitor_core.py:706-713`). Optional: treat a state older than ~10 s as unknown and say so in the confirm text.
The "Reboot Witness (HTTP)" button is not state-gated in the GUI (firmware and the confirm text only).

**L3. Monitor graph history shortened by 2 Hz (CONFIRMED, cosmetic)**: `monitor_core.py:237` `max_history = 300` samples = 150 s at 2 Hz
(was 300 s at 1 Hz; 30 s for 10 Hz USB packets). Raise to ~1200 if a longer monitor graph is wanted. Flight View has its own time-bounded history.

**L4. Legacy tile migration runs synchronously on the GUI thread (PLAUSIBLE)**: `map_tiles.py` `TileCache.__init__` ->
`migrate_legacy_cache` (rglob + `os.replace`, copy fallback across drives). One-time, at map creation, but thousands of tiles on a slow
disk freeze the first start. Run it once before flight day (open the map) rather than at the pad.

**L5. Latent unit mismatch for right-axis traces in flight readout rows (PLAUSIBLE, not in default layout)**:
`flight_render.flight_panel` / `status_rows` use `trace.unit or row.unit` (no right-axis lookup), while plots and legends use `trace_unit()`.
A custom layout with `right_unit` on a readout row would convert/label by the row unit while the plot uses the right unit. The default layout
(`flight_view_default.json`: "A GPS" on the right axis, unit "GPS raw") is consistent in plots and legends; no readout row uses a right axis.

**L6. Worker signals after widget deletion (PLAUSIBLE, shutdown only)**: `wifi_ota_box.py` `_start.run` emits `signals.done` in `finally`; if the
box is destroyed while a transfer runs, the emit raises RuntimeError on the daemon thread (caught by `threading.excepthook`, `main.py:115`). No flight effect.

**L7. Stale comments**: `attitude_data.py:6,23` still say 0x02 is 1 Hz and 0xE0 is 1 Hz over LoRa; `E0_FRESH_S = 2.5` still works at 2 Hz.

## Checked and OK
- Imperial display: each path converts once. `display_scale` (main.GraphPlot) scales plot data, and the title uses the formatted value plus unit
  (`main.py:254-259`); `format_display_value` is only for text; `flight_render` converts at draw time (`scale_series`, `value_text`, fixed
  `y_range` limits, axis labels; history values are Python floats per `flight_data.py` so the `type(value) in (int, float)` test holds);
  LANDED line `/ FT_M` (flight_render.py:316); map labels `m_to_ft`; pad altitude; rings and scale bar via `format_distance` (scale-bar suffix
  removed, text set explicitly, `visualizer.py:767-780`); map axes use `setScale(1/FT_M)` with SI scene coordinates; GGA summary converts once and labels "ft".
  Histories, feed, JSONL, raw `.bin`, NMEA CSV and layout JSON stay SI. Featherweight `altitude_ft` / `vertical_velocity_fps` go to SI once
  (scale 0.3048), then to ft once for display. `format_distance` boundary at 5280 ft and None/NaN are handled.
- Gyro graph/readout: `gyro[0..2]` from sensors/witness_debug exist in `SECTION_SPECS`; unit "deg/s" passes through unconverted.
- 2 Hz: all staleness thresholds (3 s readouts, 3 s link lost, 2/5/10 s age colours, 6 s OTA telemetry) are at least 1.5 packet periods at 2 Hz;
  nothing tuned to exactly 1 s. Camera stays 1 Hz (3 s threshold fine).
- Wi-Fi / OTA box: one operation at a time (`_busy`), worker only emits Qt signals (queued to the GUI thread), all slots use `guarded_slot`,
  worker catches `Exception`, 15 s HTTP timeout per request, `wait_for_reboot` bounded at 90 s, no HTTP on the GUI thread (only the small image-head
  read and file dialogs). Wi-Fi ON confirm text states flight logging OFF; core `confirm` defaults to False. Wi-Fi OFF sends the same acked
  command (no-op in the flight image). Click lambdas are not `guarded_slot`, but `install_global_excepthook` (`main.py:104-115`) prevents the PyQt6 abort.
- Logging survival: `RawLog.write` never raises, retries every 2 s, fsync on a separate thread (`link_health.py:61-130`, `monitor_core.py:926`);
  debug log writes wrapped in try/except; sys/threading excepthooks installed; timer slots use `guarded_slot`/`guard_call`.
- Memory: monitor history `RingHistory(300)`; flight histories time- and count-bounded (`flight_data.py`); map tracks `deque(maxlen)`;
  attitude history `deque(maxlen=6000)`; console `setMaximumBlockCount`; tile `_failed` grows only per failed tile.
- Tile pre-cache plan: 10 km z10-15 + 3 km z16 + 1.5 km z17 is roughly 1100 tiles at mid latitudes, capped at 1500 (`precache_area_plan`), low zoom
  first, 2 req/s (about 10 min). Cache under `%LOCALAPPDATA%`; migration never raises and keeps existing target tiles.

## Tests
- `QT_QPA_PLATFORM=offscreen`, Horizon venv, `python -m unittest discover -s tests` in Software/Horizon: Ran 534 tests, OK (91 s).
  (`pytest` is not installed in that venv, so unittest was used.) No source changes were made.
