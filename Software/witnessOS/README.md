# witnessOS

The Witness bench tools in one PyQt6 window (plots by pyqtgraph), one tab per
tool. witnessOS is the only way to run the HITL bench GUI and the Witness
monitor; they have no standalone windows any more.

| Tab | Tool | Code |
|---|---|---|
| **HITL bench** | Streams simulated `0x0C` sensor frames to Witness and scores flight-state transitions and attitude | `wos_hitl_tab.py` (`HitlWidget`) on `Software/HITL`'s `hitl_session.py`, `sim_model.py`, `trajectory_csv.py`, `attitude.py`, `hitl_protocol.py`, `hitl_gui.py` (Qt-free helpers and the headless runner) |
| **CSV viewer** | Summary and plots of CSVs made by `converter.py`. Several CSVs can be overlaid on Witness uptime | `Software/Horizon/viewer.py` (`CsvViewerWidget`) |
| **Witness monitor** | The Witness Communications Monitor: telemetry readout, status decoders, graphs, console, commands, test source and the Horizon `horizon.witness.v1` JSON feed on `127.0.0.1:8766` | `wos_monitor_tab.py`, embedding `Software/Horizon/main.py`'s `MonitorPanel` (logic in `monitor_core.py`) |

## Run

You need Python 3.14 with PyQt6, pyqtgraph and numpy; the 960 Hz IMU
synthesis also needs scipy and soundfile:

```bash
python -m pip install -r Software/witnessOS/requirements.txt
```

```bash
python Software/witnessOS/witnessos.py
```

```bash
python Software/witnessOS/witnessos.py S0000434.csv P0000434.csv
```

```bash
python Software/witnessOS/witnessos.py --tab monitor --no-feed --no-auto-connect
```

| Option | Effect |
|---|---|
| CSV paths | Opened together in the viewer, overlaid on Witness uptime |
| `--tab hitl\|viewer\|monitor` | Picks the first tab. `relay` still works as an alias for `monitor` |
| `--no-feed` | Doesn't start the JSON feed |
| `--no-auto-connect` | Stops the monitor opening Witness USB ports automatically |
| `--hitl-headless [HITL args]` | Runs one HITL simulation without a window (bench automation); see below |

## HITL bench

The tab is the HITL bench GUI described in `Software/HITL/README.md`:
connection bar (host, port, Connect, Enter HITL, Start sim, Stop, Exit HITL,
tolerance), preset or trajectory CSV source, the parameter list (including
the vibration profile picker and scale), final tilt threshold, the FC log
fetch settings (bench Pi host, user, SSH key, hub power-cycle), Predict
transitions, the flight state and status lamps, the live readout (KF alt AGL,
GPS, accel X, battery, uptime, sim time), the transition table, and the log
with `[csv]`, `[imu]` and `[result]` lines (the result table plus the
observed-vs-truth comparison).

Plots are pyqtgraph: altitude MSL (sim true vs Witness Kalman), X accel (sim
low-g vs Witness) and, for CSV runs, tilt / attitude error. The x axes are
linked; drag to pan, wheel to zoom, right-click for the plot menu. Live data
is converted incrementally and redrawn by a throttle (about 7 Hz), never per
packet. CSV parsing, prediction, the 960 Hz IMU synthesis and connecting run
on worker threads with a busy indicator, so the window never freezes.
Settings persist in `Software/HITL/hitl_settings.json`.

**Headless.** For scripts and the bench, without a window and without
importing Qt:

```bash
python Software/witnessOS/witnessos.py --hitl-headless --preset "Nominal 10 g" --host prs2026hitl --port 7100
python Software/witnessOS/witnessos.py --hitl-headless --csv Software/HITL/trajectories/a3241569-trajectory.csv --dry-run
python Software/witnessOS/witnessos.py --hitl-headless --help
```

The arguments are the HITL runner's (see `Software/HITL/README.md`). The
exit code is 0 for a pass, 1 for a fail and 2 for a setup error.
`python Software/HITL/hitl_gui.py --headless ...` still works as a deprecated
alias.

## Witness monitor

This tab embeds Horizon's monitor widget (`main.MonitorPanel` over
`monitor_core.MonitorCore`). It shows:

- Data-age indicators.
- Status-byte flag, flight-state and GPS/HITL decoders.
- Selectable graphs.
- The console with hex and quick commands.
- The built-in test flight and JSON replay.

Its JSON feed is the one `horizon.py` and the ground receivers read. Only one
program can serve port 8766, so run one witnessOS at a time.

**Mirror HITL tab connection** is on by default. Witness's raw TCP port
(ser2net, for example `prs2026hitl:7100`) serves one client at a time. While
the HITL tab holds that connection, the monitor decodes the bytes the HITL
session receives (`HitlSession.rx_tap` → `MonitorPanel.stream_feeder`) instead
of opening a second connection. Mirroring only reads.

## CSV viewer: overlaying logs

1. **Open CSV…** replaces the loaded files. **Add CSV…** adds more. Both
   accept several files at once.
2. The trace list groups numeric columns under their CSV.
3. To plot or overlay a trace:
   - Drag it onto a graph to overlay it there.
   - Drag it onto the dashed area below the last graph to start a new graph.
   - Double-click it to add it as a new graph.
   - Select several traces, then drag them together.
4. To rearrange a graph:
   - Drag a legend entry onto another graph to move that trace there.
   - Drag it below the last graph to give it its own graph.
   - Drag it back onto the list to remove it.
   - **✕** closes a graph. **Clear graphs** empties the plot.
5. Overlaid traces get one colour per trace and one dash pattern per CSV.
6. **Plot CSV** plots every column of the selected CSV, one graph each. The
   first CSV you open does this automatically. **Remove CSV** removes the
   selected CSV.

**Raw .bin logs.** **Open .BIN…** decodes Witness logs directly and adds them
alongside the loaded files. It uses `converter.py`'s own decoders:
- `I*.BIN` as IMU records.
- `P*.BIN` as the packet log.
- `S*.BIN` (WL magic) as flash records.
- Anything else through the wire packet scanner.

Decoding runs on the worker thread, and rows become numpy columns as they are
decoded. The columns, the paired `.TXT` flight state and the default time
axis are the same as for the converted CSV. A file nothing decodes from gets
a clear error. **Save CSV** writes the selected decoded log next to it, with
exactly what `converter.py` would write. A 2.1M-record (51 MB) `I*.BIN`
decodes in about 28 s in the background, holding about 400 MB.

**Large files.** Converted flash logs can have millions of rows (`I*.csv` is
960 Hz). The viewer:
- Reads files on a worker thread, with a progress bar and **Cancel** in the
  toolbar, so the window stays usable.
- Keeps each numeric column as one float64 numpy array (`csv_columns.py`
  parses ~4 MB chunks with numpy's C reader). Text columns other than
  `packet_type`/`packet_id` are skipped.
- Computes derived columns and the summary on those arrays.
- Above 1 GiB of column data, loads every Nth row and says so on the Summary
  tab.
- Draws only the visible span of each trace, as a min/max envelope per pixel,
  recomputed on every zoom.

On a 139 MB, 536k-row I log, memory held went from 1.5 GB to 104 MB, and the
UI freeze when opening it went from about 36 s to 0.26 s. A 557 MB,
2.1M-row file loads in about 12 s in the background, holding 403 MB.

**Per-file time axes.** The default x axis is **time [s] (per-file time
axes)**: each CSV is placed by its own time column, converted to seconds, plus
its own offset. Select a CSV in the list to edit its settings in the **Time
axis** box. The plots redraw as you change them.

- **Column**: which column is time. **auto (Witness uptime)** uses `uptime`,
  then `fc_uptime` (camera packets), then `time_seconds`, row by row.
- **Units**: s, ms, us or ns. Choosing a column picks the units its name
  implies; correct them here if that guess is wrong.
- **Offset s**: added after unit conversion, so one file can be slid against
  the others. Type a value (applied when you pause or press Enter), use the
  spin arrows (0.1 s), or the −1 / −0.1 / +0.1 / +1 buttons.
- The note under the controls shows where the file now sits on the shared
  axis. The list shows each file's setting, for example
  `truth.csv [sim_s s, +674.401 s]`.
- Settings stay with each file while it's loaded.

Defaults put Witness-clock files on the same clock, so the flash logs and the
HITL comparison files line up without any offset:

| File | Default |
|---|---|
| `P*.csv`, `S*.csv` and packet logs (have `uptime`) | auto |
| `comparison*.csv` | `uptime_ms` (ms) |
| `I*.csv` | `timestamp_us` (µs) |
| `truth.csv` | `sim_s` (s) |
| `sent.csv` | `sim_ms` (ms) |

Sim-time files need one offset to join the Witness clock. In a HITL run that
offset is `comparison.csv`'s first `uptime_ms`/1000 minus its first `sim_s`.

You can also pick any single numeric column as a shared x axis instead.

**Zoom.** All graphs share one x axis, and zooming any graph zooms them all.
Each graph's y axis refits to the data in view, and its statistics then read
"in view".

- Drag across a plot to zoom to that span. The band shows on every graph.
- **Ctrl+wheel** zooms in or out around the cursor.
- **Shift+wheel** pans when zoomed in.
- Double-click a plot, or press **Reset zoom**, to show everything.
  **Zoom out** doubles the span.
- Changing the x axis resets the zoom.

**Hover.** Hold the pointer over a plot to get a dashed crosshair at that x
in every graph, with a ring on each trace's nearest sample. A box next to the
pointer lists the exact values from the CSV, for example
`S0000010.csv: barometric_altitude = 245.9904175  @ 31.788` (the value, then
that sample's x). A trace whose log has ended, or hasn't started yet, at that
time is left out of the box.

The Summary tab shows one CSV at a time; pick it from the CSV list.

## Window sizes

- The window opens at up to 1320 x 900 (less on small screens); the
  minimum is 420 x 300.
- Each tab sits in a `QScrollArea` (`wos_widgets.scroll_tab`). When the
  window is larger than the tab needs, the tab stretches to fill it; when it
  is smaller, scrollbars appear and nothing is squashed or clipped. Qt sends
  the mouse wheel to the innermost scrollable widget under the pointer.
- The window title follows the tab: the viewer's open files, the monitor's
  connection.
- Closing the window shuts down all three panels: the monitor (mirroring,
  inputs, feed), the HITL bench (stops a run, sends HITL EXIT) and the viewer
  (loader threads).

## Files

| File | Contents |
|---|---|
| `witnessos.py` | The entry point and CLI, including `--hitl-headless`. Qt-free until the window opens |
| `wos_app.py` | The `WitnessOS` main window (`QTabWidget`) |
| `wos_hitl_tab.py` | The HITL bench tab, `HitlWidget` |
| `wos_monitor_tab.py` | The monitor tab: `main.MonitorPanel` plus HITL mirroring |
| `wos_widgets.py` | `scroll_tab()` |
| `wos_paths.py` | Adds `Software/HITL` and `Software/Horizon` to `sys.path` |

## Tests

```bash
cd Software/witnessOS && python -B -m unittest discover -s tests -q
```

- `test_app.py`: the shell with stand-in viewer/monitor widgets, plus an
  integration test with the real Horizon widgets (skipped until they are Qt).
- `test_hitl_tab.py`: the HITL bench tab, including a run against
  `Software/HITL/tests/fake_witness.py`.
- `test_cli.py`: `--hitl-headless` runs and never imports Qt.

GUI tests run offscreen (`QT_QPA_PLATFORM=offscreen`). Related tests are in
`Software/HITL/tests` and `Software/Horizon/tests`.
