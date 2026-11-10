# witnessOS

One Tkinter window for the Witness bench tools, one tab per tool:

| Tab | Tool | Reused from |
|---|---|---|
| **HITL bench** | Streams simulated `0x0C` sensor frames to Witness and scores flight-state transitions and attitude | `Software/HITL/hitl_gui.py` (`HitlApp`), `hitl_session.py`, `sim_model.py`, `trajectory_csv.py`, `attitude.py`, `hitl_protocol.py` |
| **CSV viewer** | Summary and plots of CSVs made by `converter.py`. Several CSVs can be overlaid on Witness uptime | `Software/Horizon/viewer.py` (`CsvViewerPanel`) |
| **Witness monitor** | The Witness Communications Monitor: telemetry readout, status decoders, graphs, console, commands, test source and the Horizon `horizon.witness.v1` JSON feed on `127.0.0.1:8766` | `Software/Horizon/main.py` (`MonitorPanel`) |

The tools are imported, not copied. Their standalone entry points still work:
`hitl_gui.py` (GUI and `--headless`), `viewer.py`, `main.py` and `converter.py`.

## Run

You need Python 3.14 with Tk. For USB serial ports, install Horizon's
requirements (`pip install -r Software/Horizon/requirements.txt`).

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

## Witness monitor

This tab is `main.py`'s monitor, embedded. It shows the same per-packet
sections as `python main.py`:

- Data-age indicators.
- Status-byte flag, flight-state and GPS/HITL decoders.
- Selectable graphs.
- The console with hex and quick commands.
- The built-in test flight and JSON replay.

Its JSON feed is the one `horizon.py` and the ground receivers read. Run
either witnessOS or `main.py`, not both, because only one of them can serve
port 8766.

**Mirror HITL tab connection** is on by default. Witness's raw TCP port
(ser2net, for example `prs2026hitl:7100`) serves one client at a time. While
the HITL tab holds that connection, the monitor decodes the bytes the HITL
session receives (`HitlSession.rx_tap` → `MonitorBase.stream_feeder`) instead
of opening a second connection. Mirroring only reads.

Rendering fixes compared with standalone `main.py` before this change. They
apply to `python main.py` too:

- The Graphs tab has a scrollbar and responds to the mouse wheel. Before,
  graphs past the window bottom couldn't be reached.
- Graph height fits the view. Values get round ticks scaled to the space.
  Wall-clock time ticks no longer overlap. Vector fields get an x/y/z(/w)
  legend.
- Graphs redraw on resize (debounced), skip drawing while hidden, and reduce
  long histories to min/max per pixel.
- Telemetry sections tile by their real width, so the status-flag grids are
  no longer clipped.
- The wheel binds per widget instead of through `bind_all`, so it can't take
  over the host window's wheel.

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

## Window sizes and rendering

- Each tab sits in a `ScrollableFrame` (`wos_widgets.py`). When the window
  is larger than the content needs, the content stretches to fill it. When
  the window is smaller, scrollbars appear and nothing is squashed or clipped
  out of reach.
- One `WheelRouter` sends the mouse wheel to the innermost scroll area under
  the pointer that can still move, and then to the whole tab. Text boxes and
  tables keep their own scrolling.
- **Viewer plots:**
  - Graphs redraw on resize at the real width.
  - Ticks are round values, and overlapping labels are skipped. Margins are
    measured to fit the labels.
  - Statistics and legends wrap instead of overdrawing.
  - Long logs are reduced to min/max per pixel.
  - Only graphs on screen are drawn.
- **HITL plots:**
  - Panels have round y and time ticks.
  - Legends wrap.
  - Data is clipped to the plot.
  - Panels redraw on resize and skip work while hidden.
  - Plot and log/table are split by a draggable sash.

## Files

| File | Contents |
|---|---|
| `witnessos.py` | The entry point: the `WitnessOS` notebook and the CLI |
| `wos_paths.py` | Adds `Software/HITL` and `Software/Horizon` to `sys.path` |
| `wos_widgets.py` | `ScrollableFrame`, `WheelRouter` and the pure layout helpers |
| `wos_monitor_tab.py` | The monitor tab: `main.MonitorPanel` plus HITL mirroring |

## Tests

```bash
cd Software/witnessOS && python -B -m unittest discover -s tests -q
```

Related tests are in `Software/HITL/tests/test_gui_plot.py` and
`Software/Horizon/tests/test_viewer_plotting.py`.
