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

**Syncing.** The default x axis is **uptime [s] (synced)**: every CSV is placed
on Witness `uptime` (ms ÷ 1000), or on `fc_uptime` for camera packets. Logs
from the same power-on line up exactly. If a CSV has no uptime (for example
`I*.BIN` IMU records, whose `time_seconds` counts from the first record), it
falls back to `time_seconds`, and the list marks it `[no uptime]`. To align
such a CSV, or logs from different boots, select it and set **Time offset
s**. Any numeric column can be chosen as the x axis instead.

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
