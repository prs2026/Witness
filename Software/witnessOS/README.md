# witnessOS

One Tkinter window for the Witness bench tools, one tab per tool:

| Tab | Tool | Reused from |
|---|---|---|
| **HITL bench** | Streams simulated `0x0C` sensor frames to Witness and scores flight-state transitions and attitude | `Software/HITL/hitl_gui.py` (`HitlApp`), `hitl_session.py`, `sim_model.py`, `trajectory_csv.py`, `attitude.py`, `hitl_protocol.py` |
| **CSV viewer** | Summary and per-column plots of CSVs made by `converter.py` | `Software/Horizon/viewer.py` (`CsvViewerPanel`) |
| **Witness relay** | Decodes a Witness byte stream and serves it as the Horizon `horizon.witness.v1` JSON feed on `127.0.0.1:8766` | `Horizon/main.py` `PacketDecoder` and `Horizon/witness_link.py` `WitnessPublisher` |

The tools are imported, not copied. Their standalone entry points still work:
`hitl_gui.py` (GUI and `--headless`), `viewer.py`, `main.py` and `converter.py`.

## Run

Python 3.14 with Tk. No extra packages are needed. USB serial input on the
relay tab needs `pyserial`; without it the option is disabled.

```bash
python Software/witnessOS/witnessos.py
```

```bash
python Software/witnessOS/witnessos.py path/to/S0000434.csv
```

```bash
python Software/witnessOS/witnessos.py --tab relay --no-feed
```

`--tab hitl|viewer|relay` picks the first tab. `--no-feed` stops the relay
from starting the JSON feed at launch.

## Witness relay

Inputs:

- **Raw TCP**, for example the bench Pi's ser2net port `prs2026hitl:7100`.
- **USB serial**, which needs pyserial.
- **Share HITL tab connection.** Witness's raw TCP port serves one client
  at a time, so while the HITL tab is connected the relay mirrors the bytes
  that connection receives (`HitlSession.rx_tap`). It does not open a second
  connection.

Every decoded packet is published through `WitnessPublisher`, the same feed
that `main.py` serves, so `horizon.py` and the ground receivers can consume
it unchanged. If `main.py` is already serving port 8766, the tab reports that
the port is in use. Pick another port, or stop one of the two. "Record raw
.bin" writes the received bytes to `Horizon/logs/<source>_<time>.bin`, the
same naming `main.py` uses, so `converter.py` can read them. The relay never
transmits to Witness. HITL commands stay on the HITL tab.

## Window sizes and rendering

- Each tab sits in a `ScrollableFrame` (`wos_widgets.py`). When the window
  is larger than the content needs, the content stretches to fill it. When
  the window is smaller, scrollbars appear and nothing is squashed or clipped
  out of reach.
- One `WheelRouter` sends the mouse wheel to the innermost scroll area under
  the pointer that can still move: the HITL parameter list, the viewer
  graphs and trace list, and then the whole tab. Text boxes and tables keep
  their own scrolling.
- **Viewer plots** (`viewer.py`):
  - Graphs redraw on resize at the real canvas width. The old fixed 650 px
    floor clipped narrow windows.
  - Graph height fills the view when only a few traces are shown, and the
    view scrolls when there are many.
  - Y-axis and x-axis ticks are round values, with the count scaled to the
    available pixels. Tick labels that would overlap are skipped. The left
    margin is sized to fit the widest y label.
  - When a graph's name and statistics don't fit on one line, the statistics
    move to a second line and are shortened instead of overdrawing the name.
  - Long logs are reduced to min/max per pixel column, which keeps spikes
    visible.
  - Only graphs on screen are drawn. The rest draw as you scroll to them.
- **HITL plots** (`hitl_gui.py`):
  - Panels redraw immediately on resize. The periodic redraw is skipped while
    the plot is hidden or nothing has changed.
  - Panels have round y and time ticks with gridlines. Before, only the
    min/max values and the end time were labelled.
  - Legends wrap instead of running off the panel.
  - Data and the sim-time cursor stay inside the plot frame.
  - Plot and log/table are split by a draggable sash.

## Files

- `witnessos.py` is the entry point: the `WitnessOS` notebook and the CLI.
- `wos_paths.py` adds `Software/HITL` and `Software/Horizon` to `sys.path`.
- `wos_widgets.py` holds `ScrollableFrame`, `WheelRouter` and the pure
  layout helpers.
- `wos_relay.py` is `RelayEngine`, the relay without a GUI. It is
  thread-safe and testable.
- `wos_relay_tab.py` is the relay tab GUI.

## Tests

```bash
cd Software/witnessOS && python -B -m unittest discover -s tests -q
```

New tests also live in `Software/HITL/tests/test_gui_plot.py` and
`Software/Horizon/tests/test_viewer_plotting.py`.
