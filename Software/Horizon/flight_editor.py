"""Form-based editor for Flight View JSON; changes stay in a separate draft.

Applying is explicit and resets plot history/source bindings, never receivers
or logs. Saving writes the draft, not the live layout. Existing custom mappings
are preserved, including multiple packet names and optional filter objects.
"""

from copy import deepcopy
import json
from pathlib import Path
import tkinter as tk
from tkinter import colorchooser, filedialog, messagebox, ttk

from aim_mqtt import AIM_TOPIC_FIELDS
from aim_altitude import BARO_ALTITUDE, BARO_VELOCITY
from aim_orientation import GYRO_FIELDS
from flight_data import DEFAULT_LAYOUT, load_layout, validate_layout
from ground_tabs import FEATHERWEIGHT_FIELDS, WITNESS_FIELDS


FIELDS = {"witness": WITNESS_FIELDS, "featherweight": FEATHERWEIGHT_FIELDS,
          "aim": {**AIM_TOPIC_FIELDS,
                  "pressure_payload/v1": (*AIM_TOPIC_FIELDS["pressure_payload/v1"], BARO_ALTITUDE, BARO_VELOCITY),
                  "orientation_payload/v1": (*AIM_TOPIC_FIELDS["orientation_payload/v1"], *GYRO_FIELDS)}}
GROUPS = {"flight": "Flight overview (left)", "state": "Flight state (left)",
          "gps": "GPS (right)", "power": "Power (right)"}


class FlightLayoutEditor(ttk.Frame):
    def __init__(self, parent, flight):
        super().__init__(parent, padding=6)
        self.flight = flight
        self.document = deepcopy(flight.data.layout.document)
        self.path = None  # Save As by default; never silently overwrite the default file.
        self.selected = None
        self.variables = {}
        self.widgets = {}
        self.dirty = False
        self.loading = False
        self.status = tk.StringVar(value="Edit a draft, then Apply to Flight View. Applying clears graph history, not logs or connections.")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)
        toolbar = ttk.Frame(self)
        toolbar.grid(row=0, column=0, sticky="ew")
        for label, command in (("Use current view", self.use_current), ("Load JSON", self.load_file),
                               ("Defaults", self.defaults), ("Apply to Flight View", self.apply),
                               ("Save JSON as...", self.save_file)):
            ttk.Button(toolbar, text=label, command=command).pack(side="left", padx=(0, 5))
        ttk.Label(self, textvariable=self.status, wraplength=850).grid(row=1, column=0, sticky="ew", pady=5)
        self.bind("<Configure>", self._resize_status)
        panes = ttk.PanedWindow(self, orient="horizontal")
        panes.grid(row=2, column=0, sticky="nsew")
        left = ttk.Frame(panes)
        left.columnconfigure(0, weight=1)
        left.rowconfigure(0, weight=1)
        self.tree = ttk.Treeview(left, show="tree", selectmode="browse", height=15)
        self.tree.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(left, command=self.tree.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.tree.configure(yscrollcommand=scroll.set)
        self.tree.bind("<<TreeviewSelect>>", self._select)
        buttons = ttk.Frame(left)
        buttons.grid(row=1, column=0, columnspan=2, sticky="ew", pady=4)
        for index, (label, command) in enumerate((
                ("Add graph", lambda: self.add_row("plots")), ("Add readout", lambda: self.add_row("readouts")),
                ("Add trace", self.add_trace), ("Remove", self.remove),
                ("Move up", lambda: self.move(-1)), ("Move down", lambda: self.move(1)))):
            ttk.Button(buttons, text=label, command=command).grid(row=index // 2, column=index % 2, sticky="ew")
        buttons.columnconfigure((0, 1), weight=1)
        panes.add(left, weight=2)
        right = ttk.Frame(panes)
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=1)
        self.canvas = tk.Canvas(right, highlightthickness=0, width=520, background="white")
        self.canvas.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(right, command=self.canvas.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.canvas.configure(yscrollcommand=scroll.set)
        self.form = ttk.Frame(self.canvas, padding=10)
        self.form.columnconfigure(1, weight=1)
        self.form_window = self.canvas.create_window(0, 0, anchor="nw", window=self.form)
        self.canvas.bind("<Configure>", lambda e: self.canvas.itemconfigure(self.form_window, width=e.width))
        self.form.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        panes.add(right, weight=3)
        self.rebuild("layout")

    def _resize_status(self, event):
        # Status text wraps without forcing a minimum application width.
        if event.widget is self:
            for child in self.winfo_children():
                if isinstance(child, ttk.Label):
                    child.configure(wraplength=max(200, event.width - 15))

    def changed(self, *_):
        if not self.loading:
            self.dirty = True
            self.status.set("Draft changed — Apply updates Flight View; Save JSON stores the draft. Live data is unchanged.")

    def _target(self, key):
        if key == "layout":
            return self.document
        parts = key.split(":")
        row = self.document[parts[0]][int(parts[1])]
        return row if len(parts) == 2 else row["traces"][int(parts[2])]

    def rebuild(self, select="layout"):
        self.selected = None
        self.tree.delete(*self.tree.get_children())
        self.tree.insert("", "end", iid="layout", text="Layout settings", open=True)
        for section, title in (("plots", "Graphs (left-to-right, top-to-bottom)"), ("readouts", "Number readouts")):
            self.tree.insert("", "end", iid=section, text=title, open=True)
            for i, row in enumerate(self.document.get(section, [])):
                key = f"{section}:{i}"
                self.tree.insert(section, "end", iid=key,
                    text=f"{i+1}. {row['title']}" + (f" [{row['group']}]" if section == "readouts" else ""), open=True)
                for j, trace in enumerate(row["traces"]):
                    self.tree.insert(key, "end", iid=f"{key}:{j}",
                        text=f"{trace['label']} ({trace['input']})" + (" [off]" if not trace.get("enabled", True) else ""))
        self.selected = select if self.tree.exists(select) else "layout"
        self.tree.selection_set(self.selected)
        self.tree.see(self.selected)
        self.show_form()

    def _select(self, _event=None):
        selection = self.tree.selection()
        if not selection or selection[0] == self.selected:
            return
        key = selection[0]
        if key in ("plots", "readouts") or not self.commit():
            if self.selected:
                self.tree.selection_set(self.selected)
            return
        self.selected = key
        self.show_form()

    def field(self, key, title, value, choices=None, boolean=False):
        row = len(self.variables) + 1
        variable = tk.BooleanVar(value=value) if boolean else tk.StringVar(value=str(value))
        self.variables[key] = variable
        ttk.Label(self.form, text=title).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=4)
        widget = (ttk.Checkbutton(self.form, variable=variable) if boolean else
                  ttk.Combobox(self.form, textvariable=variable, values=choices) if choices is not None else
                  ttk.Entry(self.form, textvariable=variable))
        widget.grid(row=row, column=1, sticky="ew", pady=4)
        variable.trace_add("write", self.changed)
        self.widgets[key] = widget
        return widget

    def show_form(self):
        self.loading = True
        for widget in self.form.winfo_children():
            widget.destroy()
        self.variables, self.widgets = {}, {}
        target = self._target(self.selected)
        if self.selected == "layout":
            title = "Overall layout"
            for key, label, default in (("title", "Dashboard title", "Flight View"),
                    ("window_seconds", "Time window (seconds)", 120), ("columns", "Graph columns (1–2)", 2),
                    ("refresh_ms", "Refresh interval (ms)", 250), ("gap_seconds", "Gap threshold (seconds)", 3),
                    ("max_points", "History points per trace", 6000)):
                self.field(key, label, target.get(key, default))
            self.field("auto_scale", "Auto-scale all graphs", target.get("auto_scale", True), boolean=True)
        elif self.selected.count(":") == 1:
            title = "Graph / readout"
            self.field("title", "Title", target["title"])
            self.field("unit", "Units", target.get("unit", "value"))
            if self.selected.startswith("readouts"):
                self.field("group", "Position / section", GROUPS[target["group"]], list(GROUPS.values()))
            else:
                self.field("right_unit", "Right-axis units (optional)", target.get("right_unit") or "")
                limits = target.get("y_range") or ("", "")
                self.field("y_min", "Fixed Y minimum (optional)", limits[0])
                self.field("y_max", "Fixed Y maximum (optional)", limits[1])
        else:
            title = "Trace / number"
            self.field("label", "Label", target["label"])
            self.field("enabled", "Show this trace", target.get("enabled", True), boolean=True)
            self.field("input", "Data input", target["input"], list(FIELDS)).bind("<<ComboboxSelected>>", self.suggestions)
            packet = target["packet"]
            self.field("packet", "Packet(s) / MQTT topic(s)", ", ".join(packet) if isinstance(packet, list) else packet, []).bind(
                "<<ComboboxSelected>>", self.suggestions)
            self.field("field", "Data field", target["field"], [])
            for key, label, default in (("source", "Source (* = automatic)", "*"), ("scale", "Multiply by", 1),
                    ("offset", "Then add", 0), ("unit", "Override units (optional)", ""), ("color", "Color (#RRGGBB)", "#004b80")):
                self.field(key, label, target.get(key, default))
            self.field("axis", "Graph axis", target.get("axis", "left"), ["left", "right"])
            self.field("format", "Display format", target.get("format", "number"), ["number", "state", "continuity"])
            self.field("match", "Filters (optional JSON)", json.dumps(target.get("match", {})))
            self.field("note", "Note", target.get("note", ""))
            self.suggestions()
            ttk.Button(self.form, text="Choose color...", command=self.pick_color).grid(row=17, column=1, sticky="w", pady=5)
        ttk.Label(self.form, text=title, font=("Segoe UI", 12, "bold")).grid(row=0, column=0, columnspan=2, sticky="w")
        hint = ("Select an item on the left to edit it. Graph order is reading order.\n"
                "Limits: 6 graphs, 3 flight rows, 1 state row, 6 GPS rows, 10 power rows.\n"
                "Packet lists use commas. Fixed Y limits apply only with Auto-scale off.\n"
                'FW tracker filter: {"unit_type": "TRACKER"}. Leave {} for no filter.\n'
                "Existing custom fields and filters are preserved. Right axis requires right-axis units.")
        ttk.Label(self.form, text=hint, wraplength=420, justify="left").grid(row=18, column=0, columnspan=2, sticky="w", pady=8)
        ttk.Button(self.form, text="Update draft item", command=self.update_item).grid(row=19, column=0, columnspan=2, sticky="ew")
        self.canvas.yview_moveto(0)
        self.loading = False

    def suggestions(self, _event=None):
        if "input" not in self.variables:
            return
        schema = FIELDS.get(self.variables["input"].get(), {})
        self.widgets["packet"]["values"] = tuple(schema)
        packets = [p.strip() for p in self.variables["packet"].get().split(",")]
        fields = set(field for packet in packets for field in schema.get(packet, ()))
        fields.add("_derived.horizontal_distance_m")
        self.widgets["field"]["values"] = tuple(sorted(fields))

    def pick_color(self):
        color = colorchooser.askcolor(parent=self, title="Trace color")[1]
        if color:
            self.variables["color"].set(color)

    def commit(self):
        if self.selected is None:
            return True
        try:
            updated = deepcopy(self._target(self.selected))
            values = {key: var.get() for key, var in self.variables.items()}
            if self.selected == "layout":
                for key in ("window_seconds", "gap_seconds"):
                    values[key] = float(values[key])
                for key in ("columns", "refresh_ms", "max_points"):
                    values[key] = int(values[key])
            elif self.selected.count(":") == 1:
                if "group" in values:
                    values["group"] = next((key for key, label in GROUPS.items() if label == values["group"]), values["group"])
                else:
                    low, high = values.pop("y_min"), values.pop("y_max")
                    values["y_range"] = None if not low.strip() and not high.strip() else [float(low), float(high)]
                    if not values["right_unit"].strip():
                        values.pop("right_unit")
                        updated.pop("right_unit", None)
            else:
                values["scale"], values["offset"] = float(values["scale"]), float(values["offset"])
                packets = [p.strip() for p in values["packet"].split(",") if p.strip()]
                values["packet"] = packets[0] if len(packets) == 1 else packets
                values["match"] = json.loads(values["match"] or "{}")
                if not values["unit"].strip():
                    values.pop("unit")
                    updated.pop("unit", None)
            updated.update(values)
        except (ValueError, TypeError, tk.TclError) as exc:
            self.status.set(f"Please correct this item: {exc}")
            return False
        target = self._target(self.selected)
        target.clear()
        target.update(updated)
        return True

    def update_item(self):
        if self.commit():
            self.rebuild(self.selected)

    def add_row(self, section):
        if not self.commit():
            return
        row = {"title": "New graph" if section == "plots" else "New readout", "unit": "m", "traces": [self.new_trace()]}
        if section == "readouts":
            row["group"] = "power"
        rows = self.document.setdefault(section, [])
        rows.append(row)
        self.changed()
        self.rebuild(f"{section}:{len(rows)-1}")

    @staticmethod
    def new_trace():
        return {"label": "Witness altitude", "input": "witness", "packet": "state",
                "field": "barometric_altitude", "color": "#171717"}

    def add_trace(self):
        if not self.selected or self.selected == "layout" or not self.commit():
            self.status.set("Select a graph or readout to add a trace.")
            return
        row_key = ":".join(self.selected.split(":")[:2])
        traces = self._target(row_key)["traces"]
        traces.append(self.new_trace())
        self.changed()
        self.rebuild(f"{row_key}:{len(traces)-1}")

    def _siblings(self):
        parts = self.selected.split(":")
        parent = ":".join(parts[:-1])
        items = self.document[parts[0]] if len(parts) == 2 else self._target(parent)["traces"]
        return parent, items, int(parts[-1])

    def remove(self):
        if not self.selected or self.selected == "layout":
            return
        if not messagebox.askyesno("Remove item", "Remove the selected item from the draft?", parent=self):
            return
        parent, items, index = self._siblings()
        del items[index]
        self.changed()
        self.rebuild(parent if ":" in parent else "layout")

    def move(self, direction):
        if not self.selected or self.selected == "layout" or not self.commit():
            return
        parent, items, index = self._siblings()
        other = index + direction
        if 0 <= other < len(items):
            items[index], items[other] = items[other], items[index]
            self.changed()
            self.rebuild(f"{parent}:{other}")

    def replace_draft(self, document, path=None):
        if self.dirty and not messagebox.askyesno("Replace draft", "Discard unsaved editor changes?", parent=self):
            return False
        self.document = deepcopy(document)
        self.path = Path(path) if path is not None else None
        self.rebuild()
        self.dirty = False
        self.status.set("Draft loaded. Flight View is unchanged until Apply.")
        return True

    def use_current(self):
        self.replace_draft(self.flight.data.layout.document)

    def defaults(self):
        self.load_file(DEFAULT_LAYOUT)

    def load_file(self, filename=None):
        filename = filename or filedialog.askopenfilename(parent=self, filetypes=(("Layout JSON", "*.json"),))
        if not filename:
            return False
        try:
            layout = load_layout(filename)
            return self.replace_draft(layout.document, filename)
        except (OSError, ValueError) as exc:
            self.status.set(f"Load failed: {exc}")
            return False

    def validated(self):
        if not self.commit():
            return None
        try:
            return validate_layout(deepcopy(self.document))
        except ValueError as exc:
            self.status.set(f"Layout not applied/saved: {exc}")
            return None

    def apply(self):
        layout = self.validated()
        if layout is None:
            return False
        if not messagebox.askyesno("Apply layout", "Apply this draft to Flight View? Graph history and source bindings will reset. Logging and connections continue.", parent=self):
            return False
        self.flight.apply_layout(layout, self.path or Path("Unsaved editor layout"))
        self.status.set("Applied to Flight View. Save JSON to keep this setup for next time.")
        return True

    def save_file(self, filename=None):
        layout = self.validated()
        if layout is None:
            return False
        filename = filename or filedialog.asksaveasfilename(parent=self, defaultextension=".json",
            initialfile="flight_view_custom.json", filetypes=(("Layout JSON", "*.json"),))
        if not filename:
            return False
        try:
            Path(filename).write_text(json.dumps(layout.document, indent=2) + "\n", encoding="utf-8")
        except OSError as exc:
            self.status.set(f"Save failed: {exc}")
            return False
        self.path = Path(filename)
        self.dirty = False
        self.status.set(f"Saved {self.path.name}. Apply separately to update Flight View.")
        return True
