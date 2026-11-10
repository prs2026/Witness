"""Flight View layout draft: the Layout Editor's logic, without any GUI.

The draft is a separate copy of the layout JSON. Edits never touch live
telemetry until the view applies it; saving writes the draft. Existing custom
mappings are preserved, including multiple packet names and filter objects.

Items are addressed by keys: "layout", "plots:<i>", "readouts:<i>",
"plots:<i>:<j>" (trace j of plot i), "readouts:<i>:<j>". form_fields()
describes the form for the selected item; its values live in ``variables``
(observable Cells), and commit() writes them back into the draft.
"""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path

from aim_altitude import BARO_ALTITUDE, BARO_VELOCITY
from aim_mqtt import AIM_TOPIC_FIELDS
from aim_orientation import GYRO_FIELDS
from flight_data import DEFAULT_LAYOUT, DISPLAY_FORMATS, load_layout, validate_layout
from observable import Cell
from telemetry_model import FEATHERWEIGHT_FIELDS, WITNESS_FIELDS

# Derived Witness status fields computed by FlightData from status/fc_status:
# byte 1 bits 2:0 flight state, bit 3 GPS locked, bit 4 HITL active.
_STATUS_DERIVED = ("flight_state", "gps_locked", "hitl_active")
WITNESS_EDITOR_FIELDS = {
    packet: (*fields, *(f"{status}.{name}" for status in ("status", "fc_status") if status in fields
                        and not (status == "status" and packet in ("camera", "iris_debug"))
                        for name in _STATUS_DERIVED))
    for packet, fields in WITNESS_FIELDS.items()
}

FIELDS = {"witness": WITNESS_EDITOR_FIELDS, "featherweight": FEATHERWEIGHT_FIELDS,
          "aim": {**AIM_TOPIC_FIELDS,
                  "pressure_payload/v1": (*AIM_TOPIC_FIELDS["pressure_payload/v1"], BARO_ALTITUDE, BARO_VELOCITY),
                  "orientation_payload/v1": (*AIM_TOPIC_FIELDS["orientation_payload/v1"], *GYRO_FIELDS)}}
GROUPS = {"flight": "Flight overview (left)", "state": "Flight state (left)",
          "gps": "GPS (right)", "power": "Power (right)"}
HINT = ("Select an item on the left to edit it. Graph order is reading order.\n"
        "Limits: 6 graphs, 3 flight rows, 1 state row, 6 GPS rows, 10 power rows.\n"
        "Values per row: flight 3; state, GPS and power 2 (state: add format hitl).\n"
        "Packet lists use commas. Fixed Y limits apply only with Auto-scale off.\n"
        'FW tracker filter: {"unit_type": "TRACKER"}. Leave {} for no filter.\n'
        "Existing custom fields and filters are preserved. Right axis requires right-axis units.")
DEFAULT_STATUS = "Edit a draft, then Apply to Flight View. Applying clears graph history, not logs or connections."


class FormField:
    """One form row: key, title, kind ("text" | "choice" | "bool") and choices."""

    def __init__(self, key, title, kind="text", choices=None):
        self.key, self.title, self.kind, self.choices = key, title, kind, choices

    def __repr__(self):  # pragma: no cover
        return f"FormField({self.key!r}, {self.kind})"


class LayoutDraft:
    def __init__(self, document):
        self.document = deepcopy(document)
        self.path = None   # Save As by default; never silently overwrite the default file.
        self.selected = "layout"
        self.variables: dict[str, Cell] = {}
        self.fields: list[FormField] = []
        self.title = ""
        self.dirty = False
        self.loading = False
        self.status = Cell(DEFAULT_STATUS)
        self.select("layout")

    # ------------------------------------------------------------ structure
    def target(self, key):
        if key == "layout":
            return self.document
        parts = key.split(":")
        row = self.document[parts[0]][int(parts[1])]
        return row if len(parts) == 2 else row["traces"][int(parts[2])]

    _target = target

    def exists(self, key):
        if key in ("layout", "plots", "readouts"):
            return True
        try:
            self.target(key)
            return True
        except (KeyError, IndexError, ValueError):
            return False

    def tree(self):
        """[(key, parent_key, text)] in display order."""
        items = [("layout", None, "Layout settings")]
        for section, title in (("plots", "Graphs (left-to-right, top-to-bottom)"), ("readouts", "Number readouts")):
            items.append((section, None, title))
            for i, row in enumerate(self.document.get(section, [])):
                key = f"{section}:{i}"
                items.append((key, section, f"{i + 1}. {row['title']}"
                              + (f" [{row['group']}]" if section == "readouts" else "")))
                for j, trace in enumerate(row["traces"]):
                    items.append((f"{key}:{j}", key, f"{trace['label']} ({trace['input']})"
                                  + (" [off]" if not trace.get("enabled", True) else "")))
        return items

    # ----------------------------------------------------------------- form
    def changed(self, *_):
        if not self.loading:
            self.dirty = True
            self.status.set("Draft changed — Apply updates Flight View; Save JSON stores the draft. "
                            "Live data is unchanged.")

    def _field(self, key, title, value, kind="text", choices=None):
        cell = Cell(bool(value) if kind == "bool" else str(value))
        cell.subscribe(self.changed)
        self.variables[key] = cell
        self.fields.append(FormField(key, title, kind, choices))

    def select(self, key):
        """Show ``key``'s form (after committing the current one). False if blocked."""
        if key in ("plots", "readouts"):
            return False
        if self.variables and key != self.selected and not self.commit():
            return False
        self.selected = key if self.exists(key) else "layout"
        self._build_form()
        return True

    def _build_form(self):
        self.loading = True
        self.variables, self.fields = {}, []
        target = self.target(self.selected)
        if self.selected == "layout":
            self.title = "Overall layout"
            for key, label, default in (("title", "Dashboard title", "Flight View"),
                    ("window_seconds", "Time window (seconds)", 120), ("columns", "Graph columns (1–2)", 2),
                    ("refresh_ms", "Refresh interval (ms)", 250), ("gap_seconds", "Gap threshold (seconds)", 3),
                    ("max_points", "History points per trace", 6000)):
                self._field(key, label, target.get(key, default))
            self._field("auto_scale", "Auto-scale all graphs", target.get("auto_scale", True), "bool")
        elif self.selected.count(":") == 1:
            self.title = "Graph / readout"
            self._field("title", "Title", target["title"])
            self._field("unit", "Units", target.get("unit", "value"))
            if self.selected.startswith("readouts"):
                self._field("group", "Position / section", GROUPS.get(target["group"], target["group"]),
                            "choice", list(GROUPS.values()))
            else:
                self._field("right_unit", "Right-axis units (optional)", target.get("right_unit") or "")
                limits = target.get("y_range") or ("", "")
                self._field("y_min", "Fixed Y minimum (optional)", limits[0])
                self._field("y_max", "Fixed Y maximum (optional)", limits[1])
        else:
            self.title = "Trace / number"
            self._field("label", "Label", target["label"])
            self._field("enabled", "Show this trace", target.get("enabled", True), "bool")
            self._field("input", "Data input", target["input"], "choice", list(FIELDS))
            packet = target["packet"]
            self._field("packet", "Packet(s) / MQTT topic(s)",
                        ", ".join(packet) if isinstance(packet, list) else packet, "choice", [])
            self._field("field", "Data field", target["field"], "choice", [])
            for key, label, default in (("source", "Source (* = automatic)", "*"), ("scale", "Multiply by", 1),
                    ("offset", "Then add", 0), ("unit", "Override units (optional)", ""),
                    ("color", "Color (#RRGGBB)", "#004b80")):
                self._field(key, label, target.get(key, default))
            self._field("axis", "Graph axis", target.get("axis", "left"), "choice", ["left", "right"])
            self._field("format", "Display format", target.get("format", "number"), "choice", list(DISPLAY_FORMATS))
            self._field("match", "Filters (optional JSON)", json.dumps(target.get("match", {})))
            self._field("note", "Note", target.get("note", ""))
            self.suggestions()
        self.loading = False

    def suggestions(self):
        """Refresh packet/field choices for the trace form from the chosen input."""
        if "input" not in self.variables:
            return
        schema = FIELDS.get(self.variables["input"].get(), {})
        packets = [p.strip() for p in str(self.variables["packet"].get()).split(",")]
        fields = set(field for packet in packets for field in schema.get(packet, ()))
        fields.add("_derived.horizontal_distance_m")
        for form_field in self.fields:
            if form_field.key == "packet":
                form_field.choices = list(schema)
            elif form_field.key == "field":
                form_field.choices = sorted(fields)

    def choices(self, key):
        return next((f.choices for f in self.fields if f.key == key), None)

    def commit(self):
        if self.selected is None:
            return True
        try:
            updated = deepcopy(self.target(self.selected))
            values = {key: cell.get() for key, cell in self.variables.items()}
            if self.selected == "layout":
                for key in ("window_seconds", "gap_seconds"):
                    values[key] = float(values[key])
                for key in ("columns", "refresh_ms", "max_points"):
                    values[key] = int(values[key])
            elif self.selected.count(":") == 1:
                if "group" in values:
                    values["group"] = next((key for key, label in GROUPS.items() if label == values["group"]),
                                           values["group"])
                    if values["group"] not in GROUPS:
                        raise ValueError("Choose a valid position / section")
                else:
                    low, high = str(values.pop("y_min")), str(values.pop("y_max"))
                    values["y_range"] = None if not low.strip() and not high.strip() else [float(low), float(high)]
                    if not str(values["right_unit"]).strip():
                        values.pop("right_unit")
                        updated.pop("right_unit", None)
            else:
                values["scale"], values["offset"] = float(values["scale"]), float(values["offset"])
                packets = [p.strip() for p in str(values["packet"]).split(",") if p.strip()]
                values["packet"] = packets[0] if len(packets) == 1 else packets
                values["match"] = json.loads(values["match"] or "{}")
                if not str(values["unit"]).strip():
                    values.pop("unit")
                    updated.pop("unit", None)
            updated.update(values)
        except (ValueError, TypeError, RecursionError) as exc:
            self.status.set(f"Please correct this item: {exc}")
            return False
        target = self.target(self.selected)
        target.clear()
        target.update(updated)
        return True

    # -------------------------------------------------------------- editing
    def add_row(self, section):
        if not self.commit():
            return None
        row = {"title": "New graph" if section == "plots" else "New readout", "unit": "m",
               "traces": [self.new_trace()]}
        if section == "readouts":
            row["group"] = "power"
        rows = self.document.setdefault(section, [])
        rows.append(row)
        self.changed()
        self.select_after_edit(f"{section}:{len(rows) - 1}")
        return self.selected

    @staticmethod
    def new_trace():
        return {"label": "Witness altitude", "input": "witness", "packet": "state",
                "field": "kalman_altitude", "color": "#171717"}

    def add_trace(self):
        if not self.selected or self.selected == "layout":
            self.status.set("Select a graph or readout to add a trace.")
            return None
        if not self.commit():
            return None
        row_key = ":".join(self.selected.split(":")[:2])
        traces = self.target(row_key)["traces"]
        traces.append(self.new_trace())
        self.changed()
        self.select_after_edit(f"{row_key}:{len(traces) - 1}")
        return self.selected

    def _siblings(self):
        parts = self.selected.split(":")
        parent = ":".join(parts[:-1])
        items = self.document[parts[0]] if len(parts) == 2 else self.target(parent)["traces"]
        return parent, items, int(parts[-1])

    def remove(self):
        if not self.selected or self.selected == "layout":
            return False
        parent, items, index = self._siblings()
        del items[index]
        self.changed()
        self.select_after_edit(parent if ":" in parent else "layout")
        return True

    def move(self, direction):
        if not self.selected or self.selected == "layout" or not self.commit():
            return False
        parent, items, index = self._siblings()
        other = index + direction
        if 0 <= other < len(items):
            items[index], items[other] = items[other], items[index]
            self.changed()
            self.select_after_edit(f"{parent}:{other}")
            return True
        return False

    def select_after_edit(self, key):
        """Select without committing (the draft was just changed structurally)."""
        self.selected = key if self.exists(key) and key not in ("plots", "readouts") else "layout"
        self._build_form()

    def update_item(self):
        if self.commit():
            self.select_after_edit(self.selected)
            return True
        return False

    # ------------------------------------------------------------ documents
    def replace(self, document, path=None):
        self.document = deepcopy(document)
        self.path = Path(path) if path is not None else None
        self.select_after_edit("layout")
        self.dirty = False
        self.status.set("Draft loaded. Flight View is unchanged until Apply.")

    def load_document(self, filename):
        """Validated layout document from a file (raises OSError/ValueError)."""
        return load_layout(filename).document

    def validated(self):
        if not self.commit():
            return None
        try:
            return validate_layout(deepcopy(self.document))
        except ValueError as exc:
            self.status.set(f"Layout not applied/saved: {exc}")
            return None

    def save(self, filename):
        layout = self.validated()
        if layout is None:
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


__all__ = ["LayoutDraft", "FIELDS", "GROUPS", "HINT", "DEFAULT_LAYOUT", "WITNESS_EDITOR_FIELDS"]
