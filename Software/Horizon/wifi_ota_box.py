"""Wi-Fi / OTA group box for the Witness monitor.

Two command buttons (Wi-Fi ON/OFF) go through MonitorCore.send_wifi, i.e. the same send path and
target as the other command buttons. The HTTP buttons talk to Witness's access point through
witness_ota.Witness on ONE background thread at a time; results come back to the GUI thread as Qt
signals and are written to the console. Joining the Witness Wi-Fi is the operator's job (no netsh).
"""

from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path

from PyQt6 import QtCore, QtWidgets

import witness_ota
from qt_common import guarded_slot
from qt_widgets import bind_enabled, hbox, wrap_label

NOTE = ("HTTP buttons need this computer joined to the witness-XXXX Wi-Fi (password "
        "witness-ota-2026, Witness at 192.168.4.1). Wi-Fi turns itself off after 15 min idle; "
        "any reset returns Witness to flight.")
LOGS_TIP = ("Lists the flight logs. Listing freezes flash logging until Witness reboots; "
            "downloads are only allowed in Landed.")


class _Signals(QtCore.QObject):
    line = QtCore.pyqtSignal(str)
    progress = QtCore.pyqtSignal(int, int)
    done = QtCore.pyqtSignal()


class WifiOtaBox(QtWidgets.QGroupBox):
    def __init__(self, core, parent=None):
        super().__init__("Wi-Fi / OTA", parent)
        self.core = core
        self._signals = _Signals(self)
        self._signals.line.connect(self._on_line)
        self._signals.progress.connect(self._on_progress)
        self._signals.done.connect(self._on_done)
        self._thread = None
        self._busy = False

        def button(text, slot, tip=""):
            b = QtWidgets.QPushButton(text)
            b.setToolTip(tip)
            b.clicked.connect(lambda _c=False: slot())
            return b

        self.wifi_on_button = button(
            "Wi-Fi ON (maintenance)", lambda: core.send_wifi(True),
            "Restart Witness into its maintenance image with the Wi-Fi access point on. "
            "Pad Idle or Landed only; asks first.")
        self.wifi_off_button = button("Wi-Fi OFF (back to flight)", lambda: core.send_wifi(False),
                                      "Turn Wi-Fi off; Witness restarts into the flight image.")
        self.info_button = button("OTA info", self.ota_info, "GET /info over HTTP")
        self.upload_button = button("Upload image…", self.upload_image,
                                    "Flash a softwarev1.bin (flight or maintenance) over Wi-Fi.")
        self.force_check = QtWidgets.QCheckBox("Force (both images, maintenance first)")
        self.force_check.setToolTip("Install even if the other image's interface is incompatible "
                                    "(witness_ota --force). Update the maintenance image first.")
        self.logs_button = button("List logs", self.list_logs, LOGS_TIP)
        self.download_button = button("Download all logs…", self.download_logs, LOGS_TIP)
        self.reboot_button = button("Reboot Witness (HTTP)", self.reboot,
                                    "POST /reboot. Pad Idle or Landed only; asks first.")
        self.http_buttons = [self.info_button, self.upload_button, self.logs_button,
                             self.download_button, self.reboot_button]
        self.progress_bar = QtWidgets.QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setVisible(False)
        self.note = wrap_label(NOTE, muted=True)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(hbox(self.wifi_on_button, self.wifi_off_button, self.info_button,
                              self.upload_button, self.force_check, None))
        layout.addLayout(hbox(self.logs_button, self.download_button, self.reboot_button, None))
        layout.addWidget(self.progress_bar)
        layout.addWidget(self.note)
        bind_enabled(core.wifi_on_enabled, self.wifi_on_button)
        bind_enabled(core.commands_enabled, self.wifi_off_button)

    # ------------------------------------------------------------------ plumbing
    @property
    def busy(self):
        return self._busy

    def make_witness(self):
        return witness_ota.Witness(timeout=15.0)

    def wait_finished(self, timeout=10.0):
        thread = self._thread
        if thread is not None:
            thread.join(timeout)

    def _start(self, work, *, progress=False):
        """Run work(emit, report) on a worker thread. Only one HTTP operation at a time."""
        if self._busy:
            self.core.console_write("OTA: another Wi-Fi operation is still running")
            return False
        self._busy = True
        for b in self.http_buttons:
            b.setEnabled(False)
        self.progress_bar.setVisible(progress)
        self.progress_bar.setValue(0)
        signals = self._signals

        def run():
            try:
                work(signals.line.emit, signals.progress.emit)
            except witness_ota.OtaError as exc:
                signals.line.emit(f"OTA_ERROR {exc}")
            except Exception as exc:  # never let a worker crash the monitor
                signals.line.emit(f"OTA_ERROR {type(exc).__name__}: {exc}")
            finally:
                signals.done.emit()

        self._thread = threading.Thread(target=run, name="wifi-ota", daemon=True)
        self._thread.start()
        return True

    @guarded_slot
    def _on_line(self, text):
        for line in str(text).splitlines() or [""]:
            self.core.console_write(line if line.startswith("OTA") else f"OTA {line}")

    @guarded_slot
    def _on_progress(self, done, total):
        if total > 0:
            self.progress_bar.setValue(min(100, 100 * done // total))

    @guarded_slot
    def _on_done(self):
        self._busy = False
        for b in self.http_buttons:
            b.setEnabled(True)
        self.progress_bar.setVisible(False)

    @staticmethod
    def _throttled(emit_line, label):
        """Progress callback that logs at most every 2 s and at completion."""
        state = {"t": 0.0}

        def show(done, total):
            now = time.monotonic()
            if done < total and now - state["t"] < 2.0:
                return
            state["t"] = now
            emit_line(f"{label}: {done} / {total} bytes" if total > 0 else f"{label}: {done} bytes")
        return show

    # ------------------------------------------------------------------ operations
    def ota_info(self):
        def work(emit, _progress):
            info = self.make_witness().info()
            emit("info: " + json.dumps(info, indent=2))
        return self._start(work)

    def list_logs(self):
        def work(emit, _progress):
            listing = self.make_witness().logs()
            emit(f"logs: {listing.get('count', 0)} file(s); Witness flash logging is frozen "
                 "until it reboots")
            for entry in listing.get("files", []):
                emit("%-14s %10d bytes  session %s" % (entry.get("name", "?"), entry.get("size", 0),
                                                       entry.get("session", "?")))
        return self._start(work)

    def reboot(self):
        if not self.core.confirm("Confirm", "Reboot Witness over Wi-Fi (HTTP)? Only accepted in "
                                            "Pad Idle or Landed; it returns to the flight image."):
            return False

        def work(emit, _progress):
            emit("reboot: " + json.dumps(self.make_witness().reboot()))
        return self._start(work)

    def pick_image(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Witness image", "", "Witness image (*.bin)")
        return path

    def pick_directory(self, default):
        return QtWidgets.QFileDialog.getExistingDirectory(self, "Download logs to", default)

    def upload_image(self, path=None):
        if path is None:
            path = self.pick_image()
        if not path:
            return False
        try:
            with open(path, "rb") as handle:
                image = handle.read()
            desc = witness_ota.check_image(image[:witness_ota.HEAD_LENGTH + witness_ota.INFO_LENGTH])
        except (OSError, witness_ota.OtaError) as exc:
            self.core.console_write(f"OTA_ERROR {path}: {exc}")
            return False
        kind = desc.get("kind") or "unknown kind"
        name = os.path.basename(path)
        self.core.console_write(f"OTA image {name}: version {desc['version']}, {len(image)} bytes, {kind}")
        force = self.force_check.isChecked()
        if not self.core.confirm(
                "Confirm upload",
                f"Upload {name} ({kind} image, version {desc['version']}, {len(image)} bytes) to "
                f"Witness over Wi-Fi?" + (" FORCE is on." if force else "")):
            return False

        def work(emit, progress):
            witness = self.make_witness()
            before = witness.info()
            emit(f"Witness runs {before.get('version')} ({before.get('running')}), "
                 f"{before.get('kind', 'flight')} image")
            show = self._throttled(emit, "uploading")

            def on_progress(done, total):
                show(done, total)
                progress(done, total)
            result = witness.upload(image, on_progress, force=force)
            emit(f"uploaded {result.get('bytes')} bytes into {result.get('slot')}; Witness reboots "
                 f"in {result.get('reboot_ms')} ms")
            if result.get("kind") == "flight" or kind == "flight":
                emit("Witness restarts into the new FLIGHT image (no Wi-Fi there); it confirms "
                     "itself within 30 s, a failed self-test rolls back. Check the state packets.")
                return
            emit("waiting for the reboot and the new image's self-test...")
            final = witness_ota.wait_for_reboot(witness, desc["version"], timeout=90.0)
            emit(f"now running {final.get('version')} from {final.get('running')}, "
                 f"app state {final.get('app_state')}")
        return self._start(work, progress=True)

    def download_logs(self, directory=None):
        if directory is None:
            default = Path(__file__).resolve().parent / "logs" / time.strftime("witness_wifi_%Y%m%d_%H%M%S")
            directory = self.pick_directory(str(default))
        if not directory:
            return False

        def work(emit, progress):
            witness = self.make_witness()
            files = witness.logs().get("files", [])
            emit(f"logs: downloading {len(files)} file(s) to {directory}")
            os.makedirs(directory, exist_ok=True)
            for entry in files:
                name = entry["name"]
                dest = os.path.join(directory, os.path.basename(name))
                size = witness.download(name, dest, lambda d, t: progress(d, t))
                emit(f"{name}: {size} bytes -> {dest}")
            emit(f"logs: done, {len(files)} file(s)")
        return self._start(work, progress=True)
