"""GUI-independent Wi-Fi OTA workflow for witnessOS (field laptop, no Pi).

Drives: inspect image -> check flight state -> switch Witness to its
maintenance image (radio/USB command) -> wait for the 0x08 beacon -> join the
Witness access point with the laptop's own Wi-Fi (explicit confirmation, or
guided by hand) -> upload with witness_ota.Witness -> verify -> leave
maintenance -> restore the previous Wi-Fi -> confirm the flight image from the
next telemetry. See WITNESS_OTA_GUI_DESIGN.md.

No Qt, no sockets of its own, no sleeps it cannot be given: everything it
touches is injected (Ports for the command/telemetry link, a WifiBackend for
the network, callbacks for the UI), so tests run against fakes.
"""

from __future__ import annotations

import hashlib
import os
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

import ota_wifi
import witness_ota as wo
import witness_protocol as wp

SLOT_LIMIT = {"flight": 0x200000, "maintenance": 0x180000}
BEACON_FRESH_S = 12.0        # beacons come every 5 s
TELEMETRY_FRESH_S = 6.0
SAFE_STATES = (0, 4)         # Pad Idle, Landed (command_policy)
STEPS = ("inspect", "precheck", "switch", "beacon", "wifi", "check", "upload", "verify",
         "leave", "restore", "confirm")

# ------------------------------------------------------------- image info
INFO_FMT = "<IBBBBBBBBIIHHIB19s"      # main/image_info.h Info, 48 bytes
assert struct.calcsize(INFO_FMT) == 48


def parse_image_info(head: bytes) -> dict | None:
    """The 48-byte image-info record at file offset 288, or None."""
    offset = wo.HEAD_LENGTH
    if len(head) < offset + 48:
        return None
    (magic, struct_version, kind, store, store_min, command_set, _r0, sf, cr, freq, bw, preamble,
     sync, build_id, settings_version, _r1) = struct.unpack_from(INFO_FMT, head, offset)
    if magic != wo.INFO_MAGIC:
        return None
    return {"struct_version": struct_version, "kind": wo.KINDS.get(kind), "store_version": store,
            "store_min_version": store_min, "command_set": command_set, "sf": sf, "cr": cr,
            "freq_hz": freq, "bw_hz": bw, "preamble": preamble, "sync_word": sync,
            "build_id": build_id, "radio_settings_version": settings_version}


def version_id(text: str) -> int:
    """image_info::version_id: FNV-1a folded to 16 bits (the beacon's image ids)."""
    h = 2166136261
    for byte in text.encode("ascii", "replace")[:32]:
        h = ((h ^ byte) * 16777619) & 0xFFFFFFFF
    if not text:
        return 0
    folded = ((h >> 16) ^ (h & 0xFFFF)) & 0xFFFF
    return folded or 1


def check_pair(flight: dict, maint: dict) -> str | None:
    """Python mirror of image_info::check_maintenance. None = compatible."""
    if maint["kind"] != "maintenance":
        return "bad image info"
    if maint["struct_version"] != flight["struct_version"]:
        return "image-info layout differs"
    if not maint["store_min_version"] <= flight["store_version"] <= maint["store_version"]:
        return "log storage format differs"
    keys = ("radio_settings_version", "freq_hz", "bw_hz", "sf", "cr", "preamble", "sync_word")
    if any(maint[k] != flight[k] for k in keys):
        return "LoRa settings differ"
    if maint["command_set"] != flight["command_set"]:
        return "command set differs"
    return None


def inspect_image(path: str) -> dict:
    """Everything the dialog shows about a .bin; raises wo.OtaError if it is not
    a Witness app image. Pure file read."""
    with open(path, "rb") as handle:
        data = handle.read()
    desc = wo.check_image(data[:wo.HEAD_LENGTH + wo.INFO_LENGTH])
    info = parse_image_info(data)
    if info is None or info["kind"] is None:
        raise wo.OtaError("image has no Witness image-info record (built before two-image boot?)")
    if len(data) > SLOT_LIMIT[info["kind"]]:
        raise wo.OtaError("image is %d bytes, larger than the %s slot" % (len(data), info["kind"]))
    return {"path": path, "data": data, "size": len(data), "sha256": hashlib.sha256(data).hexdigest(),
            "version": desc["version"], "kind": info["kind"], "id": version_id(desc["version"]),
            "info": info}


def find_sibling(report: dict) -> dict | None:
    """flight.bin <-> maint.bin next to it (build_images.ps1 output), if present and of the other kind."""
    folder = os.path.dirname(report["path"])
    name = "maint.bin" if report["kind"] == "flight" else "flight.bin"
    candidate = os.path.join(folder, name)
    if not os.path.isfile(candidate):
        return None
    try:
        other = inspect_image(candidate)
    except wo.OtaError:
        return None
    return other if other["kind"] != report["kind"] else None


def offline_problems(report: dict, sibling: dict | None, installed: dict | None = None) -> list:
    """Reasons to refuse before anything is sent (stage A). `installed` is the
    latest beacon (ids, radio settings version) if one has been seen."""
    problems = []
    if sibling is not None:
        flight, maint = (report, sibling) if report["kind"] == "flight" else (sibling, report)
        why = check_pair(flight["info"], maint["info"])
        if why:
            problems.append("image does not match %s from the same folder: %s" % (os.path.basename(sibling["path"]), why))
    return problems


def check_against_info(report: dict, info: dict) -> str | None:
    """Stage B, once /info answers: the running maintenance image must be the
    one the board reports and, when it reports its store/command-set versions,
    compatible with this flight image (the firmware 409 stays the authority)."""
    if info.get("kind") not in (None, "maintenance"):
        return "Witness is not running its maintenance image (kind %s)" % info.get("kind")
    if report["kind"] != "flight":
        return None
    running = next((m for m in info.get("maintenance", []) if m and m.get("slot") == info.get("running")), None)
    if not running:
        return None
    i = report["info"]
    store = running.get("store_version")
    if store is not None and i["store_version"] > store:
        return "log storage format differs from the running maintenance image"
    cmd = running.get("command_set")
    if cmd is not None and cmd != i["command_set"]:
        return "command set differs from the running maintenance image (update maintenance first)"
    return None


# ------------------------------------------------------------- workflow
class Cancelled(Exception):
    pass


class WorkflowError(Exception):
    def __init__(self, message, outcome="failed"):
        super().__init__(message)
        self.outcome = outcome


@dataclass
class Ports:
    """The command/telemetry link witnessOS already has (USB, or LoRa ground station)."""
    send: Callable            # (frame bytes) -> ack dict | None   (reliable, resends)
    beacon: Callable          # () -> (0x08 packet | None, age_s)
    flight: Callable          # () -> (status int | None, uptime int | None, age_s)


@dataclass
class Callbacks:
    step: Callable = lambda name, state, message="": None
    progress: Callable = lambda done, total, label="": None
    log: Callable = lambda message: None
    confirm_join: Callable = lambda text: False       # explicit consent for automatic join
    show_credentials: Callable = lambda ssid, password: None   # guided mode
    choose_ssid: Callable = lambda ssids: ssids[0] if ssids else None


@dataclass
class Result:
    ok: bool
    outcome: str       # success | refused | rolled_back | unknown | cancelled | failed
    message: str
    board: str         # which image Witness is believed to run
    data: dict = field(default_factory=dict)


class NetWitness(wo.Witness):
    """witness_ota.Witness whose sockets bind to the Wi-Fi adapter's address, so
    traffic to 192.168.4.1 can never leave through Ethernet; reconnects through
    the backend while waiting for a restart."""

    def __init__(self, base, source_ip=None, on_unreachable=None, timeout=15.0):
        super().__init__(base, timeout)
        self.source_ip = source_ip
        self.on_unreachable = on_unreachable

    def _connect(self, timeout=None):
        import http.client
        return http.client.HTTPConnection(
            self.host, self.port, timeout=timeout or self.timeout,
            source_address=(self.source_ip, 0) if self.source_ip else None)

    def info(self, timeout=None):
        try:
            return super().info(timeout=timeout)
        except wo.OtaError:
            if self.on_unreachable:
                self.on_unreachable()
            raise


class OtaWorkflow:
    def __init__(self, ports: Ports, wifi: ota_wifi.WifiBackend, ui: Callbacks | None = None, *,
                 journal: ota_wifi.RestoreJournal | None = None, password: str = ota_wifi.DEFAULT_PASSWORD,
                 base: str = wo.DEFAULT_BASE, auto_join: bool = False, wifi_minutes: int = 30,
                 witness_factory=NetWitness, sleep=time.sleep, clock=time.monotonic,
                 cancel: threading.Event | None = None, timeouts: dict | None = None):
        self.ports, self.wifi, self.ui = ports, wifi, ui or Callbacks()
        self.journal, self.password, self.base = journal, password, base
        self.auto_join, self.wifi_minutes = auto_join, wifi_minutes
        self.witness_factory, self.sleep, self.clock = witness_factory, sleep, clock
        self.cancel = cancel or threading.Event()
        self.t = {"ack_beacon": 20.0, "beacon": 75.0, "unconfirmed": 45.0, "reachable": 120.0,
                  "reboot": 120.0, "flight": 90.0, "stable": 45.0, **(timeouts or {})}
        self.snapshot = None
        self.joined = False
        self.witness = None
        self.in_maintenance = False
        self.board = "unknown"

    # ---------------------------------------------------------- helpers
    def _check(self):
        if self.cancel.is_set():
            raise Cancelled()

    def _wait(self, predicate, timeout, interval=1.0):
        deadline = self.clock() + timeout
        while True:
            self._check()
            value = predicate()
            if value:
                return value
            if self.clock() >= deadline:
                return None
            self.sleep(interval)

    def _step(self, name, state, message=""):
        self.ui.step(name, state, message)
        if message:
            self.ui.log("%s: %s" % (name, message))

    def _fresh_beacon(self):
        packet, age = self.ports.beacon()
        return packet if packet is not None and age <= BEACON_FRESH_S else None

    # ---------------------------------------------------------- public API
    def update(self, path: str, force: bool = False) -> Result:
        return self._session(lambda: self._do_update(path, force))

    def download_logs(self, destination: str) -> Result:
        return self._session(lambda: self._do_logs(destination))

    def _session(self, body) -> Result:
        try:
            return body()
        except Cancelled:
            self._cleanup("cancelled")
            return Result(False, "cancelled", "cancelled; Wi-Fi restored, Witness returning to flight", self.board)
        except (WorkflowError, wo.OtaError, ota_wifi.WifiError) as exc:
            outcome = exc.outcome if isinstance(exc, WorkflowError) else "failed"
            self._cleanup(outcome)
            return Result(False, outcome, str(exc), self.board)

    # ---------------------------------------------------------- steps
    def _do_update(self, path, force):
        self._step("inspect", "running")
        try:
            report = inspect_image(path)
        except OSError as exc:
            raise WorkflowError("cannot read %s: %s" % (path, exc), "refused")
        except wo.OtaError as exc:
            raise WorkflowError(str(exc), "refused")
        sibling = find_sibling(report)
        problems = [] if force else offline_problems(report, sibling)
        if problems:
            raise WorkflowError("; ".join(problems) + " (update both images, maintenance first)", "refused")
        beacon = self._fresh_beacon()
        if beacon and report["id"] in (beacon["maint_image_id"], beacon["flight_image_id"]) and not force:
            self.ui.log("note: Witness already reports this image id; uploading anyway replaces the inactive slot")
        self._step("inspect", "done", "%s image %s, %d bytes" % (report["kind"], report["version"], report["size"]))

        self._enter_maintenance()
        info = self._join_and_check(report, force)
        self._step("upload", "running")
        result = self._upload(report, force)
        self._step("upload", "done", "%d bytes into %s" % (result.get("bytes", 0), result.get("slot")))
        if report["kind"] == "maintenance":
            self._step("verify", "running", "new maintenance image boots with Wi-Fi up for its self-test")
            self.witness.on_unreachable = self._rejoin_throttled()
            try:
                final = wo.wait_for_reboot(self.witness, report["version"], timeout=self.t["reboot"],
                                           sleep=self._sleep_checked, clock=self.clock)
            except wo.OtaError as exc:
                self._step("verify", "failed", str(exc))
                return self._rollback_or_unknown(report, str(exc))
            self._step("verify", "done", "running %s, app state %s" % (final.get("version"), final.get("app_state")))
        else:
            self._step("verify", "done", "flight image installed; Witness restarts into it (no Wi-Fi in flight)")
            self.in_maintenance = False
            self.board = "flight (new, pending self-test)"
        self._leave_and_restore()
        return self._confirm(report)

    def _do_logs(self, destination):
        self._enter_maintenance()
        self._join_and_check(None, False)
        os.makedirs(destination, exist_ok=True)
        self._step("upload", "running", "downloading logs")
        listing = self.witness.logs()
        files = []
        for entry in listing.get("files", []):
            name = os.path.basename(entry["name"])
            self._check()
            size = self.witness.download(
                name, os.path.join(destination, name),
                lambda done, total, n=name: (self._check(), self.ui.progress(done, total, n)))
            files.append({"name": name, "size": size})
        self._step("upload", "done", "%d file(s)" % len(files))
        self._leave_and_restore()
        self._wait_flight(self.t["flight"])
        return Result(True, "success", "%d log file(s) saved to %s" % (len(files), destination),
                      "flight (unchanged)", {"files": files})

    def _enter_maintenance(self):
        beacon = self._fresh_beacon()
        if beacon is not None:
            if beacon["mode"] != 0:
                raise WorkflowError("Witness is in USB export mode, not Wi-Fi mode; power cycle it first", "refused")
            self._step("precheck", "done", "already in maintenance image")
            self._step("switch", "skipped")
            self.in_maintenance = True
            self.board = "maintenance"
        else:
            self._step("precheck", "running")
            status, _uptime, age = self.ports.flight()
            if status is None or age > TELEMETRY_FRESH_S:
                raise WorkflowError("no recent Witness telemetry: cannot confirm it is on the pad or landed", "refused")
            if status & wp.FLIGHT_STATE_MASK not in SAFE_STATES:
                raise WorkflowError("Witness is in flight state %d; OTA only in Pad Idle or Landed"
                                    % (status & wp.FLIGHT_STATE_MASK), "refused")
            self._step("precheck", "done", "Pad Idle/Landed")
            self._step("switch", "running", "sending maintenance switch (0x5746 on)")
            self.board = "flight (unchanged)"
            ack = self.ports.send(wp.build_wifi_packet(True, 0, self.wifi_minutes))
            if ack is not None and ack.get("result") != 0:
                reason = ack.get("detail_name") or ("refused by flight state/source" if not ack.get("detail")
                                                    else "detail 0x%04X" % ack.get("detail"))
                raise WorkflowError("Witness refused the switch: %s" % reason, "refused")
            if ack is None:
                self.ui.log("no ack (ack lost, or Witness already restarting); waiting for the beacon")
            self._step("switch", "done", "acknowledged" if ack else "no ack")
            self._step("beacon", "running")
            self.in_maintenance = True   # from here a reset is what returns it to flight
            beacon = self._wait(lambda: self._fresh_beacon() if self._fresh_beacon() and
                                self._fresh_beacon().get("ap_up") else None,
                                self.t["beacon"] if ack else self.t["ack_beacon"])
            if beacon is None:
                self.in_maintenance = False
                status, _u, age = self.ports.flight()
                if status is not None and age <= TELEMETRY_FRESH_S:
                    raise WorkflowError("Witness is still running its flight image (the switch did not happen); "
                                        "nothing was changed", "refused")
                raise WorkflowError("no maintenance beacon and no flight telemetry: Witness is out of radio "
                                    "range, rebooting, or off. Any reset returns it to flight; check it and retry",
                                    "unknown")
            self.board = "maintenance"
        if beacon.get("unconfirmed"):
            self._step("beacon", "running", "new maintenance image still self-testing")
            beacon = self._wait(lambda: (b if (b := self._fresh_beacon()) and not b.get("unconfirmed") else None),
                                self.t["unconfirmed"])
            if beacon is None:
                raise WorkflowError("maintenance image stayed unconfirmed (it refuses OTA until it passes its "
                                    "self-test); wait and retry", "failed")
        self._step("beacon", "done", "maintenance image up (battery %.2f V, %d log file(s))"
                   % (beacon["battery_voltage"], beacon["log_files"]))

    def _join_and_check(self, report, force):
        self._step("wifi", "running")
        self.snapshot = self.wifi.snapshot()
        ssids = [s for s in self.wifi.visible_ssids() if s.lower().startswith(ota_wifi.WITNESS_SSID_PREFIX)]
        ssid = self.ui.choose_ssid(ssids) if len(ssids) != 1 else ssids[0]
        automatic = False
        if self.auto_join and self.wifi.automatic and ssid:
            prev = self.snapshot.get("ssid") or "no network"
            automatic = self.ui.confirm_join(
                "Join %s with this computer's Wi-Fi? You will leave %s and lose internet until the update "
                "finishes; the previous network is restored afterwards." % (ssid, prev))
        if automatic:
            if self.journal:
                self.journal.write(self.snapshot)
            self.joined = True
            self.wifi.join(ssid, self.password, self.snapshot)
            self.ui.log("joined %s" % ssid)
        else:
            self.ui.show_credentials(ssid or "witness-XXXX", self.password)
        self.witness = self.witness_factory(self.base, source_ip=None, on_unreachable=None)
        ready = self._wait(lambda: self._probe(automatic), self.t["reachable"], interval=2.0)
        if not ready:
            raise WorkflowError("Witness HTTP endpoint %s never became reachable (joined the wrong network? "
                                "Witness Wi-Fi off?)" % self.base, "failed")
        self._step("wifi", "done", "Witness reachable")
        self._step("check", "running")
        info = ready
        if report is not None:
            why = None if force else check_against_info(report, info)
            if why:
                raise WorkflowError(why, "refused")
        if info.get("unconfirmed"):
            raise WorkflowError("Witness maintenance image is unconfirmed; wait for its self-test", "failed")
        self._step("check", "done", "running %s (%s)" % (info.get("version"), info.get("running")))
        return info

    def _probe(self, automatic):
        if automatic:
            ip = self.wifi.local_ip(self.snapshot)
            if ip:
                self.witness.source_ip = ip
        try:
            return self.witness.info(timeout=3.0)
        except wo.OtaError:
            return None

    def _rejoin_throttled(self):
        last = {"t": 0.0}

        def hook():
            if not (self.joined and self.wifi.automatic) or self.clock() - last["t"] < 8.0:
                return
            last["t"] = self.clock()
            try:
                ssid = self.snapshot and self.wifi.visible_ssids()
                names = [s for s in (ssid or []) if s.lower().startswith(ota_wifi.WITNESS_SSID_PREFIX)]
                if names:
                    self.wifi.join(names[0], self.password, self.snapshot)
            except ota_wifi.WifiError:
                pass
        return hook

    def _sleep_checked(self, seconds):
        self._check()
        self.sleep(seconds)

    def _upload(self, report, force):
        last_error = None
        for attempt in range(2):
            self._check()
            try:
                return self.witness.upload(
                    report["data"],
                    lambda done, total: (self._check(), self.ui.progress(done, total, "upload")),
                    force=force)
            except wo.OtaError as exc:
                last_error = exc
                text = str(exc)
                # A firmware refusal (HTTP 4xx) is final; a dropped connection is retried
                # from byte 0 (the inactive slot is only booted after the full image validates).
                if "HTTP 4" in text or "refused the update" in text:
                    raise WorkflowError(text, "refused")
                self.ui.log("upload attempt %d failed: %s" % (attempt + 1, text))
                if not self._wait(lambda: self._probe(self.joined), 30.0, 2.0):
                    break
        raise WorkflowError("upload failed (%s). Witness keeps its old image; it is still in maintenance "
                            "and returns to flight on any reset" % last_error, "failed")

    def _rollback_or_unknown(self, report, why):
        """The new maintenance image did not confirm: judge from the beacon and telemetry
        after restoring the network."""
        self._restore_wifi()
        status, _u, age = self.ports.flight()
        beacon = self._fresh_beacon()
        if beacon is None and status is not None and age <= TELEMETRY_FRESH_S:
            self.board = "flight (old maintenance image restored by rollback)"
            return Result(False, "rolled_back", "new maintenance image failed its self-test and was rolled "
                          "back; Witness is back in flight on the old images (%s)" % why, self.board)
        self.board = "unknown"
        return Result(False, "unknown", "could not confirm the new maintenance image (%s); Witness may still "
                      "be in maintenance (returns to flight on any reset)" % why, self.board)

    def _leave_and_restore(self):
        self._step("leave", "running")
        if self.in_maintenance and self.witness is not None:
            try:
                self.witness.reboot()
            except wo.OtaError as exc:
                self.ui.log("HTTP reboot failed (%s); falling back to the radio" % exc)
        self.in_maintenance = False
        self._step("leave", "done")
        self._restore_wifi()

    def _restore_wifi(self):
        if not self.joined and not (self.snapshot and self.journal and self.journal.read()):
            self._step("restore", "skipped", "laptop Wi-Fi was never changed")
            return
        self._step("restore", "running", "reconnecting to %s" % ((self.snapshot or {}).get("ssid") or "previous state"))
        try:
            self.wifi.restore(self.snapshot)
        except ota_wifi.WifiError as exc:
            self._step("restore", "failed", str(exc))
            raise WorkflowError("could not restore the previous Wi-Fi (%s): reconnect manually" % exc, "failed")
        self.joined = False
        if self.journal:
            self.journal.clear()
        self._step("restore", "done")

    def _wait_flight(self, timeout):
        return self._wait(lambda: (lambda s: s if s[0] is not None and s[2] <= TELEMETRY_FRESH_S
                                   and self._fresh_beacon() is None else None)(self.ports.flight()), timeout)

    def _confirm(self, report):
        """Flight telemetry returns; a second reset or a new beacon means rollback."""
        self._step("confirm", "running")
        first = self._wait_flight(self.t["flight"])
        if first is None:
            self.board = "unknown"
            raise WorkflowError("no flight telemetry after the update; check Witness (it returns to its old "
                                "image if the new one fails)", "unknown")
        if report["kind"] == "maintenance":
            self.board = "flight, new maintenance image installed"
            self._step("confirm", "done", "flight telemetry is back")
            return Result(True, "success", "maintenance image %s installed and confirmed; Witness is back in flight"
                          % report["version"], self.board)
        uptime0 = first[1]
        deadline = self.clock() + self.t["stable"]
        last = uptime0
        while self.clock() < deadline:
            self._check()
            self.sleep(1.0)
            status, uptime, age = self.ports.flight()
            if self._fresh_beacon() is not None:
                return self._rolled_back("Witness re-entered maintenance (the bootloader rolled back)")
            if uptime is not None and last is not None and uptime < last and age <= TELEMETRY_FRESH_S:
                return self._rolled_back("Witness reset again during its 30 s self-test")
            last = uptime if uptime is not None else last
        status = first[0]
        warn = [t for m, t in ((0x10, "flash init failed"), (0x80, "logging failed")) if status & m]
        self.board = "flight (new)"
        self._step("confirm", "done", "stable past the self-test")
        msg = ("flight image %s installed; telemetry stable past the 30 s self-test. Flight packets carry no "
               "image id: confirm the version on the next maintenance entry" % report["version"])
        if warn:
            msg += " WARNING: " + ", ".join(warn)
        return Result(True, "success", msg, self.board)

    def _rolled_back(self, why):
        self.board = "flight (previous image)"
        self._step("confirm", "failed", why)
        return Result(False, "rolled_back", "new flight image did not confirm: %s. Witness runs the previous "
                      "flight image." % why, self.board)

    def _cleanup(self, outcome):
        """Failure/cancel: leave maintenance if we can, then always give the Wi-Fi back."""
        if self.in_maintenance and self.witness is not None and self.joined:
            try:
                self.witness.reboot()
                self.in_maintenance = False
            except wo.OtaError:
                pass
        try:
            self._restore_wifi()
        except WorkflowError as exc:
            self.ui.log(str(exc))
        if self.in_maintenance and outcome != "unknown":
            # Still in maintenance (HTTP unreachable or never joined): ask over the radio.
            try:
                self.ports.send(wp.build_wifi_packet(False))
            except Exception as exc:  # noqa: BLE001
                self.ui.log("radio wifi-off failed: %s" % exc)
