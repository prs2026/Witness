"""Wi-Fi join backends for the witnessOS OTA workflow (field laptop, no Pi).

The laptop running witnessOS joins the Witness access point with its own
Wi-Fi adapter, so these backends change the machine's network state. That is
why the design is defensive:

- every change goes through one injectable ``run(command) -> (code, text)``
  so tests use fakes and never touch a real adapter;
- ``snapshot()`` records the network the laptop is on BEFORE anything changes
  and ``restore()`` goes back to it (the workflow calls it on success, failure
  and cancel; ``RestoreJournal`` repeats it after a crash on the next start);
- the Witness profile is temporary (manual connect, never autoconnect) and is
  deleted again;
- joining is only ever done after the workflow's explicit user confirmation.

Windows (netsh) is implemented. Linux (nmcli) and macOS (networksetup) are
described in WITNESS_OTA_GUI_DESIGN.md. ManualBackend implements the guided
mode: the user joins by hand, nothing on the machine is changed.
"""

from __future__ import annotations

import json
import os
import subprocess
import tempfile
import time
from xml.sax.saxutils import escape

TEMP_PROFILE = "witness-ota-tmp"
WITNESS_SSID_PREFIX = "witness-"
DEFAULT_PASSWORD = "witness-ota-2026"   # CONFIG_WITNESS_WIFI_PASSWORD default; settable


class WifiError(Exception):
    pass


def default_run(command, timeout=30):
    try:
        done = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 127, str(exc)
    return done.returncode, (done.stdout + done.stderr)


def parse_keyed(text: str) -> list[dict]:
    """netsh-style 'Key : value' blocks -> list of dicts keyed by lowercase key.
    A new block starts when a key repeats. English output only (localized
    Windows needs the keys translated: see the design document)."""
    blocks, current = [], {}
    for line in text.splitlines():
        if ":" not in line:
            continue
        key, _, value = line.partition(":")
        key = key.strip().lower()
        if not key:
            continue
        if key in current:
            blocks.append(current)
            current = {}
        current[key] = value.strip()
    if current:
        blocks.append(current)
    return blocks


def profile_xml(ssid: str, password: str, name: str = TEMP_PROFILE) -> str:
    """Windows WLAN profile: WPA2-PSK/AES, manual connect only (never roams back to it)."""
    return (
        '<?xml version="1.0"?>'
        '<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">'
        "<name>%s</name><SSIDConfig><SSID><name>%s</name></SSID></SSIDConfig>"
        "<connectionType>ESS</connectionType><connectionMode>manual</connectionMode>"
        "<MSM><security><authEncryption><authentication>WPA2PSK</authentication>"
        "<encryption>AES</encryption><useOneX>false</useOneX></authEncryption>"
        "<sharedKey><keyType>passPhrase</keyType><protected>false</protected>"
        "<keyMaterial>%s</keyMaterial></sharedKey></security></MSM></WLANProfile>"
        % (escape(name), escape(ssid), escape(password)))


class WifiBackend:
    """Interface. All methods may raise WifiError."""

    automatic = False          # can join/restore without the user's hands

    def snapshot(self) -> dict:
        """The network the machine is on now: {"iface", "ssid", "profile"} (ssid None if none)."""
        raise NotImplementedError

    def visible_ssids(self) -> list:
        return []

    def join(self, ssid: str, password: str, snapshot: dict) -> None:
        raise NotImplementedError

    def local_ip(self, snapshot: dict):
        """IPv4 of the Wi-Fi adapter on the Witness network, or None (unknown/not yet)."""
        return None

    def restore(self, snapshot: dict) -> None:
        raise NotImplementedError

    def current_ssid(self, snapshot: dict):
        return self.snapshot().get("ssid")


class ManualBackend(WifiBackend):
    """Guided mode: the user joins and leaves by hand; the machine is never changed."""

    def snapshot(self):
        return {"iface": None, "ssid": None, "profile": None}

    def join(self, ssid, password, snapshot):
        raise WifiError("manual backend cannot join; the user joins the network")

    def restore(self, snapshot):
        return None


class NetshBackend(WifiBackend):
    """Windows. Uses only: netsh wlan show interfaces / show networks (read-only),
    netsh wlan add profile / connect / delete profile / disconnect, and
    netsh interface ip show address (read-only). Never run in tests: pass a fake run."""

    automatic = True

    def __init__(self, run=default_run, sleep=time.sleep, clock=time.monotonic,
                 join_timeout=40.0, restore_timeout=60.0, profile_dir=None):
        self.run = run
        self.sleep = sleep
        self.clock = clock
        self.join_timeout = join_timeout
        self.restore_timeout = restore_timeout
        self.profile_dir = profile_dir or tempfile.gettempdir()

    def _netsh(self, *args, timeout=30):
        return self.run(["netsh", *args], timeout)

    def snapshot(self):
        code, out = self._netsh("wlan", "show", "interfaces")
        if code != 0:
            raise WifiError("netsh wlan show interfaces failed: %s" % out.strip()[:120])
        blocks = [b for b in parse_keyed(out) if "name" in b]
        if not blocks:
            raise WifiError("no Wi-Fi adapter found")
        # Several adapters: prefer the connected one (the one the user would lose).
        chosen = next((b for b in blocks if b.get("state", "").lower() == "connected"), blocks[0])
        connected = chosen.get("state", "").lower() == "connected"
        return {"iface": chosen["name"], "ssid": chosen.get("ssid") if connected else None,
                "profile": chosen.get("profile") if connected else None}

    def visible_ssids(self):
        code, out = self._netsh("wlan", "show", "networks")
        names = []
        for line in out.splitlines():
            head, _, value = line.partition(":")
            if code == 0 and head.strip().lower().startswith("ssid ") and value.strip():
                names.append(value.strip())
        return names

    def join(self, ssid, password, snapshot):
        iface = snapshot["iface"]
        path = os.path.join(self.profile_dir, TEMP_PROFILE + ".xml")
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(profile_xml(ssid, password))
        try:
            code, out = self._netsh("wlan", "add", "profile", "filename=" + path, "interface=" + iface,
                                    "user=current")
        finally:
            try:
                os.remove(path)    # holds the password in clear
            except OSError:
                pass
        if code != 0:
            raise WifiError("could not add the Witness profile: %s" % out.strip()[:120])
        code, out = self._netsh("wlan", "connect", "name=" + TEMP_PROFILE, "ssid=" + ssid, "interface=" + iface)
        if code != 0:
            raise WifiError("netsh wlan connect failed: %s" % out.strip()[:120])
        self._wait_ssid(iface, ssid, self.join_timeout, "join " + ssid)

    def _wait_ssid(self, iface, ssid, timeout, what):
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            snap = self.snapshot()
            if snap.get("ssid") == ssid:
                return
            self.sleep(1.0)
        raise WifiError("timed out waiting to %s" % what)

    def local_ip(self, snapshot):
        code, out = self._netsh("interface", "ip", "show", "address", "name=" + snapshot["iface"])
        if code != 0:
            return None
        for block in parse_keyed(out):
            address = block.get("ip address")
            if address and not address.startswith("169.254."):
                return address
        return None

    def restore(self, snapshot):
        iface = snapshot["iface"]
        previous = snapshot.get("ssid")
        if iface is None:
            return
        try:
            if previous:
                code, out = self._netsh("wlan", "connect", "name=" + (snapshot.get("profile") or previous),
                                        "ssid=" + previous, "interface=" + iface)
                if code != 0:
                    raise WifiError("could not reconnect to %s: %s" % (previous, out.strip()[:120]))
                self._wait_ssid(iface, previous, self.restore_timeout, "reconnect to " + previous)
            else:
                self._netsh("wlan", "disconnect", "interface=" + iface)
        finally:
            # Only after reconnecting: deleting the profile of the connected network drops it.
            self._netsh("wlan", "delete", "profile", "name=" + TEMP_PROFILE, "interface=" + iface)


class RestoreJournal:
    """Remembers the pre-join network on disk so a crash or kill mid-update can
    still be undone at the next start (ota_workflow.recover)."""

    def __init__(self, path):
        self.path = path

    def write(self, snapshot):
        with open(self.path, "w", encoding="utf-8") as handle:
            json.dump(snapshot, handle)

    def read(self):
        try:
            with open(self.path, encoding="utf-8") as handle:
                return json.load(handle)
        except (OSError, ValueError):
            return None

    def clear(self):
        try:
            os.remove(self.path)
        except OSError:
            pass


def recover(backend: WifiBackend, journal: RestoreJournal) -> bool:
    """Call at startup: undo an interrupted join. True if a restore was attempted."""
    snapshot = journal.read()
    if not snapshot:
        return False
    backend.restore(snapshot)
    journal.clear()
    return True
