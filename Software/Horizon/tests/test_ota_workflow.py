"""ota_workflow / ota_wifi against fakes: fake board, fake Wi-Fi backend, fake
netsh runner and a local fake Witness HTTP server. No network or Wi-Fi command
of the machine running the tests is ever touched."""

import os
import struct
import sys
import tempfile
import threading
import unittest
from http.server import HTTPServer

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import ota_wifi  # noqa: E402
import ota_workflow as ow  # noqa: E402
import witness_ota as wo  # noqa: E402
import witness_protocol as wp  # noqa: E402
from test_witness_ota import FakeState, Handler, make_image  # noqa: E402

BEACON_PAYLOAD = bytes([1, 0, 0x25, 4, 1, 1, 0xBE, 0xEF, 0x12, 0x34, 0x1D, 0x14, 0, 0, 0, 5, 3])


def image_with_info(kind=1, version="v2", store=4, store_min=3, cmd=1, sf=7, settings=1, size=3000):
    data = bytearray(make_image(version=version, size=size))
    data[288:336] = struct.pack(ow.INFO_FMT, wo.INFO_MAGIC, 1, kind, store, store_min, cmd, 0, sf, 5,
                                915000000, 500000, 8, 0x1424, 1700000000, settings, bytes(19))
    return bytes(data)


def write(folder, name, data):
    path = os.path.join(folder, name)
    with open(path, "wb") as handle:
        handle.write(data)
    return path


class FakeClock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeBoard:
    """Flight/maintenance phases driven by the fake clock."""

    def __init__(self, clock, status=0):
        self.clock, self.phase, self.status = clock, "flight", status
        self.boot = clock() - 100
        self.acks = []
        self.sent = []
        self.ack = {"packet": "command_ack", "result": 0, "detail": 0}
        self.switch_works = True
        self.events = []        # (time, callable)

    def _run_events(self):
        for event in [e for e in self.events if e[0] <= self.clock()]:
            self.events.remove(event)
            event[1]()

    def send(self, frame):
        self.sent.append(frame)
        if frame == wp.build_wifi_packet(False):
            return self.ack
        if self.ack.get("result") == 0 and self.switch_works:
            self.phase = "maint"
            self.boot = self.clock()
        return self.ack

    def beacon(self):
        self._run_events()
        if self.phase != "maint":
            return None, 999.0
        packet = wp.decode_maintenance_status(BEACON_PAYLOAD)
        return packet, 1.0

    def flight(self):
        self._run_events()
        if self.phase != "flight":
            return None, None, 999.0
        return self.status, int((self.clock() - self.boot) * 1000), 1.0

    def to_flight(self):
        self.phase = "flight"
        self.boot = self.clock()


class FakeWifi(ota_wifi.WifiBackend):
    automatic = True

    def __init__(self, local_ip="127.0.0.1"):
        self.calls = []
        self.ip = local_ip

    def snapshot(self):
        self.calls.append("snapshot")
        return {"iface": "Wi-Fi", "ssid": "eduroam", "profile": "eduroam"}

    def visible_ssids(self):
        return ["eduroam", "witness-1A2B"]

    def join(self, ssid, password, snapshot):
        self.calls.append(("join", ssid))

    def local_ip(self, snapshot):
        return self.ip

    def restore(self, snapshot):
        self.calls.append(("restore", snapshot["ssid"]))


class WorkflowCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.clock = FakeClock()
        self.board = FakeBoard(self.clock)
        self.state = FakeState()
        board = self.board

        class H(Handler):
            def do_POST(handler):
                Handler.do_POST(handler)
                if handler.path == "/reboot" or (handler.path == "/ota" and board.upload_kind == "flight"):
                    board.to_flight()

        self.board.upload_kind = "flight"
        H.state = self.state
        self.server = HTTPServer(("127.0.0.1", 0), H)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.base = "http://127.0.0.1:%d" % self.server.server_address[1]
        self.wifi = FakeWifi()
        self.logs, self.steps = [], []
        self.ui = ow.Callbacks(step=lambda n, s, m="": self.steps.append((n, s)),
                               log=self.logs.append, confirm_join=lambda t: True,
                               show_credentials=lambda ssid, pw: self.logs.append("CRED " + ssid))

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def flow(self, **kw):
        ports = ow.Ports(self.board.send, self.board.beacon, self.board.flight)
        return ow.OtaWorkflow(ports, self.wifi, self.ui, base=self.base, sleep=self.clock.sleep,
                              clock=self.clock, timeouts={"stable": 10.0, "flight": 20.0, "reachable": 10.0},
                              **kw)


class ImageTests(unittest.TestCase):
    def test_parse_and_inspect(self):
        tmp = tempfile.mkdtemp()
        report = ow.inspect_image(write(tmp, "flight.bin", image_with_info(1, "vA")))
        self.assertEqual((report["kind"], report["version"]), ("flight", "vA"))
        self.assertEqual(report["info"]["store_version"], 4)
        self.assertNotEqual(report["id"], 0)
        with self.assertRaises(wo.OtaError):
            ow.inspect_image(write(tmp, "old.bin", make_image()))      # no image-info record

    def test_pair_check(self):
        tmp = tempfile.mkdtemp()
        f = ow.inspect_image(write(tmp, "flight.bin", image_with_info(1)))
        m_ok = ow.inspect_image(write(tmp, "maint.bin", image_with_info(2)))
        m_bad = ow.inspect_image(write(tmp, "m2.bin", image_with_info(2, sf=9)))
        self.assertIsNone(ow.check_pair(f["info"], m_ok["info"]))
        self.assertEqual(ow.check_pair(f["info"], m_bad["info"]), "LoRa settings differ")
        m_cmd = ow.inspect_image(write(tmp, "m3.bin", image_with_info(2, cmd=2)))
        self.assertEqual(ow.check_pair(f["info"], m_cmd["info"]), "command set differs")


class FlowTests(WorkflowCase):
    def test_flight_update_guided_no_wifi_changes(self):
        path = write(self.tmp, "flight.bin", image_with_info(1, "v2"))
        self.wifi.automatic = False        # guided: user joins by hand
        result = self.flow(auto_join=True).update(path)
        self.assertTrue(result.ok, result.message)
        self.assertEqual(self.wifi.calls, ["snapshot"])      # nothing joined, nothing to restore
        self.assertTrue(any("CRED witness-1A2B" in m for m in self.logs))
        self.assertEqual(self.state.uploaded, image_with_info(1, "v2"))

    def test_flight_update_auto_join_restores_before_confirm(self):
        path = write(self.tmp, "flight.bin", image_with_info(1, "v2"))
        result = self.flow(auto_join=True).update(path)
        self.assertTrue(result.ok, result.message)
        self.assertEqual(self.wifi.calls, ["snapshot", ("join", "witness-1A2B"), ("restore", "eduroam")])
        names = [n for n, s in self.steps if s == "running"]
        self.assertLess(names.index("restore"), names.index("confirm"))

    def test_join_needs_confirmation(self):
        self.ui.confirm_join = lambda text: False
        path = write(self.tmp, "flight.bin", image_with_info(1, "v2"))
        result = self.flow(auto_join=True).update(path)
        self.assertTrue(result.ok, result.message)
        self.assertNotIn(("join", "witness-1A2B"), self.wifi.calls)

    def test_refused_in_flight_state(self):
        self.board.status = 1                     # Boost
        result = self.flow().update(write(self.tmp, "flight.bin", image_with_info(1)))
        self.assertEqual(result.outcome, "refused")
        self.assertEqual(self.board.sent, [])
        self.assertEqual(self.wifi.calls, [])

    def test_switch_refused_by_firmware(self):
        self.board.ack = {"packet": "command_ack", "result": 1, "detail": 4,
                          "detail_name": "LoRa settings differ"}
        result = self.flow().update(write(self.tmp, "flight.bin", image_with_info(1)))
        self.assertEqual(result.outcome, "refused")
        self.assertIn("LoRa settings", result.message)
        self.assertEqual(self.wifi.calls, [])

    def test_switch_acked_but_board_stays_in_flight(self):
        self.board.switch_works = False
        result = self.flow().update(write(self.tmp, "flight.bin", image_with_info(1)))
        self.assertEqual(result.outcome, "refused")
        self.assertIn("still running its flight image", result.message)

    def test_mismatched_sibling_refused_before_anything_is_sent(self):
        write(self.tmp, "maint.bin", image_with_info(2, sf=9))
        result = self.flow().update(write(self.tmp, "flight.bin", image_with_info(1)))
        self.assertEqual(result.outcome, "refused")
        self.assertEqual(self.board.sent, [])

    def test_firmware_rejects_upload_wifi_restored_and_board_left(self):
        self.state.reject_upload = True
        result = self.flow(auto_join=True).update(write(self.tmp, "flight.bin", image_with_info(1)))
        self.assertEqual(result.outcome, "refused")
        self.assertEqual(self.wifi.calls[-1], ("restore", "eduroam"))
        self.assertEqual(self.board.phase, "flight")       # HTTP reboot sent the board back

    def test_cancel_mid_upload_restores_wifi(self):
        cancel = threading.Event()
        self.ui.progress = lambda done, total, label="": cancel.set()
        result = self.flow(auto_join=True, cancel=cancel).update(write(self.tmp, "flight.bin", image_with_info(1)))
        self.assertEqual(result.outcome, "cancelled")
        self.assertEqual(self.wifi.calls[-1], ("restore", "eduroam"))
        self.assertEqual(self.board.phase, "flight")

    def test_flight_rollback_detected_by_second_reset(self):
        self.board.events.append((self.clock() + 400, lambda: None))
        path = write(self.tmp, "flight.bin", image_with_info(1, "v2"))
        flow = self.flow()
        original_to_flight = self.board.to_flight

        def to_flight_then_reset():
            original_to_flight()
            self.board.events.append((self.clock() + 6, original_to_flight))   # self-test fails
        self.board.to_flight = to_flight_then_reset
        result = flow.update(path)
        self.assertEqual(result.outcome, "rolled_back")
        self.assertFalse(result.ok)

    def test_maintenance_update(self):
        self.board.upload_kind = "maintenance"
        path = write(self.tmp, "maint.bin", image_with_info(2, "v2"))
        result = self.flow(auto_join=True).update(path)
        self.assertTrue(result.ok, result.message)
        self.assertEqual(self.wifi.calls[-1], ("restore", "eduroam"))
        self.assertEqual(self.board.phase, "flight")

    def test_log_download(self):
        dest = os.path.join(self.tmp, "logs")
        result = self.flow(auto_join=True).download_logs(dest)
        self.assertTrue(result.ok, result.message)
        self.assertEqual(sorted(os.listdir(dest)), ["I0001.BIN", "S0001.TXT"])
        self.assertEqual(self.wifi.calls[-1], ("restore", "eduroam"))


NETSH_INTERFACES = """
There is 1 interface on the system:

    Name                   : Wi-Fi
    Description            : Intel(R) Wi-Fi 6
    State                  : connected
    SSID                   : eduroam
    BSSID                  : aa:bb:cc:dd:ee:ff
    Profile                : eduroam
"""


class NetshTests(unittest.TestCase):
    def setUp(self):
        self.commands = []
        self.ssid = "eduroam"
        self.clock = FakeClock()

        def run(command, timeout=30):
            self.commands.append(command)
            if command[:3] == ["netsh", "wlan", "show"] and command[3] == "interfaces":
                return 0, NETSH_INTERFACES.replace("eduroam", self.ssid)
            if command[:3] == ["netsh", "wlan", "connect"]:
                self.ssid = [c for c in command if c.startswith("ssid=")][0][5:]
            if command[:3] == ["netsh", "interface", "ip"]:
                return 0, "Configuration for interface \"Wi-Fi\"\n    IP Address:   192.168.4.2\n"
            return 0, "ok"
        self.backend = ota_wifi.NetshBackend(run=run, sleep=self.clock.sleep, clock=self.clock,
                                             profile_dir=tempfile.mkdtemp())

    def test_snapshot_join_restore_order(self):
        snap = self.backend.snapshot()
        self.assertEqual(snap, {"iface": "Wi-Fi", "ssid": "eduroam", "profile": "eduroam"})
        self.backend.join("witness-1A2B", "pw<&>", snap)
        self.assertEqual(self.backend.local_ip(snap), "192.168.4.2")
        self.backend.restore(snap)
        words = [" ".join(c[1:4]) for c in self.commands if c[1] == "wlan" and c[2] != "show"]
        self.assertEqual(words, ["wlan add profile", "wlan connect name=witness-ota-tmp",
                                 "wlan connect name=eduroam", "wlan delete profile"])
        self.assertEqual(self.ssid, "eduroam")
        self.assertEqual(os.listdir(self.backend.profile_dir), [])      # password file removed

    def test_profile_xml_is_manual_and_escaped(self):
        xml = ota_wifi.profile_xml("witness-1", "a<b&c")
        self.assertIn("<connectionMode>manual</connectionMode>", xml)
        self.assertIn("a&lt;b&amp;c", xml)

    def test_journal_recovery(self):
        journal = ota_wifi.RestoreJournal(os.path.join(tempfile.mkdtemp(), "restore.json"))
        self.assertFalse(ota_wifi.recover(self.backend, journal))
        journal.write({"iface": "Wi-Fi", "ssid": "eduroam", "profile": "eduroam"})
        self.assertTrue(ota_wifi.recover(self.backend, journal))
        self.assertIsNone(journal.read())
        self.assertTrue(any(c[2] == "connect" for c in self.commands))


if __name__ == "__main__":
    unittest.main()
