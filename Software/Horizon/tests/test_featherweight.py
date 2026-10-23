"""Observed Featherweight status traffic, CRC/error recovery and flight routing."""

import unittest
import time
import tkinter as tk

from featherweight import FeatherweightParseError, FeatherweightStreamParser, crc16_buypass, parse_featherweight_line
from flight_data import DEFAULT_LAYOUT, FlightData, load_layout
from ground_tabs import FEATHERWEIGHT_FIELDS
from telemetry_view import TelemetryView


# Unmodified records from the 2026-10-22 COM5 capture, with original CRCs.
CAPTURE = (
    "@ BATT_BLE 067 0000 00 00 00:00:04.615 3730 BLE- 0 degC CRC: 95ED",
    "@ BATT_BLE 067 0000 00 00 00:00:04.573 3730 BLE+ 0 degC CRC: 7C19",
    "@ RX_TMOUT 072  0000 00 00 00:00:38.898 Rx Timeout :( SF: 10 CRC: 8948",
    "@ GS_STAT  197 0000 00 00 00:00:38.898 CRC_OK ABJ SUS     PkRx    0 PkSnt    0 p_RSSI -50 AckRx    0 AckSnt    0 a_RSSI -50 temporary FndRx    0 FndSnt    0 f_RSSI -50 SF 10 f 906200012 CRC: 7231",
    "@ RX_NOMTK 199 0000 00 00 00:00:41.801 CRC_OK  Rx NomTrk ABJ SUS     PkRx     1 PkTx     0 RSSI -047 SNR +06 AckRx     0 AckTx     0 RSSI -050 SNR  +0 SF 10 frq 906200012 trk_B_V 0   +0 C CRC: 63EB",
    "@ RX_NOMTK 202 0000 00 00 00:01:43.802 CRC_OK  Rx NomTrk ABJ SUS     PkRx    62 PkTx    61 RSSI -047 SNR +06 AckRx     0 AckTx    61 RSSI -060 SNR  +5 SF 10 frq 906200000 trk_B_V 3513   +0 C CRC: 5923",
    "@ GPS_STAT 202 0000 00 00 00:00:42.828 CRC_OK  TRK ABJ SUS     Alt 000000 lt +00.00000 ln +00.00000 Vel +0000 +000 +0000 Fix 0 #  0  0  0  0 000_00_00 000_00_00 000_00_00 000_00_00 000_00_00 CRC: 7443",
    "@ TX_STAT  109 0000 00 00 00:00:42.854 ABJ SUS     Tx Apid 00 Tx dur: 62 msec SF10 Freq 906200012 CRC: 20D2",
    "@ TX_STAT  108 0000 00 00 00:00:42.377 ABJ SUS     Tx Apid 03 Tx dur: 17 msec SF7 Freq 925600000 CRC: 770B",
)


def with_crc(body):
    return f"{body} CRC: {crc16_buypass(body.encode('ascii')):04X}"


def replace_payload(line, old, new):
    return with_crc(line.split(" CRC:")[0].replace(old, new))


class FeatherweightStatusTests(unittest.TestCase):
    def test_original_capture_fields_and_units(self):
        packets = [parse_featherweight_line(line) for line in CAPTURE]
        self.assertTrue(all(p['text_crc_valid'] for p in packets))
        self.assertTrue(all(p['packet_type'] in FEATHERWEIGHT_FIELDS for p in packets))
        local_off, local_on, timeout, ground, empty, tracker, gps, tx, tx_coord = packets
        self.assertEqual(local_on['battery_mv'], 3730)
        self.assertEqual(local_on['battery_voltage_v'], 3.73)
        self.assertEqual(local_on['battery_scope'], 'LOCAL')
        self.assertTrue(local_on['ble_connected'])
        self.assertFalse(local_off['ble_connected'])
        self.assertEqual(local_on['temperature_c'], 0)
        self.assertIsNone(empty['tracker_battery_voltage_v'])
        self.assertEqual(empty['tracker_battery_mv'], 0)
        self.assertFalse(empty['tracker_battery_valid'])
        self.assertEqual(tracker['tracker_battery_voltage_v'], 3.513)
        self.assertEqual(tracker['unit_id'], 'ABJ SUS')
        self.assertEqual(tracker['packets_received'], 62)
        self.assertEqual(tracker['packets_sent'], 61)
        self.assertEqual(tracker['ack_sent'], 61)
        self.assertEqual((tracker['rssi_dbm'], tracker['ack_rssi_dbm']), (-47, -60))
        self.assertEqual((tracker['snr_db'], tracker['ack_snr_db']), (6, 5))
        self.assertEqual(tracker['frequency_hz'], 906200000)
        self.assertEqual(ground['found_unit_id'], 'temporary')
        self.assertEqual(ground['found_rssi_dbm'], -50)
        self.assertTrue(timeout['receive_timeout'])
        self.assertEqual(timeout['spreading_factor'], 10)
        self.assertEqual((gps['unit_type_raw'], gps['unit_type']), ('TRK', 'TRACKER'))
        self.assertEqual(gps['unit_id'], 'ABJ SUS')
        self.assertEqual((tx['apid'], tx['tx_duration_ms'], tx['spreading_factor']), (0, 62, 10))
        self.assertEqual((tx_coord['apid'], tx_coord['tx_duration_ms']), (3, 17))

    def test_fragmented_capture_with_binary_chatter(self):
        noise = b'FWT\x00@\xff\r\n'
        wire = noise + noise.join((line + '\r\n').encode() for line in CAPTURE)
        expected = [parse_featherweight_line(line) for line in CAPTURE]
        for size in (1, 7, 37, 4096):
            packets, errors = [], []
            parser = FeatherweightStreamParser(packets.append, errors.append)
            for offset in range(0, len(wire), size):
                parser.feed(wire[offset:offset + size])
            self.assertEqual(packets, expected)
            self.assertEqual(errors, [])

    def test_bad_status_recovers_and_unknown_packets_still_visible(self):
        for line, old, new in ((CAPTURE[1], '3730', '-1'), (CAPTURE[1], 'BLE+', 'BLE?'),
                               (CAPTURE[5], 'SNR +06', 'SNR nan'), (CAPTURE[5], ' +0 C', ''),
                               (CAPTURE[7], 'Freq 906200012', 'Freq')):
            malformed = replace_payload(line, old, new)
            with self.subTest(malformed=malformed):
                with self.assertRaises(FeatherweightParseError):
                    parse_featherweight_line(malformed)
                packets, errors = [], []
                parser = FeatherweightStreamParser(packets.append, errors.append)
                parser.feed((malformed + '\n' + CAPTURE[1] + '\n').encode())
                self.assertEqual(len(errors), 1)
                self.assertEqual(len(packets), 1)
        unknown = parse_featherweight_line(with_crc('@ NEW_TYPE 050 2026 10 22 12:00:00 extra 123'))
        self.assertEqual(unknown['payload_text'], 'extra 123')
        legacy = parse_featherweight_line(with_crc(
            '@ TX_STAT 091 2020 11 27 00:16:49.800 Tx Apid 11 Tx dur: 371 msec. SF12 Freq 926800000'))
        self.assertEqual(legacy['apid'], 11)
        self.assertNotIn('unit_id', legacy)

    def test_flight_view_separates_batteries_and_checks_quality(self):
        model = FlightData(load_layout(DEFAULT_LAYOUT))
        def accept(line, stamp):
            packet = parse_featherweight_line(line)
            model.accept_record('featherweight', {'source': 'COM5', 'received_mono': stamp,
                'packet': {**packet, 'packet': packet['packet_type']}})
        def history(title, index=0):
            row = next(i for i, row in enumerate(model.layout.rows) if row.title == title)
            return model.snapshot()[row][index]
        accept(CAPTURE[1], 1)
        self.assertEqual(history('FW local / GS battery').latest, 3.73)
        self.assertIsNone(history('FW tracker battery').latest)
        accept(CAPTURE[4], 2)
        self.assertIsNone(history('FW tracker battery').latest)
        accept(CAPTURE[5], 3)
        self.assertEqual(history('FW tracker battery').latest, 3.513)
        self.assertEqual(history('RSSI', 2).latest, -47)
        self.assertEqual(history('FW local / GS battery').received, 1)
        accept(CAPTURE[6], 4)
        self.assertEqual(history('Featherweight satellites').latest, 0)
        self.assertIsNone(history('Altitude', 1).latest)  # No GPS fix.
        accept(replace_payload(CAPTURE[6], 'Fix 0', 'Fix 3'), 5)
        self.assertEqual(history('Altitude', 1).latest, 0)
        self.assertEqual(history('Altitude', 1).quality, 'LIVE')
        accept(replace_payload(CAPTURE[5], 'CRC_OK', 'CRC_ERR'), 6)
        self.assertEqual(history('FW tracker battery').latest, 3.513)
        self.assertEqual(history('FW tracker battery').received, 3)
        self.assertEqual(history('RSSI', 2).quality, 'INVALID')
        accept(CAPTURE[5], 7)
        self.assertEqual(history('FW tracker battery').latest, 3.513)
        # Text CRC must protect battery values too, not just GPS.
        accept(CAPTURE[1].replace('3730', '9999'), 8)
        self.assertEqual(history('FW local / GS battery').quality, 'INVALID')
        self.assertEqual(history('FW tracker battery').latest, 3.513)


class FeatherweightDisplayTests(unittest.TestCase):
    def test_new_fields_and_radio_crc_quality_in_tab(self):
        try:
            root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest(f'Tk display unavailable: {exc}')
        root.withdraw()
        try:
            view = TelemetryView(root, schema=FEATHERWEIGHT_FIELDS)
            view.add_source('COM5')
            self.assertEqual(view.tree.set(view._rows[('COM5', 'BATT_BLE', 'battery_voltage_v')], 'value'), 'null')
            for line in CAPTURE:
                packet = parse_featherweight_line(line)
                view.update_record({'source': 'COM5', 'received_at': time.time(),
                    'packet': {**packet, 'packet': packet['packet_type']}})
            row = view._rows[('COM5', 'RX_NOMTK', 'tracker_battery_voltage_v')]
            self.assertEqual(view.tree.set(row, 'value'), '3.513')
            local = view._rows[('COM5', 'BATT_BLE', 'battery_voltage_v')]
            self.assertEqual(view.tree.set(local, 'value'), '3.73')
            packet = parse_featherweight_line(replace_payload(CAPTURE[5], 'CRC_OK', 'CRC_ERR'))
            view.update_record({'source': 'COM5', 'received_at': time.time(),
                'packet': {**packet, 'packet': 'RX_NOMTK'}})
            view.update_ages()
            self.assertEqual(view.tree.set(row, 'quality'), 'BAD RADIO CRC')
            self.assertIn('invalid', view.tree.item(row, 'tags'))
            self.assertEqual(view.tree.set(local, 'quality'), 'Live')
        finally:
            root.destroy()


if __name__ == '__main__':
    unittest.main()
