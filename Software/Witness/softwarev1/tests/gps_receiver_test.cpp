#include "gps_receiver.h"

#include <cassert>
#include <cstdio>
#include <initializer_list>
#include <string>
#include <vector>

using Bytes = std::vector<std::uint8_t>;
using Status = GpsReceiver::Status;
using Model = GpsReceiver::Model;

struct FakeUart {
    std::vector<Bytes> sent;
    std::vector<std::uint32_t> baud_changes;
    static bool send(void *context, const std::uint8_t *data, std::size_t length) {
        static_cast<FakeUart *>(context)->sent.emplace_back(data, data + length);
        return true;
    }
    static bool baud(void *context, std::uint32_t rate) {
        static_cast<FakeUart *>(context)->baud_changes.push_back(rate);
        return true;
    }
    GpsReceiver::Transport transport() { return {this, send, baud}; }
};

Bytes skytraq(std::initializer_list<std::uint8_t> payload) {
    Bytes bytes{0xA0, 0xA1, 0, static_cast<std::uint8_t>(payload.size())};
    std::uint8_t checksum = 0;
    for (const auto byte : payload) { bytes.push_back(byte); checksum ^= byte; }
    bytes.insert(bytes.end(), {checksum, 0x0D, 0x0A});
    return bytes;
}

Bytes ubx(std::uint8_t cls, std::uint8_t id, const Bytes &payload) {
    Bytes bytes{0xB5, 0x62, cls, id, static_cast<std::uint8_t>(payload.size()),
                static_cast<std::uint8_t>(payload.size() >> 8)};
    bytes.insert(bytes.end(), payload.begin(), payload.end());
    std::uint8_t a = 0, b = 0;
    for (std::size_t i = 2; i < bytes.size(); ++i) { a += bytes[i]; b += a; }
    bytes.push_back(a);
    bytes.push_back(b);
    return bytes;
}

void feed(GpsReceiver &gps, const Bytes &bytes, std::uint32_t time) {
    gps.consume(bytes.data(), bytes.size(), time);
}

// Runs service() past the boot delay so the first identification polls go
// out. heard: the receiver is streaming NMEA (polls are only sent then).
std::uint32_t start(GpsReceiver &gps, bool heard = true) {
    if (heard) gps.observe_nmea();
    gps.service(0);
    gps.service(GpsReceiver::kStartupDelayMs);
    gps.service(GpsReceiver::kStartupDelayMs + 1);
    return GpsReceiver::kStartupDelayMs + 2;
}

// Lets every attempt of the current step time out.
std::uint32_t time_out(GpsReceiver &gps, std::uint32_t time) {
    for (unsigned i = 0; i < GpsReceiver::kMaximumAttempts; ++i) {
        time += GpsReceiver::kReplyTimeoutMs;
        gps.service(time);
    }
    return time;
}

const Bytes kMonVerPoll{0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};

// After a restart: the restart wait ends, the receiver is identified again at
// the rate it answered at boot (it may come back at its default rate), then
// configuration starts. Returns the time to continue from.
std::uint32_t reidentify(GpsReceiver &gps, std::uint32_t time, bool ublox = false) {
    if (GpsReceiver::kFixedBaud) return time;   // configured straight away
    gps.service(time);                 // RestartWait -> Identify
    gps.observe_nmea();                // the receiver is talking again
    gps.service(time + 1);             // identification poll
    if (ublox) feed(gps, ubx(0x0A, 0x04, {}), time + 2);
    else feed(gps, skytraq({0x64, 0x8B, 0}), time + 2);
    return time + 3;
}

// Fixed-baud rule: nothing the transport ever sees may change the receiver
// baud (SkyTraq 0x05, u-blox CFG-PRT) or reset/save its configuration
// (SkyTraq 0x04 factory defaults, u-blox CFG-CFG).
void assert_no_forbidden(const FakeUart &uart) {
    for (const auto &f : uart.sent) {
        if (f.size() > 4 && f[0] == 0xA0 && f[1] == 0xA1) assert(f[4] != 0x05 && f[4] != 0x04);
        if (f.size() > 3 && f[0] == 0xB5 && f[1] == 0x62)
            assert(!(f[2] == 0x06 && (f[3] == 0x00 || f[3] == 0x09)));
    }
}

std::uint32_t finish_skytraq(GpsReceiver &gps, FakeUart &uart, std::uint32_t time, bool acks = true) {
    // Airborne mode (frame verified on hardware), NMEA GGA+RMC, then 25 Hz.
    gps.service(time);
    assert(uart.sent.back() == (Bytes{0xA0, 0xA1, 0x00, 0x04, 0x64, 0x17, 0x05, 0x00,
                                      0x76, 0x0D, 0x0A}));
    if (acks) feed(gps, skytraq({0x83, 0x64, 0x17}), time + 1);
    else time = time_out(gps, time);
    gps.service(time + 2);
    assert(uart.sent.back() == skytraq({0x08, 1, 5, 0, 0, 1, 0, 0, 0}));
    if (acks) feed(gps, skytraq({0x83, 0x08}), time + 3);
    else time = time_out(gps, time + 2);
    // All constellations, elevation mask, QZSS, SBAS (optional settings).
    const Bytes optional_cmds[] = {skytraq({0x64, 0x19, 0x00, 0x0F, 0x00}),
                                   skytraq({0x2B, 0x02, 0x03, 0x00, 0x00}),
                                   skytraq({0x62, 0x03, 0x01, 0x01, 0x00}),
                                   skytraq({0x62, 0x01, 0x01, 0x02, 0x08, 0x01, 0x01, 0x07, 0x00})};
    for (const auto &cmd : optional_cmds) {
        gps.service(time + 4);
        assert(uart.sent.back() == cmd);
        if (acks) {
            const std::uint8_t id = cmd[4];
            if (id == 0x64 || id == 0x62) feed(gps, skytraq({0x83, id, cmd[5]}), time + 4);
            else feed(gps, skytraq({0x83, id}), time + 4);
        } else {
            time = time_out(gps, time + 4);
        }
    }
    gps.service(time + 4);
    assert(uart.sent.back() == skytraq({0x0E, 25, 0}));
    if (acks) feed(gps, skytraq({0x83, 0x0E}), time + 5);
    else time = time_out(gps, time + 4);
    assert(gps.status() == Status::Configuring);  // ACKs alone are insufficient.
    gps.service(time + 6);
    assert(uart.sent.back() == skytraq({0x64, 0x18}));
    feed(gps, skytraq({0x64, 0x8B, 0x05}), time + 7);
    gps.service(time + 8);
    assert(uart.sent.back() == skytraq({0x10}));
    feed(gps, skytraq({0x86, 25}), time + 9);
    // G7: without ACKs the NMEA message set (no readback) is unproven.
    assert(gps.status() == (acks ? Status::Ready : Status::Streaming));
    assert(gps.update_rate_hz() == 25);
    return time + 9;
}

void finish_ublox(GpsReceiver &gps, FakeUart &uart, std::uint32_t time) {
    const Bytes expected_msgs[] = {{0xF0, 0x01, 0}, {0xF0, 0x02, 5}, {0xF0, 0x03, 0},
                                   {0xF0, 0x05, 0}, {0xF0, 0x00, 1}, {0xF0, 0x04, 1}};
    for (const auto &msg : expected_msgs) {
        gps.service(time++);
        assert(uart.sent.back() == ubx(0x06, 0x01, msg));
        feed(gps, ubx(0x05, 0x01, {0x06, 0x01}), time++);
    }
    gps.service(time++);
    Bytes nav5(36, 0);
    nav5[0] = 0x01;  // mask: dynamic model only
    nav5[2] = 8;     // airborne < 4 g
    assert(uart.sent.back() == ubx(0x06, 0x24, nav5));
    feed(gps, ubx(0x05, 0x01, {0x06, 0x24}), time++);
    gps.service(time++);
    assert(uart.sent.back() == (Bytes{0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00,
                                      0x01, 0x00, 0x01, 0x00, 0x7A, 0x12}));
    feed(gps, ubx(0x05, 0x01, {0x06, 0x08}), time++);
    assert(gps.status() == Status::Configuring);

    gps.service(time++);
    assert(uart.sent.back() == ubx(0x06, 0x24, {}));
    Bytes nav5_reply(36, 0);
    nav5_reply[2] = 8;
    feed(gps, ubx(0x06, 0x24, nav5_reply), time++);
    gps.service(time++);
    assert(uart.sent.back() == ubx(0x06, 0x08, {}));
    feed(gps, ubx(0x06, 0x08, {0x64, 0x00, 0x01, 0x00, 0x01, 0x00}), time++);
    assert(gps.status() == Status::Ready);
    assert(gps.update_rate_hz() == 10);
}

int main() {
    // PX1120S at 115200: identified by its binary reply; both polls are sent.
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        gps.service(0);
        gps.service(1499);
        assert(uart.sent.empty());
        auto time = start(gps);
        assert(uart.sent.size() == 2);
        assert(uart.sent[0] == (Bytes{0xA0, 0xA1, 0, 2, 0x64, 0x18, 0x7C, 0x0D, 0x0A}));
        assert(uart.sent[1] == kMonVerPoll);
        auto damaged = skytraq({0x64, 0x8B, 0});
        damaged[damaged.size() - 3] ^= 1;
        feed(gps, damaged, time);
        assert(gps.model() == Model::Unknown);
        for (const auto byte : skytraq({0x64, 0x8B, 0})) gps.consume(&byte, 1, time);
        assert(gps.model() == Model::Px1120s && !gps.model_assumed());
        finish_skytraq(gps, uart, time + 1);
        assert(uart.baud_changes.empty());
    }
    // PX1120S firmware that misses ACKs still configures; readback decides.
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        feed(gps, skytraq({0x64, 0x8B, 0}), time);
        finish_skytraq(gps, uart, time + 1, false);
    }
    // Valid NMEA but no binary reply: no baud hunting, assume the PX1120S.
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        gps.observe_nmea();
        time = time_out(gps, time);
        assert(gps.model() == Model::Px1120s && gps.model_assumed());
        assert(uart.baud_changes.empty());
        finish_skytraq(gps, uart, time + 1);
    }
    // PX1120S NACK fails without changing baud.
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        feed(gps, skytraq({0x64, 0x8B, 0}), time);
        gps.service(time + 1);
        feed(gps, skytraq({0x84, 0x64, 0x17}), time + 2);
        assert(gps.status() == Status::Failed);
        assert(uart.baud_changes.empty());
    }
    // PX1120S rate readback mismatch fails.
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        feed(gps, skytraq({0x64, 0x8B, 0}), time++);
        const Bytes acks[] = {skytraq({0x83, 0x64, 0x17}), skytraq({0x83, 0x08}),
                              skytraq({0x83, 0x64, 0x19}), skytraq({0x83, 0x2B}),
                              skytraq({0x83, 0x62, 0x03}), skytraq({0x83, 0x62, 0x01}),
                              skytraq({0x83, 0x0E})};
        for (const auto &ack : acks) {
            gps.service(time++);
            feed(gps, ack, time++);
        }
        gps.service(time++);
        feed(gps, skytraq({0x64, 0x8B, 0x05}), time++);
        gps.service(time++);
        feed(gps, skytraq({0x86, 1}), time++);
        assert(gps.status() == Status::Failed);
    }
    // u-blox already at 115200: identified by MON-VER, configured in place.
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        Bytes version(160, 0x20);  // Longer than the stored payload prefix.
        const auto reply = ubx(0x0A, 0x04, version);
        for (const auto byte : reply) gps.consume(&byte, 1, time);
        assert(gps.model() == Model::UbloxM8);
        finish_ublox(gps, uart, time + 1);
        assert(uart.baud_changes.empty());
    }
    // u-blox at factory 9600: silent at 115200, found at 9600, moved to 115200.
    if (!GpsReceiver::kFixedBaud) {   // baud scanning is off on the bench
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps, false);
        assert(uart.sent.empty());              // silent line: nothing transmitted
        time = time_out(gps, time);
        assert(uart.baud_changes == std::vector<std::uint32_t>{9600});
        assert(gps.baud() == 9600 && gps.model() == Model::Unknown);
        assert(uart.sent.empty());
        gps.observe_nmea();                     // NMEA at 9600: now it is polled
        gps.service(time + 1);
        assert(uart.sent.back() == kMonVerPoll);
        feed(gps, ubx(0x0A, 0x04, Bytes(40, 0x20)), time + 2);
        assert(gps.model() == Model::UbloxM8);
        gps.service(time + 3);
        const Bytes prt{0x01, 0, 0, 0, 0xD0, 0x08, 0, 0, 0x00, 0xC2, 0x01, 0x00,
                        0x03, 0x00, 0x03, 0x00, 0, 0, 0, 0};
        assert(uart.sent.back() == ubx(0x06, 0x00, prt));
        feed(gps, ubx(0x05, 0x01, {0x06, 0x00}), time + 4);  // ACK at old rate
        gps.service(time + 4);
        assert(uart.baud_changes.size() == 1);  // waits for the receiver to switch
        time += 3 + GpsReceiver::kBaudSwitchDelayMs;
        gps.service(time);
        assert(uart.baud_changes.back() == 115200 && gps.baud() == 115200);
        gps.service(time + 1);
        assert(uart.sent.back() == kMonVerPoll);
        feed(gps, ubx(0x0A, 0x04, Bytes(40, 0x20)), time + 2);
        finish_ublox(gps, uart, time + 3);
        assert(uart.baud_changes.size() == 2);
    }
    // u-blox that does not answer after the baud change: host follows it back.
    if (!GpsReceiver::kFixedBaud) {   // baud scanning is off on the bench
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = time_out(gps, start(gps, false));
        gps.observe_nmea();
        gps.service(time + 1);
        feed(gps, ubx(0x0A, 0x04, Bytes(40, 0x20)), time + 2);
        gps.service(time + 3);
        time += 3 + GpsReceiver::kBaudSwitchDelayMs;
        gps.service(time);      // host moves to 115200
        gps.service(time + 1);  // first MON-VER poll at the new rate
        time = time_out(gps, time + 1);
        assert(gps.status() == Status::Failed);
        assert(uart.baud_changes.back() == 9600);
    }
    // Nothing at any rate: every rate is tried, then the host returns to the
    // operational rate.
    if (!GpsReceiver::kFixedBaud) {   // baud scanning is off on the bench
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = time_out(gps, start(gps, false));
        for (std::size_t i = 1; i < GpsReceiver::kScanBaudCount; ++i) {
            gps.service(++time);  // listens at the next rate
            time = time_out(gps, time);
        }
        assert(gps.status() == Status::Failed);
        assert(uart.sent.empty());   // never heard: never transmitted
        std::vector<std::uint32_t> expected(GpsReceiver::kScanBauds + 1,
                                            GpsReceiver::kScanBauds + GpsReceiver::kScanBaudCount);
        expected.push_back(115200);
        assert(uart.baud_changes == expected);
    }
    // PX1120S streaming NMEA at 460800: found by the scan (NMEA, no binary
    // reply), moved to 115200 with SkyTraq 0x05 in SRAM, then configured.
    if (!GpsReceiver::kFixedBaud) {   // baud scanning is off on the bench
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = time_out(gps, start(gps, false));   // 115200: garbage only
        while (gps.baud() != 460800) {
            gps.service(++time);
            time = time_out(gps, time);
        }
        gps.service(++time);                            // poll at 460800
        gps.observe_nmea();                              // valid GGA at this rate
        time = time_out(gps, time);
        assert(gps.model() == Model::Px1120s && gps.model_assumed());
        gps.service(++time);
        assert(uart.sent.back() == skytraq({0x05, 0x00, 0x05, 0x00}));
        assert(gps.baud() == 460800);                    // waits for the receiver
        time += GpsReceiver::kBaudSwitchDelayMs;
        gps.service(time);
        assert(gps.baud() == 115200 && uart.baud_changes.back() == 115200);
        gps.service(++time);
        assert(uart.sent.back() == skytraq({0x64, 0x18}));
        feed(gps, skytraq({0x64, 0x8B, 0}), ++time);    // answers at 115200
        finish_skytraq(gps, uart, time + 1);
    }
    // A measured rate is tried first: 460800 measured -> polls at 460800.
    if (!GpsReceiver::kFixedBaud) {   // baud scanning is off on the bench
        assert(GpsReceiver::nearest_scan_baud(452000) == 460800);
        assert(GpsReceiver::nearest_scan_baud(118000) == 115200);
        assert(GpsReceiver::nearest_scan_baud(75000) == 0);
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        gps.set_baud_hint(458000);
        auto time = start(gps, false);
        assert(uart.baud_changes == std::vector<std::uint32_t>{460800} && gps.baud() == 460800);
        time = time_out(gps, time);                     // nothing there: next is 115200
        gps.service(++time);
        assert(gps.baud() == 115200);
    }
    // Nothing on one orientation: the pins are flipped and every rate tried
    // again; a receiver found there is configured normally.
    if (!GpsReceiver::kFixedBaud) {   // baud scanning is off on the bench
        static int swaps = 0;
        FakeUart uart;
        GpsReceiver::Transport t = uart.transport();
        t.swap_pins = [](void *) { ++swaps; return true; };
        GpsReceiver gps(t);
        auto time = time_out(gps, start(gps, false));
        for (std::size_t i = 1; i < GpsReceiver::kScanBaudCount; ++i) {
            gps.service(++time);
            time = time_out(gps, time);
        }
        assert(swaps == 1 && gps.pins_swapped() && gps.status() == Status::Configuring);
        assert(gps.baud() == 115200 && uart.sent.empty());
        gps.observe_nmea();                             // heard on the flipped pins
        gps.service(++time);                            // poll at 115200, flipped
        feed(gps, skytraq({0x64, 0x8B, 0}), ++time);
        finish_skytraq(gps, uart, time + 1);
    }
    // A non-standard measured rate is tried as measured, first.
    if (!GpsReceiver::kFixedBaud) {   // baud scanning is off on the bench
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        gps.set_baud_hint(88398);
        start(gps);
        assert(gps.baud() == 88398);
    }
    // A PX1120S that ignores the change: the host follows it back and fails.
    if (!GpsReceiver::kFixedBaud) {   // baud scanning is off on the bench
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = time_out(gps, start(gps, false));
        gps.observe_nmea();
        gps.service(++time);                            // poll at 9600
        feed(gps, skytraq({0x64, 0x8B, 0}), ++time);    // binary reply at 9600
        assert(gps.model() == Model::Px1120s && !gps.model_assumed());
        gps.service(++time);                            // 0x05
        time += GpsReceiver::kBaudSwitchDelayMs;
        gps.service(time);
        gps.service(++time);
        time = time_out(gps, time);
        assert(gps.status() == Status::Failed && gps.baud() == 9600);
    }
    // One that ignores the change at a fast enough rate (88.4 kbaud seen after
    // a SkyTraq firmware update) is configured where it talks; cold start works.
    if (!GpsReceiver::kFixedBaud) {   // baud scanning is off on the bench
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        gps.set_baud_hint(88398);
        auto time = start(gps);
        feed(gps, skytraq({0x64, 0x8B, 0}), ++time);    // binary reply at 88398
        gps.service(++time);                            // 0x05
        time += GpsReceiver::kBaudSwitchDelayMs;
        gps.service(time);                              // host to 115200
        gps.service(++time);
        time = time_out(gps, time);                     // silent there: back to 88398
        assert(gps.baud() == 88398 && gps.status() == Status::Configuring);
        finish_skytraq(gps, uart, time + 1);
        assert(gps.status() == Status::Ready && gps.baud() == 88398);
        assert(gps.cold_start(time + 1000));
    }
    // Fixed baud: a silent receiver is never hunted through the scan rates and
    // no baud command is ever sent. Only the measured line rate (here the
    // 88.4 kbaud seen on the bench 2026-11-14) is listened to, once, on the
    // probed orientation; then the host returns to 115200.
    if (GpsReceiver::kFixedBaud) {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        gps.set_baud_hint(88398);
        auto time = time_out(gps, start(gps, false));   // nothing at 115200
        gps.service(++time);
        assert(gps.baud() == 88398);                    // listen at the measured rate
        time = time_out(gps, time);
        for (int i = 0; i < 20; ++i) {
            gps.service(++time);
            time = time_out(gps, time);
        }
        assert(gps.status() == Status::Failed && gps.baud() == 115200);
        assert((uart.baud_changes == std::vector<std::uint32_t>{88398, 115200}));
        for (const auto &frame : uart.sent) assert(frame.size() < 5 || frame[4] != 0x05);
    }
    // Fixed baud: a measurement near 115200 (or too slow) adds no extra rate.
    if (GpsReceiver::kFixedBaud) {
        for (const std::uint32_t measured : {118000U, 38400U, 0U}) {
            FakeUart uart;
            GpsReceiver gps(uart.transport());
            gps.set_baud_hint(measured);
            auto time = time_out(gps, start(gps, false));
            for (int i = 0; i < 5; ++i) {
                gps.service(++time);
                time = time_out(gps, time);
            }
            assert(uart.baud_changes.empty() && gps.baud() == 115200);
            assert(gps.status() == Status::Failed);
        }
    }
    // Fixed baud: a PX1120S talking at the measured rate is identified and
    // configured there (RAM-only settings, no 0x05), and can sleep.
    if (GpsReceiver::kFixedBaud) {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        gps.set_baud_hint(88398);
        auto time = time_out(gps, start(gps, false));
        gps.service(++time);                            // host to 88398
        assert(gps.baud() == 88398);
        gps.observe_nmea();                             // valid NMEA there
        time += GpsReceiver::kReplyTimeoutMs;           // the silent attempt expires
        gps.service(time);                              // identification polls
        assert(uart.sent.size() >= 2 && uart.sent[uart.sent.size() - 2] == skytraq({0x64, 0x18}));
        assert(uart.sent.back() == kMonVerPoll);
        feed(gps, skytraq({0x64, 0x8B, 0}), ++time);
        finish_skytraq(gps, uart, time + 1);
        assert(gps.status() == Status::Ready && gps.baud() == 88398);
        for (const auto &frame : uart.sent) assert(frame.size() < 5 || frame[4] != 0x05);
        assert(gps.sleep(time + 100));
    }
    // Fixed baud: identification failed because the receiver was still booting
    // (no measured rate); a retry with the rate measured later finds it.
    if (GpsReceiver::kFixedBaud) {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = time_out(gps, start(gps, false));
        for (int i = 0; i < 5; ++i) {
            gps.service(++time);
            time = time_out(gps, time);
        }
        assert(gps.status() == Status::Failed && gps.model() == Model::Unknown);
        assert(gps.retry_identification(time));
        assert(gps.status() == Status::Configuring && gps.identification_retries() == 1);
        gps.set_baud_hint(88398);                       // re-probe measured the line
        gps.service(++time);                            // first (silent) attempt at 115200
        time = time_out(gps, time);                     // nothing at 115200
        gps.service(++time);
        assert(gps.baud() == 88398);
        gps.observe_nmea();
        time += GpsReceiver::kReplyTimeoutMs;
        gps.service(time);
        feed(gps, skytraq({0x64, 0x8B, 0}), ++time);
        finish_skytraq(gps, uart, time + 1);
        assert(gps.status() == Status::Ready && gps.baud() == 88398);
        assert(!gps.retry_identification(time + 5000));   // Ready: nothing to retry
    }
    // Fixed baud: one GGA decoded at 115200 at power-on made the PX1120S
    // "assumed"; it then failed its readback (it really talks at 88.4 kbaud).
    // A failed assumed model may be retried; a real one may not.
    if (GpsReceiver::kFixedBaud) {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps, true);                   // NMEA seen at 115200
        time = time_out(gps, time);                     // no binary reply: assumed
        assert(gps.model() == Model::Px1120s && gps.model_assumed());
        for (int i = 0; i < 40 && gps.status() == Status::Configuring; ++i) {
            gps.service(++time);
            time = time_out(gps, time);                 // no ACKs, no readback
        }
        assert(gps.status() == Status::Failed);
        assert(gps.can_retry_identification());
        assert(gps.retry_identification(time));
        assert(gps.model() == Model::Unknown && !gps.model_assumed() &&
               gps.status() == Status::Configuring && gps.baud() == 115200);
    }
    // Fixed baud, nothing on either orientation: the pins are flipped once to
    // look, then flipped back to the probed orientation before giving up (the
    // bench board ended on the silent pin before this).
    if (GpsReceiver::kFixedBaud) {
        static int swaps = 0;
        swaps = 0;
        FakeUart uart;
        GpsReceiver::Transport t = uart.transport();
        t.swap_pins = [](void *) { ++swaps; return true; };
        GpsReceiver gps(t);
        auto time = time_out(gps, start(gps, false));
        for (int i = 0; i < 10; ++i) {
            gps.service(++time);
            time = time_out(gps, time);
        }
        assert(gps.status() == Status::Failed);
        assert(swaps == 2 && !gps.pins_swapped());
    }
    // Fixed baud, measured 88.4 kbaud, receiver on the other pin (bench
    // 2026-11-17 power-on): the measured rate is tried on BOTH orientations.
    if (GpsReceiver::kFixedBaud) {
        static int swaps = 0;
        swaps = 0;
        FakeUart uart;
        GpsReceiver::Transport t = uart.transport();
        t.swap_pins = [](void *) { ++swaps; return true; };
        GpsReceiver gps(t);
        gps.set_baud_hint(88398);
        auto time = time_out(gps, start(gps, false));   // 115200, as wired: nothing
        gps.service(++time);
        assert(gps.baud() == 88398 && swaps == 0);      // measured, as wired: nothing
        time = time_out(gps, time);
        gps.service(++time);
        assert(swaps == 1 && gps.pins_swapped() && gps.baud() == 115200);
        time = time_out(gps, time);
        gps.service(++time);
        assert(gps.baud() == 88398 && gps.pins_swapped());   // measured, swapped
        gps.observe_nmea();                                  // the receiver is here
        time = time_out(gps, time);
        gps.service(++time);
        assert(gps.model() == Model::Px1120s && gps.baud() == 88398 && gps.pins_swapped());
    }
    // A start rate verified by decoding NMEA at the probe is tried first.
    if (GpsReceiver::kFixedBaud) {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        gps.set_baud_hint(88398);
        gps.set_start_baud(88398);
        start(gps, false);
        assert(gps.baud() == 88398 && uart.baud_changes.front() == 88398);
    }
    // u-blox NAK and readback mismatch both fail.
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        feed(gps, ubx(0x0A, 0x04, {}), time);
        gps.service(time + 1);
        feed(gps, ubx(0x05, 0x01, {0x06, 0x24}), time + 2);  // ACK for another message
        assert(gps.status() == Status::Configuring);
        feed(gps, ubx(0x05, 0x00, {0x06, 0x01}), time + 3);
        assert(gps.status() == Status::Failed);
    }
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        feed(gps, ubx(0x0A, 0x04, {}), time++);
        for (int i = 0; i < 8; ++i) {
            gps.service(time++);
            const auto &sent = uart.sent.back();
            feed(gps, ubx(0x05, 0x01, {sent[2], sent[3]}), time++);
        }
        gps.service(time++);
        Bytes nav5_reply(36, 0);
        nav5_reply[2] = 0;  // portable, not airborne
        feed(gps, ubx(0x06, 0x24, nav5_reply), time++);
        assert(gps.status() == Status::Failed);
    }
    // A u-blox that never acknowledges configuration fails.
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        feed(gps, ubx(0x0A, 0x04, {}), time);
        gps.service(time + 1);  // first configuration command
        time_out(gps, time + 1);
        assert(gps.status() == Status::Failed);
    }
    // Ephemeris assistance on a configured PX1120S: one record at a time,
    // each waiting for its ACK/NACK; an unanswered record is retried once.
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        feed(gps, skytraq({0x64, 0x8B, 0}), time);
        finish_skytraq(gps, uart, time + 1);
        time += 100;
        Bytes eph(87, 0);
        eph[0] = 0x41;
        Bytes blob;
        for (std::uint8_t sv : {3, 7, 9}) {
            eph[2] = sv;
            blob.push_back(0);
            blob.push_back(87);
            blob.insert(blob.end(), eph.begin(), eph.end());
        }
        assert(gps.begin_assist(blob.data(), blob.size(), time));
        assert(!gps.begin_assist(blob.data(), blob.size(), time));  // one at a time
        assert(gps.assist_progress().total == 3 && gps.assist_progress().active);
        const auto sent_before = uart.sent.size();
        gps.service(++time);
        assert(uart.sent.size() == sent_before + 1);
        assert(uart.sent.back().size() == 94 && uart.sent.back()[4] == 0x41 &&
               uart.sent.back()[6] == 3);
        gps.service(++time);                      // waiting: nothing new
        assert(uart.sent.size() == sent_before + 1);
        feed(gps, skytraq({0x83, 0x41}), ++time); // ACK SV 3
        gps.service(++time);
        assert(uart.sent.back()[6] == 7);
        feed(gps, skytraq({0x84, 0x41}), ++time); // NACK SV 7
        gps.service(++time);
        assert(uart.sent.back()[6] == 9);
        time += GpsReceiver::kReplyTimeoutMs;     // SV 9 unanswered: retried once
        gps.service(time);
        assert(uart.sent.back()[6] == 9 && uart.sent.size() == sent_before + 4);
        time += GpsReceiver::kReplyTimeoutMs;
        gps.service(time);
        const auto progress = gps.assist_progress();
        assert(!progress.active && progress.sent == 3 && progress.acked == 1 &&
               progress.nacked == 1 && progress.timed_out == 1 && progress.uploads == 1);
        Bytes bad{0, 200};
        assert(!gps.begin_assist(bad.data(), bad.size(), time));
    }
    // Cold start: SkyTraq 0x01 mode 3, then a full reconfiguration.
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        feed(gps, skytraq({0x64, 0x8B, 0}), time);
        finish_skytraq(gps, uart, time + 1);
        time += 50;
        assert(gps.cold_start(time));
        assert(uart.sent.back() == skytraq({0x01, 0x03, 0x07, 0xEA, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
        assert(gps.status() == Status::Configuring && gps.cold_starts() == 1);
        const auto sent = uart.sent.size();
        gps.service(time + GpsReceiver::kRestartDelayMs - 1);   // still waiting
        assert(uart.sent.size() == sent);
        finish_skytraq(gps, uart, reidentify(gps, time + GpsReceiver::kRestartDelayMs));
    }
    // Host hot start through the assist path: 0x01 mode 1 with UTC and
    // position; once acknowledged, the receiver is reconfigured as after a
    // cold start and later records are dropped.
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        feed(gps, skytraq({0x64, 0x8B, 0}), time);
        finish_skytraq(gps, uart, time + 1);
        time += 50;
        // G4: only 0x41 ephemeris records are allowed now; a 0x01 restart
        // record in an upload is refused (the receiver-side restart handler
        // stays for the day a whitelist entry needs it).
        Bytes blob{0, 15, 0x01, 0x01, 0x07, 0xEA, 11, 11, 10, 31, 20, 0x0D, 0xCE, 0xD1, 0xFA, 0x02, 0x61,
                   0, 2, 0x0E, 0x01};
        const auto sent = uart.sent.size();
        assert(!gps.begin_assist(blob.data(), blob.size(), time));
        gps.service(++time);
        assert(uart.sent.size() == sent && gps.status() == Status::Ready);
    }
    // u-blox cold start: CFG-RST (no ACK), then reconfiguration.
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        feed(gps, ubx(0x0A, 0x04, {}), time);
        finish_ublox(gps, uart, time + 1);
        time += 100;
        assert(gps.cold_start(time));
        assert(uart.sent.back() == ubx(0x06, 0x04, {0xFF, 0xFF, 0x02, 0x00}));
        finish_ublox(gps, uart, reidentify(gps, time + GpsReceiver::kRestartDelayMs, true));
    }
    // Landed low-power cycle, SkyTraq: doze (AN0037 0x64 0x1C), then hot-start
    // restart (0x01 mode 1) and a full reconfiguration; cold start is only the
    // explicit fallback. The baud rate is never touched.
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        feed(gps, skytraq({0x64, 0x8B, 0}), time);
        finish_skytraq(gps, uart, time + 1);
        time += 50;
        const auto baud_changes = uart.baud_changes.size();
        assert(!gps.wake(time, GpsReceiver::WakeKind::Hot));      // not asleep
        assert(gps.sleep(time));
        assert(uart.sent.back() == (Bytes{0xA0, 0xA1, 0x00, 0x02, 0x64, 0x1C, 0x78, 0x0D, 0x0A}));
        assert(gps.asleep() && gps.sleeps() == 1);
        assert(!gps.sleep(time + 1));                              // already asleep
        time += 600'000;
        assert(gps.wake(time, GpsReceiver::WakeKind::Hot));
        assert(uart.sent.back() == skytraq({0x01, 0x01, 0x07, 0xEA, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
        assert(!gps.asleep() && gps.cold_starts() == 0 && gps.status() == Status::Configuring);
        const auto sent = uart.sent.size();
        gps.service(time + GpsReceiver::kRestartDelayMs - 1);
        assert(uart.sent.size() == sent);
        finish_skytraq(gps, uart, reidentify(gps, time + GpsReceiver::kRestartDelayMs));
        assert(gps.status() == Status::Ready);
        assert(uart.baud_changes.size() == baud_changes || !GpsReceiver::kFixedBaud);
        // Fallback: second cycle wakes with a cold start.
        time += 700'000;
        assert(gps.sleep(time));
        assert(gps.wake(time + 1000, GpsReceiver::WakeKind::Cold));
        assert(uart.sent.back()[5] == 0x03 && gps.cold_starts() == 1);
    }
    // Landed low-power cycle, u-blox: UBX-RXM-PMREQ backup, wake by RXD1 edges.
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        feed(gps, ubx(0x0A, 0x04, {}), time);
        finish_ublox(gps, uart, time + 1);
        time += 100;
        assert(gps.sleep(time));
        assert(uart.sent.back() == ubx(0x02, 0x41, {0, 0, 0, 0, 2, 0, 0, 0}));
        assert(gps.wake(time + 600'000, GpsReceiver::WakeKind::Hot));
        assert(uart.sent.back() == (Bytes{0xFF, 0xFF, 0xFF, 0xFF}));
        finish_ublox(gps, uart, reidentify(gps, time + 600'000 + GpsReceiver::kRestartDelayMs, true));
        assert(gps.status() == Status::Ready);
    }
    // Sleep refused with no identified receiver.
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        assert(!gps.sleep(0) && !gps.asleep());
    }
    // Nothing identified: refused.
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        assert(!gps.cold_start(0));
    }
    // A rejected optional setting is logged and configuration goes on; an ACK
    // for another sub-ID message does not count.
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        feed(gps, skytraq({0x64, 0x8B, 0}), time);
        gps.service(++time);
        feed(gps, skytraq({0x83, 0x64, 0x17}), ++time);
        gps.service(++time);
        feed(gps, skytraq({0x83, 0x08}), ++time);
        gps.service(++time);                                    // 64 19 constellations
        feed(gps, skytraq({0x83, 0x64, 0x17}), ++time);         // wrong sub-ID: ignored
        assert(uart.sent.back() == skytraq({0x64, 0x19, 0x00, 0x0F, 0x00}));
        feed(gps, skytraq({0x84, 0x64, 0x19}), ++time);         // NACK: optional
        assert(gps.status() == Status::Configuring && gps.optional_rejects() == 1);
        gps.service(++time);
        assert(uart.sent.back() == skytraq({0x2B, 0x02, 0x03, 0x00, 0x00}));
    }
    // Not configured: refused.
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        Bytes blob{0, 1, 0x41};
        assert(!gps.begin_assist(blob.data(), blob.size(), 0));
    }
    // ---- Supervision (audit G1, G2, G5, G7) ----
    if (GpsReceiver::kFixedBaud) {
        // G2: a NACK while valid NMEA streams: usable (Streaming), no
        // re-probe, no baud change, no pin swap; only the configuration is
        // retried later at this baud and these pins, with a backoff.
        static int swaps = 0;
        swaps = 0;
        FakeUart uart;
        GpsReceiver::Transport t = uart.transport();
        t.swap_pins = [](void *) { ++swaps; return true; };
        GpsReceiver gps(t);
        auto time = start(gps);
        feed(gps, skytraq({0x64, 0x8B, 0}), time);
        gps.observe_gga(time, 1);
        gps.service(time + 1);
        feed(gps, skytraq({0x84, 0x64, 0x17}), time + 2);
        assert(gps.status() == Status::Failed);
        time += 3;
        gps.observe_gga(time, 1);
        gps.service(time);
        assert(gps.status() == Status::Streaming && !gps.configured());
        assert(std::string(gps.state_text(time)) == "streaming (unconfigured)");
        assert(!gps.retry_identification(time) && !gps.probe_due(time));
        std::uint32_t t0 = time;
        for (; time < t0 + 4000; time += 40) { gps.observe_gga(time, 1); gps.service(time); }
        assert(gps.status() == Status::Streaming && gps.config_retries() == 0);   // backoff
        for (; time < t0 + 6000; time += 40) { gps.observe_gga(time, 1); gps.service(time); }
        assert(gps.status() == Status::Configuring && gps.config_retries() == 1);
        assert(std::string(gps.state_text(time)) == "streaming (reconfiguring)");
        assert(uart.sent.back() == skytraq({0x64, 0x17, 5, 0}));
        assert(uart.baud_changes.empty() && swaps == 0);
        // The retry now succeeds: Ready (configured).
        finish_skytraq(gps, uart, time);
        assert(gps.status() == Status::Ready && uart.baud_changes.empty() && swaps == 0);
        assert(std::string(gps.state_text(time + 20)) == "ready (configured)");
        assert_no_forbidden(uart);
    }
    if (GpsReceiver::kFixedBaud) {
        // G2/G7: missing ACKs -> Streaming (not Ready); the background retry
        // with ACKs makes it Ready.
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        feed(gps, skytraq({0x64, 0x8B, 0}), time);
        time = finish_skytraq(gps, uart, time + 1, false);
        assert(gps.status() == Status::Streaming);
        const std::uint32_t t0 = time;
        for (; time < t0 + 6000; time += 40) { gps.observe_gga(time, 1); gps.service(time); }
        assert(gps.status() == Status::Configuring && gps.config_retries() == 1);
        finish_skytraq(gps, uart, time);
        assert(gps.status() == Status::Ready && uart.baud_changes.empty());
        assert_no_forbidden(uart);
    }
    if (GpsReceiver::kFixedBaud) {
        // G2: Failed with no data: the full probe waits for 10 s and 5 s of
        // silence; with data it is never due.
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = time_out(gps, start(gps, false));
        for (int i = 0; i < 5; ++i) { gps.service(++time); time = time_out(gps, time); }
        assert(gps.status() == Status::Failed);
        assert(gps.probe_due(time + GpsReceiver::kProbeRetryMs));
        gps.observe_gga(time + GpsReceiver::kProbeRetryMs - 100, 1);   // a GGA shows up
        assert(!gps.probe_due(time + GpsReceiver::kProbeRetryMs));
        assert(!gps.retry_identification(time + GpsReceiver::kProbeRetryMs));
        assert_no_forbidden(uart);
    }
    if (GpsReceiver::kFixedBaud) {
        // G1: Ready, then silence: listen at 115200, then the fallback rate;
        // the receiver turns up at the fallback rate with default settings:
        // it is configured again there. Then total silence ends in Failed
        // with the full probe due at once; the remembered good rate is tried
        // first, then 115200 (G5).
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        feed(gps, skytraq({0x64, 0x8B, 0}), time);
        gps.observe_gga(time, 1);
        time = finish_skytraq(gps, uart, time + 1);
        assert(gps.status() == Status::Ready);
        gps.set_baud_hint(88398);                        // measured at the startup probe
        gps.observe_gga(time, 1);
        const std::uint32_t t0 = time;
        gps.service(t0 + GpsReceiver::kSilenceRecoveryMs - 1);
        assert(gps.status() == Status::Ready);
        gps.service(t0 + GpsReceiver::kSilenceRecoveryMs);
        assert(gps.status() == Status::Recovering && uart.baud_changes.empty());   // 115200 already
        time = t0 + GpsReceiver::kSilenceRecoveryMs;
        gps.service(time + GpsReceiver::kRecoveryListenMs);
        time += GpsReceiver::kRecoveryListenMs;
        assert(gps.status() == Status::Recovering && gps.baud() == 88398);
        const auto sent_before = uart.sent.size();
        gps.observe_gga(time + 100, 1);                  // back, at the fallback rate
        assert(gps.status() == Status::Configuring && gps.recoveries() == 1);
        time += 101;
        gps.service(time);
        assert(uart.sent.size() == sent_before + 1 && uart.sent.back() == skytraq({0x64, 0x17, 5, 0}));
        assert(gps.baud() == 88398 && uart.baud_changes.size() == 1);
        time = finish_skytraq(gps, uart, time);
        assert(gps.status() == Status::Ready && gps.good_baud() == 88398);
        // Silence again, nothing anywhere: ladder 115200 -> 88398 -> Failed.
        gps.observe_gga(time, 1);
        const std::uint32_t t1 = time + GpsReceiver::kSilenceRecoveryMs;
        gps.service(t1);
        assert(gps.status() == Status::Recovering && gps.baud() == 115200);
        gps.service(t1 + GpsReceiver::kRecoveryListenMs);
        assert(gps.baud() == 88398);
        gps.service(t1 + 2 * GpsReceiver::kRecoveryListenMs);
        assert(gps.status() == Status::Failed);
        time = t1 + 2 * GpsReceiver::kRecoveryListenMs;
        assert(gps.probe_due(time));
        assert(gps.retry_identification(time));
        assert(gps.baud() == 88398);                     // remembered good rate first
        gps.service(++time);
        time = time_out(gps, time);
        gps.service(++time);
        assert(gps.baud() == 115200);                    // then the default
        assert_no_forbidden(uart);
    }
    if (GpsReceiver::kFixedBaud) {
        // G1: a receiver back with its default configuration (1 Hz GGA) is
        // configured again at the same baud and pins; nothing else moves.
        static int swaps = 0;
        swaps = 0;
        FakeUart uart;
        GpsReceiver::Transport t = uart.transport();
        t.swap_pins = [](void *) { ++swaps; return true; };
        GpsReceiver gps(t);
        auto time = start(gps);
        feed(gps, skytraq({0x64, 0x8B, 0}), time);
        gps.observe_gga(time, 1);
        time = finish_skytraq(gps, uart, time + 1);
        assert(gps.status() == Status::Ready);
        const std::uint32_t t0 = time;
        for (; time < t0 + 1000 * 7 && gps.status() == Status::Ready; time += 1000) {
            gps.observe_gga(time, 1);
            gps.service(time);
        }
        assert(gps.status() == Status::Configuring && gps.config_retries() == 1);
        assert(gps.measured_rate_hz() == 1);
        assert(uart.baud_changes.empty() && swaps == 0 && gps.baud() == 115200);
        assert(uart.sent.back() == skytraq({0x64, 0x17, 5, 0}));
        assert_no_forbidden(uart);
        // A healthy 25 Hz stream never triggers it.
        FakeUart uart2;
        GpsReceiver ok(uart2.transport());
        auto tt = start(ok);
        feed(ok, skytraq({0x64, 0x8B, 0}), tt);
        ok.observe_gga(tt, 1);
        tt = finish_skytraq(ok, uart2, tt + 1);
        const auto sent2 = uart2.sent.size();
        for (std::uint32_t k = 0; k < 20000; k += 40) { ok.observe_gga(tt + k, 1); ok.service(tt + k); }
        assert(ok.status() == Status::Ready && uart2.sent.size() == sent2 && ok.measured_rate_hz() == 25);
    }
    {
        // G4: records other than 0x41 (87 bytes) are refused by begin_assist;
        // the transport never sees baud, factory-default or flash messages.
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        feed(gps, skytraq({0x64, 0x8B, 0}), time);
        time = finish_skytraq(gps, uart, time + 1);
        for (std::uint8_t id : {0x05, 0x04, 0x01, 0x0B, 0x64}) {
            Bytes body(87, 0);
            body[0] = id;
            Bytes blob{0, 87};
            blob.insert(blob.end(), body.begin(), body.end());
            assert(!gps.begin_assist(blob.data(), blob.size(), time));
            gps.service(++time);
        }
        assert(!gps.assist_progress().active);
        assert_no_forbidden(uart);
    }
    // Power-on (2026-11-17 bench): identified at 115200 while GGA streams, then the
    // receiver moves to another rate during configuration and goes silent. The
    // configuration fails as soon as the silence has lasted kConfigSilenceFailMs
    // (not after the full readback timeouts), and the first full probe is due
    // kFirstProbeRetryMs later; the second failure uses the normal 10 s.
    if (GpsReceiver::kFixedBaud) {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        feed(gps, skytraq({0x64, 0x8B, 0}), time);
        gps.observe_gga(time, 1);
        const std::uint32_t t0 = time;
        std::uint32_t t = time + 1;
        gps.service(t);                                     // first configuration command
        assert(gps.status() == Status::Configuring && !uart.sent.empty());
        for (; gps.status() == Status::Configuring && t < t0 + 20000; t += 50) gps.service(t);
        assert(gps.status() == Status::Failed);
        t -= 50;                                            // the pass that failed
        assert(t - t0 >= GpsReceiver::kConfigSilenceFailMs && t - t0 <= GpsReceiver::kConfigSilenceFailMs + 100);
        assert(!gps.probe_due(t + GpsReceiver::kFirstProbeRetryMs - 1));
        assert(gps.probe_due(t + GpsReceiver::kFirstProbeRetryMs));
        assert(gps.retry_identification(t + GpsReceiver::kFirstProbeRetryMs));
        assert(gps.identification_retries() == 1);
        // Second failure: the long retry delay and the 5 s silence rule are back.
        std::uint32_t u = t + GpsReceiver::kFirstProbeRetryMs;
        std::uint32_t failed_at = u;
        for (int i = 0; gps.status() != Status::Failed && i < 4000; ++i) { u += 50; gps.service(u); failed_at = u; }
        assert(gps.status() == Status::Failed);
        u = failed_at;
        assert(!gps.probe_due(u + GpsReceiver::kProbeRetryMs - 1));
        assert(gps.probe_due(u + GpsReceiver::kProbeRetryMs));
        assert_no_forbidden(uart);
    }
    // A receiver that keeps sending GGA (1 Hz here) while it ignores commands is
    // not failed by the silence rule.
    if (GpsReceiver::kFixedBaud) {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        feed(gps, skytraq({0x64, 0x8B, 0}), time);
        gps.observe_gga(time, 1);
        std::uint32_t t = time;
        for (int i = 0; i < 6000 / 50; ++i) {
            t += 50;
            if (i % 20 == 0) gps.observe_gga(t, 1);
            gps.service(t);
            assert(gps.status() == Status::Configuring);
        }
    }
    std::puts("gps_receiver tests passed");
}
