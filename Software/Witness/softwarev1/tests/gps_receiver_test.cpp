#include "gps_receiver.h"

#include <cassert>
#include <cstdio>
#include <initializer_list>
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

// Runs service() past the boot delay so the first identification polls go out.
std::uint32_t start(GpsReceiver &gps) {
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

void finish_skytraq(GpsReceiver &gps, FakeUart &uart, std::uint32_t time, bool acks = true) {
    // Airborne mode (frame verified on hardware), NMEA GGA+RMC, then 25 Hz.
    gps.service(time);
    assert(uart.sent.back() == (Bytes{0xA0, 0xA1, 0x00, 0x04, 0x64, 0x17, 0x05, 0x00,
                                      0x76, 0x0D, 0x0A}));
    if (acks) feed(gps, skytraq({0x83, 0x64, 0x17}), time + 1);
    else time = time_out(gps, time);
    gps.service(time + 2);
    assert(uart.sent.back() == skytraq({0x08, 1, 0, 0, 0, 1, 0, 0, 0}));
    if (acks) feed(gps, skytraq({0x83, 0x08}), time + 3);
    else time = time_out(gps, time + 2);
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
    assert(gps.status() == Status::Ready);
    assert(gps.update_rate_hz() == 25);
}

void finish_ublox(GpsReceiver &gps, FakeUart &uart, std::uint32_t time) {
    const Bytes expected_msgs[] = {{0xF0, 0x01, 0}, {0xF0, 0x02, 0}, {0xF0, 0x03, 0},
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
        for (std::uint8_t id : {0x64, 0x08, 0x0E}) {
            gps.service(time++);
            if (id == 0x64) feed(gps, skytraq({0x83, 0x64, 0x17}), time++);
            else feed(gps, skytraq({0x83, id}), time++);
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
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = start(gps);
        time = time_out(gps, time);
        assert(uart.baud_changes == std::vector<std::uint32_t>{9600});
        assert(gps.baud() == 9600 && gps.model() == Model::Unknown);
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
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = time_out(gps, start(gps));
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
    // Nothing at either rate: fail and leave the host at the operational rate.
    {
        FakeUart uart;
        GpsReceiver gps(uart.transport());
        auto time = time_out(gps, start(gps));
        gps.service(time + 1);  // first poll at 9600
        time = time_out(gps, time + 1);
        assert(gps.status() == Status::Failed);
        assert(uart.baud_changes == (std::vector<std::uint32_t>{9600, 115200}));
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
    std::puts("gps_receiver tests passed");
}
