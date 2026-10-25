#include "px1120s_gps.h"

#include <cassert>
#include <cstdio>
#include <initializer_list>
#include <vector>

struct FakeUart {
    std::vector<std::vector<std::uint8_t>> sent;
    std::vector<std::uint32_t> baud_changes;
    bool send_ok = true;
    static bool send(void *context, const std::uint8_t *data, std::size_t length) {
        auto &self = *static_cast<FakeUart *>(context);
        self.sent.emplace_back(data, data + length);
        return self.send_ok;
    }
    static bool baud(void *context, std::uint32_t rate) {
        static_cast<FakeUart *>(context)->baud_changes.push_back(rate);
        return true;
    }
    Px1120sGps::Transport transport() { return {this, send, baud}; }
};

std::vector<std::uint8_t> frame(std::initializer_list<std::uint8_t> payload) {
    std::vector<std::uint8_t> bytes{0xA0, 0xA1, 0, static_cast<std::uint8_t>(payload.size())};
    std::uint8_t checksum = 0;
    for (const auto byte : payload) { bytes.push_back(byte); checksum ^= byte; }
    bytes.insert(bytes.end(), {checksum, 0x0D, 0x0A});
    return bytes;
}

void reply(Px1120sGps &gps, std::initializer_list<std::uint8_t> payload, std::uint32_t time) {
    const auto bytes = frame(payload);
    gps.consume(bytes.data(), bytes.size(), time);
}

void start_probe(Px1120sGps &gps) {
    gps.service(0);
    gps.service(Px1120sGps::kStartupDelayMs);
    gps.service(1501);
}

void finish_configuration(Px1120sGps &gps, FakeUart &uart, std::uint32_t time) {
    gps.service(time);
    const std::vector<std::uint8_t> airborne{
        0xA0, 0xA1, 0x00, 0x04, 0x64, 0x17, 0x05, 0x00, 0x76, 0x0D, 0x0A};
    assert(uart.sent.back() == airborne);
    reply(gps, {0x83, 0x64, 0x17}, time + 1);
    assert(gps.status() == Px1120sGps::Status::Configuring); // ACK alone is insufficient.
    gps.service(time + 2);
    assert(uart.sent.back() == frame({0x64, 0x18}));
    reply(gps, {0x64, 0x8B, 0x05}, time + 3);
    assert(gps.status() == Px1120sGps::Status::Ready);
}

int main() {
    // Fixed operational baud: ignore damaged frames; accept fragmented binary replies.
    {
        FakeUart uart;
        Px1120sGps gps(uart.transport());
        gps.service(0);
        gps.service(1499);
        assert(uart.sent.empty());
        gps.service(1500);
        gps.service(1501);
        const std::vector<std::uint8_t> query{0xA0, 0xA1, 0, 2, 0x64, 0x18, 0x7C, 0x0D, 0x0A};
        assert(uart.sent.back() == query);
        const std::uint8_t nmea[] = "$GNGGA,0,0*00\r\n";
        gps.consume(nmea, sizeof(nmea), 1502);
        auto damaged = frame({0x64, 0x8B, 0});
        damaged[damaged.size() - 3] ^= 1;
        gps.consume(damaged.data(), damaged.size(), 1502);
        gps.service(1502);
        assert(uart.sent.size() == 1);
        const auto bytes = frame({0x64, 0x8B, 0});
        for (const auto byte : bytes) gps.consume(&byte, 1, 1503);
        finish_configuration(gps, uart, 1504);
        assert(uart.baud_changes.empty());
    }
    // Receiver is never told to change baud, even after NMEA-only responses.
    {
        FakeUart uart;
        Px1120sGps gps(uart.transport());
        start_probe(gps);
        gps.observe_nmea();
        for (auto time : {2501U, 3501U, 4501U}) gps.service(time);
        finish_configuration(gps, uart, 4502);
        assert(uart.baud_changes.empty());
        for (const auto &sent : uart.sent) assert(sent[4] != 0x05);
    }
    // No receiver: retries terminate without hunting for another baud.
    {
        FakeUart uart;
        Px1120sGps gps(uart.transport());
        start_probe(gps);
        for (auto time : {2501U, 3501U, 4501U, 4502U, 5502U, 6502U, 7502U})
            gps.service(time);
        assert(gps.status() == Px1120sGps::Status::Failed);
        assert(uart.sent.size() == 3);
        assert(uart.baud_changes.back() == Px1120sGps::kBaudRate);
        gps.service(20000);
        assert(uart.sent.size() == 3);
    }
    // An explicit NACK or wrong readback must never be reported as successful.
    for (const bool nack : {false, true}) {
        FakeUart uart;
        Px1120sGps gps(uart.transport());
        start_probe(gps);
        reply(gps, {0x64, 0x8B, 0}, 1502);
        gps.service(1503);
        if (nack) reply(gps, {0x84, 0x64, 0x17}, 1504);
        else {
            reply(gps, {0x83, 0x64}, 1504); // Older firmware's two-byte ACK.
            gps.service(1505);
            reply(gps, {0x64, 0x8B, 0x02}, 1506);
        }
        assert(gps.status() == Px1120sGps::Status::Failed);
        assert(uart.baud_changes.back() == Px1120sGps::kBaudRate);
    }
    // Missing ACK: retry, then verify actual mode rather than declaring success.
    {
        FakeUart uart;
        Px1120sGps gps(uart.transport());
        start_probe(gps);
        reply(gps, {0x64, 0x8B, 0}, 1502);
        for (auto time : {1503U, 2503U, 3503U, 4503U, 4504U}) gps.service(time);
        assert(gps.status() == Px1120sGps::Status::Configuring);
        reply(gps, {0x64, 0x8B, 5}, 4505);
        assert(gps.status() == Px1120sGps::Status::Ready);
    }
    // Bad lengths, idle-truncated frames and failed TX are bounded and recoverable.
    {
        FakeUart uart;
        Px1120sGps gps(uart.transport());
        start_probe(gps);
        const std::uint8_t invalid[] = {0xA0, 0xA1, 0xFF, 0xFF, 0xA0, 0xA1};
        gps.consume(invalid, sizeof(invalid), 1502);
        reply(gps, {0x64, 0x8B, 0}, 1803);
        finish_configuration(gps, uart, 1804);
    }
    {
        FakeUart uart;
        uart.send_ok = false;
        Px1120sGps gps(uart.transport());
        start_probe(gps);
        assert(gps.status() == Px1120sGps::Status::Failed);
    }
    // Final readback is decisive even when another response trails in the same UART read.
    {
        FakeUart uart;
        Px1120sGps gps(uart.transport());
        start_probe(gps);
        reply(gps, {0x64, 0x8B, 0}, 1502);
        gps.service(1503);
        reply(gps, {0x83, 0x64, 0x17}, 1504);
        gps.service(1505);
        auto combined = frame({0x64, 0x8B, 5});
        const auto stale_nack = frame({0x84, 0x64, 0x18});
        combined.insert(combined.end(), stale_nack.begin(), stale_nack.end());
        gps.consume(combined.data(), combined.size(), 1506);
        assert(gps.status() == Px1120sGps::Status::Ready);
    }
    std::puts("PX1120S startup tests passed");
    // NMEA-only response must never trigger factory-baud probing.
    {
        FakeUart uart;
        Px1120sGps gps(uart.transport());
        start_probe(gps);
        gps.observe_nmea(); // Includes checksum-valid no-fix GGA.
        for (std::uint32_t time = 2501; time <= 20001; time += 1000)
            gps.service(time);
        assert(gps.status() == Px1120sGps::Status::Failed); // No mode readback.
        for (auto baud : uart.baud_changes) assert(baud == Px1120sGps::kBaudRate);
        bool airborne_sent = false;
        for (const auto &sent : uart.sent)
            if (sent == frame({0x64, 0x17, 5, 0})) airborne_sent = true;
        assert(airborne_sent);
    }
    // A missing initial query response does not prevent later verified setup.
    {
        FakeUart uart;
        Px1120sGps gps(uart.transport());
        start_probe(gps);
        gps.observe_nmea();
        for (auto time : {2501U, 3501U, 4501U}) gps.service(time);
        finish_configuration(gps, uart, 4502);
        assert(uart.baud_changes.empty());
    }
    // I2C shares mode configuration/readback but must never issue baud commands.
    {
        FakeUart link;
        Px1120sGps gps({&link, FakeUart::send, nullptr});
        start_probe(gps);
        reply(gps, {0x64, 0x8B, 0}, 1502);
        finish_configuration(gps, link, 1503);
        assert(link.baud_changes.empty());
    }
    {
        FakeUart link;
        Px1120sGps gps({&link, FakeUart::send, nullptr});
        start_probe(gps);
        for (auto time : {2501U, 3501U, 4501U}) gps.service(time);
        assert(gps.status() == Px1120sGps::Status::Failed);
        assert(link.sent.size() == 3);
        assert(link.baud_changes.empty());
    }
    std::puts("PX1120S I2C configuration tests passed");
}
