#pragma once

#include <cstdint>

// Pure allow/refuse rules for commands that can disturb a flight (no ESP-IDF
// dependency; host test tests/command_policy_test.cpp). Flight-state values
// are FlightStateMachine::State (0 Pad Idle, 1 Boost, 2 Coast, 3 Descent,
// 4 Landed). The caller sends IRIS_ACK_REFUSED and logs on a false result.
namespace command_policy {

enum class Source : std::uint8_t { Usb, Radio };

constexpr std::uint8_t kPadIdle = 0x00;
constexpr std::uint8_t kBoost = 0x01;
constexpr std::uint8_t kCoast = 0x02;
constexpr std::uint8_t kDescent = 0x03;
constexpr std::uint8_t kLanded = 0x04;

constexpr std::uint16_t kMassStorageStart = 0x6868U;
constexpr std::uint16_t kEraseLogSessions = 0x6869U;
constexpr std::uint16_t kClearLogSessions = 0x686AU;

constexpr bool is_log_maintenance_command(const std::uint16_t command)
{
    return command == kMassStorageStart || command == kEraseLogSessions ||
           command == kClearLogSessions;
}

// Mass storage freezes logging and silences LoRa; erase/clear wipes the
// flight logs. Never over radio; over USB only on the pad or after landing.
constexpr bool allow_log_maintenance(const Source source, const std::uint8_t flight_state)
{
    return source == Source::Usb && (flight_state == kPadIdle || flight_state == kLanded);
}

// Manual flight-state override.
// Radio: only Landed -> Pad Idle (re-arm after recovery). No launch trigger
// and no other jump, so a stray or malicious frame cannot move a flying vehicle.
// USB: any value 0..4, except that from Boost/Coast/Descent only Landed (or
// the same state) is accepted.
constexpr bool allow_set_flight_state(const Source source, const std::uint8_t current,
                                      const std::uint8_t requested)
{
    if (requested > kLanded || current > kLanded) return false;
    if (source == Source::Radio) return current == kLanded && requested == kPadIdle;
    if (current >= kBoost && current <= kDescent) {
        return requested == kLanded || requested == current;
    }
    return true;
}

}  // namespace command_policy
