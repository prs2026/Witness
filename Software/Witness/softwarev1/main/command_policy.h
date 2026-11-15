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

// Only on the pad or after landing. Mass storage (USB export: restart into the
// maintenance image's USB disk) is USB only. Erase/clear wipe the flight logs
// and are also accepted over radio (since 2026-11-14), because Witness never
// erases a flight on its own: with both log slots holding flights it logs
// nothing until the operator frees the store, and on the backplane the USB port
// is unreachable.
constexpr bool allow_log_maintenance(const Source source, const std::uint8_t flight_state,
                                     const std::uint16_t command = kMassStorageStart)
{
    if (flight_state != kPadIdle && flight_state != kLanded) return false;
    if (command == kMassStorageStart) return source == Source::Usb;
    return command == kEraseLogSessions || command == kClearLogSessions;
}

constexpr std::uint16_t kRadioPower = 0x5250U;
constexpr std::uint16_t kGpsAssistBegin = 0x4741U;
constexpr std::uint16_t kGpsAssistData = 0x4744U;
constexpr std::uint16_t kGpsAssistCommit = 0x4745U;
constexpr std::uint16_t kGpsAssistStatus = 0x4751U;

// Radio drive / front-end (0x5250): refused in Boost from any source (turning
// the front end off or down there would lose the link at the worst moment and
// it cannot be undone over that link).
constexpr bool allow_radio_power(const std::uint8_t flight_state)
{
    return flight_state != kBoost;
}

// GPS assist uploads (BEGIN/DATA/COMMIT; they end in records written to the
// receiver) are refused in Boost from any source; STATUS (read only) always
// works.
constexpr bool allow_gps_assist(const std::uint16_t command, const std::uint8_t flight_state)
{
    if (command == kGpsAssistStatus) return true;
    return flight_state != kBoost;
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
