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

// LoRa drive range of the SX1262 (hardware_pins.h IRIS_RADIO_TX_POWER_MIN/MAX_DBM;
// usb_serial_echo.cpp static_asserts they agree).
constexpr int kRadioPowerMinDbm = -17;
constexpr int kRadioPowerMaxDbm = 3;
constexpr bool radio_power_in_range(const int dbm)
{
    return dbm >= kRadioPowerMinDbm && dbm <= kRadioPowerMaxDbm;
}

// The dangerous half of 0x5250. Boost: nothing (see allow_radio_power). In
// Coast and Descent the downlink is all the ground has, so the front end may
// not be switched OFF and the drive may not go BELOW the level in use (it is
// the level the flight was started with); raising it stays allowed. Pad Idle
// and Landed: everything (bench / recovery use).
constexpr bool allow_radio_power_change(const std::uint8_t flight_state, const int requested_dbm,
                                        const int current_dbm, const bool front_end_off)
{
    if (!allow_radio_power(flight_state)) return false;
    if (flight_state == kCoast || flight_state == kDescent) {
        if (front_end_off) return false;
        if (requested_dbm < current_dbm) return false;
    }
    return true;
}

// Internal flash (NVS) writes stall both cores for a few ms: the chosen power
// level is only stored while on the ground.
constexpr bool allow_persist_radio_power(const std::uint8_t flight_state)
{
    return flight_state == kPadIdle || flight_state == kLanded;
}

// GPS assist uploads (BEGIN/DATA/COMMIT; they end in records written to the
// receiver) are for the ground only: Pad Idle and Landed, any source. A blob
// can hold any SkyTraq message (restart, baud, factory default), so it must
// never reach the GPS in flight. STATUS (read only) always works.
constexpr bool allow_gps_assist(const std::uint16_t command, const std::uint8_t flight_state)
{
    if (command == kGpsAssistStatus) return true;
    return flight_state == kPadIdle || flight_state == kLanded;
}

// Bench-only commands (comms.h: 0x4252 restart, 0x4248 hang, 0x4249 Iris
// reset). They exist only for tests: accepted as a canonical USB frame while
// HITL is active, never from radio and never forwarded to Iris or re-sent over
// LoRa (the bridge refuses them for the radio and USB-forward paths; USB
// handles them locally).
constexpr std::uint16_t kBenchRestart = 0x4252U;
constexpr std::uint16_t kBenchHang = 0x4248U;
constexpr std::uint16_t kBenchIrisReset = 0x4249U;
constexpr bool is_bench_command(const std::uint16_t command)
{
    return command == kBenchRestart || command == kBenchHang || command == kBenchIrisReset;
}
// The only source and modes in which a bench command runs: USB, and either HITL
// active or this boot RESUMED an in-flight state after a warm reset (HITL is off
// after every reset, so without the second case a test could not reset twice in a
// row). USB is never connected in a real flight; radio is never allowed.
constexpr bool allow_bench_command(const Source source, const bool hitl_active,
                                   const bool resumed_after_reset = false)
{
    return source == Source::Usb && (hitl_active || resumed_after_reset);
}

// GPS cold start (0x4752): loses the fix for 30 s or more. Pad Idle only; the
// state is checked again when the request is made.
constexpr bool allow_gps_cold_start(const std::uint8_t flight_state)
{
    return flight_state == kPadIdle;
}

// Iris loads (CH1-3, 0x0030-0x0035) and the 5 V regulator (0x0050/51).
// "ON" is always allowed; "OFF" is refused in Boost, Coast and Descent from
// any source (a load switched off in flight cannot be recovered over a
// failed link, and what is behind OUT1-3 may be a camera or a pyro driver).
// The CommandBridge applies this before anything is sent to Iris or re-sent
// over LoRa, so a refused command never leaves Witness.
constexpr std::uint16_t kCh1LoadOff = 0x0030U;
constexpr std::uint16_t kCh1LoadOn = 0x0031U;
constexpr std::uint16_t kCh2LoadOff = 0x0032U;
constexpr std::uint16_t kCh2LoadOn = 0x0033U;
constexpr std::uint16_t kCh3LoadOff = 0x0034U;
constexpr std::uint16_t kCh3LoadOn = 0x0035U;
constexpr std::uint16_t kRegulator5vOff = 0x0050U;
constexpr std::uint16_t kRegulator5vOn = 0x0051U;

constexpr bool is_iris_load_off(const std::uint16_t command)
{
    return command == kCh1LoadOff || command == kCh2LoadOff || command == kCh3LoadOff ||
           command == kRegulator5vOff;
}
constexpr bool is_iris_load_command(const std::uint16_t command)
{
    return is_iris_load_off(command) || command == kCh1LoadOn || command == kCh2LoadOn ||
           command == kCh3LoadOn || command == kRegulator5vOn;
}
constexpr bool allow_iris_load_command(const std::uint16_t command, const std::uint8_t flight_state)
{
    if (!is_iris_load_off(command)) return true;
    return flight_state == kPadIdle || flight_state == kLanded;
}

// Radio erase / clear (0x6869/0x686A) need the same command twice within
// kEraseConfirmWindowMs (accident and replay protection with no protocol
// change). The first send does nothing and is answered IRIS_ACK_BUSY with
// detail IRIS_ACK_DETAIL_CONFIRM_NEEDED (comms.h); the second executes. USB stays single shot.
constexpr std::uint32_t kEraseConfirmWindowMs = 10000U;

class EraseConfirm {
public:
    // True when this command may execute now. A different command, or one
    // after the window, only arms the confirmation (returns false).
    bool accept(const std::uint16_t command, const std::uint32_t now_ms)
    {
        if (armed_ && command == command_ && static_cast<std::uint32_t>(now_ms - armed_ms_) <= kEraseConfirmWindowMs) {
            armed_ = false;
            return true;
        }
        armed_ = true;
        command_ = command;
        armed_ms_ = now_ms;
        return false;
    }
    void reset() { armed_ = false; }

private:
    bool armed_ = false;
    std::uint16_t command_ = 0;
    std::uint32_t armed_ms_ = 0;
};

// Deferred restarts (maintenance image switch, Landed -> Pad reset). The
// restart fires 0.4-1.1 s after the ack; the flight state is checked again
// then, and a restart that is no longer allowed is abandoned (and, for the
// maintenance switch, the boot partition is put back to the flight image).
enum class PendingRestart : std::uint8_t { None, Maintenance, ResetToPad };
constexpr bool restart_still_allowed(const PendingRestart kind, const std::uint8_t flight_state)
{
    switch (kind) {
        case PendingRestart::Maintenance: return flight_state == kPadIdle || flight_state == kLanded;
        case PendingRestart::ResetToPad: return flight_state == kLanded;
        default: return false;
    }
}

// Landed -> Pad Idle (0x7300 while Landed) is not a state transition: the
// flight states are one-way (FlightStateMachine::set_state refuses to go
// backwards), so this command makes Witness acknowledge and then restart
// (a Landed record is never resumed after a reset: it boots Pad Idle with a
// fresh filter and a fresh log session).
constexpr bool is_reset_to_pad(const std::uint8_t current, const std::uint8_t requested)
{
    return current == kLanded && requested == kPadIdle;
}

// Manual flight-state override. The states only move forward (Pad Idle ->
// Boost -> Coast -> Descent -> Landed); the same state is an accepted no-op.
// The one way back is is_reset_to_pad() (a restart).
// Radio: only Landed -> Pad Idle (reset) and Pad Idle -> Pad Idle (no-op). No
// launch trigger and no other jump, so a stray or malicious frame cannot move
// a flying vehicle.
// USB: from Pad Idle any state 0..4 (1 is a manual launch); from
// Boost/Coast/Descent only Landed (or the same state); from Landed only the
// reset to Pad Idle (or Landed itself). Everything backwards is refused.
constexpr bool allow_set_flight_state(const Source source, const std::uint8_t current,
                                      const std::uint8_t requested)
{
    if (requested > kLanded || current > kLanded) return false;
    if (is_reset_to_pad(current, requested)) return true;
    if (requested == current) return current == kPadIdle || source == Source::Usb;
    if (source == Source::Radio) return false;
    if (requested < current) return false;
    if (current >= kBoost && current <= kDescent) return requested == kLanded;
    return true;
}

}  // namespace command_policy
