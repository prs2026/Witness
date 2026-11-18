#pragma once

#include <atomic>
#include <cstdint>

#include "bit_limits.h"
#include "bit_types.h"
#include "comms.h"

// BIT command and indication rules. Pure; host test tests/bit_policy_test.cpp.
// Kept out of command_policy.h so the other firmware work that edits that file
// merges cleanly; command_policy.h is not changed.
namespace bit::policy {

enum class Source : std::uint8_t { Usb, Radio };

// ---- Which opcodes belong to BIT (never forwarded to Iris) ------------------
constexpr bool is_bit_command(const std::uint16_t opcode)
{
    return opcode == IRIS_COMMAND_BIT_RUN || opcode == IRIS_COMMAND_BIT_REPORT ||
           opcode == IRIS_COMMAND_BIT_CONFIG || opcode == IRIS_COMMAND_BIT_FAULT;
}

// ---- 0x5442 run ---------------------------------------------------------------
enum class Decision : std::uint8_t { Ok, Refused, Busy, BadArgument };

constexpr bool run_option_bits_valid(const std::uint8_t options)
{
    // Reserved bits and the camera option (bit 7) are refused until the camera
    // phase exists.
    return (options & (IRIS_BIT_OPTION_RESERVED_MASK | IRIS_BIT_OPTION_CAMERA)) == 0U;
}

// A commanded BIT runs only in Pad Idle and Landed, never after a warm reset
// in flight (RESUMED), and never twice at once. Source does not matter: USB and
// radio are equal.
constexpr Decision allow_bit_run(const Source, const std::uint8_t flight_state, const bool resumed,
                                 const bool run_in_progress, const std::uint8_t d0, const std::uint8_t options)
{
    if (d0 != IRIS_BIT_RUN_FULL && d0 != IRIS_BIT_RUN_PASSIVE) return Decision::BadArgument;
    if (!run_option_bits_valid(options)) return Decision::BadArgument;
    if (flight_state != kPadIdle && flight_state != kLanded) return Decision::Refused;
    if (resumed) return Decision::Refused;
    if (run_in_progress) return Decision::Busy;
    return Decision::Ok;
}

// The IMU self-test option only makes sense on the pad (it blinds the IMU).
constexpr bool option_allowed_in_state(const std::uint8_t options, const std::uint8_t flight_state)
{
    if ((options & IRIS_BIT_OPTION_IMU_SELF_TEST) != 0U && flight_state != kPadIdle) return false;
    return true;
}

// ---- 0x5452 report ------------------------------------------------------------
// No extra radio traffic during the burn; everything else is a cheap reply.
constexpr bool allow_bit_report(const std::uint8_t flight_state)
{
    return flight_state != kBoost && flight_state <= kLanded;
}

// ---- 0x4246 fault injection (bench) -------------------------------------------
// Same gate as 0x4252: HITL active (which only USB can start, only in Pad
// Idle), canonical USB frame, never radio.
constexpr bool allow_bit_fault(const Source source, const bool hitl_active)
{
    return source == Source::Usb && hitl_active;
}

constexpr bool fault_id_valid(const std::uint8_t id)
{
    return (id >= IRIS_BIT_FAULT_NAND_WRITE && id <= IRIS_BIT_FAULT_GRAVITY) ||
           id == IRIS_BIT_FAULT_HEAP_PRESSURE || id == IRIS_BIT_FAULT_LOGGER_STALL ||
           id == IRIS_BIT_FAULT_WDT_UNSUBSCRIBED || id == IRIS_BIT_FAULT_CLEAR_ALL;
}

constexpr std::uint32_t fault_duration_ms(const std::uint16_t requested)
{
    return (requested == 0U || requested > limits::kFaultMaxDurationMs)
               ? limits::kFaultMaxDurationMs
               : requested;
}

// ---- Summary resend on the radio ----------------------------------------------
// The summary goes out on change and every kReportPeriodNotGoMs while the
// verdict is not GO. Never in Boost (no extra traffic during the burn) except
// on a verdict change.
constexpr bool radio_summary_due(const std::uint8_t flight_state, const Verdict verdict, const bool changed,
                                 const std::uint32_t ms_since_last)
{
    if (flight_state == kBoost) return changed;
    if (changed) return true;
    return verdict != Verdict::Go && ms_since_last >= limits::kReportPeriodNotGoMs;
}

// ---- LED / buzzer ---------------------------------------------------------------
// NO-GO is 4 pulses per second (owner Q6); the buzzer follows the same pattern
// when it is enabled. An existing warning pattern with more pulses (3 for "not
// logging") wins, because NOT LOGGING is more urgent than any BIT result.
constexpr unsigned kNoGoPulses = 4;
constexpr unsigned pulses(const unsigned existing_pulses, const bool nogo)
{
    // The existing patterns are 1 (heartbeat) or 3 (not logging).
    if (existing_pulses > 1U) return existing_pulses;
    return nogo ? kNoGoPulses : existing_pulses;
}

}  // namespace bit::policy

namespace bit {

// Lock-free publication of the current verdict for the heartbeat LED and the
// buzzer (written by the BIT task, read by the heartbeat task).
struct Published {
    std::atomic<std::uint8_t> verdict{0};
    std::atomic<bool> bench{false};
    std::atomic<bool> nogo{false};
};

inline Published &published()
{
    static Published instance;
    return instance;
}

inline void publish(const Verdict verdict, const bool bench)
{
    Published &p = published();
    p.verdict.store(static_cast<std::uint8_t>(verdict), std::memory_order_relaxed);
    p.bench.store(bench, std::memory_order_relaxed);
    p.nogo.store(verdict == Verdict::NoGo, std::memory_order_relaxed);
}

inline bool led_nogo() { return published().nogo.load(std::memory_order_relaxed); }

}  // namespace bit
