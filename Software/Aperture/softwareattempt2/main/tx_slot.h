#pragma once

// GPS-synchronised LoRa time slots and the uplink hold-off (2026-11-14).
// Pure logic, no ESP-IDF dependency; host test tests/tx_slot_test.cpp. The
// same file is used by Witness (Witness/softwarev1/main/tx_slot.h) and by
// Aperture (Aperture/softwareattempt2/main/tx_slot.h): keep the copies
// byte-identical, like comms.h.
//
// One radio channel, half duplex: Witness cannot hear Aperture while it
// transmits. Each GPS (UTC) second is split:
//
//   0 ms ........................ 700 | 750 ...... 800 ........... 950 | 1000
//   Witness telemetry (downlink)      | guard        Aperture commands  | guard
//
// - Witness starts a telemetry packet only if it ENDS by kDownlinkEndMs (700),
//   by its UTC clock taken from GGA (no PPS line: +-50 ms, hence the guards).
// - Aperture starts a command only if it starts at or after
//   kUplinkStartMs (800) and ends by kUplinkEndMs (950), by its PPS edge.
// - Either side without a recent GPS time sends whenever it can (the
//   behaviour before slots), so nothing depends on GPS.
//
// Hold-off (works without GPS): when Witness receives a command frame (0x05)
// it holds its telemetry for kUplinkHoldOffMs, so a stream of commands (GPS
// assist upload) gets a quiet channel. Command acks and replies (the
// immediate queue) are never held. Aperture, having sent a command, may send
// the next one at once ("burst") for kUplinkBurstMs, which is shorter than the
// hold-off its previous frame started on Witness.
//
// 2026-11-15 audit F4: the hold-off is armed only by a frame that passed the
// command validation (Sensors uses the CommandBridge result), is never armed
// in Boost, and is capped: UplinkHoldOff below lets a hold run for at most
// kHoldOffMaxMs, then forces one telemetry cycle (every packet type once)
// before another hold can start, so a steady command stream can never silence
// the downlink for more than about kHoldOffMaxMs at a time.

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace tx_slot {

inline constexpr std::uint32_t kSecondMs = 1000;
inline constexpr std::uint32_t kDownlinkEndMs = 700;     // Witness TX must end by this phase
inline constexpr std::uint32_t kUplinkStartMs = 800;     // Aperture TX may start at this phase
inline constexpr std::uint32_t kUplinkEndMs = 950;       // ... and must end by this phase
inline constexpr std::uint32_t kUplinkHoldOffMs = 1000;  // Witness quiet after hearing a command
inline constexpr std::uint32_t kHoldOffMaxMs = 3000;      // longest continuous hold-off
inline constexpr std::uint32_t kForcedCycleMaxMs = 2000;  // forced telemetry cycle gives up after this
inline constexpr std::uint32_t kUplinkBurstMs = 800;     // Aperture: next command at once if within this
// A GGA arrives this long after its epoch (25 Hz output at 88-115 kbaud): the
// UTC read from it is advanced by this much on Witness.
inline constexpr std::uint32_t kGgaLatencyMs = 20;
// GPS time older than this (no fresh fix / PPS) is not used for slots.
inline constexpr std::uint32_t kTimeMaxAgeMs = 3000;

// LoRa time on air (Semtech AN1200.13), explicit header, CRC on, low data rate
// optimisation off (SF7/500 kHz). cr 1..4 = 4/5..4/8.
inline std::uint32_t lora_airtime_ms(const std::size_t payload_bytes, const unsigned sf = 7,
                                     const std::uint32_t bandwidth_hz = 500000U,
                                     const unsigned cr = 1, const unsigned preamble = 8)
{
    const double t_sym_ms = std::ldexp(1.0, static_cast<int>(sf)) * 1000.0 / bandwidth_hz;
    const double numerator = 8.0 * payload_bytes - 4.0 * sf + 28.0 + 16.0;
    double payload_symbols = 8.0;
    if (numerator > 0.0) payload_symbols += std::ceil(numerator / (4.0 * sf)) * (cr + 4.0);
    const double total = (preamble + 4.25 + payload_symbols) * t_sym_ms;
    return static_cast<std::uint32_t>(std::ceil(total));
}

// Phase (0..999 ms) within the UTC second.
inline double phase_ms(const double utc_ms_of_day)
{
    double phase = std::fmod(utc_ms_of_day, static_cast<double>(kSecondMs));
    if (phase < 0.0) phase += kSecondMs;
    return phase;
}

enum class Decision : std::uint8_t {
    Send,      // transmit now
    HoldOff,   // a command was heard recently: keep quiet
    Wait,      // outside the downlink window: wait for the next second
};

// Witness telemetry. utc_known: a fix with UTC no older than kTimeMaxAgeMs.
inline Decision witness_telemetry(const std::uint64_t now_ms, const bool uplink_heard,
                                  const std::uint64_t last_uplink_ms, const bool utc_known,
                                  const double utc_ms_now, const std::uint32_t airtime_ms)
{
    if (uplink_heard && now_ms - last_uplink_ms < kUplinkHoldOffMs) return Decision::HoldOff;
    if (!utc_known) return Decision::Send;
    const double phase = phase_ms(utc_ms_now + kGgaLatencyMs);
    return phase + airtime_ms <= kDownlinkEndMs ? Decision::Send : Decision::Wait;
}

// Hold-off governor: wraps the "heard a command" state with the cap above.
//   note_command(now, allow): a validated command frame was received.
//     allow=false (Boost) ignores it: commands are mostly refused there and
//     the downlink must keep flowing.
//   active(now): true while telemetry must stay quiet; pass it as the
//     uplink_heard argument of witness_telemetry().
//   note_telemetry_sent(index): one scheduled packet type went out; index
//     0..type_count-1. When the cap forces a cycle, the cycle ends once every
//     type has been sent (or kForcedCycleMaxMs passed, e.g. a type with no
//     data), and only then can a new hold start.
// Pure logic; sensor task only (not thread safe).
class UplinkHoldOff {
public:
    explicit UplinkHoldOff(const unsigned type_count = 3)
        : all_types_mask_(type_count >= 8U ? 0xFFU : static_cast<std::uint8_t>((1U << type_count) - 1U)) {}

    void note_command(const std::uint64_t now_ms, const bool allow)
    {
        if (!allow || forcing_) return;     // forced cycle: heard but not held
        if (!armed_ || now_ms - last_heard_ms_ >= kUplinkHoldOffMs) hold_start_ms_ = now_ms;
        armed_ = true;
        last_heard_ms_ = now_ms;
    }

    bool active(const std::uint64_t now_ms)
    {
        if (forcing_) {
            if (pending_ == 0U || now_ms - force_start_ms_ >= kForcedCycleMaxMs) {
                forcing_ = false;
                armed_ = false;
            } else {
                return false;
            }
        }
        if (!armed_) return false;
        if (now_ms - last_heard_ms_ >= kUplinkHoldOffMs) {
            armed_ = false;
            return false;
        }
        if (now_ms - hold_start_ms_ >= kHoldOffMaxMs) {
            forcing_ = true;
            force_start_ms_ = now_ms;
            pending_ = all_types_mask_;
            ++forced_cycles_;
            return false;
        }
        return true;
    }

    void note_telemetry_sent(const unsigned type_index)
    {
        if (type_index < 8U) pending_ = static_cast<std::uint8_t>(pending_ & ~(1U << type_index));
    }

    bool forcing() const { return forcing_; }
    std::uint32_t forced_cycles() const { return forced_cycles_; }

private:
    std::uint8_t all_types_mask_;
    bool armed_ = false;
    bool forcing_ = false;
    std::uint64_t last_heard_ms_ = 0;
    std::uint64_t hold_start_ms_ = 0;
    std::uint64_t force_start_ms_ = 0;
    std::uint8_t pending_ = 0;
    std::uint32_t forced_cycles_ = 0;
};

// Aperture command. pps_known: a PPS edge no older than kTimeMaxAgeMs with a
// GPS fix; since_pps_ms: time since that edge (UTC second boundary).
inline bool aperture_may_send(const std::uint64_t now_ms, const bool sent_before,
                              const std::uint64_t last_sent_ms, const bool pps_known,
                              const std::uint64_t since_pps_ms, const std::uint32_t airtime_ms)
{
    if (!pps_known) return true;
    if (sent_before && now_ms - last_sent_ms < kUplinkBurstMs) return true;
    const double phase = phase_ms(static_cast<double>(since_pps_ms));
    return phase >= kUplinkStartMs && phase + airtime_ms <= kUplinkEndMs;
}

}  // namespace tx_slot
