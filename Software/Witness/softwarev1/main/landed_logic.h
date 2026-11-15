#pragma once

#include <cmath>
#include <cstdint>

// Pure scheduling logic for the reduced Pad Idle radio rate and the Landed
// low-power mode (no ESP-IDF dependency; host test tests/landed_logic_test.cpp).
// Every timing of these two behaviours lives in this header.
//
// All times are milliseconds on a monotonic clock (esp_timer / 1000), 64-bit,
// except the radio period helper, which only returns a duration.
namespace landed {

// Landed low-power mode on/off (2026-11-15: OFF by the owner's decision, after
// the bench found that a light sleep with USB attached killed the USB link).
// false: Landed runs the normal sensor loop (full telemetry at the normal
// rates, no MCU light sleep, no GPS sleep, CAN not paused, no 0x09 schedule,
// ping acked UNKNOWN as before). true restores Sensors::run_landed().
constexpr bool kLowPowerModeEnabled = false;

// ---- Pad Idle: reduced radio rate -----------------------------------------
// While the flight state is Pad Idle each locally generated radio packet type
// (sensors, state) is sent once per kPadIdleRadioPeriodMs instead of its normal
// period. A packet relayed from CAN (the Iris camera packet 0x03) keeps its
// normal rate unless kPadIdleThrottlesCanRelayed is set.
constexpr std::uint32_t kPadIdleRadioPeriodMs = 3000;
constexpr bool kPadIdleThrottlesCanRelayed = false;

constexpr std::uint8_t kStatePadIdle = 0;
constexpr std::uint8_t kStateLanded = 4;

// Period of one radio packet type in the given flight state. Evaluated on every
// pass from the current state, so leaving Pad Idle restores the normal period
// immediately (a packet last sent 2.5 s ago is already due again).
constexpr std::uint32_t radio_period_ms(std::uint8_t flight_state,
                                        std::uint32_t normal_period_ms,
                                        bool can_relayed)
{
    if (flight_state != kStatePadIdle) return normal_period_ms;
    if (can_relayed && !kPadIdleThrottlesCanRelayed) return normal_period_ms;
    return normal_period_ms > kPadIdleRadioPeriodMs ? normal_period_ms
                                                    : kPadIdleRadioPeriodMs;
}

// Wrap-safe "at least period_ms since last" for a 32-bit millisecond/tick count.
constexpr bool period_elapsed(std::uint32_t now, std::uint32_t last,
                              std::uint32_t period_ms)
{
    return static_cast<std::uint32_t>(now - last) >= period_ms;
}

// ---- Landed: low-power mode ------------------------------------------------
constexpr std::uint32_t kLandedTxPeriodMs = 60'000;      // landed position packet
constexpr std::uint32_t kLandedTxMinGapMs = 30'000;      // never closer than this
constexpr std::uint32_t kGpsWakeIntervalMs = 600'000;    // GPS wake every 10 min
constexpr std::uint32_t kGpsFixTimeoutMs = 90'000;       // give up one cycle
constexpr std::uint8_t kGpsMinSatellites = 4;            // for a "good" fix
constexpr std::uint32_t kPingMinIntervalMs = 5'000;      // ping replies
constexpr std::uint32_t kRadioLingerMs = 2'000;          // stay awake after radio RX/TX
constexpr std::uint32_t kMaxSleepChunkMs = 20'000;       // re-evaluate clocks this often
constexpr std::uint32_t kTxLeadMs = 150;                 // wake this much before a TX
// GPS wake handshake (UartDriver): hot restart first; if the receiver has sent
// nothing after this long, a cold start is used as the last resort.
constexpr std::uint32_t kGpsWakeHotWaitMs = 4'000;
// A radio command to Iris in Landed brings CAN back for this long.
constexpr std::uint32_t kCanGraceMs = 3'000;
// Landed housekeeping tasks (heartbeat, flash capture/writer) wake this often.
constexpr std::uint32_t kLowPowerTaskPeriodMs = 10'000;

// ---- UTC clock model -------------------------------------------------------
// GPS time (GGA hhmmss) is known only at fixes. Between them it is extrapolated
// from the local clock; the rate (utc ms per local ms) is learned from
// successive anchors because the clock that keeps time in light sleep (RC
// slow clock) is a percent or so off.
class UtcClock {
public:
    static constexpr std::uint64_t kDayMs = 86'400'000ULL;
    static constexpr std::uint64_t kMinRateSpanMs = 300'000;
    static constexpr double kMaxRateError = 0.03;

    bool valid() const { return valid_; }
    double rate() const { return rate_; }

    void observe(std::uint64_t local_ms, std::uint32_t utc_ms_of_day)
    {
        if (valid_ && local_ms > anchor_local_ms_) {
            const double elapsed = static_cast<double>(local_ms - anchor_local_ms_);
            const double predicted = static_cast<double>(anchor_utc_ms_) + elapsed * rate_;
            double error = static_cast<double>(utc_ms_of_day) - std::fmod(predicted, kDay());
            // Wrap to the nearest day boundary.
            if (error > kDay() / 2) error -= kDay();
            if (error < -kDay() / 2) error += kDay();
            if (elapsed >= static_cast<double>(kMinRateSpanMs)) {
                const double learned = (elapsed * rate_ + error) / elapsed;
                if (std::fabs(learned - 1.0) <= kMaxRateError) rate_ = learned;
            }
        }
        anchor_local_ms_ = local_ms;
        anchor_utc_ms_ = utc_ms_of_day;
        valid_ = true;
    }

    // UTC milliseconds of day at local_ms (valid() must be true).
    double utc_ms_at(std::uint64_t local_ms) const
    {
        const double elapsed = static_cast<double>(local_ms) -
                               static_cast<double>(anchor_local_ms_);
        double utc = std::fmod(static_cast<double>(anchor_utc_ms_) + elapsed * rate_, kDay());
        if (utc < 0) utc += kDay();
        return utc;
    }

    // First local time at or after local_ms where UTC seconds are 0.
    std::uint64_t minute_boundary_at_or_after(std::uint64_t local_ms) const
    {
        const double utc = utc_ms_at(local_ms);
        double remaining = 60'000.0 - std::fmod(utc, 60'000.0);
        if (remaining >= 60'000.0) remaining = 0.0;
        return local_ms + static_cast<std::uint64_t>(std::ceil(remaining / rate_));
    }

private:
    static constexpr double kDay() { return static_cast<double>(kDayMs); }
    bool valid_ = false;
    std::uint64_t anchor_local_ms_ = 0;
    std::uint32_t anchor_utc_ms_ = 0;
    double rate_ = 1.0;
};

// ---- Landed position transmit schedule --------------------------------------
// One packet per minute, at the top of each GPS minute when UTC is known,
// else on a free-running 60 s timer counted from the last fix (or from
// landing when no fix was ever seen). The first packet goes out immediately.
class TxSchedule {
public:
    void begin(std::uint64_t now_ms)
    {
        have_last_ = false;
        free_anchor_ms_ = now_ms;
    }
    void on_fix(std::uint64_t local_ms) { free_anchor_ms_ = local_ms; }
    void on_sent(std::uint64_t now_ms)
    {
        have_last_ = true;
        last_tx_ms_ = now_ms;
    }

    std::uint64_t next_tx_ms(const UtcClock &clock, std::uint64_t now_ms) const
    {
        if (!have_last_) return now_ms;
        const std::uint64_t earliest = last_tx_ms_ + kLandedTxMinGapMs;
        if (clock.valid()) return clock.minute_boundary_at_or_after(earliest);
        // Free-running: free_anchor_ + n * period, first one >= earliest.
        if (earliest <= free_anchor_ms_) return free_anchor_ms_ + kLandedTxPeriodMs;
        const std::uint64_t n =
            (earliest - free_anchor_ms_ + kLandedTxPeriodMs - 1) / kLandedTxPeriodMs;
        return free_anchor_ms_ + n * kLandedTxPeriodMs;
    }
    bool due(const UtcClock &clock, std::uint64_t now_ms) const
    {
        return now_ms >= next_tx_ms(clock, now_ms);
    }

private:
    bool have_last_ = false;
    std::uint64_t last_tx_ms_ = 0;
    std::uint64_t free_anchor_ms_ = 0;
};

// ---- GPS duty cycle ----------------------------------------------------------
// Active until a good fix arrives (then asleep) or kGpsFixTimeoutMs passes
// (asleep, previous landed location kept and marked stale). Asleep until
// kGpsWakeIntervalMs after the end of the previous active period.
class GpsDuty {
public:
    enum class Phase : std::uint8_t { Active, Asleep };
    enum class Action : std::uint8_t { None, Sleep, Wake };

    // Starts an active period (landing: the receiver is already running).
    void begin(std::uint64_t now_ms)
    {
        phase_ = Phase::Active;
        deadline_ms_ = now_ms + kGpsFixTimeoutMs;
    }

    // good_fix: a new valid fix with enough satellites arrived since the
    // active period began. result_ok is set when the action is Sleep.
    Action poll(std::uint64_t now_ms, bool good_fix, bool &result_ok)
    {
        result_ok = false;
        if (phase_ == Phase::Active) {
            if (good_fix || now_ms >= deadline_ms_) {
                result_ok = good_fix;
                phase_ = Phase::Asleep;
                next_wake_ms_ = now_ms + kGpsWakeIntervalMs;
                return Action::Sleep;
            }
        } else if (now_ms >= next_wake_ms_) {
            phase_ = Phase::Active;
            deadline_ms_ = now_ms + kGpsFixTimeoutMs;
            return Action::Wake;
        }
        return Action::None;
    }

    Phase phase() const { return phase_; }
    // Next time poll() can act on its own (the timeout or the wake time).
    std::uint64_t next_event_ms() const
    {
        return phase_ == Phase::Active ? deadline_ms_ : next_wake_ms_;
    }

private:
    Phase phase_ = Phase::Active;
    std::uint64_t deadline_ms_ = 0;
    std::uint64_t next_wake_ms_ = 0;
};

// ---- Ping reply limiter --------------------------------------------------------
class PingLimiter {
public:
    bool allow(std::uint64_t now_ms)
    {
        if (have_ && now_ms - last_ms_ < kPingMinIntervalMs) return false;
        have_ = true;
        last_ms_ = now_ms;
        return true;
    }

private:
    bool have_ = false;
    std::uint64_t last_ms_ = 0;
};

// How long the Landed loop may sleep: until the earliest event, less the lead,
// capped, and at least min_ms (a due event returns min_ms so the loop spins
// once instead of sleeping).
constexpr std::uint32_t sleep_ms(std::uint64_t now_ms, std::uint64_t event_ms,
                                 std::uint32_t lead_ms, std::uint32_t min_ms)
{
    if (event_ms <= now_ms + lead_ms) return min_ms;
    const std::uint64_t wait = event_ms - now_ms - lead_ms;
    const std::uint64_t capped = wait > kMaxSleepChunkMs ? kMaxSleepChunkMs : wait;
    return capped < min_ms ? min_ms : static_cast<std::uint32_t>(capped);
}

}  // namespace landed
