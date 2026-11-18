#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_err.h"

// Flight-state settings. These switches define which states permit binary
// sensor logging.
#define FLIGHT_LOG_BIN_PAD_IDLE 0
#define FLIGHT_LOG_BIN_BOOST 1
#define FLIGHT_LOG_BIN_COAST 1
#define FLIGHT_LOG_BIN_DESCENT 1
#define FLIGHT_LOG_BIN_LANDED 0

class WitnessStatus;

class FlightStateMachine final {
public:
    enum class State : std::uint8_t {
        PadIdle = 0x00,
        Boost = 0x01,
        Coast = 0x02,
        Descent = 0x03,
        Landed = 0x04,
    };

    // Automatic flight-state thresholds. Durations are consecutive qualifying
    // time, not a count of sensor-task iterations.
    //
    // Every altitude in the state machine is the position filter's (Kalman)
    // height above the pad (AGL, its "up" state; the filter pins it to 0
    // while the vehicle stands still on the pad). The raw barometer is never
    // used here: its static pressure is wrong around Mach 1 in Boost and
    // Coast, and the filter already gates it (see position_filter.h).
    static constexpr bool kAutomaticTransitionsEnabled = true;
    // Pad Idle -> Boost: Kalman AGL above kLaunchAglMeters AND Kalman vertical
    // velocity at or above kLaunchVerticalVelocityMps, both true continuously
    // for kLaunchQualificationMs. Detection delay after liftoff for net
    // acceleration a: max(sqrt(60/a), 30/a) + 0.3 s, i.e. ~2.1 s at 2 g net
    // and ~3.4 s at 1 g net (the pre-launch RAM ring is 2 s: the acceleration
    // rule below fires 0.3 s after ignition for >= 2.5 g instead). A rocket is
    // far above 30 m/s by 100 m AGL; a slow climb (vehicle, hoist, aircraft,
    // a stale position offset) never reaches the velocity and never launches.
    static constexpr float kLaunchAglMeters = 30.0F;
    static constexpr float kLaunchVerticalVelocityMps = 30.0F;
    static constexpr std::uint32_t kLaunchQualificationMs = 300;
    // Pad Idle -> Boost, second and independent trigger (needs no attitude and
    // no position filter): the filtered axial acceleration (body +X, nose) at
    // or above kLaunchAccelerationG for kLaunchAccelerationQualificationMs.
    // The time above the threshold is a leaky budget: below the threshold it
    // drains kLaunchAccelerationDrainFactor times as fast as it fills, so a
    // motor that chuffs for a few samples still qualifies while handling
    // shocks (short, spiky, mostly below the threshold) never accumulate.
    // 2.5 g for 0.3 s along the nose axis is >= 4.4 m/s of delta-v above 1 g,
    // continuously: transport, a drop (free fall reads 0 g), a tipped rail or
    // a hand toss do not do that; any rail-launched motor (thrust-to-weight
    // above 2.5) does. Weaker motors are caught by the Kalman AGL + velocity
    // rule above.
    static constexpr float kLaunchAccelerationG = 2.5F;
    static constexpr std::uint32_t kLaunchAccelerationQualificationMs = 300;
    static constexpr std::uint32_t kLaunchAccelerationDrainFactor = 2;
    // Airborne at boot (cold reset in flight, or a launch missed by both rules
    // above): in Pad Idle a raw barometer rate (BaroRate, independent of the
    // filter and its pad hold) of at least kAirborneRateMps up (Boost) or
    // down (Descent) for kAirborneQualificationMs. 20 m/s sustained for a
    // second is 20 m of altitude, ~240 Pa: not weather, wind on the static
    // port, a door or an elevator (a lift is < 10 m/s). A parachute descent
    // slower than 20 m/s (main) is not recognised; the drogue (typ. 25-40) is.
    static constexpr float kAirborneRateMps = 20.0F;
    static constexpr std::uint32_t kAirborneQualificationMs = 1000;
    // A flight that never exceeded kFlightEvidenceAglMeters of AGL and never
    // read kFlightEvidenceAxialG of axial acceleration (in Boost/Coast/Descent)
    // is treated as a false trigger when Landed is declared: the false-flight
    // callback (if the application set one) is called right after Landed was
    // persisted, so the application can do the soft reset to Pad Idle that
    // 0x7300 does. Without a callback nothing changes. OFF by the owner's
    // decision of 2026-11-16: the code and the callback hook stay, but the
    // callback is never called (and nothing registers one today).
    static constexpr bool kFalseFlightRestartEnabled = false;
    static constexpr float kFlightEvidenceAglMeters = 50.0F;
    static constexpr float kFlightEvidenceAxialG = 3.0F;
    static constexpr float kBurnoutAccelerationG = 1.0F;
    static constexpr std::uint32_t kBurnoutQualificationMs = 500;
    // Coast -> Descent (Kalman path): AGL below kDescentAltitudeFraction of
    // the maximum AGL, with downward Kalman vertical velocity, for
    // kDescentQualificationMs.
    static constexpr float kDescentAltitudeFraction = 0.95F;
    // ... and at least this far below the maximum (a commanded Coast on the
    // bench, or a very low hop, has a maximum near 0 m where 5 % is nothing).
    static constexpr float kDescentMinimumDropMeters = 10.0F;
    static constexpr std::uint32_t kDescentQualificationMs = 500;
    static constexpr float kLandingMaximumGyroDps = 10.0F;
    static constexpr float kLandingMinimumAccelerationG = 0.8F;
    static constexpr float kLandingMaximumAccelerationG = 1.2F;
    // Kalman AGL span allowed over the landing window.
    static constexpr float kLandingAltitudeStabilityMeters = 10.0F;
    static constexpr std::uint32_t kLandingQualificationMs = 30'000;
    // Coast -> Descent (velocity path, independent of the AGL fraction):
    // Kalman vertical velocity below kDescentVelocityMps for
    // kDescentVelocityQualificationMs, at least kDescentMinimumCoastMs after
    // burnout, and only after the vehicle was seen climbing faster than
    // kApogeeSeenVelocityMps (an apogee happened).
    static constexpr float kDescentVelocityMps = -30.0F;
    static constexpr std::uint32_t kDescentVelocityQualificationMs = 2000;
    static constexpr std::uint32_t kDescentMinimumCoastMs = 3000;
    static constexpr float kApogeeSeenVelocityMps = 20.0F;

    struct Observation {
        float axial_acceleration_g = 0.0F;
        float low_g_axial_acceleration_g = 0.0F;
        float acceleration_g[3]{};
        float angular_rate_dps[3]{};
        std::uint64_t timestamp_us = 0;
        bool axial_acceleration_valid = false;
        bool low_g_axial_acceleration_valid = false;
        bool acceleration_valid = false;
        bool angular_rate_valid = false;
        // Position filter (Kalman) height above the pad and vertical velocity
        // (up positive). Valid once the filter runs.
        bool agl_valid = false;
        float agl_meters = 0.0F;
        bool vertical_velocity_valid = false;
        float vertical_velocity_mps = 0.0F;
        // Raw barometer altitude rate (position::BaroRate), up positive.
        bool baro_rate_valid = false;
        float baro_rate_mps = 0.0F;
    };

    // Called (from any task) on every state change with the new state, the
    // tracked maximum AGL and whether it is valid. Used to persist the state
    // across a reset (flight_state_store).
    using PersistFn = void (*)(std::uint8_t state, float maximum_altitude_m,
                               bool maximum_valid);
    void set_persist_callback(PersistFn fn) { persist_ = fn; }
    // Called (from the sensor task) right after Landed was declared by the
    // automatic landing test when the flight showed no evidence of being real
    // (see kFalseFlightRestartEnabled). The callback should schedule the same
    // soft reset to Pad Idle that 0x7300 performs in Landed.
    using FalseFlightFn = void (*)();
    void set_false_flight_callback(FalseFlightFn fn) { false_flight_ = fn; }
    // True once the flight is known to be real: resumed after a reset, inferred
    // airborne at boot, AGL >= kFlightEvidenceAglMeters or axial acceleration
    // >= kFlightEvidenceAxialG seen in flight.
    bool flight_evidence() const;
    float peak_axial_acceleration_g() const { return peak_axial_g_; }
    // Boot-time restore after an in-flight reset: sets the state and the
    // tracked maximum AGL without logging a transition or invoking the persist
    // callback.
    void restore(State state, float maximum_altitude_m, bool maximum_valid);
    // Discards the tracked maximum and pending qualification timers (sensor
    // source changed, e.g. HITL entry/exit). Thread-safe; takes effect on the
    // next update().
    void rebaseline() { rebaseline_requested_.store(true); }
    // Maximum Kalman AGL since launch (metres above the pad).
    float maximum_altitude() const { return maximum_altitude_meters_; }
    bool maximum_altitude_valid() const { return maximum_altitude_valid_; }

    explicit FlightStateMachine(WitnessStatus &witness_status);
    FlightStateMachine(const FlightStateMachine &) = delete;
    FlightStateMachine &operator=(const FlightStateMachine &) = delete;

    State state() const;
    // The flight states are one-way (Pad Idle -> Boost -> Coast -> Descent ->
    // Landed). Setting the current state is a no-op that succeeds; a lower
    // state is refused with ESP_ERR_INVALID_STATE. The only way back to Pad
    // Idle is a restart (restore() is the separate boot-time path).
    esp_err_t set_state(std::uint8_t state_value);
    // Performs the same Pad Idle -> Boost transition used by automatic
    // launch detection. Intended for a commanded/manual launch trigger.
    esp_err_t trigger_launch();
    void update(const Observation &observation);
    bool binary_logging_enabled() const;
    static const char *state_name(State state);

private:
    static_assert(FLIGHT_LOG_BIN_PAD_IDLE == 0 || FLIGHT_LOG_BIN_PAD_IDLE == 1);
    static_assert(FLIGHT_LOG_BIN_BOOST == 0 || FLIGHT_LOG_BIN_BOOST == 1);
    static_assert(FLIGHT_LOG_BIN_COAST == 0 || FLIGHT_LOG_BIN_COAST == 1);
    static_assert(FLIGHT_LOG_BIN_DESCENT == 0 || FLIGHT_LOG_BIN_DESCENT == 1);
    static_assert(FLIGHT_LOG_BIN_LANDED == 0 || FLIGHT_LOG_BIN_LANDED == 1);

    static constexpr std::uint64_t milliseconds_to_microseconds(
        const std::uint32_t milliseconds)
    {
        return static_cast<std::uint64_t>(milliseconds) * 1000ULL;
    }

    bool condition_held(
        bool condition,
        std::uint64_t timestamp_us,
        std::uint32_t duration_ms);
    bool landing_condition_held(
        bool condition,
        std::uint64_t timestamp_us,
        std::uint32_t duration_ms);
    bool landing_detected(const Observation &observation);
    bool acceleration_launch_detected(const Observation &observation);
    struct Dwell {
        bool active = false;
        std::uint64_t start_us = 0;
    };
    static bool dwell_held(Dwell &dwell, bool condition, std::uint64_t timestamp_us,
                           std::uint32_t duration_ms);
    void reset_condition();
    void reset_landing_condition();
    void reset_for_state(State state, const Observation &observation);
    void persist() const;

    std::atomic<State> state_{State::PadIdle};
    WitnessStatus &witness_status_;
    State observed_state_ = State::PadIdle;
    std::uint64_t condition_started_us_ = 0;
    bool condition_active_ = false;
    std::uint64_t landing_condition_started_us_ = 0;
    bool landing_condition_active_ = false;
    float maximum_altitude_meters_ = 0.0F;
    bool maximum_altitude_valid_ = false;
    std::atomic<bool> rebaseline_requested_{false};
    PersistFn persist_ = nullptr;
    std::uint64_t coast_entered_us_ = 0;
    bool climb_seen_ = false;
    bool velocity_condition_active_ = false;
    std::uint64_t velocity_condition_started_us_ = 0;
    FalseFlightFn false_flight_ = nullptr;
    std::uint64_t launch_accel_budget_us_ = 0;
    std::uint64_t launch_accel_last_us_ = 0;
    Dwell airborne_up_{};
    Dwell airborne_down_{};
    float peak_axial_g_ = 0.0F;
    bool flight_inferred_ = false;   // resumed or declared from the barometer
    float landing_minimum_altitude_meters_ = 0.0F;
    float landing_maximum_altitude_meters_ = 0.0F;
};
