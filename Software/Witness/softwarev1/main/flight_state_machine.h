#pragma once

#include <atomic>
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
    static constexpr bool kAutomaticTransitionsEnabled = true;
    static constexpr float kLaunchAccelerationG = 10.0F;
    static constexpr std::uint32_t kLaunchQualificationMs = 300;
    static constexpr float kBurnoutAccelerationG = 1.0F;
    static constexpr std::uint32_t kBurnoutQualificationMs = 500;
    static constexpr float kDescentAltitudeFraction = 0.95F;
    static constexpr std::uint32_t kDescentQualificationMs = 500;
    static constexpr float kLandingMaximumGyroDps = 10.0F;
    static constexpr float kLandingMinimumAccelerationG = 0.8F;
    static constexpr float kLandingMaximumAccelerationG = 1.2F;
    static constexpr float kLandingAltitudeStabilityMeters = 10.0F;
    static constexpr std::uint32_t kLandingQualificationMs = 30'000;

    struct Observation {
        float axial_acceleration_g = 0.0F;
        float low_g_axial_acceleration_g = 0.0F;
        float acceleration_g[3]{};
        float angular_rate_dps[3]{};
        float barometric_altitude_meters = 0.0F;
        std::uint64_t timestamp_us = 0;
        bool axial_acceleration_valid = false;
        bool low_g_axial_acceleration_valid = false;
        bool acceleration_valid = false;
        bool angular_rate_valid = false;
        bool barometric_altitude_valid = false;
    };

    explicit FlightStateMachine(WitnessStatus &witness_status);
    FlightStateMachine(const FlightStateMachine &) = delete;
    FlightStateMachine &operator=(const FlightStateMachine &) = delete;

    State state() const;
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
    void reset_condition();
    void reset_landing_condition();
    void reset_for_state(State state, const Observation &observation);

    std::atomic<State> state_{State::PadIdle};
    WitnessStatus &witness_status_;
    State observed_state_ = State::PadIdle;
    std::uint64_t condition_started_us_ = 0;
    bool condition_active_ = false;
    std::uint64_t landing_condition_started_us_ = 0;
    bool landing_condition_active_ = false;
    float maximum_altitude_meters_ = 0.0F;
    bool maximum_altitude_valid_ = false;
    float landing_minimum_altitude_meters_ = 0.0F;
    float landing_maximum_altitude_meters_ = 0.0F;
};
