#include "flight_state_machine.h"

#include <algorithm>
#include <cmath>

#include "esp_log.h"
#include "witness_status.h"

namespace {
constexpr char kLogTag[] = "flight_state";
}

FlightStateMachine::FlightStateMachine(WitnessStatus &witness_status)
    : witness_status_(witness_status)
{
    witness_status_.set_flight_state(
        static_cast<std::uint8_t>(State::PadIdle));
}

FlightStateMachine::State FlightStateMachine::state() const
{
    return state_.load();
}

esp_err_t FlightStateMachine::set_state(const std::uint8_t state_value)
{
    if (state_value > static_cast<std::uint8_t>(State::Landed)) {
        return ESP_ERR_INVALID_ARG;
    }

    const State next = static_cast<State>(state_value);
    const State previous = state_.exchange(next);
    witness_status_.set_flight_state(state_value);
    if (previous != next) {
        ESP_LOGI(kLogTag, "%s -> %s", state_name(previous), state_name(next));
    }
    return ESP_OK;
}

esp_err_t FlightStateMachine::trigger_launch()
{
    if (state() != State::PadIdle) {
        return ESP_ERR_INVALID_STATE;
    }
    return set_state(static_cast<std::uint8_t>(State::Boost));
}

bool FlightStateMachine::condition_held(
    const bool condition,
    const std::uint64_t timestamp_us,
    const std::uint32_t duration_ms)
{
    if (!condition) {
        reset_condition();
        return false;
    }
    if (!condition_active_) {
        condition_active_ = true;
        condition_started_us_ = timestamp_us;
        return false;
    }
    return timestamp_us - condition_started_us_ >=
           milliseconds_to_microseconds(duration_ms);
}

void FlightStateMachine::reset_condition()
{
    condition_active_ = false;
    condition_started_us_ = 0;
}

bool FlightStateMachine::landing_condition_held(
    const bool condition,
    const std::uint64_t timestamp_us,
    const std::uint32_t duration_ms)
{
    if (!condition) {
        reset_landing_condition();
        return false;
    }
    if (!landing_condition_active_) {
        landing_condition_active_ = true;
        landing_condition_started_us_ = timestamp_us;
        return false;
    }
    return timestamp_us - landing_condition_started_us_ >=
           milliseconds_to_microseconds(duration_ms);
}

void FlightStateMachine::reset_landing_condition()
{
    landing_condition_active_ = false;
    landing_condition_started_us_ = 0;
}

bool FlightStateMachine::landing_detected(const Observation &observation)
{
    const float acceleration_magnitude = std::sqrt(
        observation.acceleration_g[0] * observation.acceleration_g[0] +
        observation.acceleration_g[1] * observation.acceleration_g[1] +
        observation.acceleration_g[2] * observation.acceleration_g[2]);
    const float gyro_magnitude = std::sqrt(
        observation.angular_rate_dps[0] * observation.angular_rate_dps[0] +
        observation.angular_rate_dps[1] * observation.angular_rate_dps[1] +
        observation.angular_rate_dps[2] * observation.angular_rate_dps[2]);
    const bool motion_stable =
        observation.acceleration_valid &&
        observation.angular_rate_valid &&
        acceleration_magnitude >= kLandingMinimumAccelerationG &&
        acceleration_magnitude <= kLandingMaximumAccelerationG &&
        gyro_magnitude < kLandingMaximumGyroDps;

    if (!motion_stable || !observation.barometric_altitude_valid) {
        reset_landing_condition();
        return false;
    }

    if (!landing_condition_active_) {
        landing_minimum_altitude_meters_ =
            observation.barometric_altitude_meters;
        landing_maximum_altitude_meters_ =
            observation.barometric_altitude_meters;
    } else {
        landing_minimum_altitude_meters_ = std::min(
            landing_minimum_altitude_meters_,
            observation.barometric_altitude_meters);
        landing_maximum_altitude_meters_ = std::max(
            landing_maximum_altitude_meters_,
            observation.barometric_altitude_meters);
    }

    const bool altitude_stable =
        landing_maximum_altitude_meters_ -
            landing_minimum_altitude_meters_ <=
        kLandingAltitudeStabilityMeters;

    if (!altitude_stable) {
        reset_landing_condition();
        landing_minimum_altitude_meters_ =
            observation.barometric_altitude_meters;
        landing_maximum_altitude_meters_ =
            observation.barometric_altitude_meters;
        return false;
    }

    if (!landing_condition_held(
            true,
            observation.timestamp_us,
            kLandingQualificationMs)) {
        return false;
    }

    ESP_LOGI(
        kLogTag,
        "landing detected: accel %.2f g, gyro %.2f dps, "
        "altitude range %.2f m",
        acceleration_magnitude,
        gyro_magnitude,
        landing_maximum_altitude_meters_ -
            landing_minimum_altitude_meters_);
    return true;
}

void FlightStateMachine::reset_for_state(
    const State state,
    const Observation &observation)
{
    observed_state_ = state;
    reset_condition();

    if (state == State::PadIdle || state == State::Boost) {
        // Start flight-relative peak tracking at launch. This prevents a
        // prelaunch pressure spike from making the Coast -> Descent threshold
        // unreachable after either an automatic or commanded launch.
        maximum_altitude_valid_ = observation.barometric_altitude_valid;
        maximum_altitude_meters_ =
            observation.barometric_altitude_meters;
    }
    if (state == State::PadIdle) {
        reset_landing_condition();
    }
    if (state == State::Landed) {
        reset_landing_condition();
    }
}

void FlightStateMachine::update(const Observation &observation)
{
    if (!kAutomaticTransitionsEnabled || observation.timestamp_us == 0U) {
        return;
    }

    const State current = state();
    if (current != observed_state_) {
        reset_for_state(current, observation);
    }

    if (observation.barometric_altitude_valid) {
        if (!maximum_altitude_valid_ ||
            observation.barometric_altitude_meters >
                maximum_altitude_meters_) {
            maximum_altitude_meters_ =
                observation.barometric_altitude_meters;
            maximum_altitude_valid_ = true;
            if (current == State::Coast) {
                reset_condition();
            }
        }
    }

    if ((current == State::Boost || current == State::Coast ||
         current == State::Descent) &&
        landing_detected(observation)) {
        (void)set_state(static_cast<std::uint8_t>(State::Landed));
        return;
    }

    switch (current) {
    case State::PadIdle:
        if (condition_held(
                observation.axial_acceleration_valid &&
                    observation.axial_acceleration_g >
                        kLaunchAccelerationG,
                observation.timestamp_us,
                kLaunchQualificationMs)) {
            ESP_LOGI(kLogTag, "launch detected: axial acceleration %.2f g",
                     observation.axial_acceleration_g);
            (void)trigger_launch();
        }
        break;

    case State::Boost:
        if (condition_held(
                observation.low_g_axial_acceleration_valid &&
                    observation.low_g_axial_acceleration_g <
                        kBurnoutAccelerationG,
                observation.timestamp_us,
                kBurnoutQualificationMs)) {
            ESP_LOGI(kLogTag, "burnout detected: axial acceleration %.2f g",
                     observation.low_g_axial_acceleration_g);
            (void)set_state(static_cast<std::uint8_t>(State::Coast));
        }
        break;

    case State::Coast: {
        // For positive altitude this is exactly 95% of the recorded maximum.
        // Using the absolute maximum in the drop term also behaves correctly
        // at launch sites whose pressure altitude is below zero.
        const float descent_altitude_threshold =
            maximum_altitude_meters_ -
            std::fabs(maximum_altitude_meters_) *
                (1.0F - kDescentAltitudeFraction);
        const bool below_descent_threshold =
            observation.barometric_altitude_valid &&
            maximum_altitude_valid_ &&
            observation.barometric_altitude_meters <
                descent_altitude_threshold;
        if (condition_held(
                below_descent_threshold,
                observation.timestamp_us,
                kDescentQualificationMs)) {
            ESP_LOGI(
                kLogTag,
                "descent detected: altitude %.2f m, maximum %.2f m",
                observation.barometric_altitude_meters,
                maximum_altitude_meters_);
            (void)set_state(static_cast<std::uint8_t>(State::Descent));
        }
        break;
    }

    case State::Descent:
        break;

    case State::Landed:
        reset_condition();
        reset_landing_condition();
        break;
    }
}

bool FlightStateMachine::binary_logging_enabled() const
{
    switch (state()) {
    case State::PadIdle:
        return FLIGHT_LOG_BIN_PAD_IDLE != 0;
    case State::Boost:
        return FLIGHT_LOG_BIN_BOOST != 0;
    case State::Coast:
        return FLIGHT_LOG_BIN_COAST != 0;
    case State::Descent:
        return FLIGHT_LOG_BIN_DESCENT != 0;
    case State::Landed:
        return FLIGHT_LOG_BIN_LANDED != 0;
    }
    return false;
}

const char *FlightStateMachine::state_name(const State state)
{
    switch (state) {
    case State::PadIdle: return "pad idle";
    case State::Boost: return "boost";
    case State::Coast: return "coast";
    case State::Descent: return "descent";
    case State::Landed: return "landed";
    }
    return "invalid";
}
