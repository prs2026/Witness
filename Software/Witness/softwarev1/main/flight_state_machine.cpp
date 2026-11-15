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
        persist();
    }
    return ESP_OK;
}

void FlightStateMachine::persist() const
{
    if (persist_ != nullptr) {
        persist_(static_cast<std::uint8_t>(state_.load()),
                 maximum_altitude_meters_, maximum_altitude_valid_);
    }
}

void FlightStateMachine::restore(
    const State state, const float maximum_altitude_m, const bool maximum_valid)
{
    state_.store(state);
    observed_state_ = state;   // keep the restored maximum (no re-baseline)
    witness_status_.set_flight_state(static_cast<std::uint8_t>(state));
    maximum_altitude_meters_ = maximum_altitude_m;
    maximum_altitude_valid_ = maximum_valid;
    coast_entered_us_ = 0;           // set on the first update
    climb_seen_ = true;              // a persisted flight state implies a climb
    velocity_condition_active_ = false;
    reset_condition();
    reset_landing_condition();
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

    if (!motion_stable || !observation.agl_valid) {
        reset_landing_condition();
        return false;
    }

    if (!landing_condition_active_) {
        landing_minimum_altitude_meters_ = observation.agl_meters;
        landing_maximum_altitude_meters_ = observation.agl_meters;
    } else {
        landing_minimum_altitude_meters_ =
            std::min(landing_minimum_altitude_meters_, observation.agl_meters);
        landing_maximum_altitude_meters_ =
            std::max(landing_maximum_altitude_meters_, observation.agl_meters);
    }

    const bool altitude_stable =
        landing_maximum_altitude_meters_ - landing_minimum_altitude_meters_ <=
        kLandingAltitudeStabilityMeters;

    if (!altitude_stable) {
        reset_landing_condition();
        landing_minimum_altitude_meters_ = observation.agl_meters;
        landing_maximum_altitude_meters_ = observation.agl_meters;
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
        "AGL range %.2f m (AGL %.1f m)",
        acceleration_magnitude,
        gyro_magnitude,
        landing_maximum_altitude_meters_ - landing_minimum_altitude_meters_,
        observation.agl_meters);
    return true;
}

void FlightStateMachine::reset_for_state(
    const State state,
    const Observation &observation)
{
    observed_state_ = state;
    reset_condition();
    velocity_condition_active_ = false;
    if (state == State::Coast) {
        coast_entered_us_ = observation.timestamp_us;
    }
    if (state == State::PadIdle || state == State::Boost) {
        climb_seen_ = false;
        // Start flight-relative peak tracking at launch (Kalman AGL).
        maximum_altitude_valid_ = observation.agl_valid;
        maximum_altitude_meters_ = observation.agl_meters;
    }
    if (state == State::PadIdle || state == State::Landed) {
        reset_landing_condition();
    }
}

void FlightStateMachine::update(const Observation &observation)
{
    if (!kAutomaticTransitionsEnabled || observation.timestamp_us == 0U) {
        return;
    }

    if (rebaseline_requested_.exchange(false)) {
        maximum_altitude_valid_ = false;
        maximum_altitude_meters_ = 0.0F;
        velocity_condition_active_ = false;
        reset_condition();
        reset_landing_condition();
        persist();
    }

    const bool agl_valid = observation.agl_valid && std::isfinite(observation.agl_meters);
    const float agl = agl_valid ? observation.agl_meters : 0.0F;

    const State current = state();
    if (current != observed_state_) {
        reset_for_state(current, observation);
    }

    if ((current == State::Boost || current == State::Coast) &&
        observation.vertical_velocity_valid &&
        observation.vertical_velocity_mps > kApogeeSeenVelocityMps) {
        climb_seen_ = true;
    }
    if (current == State::Coast && coast_entered_us_ == 0U) {
        coast_entered_us_ = observation.timestamp_us;
    }

    // Maximum AGL since launch (the filter is gated against barometer
    // outliers, so no median or rate limit is needed here).
    if (agl_valid && (current == State::Boost || current == State::Coast)) {
        if (!maximum_altitude_valid_ || agl > maximum_altitude_meters_) {
            maximum_altitude_meters_ = agl;
            maximum_altitude_valid_ = true;
            if (current == State::Coast) {
                reset_condition();   // still climbing: no Descent dwell
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
                agl_valid && agl > kLaunchAglMeters,
                observation.timestamp_us,
                kLaunchQualificationMs)) {
            ESP_LOGI(kLogTag, "launch detected: Kalman AGL %.1f m, vertical velocity %.1f m/s",
                     agl, observation.vertical_velocity_mps);
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
        // AGL path: below 95 % of the maximum AGL (and at least 10 m below
        // it) and falling.
        const float drop = std::max(maximum_altitude_meters_ * (1.0F - kDescentAltitudeFraction),
                                    kDescentMinimumDropMeters);
        const float descent_threshold = maximum_altitude_meters_ - drop;
        const bool falling =
            observation.vertical_velocity_valid &&
            observation.vertical_velocity_mps < 0.0F;
        const bool below_descent_threshold =
            agl_valid && maximum_altitude_valid_ && maximum_altitude_meters_ > 0.0F &&
            falling && agl < descent_threshold;
        if (condition_held(
                below_descent_threshold,
                observation.timestamp_us,
                kDescentQualificationMs)) {
            ESP_LOGI(kLogTag, "descent detected (Kalman AGL): AGL %.1f m, maximum %.1f m",
                     agl, maximum_altitude_meters_);
            (void)set_state(static_cast<std::uint8_t>(State::Descent));
            break;
        }

        // Velocity path: Kalman vertical velocity clearly downward after an
        // observed climb (covers a maximum that is not valid or not positive).
        const bool velocity_descending =
            climb_seen_ &&
            observation.vertical_velocity_valid &&
            observation.vertical_velocity_mps < kDescentVelocityMps &&
            observation.timestamp_us - coast_entered_us_ >=
                milliseconds_to_microseconds(kDescentMinimumCoastMs);
        if (!velocity_descending) {
            velocity_condition_active_ = false;
        } else if (!velocity_condition_active_) {
            velocity_condition_active_ = true;
            velocity_condition_started_us_ = observation.timestamp_us;
        } else if (observation.timestamp_us - velocity_condition_started_us_ >=
                   milliseconds_to_microseconds(
                       kDescentVelocityQualificationMs)) {
            ESP_LOGI(kLogTag, "descent detected (Kalman velocity): vertical velocity %.1f m/s, AGL %.1f m",
                     observation.vertical_velocity_mps, agl);
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
