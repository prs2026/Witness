#include "flight_state_machine.h"

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
