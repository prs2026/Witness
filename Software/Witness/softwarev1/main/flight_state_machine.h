#pragma once

#include <atomic>
#include <cstdint>

#include "esp_err.h"

// Flight-state settings. Automatic transition logic will be added later;
// these switches define which states currently permit binary sensor logging.
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

    explicit FlightStateMachine(WitnessStatus &witness_status);
    FlightStateMachine(const FlightStateMachine &) = delete;
    FlightStateMachine &operator=(const FlightStateMachine &) = delete;

    State state() const;
    esp_err_t set_state(std::uint8_t state_value);
    bool binary_logging_enabled() const;
    static const char *state_name(State state);

private:
    static_assert(FLIGHT_LOG_BIN_PAD_IDLE == 0 || FLIGHT_LOG_BIN_PAD_IDLE == 1);
    static_assert(FLIGHT_LOG_BIN_BOOST == 0 || FLIGHT_LOG_BIN_BOOST == 1);
    static_assert(FLIGHT_LOG_BIN_COAST == 0 || FLIGHT_LOG_BIN_COAST == 1);
    static_assert(FLIGHT_LOG_BIN_DESCENT == 0 || FLIGHT_LOG_BIN_DESCENT == 1);
    static_assert(FLIGHT_LOG_BIN_LANDED == 0 || FLIGHT_LOG_BIN_LANDED == 1);

    std::atomic<State> state_{State::PadIdle};
    WitnessStatus &witness_status_;
};
