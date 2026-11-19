#pragma once

// Red-LED warning pattern (pure, host test tests/led_pattern_test.cpp).
//
// The heartbeat task pulses the red LED once a second. When the vehicle is NOT
// LOGGING to flash the pulse becomes a burst of kNotLoggingPulses, which is
// visible on the pad without a ground station (the ground already shows the
// status bits). Not logging means any of: the flash failed its start-up
// (FLASH_INIT_FAILED), logging stopped on a write error (FLASH_LOG_FAILED), or
// both log slots hold a flight and nothing is erased automatically
// (LOG_SLOTS_FULL).

#include <cstdint>

#include "comms.h"

namespace led_pattern {

inline constexpr unsigned kNormalPulses = 1;
inline constexpr unsigned kNotLoggingPulses = 3;

constexpr bool not_logging(const std::uint8_t status_byte0, const std::uint8_t status_byte1)
{
    return (status_byte0 & (IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK |
                            IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK)) != 0U ||
           (status_byte1 & IRIS_WITNESS_STATUS_LOG_SLOTS_FULL_MASK) != 0U;
}

constexpr unsigned pulses(const std::uint8_t status_byte0, const std::uint8_t status_byte1)
{
    return not_logging(status_byte0, status_byte1) ? kNotLoggingPulses : kNormalPulses;
}

}  // namespace led_pattern
