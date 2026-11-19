// g++ -std=c++17 -Wall -Wextra -Werror -I main tests/led_pattern_test.cpp -o build/led_pattern_test.exe
#include "led_pattern.h"

#include <cassert>
#include <cstdio>

using namespace led_pattern;

int main()
{
    assert(pulses(0, 0) == kNormalPulses);
    // Healthy status with every other flag set: still a single pulse.
    assert(pulses(static_cast<std::uint8_t>(~(IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK |
                                              IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK)),
                  static_cast<std::uint8_t>(~IRIS_WITNESS_STATUS_LOG_SLOTS_FULL_MASK)) == kNormalPulses);
    assert(pulses(IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK, 0) == kNotLoggingPulses);
    assert(pulses(IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK, 0) == kNotLoggingPulses);
    assert(pulses(0, IRIS_WITNESS_STATUS_LOG_SLOTS_FULL_MASK) == kNotLoggingPulses);
    assert(not_logging(IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK | IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK,
                       IRIS_WITNESS_STATUS_LOG_SLOTS_FULL_MASK));
    // The flight state and HITL/resumed bits in byte 1 are not a logging problem.
    assert(!not_logging(0, IRIS_WITNESS_STATUS_HITL_ACTIVE_MASK | IRIS_WITNESS_STATUS_RESUMED_MASK | 0x07U));
    std::puts("led_pattern_test OK");
    return 0;
}
