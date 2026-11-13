#pragma once

#include <cstdint>

// Keeps the last commanded load mask (OUT1..OUT3, 5V) in RTC_NOINIT memory so
// it can be re-applied after a warm Iris reset (panic, watchdog, brownout).
namespace load_state {

// Decide from esp_reset_reason() and the RTC record. Must be called once,
// first thing in app_main. When restore is false the record is re-initialised
// to "all off". Logs loudly.
struct BootResult {
    bool restore;
    std::uint8_t mask;
};
BootResult begin_boot();

// Persist the currently commanded mask (call whenever it changes).
void store(std::uint8_t mask);

}  // namespace load_state
