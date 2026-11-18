#pragma once

#include <cstdint>

// Remembers the commanded load mask (OUT1..OUT3, 5V; MCP pin bits 0..3)
// across WARM resets only.
// Owner decision 2026-11-16: no output is energised at power-up. The mask is
// kept in one RTC_NOINIT record (magic + CRC32), written at once on every
// command and cleared when all loads are commanded off. It is restored only
// when the reset reason is warm (software, panic, watchdogs, brownout) AND the
// record is valid. Nothing is read from or written to flash (NVS is only used
// once, to erase the stale "loads" key an older build stored).
namespace load_state {

struct BootResult {
    bool restore;
    std::uint8_t mask;
};

// Must be called once, first thing in app_main. Logs loudly.
BootResult begin_boot();

// Record the currently commanded mask (call whenever it changes). Updates the
// RTC record immediately (cleared when mask is 0).
void store(std::uint8_t mask);

// Newest commanded mask (what the outputs should be), including commands that
// could not be applied yet because the expander was not ready.
std::uint8_t commanded_mask();

// Call from the main loop: erases the stale NVS "loads" key once (a few
// seconds after boot, off the restore path). Cheap when idle.
void service(std::int64_t now_us);

}  // namespace load_state
