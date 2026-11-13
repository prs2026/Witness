#pragma once

#include <cstdint>

// Maintenance boot (Wi-Fi OTA and log download). Witness does not have the
// RAM to run Wi-Fi next to the flash logger's write path, so Wi-Fi only runs
// after a reboot into maintenance mode, in which the logger only reads (its
// queues, capture task and console hook are never created). Maintenance is
// requested through RTC memory that survives esp_restart() and panics but not
// a power cycle, and it is consumed at boot: a crash, watchdog reset or power
// loss in maintenance mode comes back in normal flight mode.
//
// While active, status byte 1 carries IRIS_WITNESS_STATUS_MAINTENANCE_MASK so
// the ground station shows Witness as NOT ready for flight.
namespace maintenance_mode {

// Call once, first thing in app_main: reads and clears the RTC request.
void consume_boot_request();
// True when this boot is a maintenance boot.
bool active();
// Wi-Fi parameters carried across the reboot (0 = defaults).
int requested_tx_dbm();
std::uint32_t requested_minutes();

// Arm a maintenance boot for the next esp_restart(). The caller restarts.
void request(int tx_dbm, std::uint32_t minutes);
// Make sure the next boot is a normal one.
void cancel();

}  // namespace maintenance_mode
