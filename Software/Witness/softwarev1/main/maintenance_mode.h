#pragma once

#include <cstdint>

// Hand-over of a maintenance request from one image to the other (two-image
// boot, see boot_manager.h and FIRMWARE_HANDOFF.md). The FLIGHT image arms a
// request in RTC memory (survives esp_restart() and panics, not a power cycle)
// right before it selects the maintenance slot and restarts; the MAINTENANCE
// image consumes it as the first thing at boot, and ONLY when that boot's
// reset reason is ESP_RST_SW. A maintenance boot without a valid request (a
// crash, watchdog reset, power loss, or a bootloader rollback landing in the
// maintenance slot) just restores the flight boot slot and restarts into
// flight. The maintenance image also arms a request for its own
// verification boot after a maintenance OTA.
namespace maintenance_mode {

// What the maintenance boot is for.
enum class Kind : std::uint32_t {
    Wifi = 0,        // Wi-Fi AP: OTA and log download
    UsbExport = 1,   // USB mass storage (log export over the USB port)
};

struct Request {
    Kind kind = Kind::Wifi;
    int tx_dbm = 0;                  // Wi-Fi TX power, 0 = default
    std::uint32_t minutes = 0;       // Wi-Fi auto-off, 0 = default
    bool verify = false;             // first boot of a freshly uploaded maintenance image
    int flight_slot = -1;            // flight slot (0/1) to return to, -1 unknown
    std::uint8_t flight_state = 0;   // FlightStateMachine::State when requested (display only)
    // LoRa drive the flight image was using (IRIS_COMMAND_RADIO_POWER), so the
    // maintenance link keeps the range the operator set; radio_dbm 127 = use
    // the boot default.
    int radio_dbm = 127;
    bool radio_front_end = true;};

// Arms a request for the next esp_restart(); honoured only if that boot's
// reset reason is ESP_RST_SW. The caller restarts (see restart_after_ms).
void arm(const Request &request);
// Maintenance image, first thing in app_main: true and fills `out` if a
// valid request was armed for this software restart. Clears the RTC copy
// either way (one boot only).
bool consume(Request &out);
// Make sure the next boot does not carry a request.
void cancel();
// esp_restart() from an esp_timer after delay_ms (lets an ack go out first).
bool restart_after_ms(std::uint32_t delay_ms);

}  // namespace maintenance_mode
