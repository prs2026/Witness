#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "comms.h"

// Shared state and helpers of the MAINTENANCE image (CONFIG_WITNESS_IMAGE_MAINTENANCE).
// The maintenance image has no sensors and no flight logic: it listens on
// LoRa and USB, serves Wi-Fi OTA / log download or a USB log-export disk,
// beacons its status every IRIS_MAINT_BEACON_PERIOD_MS, and restarts into the
// flight image when told to (boot_manager.h restores the flight boot slot
// first thing, so every reset leaves maintenance).
namespace maint {

struct Shared {
    std::atomic<std::uint8_t> mode{IRIS_MAINT_MODE_WIFI};
    std::atomic<std::uint8_t> flags{0};       // IRIS_MAINT_FLAG_*
    std::atomic<std::uint8_t> log_files{0};
    std::atomic<std::uint16_t> wifi_status{0};   // IRIS_COMMAND_WIFI d0=2 word: bit 8 = AP up
    std::uint16_t own_image_id = 0;
    std::uint16_t flight_image_id = 0;
    int flight_slot = -1;                      // flight slot the next boot returns to
    std::uint8_t flight_state = 0;             // flight state when the switch was requested
};
Shared &shared();
void set_flag(std::uint8_t mask, bool on);

// Restart into the flight image after delay_ms (lets an ack/HTTP reply out).
void leave(std::uint32_t delay_ms, const char *reason);
// Extend/replace the Wi-Fi auto-off (minutes; 0 = never) and TX power.
void wifi_keep_alive(std::uint32_t minutes);

// Command frames from LoRa or USB: canonical 10-byte 0x05 frames only.
// radio = arrived over LoRa (the ack goes back over LoRa), else USB.
void handle_command(const std::uint8_t *frame, std::size_t length, bool radio);

// Output helpers (maint_radio.cpp / maint_usb.cpp).
void send_ack(bool radio, std::uint16_t command, const std::uint8_t *data6, std::uint8_t result,
              std::uint16_t detail = 0);

}  // namespace maint
