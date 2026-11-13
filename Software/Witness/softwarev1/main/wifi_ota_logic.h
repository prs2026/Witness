#pragma once

#include <cstddef>
#include <cstdint>

// Pure helpers for the Wi-Fi OTA / log download feature (wifi_ota.cpp). They
// have no ESP-IDF dependency so they can be tested on the host
// (tests/wifi_ota_logic_test.cpp).
namespace wifi_ota_logic {

// Flight states (FlightStateMachine::State values) in which Wi-Fi may run.
constexpr std::uint8_t kStatePadIdle = 0x00;
constexpr std::uint8_t kStateLanded = 0x04;
constexpr bool state_allows_wifi(const std::uint8_t flight_state)
{
    return flight_state == kStatePadIdle || flight_state == kStateLanded;
}

// IRIS_COMMAND_WIFI arguments (comms.h): d0 on/off, d1 TX dBm (int8, 0 = keep
// the default), d2 auto-off minutes (0 = default).
constexpr int kMinTxPowerDbm = 2;    // esp_wifi_set_max_tx_power floor (8 units)
constexpr int kMaxTxPowerDbm = 20;   // ceiling (80 units) for this project
constexpr std::uint32_t kMaxAutoOffMinutes = 240;

struct WifiCommand {
    bool valid = false;
    bool on = false;
    int tx_power_dbm = 0;            // resolved, never 0 when valid
    std::uint32_t auto_off_minutes = 0;
};
WifiCommand parse_wifi_command(const std::uint8_t data[3], int default_tx_dbm,
                               std::uint32_t default_auto_off_minutes);
// esp_wifi_set_max_tx_power takes 0.25 dBm units.
constexpr int tx_power_to_wifi_units(const int dbm) { return dbm * 4; }

// "witness-XXXX" from the last two MAC bytes; a non-empty override wins.
// Returns the length written (always NUL terminated), 0 if cap is too small.
std::size_t format_ssid(char *out, std::size_t cap, const std::uint8_t mac[6],
                        const char *override_ssid);

// First bytes of an ESP32-S3 application image: esp_image_header_t (24 bytes),
// the first esp_image_segment_header_t (8), then esp_app_desc_t. The project
// name is 32 bytes at offset 48 of the descriptor.
constexpr std::size_t kImageMagicOffset = 0;
constexpr std::size_t kImageChipIdOffset = 12;
constexpr std::size_t kAppDescOffset = 32;
constexpr std::size_t kAppDescSize = 256;
constexpr std::size_t kAppDescProjectOffset = 48;
constexpr std::size_t kAppDescProjectLength = 32;
constexpr std::size_t kImageHeadLength = kAppDescOffset + kAppDescSize;
constexpr std::uint8_t kImageMagic = 0xE9;
constexpr std::uint16_t kChipIdEsp32S3 = 9;
constexpr std::uint32_t kAppDescMagic = 0xABCD5432U;

enum class ImageCheck : std::uint8_t {
    Ok,
    TooShort,
    BadMagic,
    WrongChip,
    BadDescriptor,
    WrongProject,
};
ImageCheck check_image_head(const std::uint8_t *data, std::size_t length,
                            const char *project_name);
const char *image_check_message(ImageCheck check);

// A log file name from the export table is 8.3 (letters, digits, '_', '-', one
// '.'); anything else is refused before it can reach a lookup.
bool valid_log_name(const char *name);

// Escapes a string for a JSON string literal; returns the length written
// (truncates cleanly if cap is too small).
std::size_t json_escape(char *out, std::size_t cap, const char *in);

}  // namespace wifi_ota_logic
