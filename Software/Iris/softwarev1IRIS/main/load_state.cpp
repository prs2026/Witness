#include "load_state.h"

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "load_state_logic.h"
#include "nvs.h"
#include "nvs_flash.h"

namespace load_state {
namespace {
constexpr char kTag[] = "load_state";
// Key stored by the 2026-11-15/16 builds. It is erased once so an old stored
// "on" can never be used by any future build. Never read.
constexpr char kStaleNamespace[] = "iris";
constexpr char kStaleKey[] = "loads";

RTC_NOINIT_ATTR Record g_record;
std::uint8_t g_commanded = 0;
bool g_stale_done = false;
std::int64_t g_next_stale_us = 0;

// Warm = the board never lost power and RTC memory is intact. Everything else
// (power-on, EN/EXT, USB, JTAG, deep sleep, eFuse, glitch, unknown) is cold.
// Brownout is warm: see load_state_logic.h / the 07 audit follow-up.
ResetClass classify(const esp_reset_reason_t reason)
{
    switch (reason) {
        case ESP_RST_SW:
        case ESP_RST_PANIC:
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT:
        case ESP_RST_BROWNOUT:
            return ResetClass::warm;
        default:
            return ResetClass::cold;
    }
}

// Erases the stale key. Returns true when finished (erased or nothing there).
bool erase_stale_key()
{
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Unusable or foreign format: nothing in it can be read back as a
        // valid stored state; start clean.
        (void)nvs_flash_erase();
        result = nvs_flash_init();
    }
    if (result != ESP_OK) {
        ESP_LOGW(kTag, "NVS init failed: %s (stale key not erased yet)", esp_err_to_name(result));
        return false;
    }
    nvs_handle_t handle = 0;
    result = nvs_open(kStaleNamespace, NVS_READWRITE, &handle);
    if (result != ESP_OK) {
        ESP_LOGW(kTag, "NVS open failed: %s", esp_err_to_name(result));
        (void)nvs_flash_deinit();
        return false;
    }
    result = nvs_erase_key(handle, kStaleKey);
    if (result == ESP_OK) {
        result = nvs_commit(handle);
        ESP_LOGW(kTag, "stale NVS load key erased (%s)", esp_err_to_name(result));
    } else if (result == ESP_ERR_NVS_NOT_FOUND) {
        result = ESP_OK;
    } else {
        ESP_LOGW(kTag, "NVS erase key: %s", esp_err_to_name(result));
    }
    nvs_close(handle);
    (void)nvs_flash_deinit();
    return result == ESP_OK;
}

}  // namespace

BootResult begin_boot()
{
    const esp_reset_reason_t reason = esp_reset_reason();
    const ResetClass reset_class = classify(reason);
    const bool rtc_valid = record_valid(g_record);
    const BootDecision decision = decide_boot(reset_class, g_record);

    if (decision.restore) {
        ESP_LOGW(kTag, "warm reset (reason %d), valid RTC record: restoring commanded load mask 0x%02X "
                       "(OUT1=%d OUT2=%d OUT3=%d 5V=%d)",
                 static_cast<int>(reason), decision.mask,
                 decision.mask & 1, (decision.mask >> 1) & 1, (decision.mask >> 2) & 1,
                 (decision.mask >> 3) & 1);
    } else {
        ESP_LOGW(kTag, "reset reason %d (%s), RTC record %s: all loads OFF until commanded",
                 static_cast<int>(reason), reset_class == ResetClass::warm ? "warm" : "cold",
                 rtc_valid ? "valid" : "invalid");
    }

    g_next_stale_us = esp_timer_get_time() + 3000000;  // off the restore path
    g_commanded = decision.mask;
    g_record = record_for_commanded(decision.mask);
    return BootResult{decision.restore, decision.mask};
}

void store(const std::uint8_t mask)
{
    const std::uint8_t masked = mask & kMaskBits;
    g_commanded = masked;
    g_record = record_for_commanded(masked);
}

std::uint8_t commanded_mask()
{
    return g_commanded;
}

void service(const std::int64_t now_us)
{
    if (g_stale_done || now_us < g_next_stale_us) {
        return;
    }
    if (erase_stale_key()) {
        g_stale_done = true;
    } else {
        g_next_stale_us = now_us + 30000000;
    }
}

}  // namespace load_state
