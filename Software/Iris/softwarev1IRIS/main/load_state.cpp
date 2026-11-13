#include "load_state.h"

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "load_state_logic.h"

namespace load_state {
namespace {
constexpr char kTag[] = "load_state";
RTC_NOINIT_ATTR Record g_record;

ResetClass classify(const esp_reset_reason_t reason)
{
    switch (reason) {
        case ESP_RST_SW:
        case ESP_RST_PANIC:
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT:
        case ESP_RST_BROWNOUT:
        case ESP_RST_CPU_LOCKUP:
            return ResetClass::warm;
        default:
            // POWERON, EXT, USB, DEEPSLEEP, SDIO, UNKNOWN, ...
            return ResetClass::cold;
    }
}
}  // namespace

BootResult begin_boot()
{
    const esp_reset_reason_t reason = esp_reset_reason();
    const bool valid = record_valid(g_record);
    const BootDecision decision = decide_boot(classify(reason), g_record);
    if (decision.restore) {
        ESP_LOGW(kTag, "WARM RESET (reason %d): restoring commanded load mask 0x%02X "
                       "(OUT1=%d OUT2=%d OUT3=%d 5V=%d)",
                 static_cast<int>(reason), decision.mask, decision.mask & 1, (decision.mask >> 1) & 1,
                 (decision.mask >> 2) & 1, (decision.mask >> 3) & 1);
    } else {
        ESP_LOGW(kTag, "cold boot or no valid record (reason %d, record %s): loads default OFF",
                 static_cast<int>(reason), valid ? "valid" : "invalid");
        g_record = make_record(0);
    }
    return BootResult{decision.restore, decision.mask};
}

void store(const std::uint8_t mask)
{
    g_record = make_record(mask);
}

}  // namespace load_state
