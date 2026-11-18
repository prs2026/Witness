#include "flight_state_store.h"

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

namespace {
constexpr char kLogTag[] = "flight_store";

// Not initialised at boot (survives esp_restart(), panics, watchdog and
// brownout resets); random after power-on, which the magic and checksum
// reject.
RTC_NOINIT_ATTR flight_state_store::Record s_record;
// Pad references (second record: the state record's layout is unchanged).
RTC_NOINIT_ATTR flight_state_store::Extension s_extension;
// Copy of the references taken at boot when a resume happens.
flight_state_store::PadData s_resume_data;
bool s_resume_data_valid = false;
// Guards both records against concurrent writers (sensor task, USB task).
portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

flight_state_store::ResetKind classify(const esp_reset_reason_t reason)
{
    switch (reason) {
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
    case ESP_RST_SW:
    case ESP_RST_BROWNOUT:
        return flight_state_store::ResetKind::Warm;
    default:
        return flight_state_store::ResetKind::Cold;
    }
}
}  // namespace

namespace flight_state_store {

bool take_boot_snapshot(Snapshot &out)
{
    const esp_reset_reason_t reason = esp_reset_reason();
    if (resumable(s_record, classify(reason))) {
        out.state = static_cast<std::uint8_t>(s_record.state);
        out.maximum_altitude_m = s_record.maximum_altitude_m;
        out.maximum_valid = s_record.maximum_valid != 0U;
        if (extension_valid(s_extension)) {
            s_resume_data = unpack_extension(s_extension);
            s_resume_data_valid = true;
        }
        ESP_LOGW(kLogTag,
                 "RESUMING IN-FLIGHT STATE %u after reset reason %d (max %.1f m)",
                 static_cast<unsigned>(out.state), static_cast<int>(reason),
                 static_cast<double>(out.maximum_altitude_m));
        return true;
    }
    portENTER_CRITICAL(&s_lock);
    seal(s_record, 0, 0.0F, false);
    invalidate_extension(s_extension);   // references of an earlier run are not this run's
    portEXIT_CRITICAL(&s_lock);
    return false;
}

void save(const std::uint8_t state, const float maximum_altitude_m,
          const bool maximum_valid)
{
    portENTER_CRITICAL(&s_lock);
    seal(s_record, state, maximum_altitude_m, maximum_valid);
    portEXIT_CRITICAL(&s_lock);
}

void save_references(const PadData &data)
{
    portENTER_CRITICAL(&s_lock);
    seal_extension(s_extension, data);
    portEXIT_CRITICAL(&s_lock);
}

bool resume_references(PadData &out)
{
    if (!s_resume_data_valid) return false;
    out = s_resume_data;
    return true;
}

}  // namespace flight_state_store
