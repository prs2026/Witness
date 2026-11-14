#include "flight_selftest.h"

#include "boot_manager.h"
#include "comms.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sensors.h"
#include "task_cores.h"
#include "witness_status.h"

namespace {
constexpr char kLogTag[] = "selftest";

struct Context {
    const WitnessStatus *status;
    const Sensors *sensors;
};
Context g_context;

bool passed()
{
    const std::uint8_t flags = g_context.status->flags();
    constexpr std::uint8_t kImuReady =
        IRIS_WITNESS_STATUS_LOW_G_ACCEL_READY_MASK | IRIS_WITNESS_STATUS_GYRO_READY_MASK;
    constexpr std::uint8_t kFlashFailed =
        IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK | IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK;
    return (flags & kImuReady) == kImuReady && (flags & kFlashFailed) == 0 && g_context.sensors->radio_ready();
}

std::uint32_t now_ms()
{
    return static_cast<std::uint32_t>(esp_timer_get_time() / 1000);
}

void task(void *)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));
        if (passed()) {
            const esp_err_t result = esp_ota_mark_app_valid_cancel_rollback();
            ESP_LOGI(kLogTag, "self-test passed at %u ms: image marked valid (%s)",
                     static_cast<unsigned>(now_ms()), esp_err_to_name(result));
            boot_manager::note_flight_good();
            break;
        }
        if (now_ms() >= flight_selftest::kWindowMs) {
            ESP_LOGE(kLogTag, "self-test failed (status 0x%02X, radio %s): rolling back",
                     g_context.status->flags(), g_context.sensors->radio_ready() ? "ok" : "not ready");
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_ota_mark_app_invalid_rollback_and_reboot();   // returns only if there is nothing to roll back to
            break;
        }
    }
    vTaskDelete(nullptr);
}
}  // namespace

namespace flight_selftest {

esp_err_t start(const WitnessStatus &witness_status, const Sensors &sensors)
{
    g_context = {&witness_status, &sensors};
    ESP_LOGI(kLogTag, "running flight slot %d (%s)", boot_manager::running_slot(),
             esp_app_get_description()->version);
    if (!boot_manager::running_pending_verify()) {
        boot_manager::note_flight_good();
        return ESP_OK;
    }
    ESP_LOGW(kLogTag, "image is pending verification: self-test within %u s or roll back", kWindowMs / 1000U);
    return xTaskCreatePinnedToCore(task, "selftest", 3072, nullptr, 3, nullptr, task_cores::kSystem) == pdPASS
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}

}  // namespace flight_selftest
