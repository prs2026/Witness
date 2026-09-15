#include "heartbeat.h"

#include <cinttypes>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_log.h"

namespace {

constexpr char kLogTag[] = "heartbeat";
constexpr gpio_num_t kHeartbeatGpio = GPIO_NUM_13;
constexpr TickType_t kPulseDuration = pdMS_TO_TICKS(100);

}  // namespace

esp_err_t Heartbeat::start()
{
    if (task_handle_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    const esp_err_t gpio_result = configure_gpio();
    if (gpio_result != ESP_OK) {
        return gpio_result;
    }

    const BaseType_t task_created = xTaskCreate(
        task_entry,
        "heartbeat",
        kTaskStackSize,
        this,
        kTaskPriority,
        &task_handle_);

    if (task_created != pdPASS) {
        task_handle_ = nullptr;
        gpio_reset_pin(kHeartbeatGpio);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void Heartbeat::task_entry(void *context)
{
    static_cast<Heartbeat *>(context)->run();
}

esp_err_t Heartbeat::configure_gpio()
{
    gpio_config_t config{};
    config.pin_bit_mask = 1ULL << kHeartbeatGpio;
    config.mode = GPIO_MODE_OUTPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;

    const esp_err_t result = gpio_config(&config);
    if (result != ESP_OK) {
        return result;
    }

    return gpio_set_level(kHeartbeatGpio, 0);
}

void Heartbeat::blink_gpio()
{
    esp_err_t result = gpio_set_level(kHeartbeatGpio, 1);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "failed to set GPIO13 high: %s", esp_err_to_name(result));
        return;
    }

    vTaskDelay(kPulseDuration);

    result = gpio_set_level(kHeartbeatGpio, 0);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "failed to set GPIO13 low: %s", esp_err_to_name(result));
    }
}

void Heartbeat::run()
{
    std::uint32_t count = 0;
    TickType_t next_wake_time = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&next_wake_time, kPeriod);
        ESP_LOGI(kLogTag, "count=%" PRIu32, ++count);
        blink_gpio();
    }
}
