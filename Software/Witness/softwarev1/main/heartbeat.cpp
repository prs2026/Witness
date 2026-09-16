#include "heartbeat.h"

#include <cstdint>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hardware_pins.h"
#include "spi_data_forwarder.h"

namespace {

constexpr char kLogTag[] = "heartbeat";
constexpr gpio_num_t kHeartbeatGpio = HW_PIN_LED_RED;
constexpr TickType_t kPulseDuration = pdMS_TO_TICKS(100);

}  // namespace

Heartbeat::Heartbeat(SpiDataForwarder &data_forwarder)
    : data_forwarder_(data_forwarder)
{
}

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

esp_err_t Heartbeat::set_led(const bool on)
{
    const esp_err_t result = gpio_set_level(kHeartbeatGpio, on);
    if (result == ESP_OK) {
        led_on_.store(on);
    }
    return result;
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

    return set_led(false);
}

void Heartbeat::blink_gpio()
{
    const bool base_state = led_on_.load();
    esp_err_t result = gpio_set_level(kHeartbeatGpio, !base_state);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "failed to pulse red LED GPIO%d: %s",
                 static_cast<int>(kHeartbeatGpio), esp_err_to_name(result));
        return;
    }

    vTaskDelay(kPulseDuration);

    // Restore the latest commanded state in case it changed during the pulse.
    result = gpio_set_level(kHeartbeatGpio, led_on_.load());
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "failed to restore red LED GPIO%d: %s",
                 static_cast<int>(kHeartbeatGpio), esp_err_to_name(result));
    }
}

void Heartbeat::queue_heartbeat_packet()
{
    heartbeat_status_ = !heartbeat_status_;
    const std::uint32_t uptime_seconds =
        static_cast<std::uint32_t>(esp_timer_get_time() / 1000000ULL);

    const std::uint8_t payload[6] = {
        static_cast<std::uint8_t>(heartbeat_status_ ? 0x01 : 0x00),
        0x00,
        static_cast<std::uint8_t>(uptime_seconds >> 24),
        static_cast<std::uint8_t>(uptime_seconds >> 16),
        static_cast<std::uint8_t>(uptime_seconds >> 8),
        static_cast<std::uint8_t>(uptime_seconds),
    };

    // A full forwarding queue drops this heartbeat rather than delaying the
    // GPIO heartbeat task.
    (void)data_forwarder_.queue_packet(0xFF, payload, sizeof(payload));
}

void Heartbeat::run()
{
    TickType_t next_wake_time = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&next_wake_time, kPeriod);
        queue_heartbeat_packet();
        blink_gpio();
    }
}
