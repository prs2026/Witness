#include "heartbeat.h"

// Set to 0 to disable heartbeat buzzing. GPIO48 remains configured as an
// output and held low. The frequency controls the passive-buzzer PWM tone.
#define HEARTBEAT_BUZZER_ENABLED 1
#define HEARTBEAT_BUZZER_FREQUENCY_HZ 2700

#include <cstdint>

#include "comms.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hardware_pins.h"
#include "spi_data_forwarder.h"
#include "witness_status.h"

namespace {

constexpr char kLogTag[] = "heartbeat";
constexpr gpio_num_t kHeartbeatGpio = HW_PIN_LED_RED;
constexpr gpio_num_t kBuzzerGpio = HW_PIN_BUZZER;
constexpr TickType_t kPulseDuration = pdMS_TO_TICKS(100);
constexpr ledc_mode_t kBuzzerSpeedMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kBuzzerTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kBuzzerChannel = LEDC_CHANNEL_0;
constexpr ledc_timer_bit_t kBuzzerDutyResolution = LEDC_TIMER_10_BIT;

static_assert(
    HEARTBEAT_BUZZER_ENABLED == 0 || HEARTBEAT_BUZZER_ENABLED == 1,
    "HEARTBEAT_BUZZER_ENABLED must be 0 or 1");
static_assert(
    HEARTBEAT_BUZZER_FREQUENCY_HZ >= 100 &&
        HEARTBEAT_BUZZER_FREQUENCY_HZ <= 20'000,
    "Buzzer frequency must be between 100 Hz and 20 kHz");

}  // namespace

Heartbeat::Heartbeat(
    SpiDataForwarder &data_forwarder,
    WitnessStatus &witness_status)
    : data_forwarder_(data_forwarder),
      witness_status_(witness_status)
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
#if HEARTBEAT_BUZZER_ENABLED
        (void)ledc_stop(kBuzzerSpeedMode, kBuzzerChannel, 0);
#endif
        // Never leave the buzzer pin floating on failure.
        (void)gpio_set_direction(kBuzzerGpio, GPIO_MODE_OUTPUT);
        gpio_set_level(kBuzzerGpio, 0);
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
    // Preload the inactive output level before enabling the buzzer GPIO to
    // avoid an active-high pulse during configuration.
    esp_err_t result = gpio_set_level(kBuzzerGpio, 0);
    if (result != ESP_OK) {
        return result;
    }

    gpio_config_t config{};
    config.pin_bit_mask = (1ULL << kHeartbeatGpio);
    config.mode = GPIO_MODE_OUTPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;

    result = gpio_config(&config);
    if (result != ESP_OK) {
        return result;
    }

    result = set_led(false);
    if (result != ESP_OK) {
        return result;
    }

#if HEARTBEAT_BUZZER_ENABLED
    return configure_buzzer_pwm();
#else
    result = gpio_set_direction(kBuzzerGpio, GPIO_MODE_OUTPUT);
    return result == ESP_OK ? gpio_set_level(kBuzzerGpio, 0) : result;
#endif
}

esp_err_t Heartbeat::configure_buzzer_pwm()
{
    ledc_timer_config_t timer{};
    timer.speed_mode = kBuzzerSpeedMode;
    timer.duty_resolution = kBuzzerDutyResolution;
    timer.timer_num = kBuzzerTimer;
    timer.freq_hz = HEARTBEAT_BUZZER_FREQUENCY_HZ;
    timer.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t result = ledc_timer_config(&timer);
    if (result != ESP_OK) {
        return result;
    }

    ledc_channel_config_t channel{};
    channel.gpio_num = kBuzzerGpio;
    channel.speed_mode = kBuzzerSpeedMode;
    channel.channel = kBuzzerChannel;
    channel.timer_sel = kBuzzerTimer;
    channel.duty = 0;
    channel.hpoint = 0;
    result = ledc_channel_config(&channel);
    if (result != ESP_OK) {
        return result;
    }
    return ledc_stop(kBuzzerSpeedMode, kBuzzerChannel, 0);
}

esp_err_t Heartbeat::set_buzzer_pwm(const bool enabled)
{
#if HEARTBEAT_BUZZER_ENABLED
    if (!enabled) {
        return ledc_stop(kBuzzerSpeedMode, kBuzzerChannel, 0);
    }

    esp_err_t result = ledc_set_duty(
        kBuzzerSpeedMode, kBuzzerChannel, kBuzzerDuty);
    if (result == ESP_OK) {
        result = ledc_update_duty(kBuzzerSpeedMode, kBuzzerChannel);
    }
    return result;
#else
    (void)enabled;
    return gpio_set_level(kBuzzerGpio, 0);
#endif
}

void Heartbeat::beep(const std::uint32_t duration_ms)
{
    esp_err_t result = set_buzzer_pwm(true);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "failed to start buzzer PWM on GPIO%d: %s",
                 static_cast<int>(kBuzzerGpio), esp_err_to_name(result));
        (void)set_buzzer_pwm(false);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    result = set_buzzer_pwm(false);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "failed to stop buzzer PWM on GPIO%d: %s",
                 static_cast<int>(kBuzzerGpio), esp_err_to_name(result));
    }
}

void Heartbeat::play_buzzer_pattern()
{
#if !HEARTBEAT_BUZZER_ENABLED
    (void)gpio_set_level(kBuzzerGpio, 0);
    return;
#else
    ++buzzer_cycle_;
    const std::uint8_t flight_state = static_cast<std::uint8_t>(
        witness_status_.flight_state() &
        IRIS_WITNESS_STATUS_FLIGHT_STATE_MASK);

    switch (flight_state) {
    case IRIS_FLIGHT_STATE_PAD_IDLE:
        // One unobtrusive chirp every five seconds.
        if ((buzzer_cycle_ % 5U) == 0U) {
            beep(75);
        }
        break;
    case IRIS_FLIGHT_STATE_BOOST:
        // A long tone every second.
        beep(700);
        break;
    case IRIS_FLIGHT_STATE_COAST:
        // Two short tones.
        beep(100);
        vTaskDelay(pdMS_TO_TICKS(100));
        beep(100);
        break;
    case IRIS_FLIGHT_STATE_DESCENT:
        // Three short tones.
        for (std::uint32_t tone = 0; tone < 3U; ++tone) {
            beep(100);
            if (tone != 2U) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        break;
    case IRIS_FLIGHT_STATE_LANDED:
        // Two long tones every other second.
        if ((buzzer_cycle_ % 2U) == 0U) {
            beep(250);
            vTaskDelay(pdMS_TO_TICKS(150));
            beep(250);
        }
        break;
    default:
        (void)set_buzzer_pwm(false);
        break;
    }
#endif
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
    witness_status_.set(
        IRIS_WITNESS_STATUS_HEARTBEAT_MASK, heartbeat_status_);
    const std::uint32_t uptime_milliseconds =
        static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL);

    std::uint8_t payload[IRIS_PACKET_HEARTBEAT_DATA_LENGTH]{};
    witness_status_.write(
        &payload[IRIS_PACKET_HEARTBEAT_STATUS_OFFSET]);
    payload[IRIS_PACKET_HEARTBEAT_UPTIME_OFFSET] =
        static_cast<std::uint8_t>(uptime_milliseconds >> 24);
    payload[IRIS_PACKET_HEARTBEAT_UPTIME_OFFSET + 1U] =
        static_cast<std::uint8_t>(uptime_milliseconds >> 16);
    payload[IRIS_PACKET_HEARTBEAT_UPTIME_OFFSET + 2U] =
        static_cast<std::uint8_t>(uptime_milliseconds >> 8);
    payload[IRIS_PACKET_HEARTBEAT_UPTIME_OFFSET + 3U] =
        static_cast<std::uint8_t>(uptime_milliseconds);

    // A full forwarding queue drops this heartbeat rather than delaying the
    // GPIO heartbeat task.
    (void)data_forwarder_.queue_packet(
        IRIS_PACKET_ID_HEARTBEAT, payload, sizeof(payload));
}

void Heartbeat::run()
{
    TickType_t next_wake_time = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&next_wake_time, kPeriod);
        queue_heartbeat_packet();
        blink_gpio();
        play_buzzer_pattern();
    }
}
