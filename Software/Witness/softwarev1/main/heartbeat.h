#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class SpiDataForwarder;
class WitnessStatus;

class Heartbeat final {
public:
    Heartbeat(
        SpiDataForwarder &data_forwarder,
        WitnessStatus &witness_status);

    Heartbeat(const Heartbeat &) = delete;
    Heartbeat &operator=(const Heartbeat &) = delete;

    esp_err_t start();
    esp_err_t set_led(bool on);

private:
    static constexpr UBaseType_t kTaskPriority = 2;
    static constexpr std::uint32_t kTaskStackSize = 2048;
    static constexpr TickType_t kPeriod = pdMS_TO_TICKS(1000);
    static constexpr std::uint32_t kBuzzerDuty = 512;

    static void task_entry(void *context);
    esp_err_t configure_gpio();
    esp_err_t configure_buzzer_pwm();
    esp_err_t set_buzzer_pwm(bool enabled);
    void beep(std::uint32_t duration_ms);
    void play_buzzer_pattern();
    void blink_gpio();
    void queue_heartbeat_packet();
    void run();

    SpiDataForwarder &data_forwarder_;
    WitnessStatus &witness_status_;
    TaskHandle_t task_handle_ = nullptr;
    std::atomic_bool led_on_{false};
    bool heartbeat_status_ = false;
    std::uint32_t buzzer_cycle_ = 0;
};
