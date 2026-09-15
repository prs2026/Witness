#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class SpiDataForwarder;

class Heartbeat final {
public:
    explicit Heartbeat(SpiDataForwarder &data_forwarder);

    Heartbeat(const Heartbeat &) = delete;
    Heartbeat &operator=(const Heartbeat &) = delete;

    esp_err_t start();
    esp_err_t set_led(bool on);

private:
    static constexpr UBaseType_t kTaskPriority = 2;
    static constexpr std::uint32_t kTaskStackSize = 2048;
    static constexpr TickType_t kPeriod = pdMS_TO_TICKS(1000);

    static void task_entry(void *context);
    esp_err_t configure_gpio();
    void blink_gpio();
    void queue_heartbeat_packet();
    void run();

    SpiDataForwarder &data_forwarder_;
    TaskHandle_t task_handle_ = nullptr;
    std::atomic_bool led_on_{false};
    bool heartbeat_status_ = false;
};
