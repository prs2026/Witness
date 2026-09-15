#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class Heartbeat final {
public:
    Heartbeat() = default;

    Heartbeat(const Heartbeat &) = delete;
    Heartbeat &operator=(const Heartbeat &) = delete;

    esp_err_t start();

private:
    static constexpr UBaseType_t kTaskPriority = 2;
    static constexpr std::uint32_t kTaskStackSize = 2048;
    static constexpr TickType_t kPeriod = pdMS_TO_TICKS(1000);

    static void task_entry(void *context);
    esp_err_t configure_gpio();
    void blink_gpio();
    void run();

    TaskHandle_t task_handle_ = nullptr;
};
