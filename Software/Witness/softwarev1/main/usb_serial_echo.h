#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class UsbSerialEcho final {
public:
    UsbSerialEcho() = default;

    UsbSerialEcho(const UsbSerialEcho &) = delete;
    UsbSerialEcho &operator=(const UsbSerialEcho &) = delete;

    // Installs the built-in USB Serial/JTAG driver and starts the echo task.
    esp_err_t start();

private:
    static constexpr UBaseType_t kTaskPriority = 3;
    static constexpr std::size_t kBufferSize = 512;
    static constexpr std::uint32_t kTaskStackSize = 2048;

    static void task_entry(void *context);
    void run();

    TaskHandle_t task_handle_ = nullptr;
};
