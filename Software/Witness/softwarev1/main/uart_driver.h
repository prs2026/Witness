#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class SpiDataForwarder;

class UartDriver final {
public:
    // UART2 and task settings. The UART baud rate can be changed here without
    // modifying the implementation.
    static constexpr uart_port_t kPort = UART_NUM_2;
    static constexpr std::uint32_t kBaudRate = 115200;
    static constexpr UBaseType_t kTaskPriority = 3;
    static constexpr std::uint32_t kTaskStackSize = 4096;
    static constexpr std::uint32_t kTaskTickMs = 2;
    static constexpr std::uint32_t kLoopbackReplyTimeoutMs = 1000;

    explicit UartDriver(SpiDataForwarder &serial_forwarder);
    ~UartDriver();

    UartDriver(const UartDriver &) = delete;
    UartDriver &operator=(const UartDriver &) = delete;

    esp_err_t start();
    esp_err_t shutdown();

    // Writes the exact bytes supplied to UART2.
    esp_err_t transmit(
        const std::uint8_t *data,
        std::size_t length,
        TickType_t timeout_ticks = 0);

private:
    static constexpr std::size_t kDriverBufferSize = 1024;
    static constexpr std::size_t kReadBufferSize = 64;

    static void task_entry(void *context);
    esp_err_t initialize();
    esp_err_t send_loopback_request();
    void run();

    SpiDataForwarder &serial_forwarder_;
    TaskHandle_t task_handle_ = nullptr;
    bool initialized_ = false;
};
