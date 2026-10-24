#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class SpiDataForwarder;
class FlashLogger;

class UartDriver final {
public:
    // UART task settings. The UART baud rate can be changed here without
    // modifying the implementation.
    static constexpr std::uint32_t kBaudRate = 115200;
    // GPS receivers must not receive the legacy 05 22 startup probe.
    static constexpr bool kStartupLoopbackTestEnabled = false;
    static constexpr UBaseType_t kTaskPriority = 3;
    static constexpr std::uint32_t kTaskStackSize = 4096;
    static constexpr std::uint32_t kTaskTickMs = 2;
    static constexpr std::uint32_t kLoopbackReplyTimeoutMs = 1000;

    UartDriver(SpiDataForwarder &serial_forwarder, uart_port_t port,
               gpio_num_t tx_pin, gpio_num_t rx_pin);
    void set_log_sink(FlashLogger &logger) { logger_ = &logger; }
    ~UartDriver();

    UartDriver(const UartDriver &) = delete;
    UartDriver &operator=(const UartDriver &) = delete;

    esp_err_t start();
    esp_err_t shutdown();

    // Writes the exact bytes supplied to this UART.
    esp_err_t transmit(
        const std::uint8_t *data,
        std::size_t length,
        TickType_t timeout_ticks = 0);

private:
    static constexpr std::size_t kDriverBufferSize = 8192;
    static constexpr std::size_t kReadBufferSize = 64;

    static void task_entry(void *context);
    esp_err_t initialize();
    esp_err_t send_loopback_request();
    void run();

    SpiDataForwarder &serial_forwarder_;
    FlashLogger *logger_ = nullptr;
    uart_port_t port_;
    gpio_num_t tx_pin_;
    gpio_num_t rx_pin_;
    TaskHandle_t task_handle_ = nullptr;
    bool initialized_ = false;
};
