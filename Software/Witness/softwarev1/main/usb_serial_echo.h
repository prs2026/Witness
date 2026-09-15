#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class Heartbeat;
class SpiDataForwarder;

class UsbSerialEcho final {
public:
    UsbSerialEcho(Heartbeat &heartbeat, SpiDataForwarder &spi_interface);

    UsbSerialEcho(const UsbSerialEcho &) = delete;
    UsbSerialEcho &operator=(const UsbSerialEcho &) = delete;

    // Installs the built-in USB Serial/JTAG driver and starts the echo task.
    esp_err_t start();

private:
    static constexpr UBaseType_t kTaskPriority = 3;
    static constexpr std::size_t kBufferSize = 512;
    static constexpr std::size_t kCommandBufferSize = 256;
    static constexpr std::uint32_t kTaskStackSize = 3072;

    static void task_entry(void *context);
    void echo_bytes(const std::uint8_t *data, std::size_t length);
    void consume_command_bytes(const std::uint8_t *data, std::size_t length);
    void process_command();
    void process_tx_command();
    void set_camera_state(std::size_t camera_index, bool enabled);
    void run();

    Heartbeat &heartbeat_;
    SpiDataForwarder &spi_interface_;
    TaskHandle_t task_handle_ = nullptr;
    char command_buffer_[kCommandBufferSize]{};
    std::size_t command_length_ = 0;
    bool command_overflow_ = false;
    bool camera_enabled_[2]{};
};
