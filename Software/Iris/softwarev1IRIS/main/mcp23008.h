#pragma once

#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"

class Mcp23008 final {
public:
    Mcp23008() = default;

    Mcp23008(const Mcp23008 &) = delete;
    Mcp23008 &operator=(const Mcp23008 &) = delete;

    esp_err_t initialize();
    esp_err_t configure_output(std::uint8_t pin, bool initial_level);
    esp_err_t write_pin(std::uint8_t pin, bool level);

private:
    static constexpr std::uint32_t kBusFrequencyHz = 400000;
    static constexpr int kTransactionTimeoutMs = 20;
    static constexpr std::uint8_t kRegisterIodir = 0x00;
    static constexpr std::uint8_t kRegisterOlat = 0x0A;
    static constexpr std::uint8_t kPinCount = 8;

    esp_err_t write_register(std::uint8_t address, std::uint8_t value);

    i2c_master_bus_handle_t bus_handle_ = nullptr;
    i2c_master_dev_handle_t device_handle_ = nullptr;
    std::uint8_t direction_ = 0xFF;
    std::uint8_t output_latch_ = 0x00;
};

