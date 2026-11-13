#pragma once

#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"

class Mcp23008 final {
public:
    Mcp23008() = default;

    Mcp23008(const Mcp23008 &) = delete;
    Mcp23008 &operator=(const Mcp23008 &) = delete;

    // pulse_reset=true (cold boot) pulses the expander RESET (GPIO9) low.
    // pulse_reset=false (warm-reset restore) drives GPIO9 high immediately and
    // leaves the expander alone so any output state it still holds survives.
    esp_err_t initialize(bool pulse_reset = true);
    // Writes OLAT=latch then IODIR=~output_mask (with short retries) and
    // caches both on success. Used for boot and for retrying after I2C errors.
    esp_err_t configure_outputs(std::uint8_t output_mask, std::uint8_t latch);
    bool outputs_configured() const { return configured_; }
    bool bus_ready() const { return device_handle_ != nullptr; }
    // Re-pushes the cached OLAT and IODIR to the chip (heals an expander-only
    // reset or a failed earlier write). No-op until configured.
    esp_err_t resync();
    esp_err_t configure_output(std::uint8_t pin, bool initial_level);
    esp_err_t write_pin(std::uint8_t pin, bool level);
    i2c_master_bus_handle_t bus_handle() const { return bus_handle_; }

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
    bool configured_ = false;
};

