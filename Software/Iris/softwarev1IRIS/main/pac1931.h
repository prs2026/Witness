#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "hardware.h"

// Channel 1 of this family is compatible with the PAC1931. The Iris board
// uses a three-channel family member, so the same register interface samples
// channels 1-3.
class Pac1931 final {
public:
    static constexpr std::size_t kChannelCount =
        IRIS_PAC1931_CHANNEL_COUNT;
    using CurrentArray = std::array<std::uint32_t, kChannelCount>;

    Pac1931() = default;
    Pac1931(const Pac1931 &) = delete;
    Pac1931 &operator=(const Pac1931 &) = delete;

    esp_err_t initialize(i2c_master_bus_handle_t bus_handle);
    esp_err_t read_currents_microamps(CurrentArray &currents_microamps);

private:
    static constexpr std::uint32_t kBusFrequencyHz = 400000;
    static constexpr int kTransactionTimeoutMs = 20;
    static constexpr std::uint8_t kRegisterVsense1 = 0x0B;
    static constexpr std::uint8_t kCommandRefreshV = 0x1F;
    static constexpr std::uint32_t kVsenseFullScaleMicrovolts = 100000;

    esp_err_t read_register16(std::uint8_t address, std::uint16_t &value);
    static std::uint32_t raw_to_microamps(std::uint16_t raw);

    i2c_master_dev_handle_t device_handle_ = nullptr;
};
