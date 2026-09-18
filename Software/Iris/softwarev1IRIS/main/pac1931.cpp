#include "pac1931.h"

#include <cstddef>
#include <cstdint>

#include "esp_rom_sys.h"

esp_err_t Pac1931::initialize(const i2c_master_bus_handle_t bus_handle)
{
    if (bus_handle == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (device_handle_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t device_config{};
    device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device_config.device_address = IRIS_PAC1931_I2C_ADDRESS;
    device_config.scl_speed_hz = kBusFrequencyHz;

    return i2c_master_bus_add_device(
        bus_handle, &device_config, &device_handle_);
}

esp_err_t Pac1931::read_currents_microamps(
    CurrentArray &currents_microamps)
{
    if (device_handle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    // REFRESH_V latches the latest voltage samples without clearing the
    // accumulator. The data sheet specifies up to 1 ms before the result
    // registers are stable.
    const std::uint8_t command = kCommandRefreshV;
    esp_err_t result = i2c_master_transmit(
        device_handle_, &command, sizeof(command), kTransactionTimeoutMs);
    if (result != ESP_OK) {
        return result;
    }
    esp_rom_delay_us(1100);

    CurrentArray new_values{};
    for (std::size_t channel = 0; channel < kChannelCount; ++channel) {
        std::uint16_t raw = 0;
        result = read_register16(
            static_cast<std::uint8_t>(kRegisterVsense1 + channel), raw);
        if (result != ESP_OK) {
            return result;
        }
        new_values[channel] = raw_to_microamps(raw);
    }

    currents_microamps = new_values;
    return ESP_OK;
}

esp_err_t Pac1931::read_register16(
    const std::uint8_t address,
    std::uint16_t &value)
{
    std::uint8_t bytes[2]{};
    const esp_err_t result = i2c_master_transmit_receive(
        device_handle_, &address, sizeof(address), bytes, sizeof(bytes),
        kTransactionTimeoutMs);
    if (result == ESP_OK) {
        value = static_cast<std::uint16_t>(
            (static_cast<std::uint16_t>(bytes[0]) << 8U) | bytes[1]);
    }
    return result;
}

std::uint32_t Pac1931::raw_to_microamps(const std::uint16_t raw)
{
    // Default unipolar VSENSE range is 0-100 mV over 16 bits.
    constexpr std::uint64_t kAdcCounts = 65536ULL;
    constexpr std::uint64_t kMicroampsPerAmp = 1000000ULL;
    constexpr std::uint64_t denominator =
        kAdcCounts * IRIS_PAC1931_SENSE_RESISTOR_MICROOHMS;
    const std::uint64_t numerator =
        static_cast<std::uint64_t>(raw) * kVsenseFullScaleMicrovolts *
        kMicroampsPerAmp;
    const std::uint64_t rounded = (numerator + denominator / 2U) / denominator;
    return static_cast<std::uint32_t>(rounded);
}
