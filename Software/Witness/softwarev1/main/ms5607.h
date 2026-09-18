#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/spi_master.h"
#include "esp_err.h"

class Ms5607 final {
public:
    // Hardware and measurement configuration.
    static constexpr int kSpiClockFrequencyHz = 10'000'000;
    static constexpr std::uint8_t kSpiMode = 3;
    static constexpr std::uint8_t kPressureConversionCommand = 0x48;    // OSR 4096
    static constexpr std::uint8_t kTemperatureConversionCommand = 0x58; // OSR 4096
    static constexpr std::uint32_t kConversionTimeUs = 9'100;
    static constexpr std::uint32_t kResetDelayMs = 3;
    static constexpr float kSeaLevelPressureMbar = 1013.25F;

    struct Sample {
        std::uint32_t pressure_raw = 0;
        std::uint32_t temperature_raw = 0;
        std::int32_t pressure_centi_mbar = 0;
        std::int32_t temperature_centi_celsius = 0;
        float altitude_meters = 0.0F;
        std::uint64_t timestamp_us = 0;
        bool valid = false;
    };

    Ms5607() = default;
    Ms5607(const Ms5607 &) = delete;
    Ms5607 &operator=(const Ms5607 &) = delete;

    esp_err_t initialize(spi_host_device_t host);
    esp_err_t poll(bool &sample_updated);
    const Sample &sample() const;
    void release();

private:
    enum class ConversionState : std::uint8_t {
        temperature,
        pressure,
    };

    static constexpr std::uint8_t kResetCommand = 0x1E;
    static constexpr std::uint8_t kAdcReadCommand = 0x00;
    static constexpr std::uint8_t kPromReadBaseCommand = 0xA0;
    static constexpr std::size_t kPromWordCount = 8;

    static std::uint8_t calculate_crc4(const std::uint16_t *prom);
    esp_err_t send_command(std::uint8_t command);
    esp_err_t read_command(
        std::uint8_t command,
        std::uint8_t *data,
        std::size_t length);
    esp_err_t read_prom();
    esp_err_t read_adc(std::uint32_t &value);
    esp_err_t start_conversion(
        ConversionState state,
        std::uint8_t command);
    void compensate();

    spi_device_handle_t spi_device_ = nullptr;
    std::uint16_t prom_[kPromWordCount]{};
    std::uint32_t raw_pressure_ = 0;
    std::uint32_t raw_temperature_ = 0;
    std::uint64_t conversion_started_us_ = 0;
    ConversionState conversion_state_ = ConversionState::temperature;
    Sample sample_{};
};
