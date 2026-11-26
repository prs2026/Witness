#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "pressure_altitude.h"

class Ms5607 final {
public:
    // Hardware and measurement configuration.
    static constexpr int kPromRetries = 3;          // reset + PROM read again on a CRC failure (boot)
    static constexpr std::uint32_t kPromRetryDelayMs = 5;   // > 2.8 ms reload time
    // SPI settings tried in order. The flight board of 2026-11-21 read the same
    // PROM words (CRC mismatch) 2,227 times at 10 MHz mode 3; slower clocks
    // and mode 0 rule out sampling-edge and signal-integrity causes, and the
    // first setting whose PROM passes its CRC is kept.
    struct BusConfig {
        int clock_hz;
        std::uint8_t mode;
    };
    static constexpr BusConfig kBusConfigs[] = {
        {10'000'000, 3}, {2'000'000, 3}, {1'000'000, 0}, {250'000, 3}};
    static constexpr int kBusConfigCount = sizeof(kBusConfigs) / sizeof(kBusConfigs[0]);
    static constexpr int kSpiClockFrequencyHz = kBusConfigs[0].clock_hz;
    static constexpr std::uint8_t kSpiMode = kBusConfigs[0].mode;
    static constexpr std::uint8_t kPressureConversionCommand = 0x48;    // OSR 4096
    static constexpr std::uint8_t kTemperatureConversionCommand = 0x58; // OSR 4096
    static constexpr std::uint32_t kConversionTimeUs = 9'100;
    static constexpr std::uint32_t kResetDelayMs = 3;

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

    // boot: every bus setting, each with kPromRetries resets (blocks up to
    // ~80 ms). Otherwise (retry from the sensor task): ONE reset and PROM read
    // with the next bus setting in turn, about 5 ms, so the IMU FIFO is not
    // starved (the old blocking retry held the task ~20 ms every 0.5 s).
    esp_err_t initialize(spi_host_device_t host, bool boot = true);
    // Bus setting in use (index into kBusConfigs), valid after a successful
    // initialize().
    int bus_config_index() const { return config_index_; }
    std::uint32_t prom_failures() const { return prom_failures_; }
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
    esp_err_t try_bus_config(spi_host_device_t host, int index, int retries);
    void remove_device();
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
    int config_index_ = 0;
    int next_config_ = 0;
    std::uint32_t prom_failures_ = 0;
    std::uint16_t logged_prom_[kBusConfigCount][kPromWordCount]{};
};
