#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

class Sensors final {
public:
    // Task configuration. kTaskRateHz must divide evenly into 1000 ms.
    static constexpr UBaseType_t kTaskPriority = 4;
    static constexpr std::uint32_t kTaskRateHz = 100;
    static constexpr std::uint32_t kTaskStackSize = 4096;

    // LSM6DSV320X SPI and sampling configuration.
    static constexpr int kSpiClockFrequencyHz = 10'000'000;
    static constexpr std::uint8_t kSpiMode = 3;
    static constexpr std::uint8_t kLowGAccelOutputDataRate = 0x06;  // 120 Hz
    static constexpr std::uint8_t kLowGAccelFullScale = 0x03;       // +/-16 g
    static constexpr std::uint8_t kGyroOutputDataRate = 0x06;       // 120 Hz
    static constexpr std::uint8_t kGyroFullScale = 0x04;            // +/-2000 dps
    static constexpr std::uint8_t kHighGAccelOutputDataRate = 0x04; // 960 Hz
    static constexpr std::uint8_t kHighGAccelFullScale = 0x04;      // +/-320 g

    struct Sample {
        std::int16_t low_g_accel_raw[3]{};
        std::int16_t high_g_accel_raw[3]{};
        std::int16_t gyro_raw[3]{};
        std::int16_t temperature_raw = 0;
        float low_g_accel_mg[3]{};
        float high_g_accel_mg[3]{};
        float gyro_mdps[3]{};
        float temperature_celsius = 0.0F;
        std::uint64_t timestamp_us = 0;
        bool low_g_accel_ready = false;
        bool high_g_accel_ready = false;
        bool gyro_ready = false;
        bool temperature_ready = false;
    };

    Sensors() = default;

    Sensors(const Sensors &) = delete;
    Sensors &operator=(const Sensors &) = delete;

    esp_err_t start();
    esp_err_t latest_sample(Sample &sample, TickType_t timeout = 0) const;

private:
    static_assert(kTaskRateHz > 0 && (1000U % kTaskRateHz) == 0,
                  "Sensors task rate must divide evenly into 1000 ms");
    static constexpr TickType_t kTaskPeriod =
        pdMS_TO_TICKS(1000U / kTaskRateHz);
    static constexpr spi_host_device_t kSpiHost = SPI3_HOST;
    static constexpr std::size_t kMaximumRegisterTransfer = 14;
    static constexpr std::uint32_t kBootDelayMs = 35;
    static constexpr std::uint32_t kResetDelayMs = 30;

    // Main-page registers used by the driver.
    static constexpr std::uint8_t kRegFuncCfgAccess = 0x01;
    static constexpr std::uint8_t kRegWhoAmI = 0x0F;
    static constexpr std::uint8_t kRegCtrl1 = 0x10;
    static constexpr std::uint8_t kRegCtrl2 = 0x11;
    static constexpr std::uint8_t kRegCtrl3 = 0x12;
    static constexpr std::uint8_t kRegCtrl6 = 0x15;
    static constexpr std::uint8_t kRegCtrl8 = 0x17;
    static constexpr std::uint8_t kRegStatus = 0x1E;
    static constexpr std::uint8_t kRegOutputStart = 0x20;
    static constexpr std::uint8_t kRegHighGOutputStart = 0x34;
    static constexpr std::uint8_t kRegHighGCtrl1 = 0x4E;

    static constexpr std::uint8_t kExpectedWhoAmI = 0x73;
    static constexpr std::uint8_t kReadCommand = 0x80;
    static constexpr std::uint8_t kSoftwarePowerOnReset = 0x04;
    static constexpr std::uint8_t kCtrl3Configuration = 0x44; // BDU + IF_INC
    static constexpr std::uint8_t kHighGRegisterOutputEnable = 0x80;

    static constexpr std::uint8_t kStatusLowGAccelReady = 1U << 0;
    static constexpr std::uint8_t kStatusGyroReady = 1U << 1;
    static constexpr std::uint8_t kStatusTemperatureReady = 1U << 2;
    static constexpr std::uint8_t kStatusHighGAccelReady = 1U << 3;

    static constexpr float kLowGAccelSensitivityMg = 0.488F;
    static constexpr float kHighGAccelSensitivityMg = 10.417F;
    static constexpr float kGyroSensitivityMdps = 70.0F;

    static void task_entry(void *context);
    static std::int16_t decode_i16(const std::uint8_t *bytes);
    esp_err_t initialize_spi();
    esp_err_t initialize_lsm6dsv320x();
    esp_err_t read_registers(
        std::uint8_t first_register,
        std::uint8_t *data,
        std::size_t length);
    esp_err_t write_registers(
        std::uint8_t first_register,
        const std::uint8_t *data,
        std::size_t length);
    esp_err_t write_register(std::uint8_t register_address, std::uint8_t value);
    esp_err_t fetch_lsm6dsv320x();
    void publish_sample();
    void run();
    void release_resources();

    spi_device_handle_t spi_device_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;
    SemaphoreHandle_t sample_mutex_ = nullptr;
    bool owns_spi_bus_ = false;
    bool sample_available_ = false;
    Sample working_sample_{};
    Sample latest_sample_{};
};
