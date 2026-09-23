#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "comms.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "ms5607.h"
#include "ra01.h"

class SpiDataForwarder;
class WitnessStatus;
class CommandBridge;

class Sensors final {
public:
    // Task configuration. kTaskRateHz must divide evenly into 1000 ms.
    static constexpr UBaseType_t kTaskPriority = 4;
    static constexpr std::uint32_t kTaskRateHz = 100;
    static constexpr std::uint32_t kPacketRateHz = 10;
    static constexpr std::uint32_t kStatePacketRateHz = 1;
    static constexpr std::uint32_t kDebugPacketRateHz = 10;
    static constexpr std::uint32_t kBatterySampleRateHz = 10;
    static constexpr std::uint32_t kTaskStackSize = 4096;

    // GPIO1 uses ADC1 channel 0. The board divider is 15k high / 5k low.
    static constexpr adc_unit_t kBatteryAdcUnit = ADC_UNIT_1;
    static constexpr adc_channel_t kBatteryAdcChannel = ADC_CHANNEL_0;
    static constexpr adc_atten_t kBatteryAdcAttenuation = ADC_ATTEN_DB_12;
    static constexpr adc_bitwidth_t kBatteryAdcBitWidth =
        ADC_BITWIDTH_DEFAULT;
    static constexpr std::uint32_t kBatteryDividerHighOhms = 15'000;
    static constexpr std::uint32_t kBatteryDividerLowOhms = 5'000;
    static constexpr std::uint32_t kBatteryAdcSamples = 16;
    // The canonical 0x02 packet stores FC battery voltage in millivolts.
    static constexpr std::uint32_t kBatteryPacketMillivoltsPerCount =
        IRIS_FIELD_BATTERY_VOLTAGE_MILLIVOLTS_PER_COUNT;

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
        std::int32_t pressure_centi_mbar = 0;
        std::int32_t ms5607_temperature_centi_celsius = 0;
        float barometric_altitude_meters = 0.0F;
        std::uint32_t battery_voltage_mv = 0;
        int battery_adc_raw = 0;
        std::uint64_t timestamp_us = 0;
        bool low_g_accel_ready = false;
        bool high_g_accel_ready = false;
        bool gyro_ready = false;
        bool temperature_ready = false;
        bool ms5607_ready = false;
    };

    Sensors(
        SpiDataForwarder &data_forwarder,
        WitnessStatus &witness_status,
        CommandBridge &command_bridge);

    Sensors(const Sensors &) = delete;
    Sensors &operator=(const Sensors &) = delete;

    esp_err_t start();
    esp_err_t latest_sample(Sample &sample, TickType_t timeout = 0) const;
    esp_err_t handle_command(std::uint16_t command);

private:
    static_assert(kTaskRateHz > 0 && (1000U % kTaskRateHz) == 0,
                  "Sensors task rate must divide evenly into 1000 ms");
    static_assert(kPacketRateHz > 0 && (1000U % kPacketRateHz) == 0,
                  "Sensor packet rate must divide evenly into 1000 ms");
    static_assert(
        kStatePacketRateHz > 0 && (1000U % kStatePacketRateHz) == 0,
        "State packet rate must divide evenly into 1000 ms");
    static_assert(
        kDebugPacketRateHz > 0 &&
            (kTaskRateHz % kDebugPacketRateHz) == 0,
        "Debug packet rate must be an integer divisor of the sensor rate");
    static_assert(
        kBatterySampleRateHz > 0 &&
            (kTaskRateHz % kBatterySampleRateHz) == 0,
        "Battery sample rate must be an integer divisor of the sensor rate");
    static constexpr TickType_t kTaskPeriod =
        pdMS_TO_TICKS(1000U / kTaskRateHz);
    static constexpr TickType_t kPacketPeriod =
        pdMS_TO_TICKS(1000U / kPacketRateHz);
    static constexpr TickType_t kStatePacketPeriod =
        pdMS_TO_TICKS(1000U / kStatePacketRateHz);
    static constexpr TickType_t kDebugPacketPeriod =
        pdMS_TO_TICKS(1000U / kDebugPacketRateHz);
    static constexpr std::uint32_t kBatterySampleDivider =
        kTaskRateHz / kBatterySampleRateHz;
    static constexpr spi_host_device_t kSpiHost = SPI3_HOST;
    static constexpr std::size_t kMaximumRegisterTransfer = 14;
    // Accommodates the SX1262 opcode/offset plus its 255-byte packet buffer.
    static constexpr std::size_t kMaximumSpiTransfer = 258;
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

    // LSM6DSV320X STATUS_REG bits. These are sensor-register bits, not the
    // application-level Witness status byte defined in comms.h.
    static constexpr std::uint8_t kStatusLowGAccelReady = 1U << 0;
    static constexpr std::uint8_t kStatusGyroReady = 1U << 1;
    static constexpr std::uint8_t kStatusTemperatureReady = 1U << 2;
    static constexpr std::uint8_t kStatusHighGAccelReady = 1U << 3;

    static constexpr float kLowGAccelSensitivityMg =
        static_cast<float>(IRIS_FIELD_LSM6_LOW_ACCEL_MICRO_G_PER_COUNT) /
        1000.0F;
    static constexpr float kHighGAccelSensitivityMg =
        static_cast<float>(IRIS_FIELD_LSM6_HIGH_ACCEL_MICRO_G_PER_COUNT) /
        1000.0F;
    static constexpr float kGyroSensitivityMdps = static_cast<float>(
        IRIS_FIELD_LSM6_GYRO_MILLIDEGREES_PER_SECOND_PER_COUNT);

    static void task_entry(void *context);
    static std::int16_t decode_i16(const std::uint8_t *bytes);
    static std::uint16_t encode_battery_voltage(std::uint32_t millivolts);
    esp_err_t initialize_spi();
    esp_err_t initialize_battery_adc();
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
    esp_err_t queue_sensor_packet();
    esp_err_t queue_state_packet();
    esp_err_t queue_debug_packet();
    esp_err_t read_battery_voltage();
    void poll_radio_commands();
    void publish_sample();
    void run();
    void release_resources();

    SpiDataForwarder &data_forwarder_;
    WitnessStatus &witness_status_;
    CommandBridge &command_bridge_;
    Ms5607 ms5607_{};
    Ra01 radio_{};
    spi_device_handle_t spi_device_ = nullptr;
    adc_oneshot_unit_handle_t battery_adc_handle_ = nullptr;
    adc_cali_handle_t battery_adc_calibration_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;
    SemaphoreHandle_t sample_mutex_ = nullptr;
    bool owns_spi_bus_ = false;
    bool sample_available_ = false;
    bool ms5607_initialized_ = false;
    bool radio_ready_ = false;
    std::atomic_bool debug_packet_output_enabled_{false};
    Sample working_sample_{};
    Sample latest_sample_{};
};
