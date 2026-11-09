#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "attitude_estimator.h"
#include "comms.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hitl_input.h"
#include "ms5607.h"
#include "nmea_gps.h"
#include "ra01.h"

class SpiDataForwarder;
class WitnessStatus;
class CommandBridge;
class FlightStateMachine;
class UartDriver;

class Sensors final {
public:
    // Task configuration. kTaskRateHz must divide configTICK_RATE_HZ (1 kHz,
    // the ESP-IDF maximum) so each pass is a whole number of ticks. The IMU
    // read and attitude step run every pass; the estimator steps only when the
    // gyro has a new sample, so its rate is min(kTaskRateHz, gyro ODR).
    static constexpr UBaseType_t kTaskPriority = 4;
    static constexpr std::uint32_t kTaskRateHz = 500;
    static constexpr std::uint32_t kPacketRateHz = 10;
    static constexpr std::uint32_t kStatePacketRateHz = 1;
    static constexpr std::uint32_t kDebugPacketRateHz = 10;
    static constexpr std::uint32_t kBatterySampleRateHz = 10;
    // Float formatting in the attitude log lines needs more than 4 KiB.
    static constexpr std::uint32_t kTaskStackSize = 6144;

    // Independent LoRa downlink rates. Each transmission uses the most recent
    // canonical frame generated at the packet's normal producer/USB rate.
    // Rates must be non-zero integer divisors of 1000 Hz.
    static constexpr std::uint32_t kRadioSensorPacketRateHz = 1;
    static constexpr std::uint32_t kRadioStatePacketRateHz = 1;
    static constexpr std::uint32_t kRadioCameraPacketRateHz = 1;
    static constexpr std::uint32_t kRadioCommandPacketRateHz = 1;
    static constexpr std::uint32_t kRadioHeartbeatPacketRateHz = 1;
    static constexpr std::uint32_t kRadioWitnessDebugPacketRateHz = 1;
    static constexpr std::uint32_t kRadioIrisDebugPacketRateHz = 1;
    static constexpr std::size_t kRadioPacketTypeCount = 7;

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
    // On-chip filtering, per datasheet DS14623 Rev 3 (LSM6DSV320X).
    // Low-g accelerometer: 960 Hz internal rate with LPF2 at ODR/45 = 21.3 Hz,
    // read at 100 Hz, so it is band-limited below the 50 Hz Nyquist rate of
    // the read loop (true anti-aliasing; LPF1 alone is ODR/2).
    //   CTRL1 (10h) ODR_XL = 1001 -> 960 Hz, OP_MODE_XL = 000 high performance.
    //   CTRL8 (17h) HP_LPF2_XL_BW = 011 -> ODR/45 (Table 71), FS_XL = 11
    //               -> +/-16 g. Bits 4:2 must be 0 (Table 69).
    //   CTRL9 (18h) LPF2_XL_EN = 1 (bit 3), low-pass path (HP_SLOPE_XL_EN = 0);
    //               other bits 0 (Table 72).
    // Gyroscope: 120 Hz, LPF1 OFF (its fixed LPF2 is 49.4 Hz at 120 Hz,
    // Table 23).
    //   CTRL2 (11h) ODR_G = 0110 -> 120 Hz, OP_MODE_G = 000 high performance.
    //   CTRL6 (15h) bit 3 = 1 (required by Table 64; previously written 0),
    //               FS_G = 100 -> +/-2000 dps, LPF1_G_BW unused, bit 7 = 0.
    //   CTRL7 (16h) LPF1_G_EN = 0.
    // Gyro LPF1 was tested on the bench 2026-11-08 with the LoRa radio on, in
    // the same session: 120 Hz + LPF1 code 110 (24.2 Hz, Table 66) raised z
    // noise from 76 to 114 dps sd and the mean offset from -13 to -38 dps;
    // 960 Hz + LPF1 code 110 (28.7 Hz) raised it to 177 dps sd and -60 dps,
    // and the attitude estimator never initialized. The interference couples
    // into the gyro's analog front end, so on-chip digital filtering
    // rectifies it into bias instead of removing it. The estimator's running
    // median handles the isolated spikes instead. Revisit LPF1 once the
    // board-level radio coupling is fixed.
    static constexpr std::uint8_t kLowGAccelOutputDataRate = 0x09;      // 960 Hz
    static constexpr std::uint8_t kLowGAccelFullScale = 0x63;           // LPF2 ODR/45, +/-16 g
    static constexpr std::uint8_t kLowGAccelFilterConfiguration = 0x08; // CTRL9 LPF2_XL_EN
    // Gyro for the 500 Hz estimator: 960 Hz ODR so every 2 ms pass has a fresh
    // sample, LPF1 code 100 -> 100 Hz LPF1+LPF2 bandwidth at 960 Hz (Table 66),
    // below the 250 Hz Nyquist rate of a 500 Hz read.
    static constexpr std::uint8_t kGyroOutputDataRate = 0x09;           // 960 Hz
    static constexpr std::uint8_t kGyroFullScale = 0x4C;                // LPF1 100, bit 3, +/-2000 dps
    static constexpr std::uint8_t kGyroFilterConfiguration = 0x01;      // CTRL7: LPF1 on
    static constexpr std::uint8_t kHighGAccelOutputDataRate = 0x04; // 960 Hz
    static constexpr std::uint8_t kHighGAccelFullScale = 0x04;      // +/-320 g

    // Register facts checked 2026-11-08 against ST's official driver header
    // github.com/STMicroelectronics/lsm6dsv320x-pid (master, lsm6dsv320x_reg.h):
    //   CTRL1 0x10 odr_xl[3:0] op_mode_xl[6:4]; CTRL2 0x11 odr_g[3:0]
    //   op_mode_g[6:4]; ODR code 0x6 = 120 Hz, 0x7 = 240, 0x8 = 480, 0x9 = 960.
    //   CTRL6 0x15 fs_g[2:0] (0x4 = 2000 dps), lpf1_g_bw[6:4].
    //   CTRL7 0x16 lpf1_g_en bit 0.
    //   CTRL8 0x17 fs_xl[1:0] (0x3 = 16 g), hp_lpf2_xl_bw[7:5].
    //   CTRL9 0x18 lpf2_xl_en bit 3, hp_slope_xl_en bit 4.
    //   CTRL1_XL_HG 0x4E fs_xl_hg[2:0] (0x4 = 320 g), odr_xl_hg[5:3]
    //   (0x4 = 960 Hz), xl_hg_regout_en bit 7.
    // The driver header fields agree with the datasheet tables cited above.

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
        CommandBridge &command_bridge,
        FlightStateMachine &flight_state_machine,
        UartDriver &gps_uart);

    Sensors(const Sensors &) = delete;
    Sensors &operator=(const Sensors &) = delete;

    esp_err_t start();
    esp_err_t latest_sample(Sample &sample, TickType_t timeout = 0) const;
    esp_err_t handle_command(std::uint16_t command);
    void set_radio_transmission_enabled(bool enabled);

    // HITL mode (see hitl_input.h). Thread-safe; called from the USB command
    // task. Entry requires USB and Pad Idle; every outcome is logged.
    esp_err_t enter_hitl(hitl::Source source);
    esp_err_t exit_hitl(hitl::Source source);
    // Accepts a decoded 0x0C sample while HITL is active; false otherwise.
    bool submit_hitl_sample(const hitl::Sample &sample);
    bool hitl_active() const;

private:
    static_assert(kTaskRateHz > 0 && (1000U % kTaskRateHz) == 0,
                  "Sensors task rate must divide evenly into 1000 ms");
    // The IMU/attitude loop is a whole number of scheduler ticks
    // (2 ticks for 500 Hz at the 1 kHz CONFIG_FREERTOS_HZ).
    static_assert((configTICK_RATE_HZ % kTaskRateHz) == 0,
                  "Scheduler tick rate must be a multiple of the sensor rate");
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
        static_cast<TickType_t>(configTICK_RATE_HZ / kTaskRateHz);
    static_assert(kTaskPeriod >= 1, "Sensor loop cannot run faster than the tick");
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
    static constexpr std::uint32_t kHitlSummaryIntervalMs = 10'000;
    static constexpr std::uint32_t kAttitudeReportIntervalMs = 10'000;

    // Main-page registers used by the driver.
    static constexpr std::uint8_t kRegFuncCfgAccess = 0x01;
    static constexpr std::uint8_t kRegWhoAmI = 0x0F;
    static constexpr std::uint8_t kRegCtrl1 = 0x10;
    static constexpr std::uint8_t kRegCtrl2 = 0x11;
    static constexpr std::uint8_t kRegCtrl3 = 0x12;
    static constexpr std::uint8_t kRegCtrl6 = 0x15;
    static constexpr std::uint8_t kRegCtrl7 = 0x16;
    static constexpr std::uint8_t kRegCtrl8 = 0x17;
    static constexpr std::uint8_t kRegCtrl9 = 0x18;
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
    void service_radio_transmit(TickType_t now);
    // Completes a non-blocking radio transmission; true while one is in flight.
    bool service_radio_transmit_completion();
    // Feeds the attitude estimator with the current IMU sample.
    void update_attitude(std::uint64_t timestamp_us);
    void report_attitude_rate(std::uint64_t now_us);
    void publish_sample();
    // Applies HITL enter/exit/timeout and any new sample to working_sample_.
    // Returns true while HITL replaces the real IMU, barometer and battery.
    bool service_hitl(std::uint32_t now_ms, bool &new_sample);
    void apply_hitl_sample();
    void reset_sensor_sample();
    // UART GPS fix, or the simulated fix while HITL is active.
    NmeaGps::Fix current_gps_fix() const;
    void run();
    void release_resources();

    SpiDataForwarder &data_forwarder_;
    WitnessStatus &witness_status_;
    CommandBridge &command_bridge_;
    FlightStateMachine &flight_state_machine_;
    UartDriver &gps_uart_;
    Ms5607 ms5607_{};
    Ra01 radio_{};
    spi_device_handle_t spi_device_ = nullptr;
    adc_oneshot_unit_handle_t battery_adc_handle_ = nullptr;
    adc_cali_handle_t battery_adc_calibration_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;
    SemaphoreHandle_t sample_mutex_ = nullptr;
    bool owns_spi_bus_ = false;
    bool sample_available_ = false;
    bool has_low_g_accel_sample_ = false;
    bool has_high_g_accel_sample_ = false;
    bool has_gyro_sample_ = false;
    bool ms5607_initialized_ = false;
    bool radio_ready_ = false;
    std::array<TickType_t, kRadioPacketTypeCount> radio_last_sent_{};
    std::size_t next_radio_packet_index_ = 0;
    std::atomic_bool radio_transmission_enabled_{true};
    std::atomic_bool debug_packet_output_enabled_{false};
    Sample working_sample_{};
    Sample latest_sample_{};

    // hitl_controller_ is shared with the USB task under hitl_lock_. The
    // remaining HITL members belong to the sensor task.
    mutable portMUX_TYPE hitl_lock_ = portMUX_INITIALIZER_UNLOCKED;
    hitl::Controller hitl_controller_{};
    hitl::Sample hitl_sample_{};
    bool hitl_in_use_ = false;
    bool hitl_has_sample_ = false;
    std::uint32_t hitl_session_in_use_ = 0;
    std::uint32_t hitl_samples_since_summary_ = 0;
    std::uint32_t hitl_last_summary_ms_ = 0;

    // Attitude estimator and 100 Hz loop diagnostics (sensor task only).
    // Estimator timing scales with the loop: nominal step 1/kTaskRateHz,
    // "late" at 1.5x, and a stationary window needs at least half its samples.
    static attitude::Config attitude_config()
    {
        attitude::Config config{};
        config.nominal_dt_us = 1'000'000U / kTaskRateHz;
        config.late_dt_us = config.nominal_dt_us + config.nominal_dt_us / 2U;
        config.window_min_samples = kTaskRateHz / 2U;
        return config;
    }
    static_assert(kTaskRateHz <= attitude::Estimator::kMaxWindowSamples,
                  "A 1 s stationary window must fit the estimator's buffer");
    attitude::Estimator attitude_{attitude_config()};
    std::uint32_t attitude_updates_since_report_ = 0;
    std::uint64_t attitude_report_start_us_ = 0;
    std::uint32_t loop_overruns_ = 0;
    std::uint32_t max_pass_us_ = 0;
    std::uint32_t radio_tx_started_ = 0;
    std::uint32_t radio_tx_failed_ = 0;
};
