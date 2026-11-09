#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "attitude_estimator.h"
#include "accel_filter.h"
#include "imu_calibration.h"
#include "position_filter.h"
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
class FlashLogger;
class WitnessStatus;
class CommandBridge;
class FlightStateMachine;
class UartDriver;

class Sensors final {
public:
    // Task configuration. kTaskRateHz must divide configTICK_RATE_HZ (1 kHz,
    // the ESP-IDF maximum) so each pass is a whole number of ticks. Each pass
    // drains the LSM6 FIFO, which the sensor fills at kImuSampleRateHz with
    // its own clock, so the estimator and the NAND IMU log step once per
    // FIFO sample (960 Hz) regardless of loop jitter.
    static constexpr UBaseType_t kTaskPriority = 4;
    static constexpr std::uint32_t kTaskRateHz = 500;
    // IMU sample rate: the FIFO batch data rate (BDR) for gyro, low-g and
    // high-g. 960 Hz is an exact 1/8 of the 7.68 kHz ODR, so every batched
    // sample is one filtered output sample; 1 kHz would need HAODR (8 kHz ODR,
    // different filter tables) or software polling with tick jitter.
    static constexpr std::uint32_t kImuSampleRateHz = 960;
    // Filtered-acceleration low-pass cutoff (2nd-order Butterworth on the
    // 960 Hz stream; about 2.5 ms group delay). 0 disables the filter.
    static constexpr float kFilteredAccelCutoffHz = 100.0F;
    // Position Kalman filter (position_filter.h): predict on every IMU
    // sample, update with the newest baro reading and any new GPS fix at this
    // rate. 25 Hz suits both GPS options: a 25 Hz receiver (PX1120S) gets
    // every fix used, a 10 Hz one (u-blox M8) appears on every 2nd-3rd tick.
    static constexpr std::uint32_t kPositionUpdateRateHz = 25;
    // State packets are logged to NAND at 50 Hz, the barometer's rate (USB and
    // radio keep 1 Hz).
    static constexpr std::uint32_t kStateLogRateHz = 50;
    static constexpr std::uint32_t kPacketRateHz = 10;
    static constexpr std::uint32_t kStatePacketRateHz = 1;
    static constexpr std::uint32_t kDebugPacketRateHz = 10;
    static constexpr std::uint32_t kBatterySampleRateHz = 10;
    // Float formatting in the attitude log lines needs more than 4 KiB; the
    // rest is headroom for the planned Kalman filter on this core-1 task
    // (about 4 KiB was free at 6 KiB with the 500 Hz loop).
    static constexpr std::uint32_t kTaskStackSize = 8192;

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
    // Both UI sensors run at the maximum 7.68 kHz ODR (high-performance mode),
    // are low-pass filtered on chip, and are batched into the FIFO at 960 Hz
    // (FIFO_CTRL3 BDR), i.e. every 8th filtered sample. The filters are set
    // below the 480 Hz Nyquist rate of that 960 Hz stream.
    // Low-g accelerometer:
    //   CTRL1 (10h) ODR_XL = 1100 -> 7.68 kHz, OP_MODE_XL = 000 high perf.
    //   CTRL8 (17h) HP_LPF2_XL_BW = 010 -> ODR/20 = 384 Hz (Table 71),
    //               FS_XL = 11 -> +/-16 g. Bits 4:2 must be 0 (Table 69).
    //   CTRL9 (18h) LPF2_XL_EN = 1 (bit 3), low-pass path (HP_SLOPE_XL_EN = 0);
    //               other bits 0 (Table 72). (ODR/10 would be 768 Hz.)
    // Gyroscope:
    //   CTRL2 (11h) ODR_G = 1100 -> 7.68 kHz, OP_MODE_G = 000 high perf.
    //   CTRL6 (15h) LPF1_G_BW = 011 -> LPF1+LPF2 407 Hz at 7.68 kHz
    //               (Table 66), bit 3 = 1 (Table 64), FS_G = 100 -> 2000 dps.
    //   CTRL7 (16h) LPF1_G_EN = 1.
    //   Without LPF1 the fixed LPF2 alone is 537 Hz at 7.68 kHz (Table 23),
    //   above the 480 Hz Nyquist rate, so it would alias; no LPF1 code gives
    //   ~532 Hz (7.68 kHz column: 281/213/156/407/102/58/28.8/14.4 Hz).
    //   To run LPF2-only (537 Hz), set kGyroFilterConfiguration to 0x00.
    // High-g accelerometer: 960 Hz ODR. It has no selectable digital filter,
    // so it runs at the sample rate instead of being decimated from 7.68 kHz.
    // Earlier gyro LPF1 note (960 Hz ODR, 2026-11-08): LPF1 was tested on the bench 2026-11-08 with the LoRa radio on, in
    // the same session: 120 Hz + LPF1 code 110 (24.2 Hz, Table 66) raised z
    // noise from 76 to 114 dps sd and the mean offset from -13 to -38 dps;
    // 960 Hz + LPF1 code 110 (28.7 Hz) raised it to 177 dps sd and -60 dps,
    // and the attitude estimator never initialized. The interference couples
    // into the gyro's analog front end, so on-chip digital filtering
    // rectifies it into bias instead of removing it. The estimator's running
    // median handles the isolated spikes instead. Revisit LPF1 once the
    // board-level radio coupling is fixed.
    static constexpr std::uint8_t kLowGAccelOutputDataRate = 0x0C;      // 7.68 kHz
    static constexpr std::uint8_t kLowGAccelFullScale = 0x43;           // LPF2 ODR/20, +/-16 g
    static constexpr std::uint8_t kLowGAccelFilterConfiguration = 0x08; // CTRL9 LPF2_XL_EN
    static constexpr std::uint8_t kGyroOutputDataRate = 0x0C;           // 7.68 kHz
    static constexpr std::uint8_t kGyroFullScale = 0x3C;                // LPF1 011 (407 Hz), bit 3, +/-2000 dps
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
        // Filtered acceleration (accel_filter.h): low-g, or high-g while the
        // low-g is near saturation, through a 100 Hz low-pass. mg.
        float filtered_accel_mg[3]{};
        bool high_g_accel_active = false;
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
    // NAND IMU stream: one record per fresh IMU sample. Set before start().
    void set_imu_log_sink(FlashLogger *logger) { imu_log_sink_ = logger; }
    esp_err_t latest_sample(Sample &sample, TickType_t timeout = 0) const;
    esp_err_t handle_command(std::uint16_t command);
    void set_radio_transmission_enabled(bool enabled);

    // HITL mode (see hitl_input.h). Thread-safe; called from the USB command
    // task. Entry requires USB and Pad Idle; every outcome is logged.
    esp_err_t enter_hitl(hitl::Source source);
    esp_err_t exit_hitl(hitl::Source source);
    // Accepts a decoded 0x0C sample while HITL is active; false otherwise.
    bool submit_hitl_sample(const hitl::Sample &sample);
    // Accepts eight 960 Hz IMU samples (0x0E) while HITL is active. While
    // these keep arriving, they replace the 0x0C IMU fields: each sample runs
    // through the same path as a real FIFO sample (filtered accel, attitude,
    // position predict, NAND IMU log), without calibration.
    bool submit_hitl_imu_batch(const hitl::ImuBatch &batch);
    bool hitl_active() const;

    // IMU calibration command from USB (argument: IRIS_IMU_CALIBRATION_*).
    // Thread-safe; the capture runs in the sensor task and logs its result.
    esp_err_t request_calibration(std::uint8_t argument);

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
    static constexpr std::uint8_t kRegFifoCtrl3 = 0x09;
    static constexpr std::uint8_t kRegFifoCtrl4 = 0x0A;
    static constexpr std::uint8_t kRegCounterBdr1 = 0x0B;
    static constexpr std::uint8_t kRegFifoStatus1 = 0x1B;
    static constexpr std::uint8_t kRegFifoDataOutTag = 0x78;
    static constexpr std::uint8_t kRegInternalFreqFine = 0x4F;

    // FIFO (datasheet section 6.5, registers 9.6-9.10, 9.25-9.26, 9.90).
    static constexpr std::uint8_t kFifoBatchRates = 0x99;   // BDR_GY = BDR_XL = 960 Hz
    static constexpr std::uint8_t kFifoContinuous = 0x06;   // FIFO_MODE 110, no temp/ts
    static constexpr std::uint8_t kFifoBypass = 0x00;       // clears the FIFO
    static constexpr std::uint8_t kFifoHighGBatch = 0x08;   // XL_HG_BATCH_EN
    static constexpr std::uint8_t kFifoStatusDiff8 = 0x01;  // FIFO_STATUS2 DIFF_FIFO_8
    static constexpr std::uint8_t kFifoStatusOverrun = 0x08; // FIFO_OVR_LATCHED
    static constexpr std::uint8_t kFifoTagGyro = 0x01;
    static constexpr std::uint8_t kFifoTagLowG = 0x02;
    static constexpr std::uint8_t kFifoTagHighG = 0x1D;
    static constexpr std::size_t kFifoWordSize = 7;         // tag + 3 x int16
    // Words per SPI burst (tag-address rollover with IF_INC), bounded by the
    // bus max_transfer_sz. 36 words = 12 samples = 12.5 ms of data.
    static constexpr std::size_t kFifoWordsPerBurst = 36;
    // Samples one pass can hand to the estimator; the rest stay in the FIFO
    // (1.5 KB, about 73 samples or 76 ms) for the next pass.
    static constexpr std::size_t kMaxImuSamplesPerPass = 48;
    static constexpr std::uint32_t kTemperatureDivider = kTaskRateHz / 10U;
    static_assert(kFifoWordsPerBurst * kFifoWordSize + 1 <= kMaximumSpiTransfer,
                  "FIFO burst must fit the SPI transfer limit");

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
    // One 960 Hz FIFO time slot: gyro, low-g and high-g words that share a
    // TAG_CNT. Missing channels keep the previous value (fresh flag clear).
    struct ImuSample {
        std::int16_t gyro[3]{};
        std::int16_t low_g[3]{};
        std::int16_t high_g[3]{};
        bool has_gyro = false;
        bool has_low_g = false;
        bool has_high_g = false;
        std::uint8_t tag_count = 0;
        std::uint64_t timestamp_us = 0;
    };
    esp_err_t fetch_lsm6dsv320x();
    esp_err_t read_fifo_words(std::size_t words);
    void push_fifo_word(const std::uint8_t *word);
    void finish_fifo_slot();
    // feed_capture: count this sample toward a running calibration capture
    // (once per FIFO sample, not for the per-pass copy of the newest one).
    void apply_imu_sample(const ImuSample &sample, bool feed_capture = false,
                          bool calibrate = true);
    // Every FIFO sample of this pass, oldest first: calibration, filtered
    // acceleration, NAND IMU log and attitude. working_sample_ ends on the
    // newest sample, before the flight state machine and packets use it.
    void process_imu_batch(bool hitl = false);
    // HITL 0x0E: drains queued 960 Hz samples into imu_batch_ (sensor task).
    std::size_t take_hitl_imu_samples(std::uint64_t now_us);
    // Position filter: predict from the filtered acceleration rotated to ENU
    // (needs a valid attitude), and the 25 Hz baro/GPS update.
    void predict_position(std::uint64_t timestamp_us);
    void service_position(std::uint64_t now_us);
    void report_position(std::uint64_t now_us);
    // Sensor task: starts/finishes calibration captures and applies requests.
    void service_calibration();
    void finish_calibration_capture();
    void log_calibration(const char *prefix) const;
    // Telemetry IMU values in wire counts: the stored calibration applied,
    // and for the gyro also the estimator's live pad bias (which tracks
    // warm-up drift; it freezes at launch). Raw counts stay in the NAND log.
    void output_counts(std::int16_t gyro[3], std::int16_t low_g[3],
                       std::int16_t high_g[3]) const;
    static std::int16_t to_counts(float value, float units_per_count);
    esp_err_t queue_sensor_packet();
    esp_err_t queue_state_packet();
    void build_state_payload(std::uint8_t *payload);
    // NAND packet stream: the full 0x02 frame every kStateLogPeriod.
    void log_state_packet();
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
    // 0x0E samples, filled by the USB task under hitl_lock_.
    struct HitlImuSlot {
        std::uint32_t index;
        std::int16_t low_g[3];
        std::int16_t high_g[3];
        std::int16_t gyro[3];
    };
    static constexpr std::size_t kHitlImuQueueLength = 256;  // ~0.27 s
    std::array<HitlImuSlot, kHitlImuQueueLength> hitl_imu_queue_{};
    std::size_t hitl_imu_head_ = 0;
    std::size_t hitl_imu_count_ = 0;
    std::uint32_t hitl_imu_overflows_ = 0;
    std::uint32_t hitl_imu_last_ms_ = 0;
    // Sensor task: sample clock for 0x0E (index -> timestamp).
    bool hitl_imu_streaming_ = false;
    bool hitl_imu_epoch_valid_ = false;
    std::uint64_t hitl_imu_epoch_us_ = 0;
    std::uint32_t hitl_imu_epoch_index_ = 0;
    std::uint32_t hitl_imu_next_index_ = 0;
    std::uint32_t hitl_imu_gaps_ = 0;
    bool hitl_in_use_ = false;
    bool hitl_has_sample_ = false;
    std::uint32_t hitl_session_in_use_ = 0;
    std::uint32_t hitl_samples_since_summary_ = 0;
    std::uint32_t hitl_last_summary_ms_ = 0;

    // Attitude estimator and 100 Hz loop diagnostics (sensor task only).
    // Estimator timing scales with the IMU rate: nominal step 1/kImuSampleRateHz,
    // "late" at 1.5x, and a stationary window needs at least half its samples.
    static attitude::Config attitude_config()
    {
        attitude::Config config{};
        config.nominal_dt_us = 1'000'000U / kImuSampleRateHz;
        config.late_dt_us = config.nominal_dt_us + config.nominal_dt_us / 2U;
        config.window_min_samples = kImuSampleRateHz / 2U;
        return config;
    }
    static_assert(kImuSampleRateHz <= attitude::Estimator::kMaxWindowSamples,
                  "A 1 s stationary window must fit the estimator's buffer");

    attitude::Estimator attitude_{attitude_config()};
    FlashLogger *imu_log_sink_ = nullptr;
    void log_imu_sample(std::uint64_t timestamp_us, bool hitl);

    // FIFO drain state (sensor task only). DMA-capable SPI buffers.
    std::uint8_t *fifo_tx_ = nullptr;
    std::uint8_t *fifo_rx_ = nullptr;
    ImuSample fifo_slot_{};
    bool fifo_slot_open_ = false;
    ImuSample last_imu_{};
    std::array<ImuSample, kMaxImuSamplesPerPass> imu_batch_{};
    std::size_t imu_batch_count_ = 0;
    std::uint32_t temperature_counter_ = 0;
    std::uint32_t fifo_samples_ = 0;
    std::uint32_t fifo_overruns_ = 0;
    std::uint32_t fifo_bad_words_ = 0;
    std::uint32_t fifo_partial_slots_ = 0;
    std::uint32_t fifo_max_words_ = 0;
    // Core-1 time per report interval: FIFO read and per-sample processing.
    std::uint64_t imu_fetch_us_ = 0;
    std::uint64_t imu_process_us_ = 0;
    // Sample clock. The sensor ODR differs from nominal by
    // 0.13% x INTERNAL_FREQ_FINE (this part: about -2.5%), so samples are
    // spaced at the measured period and the clock is steered slowly toward
    // esp_timer read times, keeping dt smooth while its sum tracks real time.
    double imu_period_us_ = 1e6 / static_cast<double>(kImuSampleRateHz);
    double imu_clock_us_ = 0.0;
    bool imu_clock_valid_ = false;
    std::int8_t imu_freq_fine_ = 0;

    // IMU calibration (sensor task, except the request atomic). Corrections
    // apply to engineering units for the estimator, packets and flight state
    // machine; the NAND IMU log keeps raw counts.
    static constexpr std::uint32_t kGyroCalibrationSamples = 4U * kImuSampleRateHz;
    static constexpr std::uint32_t kFaceCalibrationSamples = 2U * kImuSampleRateHz;
    enum class CalibrationCapture : std::uint8_t { None, Gyro, Face };
    std::atomic<std::uint8_t> calibration_request_{0};
    imu_calibration::Calibration calibration_{};
    accel_filter::FilteredAccel filtered_accel_{};
    position::Filter position_filter_{};
    std::uint64_t position_last_predict_us_ = 0;
    std::uint64_t position_last_update_us_ = 0;
    std::uint64_t position_report_us_ = 0;
    std::uint32_t position_last_gps_ms_ = 0;
    bool position_baro_pending_ = false;
    std::uint64_t state_log_last_us_ = 0;

    // 10 s status lines are formatted here but printed by a low-priority
    // core-0 task: pushing them through the ESP log (USB console and NAND
    // text capture) took 16-28 ms of the core-1 loop and skipped 100 Hz
    // state log samples every 10 s.
    static constexpr std::size_t kDeferredLogLength = 400;
    struct DeferredLog {
        char text[kDeferredLogLength];
    };
    QueueHandle_t deferred_log_queue_ = nullptr;
    static void deferred_log_task_entry(void *context);
    void defer_log(const char *format, ...) __attribute__((format(printf, 2, 3)));
    // HITL frames arrive at about 100 Hz, so selection only, no low-pass.
    accel_filter::FilteredAccel hitl_filtered_accel_{};
    imu_calibration::Limits calibration_limits_{};
    imu_calibration::Accumulator calibration_capture_{};
    imu_calibration::SixFaceSolver calibration_faces_{};
    CalibrationCapture calibration_capture_kind_ = CalibrationCapture::None;
    std::uint32_t attitude_updates_since_report_ = 0;
    std::uint64_t attitude_report_start_us_ = 0;
    std::uint32_t loop_overruns_ = 0;
    std::uint32_t max_pass_us_ = 0;
    std::uint32_t radio_tx_started_ = 0;
    std::uint32_t radio_tx_failed_ = 0;
    // Packet queue failures are counted and summarized at most every 10 s;
    // logging each one filled the NAND text log (and its failure bit).
    std::uint32_t packet_queue_failures_ = 0;
    std::uint32_t packet_queue_failures_reported_ = 0;
    TickType_t packet_queue_last_report_ = 0;
    void note_packet_queue_result(const char *name, esp_err_t result, TickType_t now);
};
