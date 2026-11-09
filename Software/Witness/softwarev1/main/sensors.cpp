#include "sensors.h"
#include "flash_logger.h"
#include "imu_calibration_store.h"
#include "task_cores.h"
#include "uart_driver.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "command_bridge.h"
#include <limits>

#include "comms.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "flight_state_machine.h"
#include "hardware_pins.h"
#include "spi_data_forwarder.h"
#include "witness_status.h"

namespace {

constexpr char kLogTag[] = "sensors";
constexpr char kHitlTag[] = "hitl";

constexpr std::array<std::uint8_t, Sensors::kRadioPacketTypeCount>
    kRadioPacketIds = {
        IRIS_PACKET_ID_SENSORS,
        IRIS_PACKET_ID_STATE,
        IRIS_PACKET_ID_CAMERA,
        IRIS_PACKET_ID_COMMAND,
        IRIS_PACKET_ID_HEARTBEAT,
        IRIS_PACKET_ID_WITNESS_DEBUG,
        IRIS_PACKET_ID_IRIS_DEBUG,
};

constexpr std::array<std::uint32_t, Sensors::kRadioPacketTypeCount>
    kRadioPacketRatesHz = {
        Sensors::kRadioSensorPacketRateHz,
        Sensors::kRadioStatePacketRateHz,
        Sensors::kRadioCameraPacketRateHz,
        Sensors::kRadioCommandPacketRateHz,
        Sensors::kRadioHeartbeatPacketRateHz,
        Sensors::kRadioWitnessDebugPacketRateHz,
        Sensors::kRadioIrisDebugPacketRateHz,
};

constexpr bool valid_radio_rate(const std::uint32_t rate_hz)
{
    return rate_hz > 0U && rate_hz <= 1000U &&
           (1000U % rate_hz) == 0U;
}

static_assert(valid_radio_rate(Sensors::kRadioSensorPacketRateHz));
static_assert(valid_radio_rate(Sensors::kRadioStatePacketRateHz));
static_assert(valid_radio_rate(Sensors::kRadioCameraPacketRateHz));
static_assert(valid_radio_rate(Sensors::kRadioCommandPacketRateHz));
static_assert(valid_radio_rate(Sensors::kRadioHeartbeatPacketRateHz));
static_assert(valid_radio_rate(Sensors::kRadioWitnessDebugPacketRateHz));
static_assert(valid_radio_rate(Sensors::kRadioIrisDebugPacketRateHz));

static_assert(IRIS_PACKET_STATE_GPS_SATELLITES_LENGTH == 1U,
              "0x02 satellites field is one byte");
static_assert(IRIS_PACKET_STATE_BATTERY_VOLTAGE_LENGTH ==
                  sizeof(std::uint16_t),
              "0x02 battery encoder requires a two-byte field");
static_assert(IRIS_PACKET_SENSORS_ACCEL_COMPONENT_LENGTH == 3U,
              "filtered acceleration encoder requires a three-byte field");
static_assert(Sensors::kBatteryPacketMillivoltsPerCount ==
                  IRIS_FIELD_BATTERY_VOLTAGE_MILLIVOLTS_PER_COUNT,
              "battery telemetry scale must match the comms workbook");

void write_u16_be(std::uint8_t *destination, const std::uint16_t value)
{
    destination[0] = static_cast<std::uint8_t>(value >> 8);
    destination[1] = static_cast<std::uint8_t>(value);
}

void write_i16_be(std::uint8_t *destination, const std::int16_t value)
{
    write_u16_be(destination, static_cast<std::uint16_t>(value));
}

void write_i24_be(std::uint8_t *destination, const std::int32_t value)
{
    const std::uint32_t bits = static_cast<std::uint32_t>(value);
    destination[0] = static_cast<std::uint8_t>(bits >> 16);
    destination[1] = static_cast<std::uint8_t>(bits >> 8);
    destination[2] = static_cast<std::uint8_t>(bits);
}

void write_u24_be(std::uint8_t *destination, const std::uint32_t value)
{
    destination[0] = static_cast<std::uint8_t>(value >> 16);
    destination[1] = static_cast<std::uint8_t>(value >> 8);
    destination[2] = static_cast<std::uint8_t>(value);
}

std::int32_t encode_filtered_acceleration(const float acceleration_mg)
{
    constexpr std::int32_t kMinimumInt24 = -8'388'608;
    constexpr std::int32_t kMaximumInt24 = 8'388'607;
    const float acceleration_micro_g = acceleration_mg * 1000.0F;
    const long rounded = std::lround(
        acceleration_micro_g /
        static_cast<float>(IRIS_FIELD_FILTERED_ACCEL_MICRO_G_PER_COUNT));

    if (rounded < kMinimumInt24) {
        return kMinimumInt24;
    }
    if (rounded > kMaximumInt24) {
        return kMaximumInt24;
    }
    return static_cast<std::int32_t>(rounded);
}

void write_u32_be(std::uint8_t *destination, const std::uint32_t value)
{
    destination[0] = static_cast<std::uint8_t>(value >> 24);
    destination[1] = static_cast<std::uint8_t>(value >> 16);
    destination[2] = static_cast<std::uint8_t>(value >> 8);
    destination[3] = static_cast<std::uint8_t>(value);
}

void write_float_be(std::uint8_t *destination, const float value)
{
    static_assert(sizeof(float) == sizeof(std::uint32_t));
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    write_u32_be(destination, bits);
}

// Wire order X, Y, Z, W (scalar last); all zero until the estimator is valid.
void write_quaternion(std::uint8_t *payload, const std::size_t x_offset,
                      const std::size_t y_offset, const std::size_t z_offset,
                      const std::size_t w_offset, const attitude::Quaternion &q)
{
    write_float_be(&payload[x_offset], q.x);
    write_float_be(&payload[y_offset], q.y);
    write_float_be(&payload[z_offset], q.z);
    write_float_be(&payload[w_offset], q.w);
}

constexpr float kMilliDegreesToRadians = 3.14159265358979F / 180'000.0F;
constexpr float kRadiansToDegrees = 180.0F / 3.14159265358979F;

static_assert(static_cast<int>(attitude::FlightPhase::PadIdle) ==
              static_cast<int>(FlightStateMachine::State::PadIdle));
static_assert(static_cast<int>(attitude::FlightPhase::Boost) ==
              static_cast<int>(FlightStateMachine::State::Boost));
static_assert(static_cast<int>(attitude::FlightPhase::Coast) ==
              static_cast<int>(FlightStateMachine::State::Coast));
static_assert(static_cast<int>(attitude::FlightPhase::Descent) ==
              static_cast<int>(FlightStateMachine::State::Descent));
static_assert(static_cast<int>(attitude::FlightPhase::Landed) ==
              static_cast<int>(FlightStateMachine::State::Landed));

std::uint32_t uptime_ms()
{
    return static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL);
}

const char *hitl_source_name(const hitl::Source source)
{
    return source == hitl::Source::Usb ? "USB" : "radio";
}

constexpr std::uint8_t kSensorReadyMasks[] = {
    IRIS_WITNESS_STATUS_LOW_G_ACCEL_READY_MASK,
    IRIS_WITNESS_STATUS_GYRO_READY_MASK,
    IRIS_WITNESS_STATUS_IMU_TEMPERATURE_READY_MASK,
    IRIS_WITNESS_STATUS_HIGH_G_ACCEL_READY_MASK,
    IRIS_WITNESS_STATUS_BAROMETER_READY_MASK,
};

}  // namespace

Sensors::Sensors(
    SpiDataForwarder &data_forwarder,
    WitnessStatus &witness_status,
    CommandBridge &command_bridge,
    FlightStateMachine &flight_state_machine,
    UartDriver &gps_uart)
    : data_forwarder_(data_forwarder),
      witness_status_(witness_status),
      command_bridge_(command_bridge),
      flight_state_machine_(flight_state_machine),
      gps_uart_(gps_uart)
{
}

esp_err_t Sensors::start()
{
    if (task_handle_ != nullptr || spi_device_ != nullptr ||
        sample_mutex_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    sample_mutex_ = xSemaphoreCreateMutex();
    if (sample_mutex_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t result = initialize_spi();
    if (result != ESP_OK) {
        release_resources();
        return result;
    }

    {
        const esp_err_t load_result = imu_calibration::load(calibration_);
        if (load_result == ESP_OK) {
            log_calibration("IMU calibration loaded from NVS");
        } else {
            calibration_ = imu_calibration::Calibration{};
            ESP_LOGW(kLogTag,
                     "no IMU calibration stored (%s); IMU readings are "
                     "uncorrected. Calibrate with 05 43 47 (gyro) and six "
                     "05 43 41 face captures",
                     esp_err_to_name(load_result));
        }
    }

    fifo_tx_ = static_cast<std::uint8_t *>(heap_caps_calloc(
        1, kMaximumSpiTransfer, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    fifo_rx_ = static_cast<std::uint8_t *>(heap_caps_calloc(
        1, kMaximumSpiTransfer, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    if (fifo_tx_ == nullptr || fifo_rx_ == nullptr) {
        release_resources();
        return ESP_ERR_NO_MEM;
    }

    result = initialize_lsm6dsv320x();
    if (result != ESP_OK) {
        release_resources();
        return result;
    }

    result = ms5607_.initialize(kSpiHost);
    if (result == ESP_OK) {
        ms5607_initialized_ = true;
    } else {
        // A missing or unresponsive barometer must not prevent the IMU,
        // battery monitor, or radio from operating. Retry from the sensor
        // task at the state-packet rate.
        ms5607_.release();
        ESP_LOGE(kLogTag, "MS5607 initialization failed: %s",
                 esp_err_to_name(result));
    }

    if constexpr (HW_RADIO_ENABLED != 0) {
        result = radio_.initialize(kSpiHost);
        radio_ready_ = result == ESP_OK;
        if (!radio_ready_) {
            radio_.release();
            ESP_LOGW(kLogTag, "RA-01 unavailable (%s); radio disabled until reboot",
                     esp_err_to_name(result));
        }
    } else {
        ESP_LOGI(kLogTag, "radio disabled by HW_RADIO_ENABLED");
    }
    data_forwarder_.set_radio_available(radio_ready_);

    result = initialize_battery_adc();
    if (result != ESP_OK) {
        release_resources();
        return result;
    }

    if (xTaskCreatePinnedToCore(
            task_entry,
            "sensors",
            kTaskStackSize,
            this,
            kTaskPriority,
            &task_handle_,
            task_cores::kEstimation) != pdPASS) {
        task_handle_ = nullptr;
        release_resources();
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(kLogTag,
             "sensor task ready; radio %s", radio_ready_ ? "ready" : "disabled");
    return ESP_OK;
}

esp_err_t Sensors::latest_sample(Sample &sample, const TickType_t timeout) const
{
    if (sample_mutex_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(sample_mutex_, timeout) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    const bool available = sample_available_;
    if (available) {
        sample = latest_sample_;
    }
    xSemaphoreGive(sample_mutex_);
    return available ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t Sensors::handle_command(const std::uint16_t command)
{
    if (command != IRIS_COMMAND_WITNESS_DEBUG_START) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    debug_packet_output_enabled_.store(true);
    ESP_LOGI(kLogTag, "Witness debug packets enabled at %u Hz",
             static_cast<unsigned>(kDebugPacketRateHz));
    return ESP_OK;
}

esp_err_t Sensors::enter_hitl(const hitl::Source source)
{
    const FlightStateMachine::State state = flight_state_machine_.state();
    const std::uint32_t now_ms = uptime_ms();
    portENTER_CRITICAL(&hitl_lock_);
    const auto result = hitl_controller_.enter(
        source, state == FlightStateMachine::State::PadIdle, now_ms);
    portEXIT_CRITICAL(&hitl_lock_);

    switch (result) {
    case hitl::Controller::EnterResult::Entered:
        ESP_LOGW(kHitlTag,
                 "HITL mode entered from USB: IMU, barometer, GPS and battery "
                 "now come from 0x0C packets; exits after %u ms without input",
                 static_cast<unsigned>(hitl::kInputTimeoutMs));
        return ESP_OK;
    case hitl::Controller::EnterResult::AlreadyActive:
        ESP_LOGI(kHitlTag, "HITL enter ignored: already active");
        return ESP_OK;
    case hitl::Controller::EnterResult::RejectedSource:
        ESP_LOGW(kHitlTag,
                 "HITL enter rejected: received from %s; HITL can only be "
                 "entered over USB",
                 hitl_source_name(source));
        return ESP_ERR_NOT_ALLOWED;
    case hitl::Controller::EnterResult::RejectedFlightState:
    default:
        ESP_LOGW(kHitlTag,
                 "HITL enter rejected: flight state is %s; HITL requires pad idle",
                 FlightStateMachine::state_name(state));
        return ESP_ERR_INVALID_STATE;
    }
}

esp_err_t Sensors::exit_hitl(const hitl::Source source)
{
    portENTER_CRITICAL(&hitl_lock_);
    const bool was_active = hitl_controller_.exit();
    portEXIT_CRITICAL(&hitl_lock_);
    if (was_active) {
        ESP_LOGI(kHitlTag, "HITL mode exited by %s command; real sensors restored",
                 hitl_source_name(source));
    } else {
        ESP_LOGI(kHitlTag, "HITL exit from %s ignored: HITL not active",
                 hitl_source_name(source));
    }
    return ESP_OK;
}

bool Sensors::submit_hitl_sample(const hitl::Sample &sample)
{
    const std::uint32_t now_ms = uptime_ms();
    portENTER_CRITICAL(&hitl_lock_);
    const bool accepted = hitl_controller_.submit(sample, now_ms);
    portEXIT_CRITICAL(&hitl_lock_);
    return accepted;
}

bool Sensors::hitl_active() const
{
    portENTER_CRITICAL(&hitl_lock_);
    const bool active = hitl_controller_.active();
    portEXIT_CRITICAL(&hitl_lock_);
    return active;
}

bool Sensors::service_hitl(const std::uint32_t now_ms, bool &new_sample)
{
    new_sample = false;
    portENTER_CRITICAL(&hitl_lock_);
    const bool timed_out = hitl_controller_.expire(now_ms);
    const bool active = hitl_controller_.active();
    const std::uint32_t session = hitl_controller_.session();
    if (active) {
        new_sample = hitl_controller_.take_sample(hitl_sample_);
    }
    portEXIT_CRITICAL(&hitl_lock_);

    if (timed_out) {
        ESP_LOGW(kHitlTag,
                 "HITL input timeout: no valid 0x0C packet for %u ms; exiting "
                 "HITL, real sensors restored",
                 static_cast<unsigned>(hitl::kInputTimeoutMs));
    }

    // Entry, exit, timeout or exit+re-entry: discard every sensor value so
    // real and simulated data are never mixed. Readiness flags stay clear and
    // no sensor packet or flight-state observation is produced until the
    // first sample from the new source (a 0x0C frame or a real reading).
    if (active != hitl_in_use_ || (active && session != hitl_session_in_use_)) {
        hitl_in_use_ = active;
        hitl_session_in_use_ = session;
        hitl_has_sample_ = false;
        hitl_samples_since_summary_ = 0;
        hitl_last_summary_ms_ = now_ms;
        reset_sensor_sample();
        filtered_accel_.reset();
        hitl_filtered_accel_.reset();
        position_filter_.reset();
        position_last_predict_us_ = 0;
        position_baro_pending_ = false;
        // Never carry an attitude across a real/simulated source change.
        attitude_.reset();
        ESP_LOGI(kLogTag, "attitude reset: sensor source is now %s",
                 active ? "HITL 0x0C frames" : "the real IMU");
        witness_status_.set_state_flag(
            IRIS_WITNESS_STATUS_HITL_ACTIVE_MASK, active);
        publish_sample();
    }

    if (new_sample) {
        if (!hitl_has_sample_) {
            ESP_LOGI(kHitlTag, "first HITL sample received (sim time %lu ms)",
                     static_cast<unsigned long>(hitl_sample_.sim_time_ms));
        }
        apply_hitl_sample();
        hitl_has_sample_ = true;
        ++hitl_samples_since_summary_;
    }

    if (active && now_ms - hitl_last_summary_ms_ >= kHitlSummaryIntervalMs) {
        ESP_LOGI(kHitlTag, "HITL active: %lu samples in last %lu ms, sim time %lu ms",
                 static_cast<unsigned long>(hitl_samples_since_summary_),
                 static_cast<unsigned long>(now_ms - hitl_last_summary_ms_),
                 static_cast<unsigned long>(hitl_sample_.sim_time_ms));
        hitl_samples_since_summary_ = 0;
        hitl_last_summary_ms_ = now_ms;
    }
    return active;
}

void Sensors::apply_hitl_sample()
{
    const hitl::Sample &input = hitl_sample_;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        working_sample_.low_g_accel_raw[axis] = input.low_g_accel_raw[axis];
        working_sample_.high_g_accel_raw[axis] = input.high_g_accel_raw[axis];
        working_sample_.gyro_raw[axis] = input.gyro_raw[axis];
        working_sample_.low_g_accel_mg[axis] = input.low_g_accel_mg[axis];
        working_sample_.high_g_accel_mg[axis] = input.high_g_accel_mg[axis];
        working_sample_.gyro_mdps[axis] = input.gyro_mdps[axis];
    }
    hitl_filtered_accel_.step(working_sample_.low_g_accel_mg,
                              working_sample_.high_g_accel_mg,
                              working_sample_.filtered_accel_mg);
    working_sample_.high_g_accel_active = hitl_filtered_accel_.high_g_active();
    working_sample_.temperature_raw = input.temperature_raw;
    working_sample_.temperature_celsius = input.temperature_celsius;
    working_sample_.pressure_centi_mbar =
        static_cast<std::int32_t>(input.pressure_pa);
    working_sample_.ms5607_temperature_centi_celsius =
        input.barometer_temperature_centi_celsius;
    working_sample_.barometric_altitude_meters =
        input.barometric_altitude_meters;
    working_sample_.battery_voltage_mv = input.battery_mv;
    working_sample_.battery_adc_raw = 0;
    working_sample_.low_g_accel_ready = true;
    working_sample_.high_g_accel_ready = true;
    working_sample_.gyro_ready = true;
    working_sample_.temperature_ready = true;
    working_sample_.ms5607_ready = input.barometer_valid;
    has_low_g_accel_sample_ = true;
    has_high_g_accel_sample_ = true;
    has_gyro_sample_ = true;

    witness_status_.set(IRIS_WITNESS_STATUS_LOW_G_ACCEL_READY_MASK, true);
    witness_status_.set(IRIS_WITNESS_STATUS_GYRO_READY_MASK, true);
    witness_status_.set(IRIS_WITNESS_STATUS_IMU_TEMPERATURE_READY_MASK, true);
    witness_status_.set(IRIS_WITNESS_STATUS_HIGH_G_ACCEL_READY_MASK, true);
    witness_status_.set(IRIS_WITNESS_STATUS_BAROMETER_READY_MASK,
                        input.barometer_valid);
}

void Sensors::reset_sensor_sample()
{
    working_sample_ = Sample{};
    has_low_g_accel_sample_ = false;
    has_high_g_accel_sample_ = false;
    has_gyro_sample_ = false;
    for (const std::uint8_t mask : kSensorReadyMasks) {
        witness_status_.set(mask, false);
    }
}

NmeaGps::Fix Sensors::current_gps_fix() const
{
    if (!hitl_in_use_) {
        return gps_uart_.latest_gps_fix();
    }
    // Same shape as a fresh UART GGA: satellites even without a fix, and
    // position fields only with one. Freshness is the HITL input timeout.
    NmeaGps::Fix fix{};
    if (!hitl_has_sample_) {
        return fix;
    }
    fix.gga = true;
    fix.received_ms = uptime_ms();
    fix.satellites = hitl_sample_.satellites;
    fix.fix_quality = hitl_sample_.fix_quality;
    fix.valid = hitl_sample_.gps_fix;
    if (fix.valid) {
        fix.latitude_degrees = hitl_sample_.latitude_degrees;
        fix.longitude_degrees = hitl_sample_.longitude_degrees;
        fix.altitude_meters = hitl_sample_.gps_altitude_meters;
    }
    return fix;
}

void Sensors::set_radio_transmission_enabled(const bool enabled)
{
    radio_transmission_enabled_.store(enabled);
    data_forwarder_.set_radio_available(enabled && radio_ready_);
    ESP_LOGI(
        kLogTag,
        "radio transmission %s",
        enabled ? "enabled" : "disabled");
}

std::int16_t Sensors::decode_i16(const std::uint8_t *bytes)
{
    const std::uint16_t value =
        static_cast<std::uint16_t>(bytes[0]) |
        (static_cast<std::uint16_t>(bytes[1]) << 8);
    return static_cast<std::int16_t>(value);
}

esp_err_t Sensors::initialize_spi()
{
    spi_bus_config_t bus_config{};
    bus_config.mosi_io_num = HW_PIN_SPI3_COPI;
    bus_config.miso_io_num = HW_PIN_SPI3_CIPO;
    bus_config.sclk_io_num = HW_PIN_SPI3_CLOCK;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.data4_io_num = -1;
    bus_config.data5_io_num = -1;
    bus_config.data6_io_num = -1;
    bus_config.data7_io_num = -1;
    bus_config.max_transfer_sz = kMaximumSpiTransfer;
    bus_config.flags = SPICOMMON_BUSFLAG_MASTER;

    esp_err_t result = spi_bus_initialize(
        kSpiHost, &bus_config, SPI_DMA_CH_AUTO);
    if (result == ESP_OK) {
        owns_spi_bus_ = true;
    } else if (result != ESP_ERR_INVALID_STATE) {
        return result;
    }

    spi_device_interface_config_t device_config{};
    device_config.mode = kSpiMode;
    device_config.clock_speed_hz = kSpiClockFrequencyHz;
    device_config.spics_io_num = HW_PIN_LSM_CS;
    device_config.queue_size = 1;
    device_config.cs_ena_posttrans = 1;

    result = spi_bus_add_device(kSpiHost, &device_config, &spi_device_);
    if (result != ESP_OK && owns_spi_bus_) {
        spi_bus_free(kSpiHost);
        owns_spi_bus_ = false;
    }
    return result;
}

esp_err_t Sensors::initialize_battery_adc()
{
    adc_unit_t mapped_unit = ADC_UNIT_1;
    adc_channel_t mapped_channel = ADC_CHANNEL_0;
    esp_err_t result = adc_oneshot_io_to_channel(
        HW_PIN_VSENSE, &mapped_unit, &mapped_channel);
    if (result != ESP_OK) {
        return result;
    }
    if (mapped_unit != kBatteryAdcUnit ||
        mapped_channel != kBatteryAdcChannel) {
        ESP_LOGE(kLogTag, "VSENSE pin does not match configured ADC channel");
        return ESP_ERR_INVALID_STATE;
    }

    adc_oneshot_unit_init_cfg_t unit_config{};
    unit_config.unit_id = kBatteryAdcUnit;
    result = adc_oneshot_new_unit(&unit_config, &battery_adc_handle_);
    if (result != ESP_OK) {
        return result;
    }

    adc_oneshot_chan_cfg_t channel_config{};
    channel_config.atten = kBatteryAdcAttenuation;
    channel_config.bitwidth = kBatteryAdcBitWidth;
    result = adc_oneshot_config_channel(
        battery_adc_handle_, kBatteryAdcChannel, &channel_config);
    if (result != ESP_OK) {
        return result;
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t calibration_config{};
    calibration_config.unit_id = kBatteryAdcUnit;
    calibration_config.chan = kBatteryAdcChannel;
    calibration_config.atten = kBatteryAdcAttenuation;
    calibration_config.bitwidth = kBatteryAdcBitWidth;
    result = adc_cali_create_scheme_curve_fitting(
        &calibration_config, &battery_adc_calibration_);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "battery ADC calibration failed: %s",
                 esp_err_to_name(result));
        return result;
    }
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif

    return ESP_OK;
}

esp_err_t Sensors::initialize_lsm6dsv320x()
{
    vTaskDelay(pdMS_TO_TICKS(kBootDelayMs));

    std::uint8_t who_am_i = 0;
    esp_err_t result = read_registers(kRegWhoAmI, &who_am_i, 1);
    if (result != ESP_OK) {
        return result;
    }
    if (who_am_i != kExpectedWhoAmI) {
        ESP_LOGE(kLogTag, "unexpected WHO_AM_I: 0x%02X", who_am_i);
        return ESP_ERR_NOT_FOUND;
    }

    result = write_register(kRegFuncCfgAccess, kSoftwarePowerOnReset);
    if (result != ESP_OK) {
        return result;
    }
    vTaskDelay(pdMS_TO_TICKS(kResetDelayMs));

    result = write_register(kRegCtrl3, kCtrl3Configuration);
    if (result != ESP_OK) {
        return result;
    }
    // Filters and full scales are written while both sensors are still in
    // power-down (ODR = 0 after reset), as the datasheet requires for FS_G.
    result = write_register(kRegCtrl8, kLowGAccelFullScale);
    if (result != ESP_OK) {
        return result;
    }
    result = write_register(kRegCtrl9, kLowGAccelFilterConfiguration);
    if (result != ESP_OK) {
        return result;
    }
    result = write_register(kRegCtrl6, kGyroFullScale);
    if (result != ESP_OK) {
        return result;
    }
    result = write_register(kRegCtrl7, kGyroFilterConfiguration);
    if (result != ESP_OK) {
        return result;
    }

    const std::uint8_t high_g_configuration =
        kHighGRegisterOutputEnable |
        static_cast<std::uint8_t>(kHighGAccelOutputDataRate << 3) |
        kHighGAccelFullScale;
    result = write_register(kRegHighGCtrl1, high_g_configuration);
    if (result != ESP_OK) {
        return result;
    }

    result = write_register(kRegCtrl1, kLowGAccelOutputDataRate);
    if (result != ESP_OK) {
        return result;
    }
    result = write_register(kRegCtrl2, kGyroOutputDataRate);
    if (result != ESP_OK) {
        return result;
    }

    // FIFO: batch gyro, low-g and high-g at 960 Hz in continuous mode. The
    // sensor task drains it every pass; samples are timed by the sensor clock.
    result = write_register(kRegFifoCtrl4, kFifoBypass);
    if (result != ESP_OK) {
        return result;
    }
    result = write_register(kRegCounterBdr1, kFifoHighGBatch);
    if (result != ESP_OK) {
        return result;
    }
    result = write_register(kRegFifoCtrl3, kFifoBatchRates);
    if (result != ESP_OK) {
        return result;
    }
    std::uint8_t freq_fine = 0;
    result = read_registers(kRegInternalFreqFine, &freq_fine, 1);
    if (result != ESP_OK) {
        return result;
    }
    imu_freq_fine_ = static_cast<std::int8_t>(freq_fine);
    imu_period_us_ =
        1e6 / (static_cast<double>(kImuSampleRateHz) *
               (1.0 + 0.0013 * static_cast<double>(imu_freq_fine_)));
    imu_clock_valid_ = false;

    result = write_register(kRegFifoCtrl4, kFifoContinuous);
    if (result == ESP_OK) {
        ESP_LOGI(kLogTag,
                 "LSM6 7.68 kHz ODR, gyro LPF1 407 Hz, low-g LPF2 384 Hz, "
                 "high-g 960 Hz; FIFO batching at %lu Hz nominal, %.2f Hz "
                 "actual (FREQ_FINE %d)",
                 static_cast<unsigned long>(kImuSampleRateHz),
                 1e6 / imu_period_us_, static_cast<int>(imu_freq_fine_));
    }
    return result;
}

esp_err_t Sensors::read_registers(
    const std::uint8_t first_register,
    std::uint8_t *data,
    const std::size_t length)
{
    if (spi_device_ == nullptr || data == nullptr || length == 0 ||
        length > kMaximumRegisterTransfer) {
        return ESP_ERR_INVALID_ARG;
    }

    std::uint8_t transmit[kMaximumRegisterTransfer + 1]{};
    std::uint8_t receive[kMaximumRegisterTransfer + 1]{};
    transmit[0] = first_register | kReadCommand;

    spi_transaction_t transaction{};
    transaction.length = (length + 1) * 8;
    transaction.rxlength = transaction.length;
    transaction.tx_buffer = transmit;
    transaction.rx_buffer = receive;

    const esp_err_t result = spi_device_polling_transmit(spi_device_, &transaction);
    if (result == ESP_OK) {
        std::memcpy(data, &receive[1], length);
    }
    return result;
}

esp_err_t Sensors::write_registers(
    const std::uint8_t first_register,
    const std::uint8_t *data,
    const std::size_t length)
{
    if (spi_device_ == nullptr || data == nullptr || length == 0 ||
        length > kMaximumRegisterTransfer) {
        return ESP_ERR_INVALID_ARG;
    }

    std::uint8_t transmit[kMaximumRegisterTransfer + 1]{};
    transmit[0] = first_register;
    std::memcpy(&transmit[1], data, length);

    spi_transaction_t transaction{};
    transaction.length = (length + 1) * 8;
    transaction.tx_buffer = transmit;
    return spi_device_polling_transmit(spi_device_, &transaction);
}

esp_err_t Sensors::write_register(
    const std::uint8_t register_address,
    const std::uint8_t value)
{
    return write_registers(register_address, &value, 1);
}

esp_err_t Sensors::read_fifo_words(const std::size_t words)
{
    // One burst from FIFO_DATA_OUT_TAG: with IF_INC the address rolls back
    // from 7Eh to 78h, so consecutive 7-byte words stream out.
    const std::size_t length = words * kFifoWordSize;
    fifo_tx_[0] = kRegFifoDataOutTag | kReadCommand;
    spi_transaction_t transaction{};
    transaction.length = (length + 1) * 8;
    transaction.rxlength = transaction.length;
    transaction.tx_buffer = fifo_tx_;
    transaction.rx_buffer = fifo_rx_;
    return spi_device_polling_transmit(spi_device_, &transaction);
}

void Sensors::finish_fifo_slot()
{
    if (!fifo_slot_open_) {
        return;
    }
    fifo_slot_open_ = false;
    if (!fifo_slot_.has_gyro || !fifo_slot_.has_low_g ||
        !fifo_slot_.has_high_g) {
        ++fifo_partial_slots_;
    }
    // Missing channels hold their previous value.
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (!fifo_slot_.has_gyro) fifo_slot_.gyro[axis] = last_imu_.gyro[axis];
        if (!fifo_slot_.has_low_g) fifo_slot_.low_g[axis] = last_imu_.low_g[axis];
        if (!fifo_slot_.has_high_g) fifo_slot_.high_g[axis] = last_imu_.high_g[axis];
    }
    last_imu_ = fifo_slot_;
    if (imu_batch_count_ < imu_batch_.size()) {
        imu_batch_[imu_batch_count_++] = fifo_slot_;
    }
}

void Sensors::push_fifo_word(const std::uint8_t *word)
{
    const std::uint8_t tag = word[0] >> 3;
    const std::uint8_t tag_count = (word[0] >> 1) & 0x03U;
    std::int16_t *target = nullptr;
    bool *present = nullptr;
    switch (tag) {
    case kFifoTagGyro:
        target = fifo_slot_.gyro;
        present = &fifo_slot_.has_gyro;
        break;
    case kFifoTagLowG:
        target = fifo_slot_.low_g;
        present = &fifo_slot_.has_low_g;
        break;
    case kFifoTagHighG:
        target = fifo_slot_.high_g;
        present = &fifo_slot_.has_high_g;
        break;
    default:
        ++fifo_bad_words_;
        return;
    }
    // A new TAG_CNT, or a repeat of a channel already in the slot, starts
    // the next time slot.
    if (fifo_slot_open_ && (tag_count != fifo_slot_.tag_count || *present)) {
        finish_fifo_slot();
    }
    if (!fifo_slot_open_) {
        fifo_slot_ = ImuSample{};
        fifo_slot_.tag_count = tag_count;
        fifo_slot_open_ = true;
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
        target[axis] = decode_i16(&word[1 + axis * 2]);
    }
    *present = true;
    if (fifo_slot_.has_gyro && fifo_slot_.has_low_g && fifo_slot_.has_high_g) {
        finish_fifo_slot();
    }
}

void Sensors::apply_imu_sample(const ImuSample &sample, const bool feed_capture)
{
    working_sample_.gyro_ready = sample.has_gyro;
    working_sample_.low_g_accel_ready = sample.has_low_g;
    working_sample_.high_g_accel_ready = sample.has_high_g;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        working_sample_.gyro_raw[axis] = sample.gyro[axis];
        working_sample_.gyro_mdps[axis] =
            static_cast<float>(sample.gyro[axis]) * kGyroSensitivityMdps;
        working_sample_.low_g_accel_raw[axis] = sample.low_g[axis];
        working_sample_.low_g_accel_mg[axis] =
            static_cast<float>(sample.low_g[axis]) * kLowGAccelSensitivityMg;
        working_sample_.high_g_accel_raw[axis] = sample.high_g[axis];
        working_sample_.high_g_accel_mg[axis] =
            static_cast<float>(sample.high_g[axis]) * kHighGAccelSensitivityMg;
    }
    // Calibration captures see uncorrected values; everything else is
    // corrected. Raw counts (and so the NAND IMU log) stay uncorrected.
    if (feed_capture && calibration_capture_kind_ != CalibrationCapture::None) {
        calibration_capture_.add(working_sample_.gyro_mdps,
                                 working_sample_.low_g_accel_mg,
                                 working_sample_.high_g_accel_mg);
    }
    imu_calibration::apply(calibration_, working_sample_.gyro_mdps,
                           working_sample_.low_g_accel_mg,
                           working_sample_.high_g_accel_mg);
    if (sample.has_gyro) has_gyro_sample_ = true;
    if (sample.has_low_g) has_low_g_accel_sample_ = true;
    if (sample.has_high_g) has_high_g_accel_sample_ = true;
}

esp_err_t Sensors::fetch_lsm6dsv320x()
{
    imu_batch_count_ = 0;

    std::uint8_t fifo_status[2]{};
    esp_err_t result = read_registers(kRegFifoStatus1, fifo_status, 2);
    if (result != ESP_OK) {
        return result;
    }
    if ((fifo_status[1] & kFifoStatusOverrun) != 0) {
        // Samples were lost; the slot in progress no longer lines up.
        ++fifo_overruns_;
        fifo_slot_open_ = false;
        imu_clock_valid_ = false;
    }
    std::size_t words =
        static_cast<std::size_t>(fifo_status[0]) |
        (static_cast<std::size_t>(fifo_status[1] & kFifoStatusDiff8) << 8);
    if (words > fifo_max_words_) {
        fifo_max_words_ = static_cast<std::uint32_t>(words);
    }
    // Leave whatever does not fit this pass's batch in the FIFO.
    const std::size_t word_limit = kMaxImuSamplesPerPass * 3U;
    if (words > word_limit) {
        words = word_limit;
    }

    while (words > 0) {
        const std::size_t burst =
            words < kFifoWordsPerBurst ? words : kFifoWordsPerBurst;
        result = read_fifo_words(burst);
        if (result != ESP_OK) {
            return result;
        }
        for (std::size_t i = 0; i < burst; ++i) {
            push_fifo_word(&fifo_rx_[1 + i * kFifoWordSize]);
        }
        words -= burst;
    }

    // Timestamps: a sample clock advancing one sensor period per sample. The
    // newest sample is nominally "now"; the clock is steered 1/16 of the way
    // toward that each pass, and snaps to it after a startup, overrun or
    // error above 5 ms, so read jitter does not become dt jitter.
    if (imu_batch_count_ > 0) {
        const double now_us = static_cast<double>(esp_timer_get_time());
        const double count = static_cast<double>(imu_batch_count_);
        double offset_us = 0.0;
        if (imu_clock_valid_) {
            const double error_us =
                now_us - (imu_clock_us_ + count * imu_period_us_);
            offset_us = (error_us > 5000.0 || error_us < -5000.0)
                            ? error_us
                            : error_us / 16.0;
        } else {
            imu_clock_us_ = now_us - count * imu_period_us_;
            imu_clock_valid_ = true;
        }
        for (std::size_t i = 0; i < imu_batch_count_; ++i) {
            const double step = static_cast<double>(i + 1U);
            imu_batch_[i].timestamp_us = static_cast<std::uint64_t>(
                imu_clock_us_ + step * imu_period_us_ +
                offset_us * step / count + 0.5);
        }
        imu_clock_us_ += count * imu_period_us_ + offset_us;
    }
    fifo_samples_ += static_cast<std::uint32_t>(imu_batch_count_);

    bool any_gyro = false;
    bool any_low_g = false;
    bool any_high_g = false;
    for (std::size_t i = 0; i < imu_batch_count_; ++i) {
        any_gyro = any_gyro || imu_batch_[i].has_gyro;
        any_low_g = any_low_g || imu_batch_[i].has_low_g;
        any_high_g = any_high_g || imu_batch_[i].has_high_g;
    }
    if (imu_batch_count_ > 0) {
        // The packets and the flight state machine see the newest sample.
        apply_imu_sample(imu_batch_[imu_batch_count_ - 1U]);
    }
    working_sample_.gyro_ready = any_gyro;
    working_sample_.low_g_accel_ready = any_low_g;
    working_sample_.high_g_accel_ready = any_high_g;

    // Temperature is not batched; read the output register at 10 Hz.
    working_sample_.temperature_ready = false;
    if (++temperature_counter_ >= kTemperatureDivider) {
        temperature_counter_ = 0;
        std::uint8_t temperature[2]{};
        result = read_registers(kRegOutputStart, temperature, 2);
        if (result != ESP_OK) {
            return result;
        }
        working_sample_.temperature_raw = decode_i16(temperature);
        working_sample_.temperature_celsius =
            static_cast<float>(working_sample_.temperature_raw) / 256.0F + 25.0F;
        working_sample_.temperature_ready = true;
    }
    return ESP_OK;
}

esp_err_t Sensors::queue_sensor_packet()
{
    std::uint8_t payload[IRIS_PACKET_SENSORS_DATA_LENGTH]{};

    witness_status_.write(&payload[IRIS_PACKET_SENSORS_STATUS_OFFSET]);

    const std::uint32_t uptime_milliseconds =
        static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL);
    write_u32_be(
        &payload[IRIS_PACKET_SENSORS_UPTIME_OFFSET], uptime_milliseconds);

    // Filtered acceleration: calibrated low-g, or high-g while the low-g is
    // near saturation, through the 100 Hz low-pass (accel_filter.h).
    std::int16_t gyro_counts[3]{};
    std::int16_t low_counts[3]{};
    std::int16_t high_counts[3]{};
    output_counts(gyro_counts, low_counts, high_counts);
    for (std::size_t axis = 0; axis < IRIS_PACKET_VECTOR_COMPONENTS; ++axis) {
        write_i24_be(
            &payload[IRIS_PACKET_SENSORS_ACCEL_OFFSET +
                     axis * IRIS_PACKET_SENSORS_ACCEL_COMPONENT_LENGTH],
            encode_filtered_acceleration(
                working_sample_.filtered_accel_mg[axis]));
        write_i16_be(
            &payload[IRIS_PACKET_SENSORS_GYRO_OFFSET +
                     axis * IRIS_PACKET_SENSORS_GYRO_COMPONENT_LENGTH],
            gyro_counts[axis]);
    }

    return data_forwarder_.queue_packet(
        IRIS_PACKET_ID_SENSORS, payload, sizeof(payload));
}

esp_err_t Sensors::queue_state_packet()
{
    std::uint8_t payload[IRIS_PACKET_STATE_DATA_LENGTH]{};
    const auto gps = current_gps_fix();
    write_i24_be(&payload[IRIS_PACKET_STATE_LATITUDE_OFFSET],
        static_cast<std::int32_t>(std::lround(gps.latitude_degrees * IRIS_FIELD_GPS_COORDINATE_SCALE)));
    write_i24_be(&payload[IRIS_PACKET_STATE_LONGITUDE_OFFSET],
        static_cast<std::int32_t>(std::lround(gps.longitude_degrees * IRIS_FIELD_GPS_COORDINATE_SCALE)));
    write_float_be(&payload[IRIS_PACKET_STATE_GPS_ALTITUDE_OFFSET], gps.altitude_meters);
    witness_status_.write(&payload[IRIS_PACKET_STATE_STATUS_OFFSET]);

    const std::uint32_t uptime_milliseconds =
        static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL);
    write_u32_be(
        &payload[IRIS_PACKET_STATE_UPTIME_OFFSET], uptime_milliseconds);
    write_float_be(
        &payload[IRIS_PACKET_STATE_BAROMETRIC_ALTITUDE_OFFSET],
        working_sample_.barometric_altitude_meters);
    write_u16_be(
        &payload[IRIS_PACKET_STATE_BATTERY_VOLTAGE_OFFSET],
        encode_battery_voltage(working_sample_.battery_voltage_mv));
    write_quaternion(payload, IRIS_PACKET_STATE_ORIENTATION_X_OFFSET,
                     IRIS_PACKET_STATE_ORIENTATION_Y_OFFSET,
                     IRIS_PACKET_STATE_ORIENTATION_Z_OFFSET,
                     IRIS_PACKET_STATE_ORIENTATION_W_OFFSET,
                     attitude_.quaternion());
    {
        // Position filter: ENU metres / m/s relative to the pad.
        float p[3];
        float v[3];
        position_filter_.position(p);
        position_filter_.velocity(v);
        write_float_be(&payload[IRIS_PACKET_STATE_POSITION_X_OFFSET], p[0]);
        write_float_be(&payload[IRIS_PACKET_STATE_POSITION_Y_OFFSET], p[1]);
        write_float_be(&payload[IRIS_PACKET_STATE_POSITION_Z_OFFSET], p[2]);
        write_float_be(&payload[IRIS_PACKET_STATE_VELOCITY_X_OFFSET], v[0]);
        write_float_be(&payload[IRIS_PACKET_STATE_VELOCITY_Y_OFFSET], v[1]);
        write_float_be(&payload[IRIS_PACKET_STATE_VELOCITY_Z_OFFSET], v[2]);
    }
    // Reported from the latest GGA even without a fix; zero when stale.
    payload[IRIS_PACKET_STATE_GPS_SATELLITES_OFFSET] = gps.satellites;

    return data_forwarder_.queue_packet(
        IRIS_PACKET_ID_STATE, payload, sizeof(payload));
}

esp_err_t Sensors::queue_debug_packet()
{
    std::uint8_t payload[IRIS_PACKET_WITNESS_DEBUG_DATA_LENGTH]{};

    witness_status_.write(
        &payload[IRIS_PACKET_WITNESS_DEBUG_STATUS_OFFSET]);

    const std::uint32_t uptime_milliseconds =
        static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL);
    write_u32_be(
        &payload[IRIS_PACKET_WITNESS_DEBUG_UPTIME_OFFSET],
        uptime_milliseconds);
    write_u16_be(
        &payload[IRIS_PACKET_WITNESS_DEBUG_BATTERY_VOLTAGE_OFFSET],
        encode_battery_voltage(working_sample_.battery_voltage_mv));

    constexpr std::size_t kFilteredAccelOffsets[] = {
        IRIS_PACKET_WITNESS_DEBUG_FILTERED_ACCEL_X_OFFSET,
        IRIS_PACKET_WITNESS_DEBUG_FILTERED_ACCEL_Y_OFFSET,
        IRIS_PACKET_WITNESS_DEBUG_FILTERED_ACCEL_Z_OFFSET,
    };
    constexpr std::size_t kLowAccelOffsets[] = {
        IRIS_PACKET_WITNESS_DEBUG_LOW_ACCEL_X_OFFSET,
        IRIS_PACKET_WITNESS_DEBUG_LOW_ACCEL_Y_OFFSET,
        IRIS_PACKET_WITNESS_DEBUG_LOW_ACCEL_Z_OFFSET,
    };
    constexpr std::size_t kHighAccelOffsets[] = {
        IRIS_PACKET_WITNESS_DEBUG_HIGH_ACCEL_X_OFFSET,
        IRIS_PACKET_WITNESS_DEBUG_HIGH_ACCEL_Y_OFFSET,
        IRIS_PACKET_WITNESS_DEBUG_HIGH_ACCEL_Z_OFFSET,
    };
    constexpr std::size_t kGyroOffsets[] = {
        IRIS_PACKET_WITNESS_DEBUG_GYRO_X_OFFSET,
        IRIS_PACKET_WITNESS_DEBUG_GYRO_Y_OFFSET,
        IRIS_PACKET_WITNESS_DEBUG_GYRO_Z_OFFSET,
    };

    std::int16_t gyro_counts[3]{};
    std::int16_t low_counts[3]{};
    std::int16_t high_counts[3]{};
    output_counts(gyro_counts, low_counts, high_counts);
    for (std::size_t axis = 0; axis < IRIS_PACKET_VECTOR_COMPONENTS; ++axis) {
        write_i24_be(
            &payload[kFilteredAccelOffsets[axis]],
            encode_filtered_acceleration(
                working_sample_.filtered_accel_mg[axis]));
        write_i16_be(&payload[kLowAccelOffsets[axis]], low_counts[axis]);
        write_i16_be(&payload[kHighAccelOffsets[axis]], high_counts[axis]);
        write_i16_be(&payload[kGyroOffsets[axis]], gyro_counts[axis]);
    }

    write_i16_be(
        &payload[IRIS_PACKET_WITNESS_DEBUG_LSM6_TEMPERATURE_OFFSET],
        working_sample_.temperature_raw);

    const std::int32_t pressure = working_sample_.pressure_centi_mbar;
    const std::uint32_t encoded_pressure =
        pressure <= 0
            ? 0U
            : (pressure > 0x00FFFFFF
                   ? 0x00FFFFFFU
                   : static_cast<std::uint32_t>(pressure));
    write_u24_be(
        &payload[IRIS_PACKET_WITNESS_DEBUG_PRESSURE_OFFSET],
        encoded_pressure);

    const std::int32_t barometer_temperature =
        working_sample_.ms5607_temperature_centi_celsius;
    const std::int16_t encoded_barometer_temperature =
        barometer_temperature < std::numeric_limits<std::int16_t>::min()
            ? std::numeric_limits<std::int16_t>::min()
            : (barometer_temperature >
                       std::numeric_limits<std::int16_t>::max()
                   ? std::numeric_limits<std::int16_t>::max()
                   : static_cast<std::int16_t>(barometer_temperature));
    write_i16_be(
        &payload[IRIS_PACKET_WITNESS_DEBUG_MS5607_TEMPERATURE_OFFSET],
        encoded_barometer_temperature);
    write_float_be(
        &payload[IRIS_PACKET_WITNESS_DEBUG_BAROMETRIC_ALTITUDE_OFFSET],
        working_sample_.barometric_altitude_meters);

    const auto gps = current_gps_fix();
    write_i24_be(&payload[IRIS_PACKET_WITNESS_DEBUG_LATITUDE_OFFSET],
        static_cast<std::int32_t>(std::lround(gps.latitude_degrees * IRIS_FIELD_GPS_COORDINATE_SCALE)));
    write_i24_be(&payload[IRIS_PACKET_WITNESS_DEBUG_LONGITUDE_OFFSET],
        static_cast<std::int32_t>(std::lround(gps.longitude_degrees * IRIS_FIELD_GPS_COORDINATE_SCALE)));
    write_float_be(&payload[IRIS_PACKET_WITNESS_DEBUG_GPS_ALTITUDE_OFFSET], gps.altitude_meters);
    write_quaternion(payload, IRIS_PACKET_WITNESS_DEBUG_ORIENTATION_X_OFFSET,
                     IRIS_PACKET_WITNESS_DEBUG_ORIENTATION_Y_OFFSET,
                     IRIS_PACKET_WITNESS_DEBUG_ORIENTATION_Z_OFFSET,
                     IRIS_PACKET_WITNESS_DEBUG_ORIENTATION_W_OFFSET,
                     attitude_.quaternion());
    {
        float p[3];
        float v[3];
        position_filter_.position(p);
        position_filter_.velocity(v);
        write_float_be(&payload[IRIS_PACKET_WITNESS_DEBUG_POSITION_X_OFFSET], p[0]);
        write_float_be(&payload[IRIS_PACKET_WITNESS_DEBUG_POSITION_Y_OFFSET], p[1]);
        write_float_be(&payload[IRIS_PACKET_WITNESS_DEBUG_POSITION_Z_OFFSET], p[2]);
        write_float_be(&payload[IRIS_PACKET_WITNESS_DEBUG_VELOCITY_X_OFFSET], v[0]);
        write_float_be(&payload[IRIS_PACKET_WITNESS_DEBUG_VELOCITY_Y_OFFSET], v[1]);
        write_float_be(&payload[IRIS_PACKET_WITNESS_DEBUG_VELOCITY_Z_OFFSET], v[2]);
    }
    // MCU temperature, the other estimator outputs and GPS Unix time remain zero.
    return data_forwarder_.queue_packet(
        IRIS_PACKET_ID_WITNESS_DEBUG, payload, sizeof(payload));
}

std::uint16_t Sensors::encode_battery_voltage(const std::uint32_t millivolts)
{
    const std::uint32_t rounded_counts =
        (millivolts + (kBatteryPacketMillivoltsPerCount / 2U)) /
        kBatteryPacketMillivoltsPerCount;
    constexpr std::uint32_t kMaximumEncodedVoltage =
        std::numeric_limits<std::uint16_t>::max();

    return static_cast<std::uint16_t>(
        rounded_counts > kMaximumEncodedVoltage
            ? kMaximumEncodedVoltage
            : rounded_counts);
}

esp_err_t Sensors::read_battery_voltage()
{
    if (battery_adc_handle_ == nullptr ||
        battery_adc_calibration_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    std::int64_t raw_sum = 0;
    for (std::uint32_t sample = 0; sample < kBatteryAdcSamples; ++sample) {
        int raw = 0;
        const esp_err_t result = adc_oneshot_read(
            battery_adc_handle_, kBatteryAdcChannel, &raw);
        if (result != ESP_OK) {
            return result;
        }
        raw_sum += raw;
    }

    const int average_raw =
        static_cast<int>(raw_sum / kBatteryAdcSamples);
    int sense_voltage_mv = 0;
    const esp_err_t result = adc_cali_raw_to_voltage(
        battery_adc_calibration_, average_raw, &sense_voltage_mv);
    if (result != ESP_OK) {
        return result;
    }

    working_sample_.battery_adc_raw = average_raw;
    working_sample_.battery_voltage_mv = static_cast<std::uint32_t>(
        (static_cast<std::uint64_t>(sense_voltage_mv) *
         (kBatteryDividerHighOhms + kBatteryDividerLowOhms)) /
        kBatteryDividerLowOhms);
    return ESP_OK;
}

void Sensors::publish_sample()
{
    if (xSemaphoreTake(sample_mutex_, portMAX_DELAY) == pdTRUE) {
        latest_sample_ = working_sample_;
        sample_available_ = true;
        xSemaphoreGive(sample_mutex_);
    }
}

void Sensors::task_entry(void *context)
{
    static_cast<Sensors *>(context)->run();
}

void Sensors::poll_radio_commands()
{
    if (!radio_ready_ || gpio_get_level(HW_PIN_RAD_DIO1) == 0) {
        return;
    }

    // SX1262 packets are at most 255 bytes. Reception is polled from this task
    // so RX, radio TX, and the sensors remain serialized on SPI3.
    std::array<std::uint8_t, 255> packet{};
    std::size_t packet_length = 0;
    const esp_err_t receive_result = radio_.receive(
        packet.data(), packet.size(), packet_length);

    if (receive_result == ESP_ERR_NOT_FOUND) {
        return;
    }
    if (receive_result == ESP_ERR_INVALID_CRC) {
        ESP_LOGW(kLogTag, "radio packet rejected: CRC error");
        return;
    }
    if (receive_result != ESP_OK) {
        ESP_LOGW(
            kLogTag,
            "radio receive failed: %s",
            esp_err_to_name(receive_result));
        return;
    }
    if (packet_length == 0U) {
        return;
    }

    ESP_LOGI(
        kLogTag,
        "radio RX: %u bytes, packet ID 0x%02X",
        static_cast<unsigned>(packet_length),
        packet[0]);
    ESP_LOG_BUFFER_HEX_LEVEL(
        kLogTag,
        packet.data(),
        packet_length,
        ESP_LOG_INFO);

    // Mirror complete received packets to USB serial without adding framing.
    const esp_err_t serial_result = data_forwarder_.queue_raw_bytes(
        packet.data(), packet_length);
    if (serial_result != ESP_OK) {
        ESP_LOGW(
            kLogTag,
            "could not forward %u-byte radio packet to USB: %s",
            static_cast<unsigned>(packet_length),
            esp_err_to_name(serial_result));
    }

    if (packet[0] == IRIS_PACKET_ID_COMMAND) {
        const esp_err_t command_result =
            command_bridge_.submit_radio_command(
                packet.data(), packet_length);
        if (command_result != ESP_OK) {
            ESP_LOGW(
                kLogTag,
                "radio command bridge failed: %s",
                esp_err_to_name(command_result));
        }
    }
}

void Sensors::service_radio_transmit(const TickType_t now)
{
    if (!radio_transmission_enabled_.load() || !radio_ready_) {
        return;
    }

    if (radio_.transmit_in_progress()) {
        return;  // One packet on the air at a time.
    }

    SpiDataForwarder::RadioPacket packet{};
    if (data_forwarder_.receive_immediate_radio_packet(packet)) {
        const esp_err_t result =
            radio_.start_transmit(packet.data, packet.length);
        ++radio_tx_started_;
        if (result != ESP_OK) {
            ++radio_tx_failed_;
            ESP_LOGE(kLogTag, "immediate radio TX failed: %s",
                     esp_err_to_name(result));
        }
        return;
    }

    for (std::size_t checked = 0; checked < kRadioPacketTypeCount; ++checked) {
        const std::size_t index =
            (next_radio_packet_index_ + checked) % kRadioPacketTypeCount;
        const std::uint32_t rate_hz = kRadioPacketRatesHz[index];
        const TickType_t period = pdMS_TO_TICKS(1000U / rate_hz);
        if ((now - radio_last_sent_[index]) < period) {
            continue;
        }

        std::uint32_t generation = 0;
        if (!data_forwarder_.latest_radio_packet(
                kRadioPacketIds[index], packet, generation)) {
            continue;
        }

        radio_last_sent_[index] = now;
        next_radio_packet_index_ =
            (index + 1U) % kRadioPacketTypeCount;
        const esp_err_t result =
            radio_.start_transmit(packet.data, packet.length);
        ++radio_tx_started_;
        if (result != ESP_OK) {
            ++radio_tx_failed_;
            ESP_LOGE(
                kLogTag,
                "radio TX for packet 0x%02X failed: %s",
                kRadioPacketIds[index],
                esp_err_to_name(result));
        }
        return;
    }
}

bool Sensors::service_radio_transmit_completion()
{
    esp_err_t error = ESP_OK;
    const Ra01::TxStatus status = radio_.poll_transmit(&error);
    if (status == Ra01::TxStatus::Timeout || status == Ra01::TxStatus::Error) {
        ++radio_tx_failed_;
        ESP_LOGE(kLogTag, "radio TX did not complete: %s",
                 esp_err_to_name(error));
    }
    return radio_.transmit_in_progress();
}

esp_err_t Sensors::request_calibration(const std::uint8_t argument)
{
    switch (argument) {
    case IRIS_IMU_CALIBRATION_GYRO:
    case IRIS_IMU_CALIBRATION_FACE:
    case IRIS_IMU_CALIBRATION_CLEAR:
    case IRIS_IMU_CALIBRATION_STATUS:
        break;
    default:
        ESP_LOGW(kLogTag, "IMU calibration command 0x%02X unknown", argument);
        return ESP_ERR_INVALID_ARG;
    }
    if (argument != IRIS_IMU_CALIBRATION_STATUS &&
        flight_state_machine_.state() != FlightStateMachine::State::PadIdle) {
        ESP_LOGW(kLogTag, "IMU calibration rejected: requires pad idle");
        return ESP_ERR_INVALID_STATE;
    }
    if (hitl_active()) {
        ESP_LOGW(kLogTag, "IMU calibration rejected: HITL active");
        return ESP_ERR_INVALID_STATE;
    }
    calibration_request_.store(argument);
    return ESP_OK;
}

std::int16_t Sensors::to_counts(const float value, const float units_per_count)
{
    const float counts = std::round(value / units_per_count);
    if (counts >= 32767.0F) return 32767;
    if (counts <= -32768.0F) return -32768;
    return static_cast<std::int16_t>(counts);
}

void Sensors::output_counts(std::int16_t gyro[3], std::int16_t low_g[3],
                            std::int16_t high_g[3]) const
{
    attitude::Vector3 live_bias{};
    if (!hitl_in_use_ && attitude_.valid()) {
        live_bias = attitude_.gyro_bias_rad_s();
    }
    const float live[3] = {live_bias.x, live_bias.y, live_bias.z};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (hitl_in_use_) {
            // Simulated input is passed through unchanged.
            gyro[axis] = working_sample_.gyro_raw[axis];
            low_g[axis] = working_sample_.low_g_accel_raw[axis];
            high_g[axis] = working_sample_.high_g_accel_raw[axis];
            continue;
        }
        gyro[axis] = to_counts(
            working_sample_.gyro_mdps[axis] - live[axis] / kMilliDegreesToRadians,
            kGyroSensitivityMdps);
        low_g[axis] = to_counts(working_sample_.low_g_accel_mg[axis],
                                kLowGAccelSensitivityMg);
        high_g[axis] = to_counts(working_sample_.high_g_accel_mg[axis],
                                 kHighGAccelSensitivityMg);
    }
}

void Sensors::predict_position(const std::uint64_t timestamp_us)
{
    if (!attitude_.valid()) {
        position_last_predict_us_ = 0;  // no world frame yet
        return;
    }
    const std::uint64_t previous = position_last_predict_us_;
    position_last_predict_us_ = timestamp_us;
    if (previous == 0 || timestamp_us <= previous) {
        return;
    }
    const float dt_s = static_cast<float>(timestamp_us - previous) * 1e-6F;
    // Body specific force (m/s^2) rotated BODY -> ENU by the attitude
    // quaternion: v' = v + 2w (u x v) + 2 u x (u x v).
    constexpr float kMgToMps2 = position::kGravity / 1000.0F;
    const float vx = working_sample_.filtered_accel_mg[0] * kMgToMps2;
    const float vy = working_sample_.filtered_accel_mg[1] * kMgToMps2;
    const float vz = working_sample_.filtered_accel_mg[2] * kMgToMps2;
    const attitude::Quaternion q = attitude_.quaternion();
    const float tx = 2.0F * (q.y * vz - q.z * vy);
    const float ty = 2.0F * (q.z * vx - q.x * vz);
    const float tz = 2.0F * (q.x * vy - q.y * vx);
    const float world[3] = {
        vx + q.w * tx + (q.y * tz - q.z * ty),
        vy + q.w * ty + (q.z * tx - q.x * tz),
        vz + q.w * tz + (q.x * ty - q.y * tx),
    };
    position_filter_.predict(world, dt_s,
                             static_cast<position::Phase>(flight_state_machine_.state()));
}

void Sensors::service_position(const std::uint64_t now_us)
{
    constexpr std::uint64_t kPeriodUs = 1'000'000ULL / kPositionUpdateRateHz;
    if (position_last_update_us_ != 0 && now_us - position_last_update_us_ < kPeriodUs) {
        return;
    }
    position_last_update_us_ =
        position_last_update_us_ == 0 || now_us - position_last_update_us_ > 4 * kPeriodUs
            ? now_us
            : position_last_update_us_ + kPeriodUs;
    if (!position_filter_.initialized()) {
        return;  // no predict yet (attitude not initialized)
    }
    const auto phase = static_cast<position::Phase>(flight_state_machine_.state());
    if (position_baro_pending_) {
        position_baro_pending_ = false;
        // MS5607 pressure is in centi-mbar, i.e. pascals.
        (void)position_filter_.update_baro(
            working_sample_.barometric_altitude_meters,
            static_cast<float>(working_sample_.pressure_centi_mbar), phase);
    }
    const NmeaGps::Fix fix = current_gps_fix();
    if (fix.valid && fix.received_ms != position_last_gps_ms_) {
        position_last_gps_ms_ = fix.received_ms;
        (void)position_filter_.update_gps(fix.latitude_degrees, fix.longitude_degrees,
                                          fix.altitude_meters, fix.satellites, phase);
    }
    position_filter_.update_constraints(phase);
}

void Sensors::report_position(const std::uint64_t now_us)
{
    if (position_report_us_ == 0) {
        position_report_us_ = now_us;
        return;
    }
    if (now_us - position_report_us_ < kAttitudeReportIntervalMs * 1000ULL) {
        return;
    }
    position_report_us_ = now_us;
    float p[3];
    float v[3];
    position_filter_.position(p);
    position_filter_.velocity(v);
    ESP_LOGI(kLogTag,
             "position: ENU %.1f %.1f %.1f m, velocity %.1f %.1f %.1f m/s, Mach %.2f, "
             "alt MSL %.0f m; baro %lu used/%lu gated/%lu rejected, sigma %.1f m; "
             "GPS %lu used/%lu rejected%s; %s",
             static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(p[2]),
             static_cast<double>(v[0]), static_cast<double>(v[1]), static_cast<double>(v[2]),
             static_cast<double>(position_filter_.mach()),
             static_cast<double>(position_filter_.altitude_msl_m()),
             static_cast<unsigned long>(position_filter_.baro_updates()),
             static_cast<unsigned long>(position_filter_.baro_gated_count()),
             static_cast<unsigned long>(position_filter_.baro_rejects()),
             static_cast<double>(position_filter_.last_baro_sigma_m()),
             static_cast<unsigned long>(position_filter_.gps_updates()),
             static_cast<unsigned long>(position_filter_.gps_rejects()),
             position_filter_.gps_origin_set() ? "" : " (no GPS origin)",
             !position_filter_.initialized() ? "waiting for attitude"
             : position_filter_.still()      ? "pad hold"
                                             : "integrating");
}

void Sensors::process_imu_batch()
{
    const bool gyro_ready = working_sample_.gyro_ready;
    const bool low_g_ready = working_sample_.low_g_accel_ready;
    const bool high_g_ready = working_sample_.high_g_accel_ready;
    for (std::size_t i = 0; i < imu_batch_count_; ++i) {
        apply_imu_sample(imu_batch_[i], true);
        filtered_accel_.step(working_sample_.low_g_accel_mg,
                             working_sample_.high_g_accel_mg,
                             working_sample_.filtered_accel_mg);
        working_sample_.high_g_accel_active = filtered_accel_.high_g_active();
        log_imu_sample(imu_batch_[i].timestamp_us, false);
        if (imu_batch_[i].has_gyro) {
            update_attitude(imu_batch_[i].timestamp_us);
        }
        predict_position(imu_batch_[i].timestamp_us);
    }
    working_sample_.gyro_ready = gyro_ready;
    working_sample_.low_g_accel_ready = low_g_ready;
    working_sample_.high_g_accel_ready = high_g_ready;
}

void Sensors::note_packet_queue_result(const char *name, const esp_err_t result,
                                       const TickType_t now)
{
    if (result != ESP_OK) {
        ++packet_queue_failures_;
        if (packet_queue_failures_ - packet_queue_failures_reported_ == 1U &&
            now - packet_queue_last_report_ >= pdMS_TO_TICKS(10000)) {
            ESP_LOGE(kLogTag, "%s packet queue failed: %s (further failures are "
                     "summarized every 10 s)", name, esp_err_to_name(result));
            packet_queue_failures_reported_ = packet_queue_failures_;
            packet_queue_last_report_ = now;
        }
    }
    if (packet_queue_failures_ != packet_queue_failures_reported_ &&
        now - packet_queue_last_report_ >= pdMS_TO_TICKS(10000)) {
        ESP_LOGW(kLogTag, "telemetry packet queue: %lu failures in total",
                 static_cast<unsigned long>(packet_queue_failures_));
        packet_queue_failures_reported_ = packet_queue_failures_;
        packet_queue_last_report_ = now;
    }
}

void Sensors::log_calibration(const char *prefix) const
{
    const imu_calibration::Calibration &c = calibration_;
    if (c.gyro_valid) {
        ESP_LOGI(kLogTag, "%s: gyro bias %.3f %.3f %.3f dps", prefix,
                 static_cast<double>(c.gyro_bias_mdps[0]) / 1000.0,
                 static_cast<double>(c.gyro_bias_mdps[1]) / 1000.0,
                 static_cast<double>(c.gyro_bias_mdps[2]) / 1000.0);
    } else {
        ESP_LOGI(kLogTag, "%s: gyro bias not calibrated", prefix);
    }
    if (c.accel_valid) {
        ESP_LOGI(kLogTag,
                 "%s: low-g offset %.1f %.1f %.1f mg, scale %.4f %.4f %.4f; "
                 "high-g offset %.0f %.0f %.0f mg",
                 prefix, static_cast<double>(c.accel_offset_mg[0]),
                 static_cast<double>(c.accel_offset_mg[1]),
                 static_cast<double>(c.accel_offset_mg[2]),
                 static_cast<double>(c.accel_scale[0]),
                 static_cast<double>(c.accel_scale[1]),
                 static_cast<double>(c.accel_scale[2]),
                 static_cast<double>(c.high_g_offset_mg[0]),
                 static_cast<double>(c.high_g_offset_mg[1]),
                 static_cast<double>(c.high_g_offset_mg[2]));
    } else {
        ESP_LOGI(kLogTag, "%s: accelerometers not calibrated", prefix);
    }
}

void Sensors::service_calibration()
{
    if (calibration_capture_kind_ != CalibrationCapture::None) {
        if (flight_state_machine_.state() != FlightStateMachine::State::PadIdle) {
            calibration_capture_kind_ = CalibrationCapture::None;
            ESP_LOGW(kLogTag, "IMU calibration capture aborted: left pad idle");
        } else if (calibration_capture_.count() >=
                   (calibration_capture_kind_ == CalibrationCapture::Gyro
                        ? kGyroCalibrationSamples
                        : kFaceCalibrationSamples)) {
            finish_calibration_capture();
        }
    }

    const std::uint8_t request = calibration_request_.exchange(0);
    if (request == 0) {
        return;
    }
    if (request == IRIS_IMU_CALIBRATION_STATUS) {
        log_calibration("IMU calibration");
        std::size_t faces = calibration_faces_.face_count();
        ESP_LOGI(kLogTag, "IMU calibration: %u of 6 faces captured this boot",
                 static_cast<unsigned>(faces));
        return;
    }
    if (calibration_capture_kind_ != CalibrationCapture::None) {
        ESP_LOGW(kLogTag, "IMU calibration command ignored: a capture is running");
        return;
    }
    if (request == IRIS_IMU_CALIBRATION_CLEAR) {
        calibration_ = imu_calibration::Calibration{};
        calibration_faces_.reset();
        const esp_err_t result = imu_calibration::erase_async();
        ESP_LOGW(kLogTag, "IMU calibration cleared; readings are uncorrected%s",
                 result == ESP_OK ? "" : " (NVS erase not started)");
        return;
    }
    calibration_capture_.reset();
    calibration_capture_kind_ = request == IRIS_IMU_CALIBRATION_GYRO
                                    ? CalibrationCapture::Gyro
                                    : CalibrationCapture::Face;
    ESP_LOGI(kLogTag, "IMU calibration: capturing %s for %lu ms; keep the board still",
             calibration_capture_kind_ == CalibrationCapture::Gyro ? "gyro bias"
                                                                   : "one face",
             static_cast<unsigned long>(
                 (calibration_capture_kind_ == CalibrationCapture::Gyro
                      ? kGyroCalibrationSamples
                      : kFaceCalibrationSamples) *
                 1000U / kImuSampleRateHz));
}

void Sensors::finish_calibration_capture()
{
    using imu_calibration::Status;
    const CalibrationCapture kind = calibration_capture_kind_;
    calibration_capture_kind_ = CalibrationCapture::None;

    if (kind == CalibrationCapture::Gyro) {
        imu_calibration::Calibration updated = calibration_;
        const Status status = imu_calibration::solve_gyro(
            calibration_capture_, calibration_limits_, kGyroCalibrationSamples / 2U,
            updated);
        if (status != Status::Ok) {
            ESP_LOGW(kLogTag, "IMU gyro calibration failed: %s",
                     imu_calibration::status_name(status));
            return;
        }
        calibration_ = updated;
        // The estimator learned the old bias; restart it on corrected data.
        attitude_.reset();
        log_calibration("IMU gyro calibration done");
        if (imu_calibration::save_async(calibration_) != ESP_OK) {
            ESP_LOGE(kLogTag, "IMU calibration NVS save not started");
        }
        return;
    }

    float low[3]{};
    float high[3]{};
    float sd[3]{};
    calibration_capture_.low_g_mean(low);
    calibration_capture_.high_g_mean(high);
    calibration_capture_.low_g_sd(sd);
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (sd[axis] > calibration_limits_.max_accel_sd_mg) {
            ESP_LOGW(kLogTag, "IMU face capture failed: %s",
                     imu_calibration::status_name(Status::NotStill));
            return;
        }
    }
    std::size_t face = 0;
    const Status status =
        calibration_faces_.add_face(low, high, calibration_limits_, face);
    if (status != Status::Ok) {
        ESP_LOGW(kLogTag, "IMU face capture failed: %s (mean %.0f %.0f %.0f mg)",
                 imu_calibration::status_name(status), static_cast<double>(low[0]),
                 static_cast<double>(low[1]), static_cast<double>(low[2]));
        return;
    }
    char missing[24] = {};
    std::size_t length = 0;
    for (std::size_t f = 0; f < imu_calibration::SixFaceSolver::kFaces; ++f) {
        if (!calibration_faces_.has_face(f)) {
            length += static_cast<std::size_t>(std::snprintf(
                missing + length, sizeof(missing) - length, "%s%s",
                length == 0 ? "" : " ", imu_calibration::face_name(f)));
        }
    }
    ESP_LOGI(kLogTag,
             "IMU face %s captured (low-g %.1f %.1f %.1f mg, high-g %.0f %.0f "
             "%.0f mg); %u of 6%s%s",
             imu_calibration::face_name(face), static_cast<double>(low[0]),
             static_cast<double>(low[1]), static_cast<double>(low[2]),
             static_cast<double>(high[0]), static_cast<double>(high[1]),
             static_cast<double>(high[2]),
             static_cast<unsigned>(calibration_faces_.face_count()),
             length == 0 ? "" : ", missing ", missing);
    if (!calibration_faces_.complete()) {
        return;
    }
    imu_calibration::Calibration updated = calibration_;
    const Status solved = calibration_faces_.solve(calibration_limits_, updated);
    if (solved != Status::Ok) {
        ESP_LOGW(kLogTag, "IMU accelerometer calibration failed: %s; recapture faces",
                 imu_calibration::status_name(solved));
        calibration_faces_.reset();
        return;
    }
    calibration_ = updated;
    calibration_faces_.reset();
    attitude_.reset();
    log_calibration("IMU accelerometer calibration done");
    if (imu_calibration::save_async(calibration_) != ESP_OK) {
        ESP_LOGE(kLogTag, "IMU calibration NVS save not started");
    }
}

void Sensors::log_imu_sample(const std::uint64_t timestamp_us, const bool hitl)
{
    if (imu_log_sink_ == nullptr) return;
    std::uint8_t status = static_cast<std::uint8_t>(flight_state_machine_.state()) &
                          FlashLogger::kImuStateMask;
    if (hitl) status |= FlashLogger::kImuHitlFlag;
    if (hitl || working_sample_.low_g_accel_ready) status |= FlashLogger::kImuLowGFreshFlag;
    if (hitl || working_sample_.high_g_accel_ready) status |= FlashLogger::kImuHighGFreshFlag;
    if (hitl || working_sample_.gyro_ready) status |= FlashLogger::kImuGyroFreshFlag;
    if (working_sample_.high_g_accel_active) status |= FlashLogger::kImuHighGActiveFlag;
    (void)imu_log_sink_->log_imu_record(FlashLogger::make_imu_record(
        static_cast<std::uint32_t>(timestamp_us), working_sample_.low_g_accel_raw,
        working_sample_.high_g_accel_raw, working_sample_.gyro_raw, status));
}

void Sensors::update_attitude(const std::uint64_t timestamp_us)
{
    if (!has_gyro_sample_ || !has_low_g_accel_sample_) {
        return;
    }
    attitude::Step step{};
    step.gyro_rad_s = {working_sample_.gyro_mdps[0] * kMilliDegreesToRadians,
                       working_sample_.gyro_mdps[1] * kMilliDegreesToRadians,
                       working_sample_.gyro_mdps[2] * kMilliDegreesToRadians};
    step.specific_force_g = {working_sample_.low_g_accel_mg[0] / 1000.0F,
                             working_sample_.low_g_accel_mg[1] / 1000.0F,
                             working_sample_.low_g_accel_mg[2] / 1000.0F};
    step.timestamp_us = timestamp_us;
    step.phase =
        static_cast<attitude::FlightPhase>(flight_state_machine_.state());
    const attitude::StepResult result = attitude_.update(step);
    ++attitude_updates_since_report_;

    if (result.initialized_now) {
        const attitude::Vector3 bias = attitude_.gyro_bias_rad_s();
        ESP_LOGI(kLogTag,
                 "attitude initialized: tilt %.2f deg from vertical, gyro bias "
                 "%.3f %.3f %.3f dps",
                 static_cast<double>(attitude_.tilt_from_vertical_deg()),
                 static_cast<double>(bias.x * kRadiansToDegrees),
                 static_cast<double>(bias.y * kRadiansToDegrees),
                 static_cast<double>(bias.z * kRadiansToDegrees));
    }
    if (result.entered_landed) {
        if (attitude_.valid()) {
            const attitude::Quaternion q = attitude_.quaternion();
            ESP_LOGI(kLogTag,
                     "attitude after flight: q xyzw [%.5f %.5f %.5f %.5f], "
                     "tilt %.2f deg from vertical, integrated %lu steps over "
                     "%.3f s (gaps %lu, late %lu)",
                     static_cast<double>(q.x), static_cast<double>(q.y),
                     static_cast<double>(q.z), static_cast<double>(q.w),
                     static_cast<double>(attitude_.tilt_from_vertical_deg()),
                     static_cast<unsigned long>(attitude_.flight_steps()),
                     attitude_.flight_time_s(),
                     static_cast<unsigned long>(attitude_.gaps()),
                     static_cast<unsigned long>(attitude_.late_steps()));
        } else {
            ESP_LOGW(kLogTag,
                     "attitude after flight unavailable: no stationary pad "
                     "window initialized the estimator");
        }
    }
}

void Sensors::report_attitude_rate(const std::uint64_t now_us)
{
    if (attitude_report_start_us_ == 0U) {
        attitude_report_start_us_ = now_us;
        return;
    }
    const std::uint64_t elapsed_us = now_us - attitude_report_start_us_;
    if (elapsed_us < kAttitudeReportIntervalMs * 1000ULL) {
        return;
    }
    const double rate_hz = static_cast<double>(attitude_updates_since_report_) *
                           1e6 / static_cast<double>(elapsed_us);
    const attitude::Quaternion q = attitude_.quaternion();
    ESP_LOGI(kLogTag,
             "attitude: %.1f Hz, max dt %.1f ms, gaps %lu, late %lu, "
             "spikes rejected %lu, windows %lu ok/%lu accel/%lu gyro "
             "rejected, overruns %lu, max pass %lu us, radio TX %lu "
             "(failed %lu), %s, tilt %.1f deg, q xyzw [%.4f %.4f %.4f %.4f]",
             rate_hz, static_cast<double>(attitude_.max_dt_us()) / 1000.0,
             static_cast<unsigned long>(attitude_.gaps()),
             static_cast<unsigned long>(attitude_.late_steps()),
             static_cast<unsigned long>(attitude_.spikes_rejected()),
             static_cast<unsigned long>(attitude_.windows_accepted()),
             static_cast<unsigned long>(attitude_.windows_rejected_accel()),
             static_cast<unsigned long>(attitude_.windows_rejected_gyro()),
             static_cast<unsigned long>(loop_overruns_),
             static_cast<unsigned long>(max_pass_us_),
             static_cast<unsigned long>(radio_tx_started_),
             static_cast<unsigned long>(radio_tx_failed_),
             attitude_.valid() ? (attitude_.landed_hold() ? "held" : "valid")
                               : "not initialized",
             static_cast<double>(attitude_.tilt_from_vertical_deg()),
             static_cast<double>(q.x), static_cast<double>(q.y),
             static_cast<double>(q.z), static_cast<double>(q.w));
    ESP_LOGI(kLogTag,
             "imu fifo: %.1f Hz, overruns %lu, partial slots %lu, "
             "unknown words %lu, max backlog %lu words; filtered accel "
             "%s, %lu switches to high-g, peak %.2f g; core-1 time: FIFO read "
             "%.1f%%, sample processing %.1f%%",

             static_cast<double>(fifo_samples_) * 1e6 /
                 static_cast<double>(elapsed_us),
             static_cast<unsigned long>(fifo_overruns_),
             static_cast<unsigned long>(fifo_partial_slots_),
             static_cast<unsigned long>(fifo_bad_words_),
             static_cast<unsigned long>(fifo_max_words_),
             filtered_accel_.high_g_active() ? "high-g" : "low-g",
             static_cast<unsigned long>(filtered_accel_.switches_to_high()),
             static_cast<double>(filtered_accel_.peak_mg()) / 1000.0,
             100.0 * static_cast<double>(imu_fetch_us_) / static_cast<double>(elapsed_us),
             100.0 * static_cast<double>(imu_process_us_) / static_cast<double>(elapsed_us));
    imu_fetch_us_ = 0;
    imu_process_us_ = 0;
    filtered_accel_.reset_peak();
    fifo_samples_ = 0;
    fifo_max_words_ = 0;
    attitude_updates_since_report_ = 0;
    attitude_report_start_us_ = now_us;
    attitude_.reset_max_dt();
    max_pass_us_ = 0;
}

void Sensors::run()
{
    filtered_accel_.configure(kFilteredAccelCutoffHz,
                              static_cast<float>(1e6 / imu_period_us_));
    hitl_filtered_accel_.configure(0.0F, 0.0F);
    TickType_t next_wake_time = xTaskGetTickCount();
    TickType_t last_sensor_packet_time = next_wake_time;
    TickType_t last_state_packet_time = next_wake_time;
    TickType_t last_debug_packet_time = next_wake_time;
    TickType_t last_ms5607_init_attempt = next_wake_time;
    std::uint32_t consecutive_lsm_errors = 0;
    std::uint32_t consecutive_ms5607_errors = 0;
    std::uint32_t battery_sample_counter = 0;
    std::uint32_t consecutive_battery_errors = 0;

    for (;;) {
        // IMU sampling and the attitude update come first in each 10 ms pass
        // so their timestamps stay regular; radio work follows and no longer
        // blocks (transmissions are started here and completed on later
        // passes).
        const std::uint64_t pass_start_us =
            static_cast<std::uint64_t>(esp_timer_get_time());
        bool hitl_new_sample = false;
        const bool hitl = service_hitl(
            static_cast<std::uint32_t>(pass_start_us / 1000ULL),
            hitl_new_sample);
        const TickType_t now = xTaskGetTickCount();
        // Attitude timestamp: the LSM6 read time, or in HITL the time this
        // pass consumed the 0x0C frame (frames are not timestamped on arrival).
        std::uint64_t imu_timestamp_us = pass_start_us;
        esp_err_t lsm_result = ESP_ERR_NOT_FOUND;
        bool ms5607_updated = false;
        bool battery_updated = false;
        bool barometer_healthy = false;

        if (hitl) {
            // HITL replaces the IMU, barometer and battery; they are not read.
            // Only a new 0x0C sample counts as an update, so the flight state
            // machine and sensor packet see each simulated sample once.
            if (hitl_new_sample) {
                lsm_result = ESP_OK;
                ms5607_updated = true;
                battery_updated = true;
            }
            barometer_healthy = working_sample_.ms5607_ready;
        } else {
            imu_timestamp_us = static_cast<std::uint64_t>(esp_timer_get_time());
            const std::int64_t fetch_start_us = esp_timer_get_time();
            lsm_result = fetch_lsm6dsv320x();
            const std::int64_t process_start_us = esp_timer_get_time();
            if (lsm_result == ESP_OK && imu_batch_count_ > 0) {
                process_imu_batch();
            }
            imu_fetch_us_ += static_cast<std::uint64_t>(process_start_us - fetch_start_us);
            imu_process_us_ += static_cast<std::uint64_t>(esp_timer_get_time() - process_start_us);
            witness_status_.set(
                IRIS_WITNESS_STATUS_LOW_G_ACCEL_READY_MASK,
                working_sample_.low_g_accel_ready);
            witness_status_.set(
                IRIS_WITNESS_STATUS_GYRO_READY_MASK,
                working_sample_.gyro_ready);
            witness_status_.set(
                IRIS_WITNESS_STATUS_IMU_TEMPERATURE_READY_MASK,
                working_sample_.temperature_ready);
            witness_status_.set(
                IRIS_WITNESS_STATUS_HIGH_G_ACCEL_READY_MASK,
                working_sample_.high_g_accel_ready);
            if (lsm_result == ESP_OK) {
                consecutive_lsm_errors = 0;
            } else {
                ++consecutive_lsm_errors;
                if (consecutive_lsm_errors == 1 ||
                    (consecutive_lsm_errors % kTaskRateHz) == 0) {
                    ESP_LOGE(kLogTag, "LSM6DSV320X read failed: %s",
                             esp_err_to_name(lsm_result));
                }
            }

            if (!ms5607_initialized_ &&
                (now - last_ms5607_init_attempt) >= kStatePacketPeriod) {
                last_ms5607_init_attempt = now;
                const esp_err_t initialization_result =
                    ms5607_.initialize(kSpiHost);
                if (initialization_result == ESP_OK) {
                    ms5607_initialized_ = true;
                    consecutive_ms5607_errors = 0;
                    ESP_LOGI(kLogTag, "MS5607 initialization recovered");
                } else {
                    ms5607_.release();
                    ESP_LOGE(kLogTag, "MS5607 initialization retry failed: %s",
                             esp_err_to_name(initialization_result));
                }
            }

            if (ms5607_initialized_) {
                const esp_err_t ms5607_result = ms5607_.poll(ms5607_updated);
                if (ms5607_result == ESP_OK) {
                    consecutive_ms5607_errors = 0;
                } else {
                    ++consecutive_ms5607_errors;
                    if (consecutive_ms5607_errors == 1 ||
                        (consecutive_ms5607_errors % kTaskRateHz) == 0) {
                        ESP_LOGE(kLogTag, "MS5607 read failed: %s",
                                 esp_err_to_name(ms5607_result));
                    }
                }
            }

            if (ms5607_updated) {
                const Ms5607::Sample &barometer = ms5607_.sample();
                working_sample_.pressure_centi_mbar =
                    barometer.pressure_centi_mbar;
                working_sample_.ms5607_temperature_centi_celsius =
                    barometer.temperature_centi_celsius;
                working_sample_.barometric_altitude_meters =
                    barometer.altitude_meters;
                working_sample_.ms5607_ready = barometer.valid;
                witness_status_.set(
                    IRIS_WITNESS_STATUS_BAROMETER_READY_MASK,
                    working_sample_.ms5607_ready);
            }

            ++battery_sample_counter;
            if (battery_sample_counter >= kBatterySampleDivider) {
                battery_sample_counter = 0;
                const esp_err_t battery_result = read_battery_voltage();
                if (battery_result == ESP_OK) {
                    consecutive_battery_errors = 0;
                    battery_updated = true;
                } else {
                    ++consecutive_battery_errors;
                    if (consecutive_battery_errors == 1 ||
                        (consecutive_battery_errors % kBatterySampleRateHz) == 0) {
                        ESP_LOGE(kLogTag, "battery ADC read failed: %s",
                                 esp_err_to_name(battery_result));
                    }
                }
            }
            barometer_healthy = ms5607_initialized_ &&
                                working_sample_.ms5607_ready &&
                                consecutive_ms5607_errors == 0U;
        }

        witness_status_.set_state_flag(
            IRIS_WITNESS_STATUS_GPS_LOCKED_MASK, current_gps_fix().valid);

        if (lsm_result == ESP_OK || ms5607_updated || battery_updated) {
            working_sample_.timestamp_us =
                static_cast<std::uint64_t>(esp_timer_get_time());

            FlightStateMachine::Observation observation{};
            observation.axial_acceleration_valid =
                lsm_result == ESP_OK &&
                (has_high_g_accel_sample_ || has_low_g_accel_sample_);
            // Launch detection uses the filtered acceleration: low-g
            // resolution below 15 g, the high-g sensor above it.
            observation.axial_acceleration_g =
                working_sample_.filtered_accel_mg[0] / 1000.0F;
            observation.low_g_axial_acceleration_valid =
                lsm_result == ESP_OK && has_low_g_accel_sample_;
            observation.low_g_axial_acceleration_g =
                working_sample_.low_g_accel_mg[0] / 1000.0F;
            observation.acceleration_valid =
                lsm_result == ESP_OK && has_low_g_accel_sample_;
            observation.angular_rate_valid =
                lsm_result == ESP_OK && has_gyro_sample_;
            for (std::size_t axis = 0; axis < 3; ++axis) {
                observation.acceleration_g[axis] =
                    working_sample_.low_g_accel_mg[axis] / 1000.0F;
                observation.angular_rate_dps[axis] =
                    working_sample_.gyro_mdps[axis] / 1000.0F;
            }
            observation.barometric_altitude_valid = barometer_healthy;
            observation.barometric_altitude_meters =
                working_sample_.barometric_altitude_meters;
            observation.timestamp_us = working_sample_.timestamp_us;
            flight_state_machine_.update(observation);

            publish_sample();
        }

        // Step the estimator once per fresh gyro sample. A pass that finds no
        // new sample (loop faster than the gyro ODR) must not re-integrate
        // the previous rate; in HITL each new 0x0C frame is one sample.
        // NAND IMU stream: every pass with a new IMU (or HITL) sample. The
        // logger decides what to keep (prelaunch ring, 100 Hz after apogee).
        if (!hitl) {
            service_calibration();
        } else if (calibration_capture_kind_ != CalibrationCapture::None) {
            calibration_capture_kind_ = CalibrationCapture::None;
            ESP_LOGW(kLogTag, "IMU calibration capture aborted: HITL active");
        }

        if (hitl) {
            if (lsm_result == ESP_OK) {
                log_imu_sample(imu_timestamp_us, true);
                update_attitude(imu_timestamp_us);
                predict_position(imu_timestamp_us);
            }
        }
        // Real IMU samples were processed right after the FIFO read
        // (process_imu_batch).
        if (ms5607_updated && barometer_healthy) {
            position_baro_pending_ = true;
        }
        service_position(pass_start_us);

        if (lsm_result == ESP_OK &&
            (now - last_sensor_packet_time) >= kPacketPeriod) {
            note_packet_queue_result("sensor", queue_sensor_packet(), now);
            last_sensor_packet_time = now;
        }

        if ((now - last_state_packet_time) >= kStatePacketPeriod) {
            note_packet_queue_result("state", queue_state_packet(), now);
            last_state_packet_time = now;
        }

        if (debug_packet_output_enabled_.load() &&
            (now - last_debug_packet_time) >= kDebugPacketPeriod) {
            note_packet_queue_result("debug", queue_debug_packet(), now);
            last_debug_packet_time = now;
        }

        if (radio_ready_ && !service_radio_transmit_completion()) {
            poll_radio_commands();
            service_radio_transmit(now);
        }

        const std::uint64_t pass_end_us =
            static_cast<std::uint64_t>(esp_timer_get_time());
        const std::uint64_t pass_us = pass_end_us - pass_start_us;
        if (pass_us > max_pass_us_) {
            max_pass_us_ = static_cast<std::uint32_t>(pass_us);
        }
        report_attitude_rate(pass_end_us);
        report_position(pass_end_us);

        // Deadline scheduling: wake every kTaskPeriod (1/kTaskRateHz)
        // measured from the previous deadline, not from the end of this pass.
        if (xTaskDelayUntil(&next_wake_time, kTaskPeriod) == pdFALSE) {
            ++loop_overruns_;
        }
    }
}

void Sensors::release_resources()
{
    if (battery_adc_calibration_ != nullptr) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_delete_scheme_curve_fitting(battery_adc_calibration_);
#endif
        battery_adc_calibration_ = nullptr;
    }
    if (battery_adc_handle_ != nullptr) {
        adc_oneshot_del_unit(battery_adc_handle_);
        battery_adc_handle_ = nullptr;
    }
    radio_.release();
    radio_ready_ = false;
    data_forwarder_.set_radio_available(false);
    ms5607_.release();
    ms5607_initialized_ = false;
    if (spi_device_ != nullptr) {
        spi_bus_remove_device(spi_device_);
        spi_device_ = nullptr;
    }
    if (fifo_tx_ != nullptr) {
        heap_caps_free(fifo_tx_);
        fifo_tx_ = nullptr;
    }
    if (fifo_rx_ != nullptr) {
        heap_caps_free(fifo_rx_);
        fifo_rx_ = nullptr;
    }
    if (owns_spi_bus_) {
        spi_bus_free(kSpiHost);
        owns_spi_bus_ = false;
    }
    if (sample_mutex_ != nullptr) {
        vSemaphoreDelete(sample_mutex_);
        sample_mutex_ = nullptr;
    }
}
