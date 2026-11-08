#include "sensors.h"
#include "uart_driver.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "command_bridge.h"
#include <limits>

#include "comms.h"
#include "esp_adc/adc_cali_scheme.h"
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

    if (xTaskCreate(
            task_entry,
            "sensors",
            kTaskStackSize,
            this,
            kTaskPriority,
            &task_handle_) != pdPASS) {
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
    return write_register(kRegCtrl2, kGyroOutputDataRate);
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

    const esp_err_t result = spi_device_transmit(spi_device_, &transaction);
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
    return spi_device_transmit(spi_device_, &transaction);
}

esp_err_t Sensors::write_register(
    const std::uint8_t register_address,
    const std::uint8_t value)
{
    return write_registers(register_address, &value, 1);
}

esp_err_t Sensors::fetch_lsm6dsv320x()
{
    std::uint8_t status = 0;
    esp_err_t result = read_registers(kRegStatus, &status, 1);
    if (result != ESP_OK) {
        return result;
    }

    working_sample_.low_g_accel_ready =
        (status & kStatusLowGAccelReady) != 0;
    working_sample_.gyro_ready = (status & kStatusGyroReady) != 0;
    working_sample_.temperature_ready =
        (status & kStatusTemperatureReady) != 0;
    working_sample_.high_g_accel_ready =
        (status & kStatusHighGAccelReady) != 0;

    if (working_sample_.temperature_ready || working_sample_.gyro_ready ||
        working_sample_.low_g_accel_ready) {
        std::uint8_t output[14]{};
        result = read_registers(kRegOutputStart, output, sizeof(output));
        if (result != ESP_OK) {
            return result;
        }

        if (working_sample_.temperature_ready) {
            working_sample_.temperature_raw = decode_i16(&output[0]);
            working_sample_.temperature_celsius =
                static_cast<float>(working_sample_.temperature_raw) / 256.0F +
                25.0F;
        }
        if (working_sample_.gyro_ready) {
            for (std::size_t axis = 0; axis < 3; ++axis) {
                working_sample_.gyro_raw[axis] =
                    decode_i16(&output[2 + axis * 2]);
                working_sample_.gyro_mdps[axis] =
                    static_cast<float>(working_sample_.gyro_raw[axis]) *
                    kGyroSensitivityMdps;
            }
            has_gyro_sample_ = true;
        }
        if (working_sample_.low_g_accel_ready) {
            for (std::size_t axis = 0; axis < 3; ++axis) {
                working_sample_.low_g_accel_raw[axis] =
                    decode_i16(&output[8 + axis * 2]);
                working_sample_.low_g_accel_mg[axis] =
                    static_cast<float>(working_sample_.low_g_accel_raw[axis]) *
                    kLowGAccelSensitivityMg;
            }
            has_low_g_accel_sample_ = true;
        }
    }

    if (working_sample_.high_g_accel_ready) {
        std::uint8_t output[6]{};
        result = read_registers(kRegHighGOutputStart, output, sizeof(output));
        if (result != ESP_OK) {
            return result;
        }
        for (std::size_t axis = 0; axis < 3; ++axis) {
            working_sample_.high_g_accel_raw[axis] =
                decode_i16(&output[axis * 2]);
            working_sample_.high_g_accel_mg[axis] =
                static_cast<float>(working_sample_.high_g_accel_raw[axis]) *
                kHighGAccelSensitivityMg;
        }
        has_high_g_accel_sample_ = true;
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

    // The estimator does not provide filtered acceleration yet, so the packet's
    // filtered-acceleration slots carry the latest low-g raw samples.
    for (std::size_t axis = 0; axis < IRIS_PACKET_VECTOR_COMPONENTS; ++axis) {
        write_i24_be(
            &payload[IRIS_PACKET_SENSORS_ACCEL_OFFSET +
                     axis * IRIS_PACKET_SENSORS_ACCEL_COMPONENT_LENGTH],
            encode_filtered_acceleration(
                working_sample_.low_g_accel_mg[axis]));
        write_i16_be(
            &payload[IRIS_PACKET_SENSORS_GYRO_OFFSET +
                     axis * IRIS_PACKET_SENSORS_GYRO_COMPONENT_LENGTH],
            working_sample_.gyro_raw[axis]);
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

    for (std::size_t axis = 0; axis < IRIS_PACKET_VECTOR_COMPONENTS; ++axis) {
        write_i24_be(
            &payload[kFilteredAccelOffsets[axis]],
            encode_filtered_acceleration(
                working_sample_.low_g_accel_mg[axis]));
        write_i16_be(
            &payload[kLowAccelOffsets[axis]],
            working_sample_.low_g_accel_raw[axis]);
        write_i16_be(
            &payload[kHighAccelOffsets[axis]],
            working_sample_.high_g_accel_raw[axis]);
        write_i16_be(
            &payload[kGyroOffsets[axis]],
            working_sample_.gyro_raw[axis]);
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
    attitude_updates_since_report_ = 0;
    attitude_report_start_us_ = now_us;
    attitude_.reset_max_dt();
    max_pass_us_ = 0;
}

void Sensors::run()
{
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
            lsm_result = fetch_lsm6dsv320x();
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
            observation.axial_acceleration_g =
                (has_high_g_accel_sample_
                     ? working_sample_.high_g_accel_mg[0]
                     : working_sample_.low_g_accel_mg[0]) /
                1000.0F;
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

        if (lsm_result == ESP_OK) {
            update_attitude(imu_timestamp_us);
        }

        if (lsm_result == ESP_OK &&
            (now - last_sensor_packet_time) >= kPacketPeriod) {
            const esp_err_t queue_result = queue_sensor_packet();
            if (queue_result != ESP_OK) {
                ESP_LOGE(kLogTag, "sensor packet queue failed: %s",
                         esp_err_to_name(queue_result));
            }
            last_sensor_packet_time = now;
        }

        if ((now - last_state_packet_time) >= kStatePacketPeriod) {
            const esp_err_t queue_result = queue_state_packet();
            if (queue_result != ESP_OK) {
                ESP_LOGE(kLogTag, "state packet queue failed: %s",
                         esp_err_to_name(queue_result));
            }
            last_state_packet_time = now;
        }

        if (debug_packet_output_enabled_.load() &&
            (now - last_debug_packet_time) >= kDebugPacketPeriod) {
            const esp_err_t queue_result = queue_debug_packet();
            if (queue_result != ESP_OK) {
                ESP_LOGE(kLogTag, "debug packet queue failed: %s",
                         esp_err_to_name(queue_result));
            }
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

        // Deadline scheduling: wake every kTaskPeriod (5 ticks = 10 ms)
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
    if (owns_spi_bus_) {
        spi_bus_free(kSpiHost);
        owns_spi_bus_ = false;
    }
    if (sample_mutex_ != nullptr) {
        vSemaphoreDelete(sample_mutex_);
        sample_mutex_ = nullptr;
    }
}
