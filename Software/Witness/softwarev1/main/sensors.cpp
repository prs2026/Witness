#include "sensors.h"

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
#include "hardware_pins.h"
#include "spi_data_forwarder.h"
#include "witness_status.h"

namespace {

constexpr char kLogTag[] = "sensors";

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

}  // namespace

Sensors::Sensors(
    SpiDataForwarder &data_forwarder,
    WitnessStatus &witness_status,
    CommandBridge &command_bridge)
    : data_forwarder_(data_forwarder),
      witness_status_(witness_status),
      command_bridge_(command_bridge)
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

    result = radio_.initialize(kSpiHost);
    if (result == ESP_OK) {
        radio_ready_ = true;
    } else {
        // Keep the sensor task alive if the optional radio is unavailable.
        // Initialization is retried at the state-packet rate from run().
        radio_.release();
        ESP_LOGE(kLogTag, "RA-01 initialization failed: %s",
                 esp_err_to_name(result));
    }

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
             "sensor task ready: LSM6DSV320X, MS5607, and RA-01 on SPI3");
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
    result = write_register(kRegCtrl8, kLowGAccelFullScale);
    if (result != ESP_OK) {
        return result;
    }
    result = write_register(kRegCtrl6, kGyroFullScale);
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
        }
        if (working_sample_.low_g_accel_ready) {
            for (std::size_t axis = 0; axis < 3; ++axis) {
                working_sample_.low_g_accel_raw[axis] =
                    decode_i16(&output[8 + axis * 2]);
                working_sample_.low_g_accel_mg[axis] =
                    static_cast<float>(working_sample_.low_g_accel_raw[axis]) *
                    kLowGAccelSensitivityMg;
            }
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

    const esp_err_t queue_result = data_forwarder_.queue_packet(
        IRIS_PACKET_ID_STATE, payload, sizeof(payload));

    esp_err_t radio_result = ESP_ERR_INVALID_STATE;
    if (!radio_ready_) {
        radio_result = radio_.initialize(kSpiHost);
        radio_ready_ = radio_result == ESP_OK;
        if (!radio_ready_) {
            radio_.release();
        }
    }

    if (radio_ready_) {
        std::uint8_t frame[IRIS_PACKET_STATE_FRAME_LENGTH]{};
        frame[0] = IRIS_PACKET_ID_STATE;
        std::memcpy(&frame[IRIS_PACKET_ID_LENGTH], payload, sizeof(payload));
        frame[IRIS_PACKET_ID_LENGTH + sizeof(payload)] =
            IRIS_PACKET_EOF_VALUE;
        radio_result = radio_.transmit(frame, sizeof(frame));
        if (radio_result != ESP_OK) {
            ESP_LOGE(kLogTag, "RA-01 state transmit failed: %s",
                     esp_err_to_name(radio_result));
        }
    }

    return queue_result != ESP_OK ? queue_result : radio_result;
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

    // MCU temperature, estimator outputs, and GPS fields remain zero until
    // their producers are implemented.
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
    // TODO: Poll radio_.receive() here, validate a complete 0x05 command, and
    // pass it to command_bridge_.submit_radio_command(). Keeping this hook in
    // the Sensors task ensures future RX shares SPI3 with the sensor and radio
    // transmit work instead of competing from another task.
    (void)command_bridge_;
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
        poll_radio_commands();

        const esp_err_t lsm_result = fetch_lsm6dsv320x();
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

        const TickType_t now = xTaskGetTickCount();
        bool ms5607_updated = false;
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

        bool battery_updated = false;
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

        if (lsm_result == ESP_OK || ms5607_updated || battery_updated) {
            working_sample_.timestamp_us =
                static_cast<std::uint64_t>(esp_timer_get_time());
            publish_sample();
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

        vTaskDelayUntil(&next_wake_time, kTaskPeriod);
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
