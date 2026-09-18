#include "sensors.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "esp_log.h"
#include "esp_timer.h"
#include "hardware_pins.h"

namespace {

constexpr char kLogTag[] = "sensors";

}  // namespace

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
             "LSM6DSV320X ready on SPI3: low-g 120 Hz, gyro 120 Hz, high-g 960 Hz");
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
    bus_config.max_transfer_sz = kMaximumRegisterTransfer + 1;
    bus_config.flags = SPICOMMON_BUSFLAG_MASTER;

    esp_err_t result = spi_bus_initialize(
        kSpiHost, &bus_config, SPI_DMA_DISABLED);
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

    working_sample_.timestamp_us =
        static_cast<std::uint64_t>(esp_timer_get_time());
    publish_sample();
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

void Sensors::run()
{
    TickType_t next_wake_time = xTaskGetTickCount();
    std::uint32_t consecutive_errors = 0;

    for (;;) {
        const esp_err_t result = fetch_lsm6dsv320x();
        if (result == ESP_OK) {
            consecutive_errors = 0;
        } else {
            ++consecutive_errors;
            if (consecutive_errors == 1 ||
                (consecutive_errors % kTaskRateHz) == 0) {
                ESP_LOGE(kLogTag, "LSM6DSV320X read failed: %s",
                         esp_err_to_name(result));
            }
        }

        vTaskDelayUntil(&next_wake_time, kTaskPeriod);
    }
}

void Sensors::release_resources()
{
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
