#include "ms5607.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hardware_pins.h"

namespace {

constexpr char kLogTag[] = "ms5607";

}  // namespace

esp_err_t Ms5607::initialize(const spi_host_device_t host)
{
    if (spi_device_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_device_interface_config_t device_config{};
    device_config.mode = kSpiMode;
    device_config.clock_speed_hz = kSpiClockFrequencyHz;
    device_config.spics_io_num = HW_PIN_MS56_CS;
    device_config.queue_size = 1;
    device_config.cs_ena_posttrans = 1;

    esp_err_t result = spi_bus_add_device(host, &device_config, &spi_device_);
    if (result != ESP_OK) {
        return result;
    }

    result = send_command(kResetCommand);
    if (result != ESP_OK) {
        release();
        return result;
    }

    // The RTOS tick can be coarser than the required 2.8 ms PROM reload time.
    vTaskDelay(pdMS_TO_TICKS(kResetDelayMs) + 1U);

    result = read_prom();
    if (result != ESP_OK) {
        release();
        return result;
    }

    result = start_conversion(
        ConversionState::temperature, kTemperatureConversionCommand);
    if (result != ESP_OK) {
        release();
        return result;
    }

    ESP_LOGI(kLogTag, "ready on SPI3, OSR=4096");
    return ESP_OK;
}

esp_err_t Ms5607::poll(bool &sample_updated)
{
    sample_updated = false;
    if (spi_device_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    const std::uint64_t now = static_cast<std::uint64_t>(esp_timer_get_time());
    if ((now - conversion_started_us_) < kConversionTimeUs) {
        return ESP_OK;
    }

    std::uint32_t conversion = 0;
    esp_err_t result = read_adc(conversion);
    if (result != ESP_OK || conversion == 0) {
        const std::uint8_t retry_command =
            conversion_state_ == ConversionState::temperature
                ? kTemperatureConversionCommand
                : kPressureConversionCommand;
        (void)start_conversion(conversion_state_, retry_command);
        return result != ESP_OK ? result : ESP_ERR_INVALID_RESPONSE;
    }

    if (conversion_state_ == ConversionState::temperature) {
        raw_temperature_ = conversion;
        return start_conversion(
            ConversionState::pressure, kPressureConversionCommand);
    }

    raw_pressure_ = conversion;
    compensate();
    result = start_conversion(
        ConversionState::temperature, kTemperatureConversionCommand);
    if (result == ESP_OK) {
        sample_updated = true;
    }
    return result;
}

const Ms5607::Sample &Ms5607::sample() const
{
    return sample_;
}

void Ms5607::release()
{
    if (spi_device_ != nullptr) {
        spi_bus_remove_device(spi_device_);
        spi_device_ = nullptr;
    }
    sample_ = {};
}

std::uint8_t Ms5607::calculate_crc4(const std::uint16_t *prom)
{
    std::uint16_t copy[kPromWordCount]{};
    std::memcpy(copy, prom, sizeof(copy));
    copy[7] &= 0xFF00U;

    std::uint16_t remainder = 0;
    for (std::size_t byte = 0; byte < kPromWordCount * 2U; ++byte) {
        remainder ^= static_cast<std::uint16_t>(
            (byte & 1U) != 0U
                ? copy[byte >> 1U] & 0x00FFU
                : copy[byte >> 1U] >> 8U);

        for (std::uint8_t bit = 0; bit < 8U; ++bit) {
            remainder = static_cast<std::uint16_t>(
                (remainder & 0x8000U) != 0U
                    ? (remainder << 1U) ^ 0x3000U
                    : remainder << 1U);
        }
    }
    return static_cast<std::uint8_t>((remainder >> 12U) & 0x0FU);
}

esp_err_t Ms5607::send_command(const std::uint8_t command)
{
    spi_transaction_t transaction{};
    transaction.length = 8;
    transaction.tx_buffer = &command;
    return spi_device_transmit(spi_device_, &transaction);
}

esp_err_t Ms5607::read_command(
    const std::uint8_t command,
    std::uint8_t *data,
    const std::size_t length)
{
    if (data == nullptr || length == 0 || length > 3U) {
        return ESP_ERR_INVALID_ARG;
    }

    std::uint8_t transmit[4]{command, 0, 0, 0};
    std::uint8_t receive[4]{};
    spi_transaction_t transaction{};
    transaction.length = (length + 1U) * 8U;
    transaction.rxlength = transaction.length;
    transaction.tx_buffer = transmit;
    transaction.rx_buffer = receive;

    const esp_err_t result = spi_device_transmit(spi_device_, &transaction);
    if (result == ESP_OK) {
        std::memcpy(data, &receive[1], length);
    }
    return result;
}

esp_err_t Ms5607::read_prom()
{
    bool all_zero = true;
    bool all_ones = true;
    for (std::size_t index = 0; index < kPromWordCount; ++index) {
        std::uint8_t bytes[2]{};
        const std::uint8_t command = static_cast<std::uint8_t>(
            kPromReadBaseCommand + index * 2U);
        const esp_err_t result = read_command(command, bytes, sizeof(bytes));
        if (result != ESP_OK) {
            return result;
        }

        prom_[index] = static_cast<std::uint16_t>(
            (static_cast<std::uint16_t>(bytes[0]) << 8U) | bytes[1]);
        all_zero = all_zero && prom_[index] == 0U;
        all_ones = all_ones && prom_[index] == 0xFFFFU;
    }

    if (all_zero || all_ones) {
        ESP_LOGE(kLogTag, "invalid calibration PROM contents");
        return ESP_ERR_NOT_FOUND;
    }

    const std::uint8_t stored_crc = static_cast<std::uint8_t>(prom_[7] & 0x0FU);
    const std::uint8_t calculated_crc = calculate_crc4(prom_);
    if (stored_crc != calculated_crc) {
        ESP_LOGE(kLogTag, "PROM CRC mismatch: stored=%u calculated=%u",
                 stored_crc, calculated_crc);
        return ESP_ERR_INVALID_CRC;
    }
    return ESP_OK;
}

esp_err_t Ms5607::read_adc(std::uint32_t &value)
{
    std::uint8_t bytes[3]{};
    const esp_err_t result =
        read_command(kAdcReadCommand, bytes, sizeof(bytes));
    if (result == ESP_OK) {
        value = (static_cast<std::uint32_t>(bytes[0]) << 16U) |
                (static_cast<std::uint32_t>(bytes[1]) << 8U) |
                bytes[2];
    }
    return result;
}

esp_err_t Ms5607::start_conversion(
    const ConversionState state,
    const std::uint8_t command)
{
    const esp_err_t result = send_command(command);
    if (result == ESP_OK) {
        conversion_state_ = state;
        conversion_started_us_ =
            static_cast<std::uint64_t>(esp_timer_get_time());
    }
    return result;
}

void Ms5607::compensate()
{
    const std::int64_t delta_temperature =
        static_cast<std::int64_t>(raw_temperature_) -
        (static_cast<std::int64_t>(prom_[5]) << 8U);

    std::int64_t temperature =
        2000 + ((delta_temperature * prom_[6]) >> 23U);
    std::int64_t offset =
        (static_cast<std::int64_t>(prom_[2]) << 17U) +
        ((static_cast<std::int64_t>(prom_[4]) * delta_temperature) >> 6U);
    std::int64_t sensitivity =
        (static_cast<std::int64_t>(prom_[1]) << 16U) +
        ((static_cast<std::int64_t>(prom_[3]) * delta_temperature) >> 7U);

    if (temperature < 2000) {
        const std::int64_t temperature_delta = temperature - 2000;
        std::int64_t temperature_second_order =
            (delta_temperature * delta_temperature) >> 31U;
        std::int64_t offset_second_order =
            (61 * temperature_delta * temperature_delta) >> 4U;
        std::int64_t sensitivity_second_order =
            2 * temperature_delta * temperature_delta;

        if (temperature < -1500) {
            const std::int64_t very_low_delta = temperature + 1500;
            offset_second_order += 15 * very_low_delta * very_low_delta;
            sensitivity_second_order += 8 * very_low_delta * very_low_delta;
        }

        temperature -= temperature_second_order;
        offset -= offset_second_order;
        sensitivity -= sensitivity_second_order;
    }

    const std::int64_t pressure =
        (((static_cast<std::int64_t>(raw_pressure_) * sensitivity) >> 21U) -
         offset) >> 15U;

    sample_.pressure_raw = raw_pressure_;
    sample_.temperature_raw = raw_temperature_;
    sample_.pressure_centi_mbar = static_cast<std::int32_t>(pressure);
    sample_.temperature_centi_celsius =
        static_cast<std::int32_t>(temperature);

    const float pressure_mbar = static_cast<float>(pressure) / 100.0F;
    if (pressure_mbar > 0.0F) {
        sample_.altitude_meters = 44330.0F *
            (1.0F - std::pow(
                        pressure_mbar / kSeaLevelPressureMbar,
                        0.19029495F));
    }
    sample_.timestamp_us = static_cast<std::uint64_t>(esp_timer_get_time());
    sample_.valid = true;
}
