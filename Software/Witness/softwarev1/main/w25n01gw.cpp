#include "w25n01gw.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hardware_pins.h"

namespace {

constexpr char kLogTag[] = "w25n01gw";

esp_err_t transmit(
    const spi_device_handle_t device,
    const void *tx_data,
    void *rx_data,
    const std::size_t length)
{
    spi_transaction_t transaction{};
    transaction.length = length * 8U;
    transaction.rxlength = rx_data == nullptr ? 0 : transaction.length;
    transaction.tx_buffer = tx_data;
    transaction.rx_buffer = rx_data;
    return spi_device_polling_transmit(device, &transaction);
}

}  // namespace

esp_err_t W25n01gw::initialize()
{
    if (device_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_bus_config_t bus_config{};
    bus_config.mosi_io_num = HW_PIN_SPI2_IO0;
    bus_config.miso_io_num = HW_PIN_SPI2_IO1;
    bus_config.sclk_io_num = HW_PIN_SPI2_CLOCK;
    bus_config.quadwp_io_num = HW_PIN_SPI2_IO2;
    bus_config.quadhd_io_num = HW_PIN_SPI2_IO3;
    bus_config.data4_io_num = -1;
    bus_config.data5_io_num = -1;
    bus_config.data6_io_num = -1;
    bus_config.data7_io_num = -1;
    bus_config.max_transfer_sz = kPageTotalSize + 4U;
    bus_config.flags = SPICOMMON_BUSFLAG_MASTER;

    esp_err_t result = spi_bus_initialize(
        kSpiHost, &bus_config, SPI_DMA_CH_AUTO);
    if (result == ESP_OK) {
        owns_bus_ = true;
    } else if (result != ESP_ERR_INVALID_STATE) {
        return result;
    }

    spi_device_interface_config_t device_config{};
    device_config.mode = kSpiMode;
    device_config.clock_speed_hz = kSpiClockHz;
    device_config.spics_io_num = HW_PIN_FLASH_CS;
    device_config.queue_size = 1;

    result = spi_bus_add_device(kSpiHost, &device_config, &device_);
    if (result != ESP_OK) {
        release();
        return result;
    }

    // tPUW is 5 ms maximum before the first write-related instruction.
    vTaskDelay(pdMS_TO_TICKS(5));
    result = command(kCommandReset);
    if (result != ESP_OK) {
        release();
        return result;
    }
    result = wait_until_ready(kResetTimeoutMs);
    if (result != ESP_OK) {
        release();
        return result;
    }

    std::uint8_t tx_id[5] = {kCommandReadId, 0, 0, 0, 0};
    std::uint8_t rx_id[5]{};
    result = transmit(device_, tx_id, rx_id, sizeof(tx_id));
    if (result != ESP_OK) {
        release();
        return result;
    }
    if (rx_id[2] != kManufacturerId ||
        rx_id[3] != kDeviceIdHigh || rx_id[4] != kDeviceIdLow) {
        ESP_LOGE(kLogTag, "unexpected JEDEC ID: %02X %02X %02X",
                 rx_id[2], rx_id[3], rx_id[4]);
        release();
        return ESP_ERR_INVALID_RESPONSE;
    }

    std::uint8_t protection = 0;
    result = get_feature(kProtectionRegister, protection);
    if (result != ESP_OK) {
        release();
        return result;
    }
    // The array powers up protected. Preserve SRP/WP-E and unprotect all blocks.
    result = set_feature(
        kProtectionRegister,
        static_cast<std::uint8_t>(protection & ~kBlockProtectionMask));
    if (result != ESP_OK) {
        release();
        return result;
    }

    std::uint8_t configuration = 0;
    result = get_feature(kConfigurationRegister, configuration);
    if (result != ESP_OK) {
        release();
        return result;
    }
    configuration |= kConfigurationBufferRead | kConfigurationEccEnable;
    result = set_feature(kConfigurationRegister, configuration);
    if (result != ESP_OK) {
        release();
        return result;
    }

    std::uint8_t verified_protection = 0;
    std::uint8_t verified_configuration = 0;
    result = get_feature(kProtectionRegister, verified_protection);
    if (result == ESP_OK) {
        result = get_feature(kConfigurationRegister, verified_configuration);
    }
    if (result != ESP_OK ||
        (verified_protection & kBlockProtectionMask) != 0 ||
        (verified_configuration &
         (kConfigurationBufferRead | kConfigurationEccEnable)) !=
            (kConfigurationBufferRead | kConfigurationEccEnable)) {
        release();
        return result == ESP_OK ? ESP_ERR_INVALID_STATE : result;
    }

    ESP_LOGI(kLogTag,
             "initialized JEDEC EF BA 21, SR1=0x%02X SR2=0x%02X",
             verified_protection, verified_configuration);
    return ESP_OK;
}

void W25n01gw::release()
{
    if (device_ != nullptr) {
        spi_bus_remove_device(device_);
        device_ = nullptr;
    }
    if (owns_bus_) {
        spi_bus_free(kSpiHost);
        owns_bus_ = false;
    }
}

esp_err_t W25n01gw::command(const std::uint8_t opcode)
{
    if (device_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    return transmit(device_, &opcode, nullptr, 1);
}

esp_err_t W25n01gw::get_feature(
    const std::uint8_t address,
    std::uint8_t &value)
{
    if (device_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    const std::uint8_t tx[3] = {kCommandGetFeature, address, 0};
    std::uint8_t rx[3]{};
    const esp_err_t result = transmit(device_, tx, rx, sizeof(tx));
    if (result == ESP_OK) {
        value = rx[2];
    }
    return result;
}

esp_err_t W25n01gw::set_feature(
    const std::uint8_t address,
    const std::uint8_t value)
{
    esp_err_t result = write_enable();
    if (result != ESP_OK) {
        return result;
    }
    const std::uint8_t tx[3] = {kCommandSetFeature, address, value};
    result = transmit(device_, tx, nullptr, sizeof(tx));
    return result == ESP_OK ? wait_until_ready(kReadyTimeoutMs) : result;
}

esp_err_t W25n01gw::write_enable()
{
    esp_err_t result = command(kCommandWriteEnable);
    if (result != ESP_OK) {
        return result;
    }
    std::uint8_t status = 0;
    result = get_feature(kStatusRegister, status);
    if (result != ESP_OK) {
        return result;
    }
    return (status & kStatusWriteEnableLatch) != 0
               ? ESP_OK
               : ESP_ERR_INVALID_STATE;
}

esp_err_t W25n01gw::wait_until_ready(
    const std::uint32_t timeout_ms,
    std::uint8_t *final_status)
{
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
    std::uint8_t status = 0;
    for (;;) {
        const esp_err_t result = get_feature(kStatusRegister, status);
        if (result != ESP_OK) {
            return result;
        }
        if ((status & kStatusBusy) == 0) {
            if (final_status != nullptr) {
                *final_status = status;
            }
            return ESP_OK;
        }
        if ((xTaskGetTickCount() - start) >= timeout) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

esp_err_t W25n01gw::load_page_to_cache(const std::uint32_t page)
{
    if (device_ == nullptr || page >= kPageCount) {
        return ESP_ERR_INVALID_ARG;
    }
    const std::uint8_t tx[4] = {
        kCommandPageDataRead,
        0,
        static_cast<std::uint8_t>(page >> 8),
        static_cast<std::uint8_t>(page),
    };
    esp_err_t result = transmit(device_, tx, nullptr, sizeof(tx));
    std::uint8_t status = 0;
    if (result == ESP_OK) {
        result = wait_until_ready(kReadyTimeoutMs, &status);
    }
    if (result == ESP_OK &&
        (status & kStatusEccMask) >= kStatusEccUncorrectable) {
        return ESP_ERR_INVALID_CRC;
    }
    return result;
}

esp_err_t W25n01gw::read_cache(
    const std::uint16_t column,
    std::uint8_t *data,
    const std::size_t length)
{
    if (device_ == nullptr || data == nullptr || length == 0 ||
        static_cast<std::size_t>(column) + length > kPageTotalSize) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_transaction_ext_t transaction{};
    transaction.base.flags = SPI_TRANS_VARIABLE_CMD |
                             SPI_TRANS_VARIABLE_ADDR |
                             SPI_TRANS_VARIABLE_DUMMY;
    transaction.base.cmd = kCommandReadData;
    transaction.base.addr = column;
    transaction.base.length = length * 8U;
    transaction.base.rxlength = length * 8U;
    transaction.base.rx_buffer = data;
    transaction.command_bits = 8;
    transaction.address_bits = 16;
    transaction.dummy_bits = 8;
    return spi_device_polling_transmit(device_, &transaction.base);
}

esp_err_t W25n01gw::read(
    const std::uint32_t page,
    const std::uint16_t column,
    std::uint8_t *data,
    const std::size_t length)
{
    esp_err_t result = load_page_to_cache(page);
    return result == ESP_OK ? read_cache(column, data, length) : result;
}

esp_err_t W25n01gw::program(
    const std::uint32_t page,
    const std::uint16_t column,
    const std::uint8_t *data,
    const std::size_t length)
{
    if (device_ == nullptr || page >= kPageCount || data == nullptr ||
        length == 0 ||
        static_cast<std::size_t>(column) + length > kPageDataSize) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result = write_enable();
    if (result != ESP_OK) {
        return result;
    }

    spi_transaction_ext_t load{};
    load.base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    load.base.cmd = kCommandProgramLoad;
    load.base.addr = column;
    load.base.length = length * 8U;
    load.base.tx_buffer = data;
    load.command_bits = 8;
    load.address_bits = 16;
    result = spi_device_polling_transmit(device_, &load.base);
    if (result != ESP_OK) {
        return result;
    }

    const std::uint8_t execute[4] = {
        kCommandProgramExecute,
        0,
        static_cast<std::uint8_t>(page >> 8),
        static_cast<std::uint8_t>(page),
    };
    result = transmit(device_, execute, nullptr, sizeof(execute));
    std::uint8_t status = 0;
    if (result == ESP_OK) {
        result = wait_until_ready(kReadyTimeoutMs, &status);
    }
    if (result == ESP_OK && (status & kStatusProgramFailure) != 0) {
        return ESP_FAIL;
    }
    return result;
}

esp_err_t W25n01gw::erase_block(const std::uint32_t block)
{
    if (device_ == nullptr || block >= kBlockCount) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t result = write_enable();
    if (result != ESP_OK) {
        return result;
    }
    const std::uint32_t page = block * kPagesPerBlock;
    const std::uint8_t erase[4] = {
        kCommandBlockErase,
        0,
        static_cast<std::uint8_t>(page >> 8),
        static_cast<std::uint8_t>(page),
    };
    result = transmit(device_, erase, nullptr, sizeof(erase));
    std::uint8_t status = 0;
    if (result == ESP_OK) {
        result = wait_until_ready(kReadyTimeoutMs, &status);
    }
    if (result == ESP_OK && (status & kStatusEraseFailure) != 0) {
        return ESP_FAIL;
    }
    return result;
}

esp_err_t W25n01gw::is_bad_block(
    const std::uint32_t block,
    bool &bad)
{
    if (block >= kBlockCount) {
        return ESP_ERR_INVALID_ARG;
    }
    std::uint8_t marker = 0;
    const esp_err_t result = read(
        block * kPagesPerBlock,
        static_cast<std::uint16_t>(kPageDataSize),
        &marker,
        1);
    if (result == ESP_ERR_INVALID_CRC) {
        // A factory-bad block can also report uncorrectable ECC while its
        // marker page is loaded. Treat it as unusable rather than aborting
        // initialization.
        bad = true;
        return ESP_OK;
    }
    if (result == ESP_OK) {
        bad = marker != 0xFF;
    }
    return result;
}
