#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/spi_master.h"
#include "esp_err.h"

class W25n01kv final {
public:
    static constexpr std::size_t kPageDataSize = 2048;
    // The R variant exposes 64 spare bytes. Its additional 32-byte ECC parity
    // area is managed internally and is not part of the user column range.
    static constexpr std::size_t kSpareSize = 64;
    static constexpr std::size_t kPageTotalSize =
        kPageDataSize + kSpareSize;
    static constexpr std::uint32_t kPagesPerBlock = 64;
    static constexpr std::uint32_t kBlockCount = 1024;
    static constexpr std::uint32_t kPageCount =
        kPagesPerBlock * kBlockCount;

    W25n01kv() = default;
    W25n01kv(const W25n01kv &) = delete;
    W25n01kv &operator=(const W25n01kv &) = delete;

    esp_err_t initialize();
    void release();

    esp_err_t read(
        std::uint32_t page,
        std::uint16_t column,
        std::uint8_t *data,
        std::size_t length);
    esp_err_t program(
        std::uint32_t page,
        std::uint16_t column,
        const std::uint8_t *data,
        std::size_t length);
    esp_err_t erase_block(std::uint32_t block);
    esp_err_t is_bad_block(std::uint32_t block, bool &bad);

private:
    static constexpr spi_host_device_t kSpiHost = SPI2_HOST;
    static constexpr int kSpiClockHz = 10'000'000;
    static constexpr std::uint8_t kSpiMode = 0;
    static constexpr std::uint32_t kReadyTimeoutMs = 20;
    static constexpr std::uint32_t kResetTimeoutMs = 500;

    static constexpr std::uint8_t kCommandReset = 0xFF;
    static constexpr std::uint8_t kCommandReadId = 0x9F;
    static constexpr std::uint8_t kCommandGetFeature = 0x0F;
    static constexpr std::uint8_t kCommandSetFeature = 0x1F;
    static constexpr std::uint8_t kCommandWriteEnable = 0x06;
    static constexpr std::uint8_t kCommandBlockErase = 0xD8;
    static constexpr std::uint8_t kCommandProgramLoad = 0x02;
    static constexpr std::uint8_t kCommandProgramExecute = 0x10;
    static constexpr std::uint8_t kCommandPageDataRead = 0x13;
    static constexpr std::uint8_t kCommandReadData = 0x03;

    static constexpr std::uint8_t kProtectionRegister = 0xA0;
    static constexpr std::uint8_t kConfigurationRegister = 0xB0;
    static constexpr std::uint8_t kStatusRegister = 0xC0;
    static constexpr std::uint8_t kStatusBusy = 1U << 0;
    static constexpr std::uint8_t kStatusWriteEnableLatch = 1U << 1;
    static constexpr std::uint8_t kStatusEraseFailure = 1U << 2;
    static constexpr std::uint8_t kStatusProgramFailure = 1U << 3;
    static constexpr std::uint8_t kStatusEccMask = 3U << 4;
    // ECC=10b is uncorrectable. ECC=11b is corrected data whose bit-flip
    // count exceeded the configured refresh threshold.
    static constexpr std::uint8_t kStatusEccUncorrectable = 2U << 4;
    static constexpr std::uint8_t kConfigurationBufferRead = 1U << 3;
    static constexpr std::uint8_t kConfigurationEccEnable = 1U << 4;
    static constexpr std::uint8_t kBlockProtectionMask = 0x7C;

    static constexpr std::uint8_t kManufacturerId = 0xEF;
    static constexpr std::uint8_t kDeviceIdHigh = 0xAE;
    static constexpr std::uint8_t kDeviceIdLow = 0x21;

    esp_err_t command(std::uint8_t opcode);
    esp_err_t write_enable();
    esp_err_t get_feature(std::uint8_t address, std::uint8_t &value);
    esp_err_t set_feature(std::uint8_t address, std::uint8_t value);
    esp_err_t wait_until_ready(
        std::uint32_t timeout_ms,
        std::uint8_t *final_status = nullptr);
    esp_err_t load_page_to_cache(std::uint32_t page);
    esp_err_t read_cache(
        std::uint16_t column,
        std::uint8_t *data,
        std::size_t length);

    spi_device_handle_t device_ = nullptr;
    bool owns_bus_ = false;
};
