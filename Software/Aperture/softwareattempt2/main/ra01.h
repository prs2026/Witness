#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/spi_master.h"
#include "esp_err.h"

class Ra01 final {
public:
    Ra01() = default;
    ~Ra01();

    Ra01(const Ra01 &) = delete;
    Ra01 &operator=(const Ra01 &) = delete;

    esp_err_t initialize();
    esp_err_t transmit(const std::uint8_t *data, std::size_t length);
    esp_err_t receive(std::uint8_t *data, std::size_t capacity,
                      std::size_t &length);
    std::int8_t last_packet_rssi_dbm() const { return last_packet_rssi_dbm_; }

private:
    static constexpr std::uint32_t kSpiClockHz = 8000000U;
    static constexpr std::uint32_t kBusyTimeoutMs = 500U;
    static constexpr std::uint32_t kTransmitTimeoutMs = 1000U;

    esp_err_t wait_until_ready();
    esp_err_t write_command(std::uint8_t command, const std::uint8_t *data,
                            std::size_t length);
    esp_err_t write_buffer(const std::uint8_t *data, std::size_t length);
    esp_err_t read_command(std::uint8_t command, std::uint8_t *data,
                           std::size_t length);
    esp_err_t read_buffer(std::uint8_t offset, std::uint8_t *data,
                          std::size_t length);

    spi_device_handle_t spi_device_ = nullptr;
    bool initialized_ = false;
    std::int8_t last_packet_rssi_dbm_ = -127;
};
