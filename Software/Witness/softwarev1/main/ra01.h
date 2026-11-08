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

    // Adds the radio to an SPI3 bus initialized by the Sensors task.
    esp_err_t initialize(spi_host_device_t spi_host = SPI3_HOST);
    void release();
    // Blocking transmit (start_transmit + wait). Kept for callers that can
    // afford to wait; the sensor task uses the non-blocking pair below.
    esp_err_t transmit(const std::uint8_t *data, std::size_t length);

    // Non-blocking transmit. start_transmit() loads the packet, issues SetTx
    // and returns (about 0.2 ms of SPI for a 103-byte frame); the airtime
    // (~170 ms at SF7/125 kHz for 103 bytes) elapses while the caller keeps
    // running. poll_transmit() must then be called periodically from the same
    // task: it touches SPI only when DIO1 (TxDone) is high or the deadline has
    // passed, and on completion clears the IRQ and restores continuous RX.
    enum class TxStatus : std::uint8_t { Idle, InProgress, Done, Timeout, Error };
    esp_err_t start_transmit(const std::uint8_t *data, std::size_t length);
    TxStatus poll_transmit(esp_err_t *error = nullptr);
    bool transmit_in_progress() const { return transmit_in_progress_; }
    esp_err_t receive(std::uint8_t *data, std::size_t capacity,
                      std::size_t &length);

private:
    static constexpr std::uint32_t kSpiClockHz = 8000000U;
    static constexpr std::uint32_t kBusyTimeoutMs = 500U;
    static constexpr std::uint32_t kTransmitTimeoutMs = 1000U;
    // BUSY is normally high for microseconds; spin this long before yielding.
    static constexpr std::uint32_t kBusySpinUs = 300U;

    esp_err_t wait_until_ready();
    esp_err_t write_command(std::uint8_t command, const std::uint8_t *data,
                            std::size_t length);
    esp_err_t write_buffer(const std::uint8_t *data, std::size_t length);
    esp_err_t read_command(std::uint8_t command, std::uint8_t *data,
                           std::size_t length);
    esp_err_t read_buffer(std::uint8_t offset, std::uint8_t *data,
                          std::size_t length);

    spi_device_handle_t spi_device_ = nullptr;
    spi_host_device_t spi_host_ = SPI3_HOST;
    bool initialized_ = false;
    bool transmit_in_progress_ = false;
    std::int64_t transmit_deadline_us_ = 0;
};
