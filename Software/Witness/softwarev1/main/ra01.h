#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "hardware_pins.h"

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
    // (~43 ms at SF7/500 kHz for 103 bytes) elapses while the caller keeps
    // running. poll_transmit() must then be called periodically from the same
    // task: it touches SPI only when DIO1 (TxDone) is high or the deadline has
    // passed, and on completion clears the IRQ and restores continuous RX.
    enum class TxStatus : std::uint8_t { Idle, InProgress, Done, Timeout, Error };
    esp_err_t start_transmit(const std::uint8_t *data, std::size_t length);
    TxStatus poll_transmit(esp_err_t *error = nullptr);
    bool transmit_in_progress() const { return transmit_in_progress_; }
    esp_err_t receive(std::uint8_t *data, std::size_t capacity,
                      std::size_t &length);

    // Runtime transmit drive (SX1262 dBm into the Ra-01SH-P front end).
    // Applied from the next transmission. Below -9 dBm the SX1262's low-power
    // PA is used (SetPaConfig deviceSel=1, -17..-10 dBm); -9 and up use the
    // high-power PA. Returns false (and keeps the current value) outside
    // IRIS_RADIO_TX_POWER_MIN_DBM..IRIS_RADIO_TX_POWER_MAX_DBM.
    bool set_tx_power_dbm(int dbm);
    std::int8_t tx_power_dbm() const { return tx_power_dbm_.load(); }
    // RF_EN: the Ra-01SH-P front end (PA + LNA + switch). Off = FEM powered
    // down (datasheet: "RF function turned off"): no PA current, and both the
    // transmitted and received signal pass only attenuated. Bench use only.
    void set_front_end_enabled(bool on);
    bool front_end_enabled() const { return front_end_enabled_.load(); }

private:
    static constexpr std::uint32_t kSpiClockHz = 8000000U;
    static constexpr std::uint32_t kBusyTimeoutMs = 500U;
    static constexpr std::uint32_t kTransmitTimeoutMs = 1000U;
    // BUSY is normally high for microseconds; spin this long before yielding.
    static constexpr std::uint32_t kBusySpinUs = 300U;

    esp_err_t wait_until_ready();
    // SetPaConfig for the PA the current drive needs (high-power PA for
    // >= -9 dBm, low-power PA below). No-op when already configured.
    esp_err_t configure_pa_for(std::int8_t dbm);
    // SX1261/2 erratum 15.1 (TxModulation 0x0889 bit 2 per bandwidth).
    esp_err_t apply_tx_modulation_erratum();
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
    std::atomic<std::int8_t> tx_power_dbm_{static_cast<std::int8_t>(IRIS_RADIO_TX_POWER_DBM)};
    std::atomic<bool> front_end_enabled_{true};
    int pa_low_configured_ = -1;   // -1 unknown, 0 high-power PA, 1 low-power PA
};
