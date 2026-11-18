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
    // Blocking: long BUSY waits and delays; use only from start-up or a task
    // that can afford it. In flight, recovery uses begin_reinit() instead.
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

    // ---- Fault handling for the 500 Hz sensor task (2026-11-15 audit F1-F3, F10) ----
    // Real-time mode (flight sensor task only; the maintenance image leaves it
    // off and keeps the old behaviour): a BUSY wait is a short spin, never a
    // sleep, and a BUSY timeout, three SPI failures in a row, or three failed
    // transmissions in a row latch the driver as FAULTED. Every call then
    // returns ESP_ERR_INVALID_STATE at once, so a dead radio costs telemetry
    // but never sensor-task time.
    void set_realtime(bool on) { realtime_ = on; }
    bool faulted() const { return faulted_.load(); }
    std::uint32_t fault_count() const { return fault_count_; }
    std::uint32_t busy_timeout_count() const { return busy_timeouts_; }
    std::uint32_t reinit_attempts() const { return reinit_attempts_; }
    esp_err_t last_fault() const { return last_fault_; }

    // Non-blocking full re-initialisation (hardware reset, then every
    // configuration command, one step per service_reinit() call). Each call
    // does at most one SPI command (under about 0.3 ms) and never waits on
    // BUSY: a busy radio just returns InProgress. The driver stays faulted
    // until Done. Gives up (Failed) after kReinitBudgetMs.
    enum class ReinitStatus : std::uint8_t { Idle, InProgress, Done, Failed };
    void begin_reinit();
    ReinitStatus service_reinit(esp_err_t *error = nullptr);
    bool reinit_active() const { return reinit_step_ >= 0; }

    // Cheap liveness check (one 3-byte SPI read, about 10 us): GetPacketType
    // plus the chip mode from the status byte. A radio that reset by itself
    // comes back in GFSK / STDBY_RC, which is detected here. Lost = settings
    // gone: the driver latches FAULTED so the caller re-initialises.
    // Repaired = idle in standby with the settings intact: RX was re-entered.
    enum class Health : std::uint8_t { Ok, Repaired, Lost, Error };
    Health health_check();

private:
    // Real-time BUSY budget. BUSY is normally high for microseconds (SX1262
    // datasheet: a few us to ~150 us after SetStandby/SetRx/SetTx; the long
    // cases, cold start 3.5 ms and calibration, only occur in Sleep wake-up and
    // Calibrate, which this driver never uses outside init). 2 ms is more than
    // 10x the worst command used in flight.
    static constexpr std::uint32_t kRealtimeBusyUs = 2000U;
    static constexpr unsigned kFaultThreshold = 3U;
    static constexpr std::uint32_t kReinitBudgetMs = 500U;
    static constexpr unsigned kConfigSteps = 12U;

    static constexpr std::uint32_t kSpiClockHz = 8000000U;
    static constexpr std::uint32_t kBusyTimeoutMs = 500U;
    static constexpr std::uint32_t kTransmitTimeoutMs = 1000U;
    // BUSY is normally high for microseconds; spin this long before yielding.
    static constexpr std::uint32_t kBusySpinUs = 300U;

    esp_err_t wait_until_ready();
    // Latches the fault state (first call counts one fault).
    void latch_fault(esp_err_t reason);
    // Tracks SPI results for the consecutive-failure rule.
    esp_err_t track(esp_err_t result);
    // Counts a failed transmission; latches after kFaultThreshold in a row.
    void note_tx_failure(esp_err_t reason);
    // GPIO setup shared by initialize() and the re-init stepper.
    esp_err_t configure_gpio();
    // One configuration command group; initialize() and the re-init stepper
    // both run steps 0..kConfigSteps-1 in order.
    esp_err_t configure_step(unsigned step);
    // Best effort: back to continuous RX after a failed transmit set-up.
    void restore_rx();
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

    bool realtime_ = false;
    std::atomic<bool> faulted_{false};
    bool recovering_ = false;       // re-init stepper running: BUSY timeouts are not new faults
    unsigned spi_failures_ = 0;     // consecutive SPI errors
    unsigned tx_failures_ = 0;      // consecutive failed transmissions
    std::uint32_t fault_count_ = 0;
    std::uint32_t busy_timeouts_ = 0;
    std::uint32_t reinit_attempts_ = 0;
    esp_err_t last_fault_ = ESP_OK;
    // Re-init stepper: -1 idle; 0..2 reset phases; 3.. = 3 + configuration step.
    int reinit_step_ = -1;
    std::int64_t reinit_phase_us_ = 0;
    std::int64_t reinit_deadline_us_ = 0;
};
