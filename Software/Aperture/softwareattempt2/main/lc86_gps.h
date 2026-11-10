#pragma once

#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lc86_nmea.h"

// Quectel LC86 GPS on the Aperture MCU UART. A dedicated FreeRTOS task resets
// the receiver, configures 10 Hz GGA + RMC output with PAIR commands (PMTK as a
// fallback), then parses NMEA forever. The main loop only takes snapshots, so
// GPS work never blocks the LoRa/USB bridge.
class Lc86Gps final {
public:
    // Configures the reset and PPS pins and starts the GPS task. A failure is
    // returned to the caller; the rest of the firmware does not depend on GPS.
    esp_err_t start();

    // Latest GPS state, copied under a spinlock. Timestamps are esp_timer ms.
    Lc86Nmea::State state() const;

    // PPS rising edges counted since boot, and the esp_timer time of the last
    // one (0 = none). Diagnostic only; not part of any packet.
    std::uint32_t pps_count() const;
    std::int64_t last_pps_us() const;

private:
    static void task_entry(void *context);
    static void pps_isr(void *context);
    void run();
    // Reads the UART for up to wait_ms, feeds the parser and publishes state.
    void pump(std::uint32_t wait_ms);
    bool send_command(const char *body, bool pair, std::uint16_t command,
                      bool not_supported_ok);
    void configure();
    void log_diagnostic(std::uint32_t now_ms);

    Lc86Nmea parser_{};
    Lc86Nmea::State published_{};
    mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
    volatile std::uint32_t pps_count_ = 0;
    volatile std::int64_t last_pps_us_ = 0;
    std::uint32_t rx_bytes_ = 0;
    std::uint32_t last_rx_ms_ = 0;
    TaskHandle_t task_ = nullptr;
};
