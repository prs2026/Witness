#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps_receiver.h"
#include "nmea_gps.h"

class SpiDataForwarder;
class FlashLogger;

class UartDriver final {
public:
    // Initial host rate. GpsReceiver may move the host to the u-blox factory
    // rate while identifying a silent receiver, then back to this rate.
    static constexpr std::uint32_t kBaudRate = GpsReceiver::kOperationalBaud;
    // GPS receivers must not receive the legacy 05 22 startup probe.
    static constexpr bool kStartupLoopbackTestEnabled = false;
    static constexpr UBaseType_t kTaskPriority = 3;
    static constexpr std::uint32_t kTaskStackSize = 4096;
    static constexpr std::uint32_t kTaskTickMs = 2;
    static constexpr std::uint32_t kLoopbackReplyTimeoutMs = 1000;
    static constexpr std::uint32_t kGpsDiagnosticIntervalMs = 5000;

    UartDriver(SpiDataForwarder &serial_forwarder, uart_port_t port,
               gpio_num_t tx_pin, gpio_num_t rx_pin);
    void set_log_sink(FlashLogger &logger) { logger_ = &logger; }
    ~UartDriver();

    UartDriver(const UartDriver &) = delete;
    UartDriver &operator=(const UartDriver &) = delete;

    esp_err_t start();
    esp_err_t shutdown();
    // Thread-safe snapshot of the latest GGA within the fix timeout, otherwise
    // a zero Fix. Position fields are zero unless valid; satellites are kept.
    NmeaGps::Fix latest_gps_fix() const;

    // Hands a verified assistance blob (gps_assist.h) to the GPS task, which
    // writes it to the receiver. Copies the data; any task may call it.
    // ESP_ERR_INVALID_STATE if a previous blob has not been picked up yet.
    esp_err_t submit_gps_assist(const std::uint8_t *blob, std::size_t length);
    // Logs the upload progress (GPS task, on its next pass).
    void request_gps_assist_status() { assist_status_requested_ = true; }
    // Cold-starts the receiver on the GPS task's next pass (logged there).
    void request_gps_cold_start() { cold_start_requested_ = true; }

    // Writes the exact bytes supplied to this UART.
    esp_err_t transmit(
        const std::uint8_t *data,
        std::size_t length,
        TickType_t timeout_ticks = 0);

    // RX line measurement (UART autobaud counters): edges and the shortest
    // low/high pulse over a window, so the bit rate and which pin carries the
    // receiver's TX are known before any byte is decoded.
    struct RxProbe {
        std::uint32_t edges = 0;
        std::uint32_t low_min_cycles = 0;
        std::uint32_t high_min_cycles = 0;
        std::uint32_t sclk_hz = 0;
        std::uint32_t baud_estimate = 0;   // sclk / shortest pulse; 0 if idle
    };
    static constexpr std::uint32_t kProbeWindowMs = 400;
    static constexpr std::uint32_t kProbeMinimumEdges = 40;

private:
    static constexpr std::size_t kDriverBufferSize = 8192;
    RxProbe probe_rx(std::uint32_t window_ms);
    void select_gps_pins();
    static constexpr std::size_t kReadBufferSize = 64;

    static void task_entry(void *context);
    static bool gps_send(void *context, const std::uint8_t *data, std::size_t length);
    static bool gps_set_baud(void *context, std::uint32_t baud);
    static bool gps_swap_pins(void *context);
    esp_err_t initialize();
    esp_err_t send_loopback_request();
    void run();

    SpiDataForwarder &serial_forwarder_;
    FlashLogger *logger_ = nullptr;
    uart_port_t port_;
    gpio_num_t tx_pin_;
    gpio_num_t rx_pin_;
    GpsReceiver gps_{{this, gps_send, gps_set_baud, gps_swap_pins}};
    bool pins_swapped_ = false;   // RX/TX exchanged from the configured pins
    // The UART TX output is routed to a pin only once the GPS has been heard
    // on the RX pin (first transmit), so a harness wired the other way round
    // never has two outputs driving the receiver's TX line.
    bool tx_connected_ = false;
    void listen_on(bool swapped);   // RX only, TX disconnected
    gpio_num_t current_tx() const { return pins_swapped_ ? rx_pin_ : tx_pin_; }
    gpio_num_t current_rx() const { return pins_swapped_ ? tx_pin_ : rx_pin_; }
    mutable portMUX_TYPE fix_lock_ = portMUX_INITIALIZER_UNLOCKED;
    NmeaGps::Fix latest_fix_{};
    mutable portMUX_TYPE assist_lock_ = portMUX_INITIALIZER_UNLOCKED;
    std::uint8_t assist_pending_[gps_assist::kMaximumBlob]{};
    std::size_t assist_pending_length_ = 0;  // nonzero: waiting for the GPS task
    volatile bool assist_status_requested_ = false;
    volatile bool cold_start_requested_ = false;
    TaskHandle_t task_handle_ = nullptr;
    bool initialized_ = false;
};
