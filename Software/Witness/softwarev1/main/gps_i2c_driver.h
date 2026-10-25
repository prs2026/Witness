#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "px1120s_gps.h"
#include "skytraq_i2c_mailbox.h"

class FlashLogger;
class SpiDataForwarder;

class GpsI2cDriver final {
public:
    static constexpr std::uint32_t kClockHz = 100000;
    static constexpr std::uint8_t kAddress = SkytraqI2cMailbox::kAddress;
    static constexpr UBaseType_t kTaskPriority = 3;
    static constexpr std::uint32_t kTaskStackSize = 4096;
    static constexpr std::uint32_t kTransactionTimeoutMs = 20;
    static constexpr std::uint32_t kReconnectIntervalMs = 1000;
    static constexpr std::uint32_t kCommandTimeoutMs = 2000;
    static constexpr unsigned kMaximumExchangesPerPass = 64;

    explicit GpsI2cDriver(SpiDataForwarder &forwarder) : forwarder_(forwarder) {}
    void set_log_sink(FlashLogger &logger) { logger_ = &logger; }
    esp_err_t start();
    // Queue a complete command; only the GPS task touches the I2C peripheral.
    esp_err_t transmit(const std::uint8_t *data, std::size_t length,
                       TickType_t timeout_ticks = 0);

private:
    static constexpr std::size_t kCommandSize = 128;
    struct Command {
        std::size_t length = 0;
        std::uint8_t data[kCommandSize]{};
    };
    static void task_entry(void *context);
    static bool read_register(void *context, std::uint8_t address, std::uint8_t &value);
    static bool write_register(void *context, std::uint8_t address, std::uint8_t value);
    static bool send_command(void *context, const std::uint8_t *data, std::size_t length);
    bool exchange(std::uint32_t now_ms, bool &progress);
    void flush_usb();
    void disconnect(const char *reason);
    void run();

    SpiDataForwarder &forwarder_;
    FlashLogger *logger_ = nullptr;
    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t device_ = nullptr;
    TaskHandle_t task_ = nullptr;
    QueueHandle_t commands_ = nullptr;
    std::atomic<bool> connected_{false};
    SkytraqI2cMailbox mailbox_{{this, read_register, write_register}};
    // A null baud callback selects the I2C configuration path.
    Px1120sGps gps_{{this, send_command, nullptr}};
    Command transmitting_{};
    std::size_t transmit_offset_ = 0;
    std::uint32_t transmit_started_ms_ = 0;
    bool receive_pending_ = false;
    bool receive_logged_ = false;
    std::size_t receive_length_ = 0;
    std::uint8_t receive_[SkytraqI2cMailbox::kReceiveSize]{};
    std::uint8_t usb_[64]{};
    std::size_t usb_used_ = 0;
};
