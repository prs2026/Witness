#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_err.h"
#include "hardware.h"

class Mcp23008;

class UsbSerialEcho final {
public:
    explicit UsbSerialEcho(Mcp23008 &gpio_expander);

    UsbSerialEcho(const UsbSerialEcho &) = delete;
    UsbSerialEcho &operator=(const UsbSerialEcho &) = delete;

    // Configure the hardware once, then call poll() repeatedly from app_main.
    esp_err_t initialize();
    void poll();

private:
    static constexpr std::size_t kBufferSize = 512;
    static constexpr std::size_t kCommandBufferSize = 256;
    static constexpr gpio_num_t kHeartbeatGpio = IRIS_PIN_LED_RED;
    static constexpr std::int64_t kHeartbeatPeriodUs = 1000000;
    static constexpr std::int64_t kHeartbeatPulseUs = 100000;

    void echo_bytes(const std::uint8_t *data, std::size_t length);
    void consume_command_bytes(const std::uint8_t *data, std::size_t length);
    void process_command();
    bool process_set_command();
    void set_expander_output(std::uint8_t pin, const char *name, bool enabled);
    esp_err_t set_led(bool on);
    void poll_heartbeat(std::int64_t now_us);

    Mcp23008 &gpio_expander_;
    bool initialized_ = false;
    char command_buffer_[kCommandBufferSize]{};
    std::size_t command_length_ = 0;
    bool command_overflow_ = false;
    bool led_on_ = false;
    bool blue_led_on_ = false;
    bool heartbeat_pulse_active_ = false;
    std::int64_t next_heartbeat_us_ = 0;
    std::int64_t heartbeat_pulse_end_us_ = 0;
};
