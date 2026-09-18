#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "comms.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "hardware.h"

class Mcp23008;
class Pac1931;

class UsbSerialEcho final {
public:
    UsbSerialEcho(Mcp23008 &gpio_expander, Pac1931 &current_monitor);

    UsbSerialEcho(const UsbSerialEcho &) = delete;
    UsbSerialEcho &operator=(const UsbSerialEcho &) = delete;

    // Configure the hardware once, then call poll() repeatedly from app_main.
    esp_err_t initialize();
    void update_battery_voltage(std::uint32_t millivolts, bool valid);
    void poll();

private:
    static constexpr std::size_t kBufferSize = 512;
    static constexpr std::size_t kCommandBufferSize = 256;
    static constexpr gpio_num_t kHeartbeatGpio = IRIS_PIN_LED_RED;
    static constexpr std::int64_t kHeartbeatPeriodUs = 1000000;
    static constexpr std::int64_t kHeartbeatPulseUs = 100000;
    static constexpr std::int64_t kCurrentSamplePeriodUs = 100000;
    static constexpr std::int64_t kCurrentReportPeriodUs = 1000000;

    void echo_bytes(const std::uint8_t *data, std::size_t length);
    void consume_command_bytes(const std::uint8_t *data, std::size_t length);
    void process_command();
    bool process_set_command();
    void set_expander_output(std::uint8_t pin, const char *name, bool enabled);
    esp_err_t set_led(bool on);
    void poll_heartbeat(std::int64_t now_us);
    void poll_current_monitor(std::int64_t now_us);
    void send_current_report(std::int64_t now_us);
    void write_packet(const std::uint8_t *data, std::size_t length);

    Mcp23008 &gpio_expander_;
    Pac1931 &current_monitor_;
    bool initialized_ = false;
    char command_buffer_[kCommandBufferSize]{};
    std::size_t command_length_ = 0;
    bool command_overflow_ = false;
    bool led_on_ = false;
    bool blue_led_on_ = false;
    bool heartbeat_pulse_active_ = false;
    std::int64_t next_heartbeat_us_ = 0;
    std::int64_t heartbeat_pulse_end_us_ = 0;
    std::array<std::uint32_t, IRIS_PACKET_CAMERA_CURRENT_CHANNELS>
        peak_current_microamps_{};
    std::uint32_t valid_current_samples_ = 0;
    bool current_sample_error_ = false;
    std::uint32_t battery_voltage_millivolts_ = 0;
    bool battery_voltage_valid_ = false;
    std::int64_t next_current_sample_us_ = 0;
    std::int64_t next_current_report_us_ = 0;
};
