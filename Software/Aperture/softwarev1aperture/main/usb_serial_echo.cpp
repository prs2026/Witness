#include "usb_serial_echo.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "comms.h"
#include "mcp23008.h"
#include "pac1931.h"
#include "sdkconfig.h"

namespace {

constexpr char kLogTag[] = "serial_command";
constexpr std::uint8_t kCurrentMonitorErrorFlag = 0x01U;
constexpr std::uint8_t kBatteryMonitorErrorFlag = 0x02U;
constexpr std::uint32_t kMicroampsPerMilliamp = 1000U;

static_assert(IRIS_PACKET_CAMERA_FRAME_LENGTH == 19U);
static_assert(IRIS_PACKET_COMMAND_FRAME_LENGTH == 10U);
static_assert(IRIS_PACKET_IRIS_DEBUG_FRAME_LENGTH == 14U);
static_assert(IRIS_PACKET_EOF_LENGTH == 1U);
static_assert(IRIS_PACKET_COMMAND_VALUE_LENGTH == 2U);
static_assert(
    IRIS_PACKET_CAMERA_BATTERY_VOLTAGE_OFFSET +
        IRIS_PACKET_CAMERA_BATTERY_VOLTAGE_LENGTH ==
    IRIS_PACKET_CAMERA_DATA_LENGTH);
static_assert(IRIS_PACKET_CAMERA_BATTERY_VOLTAGE_LENGTH == 2U);
static_assert(IRIS_PACKET_CAMERA_CURRENT_SENSE_COMPONENT_LENGTH == 1U);
static_assert(IRIS_FIELD_BATTERY_VOLTAGE_MILLIVOLTS_PER_COUNT > 0U);
static_assert(IRIS_FIELD_CURRENT_MILLIAMPS_PER_COUNT > 0U);

void store_u16_be(std::uint8_t *destination, const std::uint16_t value)
{
    destination[0] = static_cast<std::uint8_t>(value >> 8U);
    destination[1] = static_cast<std::uint8_t>(value);
}

void store_u32_be(std::uint8_t *destination, const std::uint32_t value)
{
    destination[0] = static_cast<std::uint8_t>(value >> 24U);
    destination[1] = static_cast<std::uint8_t>(value >> 16U);
    destination[2] = static_cast<std::uint8_t>(value >> 8U);
    destination[3] = static_cast<std::uint8_t>(value);
}

std::uint16_t load_u16_be(const std::uint8_t *source)
{
    return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(source[0]) << 8U) |
        static_cast<std::uint16_t>(source[1]));
}

std::size_t normalize_command(char *command, const std::size_t length)
{
    std::size_t read_index = 0;
    std::size_t write_index = 0;
    bool pending_space = false;

    while (read_index < length &&
           (command[read_index] == ' ' || command[read_index] == '\t')) {
        ++read_index;
    }

    for (; read_index < length; ++read_index) {
        char character = command[read_index];
        if (character == ' ' || character == '\t') {
            pending_space = write_index > 0;
            continue;
        }
        if (pending_space) {
            command[write_index++] = ' ';
            pending_space = false;
        }
        if (character >= 'a' && character <= 'z') {
            character = static_cast<char>(character - ('a' - 'A'));
        }
        command[write_index++] = character;
    }

    command[write_index] = '\0';
    return write_index;
}

}  // namespace

UsbSerialEcho::UsbSerialEcho(
    Mcp23008 &gpio_expander,
    Pac1931 &current_monitor)
    : gpio_expander_(gpio_expander), current_monitor_(current_monitor)
{
}

esp_err_t UsbSerialEcho::initialize()
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    gpio_config_t gpio_settings{};
    gpio_settings.pin_bit_mask = 1ULL << kHeartbeatGpio;
    gpio_settings.mode = GPIO_MODE_OUTPUT;
    gpio_settings.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_settings.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_settings.intr_type = GPIO_INTR_DISABLE;

    esp_err_t result = gpio_config(&gpio_settings);
    if (result != ESP_OK) {
        return result;
    }
    result = set_led(false);
    if (result != ESP_OK) {
        gpio_reset_pin(kHeartbeatGpio);
        return result;
    }

    usb_serial_jtag_driver_config_t usb_config{};
    usb_config.tx_buffer_size = kBufferSize;
    usb_config.rx_buffer_size = kBufferSize;
    result = usb_serial_jtag_driver_install(&usb_config);
    if (result != ESP_OK) {
        gpio_reset_pin(kHeartbeatGpio);
        return result;
    }

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    usb_serial_jtag_vfs_use_driver();
#endif

    // Application logs use the same USB endpoint as the binary protocol.
    // Suppress them after initialization so they cannot split or corrupt a
    // telemetry frame. Received ASCII commands are still echoed explicitly.
    esp_log_level_set("*", ESP_LOG_NONE);

    initialized_ = true;
    const std::int64_t now_us = esp_timer_get_time();
    next_heartbeat_us_ = now_us + kHeartbeatPeriodUs;
    next_current_sample_us_ = now_us + kCurrentSamplePeriodUs;
    next_current_report_us_ = now_us + kCurrentReportPeriodUs;
    next_debug_report_us_ = now_us;
    return ESP_OK;
}

void UsbSerialEcho::poll()
{
    if (!initialized_) {
        return;
    }

    std::uint8_t buffer[kBufferSize];
    const int bytes_read = usb_serial_jtag_read_bytes(buffer, sizeof(buffer), 0);
    if (bytes_read > 0) {
        const std::size_t byte_count = static_cast<std::size_t>(bytes_read);
        echo_bytes(buffer, byte_count);
        consume_command_bytes(buffer, byte_count);
    }

    const std::int64_t now_us = esp_timer_get_time();
    poll_heartbeat(now_us);
    poll_current_monitor(now_us);
    poll_debug_report(now_us);
}

void UsbSerialEcho::update_battery_voltage(
    const std::uint32_t millivolts,
    const bool valid)
{
    if (valid) {
        battery_voltage_millivolts_ = millivolts;
    }
    battery_voltage_valid_ = valid;
}

void UsbSerialEcho::echo_bytes(
    const std::uint8_t *data,
    const std::size_t length)
{
    std::size_t bytes_echoed = 0;
    while (bytes_echoed < length) {
        const int bytes_written = usb_serial_jtag_write_bytes(
            data + bytes_echoed,
            length - bytes_echoed,
            pdMS_TO_TICKS(10));
        if (bytes_written <= 0) {
            ESP_LOGW(kLogTag, "USB transmit buffer full; echo truncated");
            return;
        }
        bytes_echoed += static_cast<std::size_t>(bytes_written);
    }
}

void UsbSerialEcho::consume_command_bytes(
    const std::uint8_t *data,
    const std::size_t length)
{
    for (std::size_t index = 0; index < length; ++index) {
        const std::uint8_t byte = data[index];
        if (binary_command_length_ > 0) {
            binary_command_packet_[binary_command_length_++] = byte;
            if (binary_command_length_ == binary_command_packet_.size()) {
                process_binary_command_packet();
                binary_command_length_ = 0;
            }
            continue;
        }

        if (byte == IRIS_PACKET_ID_COMMAND) {
            // Binary and ASCII commands cannot share a partially completed
            // frame. A packet ID starts a fresh fixed-length binary frame.
            command_length_ = 0;
            command_overflow_ = false;
            binary_command_packet_[0] = byte;
            binary_command_length_ = 1;
            continue;
        }

        const char character = static_cast<char>(byte);
        if (character == '\r' || character == '\n') {
            if (command_overflow_) {
                ESP_LOGW(kLogTag, "command discarded: maximum length is %u bytes",
                         static_cast<unsigned>(kCommandBufferSize - 1));
            } else if (command_length_ > 0) {
                process_command();
            }
            command_length_ = 0;
            command_overflow_ = false;
            continue;
        }

        if (command_overflow_) {
            continue;
        }
        if (command_length_ < kCommandBufferSize - 1) {
            command_buffer_[command_length_++] = character;
        } else {
            command_overflow_ = true;
        }
    }
}

void UsbSerialEcho::process_command()
{
    if (normalize_command(command_buffer_, command_length_) == 0) {
        return;
    }

    if (process_set_command()) {
        return;
    }

    ESP_LOGW(kLogTag, "unknown command: %s", command_buffer_);
}

bool UsbSerialEcho::process_set_command()
{
    if (command_buffer_[0] != 'S') {
        return false;
    }

    // Commands are exactly "S <output> <state>". Outputs 1, 2, 3, and 5
    // control their corresponding enable nets; R and B control the LEDs.
    if (std::strlen(command_buffer_) != 5 ||
        command_buffer_[1] != ' ' || command_buffer_[3] != ' ' ||
        (command_buffer_[4] != '0' && command_buffer_[4] != '1')) {
        ESP_LOGW(kLogTag,
                 "invalid set command; use S {1|2|3|5|R|B} {0|1}");
        return true;
    }

    const bool enabled = command_buffer_[4] == '1';
    std::uint8_t pin = 0;
    const char *name = nullptr;
    switch (command_buffer_[2]) {
        case '1':
            pin = IRIS_MCP23008_PIN_OUT1_ENABLE;
            name = "OUT1_EN";
            break;
        case '2':
            pin = IRIS_MCP23008_PIN_OUT2_ENABLE;
            name = "OUT2_EN";
            break;
        case '3':
            pin = IRIS_MCP23008_PIN_OUT3_ENABLE;
            name = "OUT3_EN";
            break;
        case '5':
            pin = IRIS_MCP23008_PIN_5V_ENABLE;
            name = "5V_EN";
            break;
        case 'B':
            pin = IRIS_MCP23008_PIN_LED_BLUE;
            name = "LED_B";
            break;
        case 'R': {
            const esp_err_t result = set_led(enabled);
            if (result == ESP_OK) {
                ESP_LOGI(kLogTag, "LED_R %s", enabled ? "ON" : "OFF");
            } else {
                ESP_LOGE(kLogTag, "LED_R set failed: %s",
                         esp_err_to_name(result));
            }
            return true;
        }
        default:
            ESP_LOGW(kLogTag,
                     "unknown output; use S {1|2|3|5|R|B} {0|1}");
            return true;
    }

    set_expander_output(pin, name, enabled);
    return true;
}

void UsbSerialEcho::set_expander_output(
    const std::uint8_t pin,
    const char *const name,
    const bool enabled)
{
    const esp_err_t result = gpio_expander_.write_pin(pin, enabled);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "%s set failed: %s",
                 name, esp_err_to_name(result));
        return;
    }

    if (pin == IRIS_MCP23008_PIN_LED_BLUE) {
        blue_led_on_ = enabled;
    }
    ESP_LOGI(kLogTag, "%s %s", name, enabled ? "ON" : "OFF");
}

esp_err_t UsbSerialEcho::set_led(const bool on)
{
    const esp_err_t result = gpio_set_level(kHeartbeatGpio, on);
    if (result == ESP_OK) {
        led_on_ = on;
    }
    return result;
}

void UsbSerialEcho::poll_heartbeat(const std::int64_t now_us)
{
    if (heartbeat_pulse_active_ && now_us >= heartbeat_pulse_end_us_) {
        heartbeat_pulse_active_ = false;
        const esp_err_t result = gpio_set_level(kHeartbeatGpio, led_on_);
        if (result != ESP_OK) {
            ESP_LOGE(kLogTag, "failed to restore heartbeat GPIO: %s",
                     esp_err_to_name(result));
        }

        const esp_err_t blue_result = gpio_expander_.write_pin(
            IRIS_MCP23008_PIN_LED_BLUE, blue_led_on_);
        if (blue_result != ESP_OK) {
            ESP_LOGE(kLogTag, "failed to restore blue heartbeat LED: %s",
                     esp_err_to_name(blue_result));
        }
    }

    if (!heartbeat_pulse_active_ && now_us >= next_heartbeat_us_) {
        const esp_err_t result = gpio_set_level(kHeartbeatGpio, !led_on_);
        if (result != ESP_OK) {
            ESP_LOGE(kLogTag, "failed to pulse heartbeat GPIO: %s",
                     esp_err_to_name(result));
        }

        heartbeat_pulse_active_ = true;
        heartbeat_pulse_end_us_ = now_us + kHeartbeatPulseUs;

        const esp_err_t blue_result = gpio_expander_.write_pin(
            IRIS_MCP23008_PIN_LED_BLUE, !blue_led_on_);
        if (blue_result != ESP_OK) {
            ESP_LOGE(kLogTag, "failed to pulse blue heartbeat LED: %s",
                     esp_err_to_name(blue_result));
        }

        do {
            next_heartbeat_us_ += kHeartbeatPeriodUs;
        } while (next_heartbeat_us_ <= now_us);
    }
}

void UsbSerialEcho::poll_current_monitor(const std::int64_t now_us)
{
    if (now_us >= next_current_sample_us_) {
        Pac1931::CurrentArray currents{};
        const esp_err_t result =
            current_monitor_.read_currents_microamps(currents);
        if (result == ESP_OK) {
            latest_current_microamps_ = currents;
            latest_current_valid_ = true;
            for (std::size_t channel = 0; channel < currents.size(); ++channel) {
                peak_current_microamps_[channel] = std::max(
                    peak_current_microamps_[channel], currents[channel]);
            }
            ++valid_current_samples_;
        } else {
            current_sample_error_ = true;
            latest_current_valid_ = false;
        }

        do {
            next_current_sample_us_ += kCurrentSamplePeriodUs;
        } while (next_current_sample_us_ <= now_us);
    }

    if (now_us >= next_current_report_us_) {
        send_camera_report(now_us);
        peak_current_microamps_.fill(0);
        valid_current_samples_ = 0;
        current_sample_error_ = false;
        do {
            next_current_report_us_ += kCurrentReportPeriodUs;
        } while (next_current_report_us_ <= now_us);
    }
}

void UsbSerialEcho::poll_debug_report(const std::int64_t now_us)
{
    if (!debug_stream_enabled_ || now_us < next_debug_report_us_) {
        return;
    }

    send_iris_debug_report(now_us);
    do {
        next_debug_report_us_ += kDebugReportPeriodUs;
    } while (next_debug_report_us_ <= now_us);
}

void UsbSerialEcho::process_binary_command_packet()
{
    const std::size_t eof_index =
        IRIS_PACKET_ID_LENGTH + IRIS_PACKET_COMMAND_EOF_OFFSET;
    if (binary_command_packet_[0] != IRIS_PACKET_ID_COMMAND ||
        binary_command_packet_[eof_index] != IRIS_PACKET_EOF_VALUE) {
        return;
    }

    const std::uint8_t *const payload =
        binary_command_packet_.data() + IRIS_PACKET_ID_LENGTH;
    const std::uint16_t command = load_u16_be(
        payload + IRIS_PACKET_COMMAND_VALUE_OFFSET);
    (void)execute_binary_command(command);
}

bool UsbSerialEcho::execute_binary_command(const std::uint16_t command)
{
    std::uint8_t pin = 0;
    const char *name = nullptr;
    bool enabled = false;

    switch (command) {
        case IRIS_COMMAND_CH1_LOAD_OFF:
            pin = IRIS_MCP23008_PIN_OUT1_ENABLE;
            name = "OUT1_EN";
            break;
        case IRIS_COMMAND_CH1_LOAD_ON:
            pin = IRIS_MCP23008_PIN_OUT1_ENABLE;
            name = "OUT1_EN";
            enabled = true;
            break;
        case IRIS_COMMAND_CH2_LOAD_OFF:
            pin = IRIS_MCP23008_PIN_OUT2_ENABLE;
            name = "OUT2_EN";
            break;
        case IRIS_COMMAND_CH2_LOAD_ON:
            pin = IRIS_MCP23008_PIN_OUT2_ENABLE;
            name = "OUT2_EN";
            enabled = true;
            break;
        case IRIS_COMMAND_CH3_LOAD_OFF:
            pin = IRIS_MCP23008_PIN_OUT3_ENABLE;
            name = "OUT3_EN";
            break;
        case IRIS_COMMAND_CH3_LOAD_ON:
            pin = IRIS_MCP23008_PIN_OUT3_ENABLE;
            name = "OUT3_EN";
            enabled = true;
            break;
        case IRIS_COMMAND_5V_REGULATOR_OFF:
            pin = IRIS_MCP23008_PIN_5V_ENABLE;
            name = "5V_EN";
            break;
        case IRIS_COMMAND_5V_REGULATOR_ON:
            pin = IRIS_MCP23008_PIN_5V_ENABLE;
            name = "5V_EN";
            enabled = true;
            break;
        case IRIS_COMMAND_IRIS_DEBUG_START:
            debug_stream_enabled_ = true;
            next_debug_report_us_ = esp_timer_get_time();
            return true;
        default:
            return false;
    }

    set_expander_output(pin, name, enabled);
    return true;
}

void UsbSerialEcho::send_camera_report(const std::int64_t now_us)
{
    std::array<std::uint8_t, IRIS_PACKET_CAMERA_FRAME_LENGTH> packet{};
    packet[0] = IRIS_PACKET_ID_CAMERA;
    std::uint8_t *const data = packet.data() + IRIS_PACKET_ID_LENGTH;

    // FC fields are unavailable on Iris. This MCU owns the IRIS/camera fields.
    store_u16_be(data + IRIS_PACKET_CAMERA_FC_STATUS_OFFSET, 0);
    store_u32_be(data + IRIS_PACKET_CAMERA_FC_UPTIME_OFFSET, 0);

    std::uint8_t status_flags = 0;
    if (current_sample_error_ || valid_current_samples_ == 0) {
        status_flags |= kCurrentMonitorErrorFlag;
    }
    if (!battery_voltage_valid_) {
        status_flags |= kBatteryMonitorErrorFlag;
    }
    data[IRIS_PACKET_CAMERA_STATUS_OFFSET] = status_flags;
    data[IRIS_PACKET_CAMERA_STATUS_OFFSET + 1U] = 0;
    store_u32_be(data + IRIS_PACKET_CAMERA_UPTIME_OFFSET,
                 static_cast<std::uint32_t>(now_us / 1000));

    constexpr std::uint32_t kMicroampsPerCurrentCount =
        IRIS_FIELD_CURRENT_MILLIAMPS_PER_COUNT * kMicroampsPerMilliamp;
    for (std::size_t channel = 0;
         channel < IRIS_PACKET_CAMERA_CURRENT_CHANNELS;
         ++channel) {
        const std::uint32_t rounded_counts =
            (peak_current_microamps_[channel] +
             kMicroampsPerCurrentCount / 2U) /
            kMicroampsPerCurrentCount;
        data[IRIS_PACKET_CAMERA_CURRENT_SENSE_OFFSET + channel] =
            static_cast<std::uint8_t>(std::min<std::uint32_t>(
                rounded_counts, std::numeric_limits<std::uint8_t>::max()));
    }

    const std::uint32_t rounded_battery_counts =
        (battery_voltage_millivolts_ +
         IRIS_FIELD_BATTERY_VOLTAGE_MILLIVOLTS_PER_COUNT / 2U) /
        IRIS_FIELD_BATTERY_VOLTAGE_MILLIVOLTS_PER_COUNT;
    const std::uint16_t encoded_battery_voltage =
        static_cast<std::uint16_t>(std::min<std::uint32_t>(
            rounded_battery_counts,
            std::numeric_limits<std::uint16_t>::max()));
    store_u16_be(
        data + IRIS_PACKET_CAMERA_BATTERY_VOLTAGE_OFFSET,
        encoded_battery_voltage);
    data[IRIS_PACKET_CAMERA_EOF_OFFSET] = IRIS_PACKET_EOF_VALUE;

    write_packet(packet.data(), packet.size());
}

void UsbSerialEcho::send_iris_debug_report(const std::int64_t now_us)
{
    std::array<std::uint8_t, IRIS_PACKET_IRIS_DEBUG_FRAME_LENGTH> packet{};
    packet[0] = IRIS_PACKET_ID_IRIS_DEBUG;
    std::uint8_t *const data = packet.data() + IRIS_PACKET_ID_LENGTH;

    std::uint8_t status_flags = 0;
    if (!latest_current_valid_) {
        status_flags |= kCurrentMonitorErrorFlag;
    }
    if (!battery_voltage_valid_) {
        status_flags |= kBatteryMonitorErrorFlag;
    }
    data[IRIS_PACKET_IRIS_DEBUG_STATUS_OFFSET] = status_flags;
    data[IRIS_PACKET_IRIS_DEBUG_STATUS_OFFSET + 1U] = 0;
    store_u32_be(
        data + IRIS_PACKET_IRIS_DEBUG_UPTIME_OFFSET,
        static_cast<std::uint32_t>(now_us / 1000));

    // No ESP32-C3 internal-temperature source is configured yet.
    data[IRIS_PACKET_IRIS_DEBUG_MCU_TEMPERATURE_OFFSET] = 0;

    const std::uint16_t encoded_battery_voltage =
        static_cast<std::uint16_t>(std::min<std::uint32_t>(
            battery_voltage_millivolts_ /
                IRIS_FIELD_BATTERY_VOLTAGE_MILLIVOLTS_PER_COUNT,
            std::numeric_limits<std::uint16_t>::max()));
    store_u16_be(
        data + IRIS_PACKET_IRIS_DEBUG_BATTERY_VOLTAGE_OFFSET,
        encoded_battery_voltage);

    constexpr std::uint32_t kMicroampsPerCurrentCount =
        IRIS_FIELD_CURRENT_MILLIAMPS_PER_COUNT * kMicroampsPerMilliamp;
    const std::size_t current_offsets[] = {
        IRIS_PACKET_IRIS_DEBUG_CH1_CURRENT_OFFSET,
        IRIS_PACKET_IRIS_DEBUG_CH2_CURRENT_OFFSET,
        IRIS_PACKET_IRIS_DEBUG_CH3_CURRENT_OFFSET,
    };
    for (std::size_t channel = 0;
         channel < latest_current_microamps_.size();
         ++channel) {
        const std::uint32_t rounded_counts =
            (latest_current_microamps_[channel] +
             kMicroampsPerCurrentCount / 2U) /
            kMicroampsPerCurrentCount;
        data[current_offsets[channel]] =
            static_cast<std::uint8_t>(std::min<std::uint32_t>(
                rounded_counts, std::numeric_limits<std::uint8_t>::max()));
    }

    data[IRIS_PACKET_IRIS_DEBUG_EOF_OFFSET] = IRIS_PACKET_EOF_VALUE;
    write_packet(packet.data(), packet.size());
}

void UsbSerialEcho::write_packet(
    const std::uint8_t *data,
    const std::size_t length)
{
    std::size_t bytes_written_total = 0;
    while (bytes_written_total < length) {
        const int bytes_written = usb_serial_jtag_write_bytes(
            data + bytes_written_total,
            length - bytes_written_total,
            pdMS_TO_TICKS(10));
        if (bytes_written <= 0) {
            return;
        }
        bytes_written_total += static_cast<std::size_t>(bytes_written);
    }
}
