#include "usb_serial_echo.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "mcp23008.h"
#include "sdkconfig.h"

namespace {

constexpr char kLogTag[] = "serial_command";

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

UsbSerialEcho::UsbSerialEcho(Mcp23008 &gpio_expander)
    : gpio_expander_(gpio_expander)
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

    initialized_ = true;
    next_heartbeat_us_ = esp_timer_get_time() + kHeartbeatPeriodUs;
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

    poll_heartbeat(esp_timer_get_time());
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
        const char character = static_cast<char>(data[index]);
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
