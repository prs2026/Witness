#include "usb_serial_echo.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "comms.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
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

    if (std::strcmp(command_buffer_, "LED 1") == 0 ||
        std::strcmp(command_buffer_, "LED ON") == 0) {
        const esp_err_t result = set_led(true);
        if (result == ESP_OK) {
            ESP_LOGI(kLogTag, "LED ON");
        } else {
            ESP_LOGE(kLogTag, "LED ON failed: %s", esp_err_to_name(result));
        }
    } else if (std::strcmp(command_buffer_, "LED 0") == 0 ||
               std::strcmp(command_buffer_, "LED OFF") == 0) {
        const esp_err_t result = set_led(false);
        if (result == ESP_OK) {
            ESP_LOGI(kLogTag, "LED OFF");
        } else {
            ESP_LOGE(kLogTag, "LED OFF failed: %s", esp_err_to_name(result));
        }
    } else if (std::strcmp(command_buffer_, "CAM 1 ON") == 0) {
        set_camera_state(0, true);
    } else if (std::strcmp(command_buffer_, "CAM 1 OFF") == 0) {
        set_camera_state(0, false);
    } else if (std::strcmp(command_buffer_, "CAM 2 ON") == 0) {
        set_camera_state(1, true);
    } else if (std::strcmp(command_buffer_, "CAM 2 OFF") == 0) {
        set_camera_state(1, false);
    } else {
        ESP_LOGW(kLogTag, "unknown command: %s", command_buffer_);
    }
}

esp_err_t UsbSerialEcho::set_led(const bool on)
{
    const esp_err_t result = gpio_set_level(kHeartbeatGpio, on);
    if (result == ESP_OK) {
        led_on_ = on;
    }
    return result;
}

void UsbSerialEcho::set_camera_state(
    const std::size_t camera_index,
    const bool enabled)
{
    camera_enabled_[camera_index] = enabled;
    ESP_LOGI(kLogTag, "CAM %u %s",
             static_cast<unsigned>(camera_index + 1),
             enabled ? "ON" : "OFF");
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

    }

    if (!heartbeat_pulse_active_ && now_us >= next_heartbeat_us_) {
        const esp_err_t result = gpio_set_level(kHeartbeatGpio, !led_on_);
        if (result != ESP_OK) {
            ESP_LOGE(kLogTag, "failed to pulse heartbeat GPIO: %s",
                     esp_err_to_name(result));
        }

        heartbeat_pulse_active_ = true;
        heartbeat_pulse_end_us_ = now_us + kHeartbeatPulseUs;

        do {
            next_heartbeat_us_ += kHeartbeatPeriodUs;
        } while (next_heartbeat_us_ <= now_us);
    }
}
