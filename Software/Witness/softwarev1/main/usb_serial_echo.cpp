#include "usb_serial_echo.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_log.h"
#include "flash_logger.h"
#include "heartbeat.h"
#include "sdkconfig.h"
#include "sensors.h"
#include "spi_data_forwarder.h"

namespace {

constexpr char kLogTag[] = "serial_command";

int hex_nibble(const char character)
{
    if (character >= '0' && character <= '9') {
        return character - '0';
    }
    if (character >= 'A' && character <= 'F') {
        return character - 'A' + 10;
    }
    return -1;
}

bool parse_hex_byte(const char *&cursor, std::uint8_t &value)
{
    while (*cursor == ' ') {
        ++cursor;
    }

    const int high = hex_nibble(cursor[0]);
    const int low = cursor[0] == '\0' ? -1 : hex_nibble(cursor[1]);
    if (high < 0 || low < 0 ||
        (cursor[2] != '\0' && cursor[2] != ' ')) {
        return false;
    }

    value = static_cast<std::uint8_t>((high << 4) | low);
    cursor += 2;
    return true;
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
    Heartbeat &heartbeat,
    Sensors &sensors,
    SpiDataForwarder &spi_interface,
    FlashLogger &flash_logger)
    : heartbeat_(heartbeat),
      sensors_(sensors),
      spi_interface_(spi_interface),
      flash_logger_(flash_logger),
      mass_storage_(flash_logger)
{
}

esp_err_t UsbSerialEcho::start()
{
    if (task_handle_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    usb_serial_jtag_driver_config_t usb_config{};
    usb_config.tx_buffer_size = kBufferSize;
    usb_config.rx_buffer_size = kBufferSize;

    const esp_err_t result = usb_serial_jtag_driver_install(&usb_config);
    if (result != ESP_OK) {
        return result;
    }

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    // The USB console initially uses polling I/O. Route it through the
    // interrupt-driven driver shared with the echo task.
    usb_serial_jtag_vfs_use_driver();
#endif

    const BaseType_t task_created = xTaskCreate(
        task_entry,
        "usb_serial_echo",
        kTaskStackSize,
        this,
        kTaskPriority,
        &task_handle_);

    if (task_created != pdPASS) {
        task_handle_ = nullptr;
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
        usb_serial_jtag_vfs_use_nonblocking();
#endif
        usb_serial_jtag_driver_uninstall();
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void UsbSerialEcho::task_entry(void *context)
{
    static_cast<UsbSerialEcho *>(context)->run();
}

void UsbSerialEcho::echo_bytes(const std::uint8_t *data, const std::size_t length)
{
    std::size_t bytes_echoed = 0;

    while (bytes_echoed < length) {
        const int bytes_written = usb_serial_jtag_write_bytes(
            data + bytes_echoed,
            length - bytes_echoed,
            portMAX_DELAY);

        if (bytes_written > 0) {
            bytes_echoed += static_cast<std::size_t>(bytes_written);
        } else {
            taskYIELD();
        }
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
            if (command_length_ == 3U &&
                static_cast<std::uint8_t>(command_buffer_[0]) == IRIS_PACKET_ID_COMMAND &&
                static_cast<std::uint8_t>(command_buffer_[1]) ==
                    static_cast<std::uint8_t>(IRIS_COMMAND_MASS_STORAGE_START >> 8) &&
                static_cast<std::uint8_t>(command_buffer_[2]) ==
                    static_cast<std::uint8_t>(IRIS_COMMAND_MASS_STORAGE_START)) {
                start_mass_storage();
                return;
            }
        } else {
            command_overflow_ = true;
        }
    }
}

void UsbSerialEcho::process_command()
{
    if (command_length_ == 1U &&
        static_cast<std::uint8_t>(command_buffer_[0]) ==
            IRIS_COMMAND_WITNESS_DEBUG_START) {
        process_protocol_command(IRIS_COMMAND_WITNESS_DEBUG_START);
        return;
    }

    if (normalize_command(command_buffer_, command_length_) == 0) {
        return;
    }

    if (std::strncmp(command_buffer_, "TX ", 3) == 0) {
        process_tx_command();
    } else if (std::strcmp(command_buffer_, "E0") == 0 ||
               std::strcmp(command_buffer_, "0XE0") == 0) {
        process_protocol_command(IRIS_COMMAND_WITNESS_DEBUG_START);
    } else if (std::strcmp(command_buffer_, "LED 1") == 0 ||
        std::strcmp(command_buffer_, "LED ON") == 0) {
        const esp_err_t result = heartbeat_.set_led(true);
        if (result == ESP_OK) {
            ESP_LOGI(kLogTag, "LED ON");
        } else {
            ESP_LOGE(kLogTag, "LED ON failed: %s", esp_err_to_name(result));
        }
    } else if (std::strcmp(command_buffer_, "LED 0") == 0 ||
               std::strcmp(command_buffer_, "LED OFF") == 0) {
        const esp_err_t result = heartbeat_.set_led(false);
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

void UsbSerialEcho::start_mass_storage()
{
    ESP_LOGI(kLogTag, "freezing logs for read-only USB export");
    const esp_err_t freeze_result = flash_logger_.prepare_mass_storage();
    if (freeze_result != ESP_OK) {
        ESP_LOGE(kLogTag, "cannot start mass storage: %s", esp_err_to_name(freeze_result));
        command_length_ = 0;
        return;
    }
    spi_interface_.set_output_enabled(false);
    vTaskDelay(pdMS_TO_TICKS(50));
    flash_logger_.disable_console_output();
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    usb_serial_jtag_vfs_use_nonblocking();
#endif
    usb_serial_jtag_driver_uninstall();
    mass_storage_started_ = mass_storage_.start() == ESP_OK;
}

void UsbSerialEcho::process_protocol_command(const std::uint16_t command)
{
    const esp_err_t result = sensors_.handle_command(command);
    if (result != ESP_OK) {
        ESP_LOGW(kLogTag, "command 0x%04X rejected: %s",
                 static_cast<unsigned>(command), esp_err_to_name(result));
    }
}

void UsbSerialEcho::process_tx_command()
{
    std::uint8_t bytes[
        IRIS_PACKET_ID_LENGTH + IRIS_PACKET_MAX_DATA_LENGTH];
    std::size_t byte_count = 0;
    const char *cursor = command_buffer_ + 3;

    while (*cursor != '\0' && byte_count < sizeof(bytes)) {
        if (!parse_hex_byte(cursor, bytes[byte_count])) {
            ESP_LOGW(kLogTag, "TX requires two-digit hexadecimal bytes");
            return;
        }
        ++byte_count;
    }

    while (*cursor == ' ') {
        ++cursor;
    }

    if (*cursor != '\0' ||
        byte_count < IRIS_PACKET_ID_LENGTH + 6U) {
        ESP_LOGW(kLogTag, "TX requires an ID and 6 to %u payload bytes",
                 static_cast<unsigned>(IRIS_PACKET_MAX_DATA_LENGTH));
        return;
    }

    const esp_err_t result = spi_interface_.queuecommand(
        bytes[0], &bytes[1], byte_count - 1);
    if (result != ESP_OK) {
        ESP_LOGW(kLogTag, "TX command rejected: %s", esp_err_to_name(result));
    }
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

void UsbSerialEcho::run()
{
    std::uint8_t buffer[kBufferSize];

    for (;;) {
        const int bytes_read = usb_serial_jtag_read_bytes(
            buffer, sizeof(buffer), portMAX_DELAY);

        if (bytes_read > 0) {
            const std::size_t bytes_to_echo =
                static_cast<std::size_t>(bytes_read);
            echo_bytes(buffer, bytes_to_echo);
            consume_command_bytes(buffer, bytes_to_echo);
            if (mass_storage_started_) {
                task_handle_ = nullptr;
                vTaskDelete(nullptr);
                return;
            }
        }
    }
}
