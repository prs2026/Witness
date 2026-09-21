#include <array>
#include <cstddef>
#include <cstdint>

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ra01.h"

namespace {

constexpr std::size_t kRadioPacketBufferSize = 255U;
constexpr std::size_t kUsbTxBufferSize = 512U;
constexpr std::size_t kUsbRxBufferSize = 64U;
constexpr TickType_t kUsbWriteTimeout = pdMS_TO_TICKS(100);

void write_usb(const std::uint8_t *data, const std::size_t length)
{
    std::size_t bytes_written = 0;
    while (bytes_written < length) {
        const int result = usb_serial_jtag_write_bytes(
            data + bytes_written, length - bytes_written, kUsbWriteTimeout);
        if (result <= 0) return;
        bytes_written += static_cast<std::size_t>(result);
    }
}

}  // namespace

extern "C" void app_main(void)
{
    std::array<std::uint8_t, kRadioPacketBufferSize> packet{};

    usb_serial_jtag_driver_config_t usb_config{};
    usb_config.tx_buffer_size = kUsbTxBufferSize;
    usb_config.rx_buffer_size = kUsbRxBufferSize;
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));

    // The USB endpoint carries raw radio payloads; log messages would corrupt
    // that byte stream.
    esp_log_level_set("*", ESP_LOG_NONE);

    Ra01 radio;
    ESP_ERROR_CHECK(radio.initialize());

    for (;;) {
        std::size_t packet_length = 0;
        const esp_err_t result = radio.receive(
            packet.data(), packet.size(), packet_length);
        if (result == ESP_OK && packet_length > 0U) {
            write_usb(packet.data(), packet_length);
        }
        vTaskDelay(1);
    }
}
