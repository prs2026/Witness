#include <array>
#include <cstddef>
#include <cstdint>

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ra01.h"

extern "C" void app_main(void)
{
    static constexpr std::size_t kPacketBufferSize = 255U;
    std::array<std::uint8_t, kPacketBufferSize> packet{};

    usb_serial_jtag_driver_config_t usb_config{};
    usb_config.tx_buffer_size = 512U;
    usb_config.rx_buffer_size = 64U;
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));

    Ra01 radio;
    ESP_ERROR_CHECK(radio.initialize());

    for (;;) {
        std::size_t packet_length = 0;
        const esp_err_t result = radio.receive(
            packet.data(), packet.size(), packet_length);
        if (result == ESP_OK && packet_length > 0U) {
            std::size_t written = 0;
            while (written < packet_length) {
                const int count = usb_serial_jtag_write_bytes(
                    packet.data() + written, packet_length - written,
                    pdMS_TO_TICKS(100));
                if (count <= 0) break;
                written += static_cast<std::size_t>(count);
            }
        }
        vTaskDelay(1);
    }
}
