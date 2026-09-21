#include <array>
#include <cstddef>
#include <cstdint>

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "comms.h"
#include "hardware.h"
#include "ra01.h"

namespace {

constexpr const char *kLogTag = "aperture";

void store_u32_be(std::uint8_t *data, const std::uint32_t value)
{
    data[0] = static_cast<std::uint8_t>(value >> 24U);
    data[1] = static_cast<std::uint8_t>(value >> 16U);
    data[2] = static_cast<std::uint8_t>(value >> 8U);
    data[3] = static_cast<std::uint8_t>(value);
}

void make_heartbeat(std::uint8_t *packet, const bool heartbeat_state)
{
    packet[0] = IRIS_PACKET_ID_HEARTBEAT;
    constexpr std::size_t data_offset = IRIS_PACKET_ID_LENGTH;
    packet[data_offset + IRIS_PACKET_HEARTBEAT_STATUS_OFFSET] =
        heartbeat_state ? IRIS_WITNESS_STATUS_HEARTBEAT_MASK : 0U;
    packet[data_offset + IRIS_PACKET_HEARTBEAT_STATUS_OFFSET + 1U] =
        IRIS_WITNESS_STATUS_RESERVED_BYTE_VALUE;
    const std::uint32_t uptime_ms = static_cast<std::uint32_t>(
        esp_timer_get_time() / 1000LL);
    store_u32_be(packet + data_offset + IRIS_PACKET_HEARTBEAT_UPTIME_OFFSET,
                 uptime_ms);
    packet[data_offset + IRIS_PACKET_HEARTBEAT_EOF_OFFSET] =
        IRIS_PACKET_EOF_VALUE;
}

}  // namespace

extern "C" void app_main(void)
{
    static constexpr std::size_t kPacketBufferSize = 255U;
    std::array<std::uint8_t, kPacketBufferSize> packet{};

    usb_serial_jtag_driver_config_t usb_config{};
    usb_config.tx_buffer_size = 512U;
    // ESP-IDF requires the RX buffer to be larger than 64 bytes.
    usb_config.rx_buffer_size = 128U;
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));

    Ra01 radio;
    const esp_err_t radio_result = radio.initialize();
    if (radio_result != ESP_OK) {
        // Keep the board alive and report the failure instead of aborting and
        // rebooting. This message is emitted once per skipped heartbeat.
        ESP_LOGE(kLogTag, "Radio initialization failed: %s (0x%x)",
                 esp_err_to_name(radio_result),
                 static_cast<unsigned>(radio_result));
        for (;;) {
            ESP_LOGE(kLogTag, "Radio is not initialized; heartbeat skipped");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_LOGI(kLogTag, "Radio initialized; transmit power is %d dBm",
             IRIS_RADIO_TX_POWER_DBM);

    std::array<std::uint8_t, IRIS_PACKET_HEARTBEAT_FRAME_LENGTH> heartbeat{};
    TickType_t next_heartbeat = xTaskGetTickCount();
    bool heartbeat_state = false;

    for (;;) {
        const TickType_t now = xTaskGetTickCount();
        if (now >= next_heartbeat) {
            make_heartbeat(heartbeat.data(), heartbeat_state);
            ESP_LOGI(kLogTag, "Sending heartbeat (%u bytes)",
                     static_cast<unsigned>(heartbeat.size()));
            const esp_err_t transmit_result =
                radio.transmit(heartbeat.data(), heartbeat.size());
            if (transmit_result != ESP_OK) {
                ESP_LOGE(kLogTag, "Heartbeat send failed: %s (0x%x)",
                         esp_err_to_name(transmit_result),
                         static_cast<unsigned>(transmit_result));
            } else {
                ESP_LOGI(kLogTag, "Heartbeat sent");
            }
            heartbeat_state = !heartbeat_state;
            next_heartbeat += pdMS_TO_TICKS(1000);
        }

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
