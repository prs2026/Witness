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

void make_ground_station_status(std::uint8_t *packet,
                                const std::int8_t rssi_dbm)
{
    packet[0] = IRIS_PACKET_ID_GROUND_STATION_STATUS;
    packet[IRIS_PACKET_GROUND_STATION_STATUS_RSSI_OFFSET + 1U] =
        static_cast<std::uint8_t>(rssi_dbm);
    const std::uint32_t uptime_ms = static_cast<std::uint32_t>(
        esp_timer_get_time() / 1000LL);
    packet[2] = static_cast<std::uint8_t>(uptime_ms >> 24U);
    packet[3] = static_cast<std::uint8_t>(uptime_ms >> 16U);
    packet[4] = static_cast<std::uint8_t>(uptime_ms >> 8U);
    packet[5] = static_cast<std::uint8_t>(uptime_ms);
    for (std::size_t index = 6U; index < 12U; ++index) packet[index] = 0U;
    packet[IRIS_PACKET_GROUND_STATION_STATUS_FRAME_LENGTH - 1U] =
        IRIS_PACKET_EOF_VALUE;
}

}  // namespace

extern "C" void app_main(void)
{
    static constexpr std::size_t kPacketBufferSize =
        IRIS_RADIO_MAX_PACKET_LENGTH;
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

    std::array<std::uint8_t, IRIS_PACKET_COMMAND_FRAME_LENGTH> command{};
    std::size_t command_length = 0U;
    std::array<std::uint8_t,
               IRIS_PACKET_GROUND_STATION_STATUS_FRAME_LENGTH> status_packet{};
    TickType_t next_status = xTaskGetTickCount();

    for (;;) {
        if (xTaskGetTickCount() >= next_status) {
            make_ground_station_status(status_packet.data(),
                                       radio.last_packet_rssi_dbm());
            (void)usb_serial_jtag_write_bytes(
                status_packet.data(), status_packet.size(),
                pdMS_TO_TICKS(100));
            next_status += pdMS_TO_TICKS(1000);
        }

        std::uint8_t usb_input[64]{};
        const int usb_count = usb_serial_jtag_read_bytes(
            usb_input, sizeof(usb_input), 0);
        for (int index = 0; index < usb_count; ++index) {
            const std::uint8_t byte = usb_input[index];

            // The protocol requires USB input to be echoed byte-for-byte.
            (void)usb_serial_jtag_write_bytes(&byte, 1U, 0);

            // Resynchronize at the command packet ID. This bridge sends only
            // complete canonical 0x05 command frames over LoRa.
            if (command_length == 0U) {
                if (byte != IRIS_PACKET_ID_COMMAND) continue;
            }
            command[command_length++] = byte;

            if (command_length == command.size()) {
                if (command.back() == IRIS_PACKET_EOF_VALUE) {
                    ESP_LOGI(kLogTag, "Sending command to Witness");
                    const esp_err_t result = radio.transmit(
                        command.data(), command.size());
                    if (result != ESP_OK) {
                        ESP_LOGE(kLogTag, "Command send failed: %s (0x%x)",
                                 esp_err_to_name(result),
                                 static_cast<unsigned>(result));
                    }
                }
                command_length = 0U;
            }
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
        } else if (result != ESP_ERR_NOT_FOUND) {
            ESP_LOGE(kLogTag, "Receive failed: %s (0x%x)",
                     esp_err_to_name(result),
                     static_cast<unsigned>(result));
        }
        vTaskDelay(1);
    }
}
