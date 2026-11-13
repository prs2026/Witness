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
#include "ground_station_status.h"
#include "hardware.h"
#include "lc86_gps.h"
#include "ra01.h"

namespace {

constexpr const char *kLogTag = "aperture";

Lc86Gps gps;

void make_ground_station_status(std::uint8_t *packet,
                                const std::int8_t rssi_dbm)
{
    const std::uint32_t uptime_ms = static_cast<std::uint32_t>(
        esp_timer_get_time() / 1000LL);
    build_ground_station_status(packet, rssi_dbm, uptime_ms, gps.state());
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

    // The GPS runs on its own task; a failure here leaves the bridge running
    // and the 0xF0 GPS fields at their no-fix values.
    const esp_err_t gps_result = gps.start();
    if (gps_result != ESP_OK) {
        ESP_LOGE(kLogTag, "GPS start failed: %s (0x%x)",
                 esp_err_to_name(gps_result),
                 static_cast<unsigned>(gps_result));
    }

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
                const std::uint16_t opcode = static_cast<std::uint16_t>(
                    (command[7] << 8U) | command[8]);
                if (command.back() == IRIS_PACKET_EOF_VALUE &&
                    opcode == IRIS_COMMAND_RADIO_POWER &&
                    command[1] == IRIS_RADIO_POWER_TARGET_GROUND) {
                    // Ground-station radio power: applied here, acked on
                    // USB (0x06, detail = dBm now in use), never transmitted.
                    const std::uint8_t front_end = command[3];
                    const bool ok = front_end <= IRIS_RADIO_FRONT_END_OFF &&
                                    radio.set_tx_power_dbm(static_cast<std::int8_t>(command[2]));
                    if (ok && front_end != IRIS_RADIO_FRONT_END_UNCHANGED) {
                        radio.set_front_end_enabled(front_end == IRIS_RADIO_FRONT_END_ON);
                    }
                    const std::int16_t now = radio.tx_power_dbm();
                    const bool fem = radio.front_end_enabled();
                    std::array<std::uint8_t, IRIS_PACKET_COMMAND_ACK_FRAME_LENGTH> ack{};
                    ack[0] = IRIS_PACKET_ID_COMMAND_ACK;
                    ack[1] = command[7];
                    ack[2] = command[8];
                    for (std::size_t i = 0; i < 6U; ++i) ack[3U + i] = command[1U + i];
                    ack[9] = ok ? IRIS_ACK_OK : IRIS_ACK_BAD_ARGUMENT;
                    ack[10] = fem ? 1U : 0U;
                    ack[11] = static_cast<std::uint8_t>(now);
                    ack[12] = IRIS_PACKET_EOF_VALUE;
                    (void)usb_serial_jtag_write_bytes(ack.data(), ack.size(), pdMS_TO_TICKS(50));
                    ESP_LOGI(kLogTag, "ground radio TX drive %s: %d dBm, front end %s",
                             ok ? "set" : "refused", static_cast<int>(now), fem ? "on" : "off");
                    command_length = 0U;
                    continue;
                }
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
