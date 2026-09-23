#include "spi_data_forwarder.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "driver/usb_serial_jtag.h"

esp_err_t SpiDataForwarder::start()
{
    if (forward_queue_ != nullptr || command_queue_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    forward_queue_ = xQueueCreate(kQueueDepth, sizeof(Packet));
    command_queue_ = xQueueCreate(kQueueDepth, sizeof(Packet));
    if (forward_queue_ == nullptr || command_queue_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(
            forward_task_entry,
            "data_forwarder",
            kTaskStackSize,
            this,
            kForwardTaskPriority,
            &forward_task_handle_) != pdPASS) {
        forward_task_handle_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(
            spi_task_entry,
            "spi_interface",
            kTaskStackSize,
            this,
            kSpiTaskPriority,
            &spi_task_handle_) != pdPASS) {
        spi_task_handle_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void SpiDataForwarder::set_output_enabled(const bool enabled)
{
    output_enabled_.store(enabled);
}

bool SpiDataForwarder::valid_packet(
    const std::uint8_t packet_id,
    const std::uint8_t *payload,
    const std::size_t payload_length)
{
    return packet_id != 0 && payload != nullptr &&
           payload_length >= kMinimumPayloadSize &&
           payload_length <= kMaximumPayloadSize;
}

esp_err_t SpiDataForwarder::queue_packet(
    const std::uint8_t packet_id,
    const std::uint8_t *payload,
    const std::size_t payload_length)
{
    if (forward_queue_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!valid_packet(packet_id, payload, payload_length)) {
        return ESP_ERR_INVALID_ARG;
    }

    Packet packet{};
    packet.packet_id = packet_id;
    packet.payload_length = static_cast<std::uint8_t>(payload_length);
    std::memcpy(packet.payload, payload, payload_length);
    packet.raw = false;

    return xQueueSend(forward_queue_, &packet, 0) == pdTRUE
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}

esp_err_t SpiDataForwarder::queue_raw_bytes(
    const std::uint8_t *data,
    const std::size_t length)
{
    if (forward_queue_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == nullptr || length == 0 || length > kMaximumPayloadSize) {
        return ESP_ERR_INVALID_ARG;
    }

    Packet packet{};
    packet.payload_length = static_cast<std::uint8_t>(length);
    std::memcpy(packet.payload, data, length);
    packet.raw = true;
    return xQueueSend(forward_queue_, &packet, 0) == pdTRUE
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}

esp_err_t SpiDataForwarder::queuecommand(
    const std::uint8_t packet_id,
    const std::uint8_t *payload,
    const std::size_t payload_length)
{
    if (command_queue_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!valid_packet(packet_id, payload, payload_length)) {
        return ESP_ERR_INVALID_ARG;
    }

    Packet packet{};
    packet.packet_id = packet_id;
    packet.payload_length = static_cast<std::uint8_t>(payload_length);
    std::memcpy(packet.payload, payload, payload_length);
    packet.raw = false;

    return xQueueSend(command_queue_, &packet, 0) == pdTRUE
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}

void SpiDataForwarder::forward_task_entry(void *context)
{
    static_cast<SpiDataForwarder *>(context)->run_forwarder();
}

void SpiDataForwarder::spi_task_entry(void *context)
{
    static_cast<SpiDataForwarder *>(context)->run_spi_interface();
}

void SpiDataForwarder::send_spi_packet(const Packet &packet)
{
    // TODO: Send packet.packet_id, packet.payload_length raw payload bytes, and
    // IRIS_PACKET_EOF_VALUE through the SPI peripheral. Commands are consumed
    // by the SPI task even though hardware transmission is not implemented yet.
    (void)packet;
}

void SpiDataForwarder::forward_packet(const Packet &packet)
{
    if (!output_enabled_.load()) {
        return;
    }
    std::uint8_t frame[
        IRIS_PACKET_ID_LENGTH + kMaximumPayloadSize + IRIS_PACKET_EOF_LENGTH];
    std::size_t frame_length = packet.payload_length;
    if (packet.raw) {
        std::memcpy(frame, packet.payload, packet.payload_length);
    } else {
        frame[0] = packet.packet_id;
        std::memcpy(&frame[1], packet.payload, packet.payload_length);
        frame[IRIS_PACKET_ID_LENGTH + packet.payload_length] =
            IRIS_PACKET_EOF_VALUE;
        frame_length = IRIS_PACKET_ID_LENGTH + packet.payload_length +
                       IRIS_PACKET_EOF_LENGTH;
    }
    std::size_t forwarded = 0;
    while (forwarded < frame_length && output_enabled_.load()) {
        const int bytes_written = usb_serial_jtag_write_bytes(
            frame + forwarded,
            frame_length - forwarded,
            pdMS_TO_TICKS(20));

        if (bytes_written > 0) {
            forwarded += static_cast<std::size_t>(bytes_written);
        } else {
            taskYIELD();
        }
    }
}

void SpiDataForwarder::run_forwarder()
{
    Packet packet{};
    for (;;) {
        if (xQueueReceive(forward_queue_, &packet, portMAX_DELAY) == pdTRUE) {
            forward_packet(packet);
        }
    }
}

void SpiDataForwarder::run_spi_interface()
{
    Packet command{};

    for (;;) {
        if (xQueueReceive(
                command_queue_,
                &command,
                portMAX_DELAY) == pdTRUE) {
            send_spi_packet(command);
        }
    }
}
