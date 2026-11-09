#include "can_link.h"
#include "task_cores.h"

#include <cstring>
#include "comms.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "spi_data_forwarder.h"
#include "witness_status.h"

namespace {
constexpr const char *kTag = "iris_can";
void write_u32(std::uint8_t *out, const std::uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i) out[i] = value >> (24U - i * 8U);
}
}

esp_err_t CanLink::start()
{
    if (commands_ != nullptr) return ESP_ERR_INVALID_STATE;
    commands_ = xQueueCreate(8, sizeof(Command));
    if (commands_ == nullptr) return ESP_ERR_NO_MEM;
    const esp_err_t result = driver_.initialize();
    if (result != ESP_OK) {
        vQueueDelete(commands_);
        commands_ = nullptr;
        return result;
    }
    if (xTaskCreatePinnedToCore(task_entry, "iris_can", 4096, this, 3, &task_, task_cores::kSystem) != pdPASS) {
        (void)driver_.shutdown();
        vQueueDelete(commands_);
        commands_ = nullptr;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(kTag, "CAN enabled: 500 kbit/s, TX37 RX36, Iris link");
    return ESP_OK;
}

esp_err_t CanLink::send_command(const std::uint8_t *data, const std::size_t length)
{
    if (data == nullptr || length != IRIS_PACKET_COMMAND_FRAME_LENGTH ||
        data[0] != IRIS_PACKET_ID_COMMAND || data[length - 1] != IRIS_PACKET_EOF_VALUE)
        return ESP_ERR_INVALID_ARG;
    if (commands_ == nullptr) return ESP_ERR_INVALID_STATE;
    Command command{};
    std::memcpy(command.data, data, length);
    return xQueueSend(commands_, &command, 0) == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM;
}

void CanLink::task_entry(void *context) { static_cast<CanLink *>(context)->run(); }

void CanLink::send_packet(const std::uint8_t *data, const std::size_t length)
{
    if (pending_length_ != 0 || length > pending_.size()) return;
    std::memcpy(pending_.data(), data, length);
    pending_length_ = length;
    pending_fragment_ = 0;
    ++sequence_;
    pending_deadline_us_ = esp_timer_get_time() + 200000;
    pump_transmit();
}

void CanLink::pump_transmit()
{
    if (pending_length_ == 0) return;
    if (esp_timer_get_time() >= pending_deadline_us_) {
        ESP_LOGW(kTag, "CAN packet expired before all fragments could be queued");
        pending_length_ = 0;
        return;
    }
    while (pending_fragment_ < intercomm::fragment_count(pending_length_)) {
        intercomm::Frame fragment{};
        if (!intercomm::encode_fragment(pending_.data(), pending_length_,
                                       sequence_, pending_fragment_, fragment)) {
            pending_length_ = 0;
            return;
        }
        TwaiDriver::Frame frame{};
        frame.identifier = intercomm::kWitnessToIrisId;
        frame.data = fragment.data;
        frame.length = fragment.length;
        // Retain the cursor across transient queue congestion. Never replay
        // an already submitted fragment or an entire actuation command.
        const esp_err_t result = driver_.transmit(frame);
        if (result == ESP_ERR_TIMEOUT) return;
        if (result != ESP_OK) {
            ESP_LOGW(kTag, "CAN packet submission failed: %s", esp_err_to_name(result));
            pending_length_ = 0;
            return;
        }
        ++pending_fragment_;
    }
    pending_length_ = 0;
}

void CanLink::receive_packets()
{
    TwaiDriver::Frame native{};
    while (driver_.receive(native)) {
        intercomm::Frame fragment{};
        fragment.identifier = native.identifier;
        fragment.data = native.data;
        fragment.length = native.length;
        fragment.extended = native.extended;
        fragment.remote = native.remote;
        if (!receiver_.accept(fragment, esp_timer_get_time() / 1000U)) continue;
        const auto *data = receiver_.data();
        const auto length = receiver_.size();
        const bool allowed =
            (data[0] == IRIS_PACKET_ID_CAMERA && length == IRIS_PACKET_CAMERA_FRAME_LENGTH) ||
            (data[0] == IRIS_PACKET_ID_IRIS_DEBUG && length == IRIS_PACKET_IRIS_DEBUG_FRAME_LENGTH) ||
            (data[0] == IRIS_PACKET_ID_COMMAND && length == IRIS_PACKET_COMMAND_FRAME_LENGTH);
        if (allowed && data[length - 1] == IRIS_PACKET_EOF_VALUE)
            (void)forwarder_.queue_packet(data[0], data + 1, length - 2);
        // Iris responses never enter CommandBridge, preventing command loops.
    }
}

void CanLink::run()
{
    std::int64_t next_heartbeat = 0;
    std::int64_t next_diagnostic = 0;
    while (true) {
        (void)driver_.poll();
        receive_packets();
        pump_transmit();
        Command command{};
        if (pending_length_ == 0 && xQueueReceive(commands_, &command, 0) == pdTRUE)
            send_packet(command.data, sizeof(command.data));
        const std::int64_t now = esp_timer_get_time();
        if (pending_length_ == 0 && now >= next_heartbeat) {
            std::uint8_t heartbeat[IRIS_PACKET_HEARTBEAT_FRAME_LENGTH]{};
            heartbeat[0] = IRIS_PACKET_ID_HEARTBEAT;
            status_.write(heartbeat + 1);
            write_u32(heartbeat + 3, static_cast<std::uint32_t>(now / 1000U));
            heartbeat[sizeof(heartbeat) - 1] = IRIS_PACKET_EOF_VALUE;
            send_packet(heartbeat, sizeof(heartbeat));
            next_heartbeat = now + 1000000;
        }
        if (now >= next_diagnostic) {
            twai_node_status_t status{};
            twai_node_record_t record{};
            if (driver_.get_status(status, record) == ESP_OK)
                ESP_LOGI(kTag, "state=%u RX dropped=%lu TX failed=%lu",
                         static_cast<unsigned>(status.state),
                         static_cast<unsigned long>(driver_.dropped_receive_frames()),
                         static_cast<unsigned long>(driver_.failed_transmit_frames()));
            next_diagnostic = now + 5000000;
        }
        vTaskDelay(1);
    }
}
