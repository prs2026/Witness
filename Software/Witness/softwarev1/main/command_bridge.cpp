#include "command_bridge.h"

#include <cstring>

#include "command_policy.h"
#include "comms.h"
#include "freertos/FreeRTOS.h"
#include "spi_data_forwarder.h"
#include "can_link.h"
#include "esp_timer.h"

namespace {
bool canonical_command(const std::uint8_t *data, std::size_t length,
                       std::uint8_t *out)
{
    if (data == nullptr || data[0] != IRIS_PACKET_ID_COMMAND) return false;
    if (length == IRIS_PACKET_COMMAND_FRAME_LENGTH) {
        if (data[length - 1] != IRIS_PACKET_EOF_VALUE) return false;
        std::memcpy(out, data, length);
        return true;
    }
    if (length != 3) return false;
    std::memset(out, 0, IRIS_PACKET_COMMAND_FRAME_LENGTH);
    out[0] = IRIS_PACKET_ID_COMMAND;
    const std::uint32_t uptime = esp_timer_get_time() / 1000U;
    for (unsigned i = 0; i < 4; ++i) out[3 + i] = uptime >> (24U - 8U * i);
    out[7] = data[1];
    out[8] = data[2];
    out[9] = IRIS_PACKET_EOF_VALUE;
    return true;
}
}

CommandBridge::CommandBridge(
    CanLink &iris,
    SpiDataForwarder &data_forwarder)
    : iris_(iris),
      data_forwarder_(data_forwarder)
{
}

esp_err_t CommandBridge::start()
{
    if (radio_command_queue_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    radio_command_queue_ =
        xQueueCreate(kQueueDepth, sizeof(QueuedCommand));
    return radio_command_queue_ != nullptr ? ESP_OK : ESP_ERR_NO_MEM;
}

void CommandBridge::set_state_reader(const StateReader reader, void *const context)
{
    state_context_ = context;
    state_reader_ = reader;
}

esp_err_t CommandBridge::check_policy(const std::uint8_t *const canonical) const
{
    const std::uint16_t opcode = static_cast<std::uint16_t>((canonical[7] << 8U) | canonical[8]);
    const std::uint8_t state = state_reader_ != nullptr ? state_reader_(state_context_) : command_policy::kPadIdle;
    return command_policy::allow_iris_load_command(opcode, state) ? ESP_OK : ESP_ERR_NOT_ALLOWED;
}

esp_err_t CommandBridge::submit_usb_command(
    const std::uint8_t *const data,
    const std::size_t length)
{
    std::uint8_t canonical[IRIS_PACKET_COMMAND_FRAME_LENGTH]{};
    if (length == 0 || !canonical_command(data, length, canonical))
        return ESP_ERR_INVALID_ARG;
    // A command the policy refuses (load / 5 V OFF in flight) goes nowhere:
    // not to Iris, not over LoRa (audit 07 #2, #16).
    if (check_policy(canonical) != ESP_OK) return ESP_ERR_NOT_ALLOWED;
    const esp_err_t transport_result = forward_command(canonical, sizeof(canonical));
    const esp_err_t radio_result =
        data_forwarder_.queue_immediate_radio_packet(canonical, sizeof(canonical));
    return transport_result != ESP_OK ? transport_result : radio_result;
}

esp_err_t CommandBridge::submit_radio_command(
    const std::uint8_t *const data,
    const std::size_t length)
{
    std::uint8_t canonical[IRIS_PACKET_COMMAND_FRAME_LENGTH]{};
    if (length == 0 || !canonical_command(data, length, canonical))
        return ESP_ERR_INVALID_ARG;
    // GPS assistance chunks are for Witness only (hundreds per upload).
    const std::uint16_t opcode = static_cast<std::uint16_t>((canonical[7] << 8U) | canonical[8]);
    const bool local_only = opcode == IRIS_COMMAND_GPS_ASSIST_BEGIN ||
                            opcode == IRIS_COMMAND_GPS_ASSIST_DATA ||
                            opcode == IRIS_COMMAND_GPS_ASSIST_COMMIT ||
                            opcode == IRIS_COMMAND_GPS_ASSIST_STATUS ||
                            opcode == IRIS_COMMAND_GPS_COLD_START;
    const esp_err_t policy_result = local_only ? ESP_OK : check_policy(canonical);
    const esp_err_t transport_result =
        local_only ? ESP_OK
                   : (policy_result != ESP_OK ? policy_result : forward_command(canonical, sizeof(canonical)));

    if (radio_command_queue_ == nullptr) {
        return transport_result == ESP_OK ? ESP_ERR_INVALID_STATE : transport_result;
    }
    QueuedCommand command{};
    command.length = sizeof(canonical);
    command.forward_result = static_cast<std::int32_t>(transport_result);
    std::memcpy(command.data, canonical, sizeof(canonical));
    const esp_err_t queue_result =
        xQueueSend(radio_command_queue_, &command, 0) == pdTRUE
            ? ESP_OK
            : ESP_ERR_NO_MEM;
    return transport_result != ESP_OK ? transport_result : queue_result;
}

bool CommandBridge::receive_radio_command(
    std::uint8_t *const data,
    const std::size_t capacity,
    std::size_t &length,
    esp_err_t *const forward_result)
{
    length = 0;
    if (radio_command_queue_ == nullptr || data == nullptr) {
        return false;
    }

    QueuedCommand command{};
    if (xQueueReceive(radio_command_queue_, &command, 0) != pdTRUE ||
        command.length > capacity) {
        return false;
    }

    length = command.length;
    if (forward_result != nullptr) *forward_result = static_cast<esp_err_t>(command.forward_result);
    std::memcpy(data, command.data, length);
    return true;
}

esp_err_t CommandBridge::forward_command(
    const std::uint8_t *const data,
    const std::size_t length)
{
    if (data == nullptr || length == 0 || data[0] != IRIS_PACKET_ID_COMMAND) {
        return ESP_ERR_INVALID_ARG;
    }

    return iris_.send_command(data, length);
}
