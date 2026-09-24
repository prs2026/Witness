#include "command_bridge.h"

#include <cstring>

#include "comms.h"
#include "freertos/FreeRTOS.h"
#include "spi_data_forwarder.h"
#include "uart_driver.h"

CommandBridge::CommandBridge(
    UartDriver &uart,
    SpiDataForwarder &data_forwarder)
    : uart_(uart),
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

esp_err_t CommandBridge::submit_usb_command(
    const std::uint8_t *const data,
    const std::size_t length)
{
    const esp_err_t uart_result = forward_command(data, length);
    const esp_err_t radio_result =
        data_forwarder_.queue_immediate_radio_packet(data, length);
    return uart_result != ESP_OK ? uart_result : radio_result;
}

esp_err_t CommandBridge::submit_radio_command(
    const std::uint8_t *const data,
    const std::size_t length)
{
    const esp_err_t uart_result = forward_command(data, length);

    if (radio_command_queue_ == nullptr) {
        return uart_result == ESP_OK ? ESP_ERR_INVALID_STATE : uart_result;
    }
    if (data == nullptr || length == 0U ||
        length > IRIS_PACKET_COMMAND_FRAME_LENGTH ||
        data[0] != IRIS_PACKET_ID_COMMAND) {
        return uart_result == ESP_OK ? ESP_ERR_INVALID_ARG : uart_result;
    }

    QueuedCommand command{};
    command.length = static_cast<std::uint8_t>(length);
    std::memcpy(command.data, data, length);
    const esp_err_t queue_result =
        xQueueSend(radio_command_queue_, &command, 0) == pdTRUE
            ? ESP_OK
            : ESP_ERR_NO_MEM;
    return uart_result != ESP_OK ? uart_result : queue_result;
}

bool CommandBridge::receive_radio_command(
    std::uint8_t *const data,
    const std::size_t capacity,
    std::size_t &length)
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

    return uart_.transmit(data, length, pdMS_TO_TICKS(100));
}
