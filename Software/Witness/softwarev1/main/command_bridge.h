#pragma once

#include <cstddef>
#include <cstdint>

#include "comms.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

class UartDriver;
class SpiDataForwarder;

// Common command egress for commands received by USB now and by the radio in
// the future. Commands are kept byte-for-byte intact when sent over UART2.
class CommandBridge final {
public:
    CommandBridge(UartDriver &uart, SpiDataForwarder &data_forwarder);

    CommandBridge(const CommandBridge &) = delete;
    CommandBridge &operator=(const CommandBridge &) = delete;

    esp_err_t start();

    esp_err_t submit_usb_command(
        const std::uint8_t *data,
        std::size_t length);

    // Called by the radio polling code for each complete received command.
    // The command is transmitted over UART2 and queued for local processing.
    esp_err_t submit_radio_command(
        const std::uint8_t *data,
        std::size_t length);

    // Retrieves a radio command queued for local interpretation. This does not
    // affect the UART copy, which is transmitted when the command is submitted.
    bool receive_radio_command(
        std::uint8_t *data,
        std::size_t capacity,
        std::size_t &length);

private:
    esp_err_t forward_command(
        const std::uint8_t *data,
        std::size_t length);

    UartDriver &uart_;
    SpiDataForwarder &data_forwarder_;
    static constexpr std::size_t kQueueDepth = 8;

    struct QueuedCommand {
        std::uint8_t length;
        std::uint8_t data[IRIS_PACKET_COMMAND_FRAME_LENGTH];
    };

    QueueHandle_t radio_command_queue_ = nullptr;
};
