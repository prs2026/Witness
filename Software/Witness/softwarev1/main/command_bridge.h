#pragma once

#include <cstddef>
#include <cstdint>

#include "comms.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

class CanLink;
class SpiDataForwarder;

// Common command egress for commands received by USB now and by the radio in
// the future. Commands use the Iris CAN link; GPS UART1 is never used here.
class CommandBridge final {
public:
    CommandBridge(CanLink &iris, SpiDataForwarder &data_forwarder);

    CommandBridge(const CommandBridge &) = delete;
    CommandBridge &operator=(const CommandBridge &) = delete;

    esp_err_t start();

    esp_err_t submit_usb_command(
        const std::uint8_t *data,
        std::size_t length);

    // Called by the radio polling code for each complete received command.
    // The command is queued for CAN transmission and local processing.
    esp_err_t submit_radio_command(
        const std::uint8_t *data,
        std::size_t length);

    // Bench only (comms.h IRIS_COMMAND_BENCH_IRIS_RESET): sends the armed Iris
    // warm-reset frame over CAN. The caller (USB handler) has checked HITL.
    esp_err_t send_bench_iris_reset();

    // Flight-state source for the load-command gate (command_policy.h:
    // allow_iris_load_command). Without one the state is taken as Pad Idle.
    // The reader must be thread safe (it is called from the USB and sensor
    // tasks).
    using StateReader = std::uint8_t (*)(void *context);
    void set_state_reader(StateReader reader, void *context);

    // Retrieves a radio command queued for local interpretation. This does not
    // affect the CAN copy, which is queued when the command is submitted.
    // forward_result (optional) = what the CAN forward of that command
    // returned: ESP_OK (queued for Iris), ESP_ERR_NOT_ALLOWED (refused by the
    // command policy: nothing was sent) or another error (queue full / link
    // not up). The local handler acks loads from it.
    bool receive_radio_command(
        std::uint8_t *data,
        std::size_t capacity,
        std::size_t &length,
        esp_err_t *forward_result = nullptr);

private:
    esp_err_t forward_command(
        const std::uint8_t *data,
        std::size_t length);
    // ESP_ERR_NOT_ALLOWED when command_policy refuses this canonical frame now.
    esp_err_t check_policy(const std::uint8_t *canonical) const;

    CanLink &iris_;
    SpiDataForwarder &data_forwarder_;
    static constexpr std::size_t kQueueDepth = 8;

    struct QueuedCommand {
        std::uint8_t length;
        std::uint8_t data[IRIS_PACKET_COMMAND_FRAME_LENGTH];
        std::int32_t forward_result;   // esp_err_t of the CAN forward
    };

    StateReader state_reader_ = nullptr;
    void *state_context_ = nullptr;

    QueueHandle_t radio_command_queue_ = nullptr;
};

