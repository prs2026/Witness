#pragma once

#include "../../../Intercomm/can_transport.h"
#include "twai_driver.h"
#include "freertos/queue.h"
#include "freertos/task.h"

class SpiDataForwarder;
class WitnessStatus;

// CAN is the Iris device link; UART1 belongs exclusively to the GPS receiver.
class CanLink final {
public:
    CanLink(SpiDataForwarder &forwarder, WitnessStatus &status)
        : forwarder_(forwarder), status_(status) {}
    esp_err_t start();
    esp_err_t send_command(const std::uint8_t *data, std::size_t length);

private:
    struct Command { std::uint8_t data[10]; };
    static void task_entry(void *context);
    void run();
    void send_packet(const std::uint8_t *data, std::size_t length);
    void receive_packets();
    TwaiDriver driver_;
    SpiDataForwarder &forwarder_;
    WitnessStatus &status_;
    QueueHandle_t commands_ = nullptr;
    TaskHandle_t task_ = nullptr;
    intercomm::Reassembler receiver_{intercomm::kIrisToWitnessId};
    std::uint8_t sequence_ = 0;
};
