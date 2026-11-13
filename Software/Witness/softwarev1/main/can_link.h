#pragma once

#include "../../../Intercomm/can_transport.h"
#include "iris_load_tracker.h"
#include "twai_driver.h"
#include "freertos/queue.h"
#include "freertos/task.h"

class FlashLogger;
class SpiDataForwarder;
class WitnessStatus;

// CAN is the Iris device link; UART1 belongs exclusively to the GPS receiver.
class CanLink final {
public:
    CanLink(SpiDataForwarder &forwarder, WitnessStatus &status)
        : forwarder_(forwarder), status_(status) {}
    esp_err_t start();
    esp_err_t send_command(const std::uint8_t *data, std::size_t length);
    // NAND packet stream: every reassembled CAN packet in and out.
    void set_log_sink(FlashLogger *logger) { logger_ = logger; }

private:
    struct Command { std::uint8_t data[10]; };
    static void task_entry(void *context);
    void run();
    void send_packet(const std::uint8_t *data, std::size_t length);
    void pump_transmit();
    void receive_packets();
    void queue_resend(const iris_load_tracker::Action &action);
    TwaiDriver driver_;
    // Backup for an Iris reset that lost its RTC state (CAN task only).
    iris_load_tracker::Tracker load_tracker_;
    std::uint16_t resend_opcodes_[4]{};
    std::uint8_t resend_count_ = 0;
    std::uint8_t resend_index_ = 0;
    SpiDataForwarder &forwarder_;
    WitnessStatus &status_;
    QueueHandle_t commands_ = nullptr;
    FlashLogger *logger_ = nullptr;
    TaskHandle_t task_ = nullptr;
    intercomm::Reassembler receiver_{intercomm::kIrisToWitnessId};
    std::uint8_t sequence_ = 0;
    std::array<std::uint8_t, intercomm::kMaxPacketLength> pending_{};
    std::size_t pending_length_ = 0;
    std::size_t pending_fragment_ = 0;
    std::int64_t pending_deadline_us_ = 0;
};
