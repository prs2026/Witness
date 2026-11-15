#pragma once

#include <atomic>

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
    // Landed low-power mode: while paused the TWAI node is disabled (no RX
    // interrupts, no queue to overflow, no bus errors counted, and the driver's
    // no-light-sleep lock is released so the MCU can sleep); Witness neither
    // forwards nor sends heartbeats. A command for Iris (send_command) brings
    // the bus back for kCanGraceMs, sends it, then pauses again. set_paused(false)
    // resumes normal operation. Any task.
    void set_paused(bool paused);
    bool paused() const { return paused_.load(); }
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
    std::atomic<bool> paused_{false};
    std::int64_t grace_until_us_ = 0;   // CAN task only
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
