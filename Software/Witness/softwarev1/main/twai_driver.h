#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "esp_twai.h"
#include "freertos/FreeRTOS.h"

class TwaiDriver final {
public:
    static constexpr std::size_t kMaxDataLength = 8;

    struct Frame {
        std::uint32_t identifier = 0;
        std::array<std::uint8_t, kMaxDataLength> data{};
        std::uint8_t length = 0;
        bool extended = false;
        bool remote = false;
        std::uint64_t timestamp_us = 0;
    };

    TwaiDriver() = default;
    ~TwaiDriver();

    TwaiDriver(const TwaiDriver &) = delete;
    TwaiDriver &operator=(const TwaiDriver &) = delete;

    esp_err_t initialize();
    esp_err_t shutdown();

    // Queues a copied frame without blocking. ESP_ERR_TIMEOUT means all local
    // transmit slots or the hardware transmit queue are currently full.
    esp_err_t transmit(const Frame &frame);

    // Returns false when no received frame is waiting.
    bool receive(Frame &frame);

    // Call from the application's direct loop to initiate bus-off recovery.
    esp_err_t poll();
    esp_err_t get_status(twai_node_status_t &status,
                         twai_node_record_t &record) const;

    std::uint32_t dropped_receive_frames() const;
    std::uint32_t failed_transmit_frames() const;

private:
    static constexpr std::size_t kReceiveQueueDepth = 16;
    static constexpr std::size_t kTransmitQueueDepth = 8;

    struct ReceiveSlot {
        twai_frame_header_t header{};
        std::array<std::uint8_t, kMaxDataLength> data{};
    };

    struct TransmitSlot {
        twai_frame_t native_frame{};
        std::array<std::uint8_t, kMaxDataLength> data{};
        bool in_use = false;
    };

    static bool IRAM_ATTR on_receive(
        twai_node_handle_t node,
        const twai_rx_done_event_data_t *event,
        void *context);
    static bool IRAM_ATTR on_transmit_done(
        twai_node_handle_t node,
        const twai_tx_done_event_data_t *event,
        void *context);
    static bool IRAM_ATTR on_state_change(
        twai_node_handle_t node,
        const twai_state_change_event_data_t *event,
        void *context);

    bool handle_receive_from_isr(twai_node_handle_t node);
    void handle_transmit_done_from_isr(
        const twai_tx_done_event_data_t &event);
    void handle_state_change_from_isr(
        const twai_state_change_event_data_t &event);

    twai_node_handle_t node_ = nullptr;
    std::array<ReceiveSlot, kReceiveQueueDepth> receive_queue_{};
    std::array<TransmitSlot, kTransmitQueueDepth> transmit_slots_{};
    std::size_t receive_read_index_ = 0;
    std::size_t receive_write_index_ = 0;
    std::uint32_t dropped_receive_frames_ = 0;
    std::uint32_t failed_transmit_frames_ = 0;
    bool bus_off_ = false;
    bool recovery_started_ = false;
    mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
};
