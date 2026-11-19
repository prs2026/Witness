#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "comms.h"
#include "esp_err.h"
#include "radio_edge_logic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

class SpiDataForwarder final {
public:
    // When enabled, complete packets received through USB/UART can bypass the
    // per-type radio scheduler and enter the immediate radio queue.
    static constexpr bool kForwardExternalPacketsImmediately = true;
    static constexpr std::size_t kMaximumFrameLength =
        IRIS_PACKET_ID_LENGTH + IRIS_PACKET_MAX_DATA_LENGTH +
        IRIS_PACKET_EOF_LENGTH;

    struct RadioPacket {
        std::uint8_t length = 0;
        std::uint8_t data[kMaximumFrameLength]{};
    };

    SpiDataForwarder() = default;

    SpiDataForwarder(const SpiDataForwarder &) = delete;
    SpiDataForwarder &operator=(const SpiDataForwarder &) = delete;

    esp_err_t start();
    void set_output_enabled(bool enabled);
    void set_radio_available(bool available) { radio_available_.store(available); }
    bool radio_available() const { return radio_available_.load(); }

    // Adds an SPI-received packet to the USB forwarding buffer.
    esp_err_t queue_packet(
        std::uint8_t packet_id,
        const std::uint8_t *payload,
        std::size_t payload_length);

    // Same as queue_packet, but waits up to `wait` for room in the USB queue
    // instead of dropping at once. For command acknowledgements: at 936 Hz
    // sensor packets keep the 16-deep queue nearly full, and a stalled
    // forwarder (a flash write) used to drop the ack.
    esp_err_t queue_packet_wait(
        std::uint8_t packet_id,
        const std::uint8_t *payload,
        std::size_t payload_length,
        TickType_t wait);

    // Returns once every packet queued before this call has been written to the
    // USB driver and the USB FIFO has drained (or `timeout` passes). Used before
    // a deliberate restart so the last ack is not cut off by it.
    esp_err_t flush(TickType_t timeout);

    // Queues bytes exactly as received from another transport. Unlike an
    // application packet, no packet ID or EOF byte is added.
    esp_err_t queue_raw_bytes(
        const std::uint8_t *data,
        std::size_t length);

    // Queues a complete externally received frame for immediate radio TX.
    esp_err_t queue_immediate_radio_packet(
        const std::uint8_t *data,
        std::size_t length);
    bool receive_immediate_radio_packet(RadioPacket &packet);
    // Immediate-queue state for the radio ack pacing (sensor task).
    bool immediate_radio_pending() const;
    std::uint32_t immediate_radio_dropped() const { return immediate_dropped_.load(); }

    // Returns the newest locally generated canonical frame for a packet ID.
    bool latest_radio_packet(
        std::uint8_t packet_id,
        RadioPacket &packet,
        std::uint32_t &generation) const;

    // Queues a packet for a future SPI transmit implementation.
    esp_err_t queuecommand(
        std::uint8_t packet_id,
        const std::uint8_t *payload,
        std::size_t payload_length);

private:
    static constexpr UBaseType_t kForwardTaskPriority = 2;
    static constexpr UBaseType_t kSpiTaskPriority = 2;
    static constexpr std::uint32_t kTaskStackSize = 3072;
    static constexpr std::size_t kQueueDepth = 16;
    static constexpr std::size_t kImmediateRadioQueueDepth = 16;
    static constexpr std::size_t kMinimumPayloadSize = 6;
    static constexpr std::size_t kMaximumPayloadSize =
        IRIS_PACKET_MAX_DATA_LENGTH;
    static constexpr std::size_t kMaximumRawLength = kMaximumFrameLength;
    static constexpr std::size_t kRadioPacketTypeCount = 7;
    static_assert(IRIS_PACKET_WITNESS_DEBUG_DATA_LENGTH <= kMaximumPayloadSize,
                  "canonical packets must fit the forwarding queue");

    struct Packet {
        std::uint8_t packet_id;
        std::uint8_t payload_length;
        std::uint8_t payload[kMaximumRawLength];
        bool raw;
    };

    struct RadioCacheSlot {
        std::uint8_t packet_id = 0;
        RadioPacket packet{};
        std::uint32_t generation = 0;
    };

    static void forward_task_entry(void *context);
    static void spi_task_entry(void *context);
    static bool valid_packet(
        std::uint8_t packet_id,
        const std::uint8_t *payload,
        std::size_t payload_length);
    static void send_spi_packet(const Packet &packet);
    void forward_packet(const Packet &packet);
    void run_forwarder();
    void run_spi_interface();
    void cache_radio_packet(
        std::uint8_t packet_id,
        const std::uint8_t *payload,
        std::size_t payload_length);

    QueueHandle_t forward_queue_ = nullptr;
    QueueHandle_t command_queue_ = nullptr;
    QueueHandle_t immediate_radio_queue_ = nullptr;
    TaskHandle_t forward_task_handle_ = nullptr;
    TaskHandle_t spi_task_handle_ = nullptr;
    std::atomic<bool> output_enabled_{true};
    // USB output with no host reading (no cable in flight, or no ser2net
    // client on the bench) must not stall the forwarder: after one 20 ms
    // write timeout the link counts as stalled and packets are dropped
    // without waiting until a write succeeds again.
    bool usb_stalled_ = false;
    std::uint32_t usb_packets_dropped_ = 0;
    std::uint32_t usb_drops_reported_ = 0;
    TickType_t usb_last_drop_report_ = 0;
    std::atomic<bool> radio_available_{false};
    std::atomic<std::uint32_t> immediate_dropped_{0};
    // flush(): packets accepted into the USB queue / fully handled by the forwarder.
    std::atomic<std::uint32_t> usb_queued_{0};
    std::atomic<std::uint32_t> usb_handled_{0};
    std::array<RadioCacheSlot, kRadioPacketTypeCount> radio_cache_{{
        {IRIS_PACKET_ID_SENSORS},
        {IRIS_PACKET_ID_STATE},
        {IRIS_PACKET_ID_CAMERA},
        {IRIS_PACKET_ID_COMMAND},
        {IRIS_PACKET_ID_HEARTBEAT},
        {IRIS_PACKET_ID_WITNESS_DEBUG},
        {IRIS_PACKET_ID_IRIS_DEBUG},
    }};
    mutable portMUX_TYPE radio_cache_lock_ = portMUX_INITIALIZER_UNLOCKED;
};
