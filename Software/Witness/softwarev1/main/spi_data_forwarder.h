#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "comms.h"
#include "esp_err.h"
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

    // Adds an SPI-received packet to the USB forwarding buffer.
    esp_err_t queue_packet(
        std::uint8_t packet_id,
        const std::uint8_t *payload,
        std::size_t payload_length);

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
    static constexpr std::size_t kImmediateRadioQueueDepth = 8;
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
