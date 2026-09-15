#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

class SpiDataForwarder final {
public:
    SpiDataForwarder() = default;

    SpiDataForwarder(const SpiDataForwarder &) = delete;
    SpiDataForwarder &operator=(const SpiDataForwarder &) = delete;

    esp_err_t start();

    // Adds an SPI-received packet to the USB forwarding buffer.
    esp_err_t queue_packet(
        std::uint8_t packet_id,
        const std::uint8_t *payload,
        std::size_t payload_length);

    // Queues a packet for a future SPI transmit implementation.
    esp_err_t queuecommand(
        std::uint8_t packet_id,
        const std::uint8_t *payload,
        std::size_t payload_length);

private:
    static constexpr UBaseType_t kForwardTaskPriority = 2;
    static constexpr UBaseType_t kSpiTaskPriority = 2;
    static constexpr std::uint32_t kTaskStackSize = 3072;
    static constexpr TickType_t kDummyPacketPeriod = pdMS_TO_TICKS(1000);
    static constexpr std::size_t kQueueDepth = 16;
    static constexpr std::size_t kMinimumPayloadSize = 6;
    static constexpr std::size_t kMaximumPayloadSize = 64;

    struct Packet {
        std::uint8_t packet_id;
        std::uint8_t payload_length;
        std::uint8_t payload[kMaximumPayloadSize];
    };

    static void forward_task_entry(void *context);
    static void spi_task_entry(void *context);
    static bool valid_packet(
        std::uint8_t packet_id,
        const std::uint8_t *payload,
        std::size_t payload_length);
    static void send_spi_packet(const Packet &packet);
    void generate_dummy_spi_packet();
    void forward_packet(const Packet &packet);
    void run_forwarder();
    void run_spi_interface();

    QueueHandle_t forward_queue_ = nullptr;
    QueueHandle_t command_queue_ = nullptr;
    TaskHandle_t forward_task_handle_ = nullptr;
    TaskHandle_t spi_task_handle_ = nullptr;
    std::uint32_t dummy_sequence_number_ = 0;
};
