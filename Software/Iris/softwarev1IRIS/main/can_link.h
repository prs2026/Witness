#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "can_transport.h"
#include "twai_driver.h"

// Canonical packets queued as units; CAN backpressure cannot interleave them.
class CanLink final {
public:
    esp_err_t initialize();
    bool queue_packet(const std::uint8_t *data, std::size_t length);
    void poll();
    bool receive_packet(std::uint8_t *data, std::size_t &length);

private:
    struct Packet {
        std::array<std::uint8_t, 103> data{};
        std::size_t length = 0;
    };
    TwaiDriver driver_;
    intercomm::Reassembler receiver_{intercomm::kWitnessToIrisId};
    std::array<Packet, 8> transmit_queue_{};
    std::size_t read_index_ = 0;
    std::size_t write_index_ = 0;
    std::size_t count_ = 0;
    std::uint8_t sequence_ = 0;
    std::uint8_t fragment_ = 0;
    bool initialized_ = false;
};
