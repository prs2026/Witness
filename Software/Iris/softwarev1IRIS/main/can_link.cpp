#include "can_link.h"

#include <algorithm>

#include "esp_timer.h"

esp_err_t CanLink::initialize()
{
    const esp_err_t result = driver_.initialize();
    initialized_ = result == ESP_OK;
    return result;
}

bool CanLink::queue_packet(const std::uint8_t *data, const std::size_t length)
{
    if (!initialized_ || data == nullptr || length == 0 || length > 103 ||
        count_ == transmit_queue_.size()) {
        return false;
    }
    Packet &packet = transmit_queue_[write_index_];
    std::copy_n(data, length, packet.data.begin());
    packet.length = length;
    write_index_ = (write_index_ + 1U) % transmit_queue_.size();
    ++count_;
    return true;
}

void CanLink::poll()
{
    if (!initialized_) {
        return;
    }
    (void)driver_.poll();
    // Bound work per loop so a busy CAN bus cannot starve sampling or USB.
    for (std::size_t sent = 0; sent < 8 && count_ > 0; ++sent) {
        const Packet &packet = transmit_queue_[read_index_];
        intercomm::Frame encoded{};
        if (!intercomm::encode_fragment(packet.data.data(), packet.length,
                                        sequence_, fragment_, encoded)) {
            return;
        }
        TwaiDriver::Frame frame{};
        frame.identifier = intercomm::kIrisToWitnessId;
        frame.data = encoded.data;
        frame.length = encoded.length;
        if (driver_.transmit(frame) != ESP_OK) {
            return;
        }
        ++fragment_;
        if (fragment_ == intercomm::fragment_count(packet.length)) {
            fragment_ = 0;
            ++sequence_;
            read_index_ = (read_index_ + 1U) % transmit_queue_.size();
            --count_;
        }
    }
}

bool CanLink::receive_packet(std::uint8_t *data, std::size_t &length)
{
    if (!initialized_) {
        return false;
    }
    TwaiDriver::Frame received{};
    while (driver_.receive(received)) {
        intercomm::Frame frame{};
        frame.identifier = received.identifier;
        frame.data = received.data;
        frame.length = received.length;
        frame.extended = received.extended;
        frame.remote = received.remote;
        if (receiver_.accept(frame,
                static_cast<std::uint32_t>(esp_timer_get_time() / 1000))) {
            length = receiver_.size();
            std::copy_n(receiver_.data(), length, data);
            return true;
        }
    }
    return false;
}
