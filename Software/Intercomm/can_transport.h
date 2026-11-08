#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

// Version 1 transport: canonical application frames over Classical CAN.
namespace intercomm {
constexpr std::uint32_t kWitnessToIrisId = 0x120;
constexpr std::uint32_t kIrisToWitnessId = 0x121;
constexpr std::uint32_t kBitrate = 500000;
constexpr std::size_t kMaxPacketLength = 103;
constexpr std::size_t kFragmentPayloadLength = 5;
constexpr std::uint32_t kReassemblyTimeoutMs = 250;

struct Frame {
    std::uint32_t identifier = 0;
    std::array<std::uint8_t, 8> data{};
    std::uint8_t length = 0;
    bool extended = false;
    bool remote = false;
};

constexpr std::size_t fragment_count(std::size_t length) {
    return (length + kFragmentPayloadLength - 1) / kFragmentPayloadLength;
}

inline bool encode_fragment(const std::uint8_t *packet, std::size_t length,
                            std::uint8_t sequence, std::size_t index,
                            Frame &frame) {
    if (!packet || length < 2 || length > kMaxPacketLength ||
        index >= fragment_count(length)) return false;
    const auto offset = index * kFragmentPayloadLength;
    const auto count = std::min(kFragmentPayloadLength, length - offset);
    frame.data.fill(0);
    frame.extended = false;
    frame.remote = false;
    frame.length = static_cast<std::uint8_t>(3 + count);
    frame.data[0] = sequence;
    frame.data[1] = static_cast<std::uint8_t>(index);
    frame.data[2] = static_cast<std::uint8_t>(length);
    std::copy_n(packet + offset, count, frame.data.begin() + 3);
    return true;
}

class Reassembler {
public:
    explicit Reassembler(std::uint32_t expected_identifier)
        : expected_identifier_(expected_identifier) {}

    void reset() { active_ = false; size_ = 0; next_index_ = 0; }
    const std::uint8_t *data() const { return packet_.data(); }
    std::size_t size() const { return size_; }

    // A true result exposes one complete frame until the next accepted transfer.
    // Callers must additionally validate packet ID, canonical length and EOF.
    bool accept(const Frame &frame, std::uint32_t now_ms) {
        if (frame.identifier != expected_identifier_ || frame.extended ||
            frame.remote) return false;
        if (active_ && static_cast<std::uint32_t>(now_ms - started_ms_) >=
                           kReassemblyTimeoutMs) reset();
        if (frame.length < 4 || frame.length > 8) { reset(); return false; }
        const auto sequence = frame.data[0];
        const auto index = frame.data[1];
        const std::size_t total = frame.data[2];
        if (total < 2 || total > kMaxPacketLength ||
            index >= fragment_count(total)) { reset(); return false; }
        const auto offset = index * kFragmentPayloadLength;
        const auto count = std::min(kFragmentPayloadLength, total - offset);
        if (frame.length != 3 + count) { reset(); return false; }
        if (index == 0) {
            reset();
            active_ = true;
            sequence_ = sequence;
            total_ = total;
            started_ms_ = now_ms;
        }
        if (!active_ || sequence != sequence_ || total != total_ ||
            index != next_index_) { reset(); return false; }
        std::copy_n(frame.data.begin() + 3, count, packet_.begin() + offset);
        ++next_index_;
        if (offset + count != total_) return false;
        active_ = false;
        size_ = total_;
        return true;
    }

private:
    const std::uint32_t expected_identifier_;
    std::array<std::uint8_t, kMaxPacketLength> packet_{};
    std::size_t total_ = 0;
    std::size_t size_ = 0;
    std::size_t next_index_ = 0;
    std::uint8_t sequence_ = 0;
    std::uint32_t started_ms_ = 0;
    bool active_ = false;
};
} // namespace intercomm
