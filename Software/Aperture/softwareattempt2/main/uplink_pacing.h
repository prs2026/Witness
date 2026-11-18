#pragma once

// Uplink queue with duplicate suppression, and burst pacing (pure logic, host
// test). Witness holds its telemetry for 1 s after EVERY command it hears
// (tx_slot.h kUplinkHoldOffMs), so a steady command stream silences the
// downlink. Rule: at most kMaxBurst commands, then at least kQuietMs with no
// uplink before the next burst, even when the queue is not empty. Applies with
// or without GPS/PPS; tx_slot::aperture_may_send is still checked on top.

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace uplink_pacing {

inline constexpr unsigned kMaxBurst = 3;
inline constexpr std::uint32_t kQuietMs = 1500;   // > 1000 ms hold-off + part of a telemetry window

class Pacer {
public:
    bool allow(std::uint64_t now_ms) const
    {
        if (!sent_before_) return true;
        if (now_ms - last_sent_ms_ >= kQuietMs) return true;   // new burst
        return in_burst_ < kMaxBurst;
    }
    void on_sent(std::uint64_t now_ms)
    {
        if (!sent_before_ || now_ms - last_sent_ms_ >= kQuietMs) in_burst_ = 0;
        ++in_burst_;
        sent_before_ = true;
        last_sent_ms_ = now_ms;
    }
    unsigned in_burst() const { return in_burst_; }

private:
    bool sent_before_ = false;
    std::uint64_t last_sent_ms_ = 0;
    unsigned in_burst_ = 0;
};

template <std::size_t FrameLength, std::size_t Capacity>
class Queue {
public:
    enum class Push { Queued, Duplicate, Full };

    Push push(const std::uint8_t *frame)
    {
        for (std::size_t i = 0; i < count_; ++i) {
            if (std::memcmp(items_[(head_ + i) % Capacity], frame, FrameLength) == 0) return Push::Duplicate;
        }
        if (count_ == Capacity) return Push::Full;
        std::memcpy(items_[(head_ + count_) % Capacity], frame, FrameLength);
        ++count_;
        return Push::Queued;
    }
    bool empty() const { return count_ == 0; }
    std::size_t size() const { return count_; }
    const std::uint8_t *front() const { return items_[head_]; }
    void pop() { head_ = (head_ + 1) % Capacity; --count_; }

private:
    std::uint8_t items_[Capacity][FrameLength] = {};
    std::size_t head_ = 0, count_ = 0;
};

}  // namespace uplink_pacing
