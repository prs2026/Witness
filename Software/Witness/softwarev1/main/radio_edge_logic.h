#pragma once

// Pure logic for the radio edge cases of the 2026-11-18 batch (no ESP-IDF
// dependency; host test tests/radio_edge_logic_test.cpp). The sensor task owns
// all of it (not thread safe).
//
//  * Dio1SlowPoll: the SX1262 interrupt line (DIO1) is the only trigger for
//    reading a received command. A line stuck LOW (broken trace, bad solder)
//    would silence the uplink for good. With the line low the radio IRQ
//    status is still read every kDio1SlowPollMs, so a command is heard late
//    instead of never.
//  * arms_holdoff: only a complete 10-byte canonical command frame may arm the
//    telemetry hold-off. A 3-byte frame is accepted by the parser as a compact
//    command, but any stray 0x05 + 2 bytes (noise, another system) must not
//    silence the downlink.
//  * AckPacer: the link is half duplex. The ack of command N is held until the
//    radio has heard nothing for kAckQuietMs, so a burst of commands is not cut
//    by the transmission of an ack (the next command would arrive while the
//    ack is on the air and be lost, and the ack would be lost in turn). Acks
//    still go out in order and none is replaced; at most kAckMaxWaitMs of delay.
//  * sequence_reached: queued / done counters for "everything queued before
//    this point has been written", without waiting for an empty queue (a
//    producer that keeps the queue non-empty cannot hold a flush up forever).

#include <cstddef>
#include <cstdint>

namespace radio_edge {

inline constexpr std::uint32_t kDio1SlowPollMs = 200;
inline constexpr std::uint32_t kAckQuietMs = 40;
inline constexpr std::uint32_t kAckMaxWaitMs = 300;
inline constexpr std::size_t kCanonicalCommandLength = 10;

class Dio1SlowPoll {
public:
    // True when the radio IRQ status should be read now. DIO1 high: always
    // (the normal path). DIO1 low: once per kDio1SlowPollMs.
    bool should_poll(const bool dio1_high, const std::uint64_t now_ms)
    {
        if (dio1_high) return true;
        if (!started_ || now_ms - last_poll_ms_ >= kDio1SlowPollMs) {
            started_ = true;
            last_poll_ms_ = now_ms;
            return true;
        }
        return false;
    }
    // A read found a received frame. Returns true when DIO1 was low at that
    // time, which proves the line is stuck (or the edge was missed).
    bool note_frame(const bool dio1_high)
    {
        if (dio1_high) return false;
        ++missed_;
        return true;
    }
    std::uint32_t frames_found_with_dio1_low() const { return missed_; }

private:
    bool started_ = false;
    std::uint64_t last_poll_ms_ = 0;
    std::uint32_t missed_ = 0;
};

constexpr bool arms_holdoff(const std::size_t frame_length, const bool accepted_as_command,
                            const bool state_allows)
{
    return accepted_as_command && state_allows && frame_length == kCanonicalCommandLength;
}

class AckPacer {
public:
    // A radio frame (command) was received at now_ms.
    void note_command_rx(const std::uint64_t now_ms)
    {
        last_rx_ms_ = now_ms;
        have_rx_ = true;
    }
    // pending: something is waiting in the immediate radio queue.
    bool may_transmit(const bool pending, const std::uint64_t now_ms)
    {
        if (!pending) {
            waiting_ = false;
            return false;
        }
        if (!waiting_) {
            waiting_ = true;
            wait_start_ms_ = now_ms;
        }
        if (!have_rx_ || now_ms - last_rx_ms_ >= kAckQuietMs) return true;
        return now_ms - wait_start_ms_ >= kAckMaxWaitMs;
    }
    // The packet was started: the next one gets its own wait.
    void note_sent() { waiting_ = false; }

private:
    bool have_rx_ = false;
    bool waiting_ = false;
    std::uint64_t last_rx_ms_ = 0;
    std::uint64_t wait_start_ms_ = 0;
};

constexpr bool sequence_reached(const std::uint32_t done, const std::uint32_t target)
{
    return static_cast<std::int32_t>(done - target) >= 0;
}

}  // namespace radio_edge
