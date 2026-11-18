#pragma once

// Drop-oldest ring of whole frames for the USB output (pure logic, host test
// tests/aperture_logic_test.cpp). Each frame is stored with a 1-byte length,
// so a frame is never split. When full, the OLDEST frames are discarded and
// counted: a host that does not drain USB can never stall the radio loop.
// Not thread-safe: the caller serialises push/peek/pop_if.

#include <cstddef>
#include <cstdint>

template <std::size_t Capacity>
class FrameRing {
public:
    static constexpr std::size_t kMaxFrame = 255;

    // Returns false (and counts a drop) if the frame cannot ever fit.
    bool push(const std::uint8_t *data, std::size_t length)
    {
        if (length == 0 || length > kMaxFrame || length + 1 > Capacity) {
            ++drops_;
            return false;
        }
        while (Capacity - used_ < length + 1) {
            drop_oldest();
            ++drops_;
        }
        put((tail_), static_cast<std::uint8_t>(length));
        std::size_t pos = advance(tail_, 1);
        for (std::size_t i = 0; i < length; ++i) {
            buf_[pos] = data[i];
            pos = advance(pos, 1);
        }
        tail_ = pos;
        used_ += length + 1;
        ++count_;
        return true;
    }

    // Copies the oldest frame to out (max bytes); returns its length, 0 if
    // empty. seq identifies it for pop_if().
    std::size_t peek(std::uint8_t *out, std::size_t max, std::uint32_t &seq) const
    {
        if (count_ == 0) return 0;
        const std::size_t length = buf_[head_];
        if (length > max) return 0;
        std::size_t pos = advance(head_, 1);
        for (std::size_t i = 0; i < length; ++i) {
            out[i] = buf_[pos];
            pos = advance(pos, 1);
        }
        seq = removed_;
        return length;
    }

    // Removes the oldest frame only if it is still the one peeked (a push may
    // have dropped it meanwhile).
    bool pop_if(std::uint32_t seq)
    {
        if (count_ == 0 || seq != removed_) return false;
        drop_oldest();
        return true;
    }

    std::uint32_t drops() const { return drops_; }
    std::size_t frames() const { return count_; }

private:
    static std::size_t advance(std::size_t pos, std::size_t n) { return (pos + n) % Capacity; }
    void put(std::size_t pos, std::uint8_t v) { buf_[pos] = v; }
    void drop_oldest()
    {
        const std::size_t length = buf_[head_];
        head_ = advance(head_, length + 1);
        used_ -= length + 1;
        --count_;
        ++removed_;
    }

    std::uint8_t buf_[Capacity] = {};
    std::size_t head_ = 0, tail_ = 0, used_ = 0, count_ = 0;
    std::uint32_t drops_ = 0;
    std::uint32_t removed_ = 0;   // frames removed so far (popped or dropped)
};
