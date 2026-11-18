#pragma once

// PC -> Aperture command frame assembler with resync (pure logic, host test).
// A command frame is 10 bytes: 0x05 ... 0x0A. The old parser threw the whole
// buffer away when byte 10 was not 0x0A, so one lost USB byte destroyed that
// frame AND the next one. Here, on a bad frame the buffer is realigned to the
// next 0x05 already collected, so only the damaged frame is lost.

#include <cstddef>
#include <cstdint>
#include <cstring>

class CommandAssembler {
public:
    static constexpr std::size_t kLength = 10;
    static constexpr std::uint8_t kId = 0x05;
    static constexpr std::uint8_t kEof = 0x0A;

    // Feed one byte. True when a complete frame (0x05..0x0A) is available via
    // frame(); valid until the next feed().
    bool feed(std::uint8_t byte)
    {
        if (length_ == 0 && byte != kId) { ++discarded_; return false; }
        buf_[length_++] = byte;
        if (length_ < kLength) return false;
        if (buf_[kLength - 1] == kEof) {
            std::memcpy(frame_, buf_, kLength);
            length_ = 0;
            return true;
        }
        // Bad frame: restart at the next 0x05 among the bytes already held.
        std::size_t k = 1;
        while (k < kLength && buf_[k] != kId) ++k;
        ++resyncs_;
        if (k >= kLength) { length_ = 0; return false; }
        std::memmove(buf_, buf_ + k, kLength - k);
        length_ = kLength - k;
        return false;
    }
    const std::uint8_t *frame() const { return frame_; }
    std::uint32_t resyncs() const { return resyncs_; }
    std::uint32_t discarded() const { return discarded_; }

private:
    std::uint8_t buf_[kLength] = {};
    std::uint8_t frame_[kLength] = {};
    std::size_t length_ = 0;
    std::uint32_t resyncs_ = 0;
    std::uint32_t discarded_ = 0;
};
