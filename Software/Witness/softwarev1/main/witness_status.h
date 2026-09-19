#pragma once

#include <atomic>
#include <cstdint>

// Thread-safe owner of the first Witness status byte. The second byte is
// reserved by the communications standard and remains zero.
class WitnessStatus final {
public:
    void set(const std::uint8_t mask, const bool active)
    {
        if (active) {
            flags_.fetch_or(mask);
        } else {
            flags_.fetch_and(static_cast<std::uint8_t>(~mask));
        }
    }

    std::uint8_t flags() const
    {
        return flags_.load();
    }

private:
    std::atomic_uint8_t flags_{0};
};
