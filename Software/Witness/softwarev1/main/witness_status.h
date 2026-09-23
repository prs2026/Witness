#pragma once

#include <atomic>
#include <cstdint>

#include "comms.h"

// Thread-safe owner of the canonical two-byte Witness status field: health and
// readiness flags in byte 0, and the current flight state in byte 1.
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

    void set_flight_state(const std::uint8_t state)
    {
        flight_state_.store(static_cast<std::uint8_t>(
            (state << IRIS_WITNESS_STATUS_FLIGHT_STATE_SHIFT) &
            IRIS_WITNESS_STATUS_FLIGHT_STATE_MASK));
    }

    std::uint8_t flight_state() const
    {
        return flight_state_.load();
    }

    // Writes the canonical two-byte Witness status field.
    void write(std::uint8_t *destination) const
    {
        destination[0] = flags();
        destination[1] = flight_state();
    }

private:
    std::atomic_uint8_t flags_{0};
    std::atomic_uint8_t flight_state_{0};
};
