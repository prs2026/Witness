#pragma once

#include <atomic>
#include <cstdint>

#include "comms.h"

// Thread-safe owner of the canonical two-byte Witness status field: health and
// readiness flags in byte 0; the flight state (bits 0..2) plus the GPS-locked
// and HITL-active flags (bits 3 and 4) in byte 1.
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

    // Byte-1 flags: IRIS_WITNESS_STATUS_GPS_LOCKED_MASK and
    // IRIS_WITNESS_STATUS_HITL_ACTIVE_MASK. Other bits are ignored.
    void set_state_flag(const std::uint8_t mask, const bool active)
    {
        const std::uint8_t bits = static_cast<std::uint8_t>(mask & kStateFlagMask);
        if (active) {
            state_flags_.fetch_or(bits);
        } else {
            state_flags_.fetch_and(static_cast<std::uint8_t>(~bits));
        }
    }

    std::uint8_t state_flags() const
    {
        return state_flags_.load();
    }

    // Writes the canonical two-byte Witness status field.
    void write(std::uint8_t *destination) const
    {
        destination[0] = flags();
        destination[1] = static_cast<std::uint8_t>(
            flight_state() | state_flags());
    }

private:
    static constexpr std::uint8_t kStateFlagMask =
        IRIS_WITNESS_STATUS_GPS_LOCKED_MASK |
        IRIS_WITNESS_STATUS_HITL_ACTIVE_MASK;
    static_assert((kStateFlagMask & IRIS_WITNESS_STATUS_FLIGHT_STATE_MASK) == 0U,
                  "byte-1 flags must not overlap the flight state");

    std::atomic_uint8_t flags_{0};
    std::atomic_uint8_t flight_state_{0};
    std::atomic_uint8_t state_flags_{0};
};
