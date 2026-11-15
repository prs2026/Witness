#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "comms.h"

// Encoder for the 0x09 landed position packet (comms.h). Pure; host tested in
// tests/landed_logic_test.cpp.
namespace landed {

struct Position {
    bool valid = false;            // a fix has been saved
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    float altitude_m = 0.0F;
    std::uint8_t satellites = 0;
    bool stale = false;            // latest GPS cycle did not refresh it
    std::uint32_t fix_age_s = IRIS_LANDED_FIX_AGE_NONE;
};

inline void put_u32(std::uint8_t *out, std::uint32_t v)
{
    out[0] = static_cast<std::uint8_t>(v >> 24);
    out[1] = static_cast<std::uint8_t>(v >> 16);
    out[2] = static_cast<std::uint8_t>(v >> 8);
    out[3] = static_cast<std::uint8_t>(v);
}

inline std::int32_t coordinate_counts(double degrees)
{
    const double scaled = std::round(degrees * static_cast<double>(IRIS_FIELD_LANDED_COORDINATE_SCALE));
    if (scaled > 2147483647.0) return 2147483647;
    if (scaled < -2147483648.0) return static_cast<std::int32_t>(-2147483647 - 1);
    return static_cast<std::int32_t>(scaled);
}

// Writes the complete frame (ID, 26-byte payload, EOF) and returns its length.
inline std::size_t encode_position(std::uint8_t *frame, const std::uint8_t status[2],
                                   std::uint32_t uptime_ms, const Position &position,
                                   std::uint16_t battery_encoded, bool ping_reply,
                                   bool utc_aligned)
{
    std::uint8_t *p = frame + IRIS_PACKET_ID_LENGTH;
    std::memset(frame, 0, IRIS_PACKET_LANDED_POSITION_FRAME_LENGTH);
    frame[0] = IRIS_PACKET_ID_LANDED_POSITION;
    p[IRIS_PACKET_LANDED_STATUS_OFFSET] = status[0];
    p[IRIS_PACKET_LANDED_STATUS_OFFSET + 1] = status[1];
    put_u32(p + IRIS_PACKET_LANDED_UPTIME_OFFSET, uptime_ms);
    if (position.valid) {
        put_u32(p + IRIS_PACKET_LANDED_LATITUDE_OFFSET,
                static_cast<std::uint32_t>(coordinate_counts(position.latitude_deg)));
        put_u32(p + IRIS_PACKET_LANDED_LONGITUDE_OFFSET,
                static_cast<std::uint32_t>(coordinate_counts(position.longitude_deg)));
        std::uint32_t bits = 0;
        std::memcpy(&bits, &position.altitude_m, sizeof(bits));
        put_u32(p + IRIS_PACKET_LANDED_ALTITUDE_OFFSET, bits);
        p[IRIS_PACKET_LANDED_SATELLITES_OFFSET] = position.satellites;
    }
    put_u32(p + IRIS_PACKET_LANDED_FIX_AGE_OFFSET,
            position.valid ? position.fix_age_s : IRIS_LANDED_FIX_AGE_NONE);
    std::uint8_t flags = 0;
    if (position.valid) flags |= IRIS_LANDED_FLAG_POSITION_VALID;
    if (position.stale) flags |= IRIS_LANDED_FLAG_STALE;
    if (ping_reply) flags |= IRIS_LANDED_FLAG_PING_REPLY;
    if (utc_aligned) flags |= IRIS_LANDED_FLAG_UTC_ALIGNED;
    p[IRIS_PACKET_LANDED_FLAGS_OFFSET] = flags;
    p[IRIS_PACKET_LANDED_BATTERY_OFFSET] = static_cast<std::uint8_t>(battery_encoded >> 8);
    p[IRIS_PACKET_LANDED_BATTERY_OFFSET + 1] = static_cast<std::uint8_t>(battery_encoded);
    frame[IRIS_PACKET_LANDED_POSITION_FRAME_LENGTH - 1] = IRIS_PACKET_EOF_VALUE;
    return IRIS_PACKET_LANDED_POSITION_FRAME_LENGTH;
}

}  // namespace landed
