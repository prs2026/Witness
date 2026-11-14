#pragma once

#include <cstddef>
#include <cstdint>

#include "comms.h"

// 0x08 maintenance status beacon (comms.h) encode/decode. Pure: no ESP-IDF
// dependency (host test tests/image_info_test.cpp). The maintenance image
// encodes; the decoder is what the ground-station tools mirror.
namespace maint_beacon {

struct Status {
    std::uint8_t mode = IRIS_MAINT_MODE_WIFI;
    std::uint8_t flags = 0;
    std::uint8_t store_version = IRIS_MAINT_STORE_FORMAT_VERSION;
    std::uint8_t command_set_version = IRIS_MAINT_COMMAND_SET_VERSION;
    std::uint8_t radio_settings_version = IRIS_MAINT_RADIO_SETTINGS_VERSION;
    std::uint16_t maint_image_id = 0;
    std::uint16_t flight_image_id = 0;
    std::uint16_t battery_mv = 0;
    std::uint32_t uptime_ms = 0;
    std::uint8_t log_files = 0;
};

constexpr std::size_t kFrameLength = IRIS_PACKET_MAINTENANCE_STATUS_FRAME_LENGTH;

inline void put_u16(std::uint8_t *d, const std::uint16_t v)
{
    d[0] = static_cast<std::uint8_t>(v >> 8);
    d[1] = static_cast<std::uint8_t>(v);
}

// Writes the whole frame (ID, 17 data bytes, EOF); returns kFrameLength.
inline std::size_t encode(const Status &s, std::uint8_t *out)
{
    out[0] = IRIS_PACKET_ID_MAINTENANCE_STATUS;
    std::uint8_t *d = out + IRIS_PACKET_ID_LENGTH;
    d[0] = IRIS_MAINT_BEACON_VERSION;
    d[1] = s.mode;
    d[2] = s.flags;
    d[3] = s.store_version;
    d[4] = s.command_set_version;
    d[5] = s.radio_settings_version;
    put_u16(d + 6, s.maint_image_id);
    put_u16(d + 8, s.flight_image_id);
    put_u16(d + 10, s.battery_mv);
    d[12] = static_cast<std::uint8_t>(s.uptime_ms >> 24);
    d[13] = static_cast<std::uint8_t>(s.uptime_ms >> 16);
    d[14] = static_cast<std::uint8_t>(s.uptime_ms >> 8);
    d[15] = static_cast<std::uint8_t>(s.uptime_ms);
    d[16] = s.log_files;
    out[kFrameLength - 1] = IRIS_PACKET_EOF_VALUE;
    return kFrameLength;
}

// Decodes a whole frame; false for a wrong ID/length/EOF or beacon version.
inline bool decode(const std::uint8_t *frame, const std::size_t length, Status &s)
{
    if (frame == nullptr || length != kFrameLength || frame[0] != IRIS_PACKET_ID_MAINTENANCE_STATUS ||
        frame[kFrameLength - 1] != IRIS_PACKET_EOF_VALUE) {
        return false;
    }
    const std::uint8_t *d = frame + IRIS_PACKET_ID_LENGTH;
    if (d[0] != IRIS_MAINT_BEACON_VERSION) return false;
    s.mode = d[1];
    s.flags = d[2];
    s.store_version = d[3];
    s.command_set_version = d[4];
    s.radio_settings_version = d[5];
    s.maint_image_id = static_cast<std::uint16_t>((d[6] << 8) | d[7]);
    s.flight_image_id = static_cast<std::uint16_t>((d[8] << 8) | d[9]);
    s.battery_mv = static_cast<std::uint16_t>((d[10] << 8) | d[11]);
    s.uptime_ms = (static_cast<std::uint32_t>(d[12]) << 24) | (static_cast<std::uint32_t>(d[13]) << 16) |
                  (static_cast<std::uint32_t>(d[14]) << 8) | d[15];
    s.log_files = d[16];
    return true;
}

}  // namespace maint_beacon
