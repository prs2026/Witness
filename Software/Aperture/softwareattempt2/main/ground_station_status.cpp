#include "ground_station_status.h"

#include <cmath>
#include <cstring>

namespace {

constexpr std::uint16_t kFixAgeUnknown = 0xFFFF;

std::uint8_t *field(std::uint8_t *frame, unsigned payload_offset) {
    return frame + IRIS_PACKET_ID_LENGTH + payload_offset;
}

void put_u16(std::uint8_t *p, std::uint16_t value) {
    p[0] = static_cast<std::uint8_t>(value >> 8);
    p[1] = static_cast<std::uint8_t>(value);
}

void put_u32(std::uint8_t *p, std::uint32_t value) {
    p[0] = static_cast<std::uint8_t>(value >> 24);
    p[1] = static_cast<std::uint8_t>(value >> 16);
    p[2] = static_cast<std::uint8_t>(value >> 8);
    p[3] = static_cast<std::uint8_t>(value);
}

std::int32_t e7(double degrees) {
    return static_cast<std::int32_t>(std::llround(degrees * 1e7));
}

}  // namespace

void build_ground_station_status(std::uint8_t *frame, const std::int8_t rssi_dbm,
                                 const std::uint32_t now_ms,
                                 const Lc86Nmea::State &gps) {
    std::memset(frame, 0, IRIS_PACKET_GROUND_STATION_STATUS_FRAME_LENGTH);
    frame[0] = IRIS_PACKET_ID_GROUND_STATION_STATUS;
    *field(frame, IRIS_PACKET_GROUND_STATION_STATUS_RSSI_OFFSET) =
        static_cast<std::uint8_t>(rssi_dbm);
    put_u32(field(frame, IRIS_PACKET_GROUND_STATION_STATUS_UPTIME_OFFSET), now_ms);

    // Quality, satellites and HDOP describe the latest GGA, and only while it
    // is younger than Lc86Nmea::kGgaTimeoutMs.
    const bool gga_fresh = Lc86Nmea::gga_fresh(gps, now_ms);
    *field(frame, IRIS_PACKET_GROUND_STATION_STATUS_GPS_FIX_QUALITY_OFFSET) =
        gga_fresh ? gps.fix_quality : 0U;
    *field(frame, IRIS_PACKET_GROUND_STATION_STATUS_GPS_SATELLITES_OFFSET) =
        gga_fresh ? gps.satellites : 0U;
    put_u16(field(frame, IRIS_PACKET_GROUND_STATION_STATUS_GPS_HDOP_OFFSET),
            gga_fresh ? gps.hdop_centi : Lc86Nmea::kHdopUnknown);

    // Position is the last valid fix, however old; its age says how old.
    std::uint16_t fix_age = kFixAgeUnknown;
    if (gps.fix) {
        put_u32(field(frame, IRIS_PACKET_GROUND_STATION_STATUS_LATITUDE_OFFSET),
                static_cast<std::uint32_t>(e7(gps.latitude_degrees)));
        put_u32(field(frame, IRIS_PACKET_GROUND_STATION_STATUS_LONGITUDE_OFFSET),
                static_cast<std::uint32_t>(e7(gps.longitude_degrees)));
        std::uint32_t altitude_bits;
        static_assert(sizeof(altitude_bits) == sizeof(gps.altitude_meters), "float32");
        std::memcpy(&altitude_bits, &gps.altitude_meters, sizeof(altitude_bits));
        put_u32(field(frame, IRIS_PACKET_GROUND_STATION_STATUS_GPS_ALTITUDE_OFFSET),
                altitude_bits);
        const std::uint32_t age = now_ms - gps.fix_ms;
        if (age < kFixAgeUnknown) fix_age = static_cast<std::uint16_t>(age);
    }
    put_u16(field(frame, IRIS_PACKET_GROUND_STATION_STATUS_GPS_FIX_AGE_OFFSET), fix_age);

    // UTC of the last valid RMC, advanced by the local time since it arrived.
    std::uint32_t unix_seconds = 0;
    if (gps.time) {
        const std::int64_t ms = gps.unix_ms + static_cast<std::int64_t>(now_ms - gps.time_ms);
        if (ms > 0 && ms / 1000 <= 0xFFFFFFFFLL)
            unix_seconds = static_cast<std::uint32_t>(ms / 1000);
    }
    put_u32(field(frame, IRIS_PACKET_GROUND_STATION_STATUS_GPS_TIME_OFFSET), unix_seconds);

    frame[IRIS_PACKET_ID_LENGTH + IRIS_PACKET_GROUND_STATION_STATUS_EOF_OFFSET] =
        IRIS_PACKET_EOF_VALUE;
}

static_assert(IRIS_PACKET_ID_LENGTH + IRIS_PACKET_GROUND_STATION_STATUS_DATA_LENGTH +
                  IRIS_PACKET_EOF_LENGTH == IRIS_PACKET_GROUND_STATION_STATUS_FRAME_LENGTH,
              "0xF0 frame length");
static_assert(IRIS_PACKET_GROUND_STATION_STATUS_GPS_FIX_AGE_OFFSET +
                  IRIS_PACKET_GROUND_STATION_STATUS_GPS_FIX_AGE_LENGTH ==
                  IRIS_PACKET_GROUND_STATION_STATUS_DATA_LENGTH,
              "0xF0 fields fill the payload");
