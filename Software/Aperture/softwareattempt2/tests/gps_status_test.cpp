// Host test for the Aperture LC86 NMEA parser and the 0xF0 status builder.
//   g++ -std=c++17 -Wall -Wextra -Werror -I main tests/gps_status_test.cpp
//       main/lc86_nmea.cpp main/ground_station_status.cpp -o build/gps_status_test.exe
#undef NDEBUG
#include "comms.h"
#include "ground_station_status.h"
#include "lc86_nmea.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

static std::string sentence(const std::string &body) {
    char out[128];
    const std::size_t n = Lc86Nmea::build_sentence(body.c_str(), out, sizeof(out));
    assert(n > 0);
    return std::string(out, n);
}
static void feed(Lc86Nmea &p, const std::string &s, std::uint32_t now) {
    p.consume(reinterpret_cast<const std::uint8_t *>(s.data()), s.size(), now);
}
static std::uint32_t u32(const std::uint8_t *f, unsigned payload_offset) {
    const std::uint8_t *p = f + 1 + payload_offset;
    return (std::uint32_t(p[0]) << 24) | (std::uint32_t(p[1]) << 16) |
           (std::uint32_t(p[2]) << 8) | p[3];
}
static std::uint16_t u16(const std::uint8_t *f, unsigned payload_offset) {
    const std::uint8_t *p = f + 1 + payload_offset;
    return static_cast<std::uint16_t>((p[0] << 8) | p[1]);
}
static std::uint8_t u8(const std::uint8_t *f, unsigned payload_offset) {
    return f[1 + payload_offset];
}
static float f32(const std::uint8_t *f, unsigned payload_offset) {
    const std::uint32_t bits = u32(f, payload_offset);
    float v;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
}

static void test_build_sentence() {
    // Checksums printed in the Quectel protocol specifications.
    assert(sentence("PAIR050,1000") == "$PAIR050,1000*12\r\n");
    assert(sentence("PAIR062,0,3") == "$PAIR062,0,3*3D\r\n");
    assert(sentence("PAIR051") == "$PAIR051*3E\r\n");
    assert(sentence("PMTK220,1000") == "$PMTK220,1000*1F\r\n");
    assert(sentence("PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0") ==
           "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*35\r\n");
    assert(sentence("PMTK314,-1") == "$PMTK314,-1*04\r\n");
    // Commands actually sent at boot.
    std::printf("%s", sentence("PAIR050,100").c_str());
    std::printf("%s", sentence("PAIR062,0,1").c_str());
    std::printf("%s", sentence("PAIR062,4,1").c_str());
    char tiny[8];
    assert(Lc86Nmea::build_sentence("PAIR050,100", tiny, sizeof(tiny)) == 0);
}

static void test_unix_seconds() {
    assert(Lc86Nmea::unix_seconds(1970, 1, 1, 0, 0, 0) == 0);
    assert(Lc86Nmea::unix_seconds(2000, 3, 1, 0, 0, 0) == 951868800);
    assert(Lc86Nmea::unix_seconds(2024, 2, 29, 12, 34, 56) == 1709210096);
    assert(Lc86Nmea::unix_seconds(2026, 11, 9, 13, 19, 10) == 1794230350);
}

static void test_parser() {
    Lc86Nmea p;
    assert(!p.state().gga && !p.state().fix && !p.state().time);

    // Published NMEA example with fixed checksum, byte by byte.
    const std::string known =
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    for (char c : known) feed(p, std::string(1, c), 100);
    auto s = p.state();
    assert(s.gga && s.fix && s.gga_ms == 100 && s.fix_ms == 100);
    assert(s.fix_quality == 1 && s.satellites == 8 && s.hdop_centi == 90);
    assert(std::abs(s.latitude_degrees - 48.1173) < 1e-9);
    assert(std::abs(s.longitude_degrees - 11.5166666667) < 1e-9);
    assert(std::abs(s.altitude_meters - 545.4F) < 1e-3F);
    assert(Lc86Nmea::gga_fresh(s, 3099) && !Lc86Nmea::gga_fresh(s, 3100));

    // Bad checksum is ignored and counted.
    feed(p, "$GPGGA,123520,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48\r\n", 150);
    assert(p.state().gga_ms == 100 && p.checksum_error_count() == 1);
    feed(p, "$GPGGA,123520,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,\r\n", 150);
    assert(p.state().gga_ms == 100 && p.checksum_error_count() == 2);

    // Any talker; southern/western hemisphere; negative altitude.
    for (const char *talker : {"GN", "GL", "GA", "GB", "GP"}) {
        feed(p, sentence(std::string(talker) +
                         "GGA,123521.100,2447.09094,S,12100.52369,W,2,12,1.23,-12.5,M,0,M,,"),
             200);
        s = p.state();
        assert(s.fix && s.fix_ms == 200 && s.fix_quality == 2 && s.satellites == 12);
        assert(s.hdop_centi == 123 && s.altitude_meters == -12.5F);
        assert(std::abs(s.latitude_degrees + 24.784849) < 1e-9);
        assert(std::abs(s.longitude_degrees + 121.0087281667) < 1e-9);
    }

    // No-fix GGA updates quality/sats/HDOP but keeps the last valid fix.
    feed(p, sentence("GNGGA,123522.000,,,,,0,03,99.99,,M,,M,,"), 300);
    s = p.state();
    assert(s.gga_ms == 300 && s.fix_quality == 0 && s.satellites == 3 && s.hdop_centi == 9999);
    assert(s.fix && s.fix_ms == 200 && std::abs(s.latitude_degrees + 24.784849) < 1e-9);

    // Malformed fields: no position update, unknown HDOP, zero sats.
    for (const char *body : {
             "GPGGA,1,4860.000,N,01131.000,E,1,x8,,10,M,0,M,,",
             "GPGGA,1,9100.000,N,01131.000,E,1,4.5,abc,10,M,0,M,,",
             "GPGGA,1,4807.038,N,18100.000,E,1,300,-1,10,M,0,M,,",
             "GPGGA,1,4807.038,N,01131.000,E,1,,0.9,nan,M,0,M,,",
             "GPGGA,1,4807.038,X,01131.000,E,1,08,0.9,10,M,0,M,,",
             "GPGGA,1,4807.038,N,01131.000,E,1,08,0.9,10,F,0,M,,",
             "GPGGA,1,4807.038,N,01131.000,E,9,08,0.9,10,M,0,M,,"}) {
        feed(p, sentence(body), 310);
        s = p.state();
        assert(s.gga_ms == 310 && s.fix_ms == 200);
    }
    feed(p, sentence("GPGGA,1,,,,,1,x8,,10,M,0,M,,"), 320);
    assert(p.state().satellites == 0 && p.state().hdop_centi == Lc86Nmea::kHdopUnknown);
    feed(p, sentence("GPGGA,1,,,,,1,08,800.0,10,M,0,M,,"), 320);
    assert(p.state().hdop_centi == 0xFFFE);  // Clamped below the unknown marker.
    feed(p, sentence("GPGGA,123519,4807.038,N,01131.000,E,1,08"), 330);
    assert(p.state().gga_ms == 320);  // Too few fields.

    // RMC: valid date/time with milliseconds; any talker.
    feed(p, sentence("GNRMC,131910.250,A,4807.038,N,01131.000,E,0.01,0.0,091126,,,A,V"), 400);
    s = p.state();
    assert(s.time && s.time_ms == 400 && s.unix_ms == 1794230350250LL);
    feed(p, sentence("GPRMC,000000,A,4807.038,N,01131.000,E,0,0,290224,,"), 410);
    assert(p.state().unix_ms == 1709164800000LL);
    // Void status, bad date or time: ignored.
    for (const char *body : {
             "GNRMC,131911.000,V,,,,,,,091126,,,N",
             "GNRMC,131911.000,A,,,,,,,321126,,,A",
             "GNRMC,131911.000,A,,,,,,,091326,,,A",
             "GNRMC,251911.000,A,,,,,,,091126,,,A",
             "GNRMC,1319,A,,,,,,,091126,,,A",
             "GNRMC,131911.0x0,A,,,,,,,091126,,,A",
             "GNRMC,131911.000,A,,,,,,,0911,,,A"}) {
        feed(p, sentence(body), 420);
        assert(p.state().time_ms == 410);
    }

    // Acknowledgements.
    feed(p, "$PAIR001,050,0*3E\r\n", 500);
    assert(p.last_ack().count == 1 && p.last_ack().pair && p.last_ack().command == 50 &&
           p.last_ack().result == 0);
    feed(p, "$PAIR001,062,0*3F\r\n", 500);
    assert(p.last_ack().count == 2 && p.last_ack().command == 62);
    feed(p, "$PMTK001,314,3*36\r\n", 500);
    assert(p.last_ack().count == 3 && !p.last_ack().pair && p.last_ack().command == 314 &&
           p.last_ack().result == 3);
    feed(p, "$PAIR001,050,0*3F\r\n", 500);  // Bad checksum.
    assert(p.last_ack().count == 3);

    // Overflow then resync; GSV etc. ignored; clock wrap for freshness.
    feed(p, "$" + std::string(300, 'x') + known, 600);
    assert(p.state().fix_ms == 600);
    feed(p, sentence("GPGSV,1,1,00"), 700);
    assert(p.state().gga_ms == 600);
    s = p.state();
    s.gga_ms = 0xFFFFFF00U;
    assert(Lc86Nmea::gga_fresh(s, 100));
}

static void test_status_packet() {
    static_assert(IRIS_PACKET_GROUND_STATION_STATUS_FRAME_LENGTH == 29, "frame");
    std::uint8_t f[IRIS_PACKET_GROUND_STATION_STATUS_FRAME_LENGTH];

    // No GPS ever: zero position/time, unknown HDOP and fix age.
    std::memset(f, 0xAA, sizeof(f));
    build_ground_station_status(f, -127, 0x01020304U, Lc86Nmea::State{});
    const std::uint8_t expected_empty[29] = {
        0xF0, 0x81, 0x01, 0x02, 0x03, 0x04, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0x0A};
    assert(std::memcmp(f, expected_empty, sizeof(f)) == 0);

    // Fresh fix.
    Lc86Nmea p;
    feed(p, sentence("GNGGA,131910.200,2447.09094,S,12100.52369,W,1,09,0.87,123.25,M,0,M,,"),
         10000);
    feed(p, sentence("GNRMC,131910.200,A,2447.09094,S,12100.52369,W,0,0,091126,,,A,V"), 10000);
    build_ground_station_status(f, -42, 10500, p.state());
    assert(f[0] == IRIS_PACKET_ID_GROUND_STATION_STATUS);
    assert(static_cast<std::int8_t>(u8(f, IRIS_PACKET_GROUND_STATION_STATUS_RSSI_OFFSET)) == -42);
    assert(u32(f, IRIS_PACKET_GROUND_STATION_STATUS_UPTIME_OFFSET) == 10500);
    assert(u8(f, IRIS_PACKET_GROUND_STATION_STATUS_GPS_FIX_QUALITY_OFFSET) == 1);
    assert(u8(f, IRIS_PACKET_GROUND_STATION_STATUS_GPS_SATELLITES_OFFSET) == 9);
    assert(static_cast<std::int32_t>(u32(f, IRIS_PACKET_GROUND_STATION_STATUS_LATITUDE_OFFSET)) ==
           -247848490);
    assert(static_cast<std::int32_t>(u32(f, IRIS_PACKET_GROUND_STATION_STATUS_LONGITUDE_OFFSET)) ==
           -1210087282);
    assert(f32(f, IRIS_PACKET_GROUND_STATION_STATUS_GPS_ALTITUDE_OFFSET) == 123.25F);
    assert(u32(f, IRIS_PACKET_GROUND_STATION_STATUS_GPS_TIME_OFFSET) == 1794230350U);
    assert(u16(f, IRIS_PACKET_GROUND_STATION_STATUS_GPS_HDOP_OFFSET) == 87);
    assert(u16(f, IRIS_PACKET_GROUND_STATION_STATUS_GPS_FIX_AGE_OFFSET) == 500);
    assert(f[28] == IRIS_PACKET_EOF_VALUE);
    // Exact bytes, computed independently with Python struct.pack:
    // lat -247848490 = 0xF13A21D6, lon -1210087282 = 0xB7DF888E,
    // 123.25f = 0x42F68000, 1794230350 = 0x6AF1C84E.
    const std::uint8_t expected_fix[29] = {
        0xF0, 0xD6, 0x00, 0x00, 0x29, 0x04, 0x01, 0x09, 0xF1, 0x3A, 0x21, 0xD6,
        0xB7, 0xDF, 0x88, 0x8E, 0x42, 0xF6, 0x80, 0x00, 0x6A, 0xF1, 0xC8, 0x4E,
        0x00, 0x57, 0x01, 0xF4, 0x0A};
    assert(std::memcmp(f, expected_fix, sizeof(f)) == 0);

    // Time advances with local time since the RMC (13:19:10.200 + 2.799 s).
    build_ground_station_status(f, -42, 12799, p.state());
    assert(u32(f, IRIS_PACKET_GROUND_STATION_STATUS_GPS_TIME_OFFSET) == 1794230352U);
    build_ground_station_status(f, -42, 12800, p.state());
    assert(u32(f, IRIS_PACKET_GROUND_STATION_STATUS_GPS_TIME_OFFSET) == 1794230353U);

    // No-fix GGA: quality/sats from it; position kept with growing age.
    feed(p, sentence("GNGGA,131915.000,,,,,0,02,25.5,,M,,M,,"), 15000);
    build_ground_station_status(f, -42, 15100, p.state());
    assert(u8(f, IRIS_PACKET_GROUND_STATION_STATUS_GPS_FIX_QUALITY_OFFSET) == 0);
    assert(u8(f, IRIS_PACKET_GROUND_STATION_STATUS_GPS_SATELLITES_OFFSET) == 2);
    assert(u16(f, IRIS_PACKET_GROUND_STATION_STATUS_GPS_HDOP_OFFSET) == 2550);
    assert(static_cast<std::int32_t>(u32(f, IRIS_PACKET_GROUND_STATION_STATUS_LATITUDE_OFFSET)) ==
           -247848490);
    assert(u16(f, IRIS_PACKET_GROUND_STATION_STATUS_GPS_FIX_AGE_OFFSET) == 5100);

    // GGA older than 3 s: quality/sats 0, HDOP unknown; position still held.
    build_ground_station_status(f, -42, 18000, p.state());
    assert(u8(f, IRIS_PACKET_GROUND_STATION_STATUS_GPS_FIX_QUALITY_OFFSET) == 0);
    assert(u8(f, IRIS_PACKET_GROUND_STATION_STATUS_GPS_SATELLITES_OFFSET) == 0);
    assert(u16(f, IRIS_PACKET_GROUND_STATION_STATUS_GPS_HDOP_OFFSET) == 0xFFFF);
    assert(u16(f, IRIS_PACKET_GROUND_STATION_STATUS_GPS_FIX_AGE_OFFSET) == 8000);
    assert(f32(f, IRIS_PACKET_GROUND_STATION_STATUS_GPS_ALTITUDE_OFFSET) == 123.25F);

    // Fix age saturates at 0xFFFF (65.535 s or older).
    build_ground_station_status(f, -42, 10000 + 65534, p.state());
    assert(u16(f, IRIS_PACKET_GROUND_STATION_STATUS_GPS_FIX_AGE_OFFSET) == 65534);
    build_ground_station_status(f, -42, 10000 + 65535, p.state());
    assert(u16(f, IRIS_PACKET_GROUND_STATION_STATUS_GPS_FIX_AGE_OFFSET) == 0xFFFF);
    assert(f[28] == IRIS_PACKET_EOF_VALUE);
}

int main() {
    test_build_sentence();
    test_unix_seconds();
    test_parser();
    test_status_packet();
    std::puts("Aperture GPS status tests passed");
}
