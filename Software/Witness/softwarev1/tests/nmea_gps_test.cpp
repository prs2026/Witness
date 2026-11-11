#include "nmea_gps.h"
#include "comms.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>

static std::string sentence(const std::string &body) {
    unsigned checksum = 0;
    for (unsigned char c : body) checksum ^= c;
    char tail[8];
    std::snprintf(tail, sizeof(tail), "*%02X\r\n", checksum);
    return "$" + body + tail;
}
static void feed(NmeaGps &parser, const std::string &s, std::uint32_t now) {
    parser.consume(reinterpret_cast<const std::uint8_t *>(s.data()), s.size(), now);
}
int main() {
    NmeaGps parser;
    assert(!parser.latest_fix().valid);
    assert(parser.gga_sentence_count() == 0);
    // Published NMEA example, independent fixed checksum; byte-by-byte input.
    const std::string known = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    for (const auto c : known) feed(parser, std::string(1, c), 100);
    auto fix = parser.latest_fix();
    assert(fix.valid);
    assert(std::abs(fix.latitude_degrees - 48.1173) < 1e-8);
    assert(std::abs(fix.longitude_degrees - 11.5166666667) < 1e-8);
    assert(std::abs(fix.altitude_meters - 545.4F) < 0.001F);
    assert(std::lround(fix.latitude_degrees * IRIS_FIELD_GPS_COORDINATE_SCALE) == 481173);
    assert(NmeaGps::fresh(fix, 3099) && !NmeaGps::fresh(fix, 3100));
    assert(fix.gga && fix.satellites == 8 && fix.fix_quality == 1);
    assert(NmeaGps::gga_fresh(fix, 3099) && !NmeaGps::gga_fresh(fix, 3100));
    feed(parser, "$GPGGA,123519,4807.038,N,01131.000,E,0,08,0.9,545.4,M,46.9,M,,*47\r\n", 200);
    assert(parser.latest_fix().received_ms == 100); // Bad checksum ignored.
    assert(parser.gga_sentence_count() == 1);
    feed(parser, sentence("GNGGA,123520,2447.09094,S,12100.52369,W,1,10,0.8,-12.5,M,0,M,,"), 300);
    fix = parser.latest_fix();
    assert(fix.valid && std::abs(fix.latitude_degrees + 24.784849) < 1e-8);
    assert(std::abs(fix.longitude_degrees + 121.0087281667) < 1e-8);
    assert(fix.altitude_meters == -12.5F);
    feed(parser, sentence("GNGGA,123521,,,,,0,00,99.9,,M,,M,,"), 400);
    assert(!parser.latest_fix().valid);
    assert(parser.latest_fix().satellites == 0 && parser.latest_fix().gga);
    assert(parser.gga_sentence_count() == 3); // No fix still proves UART reception.
    for (const auto &body : {
         "GPGGA,123519,4860.000,N,01131.000,E,1,08,0.9,10,M,0,M,,",
         "GPGGA,123519,9100.000,N,01131.000,E,1,08,0.9,10,M,0,M,,",
         "GPGGA,123519,4807.038,N,18100.000,E,1,08,0.9,10,M,0,M,,",
         "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,nan,M,0,M,,",
         "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,10,F,0,M,,"}) {
        feed(parser, sentence(body), 500);
        fix = parser.latest_fix();
        assert(!fix.valid && fix.latitude_degrees == 0 && fix.altitude_meters == 0);
        assert(fix.gga && fix.satellites == 8 && fix.fix_quality == 1);
    }
    // No fix (quality 0) still reports satellites; position stays zero and the
    // GGA, not the position, is fresh.
    feed(parser, sentence("GNGGA,123522,4807.038,N,01131.000,E,0,05,99.9,545.4,M,,M,,"), 550);
    fix = parser.latest_fix();
    assert(!fix.valid && fix.gga && fix.satellites == 5 && fix.fix_quality == 0);
    assert(fix.latitude_degrees == 0 && fix.longitude_degrees == 0 && fix.altitude_meters == 0);
    assert(fix.received_ms == 550 && NmeaGps::gga_fresh(fix, 600) && !NmeaGps::fresh(fix, 600));
    // DGPS quality, 12 satellites; malformed satellite fields report zero.
    feed(parser, sentence("GPGGA,123523,4807.038,N,01131.000,E,2,12,0.9,545.4,M,46.9,M,,"), 560);
    fix = parser.latest_fix();
    assert(fix.valid && fix.fix_quality == 2 && fix.satellites == 12);
    for (const auto &body : {
         "GPGGA,123524,4807.038,N,01131.000,E,1,,0.9,545.4,M,46.9,M,,",
         "GPGGA,123524,4807.038,N,01131.000,E,1,x5,0.9,545.4,M,46.9,M,,",
         "GPGGA,123524,4807.038,N,01131.000,E,1,4.5,0.9,545.4,M,46.9,M,,",
         "GPGGA,123524,4807.038,N,01131.000,E,1,-3,0.9,545.4,M,46.9,M,,",
         "GPGGA,123524,4807.038,N,01131.000,E,1,300,0.9,545.4,M,46.9,M,,"}) {
        feed(parser, sentence(body), 570);
        fix = parser.latest_fix();
        assert(fix.valid && fix.satellites == 0);
    }
    feed(parser, "$" + std::string(300, 'x') + known, 600);
    assert(parser.latest_fix().valid); // Overflow and resynchronization.
    feed(parser, sentence("GPRMC,123519,V,,,,,,,,,,"), 700);
    assert(parser.latest_fix().received_ms == 600); // Other sentences ignored.
    fix.received_ms = 0xFFFFFF00U;
    assert(NmeaGps::fresh(fix, 100)); // Millisecond clock wrap.
    assert(NmeaGps::gga_fresh(fix, 100));
    assert(!NmeaGps::gga_fresh(NmeaGps::Fix{}, 0)); // Never received.
    // Dilution of precision: HDOP from every GGA, PDOP/VDOP from a fresh GSA.
    {
        NmeaGps p;
        feed(p, sentence("GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"), 1000);
        auto f = p.latest_fix();
        assert(std::abs(f.hdop - 0.9F) < 1e-6F && std::isnan(f.vdop) && std::isnan(f.pdop));
        assert(f.sequence == 1);
        feed(p, sentence("GNGSA,A,3,01,02,03,04,05,06,,,,,,,1.8,1.0,1.5,1"), 1010);
        f = p.latest_fix();  // GSA attaches to the current fix
        assert(std::abs(f.vdop - 1.5F) < 1e-6F && std::abs(f.pdop - 1.8F) < 1e-6F);
        assert(p.gsa_sentence_count() == 1);
        feed(p, sentence("GNGGA,123519.04,4807.038,N,01131.000,E,1,08,1.1,545.4,M,46.9,M,,"), 1040);
        f = p.latest_fix();
        assert(std::abs(f.hdop - 1.1F) < 1e-6F && std::abs(f.vdop - 1.5F) < 1e-6F);
        assert(f.sequence == 2);
        // GSA too old: VDOP dropped; HDOP still from GGA.
        feed(p, sentence("GNGGA,123522,4807.038,N,01131.000,E,1,08,1.2,545.4,M,46.9,M,,"),
             1010 + NmeaGps::kDopTimeoutMs);
        f = p.latest_fix();
        assert(std::isnan(f.vdop) && std::abs(f.hdop - 1.2F) < 1e-6F);
        // No-fix GSA and 99.99 DOPs are not used; empty GGA HDOP falls back to GSA.
        feed(p, sentence("GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,1"), 20000);
        feed(p, sentence("GNGGA,123523,4807.038,N,01131.000,E,1,08,,545.4,M,46.9,M,,"), 20010);
        f = p.latest_fix();
        assert(std::isnan(f.hdop) && std::isnan(f.vdop));
        feed(p, sentence("GPGSA,A,3,01,02,03,04,,,,,,,,,2.5,2.0,1.6"), 20020);
        feed(p, sentence("GNGGA,123524,4807.038,N,01131.000,E,1,08,,545.4,M,46.9,M,,"), 20030);
        f = p.latest_fix();
        assert(std::abs(f.hdop - 2.0F) < 1e-6F && std::abs(f.vdop - 1.6F) < 1e-6F);
        // PX1120S: one GSA per constellation; Galileo/BeiDou with no
        // satellites report "no fix" and 0.0, which must not clear GPS's DOPs.
        feed(p, sentence("GAGSA,A,1,,,,,,,,,,,,,0.0,0.0,0.0,3"), 20040);
        feed(p, sentence("GBGSA,A,1,,,,,,,,,,,,,0.0,0.0,0.0,4"), 20041);
        feed(p, sentence("GNGGA,123525,4807.038,N,01131.000,E,1,08,1.3,545.4,M,46.9,M,,"), 25000);
        f = p.latest_fix();
        assert(std::abs(f.vdop - 1.6F) < 1e-6F && std::abs(f.hdop - 1.3F) < 1e-6F);
        feed(p, sentence("GNGGA,123525,,,,,0,00,99.99,,,,,,"), 25040);
        assert(std::isnan(p.latest_fix().hdop));
    }
    std::puts("NMEA GPS tests passed");
}
