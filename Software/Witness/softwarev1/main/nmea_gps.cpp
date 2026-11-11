#include "nmea_gps.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace {
int hex(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}
bool number(const char *s, double &value) {
    // NMEA decimal fields do not contain whitespace, exponents or NaNs.
    const char *p = s;
    if (*p == '-' || *p == '+') ++p;
    bool digit = false, point = false;
    for (; *p; ++p) {
        if (*p >= '0' && *p <= '9') digit = true;
        else if (*p == '.' && !point) point = true;
        else return false;
    }
    if (!digit) return false;
    value = std::strtod(s, nullptr);
    return std::isfinite(value);
}
// DOP fields: positive and finite; 99.99 is the receivers' "no solution".
float dop(const char *s) {
    double value;
    if (!number(s, value) || value <= 0.0 || value >= 99.0)
        return std::numeric_limits<float>::quiet_NaN();
    return static_cast<float>(value);
}
bool coordinate(const char *s, const char *hemisphere, bool latitude, double &value) {
    double raw;
    if (!number(s, raw) || raw < 0 || std::strlen(hemisphere) != 1) return false;
    const double degrees = std::floor(raw / 100);
    const double minutes = raw - degrees * 100;
    value = degrees + minutes / 60;
    if (minutes >= 60 || value > (latitude ? 90 : 180)) return false;
    if (*hemisphere == (latitude ? 'S' : 'W')) value = -value;
    else if (*hemisphere != (latitude ? 'N' : 'E')) return false;
    return true;
}
}

void NmeaGps::consume(const std::uint8_t *data, std::size_t length, std::uint32_t now_ms) {
    for (std::size_t i = 0; i < length; ++i) {
        const char c = static_cast<char>(data[i]);
        if (c == '$') { collecting_ = true; length_ = 0; }
        else if (!collecting_) continue;
        else if (c == '\r' || c == '\n') {
            sentence_[length_] = '\0';
            parse(now_ms);
            collecting_ = false;
        } else if (c < 32 || c > 126 || length_ + 1 >= sizeof(sentence_)) {
            collecting_ = false;
        } else sentence_[length_++] = c;
    }
}

void NmeaGps::parse(std::uint32_t now_ms) {
    char *star = std::strchr(sentence_, '*');
    if (!star || std::strlen(star) != 3 || hex(star[1]) < 0 || hex(star[2]) < 0) return;
    unsigned checksum = 0;
    for (char *p = sentence_; p != star; ++p) checksum ^= static_cast<unsigned char>(*p);
    if (checksum != static_cast<unsigned>(hex(star[1]) * 16 + hex(star[2]))) return;
    *star = '\0';
    char *fields[20]{};
    std::size_t count = 1;
    fields[0] = sentence_;
    for (char *p = sentence_; *p; ++p) {
        if (*p == ',') {
            if (count == 20) return;
            *p = '\0'; fields[count++] = p + 1;
        }
    }
    if (std::strlen(fields[0]) == 5 && std::strcmp(fields[0] + 2, "GSA") == 0) {
        parse_gsa(fields, count, now_ms);
        return;
    }
    if (count < 11 || std::strlen(fields[0]) != 5 ||
        std::strcmp(fields[0] + 2, "GGA") != 0) return;
    ++gga_sentence_count_;
    // A checksum-valid GGA replaces the prior fix, including no-fix reports;
    // its quality and satellite count are retained even without a position.
    fix_ = {};
    fix_.gga = true;
    fix_.received_ms = now_ms;
    fix_.sequence = gga_sentence_count_;
    if (std::strlen(fields[6]) == 1 && fields[6][0] >= '0' && fields[6][0] <= '9')
        fix_.fix_quality = static_cast<std::uint8_t>(fields[6][0] - '0');
    double satellites;
    if (number(fields[7], satellites) && satellites >= 0 && satellites <= 255 &&
        satellites == std::floor(satellites))
        fix_.satellites = static_cast<std::uint8_t>(satellites);
    fix_.hdop = dop(fields[8]);
    if (fix_.fix_quality >= 1 && fix_.fix_quality <= 8) attach_dop(fix_, now_ms);
    if (fix_.fix_quality < 1 || fix_.fix_quality > 8 ||
        std::strcmp(fields[10], "M") != 0) return;
    Fix next = fix_;
    double altitude;
    if (!coordinate(fields[2], fields[3], true, next.latitude_degrees) ||
        !coordinate(fields[4], fields[5], false, next.longitude_degrees) ||
        !number(fields[9], altitude)) return;
    next.altitude_meters = static_cast<float>(altitude);
    if (!std::isfinite(next.altitude_meters)) return;
    next.valid = true;
    fix_ = next;
}

// $xxGSA,mode,fix type,12 x SV,PDOP,HDOP,VDOP[,system ID]
void NmeaGps::parse_gsa(char *const *fields, const std::size_t count, const std::uint32_t now_ms)
{
    if (count < 18) return;
    ++gsa_sentence_count_;
    // Fix type 1 = no fix: ignored, not cleared. A multi-GNSS receiver sends
    // one GSA per constellation, and a system with no satellites in use
    // reports "no fix" (DOPs 0.0) while the combined solution is fine; the
    // timeout drops old values instead.
    if (std::strcmp(fields[2], "2") != 0 && std::strcmp(fields[2], "3") != 0) return;
    const float pdop = dop(fields[15]);
    const float hdop = dop(fields[16]);
    const float vdop = dop(fields[17]);
    if (!std::isfinite(hdop) && !std::isfinite(vdop)) return;
    gsa_pdop_ = pdop;
    gsa_hdop_ = hdop;
    gsa_vdop_ = vdop;
    gsa_ms_ = now_ms;
    gsa_seen_ = true;
    if (fix_.fix_quality >= 1 && fix_.fix_quality <= 8) attach_dop(fix_, now_ms);
}

void NmeaGps::attach_dop(Fix &fix, const std::uint32_t now_ms) const
{
    const bool fresh = gsa_seen_ && now_ms - gsa_ms_ < kDopTimeoutMs;
    fix.pdop = fresh ? gsa_pdop_ : std::numeric_limits<float>::quiet_NaN();
    fix.vdop = fresh ? gsa_vdop_ : std::numeric_limits<float>::quiet_NaN();
    // GGA HDOP is per fix; fall back to the GSA value if GGA left it empty.
    if (!std::isfinite(fix.hdop) && fresh) fix.hdop = gsa_hdop_;
}
