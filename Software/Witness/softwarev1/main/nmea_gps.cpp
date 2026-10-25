#include "nmea_gps.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

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
    if (count < 11 || std::strlen(fields[0]) != 5 ||
        std::strcmp(fields[0] + 2, "GGA") != 0) return;
    ++gga_sentence_count_;
    // A checksum-valid GGA replaces the prior fix, including no-fix reports.
    fix_ = {};
    if (std::strlen(fields[6]) != 1 || fields[6][0] < '1' || fields[6][0] > '8' ||
        std::strcmp(fields[10], "M") != 0) return;
    Fix next{};
    double altitude;
    if (!coordinate(fields[2], fields[3], true, next.latitude_degrees) ||
        !coordinate(fields[4], fields[5], false, next.longitude_degrees) ||
        !number(fields[9], altitude)) return;
    next.altitude_meters = static_cast<float>(altitude);
    if (!std::isfinite(next.altitude_meters)) return;
    next.received_ms = now_ms;
    next.valid = true;
    fix_ = next;
}
