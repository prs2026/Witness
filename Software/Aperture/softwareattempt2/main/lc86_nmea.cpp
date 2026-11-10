#include "lc86_nmea.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

int hex(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// NMEA decimal fields contain no whitespace, exponents or NaNs.
bool number(const char *s, double &value) {
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

bool unsigned_integer(const char *s, unsigned long max, unsigned long &value) {
    if (!*s) return false;
    unsigned long result = 0;
    for (const char *p = s; *p; ++p) {
        if (*p < '0' || *p > '9') return false;
        result = result * 10 + static_cast<unsigned long>(*p - '0');
        if (result > max) return false;
    }
    value = result;
    return true;
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

int two_digits(const char *s) {
    if (s[0] < '0' || s[0] > '9' || s[1] < '0' || s[1] > '9') return -1;
    return (s[0] - '0') * 10 + (s[1] - '0');
}

// Talker-independent sentence match: "GPGGA", "GNRMC", ...
bool is_type(const char *address, const char *type) {
    return std::strlen(address) == 5 && std::strcmp(address + 2, type) == 0;
}

}  // namespace

std::int64_t Lc86Nmea::unix_seconds(int year, int month, int day,
                                    int hour, int minute, int second) {
    // Howard Hinnant's days_from_civil.
    year -= month <= 2;
    const std::int64_t era = (year >= 0 ? year : year - 399) / 400;
    const std::int64_t yoe = year - era * 400;
    const std::int64_t doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
    const std::int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    const std::int64_t days = era * 146097 + doe - 719468;
    return days * 86400 + hour * 3600 + minute * 60 + second;
}

std::size_t Lc86Nmea::build_sentence(const char *body, char *out, std::size_t out_size) {
    unsigned checksum = 0;
    for (const char *p = body; *p; ++p) checksum ^= static_cast<unsigned char>(*p);
    const int written = std::snprintf(out, out_size, "$%s*%02X\r\n", body, checksum);
    if (written < 0 || static_cast<std::size_t>(written) >= out_size) return 0;
    return static_cast<std::size_t>(written);
}

void Lc86Nmea::consume(const std::uint8_t *data, std::size_t length, std::uint32_t now_ms) {
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

void Lc86Nmea::parse(std::uint32_t now_ms) {
    char *star = std::strchr(sentence_, '*');
    if (!star || std::strlen(star) != 3 || hex(star[1]) < 0 || hex(star[2]) < 0) {
        ++checksum_errors_;
        return;
    }
    unsigned checksum = 0;
    for (char *p = sentence_; p != star; ++p) checksum ^= static_cast<unsigned char>(*p);
    if (checksum != static_cast<unsigned>(hex(star[1]) * 16 + hex(star[2]))) {
        ++checksum_errors_;
        return;
    }
    ++sentence_count_;
    *star = '\0';
    char *fields[24]{};
    std::size_t count = 1;
    fields[0] = sentence_;
    for (char *p = sentence_; *p; ++p) {
        if (*p == ',') {
            if (count == sizeof(fields) / sizeof(fields[0])) return;
            *p = '\0';
            fields[count++] = p + 1;
        }
    }
    if (is_type(fields[0], "GGA")) parse_gga(fields, count, now_ms);
    else if (is_type(fields[0], "RMC")) parse_rmc(fields, count, now_ms);
    else if (std::strcmp(fields[0], "PAIR001") == 0) parse_ack(fields, count, true);
    else if (std::strcmp(fields[0], "PMTK001") == 0) parse_ack(fields, count, false);
}

void Lc86Nmea::parse_gga(char **fields, std::size_t count, std::uint32_t now_ms) {
    // GGA: time, lat, N/S, lon, E/W, quality, sats, HDOP, alt, M, ...
    if (count < 11) return;
    state_.gga = true;
    state_.gga_ms = now_ms;
    state_.fix_quality = 0;
    state_.satellites = 0;
    state_.hdop_centi = kHdopUnknown;
    if (std::strlen(fields[6]) == 1 && fields[6][0] >= '0' && fields[6][0] <= '9')
        state_.fix_quality = static_cast<std::uint8_t>(fields[6][0] - '0');
    unsigned long satellites;
    if (unsigned_integer(fields[7], 255, satellites))
        state_.satellites = static_cast<std::uint8_t>(satellites);
    double hdop;
    if (number(fields[8], hdop) && hdop >= 0) {
        const double centi = std::round(hdop * 100);
        state_.hdop_centi = centi >= kHdopUnknown
            ? static_cast<std::uint16_t>(kHdopUnknown - 1)
            : static_cast<std::uint16_t>(centi);
    }
    if (state_.fix_quality < 1 || state_.fix_quality > 8 ||
        std::strcmp(fields[10], "M") != 0) return;
    double latitude, longitude, altitude;
    if (!coordinate(fields[2], fields[3], true, latitude) ||
        !coordinate(fields[4], fields[5], false, longitude) ||
        !number(fields[9], altitude)) return;
    const float altitude_f = static_cast<float>(altitude);
    if (!std::isfinite(altitude_f)) return;
    state_.fix = true;
    state_.fix_ms = now_ms;
    state_.latitude_degrees = latitude;
    state_.longitude_degrees = longitude;
    state_.altitude_meters = altitude_f;
}

void Lc86Nmea::parse_rmc(char **fields, std::size_t count, std::uint32_t now_ms) {
    // RMC: time, status, lat, N/S, lon, E/W, speed, course, date, ...
    if (count < 10 || std::strcmp(fields[2], "A") != 0) return;
    const char *t = fields[1];
    const char *d = fields[9];
    if (std::strlen(t) < 6 || std::strlen(d) != 6) return;
    const int hour = two_digits(t), minute = two_digits(t + 2), second = two_digits(t + 4);
    const int day = two_digits(d), month = two_digits(d + 2), yy = two_digits(d + 4);
    if (hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 60 ||
        day < 1 || day > 31 || month < 1 || month > 12 || yy < 0) return;
    int millis = 0;
    if (t[6] != '\0') {
        if (t[6] != '.') return;
        int scale = 100;
        for (const char *p = t + 7; *p; ++p) {
            if (*p < '0' || *p > '9') return;
            millis += (*p - '0') * scale;
            scale /= 10;
        }
    }
    state_.time = true;
    state_.time_ms = now_ms;
    state_.unix_ms = unix_seconds(2000 + yy, month, day, hour, minute, second) * 1000 + millis;
}

void Lc86Nmea::parse_ack(char **fields, std::size_t count, bool pair) {
    unsigned long command, result;
    if (count < 3 || !unsigned_integer(fields[1], 9999, command) ||
        !unsigned_integer(fields[2], 255, result)) return;
    ++ack_.count;
    ack_.pair = pair;
    ack_.command = static_cast<std::uint16_t>(command);
    ack_.result = static_cast<std::uint8_t>(result);
}
