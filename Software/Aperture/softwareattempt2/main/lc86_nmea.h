#pragma once

#include <cstddef>
#include <cstdint>

// Streaming NMEA decoder for the Aperture LC86 GPS. Pure C++ (no ESP-IDF) so it
// can be host-tested. Based on the Witness NmeaGps parser, extended with RMC
// date/time, GGA HDOP, a retained last-valid fix, and PAIR/PMTK
// acknowledgements for receiver configuration.
//
// Every sentence must carry a valid "*hh" checksum; any two-letter talker
// ($GP, $GN, $GL, $GA, $GB, ...) is accepted for GGA and RMC.
class Lc86Nmea final {
public:
    // GGA fix quality / satellites / HDOP are reported only this long.
    static constexpr std::uint32_t kGgaTimeoutMs = 3000;
    static constexpr std::uint16_t kHdopUnknown = 0xFFFF;

    struct State {
        // Most recent checksum-valid GGA, with or without a fix.
        bool gga = false;
        std::uint32_t gga_ms = 0;          // Local time it was received.
        std::uint8_t fix_quality = 0;      // GGA field 6; 0 = no fix.
        std::uint8_t satellites = 0;       // GGA field 7.
        std::uint16_t hdop_centi = kHdopUnknown; // GGA field 8 x 100.

        // Last GGA with quality 1..8 and parseable position. Retained across
        // later no-fix GGAs so the pad position survives a dropout.
        bool fix = false;
        std::uint32_t fix_ms = 0;
        double latitude_degrees = 0;
        double longitude_degrees = 0;
        float altitude_meters = 0;         // MSL (GGA field 9).

        // Last RMC with status 'A' and a parseable date and time.
        bool time = false;
        std::uint32_t time_ms = 0;         // Local time it was received.
        std::int64_t unix_ms = 0;          // UTC of that RMC.
    };

    // Last proprietary acknowledgement: $PAIR001,<cmd>,<result> or
    // $PMTK001,<cmd>,<flag>.
    struct Ack {
        std::uint32_t count = 0;   // Increments on every ack received.
        bool pair = false;         // true: PAIR001, false: PMTK001.
        std::uint16_t command = 0;
        std::uint8_t result = 0;
    };

    void consume(const std::uint8_t *data, std::size_t length, std::uint32_t now_ms);
    const State &state() const { return state_; }
    const Ack &last_ack() const { return ack_; }
    std::uint32_t sentence_count() const { return sentence_count_; }
    std::uint32_t checksum_error_count() const { return checksum_errors_; }

    static bool gga_fresh(const State &state, std::uint32_t now_ms) {
        return state.gga && now_ms - state.gga_ms < kGgaTimeoutMs;
    }

    // Writes "$<body>*hh\r\n" into out. Returns the length written (excluding
    // the terminating NUL) or 0 when out is too small.
    static std::size_t build_sentence(const char *body, char *out, std::size_t out_size);

    // Days-from-civil conversion; exposed for tests.
    static std::int64_t unix_seconds(int year, int month, int day,
                                     int hour, int minute, int second);

private:
    void parse(std::uint32_t now_ms);
    void parse_gga(char **fields, std::size_t count, std::uint32_t now_ms);
    void parse_rmc(char **fields, std::size_t count, std::uint32_t now_ms);
    void parse_ack(char **fields, std::size_t count, bool pair);

    char sentence_[160]{};
    std::size_t length_ = 0;
    bool collecting_ = false;
    State state_{};
    Ack ack_{};
    std::uint32_t sentence_count_ = 0;
    std::uint32_t checksum_errors_ = 0;
};
