#pragma once

#include <cstddef>
#include <cstdint>

// Streaming NMEA GGA decoder. Independent of UART and RTOS for host testing.
class NmeaGps final {
public:
    static constexpr std::uint32_t kFixTimeoutMs = 3000;
    // Fields from the most recent checksum-valid GGA. Satellites and quality
    // are kept for every GGA, including no-fix reports; position fields are
    // zero unless valid (quality 1..8 and parseable coordinates/altitude).
    struct Fix {
        double latitude_degrees = 0;
        double longitude_degrees = 0;
        float altitude_meters = 0; // GGA orthometric altitude, mean sea level.
        std::uint32_t received_ms = 0; // Time of that GGA (gga == true).
        std::uint8_t satellites = 0;   // GGA field 7, satellites used.
        std::uint8_t fix_quality = 0;  // GGA field 6; 0 = no fix.
        bool gga = false;              // A GGA has been received.
        bool valid = false;            // Position fields hold a fix.
    };
    void consume(const std::uint8_t *data, std::size_t length, std::uint32_t now_ms);
    Fix latest_fix() const { return fix_; }
    // Includes no-fix GGA: a navigation fix is not required to prove the link.
    std::uint32_t gga_sentence_count() const { return gga_sentence_count_; }
    static bool fresh(const Fix &fix, std::uint32_t now_ms) {
        return fix.valid && gga_fresh(fix, now_ms);
    }
    // The GGA (satellites/quality) is recent, whether or not it had a fix.
    static bool gga_fresh(const Fix &fix, std::uint32_t now_ms) {
        return fix.gga && now_ms - fix.received_ms < kFixTimeoutMs;
    }
private:
    void parse(std::uint32_t now_ms);
    char sentence_[160]{};
    std::size_t length_ = 0;
    bool collecting_ = false;
    Fix fix_{};
    std::uint32_t gga_sentence_count_ = 0;
};
