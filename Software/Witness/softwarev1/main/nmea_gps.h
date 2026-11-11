#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

// Streaming NMEA GGA (+ GSA dilution of precision) decoder. Independent of UART and RTOS for host testing.
class NmeaGps final {
public:
    static constexpr std::uint32_t kFixTimeoutMs = 3000;
    // A GSA (PDOP/HDOP/VDOP) older than this is not attached to a fix. The
    // PX1120S sends GSA every 5 s (its interval is in seconds), one per
    // constellation; this spans two of those periods.
    static constexpr std::uint32_t kDopTimeoutMs = 12000;
    // Fields from the most recent checksum-valid GGA. Satellites and quality
    // are kept for every GGA, including no-fix reports; position fields are
    // zero unless valid (quality 1..8 and parseable coordinates/altitude).
    struct Fix {
        double latitude_degrees = 0;
        double longitude_degrees = 0;
        float altitude_meters = 0; // GGA orthometric altitude, mean sea level.
        std::uint32_t received_ms = 0; // Time of that GGA (gga == true).
        // Increments with every GGA, so a consumer can use each fix once.
        std::uint32_t sequence = 0;
        std::uint8_t satellites = 0;   // GGA field 7, satellites used.
        std::uint8_t fix_quality = 0;  // GGA field 6; 0 = no fix.
        // Dilution of precision; NaN when unknown. HDOP is GGA field 8 (every
        // fix); PDOP and VDOP come from the latest fresh GSA.
        float hdop = std::numeric_limits<float>::quiet_NaN();
        float vdop = std::numeric_limits<float>::quiet_NaN();
        float pdop = std::numeric_limits<float>::quiet_NaN();
        bool gga = false;              // A GGA has been received.
        bool valid = false;            // Position fields hold a fix.
    };
    void consume(const std::uint8_t *data, std::size_t length, std::uint32_t now_ms);
    Fix latest_fix() const { return fix_; }
    // Includes no-fix GGA: a navigation fix is not required to prove the link.
    std::uint32_t gga_sentence_count() const { return gga_sentence_count_; }
    std::uint32_t gsa_sentence_count() const { return gsa_sentence_count_; }
    static bool fresh(const Fix &fix, std::uint32_t now_ms) {
        return fix.valid && gga_fresh(fix, now_ms);
    }
    // The GGA (satellites/quality) is recent, whether or not it had a fix.
    static bool gga_fresh(const Fix &fix, std::uint32_t now_ms) {
        return fix.gga && now_ms - fix.received_ms < kFixTimeoutMs;
    }
private:
    void parse(std::uint32_t now_ms);
    void parse_gsa(char *const *fields, std::size_t count, std::uint32_t now_ms);
    void attach_dop(Fix &fix, std::uint32_t now_ms) const;
    char sentence_[160]{};
    std::size_t length_ = 0;
    bool collecting_ = false;
    Fix fix_{};
    std::uint32_t gga_sentence_count_ = 0;
    std::uint32_t gsa_sentence_count_ = 0;
    // Latest GSA values; a multi-constellation receiver sends one GSA per
    // system with the same combined DOPs, so the last one is kept.
    float gsa_pdop_ = std::numeric_limits<float>::quiet_NaN();
    float gsa_hdop_ = std::numeric_limits<float>::quiet_NaN();
    float gsa_vdop_ = std::numeric_limits<float>::quiet_NaN();
    std::uint32_t gsa_ms_ = 0;
    bool gsa_seen_ = false;
};
