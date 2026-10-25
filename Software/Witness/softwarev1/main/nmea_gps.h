#pragma once

#include <cstddef>
#include <cstdint>

// Streaming NMEA GGA decoder. Independent of UART and RTOS for host testing.
class NmeaGps final {
public:
    static constexpr std::uint32_t kFixTimeoutMs = 3000;
    struct Fix {
        double latitude_degrees = 0;
        double longitude_degrees = 0;
        float altitude_meters = 0; // GGA orthometric altitude, mean sea level.
        std::uint32_t received_ms = 0;
        bool valid = false;
    };
    void consume(const std::uint8_t *data, std::size_t length, std::uint32_t now_ms);
    Fix latest_fix() const { return fix_; }
    // Includes no-fix GGA: a navigation fix is not required to prove the link.
    std::uint32_t gga_sentence_count() const { return gga_sentence_count_; }
    static bool fresh(const Fix &fix, std::uint32_t now_ms) {
        return fix.valid && now_ms - fix.received_ms < kFixTimeoutMs;
    }
private:
    void parse(std::uint32_t now_ms);
    char sentence_[160]{};
    std::size_t length_ = 0;
    bool collecting_ = false;
    Fix fix_{};
    std::uint32_t gga_sentence_count_ = 0;
};
