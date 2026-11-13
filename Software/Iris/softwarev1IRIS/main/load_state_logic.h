#pragma once

// Pure (host-testable) logic for remembering the commanded load mask across
// Iris resets. No ESP-IDF dependencies.
//
// Mask bits are MCP23008 pin numbers: bit0 OUT1_EN, bit1 OUT2_EN,
// bit2 OUT3_EN, bit3 5V_EN. The blue LED (pin 7) is never persisted.

#include <cstddef>
#include <cstdint>

namespace load_state {

constexpr std::uint32_t kMagic = 0x4C443A53U;  // "LD:S"
constexpr std::uint8_t kMaskBits = 0x0FU;

struct Record {
    std::uint32_t magic;
    std::uint32_t mask;
    std::uint32_t crc;
};

inline std::uint32_t crc32(const std::uint8_t *data, std::size_t length)
{
    std::uint32_t crc = 0xFFFFFFFFU;
    for (std::size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
        }
    }
    return ~crc;
}

inline std::uint32_t record_crc(std::uint32_t magic, std::uint32_t mask)
{
    const std::uint8_t bytes[8] = {
        static_cast<std::uint8_t>(magic), static_cast<std::uint8_t>(magic >> 8U),
        static_cast<std::uint8_t>(magic >> 16U), static_cast<std::uint8_t>(magic >> 24U),
        static_cast<std::uint8_t>(mask), static_cast<std::uint8_t>(mask >> 8U),
        static_cast<std::uint8_t>(mask >> 16U), static_cast<std::uint8_t>(mask >> 24U)};
    return crc32(bytes, sizeof(bytes));
}

inline Record make_record(std::uint8_t mask)
{
    const std::uint32_t m = mask & kMaskBits;
    return Record{kMagic, m, record_crc(kMagic, m)};
}

inline bool record_valid(const Record &record)
{
    return record.magic == kMagic && (record.mask & ~static_cast<std::uint32_t>(kMaskBits)) == 0U &&
           record.crc == record_crc(record.magic, record.mask);
}

enum class ResetClass : std::uint8_t {
    cold,  // power-on, external pin, USB/JTAG reset, deep sleep, unknown
    warm,  // panic, watchdogs, software, brownout, CPU lockup
};

struct BootDecision {
    bool restore;       // re-apply `mask` without pulsing the expander reset
    std::uint8_t mask;  // mask to apply (0 on cold boot)
};

// Cold boot always yields the safe default (everything off). A warm reset with
// a valid record restores the last commanded mask.
inline BootDecision decide_boot(ResetClass reset, const Record &record)
{
    if (reset == ResetClass::warm && record_valid(record)) {
        return BootDecision{true, static_cast<std::uint8_t>(record.mask & kMaskBits)};
    }
    return BootDecision{false, 0};
}

}  // namespace load_state
