#pragma once

// Pure (host-testable) backup for Iris load state. Witness remembers the last
// load/channel commands it forwarded to Iris and decides when to re-send them:
//  - Iris uptime went backwards (Iris rebooted), or
//  - Iris-reported OUT1..3/5V flags disagree with the commanded state for
//    more than kMismatchMs (and no command was sent in the last kGraceMs).
// Re-sends are rate limited.

#include <cstdint>

namespace iris_load_tracker {

constexpr std::uint16_t kCh1Off = 0x30, kCh1On = 0x31, kCh2Off = 0x32, kCh2On = 0x33;
constexpr std::uint16_t kCh3Off = 0x34, kCh3On = 0x35, k5vOff = 0x50, k5vOn = 0x51;
constexpr std::uint8_t kStatusOutShift = 3;  // status bits 3..6 = OUT1,OUT2,OUT3,5V
constexpr std::int64_t kMismatchMs = 900;
constexpr std::int64_t kGraceMs = 1500;
constexpr std::int64_t kMismatchResendIntervalMs = 2000;
constexpr std::int64_t kRebootResendIntervalMs = 500;

struct Action {
    std::uint16_t opcodes[4];
    std::uint8_t count;
};

class Tracker {
public:
    // Returns false for opcodes that are not load/5V commands.
    static bool decode(std::uint16_t opcode, std::uint8_t &bit, bool &on)
    {
        switch (opcode) {
            case kCh1Off: bit = 0; on = false; return true;
            case kCh1On: bit = 0; on = true; return true;
            case kCh2Off: bit = 1; on = false; return true;
            case kCh2On: bit = 1; on = true; return true;
            case kCh3Off: bit = 2; on = false; return true;
            case kCh3On: bit = 2; on = true; return true;
            case k5vOff: bit = 3; on = false; return true;
            case k5vOn: bit = 3; on = true; return true;
            default: return false;
        }
    }

    void note_command(std::uint16_t opcode, std::int64_t now_ms)
    {
        std::uint8_t bit = 0;
        bool on = false;
        if (!decode(opcode, bit, on)) return;
        const std::uint8_t m = static_cast<std::uint8_t>(1U << bit);
        known_ |= m;
        commanded_ = on ? (commanded_ | m) : (commanded_ & ~m);
        last_command_ms_ = now_ms;
        have_command_ = true;
        mismatch_since_ms_ = -1;
    }

    // Call for every Iris 0x03 camera packet.
    Action on_status(std::int64_t now_ms, std::uint8_t status_flags, std::uint32_t iris_uptime_ms)
    {
        Action action{};
        const std::uint8_t reported = (status_flags >> kStatusOutShift) & 0x0F;
        const bool rebooted = have_uptime_ && iris_uptime_ms < last_uptime_ms_;
        have_uptime_ = true;
        last_uptime_ms_ = iris_uptime_ms;
        if (known_ == 0) return action;

        const bool mismatch = ((reported ^ commanded_) & known_) != 0;
        bool resend = false;
        if (rebooted) {
            mismatch_since_ms_ = -1;
            resend = !have_resend_ || now_ms - last_resend_ms_ >= kRebootResendIntervalMs;
        } else if (!mismatch) {
            mismatch_since_ms_ = -1;
        } else if (have_command_ && now_ms - last_command_ms_ < kGraceMs) {
            mismatch_since_ms_ = -1;
        } else {
            if (mismatch_since_ms_ < 0) mismatch_since_ms_ = now_ms;
            if (now_ms - mismatch_since_ms_ >= kMismatchMs)
                resend = !have_resend_ || now_ms - last_resend_ms_ >= kMismatchResendIntervalMs;
        }
        if (!resend) return action;

        last_resend_ms_ = now_ms;
        have_resend_ = true;
        static constexpr std::uint16_t off[4] = {kCh1Off, kCh2Off, kCh3Off, k5vOff};
        for (std::uint8_t b = 0; b < 4; ++b) {
            const std::uint8_t m = static_cast<std::uint8_t>(1U << b);
            if (known_ & m) action.opcodes[action.count++] = (commanded_ & m) ? off[b] + 1 : off[b];
        }
        return action;
    }

    std::uint8_t commanded() const { return commanded_; }
    std::uint8_t known() const { return known_; }

private:
    std::uint8_t commanded_ = 0;
    std::uint8_t known_ = 0;
    bool have_uptime_ = false;
    bool have_command_ = false;
    bool have_resend_ = false;
    std::uint32_t last_uptime_ms_ = 0;
    std::int64_t last_command_ms_ = 0;
    std::int64_t last_resend_ms_ = 0;
    std::int64_t mismatch_since_ms_ = -1;
};

}  // namespace iris_load_tracker
