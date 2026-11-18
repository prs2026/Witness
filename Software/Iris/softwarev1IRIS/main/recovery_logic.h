#pragma once

// Pure (host-testable) policies for the Iris recovery paths. No ESP-IDF.

#include <cstdint>

namespace iris_recovery {

// I2C bus recovery trigger: after kFailThreshold consecutive failed
// transactions ask for a bus recovery, at most once per kMinSpacingUs.
class I2cFailureTracker {
public:
    static constexpr int kFailThreshold = 2;
    static constexpr std::int64_t kMinSpacingUs = 500000;

    // Returns true when the caller should run a bus recovery now.
    bool note(bool ok, std::int64_t now_us)
    {
        if (ok) {
            failures_ = 0;
            return false;
        }
        if (failures_ < 1000) ++failures_;
        if (failures_ < kFailThreshold) return false;
        if (have_recovered_ && now_us - last_recovery_us_ < kMinSpacingUs) return false;
        have_recovered_ = true;
        last_recovery_us_ = now_us;
        return true;
    }
    int failures() const { return failures_; }

private:
    int failures_ = 0;
    bool have_recovered_ = false;
    std::int64_t last_recovery_us_ = 0;
};

// TWAI transmit slots are stuck when every slot is in use and no transmit
// completion has been seen for timeout_us (e.g. frames lost across bus-off).
inline bool tx_slots_stuck(int in_use, int total, std::int64_t now_us,
                           std::int64_t last_progress_us, std::int64_t timeout_us)
{
    return in_use >= total && now_us - last_progress_us >= timeout_us;
}

}  // namespace iris_recovery
