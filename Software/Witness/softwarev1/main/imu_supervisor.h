#pragma once

// IMU liveness decisions for the sensor task (host-testable: no ESP-IDF).
//
// The LSM6DSV320X is read through its FIFO at ~960 Hz; a healthy pass (2 ms)
// returns about two samples. A chip that browned out, reset or lost its
// configuration answers every SPI read with success but delivers nothing (or
// 0x00 / 0xFF). This class turns per-pass observations into a verdict; the
// sensor task acts on it (mark not ready, re-initialise, retry).
//
// Thresholds (all deliberately 25x or more above normal operation):
//   no_data_ms          50   no FIFO sample for 50 ms (about 48 samples missing;
//                            a normal pass delivers one every ~1 ms)
//   constant_samples    100  100 consecutive samples whose nine raw channels
//                            are all 0x0000 or all 0xFFFF (a live part always
//                            shows gravity on low-g and noise on the gyro)
//   error_passes        25   25 consecutive failed SPI reads (50 ms)
//   who_am_i_period_ms  1000 cheap register check once a second; a bad value
//                            must repeat on an immediate second read
//   retry_period_ms     500  between re-initialisation attempts

#include <cstddef>
#include <cstdint>

namespace imu_supervisor {

struct Config {
    std::uint32_t no_data_ms = 50;
    std::uint32_t constant_samples = 100;
    std::uint32_t error_passes = 25;
    std::uint32_t who_am_i_period_ms = 1000;
    std::uint32_t retry_period_ms = 500;
};

enum class Fault : std::uint8_t { None, NoData, ConstantData, ReadErrors, WhoAmI };

inline const char *fault_name(const Fault fault)
{
    switch (fault) {
    case Fault::None: return "none";
    case Fault::NoData: return "no FIFO data";
    case Fault::ConstantData: return "constant 0x00/0xFF data";
    case Fault::ReadErrors: return "SPI read errors";
    case Fault::WhoAmI: return "WHO_AM_I mismatch";
    }
    return "?";
}

class Monitor {
public:
    explicit Monitor(const Config &config = Config{}) : config_(config) {}

    // Call after a successful initialisation (or when the data path restarts).
    void arm(const std::uint32_t now_ms)
    {
        last_data_ms_ = now_ms;
        last_who_am_i_ms_ = now_ms;
        constant_run_ = 0;
        error_run_ = 0;
    }

    // One fetch pass. samples = FIFO slots delivered; constant_tail = number of
    // trailing samples in this batch that were all-0x0000 / all-0xFFFF;
    // varying = true when the batch holds any sample that was neither.
    Fault on_pass(const std::uint32_t now_ms, const bool read_ok, const std::size_t samples,
                  const std::size_t constant_tail, const bool varying)
    {
        if (!read_ok) {
            if (++error_run_ >= config_.error_passes) return Fault::ReadErrors;
        } else {
            error_run_ = 0;
        }
        if (samples > 0) {
            last_data_ms_ = now_ms;
            if (varying) {
                constant_run_ = static_cast<std::uint32_t>(constant_tail);
            } else {
                constant_run_ += static_cast<std::uint32_t>(samples);
            }
            if (constant_run_ >= config_.constant_samples) return Fault::ConstantData;
        } else if (read_ok && now_ms - last_data_ms_ >= config_.no_data_ms) {
            return Fault::NoData;
        }
        return Fault::None;
    }

    bool who_am_i_due(const std::uint32_t now_ms)
    {
        if (now_ms - last_who_am_i_ms_ < config_.who_am_i_period_ms) return false;
        last_who_am_i_ms_ = now_ms;
        return true;
    }

    // Retry schedule while the IMU is down. The first attempt is immediate.
    void down(const std::uint32_t now_ms) { next_retry_ms_ = now_ms; }
    bool retry_due(const std::uint32_t now_ms)
    {
        if (static_cast<std::int32_t>(now_ms - next_retry_ms_) < 0) return false;
        next_retry_ms_ = now_ms + config_.retry_period_ms;
        return true;
    }

private:
    Config config_;
    std::uint32_t last_data_ms_ = 0;
    std::uint32_t last_who_am_i_ms_ = 0;
    std::uint32_t next_retry_ms_ = 0;
    std::uint32_t constant_run_ = 0;
    std::uint32_t error_run_ = 0;
};

}  // namespace imu_supervisor
