#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "bit_limits.h"
#include "bit_logic.h"

// Small pure helpers the BIT collector uses to turn the sample stream and the
// owners' counters into the Inputs of bit_logic.h. Host test
// tests/bit_stats_test.cpp.
namespace bit {

// Rate of a free-running counter over windows of at least `window_ms`.
class RateMeter {
public:
    explicit RateMeter(const std::uint32_t window_ms = 2000) : window_ms_(window_ms) {}
    // Returns true when a rate is available (it keeps the last one between windows).
    bool update(const std::uint32_t counter, const std::uint32_t now_ms)
    {
        if (!have_base_) {
            base_counter_ = counter; base_ms_ = now_ms; have_base_ = true;
            return valid_;
        }
        if (counter < base_counter_) {
            // The owner reset its counter (the sensor task clears them with its 10 s report): start a new window.
            base_counter_ = counter; base_ms_ = now_ms;
            return valid_;
        }
        const std::uint32_t dt = now_ms - base_ms_;
        if (dt >= window_ms_) {
            rate_hz_ = static_cast<float>(static_cast<std::uint32_t>(counter - base_counter_)) * 1000.0F /
                       static_cast<float>(dt);
            valid_ = true;
            base_counter_ = counter; base_ms_ = now_ms;
        }
        return valid_;
    }
    bool valid() const { return valid_; }
    float rate_hz() const { return rate_hz_; }
    void reset() { have_base_ = false; valid_ = false; rate_hz_ = 0.0F; }

private:
    std::uint32_t window_ms_;
    bool have_base_ = false, valid_ = false;
    std::uint32_t base_counter_ = 0, base_ms_ = 0;
    float rate_hz_ = 0.0F;
};

// "Did this counter grow in the last hold_ms" (loss counters: overruns, drops).
class GrowthFlag {
public:
    explicit GrowthFlag(const std::uint32_t hold_ms = 10'000) : hold_ms_(hold_ms) {}
    bool update(const std::uint32_t counter, const std::uint32_t now_ms)
    {
        if (!have_) { last_ = counter; have_ = true; }
        if (counter > last_) { grew_ms_ = now_ms; grew_ = true; }
        last_ = counter;   // a smaller value means the owner reset the counter: not growth
        return grew_ && static_cast<std::uint32_t>(now_ms - grew_ms_) < hold_ms_;
    }
    void reset() { have_ = false; grew_ = false; }

private:
    std::uint32_t hold_ms_;
    bool have_ = false, grew_ = false;
    std::uint32_t last_ = 0, grew_ms_ = 0;
};

// One-second windows of low-g, gyro and high-g samples (the board is expected to
// be still; if it is not, the numbers say so and the checks fail, which is right).
class StillWindow {
public:
    static constexpr std::uint32_t kWindowMs = limits::kStillWindowMs;
    static constexpr std::uint32_t kMinSamples = 20;
    static constexpr std::uint32_t kStaleMs = 3 * kWindowMs;

    void add(const float low_g_mg[3], const float gyro_mdps[3], const float high_g_mg[3], const bool high_g_valid,
             const std::uint32_t now_ms)
    {
        if (n_ == 0U) start_ms_ = now_ms;
        for (int i = 0; i < 3; ++i) {
            acc_[i].add(static_cast<double>(low_g_mg[i]));
            gyr_[i].add(static_cast<double>(gyro_mdps[i]) / 1000.0);
            hg_[i].add(static_cast<double>(high_g_mg[i]));
        }
        high_valid_ = high_g_valid;
        ++n_;
        if (now_ms - start_ms_ >= kWindowMs) finish(now_ms);
    }

    // The latest complete window; invalid when none yet or when it is stale.
    StillStats latest(const std::uint32_t now_ms) const
    {
        StillStats s = last_;
        if (s.valid && now_ms - last_ms_ > kStaleMs) s.valid = false;
        return s;
    }
    void reset() { n_ = 0; last_ = StillStats{}; for (int i = 0; i < 3; ++i) { acc_[i] = {}; gyr_[i] = {}; hg_[i] = {}; } }

private:
    struct Acc {
        double sum = 0.0, sumsq = 0.0;
        void add(const double v) { sum += v; sumsq += v * v; }
    };
    static double mean(const Acc &a, const std::uint32_t n) { return a.sum / n; }
    static double sd(const Acc &a, const std::uint32_t n)
    {
        const double m = a.sum / n;
        const double v = a.sumsq / n - m * m;
        return v > 0.0 ? std::sqrt(v) : 0.0;
    }
    void finish(const std::uint32_t now_ms)
    {
        if (n_ >= kMinSamples) {
            StillStats s;
            s.valid = true;
            double ax = mean(acc_[0], n_), ay = mean(acc_[1], n_), az = mean(acc_[2], n_);
            s.accel_mag_g = static_cast<float>(std::sqrt(ax * ax + ay * ay + az * az) / 1000.0);
            s.axial_g = static_cast<float>(ax / 1000.0);
            double w = 0.0, d = 0.0;
            for (int i = 0; i < 3; ++i) {
                const double sda = sd(acc_[i], n_), sdg = sd(gyr_[i], n_);
                if (sda > d) d = sda;
                if (sdg > w) w = sdg;
            }
            s.accel_sd_mg = static_cast<float>(d);
            s.gyro_sd_dps = static_cast<float>(w);
            const double gx = mean(gyr_[0], n_), gy = mean(gyr_[1], n_), gz = mean(gyr_[2], n_);
            s.gyro_mean_dps = static_cast<float>(std::sqrt(gx * gx + gy * gy + gz * gz));
            s.high_g_valid = high_valid_;
            const double hx = mean(hg_[0], n_), hy = mean(hg_[1], n_), hz = mean(hg_[2], n_);
            s.high_g_mag_g = static_cast<float>(std::sqrt(hx * hx + hy * hy + hz * hz) / 1000.0);
            last_ = s;
            last_ms_ = now_ms;
        }
        n_ = 0;
        for (int i = 0; i < 3; ++i) { acc_[i] = {}; gyr_[i] = {}; hg_[i] = {}; }
    }

    Acc acc_[3], gyr_[3], hg_[3];
    std::uint32_t n_ = 0, start_ms_ = 0, last_ms_ = 0;
    bool high_valid_ = false;
    StillStats last_{};
};

// Two-second barometer windows from 100 Hz polls: noise, rate (least-squares slope
// of the altitude), the rate of distinct values, and a frozen-output run counter.
class BaroWindow {
public:
    static constexpr std::uint32_t kWindowMs = 2000;
    static constexpr std::uint32_t kMinSamples = 100;

    void add(const std::int32_t pressure_pa, const std::int32_t temp_centi_c, const float altitude_m,
             const std::uint32_t now_ms)
    {
        const bool same = have_prev_ && pressure_pa == prev_pressure_ && temp_centi_c == prev_temp_;
        if (same) { ++identical_run_; } else { identical_run_ = 0; ++changes_; }
        prev_pressure_ = pressure_pa; prev_temp_ = temp_centi_c; have_prev_ = true;
        if (n_ == 0U) start_ms_ = now_ms;
        const double t = static_cast<double>(now_ms - start_ms_) / 1000.0;
        const double p = static_cast<double>(pressure_pa);
        sum_p_ += p; sumsq_p_ += p * p;
        sum_t_ += t; sum_tt_ += t * t; sum_a_ += altitude_m; sum_ta_ += t * altitude_m;
        ++n_;
        if (now_ms - start_ms_ >= kWindowMs) finish(now_ms);
    }
    BaroStats latest(const std::uint32_t now_ms) const
    {
        BaroStats s = last_;
        if (s.valid && now_ms - last_ms_ > 3U * kWindowMs) s.valid = false;
        s.frozen = s.valid && identical_run_ >= limits::kBaroFrozenSamples;
        return s;
    }
    void reset() { n_ = 0; last_ = BaroStats{}; identical_run_ = 0; changes_ = 0; have_prev_ = false; clear(); }

private:
    void clear()
    {
        sum_p_ = sumsq_p_ = sum_t_ = sum_tt_ = sum_a_ = sum_ta_ = 0.0;
        n_ = 0; changes_ = 0;
    }
    void finish(const std::uint32_t now_ms)
    {
        if (n_ >= kMinSamples) {
            BaroStats s;
            s.valid = true;
            const double mp = sum_p_ / n_;
            const double v = sumsq_p_ / n_ - mp * mp;
            s.sd_pa = static_cast<float>(v > 0.0 ? std::sqrt(v) : 0.0);
            const double den = n_ * sum_tt_ - sum_t_ * sum_t_;
            s.rate_mps = den > 1e-9 ? static_cast<float>((n_ * sum_ta_ - sum_t_ * sum_a_) / den) : 0.0F;
            const double span = static_cast<double>(now_ms - start_ms_) / 1000.0;
            s.sample_rate_hz = span > 0.0 ? static_cast<float>(changes_ / span) : 0.0F;
            last_ = s;
            last_ms_ = now_ms;
        }
        clear();
    }
    std::uint32_t n_ = 0, start_ms_ = 0, last_ms_ = 0, changes_ = 0, identical_run_ = 0;
    bool have_prev_ = false;
    std::int32_t prev_pressure_ = 0, prev_temp_ = 0;
    double sum_p_ = 0, sumsq_p_ = 0, sum_t_ = 0, sum_tt_ = 0, sum_a_ = 0, sum_ta_ = 0;
    BaroStats last_{};
};

// Reset history ring (kept in RTC_NOINIT memory by the runtime): the last 8 reset
// reasons with the uptime at the time. This is the pure ring with a validity
// word; the runtime places an instance in RTC_NOINIT and clears it on power-on.
struct ResetHistory {
    static constexpr std::uint32_t kMagic = 0x42495448U;   // "BITH"
    static constexpr std::size_t kDepth = 8;
    std::uint32_t magic;
    std::uint8_t count;
    std::uint8_t head;
    std::uint8_t reason[kDepth];
    std::uint32_t uptime_at_reset_s[kDepth];

    bool valid() const { return magic == kMagic && count <= kDepth && head < kDepth; }
    void clear()
    {
        magic = kMagic; count = 0; head = 0;
        for (std::size_t i = 0; i < kDepth; ++i) { reason[i] = 0; uptime_at_reset_s[i] = 0; }
    }
    void push(const ResetClass r, const std::uint32_t previous_uptime_s)
    {
        if (!valid()) clear();
        reason[head] = static_cast<std::uint8_t>(r);
        uptime_at_reset_s[head] = previous_uptime_s;
        head = static_cast<std::uint8_t>((head + 1U) % kDepth);
        if (count < kDepth) ++count;
    }
    // Number of fault resets (panic, watchdogs, brownout) in the history.
    std::uint8_t fault_count() const
    {
        std::uint8_t n = 0;
        if (!valid()) return 0;
        for (std::size_t i = 0; i < count; ++i) {
            if (reset_is_fault(static_cast<ResetClass>(reason[i]))) ++n;
        }
        return n;
    }
};

}  // namespace bit
