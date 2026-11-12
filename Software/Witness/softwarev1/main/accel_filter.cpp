#include "accel_filter.h"

#include <cmath>

namespace accel_filter {

void Biquad::configure(const float cutoff_hz, const float sample_rate_hz)
{
    enabled_ = cutoff_hz > 0.0F && sample_rate_hz > 0.0F &&
               cutoff_hz < 0.49F * sample_rate_hz;
    primed_ = false;
    if (!enabled_) {
        return;
    }
    // Bilinear transform of H(s) = 1 / (s^2 + sqrt(2) s + 1), pre-warped so
    // the -3 dB point lands exactly on cutoff_hz.
    const double k = std::tan(3.14159265358979 * cutoff_hz / sample_rate_hz);
    const double q = std::sqrt(2.0);
    const double norm = 1.0 / (1.0 + q * k + k * k);
    b0_ = static_cast<float>(k * k * norm);
    b1_ = 2.0F * b0_;
    b2_ = b0_;
    a1_ = static_cast<float>(2.0 * (k * k - 1.0) * norm);
    a2_ = static_cast<float>((1.0 - q * k + k * k) * norm);
}

float Biquad::step(const float input)
{
    if (!enabled_) {
        return input;
    }
    if (!primed_) {
        // Steady state for a constant input x: y = x, so
        // z1 = x - b0 x, z2 = b2 x - a2 x.
        z1_ = input * (1.0F - b0_);
        z2_ = input * (b2_ - a2_);
        primed_ = true;
    }
    const float output = b0_ * input + z1_;
    z1_ = b1_ * input - a1_ * output + z2_;
    z2_ = b2_ * input - a2_ * output;
    return output;
}

void FilteredAccel::configure(const float cutoff_hz, const float sample_rate_hz)
{
    for (Biquad &axis : axes_) {
        axis.configure(cutoff_hz, sample_rate_hz);
    }
    reset();
}

void FilteredAccel::reset()
{
    for (Biquad &axis : axes_) {
        axis.reset();
    }
    high_g_active_ = false;
    below_count_ = 0;
}

void FilteredAccel::step(const float low_g_mg[3], const float high_g_mg[3],
                         float out_mg[3])
{
    bool any_high = false;
    bool all_low = true;
    for (int axis = 0; axis < 3; ++axis) {
        const float low = std::fabs(low_g_mg[axis]);
        const float high = std::fabs(high_g_mg[axis]);
        if (low >= kSwitchToHighMg || high >= kSwitchToHighMg) any_high = true;
        if (low >= kSwitchToLowMg || high >= kSwitchToLowMg) all_low = false;
    }
    if (any_high) {
        if (!high_g_active_) ++switches_to_high_;
        high_g_active_ = true;  // same sample: no delay on the way up
        below_count_ = 0;
    } else if (high_g_active_) {
        below_count_ = all_low ? below_count_ + 1 : 0;
        if (below_count_ >= kReturnSamples) {
            high_g_active_ = false;
            below_count_ = 0;
        }
    }
    const float *selected = high_g_active_ ? high_g_mg : low_g_mg;
    for (int axis = 0; axis < 3; ++axis) {
        const float magnitude = std::fabs(selected[axis]);
        if (magnitude > peak_mg_) peak_mg_ = magnitude;
        out_mg[axis] = axes_[axis].step(selected[axis]);
    }
}

}  // namespace accel_filter


namespace accel_filter {

void HighGCalibration::reset()
{
    for (Sums &s : sums_) s = Sums{};
    for (int axis = 0; axis < 3; ++axis) block_x_[axis] = block_y_[axis] = 0.0;
    samples_ = 0;
    block_n_ = 0;
    holdoff_ = 0;
}

void HighGCalibration::add(const double x[3], const double y[3], const double keep)
{
    for (int axis = 0; axis < 3; ++axis) {
        Sums &s = sums_[axis];
        s.n = s.n * keep + 1.0;
        s.x = s.x * keep + x[axis];
        s.y = s.y * keep + y[axis];
        s.xx = s.xx * keep + x[axis] * x[axis];
        s.xy = s.xy * keep + x[axis] * y[axis];
    }
}

void HighGCalibration::observe(const float low_g_mg[3], const float high_g_mg[3], const bool on_pad)
{
    // Pad samples: Pad Idle and every axis near rest. Anything else in the
    // overlap range is a flight block, including the ignition ramp before
    // the state machine has declared Boost (1 g to 13 g in the first ~0.1 s
    // of the burn: the spread that makes the thrust-axis scale observable
    // before most of the first boost).
    bool in_range = true;
    bool near_rest = true;
    for (int axis = 0; axis < 3; ++axis) {
        if (!(std::fabs(low_g_mg[axis]) < kOverlapMaxMg) || !(std::fabs(high_g_mg[axis]) < kOverlapMaxMg))
            in_range = false;
        if (!(std::fabs(low_g_mg[axis]) < kPadMaxMg)) near_rest = false;
    }
    const bool pad_sample = on_pad && near_rest;
    if (!in_range) {
        holdoff_ = kHoldoffSamples;
        block_n_ = 0;   // a block never spans a clipped stretch
        for (int axis = 0; axis < 3; ++axis) block_x_[axis] = block_y_[axis] = 0.0;
        return;
    }
    if (holdoff_ > 0) {
        --holdoff_;
        return;
    }
    if (pad_sample) {
        // Still on the pad: per sample, forgotten with the pad window.
        if (samples_ < kMinSamples) ++samples_;
        const double x[3] = {low_g_mg[0], low_g_mg[1], low_g_mg[2]};
        const double y[3] = {high_g_mg[0], high_g_mg[1], high_g_mg[2]};
        add(x, y, 1.0 - 1.0 / kPadWindowSamples);
        return;
    }
    for (int axis = 0; axis < 3; ++axis) {
        block_x_[axis] += low_g_mg[axis];
        block_y_[axis] += high_g_mg[axis];
    }
    if (++block_n_ < kBlockSamples) return;
    double x[3], y[3];
    for (int axis = 0; axis < 3; ++axis) {
        x[axis] = block_x_[axis] / block_n_;
        y[axis] = block_y_[axis] / block_n_;
        block_x_[axis] = block_y_[axis] = 0.0;
    }
    block_n_ = 0;
    // One block mean stands for kBlockSamples pad-equivalent samples' worth
    // of weight against the pad window: weight it the same as the samples.
    for (std::uint32_t i = 0; i < kBlockSamples; ++i) add(x, y, 1.0);
}

float HighGCalibration::scale(const int axis) const
{
    const Sums &s = sums_[axis];
    if (!(s.n > 0.0)) return 1.0F;
    const double mx = s.x / s.n;
    const double var = s.xx / s.n - mx * mx;
    if (!(var > static_cast<double>(kMinSpreadMg) * kMinSpreadMg)) return 1.0F;
    const double cov = s.xy / s.n - mx * (s.y / s.n);
    const double k = cov / var;
    if (!(k >= kMinScale && k <= kMaxScale)) return 1.0F;
    return static_cast<float>(k);
}

float HighGCalibration::offset_mg(const int axis) const
{
    const Sums &s = sums_[axis];
    if (!(s.n > 0.0)) return 0.0F;
    // With scale k: offset = mean(y) - k mean(x); k = 1 gives mean(y - x).
    return static_cast<float>(s.y / s.n - scale(axis) * (s.x / s.n));
}

void HighGCalibration::apply(float high_g_mg[3]) const
{
    if (!valid()) return;
    for (int axis = 0; axis < 3; ++axis)
        high_g_mg[axis] = (high_g_mg[axis] - offset_mg(axis)) / scale(axis);
}

}  // namespace accel_filter
