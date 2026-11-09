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
