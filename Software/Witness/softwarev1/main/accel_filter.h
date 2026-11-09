#pragma once

// Filtered acceleration: low-g / high-g selection plus a low-pass filter.
//
// Selection: the low-g accelerometer (+/-16 g, 0.488 mg/count) is used until
// it nears saturation, then the high-g accelerometer (+/-320 g, 10.4 mg/count)
// is used for all three axes from that same sample on. "Nears saturation" is
// any axis of either sensor at or above kSwitchToHighMg: the low-g output is
// filtered on chip after its ADC clips, so a clipped reading can sit below
// full scale and is not a reliable trigger by itself. The switch back to low-g
// needs every axis of both sensors below kSwitchToLowMg for kReturnSamples
// consecutive samples (hysteresis, so it cannot chatter at the threshold).
//
// Filter: 2nd-order Butterworth low-pass per axis (bilinear transform with
// pre-warping), run on the selected signal. The filter state carries across a
// switch, so the output has no step; the cost is the filter's group delay
// (about 2.5 ms at 100 Hz / 960 Hz). A cutoff of 0 disables filtering.
//
// No ESP-IDF dependencies (host-tested in tests/accel_filter_test.cpp).

#include <cstdint>

namespace accel_filter {

inline constexpr float kSwitchToHighMg = 15000.0F;
inline constexpr float kSwitchToLowMg = 13000.0F;
inline constexpr std::uint32_t kReturnSamples = 10;

class Biquad {
public:
    // Butterworth low-pass; cutoff_hz <= 0 or >= sample_rate / 2 passes through.
    void configure(float cutoff_hz, float sample_rate_hz);
    // The first sample after reset() initializes the state to steady state.
    void reset() { primed_ = false; }
    float step(float input);

private:
    bool enabled_ = false;
    bool primed_ = false;
    float b0_ = 1.0F, b1_ = 0.0F, b2_ = 0.0F, a1_ = 0.0F, a2_ = 0.0F;
    float z1_ = 0.0F, z2_ = 0.0F;  // transposed direct form II state
};

class FilteredAccel {
public:
    void configure(float cutoff_hz, float sample_rate_hz);
    void reset();
    // Corrected low-g and high-g readings in mg; returns the filtered vector.
    void step(const float low_g_mg[3], const float high_g_mg[3], float out_mg[3]);
    bool high_g_active() const { return high_g_active_; }
    std::uint32_t switches_to_high() const { return switches_to_high_; }
    float peak_mg() const { return peak_mg_; }  // largest |axis| seen on the selected input
    void reset_peak() { peak_mg_ = 0.0F; }

private:
    Biquad axes_[3];
    bool high_g_active_ = false;
    std::uint32_t below_count_ = 0;
    std::uint32_t switches_to_high_ = 0;
    float peak_mg_ = 0.0F;
};

}  // namespace accel_filter
