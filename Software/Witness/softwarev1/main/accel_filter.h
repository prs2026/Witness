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

// Margins for vibration: under the flight's predicted vibration the low-g
// channel clips on peaks from ~12 g of steady thrust and its (post-clip,
// on-chip filtered) average reads 0.5-2 g low; HITL eayf with vibration lost
// ~8 m in the first second of boost by dropping back to it between peaks.
// The high-g channel's extra noise costs nothing measurable in position, so
// it takes over early and is left only after 100 ms clearly below.
inline constexpr float kSwitchToHighMg = 12000.0F;
inline constexpr float kSwitchToLowMg = 10000.0F;
inline constexpr std::uint32_t kReturnSamples = 96;   // 100 ms at 960 Hz

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

// High-g accelerometer calibrated against the low-g one while both are in
// range. The high-g channel's zero-g level is specified only to about
// +/-1.5 g and its sensitivity is looser than the low-g's, yet it carries the
// whole burn above 15 g: in HITL eayf an uncorrected 1.5 g offset was 45 m/s
// by burnout, and in the Monte Carlo a few % of sensitivity error on the
// thrust axis was most of the dead-reckoning error through the GPS outage
// after the sustainer.
//
// Both accelerometers sit on one die, so whenever the low-g reading is valid
// (every |axis| below kOverlapMaxMg on both sensors: no clipping, no handoff
// region) they measure the same specific force and per axis
//     high = scale * low + offset.
// The fit is least squares over those samples: on the pad (1 g on one axis)
// it pins the offset; once the vehicle is flying, coasting phases spread the
// samples (eayf: -0.5 to -5 g between boost and sustainer) and the scale
// becomes observable. Pad samples are forgotten with a kPadWindowSamples time
// constant (temperature drift while waiting); flight samples are all kept.
// Until the spread of the low-g readings on an axis reaches kMinSpreadMg (and
// while the fitted scale is outside [kMinScale, kMaxScale]) only the offset,
// mean(high - low), is corrected on that axis. A stored six-face calibration
// is applied to both readings first; this fit absorbs what is left.
class HighGCalibration {
public:
    static constexpr float kOverlapMaxMg = 13000.0F;
    static constexpr float kPadMaxMg = 2000.0F;               // still on the pad
    static constexpr double kPadWindowSamples = 9600.0;       // 10 s at 960 Hz
    static constexpr std::uint32_t kMinSamples = 960;         // 1 s before any correction
    static constexpr float kMinSpreadMg = 700.0F;             // std of low-g readings
    static constexpr float kMinScale = 0.85F;
    static constexpr float kMaxScale = 1.15F;
    void reset();
    // on_pad: Pad Idle (pad samples need |low| < kPadMaxMg and are forgotten
    // with the pad window); otherwise flight samples (kept).
    void observe(const float low_g_mg[3], const float high_g_mg[3], bool on_pad);
    // Corrects high_g_mg in place: (high - offset) / scale, or high - offset.
    void apply(float high_g_mg[3]) const;
    bool valid() const { return samples_ >= kMinSamples; }
    // Current per-axis estimates (scale 1 when not yet observable).
    float scale(int axis) const;
    float offset_mg(int axis) const;

private:
    struct Sums {
        double n = 0, x = 0, y = 0, xx = 0, xy = 0;
    };
    Sums sums_[3]{};
    std::uint32_t samples_ = 0;
    // Flight samples are averaged in blocks of kBlockSamples before the fit
    // (vibration averages out, so an axis only shows spread from real
    // low-frequency acceleration), and none are used for kHoldoffSamples
    // after any reading reached kOverlapMaxMg (the low-g output recovers from
    // clipping through its on-chip filter and reads low meanwhile).
    static constexpr std::uint32_t kBlockSamples = 48;     // 50 ms
    static constexpr std::uint32_t kHoldoffSamples = 96;   // 100 ms
    double block_x_[3]{};
    double block_y_[3]{};
    std::uint32_t block_n_ = 0;
    std::uint32_t holdoff_ = 0;
    void add(const double x[3], const double y[3], double keep);
};

}  // namespace accel_filter
