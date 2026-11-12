#include "accel_filter.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using accel_filter::Biquad;
using accel_filter::FilteredAccel;

namespace {

int failures = 0;

void check(const bool condition, const char *what, const int line)
{
    if (!condition) {
        std::printf("FAIL line %d: %s\n", line, what);
        ++failures;
    }
}
#define CHECK(c) check((c), #c, __LINE__)

constexpr float kRate = 960.0F;
constexpr float kPi = 3.14159265F;

// Steady-state amplitude of a sine through the filter.
float gain_at(const float frequency_hz)
{
    Biquad filter;
    filter.configure(100.0F, kRate);
    float peak = 0.0F;
    for (int n = 0; n < 4000; ++n) {
        const float y = filter.step(std::sin(2.0F * kPi * frequency_hz * n / kRate));
        if (n > 2000 && std::fabs(y) > peak) peak = std::fabs(y);
    }
    return peak;
}

void test_response()
{
    CHECK(std::fabs(gain_at(10.0F) - 1.0F) < 0.01F);       // passband
    CHECK(std::fabs(gain_at(100.0F) - 0.7071F) < 0.02F);   // -3 dB at cutoff
    CHECK(gain_at(400.0F) < 0.06F);                        // 400 Hz well down

    // Starts in steady state: a constant input comes straight through.
    Biquad filter;
    filter.configure(100.0F, kRate);
    for (int n = 0; n < 50; ++n) CHECK(std::fabs(filter.step(980.0F) - 980.0F) < 0.01F);

    // Step response settles (no lasting overshoot) within 20 ms.
    filter.reset();
    filter.step(0.0F);
    float y = 0.0F;
    float maximum = 0.0F;
    for (int n = 0; n < 20; ++n) {
        y = filter.step(1000.0F);
        if (y > maximum) maximum = y;
    }
    CHECK(std::fabs(y - 1000.0F) < 5.0F);
    CHECK(maximum < 1060.0F);  // Butterworth overshoot is about 4%

    // Cutoff 0 (or at/above Nyquist) passes the input through.
    Biquad off;
    off.configure(0.0F, kRate);
    CHECK(off.step(123.0F) == 123.0F);
    off.configure(600.0F, kRate);
    CHECK(off.step(-7.0F) == -7.0F);
}

void test_instant_switch_and_hysteresis()
{
    FilteredAccel accel;
    accel.configure(0.0F, kRate);  // selection only, to see the raw choice
    float out[3];
    const float high_offset = 30.0F;  // the two sensors disagree slightly
    auto run = [&](const float true_x_mg) {
        const float low_x = std::fmin(std::fmax(true_x_mg, -15990.0F), 15990.0F);
        const float low[3] = {low_x, 0.0F, 980.0F};
        const float high[3] = {true_x_mg + high_offset, 0.0F, 980.0F};
        accel.step(low, high, out);
    };

    run(5000.0F);
    CHECK(!accel.high_g_active());
    CHECK(out[0] == 5000.0F);  // low-g below the threshold

    run(15500.0F);  // first sample at/above 12 g: high-g on this very sample
    CHECK(accel.high_g_active());
    CHECK(out[0] == 15500.0F + high_offset);
    CHECK(accel.switches_to_high() == 1);

    run(40000.0F);  // low-g clipped at 16 g, high-g follows the truth
    CHECK(out[0] == 40000.0F + high_offset);

    // Between 10 and 12 g: stays on high-g.
    for (int n = 0; n < 50; ++n) run(11000.0F);
    CHECK(accel.high_g_active());

    // Below 10 g: returns after kReturnSamples consecutive samples.
    for (std::uint32_t n = 0; n + 1 < accel_filter::kReturnSamples; ++n) run(8000.0F);
    CHECK(accel.high_g_active());
    run(14000.0F);  // a sample above 10 g restarts the count
    for (std::uint32_t n = 0; n + 1 < accel_filter::kReturnSamples; ++n) run(8000.0F);
    CHECK(accel.high_g_active());
    run(8000.0F);
    CHECK(!accel.high_g_active());
    CHECK(out[0] == 8000.0F);
    CHECK(accel.switches_to_high() == 1);

    // Any axis of either sensor can trigger it (here high-g z).
    const float low[3] = {0.0F, 0.0F, 15990.0F};
    const float high[3] = {0.0F, 0.0F, 22000.0F};
    accel.step(low, high, out);
    CHECK(accel.high_g_active());
    CHECK(out[2] == 22000.0F);
}

void test_filtered_switch_is_continuous()
{
    // A 30 g boost pulse: the filtered output tracks it with no jump at the
    // switch, and reads the full 30 g (the low-g alone would clip at 16 g).
    // The reference is the same filter fed the true (unclipped) signal; the
    // switched output must match it, i.e. switching adds no step at all.
    FilteredAccel accel;
    accel.configure(100.0F, kRate);
    Biquad ideal;
    ideal.configure(100.0F, kRate);
    float out[3];
    float largest_error = 0.0F;
    float peak = 0.0F;
    for (int n = 0; n < 960; ++n) {
        const float t = n / kRate;
        const float truth = (t > 0.2F && t < 0.7F) ? 30000.0F : 1000.0F;
        // Slew like a real thrust curve: 5 ms rise.
        const float ramp = (t > 0.2F && t < 0.205F) ? 1000.0F + (t - 0.2F) / 0.005F * 29000.0F : truth;
        const float low[3] = {std::fmin(ramp, 15990.0F), 0.0F, 0.0F};
        const float high[3] = {ramp, 0.0F, 0.0F};
        accel.step(low, high, out);
        const float error = std::fabs(out[0] - ideal.step(ramp));
        if (error > largest_error) largest_error = error;
        if (out[0] > peak) peak = out[0];
    }
    CHECK(std::fabs(peak - 30000.0F) < 1300.0F);  // reaches 30 g (small overshoot)
    CHECK(largest_error < 1.0F);                  // identical to the unclipped ideal
    CHECK(!accel.high_g_active());                // back on low-g after the pulse
}

// High-g calibration: the pad pins the offset; coasting in flight (one axis
// spread over -5..-0.5 g, the others near 0) makes that axis's scale
// observable; clipped stretches and the 100 ms after them are not used.
void test_high_g_calibration()
{
    accel_filter::HighGCalibration cal;
    const float offset[3] = {1500.0F, -1400.0F, 300.0F};
    const float scale_x = 0.95F;
    float low[3], high[3];
    // 2 s on the pad, 1 g on x.
    for (int i = 0; i < 1920; ++i) {
        low[0] = 1000.0F; low[1] = 3.0F; low[2] = -2.0F;
        for (int a = 0; a < 3; ++a) high[a] = (a == 0 ? scale_x : 1.0F) * low[a] + offset[a];
        cal.observe(low, high, true);
    }
    CHECK(cal.valid());
    CHECK(std::fabs(cal.scale(0) - 1.0F) < 1e-6F);    // no spread yet: offset only
    float h[3] = {1000.0F * scale_x + offset[0], offset[1], offset[2]};
    cal.apply(h);
    CHECK(std::fabs(h[0] - 1000.0F) < 1.0F);          // matches low-g at the pad's 1 g
    CHECK(std::fabs(h[1]) < 1.0F && std::fabs(h[2]) < 1.0F);
    // Boost: clipped (not used), then 10 s of coast from -5 g to -0.5 g. The
    // first 100 ms after the clip read 2 g low (filter recovery) and must be
    // ignored or the scale would come out high.
    for (int i = 0; i < 2000; ++i) {
        low[0] = 16000.0F; low[1] = low[2] = 0.0F;
        high[0] = 25000.0F; high[1] = high[2] = 0.0F;
        cal.observe(low, high, false);
    }
    for (int i = 0; i < 9600; ++i) {
        const float x = -5000.0F + 4500.0F * static_cast<float>(i) / 9600.0F;
        low[0] = i < 96 ? x - 2000.0F : x;
        low[1] = 10.0F * std::sin(0.1F * i); low[2] = -5.0F;
        for (int a = 0; a < 3; ++a) high[a] = (a == 0 ? scale_x * x : low[a]) + offset[a];
        cal.observe(low, high, false);
    }
    CHECK(std::fabs(cal.scale(0) - scale_x) < 0.002F);
    CHECK(std::fabs(cal.offset_mg(0) - offset[0]) < 10.0F);
    CHECK(std::fabs(cal.scale(1) - 1.0F) < 1e-6F);    // lateral axes: no real spread
    float burn[3] = {scale_x * 40000.0F + offset[0], offset[1], offset[2]};
    cal.apply(burn);
    CHECK(std::fabs(burn[0] - 40000.0F) < 120.0F);    // 40 g read right after the fit
}

}  // namespace

int main()
{
    test_response();
    test_instant_switch_and_hysteresis();
    test_filtered_switch_is_continuous();
    test_high_g_calibration();
    if (failures == 0) {
        std::printf("accel_filter_test: all passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("accel_filter_test: %d failure(s)\n", failures);
    return EXIT_FAILURE;
}
