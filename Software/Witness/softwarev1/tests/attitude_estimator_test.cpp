#include "attitude_estimator.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <vector>

using attitude::Config;
using attitude::Estimator;
using attitude::FlightPhase;
using attitude::Quaternion;
using attitude::Step;
using attitude::Vector3;

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

constexpr float kPi = 3.14159265358979F;
constexpr float kDegToRad = kPi / 180.0F;
constexpr float kGyroDpsPerCount = 0.070F;  // LSM6 +/-2000 dps

// Rotation distance in radians, independent of quaternion sign.
float angle_between(const Quaternion &a, const Quaternion &b)
{
    // Relative rotation conj(a) * b; atan2 stays accurate for small angles.
    const Quaternion r = Estimator::multiply(Quaternion{a.w, -a.x, -a.y, -a.z}, b);
    const float v = std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
    return 2.0F * std::atan2(v, std::fabs(r.w));
}

bool is_zero(const Quaternion &q)
{
    return q.w == 0.0F && q.x == 0.0F && q.y == 0.0F && q.z == 0.0F;
}

Vector3 unit(const Vector3 &v)
{
    const float n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return {v.x / n, v.y / n, v.z / n};
}

Vector3 scaled(const Vector3 &v, const float s)
{
    return {v.x * s, v.y * s, v.z * s};
}

float vec_error(const Vector3 &a, const Vector3 &b)
{
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                     (a.z - b.z) * (a.z - b.z));
}

// Simple deterministic LCG for noise and jitter.
std::uint32_t rng_state = 12345U;
float uniform(const float lo, const float hi)
{
    rng_state = rng_state * 1664525U + 1013904223U;
    return lo + (hi - lo) * static_cast<float>(rng_state >> 8) / 16777216.0F;
}

float gaussian(const float sd)
{
    const float u1 = uniform(1e-7F, 1.0F);
    const float u2 = uniform(0.0F, 1.0F);
    return sd * std::sqrt(-2.0F * std::log(u1)) * std::cos(2.0F * kPi * u2);
}

struct Feeder {
    Estimator &est;
    std::uint64_t t_us = 1'000'000U;
    attitude::StepResult last{};
    attitude::StepResult accumulated{};

    void run(const Vector3 &gyro, const Vector3 &accel, const FlightPhase phase,
             const int count, const std::uint32_t dt_us = 10'000U)
    {
        for (int i = 0; i < count; ++i) {
            t_us += dt_us;
            Step s{};
            s.gyro_rad_s = gyro;
            s.specific_force_g = accel;
            s.timestamp_us = t_us;
            s.phase = phase;
            last = est.update(s);
            accumulated.initialized_now |= last.initialized_now;
            accumulated.bias_updated |= last.bias_updated;
            accumulated.entered_landed |= last.entered_landed;
            accumulated.gap |= last.gap;
        }
    }
};

const Vector3 kZero{};
const Vector3 kUpright{1.0F, 0.0F, 0.0F};

Quaternion initialized_upright(Estimator &est, Feeder &f)
{
    f.run(kZero, kUpright, FlightPhase::PadIdle, 120);
    CHECK(est.valid());
    return est.quaternion();
}

void test_zero_before_init()
{
    Estimator est;
    Feeder f{est};
    CHECK(!est.valid());
    CHECK(is_zero(est.quaternion()));
    f.run(kZero, kUpright, FlightPhase::PadIdle, 50);  // 0.5 s: not enough
    CHECK(!est.valid());
    CHECK(is_zero(est.quaternion()));
    // An out-of-band accel sample restarts the window.
    f.run(kZero, Vector3{1.3F, 0.0F, 0.0F}, FlightPhase::PadIdle, 1);
    f.run(kZero, kUpright, FlightPhase::PadIdle, 60);
    CHECK(!est.valid());
    f.run(kZero, kUpright, FlightPhase::PadIdle, 50);
    CHECK(est.valid());
    CHECK(f.accumulated.initialized_now);
    CHECK(!is_zero(est.quaternion()));
    // Flight before initialization stays unavailable.
    Estimator est2;
    Feeder f2{est2};
    f2.run(Vector3{0.0F, 0.0F, 1.0F}, Vector3{12.0F, 0.0F, 0.0F},
           FlightPhase::Boost, 200);
    CHECK(!est2.valid());
    CHECK(is_zero(est2.quaternion()));
    // A rotating board (accel steady about the rotation axis) is rejected.
    Estimator est3;
    Feeder f3{est3};
    f3.run(Vector3{10.0F * kDegToRad, 0.0F, 0.0F}, kUpright, FlightPhase::PadIdle, 500);
    CHECK(!est3.valid());
    CHECK(est3.windows_rejected_gyro() >= 4U);
}

void test_tilt_init_orientations()
{
    const Vector3 orientations[] = {
        {1.0F, 0.0F, 0.0F},   {-1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F},
        {0.0F, 0.0F, -1.0F},  {0.0F, 0.0F, 1.0F},  {0.866F, 0.5F, 0.0F},
        {0.7F, -0.3F, 0.648F}, {0.02F, 0.0F, -0.9998F},
    };
    for (const Vector3 &o : orientations) {
        Estimator est;
        Feeder f{est};
        const Vector3 a = unit(o);
        f.run(kZero, a, FlightPhase::PadIdle, 120);
        CHECK(est.valid());
        const Quaternion q = est.quaternion();
        const float n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        CHECK(std::fabs(n - 1.0F) < 1e-5F);
        const Vector3 up = Estimator::rotate(q, a);
        CHECK(vec_error(up, Vector3{0.0F, 0.0F, 1.0F}) < 1e-4F);
        const float expected_tilt = std::acos(a.x) / kDegToRad;
        CHECK(std::fabs(est.tilt_from_vertical_deg() - expected_tilt) < 0.05F);
    }
    // Upright: shortest arc is -90 degrees about Y. Body +X -> +Z (up),
    // body +Y -> +Y (north), body +Z -> -X (west).
    Estimator est;
    Feeder f{est};
    const Quaternion q = initialized_upright(est, f);
    CHECK(vec_error(Estimator::rotate(q, Vector3{1, 0, 0}), Vector3{0, 0, 1}) < 1e-5F);
    CHECK(vec_error(Estimator::rotate(q, Vector3{0, 1, 0}), Vector3{0, 1, 0}) < 1e-5F);
    CHECK(vec_error(Estimator::rotate(q, Vector3{0, 0, 1}), Vector3{-1, 0, 0}) < 1e-5F);
    const float h = std::sqrt(0.5F);
    CHECK(angle_between(q, Quaternion{h, 0.0F, -h, 0.0F}) < 1e-5F);
    CHECK(std::fabs(est.tilt_from_vertical_deg()) < 0.05F);
}

void test_bias_estimation()
{
    Estimator est;
    Feeder f{est};
    const Vector3 bias{0.010F, -0.020F, 0.005F};  // rad/s (~0.6..1.1 dps)
    for (int i = 0; i < 150; ++i) {
        const Vector3 g{bias.x + uniform(-0.002F, 0.002F),
                        bias.y + uniform(-0.002F, 0.002F),
                        bias.z + uniform(-0.002F, 0.002F)};
        const Vector3 a{1.0F + uniform(-0.003F, 0.003F), uniform(-0.003F, 0.003F),
                        uniform(-0.003F, 0.003F)};
        f.run(g, a, FlightPhase::PadIdle, 1);
    }
    CHECK(est.valid());
    CHECK(vec_error(est.gyro_bias_rad_s(), bias) < 5e-4F);
    // Exactly bias-rate input: the median is the bias; no drift in flight.
    Estimator exact;
    Feeder fe{exact};
    fe.run(bias, kUpright, FlightPhase::PadIdle, 120);
    const Quaternion qe = exact.quaternion();
    CHECK(vec_error(exact.gyro_bias_rad_s(), bias) < 1e-6F);
    fe.run(bias, kUpright, FlightPhase::Boost, 1000);
    CHECK(angle_between(qe, exact.quaternion()) < 1e-4F);
}

void test_constant_rate()
{
    const Vector3 rates[] = {
        {0.0F, 0.0F, 1.0F},           // 57 dps about body Z
        {6.0F, 0.0F, 0.0F},           // fast roll about the nose
        {0.5F, -1.2F, 2.0F},          // general axis
        {30.0F, 0.0F, 0.0F},          // 1720 dps, near gyro full scale
    };
    for (const Vector3 &w : rates) {
        // 100 Hz exact. The median delays the integration by 2 samples, so
        // after 100 rate samples 98 of them have been integrated.
        {
            Estimator est;
            Feeder f{est};
            const Quaternion q0 = initialized_upright(est, f);
            f.run(w, kZero, FlightPhase::Coast, 100);
            const Quaternion expected =
                Estimator::multiply(q0, Estimator::exp_rotation(scaled(w, 0.98F)));
            CHECK(angle_between(expected, est.quaternion()) < 2e-4F);
            CHECK(est.flight_steps() == 100U);
            CHECK(std::fabs(est.flight_time_s() - 1.0) < 1e-5);
        }
        // Jittered dt: 6..16 ms (some late steps).
        {
            Estimator est;
            Feeder f{est};
            const Quaternion q0 = initialized_upright(est, f);
            std::vector<std::uint32_t> dts;
            for (int i = 0; i < 150; ++i) {
                const std::uint32_t dt = static_cast<std::uint32_t>(uniform(6000.0F, 16000.0F));
                dts.push_back(dt);
                f.run(w, kZero, FlightPhase::Boost, 1, dt);
            }
            double total_us = 0.0;
            for (std::size_t i = 0; i + 2U < dts.size(); ++i) total_us += dts[i];
            const float total = static_cast<float>(total_us * 1e-6);
            const Quaternion expected =
                Estimator::multiply(q0, Estimator::exp_rotation(scaled(w, total)));
            CHECK(angle_between(expected, est.quaternion()) < 5e-4F);
            CHECK(est.gaps() == 0U);
            CHECK(est.late_steps() > 0U);
            CHECK(est.max_dt_us() <= 16000U);
            CHECK(est.spikes_rejected() == 0U);  // a step is not a spike
        }
    }
    // Quarter turn about body Y (pitch) from upright tips the nose 90 degrees
    // (102 samples: 2 extra flush the filter delay).
    Estimator est;
    Feeder f{est};
    initialized_upright(est, f);
    f.run(Vector3{0.0F, kPi / 2.0F, 0.0F}, kZero, FlightPhase::Coast, 102);
    CHECK(std::fabs(est.tilt_from_vertical_deg() - 90.0F) < 0.05F);
}

void test_median_passes_fast_rotation()
{
    // 0 -> 500 dps ramp over 1 s, then held: monotonic input passes exactly.
    {
        Estimator est;
        Feeder f{est};
        const Quaternion q0 = initialized_upright(est, f);
        double expected_angle = 0.0;
        for (int j = 1; j <= 100; ++j) {
            const float rate = 500.0F * kDegToRad * static_cast<float>(j) / 100.0F;
            f.run(Vector3{0.0F, rate, 0.0F}, kZero, FlightPhase::Coast, 1);
            expected_angle += static_cast<double>(rate) * 0.01;
        }
        f.run(Vector3{0.0F, 500.0F * kDegToRad, 0.0F}, kZero, FlightPhase::Coast, 10);
        expected_angle += 8.0 * 500.0 * static_cast<double>(kDegToRad) * 0.01;
        const Quaternion expected = Estimator::multiply(
            q0, Estimator::exp_rotation(
                    Vector3{0.0F, static_cast<float>(expected_angle), 0.0F}));
        CHECK(angle_between(expected, est.quaternion()) < 1e-3F);
        CHECK(est.spikes_rejected() == 0U);
    }
    // 1000 dps steps lasting 3 and 4 samples pass; 1- and 2-sample spikes
    // are removed.
    {
        Estimator est;
        Feeder f{est};
        const Quaternion q0 = initialized_upright(est, f);
        const Vector3 fast{1000.0F * kDegToRad, 0.0F, 0.0F};
        int on_samples = 0;
        for (const int length : {3, 4, 3, 4, 3}) {
            f.run(fast, kZero, FlightPhase::Coast, length);
            f.run(kZero, kZero, FlightPhase::Coast, 10);
            on_samples += length;
        }
        const Quaternion expected = Estimator::multiply(
            q0, Estimator::exp_rotation(scaled(fast, 0.01F * static_cast<float>(on_samples))));
        CHECK(angle_between(expected, est.quaternion()) < 1e-3F);
        CHECK(est.spikes_rejected() == 0U);

        const Quaternion before = est.quaternion();
        for (const int length : {1, 2, 1, 2}) {
            f.run(fast, kZero, FlightPhase::Coast, length);
            f.run(kZero, kZero, FlightPhase::Coast, 10);
        }
        CHECK(angle_between(before, est.quaternion()) < 1e-6F);
        CHECK(est.spikes_rejected() == 6U);
    }
    // Median of 3 removes single spikes but passes 2-sample runs.
    {
        Config cfg;
        cfg.gyro_median_length = 3U;
        Estimator est(cfg);
        Feeder f{est};
        const Quaternion q0 = initialized_upright(est, f);
        const Vector3 fast{1000.0F * kDegToRad, 0.0F, 0.0F};
        f.run(fast, kZero, FlightPhase::Coast, 1);
        f.run(kZero, kZero, FlightPhase::Coast, 5);
        CHECK(angle_between(q0, est.quaternion()) < 1e-6F);
        f.run(fast, kZero, FlightPhase::Coast, 2);
        f.run(kZero, kZero, FlightPhase::Coast, 5);
        CHECK(std::fabs(angle_between(q0, est.quaternion()) - 0.02F * fast.x) < 1e-4F);
    }
}

void test_gaps()
{
    Estimator est;
    Feeder f{est};
    const Quaternion q0 = initialized_upright(est, f);
    const Vector3 w{0.0F, 0.0F, 1.0F};
    f.run(w, kZero, FlightPhase::Coast, 1, 100'000U);  // 100 ms: gap
    CHECK(est.gaps() == 1U);
    CHECK(est.max_dt_us() == 100'000U);
    f.run(w, kZero, FlightPhase::Coast, 1, 0U);  // repeated timestamp: gap
    CHECK(est.gaps() == 2U);
    f.t_us -= 20'000U;  // backwards
    f.run(w, kZero, FlightPhase::Coast, 1, 10'000U);
    CHECK(est.gaps() == 3U);
    CHECK(angle_between(q0, est.quaternion()) < 1e-6F);
    f.run(w, kZero, FlightPhase::Coast, 10, 10'000U);
    CHECK(est.gaps() == 3U);
    // 13 rate samples, the 3 gap samples are not integrated and the newest 2
    // are still inside the median delay: 8 x 10 ms.
    CHECK(std::fabs(angle_between(q0, est.quaternion()) - 0.08F) < 1e-4F);
}

void test_launch_window()
{
    Estimator est;
    Feeder f{est};
    const Quaternion q0 = initialized_upright(est, f);
    // 300 ms of >10 g still in Pad Idle, with a lateral component that a
    // tilt correction would act on, and no rotation.
    f.run(kZero, Vector3{12.0F, 3.0F, -2.0F}, FlightPhase::PadIdle, 30);
    CHECK(angle_between(q0, est.quaternion()) < 1e-6F);
    // A single-sample gyro spike on the pad is rejected.
    f.run(Vector3{0.0F, 0.5F, 0.0F}, unit(Vector3{1.0F, 1.0F, 0.0F}),
          FlightPhase::PadIdle, 1);
    f.run(kZero, Vector3{12.0F, 0.0F, 0.0F}, FlightPhase::PadIdle, 5);
    CHECK(angle_between(q0, est.quaternion()) < 1e-6F);
    CHECK(est.spikes_rejected() == 1U);

    // Rotation during the launch window is kept (gyro runs on the pad).
    Estimator est2;
    Feeder f2{est2};
    const Quaternion p0 = initialized_upright(est2, f2);
    const Vector3 roll{10.0F, 0.0F, 0.0F};
    f2.run(roll, Vector3{12.0F, 0.0F, 0.0F}, FlightPhase::PadIdle, 30);
    f2.run(roll, Vector3{12.0F, 0.0F, 0.0F}, FlightPhase::Boost, 20);
    const Quaternion expected_q =
        Estimator::multiply(p0, Estimator::exp_rotation(Vector3{10.0F * 0.48F, 0.0F, 0.0F}));
    CHECK(angle_between(expected_q, est2.quaternion()) < 2e-4F);
}

void test_pad_tilt_correction()
{
    Estimator est;
    Feeder f{est};
    initialized_upright(est, f);
    // Rail tilted 10 degrees slowly enough that the gyro saw nothing: each
    // accepted window corrects half the error (1 s window, tau 2 s).
    const Vector3 tilted{std::cos(10.0F * kDegToRad), std::sin(10.0F * kDegToRad), 0.0F};
    f.run(kZero, tilted, FlightPhase::PadIdle, 300);
    CHECK(est.windows_rejected_accel() >= 1U);  // window spanning the change
    CHECK(est.tilt_from_vertical_deg() > 6.0F && est.tilt_from_vertical_deg() < 9.5F);
    f.run(kZero, tilted, FlightPhase::PadIdle, 2000);
    CHECK(std::fabs(est.tilt_from_vertical_deg() - 10.0F) < 0.05F);
    const Vector3 up = Estimator::rotate(est.quaternion(), tilted);
    CHECK(vec_error(up, Vector3{0.0F, 0.0F, 1.0F}) < 1e-3F);
}

void test_pure_integration_in_flight()
{
    for (const FlightPhase phase : {FlightPhase::Boost, FlightPhase::Coast, FlightPhase::Descent}) {
        Estimator est;
        Feeder f{est};
        const Quaternion q0 = initialized_upright(est, f);
        // A 1 g, zero-rate, tilted specific force must not be used in flight.
        f.run(kZero, unit(Vector3{0.5F, 0.8F, 0.0F}), phase, 500);
        CHECK(angle_between(q0, est.quaternion()) < 1e-6F);
        CHECK(est.flight_steps() == 500U);
    }
}

void test_landed_hold()
{
    Estimator est;
    Feeder f{est};
    initialized_upright(est, f);
    f.run(Vector3{0.0F, 0.3F, 0.0F}, kZero, FlightPhase::Descent, 200);
    CHECK(!f.accumulated.entered_landed);
    const Quaternion held = est.quaternion();
    const std::uint32_t steps = est.steps();
    f.run(Vector3{1.0F, 2.0F, 3.0F}, kUpright, FlightPhase::Landed, 1);
    CHECK(f.last.entered_landed);
    f.run(Vector3{1.0F, 2.0F, 3.0F}, kUpright, FlightPhase::Landed, 500);
    CHECK(!f.last.entered_landed);
    CHECK(est.landed_hold());
    CHECK(est.valid());
    CHECK(angle_between(held, est.quaternion()) < 1e-7F);
    CHECK(est.steps() == steps);
    CHECK(est.flight_steps() == 200U);
    CHECK(std::fabs(est.flight_time_s() - 2.0) < 1e-5);
    // 198 of the 200 descent samples were integrated (2-sample median delay).
    CHECK(std::fabs(est.tilt_from_vertical_deg() - 0.3F * 1.98F / kDegToRad) < 0.05F);
}

void test_gyro_saturation()
{
    const float sat = 2100.0F * kDegToRad;  // beyond the 1950 dps threshold
    // One saturated sample (an interference spike) does not stop tracking.
    {
        Estimator est;
        Feeder f{est};
        initialized_upright(est, f);
        f.run(Vector3{0.0F, 0.1F, 0.0F}, kZero, FlightPhase::Coast, 50);
        f.run(Vector3{sat, 0.0F, 0.0F}, kZero, FlightPhase::Coast, 1);
        f.run(Vector3{0.0F, 0.1F, 0.0F}, kZero, FlightPhase::Coast, 50);
        CHECK(est.valid());
        CHECK(!est.lost());
    }
    // Two in a row in flight: lost for the rest of the flight.
    {
        Estimator est;
        Feeder f{est};
        initialized_upright(est, f);
        f.run(Vector3{0.0F, 0.1F, 0.0F}, kZero, FlightPhase::Descent, 50);
        const std::uint32_t steps = est.steps();
        f.run(Vector3{0.0F, -sat, 0.0F}, kZero, FlightPhase::Descent, 1);
        CHECK(!est.lost());
        f.run(Vector3{0.0F, -sat, 0.0F}, kZero, FlightPhase::Descent, 1);
        CHECK(f.last.lost_now);
        CHECK(est.lost());
        CHECK(!est.valid());
        CHECK(is_zero(est.quaternion()));
        CHECK(est.saturation_events() == 1U);
        // Quiet gyro afterwards (and Landed) does not bring it back.
        f.run(kZero, kUpright, FlightPhase::Descent, 500);
        f.run(kZero, kUpright, FlightPhase::Landed, 500);
        CHECK(est.lost());
        CHECK(is_zero(est.quaternion()));
        CHECK(est.steps() <= steps + 2U);
        // Back in Pad Idle: a stationary window re-initializes.
        f.run(kZero, kUpright, FlightPhase::PadIdle, 150);
        CHECK(!est.lost());
        CHECK(est.valid());
    }
    // Saturation on the pad is left to the stationary windows.
    {
        Estimator est;
        Feeder f{est};
        initialized_upright(est, f);
        f.run(Vector3{0.0F, 0.0F, sat}, kUpright, FlightPhase::PadIdle, 10);
        CHECK(!est.lost());
    }
}

void test_helpers()
{
    for (const float t : {1e-6F, 5e-4F, 9.99e-4F, 1.001e-3F, 0.01F}) {
        const Quaternion q = Estimator::exp_rotation(Vector3{t, 0.0F, 0.0F});
        CHECK(std::fabs(q.w - std::cos(t / 2.0F)) < 1e-7F);
        CHECK(std::fabs(q.x - std::sin(t / 2.0F)) < 1e-9F + 1e-6F * t);
    }
    const Quaternion zero = Estimator::exp_rotation(kZero);
    CHECK(zero.w == 1.0F && zero.x == 0.0F);
}

// ---------------------------------------------------------------------------
// Bench-measured interference (Witness, 2026-11-08): consecutive gyro Z
// samples in LSM6 counts (70 mdps/count); X/Y spike at the same instants with
// smaller amplitude. Injected as 18-sample bursts at random times.
constexpr int kBurst[] = {0, -5, -573, 1384, 541, 77, -19, 0, -33,
                          94, 177, -702, -2, 697, 6, 8, -17, 1};
constexpr int kBurstLength = static_cast<int>(sizeof(kBurst) / sizeof(kBurst[0]));

struct NoiseSource {
    float coverage;            // Fraction of samples inside a burst.
    Vector3 bias_dps{0.4F, -0.3F, 0.6F};
    float base_sd_counts = 2.0F;
    int position = -1;         // Inside a burst when >= 0.
    float sign_x = 1.0F;
    float sign_y = 1.0F;

    Vector3 next_gyro_rad_s()
    {
        if (position < 0) {
            const float mean_gap = static_cast<float>(kBurstLength) * (1.0F - coverage) / coverage;
            if (uniform(0.0F, 1.0F) < 1.0F / (mean_gap + 1.0F)) {
                position = 0;
                sign_x = uniform(0.0F, 1.0F) < 0.5F ? -1.0F : 1.0F;
                sign_y = uniform(0.0F, 1.0F) < 0.5F ? -1.0F : 1.0F;
            }
        }
        float spike = 0.0F;
        if (position >= 0) {
            spike = static_cast<float>(kBurst[position]);
            if (++position >= kBurstLength) position = -1;
        }
        const float x = bias_dps.x + kGyroDpsPerCount * (gaussian(base_sd_counts) + 0.15F * sign_x * spike);
        const float y = bias_dps.y + kGyroDpsPerCount * (gaussian(base_sd_counts) + 0.24F * sign_y * spike);
        const float z = bias_dps.z + kGyroDpsPerCount * (gaussian(base_sd_counts) + spike);
        return {x * kDegToRad, y * kDegToRad, z * kDegToRad};
    }
};

struct NoiseResult {
    float init_time_s = -1.0F;
    float bias_error_dps = 0.0F;
    float final_drift_deg = 0.0F;
    float max_drift_deg = 0.0F;
    std::uint32_t spikes = 0;
    std::uint32_t accepted = 0;
    std::uint32_t rejected = 0;
};

NoiseResult run_noise(const float coverage, const std::uint32_t median_length,
                      const FlightPhase phase_after_init, const std::uint32_t seed,
                      const int samples_after_init)
{
    rng_state = seed;
    Config cfg;
    cfg.gyro_median_length = median_length;
    Estimator est(cfg);
    NoiseSource noise{coverage};
    std::uint64_t t = 1'000'000U;
    NoiseResult r;
    auto feed = [&](const FlightPhase phase) {
        t += 10'000U;
        Step s{};
        s.gyro_rad_s = noise.next_gyro_rad_s();
        s.specific_force_g = {0.99F + gaussian(0.0015F), gaussian(0.0015F), gaussian(0.0015F)};
        s.timestamp_us = t;
        s.phase = phase;
        est.update(s);
    };
    int samples = 0;
    while (!est.valid() && samples < 1000) {
        feed(FlightPhase::PadIdle);
        ++samples;
    }
    if (!est.valid()) return r;
    r.init_time_s = static_cast<float>(samples) * 0.01F;
    const Quaternion q0 = est.quaternion();
    for (int i = 0; i < samples_after_init; ++i) {
        feed(phase_after_init);
        const float drift = angle_between(q0, est.quaternion()) / kDegToRad;
        if (drift > r.max_drift_deg) r.max_drift_deg = drift;
    }
    r.final_drift_deg = angle_between(q0, est.quaternion()) / kDegToRad;
    const Vector3 truth = scaled(noise.bias_dps, kDegToRad);
    r.bias_error_dps = vec_error(est.gyro_bias_rad_s(), truth) / kDegToRad;
    r.spikes = est.spikes_rejected();
    r.accepted = est.windows_accepted();
    r.rejected = est.windows_rejected_accel() + est.windows_rejected_gyro();
    return r;
}

void test_bench_interference()
{
    struct Scenario {
        const char *name;
        float coverage;
    };
    const Scenario scenarios[] = {{"radio off (15% burst coverage)", 0.15F},
                                  {"radio on  (90% burst coverage)", 0.90F}};
    for (const Scenario &sc : scenarios) {
        for (const std::uint32_t seed : {1U, 2U, 3U}) {
            for (const std::uint32_t length : {5U, 3U, 1U}) {
                const NoiseResult pad = run_noise(sc.coverage, length, FlightPhase::PadIdle, seed, 6000);
                const NoiseResult coast = run_noise(sc.coverage, length, FlightPhase::Coast, seed, 1000);
                std::printf(
                    "%s seed %u median %u: init %.2f s, bias err %.3f dps, pad 60 s drift "
                    "final %.2f max %.2f deg (windows %u ok/%u rejected, spikes %u); "
                    "coast (pure integration) 10 s drift %.1f deg\n",
                    sc.name, static_cast<unsigned>(seed), static_cast<unsigned>(length),
                    static_cast<double>(pad.init_time_s),
                    static_cast<double>(pad.bias_error_dps),
                    static_cast<double>(pad.final_drift_deg),
                    static_cast<double>(pad.max_drift_deg), static_cast<unsigned>(pad.accepted),
                    static_cast<unsigned>(pad.rejected), static_cast<unsigned>(pad.spikes),
                    static_cast<double>(coast.final_drift_deg));
                // Initialization and bias do not depend on the filter.
                CHECK(pad.init_time_s > 0.0F && pad.init_time_s <= 3.0F);
                CHECK(pad.bias_error_dps < 0.5F);
                if (length == 5U) {
                    CHECK(pad.final_drift_deg < 2.0F);
                    CHECK(pad.max_drift_deg < 2.0F);
                }
            }
        }
    }
}

}  // namespace

int main()
{
    test_zero_before_init();
    test_tilt_init_orientations();
    test_bias_estimation();
    test_constant_rate();
    test_median_passes_fast_rotation();
    test_gaps();
    test_launch_window();
    test_pad_tilt_correction();
    test_pure_integration_in_flight();
    test_landed_hold();
    test_gyro_saturation();
    test_helpers();
    test_bench_interference();
    if (failures != 0) {
        std::printf("%d failure(s)\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("attitude_estimator_test: all tests passed\n");
    return EXIT_SUCCESS;
}
