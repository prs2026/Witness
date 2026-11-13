#include "attitude_estimator.h"
#include "heading_align.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>

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

constexpr float kPi = 3.14159265F;
constexpr float kDeg = kPi / 180.0F;

// A vehicle accelerating horizontally along a true course; the inertial
// frame is rotated by `error` (what a wrong pad heading does). Each
// correction the aligner returns is applied to the frame, as sensors.cpp
// rotates the attitude. Returns the remaining heading error (rad).
float fly(const float error, const float course, const float stress, int &corrections,
          const float accel = 5.0F, const float seconds = 30.0F, const float ins_offset_north = 0.0F)
{
    heading::Aligner aligner;
    // An additive inertial velocity error (left over from burns): one step
    // of acceleration that GPS never saw.
    aligner.add_inertial(0.0F, ins_offset_north / 0.05F, 0.05F, true);
    float frame_error = error;         // inertial frame = truth rotated by this
    float ve = 0.0F, vn = 0.0F, e = 0.0F, n = 0.0F;
    const float ae = accel * std::sin(course), an = accel * std::cos(course);
    const float dt = 1.0F / 960.0F;
    double t = 0.0;
    int k = 0;
    while (t < seconds) {
        // Inertial: true acceleration seen in the rotated frame.
        const float c = std::cos(frame_error), s = std::sin(frame_error);
        aligner.add_inertial(c * ae - s * an, s * ae + c * an, dt, true);
        ve += ae * dt;
        vn += an * dt;
        e += ve * dt;
        n += vn * dt;
        t += dt;
        if (++k % 38 == 0) {   // ~25 Hz GPS
            const float corr = aligner.add_gps(e, n, t, stress, true);
            frame_error += corr;
        }
    }
    corrections = static_cast<int>(aligner.corrections());
    return std::remainder(frame_error, 2.0F * kPi);
}

void test_converges()
{
    for (const float err_deg : {-170.0F, -70.0F, -5.0F, 3.0F, 45.0F, 120.0F, 179.0F}) {
        int corrections = 0;
        const float left = fly(err_deg * kDeg, 30.0F * kDeg, 0.0F, corrections);
        std::printf("heading error %+6.1f deg -> %+.2f deg after %d corrections\n", err_deg,
                    left / kDeg, corrections);
        CHECK(std::fabs(left) < 1.0F * kDeg);
        CHECK(corrections >= 1);
    }
}

void test_no_correction_when_not_observable()
{
    int corrections = 0;
    // Too little change: 0.1 m/s^2 for 30 s is 3 m/s, below min_dv_mps
    fly(60.0F * kDeg, 0.0F, 0.0F, corrections, 0.1F);
    CHECK(corrections == 0);
    // Receiver under acceleration stress (lags): no comparison.
    fly(60.0F * kDeg, 0.0F, 80.0F, corrections);   // 8 g: in a burn
    CHECK(corrections == 0);
    // On the pad (not flying): nothing.
    heading::Aligner aligner;
    for (int i = 0; i < 1000; ++i) {
        aligner.add_inertial(1.0F, 0.0F, 0.01F, false);
        CHECK(aligner.add_gps(0.1F * i, 0.0F, 0.04 * i, 0.0F, false) == 0.0F);
    }
    CHECK(aligner.inertial_speed_mps() == 0.0F);
}

void test_additive_inertial_error_is_not_heading()
{
    // A 20 m/s inertial velocity offset with the heading right: comparing
    // velocity changes must not turn it into a heading error.
    int corrections = 0;
    const float left = fly(0.0F, 60.0F * kDeg, 0.0F, corrections, 5.0F, 30.0F, 20.0F);
    std::printf("20 m/s inertial offset, heading right: %+.2f deg after %d corrections\n",
                left / kDeg, corrections);
    CHECK(std::fabs(left) < 0.5F * kDeg);
    const float left2 = fly(90.0F * kDeg, 60.0F * kDeg, 0.0F, corrections, 5.0F, 30.0F, 20.0F);
    CHECK(std::fabs(left2) < 1.0F * kDeg);
}

void test_already_aligned()
{
    int corrections = 0;
    const float left = fly(0.0F, -100.0F * kDeg, 0.0F, corrections);
    CHECK(std::fabs(left) < 0.2F * kDeg);
}

void test_estimator_rotate_heading()
{
    // Initialize upright from gravity on the pad, then rotate the heading:
    // the tilt (pitch/roll) must not change, and body +X stays vertical.
    attitude::Estimator est;
    attitude::Step step;
    step.specific_force_g = {0.98F, 0.1F, -0.15F};   // slightly tilted on the pad
    for (int i = 0; i < 400; ++i) {
        step.timestamp_us = 10000ULL * static_cast<std::uint64_t>(i + 1);
        est.update(step);
    }
    CHECK(est.valid());
    const float tilt = est.tilt_from_vertical_deg();
    const attitude::Vector3 x = attitude::Estimator::rotate(est.quaternion(), {1.0F, 0.0F, 0.0F});
    est.rotate_heading(1.2F);
    CHECK(std::fabs(est.tilt_from_vertical_deg() - tilt) < 0.01F);
    const attitude::Vector3 x2 = attitude::Estimator::rotate(est.quaternion(), {1.0F, 0.0F, 0.0F});
    CHECK(std::fabs(x2.z - x.z) < 1e-5F);
    // The horizontal part of the nose turned by 1.2 rad about +Z.
    const float a1 = std::atan2(x.y, x.x), a2 = std::atan2(x2.y, x2.x);
    CHECK(std::fabs(std::remainder(a2 - a1 - 1.2F, 2.0F * kPi)) < 1e-3F);
    CHECK(std::fabs(est.heading_corrections_rad() - 1.2F) < 1e-6F);
}

void test_pad_heading()
{
    // On the rail: nose (+X) up, slightly tilted. After pad_heading_yaw and
    // rotate_heading, the chosen axis points to the requested azimuth.
    attitude::Estimator est;
    attitude::Step step;
    step.specific_force_g = {0.995F, 0.06F, -0.08F};
    for (int i = 0; i < 400; ++i) {
        step.timestamp_us = 10000ULL * static_cast<std::uint64_t>(i + 1);
        est.update(step);
    }
    CHECK(est.valid());
    for (std::uint8_t code = 0; code < 4; ++code) {
        for (const float az_deg : {0.0F, 37.5F, 270.0F, 359.9F}) {
            float body[3];
            CHECK(heading::pad_heading_axis(code, body));
            attitude::Vector3 w = attitude::Estimator::rotate(est.quaternion(), {body[0], body[1], body[2]});
            const float world[3] = {w.x, w.y, w.z};
            const float yaw = heading::pad_heading_yaw(world, az_deg * kDeg);
            CHECK(std::isfinite(yaw));
            const float tilt = est.tilt_from_vertical_deg();
            est.rotate_heading(yaw);
            w = attitude::Estimator::rotate(est.quaternion(), {body[0], body[1], body[2]});
            float az = std::atan2(w.x, w.y) / kDeg;
            if (az < 0.0F) az += 360.0F;
            CHECK(std::fabs(std::remainder(az - az_deg, 360.0F)) < 0.01F);
            CHECK(std::fabs(est.tilt_from_vertical_deg() - tilt) < 0.01F);   // tilt untouched
        }
    }
    float body[3];
    CHECK(!heading::pad_heading_axis(7, body));
    // The nose (near vertical) cannot carry a heading.
    const float up[3] = {0.05F, 0.02F, 0.998F};
    CHECK(!std::isfinite(heading::pad_heading_yaw(up, 1.0F)));
}

void test_trusted_heading_limits_corrections()
{
    // Heading set on the pad (trusted): a GPS window that disagrees by 60 deg
    // moves it only by the trusted step, never whole.
    heading::Aligner aligner;
    aligner.set_trusted(true);
    float ve = 0.0F, e = 0.0F;
    double t = 0.0;
    float first = 0.0F;
    for (int k = 1; k <= 960 * 6; ++k) {
        const float dt = 1.0F / 960.0F;
        ve += 5.0F * dt;            // truth: accelerating east
        e += ve * dt;
        t += dt;
        // inertial sees it 60 deg off (rotated counter-clockwise)
        aligner.add_inertial(5.0F * std::cos(60.0F * kDeg), 5.0F * std::sin(60.0F * kDeg), dt, true);
        if (k % 38 == 0) {
            const float c = aligner.add_gps(e, 0.0F, t, 0.0F, true);
            if (c != 0.0F && first == 0.0F) first = c;
        }
    }
    CHECK(first != 0.0F);
    CHECK(std::fabs(first) <= heading::Config{}.trusted_max_step_rad + 1e-6F);
    aligner.reset();
    CHECK(aligner.trusted());   // survives a filter reset
}

}  // namespace

int main()
{
    test_pad_heading();
    test_trusted_heading_limits_corrections();
    test_converges();
    test_no_correction_when_not_observable();
    test_already_aligned();
    test_additive_inertial_error_is_not_heading();
    test_estimator_rotate_heading();
    if (failures == 0) {
        std::printf("heading_align_test: all passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("heading_align_test: %d failure(s)\n", failures);
    return EXIT_FAILURE;
}
