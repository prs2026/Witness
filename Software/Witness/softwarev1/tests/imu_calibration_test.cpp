#include "imu_calibration.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace imu_calibration;

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

bool near(const float a, const float b, const float tolerance)
{
    return std::fabs(a - b) <= tolerance;
}

// Deterministic pseudo-noise in [-1, 1].
float noise(unsigned &state)
{
    state = state * 1664525U + 1013904223U;
    return static_cast<float>((state >> 8) & 0xFFFFU) / 32767.5F - 1.0F;
}

void test_gyro_bias_with_spikes()
{
    Accumulator capture;
    unsigned seed = 1;
    const float bias[3] = {910.0F, -50.0F, -890.0F};
    for (int i = 0; i < 1920; ++i) {
        float gyro[3];
        for (int axis = 0; axis < 3; ++axis) {
            gyro[axis] = bias[axis] + 800.0F * noise(seed);
        }
        if (i % 97 == 0) gyro[2] = -121000.0F;  // isolated interference spike
        const float low[3] = {0.0F, 0.0F, 992.0F};
        const float high[3] = {0.0F, 0.0F, 1000.0F};
        capture.add(gyro, low, high);
    }
    Calibration calibration;
    const Status status = solve_gyro(capture, Limits{}, 960, calibration);
    CHECK(status == Status::Ok);
    CHECK(calibration.gyro_valid);
    for (int axis = 0; axis < 3; ++axis) {
        CHECK(near(calibration.gyro_bias_mdps[axis], bias[axis], 60.0F));
    }
}

void test_gyro_rejects_motion()
{
    Accumulator capture;
    for (int i = 0; i < 1920; ++i) {
        const float gyro[3] = {20000.0F * std::sin(i * 0.01F), 0.0F, 0.0F};
        const float low[3] = {0.0F, 0.0F, 1000.0F};
        const float high[3] = {0.0F, 0.0F, 1000.0F};
        capture.add(gyro, low, high);
    }
    Calibration calibration;
    CHECK(solve_gyro(capture, Limits{}, 960, calibration) == Status::NotStill);
    CHECK(!calibration.gyro_valid);
}

void test_six_faces()
{
    const float offset[3] = {12.0F, -25.0F, 40.0F};
    const float scale[3] = {0.985F, 1.012F, 0.998F};
    const float high_offset[3] = {300.0F, -150.0F, 80.0F};
    SixFaceSolver solver;
    // Faces in a scrambled order: -Z, +X, +Y, -X, +Z, -Y.
    const int order[6] = {5, 0, 2, 1, 4, 3};
    for (int k = 0; k < 6; ++k) {
        const int face = order[k];
        // Each face sits 10 degrees off square (|truth| stays 1 g), which
        // the plain (up - down) / 2 formula would read as a 1.5% scale error.
        const float tilt = 10.0F * 3.14159265F / 180.0F;
        float truth[3] = {0.0F, 0.0F, 0.0F};
        truth[face / 2] = ((face % 2 == 0) ? 1000.0F : -1000.0F) * std::cos(tilt);
        truth[(face / 2 + 1 + face % 2) % 3] = 1000.0F * std::sin(tilt);
        float low[3];
        float high[3];
        for (int axis = 0; axis < 3; ++axis) {
            low[axis] = truth[axis] * scale[axis] + offset[axis];
            high[axis] = truth[axis] + high_offset[axis];
        }
        std::size_t classified = 99;
        CHECK(solver.add_face(low, high, Limits{}, classified) == Status::Ok);
        CHECK(classified == static_cast<std::size_t>(face));
        CHECK(solver.complete() == (k == 5));
    }
    Calibration calibration;
    CHECK(solver.solve(Limits{}, calibration) == Status::Ok);
    CHECK(calibration.accel_valid);
    for (int axis = 0; axis < 3; ++axis) {
        CHECK(near(calibration.accel_offset_mg[axis], offset[axis], 0.5F));
        CHECK(near(calibration.accel_scale[axis], scale[axis], 0.0005F));
        CHECK(near(calibration.high_g_offset_mg[axis], high_offset[axis], 1.0F));
    }

    // Applying the result recovers the truth on a new orientation.
    float gyro[3] = {0.0F, 0.0F, 0.0F};
    float low[3];
    float high[3];
    const float truth[3] = {577.0F, -577.0F, 577.0F};
    for (int axis = 0; axis < 3; ++axis) {
        low[axis] = truth[axis] * scale[axis] + offset[axis];
        high[axis] = truth[axis] + high_offset[axis];
    }
    apply(calibration, gyro, low, high);
    for (int axis = 0; axis < 3; ++axis) {
        CHECK(near(low[axis], truth[axis], 0.5F));
        CHECK(near(high[axis], truth[axis], 1.0F));
    }
}

void test_face_rejections()
{
    SixFaceSolver solver;
    std::size_t face = 0;
    const float tilted[3] = {700.0F, 0.0F, 700.0F};  // 45 degrees
    const float high[3] = {0.0F, 0.0F, 0.0F};
    CHECK(solver.add_face(tilted, high, Limits{}, face) == Status::NotSquare);
    CHECK(solver.face_count() == 0);

    Calibration calibration;
    CHECK(solver.solve(Limits{}, calibration) == Status::TooFewSamples);

    // A wildly wrong scale is refused.
    for (int f = 0; f < 6; ++f) {
        float low[3] = {0.0F, 0.0F, 0.0F};
        low[f / 2] = (f % 2 == 0) ? 1300.0F : -1300.0F;
        CHECK(solver.add_face(low, high, Limits{}, face) == Status::Ok);
    }
    CHECK(solver.solve(Limits{}, calibration) == Status::OutOfRange);
    CHECK(!calibration.accel_valid);
}

void test_apply_without_calibration()
{
    Calibration calibration;
    float gyro[3] = {1.0F, 2.0F, 3.0F};
    float low[3] = {4.0F, 5.0F, 6.0F};
    float high[3] = {7.0F, 8.0F, 9.0F};
    apply(calibration, gyro, low, high);
    CHECK(gyro[0] == 1.0F && low[1] == 5.0F && high[2] == 9.0F);
}

}  // namespace

int main()
{
    test_gyro_bias_with_spikes();
    test_gyro_rejects_motion();
    test_six_faces();
    test_face_rejections();
    test_apply_without_calibration();
    if (failures == 0) {
        std::printf("imu_calibration_test: all passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("imu_calibration_test: %d failure(s)\n", failures);
    return EXIT_FAILURE;
}
