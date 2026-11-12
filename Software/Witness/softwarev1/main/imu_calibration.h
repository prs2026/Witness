#pragma once

// Stored IMU calibration and the capture/solve logic behind it.
//
// Corrections, in engineering units:
//   gyro_mdps    = raw_mdps - gyro_bias_mdps
//   low_g_mg     = (raw_mg - accel_offset_mg) / accel_scale
//   high_g_mg    = (raw_mg - high_g_offset_mg) / high_g_scale
//
// Gyro bias: one capture with the board still in any orientation.
// Accelerometers: six captures, one per face (+X, -X, +Y, -Y, +Z, -Z up or
// down, in any order). Each capture is classified by its dominant axis; once
// all six are present, per-axis offset and scale are fitted (Gauss-Newton,
// starting from offset = (up + down) / 2, scale = (up - down) / 2 / 1 g) so
// every corrected capture has magnitude exactly 1 g. That makes the result
// independent of how square each face sits (a face 10 deg off reads only
// cos 10 deg = 0.985 g on its axis). The high-g scale and offset per axis are
// a least-squares fit of the high-g reading against the corrected low-g
// reading over the six faces (two at +/-1 g on that axis, four near 0): the
// high-g sensitivity tolerance is looser than the low-g's, and uncorrected it
// is the largest dead-reckoning error through a burn.
//
// This file has no ESP-IDF dependencies so it can be host-tested.

#include <cstddef>
#include <cstdint>

namespace imu_calibration {

struct Calibration {
    float gyro_bias_mdps[3]{};
    float accel_offset_mg[3]{};
    float accel_scale[3]{1.0F, 1.0F, 1.0F};
    float high_g_offset_mg[3]{};
    float high_g_scale[3]{1.0F, 1.0F, 1.0F};
    bool gyro_valid = false;
    bool accel_valid = false;
};

// Statistics for one capture: accelerometer mean and standard deviation, and
// gyro median and robust spread (1.4826 x median absolute deviation) from
// every kGyroStride-th sample, so isolated interference spikes do not bias the
// result (1024 x 4 covers a 4 s capture at 960 Hz).
class Accumulator {
public:
    static constexpr std::size_t kMaxGyroSamples = 1024;
    static constexpr std::uint32_t kGyroStride = 4;
    void reset();
    void add(const float gyro_mdps[3], const float low_g_mg[3],
             const float high_g_mg[3]);
    std::uint32_t count() const { return count_; }
    // Median and robust spread of the stored gyro samples (reorders them).
    void gyro_median(float median[3], float spread[3]);
    void low_g_mean(float out[3]) const { mean(low_sum_, out); }
    void high_g_mean(float out[3]) const { mean(high_sum_, out); }
    void gyro_sd(float out[3]) const { sd(gyro_sum_, gyro_sq_, out); }
    void low_g_sd(float out[3]) const { sd(low_sum_, low_sq_, out); }

private:
    void mean(const double sum[3], float out[3]) const;
    void sd(const double sum[3], const double sq[3], float out[3]) const;
    std::uint32_t count_ = 0;
    double gyro_sum_[3]{};
    double gyro_sq_[3]{};
    double low_sum_[3]{};
    double low_sq_[3]{};
    double high_sum_[3]{};
    float gyro_samples_[3][kMaxGyroSamples]{};
    std::size_t gyro_stored_ = 0;
};

// Limits a capture must meet; values in mdps / mg.
struct Limits {
    float max_gyro_sd_mdps = 3000.0F;    // robust spread; board must be still
    float max_accel_sd_mg = 20.0F;
    float max_gyro_bias_mdps = 20000.0F; // larger is a fault, not a bias
    float min_face_mg = 850.0F;          // dominant axis reading
    float max_cross_mg = 250.0F;         // the other two axes
    float max_accel_offset_mg = 300.0F;
    float min_accel_scale = 0.9F;
    float max_accel_scale = 1.1F;
    float max_high_g_offset_mg = 5000.0F;
    float min_high_g_scale = 0.85F;
    float max_high_g_scale = 1.15F;
};

enum class Status : std::uint8_t {
    Ok,
    NotStill,        // gyro or accel standard deviation too high
    BiasTooLarge,    // gyro mean beyond max_gyro_bias_mdps
    NotSquare,       // no clear dominant axis
    OutOfRange,      // solved offset/scale outside the limits
    TooFewSamples,
};

const char *status_name(Status status);

// Face index: 0 +X, 1 -X, 2 +Y, 3 -Y, 4 +Z, 5 -Z (the axis reading +/-1 g).
const char *face_name(std::size_t face);

class SixFaceSolver {
public:
    static constexpr std::size_t kFaces = 6;

    void reset();
    // Classifies a still capture and stores it. face is set even on failure
    // when a dominant axis exists.
    Status add_face(const float low_g_mean_mg[3], const float high_g_mean_mg[3],
                    const Limits &limits, std::size_t &face);
    bool has_face(std::size_t face) const { return present_[face]; }
    std::size_t face_count() const;
    bool complete() const { return face_count() == kFaces; }
    // Solves the accelerometer part of calibration (needs all six faces).
    Status solve(const Limits &limits, Calibration &calibration) const;

private:
    float low_[kFaces][3]{};
    float high_[kFaces][3]{};
    bool present_[kFaces]{};
};

// Checks a still capture and fills the gyro part of calibration.
Status solve_gyro(Accumulator &capture, const Limits &limits,
                  std::uint32_t min_samples, Calibration &calibration);

// Applies calibration in place (engineering units).
void apply(const Calibration &calibration, float gyro_mdps[3],
           float low_g_mg[3], float high_g_mg[3]);

}  // namespace imu_calibration
