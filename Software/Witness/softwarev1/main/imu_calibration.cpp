#include "imu_calibration.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace imu_calibration {

void Accumulator::reset()
{
    count_ = 0;
    gyro_stored_ = 0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        gyro_sum_[axis] = gyro_sq_[axis] = 0.0;
        low_sum_[axis] = low_sq_[axis] = high_sum_[axis] = 0.0;
    }
}

void Accumulator::gyro_median(float median[3], float spread[3])
{
    const std::size_t n = gyro_stored_;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (n == 0) {
            median[axis] = spread[axis] = 0.0F;
            continue;
        }
        float *v = gyro_samples_[axis];
        std::nth_element(v, v + n / 2, v + n);
        const float m = v[n / 2];
        for (std::size_t i = 0; i < n; ++i) v[i] = std::fabs(v[i] - m);
        std::nth_element(v, v + n / 2, v + n);
        median[axis] = m;
        spread[axis] = 1.4826F * v[n / 2];
    }
    gyro_stored_ = 0;  // samples were overwritten
}

void Accumulator::add(const float gyro_mdps[3], const float low_g_mg[3],
                      const float high_g_mg[3])
{
    for (std::size_t axis = 0; axis < 3; ++axis) {
        gyro_sum_[axis] += gyro_mdps[axis];
        gyro_sq_[axis] += static_cast<double>(gyro_mdps[axis]) * gyro_mdps[axis];
        low_sum_[axis] += low_g_mg[axis];
        low_sq_[axis] += static_cast<double>(low_g_mg[axis]) * low_g_mg[axis];
        high_sum_[axis] += high_g_mg[axis];
    }
    if ((count_ % kGyroStride) == 0U && gyro_stored_ < kMaxGyroSamples) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            gyro_samples_[axis][gyro_stored_] = gyro_mdps[axis];
        }
        ++gyro_stored_;
    }
    ++count_;
}

void Accumulator::mean(const double sum[3], float out[3]) const
{
    for (std::size_t axis = 0; axis < 3; ++axis) {
        out[axis] = count_ == 0 ? 0.0F : static_cast<float>(sum[axis] / count_);
    }
}

void Accumulator::sd(const double sum[3], const double sq[3], float out[3]) const
{
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (count_ < 2) {
            out[axis] = 0.0F;
            continue;
        }
        const double m = sum[axis] / count_;
        const double variance = sq[axis] / count_ - m * m;
        out[axis] = static_cast<float>(std::sqrt(variance > 0.0 ? variance : 0.0));
    }
}

const char *status_name(const Status status)
{
    switch (status) {
    case Status::Ok: return "ok";
    case Status::NotStill: return "board moved (standard deviation too high)";
    case Status::BiasTooLarge: return "gyro mean too large to be a bias";
    case Status::NotSquare: return "no axis within limits of +/-1 g (set the board square on a face)";
    case Status::OutOfRange: return "solved offset or scale out of range";
    case Status::TooFewSamples: return "too few samples";
    }
    return "unknown";
}

const char *face_name(const std::size_t face)
{
    static const char *const kNames[SixFaceSolver::kFaces] = {
        "+X", "-X", "+Y", "-Y", "+Z", "-Z"};
    return face < SixFaceSolver::kFaces ? kNames[face] : "?";
}

void SixFaceSolver::reset()
{
    *this = SixFaceSolver{};
}

std::size_t SixFaceSolver::face_count() const
{
    std::size_t n = 0;
    for (const bool present : present_) n += present ? 1U : 0U;
    return n;
}

Status SixFaceSolver::add_face(const float low_g_mean_mg[3],
                               const float high_g_mean_mg[3],
                               const Limits &limits, std::size_t &face)
{
    std::size_t dominant = 0;
    for (std::size_t axis = 1; axis < 3; ++axis) {
        if (std::fabs(low_g_mean_mg[axis]) > std::fabs(low_g_mean_mg[dominant])) {
            dominant = axis;
        }
    }
    face = dominant * 2U + (low_g_mean_mg[dominant] < 0.0F ? 1U : 0U);
    if (std::fabs(low_g_mean_mg[dominant]) < limits.min_face_mg) {
        return Status::NotSquare;
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (axis != dominant && std::fabs(low_g_mean_mg[axis]) > limits.max_cross_mg) {
            return Status::NotSquare;
        }
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
        low_[face][axis] = low_g_mean_mg[axis];
        high_[face][axis] = high_g_mean_mg[axis];
    }
    present_[face] = true;
    return Status::Ok;
}

namespace {

// Corrected magnitude minus 1 g for each face; p = offset[3], scale[3].
void face_residuals(const float low[SixFaceSolver::kFaces][3], const double p[6],
                    double r[SixFaceSolver::kFaces])
{
    for (std::size_t face = 0; face < SixFaceSolver::kFaces; ++face) {
        double sum = 0.0;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const double v = (low[face][axis] - p[axis]) / p[3 + axis];
            sum += v * v;
        }
        r[face] = std::sqrt(sum) - 1000.0;
    }
}

// Solves a x = b (6x6) by Gaussian elimination with partial pivoting.
bool solve6(double a[6][6], double b[6], double x[6])
{
    for (std::size_t col = 0; col < 6; ++col) {
        std::size_t pivot = col;
        for (std::size_t row = col + 1; row < 6; ++row) {
            if (std::fabs(a[row][col]) > std::fabs(a[pivot][col])) pivot = row;
        }
        if (std::fabs(a[pivot][col]) < 1e-12) return false;
        if (pivot != col) {
            for (std::size_t k = 0; k < 6; ++k) std::swap(a[col][k], a[pivot][k]);
            std::swap(b[col], b[pivot]);
        }
        for (std::size_t row = col + 1; row < 6; ++row) {
            const double f = a[row][col] / a[col][col];
            for (std::size_t k = col; k < 6; ++k) a[row][k] -= f * a[col][k];
            b[row] -= f * b[col];
        }
    }
    for (std::size_t i = 6; i-- > 0;) {
        double sum = b[i];
        for (std::size_t k = i + 1; k < 6; ++k) sum -= a[i][k] * x[k];
        x[i] = sum / a[i][i];
    }
    return true;
}

}  // namespace

Status SixFaceSolver::solve(const Limits &limits, Calibration &calibration) const
{
    if (!complete()) {
        return Status::TooFewSamples;
    }
    // Initial guess from opposite faces, then Gauss-Newton on |corrected| = 1 g.
    double p[6]{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const float up = low_[axis * 2U][axis];
        const float down = low_[axis * 2U + 1U][axis];
        p[axis] = (up + down) / 2.0;
        p[3 + axis] = (up - down) / 2.0 / 1000.0;
        if (p[3 + axis] <= 0.0) return Status::OutOfRange;
    }
    for (int iteration = 0; iteration < 20; ++iteration) {
        double r[kFaces];
        face_residuals(low_, p, r);
        double a[6][6];
        for (std::size_t k = 0; k < 6; ++k) {
            double q[6];
            for (std::size_t i = 0; i < 6; ++i) q[i] = p[i];
            const double h = k < 3 ? 0.01 : 1e-5;
            q[k] += h;
            double rq[kFaces];
            face_residuals(low_, q, rq);
            for (std::size_t face = 0; face < kFaces; ++face) {
                a[face][k] = (rq[face] - r[face]) / h;
            }
        }
        double b[6];
        for (std::size_t face = 0; face < kFaces; ++face) b[face] = -r[face];
        double step[6];
        if (!solve6(a, b, step)) return Status::OutOfRange;
        double largest = 0.0;
        for (std::size_t i = 0; i < 6; ++i) {
            p[i] += step[i];
            const double size = std::fabs(step[i]) * (i < 3 ? 1.0 : 1000.0);
            if (size > largest) largest = size;
        }
        if (largest < 1e-4) break;
    }
    float offset[3]{};
    float scale[3]{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        offset[axis] = static_cast<float>(p[axis]);
        scale[axis] = static_cast<float>(p[3 + axis]);
        if (!std::isfinite(offset[axis]) || !std::isfinite(scale[axis]) ||
            std::fabs(offset[axis]) > limits.max_accel_offset_mg ||
            scale[axis] < limits.min_accel_scale ||
            scale[axis] > limits.max_accel_scale) {
            return Status::OutOfRange;
        }
    }
    // High-g scale and offset: least squares of the high-g reading against
    // the corrected low-g reading over the six faces, per axis.
    float high_offset[3]{};
    float high_scale[3]{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (std::size_t face = 0; face < kFaces; ++face) {
            const double x = (low_[face][axis] - offset[axis]) / scale[axis];
            const double y = high_[face][axis];
            sx += x; sy += y; sxx += x * x; sxy += x * y;
        }
        const double n = static_cast<double>(kFaces);
        const double var = sxx / n - (sx / n) * (sx / n);
        const double k = var > 0.0 ? (sxy / n - (sx / n) * (sy / n)) / var : 1.0;
        high_scale[axis] = static_cast<float>(k);
        high_offset[axis] = static_cast<float>(sy / n - k * sx / n);
        if (!std::isfinite(high_scale[axis]) || !std::isfinite(high_offset[axis]) ||
            std::fabs(high_offset[axis]) > limits.max_high_g_offset_mg ||
            high_scale[axis] < limits.min_high_g_scale ||
            high_scale[axis] > limits.max_high_g_scale) {
            return Status::OutOfRange;
        }
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
        calibration.accel_offset_mg[axis] = offset[axis];
        calibration.accel_scale[axis] = scale[axis];
        calibration.high_g_offset_mg[axis] = high_offset[axis];
        calibration.high_g_scale[axis] = high_scale[axis];
    }
    calibration.accel_valid = true;
    return Status::Ok;
}

Status solve_gyro(Accumulator &capture, const Limits &limits,
                  const std::uint32_t min_samples, Calibration &calibration)
{
    if (capture.count() < min_samples) {
        return Status::TooFewSamples;
    }
    float mean[3]{};
    float gyro_sd[3]{};
    float accel_sd[3]{};
    capture.low_g_sd(accel_sd);
    capture.gyro_median(mean, gyro_sd);
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (gyro_sd[axis] > limits.max_gyro_sd_mdps ||
            accel_sd[axis] > limits.max_accel_sd_mg) {
            return Status::NotStill;
        }
        if (std::fabs(mean[axis]) > limits.max_gyro_bias_mdps) {
            return Status::BiasTooLarge;
        }
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
        calibration.gyro_bias_mdps[axis] = mean[axis];
    }
    calibration.gyro_valid = true;
    return Status::Ok;
}

void apply(const Calibration &calibration, float gyro_mdps[3], float low_g_mg[3],
           float high_g_mg[3])
{
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (calibration.gyro_valid) {
            gyro_mdps[axis] -= calibration.gyro_bias_mdps[axis];
        }
        if (calibration.accel_valid) {
            low_g_mg[axis] = (low_g_mg[axis] - calibration.accel_offset_mg[axis]) /
                             calibration.accel_scale[axis];
            high_g_mg[axis] = (high_g_mg[axis] - calibration.high_g_offset_mg[axis]) /
                              calibration.high_g_scale[axis];
        }
    }
}

}  // namespace imu_calibration
