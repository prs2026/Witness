#include "attitude_estimator.h"

#include <algorithm>
#include <cmath>

namespace attitude {
namespace {

constexpr float kRadiansToDegrees = 57.29577951308232F;
constexpr float kMadToSigma = 1.4826F;

float norm(const Vector3 &v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vector3 cross(const Vector3 &a, const Vector3 &b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

float dot(const Vector3 &a, const Vector3 &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vector3 scale(const Vector3 &v, const float s)
{
    return {v.x * s, v.y * s, v.z * s};
}

bool is_flight(const FlightPhase phase)
{
    return phase == FlightPhase::Boost || phase == FlightPhase::Coast ||
           phase == FlightPhase::Descent;
}

std::uint32_t sanitize_median_length(std::uint32_t length)
{
    if (length == 0U) length = 1U;
    if (length > Estimator::kMaxMedianLength) length = Estimator::kMaxMedianLength;
    if ((length % 2U) == 0U) length -= 1U;
    return length;
}

// Median of an odd number (<= 5) of values; sorts a small copy.
float small_median(const float *values, const std::size_t count)
{
    float sorted[Estimator::kMaxMedianLength]{};
    for (std::size_t i = 0; i < count; ++i) sorted[i] = values[i];
    std::sort(sorted, sorted + count);
    return sorted[count / 2U];
}

}  // namespace

Estimator::Estimator(const Config &config)
    : config_(config), median_length_(sanitize_median_length(config.gyro_median_length))
{
}

void Estimator::reset()
{
    q_ = Quaternion{1.0F, 0.0F, 0.0F, 0.0F};
    bias_ = Vector3{};
    valid_ = false;
    have_time_ = false;
    last_timestamp_us_ = 0;
    phase_known_ = false;
    phase_ = FlightPhase::PadIdle;
    flight_steps_ = 0;
    flight_time_s_ = 0.0;
    reset_window();
    clear_filter();
}

void Estimator::reset_window()
{
    for (int i = 0; i < 3; ++i) {
        window_accel_sum_[i] = 0.0;
        window_accel_sq_[i] = 0.0;
    }
    window_count_ = 0;
    window_start_us_ = 0;
}

void Estimator::clear_filter()
{
    history_count_ = 0;
    history_head_ = 0;
}

float Estimator::median_in_scratch(const std::size_t count)
{
    const std::size_t mid = count / 2U;
    std::nth_element(scratch_, scratch_ + mid, scratch_ + count);
    const float upper = scratch_[mid];
    if ((count % 2U) != 0U) return upper;
    const float lower = *std::max_element(scratch_, scratch_ + mid);
    return 0.5F * (lower + upper);
}

StepResult Estimator::update(const Step &step)
{
    StepResult result{};

    // Phase transitions.
    const FlightPhase previous = phase_;
    const bool first = !phase_known_;
    phase_ = step.phase;
    phase_known_ = true;
    if (is_flight(phase_) && (first || !is_flight(previous))) {
        flight_steps_ = 0;
        flight_time_s_ = 0.0;
    }
    if (phase_ == FlightPhase::Landed && (first || previous != FlightPhase::Landed)) {
        result.entered_landed = true;
    }
    if (!first && previous == FlightPhase::Landed && phase_ != FlightPhase::Landed) {
        clear_filter();  // Samples from before the hold are stale.
    }
    if (phase_ == FlightPhase::PadIdle && !first && previous != FlightPhase::PadIdle) {
        reset_window();
    }

    // Time step from the sample timestamps.
    std::uint32_t dt_us = 0;  // 0: do not integrate this sample.
    std::uint64_t previous_timestamp = step.timestamp_us;
    if (have_time_) {
        previous_timestamp = last_timestamp_us_;
        const std::int64_t delta = static_cast<std::int64_t>(step.timestamp_us) -
                                   static_cast<std::int64_t>(last_timestamp_us_);
        if (delta > 0) {
            const std::uint32_t delta_u32 =
                delta > 0xFFFFFFFFLL ? 0xFFFFFFFFU : static_cast<std::uint32_t>(delta);
            if (delta_u32 > max_dt_us_) max_dt_us_ = delta_u32;
        }
        if (delta <= 0 || delta > static_cast<std::int64_t>(config_.max_dt_us)) {
            ++gaps_;
            result.gap = true;
        } else {
            dt_us = static_cast<std::uint32_t>(delta);
            if (delta > static_cast<std::int64_t>(config_.late_dt_us)) ++late_steps_;
        }
    }
    have_time_ = true;
    last_timestamp_us_ = step.timestamp_us;

    if (phase_ == FlightPhase::Landed) {
        return result;  // Hold the final attitude.
    }

    const bool pad = phase_ == FlightPhase::PadIdle;
    const float accel_norm = norm(step.specific_force_g);
    const bool accel_ok = pad && !result.gap &&
                          accel_norm >= config_.stationary_min_g &&
                          accel_norm <= config_.stationary_max_g;
    if (pad && !accel_ok) reset_window();

    const std::uint32_t lag = median_length_ / 2U;
    if (accel_ok && window_count_ == 0U && lag == 0U) window_start_q_ = q_;

    // Running median, then integrate the delayed centre sample:
    // q <- q * exp((w - b) * dt), body-frame rate.
    history_gyro_[history_head_] = step.gyro_rad_s;
    history_dt_us_[history_head_] = dt_us;
    history_head_ = (history_head_ + 1U) % median_length_;
    if (history_count_ < median_length_) ++history_count_;
    if (history_count_ == median_length_) {
        Vector3 filtered{};
        float values[kMaxMedianLength]{};
        for (std::size_t i = 0; i < median_length_; ++i) values[i] = history_gyro_[i].x;
        filtered.x = small_median(values, median_length_);
        for (std::size_t i = 0; i < median_length_; ++i) values[i] = history_gyro_[i].y;
        filtered.y = small_median(values, median_length_);
        for (std::size_t i = 0; i < median_length_; ++i) values[i] = history_gyro_[i].z;
        filtered.z = small_median(values, median_length_);

        const std::size_t centre =
            (history_head_ + median_length_ - 1U - lag) % median_length_;
        const Vector3 &raw = history_gyro_[centre];
        if (std::fabs(raw.x - filtered.x) > config_.spike_threshold_rad_s ||
            std::fabs(raw.y - filtered.y) > config_.spike_threshold_rad_s ||
            std::fabs(raw.z - filtered.z) > config_.spike_threshold_rad_s) {
            ++spikes_rejected_;
        }
        const std::uint32_t centre_dt_us = history_dt_us_[centre];
        if (valid_ && centre_dt_us > 0U) {
            const float dt_s = static_cast<float>(centre_dt_us) * 1e-6F;
            const Vector3 rate{filtered.x - bias_.x, filtered.y - bias_.y,
                               filtered.z - bias_.z};
            q_ = normalized(multiply(q_, exp_rotation(scale(rate, dt_s))));
            ++steps_;
            if (is_flight(phase_)) {
                ++flight_steps_;
                flight_time_s_ += static_cast<double>(dt_s);
            }
        }
    }

    if (!accel_ok) {
        return result;  // Flight phases, or a non-stationary pad sample.
    }

    // Accumulate the stationary window (raw gyro; robust statistics later).
    if (window_count_ == 0U) window_start_us_ = previous_timestamp;
    window_gyro_[0][window_count_] = step.gyro_rad_s.x;
    window_gyro_[1][window_count_] = step.gyro_rad_s.y;
    window_gyro_[2][window_count_] = step.gyro_rad_s.z;
    const double a[3] = {step.specific_force_g.x, step.specific_force_g.y,
                         step.specific_force_g.z};
    for (int i = 0; i < 3; ++i) {
        window_accel_sum_[i] += a[i];
        window_accel_sq_[i] += a[i] * a[i];
    }
    ++window_count_;
    // The integrated attitude lags the newest sample by `lag` samples; this is
    // the attitude just before the window's first sample was integrated.
    if (lag > 0U && window_count_ == lag) window_start_q_ = q_;

    if (window_count_ >= kMaxWindowSamples ||
        (window_count_ >= config_.window_min_samples &&
         step.timestamp_us - window_start_us_ >= config_.window_us)) {
        finish_window(step.timestamp_us, result);
        reset_window();
    }
    return result;
}

void Estimator::finish_window(const std::uint64_t timestamp_us, StepResult &result)
{
    const std::size_t n = window_count_;
    const double count = static_cast<double>(n);

    // Accelerometer: mean and scatter.
    Vector3 mean_accel{};
    double variance_sum = 0.0;
    for (int i = 0; i < 3; ++i) {
        const double mean = window_accel_sum_[i] / count;
        const double variance = window_accel_sq_[i] / count - mean * mean;
        variance_sum += variance > 0.0 ? variance : 0.0;
        if (i == 0) mean_accel.x = static_cast<float>(mean);
        if (i == 1) mean_accel.y = static_cast<float>(mean);
        if (i == 2) mean_accel.z = static_cast<float>(mean);
    }
    if (std::sqrt(variance_sum) > static_cast<double>(config_.max_accel_sd_g)) {
        ++windows_rejected_accel_;
        result.window_rejected = true;
        return;
    }

    // Gyro: per-axis median and MAD; bounded outlier fraction.
    float median[3]{};
    for (int axis = 0; axis < 3; ++axis) {
        std::copy(window_gyro_[axis], window_gyro_[axis] + n, scratch_);
        median[axis] = median_in_scratch(n);
        for (std::size_t i = 0; i < n; ++i) {
            scratch_[i] = std::fabs(window_gyro_[axis][i] - median[axis]);
        }
        const float sigma = kMadToSigma * median_in_scratch(n);
        if (sigma > config_.max_gyro_sigma_rad_s) {
            ++windows_rejected_gyro_;
            result.window_rejected = true;
            return;
        }
    }
    std::size_t outliers = 0;
    for (std::size_t i = 0; i < n; ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            if (std::fabs(window_gyro_[axis][i] - median[axis]) >
                config_.gyro_outlier_rad_s) {
                ++outliers;
                break;
            }
        }
    }
    const Vector3 bias{median[0], median[1], median[2]};
    if (static_cast<float>(outliers) > config_.max_outlier_fraction * static_cast<float>(n) ||
        norm(bias) > config_.max_bias_rad_s) {
        ++windows_rejected_gyro_;
        result.window_rejected = true;
        return;
    }

    ++windows_accepted_;
    bias_ = bias;
    result.bias_updated = true;
    const float mean_norm = norm(mean_accel);
    if (!(mean_norm > 0.0F)) return;
    const Vector3 accel_unit = scale(mean_accel, 1.0F / mean_norm);
    if (!valid_) {
        q_ = shortest_arc(accel_unit, Vector3{0.0F, 0.0F, 1.0F});
        valid_ = true;
        result.initialized_now = true;
        return;
    }
    // The board was stationary: discard the gyro noise integrated during the
    // window, then pull the tilt toward the measured gravity direction.
    const float duration_s =
        static_cast<float>(timestamp_us - window_start_us_) * 1e-6F;
    float fraction = duration_s / config_.tilt_correction_tau_s;
    if (fraction > 1.0F) fraction = 1.0F;
    q_ = tilt_corrected(window_start_q_, accel_unit, fraction);
}

Quaternion Estimator::tilt_corrected(const Quaternion &q, const Vector3 &accel_unit,
                                     const float fraction)
{
    // Measured "up" in WORLD; rotate it toward +Z about a horizontal axis
    // (a_world x Z), which leaves heading unchanged to first order.
    const Vector3 up = rotate(q, accel_unit);
    const Vector3 axis{up.y, -up.x, 0.0F};
    const float s = norm(axis);
    if (s < 1e-7F) return q;
    const float angle = std::atan2(s, up.z);
    const Vector3 correction = scale(axis, angle * fraction / s);
    return normalized(multiply(exp_rotation(correction), q));
}

float Estimator::tilt_from_vertical_deg() const
{
    if (!valid_) return 0.0F;
    const Vector3 nose = rotate(q_, Vector3{1.0F, 0.0F, 0.0F});
    float z = nose.z;
    if (z > 1.0F) z = 1.0F;
    if (z < -1.0F) z = -1.0F;
    return std::acos(z) * kRadiansToDegrees;
}

Quaternion Estimator::multiply(const Quaternion &a, const Quaternion &b)
{
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

Quaternion Estimator::exp_rotation(const Vector3 &r)
{
    // Exact unit quaternion for rotation vector r: [cos(t/2), sin(t/2) r/t].
    // sin(t/2)/t uses its Taylor series near zero to stay accurate.
    const float t2 = r.x * r.x + r.y * r.y + r.z * r.z;
    const float t = std::sqrt(t2);
    float k = 0.0F;
    float w = 0.0F;
    if (t < 1e-3F) {
        k = 0.5F - t2 / 48.0F;
        w = 1.0F - t2 / 8.0F + t2 * t2 / 384.0F;
    } else {
        k = std::sin(0.5F * t) / t;
        w = std::cos(0.5F * t);
    }
    return {w, r.x * k, r.y * k, r.z * k};
}

Quaternion Estimator::shortest_arc(const Vector3 &from, const Vector3 &to)
{
    const float d = dot(from, to);
    if (d < -0.999999F) {
        // Antiparallel: 180 degrees about any axis perpendicular to `from`.
        Vector3 axis = cross(from, Vector3{1.0F, 0.0F, 0.0F});
        if (norm(axis) < 1e-3F) axis = cross(from, Vector3{0.0F, 1.0F, 0.0F});
        axis = scale(axis, 1.0F / norm(axis));
        return {0.0F, axis.x, axis.y, axis.z};
    }
    const Vector3 c = cross(from, to);
    return normalized(Quaternion{1.0F + d, c.x, c.y, c.z});
}

Vector3 Estimator::rotate(const Quaternion &q, const Vector3 &v)
{
    const Vector3 u{q.x, q.y, q.z};
    const Vector3 t = scale(cross(u, v), 2.0F);
    const Vector3 ut = cross(u, t);
    return {v.x + q.w * t.x + ut.x, v.y + q.w * t.y + ut.y, v.z + q.w * t.z + ut.z};
}

Quaternion Estimator::normalized(const Quaternion &q)
{
    const float n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!(n > 0.0F)) return Quaternion{1.0F, 0.0F, 0.0F, 0.0F};
    // Keep w >= 0 so the reported quaternion is a canonical representative.
    const float s = (q.w < 0.0F ? -1.0F : 1.0F) / n;
    return {q.w * s, q.x * s, q.y * s, q.z * s};
}

}  // namespace attitude
