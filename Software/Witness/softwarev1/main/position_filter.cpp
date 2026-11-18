#include "position_filter.h"

#include <algorithm>
#include <cmath>

namespace position {
namespace {

constexpr float kAirGasConstant = 287.05F;  // J/(kg K)
constexpr float kGamma = 1.4F;
constexpr double kPi = 3.14159265358979323846;

// Pad hold pseudo-measurement sigmas.
constexpr float kPadPositionSigmaM = 0.05F;
constexpr float kPadVelocitySigmaMps = 0.02F;
constexpr float kLandedVelocitySigmaMps = 0.1F;
// Pad reference tracking (per update, at 10-25 Hz: a few seconds).
constexpr float kReferenceAlpha = 0.02F;
// Gravity measured on the pad (mean |specific force| of the still samples).
constexpr float kGravityTauS = 5.0F;           // averaging time constant
constexpr float kGravityMinStillS = 1.0F;      // kGravity until this much was averaged
constexpr float kGravityClampFraction = 0.05F; // accepted: kGravity +- 5 %
constexpr float kGravityLeavePadS = 0.5F;      // continuous motion that ends the averaging
// A predict gap longer than kMaxPredictDtS is bridged (position advanced with
// the current velocity, covariance grown), up to kMaxGapBridgeS of it.
constexpr float kMaxPredictDtS = 0.1F;
constexpr float kMaxGapBridgeS = 10.0F;
// Health check bounds: beyond these the state is garbage, not a flight.
constexpr float kMaxPlausiblePositionM = 1.0e7F;
constexpr float kMaxPlausibleVelocityMps = 1.0e5F;
constexpr float kMaxPlausibleCovariance = 1.0e15F;
// Covariance after a divergence reset or a restore after a reset in flight:
// the position is only known to ~500 m and the velocity to ~100 m/s, so the
// barometer and GPS (gate_sigma x that) pull the state in at once.
constexpr float kRecoveredPositionVariance = 250000.0F;
constexpr float kRecoveredVelocityVariance = 10000.0F;

}  // namespace

void Filter::reset()
{
    const Config config = config_;
    *this = Filter(config);
}

float Filter::isa_temperature_k(float altitude_msl_m)
{
    if (altitude_msl_m < -500.0F) altitude_msl_m = -500.0F;
    if (altitude_msl_m < 11000.0F) return 288.15F - 0.0065F * altitude_msl_m;
    if (altitude_msl_m < 20000.0F) return 216.65F;
    if (altitude_msl_m < 32000.0F) return 216.65F + 0.001F * (altitude_msl_m - 20000.0F);
    if (altitude_msl_m < 47000.0F) return 228.65F + 0.0028F * (altitude_msl_m - 32000.0F);
    return 270.65F;
}

float Filter::speed_of_sound_mps(const float altitude_msl_m)
{
    return std::sqrt(kGamma * kAirGasConstant * isa_temperature_k(altitude_msl_m));
}

float Filter::baro_sigma_m(const float pressure_pa, const float altitude_msl_m) const
{
    const float density = pressure_pa / (kAirGasConstant * isa_temperature_k(altitude_msl_m));
    const float from_pressure = config_.baro_pressure_sigma_pa / (density * kGravity);
    return std::sqrt(config_.baro_min_sigma_m * config_.baro_min_sigma_m +
                     from_pressure * from_pressure);
}

void Filter::predict(const float acceleration_world_mps2[3], const float dt_s,
                     const Phase phase)
{
    const float ax = acceleration_world_mps2[0];
    const float ay = acceleration_world_mps2[1];
    const float az = acceleration_world_mps2[2];
    // A non-finite acceleration or dt must never reach the state (NaN would
    // stick in the covariance for good): drop the sample and count it.
    if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az) ||
        !std::isfinite(dt_s)) {
        ++rejected_inputs_;
        return;
    }
    initialized_ = true;
    imu_ignored_ = false;
    if (!(dt_s > 0.0F)) {
        return;
    }
    if (dt_s > kMaxPredictDtS) {
        // Gap (stall, overrun): do not integrate the IMU over an unknown
        // interval, but do not freeze either. Advance the position with the
        // current velocity and grow the covariance as without the IMU, so the
        // next baro/GPS update is judged against an honest uncertainty
        // (Kalman audit #5).
        ++predict_gaps_;
        propagate_without_acceleration(std::fmin(dt_s, kMaxGapBridgeS),
                                       config_.no_imu_accel_noise_mps2);
        check_health();
        return;
    }
    const float specific_force = std::sqrt(ax * ax + ay * ay + az * az);
    still_ = phase == Phase::PadIdle &&
             std::fabs(specific_force - kGravity) < config_.still_band_g * kGravity;

    track_pad_gravity(specific_force, dt_s, phase);
    const float a[3] = {ax, ay, az - gravity_mps2_};
    const float net = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    // Peak-hold with exponential decay (first order, cheap at 960 Hz).
    gps_stress_ -= gps_stress_ * std::fmin(dt_s / config_.gps_stress_tau_s, 1.0F);
    if (net > gps_stress_) gps_stress_ = net;
    const float sigma_a = config_.accel_noise_mps2 + config_.accel_noise_fraction * net;
    const float q = sigma_a * sigma_a;
    const float dt2 = dt_s * dt_s;
    const float q00 = q * dt2 * dt_s / 3.0F;
    const float q01 = q * dt2 / 2.0F;
    const float q11 = q * dt_s;
    for (int i = 0; i < 3; ++i) {
        Axis &s = axes_[i];
        s.p += s.v * dt_s + 0.5F * a[i] * dt2;
        s.v += a[i] * dt_s;
        // P = F P F' + Q with F = [1 dt 0; 0 1 0; 0 0 1].
        const float p00 = s.P00 + 2.0F * dt_s * s.P01 + dt2 * s.P11 + q00;
        const float p01 = s.P01 + dt_s * s.P11 + q01;
        const float p11 = s.P11 + q11;
        s.P00 = p00;
        s.P01 = p01;
        s.P11 = p11;
        s.P02 += dt_s * s.P12;
    }
    grow_baro_scale(dt_s);
    check_health();
}

void Filter::track_pad_gravity(const float specific_force, const float dt_s, const Phase phase)
{
    if (gravity_frozen_ || !config_.measure_pad_gravity) return;
    if (phase != Phase::PadIdle) {
        gravity_frozen_ = true;
        return;
    }
    if (!still_) {
        // A jolt on the pad is not a departure: freeze after kGravityLeavePadS
        // of continuous motion.
        gravity_moving_s_ += dt_s;
        if (gravity_moving_s_ >= kGravityLeavePadS) gravity_frozen_ = true;
        return;
    }
    gravity_moving_s_ = 0.0F;
    // Exponential average (time constant kGravityTauS, dt based). Until that
    // much time has been seen it is the plain mean, so it is not biased by
    // its start value.
    gravity_still_s_ += dt_s;
    const float alpha = std::fmin(dt_s / std::fmin(gravity_still_s_, kGravityTauS), 1.0F);
    gravity_mean_mps2_ += alpha * (specific_force - gravity_mean_mps2_);
    if (gravity_still_s_ < kGravityMinStillS) return;   // fallback: kGravity
    if (std::fabs(gravity_mean_mps2_ - kGravity) <= kGravityClampFraction * kGravity) {
        gravity_mps2_ = gravity_mean_mps2_;
        gravity_out_of_range_ = false;
    } else {
        gravity_mps2_ = kGravity;
        if (!gravity_out_of_range_) ++gravity_clamped_count_;
        gravity_out_of_range_ = true;
    }
}

void Filter::grow_baro_scale(const float dt_s)
{
    // k is 0 (and certain) while held on the pad, where the reference tracks
    // the pad; on leaving it, it gets its prior, then a random walk per
    // distance travelled vertically.
    if (!(config_.baro_scale_sigma > 0.0F)) return;
    Axis &up = axes_[2];
    if (still_) {
        baro_scale_started_ = false;
        return;
    }
    if (!baro_scale_started_) {
        baro_scale_started_ = true;
        up.b = 0.0F;
        up.P02 = up.P12 = 0.0F;
        up.P22 = config_.baro_scale_sigma * config_.baro_scale_sigma;
    }
    const float km = std::fabs(up.v) * dt_s * 1e-3F;
    up.P22 += config_.baro_scale_rate * config_.baro_scale_rate * km;
}

void Filter::propagate_without_acceleration(float dt_s, const float accel_noise_mps2)
{
    gps_stress_ -= gps_stress_ * std::fmin(dt_s / config_.gps_stress_tau_s, 1.0F);
    const float q = accel_noise_mps2 * accel_noise_mps2;
    const float dt2 = dt_s * dt_s;
    for (Axis &s : axes_) {
        s.p += s.v * dt_s;
        const float p00 = s.P00 + 2.0F * dt_s * s.P01 + dt2 * s.P11 + q * dt2 * dt_s / 3.0F;
        const float p01 = s.P01 + dt_s * s.P11 + q * dt2 / 2.0F;
        const float p11 = s.P11 + q * dt_s;
        s.P00 = p00;
        s.P01 = p01;
        s.P11 = p11;
        s.P02 += dt_s * s.P12;
    }
    grow_baro_scale(dt_s);
}

void Filter::predict_without_imu(const float dt_s, const Phase phase)
{
    (void)phase;
    if (!std::isfinite(dt_s)) {
        ++rejected_inputs_;
        return;
    }
    initialized_ = true;
    imu_ignored_ = true;
    still_ = false;
    gravity_frozen_ = true;
    if (!(dt_s > 0.0F)) {
        return;
    }
    if (dt_s > kMaxPredictDtS) ++predict_gaps_;   // bridged, see predict()
    propagate_without_acceleration(std::fmin(dt_s, kMaxGapBridgeS),
                                   config_.no_imu_accel_noise_mps2);
    check_health();
}

void Filter::predict_magnitude_only(const float specific_force_mps2, const float dt_s,
                                    const Phase phase)
{
    if (!std::isfinite(specific_force_mps2) || !std::isfinite(dt_s)) {
        ++rejected_inputs_;
        return;
    }
    initialized_ = true;
    imu_ignored_ = true;   // the IMU does not drive the position (log flag)
    if (!(dt_s > 0.0F)) {
        return;
    }
    const bool gap = dt_s > kMaxPredictDtS;
    if (gap) ++predict_gaps_;
    const float dt = std::fmin(dt_s, kMaxGapBridgeS);
    // Stillness and pad gravity need only |specific force|, so the pad hold
    // works without an attitude. Away from the pad (or not still) the
    // vertical acceleration is unknown: zero with the no-IMU process noise.
    still_ = phase == Phase::PadIdle &&
             std::fabs(specific_force_mps2 - kGravity) < config_.still_band_g * kGravity;
    if (!gap) track_pad_gravity(specific_force_mps2, dt, phase);
    propagate_without_acceleration(
        dt, still_ ? config_.accel_noise_mps2 : config_.no_imu_accel_noise_mps2);
    check_health();
}

void Filter::check_health()
{
    bool bad = false;
    for (const Axis &s : axes_) {
        const float values[9] = {s.p, s.v, s.b, s.P00, s.P01, s.P11, s.P02, s.P12, s.P22};
        for (const float x : values) {
            if (!std::isfinite(x)) bad = true;
        }
        if (std::fabs(s.p) > kMaxPlausiblePositionM || std::fabs(s.v) > kMaxPlausibleVelocityMps ||
            s.P00 > kMaxPlausibleCovariance || s.P11 > kMaxPlausibleCovariance) {
            bad = true;
        }
    }
    if (!std::isfinite(gps_stress_) || !std::isfinite(gravity_mps2_)) bad = true;
    if (bad) recover_from_divergence();
}

void Filter::recover_from_divergence()
{
    ++divergence_resets_;
    // Keep what the pad taught us (baro reference, GPS origin, gravity) and
    // the lifetime counters; forget the state. The large covariance lets the
    // next baro/GPS update (or, on the pad, the ungated pad hold) pull it in.
    for (Axis &s : axes_) {
        s = Axis{};
        s.P00 = kRecoveredPositionVariance;
        s.P11 = kRecoveredVelocityVariance;
    }
    gps_stress_ = 0.0F;
    if (!std::isfinite(gravity_mps2_) || !std::isfinite(gravity_mean_mps2_)) {
        gravity_mps2_ = kGravity;
        gravity_mean_mps2_ = kGravity;
    }
    baro_scale_started_ = false;
    baro_consecutive_rejects_ = 0;
    for (std::uint32_t &r : gps_consecutive_rejects_) r = 0;
}

void Filter::restore_after_reset(const PadReferences &refs)
{
    recover_from_divergence();
    --divergence_resets_;   // not a divergence: undo the helper's count
    initialized_ = true;
    imu_ignored_ = true;
    still_ = false;
    if (refs.baro_valid && std::isfinite(refs.baro_reference_m)) {
        baro_reference_m_ = refs.baro_reference_m;
        baro_reference_set_ = true;
    }
    if (refs.gps_valid && std::isfinite(refs.gps_origin_lat_deg) &&
        std::isfinite(refs.gps_origin_lon_deg) && std::isfinite(refs.gps_origin_alt_m) &&
        std::fabs(refs.gps_origin_lat_deg) <= 90.0 && std::fabs(refs.gps_origin_lon_deg) <= 180.0) {
        const double phi = refs.gps_origin_lat_deg * kPi / 180.0;
        meters_per_deg_lat_ = 111132.92 - 559.82 * std::cos(2 * phi) + 1.175 * std::cos(4 * phi);
        meters_per_deg_lon_ = 111412.84 * std::cos(phi) - 93.5 * std::cos(3 * phi);
        gps_origin_lat_ = refs.gps_origin_lat_deg;
        gps_origin_lon_ = refs.gps_origin_lon_deg;
        gps_origin_alt_ = refs.gps_origin_alt_m;
        gps_origin_set_ = true;
    }
    // Pad gravity; frozen, the vehicle left the pad.
    gravity_frozen_ = true;
    if (refs.gravity_valid && std::isfinite(refs.gravity_mps2) &&
        std::fabs(refs.gravity_mps2 - kGravity) <= kGravityClampFraction * kGravity) {
        gravity_mps2_ = refs.gravity_mps2;
        gravity_mean_mps2_ = refs.gravity_mps2;
        gravity_still_s_ = kGravityMinStillS;
    }
    if (refs.motion_valid && std::isfinite(refs.agl_m) &&
        std::isfinite(refs.vertical_velocity_mps)) {
        axes_[2].p = refs.agl_m;
        axes_[2].v = refs.vertical_velocity_mps;
    }
}

void Filter::export_pad_references(PadReferences &out) const
{
    out = PadReferences{};
    out.baro_valid = baro_reference_set_ && std::isfinite(baro_reference_m_);
    out.baro_reference_m = baro_reference_m_;
    out.gps_valid = gps_origin_set_;
    out.gps_origin_lat_deg = gps_origin_lat_;
    out.gps_origin_lon_deg = gps_origin_lon_;
    out.gps_origin_alt_m = gps_origin_alt_;
    out.gravity_valid = gravity_measured() && std::isfinite(gravity_mps2_);
    out.gravity_mps2 = gravity_mps2_;
    out.motion_valid = initialized_ && std::isfinite(axes_[2].p) && std::isfinite(axes_[2].v);
    out.agl_m = axes_[2].p;
    out.vertical_velocity_mps = axes_[2].v;
}

// ---- BaroRate ----

void BaroRate::push(const float mean)
{
    for (int i = 0; i + 1 < kBins; ++i) bins_[i] = bins_[i + 1];
    bins_[kBins - 1] = mean;
    if (filled_ < kBins) ++filled_;
    rate_valid_ = false;
    if (filled_ == kBins) {
        const float old3 = bins_[0] + bins_[1] + bins_[2];
        const float new3 = bins_[kBins - 3] + bins_[kBins - 2] + bins_[kBins - 1];
        if (std::isfinite(old3) && std::isfinite(new3)) {
            rate_ = (new3 - old3) / 3.0F / (0.1F * static_cast<float>(kBins - 3));
            rate_valid_ = true;
        }
    }
}

void BaroRate::finish_bin()
{
    push(bin_count_ > 0 ? bin_sum_ / static_cast<float>(bin_count_)
                        : std::numeric_limits<float>::quiet_NaN());
}

void BaroRate::add(const std::uint64_t t_us, const float altitude_m)
{
    if (!std::isfinite(altitude_m)) return;
    const std::uint64_t index = t_us / kBinUs;
    if (!bin_open_) {
        bin_open_ = true;
        bin_index_ = index;
    }
    if (index < bin_index_) return;   // time went backwards: ignore the sample
    if (index > bin_index_) {
        finish_bin();
        const std::uint64_t missing = index - bin_index_ - 1;
        for (std::uint64_t m = 0; m < std::min<std::uint64_t>(missing, kBins); ++m) {
            push(std::numeric_limits<float>::quiet_NaN());
        }
        bin_index_ = index;
        bin_sum_ = 0.0F;
        bin_count_ = 0;
    }
    bin_sum_ += altitude_m;
    ++bin_count_;
    last_add_us_ = t_us;
}

bool BaroRate::valid(const std::uint64_t now_us) const
{
    return rate_valid_ && now_us >= last_add_us_ && now_us - last_add_us_ <= kStaleUs;
}

void Filter::symmetrize(Axis &axis)
{
    if (axis.P00 < 1e-6F) axis.P00 = 1e-6F;
    if (axis.P11 < 1e-6F) axis.P11 = 1e-6F;
    const float limit = std::sqrt(axis.P00 * axis.P11);
    if (axis.P01 > limit) axis.P01 = limit;
    if (axis.P01 < -limit) axis.P01 = -limit;
    if (axis.P22 < 0.0F) axis.P22 = 0.0F;
    const float l02 = std::sqrt(axis.P00 * axis.P22);
    const float l12 = std::sqrt(axis.P11 * axis.P22);
    axis.P02 = std::fmax(-l02, std::fmin(l02, axis.P02));
    axis.P12 = std::fmax(-l12, std::fmin(l12, axis.P12));
}

void Filter::begin_update_cycle()
{
    if (cycles_since_baro_ < 0xFFFFFFFFU) ++cycles_since_baro_;
    if (cycles_since_gps_up_ < 0xFFFFFFFFU) ++cycles_since_gps_up_;
    baro_innovation_ = Innovation{};
    for (Innovation &innovation : gps_innovation_) innovation = Innovation{};
}

bool Filter::update_position(Axis &axis, const float measurement, const float variance,
                             std::uint32_t &consecutive_rejects, Innovation *record,
                             const float lag_s, const bool allow_resync)
{
    // Measurement of the position lag_s ago: h = p - lag_s v (extrapolation
    // form of a delayed-measurement update; lag_s = 0 is the plain position).
    return update_axis(axis, 1.0F, -lag_s, 0.0F, measurement, variance, &consecutive_rejects, record,
                       allow_resync);
}

bool Filter::update_axis(Axis &axis, const float hp, const float hv, const float hb,
                         const float measurement, const float variance,
                         std::uint32_t *consecutive_rejects, Innovation *record,
                         const bool allow_resync, const float predicted)
{
    // g = P H', S = H P H' + R, K = g / S, x += K (z - H x), P -= g g' / S.
    const float g0 = hp * axis.P00 + hv * axis.P01 + hb * axis.P02;
    const float g1 = hp * axis.P01 + hv * axis.P11 + hb * axis.P12;
    const float g2 = hp * axis.P02 + hv * axis.P12 + hb * axis.P22;
    const float model = std::isfinite(predicted) ? predicted : hp * axis.p + hv * axis.v + hb * axis.b;
    const float innovation = measurement - model;
    const float s = hp * g0 + hv * g1 + hb * g2 + variance;
    if (record != nullptr) {
        record->value = innovation;
        record->variance = s;
        record->accepted = false;
    }
    if (!std::isfinite(innovation) || !std::isfinite(s) || !(s > 0.0F)) {
        // A NaN innovation would pass the gate comparison (false) and poison
        // the axis for good: refuse it. A NaN already in the state is caught
        // by check_health() after the cycle.
        ++rejected_inputs_;
        return false;
    }
    if (consecutive_rejects != nullptr) {
        if (innovation * innovation > config_.gate_sigma * config_.gate_sigma * s &&
            (!allow_resync || *consecutive_rejects < config_.max_consecutive_rejects)) {
            ++*consecutive_rejects;
            return false;
        }
        *consecutive_rejects = 0;
    }
    if (record != nullptr) record->accepted = true;
    const float k0 = g0 / s;
    const float k1 = g1 / s;
    const float k2 = g2 / s;
    axis.p += k0 * innovation;
    axis.v += k1 * innovation;
    axis.b += k2 * innovation;
    axis.P00 -= k0 * g0;
    axis.P01 -= k0 * g1;
    axis.P11 -= k1 * g1;
    axis.P02 -= k0 * g2;
    axis.P12 -= k1 * g2;
    axis.P22 -= k2 * g2;
    symmetrize(axis);
    return true;
}

void Filter::update_velocity(Axis &axis, const float measurement, const float variance)
{
    (void)update_axis(axis, 0.0F, 1.0F, 0.0F, measurement, variance, nullptr, nullptr);
}

void Filter::update_constraints(const Phase phase)
{
    if (phase == Phase::PadIdle && still_) {
        Axis &up = axes_[2];
        up.b = 0.0F;
        up.P02 = up.P12 = up.P22 = 0.0F;
        for (Axis &axis : axes_) {
            // Ungated, like the velocity hold: on the pad the true position is
            // the origin, so a gate here could only keep a stale offset (a
            // position left over from a flight) from ever being pulled back.
            update_axis(axis, 1.0F, 0.0F, 0.0F, 0.0F, kPadPositionSigmaM * kPadPositionSigmaM,
                        nullptr, nullptr);
            update_velocity(axis, 0.0F, kPadVelocitySigmaMps * kPadVelocitySigmaMps);
        }
    } else if (phase == Phase::Landed) {
        for (Axis &axis : axes_) {
            update_velocity(axis, 0.0F, kLandedVelocitySigmaMps * kLandedVelocitySigmaMps);
        }
    }
    check_health();
}

bool Filter::update_baro(const float altitude_m, const float pressure_pa, const Phase phase)
{
    if (!std::isfinite(altitude_m) || !std::isfinite(pressure_pa) ||
        !(pressure_pa >= kBaroMinPressurePa)) {
        if (!std::isfinite(altitude_m) || !std::isfinite(pressure_pa)) ++rejected_inputs_;
        return false;  // out of the sensor's range or invalid
    }
    if (!baro_reference_set_) {
        // Normally set on the pad; if the first reading arrives in flight,
        // align it with the current estimate instead.
        baro_reference_m_ = altitude_m - axes_[2].p;
        baro_reference_set_ = true;
    }
    if (phase == Phase::PadIdle && still_) {
        baro_reference_m_ += kReferenceAlpha * (altitude_m - baro_reference_m_);
        return false;  // the pad hold pins the state; only track the reference
    }

    const float current_mach = mach();
    if (baro_gated_) {
        // Once gated, the barometer stays out for the rest of Boost and
        // Coast: the transonic static-pressure error does not end at burnout
        // (the vehicle decelerates back through Mach 1 in Coast). It returns
        // below kBaroMachResume in Descent (or any later phase).
        const bool ascent = phase == Phase::Boost || phase == Phase::Coast;
        if (current_mach < kBaroMachResume && !ascent) baro_gated_ = false;
    } else if (current_mach >= kBaroMachCutoff) {
        baro_gated_ = true;
    }
    if (baro_gated_) {
        ++baro_gated_count_;
        return false;  // zero trust above Mach 0.8
    }

    const float sigma = baro_sigma_m(pressure_pa, altitude_msl_m());
    last_baro_sigma_m_ = sigma;
    // Barometer height (1 + k) p: EKF row H = [1 + k, 0, p] (k = 0 and
    // H = [1, 0, 0] with the scale state off). Without a recent GPS altitude
    // k is not observable: it is then a "consider" parameter (Schmidt): held
    // fixed, decorrelated, and its uncertainty widens the barometer noise by
    // (p sigma_k)^2, so IMU drift found when the barometer resumes goes into
    // the altitude, not into k.
    Axis &up = axes_[2];
    const bool scaled = config_.baro_scale_sigma > 0.0F;
    const float k = scaled ? up.b : 0.0F;
    const bool observable = scaled && cycles_since_gps_up_ < kBaroHealthyCycles;
    float variance = sigma * sigma;
    if (scaled && !observable) {
        up.P02 = up.P12 = 0.0F;
        variance += up.p * up.p * up.P22;
    }
    if (!update_axis(up, 1.0F + k, 0.0F, observable ? up.p : 0.0F, altitude_m - baro_reference_m_,
                     variance, &baro_consecutive_rejects_, &baro_innovation_, true,
                     (1.0F + k) * up.p)) {
        ++baro_rejects_total_;
        return false;
    }
    ++baro_updates_;
    cycles_since_baro_ = 0;
    check_health();
    return true;
}

void Filter::gps_sigmas(const std::uint8_t satellites, const float hdop, const float vdop,
                        float &horizontal_m, float &vertical_m) const
{
    const auto clamp = [this](float sigma) {
        return std::fmin(std::fmax(sigma, config_.gps_min_sigma_m), config_.gps_max_sigma_m);
    };
    float h, v;
    if (std::isfinite(hdop) && hdop > 0.0F) {
        const float vd = std::isfinite(vdop) && vdop > 0.0F ? vdop : hdop * config_.gps_vdop_per_hdop;
        h = config_.gps_horizontal_uere_m * hdop;
        v = config_.gps_vertical_uere_m * vd;
    } else {
        const float inflate = satellites >= config_.gps_good_satellites ? 1.0F : 2.0F;
        h = config_.gps_horizontal_sigma_m * inflate;
        v = config_.gps_vertical_sigma_m * inflate;
    }
    // Receiver lag at speed and under (recent) acceleration.
    const float lag_speed = config_.gps_lag_speed_s * speed();
    const float lag_accel = config_.gps_lag_accel_s2 * gps_stress_;
    const float lag2 = lag_speed * lag_speed + lag_accel * lag_accel;
    horizontal_m = clamp(std::sqrt(h * h + lag2));
    vertical_m = clamp(std::sqrt(v * v + lag2));
}

bool Filter::update_gps(const double latitude_deg, const double longitude_deg,
                        const float altitude_m, const std::uint8_t satellites,
                        const Phase phase, const float hdop, const float vdop)
{
    if (!std::isfinite(latitude_deg) || !std::isfinite(longitude_deg) ||
        !std::isfinite(altitude_m) || std::fabs(latitude_deg) > 90.0 ||
        std::fabs(longitude_deg) > 180.0) {
        ++rejected_inputs_;
        return false;
    }
    const bool hold = phase == Phase::PadIdle && still_;
    if (!gps_origin_set_ || hold) {
        const double phi = latitude_deg * kPi / 180.0;
        meters_per_deg_lat_ = 111132.92 - 559.82 * std::cos(2 * phi) + 1.175 * std::cos(4 * phi);
        meters_per_deg_lon_ = 111412.84 * std::cos(phi) - 93.5 * std::cos(3 * phi);
        if (!gps_origin_set_) {
            // Origin = this fix minus the current estimate, so a fix that
            // first arrives in flight lines up with the integrated position.
            gps_origin_lat_ = latitude_deg - axes_[1].p / meters_per_deg_lat_;
            gps_origin_lon_ = longitude_deg - axes_[0].p / meters_per_deg_lon_;
            gps_origin_alt_ = altitude_m - axes_[2].p;
            gps_origin_set_ = true;
        } else {
            gps_origin_lat_ += kReferenceAlpha * (latitude_deg - gps_origin_lat_);
            gps_origin_lon_ += kReferenceAlpha * (longitude_deg - gps_origin_lon_);
            gps_origin_alt_ += kReferenceAlpha * (altitude_m - gps_origin_alt_);
        }
        if (hold) return false;
    }

    const float east = static_cast<float>((longitude_deg - gps_origin_lon_) * meters_per_deg_lon_);
    const float north = static_cast<float>((latitude_deg - gps_origin_lat_) * meters_per_deg_lat_);
    const float up = altitude_m - gps_origin_alt_;
    float h = 0.0F;
    float v = 0.0F;
    gps_sigmas(satellites, hdop, vdop, h, v);
    last_gps_sigma_h_ = h;
    last_gps_sigma_v_ = v;
    bool used = false;
    const float measurement[3] = {east, north, up};
    const float variance[3] = {h * h, h * h, v * v};
    for (int i = 0; i < 3; ++i) {
        // A GPS altitude stream that keeps failing the gate is forced in
        // (re-synchronization after an outage) only when no barometer was
        // accepted for kBaroHealthyCycles: a healthy subsonic barometer
        // outranks a receiver reporting km-off fixes (seen in HITL after a
        // bad reacquisition).
        const bool resync = i != 2 || cycles_since_baro_ >= kBaroHealthyCycles;
        if (update_position(axes_[i], measurement[i], variance[i], gps_consecutive_rejects_[i],
                            &gps_innovation_[i], config_.gps_latency_s, resync)) {
            used = true;
        } else {
            ++gps_rejects_total_;
        }
    }
    if (used) ++gps_updates_;
    if (gps_innovation_[2].accepted) cycles_since_gps_up_ = 0;
    check_health();
    return used;
}

void Filter::position(float out[3]) const
{
    for (int i = 0; i < 3; ++i) out[i] = axes_[i].p;
}

void Filter::velocity(float out[3]) const
{
    for (int i = 0; i < 3; ++i) out[i] = axes_[i].v;
}

float Filter::speed() const
{
    float sum = 0.0F;
    for (const Axis &axis : axes_) sum += axis.v * axis.v;
    return std::sqrt(sum);
}

float Filter::altitude_msl_m() const
{
    if (baro_reference_set_) return baro_reference_m_ + axes_[2].p;
    if (gps_origin_set_) return gps_origin_alt_ + axes_[2].p;
    return axes_[2].p;
}

float Filter::mach() const
{
    return speed() / speed_of_sound_mps(altitude_msl_m());
}

}  // namespace position
