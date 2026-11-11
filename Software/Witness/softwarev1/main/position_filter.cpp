#include "position_filter.h"

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
    initialized_ = true;
    imu_ignored_ = false;
    if (!(dt_s > 0.0F) || dt_s > 0.1F) {
        return;  // gap: skip rather than integrate a bad interval
    }
    const float ax = acceleration_world_mps2[0];
    const float ay = acceleration_world_mps2[1];
    const float az = acceleration_world_mps2[2];
    const float specific_force = std::sqrt(ax * ax + ay * ay + az * az);
    still_ = phase == Phase::PadIdle &&
             std::fabs(specific_force - kGravity) < config_.still_band_g * kGravity;

    const float a[3] = {ax, ay, az - kGravity};
    const float net = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
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
        // P = F P F' + Q with F = [1 dt; 0 1].
        const float p00 = s.P00 + 2.0F * dt_s * s.P01 + dt2 * s.P11 + q00;
        const float p01 = s.P01 + dt_s * s.P11 + q01;
        const float p11 = s.P11 + q11;
        s.P00 = p00;
        s.P01 = p01;
        s.P11 = p11;
    }
}

void Filter::predict_without_imu(const float dt_s, const Phase phase)
{
    initialized_ = true;
    imu_ignored_ = true;
    still_ = false;
    (void)phase;
    if (!(dt_s > 0.0F) || dt_s > 0.1F) {
        return;
    }
    const float q = config_.no_imu_accel_noise_mps2 * config_.no_imu_accel_noise_mps2;
    const float dt2 = dt_s * dt_s;
    for (Axis &s : axes_) {
        s.p += s.v * dt_s;
        const float p00 = s.P00 + 2.0F * dt_s * s.P01 + dt2 * s.P11 + q * dt2 * dt_s / 3.0F;
        const float p01 = s.P01 + dt_s * s.P11 + q * dt2 / 2.0F;
        const float p11 = s.P11 + q * dt_s;
        s.P00 = p00;
        s.P01 = p01;
        s.P11 = p11;
    }
}

void Filter::symmetrize(Axis &axis)
{
    if (axis.P00 < 1e-6F) axis.P00 = 1e-6F;
    if (axis.P11 < 1e-6F) axis.P11 = 1e-6F;
    const float limit = std::sqrt(axis.P00 * axis.P11);
    if (axis.P01 > limit) axis.P01 = limit;
    if (axis.P01 < -limit) axis.P01 = -limit;
}

void Filter::begin_update_cycle()
{
    baro_innovation_ = Innovation{};
    for (Innovation &innovation : gps_innovation_) innovation = Innovation{};
}

bool Filter::update_position(Axis &axis, const float measurement, const float variance,
                             std::uint32_t &consecutive_rejects, Innovation *record)
{
    const float innovation = measurement - axis.p;
    const float s = axis.P00 + variance;
    if (record != nullptr) {
        record->value = innovation;
        record->variance = s;
        record->accepted = false;
    }
    if (innovation * innovation > config_.gate_sigma * config_.gate_sigma * s &&
        consecutive_rejects < config_.max_consecutive_rejects) {
        ++consecutive_rejects;
        return false;
    }
    if (record != nullptr) record->accepted = true;
    consecutive_rejects = 0;
    const float k0 = axis.P00 / s;
    const float k1 = axis.P01 / s;
    axis.p += k0 * innovation;
    axis.v += k1 * innovation;
    const float p00 = (1.0F - k0) * axis.P00;
    const float p01 = (1.0F - k0) * axis.P01;
    const float p11 = axis.P11 - k1 * axis.P01;
    axis.P00 = p00;
    axis.P01 = p01;
    axis.P11 = p11;
    symmetrize(axis);
    return true;
}

void Filter::update_velocity(Axis &axis, const float measurement, const float variance)
{
    const float innovation = measurement - axis.v;
    const float s = axis.P11 + variance;
    const float k0 = axis.P01 / s;
    const float k1 = axis.P11 / s;
    axis.p += k0 * innovation;
    axis.v += k1 * innovation;
    const float p00 = axis.P00 - k0 * axis.P01;
    const float p01 = (1.0F - k1) * axis.P01;
    const float p11 = (1.0F - k1) * axis.P11;
    axis.P00 = p00;
    axis.P01 = p01;
    axis.P11 = p11;
    symmetrize(axis);
}

void Filter::update_constraints(const Phase phase)
{
    if (phase == Phase::PadIdle && still_) {
        for (Axis &axis : axes_) {
            std::uint32_t unused = 0;
            update_position(axis, 0.0F, kPadPositionSigmaM * kPadPositionSigmaM, unused);
            update_velocity(axis, 0.0F, kPadVelocitySigmaMps * kPadVelocitySigmaMps);
        }
    } else if (phase == Phase::Landed) {
        for (Axis &axis : axes_) {
            update_velocity(axis, 0.0F, kLandedVelocitySigmaMps * kLandedVelocitySigmaMps);
        }
    }
}

bool Filter::update_baro(const float altitude_m, const float pressure_pa, const Phase phase)
{
    if (!std::isfinite(altitude_m) || !(pressure_pa >= kBaroMinPressurePa)) {
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
        if (current_mach < kBaroMachResume) baro_gated_ = false;
    } else if (current_mach >= kBaroMachCutoff) {
        baro_gated_ = true;
    }
    if (baro_gated_) {
        ++baro_gated_count_;
        return false;  // zero trust above Mach 0.8
    }

    const float sigma = baro_sigma_m(pressure_pa, altitude_msl_m());
    last_baro_sigma_m_ = sigma;
    if (!update_position(axes_[2], altitude_m - baro_reference_m_, sigma * sigma,
                         baro_consecutive_rejects_, &baro_innovation_)) {
        ++baro_rejects_total_;
        return false;
    }
    ++baro_updates_;
    return true;
}

void Filter::gps_sigmas(const std::uint8_t satellites, const float hdop, const float vdop,
                        float &horizontal_m, float &vertical_m) const
{
    const auto clamp = [this](float sigma) {
        return std::fmin(std::fmax(sigma, config_.gps_min_sigma_m), config_.gps_max_sigma_m);
    };
    if (std::isfinite(hdop) && hdop > 0.0F) {
        const float v = std::isfinite(vdop) && vdop > 0.0F ? vdop : hdop * config_.gps_vdop_per_hdop;
        horizontal_m = clamp(config_.gps_horizontal_uere_m * hdop);
        vertical_m = clamp(config_.gps_vertical_uere_m * v);
        return;
    }
    const float inflate = satellites >= config_.gps_good_satellites ? 1.0F : 2.0F;
    horizontal_m = config_.gps_horizontal_sigma_m * inflate;
    vertical_m = config_.gps_vertical_sigma_m * inflate;
}

bool Filter::update_gps(const double latitude_deg, const double longitude_deg,
                        const float altitude_m, const std::uint8_t satellites,
                        const Phase phase, const float hdop, const float vdop)
{
    if (!std::isfinite(latitude_deg) || !std::isfinite(longitude_deg) ||
        !std::isfinite(altitude_m)) {
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
        if (update_position(axes_[i], measurement[i], variance[i], gps_consecutive_rejects_[i],
                            &gps_innovation_[i])) {
            used = true;
        } else {
            ++gps_rejects_total_;
        }
    }
    if (used) ++gps_updates_;
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
