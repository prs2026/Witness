#include "heading_align.h"

#include <cmath>

namespace heading {

void Aligner::reset()
{
    const Config config = config_;
    const bool trusted = trusted_;   // the pad setting survives a filter reset
    *this = Aligner(config);
    trusted_ = trusted;
}

float pad_heading_yaw(const float axis_world[3], const float azimuth_rad, const float min_horizontal)
{
    const float e = axis_world[0], n = axis_world[1];
    if (!(std::sqrt(e * e + n * n) >= min_horizontal) || !std::isfinite(azimuth_rad)) return NAN;
    // Compass azimuth (clockwise from north) of the axis now; a rotation by
    // yaw counter-clockwise about +Z lowers it by yaw.
    const float now = std::atan2(e, n);
    return std::remainder(now - azimuth_rad, 2.0F * 3.14159265F);
}

bool pad_heading_axis(const std::uint8_t code, float axis_body[3])
{
    axis_body[0] = axis_body[1] = axis_body[2] = 0.0F;
    switch (code) {
        case 0: axis_body[1] = 1.0F; return true;
        case 1: axis_body[1] = -1.0F; return true;
        case 2: axis_body[2] = 1.0F; return true;
        case 3: axis_body[2] = -1.0F; return true;
        default: return false;
    }
}

float Aligner::inertial_speed_mps() const
{
    return std::sqrt(ve_ * ve_ + vn_ * vn_);
}

void Aligner::add_inertial(const float accel_east, const float accel_north, const float dt_s,
                           const bool flying)
{
    if (!flying) {
        ve_ = vn_ = 0.0F;   // launch starts from rest
        return;
    }
    if (!(dt_s > 0.0F) || dt_s > 0.1F) return;
    ve_ += accel_east * dt_s;
    vn_ += accel_north * dt_s;
}

bool Aligner::gps_velocity(const double t_s, float &ge, float &gn) const
{
    // Newest fix (just stored) against the newest one at least gps_window_s older.
    const std::size_t now = (head_ + kHistory - 1) % kHistory;
    for (std::size_t k = 1; k < count_; ++k) {
        const std::size_t i = (head_ + kHistory - 1 - k) % kHistory;
        const double dt = t_s - t_hist_[i];
        if (dt >= config_.gps_window_s) {
            if (dt > 3.0 * config_.gps_window_s) return false;   // gap in the fixes
            ge = static_cast<float>((e_hist_[now] - e_hist_[i]) / dt);
            gn = static_cast<float>((n_hist_[now] - n_hist_[i]) / dt);
            return true;
        }
    }
    return false;
}

void Aligner::rotate_state(const float angle)
{
    const float c = std::cos(angle), s = std::sin(angle);
    const auto rot = [c, s](float &e, float &n) {
        const float re = c * e - s * n;
        n = s * e + c * n;
        e = re;
    };
    rot(ve_, vn_);
    rot(anchor_ve_, anchor_vn_);
}

float Aligner::add_gps(const float east_m, const float north_m, const double t_s,
                       const float stress_mps2, const bool flying)
{
    t_hist_[head_] = t_s;
    e_hist_[head_] = east_m;
    n_hist_[head_] = north_m;
    head_ = (head_ + 1) % kHistory;
    if (count_ < kHistory) ++count_;
    if (!flying) {
        // At rest on the pad: GPS and inertial velocity are both zero, which
        // anchors the first window at launch, so the horizontal velocity
        // gained in the boost is compared as soon as the receiver settles
        // after burnout (before a sustainer, on a two-stage flight).
        anchored_ = true;
        anchor_t_ = t_s;
        anchor_ve_ = anchor_vn_ = anchor_ge_ = anchor_gn_ = 0.0F;
        return 0.0F;
    }
    float ge = 0.0F, gn = 0.0F;
    if (!gps_velocity(t_s, ge, gn)) {
        anchored_ = false;   // gap in the fixes: start over
        return 0.0F;
    }
    last_gps_speed_ = std::sqrt(ge * ge + gn * gn);
    if (stress_mps2 > config_.max_stress_mps2) {
        // The receiver lags under acceleration: no comparison now, but the
        // anchor is kept, so the velocity change through the burn is used
        // once it settles.
        return 0.0F;
    }
    if (!anchored_ || t_s - anchor_t_ > config_.max_window_s) {
        anchored_ = true;
        anchor_t_ = t_s;
        anchor_ve_ = ve_;
        anchor_vn_ = vn_;
        anchor_ge_ = ge;
        anchor_gn_ = gn;
        return 0.0F;
    }
    const float dge = ge - anchor_ge_, dgn = gn - anchor_gn_;
    const float die = ve_ - anchor_ve_, din = vn_ - anchor_vn_;
    const float gps_dv = std::sqrt(dge * dge + dgn * dgn);
    const float ins_dv = std::sqrt(die * die + din * din);
    if (gps_dv < config_.min_dv_mps || ins_dv < config_.min_dv_mps) return 0.0F;
    // Angle that rotates the inertial change onto the GPS change
    // (counter-clockwise positive, East/North axes).
    const float angle = std::atan2(die * dgn - din * dge, die * dge + din * dgn);
    float correction = std::fabs(angle) > config_.whole_above_rad ? angle : config_.gain * angle;
    if (trusted_) {
        correction = config_.gain * angle;
        if (correction > config_.trusted_max_step_rad) correction = config_.trusted_max_step_rad;
        if (correction < -config_.trusted_max_step_rad) correction = -config_.trusted_max_step_rad;
    }
    rotate_state(correction);
    // Next window starts here.
    anchor_t_ = t_s;
    anchor_ve_ = ve_;
    anchor_vn_ = vn_;
    anchor_ge_ = ge;
    anchor_gn_ = gn;
    total_ = std::remainder(total_ + correction, 2.0F * 3.14159265F);
    ++corrections_;
    return correction;
}

}  // namespace heading
