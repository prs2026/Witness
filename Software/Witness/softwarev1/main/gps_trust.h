#pragma once
// GPS trust ramp for the position filter (2026-11-18). Pure logic, no ESP-IDF;
// host test tests/gps_trust_test.cpp.
//
// Why: in HITL run 20261118-214918 the receiver came back after a 25 s outage
// with 4-5 satellites and positions ~11 s behind truth. The dead-reckoned
// sigma had grown to ~1 km, so the 5-sigma gate let km-off fixes in, and the
// 25-reject resync forced the rest in at full weight (-5.8 km, then +277 m/s).
//
// How: each axis keeps a trust weight w in [floor, 1]; the GPS variance is
// divided by w, so a distrusted receiver only nudges the filter. Every
// window_s the GPS stream is checked against the filter:
//   - velocity: the GPS-implied mean velocity over the window against the
//     filter's mean velocity. Agreement ramps w up over fast_ramp_s.
//   - acceleration: the change of that velocity from the previous window
//     against the filter's (IMU-driven) change. If only this agrees (the
//     filter itself is off in velocity but the receiver tracks the motion),
//     w ramps up slowly over slow_ramp_s, so a really-off filter is still
//     pulled back by a healthy receiver.
//   - neither: w drops by drop_factor.
// Fewer than good_satellites caps w at low_satellite_cap; a gap between fixes
// longer than gap_s restarts w at restart_weight. The filter (not this file)
// gates against a capped prior sigma and limits each correction's size (slew).

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace position {

struct GpsTrustConfig {
    float window_s = 1.0F;              // consistency check baseline
    float gap_s = 2.0F;                 // fix gap that restarts the trust
    float restart_weight = 0.05F;
    float floor_weight = 0.02F;
    float fast_ramp_s = 2.5F;           // velocity-consistent: floor -> 1
    float slow_ramp_s = 20.0F;          // only acceleration-consistent (~9 s to 0.5)
    float drop_factor = 0.25F;          // inconsistent window: w *= this
    float velocity_tol_mps = 10.0F;     // + velocity_tol_fraction * speed
    float velocity_tol_fraction = 0.05F;
    float filter_velocity_sigma_cap_mps = 15.0F;   // 3 sigma of this is added
    float accel_tol_mps = 15.0F;        // velocity-change mismatch per window
    std::uint8_t good_satellites = 6;
    float low_satellite_cap = 0.1F;
};

class GpsTrust {
public:
    // One fix on one axis. t_s: filter clock; gps_m: GPS position on this
    // axis; filter_v: filter velocity now; filter_v_sigma: its sigma; speed:
    // filter speed (for the relative tolerance). Returns the weight to use.
    float on_fix(double t_s, float gps_m, float filter_v, float filter_v_sigma, float speed,
                 std::uint8_t satellites, const GpsTrustConfig &c)
    {
        if (!have_fix_ || t_s < last_fix_t_ || t_s - last_fix_t_ > c.gap_s) {
            weight_ = c.restart_weight;
            restart(t_s, gps_m, filter_v);
            restarts_ += have_fix_ ? 1U : 0U;
        } else if (t_s - anchor_t_ >= c.window_s) {
            evaluate(t_s, gps_m, filter_v, filter_v_sigma, speed, c);
        }
        have_fix_ = true;
        last_fix_t_ = t_s;
        const float cap = satellites >= c.good_satellites ? 1.0F : c.low_satellite_cap;
        weight_ = std::clamp(weight_, c.floor_weight, cap);
        return weight_;
    }

    float weight() const { return weight_; }
    // Last window's |GPS - filter| mean-velocity difference (m/s), NaN before one.
    float last_velocity_error() const { return last_velocity_error_; }
    std::uint32_t restarts() const { return restarts_; }

private:
    void restart(double t_s, float gps_m, float filter_v)
    {
        anchor_t_ = t_s;
        anchor_gps_ = gps_m;
        anchor_v_ = filter_v;
        have_prev_ = false;
    }

    void evaluate(double t_s, float gps_m, float filter_v, float filter_v_sigma, float speed,
                  const GpsTrustConfig &c)
    {
        const float dt = static_cast<float>(t_s - anchor_t_);
        const float v_gps = (gps_m - anchor_gps_) / dt;
        const float v_filter = 0.5F * (anchor_v_ + filter_v);   // mean over the window
        const float velocity_error = std::fabs(v_gps - v_filter);
        const float sigma = std::fmin(std::isfinite(filter_v_sigma) ? filter_v_sigma : 0.0F,
                                      c.filter_velocity_sigma_cap_mps);
        const float velocity_tol = c.velocity_tol_mps + c.velocity_tol_fraction * speed +
                                   3.0F * sigma;
        bool accel_ok = false;
        if (have_prev_) {
            const float accel_error = std::fabs((v_gps - prev_v_gps_) - (v_filter - prev_v_filter_));
            accel_ok = accel_error <= c.accel_tol_mps * std::fmax(1.0F, dt / c.window_s);
        }
        if (std::isfinite(velocity_error) && velocity_error <= velocity_tol) {
            weight_ += dt / c.fast_ramp_s;
        } else if (accel_ok) {
            weight_ += dt / c.slow_ramp_s;
        } else {
            weight_ *= c.drop_factor;
        }
        last_velocity_error_ = velocity_error;
        prev_v_gps_ = v_gps;
        prev_v_filter_ = v_filter;
        have_prev_ = true;
        anchor_t_ = t_s;
        anchor_gps_ = gps_m;
        anchor_v_ = filter_v;
    }

    bool have_fix_ = false;
    double last_fix_t_ = 0.0;
    double anchor_t_ = 0.0;
    float anchor_gps_ = 0.0F;
    float anchor_v_ = 0.0F;
    bool have_prev_ = false;
    float prev_v_gps_ = 0.0F;
    float prev_v_filter_ = 0.0F;
    float weight_ = 0.05F;
    float last_velocity_error_ = std::nanf("");
    std::uint32_t restarts_ = 0;
};

}  // namespace position
