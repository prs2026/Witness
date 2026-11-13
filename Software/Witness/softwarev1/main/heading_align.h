#pragma once

// GPS heading alignment for the strapdown attitude.
//
// Witness has no magnetometer: on the pad the attitude is initialized from
// gravity alone (shortest-arc to +Z), so pitch and roll are known but the
// heading is arbitrary. The vertical channel does not care, but every
// horizontal component of the thrust is integrated in a frame rotated by that
// unknown heading, so the inertial horizontal velocity changes point the
// wrong way.
//
// A heading error rotates every horizontal velocity CHANGE the IMU measures.
// The aligner integrates the inertial horizontal velocity (world acceleration
// from the current attitude) and differentiates GPS positions over
// gps_window_s, takes an anchor of both, and once the GPS velocity has changed
// by at least min_dv_mps since the anchor (and the inertial one too), the
// angle from the inertial change to the GPS change is a heading-error sample;
// then it re-anchors. The first anchor is the pad (both at rest), and it is
// kept through high-stress periods, so the boost's horizontal velocity gain is
// compared once the receiver settles after burnout. Comparing changes, not absolute velocities, cancels the
// additive inertial velocity error accumulated through the burns and GPS
// outages (comparing absolute velocity biased the result by 10-15 deg on the
// eayf replays). Nothing is compared while the receiver is under
// acceleration stress (it lags then); a gap in the fixes or a window longer
// than max_window_s starts a new anchor.
//
// Each sample is applied as a correction: large errors (> whole_above_rad,
// a wrong pad heading can be anything up to 180 deg) whole, small ones with
// gain. The caller rotates the attitude about the vertical by it; the
// aligner rotates its own inertial velocity and anchor by the same angle.
//
// Observability: the heading is only seen through horizontal velocity
// changes. A near-vertical rocket has little of that until it tilts, so the
// heading is typically found after the burns, which still matters for the
// rest of the flight (descent drift, any later burn).
//
// No ESP-IDF dependencies (host-tested in tests/heading_align_test.cpp).

#include <cstddef>
#include <cstdint>

namespace heading {

struct Config {
    float min_dv_mps = 5.0F;           // horizontal velocity change needed for a sample
    float max_window_s = 40.0F;        // re-anchor if it takes longer than this
    float max_stress_mps2 = 60.0F;     // ~6 g: no comparison in burns (receiver lag); coasting drag is fine
    float gps_window_s = 1.0F;         // GPS velocity = position change over this
    float gain = 0.5F;                 // fraction of a small error applied
    float whole_above_rad = 0.35F;     // ~20 deg: a larger error is applied whole
    // Once the heading was set on the pad (set_trusted), corrections are
    // never taken whole and are limited to this per sample: an early, noisy
    // GPS sample must not undo a known heading.
    float trusted_max_step_rad = 0.087F;   // 5 deg
};

// Pad heading: the rotation about world +Z (counter-clockwise seen from
// above, as Estimator::rotate_heading takes it) that makes a body axis,
// given in world coordinates (axis_world = q * axis_body), point to
// azimuth_rad (clockwise from north). NaN if that axis is too close to
// vertical for a heading (its horizontal part below min_horizontal).
float pad_heading_yaw(const float axis_world[3], float azimuth_rad, float min_horizontal = 0.5F);

// Body axis codes of IRIS_COMMAND_PAD_HEADING: 0 +Y, 1 -Y, 2 +Z, 3 -Z.
// Writes the unit vector; false for an unknown code.
bool pad_heading_axis(std::uint8_t code, float axis_body[3]);

class Aligner {
public:
    explicit Aligner(const Config &config = Config{}) : config_(config) {}
    void reset();
    // World-frame horizontal specific force (m/s^2, from the current
    // attitude) over dt; integrated only while flying (after launch).
    void add_inertial(float accel_east, float accel_north, float dt_s, bool flying);
    // A GPS fix in the local frame (m) at t_s. stress_mps2: the position
    // filter's decaying peak net acceleration. Returns the heading correction
    // (rad, rotation about +Up, counter-clockwise seen from above) to apply
    // to the attitude now, or 0.
    float add_gps(float east_m, float north_m, double t_s, float stress_mps2, bool flying);
    float total_correction_rad() const { return total_; }
    // The heading was set on the pad: only small, limited corrections.
    void set_trusted(bool trusted) { trusted_ = trusted; }
    bool trusted() const { return trusted_; }
    std::uint32_t corrections() const { return corrections_; }
    float inertial_speed_mps() const;
    float last_gps_speed_mps() const { return last_gps_speed_; }

private:
    static constexpr std::size_t kHistory = 128;   // > gps_window_s at 25 Hz
    bool gps_velocity(double t_s, float &ge, float &gn) const;
    void rotate_state(float angle);
    Config config_;
    float ve_ = 0.0F, vn_ = 0.0F;                  // inertial horizontal velocity
    double t_hist_[kHistory]{};
    float e_hist_[kHistory]{};
    float n_hist_[kHistory]{};
    std::size_t head_ = 0, count_ = 0;
    bool anchored_ = false;
    double anchor_t_ = 0.0;
    float anchor_ve_ = 0.0F, anchor_vn_ = 0.0F;    // inertial at the anchor
    float anchor_ge_ = 0.0F, anchor_gn_ = 0.0F;    // GPS at the anchor
    float total_ = 0.0F;
    bool trusted_ = false;
    std::uint32_t corrections_ = 0;
    float last_gps_speed_ = 0.0F;
};

}  // namespace heading
