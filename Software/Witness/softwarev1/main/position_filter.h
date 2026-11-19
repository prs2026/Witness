#pragma once

// Linear Kalman filter for position and velocity (ENU, metres, origin at the
// pad) from IMU acceleration, barometric altitude and GPS position.
//
// Model: constant-acceleration kinematics driven by the measured world-frame
// acceleration (the filtered accelerometer rotated by the attitude estimate,
// gravity removed). Process noise is white acceleration noise. The model,
// noise and all measurements are per axis, so the 6-state filter is exactly
// three independent [position, velocity] filters (E, N, U), which keeps the
// 960 Hz predict step cheap.
//
//   predict (every IMU sample):  p += v dt + a dt^2 / 2,  v += a dt
//                                P = F P F' + q [dt^3/3 dt^2/2; dt^2/2 dt]
//   update  (10/25 Hz):          baro -> U position; GPS -> E, N, U position
//
// Barometer trust:
//  - zero (update skipped) at or above kBaroMachCutoff, until the speed falls
//    below kBaroMachResume (transonic shock waves corrupt static pressure);
//    once gated in Boost or Coast it stays gated until Descent (the vehicle
//    passes back through Mach 1 in Coast, the error does not end at burnout);
//  - otherwise its altitude sigma follows the sensor's altitude accuracy,
//    sigma_h = sqrt(min^2 + (sigma_p / (rho g))^2), rho = p / (R T): the same
//    pressure error means more altitude error as the air thins (about 0.9 m
//    at sea level, 2.5 m at 10 km, 11 m at 20 km for sigma_p = 10 Pa);
//  - below the MS5607's 10 mbar range limit it is not used.
// Mach uses the filter's own speed and the ISA speed of sound at the
// estimated altitude.
//
// Pad: while the vehicle is still (|specific force| near 1 g and slow), zero
// position/velocity pseudo-measurements hold the state at the origin and the
// baro/GPS references track the pad. As soon as acceleration departs from
// 1 g the filter integrates freely, so the motion during the launch
// qualification window (before the state machine declares Boost) is kept.
// Landed: a zero-velocity pseudo-measurement stops drift.
//
// No ESP-IDF dependencies (host-tested in tests/position_filter_test.cpp).

#include <cmath>
#include "gps_trust.h"
#include <cstdint>
#include <limits>

namespace position {

inline constexpr float kGravity = 9.80665F;
inline constexpr float kBaroMachCutoff = 0.8F;
inline constexpr float kBaroMachResume = 0.75F;
inline constexpr float kBaroMinPressurePa = 1000.0F;  // MS5607: 10 mbar

struct Config {
    // Defaults tuned 2026-11-11 with HITL/kalman_mc.py (coordinate search on
    // 16 Monte Carlo replays of the eayf flight with realistic sensor,
    // attitude and atmosphere errors; objective: worst ascent altitude error,
    // descent kept under 5 %). See HITL/KALMAN_TUNING_REPORT.md.
    // White acceleration noise: base plus a fraction of |a| (attitude error
    // turns a share of the thrust into a wrong direction).
    float accel_noise_mps2 = 0.5F;
    float accel_noise_fraction = 0.05F;
    // Barometer: effective pressure error (noise, lag, model) and a floor.
    float baro_pressure_sigma_pa = 5.0F;
    float baro_min_sigma_m = 0.5F;
    // GPS position sigma. With dilution of precision (GGA HDOP, GSA VDOP) it is
    // the user range error times the DOP: sigma_h = uere_h * HDOP, sigma_v =
    // uere_v * VDOP (HDOP 1 -> 2.5 m, VDOP 1.5 -> 5 m, the old fixed values at
    // typical geometry), clamped to [min, max]. VDOP unknown but HDOP known
    // uses vdop = kVdopPerHdop * HDOP. Without any DOP: the fixed sigmas,
    // doubled below gps_good_satellites.
    float gps_horizontal_sigma_m = 2.5F;
    float gps_vertical_sigma_m = 5.0F;
    std::uint8_t gps_good_satellites = 6;
    float gps_horizontal_uere_m = 2.5F;
    float gps_vertical_uere_m = 3.3F;
    float gps_vdop_per_hdop = 1.6F;
    float gps_min_sigma_m = 1.0F;
    float gps_max_sigma_m = 100000.0F;   // no practical cap: a lagging fix under 40 g is km off
    // Receiver dynamics: under acceleration and at speed a GNSS receiver's own
    // navigation filter lags (the PX1120S airborne model is rated for 4 g;
    // HITL with RF GPS showed 50-80 m lag in boost and coast). Added in
    // quadrature to the DOP sigma: gps_lag_speed_s * speed and
    // gps_lag_accel_s2 * stress, where stress is the net (non-gravity)
    // acceleration, held as a peak that decays with gps_stress_tau_s because
    // the receiver keeps lagging for seconds after a burn ends.
    float gps_lag_speed_s = 0.1F;
    float gps_lag_accel_s2 = 5.0F;
    float gps_stress_tau_s = 2.0F;
    // Age of a GPS fix when Witness parses it (receiver solution + NMEA
    // output + UART). The fix is compared with the state that long ago,
    // p - latency * v, so at 1000 m/s a 0.12 s latency is not a 120 m error.
    float gps_latency_s = 0.0F;
    // Barometer scale (up axis): the filter converts pressure with the ISA,
    // and a real atmosphere a few K off standard reads a few % of the height
    // above the pad wrong (more on a hot or cold day). The barometer is
    // modelled as measuring (1 + k) p, k a state that starts at 0 with
    // baro_scale_sigma when the vehicle leaves the pad and wanders by
    // baro_scale_rate per sqrt(km) of vertical travel. GPS observes k
    // whenever both are available; without GPS the error still shrinks to
    // zero towards the pad's altitude, which a constant bias would not.
    // baro_scale_sigma = 0 disables it (the barometer measures p).
    float baro_scale_sigma = 0.03F;
    float baro_scale_rate = 0.005F;
    // Innovation gate (sigmas); a measurement stream rejected this many
    // times in a row is accepted anyway so the filter can re-synchronize.
    float gate_sigma = 5.0F;
    std::uint32_t max_consecutive_rejects = 25;
    // GPS in flight (Boost, Coast, Descent): trust ramp instead of the hard
    // gate + 25-reject resync (gps_trust.h). The gate compares the innovation
    // with the GPS sigma plus the filter's prior sigma capped at
    // gps_trust_sigma_cap_m, so a dead-reckoned km-wide sigma cannot make a
    // km-off fix look plausible. A gated fix is still used once the axis's
    // trust reaches gps_trust_large_accept_weight (the receiver has been
    // consistent), and every correction is slew-limited to
    // gps_trust_slew_floor_m + gps_trust_slew_mps * (time since the last fix),
    // so a really-off filter is pulled back over seconds, not in one step.
    // gps_trust_enabled = 0 restores the old behaviour in flight.
    std::uint32_t gps_trust_enabled = 1;
    float gps_trust_sigma_cap_m = 100.0F;
    float gps_trust_large_accept_weight = 0.5F;
    float gps_trust_slew_mps = 100.0F;
    float gps_trust_slew_floor_m = 1.0F;
    GpsTrustConfig gps_trust{};
    // Pad stillness: |specific force| within this of 1 g (referenced to
    // kGravity, not to the measured gravity).
    float still_band_g = 0.05F;
    // Subtract the gravity measured on the pad (mean |specific force| while
    // still, see Filter::gravity_mps2) instead of the fixed kGravity.
    bool measure_pad_gravity = true;
    // Process noise when predicting without IMU (e.g. under a parachute).
    float no_imu_accel_noise_mps2 = 10.0F;
};

enum class Phase : std::uint8_t {  // numerically FlightStateMachine::State
    PadIdle = 0, Boost = 1, Coast = 2, Descent = 3, Landed = 4,
};

// The pad references and the last motion estimate: what a reset in flight needs
// to keep AGL meaningful (persisted in RTC memory by flight_state_store, see
// export_pad_references / restore_after_reset).
struct PadReferences {
    bool baro_valid = false;
    float baro_reference_m = 0.0F;       // barometric altitude of the pad (ISA, MSL)
    bool gps_valid = false;
    double gps_origin_lat_deg = 0.0;
    double gps_origin_lon_deg = 0.0;
    float gps_origin_alt_m = 0.0F;       // GPS altitude of the pad (MSL)
    bool gravity_valid = false;
    float gravity_mps2 = kGravity;       // pad-measured gravity
    bool motion_valid = false;
    float agl_m = 0.0F;                  // last AGL (up position)
    float vertical_velocity_mps = 0.0F;  // last vertical velocity
};

// Raw barometric altitude rate (m/s), independent of the Kalman filter and of
// its pad hold: the average altitude of the newest 3 of 10 consecutive 100 ms
// bins minus that of the oldest 3, over their 0.7 s centre separation. Noise is
// ~0.4 m/s, so |rate| >= 20 m/s sustained is real vertical motion (used by the
// state machine to notice an airborne vehicle after a cold reset, when the pad
// hold would otherwise pin the filter to "still": a vehicle under a parachute
// reads 1 g). Not valid above ~Mach 0.8 (transonic static pressure).
class BaroRate {
public:
    void reset() { *this = BaroRate(); }
    // t_us: sample time; non-finite altitudes are ignored.
    void add(std::uint64_t t_us, float altitude_m);
    // True when 10 consecutive bins exist and the newest sample is at most
    // kStaleUs old at now_us.
    bool valid(std::uint64_t now_us) const;
    float rate_mps() const { return rate_; }

    static constexpr std::uint64_t kBinUs = 100'000ULL;
    static constexpr int kBins = 10;
    static constexpr std::uint64_t kStaleUs = 500'000ULL;
private:
    void finish_bin();
    void push(float mean);   // NaN = empty bin
    std::uint64_t bin_index_ = 0;
    bool bin_open_ = false;
    float bin_sum_ = 0.0F;
    std::uint32_t bin_count_ = 0;
    float bins_[kBins]{};
    int filled_ = 0;
    std::uint64_t last_add_us_ = 0;
    float rate_ = 0.0F;
    bool rate_valid_ = false;
};

// One measurement's innovation (measurement - prediction) and its variance
// S = P00 + R; NaN when that measurement was not offered this cycle.
struct Innovation {
    float value = std::numeric_limits<float>::quiet_NaN();
    float variance = std::numeric_limits<float>::quiet_NaN();
    bool accepted = false;
};

// [position, velocity, barometer bias]. The bias state is used on the up
// axis only (the barometer measures p + b); on east/north b stays 0 with zero
// variance, so their gains on it are zero and they are 2-state filters.
struct Axis {
    float p = 0.0F;
    float v = 0.0F;
    float b = 0.0F;
    float P00 = 1.0F, P01 = 0.0F, P11 = 1.0F;
    float P02 = 0.0F, P12 = 0.0F, P22 = 0.0F;
};

class Filter {
public:
    explicit Filter(const Config &config = Config{}) : config_(config) {}

    void reset();
    // acceleration_world_mps2: ENU specific force rotated to world, m/s^2
    // (gravity NOT removed; the filter subtracts it). dt in seconds.
    void predict(const float acceleration_world_mps2[3], float dt_s, Phase phase);
    // Predict without IMU (attitude lost after gyro saturation): zero net
    // acceleration and config.no_imu_accel_noise_mps2 of process noise, so
    // the barometer and GPS carry the estimate.
    void predict_without_imu(float dt_s, Phase phase);
    bool imu_ignored() const { return imu_ignored_; }
    // Predict with no attitude at all (never initialised, or lost) but with the
    // magnitude of the specific force (m/s^2, rotation invariant): the vertical
    // acceleration is unknown (zero is assumed, with no_imu_accel_noise), but
    // the pad stillness test and the pad gravity measurement only need the
    // magnitude, so on the pad the hold works exactly as with an attitude and
    // baro/GPS give AGL and vertical velocity in flight. A later switch to
    // predict() (attitude became valid) keeps the state.
    void predict_magnitude_only(float specific_force_mps2, float dt_s, Phase phase);
    // Reset in flight: re-establish the pad references and start from the last
    // persisted AGL / vertical velocity with a large covariance, so baro and
    // GPS pull the state in. Horizontal position/velocity start at 0 with a
    // large covariance (the GPS origin is the pad, the next fix sets them).
    void restore_after_reset(const PadReferences &refs);
    // Current references and motion (motion_valid only when initialized()).
    void export_pad_references(PadReferences &out) const;
    // Inputs rejected for being non-finite (acceleration, dt, baro, GPS).
    std::uint32_t rejected_inputs() const { return rejected_inputs_; }
    // Times the state or covariance became non-finite (or absurd) and the
    // filter was reset to a large covariance, keeping the pad references.
    std::uint32_t divergence_resets() const { return divergence_resets_; }
    // predict()/predict_without_imu() calls with dt > 0.1 s that were bridged
    // by propagating the current velocity with grown covariance.
    std::uint32_t predict_gaps() const { return predict_gaps_; }

    // Barometric altitude (any fixed reference, e.g. ISA from 1013.25 hPa)
    // and pressure. Returns true if it was used.
    bool update_baro(float altitude_m, float pressure_pa, Phase phase);
    // GPS fix (WGS-84 degrees, MSL metres). Returns true if it was used.
    // hdop/vdop: NaN when unknown (see Config). Returns true if it was used.
    bool update_gps(double latitude_deg, double longitude_deg, float altitude_m,
                    std::uint8_t satellites, Phase phase,
                    float hdop = std::numeric_limits<float>::quiet_NaN(),
                    float vdop = std::numeric_limits<float>::quiet_NaN());
    // Measurement sigmas (m) update_gps uses for these inputs.
    void gps_sigmas(std::uint8_t satellites, float hdop, float vdop,
                    float &horizontal_m, float &vertical_m) const;
    float last_gps_horizontal_sigma_m() const { return last_gps_sigma_h_; }
    // Decaying peak of the net acceleration (m/s^2) used for the GPS sigmas.
    float gps_dynamics_stress_mps2() const { return gps_stress_; }
    float last_gps_vertical_sigma_m() const { return last_gps_sigma_v_; }
    // Pad/landed pseudo-measurements; call at the update rate.
    void update_constraints(Phase phase);
    // Clears the per-cycle innovations; call before each update cycle.
    void begin_update_cycle();
    // Innovations from the current update cycle (for logging).
    const Innovation &baro_innovation() const { return baro_innovation_; }
    const Innovation &gps_innovation(int axis) const { return gps_innovation_[axis]; }

    const Axis &east() const { return axes_[0]; }
    const Axis &north() const { return axes_[1]; }
    const Axis &up() const { return axes_[2]; }
    void position(float out[3]) const;
    void velocity(float out[3]) const;
    float speed() const;
    float mach() const;
    // Estimated altitude above mean sea level (pad baro reference + up).
    float altitude_msl_m() const;
    bool baro_gated() const { return baro_gated_; }
    float last_baro_sigma_m() const { return last_baro_sigma_m_; }
    bool still() const { return still_; }
    bool initialized() const { return initialized_; }
    bool gps_origin_set() const { return gps_origin_set_; }
    // A GPS position in this filter's local frame (m from the GPS origin);
    // false before the origin is set.
    bool gps_to_local(double latitude_deg, double longitude_deg, float &east_m, float &north_m) const
    {
        if (!gps_origin_set_) return false;
        east_m = static_cast<float>((longitude_deg - gps_origin_lon_) * meters_per_deg_lon_);
        north_m = static_cast<float>((latitude_deg - gps_origin_lat_) * meters_per_deg_lat_);
        return true;
    }
    std::uint32_t baro_updates() const { return baro_updates_; }
    std::uint32_t gps_updates() const { return gps_updates_; }
    std::uint32_t baro_rejects() const { return baro_rejects_total_; }
    std::uint32_t gps_rejects() const { return gps_rejects_total_; }
    // GPS trust weight of an axis (0 = E, 1 = N, 2 = U), see gps_trust.h.
    float gps_trust_weight(int axis) const { return gps_trust_[axis].weight(); }
    std::uint32_t baro_gated_count() const { return baro_gated_count_; }

    // Exposed for tests and logging.
    static float isa_temperature_k(float altitude_msl_m);
    static float speed_of_sound_mps(float altitude_msl_m);
    float baro_sigma_m(float pressure_pa, float altitude_msl_m) const;

private:
    // Returns false if gated out.
    bool update_position(Axis &axis, float measurement, float variance,
                         std::uint32_t &consecutive_rejects,
                         Innovation *record = nullptr, float lag_s = 0.0F,
                         bool allow_resync = true);
    void update_velocity(Axis &axis, float measurement, float variance);
    // Scalar measurement z = hp p + hv v + hb b with variance R, gated at
    // gate_sigma (consecutive_rejects == nullptr: never gated). Returns false
    // if gated out.
    // predicted: the measurement model's value when it is not H x (EKF);
    // NaN uses H x.
    bool update_axis(Axis &axis, float hp, float hv, float hb, float measurement, float variance,
                     std::uint32_t *consecutive_rejects, Innovation *record,
                     bool allow_resync = true,
                     float predicted = std::numeric_limits<float>::quiet_NaN());
    static void symmetrize(Axis &axis);
    // Position += velocity * dt and covariance growth with the no-IMU process
    // noise (zero acceleration assumed). Shared by predict_without_imu,
    // predict_magnitude_only and the bridging of a predict gap.
    void propagate_without_acceleration(float dt_s, float accel_noise_mps2);
    // After every predict/update: a non-finite (or absurd) state or covariance
    // resets the filter to a large covariance with the pad references kept.
    void check_health();
    void recover_from_divergence();
    void grow_baro_scale(float dt_s);
    bool baro_scale_started_ = false;
public:
    // Barometer scale estimate k (the barometer reads (1 + k) x height) and
    // its standard deviation.
    float baro_scale() const { return axes_[2].b; }
    float baro_scale_sigma() const { return std::sqrt(axes_[2].P22); }
    // Gravity subtracted from the up specific force: the mean |specific
    // force| measured while still on the pad (it includes the local g and the
    // residual accelerometer offset), kGravity until ~1 s of still samples
    // was averaged, frozen once the vehicle leaves the pad.
    float gravity_mps2() const { return gravity_mps2_; }
    bool gravity_measured() const { return gravity_still_s_ >= 1.0F; }
    bool gravity_frozen() const { return gravity_frozen_; }
    // Times the pad average fell outside kGravity +- 5 % (kGravity kept).
    std::uint32_t gravity_clamped_count() const { return gravity_clamped_count_; }
private:
    void track_pad_gravity(float specific_force, float dt_s, Phase phase);

    Config config_;
    Axis axes_[3]{};
    bool initialized_ = false;
    bool still_ = true;
    bool imu_ignored_ = false;
    bool baro_gated_ = false;
    float gravity_mps2_ = kGravity;
    float gravity_mean_mps2_ = kGravity;
    float gravity_still_s_ = 0.0F;
    float gravity_moving_s_ = 0.0F;
    bool gravity_frozen_ = false;
    bool gravity_out_of_range_ = false;
    std::uint32_t gravity_clamped_count_ = 0;
    float last_baro_sigma_m_ = 0.0F;
    float last_gps_sigma_h_ = 0.0F;
    float gps_stress_ = 0.0F;
    float last_gps_sigma_v_ = 0.0F;
    // Pad references (tracked while still on the pad).
    bool baro_reference_set_ = false;
    float baro_reference_m_ = 0.0F;
    bool gps_origin_set_ = false;
    double gps_origin_lat_ = 0.0;
    double gps_origin_lon_ = 0.0;
    float gps_origin_alt_ = 0.0F;
    double meters_per_deg_lat_ = 0.0;
    double meters_per_deg_lon_ = 0.0;
    std::uint32_t baro_consecutive_rejects_ = 0;
    std::uint32_t gps_consecutive_rejects_[3]{};
    std::uint32_t baro_updates_ = 0;
    // Update cycles (begin_update_cycle) since a barometer update was
    // accepted; the barometer counts as healthy below kBaroHealthyCycles.
    static constexpr std::uint32_t kBaroHealthyCycles = 50;   // 2 s at 25 Hz
    std::uint32_t cycles_since_baro_ = 0xFFFFFFFFU;
    // Same for an accepted GPS altitude (the barometer scale is only
    // estimated while GPS altitude is recent).
    std::uint32_t cycles_since_gps_up_ = 0xFFFFFFFFU;
    std::uint32_t gps_updates_ = 0;
    std::uint32_t baro_rejects_total_ = 0;
    std::uint32_t gps_rejects_total_ = 0;
    std::uint32_t baro_gated_count_ = 0;
    std::uint32_t rejected_inputs_ = 0;
    std::uint32_t divergence_resets_ = 0;
    std::uint32_t predict_gaps_ = 0;
    Innovation baro_innovation_{};
    Innovation gps_innovation_[3]{};
    // Filter clock (sum of predict dt) for the GPS trust windows.
    double clock_s_ = 0.0;
    double gps_prev_fix_t_ = -1.0;
    GpsTrust gps_trust_[3]{};
};

}  // namespace position
