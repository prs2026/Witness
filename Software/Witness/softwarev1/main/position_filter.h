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

#include <cstdint>

namespace position {

inline constexpr float kGravity = 9.80665F;
inline constexpr float kBaroMachCutoff = 0.8F;
inline constexpr float kBaroMachResume = 0.75F;
inline constexpr float kBaroMinPressurePa = 1000.0F;  // MS5607: 10 mbar

struct Config {
    // White acceleration noise: base plus a fraction of |a| (attitude error
    // turns a share of the thrust into a wrong direction).
    float accel_noise_mps2 = 0.5F;
    float accel_noise_fraction = 0.05F;
    // Barometer: effective pressure error (noise, lag, model) and a floor.
    float baro_pressure_sigma_pa = 10.0F;
    float baro_min_sigma_m = 0.5F;
    // GPS position sigma (no HDOP in GGA as parsed), inflated with few sats.
    float gps_horizontal_sigma_m = 2.5F;
    float gps_vertical_sigma_m = 5.0F;
    std::uint8_t gps_good_satellites = 6;
    // Innovation gate (sigmas); a measurement stream rejected this many
    // times in a row is accepted anyway so the filter can re-synchronize.
    float gate_sigma = 5.0F;
    std::uint32_t max_consecutive_rejects = 25;
    // Pad stillness: |specific force| within this of 1 g.
    float still_band_g = 0.05F;
};

enum class Phase : std::uint8_t {  // numerically FlightStateMachine::State
    PadIdle = 0, Boost = 1, Coast = 2, Descent = 3, Landed = 4,
};

struct Axis {
    float p = 0.0F;
    float v = 0.0F;
    float P00 = 1.0F, P01 = 0.0F, P11 = 1.0F;
};

class Filter {
public:
    explicit Filter(const Config &config = Config{}) : config_(config) {}

    void reset();
    // acceleration_world_mps2: ENU specific force rotated to world, m/s^2
    // (gravity NOT removed; the filter subtracts it). dt in seconds.
    void predict(const float acceleration_world_mps2[3], float dt_s, Phase phase);

    // Barometric altitude (any fixed reference, e.g. ISA from 1013.25 hPa)
    // and pressure. Returns true if it was used.
    bool update_baro(float altitude_m, float pressure_pa, Phase phase);
    // GPS fix (WGS-84 degrees, MSL metres). Returns true if it was used.
    bool update_gps(double latitude_deg, double longitude_deg, float altitude_m,
                    std::uint8_t satellites, Phase phase);
    // Pad/landed pseudo-measurements; call at the update rate.
    void update_constraints(Phase phase);

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
    std::uint32_t baro_updates() const { return baro_updates_; }
    std::uint32_t gps_updates() const { return gps_updates_; }
    std::uint32_t baro_rejects() const { return baro_rejects_total_; }
    std::uint32_t gps_rejects() const { return gps_rejects_total_; }
    std::uint32_t baro_gated_count() const { return baro_gated_count_; }

    // Exposed for tests and logging.
    static float isa_temperature_k(float altitude_msl_m);
    static float speed_of_sound_mps(float altitude_msl_m);
    float baro_sigma_m(float pressure_pa, float altitude_msl_m) const;

private:
    // Returns false if gated out.
    bool update_position(Axis &axis, float measurement, float variance,
                         std::uint32_t &consecutive_rejects);
    void update_velocity(Axis &axis, float measurement, float variance);
    static void symmetrize(Axis &axis);

    Config config_;
    Axis axes_[3]{};
    bool initialized_ = false;
    bool still_ = true;
    bool baro_gated_ = false;
    float last_baro_sigma_m_ = 0.0F;
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
    std::uint32_t gps_updates_ = 0;
    std::uint32_t baro_rejects_total_ = 0;
    std::uint32_t gps_rejects_total_ = 0;
    std::uint32_t baro_gated_count_ = 0;
};

}  // namespace position
