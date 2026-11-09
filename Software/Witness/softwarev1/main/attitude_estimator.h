#pragma once

#include <cstddef>
#include <cstdint>

// Strapdown attitude estimator. Pure logic (no ESP-IDF/FreeRTOS), so it is
// host-testable; the caller supplies time and serializes access.
//
// Convention (comms.h, "Orientation quaternion"): Hamilton quaternion that
// rotates BODY vectors into WORLD, v_world = q * v_body * q^-1. WORLD is ENU
// (+Z up); BODY is the LSM6 sensor frame with +X along the nose axis. Heading
// is unobservable (no magnetometer): initialization uses the shortest-arc
// rotation that maps the measured body specific-force direction onto +Z.
//
// The gyro on the Witness board shows impulsive interference (1-3 sample
// spikes up to several hundred dps, worst while the LoRa PA transmits), so
// the design is robust rather than strict:
//  - Integration path: per-axis running median over gyro_median_length
//    samples (default 5, i.e. 20 ms latency at 100 Hz). Runs of up to
//    (length-1)/2 outlier samples are removed; ramps and steps lasting at
//    least (length+1)/2 samples pass unchanged (only delayed).
//  - Stationary windows (Pad Idle): stationarity comes from the
//    accelerometer (every sample |a| in band and low accel scatter over the
//    window). The gyro window is judged with per-axis median and MAD and may
//    contain up to max_outlier_fraction outliers; the bias is the median
//    refined (by at most one quantization step) with the mean of the samples
//    within 3 robust sigmas of it, which resolves below the 70 mdps step where
//    a median of quantized counts cannot.
//
// Phases:
//  - PadIdle: every step is gyro propagated (filtered, bias-corrected). When
//    a 1 s stationary window is accepted, the first one initializes the
//    attitude; later ones set the bias, restore the attitude to its value at
//    the window start (the board did not rotate, so gyro noise integrated
//    over the window is discarded) and apply a tilt correction toward the
//    window's mean specific force. Non-stationary samples (for example the
//    >10 g launch-detection window) restart the window and are only gyro
//    propagated, so the attitude at the Boost transition includes the motion
//    since ignition.
//  - Boost, Coast, Descent: pure filtered, bias-corrected gyro integration.
//  - Landed: integration stops and the attitude is held.
//  - Gyro saturation in flight (any raw axis at or above
//    gyro_saturation_rad_s for saturation_samples consecutive samples, as a
//    parachute deployment can cause): the rate is unknown, so the attitude is
//    unrecoverable. Integration stops, valid() becomes false (quaternion all
//    zero) and lost() is set for the rest of the flight; only a new
//    stationary window after returning to Pad Idle re-initializes it.
namespace attitude {

struct Quaternion {
    // Stored scalar-first internally; the wire order is x, y, z, w.
    float w = 0.0F;
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

struct Vector3 {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

// Numerically identical to FlightStateMachine::State.
enum class FlightPhase : std::uint8_t {
    PadIdle = 0,
    Boost = 1,
    Coast = 2,
    Descent = 3,
    Landed = 4,
};

constexpr float kDpsToRadS = 3.14159265358979F / 180.0F;

struct Config {
    // Window acceptance: accelerometer.
    float stationary_min_g = 0.95F;     // Every sample's |a| must be in band.
    float stationary_max_g = 1.05F;
    float max_accel_sd_g = 0.02F;       // RMS of per-axis standard deviations.
    // Window acceptance: gyro (robust).
    float max_gyro_sigma_rad_s = 10.0F * kDpsToRadS;  // 1.4826 * MAD per axis
    float gyro_outlier_rad_s = 10.0F * kDpsToRadS;    // |w - median| any axis
    float max_outlier_fraction = 0.4F;
    float max_bias_rad_s = 5.0F * kDpsToRadS;         // |median| (vector norm)
    // Gyro quantization step (70 mdps at +/-2000 dps). The bias is the window
    // median refined by the mean of the samples near it; the refinement is
    // clamped to one step, so dense interference cannot pull it further.
    float gyro_quantum_rad_s = 0.070F * kDpsToRadS;
    std::uint32_t window_us = 1'000'000U;
    std::uint32_t window_min_samples = 50U;
    // Pad tilt correction per accepted window: fraction window/tau.
    float tilt_correction_tau_s = 2.0F;
    // Running median length on the integration path: 1 (off), 3 or 5.
    std::uint32_t gyro_median_length = 5U;
    // A sample counts as a rejected spike when its raw value differs from the
    // median output by more than this on any axis.
    float spike_threshold_rad_s = 10.0F * kDpsToRadS;
    std::uint32_t nominal_dt_us = 10'000U;
    // dt above this (or <= 0) is a gap: the sample is not integrated.
    std::uint32_t max_dt_us = 50'000U;
    // dt above this (but not a gap) counts as a late step.
    std::uint32_t late_dt_us = 15'000U;
    // Gyro saturation (full scale is +/-2000 dps; 1950 leaves margin for the
    // on-chip filter rounding a clipped input just below full scale).
    float gyro_saturation_rad_s = 1950.0F * kDpsToRadS;
    std::uint32_t saturation_samples = 2U;
};

struct Step {
    Vector3 gyro_rad_s{};       // Body angular rate.
    Vector3 specific_force_g{}; // Body specific force (accelerometer), g.
    std::uint64_t timestamp_us = 0;
    FlightPhase phase = FlightPhase::PadIdle;
};

struct StepResult {
    bool initialized_now = false;   // First valid attitude on this step.
    bool bias_updated = false;      // A stationary window was accepted.
    bool window_rejected = false;   // A full window failed the checks.
    bool entered_landed = false;    // First step in Landed: log the hold.
    bool gap = false;               // dt rejected; not integrated.
    bool lost_now = false;          // Gyro saturated in flight on this step.
};

class Estimator final {
public:
    static constexpr std::size_t kMaxWindowSamples = 1024U;
    static constexpr std::size_t kMaxMedianLength = 5U;

    explicit Estimator(const Config &config = Config{});

    StepResult update(const Step &step);
    // Forgets attitude, bias, window, filter and timing (counters are kept).
    void reset();

    bool valid() const { return valid_; }
    // Attitude given up after gyro saturation in flight (see header comment).
    bool lost() const { return lost_; }
    std::uint32_t saturation_events() const { return saturation_events_; }
    // All-zero until valid().
    Quaternion quaternion() const { return valid_ ? q_ : Quaternion{}; }
    Vector3 gyro_bias_rad_s() const { return bias_; }
    // Angle between body +X and world +Z, degrees; 0 upright, 90 horizontal.
    float tilt_from_vertical_deg() const;
    // Integration-path delay introduced by the median, in samples.
    std::uint32_t filter_lag_samples() const { return median_length_ / 2U; }

    std::uint32_t steps() const { return steps_; }            // Integrated
    std::uint32_t gaps() const { return gaps_; }
    std::uint32_t late_steps() const { return late_steps_; }
    std::uint32_t max_dt_us() const { return max_dt_us_; }
    void reset_max_dt() { max_dt_us_ = 0U; }
    std::uint32_t spikes_rejected() const { return spikes_rejected_; }
    std::uint32_t windows_accepted() const { return windows_accepted_; }
    std::uint32_t windows_rejected_accel() const { return windows_rejected_accel_; }
    std::uint32_t windows_rejected_gyro() const { return windows_rejected_gyro_; }
    // Integration since the last Boost entry (or the first flight step).
    std::uint32_t flight_steps() const { return flight_steps_; }
    double flight_time_s() const { return flight_time_s_; }
    bool landed_hold() const { return phase_ == FlightPhase::Landed; }

    // Helpers, exposed for tests.
    static Quaternion multiply(const Quaternion &a, const Quaternion &b);
    static Quaternion exp_rotation(const Vector3 &rotation_vector_rad);
    static Quaternion shortest_arc(const Vector3 &from_unit, const Vector3 &to_unit);
    static Vector3 rotate(const Quaternion &q, const Vector3 &v);
    static Quaternion normalized(const Quaternion &q);
    static Quaternion tilt_corrected(const Quaternion &q, const Vector3 &accel_unit,
                                     float fraction);

private:
    void reset_window();
    void clear_filter();
    // Evaluates a complete window ending at timestamp_us.
    void finish_window(std::uint64_t timestamp_us, StepResult &result);
    float median_in_scratch(std::size_t count);

    Config config_;
    std::uint32_t median_length_ = 5U;
    Quaternion q_{1.0F, 0.0F, 0.0F, 0.0F};
    Vector3 bias_{};
    bool valid_ = false;
    bool lost_ = false;
    std::uint32_t saturation_run_ = 0;
    std::uint32_t saturation_events_ = 0;
    bool have_time_ = false;
    std::uint64_t last_timestamp_us_ = 0;
    FlightPhase phase_ = FlightPhase::PadIdle;
    bool phase_known_ = false;

    // Running-median history (ring), each with the sample's dt (0 = skip).
    Vector3 history_gyro_[kMaxMedianLength]{};
    std::uint32_t history_dt_us_[kMaxMedianLength]{};
    std::size_t history_count_ = 0;
    std::size_t history_head_ = 0;  // Next write position.

    // Stationary window.
    float window_gyro_[3][kMaxWindowSamples]{};
    float scratch_[kMaxWindowSamples]{};
    double window_accel_sum_[3]{};
    double window_accel_sq_[3]{};
    std::size_t window_count_ = 0;
    std::uint64_t window_start_us_ = 0;
    Quaternion window_start_q_{1.0F, 0.0F, 0.0F, 0.0F};

    std::uint32_t steps_ = 0;
    std::uint32_t gaps_ = 0;
    std::uint32_t late_steps_ = 0;
    std::uint32_t max_dt_us_ = 0;
    std::uint32_t spikes_rejected_ = 0;
    std::uint32_t windows_accepted_ = 0;
    std::uint32_t windows_rejected_accel_ = 0;
    std::uint32_t windows_rejected_gyro_ = 0;
    std::uint32_t flight_steps_ = 0;
    double flight_time_s_ = 0.0;
};

}  // namespace attitude
