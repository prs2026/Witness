#include "position_filter.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>

using position::Filter;
using position::Phase;

namespace {

int failures = 0;

void check(const bool condition, const char *what, const int line)
{
    if (!condition) {
        std::printf("FAIL line %d: %s\n", line, what);
        ++failures;
    }
}
#define CHECK(c) check((c), #c, __LINE__)

constexpr float kG = position::kGravity;
constexpr double kPadLat = 35.35;
constexpr double kPadLon = -117.81;
constexpr float kPadAltMsl = 620.0F;

// ISA pressure (Pa) at an MSL altitude, troposphere and lower stratosphere.
float isa_pressure(const float h)
{
    if (h < 11000.0F) return 101325.0F * std::pow(1.0F - 2.25577e-5F * h, 5.25588F);
    return 22632.1F * std::exp(-1.57688e-4F * (h - 11000.0F));
}

struct FlightResult {
    float max_up_error = 0.0F;
    float max_horizontal_error = 0.0F;
    float up_error_at_apogee = 0.0F;
    float speed_error_at_boost = 0.0F;
    float max_mach = 0.0F;
    std::uint32_t baro_gated = 0;
    std::uint32_t baro_used = 0;
    std::uint32_t gps_used = 0;
};

// Vertical boost at 11 g net for 4 s (about Mach 1.4), ballistic coast to
// apogee, then 20 s of descent at 30 m/s; a steady 3 m/s eastward drift.
// gps_rate_hz 0 disables GPS. transonic_error adds a baro error between
// Mach 0.8 and 1.2, as a shock wave over the static port would.
FlightResult fly(const int gps_rate_hz, const float transonic_error_m,
                 const float accel_bias_up, const unsigned seed)
{
    std::mt19937 rng(seed);
    std::normal_distribution<float> unit(0.0F, 1.0F);
    Filter filter;
    FlightResult result;

    const float rate = 960.0F;
    const float dt = 1.0F / rate;
    const int update_every = static_cast<int>(rate / 25.0F);   // 25 Hz update step
    const int gps_every = gps_rate_hz > 0 ? static_cast<int>(rate / gps_rate_hz) : 0;
    const float pad_time = 10.0F, burn = 4.0F, thrust_net = 11.0F * kG;
    float p_up = 0.0F, v_up = 0.0F, p_east = 0.0F;
    Phase phase = Phase::PadIdle;
    float boost_time = -1.0F;
    bool apogee_seen = false;
    float descent_end = 1e9F;
    bool gps_pending = false;  // a fix arrived; used at the next update tick

    for (int n = 0;; ++n) {
        const float t = n * dt;
        // Truth.
        float a_up = 0.0F;
        float v_east = 0.0F;
        if (t >= pad_time && t < pad_time + burn) a_up = thrust_net;
        else if (t >= pad_time + burn && !apogee_seen) a_up = -kG;
        if (apogee_seen) { v_up = -30.0F; a_up = 0.0F; }
        if (t >= pad_time) v_east = 3.0F;
        v_up += (apogee_seen ? 0.0F : a_up * dt);
        p_up += v_up * dt;
        p_east += v_east * dt;
        if (!apogee_seen && t > pad_time + burn && v_up <= 0.0F) {
            apogee_seen = true;
            result.up_error_at_apogee = std::fabs(filter.up().p - p_up);
            descent_end = t + 20.0F;
            phase = Phase::Descent;
        }
        if (t > descent_end) break;

        // The state machine needs 0.3 s above 10 g before declaring Boost.
        if (phase == Phase::PadIdle && t >= pad_time + 0.3F) {
            phase = Phase::Boost;
            boost_time = t;
        }
        if (phase == Phase::Boost && t >= pad_time + burn) phase = Phase::Coast;
        if (boost_time >= 0.0F && std::fabs(t - boost_time) < dt / 2) {
            result.speed_error_at_boost = std::fabs(filter.up().v - v_up);
        }

        // IMU: world-frame specific force with noise and an up bias.
        const float specific_up = (apogee_seen ? 0.0F : a_up) + kG;
        const float accel[3] = {0.3F * unit(rng), 0.3F * unit(rng),
                                specific_up + accel_bias_up + 0.5F * unit(rng)};
        filter.predict(accel, dt, phase);

        if (gps_every > 0 && n % gps_every == 0) gps_pending = true;
        if (n % update_every == 0) {
            const float h_msl = kPadAltMsl + p_up;
            const float speed = std::sqrt(v_up * v_up + v_east * v_east);
            const float mach = speed / Filter::speed_of_sound_mps(h_msl);
            if (mach > result.max_mach) result.max_mach = mach;
            const float pressure = isa_pressure(h_msl);
            const float sigma = filter.baro_sigma_m(pressure, h_msl);
            float baro = h_msl + sigma * unit(rng);
            if (mach > 0.8F && mach < 1.2F) baro += transonic_error_m;
            if (filter.update_baro(baro, pressure, phase)) ++result.baro_used;
            if (gps_pending) {
                gps_pending = false;
                const double m_per_deg_lon = 111412.84 * std::cos(kPadLat * 3.14159265 / 180);
                const double lat = kPadLat + 2.5 * unit(rng) / 110950.0;
                const double lon = kPadLon + (p_east + 2.5 * unit(rng)) / m_per_deg_lon;
                if (filter.update_gps(lat, lon, kPadAltMsl + p_up + 5.0F * unit(rng), 9, phase)) {
                    ++result.gps_used;
                }
            }
            filter.update_constraints(phase);
        }
        if (t > pad_time) {
            const float up_error = std::fabs(filter.up().p - p_up);
            const float h_error = std::fabs(filter.east().p - p_east);
            if (up_error > result.max_up_error) result.max_up_error = up_error;
            if (h_error > result.max_horizontal_error) result.max_horizontal_error = h_error;
        }
    }
    result.baro_gated = filter.baro_gated_count();
    return result;
}

void test_atmosphere_and_baro_sigma()
{
    CHECK(std::fabs(Filter::speed_of_sound_mps(0.0F) - 340.3F) < 0.5F);
    CHECK(std::fabs(Filter::speed_of_sound_mps(11000.0F) - 295.1F) < 0.5F);
    Filter filter;
    const float s0 = filter.baro_sigma_m(isa_pressure(0.0F), 0.0F);
    const float s10 = filter.baro_sigma_m(isa_pressure(10000.0F), 10000.0F);
    const float s20 = filter.baro_sigma_m(isa_pressure(20000.0F), 20000.0F);
    std::printf("baro sigma: %.2f m at 0 km, %.2f m at 10 km, %.2f m at 20 km\n", s0, s10, s20);
    // sigma_p 5 Pa (tuned; an atmosphere off ISA is the bias state's job).
    CHECK(s0 > 0.55F && s0 < 0.8F);
    CHECK(s10 > 1.1F && s10 < 1.6F);
    CHECK(s20 > 5.0F && s20 < 6.6F);
    CHECK(s0 < s10 && s10 < s20);  // trust falls with altitude
    // Out of the MS5607's range: never used.
    CHECK(!filter.update_baro(30000.0F, 900.0F, Phase::Coast));
}

void test_pad_hold()
{
    Filter filter;
    const float still[3] = {0.0F, 0.0F, kG};
    for (int n = 0; n < 9600; ++n) {
        filter.predict(still, 1.0F / 960.0F, Phase::PadIdle);
        if (n % 38 == 0) {
            filter.update_baro(620.0F + ((n / 38) % 2 ? 0.4F : -0.4F), 94000.0F, Phase::PadIdle);
            filter.update_constraints(Phase::PadIdle);
        }
    }
    CHECK(filter.still());
    CHECK(std::fabs(filter.up().p) < 0.05F && std::fabs(filter.up().v) < 0.05F);
    CHECK(std::fabs(filter.altitude_msl_m() - 620.0F) < 0.3F);  // reference tracks the pad
}

void test_flights()
{
    // Baro + IMU only, transonic baro error 150 m: gating keeps it out.
    FlightResult r = fly(0, 150.0F, 0.05F, 1);
    std::printf("no GPS:     max |up error| %.1f m, at apogee %.1f m, max Mach %.2f, "
                "baro used %u, gated %u, speed error at Boost %.2f m/s\n",
                r.max_up_error, r.up_error_at_apogee, r.max_mach,
                static_cast<unsigned>(r.baro_used), static_cast<unsigned>(r.baro_gated),
                r.speed_error_at_boost);
    CHECK(r.max_mach > 1.2F);
    CHECK(r.baro_gated > 50);
    // Without GPS the barometer scale is a consider parameter: the barometer
    // is trusted to (height x baro_scale_sigma), so IMU drift while it is
    // gated is pulled out more slowly than with a fully trusted ISA
    // barometer (peak ~125 m, back to a few m by apogee).
    CHECK(r.max_up_error < 150.0F);
    // A 150 m transonic baro error never entered: the result is the same
    // as with no transonic error at all.
    const FlightResult clean = fly(0, 0.0F, 0.05F, 1);
    CHECK(std::fabs(r.max_up_error - clean.max_up_error) < 2.0F);
    // Since 2026-11-14 the barometer stays gated for all of Coast once it was
    // gated in Boost (Mach errors continue after burnout), so without GPS the
    // apogee altitude is IMU-only from Mach 0.8 up: ~33 m here (was < 15 m
    // when the barometer resumed below Mach 0.75 in Coast). With GPS it stays
    // a few metres (checks below).
    CHECK(r.up_error_at_apogee < 40.0F);
    CHECK(r.speed_error_at_boost < 3.0F);    // ~32 m/s before Boost was integrated, not lost

    // With a 10 Hz and a 25 Hz GPS.
    for (const int rate : {10, 25}) {
        FlightResult g = fly(rate, 150.0F, 0.2F, 2);
        std::printf("GPS %2d Hz:  max |up error| %.1f m, at apogee %.1f m, max |east error| "
                    "%.1f m, gps used %u\n", rate, g.max_up_error, g.up_error_at_apogee,
                    g.max_horizontal_error, static_cast<unsigned>(g.gps_used));
        // This GPS has no lag, but the filter down-weights GPS under
        // acceleration and for gps_stress_tau_s after it (the real PX1120S
        // lagged 60 m in boost and 870 m in the 43 g sustainer in HITL RF
        // runs; tuning on Monte Carlo replays of those fixes chose strong
        // de-weighting), so through the burn this test's IMU drift shows.
        CHECK(g.max_up_error < 40.0F);
        CHECK(g.up_error_at_apogee < 10.0F);
        CHECK(g.max_horizontal_error < 20.0F);
        CHECK(g.gps_used > 100);
    }
}

// Barometer bias state: a subsonic climb to 1800 m where the barometer reads
// 5 % of the climb high (warm day: the ISA conversion is off) and GPS is
// exact. With the bias state the estimate follows GPS and the bias takes up
// the 90 m; without it the barometer (tighter sigma) pulls the estimate high.
float climb_with_baro_scale_error(const float scale_sigma, float &scale_estimate)
{
    position::Config config;
    config.baro_scale_sigma = scale_sigma;
    Filter filter(config);
    const float still[3] = {0.0F, 0.0F, kG};
    for (int n = 0; n < 960; ++n) {
        filter.predict(still, 1.0F / 960.0F, Phase::PadIdle);
        if (n % 38 == 0) {
            filter.update_baro(kPadAltMsl, isa_pressure(kPadAltMsl), Phase::PadIdle);
            filter.update_gps(kPadLat, kPadLon, kPadAltMsl, 10, Phase::PadIdle, 1.0F, 1.5F);
            filter.update_constraints(Phase::PadIdle);
        }
    }
    const float climb[3] = {0.0F, 0.0F, kG + 1.0F};   // 1 m/s^2 up
    float worst = 0.0F;
    for (int n = 1; n <= 60 * 960; ++n) {
        filter.predict(climb, 1.0F / 960.0F, Phase::Boost);
        if (n % 38 != 0) continue;   // ~25 Hz
        const float t = n / 960.0F;
        const float h = 0.5F * t * t;
        filter.update_baro(kPadAltMsl + 1.05F * h, isa_pressure(kPadAltMsl + h), Phase::Boost);
        filter.update_gps(kPadLat, kPadLon, kPadAltMsl + h, 10, Phase::Boost, 1.0F, 1.5F);
        worst = std::fmax(worst, std::fabs(filter.up().p - h));
    }
    scale_estimate = filter.baro_scale();
    return worst;
}

void test_baro_bias_state()
{
    float k_on = 0.0F, k_off = 0.0F;
    const float with_scale = climb_with_baro_scale_error(0.03F, k_on);
    const float without = climb_with_baro_scale_error(0.0F, k_off);
    std::printf("baro +5%%:   max |up error| %.1f m with the scale state (k %.4f), %.1f m without\n",
                with_scale, k_on, without);
    CHECK(with_scale < 12.0F);
    CHECK(without > 2.0F * with_scale);
    CHECK(std::fabs(k_on - 0.05F) < 0.01F);   // the barometer read 5 % high
    CHECK(k_off == 0.0F);
}

void test_gate_and_resync()
{
    Filter filter;
    const float coast[3] = {0.0F, 0.0F, 0.0F};  // free fall
    for (int n = 0; n < 960; ++n) filter.predict(coast, 1.0F / 960.0F, Phase::Coast);
    filter.update_baro(100.0F, 90000.0F, Phase::Coast);  // sets the reference
    // A single wild baro reading is rejected.
    const float before = filter.up().p;
    CHECK(!filter.update_baro(5000.0F, 90000.0F, Phase::Coast));
    CHECK(filter.up().p == before);
    CHECK(filter.baro_rejects() == 1);
}

void test_predict_without_imu()
{
    // Under a parachute at 20 m/s down with the IMU ignored: the baro alone
    // brings the estimate to the right altitude and descent rate.
    Filter filter;
    const float still[3] = {0.0F, 0.0F, kG};
    for (int n = 0; n < 960; ++n) filter.predict(still, 1.0F / 960.0F, Phase::Coast);
    filter.update_baro(3000.0F, 70000.0F, Phase::Coast);  // reference
    float truth = 0.0F;  // up from the reference point
    for (int n = 0; n < 960 * 30; ++n) {
        filter.predict_without_imu(1.0F / 960.0F, Phase::Descent);
        truth -= 20.0F / 960.0F;
        if (n % 38 == 0) filter.update_baro(3000.0F + truth, 80000.0F, Phase::Descent);
    }
    CHECK(filter.imu_ignored());
    std::printf("no-IMU descent: up error %.2f m, vz %.2f m/s (truth -20)\n",
                filter.up().p - truth, filter.up().v);
    CHECK(std::fabs(filter.up().p - truth) < 3.0F);
    CHECK(std::fabs(filter.up().v + 20.0F) < 1.5F);
    // A normal predict clears the flag again.
    filter.predict(still, 1.0F / 960.0F, Phase::Descent);
    CHECK(!filter.imu_ignored());
}

void test_baro_thrust_gate()
{
    // Liftoff: 12 g for 0.3 s (the launch qualification window, phase still
    // Pad Idle), the baro reading 6 m high (ignition transient, port lag).
    // The baro must not touch the velocity: it equals the IMU integral.
    const float pad_alt = 600.0F;
    auto pad = [&](Filter &f) {
        const float still[3] = {0.0F, 0.0F, kG};
        for (int n = 0; n < 960 * 3; ++n) {
            f.predict(still, 1.0F / 960.0F, Phase::PadIdle);
            if (n % 38 == 0) f.update_baro(pad_alt, isa_pressure(pad_alt), Phase::PadIdle);
        }
    };
    Filter f;
    pad(f);
    const float thrust[3] = {0.0F, 0.0F, 12.0F * kG};
    float truth_v = 0.0F;
    for (int n = 0; n < 288; ++n) {
        f.predict(thrust, 1.0F / 960.0F, Phase::PadIdle);
        truth_v += 11.0F * kG / 960.0F;
        if (n % 38 == 37) {
            CHECK(!f.update_baro(pad_alt + 6.0F + f.up().p, isa_pressure(pad_alt), Phase::PadIdle));
            CHECK(f.baro_thrust_gated());
        }
    }
    std::printf("thrust gate: vz %.2f m/s (IMU integral %.2f), gated %d\n", f.up().v, truth_v,
                f.baro_thrust_gated() ? 1 : 0);
    CHECK(std::fabs(f.up().v - truth_v) < 0.5F);
    CHECK(f.baro_gated());

    // Subsonic flight: burnout at ~65 m/s, Coast: the baro is back (Mach
    // gate never tripped).
    Filter s;
    pad(s);
    const float boost[3] = {0.0F, 0.0F, 3.0F * kG};
    for (int n = 0; n < 960 * 3 + 300; ++n) s.predict(boost, 1.0F / 960.0F, Phase::Boost);
    CHECK(!s.update_baro(pad_alt + s.up().p, isa_pressure(pad_alt), Phase::Boost));
    const float coast[3] = {0.0F, 0.0F, 0.0F};
    s.predict(coast, 1.0F / 960.0F, Phase::Coast);
    CHECK(s.update_baro(pad_alt + s.up().p, isa_pressure(pad_alt + s.up().p), Phase::Coast));
    CHECK(!s.baro_thrust_gated());

    // A 3 g bump on the pad gates it only until the pad hold returns.
    Filter b;
    pad(b);
    const float bump[3] = {0.0F, 0.0F, 3.0F * kG};
    b.predict(bump, 1.0F / 960.0F, Phase::PadIdle);
    CHECK(!b.update_baro(pad_alt, isa_pressure(pad_alt), Phase::PadIdle));
    CHECK(b.baro_thrust_gated());
    const float still[3] = {0.0F, 0.0F, kG};
    for (int n = 0; n < 960; ++n) b.predict(still, 1.0F / 960.0F, Phase::PadIdle);
    b.update_baro(pad_alt, isa_pressure(pad_alt), Phase::PadIdle);
    CHECK(!b.baro_thrust_gated());
}

void test_unaided_tracking_and_rest_hold()
{
    // vertical_unaided: set on the first no-IMU predict with no altitude
    // aid, with the velocity before any decay and the elapsed time.
    Filter f;
    const float still[3] = {0.0F, 0.0F, kG};
    const float up[3] = {0.0F, 0.0F, kG + 30.0F};
    for (int n = 0; n < 960; ++n) f.predict(up, 1.0F / 960.0F, Phase::Boost);   // ~30 m/s up
    const float vz0 = f.up().v;
    CHECK(!f.vertical_unaided());
    f.predict_without_imu(1.0F / 960.0F, Phase::Coast);
    CHECK(f.vertical_unaided());
    CHECK(std::fabs(f.unaided_entry_vz_mps() - vz0) < 0.05F);
    for (int n = 0; n < 960 * 2; ++n) f.predict_without_imu(1.0F / 960.0F, Phase::Coast);
    std::printf("unaided: entry vz %.2f m/s, elapsed %.2f s, vz now %.2f m/s\n",
                f.unaided_entry_vz_mps(), f.unaided_elapsed_s(), f.up().v);
    CHECK(std::fabs(f.unaided_elapsed_s() - 2.0F) < 0.01F);
    CHECK(std::fabs(f.unaided_entry_vz_mps() - vz0) < 0.05F);   // not overwritten
    // IMU back: no longer unaided.
    f.predict(still, 1.0F / 960.0F, Phase::Descent);
    CHECK(!f.vertical_unaided());

    // Rest hold with the IMU: a small tilt error (0.5 deg -> 0.086 m/s^2
    // sideways) drifts 2.6 m/s in 30 s without it; with it the velocity
    // stays near 0 and the AGL moves under a metre.
    Filter d;
    Filter h;
    const float tilted[3] = {0.0856F, 0.0F, kG + 0.02F};
    d.predict_without_imu(1.0F / 960.0F, Phase::Descent);
    h.predict_without_imu(1.0F / 960.0F, Phase::Descent);
    const float p0 = h.up().p;
    for (int n = 0; n < 960 * 30; ++n) {
        d.predict(tilted, 1.0F / 960.0F, Phase::Landed);
        h.predict(tilted, 1.0F / 960.0F, Phase::Landed);
        h.hold_unaided_velocity(1.0F / 960.0F);
    }
    std::printf("rest hold 30 s: without v (%.2f, %.2f) m/s, with v (%.3f, %.3f) m/s, AGL moved %.2f m\n",
                d.east().v, d.up().v, h.east().v, h.up().v, h.up().p - p0);
    CHECK(std::fabs(d.east().v) > 1.0F);
    CHECK(std::fabs(h.east().v) < 0.3F && std::fabs(h.up().v) < 0.1F);
    CHECK(std::fabs(h.up().p - p0) < 2.0F);   // landing allows 10 m in 30 s

    // With an accepted barometer the hold does nothing to the vertical axis.
    Filter b;
    for (int n = 0; n < 960; ++n) b.predict(still, 1.0F / 960.0F, Phase::Descent);
    const float dn[3] = {0.0F, 0.0F, kG - 5.0F};
    for (int n = 0; n < 960; ++n) b.predict(dn, 1.0F / 960.0F, Phase::Descent);   // ~ -5 m/s
    b.update_baro(b.up().p, 90000.0F, Phase::Descent);
    const float vb = b.up().v;
    b.hold_unaided_velocity(0.1F);
    CHECK(std::fabs(b.up().v - vb) < 1e-4F);
}

void test_unaided_velocity_decays()
{
    // Flight 2026-11-21: attitude lost at apogee, no barometer, no GPS. The
    // velocity used to be held for good (2.4 m/s down, 28 m/s sideways); it
    // now decays to zero, so the AGL settles and a landing can be detected.
    Filter f;
    position::PadReferences refs;
    refs.motion_valid = true;
    refs.agl_m = 2600.0F;
    refs.vertical_velocity_mps = -2.4F;
    f.restore_after_reset(refs);
    // Horizontal velocity as at the loss (restore starts it at 0; set by GPS
    // in a normal flight, here by a pad-frame fix pair is overkill: predict
    // with an acceleration instead).
    const float push[3] = {-24.7F / 0.1F, -13.8F / 0.1F, kG};
    f.predict(push, 0.1F, Phase::Coast);
    CHECK(std::fabs(f.east().v + 24.7F) < 0.5F);
    const float p0 = f.up().p;
    const float e0 = f.east().p;
    for (int i = 0; i < 960 * 20; ++i) f.predict_without_imu(1.0F / 960.0F, Phase::Coast);
    CHECK(f.velocity_decaying());
    std::printf("unaided decay after 20 s: v (%.3f, %.3f, %.3f) m/s, moved %.1f m east, %.1f m up\n",
                f.east().v, f.north().v, f.up().v, f.east().p - e0, f.up().p - p0);
    CHECK(std::fabs(f.east().v) < 0.05F && std::fabs(f.north().v) < 0.05F &&
          std::fabs(f.up().v) < 0.05F);
    // Moved about v (1 s delay + 2 s tau): bounded, not 20 s of drift.
    CHECK(std::fabs(f.east().p - e0) < 24.7F * 3.5F);
    CHECK(std::fabs(f.up().p - p0) < 2.4F * 3.5F);
    // AGL is now stable: 30 s more moves it by under a metre.
    const float p1 = f.up().p;
    for (int i = 0; i < 960 * 30; ++i) f.predict_without_imu(1.0F / 960.0F, Phase::Descent);
    CHECK(std::fabs(f.up().p - p1) < 1.0F);

    // With a working barometer the vertical velocity is NOT decayed (the
    // descent rate comes from the baro), but the horizontal one still is.
    Filter g;
    const float still[3] = {0.0F, 0.0F, kG};
    for (int n = 0; n < 960; ++n) g.predict(still, 1.0F / 960.0F, Phase::Coast);
    g.update_baro(3000.0F, 70000.0F, Phase::Coast);
    float truth = 0.0F;
    for (int n = 0; n < 960 * 30; ++n) {
        g.predict_without_imu(1.0F / 960.0F, Phase::Descent);
        truth -= 20.0F / 960.0F;
        if (n % 38 == 0) g.update_baro(3000.0F + truth, 80000.0F, Phase::Descent);
    }
    CHECK(std::fabs(g.up().v + 20.0F) < 1.5F);

    // Disabled by config: held as before.
    position::Config c{};
    c.no_aid_velocity_tau_s = 0.0F;
    Filter h(c);
    h.restore_after_reset(refs);
    for (int i = 0; i < 960 * 5; ++i) h.predict_without_imu(1.0F / 960.0F, Phase::Coast);
    CHECK(std::fabs(h.up().v + 2.4F) < 1e-3F);
}

void test_innovation_record()
{
    Filter filter;
    const float still[3] = {0.0F, 0.0F, kG};
    for (int n = 0; n < 960; ++n) filter.predict(still, 1.0F / 960.0F, Phase::Coast);
    filter.begin_update_cycle();
    CHECK(std::isnan(filter.baro_innovation().value));
    CHECK(std::isnan(filter.gps_innovation(0).value));
    // First baro sets the reference (innovation 0); a 2 m step is then seen.
    filter.update_baro(100.0F, 100000.0F, Phase::Coast);
    filter.begin_update_cycle();
    const float before = filter.up().p;
    CHECK(filter.update_baro(102.0F, 100000.0F, Phase::Coast));
    const auto &b = filter.baro_innovation();
    CHECK(b.accepted);
    CHECK(std::fabs(b.value - (2.0F - before)) < 1e-4F);
    CHECK(b.variance > filter.last_baro_sigma_m() * filter.last_baro_sigma_m());
    // A 1 km jump is gated: recorded, not accepted.
    filter.begin_update_cycle();
    CHECK(!filter.update_baro(1102.0F, 100000.0F, Phase::Coast));
    CHECK(!filter.baro_innovation().accepted);
    CHECK(filter.baro_innovation().value > 900.0F);
    filter.begin_update_cycle();
    CHECK(std::isnan(filter.baro_innovation().variance));
}

// Truth for the re-pad / gravity tests: up position/velocity in double,
// driven by a piecewise net acceleration.
struct Truth {
    double p = 0.0, v = 0.0;
    void step(const double a, const double dt) { p += v * dt + 0.5 * a * dt * dt; v += a * dt; }
};

// Runs the filter at 960 Hz with a given net up acceleration (truth), with
// baro and GPS at 25 Hz when `aiding` (baro reference only on the pad
// otherwise). bias_up is the accelerometer's offset along up.
struct Rig {
    Filter filter;
    Truth truth;
    std::mt19937 rng{7};
    std::normal_distribution<float> unit{0.0F, 1.0F};
    float accel_noise = 0.05F;
    float bias_up = 0.0F;
    long n = 0;
    explicit Rig(const position::Config &c = position::Config{}) : filter(c) {}
    void run(const float seconds, const double net_up_accel, const Phase phase, const bool aiding)
    {
        const float dt = 1.0F / 960.0F;
        const int steps = static_cast<int>(seconds * 960.0F);
        for (int i = 0; i < steps; ++i, ++n) {
            truth.step(net_up_accel, dt);
            const float f_up = static_cast<float>(net_up_accel) + kG + bias_up;
            const float accel[3] = {accel_noise * unit(rng), accel_noise * unit(rng),
                                    f_up + accel_noise * unit(rng)};
            filter.predict(accel, dt, phase);
            if (n % 38 == 0) {
                const float h_msl = kPadAltMsl + static_cast<float>(truth.p);
                const float pressure = isa_pressure(h_msl);
                if (aiding) {
                    const float sigma = filter.baro_sigma_m(pressure, h_msl);
                    filter.update_baro(h_msl + 0.5F * sigma * unit(rng), pressure, phase);
                    filter.update_gps(kPadLat, kPadLon,
                                      kPadAltMsl + static_cast<float>(truth.p) + 1.0F * unit(rng), 9,
                                      phase);
                } else {
                    filter.update_baro(h_msl, pressure, phase);
                }
                filter.update_constraints(phase);
            }
        }
    }
};

// Finding 1: after a flight that ends 75 m above the pad, a return to Pad
// Idle must pull the position back to the origin (ungated pad hold), and a
// freshly reset filter starts at the origin.
void test_repad_after_flight()
{
    Rig r;
    r.run(5.0F, 0.0, Phase::PadIdle, false);
    CHECK(std::fabs(r.filter.up().p) < 0.05F);
    // Up 75 m and stop: +18.75 m/s2 for 2 s, then -18.75 m/s2 for 2 s.
    r.run(2.0F, 18.75, Phase::Boost, true);
    r.run(2.0F, -18.75, Phase::Coast, true);
    r.run(1.0F, 0.0, Phase::Descent, true);
    r.run(10.0F, 0.0, Phase::Landed, true);
    std::printf("re-pad: truth up %.1f m, filter up %.2f m after Landed\n", r.truth.p,
                static_cast<double>(r.filter.up().p));
    CHECK(std::fabs(r.truth.p - 75.0) < 0.5);
    CHECK(std::fabs(r.filter.up().p - 75.0F) < 3.0F);
    // Back to Pad Idle at the landing site without a reset: the pad is where
    // the board stands, so the position hold must pull the state to 0.
    r.truth = Truth{};
    r.run(4.0F, 0.0, Phase::PadIdle, false);
    std::printf("re-pad: filter up %.3f m, v %.3f after 4 s in Pad Idle (ungated hold)\n",
                static_cast<double>(r.filter.up().p), static_cast<double>(r.filter.up().v));
    CHECK(r.filter.still());
    CHECK(std::fabs(r.filter.up().p) < 0.2F);
    CHECK(std::fabs(r.filter.east().p) < 0.2F && std::fabs(r.filter.north().p) < 0.2F);
    // A fresh reset filter (what the reboot gives).
    r.filter.reset();
    r.truth = Truth{};
    r.run(3.0F, 0.0, Phase::PadIdle, false);
    CHECK(std::fabs(r.filter.up().p) < 0.2F && std::fabs(r.filter.up().v) < 0.1F);
    CHECK(std::fabs(r.filter.gravity_mps2() - kG) < 0.05F);
}

// Finding 3: a 1.01 g accelerometer reading on the pad (offset 0.098 m/s2),
// then a 60 s ballistic coast with no barometer and no GPS.
float coast_error_with_pad_gravity(const bool measure, float &g_used)
{
    position::Config config;
    config.measure_pad_gravity = measure;
    Rig r(config);
    r.bias_up = 0.01F * kG;
    r.run(20.0F, 0.0, Phase::PadIdle, false);
    r.run(3.0F, 10.0 * kG, Phase::Boost, false);                    // 11 g specific force
    r.run(60.0F, -static_cast<double>(kG), Phase::Coast, false);    // ballistic
    g_used = r.filter.gravity_mps2();
    CHECK(r.filter.gravity_frozen() == measure);
    return static_cast<float>(r.filter.up().p - r.truth.p);
}

void test_pad_measured_gravity()
{
    float g_measured = 0.0F, g_fixed = 0.0F;
    const float err_measured = coast_error_with_pad_gravity(true, g_measured);
    const float err_fixed = coast_error_with_pad_gravity(false, g_fixed);
    std::printf("gravity: pad-measured g %.4f m/s2 -> altitude error %.2f m after a 60 s coast; "
                "fixed %.5f -> %.1f m\n",
                static_cast<double>(g_measured), static_cast<double>(err_measured),
                static_cast<double>(g_fixed), static_cast<double>(err_fixed));
    CHECK(std::fabs(g_measured - (kG + 0.01F * kG)) < 0.02F);
    CHECK(g_fixed == kG);
    CHECK(std::fabs(err_measured) < 5.0F);
    CHECK(std::fabs(err_fixed) > 100.0F);
    // Fallback: kGravity until about 1 s of still samples was averaged; a
    // vehicle leaving the pad freezes the value.
    {
        Rig r;
        r.bias_up = 0.01F * kG;
        r.run(0.5F, 0.0, Phase::PadIdle, false);
        CHECK(!r.filter.gravity_measured() && r.filter.gravity_mps2() == kG);
        r.run(1.0F, 0.0, Phase::PadIdle, false);
        CHECK(r.filter.gravity_measured() && r.filter.gravity_mps2() > kG + 0.05F);
        const float g_pad = r.filter.gravity_mps2();
        r.run(1.0F, 5.0 * kG, Phase::Boost, false);
        r.run(5.0F, 0.0, Phase::PadIdle, false);   // still again: frozen anyway
        CHECK(r.filter.gravity_frozen() && r.filter.gravity_mps2() == g_pad);
        CHECK(r.filter.gravity_clamped_count() == 0);
        r.filter.reset();
        CHECK(r.filter.gravity_mps2() == kG && !r.filter.gravity_frozen());
    }
    // A short jolt on the pad does not end the averaging.
    {
        Rig r;
        r.run(3.0F, 0.0, Phase::PadIdle, false);
        r.run(0.1F, 2.0 * kG, Phase::PadIdle, false);
        r.run(3.0F, 0.0, Phase::PadIdle, false);
        CHECK(!r.filter.gravity_frozen());
    }
}

}  // namespace

void test_gps_dop_sigmas()
{
    const position::Filter filter;
    float h = 0.0F, v = 0.0F;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    filter.gps_sigmas(10, nan, nan, h, v);   // no DOP: fixed sigmas
    CHECK(h == 2.5F && v == 5.0F);
    filter.gps_sigmas(4, nan, nan, h, v);    // few satellites: doubled
    CHECK(h == 5.0F && v == 10.0F);
    filter.gps_sigmas(4, 1.0F, 1.5F, h, v);  // DOP overrides the satellite rule
    CHECK(std::fabs(h - 2.5F) < 1e-5F && std::fabs(v - 4.95F) < 1e-5F);
    filter.gps_sigmas(10, 4.0F, nan, h, v);  // VDOP unknown: 1.6 x HDOP
    CHECK(std::fabs(h - 10.0F) < 1e-5F && std::fabs(v - 3.3F * 6.4F) < 1e-4F);
    filter.gps_sigmas(10, 0.1F, 0.1F, h, v); // clamped to the minimum
    CHECK(h == 1.0F && v == 1.0F);
    filter.gps_sigmas(10, 1e6F, 1e6F, h, v);  // and the maximum
    const float cap = position::Config{}.gps_max_sigma_m;
    CHECK(h == cap && v == cap);

    // Higher DOP -> less weight: one GPS update moves the estimate less.
    auto pull = [](float hdop) {
        position::Filter f;
        f.reset();
        const float a[3] = {0.0F, 0.0F, position::kGravity};
        f.predict(a, 0.01F, position::Phase::Coast);
        CHECK(f.update_gps(10.0, 20.0, 100.0F, 10, position::Phase::Coast, hdop, hdop));
        f.predict(a, 0.01F, position::Phase::Coast);
        f.begin_update_cycle();
        const double lat = 10.0 + 5.0 / 110575.0;  // 5 m north of the origin
        CHECK(f.update_gps(lat, 20.0, 100.0F, 10, position::Phase::Coast, hdop, hdop));
        return f.north().p;
    };
    const float good = pull(0.8F);
    const float poor = pull(6.0F);
    CHECK(good > poor && poor > 0.0F);
    position::Filter f;
    CHECK(f.last_gps_horizontal_sigma_m() == 0.0F);
}

void test_gps_dynamics_sigmas()
{
    // At rest: DOP only. During a 35 g burn the receiver lags: GPS sigma grows
    // with the net acceleration and speed, then the acceleration part decays
    // with gps_stress_tau_s after burnout.
    position::Filter f;
    f.reset();
    float h = 0.0F, v = 0.0F;
    const float rest[3] = {0.0F, 0.0F, position::kGravity};
    for (int i = 0; i < 960; ++i) f.predict(rest, 1.0F / 960.0F, position::Phase::PadIdle);
    f.gps_sigmas(10, 1.0F, 1.5F, h, v);
    CHECK(std::fabs(h - 2.5F) < 0.01F && std::fabs(v - 4.95F) < 0.01F);
    const float burn[3] = {0.0F, 0.0F, 36.0F * position::kGravity};   // 35 g net
    for (int i = 0; i < 960 * 3; ++i) f.predict(burn, 1.0F / 960.0F, position::Phase::Boost);
    const float net = 35.0F * position::kGravity;
    CHECK(std::fabs(f.gps_dynamics_stress_mps2() - net) < 1.0F);
    f.gps_sigmas(10, 1.0F, 1.5F, h, v);
    const float speed = f.speed();                               // about 1030 m/s
    const position::Config cfg{};
    const float la = cfg.gps_lag_accel_s2 * net;
    const float ls = cfg.gps_lag_speed_s * speed;
    const float expected = std::sqrt(2.5F * 2.5F + la * la + ls * ls);
    CHECK(std::fabs(h - std::fmin(expected, cfg.gps_max_sigma_m)) < 1.0F);
    CHECK(h > 150.0F);                                           // a lagging fix barely counts
    const float coast[3] = {0.0F, 0.0F, 0.0F};                   // ballistic: -1 g net
    for (int i = 0; i < 960 * 10; ++i) f.predict(coast, 1.0F / 960.0F, position::Phase::Coast);
    // Two time constants later the burn's share is down to ~e^-2 (plus 1 g).
    CHECK(f.gps_dynamics_stress_mps2() < 0.2F * net);
    CHECK(f.gps_dynamics_stress_mps2() >= position::kGravity * 0.99F);
}

// ---- 2026-11-15 fixes: no-attitude operation, NaN recovery, gaps, resume ----

// Runs the filter for `seconds` with the magnitude-only predict at 960 Hz, baro
// at 25 Hz and the 25 Hz constraint step, like Sensors does without attitude.
struct NoAttRig {
    Filter f;
    std::mt19937 rng{7};
    std::normal_distribution<float> unit{0.0F, 1.0F};
    double t = 0.0;
    int tick = 0;
    float baro_sigma = 0.6F;
    float true_agl = 0.0F;
    float true_vz = 0.0F;
    void step(const float specific_force_mps2, const Phase phase)
    {
        f.predict_magnitude_only(specific_force_mps2, 1.0F / 960.0F, phase);
        t += 1.0 / 960.0;
        if (++tick % 38 == 0) {   // ~25 Hz
            const float alt = kPadAltMsl + true_agl + baro_sigma * unit(rng);
            f.update_baro(alt, isa_pressure(alt), phase);
            f.update_constraints(phase);
            f.begin_update_cycle();
        }
    }
};

void test_no_attitude_pad_and_launch()
{
    // Pad: 8 s still. The pad hold works from |f| alone: AGL and vz pinned.
    NoAttRig r;
    for (int i = 0; i < 960 * 8; ++i) r.step(kG * (1.0F + 0.004F * r.unit(r.rng)), Phase::PadIdle);
    CHECK(r.f.initialized());
    CHECK(r.f.still());
    CHECK(std::fabs(r.f.up().p) < 0.3F && std::fabs(r.f.up().v) < 0.1F);
    CHECK(r.f.gravity_measured());
    CHECK(std::fabs(r.f.gravity_mps2() - kG) < 0.05F);
    CHECK(r.f.imu_ignored());

    // Launch: 4 g specific force (3 g net, straight up) for 3 s. The filter
    // has no acceleration, only the barometer.
    float t_launch_rule = -1.0F;
    float max_agl_error = 0.0F;
    const float a = 3.0F * kG;
    for (int i = 0; i < 960 * 4; ++i) {
        const float t = static_cast<float>(i) / 960.0F;
        r.true_agl = 0.5F * a * t * t;
        r.true_vz = a * t;
        r.step(4.0F * kG, Phase::PadIdle);
        if (t_launch_rule < 0.0F && r.f.up().p > 30.0F && r.f.up().v >= 30.0F) t_launch_rule = t;
        if (t > 1.0F) max_agl_error = std::fmax(max_agl_error, std::fabs(r.f.up().p - r.true_agl));
    }
    std::printf("no attitude, 3 g net launch: Kalman rule (AGL>30, vz>=30) true at t=%.2f s "
                "(truth: AGL 30 m at 1.43 s, vz 30 m/s at 1.02 s); max AGL error after 1 s %.1f m\n",
                static_cast<double>(t_launch_rule), static_cast<double>(max_agl_error));
    CHECK(t_launch_rule > 0.0F && t_launch_rule < 2.6F);
}

void test_no_attitude_pad_noise_never_looks_like_launch()
{
    // Not still (wind buffeting / handling: |f| 0.7..1.4 g) with a noisy
    // barometer (2 m): the zero-acceleration model must not fake 30 m/s.
    NoAttRig r;
    r.baro_sigma = 2.0F;
    float max_v = 0.0F, max_p = 0.0F;
    for (int i = 0; i < 960 * 120; ++i) {
        const float f = kG * (1.0F + 0.25F * r.unit(r.rng));
        r.step(f, Phase::PadIdle);
        max_v = std::fmax(max_v, std::fabs(r.f.up().v));
        max_p = std::fmax(max_p, std::fabs(r.f.up().p));
    }
    std::printf("no attitude, buffeting + 2 m baro noise, 120 s: max |vz| %.1f m/s, max |AGL| %.1f m\n",
                static_cast<double>(max_v), static_cast<double>(max_p));
    CHECK(max_v < 25.0F);   // pessimistic noise; the launch rule also needs AGL > 30 m
    CHECK(max_p < 10.0F);
}

void test_smooth_switch_to_imu_predict()
{
    NoAttRig r;
    for (int i = 0; i < 960 * 6; ++i) r.step(kG, Phase::PadIdle);
    const float before = r.f.up().p;
    const float up[3] = {0.0F, 0.0F, kG};   // attitude valid: 1 g up
    for (int i = 0; i < 960; ++i) r.f.predict(up, 1.0F / 960.0F, Phase::PadIdle);
    CHECK(!r.f.imu_ignored());
    CHECK(std::fabs(r.f.up().p - before) < 0.05F);   // no reset, no jump
    CHECK(r.f.divergence_resets() == 0);
}

void test_nan_recovery()
{
    NoAttRig r;
    for (int i = 0; i < 960 * 4; ++i) r.step(kG, Phase::PadIdle);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    // Non-finite inputs are rejected before they enter.
    const float bad[3] = {0.0F, nan, kG};
    r.f.predict(bad, 1.0F / 960.0F, Phase::PadIdle);
    r.f.predict(bad, nan, Phase::PadIdle);
    r.f.predict_magnitude_only(nan, 1.0F / 960.0F, Phase::PadIdle);
    r.f.predict_without_imu(nan, Phase::PadIdle);
    CHECK(!r.f.update_baro(nan, 101325.0F, Phase::PadIdle));
    CHECK(!r.f.update_baro(620.0F, nan, Phase::PadIdle));
    CHECK(!r.f.update_gps(nan, 1.0, 100.0F, 8, Phase::PadIdle));
    CHECK(!r.f.update_gps(35.0, -117.0, nan, 8, Phase::PadIdle));
    CHECK(r.f.rejected_inputs() >= 8);
    CHECK(std::isfinite(r.f.up().p) && std::isfinite(r.f.up().v) && std::isfinite(r.f.up().P00));
    CHECK(r.f.divergence_resets() == 0);

    // A finite but absurd acceleration drives the state to garbage: the health
    // check resets the filter (pad references kept) and the pad hold recovers.
    Filter g;
    const float zero[3] = {0.0F, 0.0F, kG};
    for (int i = 0; i < 960 * 3; ++i) g.predict(zero, 1.0F / 960.0F, Phase::PadIdle);
    float alt = kPadAltMsl;
    g.update_baro(alt, isa_pressure(alt), Phase::PadIdle);
    position::PadReferences before;
    g.export_pad_references(before);
    CHECK(before.baro_valid);
    const float huge[3] = {0.0F, 0.0F, 3.0e38F};
    g.predict(huge, 1.0F / 960.0F, Phase::PadIdle);
    CHECK(g.divergence_resets() >= 1);
    CHECK(std::isfinite(g.up().p) && std::isfinite(g.up().P00) && std::isfinite(g.east().P11));
    position::PadReferences after;
    g.export_pad_references(after);
    CHECK(after.baro_valid && after.baro_reference_m == before.baro_reference_m);
    for (int i = 0; i < 960 * 6; ++i) {
        g.predict(zero, 1.0F / 960.0F, Phase::PadIdle);
        if (i % 38 == 0) {
            g.update_baro(alt, isa_pressure(alt), Phase::PadIdle);
            g.update_constraints(Phase::PadIdle);
        }
    }
    CHECK(std::fabs(g.up().p) < 0.5F && std::fabs(g.up().v) < 0.2F);
}

void test_predict_gap_bridged()
{
    Filter f;
    position::PadReferences refs;
    refs.motion_valid = true;
    refs.agl_m = 100.0F;
    refs.vertical_velocity_mps = 50.0F;
    f.restore_after_reset(refs);
    CHECK(f.up().p == 100.0F && f.up().v == 50.0F);
    CHECK(f.divergence_resets() == 0);
    const float p00_before = f.up().P00;
    const float a[3] = {0.0F, 0.0F, kG};
    f.predict(a, 0.5F, Phase::Boost);   // a 500 ms stall: used to freeze
    CHECK(std::fabs(f.up().p - 125.0F) < 0.01F);
    CHECK(f.up().P00 > p00_before);
    CHECK(f.predict_gaps() == 1);
    f.predict_without_imu(0.3F, Phase::Boost);
    CHECK(std::fabs(f.up().p - 140.0F) < 0.01F);
    CHECK(f.predict_gaps() == 2);
}

void test_restore_after_reset_keeps_agl_continuous()
{
    // Pad, then fly (truth known), persist the references mid-descent, "reset",
    // restore into a fresh filter and let the barometer correct it.
    Filter pad;
    const float up[3] = {0.0F, 0.0F, kG};
    std::mt19937 rng(3);
    std::normal_distribution<float> unit(0.0F, 1.0F);
    for (int i = 0; i < 960 * 6; ++i) {
        pad.predict(up, 1.0F / 960.0F, Phase::PadIdle);
        if (i % 38 == 0) {
            const float alt = kPadAltMsl + 0.5F * unit(rng);
            pad.update_baro(alt, isa_pressure(alt), Phase::PadIdle);
            pad.update_gps(kPadLat, kPadLon, kPadAltMsl + 1.0F, 10, Phase::PadIdle);
            pad.update_constraints(Phase::PadIdle);
        }
    }
    position::PadReferences refs;
    pad.export_pad_references(refs);
    CHECK(refs.baro_valid && refs.gps_valid && refs.gravity_valid);
    // Last persisted motion: 600 m AGL, descending 30 m/s (data ~2 s old by now).
    refs.motion_valid = true;
    refs.agl_m = 600.0F;
    refs.vertical_velocity_mps = -30.0F;

    Filter g;
    g.restore_after_reset(refs);
    CHECK(g.initialized() && g.gps_origin_set());
    CHECK(std::fabs(g.gravity_mps2() - pad.gravity_mps2()) < 1e-4F && g.gravity_frozen());
    // Truth: 540 m AGL now and 25 m/s down (the descent slowed).
    float truth = 540.0F;
    float worst_after_2s = 0.0F;
    for (int i = 0; i < 25 * 6; ++i) {
        g.predict_magnitude_only(kG, 1.0F / 25.0F, Phase::Descent);
        truth -= 25.0F / 25.0F;
        const float alt = kPadAltMsl + truth + 0.6F * unit(rng);
        g.update_baro(alt, isa_pressure(alt), Phase::Descent);
        g.begin_update_cycle();
        if (i >= 50) worst_after_2s = std::fmax(worst_after_2s, std::fabs(g.up().p - truth));
    }
    std::printf("restore after reset (60 m, 5 m/s stale): AGL error after 2 s <= %.2f m, vz %.1f (truth -25)\n",
                static_cast<double>(worst_after_2s), static_cast<double>(g.up().v));
    CHECK(worst_after_2s < 3.0F);
    CHECK(std::fabs(g.up().v + 25.0F) < 5.0F);
    // GPS in the restored frame lands on the pad (E/N start unknown, then pulled in).
    for (int i = 0; i < 20; ++i) {
        g.update_gps(kPadLat, kPadLon, kPadAltMsl + 1.0F + truth, 10, Phase::Descent);
    }
    CHECK(std::fabs(g.east().p) < 5.0F && std::fabs(g.north().p) < 5.0F);
}

void test_baro_rate()
{
    position::BaroRate r;
    std::mt19937 rng(5);
    std::normal_distribution<float> unit(0.0F, 1.0F);
    float worst_still = 0.0F;
    std::uint64_t t = 1'000'000;
    for (int i = 0; i < 25 * 30; ++i, t += 40'000) {   // 30 s on the pad, 25 Hz
        r.add(t, 620.0F + 0.8F * unit(rng));
        if (r.valid(t)) worst_still = std::fmax(worst_still, std::fabs(r.rate_mps()));
    }
    std::printf("baro rate on the pad: worst |rate| %.2f m/s\n", static_cast<double>(worst_still));
    CHECK(worst_still < 2.0F);
    // Descent at 25 m/s.
    float alt = 3000.0F;
    for (int i = 0; i < 25 * 3; ++i, t += 40'000) {
        alt -= 25.0F * 0.04F;
        r.add(t, alt + 0.8F * unit(rng));
    }
    CHECK(r.valid(t) && std::fabs(r.rate_mps() + 25.0F) < 2.0F);
    CHECK(!r.valid(t + 600'000));                       // stale
    r.add(t + 3'000'000, 100.0F);                        // a 3 s gap: not valid
    CHECK(!r.valid(t + 3'000'000));
    r.add(t + 3'050'000, std::numeric_limits<float>::quiet_NaN());   // ignored
}

int main()
{
    test_no_attitude_pad_and_launch();
    test_no_attitude_pad_noise_never_looks_like_launch();
    test_smooth_switch_to_imu_predict();
    test_nan_recovery();
    test_predict_gap_bridged();
    test_restore_after_reset_keeps_agl_continuous();
    test_baro_rate();
    test_repad_after_flight();
    test_pad_measured_gravity();
    test_gps_dynamics_sigmas();
    test_gps_dop_sigmas();
    test_atmosphere_and_baro_sigma();
    test_pad_hold();
    test_flights();
    test_gate_and_resync();
    test_predict_without_imu();
    test_unaided_velocity_decays();
    test_unaided_tracking_and_rest_hold();
    test_baro_thrust_gate();
    test_innovation_record();
    test_baro_bias_state();
    if (failures == 0) {
        std::printf("position_filter_test: all passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("position_filter_test: %d failure(s)\n", failures);
    return EXIT_FAILURE;
}
