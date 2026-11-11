#include "position_filter.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
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
    CHECK(s0 > 0.8F && s0 < 1.1F);
    CHECK(s10 > 2.2F && s10 < 3.0F);
    CHECK(s20 > 10.0F && s20 < 13.0F);
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
    CHECK(r.max_up_error < 40.0F);           // a 150 m baro error never entered
    CHECK(r.up_error_at_apogee < 15.0F);
    CHECK(r.speed_error_at_boost < 3.0F);    // ~32 m/s before Boost was integrated, not lost

    // With a 10 Hz and a 25 Hz GPS.
    for (const int rate : {10, 25}) {
        FlightResult g = fly(rate, 150.0F, 0.2F, 2);
        std::printf("GPS %2d Hz:  max |up error| %.1f m, at apogee %.1f m, max |east error| "
                    "%.1f m, gps used %u\n", rate, g.max_up_error, g.up_error_at_apogee,
                    g.max_horizontal_error, static_cast<unsigned>(g.gps_used));
        // This GPS has no lag, but the filter now down-weights GPS under
        // acceleration and at speed (real receivers lag there: HITL RF runs
        // showed 50-80 m), so the boost relies more on the IMU.
        CHECK(g.max_up_error < 30.0F);
        CHECK(g.up_error_at_apogee < 10.0F);
        CHECK(g.max_horizontal_error < 8.0F);  // ~2.5 sigma of 2.5 m GPS noise
        CHECK(g.gps_used > 100);
    }
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
    filter.gps_sigmas(10, 900.0F, 900.0F, h, v);  // and the maximum
    CHECK(h == 1000.0F && v == 1000.0F);

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
    const float expected = std::sqrt(2.5F * 2.5F + 0.25F * net * net + 0.01F * speed * speed);
    CHECK(std::fabs(h - std::fmin(expected, 1000.0F)) < 1.0F);
    CHECK(h > 150.0F);                                           // a lagging fix barely counts
    const float coast[3] = {0.0F, 0.0F, 0.0F};                   // ballistic: -1 g net
    for (int i = 0; i < 960 * 10; ++i) f.predict(coast, 1.0F / 960.0F, position::Phase::Coast);
    // Two time constants later the burn's share is down to ~e^-2 (plus 1 g).
    CHECK(f.gps_dynamics_stress_mps2() < 0.2F * net);
    CHECK(f.gps_dynamics_stress_mps2() >= position::kGravity * 0.99F);
}

int main()
{
    test_gps_dynamics_sigmas();
    test_gps_dop_sigmas();
    test_atmosphere_and_baro_sigma();
    test_pad_hold();
    test_flights();
    test_gate_and_resync();
    test_predict_without_imu();
    test_innovation_record();
    if (failures == 0) {
        std::printf("position_filter_test: all passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("position_filter_test: %d failure(s)\n", failures);
    return EXIT_FAILURE;
}
