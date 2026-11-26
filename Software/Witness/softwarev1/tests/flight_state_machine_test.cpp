// Host test: Kalman-AGL launch / Descent / landing logic, HITL re-baseline and
// the reset-resume record. Build command: see FIRMWARE_HANDOFF.md
// ("Flight state across resets").
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>

#include "flight_state_machine.h"
#include "flight_state_store.h"
#include "witness_status.h"

namespace {
int failures = 0;
#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::printf("FAIL line %d: %s\n", __LINE__, #cond);              \
            ++failures;                                                      \
        }                                                                    \
    } while (0)

using State = FlightStateMachine::State;

struct Rig {
    WitnessStatus status;
    FlightStateMachine fsm{status};
    std::uint64_t t_us = 1'000'000;
    float axial_g = 1.0F;          // low-g axial acceleration (burnout test)
    bool still = false;            // 1 g, no rotation (landing test)
    bool unaided = false;          // nothing measures the vertical motion
    float unaided_vz = 0.0F;
    float unaided_s = 0.0F;        // advanced by step() while unaided
    // One observation every 20 ms (50 Hz): Kalman AGL and vertical velocity.
    void step(float agl, float vel = 0.0F, bool agl_valid = true)
    {
        FlightStateMachine::Observation o{};
        o.agl_valid = agl_valid;
        o.agl_meters = agl;
        o.vertical_velocity_valid = true;
        o.vertical_velocity_mps = vel;
        o.low_g_axial_acceleration_valid = true;
        o.low_g_axial_acceleration_g = axial_g;
        o.acceleration_valid = true;
        o.angular_rate_valid = true;
        o.acceleration_g[0] = still ? 1.0F : 3.0F;
        o.angular_rate_dps[2] = still ? 0.0F : 50.0F;
        o.vertical_unaided = unaided;
        o.unaided_entry_vz_mps = unaided_vz;
        o.unaided_elapsed_s = unaided ? unaided_s : 0.0F;
        o.timestamp_us = t_us;
        fsm.update(o);
        t_us += 20'000;
        if (unaided) unaided_s += 0.02F;
    }
    void run(float agl, int n, float vel = 0.0F)
    {
        for (int i = 0; i < n; ++i) step(agl, vel);
    }
    void to_coast(float agl)
    {
        run(0.0F, 10);
        CHECK(fsm.trigger_launch() == ESP_OK);
        run(agl, 3, 100.0F);
        fsm.set_state(static_cast<std::uint8_t>(State::Coast));
        run(agl, 3, 100.0F);
    }
};

int g_saved_state = -1;
void persist_cb(std::uint8_t s, float, bool) { g_saved_state = s; }

void test_launch_needs_30m_agl_held()
{
    Rig r;
    r.run(0.0F, 100);
    r.run(29.5F, 200, 10.0F);                // just below 30 m: never
    CHECK(r.fsm.state() == State::PadIdle);
    r.run(40.0F, 10, 50.0F);                 // 200 ms above, then a dip
    r.run(20.0F, 1, 50.0F);
    CHECK(r.fsm.state() == State::PadIdle);
    int steps = -1;
    for (int i = 0; i < 50; ++i) {
        r.step(40.0F + i, 60.0F);
        if (r.fsm.state() == State::Boost) { steps = i + 1; break; }
    }
    CHECK(r.fsm.state() == State::Boost);
    std::printf("launch after %d ms above 30 m AGL\n", steps * 20);
    CHECK(steps * 20 >= 300 && steps * 20 <= 340);
}

void test_launch_needs_30mps_vertical_velocity()
{
    // A slow climb (vehicle, hoist, aircraft) to 150 m at 10 m/s never launches,
    // even far above the 100 m where a rocket is long past 30 m/s.
    Rig r;
    r.run(0.0F, 100);
    for (int i = 0; i < 750; ++i) {           // 15 s at 10 m/s: 0 -> 150 m
        r.step(0.2F * static_cast<float>(i), 10.0F);
    }
    CHECK(r.fsm.state() == State::PadIdle);
    r.run(150.0F, 500, 29.0F);                // above 30 m AGL, 29 m/s: still no
    CHECK(r.fsm.state() == State::PadIdle);
    r.run(150.0F, 500, 0.0F);
    CHECK(r.fsm.state() == State::PadIdle);
    // Invalid vertical velocity is not a launch either.
    Rig q;
    for (int i = 0; i < 100; ++i) {
        FlightStateMachine::Observation o{};
        o.agl_valid = true;
        o.agl_meters = 200.0F;
        o.vertical_velocity_valid = false;
        o.vertical_velocity_mps = 100.0F;
        o.timestamp_us = q.t_us;
        q.t_us += 20'000;
        q.fsm.update(o);
    }
    CHECK(q.fsm.state() == State::PadIdle);
    // Both conditions must hold continuously for 300 ms: velocity dipping
    // below 30 m/s restarts the window.
    Rig s;
    s.run(0.0F, 50);
    s.run(40.0F, 10, 50.0F);                  // 200 ms good
    s.run(45.0F, 1, 25.0F);                   // velocity dip
    s.run(50.0F, 10, 50.0F);                  // another 200 ms good
    CHECK(s.fsm.state() == State::PadIdle);
    s.run(55.0F, 10, 50.0F);                  // now over 300 ms continuous
    CHECK(s.fsm.state() == State::Boost);
    // Exactly 30 m/s qualifies (>=).
    Rig u;
    u.run(0.0F, 50);
    u.run(40.0F, 40, 30.0F);
    CHECK(u.fsm.state() == State::Boost);
}

void test_commanded_launch_ignores_velocity()
{
    Rig r;
    r.run(0.0F, 10);
    CHECK(r.fsm.trigger_launch() == ESP_OK);
    CHECK(r.fsm.state() == State::Boost);
}

void test_states_are_one_way()
{
    Rig r;
    r.fsm.set_persist_callback(persist_cb);
    CHECK(r.fsm.set_state(static_cast<std::uint8_t>(State::PadIdle)) == ESP_OK);   // same state
    CHECK(r.fsm.set_state(9) == ESP_ERR_INVALID_ARG);
    CHECK(r.fsm.set_state(static_cast<std::uint8_t>(State::Coast)) == ESP_OK);     // forward jump
    CHECK(r.fsm.state() == State::Coast);
    g_saved_state = -1;
    for (const std::uint8_t back : {0, 1}) {
        CHECK(r.fsm.set_state(back) == ESP_ERR_INVALID_STATE);
        CHECK(r.fsm.state() == State::Coast);
        CHECK(r.status.flight_state() == 2);
        CHECK(g_saved_state == -1);            // nothing persisted by a refusal
    }
    CHECK(r.fsm.set_state(static_cast<std::uint8_t>(State::Coast)) == ESP_OK);     // same: no-op
    CHECK(r.fsm.trigger_launch() == ESP_ERR_INVALID_STATE);
    CHECK(r.fsm.set_state(static_cast<std::uint8_t>(State::Landed)) == ESP_OK);
    for (std::uint8_t back = 0; back <= 3; ++back) {
        CHECK(r.fsm.set_state(back) == ESP_ERR_INVALID_STATE);
        CHECK(r.fsm.state() == State::Landed);
    }
    CHECK(r.status.flight_state() == 4);
    // Boot restore is a separate path and may set any state.
    r.fsm.restore(State::Descent, 100.0F, true);
    CHECK(r.fsm.state() == State::Descent);
    // HITL re-baseline never moves the state.
    r.fsm.rebaseline();
    r.run(0.0F, 20);
    CHECK(r.fsm.state() == State::Descent);
}

void test_invalid_agl_no_launch()
{
    Rig r;
    for (int i = 0; i < 100; ++i) r.step(500.0F, 100.0F, false);
    CHECK(r.fsm.state() == State::PadIdle);
    for (int i = 0; i < 100; ++i) r.step(NAN, 100.0F, true);
    CHECK(r.fsm.state() == State::PadIdle);
}

void test_burnout_then_kalman_descent()
{
    Rig r;
    r.run(0.0F, 10);
    r.run(50.0F, 20, 80.0F);
    CHECK(r.fsm.state() == State::Boost);
    r.axial_g = 0.2F;                          // motor out
    r.run(400.0F, 30, 150.0F);
    CHECK(r.fsm.state() == State::Coast);
    for (int i = 0; i < 200; ++i) r.step(400.0F + 10.0F * i, 120.0F);   // climb to 2390
    CHECK(r.fsm.state() == State::Coast);
    CHECK(r.fsm.maximum_altitude() > 2380.0F);
    r.run(2300.0F, 100, 5.0F);                 // 4 % below but still rising
    CHECK(r.fsm.state() == State::Coast);
    int steps = -1;
    for (int i = 0; i < 100; ++i) {
        r.step(2200.0F, -20.0F);               // 8 % below max, falling
        if (r.fsm.state() == State::Descent) { steps = i + 1; break; }
    }
    CHECK(r.fsm.state() == State::Descent);
    std::printf("descent after %d ms\n", steps * 20);
    CHECK(steps * 20 <= 540);
}

void test_burnout_from_velocity()
{
    // Axial burnout missed (low-g axis reads > 1 g all flight): the Kalman
    // velocity falling 10 m/s below its Boost peak for 1 s ends Boost.
    Rig r;
    r.run(0.0F, 10);
    CHECK(r.fsm.trigger_launch() == ESP_OK);
    r.axial_g = 3.0F;
    for (int i = 0; i < 50; ++i) r.step(10.0F * i, 5.0F * i);        // to 245 m/s
    CHECK(r.fsm.state() == State::Boost);
    // Slight dip (5 m/s): not enough.
    r.run(600.0F, 100, 240.0F);
    CHECK(r.fsm.state() == State::Boost);
    // Decelerating at ~1.5 g: 10 m/s below the peak after ~0.7 s, then 1 s.
    int coast_ms = -1;
    for (int i = 0; i < 200; ++i) {
        r.step(700.0F + i, 245.0F - 0.3F * i);
        if (r.fsm.state() == State::Coast) { coast_ms = (i + 1) * 20; break; }
    }
    std::printf("velocity burnout: Coast %d ms after the slowdown began\n", coast_ms);
    CHECK(r.fsm.state() == State::Coast);
    CHECK(coast_ms > 1000 && coast_ms < 1800);
    // A slow (never > 20 m/s) climb never uses this path.
    Rig s;
    s.run(0.0F, 10);
    CHECK(s.fsm.trigger_launch() == ESP_OK);
    s.axial_g = 3.0F;
    s.run(10.0F, 20, 15.0F);
    s.run(10.0F, 200, 0.0F);
    CHECK(s.fsm.state() == State::Boost);
}

void test_ballistic_descent_unaided()
{
    // Flight 2026-11-21 replay: gyro saturated just after apogee (vz -2.2 m/s),
    // no barometer, no GPS. The Kalman velocity decays to 0 and holds the AGL,
    // so only the ballistic bound can end Coast: max(vz, 0) / g + 1 s.
    Rig r;
    r.to_coast(2700.0F);
    r.unaided = true;
    r.unaided_vz = -2.2F;
    int ms = -1;
    for (int i = 0; i < 500 && ms < 0; ++i) {
        r.step(2700.0F, -2.2F * std::exp(-0.02F * i / 2.0F));
        if (r.fsm.state() == State::Descent) ms = (i + 1) * 20;
    }
    std::printf("ballistic descent (vz -2.2 m/s at loss): %d ms\n", ms);
    CHECK(ms >= 1000 && ms <= 1100);

    // Lost while still climbing at 80 m/s: not before 80 / g + 1 = 9.2 s.
    Rig s;
    s.to_coast(1500.0F);
    s.unaided = true;
    s.unaided_vz = 80.0F;
    s.run(1500.0F, 450, 0.0F);                 // 9.0 s
    CHECK(s.fsm.state() == State::Coast);
    s.run(1500.0F, 20, 0.0F);                  // 9.4 s
    CHECK(s.fsm.state() == State::Descent);

    // Aided again (unaided false): the bound never applies.
    Rig q;
    q.to_coast(1500.0F);
    q.unaided_vz = -5.0F;
    q.run(1500.0F, 500, 0.0F);
    CHECK(q.fsm.state() == State::Coast);
}

void test_low_maximum_needs_10m_drop()
{
    // Bench: commanded Boost/Coast at the pad (maximum ~0 m): no Descent from
    // AGL noise.
    Rig r;
    r.to_coast(0.1F);
    r.run(-0.1F, 200, -0.2F);
    r.run(-5.0F, 200, -1.0F);
    CHECK(r.fsm.state() == State::Coast);
    // A 60 m hop: 5 % would be 3 m; 10 m is needed.
    Rig s;
    s.to_coast(60.0F);
    s.run(55.0F, 100, -3.0F);
    CHECK(s.fsm.state() == State::Coast);
    s.run(49.0F, 30, -3.0F);
    CHECK(s.fsm.state() == State::Descent);
}

void test_velocity_path()
{
    Rig r;
    r.to_coast(10000.0F);
    r.run(10000.0F, 200, +150.0F);             // 4 s of climb after burnout
    r.fsm.rebaseline();                        // maximum lost (e.g. resumed)
    int steps = -1;
    for (int i = 0; i < 400; ++i) {
        r.step(12000.0F, -80.0F);
        if (r.fsm.state() == State::Descent) { steps = i + 1; break; }
    }
    CHECK(r.fsm.state() == State::Descent);
    std::printf("velocity path descent after %d ms\n", steps * 20);
}

void test_velocity_transient_no_descent()
{
    Rig r;
    r.to_coast(10000.0F);
    r.run(10000.0F, 200, +150.0F);
    r.run(10000.0F, 80, -80.0F);               // 1.6 s negative < 2 s dwell, not 5 % low
    r.run(10000.0F, 100, +100.0F);
    r.run(10000.0F, 80, -80.0F);
    CHECK(r.fsm.state() == State::Coast);
    // Negative velocity with no climb ever seen (glitch near burnout).
    Rig q;
    q.run(0.0F, 10);
    CHECK(q.fsm.trigger_launch() == ESP_OK);
    q.run(100.0F, 3, 5.0F);                    // never faster than 20 m/s up
    q.fsm.set_state(static_cast<std::uint8_t>(State::Coast));
    q.fsm.rebaseline();
    q.run(100.0F, 400, -80.0F);
    CHECK(q.fsm.state() == State::Coast);
}

void test_landing_on_agl()
{
    Rig r;
    r.to_coast(1000.0F);
    r.fsm.set_state(static_cast<std::uint8_t>(State::Descent));
    r.run(500.0F, 10, -5.0F);
    r.still = true;
    r.run(3.0F, 1000, 0.0F);                  // 20 s still: not yet
    CHECK(r.fsm.state() == State::Descent);
    r.run(3.0F, 600, 0.0F);                   // 32 s total
    CHECK(r.fsm.state() == State::Landed);
    // AGL drifting more than 10 m restarts the window.
    Rig s;
    s.to_coast(1000.0F);
    s.fsm.set_state(static_cast<std::uint8_t>(State::Descent));
    s.still = true;
    for (int i = 0; i < 1600; ++i) s.step(100.0F - 0.02F * i, -1.0F);   // 32 m over 32 s
    CHECK(s.fsm.state() == State::Descent);
}

void test_hitl_exit_rebaseline()
{
    Rig r;
    r.to_coast(34800.0F);
    r.run(34800.0F, 20, 50.0F);
    r.fsm.rebaseline();                       // HITL exit
    r.run(0.0F, 200, 0.0F);                   // bench: back at the pad
    CHECK(r.fsm.state() == State::Coast);
    CHECK(r.fsm.maximum_altitude() < 1.0F);
}

void test_persist_callback_and_restore()
{
    Rig r;
    r.fsm.set_persist_callback(persist_cb);
    r.run(0.0F, 10);
    r.fsm.trigger_launch();
    CHECK(g_saved_state == 1);
    r.fsm.restore(State::Coast, 5000.0F, true);
    CHECK(r.fsm.state() == State::Coast);
    CHECK(r.status.flight_state() == 2);
    r.run(5100.0F, 10, 10.0F);                 // restored max kept, raised
    CHECK(r.fsm.maximum_altitude() >= 5100.0F);
    r.run(5000.0F, 200, -10.0F);
    CHECK(r.fsm.state() == State::Coast);     // 5000 is not 5 % below 5100
    r.run(4000.0F, 100, -10.0F);
    CHECK(r.fsm.state() == State::Descent);
}

void test_store_record()
{
    using namespace flight_state_store;
    Record rec{};
    CHECK(!valid(rec));
    seal(rec, 2, 1234.5F, true);
    CHECK(valid(rec));
    CHECK(resumable(rec, ResetKind::Warm));
    CHECK(!resumable(rec, ResetKind::Cold));
    rec.maximum_altitude_m += 1.0F;           // corruption
    CHECK(!valid(rec));
    seal(rec, 0, 0, false);
    CHECK(!resumable(rec, ResetKind::Warm));  // Pad Idle
    seal(rec, 4, 0, false);
    CHECK(!resumable(rec, ResetKind::Warm));  // Landed
    for (std::uint8_t s = 1; s <= 3; ++s) {
        seal(rec, s, 10, true);
        CHECK(resumable(rec, ResetKind::Warm));
    }
    seal(rec, 9, 10, true);
    CHECK(!resumable(rec, ResetKind::Warm));
    Record junk{0x12345678U, 2, 1.0F, 1, 0xDEADBEEFU};
    CHECK(!resumable(junk, ResetKind::Warm));
}
// ---- 2026-11-15 fixes: acceleration trigger, airborne-at-boot, false flight, store ----

// Observation without any filter output: only the accelerometer and (optional)
// raw barometer rate, as when the attitude never became valid.
struct RawRig {
    WitnessStatus status;
    FlightStateMachine fsm{status};
    std::uint64_t t_us = 1'000'000;
    void step(float axial_g, bool axial_valid = true, bool rate_valid = false, float rate = 0.0F,
              std::uint32_t dt_us = 2'000)
    {
        FlightStateMachine::Observation o{};
        o.axial_acceleration_valid = axial_valid;
        o.axial_acceleration_g = axial_g;
        o.low_g_axial_acceleration_valid = axial_valid;
        o.low_g_axial_acceleration_g = axial_g;
        o.baro_rate_valid = rate_valid;
        o.baro_rate_mps = rate;
        o.timestamp_us = t_us;
        fsm.update(o);
        t_us += dt_us;
    }
    int run_until_boost(float axial_g, int max_steps)
    {
        for (int i = 0; i < max_steps; ++i) {
            step(axial_g);
            if (fsm.state() != State::PadIdle) return i + 1;
        }
        return -1;
    }
};

void test_acceleration_launch()
{
    // 1 g pad, then a 3 g motor: Boost 300 ms later, with no AGL, no velocity,
    // no attitude, no filter at all.
    RawRig r;
    for (int i = 0; i < 500; ++i) r.step(1.0F);
    CHECK(r.fsm.state() == State::PadIdle);
    const int steps = r.run_until_boost(3.0F, 400);
    std::printf("acceleration launch: Boost after %d ms at 3 g\n", steps * 2);
    CHECK(steps > 0 && steps * 2 >= 300 && steps * 2 <= 304);
    // Exactly the threshold qualifies; just below never does.
    RawRig a;
    CHECK(a.run_until_boost(FlightStateMachine::kLaunchAccelerationG, 400) > 0);
    RawRig b;
    CHECK(b.run_until_boost(FlightStateMachine::kLaunchAccelerationG - 0.05F, 5000) == -1);
    // Invalid axial channel never launches.
    RawRig c;
    for (int i = 0; i < 1000; ++i) c.step(8.0F, false);
    CHECK(c.fsm.state() == State::PadIdle);
    // A motor that chuffs (single-sample dips every 50 ms) still qualifies.
    RawRig d;
    int boost_ms = -1;
    for (int i = 0; i < 600; ++i) {
        d.step(i % 25 == 24 ? 1.5F : 4.0F);
        if (d.fsm.state() == State::Boost) { boost_ms = (i + 1) * 2; break; }
    }
    CHECK(boost_ms > 0 && boost_ms < 450);
}

void test_acceleration_launch_ignores_handling()
{
    // Dropped / bumped / carried: sharp shocks, 30 g for 10 ms every 100 ms for
    // a minute, never a continuous 2.5 g for 300 ms.
    RawRig r;
    for (int i = 0; i < 30000; ++i) r.step(i % 50 < 5 ? 30.0F : 1.0F);
    CHECK(r.fsm.state() == State::PadIdle);
    // Shaking: 3 g peaks 40 % of the time (drain 2x fill): never accumulates.
    RawRig s;
    for (int i = 0; i < 30000; ++i) s.step(i % 10 < 4 ? 3.0F : 0.2F);
    CHECK(s.fsm.state() == State::PadIdle);
    // A 250 ms hand shove at 4 g is under the 300 ms.
    RawRig h;
    for (int i = 0; i < 100; ++i) h.step(1.0F);
    for (int i = 0; i < 125; ++i) h.step(4.0F);
    for (int i = 0; i < 500; ++i) h.step(1.0F);
    CHECK(h.fsm.state() == State::PadIdle);
    // A long gap in the observations credits at most 50 ms.
    RawRig g;
    g.step(4.0F);
    g.step(4.0F, true, false, 0.0F, 1'000'000);
    g.step(4.0F);
    CHECK(g.fsm.state() == State::PadIdle);
}

void test_airborne_at_boot()
{
    // Ascent by the barometer alone (cold reset in Coast, no thrust).
    RawRig up;
    for (int i = 0; i < 500; ++i) up.step(0.3F, true, true, 0.5F);
    CHECK(up.fsm.state() == State::PadIdle);
    int boost_ms = -1;
    for (int i = 0; i < 1000; ++i) {
        up.step(0.3F, true, true, 60.0F);
        if (up.fsm.state() == State::Boost) { boost_ms = (i + 1) * 2; break; }
    }
    std::printf("airborne at boot: barometer 60 m/s up -> Boost after %d ms\n", boost_ms);
    CHECK(boost_ms >= 1000 && boost_ms <= 1004);
    CHECK(up.fsm.flight_evidence());
    // Descent under the drogue.
    RawRig down;
    for (int i = 0; i < 1000; ++i) {
        down.step(1.0F, true, true, -25.0F);
        if (down.fsm.state() != State::PadIdle) break;
    }
    CHECK(down.fsm.state() == State::Descent);
    // Slower than 20 m/s, invalid, stale-dropped or interrupted: nothing.
    RawRig slow;
    for (int i = 0; i < 5000; ++i) slow.step(1.0F, true, true, -19.0F);
    CHECK(slow.fsm.state() == State::PadIdle);
    RawRig inv;
    for (int i = 0; i < 5000; ++i) inv.step(1.0F, true, false, -90.0F);
    CHECK(inv.fsm.state() == State::PadIdle);
    RawRig brk;
    for (int round = 0; round < 10; ++round) {
        for (int i = 0; i < 400; ++i) brk.step(1.0F, true, true, 40.0F);   // 800 ms
        brk.step(1.0F, true, true, 0.0F);
    }
    CHECK(brk.fsm.state() == State::PadIdle);
}

int g_false_flight_calls = 0;
void false_flight_cb() { ++g_false_flight_calls; }

// Boost (commanded), then 31 s still on the ground: the landing test fires.
void fly_to_landed(Rig &r, float agl_during, float axial_peak_g)
{
    r.run(0.0F, 10);
    CHECK(r.fsm.trigger_launch() == ESP_OK);
    r.still = true;
    for (int i = 0; i < 1600; ++i) {
        FlightStateMachine::Observation o{};
        o.agl_valid = true;
        o.agl_meters = agl_during;
        o.vertical_velocity_valid = true;
        o.vertical_velocity_mps = 0.0F;
        o.axial_acceleration_valid = true;
        o.axial_acceleration_g = i == 5 ? axial_peak_g : 1.0F;
        o.low_g_axial_acceleration_valid = true;
        o.low_g_axial_acceleration_g = 1.0F;
        o.acceleration_valid = true;
        o.angular_rate_valid = true;
        o.acceleration_g[0] = 1.0F;
        o.timestamp_us = r.t_us;
        r.t_us += 20'000;
        r.fsm.update(o);
    }
}

void test_false_flight_callback()
{
    // The restart is OFF by the owner's decision of 2026-11-16: with no
    // evidence the flight still ends in Landed (and is known not to be real),
    // but the callback is never called.
    CHECK(!FlightStateMachine::kFalseFlightRestartEnabled);
    g_false_flight_calls = 0;
    Rig r;
    r.fsm.set_false_flight_callback(false_flight_cb);
    fly_to_landed(r, 0.0F, 1.2F);
    CHECK(r.fsm.state() == State::Landed);
    CHECK(g_false_flight_calls == 0);
    CHECK(!r.fsm.flight_evidence());
    // 60 m of AGL: a real (short) flight, no callback.
    g_false_flight_calls = 0;
    Rig a;
    a.fsm.set_false_flight_callback(false_flight_cb);
    fly_to_landed(a, 60.0F, 1.2F);
    CHECK(a.fsm.state() == State::Landed && g_false_flight_calls == 0);
    CHECK(a.fsm.flight_evidence());
    // 3.5 g of axial acceleration in flight, no AGL: real.
    Rig b;
    b.fsm.set_false_flight_callback(false_flight_cb);
    fly_to_landed(b, 0.0F, 3.5F);
    CHECK(b.fsm.state() == State::Landed && g_false_flight_calls == 0);
    // A resumed flight is real. No callback set: Landed, no reset request.
    Rig c;
    c.fsm.set_false_flight_callback(false_flight_cb);
    c.fsm.restore(State::Descent, 0.0F, false);
    c.still = true;
    for (int i = 0; i < 1600; ++i) c.step(0.0F);
    CHECK(c.fsm.state() == State::Landed && g_false_flight_calls == 0);
    Rig d;
    fly_to_landed(d, 0.0F, 1.2F);              // no callback registered
    CHECK(d.fsm.state() == State::Landed && g_false_flight_calls == 0);
}

void test_store_extension_record()
{
    using namespace flight_state_store;
    PadData d;
    d.baro_valid = true;
    d.baro_reference_m = 1234.5F;
    d.gps_valid = true;
    d.gps_origin_lat_deg = 35.352718345;
    d.gps_origin_lon_deg = -117.810123456;
    d.gps_origin_alt_m = 1230.25F;
    d.gravity_valid = true;
    d.gravity_mps2 = 9.8123F;
    d.motion_valid = true;
    d.agl_m = 2345.5F;
    d.vertical_velocity_mps = -31.5F;
    Extension e{};
    CHECK(!extension_valid(e));                       // zeroed
    seal_extension(e, d);
    CHECK(extension_valid(e));
    const PadData u = unpack_extension(e);
    CHECK(u.baro_valid && u.gps_valid && u.gravity_valid && u.motion_valid);
    CHECK(u.baro_reference_m == d.baro_reference_m && u.gps_origin_alt_m == d.gps_origin_alt_m);
    CHECK(u.gps_origin_lat_deg == d.gps_origin_lat_deg && u.gps_origin_lon_deg == d.gps_origin_lon_deg);
    CHECK(u.gravity_mps2 == d.gravity_mps2 && u.agl_m == d.agl_m &&
          u.vertical_velocity_mps == d.vertical_velocity_mps);
    // A torn write (any single word changed, or the magic cleared mid-seal)
    // is rejected; so is garbage; invalidate() clears.
    for (std::size_t i = 0; i < kExtensionWords; ++i) {
        Extension t = e;
        t.w[i] ^= 0x00010000U;
        CHECK(!extension_valid(t));
    }
    Extension garbage{};
    for (std::size_t i = 0; i < kExtensionWords; ++i) garbage.w[i] = 0xA5A5A5A5U + static_cast<std::uint32_t>(i);
    CHECK(!extension_valid(garbage));
    invalidate_extension(e);
    CHECK(!extension_valid(e));
    // The state record itself is unchanged: five 32-bit words, same checksum rule.
    static_assert(sizeof(Record) == 20, "state record layout must not change");
    Record rec{};
    seal(rec, 2, 100.0F, true);
    CHECK(valid(rec) && resumable(rec, ResetKind::Warm));
}

}  // namespace

int main()
{
    test_acceleration_launch();
    test_acceleration_launch_ignores_handling();
    test_airborne_at_boot();
    test_false_flight_callback();
    test_store_extension_record();
    test_launch_needs_30m_agl_held();
    test_launch_needs_30mps_vertical_velocity();
    test_commanded_launch_ignores_velocity();
    test_states_are_one_way();
    test_invalid_agl_no_launch();
    test_burnout_then_kalman_descent();
    test_burnout_from_velocity();
    test_ballistic_descent_unaided();
    test_low_maximum_needs_10m_drop();
    test_velocity_path();
    test_velocity_transient_no_descent();
    test_landing_on_agl();
    test_hitl_exit_rebaseline();
    test_persist_callback_and_restore();
    test_store_record();
    if (failures == 0) std::printf("flight_state_machine_test: all passed\n");
    return failures == 0 ? 0 : 1;
}
