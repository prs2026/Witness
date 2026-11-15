// Host test: Kalman-AGL launch / Descent / landing logic, HITL re-baseline and
// the reset-resume record. Build command: see FIRMWARE_HANDOFF.md
// ("Flight state across resets").
#include <cmath>
#include <cstdio>
#include <cstdlib>

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
        o.timestamp_us = t_us;
        fsm.update(o);
        t_us += 20'000;
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
}  // namespace

int main()
{
    test_launch_needs_30m_agl_held();
    test_invalid_agl_no_launch();
    test_burnout_then_kalman_descent();
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
