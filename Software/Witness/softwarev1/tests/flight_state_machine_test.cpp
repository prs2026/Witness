// Host test: robust altitude / Descent logic, HITL re-baseline and the
// reset-resume record. Build command: see FIRMWARE_HANDOFF.md
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
    // One fresh barometer sample every 20 ms (50 Hz).
    void step(float alt, bool gated = false, bool vel_valid = true,
              float vel = 0.0F)
    {
        FlightStateMachine::Observation o{};
        o.barometric_altitude_valid = true;
        o.barometric_altitude_meters = alt;
        o.barometric_altitude_new = true;
        o.barometer_gated = gated;
        o.vertical_velocity_valid = vel_valid;
        o.vertical_velocity_mps = vel;
        o.low_g_axial_acceleration_valid = true;
        o.low_g_axial_acceleration_g = 0.0F;
        o.timestamp_us = t_us;
        fsm.update(o);
        t_us += 20'000;
    }
    void run(float alt, int n, bool gated = false, float vel = 0.0F,
             bool vel_valid = true)
    {
        for (int i = 0; i < n; ++i) step(alt, gated, vel_valid, vel);
    }
    void to_coast(float alt)
    {
        run(alt, 10);
        CHECK(fsm.trigger_launch() == ESP_OK);
        run(alt, 3);
        fsm.set_state(static_cast<std::uint8_t>(State::Coast));
        run(alt, 3);
    }
};

int g_saved_state = -1;
void persist_cb(std::uint8_t s, float, bool) { g_saved_state = s; }

void test_single_outlier_ignored()
{
    Rig r;
    r.to_coast(1000.0F);
    r.run(20000.0F, 1);              // one bad HIGH sample (the HITL run)
    r.run(1000.0F, 5);
    CHECK(r.fsm.state() == State::Coast);
    CHECK(r.fsm.maximum_altitude() < 1100.0F);
    for (int i = 0; i < 100; ++i) r.step(1000.0F + 2.0F * i, false, true, 100.0F);
    CHECK(r.fsm.state() == State::Coast);
    CHECK(r.fsm.maximum_altitude() > 1150.0F);
}

void test_burst_of_two_ignored()
{
    Rig r;
    r.to_coast(1000.0F);
    r.run(43000.0F, 2);
    r.run(1000.0F, 100, false, 50.0F);
    CHECK(r.fsm.state() == State::Coast);
    CHECK(r.fsm.maximum_altitude() < 1100.0F);
}

void test_rate_limit_blocks_sustained_jump()
{
    Rig r;
    r.to_coast(1000.0F);
    r.run(40000.0F, 30);            // 0.6 s of sustained garbage > window
    CHECK(r.fsm.maximum_altitude() < 1200.0F);   // 40 km in 0.6 s impossible
}

void test_nominal_descent_latency()
{
    Rig r;
    r.to_coast(30000.0F);
    r.run(30000.0F, 20);
    int steps_to_descent = -1;
    for (int i = 0; i < 200; ++i) {
        r.step(27000.0F, false, true, -30.0F);   // 10 % below max, falling
        if (r.fsm.state() == State::Descent) { steps_to_descent = i + 1; break; }
    }
    CHECK(r.fsm.state() == State::Descent);
    std::printf("descent after %d samples (%d ms)\n", steps_to_descent,
                steps_to_descent * 20);
    // 500 ms dwell (25 samples) + median lag (<= 3 samples) + 1.
    CHECK(steps_to_descent <= 25 + 3 + 2);
}

void test_gated_baro_no_descent_no_max()
{
    Rig r;
    r.to_coast(10000.0F);
    r.run(10000.0F, 20);
    const float max_before = r.fsm.maximum_altitude();
    r.run(5000.0F, 200, true, 300.0F);       // transonic baro drop while gated
    r.run(30000.0F, 50, true, 300.0F);       // and a jump up
    CHECK(r.fsm.state() == State::Coast);
    CHECK(r.fsm.maximum_altitude() == max_before);
}

void test_velocity_must_corroborate()
{
    Rig r;
    r.to_coast(10000.0F);
    r.run(10000.0F, 20);
    r.run(8000.0F, 200, false, +200.0F);     // baro low but still climbing
    CHECK(r.fsm.state() == State::Coast);
    r.run(8000.0F, 40, false, -5.0F);        // now falling too
    CHECK(r.fsm.state() == State::Descent);
}

void test_velocity_path_when_gated()
{
    Rig r;
    r.to_coast(10000.0F);
    r.run(10000.0F, 200, false, +150.0F);    // 4 s of climb after burnout
    int steps = -1;
    for (int i = 0; i < 400; ++i) {          // apogee passed, baro gated
        r.step(30000.0F, true, true, -80.0F);
        if (r.fsm.state() == State::Descent) { steps = i + 1; break; }
    }
    CHECK(r.fsm.state() == State::Descent);
    std::printf("velocity path descent after %d ms\n", steps * 20);
    CHECK(steps * 20 <= 2200);
}

void test_velocity_transient_no_descent()
{
    Rig r;
    r.to_coast(10000.0F);
    r.run(10000.0F, 200, false, +150.0F);
    r.run(10000.0F, 80, true, -80.0F);       // 1.6 s negative < 2 s dwell
    r.run(10000.0F, 100, true, +100.0F);
    r.run(10000.0F, 80, true, -80.0F);
    CHECK(r.fsm.state() == State::Coast);
    // Negative velocity with no climb ever seen (glitch near burnout).
    Rig q;
    q.to_coast(10000.0F);
    q.run(10000.0F, 400, true, -80.0F);
    CHECK(q.fsm.state() == State::Coast);
    // Too soon after burnout.
    Rig s;
    s.to_coast(10000.0F);
    s.run(10000.0F, 20, false, +150.0F);
    s.run(10000.0F, 120, true, -80.0F);      // inside 3 s of Coast entry
    CHECK(s.fsm.state() == State::Coast);
}

void test_hitl_exit_rebaseline()
{
    Rig r;
    r.to_coast(34800.0F);
    r.run(34800.0F, 20);
    r.fsm.rebaseline();                       // HITL exit
    r.run(208.0F, 200, false, 0.0F);          // bench baro
    CHECK(r.fsm.state() == State::Coast);
    CHECK(r.fsm.maximum_altitude() < 300.0F);
}

void test_persist_callback_and_restore()
{
    Rig r;
    r.fsm.set_persist_callback(persist_cb);
    r.run(100.0F, 10);
    r.fsm.trigger_launch();
    CHECK(g_saved_state == 1);
    r.fsm.restore(State::Coast, 5000.0F, true);
    CHECK(r.fsm.state() == State::Coast);
    CHECK(r.status.flight_state() == 2);
    r.run(5100.0F, 10);                       // restored max kept
    CHECK(r.fsm.maximum_altitude() >= 5000.0F);
    r.run(5000.0F, 200, false, -10.0F);
    CHECK(r.fsm.state() == State::Coast);     // 5000 is not 5 % below 5100
    r.run(4000.0F, 100, false, -10.0F);
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
    test_single_outlier_ignored();
    test_burst_of_two_ignored();
    test_rate_limit_blocks_sustained_jump();
    test_nominal_descent_latency();
    test_gated_baro_no_descent_no_max();
    test_velocity_must_corroborate();
    test_velocity_path_when_gated();
    test_velocity_transient_no_descent();
    test_hitl_exit_rebaseline();
    test_persist_callback_and_restore();
    test_store_record();
    if (failures == 0) std::printf("flight_state_machine_test: all passed\n");
    return failures == 0 ? 0 : 1;
}
