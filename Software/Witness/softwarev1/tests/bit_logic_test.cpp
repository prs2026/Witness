#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "bit_codec.h"
#include "bit_logic.h"

using namespace bit;
static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); ++failures; } } while (0)

namespace {

Inputs good()
{
    Inputs in;
    in.uptime_ms = 70'000;
    in.status0 = 0x02 | 0x04 | 0x20 | 0x40;   // low-g, gyro, high-g, baro ready
    in.reset = ResetClass::PowerOn;
    in.image_known = in.image_flight = in.image_slot_matches = in.image_other_valid = true;
    in.heap_known = true;
    in.heap_free = 42'000; in.heap_min = 38'000; in.heap_largest = 20'000;
    in.stack_sensors = 6000; in.stack_logger = 3400; in.stack_capture = 4000; in.stack_other_min = 1000;
    in.wdt_known = true;
    in.still.valid = true;
    in.still.accel_mag_g = 1.0F; in.still.axial_g = 1.0F; in.still.accel_sd_mg = 1.0F;
    in.still.gyro_mean_dps = 0.2F; in.still.gyro_sd_dps = 0.1F;
    in.still.high_g_valid = true; in.still.high_g_mag_g = 1.02F;
    in.baro.valid = true; in.baro.sd_pa = 3.0F; in.baro.rate_mps = 0.1F; in.baro.sample_rate_hz = 50.0F;
    in.prelaunch_known = true;
    in.radio_power_dbm = 0;
    return in;
}

Verdict run(Engine &e, const Inputs &in, const int n = 8)
{
    for (int i = 0; i < n; ++i) e.update(in);
    return e.summarize(in.uptime_ms).verdict;
}

Eval ev(const std::uint8_t id, const Inputs &in) { return eval_check(id, in); }

}  // namespace

int main()
{
    // ---- table sanity ----
    CHECK(kCheckCount <= 64);
    for (std::size_t i = 0; i < kCheckCount; ++i) {
        CHECK((kReservedGroupMask & (1U << id_group(kChecks[i].id))) == 0U);
        for (std::size_t j = i + 1; j < kCheckCount; ++j) CHECK(kChecks[i].id != kChecks[j].id);
    }
    CHECK(check_index(make_id(8, 3)) >= 0);
    CHECK(check_index(make_id(9, 1)) < 0);   // Iris groups are deferred

    // ---- nominal board: GO ----
    {
        Engine e;
        e.begin(RunKind::Startup, 0);
        CHECK(run(e, good()) == Verdict::Go);
        const Summary s = e.summarize(70'000);
        CHECK(s.blocking_fail_mask == 0 && s.degraded_fail_mask == 0 && s.first_fail_id == 0);
        CHECK(s.core_done && s.settled);
    }

    // ---- not evaluated is never GO ----
    {
        Engine e;
        e.begin(RunKind::Startup, 0);
        Inputs in = good();
        in.uptime_ms = 2000;
        in.still.valid = false;            // gravity window not there yet
        CHECK(run(e, in) == Verdict::Running);
        in.uptime_ms = 11'000;             // core deadline passed, core item still unresolved
        CHECK(run(e, in) == Verdict::NoGo);
        in.still.valid = true;             // resolved: the verdict recovers
        CHECK(run(e, in) == Verdict::Go);
    }

    // ---- group boundaries: heap ----
    {
        Inputs in = good();
        in.heap_min = limits::kHeapMinBlockingBytes - 1;
        CHECK(ev(make_id(0, 3), in).result == Result::Fail && ev(make_id(0, 3), in).cls == Class::Blocking);
        in.heap_min = limits::kHeapMinBlockingBytes;
        CHECK(ev(make_id(0, 3), in).cls == Class::Degraded);
        in.heap_min = limits::kHeapMinDegradedBytes;
        CHECK(ev(make_id(0, 3), in).result == Result::Pass);
        in.heap_largest = limits::kHeapLargestDegradedBytes - 1;
        CHECK(ev(make_id(0, 3), in).cls == Class::Degraded);
    }
    // ---- stacks ----
    {
        Inputs in = good();
        in.stack_sensors = static_cast<std::int32_t>(limits::kStackBlockingBytes) - 1;
        CHECK(ev(make_id(0, 4), in).cls == Class::Blocking);
        in.stack_sensors = static_cast<std::int32_t>(limits::kStackBlockingBytes);
        CHECK(ev(make_id(0, 4), in).cls == Class::Degraded);       // below 512
        in.stack_sensors = 6000; in.stack_other_min = 511;
        CHECK(ev(make_id(0, 4), in).cls == Class::Degraded);
        in.stack_other_min = 512;
        CHECK(ev(make_id(0, 4), in).result == Result::Pass);
    }
    // ---- reset reason ----
    {
        Inputs in = good();
        in.reset = ResetClass::Panic;
        CHECK(ev(make_id(0, 1), in).result == Result::Fail);
        in.resumed = true;                                           // in flight: report only
        CHECK(ev(make_id(0, 1), in).result == Result::Pass);
        in.resumed = false; in.reset = ResetClass::Software;
        CHECK(ev(make_id(0, 1), in).result == Result::Pass);
    }
    // ---- HITL: blocking, bench-flagged ----
    {
        Inputs in = good();
        in.hitl_active = true;
        const Eval e1 = ev(make_id(1, 4), in);
        CHECK(e1.result == Result::Fail && e1.cls == Class::Blocking && e1.bench);
    }
    // ---- IMU rate boundaries (960 Hz, -6/+6 %, 50 %) ----
    {
        Inputs in = good();
        in.imu_rate_hz = 960.0F * 0.94F + 0.1F;
        CHECK(ev(make_id(2, 2), in).result == Result::Pass);
        in.imu_rate_hz = 960.0F * 0.94F - 0.1F;
        CHECK(ev(make_id(2, 2), in).cls == Class::Degraded);
        in.imu_rate_hz = 960.0F * 1.06F + 0.1F;
        CHECK(ev(make_id(2, 2), in).cls == Class::Degraded);
        in.imu_rate_hz = 480.0F - 0.1F;
        CHECK(ev(make_id(2, 2), in).cls == Class::Blocking);
        in.imu_rate_hz = 936.0F;                                     // the bench value (-2.5 %)
        in.imu_loss_growth = true;
        CHECK(ev(make_id(2, 2), in).cls == Class::Degraded);
    }
    // ---- gravity ----
    {
        Inputs in = good();
        in.still.accel_mag_g = 0.79F;
        CHECK(ev(make_id(2, 3), in).cls == Class::Blocking);
        in.still.accel_mag_g = 0.81F;
        CHECK(ev(make_id(2, 3), in).cls == Class::Degraded);
        in.still.accel_mag_g = 0.94F;
        CHECK(ev(make_id(2, 3), in).result == Result::Pass);
        in.still.accel_sd_mg = 31.0F;                                // printer-vibration noise
        CHECK(ev(make_id(2, 3), in).cls == Class::Degraded);         // never blocking
        in.still.accel_mag_g = 1.21F;
        CHECK(ev(make_id(2, 3), in).cls == Class::Blocking);
        in.flight_state = kCoast;
        CHECK(ev(make_id(2, 3), in).result == Result::NotApplicable);
    }
    // ---- orientation (body +X up reads +1 g) ----
    {
        Inputs in = good();
        CHECK(ev(make_id(2, 8), in).result == Result::Pass);
        in.still.axial_g = -1.0F;                                    // nose down
        CHECK(ev(make_id(2, 8), in).result == Result::Fail);
        in.still.axial_g = 0.84F;
        CHECK(ev(make_id(2, 8), in).result == Result::Fail);
        in.still.axial_g = 0.9F; in.still.accel_mag_g = 1.0F;        // 0.90 < cos 20 = 0.94
        CHECK(ev(make_id(2, 8), in).result == Result::Fail);
        in.still.axial_g = 0.95F;
        CHECK(ev(make_id(2, 8), in).result == Result::Pass);
        in.flight_state = kLanded;
        CHECK(ev(make_id(2, 8), in).result == Result::NotApplicable);
    }
    // ---- pad heading warning (owner Q12): degraded, never blocking ----
    {
        Inputs in = good();
        in.pad_heading_set = false;
        const Eval e2 = ev(make_id(4, 7), in);
        CHECK(e2.result == Result::Fail && e2.cls == Class::Degraded);
        Engine e;
        e.begin(RunKind::Startup, 0);
        CHECK(run(e, in) == Verdict::GoWithWarnings);
    }
    // ---- baro ----
    {
        Inputs in = good();
        in.pressure_pa = limits::kBaroPressureMinPa - 1;
        CHECK(ev(make_id(3, 2), in).cls == Class::Blocking);
        in.pressure_pa = limits::kBaroPressureMinPa;
        CHECK(ev(make_id(3, 2), in).result == Result::Pass);
        in.pressure_pa = limits::kBaroPressureMaxPa + 1;
        CHECK(ev(make_id(3, 2), in).cls == Class::Blocking);
        in = good();
        in.baro.frozen = true;
        CHECK(ev(make_id(3, 3), in).cls == Class::Degraded);
        in.baro.frozen = false; in.baro.sd_pa = 26.0F;
        CHECK(ev(make_id(3, 3), in).cls == Class::Degraded);
        in.baro.sd_pa = 3.0F; in.baro.rate_mps = 1.1F;
        CHECK(ev(make_id(3, 3), in).cls == Class::Degraded);
        in = good(); in.status0 &= ~0x40; in.uptime_ms = 11'000;
        CHECK(ev(make_id(3, 1), in).cls == Class::Blocking);
        in.uptime_ms = 5000;
        CHECK(ev(make_id(3, 1), in).result == Result::NotYet);
    }
    // ---- launch-detection readiness (owner Q5: one path only is NO-GO) ----
    {
        Inputs in = good();
        CHECK(ev(make_id(4, 3), in).result == Result::Pass);
        in.rule3_ready = false;                    // rule 2 + rule 1: two paths, still GO
        CHECK(ev(make_id(4, 3), in).result == Result::Pass);
        in.rule1_ready = false; in.rule3_ready = true;   // rule 2 + rule 3
        CHECK(ev(make_id(4, 3), in).result == Result::Pass);
        in.rule1_ready = false; in.rule3_ready = false;  // only the acceleration path
        CHECK(ev(make_id(4, 3), in).result == Result::Fail && ev(make_id(4, 3), in).cls == Class::Blocking);
        in = good(); in.rule2_ready = false;             // no acceleration path
        CHECK(ev(make_id(4, 3), in).cls == Class::Blocking);
        in.flight_state = kCoast;                        // in flight: informational
        CHECK(ev(make_id(4, 3), in).cls == Class::Info);
        in = good(); in.rule1_ready = in.rule2_ready = in.rule3_ready = false; in.uptime_ms = 1000;
        CHECK(ev(make_id(4, 3), in).result == Result::NotYet);   // still settling
    }
    // ---- NAND / log session ----
    {
        Inputs in = good();
        in.nand_chip_ok = false;
        CHECK(ev(make_id(5, 1), in).cls == Class::Blocking);
        in = good(); in.writer_failed_blocks = 2;
        CHECK(ev(make_id(5, 4), in).cls == Class::Degraded);
        in.writer_failing = true;
        CHECK(ev(make_id(5, 4), in).cls == Class::Blocking);
        in = good(); in.slots_full = true;
        CHECK(ev(make_id(6, 1), in).cls == Class::Blocking);
        in = good(); in.imu_seconds_left = limits::kImuFreeSecondsBlocking - 1;
        CHECK(ev(make_id(6, 2), in).cls == Class::Blocking);
        in.imu_seconds_left = limits::kImuFreeSecondsBlocking;
        CHECK(ev(make_id(6, 2), in).cls == Class::Degraded);
        in.imu_seconds_left = limits::kImuFreeSecondsDegraded;
        CHECK(ev(make_id(6, 2), in).result == Result::Pass);
        in.binary_seconds_left = limits::kBinaryFreeSecondsBlocking - 1;
        CHECK(ev(make_id(6, 2), in).cls == Class::Blocking);
        in = good(); in.logger_pass_age_ms = limits::kLoggerHeartbeatMaxMs + 1;
        CHECK(ev(make_id(6, 5), in).cls == Class::Blocking);
        in.logger_pass_age_ms = limits::kLoggerHeartbeatMaxMs;
        CHECK(ev(make_id(6, 5), in).result == Result::Pass);
        in = good(); in.drop_growth = true;
        CHECK(ev(make_id(6, 4), in).cls == Class::Degraded);
        in.status0 |= 0x80;                         // FLASH_LOG_FAILED: logging is on and failing
        CHECK(ev(make_id(6, 4), in).cls == Class::Blocking);
    }
    // ---- active results: not run is SKIPPED, error is never a pass ----
    {
        Inputs in = good();
        CHECK(ev(make_id(6, 3), in).result == Result::Skipped);
        in.log_end_to_end.state = ActiveResult::State::Pending;
        CHECK(ev(make_id(6, 3), in).result == Result::NotYet);
        in.log_end_to_end.state = ActiveResult::State::Done;
        in.log_end_to_end.pass = true;
        CHECK(ev(make_id(6, 3), in).result == Result::Pass);
        in.log_end_to_end.pass = false;
        CHECK(ev(make_id(6, 3), in).result == Result::Fail && ev(make_id(6, 3), in).cls == Class::Blocking);
        in.log_end_to_end.pass = true; in.log_end_to_end.error = true;
        CHECK(ev(make_id(6, 3), in).result == Result::Error);
        Engine e;
        e.begin(RunKind::Commanded, 70'000);
        CHECK(run(e, in) == Verdict::NoGo);          // an ERROR on a blocking check blocks
    }
    // ---- radio: blocking; -3 dBm minimum; bench flag (owner Q1, Q4) ----
    {
        Inputs in = good();
        in.radio_power_dbm = -3;
        CHECK(ev(make_id(8, 3), in).result == Result::Pass);
        in.radio_power_dbm = -4;
        Eval e3 = ev(make_id(8, 3), in);
        CHECK(e3.result == Result::Fail && e3.cls == Class::Blocking && e3.bench);
        in.radio_power_dbm = -17;
        e3 = ev(make_id(8, 3), in);
        CHECK(e3.result == Result::Fail && e3.bench && e3.value == static_cast<std::uint16_t>(-17));
        in.radio_power_dbm = 0; in.radio_front_end_on = false;
        e3 = ev(make_id(8, 3), in);
        CHECK(e3.result == Result::Fail && !e3.bench);               // front end off is not a bench condition
        in = good(); in.radio_ready = false; in.uptime_ms = 10'000;
        CHECK(ev(make_id(8, 1), in).result == Result::NotYet);       // radio gets 30 s
        in.uptime_ms = 31'000;
        CHECK(ev(make_id(8, 1), in).cls == Class::Blocking);         // a missing radio is NO-GO
        in = good(); in.radio_faulted = true; in.uptime_ms = 5000;
        CHECK(ev(make_id(8, 1), in).cls == Class::Blocking);
    }
    // ---- bench-only NO-GO is flagged; a real failure clears the flag ----
    {
        Engine e;
        e.begin(RunKind::Startup, 0);
        Inputs in = good();
        in.radio_power_dbm = -17;                                    // the bench
        CHECK(run(e, in) == Verdict::NoGo);
        CHECK(e.summarize(in.uptime_ms).bench);
        CHECK(e.summarize(in.uptime_ms).blocking_fail_mask == (1U << kGroupRadio));
        in.hitl_active = true;                                       // HITL as well: still all bench
        CHECK(run(e, in) == Verdict::NoGo && e.summarize(in.uptime_ms).bench);
        in.nand_chip_ok = false;                                     // a real failure
        CHECK(run(e, in) == Verdict::NoGo && !e.summarize(in.uptime_ms).bench);
        CHECK(e.summarize(in.uptime_ms).first_fail_id == make_id(1, 4) ||
              e.summarize(in.uptime_ms).first_fail_id == make_id(5, 1));
    }
    // ---- battery: 1S Li-ion placeholders ----
    {
        Inputs in = good();
        in.battery_mv = limits::kBatteryBlockingBelowMv - 1;
        CHECK(ev(make_id(11, 1), in).cls == Class::Blocking);
        in.battery_mv = limits::kBatteryBlockingBelowMv;
        CHECK(ev(make_id(11, 1), in).cls == Class::Degraded);
        in.battery_mv = limits::kBatteryDegradedBelowMv;
        CHECK(ev(make_id(11, 1), in).result == Result::Pass);
        in.battery_mv = limits::kBatteryDegradedAboveMv + 1;
        CHECK(ev(make_id(11, 1), in).cls == Class::Degraded);
    }
    // ---- 3V3 rail and current: disabled until the hardware exists ----
    {
        Inputs in = good();
        in.rail_3v3_valid = true;
        CHECK(ev(make_id(11, 3), in).result == Result::Skipped);     // kRail3v3MonitorPresent is false
    }
    // ---- GPS never blocks ----
    {
        Inputs in = good();
        in.gps_status = Inputs::Gps::Failed; in.gps_rate_hz = 0; in.gps_fix = false; in.gps_satellites = 0;
        for (std::uint8_t id : {make_id(7, 1), make_id(7, 2), make_id(7, 4)}) {
            const Eval g = ev(id, in);
            CHECK(g.result == Result::Fail && g.cls == Class::Degraded);
        }
        in.uptime_ms = 5000;
        CHECK(ev(make_id(7, 4), in).result == Result::NotYet);
        in = good(); in.gps_rate_hz = 22;
        CHECK(ev(make_id(7, 2), in).result == Result::Fail);
        in.gps_rate_hz = 23;
        CHECK(ev(make_id(7, 2), in).result == Result::Pass);
    }
    // ---- debounce ----
    {
        Engine e;
        e.begin(RunKind::Startup, 0);
        Inputs in = good();
        run(e, in, 3);
        in.heap_min = 10'000;                                        // degraded fail
        e.update(in); e.update(in);
        CHECK(e.summarize(in.uptime_ms).degraded_fail_mask == 0);    // needs 3
        e.update(in);
        CHECK(e.summarize(in.uptime_ms).degraded_fail_mask == 1U);
        in.heap_min = 38'000;
        for (int i = 0; i < 4; ++i) e.update(in);
        CHECK(e.summarize(in.uptime_ms).degraded_fail_mask == 1U);   // 5 passes to recover
        e.update(in);
        CHECK(e.summarize(in.uptime_ms).degraded_fail_mask == 0);
        // a single-sample glitch of a blocking check never shows
        in.heap_min = 1000;
        e.update(in);
        CHECK(e.summarize(in.uptime_ms).blocking_fail_mask == 0);
        in.heap_min = 38'000;
        e.update(in);
        CHECK(e.summarize(in.uptime_ms).blocking_fail_mask == 0);
        in.heap_min = 1000;
        e.update(in); e.update(in);                                  // blocking: 2 evaluations
        CHECK(e.summarize(in.uptime_ms).blocking_fail_mask == 1U);
    }
    // ---- commanded run: no debounce ----
    {
        Engine e;
        e.begin(RunKind::Commanded, 70'000);
        Inputs in = good();
        in.nand_chip_ok = false;
        e.update(in);
        CHECK(e.summarize(70'100).verdict == Verdict::NoGo);
    }
    // ---- resumed in flight: passive only, nothing is demanded of the pad ----
    {
        Engine e;
        e.begin(RunKind::Passive, 0);
        Inputs in = good();
        in.flight_state = kCoast; in.resumed = true; in.reset = ResetClass::TaskWdt;
        in.still.valid = false; in.baro.valid = false;
        CHECK(run(e, in) == Verdict::NoGo || run(e, in) == Verdict::Go || run(e, in) == Verdict::Running);
        const Summary s = e.summarize(in.uptime_ms);
        CHECK(s.resumed);
        CHECK((s.degraded_fail_mask & 1U) == 0);                     // 0.1 is reported, not failed
    }

    // ---- stillness gate (plan 3.3) ----
    {
        GateInputs g;
        g.still_valid = true;
        CHECK(active_test_gate(g, false) == GateReason::Open);
        CHECK(active_test_gate(g, true) == GateReason::Open);
        g.flight_state = kLanded;
        CHECK(active_test_gate(g, false) == GateReason::Open);
        CHECK(active_test_gate(g, true) == GateReason::PadOnly);
        for (std::uint8_t s : {kBoost, kCoast, kDescent}) {
            g.flight_state = s;
            CHECK(active_test_gate(g, false) == GateReason::NotOnGround);
        }
        g = GateInputs{}; g.still_valid = true;
        g.resumed = true;
        CHECK(active_test_gate(g, false) == GateReason::Resumed);
        g.resumed = false; g.launch_budget_zero = false;
        CHECK(active_test_gate(g, false) == GateReason::NotStill);
        g.launch_budget_zero = true; g.gyro_mean_dps = 5.1F;
        CHECK(active_test_gate(g, false) == GateReason::NotStill);
        g.gyro_mean_dps = 4.9F; g.accel_mag_g = 1.11F;
        CHECK(active_test_gate(g, false) == GateReason::NotStill);   // swinging / handled
        g.accel_mag_g = 0.89F;
        CHECK(active_test_gate(g, false) == GateReason::NotStill);   // free fall
        g.accel_mag_g = 1.0F; g.baro_rate_mps = 2.1F;
        CHECK(active_test_gate(g, false) == GateReason::NotStill);
        g.baro_rate_mps = 0.0F; g.kalman_vz_mps = 0.6F;
        CHECK(active_test_gate(g, false) == GateReason::NotStill);
        g.kalman_vz_mps = 0.0F; g.still_valid = false;
        CHECK(active_test_gate(g, false) == GateReason::NotStill);
        g.still_valid = true; g.run_in_progress = true;
        CHECK(active_test_gate(g, false) == GateReason::Busy);
        g.run_in_progress = false; g.maintenance_or_erase_pending = true;
        CHECK(active_test_gate(g, false) == GateReason::Busy);
        CHECK(!active_test_tripped(kPadIdle, 1.0F, true, kPadIdle));
        CHECK(active_test_tripped(kBoost, 1.0F, true, kPadIdle));
        CHECK(active_test_tripped(kPadIdle, 1.6F, true, kPadIdle));
        CHECK(active_test_tripped(kPadIdle, 1.0F, false, kPadIdle));
    }

    // ---- codec: golden bytes ----
    {
        Summary s;
        s.verdict = Verdict::NoGo; s.bench = true; s.kind = RunKind::Commanded; s.flight_state = kPadIdle;
        s.core_done = true; s.settled = true; s.active_ran = true;
        s.blocking_fail_mask = 1U << kGroupRadio;        // 0x0100
        s.degraded_fail_mask = (1U << kGroupGps) | (1U << kGroupAttitude);   // 0x0090
        s.not_evaluated_mask = 1U << kGroupCan;          // never set by Witness; golden only
        s.first_fail_id = make_id(8, 3);
        s.sequence = 7; s.age_s = 300;
        std::uint8_t f[codec::kSummaryFrameLength]{};
        CHECK(codec::encode_summary(s, f) == 14);
        const std::uint8_t golden[14] = {0x0B, 0x1C, 0xE1, 0x01, 0x00, 0x00, 0x90, 0x02, 0x00, 0x83, 0x07, 0x01, 0x2C, 0x0A};
        CHECK(std::memcmp(f, golden, 14) == 0);
        Summary d;
        CHECK(codec::decode_summary(f, 14, d));
        CHECK(d.verdict == Verdict::NoGo && d.bench && d.kind == RunKind::Commanded && d.core_done && d.settled &&
              d.active_ran && d.blocking_fail_mask == 0x0100 && d.degraded_fail_mask == 0x0090 &&
              d.first_fail_id == 0x83 && d.sequence == 7 && d.age_s == 300);
        f[13] = 0x00;
        CHECK(!codec::decode_summary(f, 14, d));
        f[13] = 0x0A; f[1] = 0x2C;                       // version 2
        CHECK(!codec::decode_summary(f, 14, d));
        Summary a; a.verdict = Verdict::NoGo; a.blocking_fail_groups = 3;
        CHECK(codec::ack_detail(a) == 0x0403);
    }
    {
        Engine e;
        e.begin(RunKind::Startup, 0);
        Inputs in = good();
        in.radio_power_dbm = -17;
        run(e, in);
        std::uint8_t f[codec::kDetailFrameLength]{};
        CHECK(codec::kDetailPageCount == 3);
        CHECK(codec::encode_detail(e, 2, f));
        CHECK(!codec::encode_detail(e, 3, f));
        std::uint8_t page = 0, pages = 0, seq = 0;
        codec::DetailRecord recs[24];
        int n = codec::decode_detail(f, sizeof(f), page, pages, seq, recs);
        CHECK(n == static_cast<int>(kCheckCount) - 48 && page == 2 && pages == 3);
        // the radio power record sits on page 1 or 2 depending on the table; find it
        bool found = false;
        for (std::uint8_t p = 0; p < 3; ++p) {
            codec::encode_detail(e, p, f);
            n = codec::decode_detail(f, sizeof(f), page, pages, seq, recs);
            for (int i = 0; i < n; ++i) {
                if (recs[i].id == make_id(8, 3)) {
                    found = true;
                    CHECK(recs[i].result == Result::Fail && recs[i].cls == Class::Blocking && recs[i].bench);
                    CHECK(recs[i].value == static_cast<std::uint16_t>(-17));
                }
            }
        }
        CHECK(found);
    }

    if (failures == 0) std::printf("bit_logic_test: all passed\n");
    return failures == 0 ? 0 : 1;
}
