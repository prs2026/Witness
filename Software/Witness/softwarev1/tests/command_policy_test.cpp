#include "command_policy.h"
#include "comms.h"

#include <cassert>
#include <initializer_list>

using namespace command_policy;

static_assert(command_policy::kBenchRestart == IRIS_COMMAND_BENCH_RESTART &&
                  command_policy::kBenchHang == IRIS_COMMAND_BENCH_HANG &&
                  command_policy::kBenchIrisReset == IRIS_COMMAND_BENCH_IRIS_RESET,
              "bench opcodes must match comms.h");

int main()
{
    // Bench-only commands (restart, hang, Iris reset): USB with HITL active only;
    // never from radio, never without HITL; no other opcode is a bench command.
    for (const std::uint16_t c : {kBenchRestart, kBenchHang, kBenchIrisReset}) {
        assert(is_bench_command(c));
        assert(allow_bench_command(Source::Usb, true));
        assert(!allow_bench_command(Source::Usb, false));
        assert(!allow_bench_command(Source::Radio, true));
        assert(!allow_bench_command(Source::Radio, false));
        // After a warm reset (RESUMED, HITL off): USB only.
        assert(allow_bench_command(Source::Usb, false, true));
        assert(!allow_bench_command(Source::Radio, false, true));
        assert(!allow_bench_command(Source::Radio, true, true));
        assert(!allow_bench_command(Source::Usb, false, false));
    }
    assert(kBenchRestart != kBenchHang && kBenchHang != kBenchIrisReset && kBenchRestart != kBenchIrisReset);
    for (const std::uint16_t c : {0x4251, 0x4253, 0x4247, 0x424A, 0x4848, 0x4749, 0x0030, 0x5746, 0x0000, 0xFFFF})
        assert(!is_bench_command(c));
    // None of them collides with a command that has a meaning in flight.
    assert(!is_log_maintenance_command(kBenchHang) && !is_iris_load_command(kBenchIrisReset) &&
           kBenchRestart != kRadioPower && kBenchHang != kGpsAssistBegin && kBenchIrisReset != kGpsAssistData);
    assert(is_log_maintenance_command(0x6868) && is_log_maintenance_command(0x6869) &&
           is_log_maintenance_command(0x686A) && !is_log_maintenance_command(0x6867));

    for (std::uint8_t s = 0; s <= 5; ++s) {
        const bool ground = s == kPadIdle || s == kLanded;
        // USB export: USB only, Pad Idle and Landed only.
        assert(!allow_log_maintenance(Source::Radio, s));
        assert(!allow_log_maintenance(Source::Radio, s, kMassStorageStart));
        assert(allow_log_maintenance(Source::Usb, s) == ground);
        // Erase / clear: USB or radio, Pad Idle and Landed only.
        for (const Source src : {Source::Usb, Source::Radio}) {
            assert(allow_log_maintenance(src, s, kEraseLogSessions) == ground);
            assert(allow_log_maintenance(src, s, kClearLogSessions) == ground);
        }
        // Radio power: refused in Boost only (see the C/D rules below). GPS
        // assist uploads: Pad Idle and Landed only; assist status always.
        assert(allow_radio_power(s) == (s != kBoost));
        for (const std::uint16_t c : {kGpsAssistBegin, kGpsAssistData, kGpsAssistCommit})
            assert(allow_gps_assist(c, s) == ground);
        assert(allow_gps_assist(kGpsAssistStatus, s));
        assert(allow_gps_cold_start(s) == (s == kPadIdle));
        assert(allow_persist_radio_power(s) == ground);
        // Iris loads: ON always, OFF only on the ground.
        for (const std::uint16_t c : {kCh1LoadOn, kCh2LoadOn, kCh3LoadOn, kRegulator5vOn})
            assert(allow_iris_load_command(c, s) && is_iris_load_command(c) && !is_iris_load_off(c));
        for (const std::uint16_t c : {kCh1LoadOff, kCh2LoadOff, kCh3LoadOff, kRegulator5vOff})
            assert(allow_iris_load_command(c, s) == ground && is_iris_load_off(c) && is_iris_load_command(c));
        // Anything else is not a load command and is never blocked by this rule.
        for (const std::uint16_t c : {0x0020, 0x0022, 0x0036, 0x004F, 0x0052, 0x5250})
            assert(!is_iris_load_command(c) && allow_iris_load_command(c, s));
        // Radio power change.
        for (int req = kRadioPowerMinDbm; req <= kRadioPowerMaxDbm; ++req) {
            for (int cur = kRadioPowerMinDbm; cur <= kRadioPowerMaxDbm; ++cur) {
                for (const bool fe_off : {false, true}) {
                    const bool got = allow_radio_power_change(s, req, cur, fe_off);
                    if (s == kBoost) assert(!got);
                    else if (s == kCoast || s == kDescent) assert(got == (!fe_off && req >= cur));
                    else assert(got);
                }
            }
        }
    }
    assert(radio_power_in_range(-17) && radio_power_in_range(3) && !radio_power_in_range(-18) &&
           !radio_power_in_range(4));

    // Deferred restarts: re-checked when the timer fires.
    for (std::uint8_t s = 0; s <= 5; ++s) {
        assert(restart_still_allowed(PendingRestart::Maintenance, s) == (s == kPadIdle || s == kLanded));
        assert(restart_still_allowed(PendingRestart::ResetToPad, s) == (s == kLanded));
        assert(!restart_still_allowed(PendingRestart::None, s));
    }

    // Radio erase/clear confirmation: same command twice within 10 s.
    {
        EraseConfirm c;
        assert(!c.accept(kEraseLogSessions, 1000));                  // first: confirm needed
        assert(c.accept(kEraseLogSessions, 5000));                   // second: runs
        assert(!c.accept(kEraseLogSessions, 5100));                  // used up: arms again
        assert(!c.accept(kClearLogSessions, 5200));                  // different command: re-arms
        assert(!c.accept(kEraseLogSessions, 5300));                  // erase after clear: re-arms
        assert(c.accept(kEraseLogSessions, 5300 + kEraseConfirmWindowMs));    // exactly 10 s: ok
        assert(!c.accept(kClearLogSessions, 100000));
        assert(!c.accept(kClearLogSessions, 100000 + kEraseConfirmWindowMs + 1));  // too late: re-arms
        assert(c.accept(kClearLogSessions, 100000 + kEraseConfirmWindowMs + 2));
        assert(!c.accept(kClearLogSessions, 0xFFFFFFF0U));           // tick wrap
        assert(c.accept(kClearLogSessions, 0x00000100U));            // 0x110 ms later
        c.reset();
        assert(!c.accept(kClearLogSessions, 7));
    }

    for (std::uint8_t cur = 0; cur <= 4; ++cur) {
        for (std::uint8_t req = 0; req <= 5; ++req) {
            const bool radio = allow_set_flight_state(Source::Radio, cur, req);
            // Radio: reset from Landed, or the Pad Idle no-op; nothing else.
            assert(radio == ((cur == kLanded && req == kPadIdle) || (cur == kPadIdle && req == kPadIdle)));
            const bool usb = allow_set_flight_state(Source::Usb, cur, req);
            if (req > 4) {
                assert(!usb);
            } else if (cur == kLanded) {
                assert(usb == (req == kPadIdle || req == kLanded));   // reset or no-op
            } else if (cur >= kBoost && cur <= kDescent) {
                assert(usb == (req == kLanded || req == cur));
            } else {
                assert(usb);   // Pad Idle: any forward, or same
            }
            // Nothing steps back except the Landed -> Pad Idle reset.
            if (req < cur) {
                assert(usb == is_reset_to_pad(cur, req));
                assert(radio == is_reset_to_pad(cur, req));
            }
        }
    }
    assert(is_reset_to_pad(kLanded, kPadIdle) && !is_reset_to_pad(kDescent, kPadIdle) &&
           !is_reset_to_pad(kPadIdle, kPadIdle) && !is_reset_to_pad(kLanded, kBoost));
    assert(!allow_set_flight_state(Source::Radio, kPadIdle, kBoost));  // no radio launch
    assert(!allow_set_flight_state(Source::Radio, kCoast, kPadIdle));
    assert(allow_set_flight_state(Source::Usb, kPadIdle, kBoost));     // bench launch
    assert(!allow_set_flight_state(Source::Usb, kDescent, kPadIdle));  // refused: reset only from Landed
    assert(!allow_set_flight_state(Source::Usb, kDescent, kCoast));    // no stepping back
    assert(!allow_set_flight_state(Source::Usb, kLanded, kBoost));
    assert(allow_set_flight_state(Source::Usb, kLanded, kPadIdle));    // reset
    assert(allow_set_flight_state(Source::Radio, kLanded, kPadIdle));  // reset
    assert(allow_set_flight_state(Source::Usb, kBoost, kLanded));
    return 0;
}
