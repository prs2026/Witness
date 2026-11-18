#include <cstdio>
#include <initializer_list>

#include "bit_policy.h"

using namespace bit;
using namespace bit::policy;
static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); ++failures; } } while (0)

int main()
{
    // Opcodes: none of them collides with another command, and the bench
    // commands of other work (0x4248 hang, 0x4249 Iris reset) are not used.
    CHECK(IRIS_COMMAND_BIT_RUN == 0x5442U && IRIS_COMMAND_BIT_REPORT == 0x5452U);
    CHECK(IRIS_COMMAND_BIT_CONFIG == 0x5443U && IRIS_COMMAND_BIT_FAULT == 0x4246U);
    CHECK(!is_bit_command(0x4248U) && !is_bit_command(0x4249U) && !is_bit_command(0x4252U));
    CHECK(is_bit_command(0x5442U) && is_bit_command(0x5452U) && is_bit_command(0x5443U) && is_bit_command(0x4246U));
    CHECK(!is_bit_command(IRIS_COMMAND_RADIO_POWER) && !is_bit_command(IRIS_COMMAND_PING));

    // Run: Pad Idle and Landed only, both sources equal, never RESUMED, never twice.
    for (Source src : {Source::Usb, Source::Radio}) {
        CHECK(allow_bit_run(src, kPadIdle, false, false, 0, 0) == Decision::Ok);
        CHECK(allow_bit_run(src, kLanded, false, false, 0, 0) == Decision::Ok);
        CHECK(allow_bit_run(src, kBoost, false, false, 0, 0) == Decision::Refused);
        CHECK(allow_bit_run(src, kCoast, false, false, 0, 0) == Decision::Refused);
        CHECK(allow_bit_run(src, kDescent, false, false, 0, 0) == Decision::Refused);
        CHECK(allow_bit_run(src, kPadIdle, true, false, 0, 0) == Decision::Refused);
        CHECK(allow_bit_run(src, kPadIdle, false, true, 0, 0) == Decision::Busy);
        CHECK(allow_bit_run(src, 9, false, false, 0, 0) == Decision::Refused);
    }
    CHECK(allow_bit_run(Source::Usb, kPadIdle, false, false, 1, 0) == Decision::Ok);          // passive snapshot
    CHECK(allow_bit_run(Source::Usb, kPadIdle, false, false, 2, 0) == Decision::BadArgument);
    CHECK(allow_bit_run(Source::Usb, kPadIdle, false, false, 0, IRIS_BIT_OPTION_CAMERA) == Decision::BadArgument);
    CHECK(allow_bit_run(Source::Usb, kPadIdle, false, false, 0, 0x20) == Decision::BadArgument);
    CHECK(allow_bit_run(Source::Usb, kPadIdle, false, false, 0, 0x40) == Decision::BadArgument);
    CHECK(allow_bit_run(Source::Usb, kPadIdle, false, false, 0, 0x1F) == Decision::Ok);       // all real options
    // A wrong state wins over a wrong option? No: a bad argument is reported first.
    CHECK(allow_bit_run(Source::Usb, kBoost, false, false, 0, 0x80) == Decision::BadArgument);
    CHECK(option_allowed_in_state(IRIS_BIT_OPTION_IMU_SELF_TEST, kPadIdle));
    CHECK(!option_allowed_in_state(IRIS_BIT_OPTION_IMU_SELF_TEST, kLanded));
    CHECK(option_allowed_in_state(IRIS_BIT_OPTION_IMAGE_HEAP, kLanded));

    // Report: any state except Boost.
    CHECK(allow_bit_report(kPadIdle) && allow_bit_report(kCoast) && allow_bit_report(kDescent) &&
          allow_bit_report(kLanded));
    CHECK(!allow_bit_report(kBoost));

    // Fault injection: HITL active and canonical USB only; never radio.
    CHECK(allow_bit_fault(Source::Usb, true));
    CHECK(!allow_bit_fault(Source::Usb, false));
    CHECK(!allow_bit_fault(Source::Radio, true));
    CHECK(!allow_bit_fault(Source::Radio, false));
    CHECK(fault_id_valid(1) && fault_id_valid(6) && fault_id_valid(8) && fault_id_valid(9) && fault_id_valid(10) &&
          fault_id_valid(0xFF));
    CHECK(!fault_id_valid(0) && !fault_id_valid(7) && !fault_id_valid(11));
    CHECK(fault_duration_ms(0) == 60'000 && fault_duration_ms(500) == 500 && fault_duration_ms(65'535) == 60'000);

    // Radio summary resend: on change; every 10 s while not GO; nothing extra in Boost.
    CHECK(radio_summary_due(kPadIdle, Verdict::NoGo, true, 0));
    CHECK(!radio_summary_due(kPadIdle, Verdict::NoGo, false, 9999));
    CHECK(radio_summary_due(kPadIdle, Verdict::NoGo, false, 10'000));
    CHECK(!radio_summary_due(kPadIdle, Verdict::Go, false, 100'000));
    CHECK(radio_summary_due(kPadIdle, Verdict::GoWithWarnings, false, 10'000));
    CHECK(!radio_summary_due(kBoost, Verdict::NoGo, false, 100'000));
    CHECK(radio_summary_due(kBoost, Verdict::NoGo, true, 0));

    // LED: NO-GO is 4 pulses; the existing 3-pulse not-logging pattern wins.
    CHECK(pulses(1, false) == 1);
    CHECK(pulses(1, true) == 4);
    CHECK(pulses(3, true) == 3);
    CHECK(pulses(3, false) == 3);
    publish(Verdict::NoGo, true);
    CHECK(led_nogo());
    publish(Verdict::GoWithWarnings, false);
    CHECK(!led_nogo());

    if (failures == 0) std::printf("bit_policy_test: all passed\n");
    return failures == 0 ? 0 : 1;
}
