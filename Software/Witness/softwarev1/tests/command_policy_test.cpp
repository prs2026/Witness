#include "command_policy.h"

#include <cassert>

using namespace command_policy;

int main()
{
    assert(is_log_maintenance_command(0x6868) && is_log_maintenance_command(0x6869) &&
           is_log_maintenance_command(0x686A) && !is_log_maintenance_command(0x6867));

    for (std::uint8_t s = 0; s <= 5; ++s) {
        // Never over radio, in any state.
        assert(!allow_log_maintenance(Source::Radio, s));
        // USB: Pad Idle and Landed only.
        assert(allow_log_maintenance(Source::Usb, s) == (s == kPadIdle || s == kLanded));
    }

    for (std::uint8_t cur = 0; cur <= 4; ++cur) {
        for (std::uint8_t req = 0; req <= 5; ++req) {
            const bool radio = allow_set_flight_state(Source::Radio, cur, req);
            assert(radio == (cur == kLanded && req == kPadIdle));
            const bool usb = allow_set_flight_state(Source::Usb, cur, req);
            if (req > 4) {
                assert(!usb);
            } else if (cur >= kBoost && cur <= kDescent) {
                assert(usb == (req == kLanded || req == cur));
            } else {
                assert(usb);
            }
        }
    }
    assert(!allow_set_flight_state(Source::Radio, kPadIdle, kBoost));  // no radio launch
    assert(!allow_set_flight_state(Source::Radio, kCoast, kPadIdle));
    assert(allow_set_flight_state(Source::Usb, kPadIdle, kBoost));     // bench launch
    assert(!allow_set_flight_state(Source::Usb, kDescent, kPadIdle));
    assert(allow_set_flight_state(Source::Usb, kBoost, kLanded));
    return 0;
}
