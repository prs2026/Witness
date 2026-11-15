#include "command_policy.h"

#include <cassert>
#include <initializer_list>

using namespace command_policy;

int main()
{
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
        // Radio power and GPS assist uploads: refused in Boost only; assist
        // status always allowed.
        assert(allow_radio_power(s) == (s != kBoost));
        for (const std::uint16_t c : {kGpsAssistBegin, kGpsAssistData, kGpsAssistCommit})
            assert(allow_gps_assist(c, s) == (s != kBoost));
        assert(allow_gps_assist(kGpsAssistStatus, s));
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
