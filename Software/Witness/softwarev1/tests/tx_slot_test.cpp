// Host test for main/tx_slot.h (GPS time slots and the uplink hold-off).
//   g++ -std=c++17 -Wall -Wextra -Werror -I main tests/tx_slot_test.cpp -o build/tx_slot_test.exe
#include "tx_slot.h"

#include <cassert>
#include <cstdio>

using namespace tx_slot;

int main()
{
    // Airtime at SF7/500 kHz, CR 4/5, 8-symbol preamble.
    assert(lora_airtime_ms(10) == 11);    // command frame
    assert(lora_airtime_ms(65) == 31);    // state frame
    assert(lora_airtime_ms(103) == 44);   // debug frame (largest)
    assert(lora_airtime_ms(0) >= 5);

    // Witness without GPS time and without uplink: always send (old behaviour).
    for (std::uint64_t t = 0; t < 3000; t += 7)
        assert(witness_telemetry(t, false, 0, false, 0.0, 31) == Decision::Send);

    // Hold-off: 1 s after a command frame, with or without GPS time.
    assert(witness_telemetry(5000, true, 4500, false, 0.0, 31) == Decision::HoldOff);
    assert(witness_telemetry(5499, true, 4500, true, 100.0, 31) == Decision::HoldOff);
    assert(witness_telemetry(5500, true, 4500, false, 0.0, 31) == Decision::Send);

    // Slotted: the packet must end by 700 ms (GGA latency added to the clock).
    const double second = 12.0 * 3600e3 + 34e3;   // 12:00:34.000
    assert(witness_telemetry(0, false, 0, true, second + 0.0, 31) == Decision::Send);
    assert(witness_telemetry(0, false, 0, true, second + 649.0, 31) == Decision::Send);   // 669+31=700
    assert(witness_telemetry(0, false, 0, true, second + 650.0, 31) == Decision::Wait);
    assert(witness_telemetry(0, false, 0, true, second + 900.0, 31) == Decision::Wait);
    assert(witness_telemetry(0, false, 0, true, second + 985.0, 31) == Decision::Send);   // wraps to 5 ms
    // Every downlink start in a second ends before the uplink window opens,
    // even with the worst +-50 ms clock error.
    for (int ms = 0; ms < 1000; ++ms) {
        const auto d = witness_telemetry(0, false, 0, true, second + ms, 44);
        if (d == Decision::Send) {
            const double start = phase_ms(second + ms + kGgaLatencyMs);
            assert(start + 44 + 50 <= kUplinkStartMs);
        }
    }

    // Aperture: no PPS -> immediately; with PPS only in 800..950 (ending by 950).
    assert(aperture_may_send(100, false, 0, false, 0, 11));
    assert(!aperture_may_send(100, false, 0, true, 799, 11));
    assert(aperture_may_send(100, false, 0, true, 800, 11));
    assert(aperture_may_send(100, false, 0, true, 939, 11));
    assert(!aperture_may_send(100, false, 0, true, 940, 11));
    assert(!aperture_may_send(100, false, 0, true, 1200, 11));   // 200 ms into the next second
    assert(aperture_may_send(100, false, 0, true, 1850, 11));    // 850 ms, PPS 1 s old
    // Burst: right after a slotted command, the next one goes at once.
    assert(aperture_may_send(1500, true, 1000, true, 300, 11));
    assert(!aperture_may_send(1900, true, 1000, true, 300, 11));
    // Aperture's window never overlaps Witness's (with a 50 ms error each way
    // on Witness, and both ending inside their windows).
    static_assert(kDownlinkEndMs + 2 * 50 <= kUplinkStartMs, "guard between windows");
    static_assert(kUplinkEndMs + 50 <= kSecondMs, "guard before the next second");
    static_assert(kUplinkBurstMs < kUplinkHoldOffMs, "burst ends inside Witness hold-off");
    std::puts("tx_slot tests passed");
    return 0;
}
