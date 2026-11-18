#include "load_state_logic.h"

#include <cassert>
#include <cstdio>

using namespace load_state;

int main()
{
    const Record none{0, 0, 0};
    const Record garbage{~0U, ~0U, ~0U};

    // Warm reset + valid RTC record -> restored.
    BootDecision d = decide_boot(ResetClass::warm, make_record(0x05));
    assert(d.restore && d.mask == 0x05);
    d = decide_boot(ResetClass::warm, make_record(0x0F));
    assert(d.restore && d.mask == 0x0F);
    d = decide_boot(ResetClass::warm, make_record(0xFF));  // out-of-range bits dropped on creation
    assert(d.restore && d.mask == 0x0F);

    // Power-on (cold) -> all off regardless of ANY stored data, even a
    // perfectly valid RTC record (EXT/USB resets keep RTC RAM).
    for (std::uint8_t m = 0; m <= 0x0F; ++m) {
        d = decide_boot(ResetClass::cold, make_record(m));
        assert(!d.restore && d.mask == 0);
    }
    d = decide_boot(ResetClass::cold, garbage);
    assert(!d.restore && d.mask == 0);
    d = decide_boot(ResetClass::cold, none);
    assert(!d.restore && d.mask == 0);

    // Warm reset with a missing/corrupt RTC record -> all off.
    assert(!decide_boot(ResetClass::warm, none).restore);
    assert(!decide_boot(ResetClass::warm, garbage).restore);
    Record bad = make_record(0x07);
    bad.mask ^= 0x01U;
    assert(!record_valid(bad) && !decide_boot(ResetClass::warm, bad).restore);
    bad = make_record(0x07);
    bad.crc ^= 0x80U;
    assert(!decide_boot(ResetClass::warm, bad).restore);
    bad = make_record(0x07);
    bad.magic = 0;
    assert(!decide_boot(ResetClass::warm, bad).restore);
    Record wide{kMagic, 0x80U, record_crc(kMagic, 0x80U)};
    assert(!record_valid(wide) && !decide_boot(ResetClass::warm, wide).restore);
    // Every single-bit flip of a valid record is rejected.
    for (int bit = 0; bit < 96; ++bit) {
        Record r = make_record(0x0F);
        std::uint32_t *word = bit < 32 ? &r.magic : bit < 64 ? &r.mask : &r.crc;
        *word ^= 1U << (bit % 32);
        assert(!decide_boot(ResetClass::warm, r).restore);
    }

    // "All off" command clears the record: nothing to restore afterwards.
    Record rec = record_for_commanded(0x0F);
    assert(decide_boot(ResetClass::warm, rec).restore);
    rec = record_for_commanded(0x00);
    assert(!record_valid(rec));
    d = decide_boot(ResetClass::warm, rec);
    assert(!d.restore && d.mask == 0);
    // Bits other than the load bits do not keep a record alive.
    assert(!record_valid(record_for_commanded(0xF0)));
    assert(record_for_commanded(0x1F).mask == 0x0F);

    std::puts("load_state_test passed");
    return 0;
}
