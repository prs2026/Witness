#include "load_state_logic.h"

#include <cassert>
#include <cstdio>

using namespace load_state;

int main()
{
    // Cold boot: always all off, even with a valid record.
    Record r = make_record(0x0F);
    assert(record_valid(r));
    BootDecision d = decide_boot(ResetClass::cold, r);
    assert(!d.restore && d.mask == 0);

    // Warm + valid: restore the mask (mask bits only).
    d = decide_boot(ResetClass::warm, make_record(0x05));
    assert(d.restore && d.mask == 0x05);
    d = decide_boot(ResetClass::warm, make_record(0xFF));  // out-of-range bits dropped on creation
    assert(d.restore && d.mask == 0x0F);

    // Warm + corrupted record: safe default.
    Record bad = make_record(0x07);
    bad.mask ^= 0x01U;
    assert(!record_valid(bad));
    assert(!decide_boot(ResetClass::warm, bad).restore);
    bad = make_record(0x07);
    bad.crc ^= 0x80U;
    assert(!decide_boot(ResetClass::warm, bad).restore);
    bad = make_record(0x07);
    bad.magic = 0;
    assert(!decide_boot(ResetClass::warm, bad).restore);
    // Mask with bits outside the load range is invalid even with a matching CRC.
    Record wide{kMagic, 0x80U, record_crc(kMagic, 0x80U)};
    assert(!record_valid(wide));
    // Uninitialised (zeroed / 0xFF) RTC memory.
    assert(!decide_boot(ResetClass::warm, Record{0, 0, 0}).restore);
    assert(!decide_boot(ResetClass::warm, Record{~0U, ~0U, ~0U}).restore);

    std::puts("load_state_test passed");
    return 0;
}
