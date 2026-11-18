#include "recovery_logic.h"

#include <cassert>
#include <cstdio>

using namespace iris_recovery;

int main()
{
    I2cFailureTracker t;
    assert(!t.note(false, 1000));          // 1st failure: not yet
    assert(t.note(false, 2000));           // 2nd: recover
    assert(!t.note(false, 3000));          // spacing
    assert(!t.note(false, 400000));
    assert(t.note(false, 600000));         // >= 500 ms after the last one
    assert(!t.note(true, 700000));         // success resets
    assert(!t.note(false, 800000));
    assert(t.note(false, 1200000));

    assert(!tx_slots_stuck(7, 8, 5000000, 0, 1000000));
    assert(!tx_slots_stuck(8, 8, 500000, 0, 1000000));
    assert(tx_slots_stuck(8, 8, 1000000, 0, 1000000));
    std::puts("recovery_logic_test passed");
    return 0;
}
