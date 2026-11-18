#include <cstdio>

#include "imu_supervisor.h"

using namespace imu_supervisor;
static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); ++failures; } } while (0)

int main()
{
    // Normal operation: 2 ms passes with ~2 varying samples never fault.
    {
        Monitor m;
        m.arm(0);
        for (std::uint32_t t = 2; t < 20000; t += 2)
            CHECK(m.on_pass(t, true, 2, 0, true) == Fault::None);
    }
    // An occasional empty pass (catch-up burst) is fine.
    {
        Monitor m;
        m.arm(0);
        CHECK(m.on_pass(2, true, 2, 0, true) == Fault::None);
        CHECK(m.on_pass(2, true, 0, 0, false) == Fault::None);
        CHECK(m.on_pass(30, true, 0, 0, false) == Fault::None);
    }
    // A 500 ms loop stall followed by a full FIFO does not fault.
    {
        Monitor m;
        m.arm(0);
        CHECK(m.on_pass(500, true, 144, 0, true) == Fault::None);
    }
    // No data for 50 ms faults at exactly 50 ms.
    {
        Monitor m;
        m.arm(1000);
        CHECK(m.on_pass(1049, true, 0, 0, false) == Fault::None);
        CHECK(m.on_pass(1050, true, 0, 0, false) == Fault::NoData);
    }
    // Constant data: 99 samples ok, the 100th faults; a varying sample resets.
    {
        Monitor m;
        m.arm(0);
        std::uint32_t t = 0;
        for (int i = 0; i < 49; ++i) CHECK(m.on_pass(t += 2, true, 2, 2, false) == Fault::None);
        CHECK(m.on_pass(t += 2, true, 1, 1, false) == Fault::None);          // 99
        CHECK(m.on_pass(t += 2, true, 1, 1, false) == Fault::ConstantData);  // 100
        m.arm(t);
        for (int i = 0; i < 40; ++i) CHECK(m.on_pass(t += 2, true, 2, 2, false) == Fault::None);
        CHECK(m.on_pass(t += 2, true, 2, 0, true) == Fault::None);  // varying, tail 0
        for (int i = 0; i < 40; ++i) CHECK(m.on_pass(t += 2, true, 2, 2, false) == Fault::None);
    }
    // Read errors: 25 consecutive fault; one good pass resets.
    {
        Monitor m;
        m.arm(0);
        for (int i = 0; i < 24; ++i) CHECK(m.on_pass(i, false, 0, 0, false) == Fault::None);
        CHECK(m.on_pass(24, true, 2, 0, true) == Fault::None);
        for (int i = 0; i < 24; ++i) CHECK(m.on_pass(30 + i, false, 0, 0, false) == Fault::None);
        CHECK(m.on_pass(60, false, 0, 0, false) == Fault::ReadErrors);
    }
    // WHO_AM_I cadence and retry schedule (including millisecond wrap).
    {
        Monitor m;
        m.arm(0);
        CHECK(!m.who_am_i_due(999));
        CHECK(m.who_am_i_due(1000));
        CHECK(!m.who_am_i_due(1500));
        CHECK(m.who_am_i_due(2000));
        m.down(5000);
        CHECK(m.retry_due(5000));
        CHECK(!m.retry_due(5499));
        CHECK(m.retry_due(5500));
        m.down(0xFFFFFF00u);
        CHECK(m.retry_due(0xFFFFFF00u));
        CHECK(!m.retry_due(0xFFFFFFF0u));
        CHECK(m.retry_due(0x00000200u));
    }
    std::printf(failures ? "imu_supervisor: %d failure(s)\n" : "imu_supervisor: all passed\n", failures);
    return failures != 0;
}
