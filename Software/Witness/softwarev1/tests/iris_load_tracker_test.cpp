#include "iris_load_tracker.h"

#include <cassert>
#include <cstdio>

using namespace iris_load_tracker;

static std::uint8_t flags(std::uint8_t mask) { return static_cast<std::uint8_t>(mask << 3); }

int main()
{
    Tracker t;
    // Nothing commanded: never resend.
    assert(t.on_status(0, 0, 1000).count == 0);
    assert(t.on_status(1000, 0, 100).count == 0);  // reboot but nothing known

    t.note_command(0x31, 10000);  // CH1 on
    t.note_command(0x35, 10000);  // CH3 on
    t.note_command(0x51, 10000);  // 5V on
    t.note_command(0x34, 10000);  // CH3 off
    assert(t.commanded() == 0x09 && t.known() == 0x0D);
    t.note_command(0x03, 10000);  // ignored
    assert(t.known() == 0x0D);

    // Matching state: nothing.
    for (int i = 0; i < 5; ++i) assert(t.on_status(12000 + i * 1000, flags(0x09), 50000 + i * 1000).count == 0);

    // Stale packet right after a command (grace): no resend.
    t.note_command(0x33, 20000);  // CH2 on
    assert(t.on_status(20500, flags(0x09), 90000).count == 0);
    assert(t.on_status(21000, flags(0x09), 91000).count == 0);
    assert(t.on_status(22000, flags(0x0B), 92000).count == 0);

    // Persistent mismatch (Iris lost CH1): first sight starts timer, resend after >=0.9 s.
    assert(t.on_status(30000, flags(0x0A), 100000).count == 0);
    Action a = t.on_status(31000, flags(0x0A), 101000);
    assert(a.count == 4);
    bool has_ch3_off = false, has_5v_on = false;
    for (int i = 0; i < a.count; ++i) { has_ch3_off |= a.opcodes[i] == 0x34; has_5v_on |= a.opcodes[i] == 0x51; }
    assert(has_ch3_off && has_5v_on);
    // Rate limit with backoff: still mismatched 1 s later -> no resend; the
    // second re-send waits 4 s after the first (2 s doubled), the next 8 s.
    assert(t.on_status(32000, flags(0x0A), 102000).count == 0);
    assert(t.on_status(33100, flags(0x0A), 103100).count == 0);
    assert(t.on_status(35000, flags(0x0A), 105000).count == 4);
    assert(t.on_status(39000, flags(0x0A), 109000).count == 0);
    assert(t.on_status(43000, flags(0x0A), 113000).count == 4);

    // Reboot: uptime goes backwards -> immediate resend even if flags show all off.
    t.on_status(44000, flags(0x0B), 200000);
    a = t.on_status(45000, flags(0x00), 300);
    assert(a.count == 4);
    // Immediate second reboot report inside 500 ms is limited.
    assert(t.on_status(45200, flags(0x00), 100).count == 0);

    // Commanded OFF but Iris reports ON for >0.9 s: resend OFF.
    Tracker u;
    u.note_command(0x30, 0);
    assert(u.on_status(5000, flags(0x01), 1).count == 0);
    a = u.on_status(6000, flags(0x01), 2);
    assert(a.count == 1 && a.opcodes[0] == 0x30);

    // Backoff is capped (30 s) and never stops: a stuck mismatch keeps being
    // retried at the slow rate, never at the 2 s rate.
    Tracker v;
    v.note_command(0x31, 0);
    std::int64_t now = 100000;
    int sent = 0;
    std::int64_t last_sent = -1, max_gap = 0, min_gap_late = 1 << 30;
    for (int i = 0; i < 1200; ++i, now += 1000) {  // 20 minutes at 1 Hz
        if (v.on_status(now, flags(0x00), 1000 + i * 1000).count != 0) {
            if (last_sent >= 0) {
                max_gap = now - last_sent > max_gap ? now - last_sent : max_gap;
                if (sent > 6 && now - last_sent < min_gap_late) min_gap_late = now - last_sent;
            }
            last_sent = now;
            ++sent;
        }
    }
    assert(max_gap <= 31000 && min_gap_late >= 30000 && sent > 30 && sent < 60);
    // A match or a new command restarts the fast schedule.
    assert(v.on_status(now, flags(0x01), 5000000).count == 0);
    v.note_command(0x31, now);
    assert(v.on_status(now + 100, flags(0x00), 5001000).count == 0);  // grace
    assert(v.on_status(now + 1600, flags(0x00), 5002000).count == 0);  // timer starts
    assert(v.on_status(now + 2600, flags(0x00), 5003000).count == 1);  // >= 0.9 s

    // Witness reset: a fresh tracker knows nothing and sends nothing, whatever
    // Iris reports (it must not override what Iris restored from its NVS).
    Tracker w;
    for (int i = 0; i < 100; ++i) {
        assert(w.on_status(i * 1000, flags(i & 1 ? 0x0F : 0x00), (i % 7 == 0) ? 10 : 1000 + i).count == 0);
    }

    std::puts("iris_load_tracker_test passed");
    return 0;
}
