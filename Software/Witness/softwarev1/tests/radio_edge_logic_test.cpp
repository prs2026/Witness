// g++ -std=c++17 -Wall -Wextra -Werror -I main tests/radio_edge_logic_test.cpp -o build/radio_edge_logic_test.exe
#include "radio_edge_logic.h"

#include <cassert>
#include <cstdio>

using namespace radio_edge;

int main()
{
    // ---- Stuck-low DIO1: slow polling keeps the uplink alive ----
    {
        Dio1SlowPoll p;
        assert(p.should_poll(true, 0));             // normal path: always read
        assert(p.should_poll(true, 1));
        // Line low: first read at once, then once per 200 ms.
        assert(p.should_poll(false, 1000));
        assert(!p.should_poll(false, 1010));
        assert(!p.should_poll(false, 1199));
        assert(p.should_poll(false, 1200));
        assert(!p.should_poll(false, 1201));
        // A frame found with the line low is counted; one found with it high is not.
        assert(p.note_frame(false));
        assert(!p.note_frame(true));
        assert(p.frames_found_with_dio1_low() == 1);
        Dio1SlowPoll q;
        assert(q.should_poll(false, 0xFFFFFFF0ULL));
        assert(!q.should_poll(false, 0xFFFFFFF0ULL + 100));
        assert(q.should_poll(false, 0xFFFFFFF0ULL + 200));
    }
    // ---- 3-byte frames never arm the hold-off ----
    assert(arms_holdoff(10, true, true));
    assert(!arms_holdoff(3, true, true));       // compact frame
    assert(!arms_holdoff(9, true, true));
    assert(!arms_holdoff(11, true, true));
    assert(!arms_holdoff(10, false, true));     // the bridge did not accept it
    assert(!arms_holdoff(10, true, false));     // Boost
    // ---- Ack pacing ----
    {
        AckPacer a;
        assert(!a.may_transmit(false, 0));
        // No command heard yet: an ack goes at once.
        assert(a.may_transmit(true, 100));
        a.note_sent();
        // A burst: frames at 1000, 1015, 1030. The first ack must wait.
        a.note_command_rx(1000);
        assert(!a.may_transmit(true, 1001));
        a.note_command_rx(1015);
        assert(!a.may_transmit(true, 1020));
        a.note_command_rx(1030);
        assert(!a.may_transmit(true, 1060));
        assert(!a.may_transmit(true, 1069));
        assert(a.may_transmit(true, 1070));         // 40 ms of quiet after the last frame
        a.note_sent();
        // The next queued ack: quiet already, so it follows at once, in order.
        assert(a.may_transmit(true, 1071));
        a.note_sent();
        assert(a.may_transmit(true, 1072));
        a.note_sent();
        assert(!a.may_transmit(false, 1073));
    }
    {
        // A command stream that never pauses cannot hold an ack longer than 300 ms.
        AckPacer a;
        std::uint64_t t = 5000;
        a.note_command_rx(t);
        assert(!a.may_transmit(true, t));
        bool sent = false;
        for (t = 5010; t < 6000; t += 10) {
            a.note_command_rx(t);                   // a frame every 10 ms
            if (a.may_transmit(true, t)) { sent = true; break; }
        }
        assert(sent && t - 5000 >= kAckMaxWaitMs && t - 5000 <= kAckMaxWaitMs + 10);
    }
    // ---- Sequence counters (flush) ----
    assert(sequence_reached(5, 5) && sequence_reached(6, 5) && !sequence_reached(4, 5));
    assert(sequence_reached(2, 0xFFFFFFFEU));       // wrap: done is past target
    assert(!sequence_reached(0xFFFFFFFEU, 2));
    std::puts("radio_edge_logic_test OK");
    return 0;
}
