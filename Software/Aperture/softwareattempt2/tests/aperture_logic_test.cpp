// Host test for Aperture's pure logic: USB frame ring, command resync, uplink pacing.
//   g++ -std=c++17 -Wall -Wextra -Werror -I main tests/aperture_logic_test.cpp -o build/aperture_logic_test.exe
#undef NDEBUG
#include "command_parser.h"
#include "frame_ring.h"
#include "tx_slot.h"
#include "uplink_pacing.h"

#include <cassert>
#include <cstdio>
#include <vector>

static std::vector<std::uint8_t> cmd(std::uint8_t tag)
{
    return {0x05, 0, 0, 0, 0, 0, 0, 0x50, tag, 0x0A};
}

static void test_ring()
{
    FrameRing<40> r;
    std::uint8_t a[10], out[255];
    std::uint32_t seq = 0;
    for (int i = 0; i < 10; ++i) a[i] = static_cast<std::uint8_t>(i);
    assert(r.push(a, 10) && r.push(a, 10) && r.push(a, 10));   // 33 of 40 bytes
    a[0] = 99;
    assert(r.push(a, 10));                                      // drops the oldest
    assert(r.drops() == 1 && r.frames() == 3);
    // peek then a push that drops the head: pop_if must refuse
    assert(r.peek(out, sizeof(out), seq) == 10 && out[0] == 0);
    assert(r.push(a, 10));
    assert(!r.pop_if(seq));
    assert(r.peek(out, sizeof(out), seq) == 10);
    assert(r.pop_if(seq));
    // wrap-around integrity
    for (int n = 0; n < 50; ++n) {
        a[1] = static_cast<std::uint8_t>(n);
        r.push(a, 10);
        assert(r.peek(out, sizeof(out), seq) == 10 && out[9] == 9);
    }
    assert(!r.push(a, 41));          // can never fit
    std::puts("ring ok");
}

static std::size_t feed_all(CommandAssembler &p, const std::vector<std::uint8_t> &bytes,
                            std::vector<std::vector<std::uint8_t>> &frames)
{
    for (auto b : bytes)
        if (p.feed(b)) frames.emplace_back(p.frame(), p.frame() + 10);
    return frames.size();
}

static void test_parser()
{
    CommandAssembler p;
    std::vector<std::vector<std::uint8_t>> f;
    auto a = cmd(1), b = cmd(2), c = cmd(3);
    std::vector<std::uint8_t> s = a;
    s.insert(s.end(), b.begin(), b.end());
    s.insert(s.end(), c.begin(), c.end());
    assert(feed_all(p, s, f) == 3);
    // one byte lost inside the first frame: frames 2 and 3 must survive
    f.clear();
    CommandAssembler q;
    std::vector<std::uint8_t> t = a;
    t.erase(t.begin() + 4);
    t.insert(t.end(), b.begin(), b.end());
    t.insert(t.end(), c.begin(), c.end());
    feed_all(q, t, f);
    assert(f.size() == 2 && f[0] == b && f[1] == c);
    assert(q.resyncs() >= 1);
    // garbage between frames
    f.clear();
    CommandAssembler r;
    std::vector<std::uint8_t> g = {0x11, 0x22};
    g.insert(g.end(), a.begin(), a.end());
    feed_all(r, g, f);
    assert(f.size() == 1 && f[0] == a);
    std::puts("parser ok");
}

static void test_pacing()
{
    using namespace uplink_pacing;
    Pacer pc;
    std::uint64_t t = 1000;
    // queue of 10 frames, loop polls every 10 ms, each send takes 20 ms
    std::uint64_t first = 0, sent = 0, last = 0;
    std::vector<std::uint64_t> times;
    for (; t < 20000 && sent < 10; t += 10) {
        if (pc.allow(t)) { pc.on_sent(t); t += 20; times.push_back(t); ++sent; }
    }
    assert(sent == 10);
    (void)first; (void)last;
    // bursts of 3, with >= 1500 ms between the last of one and first of next
    for (std::size_t i = 3; i < times.size(); i += 3)
        assert(times[i] - times[i - 1] >= kQuietMs);
    for (std::size_t i = 1; i < 3; ++i) assert(times[i] - times[i - 1] < 100);
    // a slow trickle (one per 1.0 s) is still cut after 3 and given a quiet gap
    Pacer q;
    unsigned sends = 0;
    std::uint64_t quiet_start = 0, max_quiet = 0, prev = 0;
    for (std::uint64_t n = 0; n < 20000; n += 10) {
        if (n % 1000 == 0 && q.allow(n)) { q.on_sent(n); ++sends; if (prev && n - prev > max_quiet) max_quiet = n - prev; prev = n; }
    }
    (void)quiet_start;
    assert(max_quiet >= kQuietMs);
    assert(sends < 20);
    // after the quiet time the tx_slot rule still applies independently
    assert(tx_slot::aperture_may_send(5000, false, 0, false, 0, 11));
    std::puts("pacing ok");
}

static void test_queue()
{
    uplink_pacing::Queue<10, 4> q;
    using P = uplink_pacing::Queue<10, 4>::Push;
    auto a = cmd(1), b = cmd(2);
    assert(q.push(a.data()) == P::Queued);
    assert(q.push(a.data()) == P::Duplicate);
    assert(q.push(b.data()) == P::Queued);
    assert(q.size() == 2);
    q.pop();
    assert(q.push(a.data()) == P::Queued);   // no longer queued: accepted again
    for (std::uint8_t i = 10; q.size() < 4; ++i) q.push(cmd(i).data());
    assert(q.push(cmd(99).data()) == P::Full);
    std::puts("queue ok");
}

int main()
{
    test_ring();
    test_parser();
    test_pacing();
    test_queue();
    std::puts("ALL OK");
    return 0;
}
