#include "log_slot_logic.h"

#include <cassert>
#include <cstdio>
#include <vector>

using namespace log_slot;

namespace {

SlotState S(bool valid, bool flight, std::uint32_t gen, bool app = true)
{
    SlotState s; s.valid = valid; s.flight = flight; s.generation = gen; s.appendable = app;
    return s;
}

void test_choose()
{
    // Empty chip: slot A, pad or resumed.
    { SlotState s[2] = {S(0,0,0), S(0,0,0)};
      Choice c = choose(s, false);
      assert(c.action == Action::NewSession && c.slot == 0 && !c.have_newest);
      c = choose(s, true);
      assert(c.action == Action::NewSession && c.slot == 0); }
    // Pad boots: never the slot holding the newest flight.
    { SlotState s[2] = {S(1,1,5), S(1,0,6)};
      Choice c = choose(s, false);
      assert(c.action == Action::NewSession && c.slot == 1); }
    // Both full + pad boot -> slots full (never erase).
    { SlotState s[2] = {S(1,1,5), S(1,1,6)};
      Choice c = choose(s, false);
      assert(c.action == Action::SlotsFull); }
    // Both full + resume -> append to the newest flight.
    { SlotState s[2] = {S(1,1,5), S(1,1,6)};
      Choice c = choose(s, true);
      assert(c.action == Action::Append && c.slot == 1 && c.newest_generation == 6); }
    { SlotState s[2] = {S(1,1,9), S(1,1,6)};
      Choice c = choose(s, true);
      assert(c.action == Action::Append && c.slot == 0); }
    // One flight + resumed in the flight slot -> append.
    { SlotState s[2] = {S(0,0,0), S(1,1,4)};
      Choice c = choose(s, true);
      assert(c.action == Action::Append && c.slot == 1); }
    // Resume before the marker was written (newest session not yet a flight),
    // older flight in the other slot: append to the newest session.
    { SlotState s[2] = {S(1,1,5), S(1,0,6)};
      Choice c = choose(s, true);
      assert(c.action == Action::Append && c.slot == 1); }
    // Resume but the newest slot is an old store version: normal rule.
    { SlotState s[2] = {S(1,1,5), S(1,1,6,false)};
      assert(choose(s, true).action == Action::SlotsFull);
      SlotState t[2] = {S(1,1,5), S(1,0,6,false)};
      Choice c = choose(t, true);
      assert(c.action == Action::NewSession && c.slot == 1); }
    // Generation wrap: 0xFFFFFFFF is older than 1.
    { SlotState s[2] = {S(1,1,0xFFFFFFFFu), S(1,1,1)};
      assert(choose(s, true).slot == 1); }
    // A slot whose header block failed is avoided if the other is free.
    { SlotState s[2] = {S(0,0,0), S(1,0,3)};
      s[0].unusable = true;
      assert(choose(s, false).slot == 1);
      s[1].flight = true;   // never a flight slot
      assert(choose(s, false).slot == 0); }
    // Invalid slot first, then older slot.
    { SlotState s[2] = {S(1,0,3), S(0,0,0)};
      assert(choose(s, false).slot == 1); }
    { SlotState s[2] = {S(1,0,3), S(1,0,4)};
      assert(choose(s, false).slot == 0); }
}

// A fake stream: kinds[] per page.
struct Fake {
    std::vector<PageProbe> pages;
    PageProbe operator()(std::uint32_t i) const { return pages.at(i); }
};
Fake linear(std::uint32_t capacity, std::uint32_t written)
{
    Fake f; f.pages.assign(capacity, PageProbe{PageKind::Erased, 0});
    for (std::uint32_t i = 0; i < written; ++i) f.pages[i] = {PageKind::Valid, i};
    // stale data from older sessions after the end
    for (std::uint32_t i = written + 64; i < capacity; ++i) f.pages[i] = {PageKind::Foreign, 0};
    return f;
}

void test_linear_end()
{
    for (std::uint32_t w : {0u, 1u, 63u, 64u, 65u, 500u, 1000u, 1024u}) {
        Fake f = linear(1024, w);
        StreamEnd e = stream_end(1024, false, f);
        assert(e.page_index == w && e.next_sequence == w);
        assert(e.empty == (w == 0));
    }
    // A torn (unreadable) page at the end counts as written; the next sequence
    // follows the last valid page before it.
    { Fake f = linear(1024, 100); f.pages[99] = {PageKind::Unreadable, 0};
      StreamEnd e = stream_end(1024, false, f);
      assert(e.page_index == 100 && e.next_sequence == 99); }
    // Sequence base is a 20-bit counter that continues from the last page.
    { Fake f = linear(256, 10);
      for (std::uint32_t i = 0; i < 10; ++i) f.pages[i].sequence = 5000 + i;
      assert(stream_end(256, false, f).next_sequence == 5010); }
    // A block skipped after a NAND error leaves a gap; the end is after the
    // LAST written block, never inside the gap.
    { Fake f = linear(1024, 200);
      for (std::uint32_t i = 64; i < 128; ++i) f.pages[i] = {PageKind::Erased, 0};   // skipped block
      for (std::uint32_t i = 100; i < 128; ++i) f.pages[i] = {PageKind::Erased, 0};
      StreamEnd e = stream_end(1024, false, f);
      assert(e.page_index == 200 && e.next_sequence == 200); }
    // first_free_page skips a non-erased page inside an entered block...
    { Fake f = linear(256, 70); f.pages[70] = {PageKind::Foreign, 0};
      f.pages[71] = {PageKind::Erased, 0};
      assert(first_free_page(70, 256, f) == 71);
      // ...but a block boundary is erased on entry, so it stops there.
      Fake g = linear(256, 64); g.pages[64] = {PageKind::Foreign, 0};
      assert(first_free_page(64, 256, g) == 64); }
}

Fake ring(std::uint32_t capacity, std::uint32_t total_pages, std::uint32_t base = 0)
{
    // Pages written in order into a ring of `capacity` pages; a block is erased
    // on entry so pages after the write position in the newest block are erased.
    Fake f; f.pages.assign(capacity, PageProbe{PageKind::Foreign, 0});
    std::uint32_t pos = 0;
    for (std::uint32_t n = 0; n < total_pages; ++n) {
        if (pos == capacity) pos = 0;
        if (pos % kPagesPerBlock == 0)
            for (std::uint32_t k = 0; k < kPagesPerBlock; ++k) f.pages[pos + k] = {PageKind::Erased, 0};
        f.pages[pos] = {PageKind::Valid, (base + n) & kSequenceMask};
        ++pos;
    }
    return f;
}

void test_ring_end()
{
    const std::uint32_t cap = 8 * 64;
    assert(stream_end(cap, true, ring(cap, 0)).empty);
    for (std::uint32_t total : {1u, 10u, 64u, 65u, 200u, 511u, 512u}) {
        StreamEnd e = stream_end(cap, true, ring(cap, total));
        assert(e.page_index == total && e.next_sequence == total && !e.empty);
        assert(e.wrapped == (total >= cap));
    }
    // Wrapped: 600 pages in 512 -> position 600-512 = 88, seq 600.
    { StreamEnd e = stream_end(cap, true, ring(cap, 600));
      assert(e.page_index == 88 && e.next_sequence == 600 && e.wrapped); }
    // Exactly at a block boundary after a lap.
    { StreamEnd e = stream_end(cap, true, ring(cap, 576));
      assert(e.page_index == 64 && e.next_sequence == 576); }
    // A fresh session on a ring holding stale pages (Foreign) is not confused.
    { Fake f = ring(cap, 70, 1000);
      StreamEnd e = stream_end(cap, true, f);
      assert(e.page_index == 70 && e.next_sequence == 1070); }
}

void test_write_health()
{
    WriteHealth h;
    assert(!h.failing() && !h.backing_off(0));
    h.on_block_failed(1000);
    assert(h.failing() && !h.backing_off(1000));
    h.on_success();
    assert(!h.failing() && h.consecutive_failed_blocks() == 0 && h.total_failed_blocks() == 1);
    for (unsigned i = 0; i < kBackoffAfterBlocks; ++i) h.on_block_failed(5000);
    assert(h.backing_off(5000) && h.backing_off(7999) && !h.backing_off(8000));
    // After the pause a further failure restarts the pause (retry forever).
    h.on_block_failed(8000);
    assert(h.backing_off(8001) && !h.backing_off(11000));
    h.on_success();
    assert(!h.backing_off(8001) && !h.failing());
    // Millisecond clock wrap.
    WriteHealth w;
    for (unsigned i = 0; i < kBackoffAfterBlocks; ++i) w.on_block_failed(0xFFFFFF00u);
    assert(w.backing_off(0xFFFFFF10u) && !w.backing_off(0x00000F00u));
}

}  // namespace

int main()
{
    test_choose();
    test_linear_end();
    test_ring_end();
    test_write_health();
    std::puts("log_slot_logic_test: all passed");
    return 0;
}
