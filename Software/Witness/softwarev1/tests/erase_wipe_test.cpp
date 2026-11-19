// g++ -std=c++17 -Wall -Wextra -Werror -I main tests/erase_wipe_test.cpp -o build/erase_wipe_test.exe
#include "erase_wipe.h"

#include <cassert>
#include <cstdio>
#include <vector>

using namespace erase_wipe;

static std::vector<bool> run(std::uint8_t current_slot, std::uint32_t &chunks, std::uint32_t &biggest)
{
    BackgroundWipe w;
    w.start(502, 2);
    std::vector<bool> erased(1004, false);
    chunks = biggest = 0;
    std::uint32_t first = 0, count = 0;
    while (w.next(current_slot, true, first, count)) {
        assert(count >= 1 && count <= kChunkBlocks);
        for (std::uint32_t b = first; b < first + count; ++b) {
            assert(!erased[b]);
            erased[b] = true;
        }
        ++chunks;
        if (count > biggest) biggest = count;
    }
    assert(!w.active());
    return erased;
}

int main()
{
    // Current slot 0: the whole of slot 1 is wiped, slot 0 never touched.
    for (std::uint8_t current : {0, 1}) {
        std::uint32_t chunks = 0, biggest = 0;
        const auto erased = run(current, chunks, biggest);
        std::uint32_t n = 0;
        for (std::uint32_t b = 0; b < 1004; ++b) {
            const bool in_current = b / 502 == current;
            assert(erased[b] == !in_current);
            if (erased[b]) ++n;
        }
        assert(n == 502);
        assert(biggest == kChunkBlocks);
        assert(chunks == (502 + kChunkBlocks - 1) / kChunkBlocks);
    }
    // A launch cancels the wipe for good.
    {
        BackgroundWipe w;
        w.start(502, 2);
        std::uint32_t first = 0, count = 0;
        assert(w.next(0, true, first, count));
        assert(!w.next(0, false, first, count) && count == 0);
        assert(!w.active());
        assert(!w.next(0, true, first, count));
    }
    // Nothing to do before start() and after cancel().
    {
        BackgroundWipe w;
        std::uint32_t first = 0, count = 0;
        assert(!w.next(0, true, first, count));
        w.start(502, 2);
        w.cancel();
        assert(!w.next(0, true, first, count));
    }
    // The current slot changing during the wipe is respected from then on.
    {
        BackgroundWipe w;
        w.start(502, 2);
        std::uint32_t first = 0, count = 0;
        assert(w.next(1, true, first, count) && first == 0);   // slot 0 first
        for (int i = 0; i < 1000 && w.next(0, true, first, count); ++i) assert(first >= 502);
    }
    std::puts("erase_wipe_test OK");
    return 0;
}
