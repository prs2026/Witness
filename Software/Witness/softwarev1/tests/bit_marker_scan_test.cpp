// g++ -std=c++17 -Wall -Wextra -Werror -I main tests/bit_marker_scan_test.cpp -o build/bit_marker_scan_test.exe
#include "bit_marker_scan.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace bit_marker;

static const std::uint8_t kNeedle[kNeedleLength] = {'B', 'I', 'T', 1, 0x11, 0x22, 0x33, 0x44};

// A packet stream of `records` filler records, the marker record after `before` of them.
static std::vector<std::uint8_t> stream(const unsigned before, const unsigned records)
{
    std::vector<std::uint8_t> s;
    for (unsigned r = 0; r < records; ++r) {
        if (r == before) {
            const std::uint8_t head[6] = {0, 0, 0, 1, 7, 12};   // us, source, length
            s.insert(s.end(), head, head + 6);
            s.insert(s.end(), kNeedle, kNeedle + 8);
            const std::uint8_t tail[4] = {'E', 'N', 'D', 0x0A};
            s.insert(s.end(), tail, tail + 4);
        }
        std::vector<std::uint8_t> rec(109, static_cast<std::uint8_t>(0x30 + r % 7));
        s.insert(s.end(), rec.begin(), rec.end());
    }
    return s;
}

// Cuts the stream into pages of `page` payload bytes and scans them; the page the
// marker is on is returned (-1 when not found).
static int scan(const std::vector<std::uint8_t> &s, const std::size_t page)
{
    Scanner sc(kNeedle);
    int index = 0;
    for (std::size_t off = 0; off < s.size(); off += page, ++index) {
        const std::size_t n = s.size() - off < page ? s.size() - off : page;
        if (sc.feed(s.data() + off, n)) return index;
    }
    return -1;
}

int main()
{
    // Found wherever it is, including more than 3 pages from the end (the old failure).
    for (unsigned before = 0; before < 60; ++before) {
        const auto s = stream(before, 60);
        assert(scan(s, 2036) >= 0);
    }
    // Split over a page boundary at every possible offset of the needle.
    {
        const auto s = stream(3, 6);
        std::size_t at = 0;
        for (std::size_t i = 0; i + 8 <= s.size(); ++i)
            if (std::memcmp(s.data() + i, kNeedle, 8) == 0) { at = i; break; }
        for (std::size_t cut = at; cut <= at + 8; ++cut) {
            Scanner sc(kNeedle);
            const bool first = sc.feed(s.data(), cut);
            assert(first == (cut >= at + 8));
            assert(first || sc.feed(s.data() + cut, s.size() - cut));
        }
    }
    // Not there: not found. A partial prefix at the end of the data does not count.
    {
        std::vector<std::uint8_t> s(5000, 0x55);
        assert(scan(s, 2036) == -1);
        s[4999] = 'B'; s[4998] = 'T';
        assert(scan(s, 2036) == -1);
        // A different nonce is not the marker.
        std::uint8_t other[8] = {'B', 'I', 'T', 1, 0x11, 0x22, 0x33, 0x45};
        std::vector<std::uint8_t> t(100, 0);
        std::memcpy(t.data() + 20, other, 8);
        Scanner sc(kNeedle);
        assert(!sc.feed(t.data(), t.size()));
    }
    // A gap (unreadable page) breaks the join.
    {
        Scanner sc(kNeedle);
        assert(!sc.feed(kNeedle, 5));
        sc.gap();
        assert(!sc.feed(kNeedle + 5, 3));
    }
    // Pages to scan: forward, around the ring, bounded.
    assert(pages_to_scan(10, 13, 1000) == 3);
    assert(pages_to_scan(10, 10, 1000) == 0);
    assert(pages_to_scan(998, 2, 1000) == 4);
    assert(pages_to_scan(0, 900, 1000) == kMaxScanPages);
    assert(pages_to_scan(0, 5, 0) == 0);
    std::puts("bit_marker_scan_test: ok");
    return 0;
}
