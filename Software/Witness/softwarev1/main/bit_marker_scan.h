#pragma once

// BIT check 6.3 (log end-to-end): finding the marker again in the pages that were
// written after it was queued. Pure logic, host test tests/bit_marker_scan_test.cpp.
//
// The marker is queued behind whatever the packet stream carries and is flushed with
// it, so it can be many pages away from the end of the stream: at the 50 Hz packet
// rate one flush interval writes about three pages after it. The first version
// looked at the last three pages only and missed it whenever the traffic after the
// marker filled more than that. The scan now starts at the page the writer was on
// when the marker was queued and walks forward (the marker is in the first pages
// after that), and it matches across a page boundary (a record that straddles two
// pages of one write is split between them).

#include <cstddef>
#include <cstdint>

namespace bit_marker {

inline constexpr std::size_t kNeedleLength = 8;      // "BIT", version, 4 byte nonce
inline constexpr std::uint32_t kMaxScanPages = 40;   // reads are cheap, but bounded

// Pages to read: from the page the writer was at when the marker was queued up to the
// page it is at now (the next one to write), around the ring when it wrapped, bounded.
inline std::uint32_t pages_to_scan(const std::uint32_t start_index, const std::uint32_t now_index,
                                   const std::uint32_t capacity)
{
    if (capacity == 0U) return 0U;
    const std::uint32_t n = now_index >= start_index ? now_index - start_index
                                                     : now_index + capacity - start_index;
    return n > kMaxScanPages ? kMaxScanPages : n;
}

class Scanner {
public:
    explicit Scanner(const std::uint8_t (&needle)[kNeedleLength])
    {
        for (std::size_t i = 0; i < kNeedleLength; ++i) needle_[i] = needle[i];
    }

    // The next page is not the continuation of the last one (unreadable or skipped).
    void gap() { carry_ = 0; }

    // Feeds the payload of the next page; true when the needle was seen, within
    // the payload or across the join with the end of the previous one.
    bool feed(const std::uint8_t *data, const std::size_t length)
    {
        const std::size_t total = carry_ + length;
        const auto at = [&](const std::size_t i) { return i < carry_ ? carry_bytes_[i] : data[i - carry_]; };
        for (std::size_t i = 0; i + kNeedleLength <= total; ++i) {
            if (at(i) != needle_[0]) continue;
            std::size_t k = 1;
            while (k < kNeedleLength && at(i + k) == needle_[k]) ++k;
            if (k == kNeedleLength) return true;
        }
        const std::size_t keep = total < kNeedleLength - 1U ? total : kNeedleLength - 1U;
        std::uint8_t next[kNeedleLength]{};
        for (std::size_t i = 0; i < keep; ++i) next[i] = at(total - keep + i);
        for (std::size_t i = 0; i < keep; ++i) carry_bytes_[i] = next[i];
        carry_ = keep;
        return false;
    }

private:
    std::uint8_t needle_[kNeedleLength]{};
    std::uint8_t carry_bytes_[kNeedleLength]{};
    std::size_t carry_ = 0;
};

}  // namespace bit_marker
