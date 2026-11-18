#pragma once

#include <cstdint>

// Pure (no ESP-IDF) logic of the NAND flight logger: which slot a boot logs
// into, how the end of a stream is found when a resumed flight appends to its
// slot, and the write-error policy. Host test: tests/log_slot_logic_test.cpp.
namespace log_slot {

// ---- Slot choice -----------------------------------------------------------

struct SlotState {
    bool valid = false;        // header present and readable (or rebuilt from the marker)
    bool flight = false;       // flight marker present: never erased automatically
    bool appendable = false;   // current store version, so a resume may append
    bool unusable = false;     // its header block failed to erase/program this boot
    std::uint32_t generation = 0;
};

enum class Action : std::uint8_t {
    NewSession,   // erase `slot` (it holds no flight) and start a new session
    Append,       // resumed flight: keep appending to `slot`, nothing is erased
    SlotsFull,    // both slots hold flights and this is a pad boot: log nothing
};

struct Choice {
    Action action = Action::NewSession;
    std::uint8_t slot = 0;
    bool have_newest = false;           // some slot is valid
    std::uint32_t newest_generation = 0;
};

inline std::int32_t generation_diff(std::uint32_t a, std::uint32_t b)
{
    return static_cast<std::int32_t>(a - b);
}

// `resumed_in_flight`: this boot restored Boost/Coast/Descent (a reset during
// the flight). Then the session of the previous boot (the newest generation)
// is the flight in progress: a resumed boot appends to it instead of looking
// for an empty slot, because after Boost the current slot is itself a flight
// slot and an older flight in the other slot must not stop the logging. If
// that slot cannot be appended to (older store version) the normal rule
// applies. Pad boots never append and never erase a flight slot.
inline Choice choose(const SlotState slots[2], bool resumed_in_flight)
{
    Choice c{};
    int newest = -1;
    int newest_flight = -1;
    for (int s = 0; s < 2; ++s) {
        if (!slots[s].valid) continue;
        if (newest < 0 || generation_diff(slots[s].generation, slots[newest].generation) > 0)
            newest = s;
        if (slots[s].flight &&
            (newest_flight < 0 ||
             generation_diff(slots[s].generation, slots[newest_flight].generation) > 0))
            newest_flight = s;
    }
    c.have_newest = newest >= 0;
    c.newest_generation = newest >= 0 ? slots[newest].generation : 0;

    if (resumed_in_flight && newest >= 0 && slots[newest].appendable) {
        c.action = Action::Append;
        c.slot = static_cast<std::uint8_t>(newest);
        return c;
    }
    if (slots[0].valid && slots[0].flight && slots[1].valid && slots[1].flight) {
        c.action = Action::SlotsFull;
        c.slot = static_cast<std::uint8_t>(1 - newest_flight);
        return c;
    }
    c.action = Action::NewSession;
    if (newest_flight >= 0)      c.slot = static_cast<std::uint8_t>(1 - newest_flight);
    else if (!slots[0].valid)    c.slot = 0;
    else if (!slots[1].valid)    c.slot = 1;
    else                         c.slot = static_cast<std::uint8_t>(1 - newest);
    // A slot whose header block cannot be erased is avoided while the other
    // one is free to use (never one that holds a flight).
    const int other = 1 - c.slot;
    if (slots[c.slot].unusable && !slots[other].unusable && !slots[other].flight)
        c.slot = static_cast<std::uint8_t>(other);
    return c;
}

// ---- Finding the end of a stream (append after a resume) -------------------

enum class PageKind : std::uint8_t {
    Erased,       // reads clean, header all 0xFF: free to program
    Valid,        // this session's page (magic, type, version, tag match)
    Unreadable,   // uncorrectable ECC: torn page; counts as written, skipped by readers
    Foreign,      // reads clean but is not ours (older session, stale data)
};

struct PageProbe {
    PageKind kind = PageKind::Erased;
    std::uint32_t sequence = 0;   // low 20 bits of the page sequence (Valid only)
};

constexpr std::uint32_t kSequenceMask = 0xFFFFFU;
constexpr std::uint32_t kPagesPerBlock = 64;

inline bool present(PageKind k) { return k == PageKind::Valid || k == PageKind::Unreadable; }

struct StreamEnd {
    std::uint32_t page_index = 0;      // next page to write (may equal capacity)
    std::uint32_t next_sequence = 0;   // low 20 bits
    bool wrapped = false;              // ring has lapped
    bool empty = true;
};

// Finds where a stream's writer stopped. The newest block is the block whose
// first page is present with, for a ring, the highest sequence (sequences grow
// in write order within the 20-bit span, a stream never exceeds 2^20 pages) or,
// for a linear stream, the highest block index (blocks are filled in order;
// blocks the writer skipped after an error stay empty and never confuse this).
// Inside that block the end is found by binary search: pages are written in
// order and the block was erased when entered, so presence is monotone there.
// Blocks never written (bad, erased, stale data of older sessions) are ignored.
// `probe(i)` returns the PageProbe of page i.
template <class Probe>
StreamEnd stream_end(std::uint32_t capacity, bool ring, Probe probe)
{
    StreamEnd e{};
    const std::uint32_t blocks = capacity / kPagesPerBlock;
    bool found = false;
    std::uint32_t best_block = 0, best_seq = 0;
    for (std::uint32_t b = 0; b < blocks; ++b) {
        const PageProbe p = probe(b * kPagesPerBlock);
        if (!present(p.kind)) continue;
        if (ring) {
            // Only a readable page carries a sequence to rank by.
            if (p.kind != PageKind::Valid) continue;
            if (!found || p.sequence > best_seq) { best_block = b; best_seq = p.sequence; }
        } else {
            best_block = b;
        }
        found = true;
    }
    if (!found) return e;   // empty stream
    e.empty = false;
    std::uint32_t lo = 0, hi = kPagesPerBlock;
    while (lo < hi) {
        const std::uint32_t mid = lo + (hi - lo) / 2;
        if (present(probe(best_block * kPagesPerBlock + mid).kind)) lo = mid + 1; else hi = mid;
    }
    e.page_index = best_block * kPagesPerBlock + lo;
    // Sequence after the last readable page just before the end (skipping a
    // few torn ones); else consecutive numbering.
    e.next_sequence = (ring ? best_seq + lo : e.page_index) & kSequenceMask;
    for (std::uint32_t back = 1; back <= 8 && back <= lo; ++back) {
        const PageProbe p = probe(e.page_index - back);
        if (p.kind == PageKind::Valid) { e.next_sequence = (p.sequence + 1U) & kSequenceMask; break; }
    }
    // A ring has lapped once more pages were written than it holds.
    e.wrapped = ring && (best_seq + lo) >= capacity;
    return e;
}

// Where the writer may continue given the end page `e` of a stream: a page
// that is not erased (torn or garbage) cannot be programmed again, so it is
// skipped. Returns the first usable page index >= e (capacity = none left).
template <class Probe>
std::uint32_t first_free_page(std::uint32_t e, std::uint32_t capacity, Probe probe)
{
    while (e < capacity && probe(e).kind != PageKind::Erased) {
        // Pages at the start of an unentered block are lazily erased when the
        // writer enters the block, so only pages inside an entered block matter.
        if (e % kPagesPerBlock == 0) break;
        ++e;
    }
    return e;
}

// ---- Write-error policy -----------------------------------------------------

constexpr unsigned kEraseAttempts = 3;       // per block
constexpr unsigned kProgramAttempts = 3;     // per page, transient errors only
constexpr unsigned kMaxBlockSkipsPerChunk = 4;
constexpr unsigned kBackoffAfterBlocks = 8;  // consecutive failed blocks
constexpr std::uint32_t kBackoffMs = 3000;   // retry period once backed off

class WriteHealth {
public:
    // A page was programmed (or a block erased) successfully.
    void on_success() { consecutive_failed_blocks_ = 0; failing_ = false; }
    // A block was given up on (erase or program failed after retries).
    void on_block_failed(std::uint32_t now_ms)
    {
        ++consecutive_failed_blocks_;
        ++total_failed_blocks_;
        failing_ = true;
        if (consecutive_failed_blocks_ >= kBackoffAfterBlocks)
            backoff_until_ms_ = now_ms + kBackoffMs;
    }
    // True while writes are paused (retry every kBackoffMs forever).
    bool backing_off(std::uint32_t now_ms) const
    {
        return consecutive_failed_blocks_ >= kBackoffAfterBlocks &&
               static_cast<std::int32_t>(backoff_until_ms_ - now_ms) > 0;
    }
    // FLASH_LOG_FAILED should be asserted for the NAND while this is true.
    bool failing() const { return failing_; }
    std::uint32_t consecutive_failed_blocks() const { return consecutive_failed_blocks_; }
    std::uint32_t total_failed_blocks() const { return total_failed_blocks_; }

private:
    std::uint32_t consecutive_failed_blocks_ = 0;
    std::uint32_t total_failed_blocks_ = 0;
    std::uint32_t backoff_until_ms_ = 0;
    bool failing_ = false;
};

}  // namespace log_slot
