#pragma once

// Background wipe for the full erase (0x6869), pure logic, host test
// tests/erase_wipe_test.cpp.
//
// The full erase used to erase all 1004 log blocks in one go (4.3 s) inside the
// logger task: no packet could be logged meanwhile (202 packet records dropped
// by the queue) and the USB command task waited for the whole time. Now the
// erase is the fast clear (both slot headers, ~40 ms, a new session at once)
// followed by this wipe: the blocks of the slot that is NOT the current
// session are erased in small chunks between normal logger passes, and only
// while the vehicle is on the ground. Blocks of the current slot are erased
// lazily by the writer before it uses them (as after a clear). Old pages are
// invisible to readers either way: the session number keeps counting up and
// the page tags of older sessions never match it.

#include <cstdint>

namespace erase_wipe {

inline constexpr std::uint32_t kChunkBlocks = 8;   // ~35 ms of NAND time

class BackgroundWipe {
public:
    void start(const std::uint32_t blocks_per_slot, const std::uint32_t slot_count)
    {
        blocks_per_slot_ = blocks_per_slot;
        total_ = blocks_per_slot * slot_count;
        next_ = 0;
        active_ = total_ != 0U;
        erased_ = 0;
    }
    void cancel() { active_ = false; }
    bool active() const { return active_; }
    std::uint32_t erased() const { return erased_; }

    // The next run of up to kChunkBlocks blocks to erase, none of them in the
    // current slot. False when nothing is left (the wipe ends) or the vehicle
    // is not on the ground (the wipe is cancelled and never resumes).
    bool next(const std::uint8_t current_slot, const bool on_ground,
              std::uint32_t &first, std::uint32_t &count)
    {
        count = 0;
        if (!active_) return false;
        if (!on_ground) {
            active_ = false;
            return false;
        }
        const std::uint32_t skip_begin = current_slot * blocks_per_slot_;
        const std::uint32_t skip_end = skip_begin + blocks_per_slot_;
        if (next_ >= skip_begin && next_ < skip_end) next_ = skip_end;
        if (next_ >= total_) {
            active_ = false;
            return false;
        }
        first = next_;
        std::uint32_t end = next_ + kChunkBlocks;
        if (end > total_) end = total_;
        if (next_ < skip_begin && end > skip_begin) end = skip_begin;
        count = end - next_;
        next_ = end;
        erased_ += count;
        return true;
    }

private:
    bool active_ = false;
    std::uint32_t blocks_per_slot_ = 0;
    std::uint32_t total_ = 0;
    std::uint32_t next_ = 0;
    std::uint32_t erased_ = 0;
};

}  // namespace erase_wipe
