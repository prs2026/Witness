#pragma once

#include <cstdint>
#include <cstring>

// Flight state persisted in RTC_NOINIT memory so an in-flight reset (panic,
// watchdog, brownout, software restart) resumes Boost/Coast/Descent instead of
// restarting in Pad Idle. See FIRMWARE_HANDOFF.md, "Flight state across
// resets". The record logic here is pure (host tested); the RTC variable and
// the reset-reason mapping live in flight_state_store.cpp.
namespace flight_state_store {

struct Record {
    std::uint32_t magic;
    std::uint32_t state;           // FlightStateMachine::State value
    float maximum_altitude_m;
    std::uint32_t maximum_valid;
    std::uint32_t checksum;
};

constexpr std::uint32_t kMagic = 0x57465354U;   // "WFST"

inline std::uint32_t compute_checksum(const Record &r)
{
    std::uint32_t altitude_bits = 0;
    std::memcpy(&altitude_bits, &r.maximum_altitude_m, sizeof(altitude_bits));
    std::uint32_t sum = 0xA5C3E17BU;
    sum = (sum ^ r.magic) * 16777619U;
    sum = (sum ^ r.state) * 16777619U;
    sum = (sum ^ altitude_bits) * 16777619U;
    sum = (sum ^ r.maximum_valid) * 16777619U;
    return sum;
}

inline void seal(Record &r, const std::uint8_t state, const float maximum_m,
                 const bool maximum_valid)
{
    r.magic = 0;   // an interrupted write is invalid, not half-valid
    r.state = state;
    r.maximum_altitude_m = maximum_m;
    r.maximum_valid = maximum_valid ? 1U : 0U;
    r.magic = kMagic;
    r.checksum = compute_checksum(r);
}

inline bool valid(const Record &r)
{
    return r.magic == kMagic && r.checksum == compute_checksum(r) &&
           r.state <= 4U;
}

// Whether the reset that started this boot can have kept RTC memory.
enum class ResetKind : std::uint8_t {
    Cold,   // power-on, external pin, USB/JTAG, deep sleep, unknown: fresh start
    Warm,   // panic, watchdogs, software restart, brownout
};

// In-flight states resumed: Boost (1), Coast (2), Descent (3). Pad Idle needs
// no resume; Landed is deliberately NOT resumed (see handoff).
inline bool resumable(const Record &r, const ResetKind kind)
{
    return kind == ResetKind::Warm && valid(r) && r.state >= 1U &&
           r.state <= 3U;
}

struct Snapshot {
    std::uint8_t state = 0;
    float maximum_altitude_m = 0.0F;
    bool maximum_valid = false;
};

// Call once at boot, before anything uses the flight state. Returns true and
// fills `out` when an in-flight state is to be resumed. Otherwise the record
// is reset to Pad Idle.
bool take_boot_snapshot(Snapshot &out);
// FlightStateMachine::PersistFn: record a state change.
void save(std::uint8_t state, float maximum_altitude_m, bool maximum_valid);

}  // namespace flight_state_store
