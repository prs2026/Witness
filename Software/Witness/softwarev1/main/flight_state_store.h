#pragma once

#include <cstddef>
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

// Pad references and the last motion estimate, kept in a SECOND RTC record so
// the state Record above (and its validation) is unchanged: a record written
// by older firmware, or garbage, simply fails this record's own magic and
// checksum and the resume proceeds without references (AGL then restarts at
// the reset position). All fields are 32-bit words (doubles split in two), so
// the record has no padding and a torn write fails the checksum.
struct PadData {
    bool baro_valid = false;
    float baro_reference_m = 0.0F;
    bool gps_valid = false;
    double gps_origin_lat_deg = 0.0;
    double gps_origin_lon_deg = 0.0;
    float gps_origin_alt_m = 0.0F;
    bool gravity_valid = false;
    float gravity_mps2 = 0.0F;
    bool motion_valid = false;
    float agl_m = 0.0F;
    float vertical_velocity_mps = 0.0F;
};

constexpr std::uint32_t kExtensionMagic = 0x57465852U;   // "WFXR"
constexpr std::size_t kExtensionWords = 12;

struct Extension {
    std::uint32_t w[kExtensionWords];
};

inline std::uint32_t float_bits(const float f)
{
    std::uint32_t u;
    std::memcpy(&u, &f, sizeof(u));
    return u;
}
inline float bits_float(const std::uint32_t u)
{
    float f;
    std::memcpy(&f, &u, sizeof(f));
    return f;
}
inline void double_words(const double d, std::uint32_t &lo, std::uint32_t &hi)
{
    std::uint64_t u;
    std::memcpy(&u, &d, sizeof(u));
    lo = static_cast<std::uint32_t>(u & 0xFFFFFFFFU);
    hi = static_cast<std::uint32_t>(u >> 32);
}
inline double words_double(const std::uint32_t lo, const std::uint32_t hi)
{
    const std::uint64_t u = (static_cast<std::uint64_t>(hi) << 32) | lo;
    double d;
    std::memcpy(&d, &u, sizeof(d));
    return d;
}

inline std::uint32_t extension_checksum(const Extension &e)
{
    std::uint32_t sum = 0x3C5AA5C3U;
    for (std::size_t i = 0; i + 1 < kExtensionWords; ++i) {
        sum = (sum ^ e.w[i]) * 16777619U;
    }
    return sum;
}

inline void seal_extension(Extension &e, const PadData &d)
{
    e.w[0] = 0;   // an interrupted write is invalid, not half-valid
    e.w[1] = (d.baro_valid ? 1U : 0U) | (d.gps_valid ? 2U : 0U) |
             (d.gravity_valid ? 4U : 0U) | (d.motion_valid ? 8U : 0U);
    e.w[2] = float_bits(d.baro_reference_m);
    double_words(d.gps_origin_lat_deg, e.w[3], e.w[4]);
    double_words(d.gps_origin_lon_deg, e.w[5], e.w[6]);
    e.w[7] = float_bits(d.gps_origin_alt_m);
    e.w[8] = float_bits(d.gravity_mps2);
    e.w[9] = float_bits(d.agl_m);
    e.w[10] = float_bits(d.vertical_velocity_mps);
    e.w[0] = kExtensionMagic;
    e.w[11] = extension_checksum(e);
}

inline void invalidate_extension(Extension &e)
{
    for (std::uint32_t &x : e.w) x = 0;
}

inline bool extension_valid(const Extension &e)
{
    return e.w[0] == kExtensionMagic && e.w[11] == extension_checksum(e);
}

inline PadData unpack_extension(const Extension &e)
{
    PadData d;
    d.baro_valid = (e.w[1] & 1U) != 0U;
    d.gps_valid = (e.w[1] & 2U) != 0U;
    d.gravity_valid = (e.w[1] & 4U) != 0U;
    d.motion_valid = (e.w[1] & 8U) != 0U;
    d.baro_reference_m = bits_float(e.w[2]);
    d.gps_origin_lat_deg = words_double(e.w[3], e.w[4]);
    d.gps_origin_lon_deg = words_double(e.w[5], e.w[6]);
    d.gps_origin_alt_m = bits_float(e.w[7]);
    d.gravity_mps2 = bits_float(e.w[8]);
    d.agl_m = bits_float(e.w[9]);
    d.vertical_velocity_mps = bits_float(e.w[10]);
    return d;
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
// FlightStateMachine::PersistFn: record a state change. Thread-safe (called
// from the sensor task and the USB task): a short critical section, and a
// torn record would in any case fail its checksum.
void save(std::uint8_t state, float maximum_altitude_m, bool maximum_valid);
// Low-rate (1-5 Hz, sensor task) update of the pad references and last
// AGL / vertical velocity. RTC RAM only: no flash, no blocking.
void save_references(const PadData &data);
// After take_boot_snapshot() returned true: the references persisted before
// the reset (false if there are none or they fail validation). A copy taken at
// boot, so it is not changed by save_references().
bool resume_references(PadData &out);

}  // namespace flight_state_store
