#pragma once

#include <cstddef>
#include <cstdint>

// BIT identifiers, result codes and the check table (BIT_PLAN_2026-11-18.md).
// Pure: no ESP-IDF dependency. The same IDs are used by Horizon
// (Software/Horizon/bit_model.py); keep them in step.
namespace bit {

// Check ID = group (high nibble) and check number (low nibble).
constexpr std::uint8_t make_id(const unsigned group, const unsigned check)
{
    return static_cast<std::uint8_t>((group << 4U) | (check & 0x0FU));
}
constexpr unsigned id_group(const std::uint8_t id) { return id >> 4U; }
constexpr unsigned id_check(const std::uint8_t id) { return id & 0x0FU; }

enum class Result : std::uint8_t {
    Pass = 0,
    Fail = 1,
    Skipped = 2,   // state, option or hardware absent: never a pass
    NotApplicable = 3,
    NotYet = 4,    // still settling / not evaluated
    Error = 5,     // BIT internal error: never a pass
};

enum class Class : std::uint8_t { Info = 0, Degraded = 1, Blocking = 2 };

enum class Verdict : std::uint8_t {
    NotRun = 0,
    Running = 1,
    Go = 2,
    GoWithWarnings = 3,
    NoGo = 4,
};

enum class RunKind : std::uint8_t { Startup = 0, Commanded = 1, Passive = 2 };

// Groups (bit index in the summary masks).
constexpr unsigned kGroupPlatform = 0;
constexpr unsigned kGroupSupervision = 1;
constexpr unsigned kGroupImu = 2;
constexpr unsigned kGroupBaro = 3;
constexpr unsigned kGroupAttitude = 4;
constexpr unsigned kGroupNand = 5;
constexpr unsigned kGroupLogSession = 6;
constexpr unsigned kGroupGps = 7;
constexpr unsigned kGroupRadio = 8;
constexpr unsigned kGroupCan = 9;       // deferred with Iris
constexpr unsigned kGroupIris = 10;     // deferred with Iris
constexpr unsigned kGroupPower = 11;
constexpr unsigned kGroupClocks = 12;
constexpr unsigned kGroupGround = 13;   // computed in Horizon
constexpr unsigned kGroupCamera = 15;   // future option
constexpr unsigned kGroupCount = 16;
// Groups Witness never reports in this round (the masks carry 0 for them).
constexpr std::uint16_t kReservedGroupMask =
    (1U << kGroupCan) | (1U << kGroupIris) | (1U << kGroupGround) | (1U << 14U) | (1U << kGroupCamera);

// Flight states as in FlightStateMachine::State.
constexpr std::uint8_t kPadIdle = 0, kBoost = 1, kCoast = 2, kDescent = 3, kLanded = 4;
constexpr std::uint8_t kStatePI = 1U << 0, kStateB = 1U << 1, kStateC = 1U << 2, kStateD = 1U << 3,
                       kStateL = 1U << 4;
constexpr std::uint8_t kStatesAll = 0x1F;
constexpr std::uint8_t kStatesGround = kStatePI | kStateL;

constexpr std::uint8_t state_bit(const std::uint8_t state)
{
    return state <= kLanded ? static_cast<std::uint8_t>(1U << state) : 0U;
}

// Stage in which a blocking check must be resolved (plan 3.1).
enum class Stage : std::uint8_t { Core, Settle, Commanded };

struct CheckInfo {
    std::uint8_t id;
    Class cls;           // highest class the check can report
    Stage stage;
    std::uint8_t states; // states in which it is judged (NotApplicable elsewhere)
    bool active;         // result comes from a commanded active test (no debounce)
    const char *name;    // short text for log lines
};

// The table is the single list of checks Witness evaluates. Reserved
// (deferred) groups have no entries.
constexpr CheckInfo kChecks[] = {
    // Group 0: MCU, image, memory
    {make_id(0, 1), Class::Degraded, Stage::Core, kStatesAll, false, "reset reason"},
    {make_id(0, 2), Class::Degraded, Stage::Settle, kStatesGround, false, "image identity"},
    {make_id(0, 3), Class::Blocking, Stage::Core, kStatesAll, false, "heap"},
    {make_id(0, 4), Class::Blocking, Stage::Core, kStatesAll, false, "stack margins"},
    {make_id(0, 5), Class::Degraded, Stage::Settle, kStatesGround, false, "mcu temperature"},
    {make_id(0, 6), Class::Info, Stage::Commanded, kStatesGround, true, "image integrity"},
    {make_id(0, 7), Class::Degraded, Stage::Commanded, kStatesGround, true, "heap structure"},
    // Group 1: supervision, mode, configuration
    {make_id(1, 1), Class::Blocking, Stage::Core, kStatesAll, false, "task census"},
    {make_id(1, 2), Class::Degraded, Stage::Core, kStatesAll, false, "task watchdog"},
    {make_id(1, 3), Class::Degraded, Stage::Settle, kStatesGround, false, "nvs"},
    {make_id(1, 4), Class::Blocking, Stage::Core, kStatesAll, false, "hitl inactive"},
    {make_id(1, 5), Class::Blocking, Stage::Core, kStatesGround, false, "flight configuration"},
    {make_id(1, 6), Class::Degraded, Stage::Settle, kStatesGround, false, "stale requests"},
    // Group 2: IMU
    {make_id(2, 1), Class::Blocking, Stage::Core, kStatesAll, false, "imu alive"},
    {make_id(2, 2), Class::Blocking, Stage::Core, kStatesAll, false, "imu rate and loss"},
    {make_id(2, 3), Class::Blocking, Stage::Core, kStatesGround, false, "gravity and noise"},
    {make_id(2, 4), Class::Degraded, Stage::Settle, kStatesGround, false, "gyro bias and noise"},
    {make_id(2, 5), Class::Degraded, Stage::Settle, kStatesGround, false, "high-g consistency"},
    {make_id(2, 6), Class::Info, Stage::Settle, kStatesGround, false, "imu temperature"},
    {make_id(2, 7), Class::Degraded, Stage::Commanded, kStatePI, true, "imu self-test"},
    {make_id(2, 8), Class::Degraded, Stage::Settle, kStatePI, false, "body orientation"},
    // Group 3: barometer
    {make_id(3, 1), Class::Blocking, Stage::Core, kStatesAll, false, "baro ready"},
    {make_id(3, 2), Class::Blocking, Stage::Core, kStatesAll, false, "baro plausible"},
    {make_id(3, 3), Class::Degraded, Stage::Settle, kStatesGround, false, "baro noise"},
    {make_id(3, 4), Class::Degraded, Stage::Settle, kStatesAll, false, "baro rate"},
    {make_id(3, 5), Class::Info, Stage::Settle, kStatesGround, false, "baro vs gps"},
    // Group 4: attitude, filter, launch-detection readiness
    {make_id(4, 1), Class::Degraded, Stage::Settle, kStatePI, false, "attitude valid"},
    {make_id(4, 2), Class::Degraded, Stage::Settle, kStatePI, false, "position filter"},
    {make_id(4, 3), Class::Blocking, Stage::Core, kStatesAll, false, "launch readiness"},
    {make_id(4, 4), Class::Info, Stage::Core, kStatePI, false, "pad quiet"},
    {make_id(4, 5), Class::Info, Stage::Settle, kStatesAll, false, "state sanity"},
    {make_id(4, 6), Class::Info, Stage::Settle, kStatePI, false, "calibration"},
    {make_id(4, 7), Class::Degraded, Stage::Settle, kStatePI, false, "pad heading set"},
    // Group 5: NAND chip
    {make_id(5, 1), Class::Blocking, Stage::Core, kStatesAll, false, "nand chip"},
    {make_id(5, 2), Class::Blocking, Stage::Commanded, kStatesGround, true, "nand write test"},
    {make_id(5, 3), Class::Degraded, Stage::Settle, kStatesGround, false, "nand bad blocks"},
    {make_id(5, 4), Class::Blocking, Stage::Core, kStatesAll, false, "nand writer health"},
    // Group 6: log session and capture path
    {make_id(6, 1), Class::Blocking, Stage::Core, kStatesAll, false, "log session"},
    {make_id(6, 2), Class::Blocking, Stage::Core, kStatesGround, false, "log free space"},
    {make_id(6, 3), Class::Blocking, Stage::Commanded, kStatesGround, true, "log end-to-end"},
    {make_id(6, 4), Class::Degraded, Stage::Core, kStatesAll, false, "log queues and drops"},
    {make_id(6, 5), Class::Blocking, Stage::Core, kStatesAll, false, "logger heartbeat"},
    {make_id(6, 6), Class::Degraded, Stage::Settle, kStatePI, false, "prelaunch ring"},
    {make_id(6, 7), Class::Info, Stage::Commanded, kStateL, true, "post-flight log"},
    // Group 7: GPS (never blocking)
    {make_id(7, 1), Class::Degraded, Stage::Settle, kStatesGround, false, "gps receiver"},
    {make_id(7, 2), Class::Degraded, Stage::Settle, kStatesGround, false, "gps rate"},
    {make_id(7, 4), Class::Degraded, Stage::Settle, kStatesGround, false, "gps fix"},
    {make_id(7, 5), Class::Info, Stage::Settle, kStatesGround, false, "utc valid"},
    // Group 8: radio (blocking by owner decision Q1, Q4)
    {make_id(8, 1), Class::Blocking, Stage::Settle, kStatesAll, false, "radio ready"},
    {make_id(8, 2), Class::Degraded, Stage::Settle, kStatesGround, true, "radio device errors"},
    {make_id(8, 3), Class::Blocking, Stage::Settle, kStatesGround, false, "radio power"},
    {make_id(8, 5), Class::Degraded, Stage::Settle, kStatesGround, false, "radio tx path"},
    {make_id(8, 6), Class::Degraded, Stage::Settle, kStatesGround, false, "radio rx path"},
    // Group 11: power
    {make_id(11, 1), Class::Blocking, Stage::Core, kStatesAll, false, "battery voltage"},
    {make_id(11, 2), Class::Degraded, Stage::Settle, kStatesGround, false, "battery adc"},
    {make_id(11, 3), Class::Degraded, Stage::Settle, kStatesGround, false, "3v3 rail"},
    {make_id(11, 4), Class::Info, Stage::Settle, kStatePI, false, "brownout detector"},
    // Group 12: clocks
    {make_id(12, 1), Class::Degraded, Stage::Settle, kStatesAll, false, "imu clock"},
    {make_id(12, 4), Class::Info, Stage::Settle, kStatesGround, false, "time of day"},
};
constexpr std::size_t kCheckCount = sizeof(kChecks) / sizeof(kChecks[0]);
static_assert(kCheckCount <= 64, "the detail packet and the engine assume at most 64 checks");

constexpr int check_index(const std::uint8_t id)
{
    for (std::size_t i = 0; i < kCheckCount; ++i) {
        if (kChecks[i].id == id) return static_cast<int>(i);
    }
    return -1;
}

}  // namespace bit
