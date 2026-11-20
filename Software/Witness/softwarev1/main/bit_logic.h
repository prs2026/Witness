#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "bit_limits.h"
#include "bit_types.h"

// BIT decision logic: check evaluators, debounce, verdict. Pure (no ESP-IDF,
// no FreeRTOS); host test tests/bit_logic_test.cpp. The runtime (bit_monitor)
// fills an Inputs snapshot from read-only probes and calls Engine::update().
// This code never reads or writes anything the flight functions own.
namespace bit {

// ---- Inputs ----------------------------------------------------------------

enum class ResetClass : std::uint8_t {
    Unknown = 0, PowerOn, External, Software, Panic, TaskWdt, IntWdt, Brownout, Other,
};
constexpr bool reset_is_fault(const ResetClass c)
{
    return c == ResetClass::Panic || c == ResetClass::TaskWdt || c == ResetClass::IntWdt ||
           c == ResetClass::Brownout;
}

// Result of an active test run by the owner task (commanded runs only).
struct ActiveResult {
    enum class State : std::uint8_t { NotRun, Pending, Done };
    State state = State::NotRun;
    bool pass = false;
    bool error = false;      // the test itself could not run (BIT internal)
    std::uint16_t value = 0;
};

enum TaskBit : std::uint16_t {
    kTaskSensors = 1U << 0,
    kTaskFlashLogger = 1U << 1,
    kTaskFlashCapture = 1U << 2,
    kTaskUart1 = 1U << 3,
    kTaskUsb = 1U << 4,
    kTaskHeartbeat = 1U << 5,
    kTaskForwarder = 1U << 6,
    kTaskCanLink = 1U << 7,
};
constexpr std::uint16_t kTasksRequired = kTaskSensors | kTaskFlashLogger | kTaskFlashCapture;
constexpr std::uint16_t kTasksAll = kTasksRequired | kTaskUart1 | kTaskUsb | kTaskHeartbeat |
                                    kTaskForwarder | kTaskCanLink;
// Tasks that must be subscribed to the task watchdog (check 1.2). flash_capture
// is deliberately not: it blocks on the full record queue while the logger
// is unsubscribed for a long NAND operation, and waits 10 s per pass in Landed
// low power, so a subscription would reboot the MCU on a slow NAND.
constexpr std::uint16_t kTasksWatchdogRequired = kTaskSensors | kTaskFlashLogger;

// Window statistics gathered from the sample stream while the board is still.
struct StillStats {
    bool valid = false;            // a full window was collected
    float accel_mag_g = 0.0F;      // mean low-g magnitude
    float accel_sd_mg = 0.0F;      // worst per-axis standard deviation
    float axial_g = 0.0F;          // mean body +X
    float gyro_mean_dps = 0.0F;    // mean |omega|
    float gyro_sd_dps = 0.0F;      // worst per-axis standard deviation
    bool high_g_valid = false;
    float high_g_mag_g = 0.0F;
};

struct BaroStats {
    bool valid = false;            // a full 2 s window was collected
    float sd_pa = 0.0F;
    float rate_mps = 0.0F;
    bool frozen = false;           // identical for kBaroFrozenSamples samples
    float sample_rate_hz = 0.0F;
};

struct Inputs {
    std::uint32_t uptime_ms = 0;
    std::uint8_t flight_state = kPadIdle;
    bool resumed = false;
    bool hitl_active = false;
    std::uint8_t status0 = 0;       // Witness status byte 0 (comms.h)
    std::uint8_t status1 = 0;

    // Group 0
    ResetClass reset = ResetClass::Unknown;
    bool image_known = false;
    bool image_flight = false;
    bool image_pending_verify = false;
    bool image_slot_matches = false;
    bool image_other_valid = false;
    std::uint32_t heap_free = 0, heap_min = 0, heap_largest = 0;
    bool heap_known = false;
    std::int32_t stack_sensors = -1, stack_logger = -1, stack_capture = -1, stack_other_min = -1;
    ActiveResult image_integrity{};
    ActiveResult heap_structure{};
    // Group 1
    std::uint16_t tasks_present = kTasksAll;
    bool wdt_known = false;
    std::uint16_t wdt_subscribed = kTasksRequired;
    bool nvs_ok = true;
    bool calibration_valid = true;
    std::uint16_t cpu_mhz = limits::kExpectedCpuMhz;
    bool automatic_transitions = true;
    bool flight_image = true;
    bool restart_pending = false;
    // Group 2
    bool imu_ready = true;           // supervisor: not down
    bool imu_fault_recent = false;   // a fault in the last 10 s
    std::uint32_t imu_fault_events = 0;
    std::uint32_t imu_reinit_failed = 0;
    bool imu_rate_valid = true;
    float imu_rate_hz = static_cast<float>(limits::kImuNominalHz);
    bool imu_loss_growth = false;    // overrun or bad words grew in the window
    StillStats still{};
    bool temps_valid = true;
    float imu_temp_c = 25.0F, baro_temp_c = 25.0F;
    ActiveResult imu_selftest{};
    // Group 3
    bool baro_ready = true;
    bool baro_sample_valid = true;
    std::int32_t pressure_pa = 101325;
    BaroStats baro{};
    bool gps_alt_diff_valid = false;
    float baro_gps_alt_diff_m = 0.0F;
    // Group 4
    bool attitude_valid = true;
    float tilt_deg = 0.0F;
    bool filter_finite = true;
    bool filter_pad_hold = true;
    float filter_agl_m = 0.0F;
    float filter_vz_mps = 0.0F;
    bool filter_gps_origin_ok = true;   // true when no fix is expected yet
    bool filter_diverged_recent = false;
    bool rule2_ready = true;     // filtered axial accel valid (low-g chain)
    bool rule1_ready = true;     // Kalman AGL and vz valid
    bool rule3_ready = true;     // barometer rate valid
    bool launch_budget_zero = true;
    bool state_record_matches = true;
    bool pad_heading_set = true;
    bool calibration_loaded = true;
    // Group 5
    bool nand_chip_ok = true;        // FLASH_INIT_FAILED clear
    std::uint32_t nand_bad_blocks = 0;
    bool nand_bad_blocks_known = false;
    bool writer_failing = false;
    bool writer_backing_off = false;
    std::uint32_t writer_failed_blocks = 0;
    ActiveResult nand_write{};
    // Group 6
    bool session_ready = true;
    bool slots_full = false;
    bool free_space_known = true;
    std::uint32_t imu_seconds_left = 540;
    std::uint32_t binary_seconds_left = 1000;
    bool queue_over_half = false;
    bool drop_growth = false;        // dropped records / unreadable pages grew
    std::uint32_t logger_pass_age_ms = 0;
    bool prelaunch_known = false;
    std::uint32_t prelaunch_fill_percent = 100;
    std::uint32_t pad_idle_ms = 100'000;   // time spent in Pad Idle (ring needs 2.5 s)
    ActiveResult log_end_to_end{};
    ActiveResult log_post_flight{};
    // Group 7
    enum class Gps : std::uint8_t { Disabled, Configuring, Ready, Failed, Streaming, Recovering };
    Gps gps_status = Gps::Streaming;
    std::uint32_t gps_rate_hz = limits::kGpsRateNominalHz;
    bool gps_fix = true;
    std::uint8_t gps_satellites = 8;
    bool utc_valid = true;
    // Group 8
    bool radio_ready = true;
    bool radio_faulted = false;
    bool radio_front_end_on = true;
    int radio_power_dbm = 0;
    bool radio_tx_failures_grew = false;
    bool radio_dio1_stuck = false;
    ActiveResult radio_device_errors{};
    // Group 11
    bool battery_valid = true;
    std::uint32_t battery_mv = 3900;
    bool battery_adc_calibrated = true;
    bool brownout_detector = true;
    bool rail_3v3_valid = false;
    std::uint32_t rail_3v3_mv = 3300;
    std::uint32_t supply_current_ma = 0;
};

// ---- Evaluation ------------------------------------------------------------

struct Eval {
    Result result = Result::NotYet;
    Class cls = Class::Info;
    std::uint16_t value = 0;
    bool bench = false;     // a bench condition (HITL, bench radio power)
};

namespace detail {
inline Eval pass(const std::uint16_t value = 0) { return {Result::Pass, Class::Info, value, false}; }
inline Eval fail(const Class c, const std::uint16_t value = 0, const bool bench = false)
{
    return {Result::Fail, c, value, bench};
}
inline Eval skipped() { return {Result::Skipped, Class::Info, 0, false}; }
inline Eval not_yet() { return {Result::NotYet, Class::Info, 0, false}; }
inline Eval na() { return {Result::NotApplicable, Class::Info, 0, false}; }
inline std::uint16_t clamp16(const double v)
{
    if (!(v > 0.0)) return 0;
    if (v > 65535.0) return 65535;
    return static_cast<std::uint16_t>(v);
}
inline std::uint16_t pack_dbm(const int dbm) { return static_cast<std::uint16_t>(static_cast<std::int16_t>(dbm)); }
inline Eval from_active(const ActiveResult &a, const Class fail_class)
{
    if (a.state == ActiveResult::State::NotRun) return skipped();
    if (a.state == ActiveResult::State::Pending) return not_yet();
    if (a.error) return {Result::Error, fail_class, a.value, false};
    return a.pass ? pass(a.value) : fail(fail_class, a.value);
}
}  // namespace detail

// Unit of the "value" field of each check, for the detail packet and Horizon:
//  0.1 reset class; 0.2 flags (1 flight, 2 pending, 4 slot, 8 other valid);
//  0.3 heap minimum bytes / 16; 0.4 smallest stack margin bytes; 1.1 tasks missing mask;
//  1.2 tasks unsubscribed mask; 2.1 faults; 2.2 rate Hz x 4; 2.3 |a| mg; 2.4 gyro sd cdps
//  ... (the Horizon table bit_model.py lists them all).
inline Eval eval_check(const std::uint8_t id, const Inputs &in)
{
    using namespace detail;
    using R = Result;
    const bool pad = in.flight_state == kPadIdle;
    const bool ground = in.flight_state == kPadIdle || in.flight_state == kLanded;
    const bool flying = !ground;

    switch (id) {
    // ---- Group 0 ----
    case make_id(0, 1): {
        if (in.reset == ResetClass::Unknown) return not_yet();
        const std::uint16_t v = static_cast<std::uint16_t>(in.reset);
        if (in.resumed) return {R::Pass, Class::Info, v, false};   // reported only
        return reset_is_fault(in.reset) ? fail(Class::Degraded, v) : pass(v);
    }
    case make_id(0, 2): {
        if (!in.image_known) return not_yet();
        const std::uint16_t v = static_cast<std::uint16_t>((in.image_flight ? 1U : 0U) |
                                                           (in.image_pending_verify ? 2U : 0U) |
                                                           (in.image_slot_matches ? 4U : 0U) |
                                                           (in.image_other_valid ? 8U : 0U));
        const bool pending_late = in.image_pending_verify && in.uptime_ms > limits::kPendingVerifyGraceMs;
        if (!in.image_flight || pending_late || !in.image_slot_matches || !in.image_other_valid)
            return fail(Class::Degraded, v);
        return pass(v);
    }
    case make_id(0, 3): {
        if (!in.heap_known) return not_yet();
        const std::uint16_t v = clamp16(in.heap_min / 16.0);
        if (in.heap_min < limits::kHeapMinBlockingBytes) return fail(Class::Blocking, v);
        if (in.heap_min < limits::kHeapMinDegradedBytes || in.heap_largest < limits::kHeapLargestDegradedBytes)
            return fail(Class::Degraded, v);
        return pass(v);
    }
    case make_id(0, 4): {
        if (in.stack_sensors < 0 || in.stack_logger < 0 || in.stack_capture < 0) return not_yet();
        std::int32_t smallest = in.stack_sensors;
        if (in.stack_logger < smallest) smallest = in.stack_logger;
        if (in.stack_capture < smallest) smallest = in.stack_capture;
        const std::uint16_t v = clamp16(static_cast<double>(smallest));
        if (smallest < static_cast<std::int32_t>(limits::kStackBlockingBytes)) return fail(Class::Blocking, v);
        if (in.stack_other_min >= 0 && in.stack_other_min < static_cast<std::int32_t>(limits::kStackDegradedBytes))
            return fail(Class::Degraded, clamp16(static_cast<double>(in.stack_other_min)));
        if (smallest < static_cast<std::int32_t>(limits::kStackDegradedBytes)) return fail(Class::Degraded, v);
        return pass(v);
    }
    case make_id(0, 5):
        return skipped();   // MCU temperature driver not wired; limit not transcribed
    case make_id(0, 6):
        return from_active(in.image_integrity, Class::Info);
    case make_id(0, 7):
        return from_active(in.heap_structure, Class::Degraded);

    // ---- Group 1 ----
    case make_id(1, 1): {
        const std::uint16_t missing = static_cast<std::uint16_t>(~in.tasks_present & kTasksAll);
        if (missing & kTasksRequired) return fail(Class::Blocking, missing);
        if (missing) return fail(Class::Degraded, missing);
        return pass(0);
    }
    case make_id(1, 2): {
        if (!in.wdt_known) return not_yet();
        const std::uint16_t missing = static_cast<std::uint16_t>(~in.wdt_subscribed & kTasksWatchdogRequired);
        return missing ? fail(Class::Degraded, missing) : pass(0);
    }
    case make_id(1, 3): {
        if (!in.nvs_ok) return fail(Class::Degraded, 1);
        if (!in.calibration_valid) return fail(Class::Degraded, 2);
        return pass(0);
    }
    case make_id(1, 4):
        // HITL replaces the sensors, so launch detection would run on
        // simulated data. A bench condition, shown as such.
        return in.hitl_active ? fail(Class::Blocking, 1, true) : pass(0);
    case make_id(1, 5): {
        const std::uint16_t v = static_cast<std::uint16_t>((in.cpu_mhz == limits::kExpectedCpuMhz ? 0U : 1U) |
                                                           (in.automatic_transitions ? 0U : 2U) |
                                                           (in.flight_image ? 0U : 4U));
        if (!in.automatic_transitions || !in.flight_image) return fail(Class::Blocking, v);
        if (in.cpu_mhz != limits::kExpectedCpuMhz) return fail(Class::Degraded, v);
        return pass(0);
    }
    case make_id(1, 6):
        return in.restart_pending ? fail(Class::Degraded, 1) : pass(0);

    // ---- Group 2 ----
    case make_id(2, 1): {
        const bool low_g = (in.status0 & 0x02U) != 0U, gyro = (in.status0 & 0x04U) != 0U;
        const std::uint16_t v = static_cast<std::uint16_t>((low_g ? 0U : 1U) | (gyro ? 0U : 2U) |
                                                           (in.imu_ready ? 0U : 4U));
        if (!in.imu_ready || !low_g || !gyro || in.imu_reinit_failed != 0U) return fail(Class::Blocking, v);
        if (in.imu_fault_recent || in.imu_fault_events != 0U) return fail(Class::Degraded, v);
        return pass(0);
    }
    case make_id(2, 2): {
        if (!in.imu_rate_valid) return not_yet();
        const float nominal = static_cast<float>(limits::kImuNominalHz);
        const std::uint16_t v = clamp16(static_cast<double>(in.imu_rate_hz) * 4.0);
        if (in.imu_rate_hz < nominal * limits::kImuRateBlockingFraction) return fail(Class::Blocking, v);
        if (in.imu_rate_hz < nominal * (1.0F - limits::kImuRateTolerance) ||
            in.imu_rate_hz > nominal * (1.0F + limits::kImuRateTolerance) || in.imu_loss_growth)
            return fail(Class::Degraded, v);
        return pass(v);
    }
    case make_id(2, 3): {
        if (flying) return na();
        if (!in.still.valid) return not_yet();
        const std::uint16_t v = clamp16(static_cast<double>(in.still.accel_mag_g) * 1000.0);
        const float g = in.still.accel_mag_g;
        if (!(g >= limits::kGravityBlockingMinG && g <= limits::kGravityBlockingMaxG)) return fail(Class::Blocking, v);
        if (!(g >= limits::kGravityDegradedMinG && g <= limits::kGravityDegradedMaxG) ||
            in.still.accel_sd_mg > limits::kAccelNoiseDegradedMg)
            return fail(Class::Degraded, v);
        return pass(v);
    }
    case make_id(2, 4): {
        if (flying) return na();
        if (!in.still.valid) return not_yet();
        const std::uint16_t v = clamp16(static_cast<double>(in.still.gyro_sd_dps) * 100.0);
        if (in.still.gyro_mean_dps > limits::kGyroMeanDegradedDps || in.still.gyro_sd_dps > limits::kGyroSdDegradedDps)
            return fail(Class::Degraded, v);
        return pass(v);
    }
    case make_id(2, 5): {
        if (flying) return na();
        const bool ready = (in.status0 & 0x20U) != 0U;
        if (!ready) return fail(Class::Degraded, 1);
        if (!in.still.valid) return not_yet();
        if (!in.still.high_g_valid) return fail(Class::Degraded, 2);
        const float diff = std::fabs(in.still.high_g_mag_g - in.still.accel_mag_g);
        const std::uint16_t v = clamp16(static_cast<double>(diff) * 1000.0);
        return diff > limits::kHighGMaxDifferenceG ? fail(Class::Degraded, v) : pass(v);
    }
    case make_id(2, 6): {
        if (flying) return na();
        if (!in.temps_valid) return not_yet();
        const float d = std::fabs(in.imu_temp_c - in.baro_temp_c);
        const std::uint16_t v = clamp16(static_cast<double>(in.imu_temp_c) * 10.0);
        const bool out = in.imu_temp_c < limits::kImuTempMinC || in.imu_temp_c > limits::kImuTempMaxC;
        return (out || d > limits::kImuBaroTempMaxDiffC) ? fail(Class::Info, v) : pass(v);
    }
    case make_id(2, 7):
        return from_active(in.imu_selftest, Class::Degraded);
    case make_id(2, 8): {
        if (!pad) return na();
        if (!in.still.valid) return not_yet();
        const float x = in.still.axial_g;
        const std::uint16_t v = clamp16((static_cast<double>(x) + 2.0) * 1000.0);
        const float frac = in.still.accel_mag_g > 0.01F ? x / in.still.accel_mag_g : 0.0F;
        if (x < limits::kPadAxialMinG || x > limits::kPadAxialMaxG || frac < limits::kPadAxialFractionMin)
            return fail(Class::Degraded, v);
        return pass(v);
    }

    // ---- Group 3 ----
    case make_id(3, 1): {
        const bool ready = (in.status0 & 0x40U) != 0U;
        if (ready && in.baro_ready) return pass(0);
        if (in.uptime_ms < limits::kBaroReadyWithinMs) return not_yet();
        return fail(Class::Blocking, 1);
    }
    case make_id(3, 2): {
        if (!in.baro_sample_valid) return not_yet();
        const std::uint16_t v = clamp16(in.pressure_pa / 2.0);
        const bool p_ok = in.pressure_pa >= limits::kBaroPressureMinPa && in.pressure_pa <= limits::kBaroPressureMaxPa;
        const bool t_ok = in.baro_temp_c >= limits::kBaroTempMinC && in.baro_temp_c <= limits::kBaroTempMaxC;
        return (p_ok && t_ok) ? pass(v) : fail(Class::Blocking, v);
    }
    case make_id(3, 3): {
        if (flying) return na();
        if (!in.baro.valid) return not_yet();
        const std::uint16_t v = clamp16(static_cast<double>(in.baro.sd_pa) * 10.0);
        if (in.baro.frozen || in.baro.sd_pa > limits::kBaroNoiseMaxPa ||
            std::fabs(in.baro.rate_mps) > limits::kBaroRateMaxMps)
            return fail(Class::Degraded, v);
        return pass(v);
    }
    case make_id(3, 4): {
        if (!in.baro.valid) return not_yet();
        const float nominal = static_cast<float>(limits::kBaroNominalHz);
        const std::uint16_t v = clamp16(static_cast<double>(in.baro.sample_rate_hz) * 10.0);
        return (in.baro.sample_rate_hz < nominal * (1.0F - limits::kBaroRateTolerance) ||
                in.baro.sample_rate_hz > nominal * (1.0F + limits::kBaroRateTolerance))
                   ? fail(Class::Degraded, v)
                   : pass(v);
    }
    case make_id(3, 5): {
        if (flying || !in.gps_alt_diff_valid) return skipped();
        const std::uint16_t v = clamp16(std::fabs(static_cast<double>(in.baro_gps_alt_diff_m)));
        return std::fabs(in.baro_gps_alt_diff_m) > 200.0F ? fail(Class::Info, v) : pass(v);
    }

    // ---- Group 4 ----
    case make_id(4, 1): {
        if (!pad) return na();
        if (in.attitude_valid) return pass(0);
        return in.uptime_ms < limits::kAttitudeValidByMs ? not_yet() : fail(Class::Degraded, 1);
    }
    case make_id(4, 2): {
        if (!pad) return na();
        if (!in.filter_finite || in.filter_diverged_recent) return fail(Class::Degraded, 1);
        if (!in.filter_pad_hold || std::fabs(in.filter_agl_m) > limits::kPadAglMaxM ||
            std::fabs(in.filter_vz_mps) > limits::kPadVzMaxMps || !in.filter_gps_origin_ok)
            return fail(Class::Degraded, 2);
        return pass(0);
    }
    case make_id(4, 3): {
        // Launch-detection readiness: rule 2 (accel) plus at least one of the
        // independent paths rule 1 (Kalman) or rule 3 (barometer). Only one path
        // alive is NO-GO (owner Q5). In flight it is informational.
        const std::uint16_t v = static_cast<std::uint16_t>((in.rule1_ready ? 1U : 0U) | (in.rule2_ready ? 2U : 0U) |
                                                           (in.rule3_ready ? 4U : 0U));
        const bool ok = in.rule2_ready && (in.rule1_ready || in.rule3_ready);
        if (ok) return pass(v);
        if (in.uptime_ms < limits::kCoreDeadlineMs / 2U) return not_yet();
        return fail(flying ? Class::Info : Class::Blocking, v);
    }
    case make_id(4, 4): {
        if (!pad) return na();
        return in.launch_budget_zero ? pass(0) : fail(Class::Info, 1);
    }
    case make_id(4, 5):
        return in.state_record_matches ? pass(0) : fail(Class::Info, 1);
    case make_id(4, 6):
        if (!pad) return na();
        return in.calibration_loaded ? pass(0) : fail(Class::Info, 1);
    case make_id(4, 7):
        // Owner Q12: a warning when the pad heading has not been set.
        if (!pad) return na();
        return in.pad_heading_set ? pass(0) : fail(Class::Degraded, 1);

    // ---- Group 5 ----
    case make_id(5, 1):
        return in.nand_chip_ok ? pass(0) : fail(Class::Blocking, 1);
    case make_id(5, 2):
        return from_active(in.nand_write, Class::Blocking);
    case make_id(5, 3): {
        if (!in.nand_bad_blocks_known) return skipped();
        const std::uint16_t v = clamp16(static_cast<double>(in.nand_bad_blocks));
        return in.nand_bad_blocks > limits::kNandBadBlocksDegraded ? fail(Class::Degraded, v) : pass(v);
    }
    case make_id(5, 4): {
        const std::uint16_t v = clamp16(static_cast<double>(in.writer_failed_blocks));
        if (in.writer_backing_off || in.writer_failing) return fail(Class::Blocking, v);
        if (in.writer_failed_blocks != 0U) return fail(Class::Degraded, v);
        return pass(0);
    }

    // ---- Group 6 ----
    case make_id(6, 1): {
        if (in.slots_full) return fail(Class::Blocking, 2);
        if (!in.session_ready) return in.uptime_ms < limits::kCoreDeadlineMs ? not_yet() : fail(Class::Blocking, 1);
        return pass(0);
    }
    case make_id(6, 2): {
        if (!in.free_space_known) return not_yet();
        if (in.resumed) return na();
        const std::uint16_t v = clamp16(static_cast<double>(in.imu_seconds_left));
        if (in.imu_seconds_left < limits::kImuFreeSecondsBlocking ||
            in.binary_seconds_left < limits::kBinaryFreeSecondsBlocking)
            return fail(Class::Blocking, v);
        if (in.imu_seconds_left < limits::kImuFreeSecondsDegraded) return fail(Class::Degraded, v);
        return pass(v);
    }
    case make_id(6, 3):
        return from_active(in.log_end_to_end, Class::Blocking);
    case make_id(6, 4): {
        if (in.drop_growth) return fail((in.status0 & 0x80U) ? Class::Blocking : Class::Degraded, 1);
        if (in.queue_over_half) return fail(Class::Degraded, 2);
        return pass(0);
    }
    case make_id(6, 5): {
        const std::uint16_t v = clamp16(static_cast<double>(in.logger_pass_age_ms) / 10.0);
        return in.logger_pass_age_ms > limits::kLoggerHeartbeatMaxMs ? fail(Class::Blocking, v) : pass(v);
    }
    case make_id(6, 6): {
        if (!pad) return na();
        if (!in.prelaunch_known || in.pad_idle_ms < limits::kPrelaunchRingAfterMs) return not_yet();
        const std::uint16_t v = clamp16(static_cast<double>(in.prelaunch_fill_percent));
        return in.prelaunch_fill_percent < limits::kPrelaunchRingMinPercent ? fail(Class::Degraded, v) : pass(v);
    }
    case make_id(6, 7):
        return from_active(in.log_post_flight, Class::Info);

    // ---- Group 7 (never blocking) ----
    case make_id(7, 1): {
        using G = Inputs::Gps;
        const std::uint16_t v = static_cast<std::uint16_t>(in.gps_status);
        if (in.gps_status == G::Ready) return pass(v);
        if (in.gps_status == G::Streaming) return pass(v);
        if (in.uptime_ms < limits::kGpsReadyWithinMs && (in.gps_status == G::Configuring || in.gps_status == G::Recovering))
            return not_yet();
        return fail(Class::Degraded, v);
    }
    case make_id(7, 2): {
        const std::uint16_t v = clamp16(static_cast<double>(in.gps_rate_hz));
        if (in.uptime_ms < limits::kGpsReadyWithinMs && in.gps_rate_hz == 0U) return not_yet();
        if (in.gps_rate_hz == 0U) return fail(Class::Degraded, v);
        const std::uint32_t lo = limits::kGpsRateNominalHz - limits::kGpsRateTolerance;
        const std::uint32_t hi = limits::kGpsRateNominalHz + limits::kGpsRateTolerance;
        return (in.gps_rate_hz < lo || in.gps_rate_hz > hi) ? fail(Class::Degraded, v) : pass(v);
    }
    case make_id(7, 4): {
        const std::uint16_t v = in.gps_satellites;
        if (in.gps_fix && in.gps_satellites >= limits::kGpsMinSatellites) return pass(v);
        return in.uptime_ms < limits::kGpsReadyWithinMs ? not_yet() : fail(Class::Degraded, v);
    }
    case make_id(7, 5):
        return in.utc_valid ? pass(0) : (in.uptime_ms < limits::kGpsReadyWithinMs ? not_yet() : fail(Class::Info, 1));

    // ---- Group 8: radio ----
    case make_id(8, 1): {
        const std::uint16_t v = static_cast<std::uint16_t>((in.radio_ready ? 0U : 1U) | (in.radio_faulted ? 2U : 0U));
        if (in.radio_ready && !in.radio_faulted) return pass(0);
        return in.uptime_ms < limits::kRadioSettleMs && !in.radio_faulted ? not_yet() : fail(Class::Blocking, v);
    }
    case make_id(8, 2):
        return from_active(in.radio_device_errors, Class::Degraded);
    case make_id(8, 3): {
        // Owner Q4: configured power must be at least -3 dBm; the front end
        // must be on. Below it the result is NO-GO; when the power is the only
        // problem it is a bench condition (the bench runs at -17 dBm).
        const std::uint16_t v = pack_dbm(in.radio_power_dbm);
        if (!in.radio_front_end_on) return fail(Class::Blocking, v);
        if (in.radio_power_dbm < limits::kRadioMinFlightPowerDbm) return fail(Class::Blocking, v, true);
        return pass(v);
    }
    case make_id(8, 5):
        return in.radio_tx_failures_grew ? fail(Class::Degraded, 1) : pass(0);
    case make_id(8, 6):
        return in.radio_dio1_stuck ? fail(Class::Degraded, 1) : pass(0);

    // ---- Group 11 ----
    case make_id(11, 1): {
        if (!in.battery_valid) return not_yet();
        const std::uint16_t v = clamp16(static_cast<double>(in.battery_mv));
        if (in.battery_mv < limits::kBatteryBlockingBelowMv) return fail(Class::Blocking, v);
        if (in.battery_mv < limits::kBatteryDegradedBelowMv || in.battery_mv > limits::kBatteryDegradedAboveMv)
            return fail(Class::Degraded, v);
        return pass(v);
    }
    case make_id(11, 2):
        return in.battery_adc_calibrated ? pass(0) : fail(Class::Degraded, 1);
    case make_id(11, 3): {
        // 3V3 rail and supply current: next board revision (owner Q11).
        if (!limits::kRail3v3MonitorPresent || !in.rail_3v3_valid) return skipped();
        const std::uint16_t v = clamp16(static_cast<double>(in.rail_3v3_mv));
        if (in.rail_3v3_mv < limits::kRail3v3MinMv || in.rail_3v3_mv > limits::kRail3v3MaxMv)
            return fail(Class::Degraded, v);
        if (limits::kSupplyCurrentMonitorPresent && in.supply_current_ma > limits::kSupplyCurrentMaxMa)
            return fail(Class::Degraded, clamp16(static_cast<double>(in.supply_current_ma)));
        return pass(v);
    }
    case make_id(11, 4):
        if (!pad) return na();
        return in.brownout_detector ? pass(0) : fail(Class::Info, 1);

    // ---- Group 12 ----
    case make_id(12, 1): {
        if (!in.imu_rate_valid) return not_yet();
        const float ratio = in.imu_rate_hz / static_cast<float>(limits::kImuNominalHz);
        const std::uint16_t v = clamp16(static_cast<double>(ratio) * 1000.0);
        return std::fabs(ratio - 1.0F) > limits::kClockToleranceFraction ? fail(Class::Degraded, v) : pass(v);
    }
    case make_id(12, 4):
        return in.utc_valid ? pass(0) : (in.uptime_ms < limits::kGpsReadyWithinMs ? not_yet() : fail(Class::Info, 1));
    default:
        break;
    }
    return {Result::Error, Class::Degraded, 0, false};
}

// ---- Safety gate for active tests (plan 3.3) --------------------------------

enum class GateReason : std::uint8_t {
    Open = 0, NotOnGround, Resumed, HitlState, NotStill, Tripped, Busy, LandedOnly, PadOnly,
};

struct GateInputs {
    std::uint8_t flight_state = kPadIdle;
    bool resumed = false;
    bool launch_budget_zero = true;
    bool still_valid = false;       // a full still window exists
    float axial_g = 1.0F;
    float accel_mag_g = 1.0F;
    float gyro_mean_dps = 0.0F;
    float baro_rate_mps = 0.0F;
    float kalman_vz_mps = 0.0F;
    bool maintenance_or_erase_pending = false;
    bool run_in_progress = false;
};

// Whether an active test may start. `needs_pad` tests (IMU self-test) only run
// in Pad Idle; the others in Pad Idle or Landed.
inline GateReason active_test_gate(const GateInputs &g, const bool needs_pad)
{
    if (g.run_in_progress) return GateReason::Busy;
    if (g.flight_state != kPadIdle && g.flight_state != kLanded) return GateReason::NotOnGround;
    if (needs_pad && g.flight_state != kPadIdle) return GateReason::PadOnly;
    if (g.resumed) return GateReason::Resumed;
    if (g.maintenance_or_erase_pending) return GateReason::Busy;
    if (!g.launch_budget_zero) return GateReason::NotStill;
    if (!g.still_valid) return GateReason::NotStill;
    if (g.accel_mag_g < limits::kStillAccelMinG || g.accel_mag_g > limits::kStillAccelMaxG) return GateReason::NotStill;
    if (g.gyro_mean_dps > limits::kStillGyroMaxDps) return GateReason::NotStill;
    if (std::fabs(g.baro_rate_mps) > limits::kStillBaroRateMaxMps) return GateReason::NotStill;
    if (std::fabs(g.kalman_vz_mps) > limits::kStillKalmanVzMaxMps) return GateReason::NotStill;
    return GateReason::Open;
}

// Tripwire, checked at every step of an active test: any of these aborts it.
inline bool active_test_tripped(const std::uint8_t flight_state, const float accel_mag_g, const bool budget_zero,
                                const std::uint8_t state_at_start)
{
    return flight_state != state_at_start || !budget_zero || accel_mag_g > limits::kTripAccelG;
}

// ---- Engine: debounce and verdict -------------------------------------------

struct Summary {
    Verdict verdict = Verdict::NotRun;
    bool bench = false;              // every blocking failure is a bench condition
    RunKind kind = RunKind::Startup;
    std::uint8_t flight_state = 0;
    bool core_done = false;
    bool settled = false;
    bool active_ran = false;
    bool resumed = false;
    std::uint16_t blocking_fail_mask = 0;
    std::uint16_t degraded_fail_mask = 0;
    std::uint16_t not_evaluated_mask = 0;
    std::uint8_t first_fail_id = 0;  // 0 = none; blocking first
    std::uint8_t sequence = 0;
    std::uint16_t age_s = 0;
    std::uint8_t blocking_fail_groups = 0;   // number of groups in blocking_fail_mask
};

class Engine {
public:
    struct Slot {
        Eval shown{};            // the debounced result
        bool evaluated = false;  // a real (non NotYet) result was adopted
        Eval candidate{};
        std::uint8_t count = 0;
    };

    void begin(const RunKind kind, const std::uint32_t now_ms)
    {
        kind_ = kind;
        run_start_ms_ = now_ms;
        immediate_ = kind == RunKind::Commanded;
        started_ = true;
        if (kind != RunKind::Passive) {
            // A startup or commanded run starts from scratch; a passive monitor
            // keeps the slots (and their debounce state) it has.
            for (auto &s : slots_) s = Slot{};
            ++sequence_;
            active_ran_ = false;
        }
    }

    // One evaluation pass over every check.
    void update(const Inputs &in)
    {
        if (!started_) begin(RunKind::Startup, in.uptime_ms);
        in_ = in;
        last_update_ms_ = in.uptime_ms;
        for (std::size_t i = 0; i < kCheckCount; ++i) {
            const CheckInfo &info = kChecks[i];
            Eval raw = eval_check(info.id, in);
            if (!(info.states & state_bit(in.flight_state)) && raw.result != Result::Skipped &&
                raw.result != Result::NotYet) {
                // Outside its states: not applicable (the evaluator may already say so).
                raw = detail::na();
            }
            adopt(slots_[i], raw, info.active);
        }
    }

    const Slot &slot(const std::size_t index) const { return slots_[index]; }
    Slot &slot_mut(const std::size_t index) { return slots_[index]; }
    bool started() const { return started_; }
    std::uint8_t sequence() const { return sequence_; }
    RunKind kind() const { return kind_; }
    void mark_active_ran() { active_ran_ = true; }

    Summary summarize(const std::uint32_t now_ms) const
    {
        Summary s;
        s.kind = kind_;
        s.sequence = sequence_;
        s.flight_state = in_.flight_state;
        s.resumed = in_.resumed;
        s.active_ran = active_ran_;
        if (!started_) return s;
        bool core_all = true, settled_all = true;
        bool any_blocking_fail = false, only_bench = true;
        for (std::size_t i = 0; i < kCheckCount; ++i) {
            const CheckInfo &info = kChecks[i];
            const Slot &sl = slots_[i];
            const unsigned g = id_group(info.id);
            const bool resolved = sl.evaluated || sl.shown.result == Result::Skipped ||
                                  sl.shown.result == Result::NotApplicable;
            if (!sl.evaluated && sl.shown.result == Result::NotYet && info.cls == Class::Blocking) {
                settled_all = false;
                if (info.stage == Stage::Core) core_all = false;
                s.not_evaluated_mask = static_cast<std::uint16_t>(s.not_evaluated_mask | (1U << g));
            } else if (!resolved && info.cls != Class::Blocking) {
                s.not_evaluated_mask = static_cast<std::uint16_t>(s.not_evaluated_mask | (1U << g));
            }
            if (sl.shown.result == Result::Fail || sl.shown.result == Result::Error) {
                // An ERROR (BIT internal) is never a pass: it counts as a failure of its class.
                const std::uint16_t bit = static_cast<std::uint16_t>(1U << g);
                if (sl.shown.cls == Class::Blocking) {
                    s.blocking_fail_mask = static_cast<std::uint16_t>(s.blocking_fail_mask | bit);
                    any_blocking_fail = true;
                    if (!sl.shown.bench) only_bench = false;
                } else if (sl.shown.cls == Class::Degraded) {
                    s.degraded_fail_mask = static_cast<std::uint16_t>(s.degraded_fail_mask | bit);
                }
            }
        }
        // First failing check: blocking first, then degraded, in table order.
        for (int pass = 0; pass < 2 && s.first_fail_id == 0U; ++pass) {
            for (std::size_t i = 0; i < kCheckCount; ++i) {
                const Slot &sl = slots_[i];
                if (sl.shown.result != Result::Fail && sl.shown.result != Result::Error) continue;
                const bool blocking = sl.shown.cls == Class::Blocking;
                if ((pass == 0) == blocking) {
                    s.first_fail_id = kChecks[i].id;
                    break;
                }
            }
        }
        for (unsigned g = 0; g < kGroupCount; ++g) {
            if (s.blocking_fail_mask & (1U << g)) ++s.blocking_fail_groups;
        }
        s.core_done = core_all;
        s.settled = settled_all;
        const std::uint32_t t = now_ms - run_start_ms_;
        const std::uint32_t core_deadline = kind_ == RunKind::Commanded ? limits::kCommandedRunMaxMs : limits::kCoreDeadlineMs;
        const std::uint32_t final_deadline = kind_ == RunKind::Commanded ? limits::kCommandedRunMaxMs : limits::kFinalDeadlineMs;
        const bool by_uptime = kind_ != RunKind::Commanded;
        const std::uint32_t clock = by_uptime ? in_.uptime_ms : t;
        if (any_blocking_fail) {
            s.verdict = Verdict::NoGo;
            s.bench = only_bench;
        } else if (!core_all) {
            s.verdict = clock >= core_deadline ? Verdict::NoGo : Verdict::Running;
        } else if (!settled_all) {
            s.verdict = clock >= final_deadline ? Verdict::NoGo : Verdict::Running;
        } else {
            s.verdict = s.degraded_fail_mask ? Verdict::GoWithWarnings : Verdict::Go;
        }
        const std::uint32_t age = now_ms - last_update_ms_;
        s.age_s = static_cast<std::uint16_t>(age / 1000U > 0xFFFFU ? 0xFFFFU : age / 1000U);
        return s;
    }

private:
    void adopt(Slot &sl, const Eval &raw, const bool active)
    {
        if (raw.result == Result::NotYet) {
            // Not judged yet: an unevaluated slot stays so; a judged one keeps its last result.
            if (!sl.evaluated) sl.shown = raw;
            sl.count = 0;
            return;
        }
        if (raw.result == Result::Skipped || raw.result == Result::NotApplicable) {
            sl.shown = raw; sl.evaluated = true; sl.count = 0;
            return;
        }
        const bool judged = sl.evaluated && (sl.shown.result == Result::Pass || sl.shown.result == Result::Fail ||
                                             sl.shown.result == Result::Error);
        if (active || immediate_) {
            sl.shown = raw; sl.evaluated = true; sl.count = 0;
            return;
        }
        if (!judged) {
            // First judgement: a pass is adopted at once, a failure after the debounce count.
            if (raw.result == Result::Pass) { sl.shown = raw; sl.evaluated = true; sl.count = 0; return; }
        } else if (same(sl.shown, raw)) {
            sl.shown = raw;   // refresh the value
            sl.count = 0;
            return;
        }
        if (sl.count != 0U && same(sl.candidate, raw)) {
            if (sl.count < 255U) ++sl.count;
        } else {
            sl.candidate = raw;
            sl.count = 1;
        }
        std::uint8_t need = raw.cls == Class::Blocking ? limits::kDebounceBlocking : limits::kDebounceDegraded;
        if (raw.result == Result::Pass && judged && sl.shown.result != Result::Pass) need = limits::kDebounceRecover;
        if (sl.count >= need) {
            sl.shown = raw; sl.evaluated = true; sl.count = 0;
        }
    }
    static bool same(const Eval &a, const Eval &b) { return a.result == b.result && a.cls == b.cls; }

    Slot slots_[kCheckCount]{};
    Inputs in_{};
    RunKind kind_ = RunKind::Startup;
    bool started_ = false;
    bool immediate_ = false;
    bool active_ran_ = false;
    std::uint32_t run_start_ms_ = 0;
    std::uint32_t last_update_ms_ = 0;
    std::uint8_t sequence_ = 0;
};

}  // namespace bit
