#pragma once

#include <cstdint>

// Built-in self test (BIT): every limit and constant in one place. Pure
// constants, no ESP-IDF dependency (host tests include this header).
//
// Provenance of each value is marked:
//   [OWNER]       decided by the owner (BIT_PLAN_2026-11-18.md section 11)
//   [PROPOSED]    starting value from the plan, to be tuned on the bench
//   [CODE]        taken from a constant already in the firmware
//   [PLACEHOLDER] a datasheet value that has NOT been transcribed (the
//                 datasheets are not in the repository). The number is a
//                 neutral stand-in so the logic can be tested; it must be
//                 replaced from the datasheet before the check is relied on.
namespace bit::limits {

// ---- Radio (owner decisions Q1, Q4) ---------------------------------------
// [OWNER] minimum configured radio power for flight. The radio front end
// amplifies this further. Below it the check is NO-GO (flagged as a bench
// condition: the bench runs at -17 dBm and the report never blocks anything).
constexpr int kRadioMinFlightPowerDbm = -3;
// [CODE] hardware_pins.h / command_policy.h boot default and range.
constexpr int kRadioBenchPowerDbm = -17;
// [CODE] flight_selftest.cpp waits 30 s for the radio.
constexpr std::uint32_t kRadioSettleMs = 30'000;

// ---- Timing of the startup BIT (plan 3.1) ---------------------------------
constexpr std::uint32_t kCoreDeadlineMs = 10'000;   // [PLAN] core verdict by T+10 s
constexpr std::uint32_t kFinalDeadlineMs = 60'000;  // [PLAN] final verdict by T+60 s
constexpr std::uint32_t kMonitorPeriodMs = 1000;    // [PLAN] passive monitor, 1 Hz
constexpr std::uint32_t kStaleSummaryMs = 300'000;  // Horizon STALE rule (5 min)
// After a warm reset in flight the BIT task does not start working before this
// delay: it adds nothing to the resume timeline (plan 3.2).
constexpr std::uint32_t kResumedStartDelayMs = 3000;

// ---- Debounce (plan 3.1) ---------------------------------------------------
constexpr std::uint8_t kDebounceDegraded = 3;   // evaluations to change class
constexpr std::uint8_t kDebounceBlocking = 2;
constexpr std::uint8_t kDebounceRecover = 5;    // passes needed after a fail

// ---- Group 0: platform -----------------------------------------------------
constexpr std::uint32_t kHeapMinBlockingBytes = 6 * 1024;     // [PROPOSED]
// Degraded = close to where the flight image could actually fail. It links no
// Wi-Fi; after boot its largest allocations are the post-OTA self-test task
// (4 KB stack) and the IMU calibration save task (3 KB), about 4.5 KB each with
// the TCB, so allocations start failing near 5 KB. Bench 2026-11-18 with the
// 32 KB data cache: heap min 15.2 KB, flat through a HITL flight. Owner
// decision 2026-11-18: warn only near the real failure point; 10 KB set by the
// owner 2026-11-19.
constexpr std::uint32_t kHeapMinDegradedBytes = 10 * 1024;    // [OWNER]
constexpr std::uint32_t kHeapLargestDegradedBytes = 5 * 1024; // [OWNER] those ~4.5 KB need one free block
constexpr std::uint32_t kStackBlockingBytes = 256;            // [PROPOSED] sensors/flash_logger/flash_capture
constexpr std::uint32_t kStackDegradedBytes = 512;            // [PROPOSED] any other task
constexpr std::uint32_t kPendingVerifyGraceMs = 30'000;       // [CODE] flight_selftest kWindowMs
constexpr std::uint16_t kExpectedCpuMhz = 240;                // [CODE] sdkconfig.flight
// [PLACEHOLDER] ESP32-S3 operating temperature, not transcribed: check 0.5 is
// not wired in the firmware, the limit exists for the logic test only.
constexpr float kMcuTempMaxC = 85.0F;

// ---- Group 2: IMU ----------------------------------------------------------
constexpr std::uint32_t kImuNominalHz = 960;                  // [CODE] Sensors::kImuSampleRateHz
constexpr float kImuRateTolerance = 0.06F;                    // [PROPOSED] +/- 6 %
constexpr float kImuRateBlockingFraction = 0.50F;             // [PROPOSED]
constexpr float kGravityBlockingMinG = 0.8F;                  // [CODE] landing band (flight_state_machine.h)
constexpr float kGravityBlockingMaxG = 1.2F;
constexpr float kGravityDegradedMinG = 0.93F;                 // [PROPOSED]
constexpr float kGravityDegradedMaxG = 1.07F;
constexpr float kAccelNoiseDegradedMg = 30.0F;                // [PROPOSED] sd per axis
constexpr float kGyroMeanDegradedDps = 3.0F;                  // [PROPOSED]
constexpr float kGyroSdDegradedDps = 1.0F;                    // [PROPOSED]
constexpr float kHighGMaxDifferenceG = 0.3F;                  // [PROPOSED]
constexpr float kImuTempMinC = -20.0F;                        // [PROPOSED]
constexpr float kImuTempMaxC = 80.0F;
constexpr float kImuBaroTempMaxDiffC = 10.0F;
// Stillness gate (plan 3.3).
constexpr float kStillAccelMinG = 0.9F;
constexpr float kStillAccelMaxG = 1.1F;
constexpr float kStillGyroMaxDps = 5.0F;
constexpr float kStillBaroRateMaxMps = 2.0F;
constexpr float kStillKalmanVzMaxMps = 0.5F;
constexpr float kTripAccelG = 1.5F;                           // abort an active test above this
constexpr std::uint32_t kStillWindowMs = 1000;
// [PLACEHOLDER] LSM6DSV320X built-in self-test output-change windows (DS14623
// not transcribed). Check 2.7 is not wired to the sensor; the evaluator
// compares against these so the decision logic can be tested.
constexpr float kSelfTestAccelMinMg = 0.0F;
constexpr float kSelfTestAccelMaxMg = 0.0F;
constexpr float kSelfTestGyroMinDps = 0.0F;
constexpr float kSelfTestGyroMaxDps = 0.0F;
constexpr std::uint32_t kImuSelfTestBlindMs = 300;            // [OWNER] Q10 accepted gap

// ---- Group 2.8 / 4.7: orientation and heading ------------------------------
// Convention (derived from the code, stated in the plan): body +X is the
// nose; upright on the pad the accelerometer reads +1 g on +X.
constexpr float kPadAxialMinG = 0.85F;                        // [PROPOSED]
constexpr float kPadAxialMaxG = 1.15F;
constexpr float kPadTiltMaxDeg = 20.0F;                       // X/|a| >= cos(20 deg)
constexpr float kPadAxialFractionMin = 0.9397F;               // cos(20 deg)

// ---- Group 3: barometer ----------------------------------------------------
constexpr std::int32_t kBaroPressureMinPa = 50'000;           // [PROPOSED]
constexpr std::int32_t kBaroPressureMaxPa = 110'000;
constexpr float kBaroTempMinC = -20.0F;
constexpr float kBaroTempMaxC = 60.0F;
constexpr float kBaroNoiseMaxPa = 25.0F;                      // [PROPOSED] sd over 2 s
constexpr float kBaroRateMaxMps = 1.0F;
constexpr std::uint32_t kBaroFrozenSamples = 100;
constexpr std::uint32_t kBaroNominalHz = 50;                  // [PROPOSED]
constexpr float kBaroRateTolerance = 0.20F;
constexpr std::uint32_t kBaroReadyWithinMs = 10'000;

// ---- Group 4: attitude / Kalman -------------------------------------------
constexpr float kPadAglMaxM = 1.5F;                           // [PROPOSED]
constexpr float kPadVzMaxMps = 0.5F;
constexpr std::uint32_t kAttitudeValidByMs = 60'000;

// ---- Group 5/6: NAND and log session --------------------------------------
constexpr std::uint32_t kNandBadBlocksDegraded = 10;          // [PROPOSED] chip allows 20
constexpr std::uint32_t kImuFreeSecondsBlocking = 150;        // [PROPOSED]
constexpr std::uint32_t kImuFreeSecondsDegraded = 300;
constexpr std::uint32_t kBinaryFreeSecondsBlocking = 600;
constexpr std::uint32_t kImuBytesPerSecond = 960 * 24;        // [CODE] kImuRateHz * kImuRecordSize
constexpr std::uint32_t kBinaryBytesPerSecond = 50 * 24;      // [CODE] kLogRateHz * kRecordSize
constexpr std::uint32_t kLoggerHeartbeatMaxMs = 5000;         // [PLAN] below the 8 s task watchdog
constexpr std::uint32_t kEndToEndTimeoutMs = 3000;            // [PLAN] 6.3
constexpr std::uint32_t kPrelaunchRingMinPercent = 95;        // [PLAN] 6.6
constexpr std::uint32_t kPrelaunchRingAfterMs = 2500;
// NAND timing: [CODE] w25n01kv.h quotes the datasheet worst cases (page read
// 60 us, page program 700 us, block erase 10 ms).
constexpr std::uint32_t kNandEraseMaxUs = 10'000;
constexpr std::uint32_t kNandProgramMaxUs = 700;
constexpr std::uint32_t kNandReadMaxUs = 60;

// ---- Group 7: GPS ----------------------------------------------------------
constexpr std::uint32_t kGpsReadyWithinMs = 45'000;           // [PROPOSED] bench: about 22 s
constexpr std::uint32_t kGpsRateNominalHz = 25;
constexpr std::uint32_t kGpsRateTolerance = 2;
constexpr std::uint32_t kGpsMinSatellites = 4;

// ---- Group 11: power -------------------------------------------------------
// [OWNER] Q3: 1S Li-ion pack (Witness and Iris measure different packs).
// [PLACEHOLDER] the cut-off voltages are NOT from a pack datasheet.
constexpr std::uint32_t kBatteryBlockingBelowMv = 3300;
constexpr std::uint32_t kBatteryDegradedBelowMv = 3600;
constexpr std::uint32_t kBatteryDegradedAboveMv = 4300;
constexpr std::uint32_t kBatteryNoiseMaxMv = 50;
// [OWNER] Q11: the next board revision wires the 3V3 rail to a spare ADC pin
// and monitors the supply current. Disabled until the hardware exists.
constexpr bool kRail3v3MonitorPresent = false;
constexpr bool kSupplyCurrentMonitorPresent = false;
// [PLACEHOLDER] limits for those checks, only used when the hardware exists.
constexpr std::uint32_t kRail3v3MinMv = 3000;
constexpr std::uint32_t kRail3v3MaxMv = 3600;
constexpr std::uint32_t kSupplyCurrentMaxMa = 1000;

// ---- Group 12: clocks ------------------------------------------------------
constexpr float kClockToleranceFraction = 0.06F;              // IMU, same as 2.2

// ---- Commanded run ---------------------------------------------------------
constexpr std::uint32_t kCommandedRunMaxMs = 6000;            // [PLAN] 4..6 s
constexpr std::uint32_t kActiveTestTimeoutMs = 3000;          // owner task did not answer: ERROR
constexpr std::uint32_t kReportPeriodNotGoMs = 10'000;        // radio resend while not GO
constexpr std::uint32_t kFaultMaxDurationMs = 60'000;         // injected faults always expire

}  // namespace bit::limits
