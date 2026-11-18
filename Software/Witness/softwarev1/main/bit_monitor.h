#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "bit_codec.h"
#include "bit_logic.h"
#include "bit_policy.h"
#include "bit_stats.h"
#include "esp_err.h"
#include "sensors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class FlashLogger;
class FlightStateMachine;
class UartDriver;
class WitnessStatus;
class SpiDataForwarder;

// Built-in self test runtime (BIT_PLAN_2026-11-18.md). One low-priority task on
// core 0, not subscribed to the task watchdog: a BIT hang can never reboot the
// vehicle, and BIT only reports. It never touches logging, launch detection,
// loads, NVS or reboots. Active test slices run inside the task that owns the
// bus (the flash logger), requested through atomics.
//
// Read-only probes: the owners declare `friend class BitProbe` and BitProbe
// (bit_monitor.cpp) reads their counters and flags without locks.
class BitProbe;

class BitMonitor final {
public:
    // Sends a 0x06 ack on the link the command arrived on (set by UsbSerialEcho).
    using AckSink = void (*)(void *context, bool radio, std::uint16_t command, const std::uint8_t *data6,
                             std::uint8_t result, std::uint16_t detail);

    BitMonitor(Sensors &sensors, FlashLogger &flash_logger, FlightStateMachine &flight_state_machine,
               UartDriver &gps_uart, WitnessStatus &witness_status, SpiDataForwarder &data_forwarder,
               bool resumed_in_flight);
    BitMonitor(const BitMonitor &) = delete;
    BitMonitor &operator=(const BitMonitor &) = delete;

    esp_err_t start();
    void set_ack_sink(AckSink sink, void *context) { ack_sink_ = sink; ack_context_ = context; }

    // Handles BIT_RUN / BIT_REPORT / BIT_CONFIG / BIT_FAULT (bit::policy::is_bit_command).
    // data6 = the command's six data bytes; the ack (immediate, or at the end of a
    // commanded run) goes out through the ack sink. Any task.
    void handle_command(std::uint16_t command, const std::uint8_t *data6, bool radio);

private:
    friend class BitProbe;

    static constexpr UBaseType_t kTaskPriority = 1;
    static constexpr std::uint32_t kTaskStackSize = 4096;
    static constexpr std::uint32_t kPollPeriodMs = 10;
    static constexpr std::uint32_t kStartupEvalMs = 500;
    static constexpr std::uint32_t kMonitorEvalMs = 1000;
    static constexpr std::uint32_t kNandLogPeriodMs = 10'000;

    // A request from the command task (run or report), consumed by the BIT task.
    struct Request {
        enum Kind : std::uint8_t { Run, Report };
        Kind kind = Run;
        bool radio = false;
        std::uint8_t data6[6]{};
        std::uint8_t options = 0;
        bool passive = false;
    };

    static void task_entry(void *context);
    void run();
    void startup_snapshot();
    void poll_samples(std::uint32_t now_ms);
    void evaluate(bit::RunKind kind_hint, std::uint32_t now_ms);
    void collect(bit::Inputs &in, std::uint32_t now_ms);
    void publish(const bit::Summary &summary, bool changed, std::uint32_t now_ms, bool force_radio);
    void log_failures(const bit::Summary &summary);
    void process_request(const Request &request, std::uint32_t now_ms);
    void run_commanded(const Request &request, std::uint32_t now_ms);
    // Asks the flash logger task for an active test slice and waits (bounded) for the answer.
    // false = aborted by the tripwire (the vehicle moved or the state changed).
    bool wait_active(bit::ActiveResult &result, std::uint8_t request_kind, std::uint8_t start_state, bool &aborted);
    void send_detail(bool over_radio);
    bit::GateInputs gate_inputs(std::uint32_t now_ms) const;
    void apply_fault_view(bit::Inputs &in, std::uint32_t now_ms);
    void service_faults(std::uint32_t now_ms);
    void handle_run(const std::uint8_t *data6, bool radio);
    void handle_report(const std::uint8_t *data6, bool radio);
    void handle_fault(const std::uint8_t *data6, bool radio);
    void reply(bool radio, std::uint16_t command, const std::uint8_t *data6, std::uint8_t result,
               std::uint16_t detail);

    Sensors &sensors_;
    FlashLogger &flash_logger_;
    FlightStateMachine &flight_state_machine_;
    UartDriver &gps_uart_;
    WitnessStatus &witness_status_;
    SpiDataForwarder &data_forwarder_;
    const bool resumed_;
    TaskHandle_t task_ = nullptr;
    AckSink ack_sink_ = nullptr;
    void *ack_context_ = nullptr;

    bit::Engine engine_{};
    bit::Summary last_summary_{};
    bool have_summary_ = false;
    std::uint32_t last_radio_ms_ = 0;
    std::uint32_t last_nand_ms_ = 0;
    std::uint32_t last_usb_ms_ = 0;
    std::uint32_t last_eval_ms_ = 0;
    std::uint32_t pad_idle_since_ms_ = 0;
    bool in_pad_idle_ = false;
    std::uint8_t nonce_[4]{};

    // Collector state (BIT task only).
    bit::StillWindow still_{};
    bit::BaroWindow baro_{};
    bit::RateMeter imu_rate_{2000};
    bit::GrowthFlag imu_loss_{10'000};
    bit::GrowthFlag imu_fault_{10'000};
    bit::GrowthFlag log_drops_{10'000};
    bit::GrowthFlag radio_tx_{10'000};
    bit::ResetClass reset_class_ = bit::ResetClass::Unknown;
    bool image_known_ = false;
    bool image_flight_ = false, image_pending_ = false, image_slot_ok_ = false, image_other_ok_ = false;
    std::uint16_t tasks_present_ = bit::kTasksAll;
    std::uint16_t wdt_subscribed_ = bit::kTasksRequired;
    bool wdt_known_ = false;
    bool nvs_ok_ = true;
    std::uint32_t slow_ms_ = 0;
    std::int32_t stack_sensors_ = -1, stack_logger_ = -1, stack_capture_ = -1, stack_other_min_ = -1;
    bool fault_heap_blocking_ = false;
    bool have_sample_ = false;
    Sensors::Sample last_sample_{};
    std::uint32_t task_scan_ms_ = 0;
    float last_pressure_alt_ = 0.0F;
    bit::ActiveResult image_integrity_{};
    bit::ActiveResult heap_structure_{};
    bit::ActiveResult nand_write_{};
    bit::ActiveResult log_e2e_{};
    bit::ActiveResult log_post_{};
    bool active_ran_ = false;

    // Commanded run (handle_command -> task).
    std::atomic<bool> run_pending_{false};
    std::atomic<bool> run_active_{false};
    Request request_{};
    portMUX_TYPE request_lock_ = portMUX_INITIALIZER_UNLOCKED;
    std::atomic<std::uint32_t> verdict_word_{0};   // last verdict (high byte) and groups (low byte) for ack details

    // Fault injection (bench, HITL): what the BIT sees, never a flight function.
    struct Fault {
        std::atomic<bool> active{false};
        std::atomic<std::uint32_t> until_ms{0};
    };
    Fault faults_[16];
    void *heap_hog_ = nullptr;
};
