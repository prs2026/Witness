#include "bit_monitor.h"

#include <cmath>
#include <cstring>

#include "boot_manager.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "flash_logger.h"
#include "flight_state_machine.h"
#include "hardware_pins.h"
#include "nvs.h"
#include "sdkconfig.h"
#include "sensors.h"
#include "spi_data_forwarder.h"
#include "uart_driver.h"
#include "witness_status.h"

namespace {
constexpr char kLogTag[] = "bit";

// Survives warm resets (RTC_NOINIT) and is cleared on power-on: the reset history of check 0.1.
RTC_NOINIT_ATTR bit::ResetHistory g_reset_history;
RTC_NOINIT_ATTR std::uint32_t g_last_uptime_s;

std::uint32_t ms_now() { return static_cast<std::uint32_t>(esp_timer_get_time() / 1000); }

bit::ResetClass classify_reset(const esp_reset_reason_t reason)
{
    switch (reason) {
    case ESP_RST_POWERON: return bit::ResetClass::PowerOn;
    case ESP_RST_EXT:
    case ESP_RST_USB:
    case ESP_RST_JTAG:
    case ESP_RST_DEEPSLEEP: return bit::ResetClass::External;
    case ESP_RST_SW: return bit::ResetClass::Software;
    case ESP_RST_PANIC: return bit::ResetClass::Panic;
    case ESP_RST_TASK_WDT: return bit::ResetClass::TaskWdt;
    case ESP_RST_INT_WDT:
    case ESP_RST_WDT: return bit::ResetClass::IntWdt;
    case ESP_RST_BROWNOUT: return bit::ResetClass::Brownout;
    default: return bit::ResetClass::Other;
    }
}

struct TaskName {
    const char *name;
    std::uint16_t bit;
};
constexpr TaskName kTasks[] = {
    {"sensors", bit::kTaskSensors},         {"flash_logger", bit::kTaskFlashLogger},
    {"flash_capture", bit::kTaskFlashCapture}, {"uart1", bit::kTaskUart1},
    {"usb_serial_echo", bit::kTaskUsb},     {"heartbeat", bit::kTaskHeartbeat},
    {"data_forwarder", bit::kTaskForwarder}, {"iris_can", bit::kTaskCanLink},
};
}  // namespace

// ---------------------------------------------------------------------------------------------
// BitProbe: the only code that reads the owners' private state. Every read is a plain load of an
// aligned word or an atomic: no locks, no calls into an owner. Values are snapshots; a torn
// composite is possible and harmless (the checks are debounced).
// ---------------------------------------------------------------------------------------------
class BitProbe {
public:
    static void collect(BitMonitor &m, bit::Inputs &in, std::uint32_t now);
    static void flash_request(FlashLogger &f, std::uint8_t kind, const std::uint8_t nonce[4])
    {
        std::memcpy(f.bit_nonce_, nonce, 4);
        f.bit_value_.store(0);
        f.bit_state_.store(FlashLogger::kBitStatePending, std::memory_order_release);
        f.bit_request_.store(kind, std::memory_order_release);
    }
    static std::uint8_t flash_state(FlashLogger &f, std::uint16_t &value)
    {
        const std::uint8_t s = f.bit_state_.load(std::memory_order_acquire);
        value = f.bit_value_.load(std::memory_order_relaxed);
        return s;
    }
    static void flash_cancel(FlashLogger &f) { f.bit_request_.store(FlashLogger::kBitRequestCancel); }
    static constexpr std::uint8_t kReqEndToEnd = FlashLogger::kBitRequestEndToEnd;
    static constexpr std::uint8_t kReqNandWrite = FlashLogger::kBitRequestNandWrite;
    static constexpr std::uint8_t kStatePending = FlashLogger::kBitStatePending;
    static constexpr std::uint8_t kStatePass = FlashLogger::kBitStatePass;
    static constexpr std::uint8_t kStateError = FlashLogger::kBitStateError;
    static bool budget_zero(const FlightStateMachine &f) { return f.launch_accel_budget_us_ == 0U; }
    static float kalman_vz(const Sensors &s)
    {
        float v[3] = {0, 0, 0};
        s.position_filter_.velocity(v);
        return std::isfinite(v[2]) ? v[2] : 0.0F;
    }
    static bool baro_rate_valid(const Sensors &s, std::uint64_t timestamp_us) { return s.baro_rate_.valid(timestamp_us); }
};

BitMonitor::BitMonitor(Sensors &sensors, FlashLogger &flash_logger, FlightStateMachine &flight_state_machine,
                       UartDriver &gps_uart, WitnessStatus &witness_status, SpiDataForwarder &data_forwarder,
                       const bool resumed_in_flight)
    : sensors_(sensors),
      flash_logger_(flash_logger),
      flight_state_machine_(flight_state_machine),
      gps_uart_(gps_uart),
      witness_status_(witness_status),
      data_forwarder_(data_forwarder),
      resumed_(resumed_in_flight)
{
}

esp_err_t BitMonitor::start()
{
    if (task_ != nullptr) return ESP_ERR_INVALID_STATE;
    // Core 0, priority 1 (the idle-adjacent priority of cpu_load), not subscribed to the task watchdog.
    return xTaskCreatePinnedToCore(task_entry, "bit", kTaskStackSize, this, kTaskPriority, &task_, 0) == pdPASS
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}

void BitMonitor::task_entry(void *context) { static_cast<BitMonitor *>(context)->run(); }

// ---- collector -------------------------------------------------------------------------------

void BitProbe::collect(BitMonitor &m, bit::Inputs &in, const std::uint32_t now)
{
    using namespace bit;
    Sensors &s = m.sensors_;
    FlashLogger &f = m.flash_logger_;
    FlightStateMachine &fsm = m.flight_state_machine_;

    in.uptime_ms = now;
    in.flight_state = static_cast<std::uint8_t>(fsm.state());
    in.resumed = m.resumed_;
    in.hitl_active = s.hitl_active();
    in.status0 = m.witness_status_.flags();
    in.status1 = static_cast<std::uint8_t>(m.witness_status_.state_flags() | m.witness_status_.flight_state());
    const bool ground = in.flight_state == kPadIdle || in.flight_state == kLanded;

    // Group 0
    in.reset = m.reset_class_;
    in.image_known = m.image_known_;
    in.image_flight = m.image_flight_;
    in.image_pending_verify = m.image_pending_;
    in.image_slot_matches = m.image_slot_ok_;
    in.image_other_valid = m.image_other_ok_;
    in.heap_known = true;
    in.heap_free = static_cast<std::uint32_t>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    in.heap_min = static_cast<std::uint32_t>(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
    in.heap_largest = static_cast<std::uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    in.image_integrity = m.image_integrity_;
    in.heap_structure = m.heap_structure_;

    // Group 1 (task handles are looked up in the slow path, see BitMonitor::collect)
    in.tasks_present = m.tasks_present_;
    in.wdt_known = m.wdt_known_;
    in.wdt_subscribed = m.wdt_subscribed_;
    in.nvs_ok = m.nvs_ok_;
    in.cpu_mhz = static_cast<std::uint16_t>(CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
    in.automatic_transitions = FlightStateMachine::kAutomaticTransitionsEnabled;
    in.flight_image = boot_manager::running_flight();

    // Group 2: IMU
    in.imu_ready = s.imu_ready_;
    in.imu_fault_events = s.imu_fault_events_;
    in.imu_fault_recent = m.imu_fault_.update(s.imu_fault_events_, now);
    in.imu_reinit_failed = s.imu_reinit_failed_;
    if (in.hitl_active) {
        // HITL replaces the IMU: the 960 Hz stream is simulated, so there is no FIFO rate to judge.
        in.imu_rate_valid = true;
        in.imu_rate_hz = static_cast<float>(limits::kImuNominalHz);
    } else {
        in.imu_rate_valid = m.imu_rate_.update(s.fifo_samples_, now);
        in.imu_rate_hz = m.imu_rate_.rate_hz();
    }
    in.imu_loss_growth = m.imu_loss_.update(s.fifo_overruns_ + s.fifo_bad_words_, now);
    in.still = m.still_.latest(now);

    // Group 3, 4: the newest sample (read by BitMonitor::poll_samples at 100 Hz on the ground)
    in.baro = m.baro_.latest(now);
    in.rule2_ready = (in.status0 & 0x02U) != 0U && s.imu_ready_;
    in.rule1_ready = s.position_filter_.initialized() && std::isfinite(s.position_filter_.up().p);
    in.attitude_valid = s.attitude_.valid();
    in.tilt_deg = s.attitude_.valid() ? s.attitude_.tilt_from_vertical_deg() : 0.0F;
    in.filter_finite = std::isfinite(s.position_filter_.up().p);
    in.filter_pad_hold = s.position_filter_.still();
    in.filter_agl_m = in.filter_finite ? s.position_filter_.up().p : 0.0F;
    in.filter_vz_mps = kalman_vz(s);
    in.filter_gps_origin_ok = !(in.status1 & IRIS_WITNESS_STATUS_GPS_LOCKED_MASK) || s.position_filter_.gps_origin_set();
    in.filter_diverged_recent = s.position_filter_.divergence_resets() != 0U;
    in.launch_budget_zero = budget_zero(fsm);
    in.pad_heading_set = s.pad_heading_active_;
    in.calibration_valid = s.calibration_.gyro_valid && s.calibration_.accel_valid;
    in.calibration_loaded = in.calibration_valid;
    in.state_record_matches = true;

    // Group 5, 6: NAND and the log session (all plain loads of the logger's own counters)
    in.nand_chip_ok = (in.status0 & IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK) == 0U;
    in.writer_failing = f.write_health_.failing();
    in.writer_backing_off = f.backoff_active_;
    in.writer_failed_blocks = f.write_health_.total_failed_blocks();
    in.nand_write = m.nand_write_;
    in.session_ready = f.session_ready_;
    in.slots_full = f.slots_full_ || f.choice_action_ == log_slot::Action::SlotsFull;
    {
        const auto left = [](const FlashLogger::Writer &w) -> std::uint32_t {
            const std::uint32_t capacity = w.block_count * W25n01kv::kPagesPerBlock;
            return (w.full || w.page_index >= capacity) ? 0U : capacity - w.page_index;
        };
        in.free_space_known = f.session_ready_;
        const std::uint64_t imu_bytes = static_cast<std::uint64_t>(left(f.imu_writer_)) * FlashLogger::kPagePayloadSize;
        const std::uint64_t bin_bytes = static_cast<std::uint64_t>(left(f.binary_writer_)) * FlashLogger::kPagePayloadSize;
        in.imu_seconds_left = static_cast<std::uint32_t>(imu_bytes / limits::kImuBytesPerSecond);
        in.binary_seconds_left = static_cast<std::uint32_t>(bin_bytes / limits::kBinaryBytesPerSecond);
    }
    in.queue_over_half =
        (f.imu_record_queue_ && uxQueueMessagesWaiting(f.imu_record_queue_) > FlashLogger::kImuQueueDepth / 2U) ||
        (f.binary_record_queue_ && uxQueueMessagesWaiting(f.binary_record_queue_) > FlashLogger::kRecordQueueDepth / 2U) ||
        (f.packet_queue_ && uxQueueMessagesWaiting(f.packet_queue_) > FlashLogger::kPacketQueueDepth / 2U);
    in.drop_growth = m.log_drops_.update(f.packets_dropped_.load() + f.imu_records_dropped_.load() +
                                             f.chunks_dropped_ + f.unreadable_pages_, now);
    {
        const std::uint32_t passes = f.bit_passes_.load(std::memory_order_relaxed);
        static std::uint32_t last_passes = 0, last_change_ms = 0;   // BIT task only
        if (passes != last_passes || last_change_ms == 0U) { last_passes = passes; last_change_ms = now; }
        in.logger_pass_age_ms = now - last_change_ms;
    }
    in.prelaunch_known = in.flight_state == kPadIdle && f.session_ready_;
    {
        const std::uint32_t a = static_cast<std::uint32_t>(f.prelaunch_record_count_ * 100U / FlashLogger::kPrelaunchRecordCount);
        const std::uint32_t b = static_cast<std::uint32_t>(f.imu_prelaunch_count_ * 100U / FlashLogger::kImuPrelaunchRecordCount);
        in.prelaunch_fill_percent = a < b ? a : b;
    }
    in.pad_idle_ms = m.in_pad_idle_ ? now - m.pad_idle_since_ms_ : 0U;
    in.log_end_to_end = m.log_e2e_;
    in.log_post_flight = m.log_post_;

    // Group 7: GPS
    {
        const GpsReceiver &g = m.gps_uart_.gps_;
        switch (g.status()) {
        case GpsReceiver::Status::Disabled: in.gps_status = Inputs::Gps::Disabled; break;
        case GpsReceiver::Status::Configuring: in.gps_status = Inputs::Gps::Configuring; break;
        case GpsReceiver::Status::Ready: in.gps_status = Inputs::Gps::Ready; break;
        case GpsReceiver::Status::Failed: in.gps_status = Inputs::Gps::Failed; break;
        case GpsReceiver::Status::Streaming: in.gps_status = Inputs::Gps::Streaming; break;
        case GpsReceiver::Status::Recovering: in.gps_status = Inputs::Gps::Recovering; break;
        }
        in.gps_rate_hz = g.measured_rate_hz();
        const NmeaGps::Fix fix = m.gps_uart_.latest_gps_fix();
        in.gps_fix = fix.valid;
        in.gps_satellites = fix.satellites;
    }
    in.utc_valid = s.have_utc_fix_ && s.utc_clock_.valid();

    // Group 8: radio
    in.radio_ready = s.radio_ready_;
    in.radio_faulted = s.radio_.faulted();
    in.radio_front_end_on = s.radio_.front_end_enabled();
    in.radio_power_dbm = s.radio_.tx_power_dbm();
    in.radio_tx_failures_grew = m.radio_tx_.update(s.radio_tx_failed_ + s.radio_.fault_count(), now);
    in.radio_dio1_stuck = false;   // the DIO1 slow-poll counter is not in this tree; wired when it merges

    // Group 11: power
    in.battery_adc_calibrated = s.battery_adc_calibration_ != nullptr;
#ifdef CONFIG_ESP_BROWNOUT_DET
    in.brownout_detector = true;
#else
    in.brownout_detector = false;
#endif
    (void)ground;
}

// ---- samples ---------------------------------------------------------------------------------

void BitMonitor::poll_samples(const std::uint32_t now_ms)
{
    // On the ground at 100 Hz (window statistics); in flight at 1 Hz (plausibility only): the
    // sample copy takes the sensor task's publish mutex for a few microseconds.
    const std::uint8_t state = static_cast<std::uint8_t>(flight_state_machine_.state());
    const bool ground = state == bit::kPadIdle || state == bit::kLanded;
    static std::uint32_t last_ms = 0;
    if (now_ms - last_ms < (ground ? 10U : 1000U)) return;
    last_ms = now_ms;
    Sensors::Sample s{};
    if (sensors_.latest_sample(s, 0) != ESP_OK) return;
    if (ground) {
        still_.add(s.low_g_accel_mg, s.gyro_mdps, s.high_g_accel_mg, s.high_g_accel_ready, now_ms);
        if (s.ms5607_ready && s.pressure_centi_mbar != 0)
            baro_.add(s.pressure_centi_mbar, s.ms5607_temperature_centi_celsius, s.barometric_altitude_meters, now_ms);
    }
    last_sample_ = s;
    have_sample_ = true;
}

// ---- evaluation ------------------------------------------------------------------------------

void BitMonitor::evaluate(const bit::RunKind, const std::uint32_t now_ms)
{
    bit::Inputs in;
    collect(in, now_ms);
    engine_.update(in);
    const bit::Summary s = engine_.summarize(now_ms);
    const bool changed = !have_summary_ || s.verdict != last_summary_.verdict ||
                         s.blocking_fail_mask != last_summary_.blocking_fail_mask ||
                         s.degraded_fail_mask != last_summary_.degraded_fail_mask ||
                         s.first_fail_id != last_summary_.first_fail_id || s.bench != last_summary_.bench ||
                         s.core_done != last_summary_.core_done || s.settled != last_summary_.settled;
    if (changed) log_failures(s);
    publish(s, changed, now_ms, false);
    last_summary_ = s;
    have_summary_ = true;
}

void BitMonitor::collect(bit::Inputs &in, const std::uint32_t now_ms)
{
    // Slow items first (never after a warm reset in flight: passive status reads only).
    if (!resumed_ && now_ms - slow_ms_ >= 10'000U) {
        slow_ms_ = now_ms;
        nvs_stats_t stats{};
        nvs_ok_ = nvs_get_stats(nullptr, &stats) == ESP_OK && stats.free_entries >= 10U;
    }
    if (now_ms - task_scan_ms_ >= 10'000U || task_scan_ms_ == 0U) {
        task_scan_ms_ = now_ms;
        std::uint16_t present = 0, subscribed = 0;
        std::int32_t smallest_other = -1;
        for (const TaskName &t : kTasks) {
            const TaskHandle_t h = xTaskGetHandle(t.name);
            if (h == nullptr) continue;
            present |= t.bit;
            if (esp_task_wdt_status(h) == ESP_OK) subscribed |= t.bit;
            const std::int32_t free_bytes = static_cast<std::int32_t>(uxTaskGetStackHighWaterMark(h));
            switch (t.bit) {
            case bit::kTaskSensors: stack_sensors_ = free_bytes; break;
            case bit::kTaskFlashLogger: stack_logger_ = free_bytes; break;
            case bit::kTaskFlashCapture: stack_capture_ = free_bytes; break;
            default:
                if (smallest_other < 0 || free_bytes < smallest_other) smallest_other = free_bytes;
                break;
            }
        }
        tasks_present_ = present;
        wdt_subscribed_ = subscribed;
        wdt_known_ = true;
        stack_other_min_ = smallest_other;
    }
    BitProbe::collect(*this, in, now_ms);
    in.stack_sensors = stack_sensors_;
    in.stack_logger = stack_logger_;
    in.stack_capture = stack_capture_;
    in.stack_other_min = stack_other_min_;
    // Sample-derived values.
    if (have_sample_) {
        in.imu_temp_c = last_sample_.temperature_celsius;
        in.baro_temp_c = last_sample_.ms5607_temperature_centi_celsius / 100.0F;
        in.temps_valid = last_sample_.temperature_ready && last_sample_.ms5607_ready;
        in.pressure_pa = last_sample_.pressure_centi_mbar;
        in.baro_sample_valid = last_sample_.ms5607_ready && last_sample_.pressure_centi_mbar != 0;
        in.baro_ready = last_sample_.ms5607_ready;
        in.battery_valid = last_sample_.battery_voltage_mv != 0U;
        in.battery_mv = last_sample_.battery_voltage_mv;
        in.rule3_ready = in.baro_ready && BitProbe::baro_rate_valid(sensors_, last_sample_.timestamp_us);
        in.rule2_ready = in.rule2_ready && std::isfinite(last_sample_.filtered_accel_mg[0]);
    } else {
        in.temps_valid = false;
        in.baro_sample_valid = false;
        in.battery_valid = false;
        in.baro_ready = false;
        in.rule3_ready = false;
    }
    in.gps_alt_diff_valid = false;
    // Gate for 4.4 and the stillness-based checks.
    in.restart_pending = false;
    apply_fault_view(in, now_ms);
}

// ---- faults (bench) -----------------------------------------------------------------------------

void BitMonitor::service_faults(const std::uint32_t now_ms)
{
    for (std::size_t i = 0; i < 16; ++i) {
        if (!faults_[i].active.load()) continue;
        if (static_cast<std::int32_t>(now_ms - faults_[i].until_ms.load()) >= 0 || !sensors_.hitl_active()) {
            faults_[i].active.store(false);
            ESP_LOGW(kLogTag, "BIT fault %u cleared", static_cast<unsigned>(i));
        }
    }
}

void BitMonitor::apply_fault_view(bit::Inputs &in, const std::uint32_t)
{
    using namespace bit;
    const auto on = [this](const unsigned id) { return faults_[id].active.load(); };
    if (on(IRIS_BIT_FAULT_NAND_WRITE)) { in.writer_failing = true; in.writer_failed_blocks += 3; }
    if (on(IRIS_BIT_FAULT_NAND_ECC)) {
        if (in.nand_write.state == ActiveResult::State::Done) { in.nand_write.pass = false; in.nand_write.value = 4; }
        if (in.log_end_to_end.state == ActiveResult::State::Done) { in.log_end_to_end.pass = false; in.log_end_to_end.value = 3; }
    }
    if (on(IRIS_BIT_FAULT_IMU_SILENT)) {
        in.imu_ready = false;
        in.status0 = static_cast<std::uint8_t>(in.status0 & ~0x26U);
        in.imu_rate_valid = true; in.imu_rate_hz = 0.0F;
        in.rule2_ready = false;
    }
    if (on(IRIS_BIT_FAULT_BARO_FROZEN)) { in.baro.valid = true; in.baro.frozen = true; in.rule3_ready = false; }
    if (on(IRIS_BIT_FAULT_RADIO)) { in.radio_ready = false; in.radio_faulted = true; }
    if (on(IRIS_BIT_FAULT_GRAVITY)) { in.still.valid = true; in.still.accel_mag_g = 2.0F; }
    if (on(IRIS_BIT_FAULT_HEAP_PRESSURE)) {
        // View-level: nothing is allocated. Level 1 (d2 = 1 on the command) shows a blocking minimum.
        in.heap_known = true; in.heap_min = fault_heap_blocking_ ? 4000U : 10'000U; in.heap_largest = 5000U;
    }
    if (on(IRIS_BIT_FAULT_LOGGER_STALL)) in.logger_pass_age_ms = 6000;
    if (on(IRIS_BIT_FAULT_WDT_UNSUBSCRIBED)) in.wdt_subscribed = static_cast<std::uint16_t>(in.wdt_subscribed & ~kTaskSensors);
}

// ---- publication --------------------------------------------------------------------------------

void BitMonitor::publish(const bit::Summary &s, const bool changed, const std::uint32_t now_ms, const bool force_radio)
{
    bit::publish(s.verdict, s.bench);
    verdict_word_.store((static_cast<std::uint32_t>(s.verdict) << 8U) | s.blocking_fail_groups);
    std::uint8_t frame[bit::codec::kSummaryFrameLength];
    bit::codec::encode_summary(s, frame);
    if (now_ms - last_usb_ms_ >= bit::limits::kMonitorPeriodMs || changed) {
        last_usb_ms_ = now_ms;
        (void)data_forwarder_.queue_packet(IRIS_PACKET_ID_BIT_SUMMARY, frame + IRIS_PACKET_ID_LENGTH,
                                           IRIS_PACKET_BIT_SUMMARY_DATA_LENGTH);
    }
    if (force_radio || bit::policy::radio_summary_due(s.flight_state, s.verdict, changed, now_ms - last_radio_ms_)) {
        last_radio_ms_ = now_ms;
        (void)data_forwarder_.queue_immediate_radio_packet(frame, sizeof(frame));
    }
    if (changed || now_ms - last_nand_ms_ >= kNandLogPeriodMs) {
        last_nand_ms_ = now_ms;
        (void)flash_logger_.log_packet(FlashLogger::kPacketSourceBit, frame, sizeof(frame));
    }
    g_last_uptime_s = now_ms / 1000U;
}

void BitMonitor::log_failures(const bit::Summary &s)
{
    static const char *const kVerdict[] = {"NOT RUN", "RUNNING", "GO", "GO WITH WARNINGS", "NO-GO"};
    ESP_LOGW(kLogTag, "BIT verdict %s%s (blocking groups 0x%04X, degraded 0x%04X, first %02X)",
             kVerdict[static_cast<unsigned>(s.verdict) <= 4U ? static_cast<unsigned>(s.verdict) : 0U],
             s.bench ? " [bench condition]" : "", static_cast<unsigned>(s.blocking_fail_mask),
             static_cast<unsigned>(s.degraded_fail_mask), static_cast<unsigned>(s.first_fail_id));
    for (std::size_t i = 0; i < bit::kCheckCount; ++i) {
        const bit::Eval &e = engine_.slot(i).shown;
        if (e.result != bit::Result::Fail && e.result != bit::Result::Error) continue;
        ESP_LOGW(kLogTag, "BIT: %02X %s: %s class %u value %u%s", static_cast<unsigned>(bit::kChecks[i].id),
                 bit::kChecks[i].name, e.result == bit::Result::Error ? "ERROR" : "FAIL",
                 static_cast<unsigned>(e.cls), static_cast<unsigned>(e.value), e.bench ? " (bench)" : "");
    }
}

// ---- startup snapshot (S0) ---------------------------------------------------------------------

void BitMonitor::startup_snapshot()
{
    reset_class_ = classify_reset(esp_reset_reason());
    if (reset_class_ == bit::ResetClass::PowerOn || !g_reset_history.valid()) g_reset_history.clear();
    g_reset_history.push(reset_class_, g_last_uptime_s);
    ESP_LOGI(kLogTag, "reset class %u; %u fault resets in the last %u", static_cast<unsigned>(reset_class_),
             static_cast<unsigned>(g_reset_history.fault_count()), static_cast<unsigned>(g_reset_history.count));
    if (!resumed_) {
        // Flash reads of the other slot's record: pad boots only (never after a warm reset in flight).
        image_flight_ = boot_manager::running_flight();
        image_pending_ = boot_manager::running_pending_verify();
        const int running = boot_manager::running_slot();
        image_slot_ok_ = running >= 0 && boot_manager::last_good_flight() == running;
        image_other_ok_ = false;
        if (running >= 0) {
            image_info::Info info{};
            const esp_partition_t *other = boot_manager::slot_partition(boot_manager::Kind::Flight, running ^ 1);
            image_other_ok_ = other != nullptr && boot_manager::read_info(other, boot_manager::Kind::Flight, info);
        }
        image_known_ = true;
    }
}

// ---- commands ----------------------------------------------------------------------------------

void BitMonitor::reply(const bool radio, const std::uint16_t command, const std::uint8_t *data6,
                       const std::uint8_t result, const std::uint16_t detail)
{
    if (ack_sink_ != nullptr) ack_sink_(ack_context_, radio, command, data6, result, detail);
}

void BitMonitor::handle_command(const std::uint16_t command, const std::uint8_t *data6, const bool radio)
{
    switch (command) {
    case IRIS_COMMAND_BIT_RUN: handle_run(data6, radio); break;
    case IRIS_COMMAND_BIT_REPORT: handle_report(data6, radio); break;
    case IRIS_COMMAND_BIT_FAULT: handle_fault(data6, radio); break;
    default: reply(radio, command, data6, IRIS_ACK_UNKNOWN, 0); break;   // BIT_CONFIG: reserved
    }
}

void BitMonitor::handle_run(const std::uint8_t *data6, const bool radio)
{
    const std::uint8_t state = static_cast<std::uint8_t>(flight_state_machine_.state());
    const std::uint16_t detail = static_cast<std::uint16_t>(verdict_word_.load());
    const bit::policy::Source source = radio ? bit::policy::Source::Radio : bit::policy::Source::Usb;
    bit::policy::Decision d = bit::policy::allow_bit_run(source, state, resumed_, run_active_.load() || run_pending_.load(),
                                                         data6[0], data6[1]);
    if (d == bit::policy::Decision::Ok && !bit::policy::option_allowed_in_state(data6[1], state))
        d = bit::policy::Decision::Refused;
    if (d == bit::policy::Decision::Ok && (data6[1] & IRIS_BIT_OPTION_IMU_SELF_TEST) != 0U) {
        // The IMU hardware self-test needs the LSM6 register sequence from the datasheet, which is not
        // transcribed: the option is accepted and the check reports SKIPPED (never PASS).
        ESP_LOGW(kLogTag, "BIT: IMU self-test requested; not wired (datasheet pending), reported SKIPPED");
    }
    if (d != bit::policy::Decision::Ok) {
        const std::uint8_t result = d == bit::policy::Decision::Busy         ? IRIS_ACK_BUSY
                                    : d == bit::policy::Decision::BadArgument ? IRIS_ACK_BAD_ARGUMENT
                                                                              : IRIS_ACK_REFUSED;
        ESP_LOGW(kLogTag, "BIT run %s refused (%u)", radio ? "(radio)" : "(USB)", static_cast<unsigned>(d));
        reply(radio, IRIS_COMMAND_BIT_RUN, data6, result, detail);
        return;
    }
    portENTER_CRITICAL(&request_lock_);
    const bool taken = run_pending_.load();
    if (!taken) {
        request_.kind = Request::Run;
        request_.radio = radio;
        std::memcpy(request_.data6, data6, 6);
        request_.options = data6[1];
        request_.passive = data6[0] == IRIS_BIT_RUN_PASSIVE;
        run_pending_.store(true);
    }
    portEXIT_CRITICAL(&request_lock_);
    if (taken) reply(radio, IRIS_COMMAND_BIT_RUN, data6, IRIS_ACK_BUSY, detail);
    // else: the ack comes from the BIT task when the run is done.
}

void BitMonitor::handle_report(const std::uint8_t *data6, const bool radio)
{
    const std::uint8_t state = static_cast<std::uint8_t>(flight_state_machine_.state());
    const std::uint16_t detail = static_cast<std::uint16_t>(verdict_word_.load());
    if (!bit::policy::allow_bit_report(state)) {
        reply(radio, IRIS_COMMAND_BIT_REPORT, data6, IRIS_ACK_REFUSED, detail);
        return;
    }
    if (data6[0] != IRIS_BIT_REPORT_SUMMARY && data6[0] >= bit::codec::kDetailPageCount) {
        reply(radio, IRIS_COMMAND_BIT_REPORT, data6, IRIS_ACK_BAD_ARGUMENT, detail);
        return;
    }
    portENTER_CRITICAL(&request_lock_);
    const bool taken = run_pending_.load();
    if (!taken) {
        request_.kind = Request::Report;
        request_.radio = radio;
        std::memcpy(request_.data6, data6, 6);
        run_pending_.store(true);
    }
    portEXIT_CRITICAL(&request_lock_);
    if (taken) reply(radio, IRIS_COMMAND_BIT_REPORT, data6, IRIS_ACK_BUSY, detail);
}

void BitMonitor::handle_fault(const std::uint8_t *data6, const bool radio)
{
    // Same gate as 0x4252: HITL active (USB only, Pad Idle only), canonical USB frame, never radio.
    const bit::policy::Source source = radio ? bit::policy::Source::Radio : bit::policy::Source::Usb;
    if (!bit::policy::allow_bit_fault(source, sensors_.hitl_active())) {
        ESP_LOGW(kLogTag, "BIT fault injection refused (%s)", radio ? "radio" : "HITL not active");
        reply(radio, IRIS_COMMAND_BIT_FAULT, data6, radio ? IRIS_ACK_UNKNOWN : IRIS_ACK_REFUSED, 0);
        return;
    }
    const std::uint8_t id = data6[0];
    if (!bit::policy::fault_id_valid(id) || (id != IRIS_BIT_FAULT_CLEAR_ALL && id >= 16U)) {
        reply(radio, IRIS_COMMAND_BIT_FAULT, data6, IRIS_ACK_BAD_ARGUMENT, 0);
        return;
    }
    if (id == IRIS_BIT_FAULT_CLEAR_ALL) {
        for (auto &f : faults_) f.active.store(false);
    } else {
        const std::uint16_t requested = static_cast<std::uint16_t>((data6[2] << 8U) | data6[3]);
        const std::uint32_t duration = bit::policy::fault_duration_ms(requested);
        faults_[id].until_ms.store(ms_now() + duration);
        fault_heap_blocking_ = data6[4] != 0U;
        faults_[id].active.store(data6[1] != 0U);
        ESP_LOGW(kLogTag, "BIT fault %u %s for %lu ms", static_cast<unsigned>(id), data6[1] ? "ON" : "OFF",
                 static_cast<unsigned long>(duration));
    }
    reply(radio, IRIS_COMMAND_BIT_FAULT, data6, IRIS_ACK_OK, 0);
}

// ---- gate and commanded run --------------------------------------------------------------------

bit::GateInputs BitMonitor::gate_inputs(const std::uint32_t now_ms) const
{
    bit::GateInputs g;
    g.flight_state = static_cast<std::uint8_t>(flight_state_machine_.state());
    g.resumed = resumed_;
    g.launch_budget_zero = BitProbe::budget_zero(flight_state_machine_);
    const bit::StillStats s = still_.latest(now_ms);
    g.still_valid = s.valid;
    g.axial_g = s.axial_g;
    g.accel_mag_g = s.accel_mag_g;
    g.gyro_mean_dps = s.gyro_mean_dps;
    g.baro_rate_mps = baro_.latest(now_ms).rate_mps;
    g.kalman_vz_mps = BitProbe::kalman_vz(sensors_);
    g.run_in_progress = false;
    return g;
}

void BitMonitor::send_detail(const bool over_radio)
{
    std::uint8_t frame[bit::codec::kDetailFrameLength];
    for (std::uint8_t page = 0; page < bit::codec::kDetailPageCount; ++page) {
        if (!bit::codec::encode_detail(engine_, page, frame)) break;
        if (over_radio) {
            (void)data_forwarder_.queue_immediate_radio_packet(frame, sizeof(frame));
        } else {
            (void)data_forwarder_.queue_packet(IRIS_PACKET_ID_BIT_DETAIL, frame + IRIS_PACKET_ID_LENGTH,
                                               IRIS_PACKET_BIT_DETAIL_DATA_LENGTH);
        }
        (void)flash_logger_.log_packet(FlashLogger::kPacketSourceBit, frame, sizeof(frame));
    }
}

void BitMonitor::process_request(const Request &req, const std::uint32_t now_ms)
{
    if (req.kind == Request::Report) {
        const std::uint16_t detail = static_cast<std::uint16_t>(verdict_word_.load());
        reply(req.radio, IRIS_COMMAND_BIT_REPORT, req.data6, IRIS_ACK_OK, detail);
        const bit::Summary s = engine_.summarize(now_ms);
        if (req.data6[0] == IRIS_BIT_REPORT_SUMMARY) {
            std::uint8_t frame[bit::codec::kSummaryFrameLength];
            bit::codec::encode_summary(s, frame);
            if (req.radio) (void)data_forwarder_.queue_immediate_radio_packet(frame, sizeof(frame));
            else (void)data_forwarder_.queue_packet(IRIS_PACKET_ID_BIT_SUMMARY, frame + IRIS_PACKET_ID_LENGTH,
                                                    IRIS_PACKET_BIT_SUMMARY_DATA_LENGTH);
        } else {
            std::uint8_t frame[bit::codec::kDetailFrameLength];
            if (bit::codec::encode_detail(engine_, req.data6[0], frame)) {
                if (req.radio) (void)data_forwarder_.queue_immediate_radio_packet(frame, sizeof(frame));
                else (void)data_forwarder_.queue_packet(IRIS_PACKET_ID_BIT_DETAIL, frame + IRIS_PACKET_ID_LENGTH,
                                                        IRIS_PACKET_BIT_DETAIL_DATA_LENGTH);
            }
        }
        return;
    }
    run_commanded(req, now_ms);
}

// Waits for the logger slice; polls samples meanwhile and checks the tripwire at every step.
bool BitMonitor::wait_active(bit::ActiveResult &result, const std::uint8_t request_kind, const std::uint8_t start_state,
                             bool &aborted)
{
    using namespace bit;
    result = ActiveResult{};
    result.state = ActiveResult::State::Pending;
    std::uint8_t nonce[4];
    const std::uint32_t seed = static_cast<std::uint32_t>(esp_timer_get_time());
    nonce[0] = static_cast<std::uint8_t>(seed); nonce[1] = static_cast<std::uint8_t>(seed >> 8);
    nonce[2] = static_cast<std::uint8_t>(seed >> 16); nonce[3] = static_cast<std::uint8_t>(seed >> 24);
    BitProbe::flash_request(flash_logger_, request_kind, nonce);
    const std::uint32_t t0 = ms_now();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(kPollPeriodMs));
        const std::uint32_t now = ms_now();
        poll_samples(now);
        std::uint16_t value = 0;
        const std::uint8_t st = BitProbe::flash_state(flash_logger_, value);
        if (st != BitProbe::kStatePending) {
            result.state = ActiveResult::State::Done;
            result.pass = st == BitProbe::kStatePass;
            result.error = st == BitProbe::kStateError;
            result.value = value;
            return true;
        }
        const bit::StillStats s = still_.latest(now);
        const bool tripped = active_test_tripped(static_cast<std::uint8_t>(flight_state_machine_.state()),
                                                 s.accel_mag_g, BitProbe::budget_zero(flight_state_machine_), start_state);
        if (tripped || now - t0 > limits::kActiveTestTimeoutMs + 1000U) {
            BitProbe::flash_cancel(flash_logger_);
            result = ActiveResult{};
            aborted = tripped;
            if (!tripped) { result.state = ActiveResult::State::Done; result.error = true; result.value = 0xFFFF; return true; }
            return false;
        }
    }
}

void BitMonitor::run_commanded(const Request &req, const std::uint32_t now_ms)
{
    using namespace bit;
    run_active_.store(true);
    const std::uint8_t start_state = static_cast<std::uint8_t>(flight_state_machine_.state());
    const auto finish_refused = [&](const std::uint16_t detail) {
        run_active_.store(false);
        reply(req.radio, IRIS_COMMAND_BIT_RUN, req.data6, IRIS_ACK_REFUSED, detail);
    };
    if (!req.passive) {
        const bool needs_pad = (req.options & IRIS_BIT_OPTION_IMU_SELF_TEST) != 0U;
        const GateReason reason = active_test_gate(gate_inputs(now_ms), needs_pad);
        if (reason != GateReason::Open) {
            ESP_LOGW(kLogTag, "BIT run refused by the gate (reason %u)", static_cast<unsigned>(reason));
            finish_refused(static_cast<std::uint16_t>(0x8000U | static_cast<std::uint16_t>(reason)));
            return;
        }
        ActiveResult nand{}, e2e{}, heap{};
        bool aborted = false;
        const bool ok1 = wait_active(nand, BitProbe::kReqNandWrite, start_state, aborted);
        const bool ok2 = ok1 && wait_active(e2e, BitProbe::kReqEndToEnd, start_state, aborted);
        if (!ok1 || !ok2) {
            ESP_LOGW(kLogTag, "BIT run aborted: the vehicle moved or the state changed");
            finish_refused(static_cast<std::uint16_t>(0x8000U | static_cast<std::uint16_t>(GateReason::Tripped)));
            return;
        }
        if ((req.options & IRIS_BIT_OPTION_IMAGE_HEAP) != 0U) {
            heap.state = ActiveResult::State::Done;
            heap.pass = heap_caps_check_integrity_all(true);   // locks the allocator for a few ms
            heap.value = heap.pass ? 0 : 1;
        }
        nand_write_ = nand;
        log_e2e_ = e2e;
        heap_structure_ = heap;
        active_ran_ = true;
        // The post-flight readability check (6.7) is only meaningful in Landed; it is reported as
        // SKIPPED here (the re-scan needs the logger's session table: bench work).
    }
    engine_.begin(RunKind::Commanded, now_ms);
    if (active_ran_ && !req.passive) engine_.mark_active_ran();
    // Evaluate until the verdict is final (or the run budget is spent).
    const std::uint32_t t0 = ms_now();
    Summary summary{};
    for (;;) {
        const std::uint32_t now = ms_now();
        poll_samples(now);
        bit::Inputs in;
        collect(in, now);
        engine_.update(in);
        summary = engine_.summarize(now);
        if (summary.verdict != Verdict::Running || now - t0 >= limits::kCommandedRunMaxMs) break;
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    summary = engine_.summarize(ms_now());
    log_failures(summary);
    publish(summary, true, ms_now(), true);
    last_summary_ = summary;
    have_summary_ = true;
    if (!req.radio) send_detail(false);
    else (void)0;   // radio: summary and ack only; the operator pulls detail pages with 0x5452
    run_active_.store(false);
    reply(req.radio, IRIS_COMMAND_BIT_RUN, req.data6, IRIS_ACK_OK, codec::ack_detail(summary));
    // Back to the passive monitor (keeps the slots and the active results).
    engine_.begin(RunKind::Passive, ms_now());
}

// ---- task --------------------------------------------------------------------------------------

void BitMonitor::run()
{
    // After a warm reset in flight nothing starts before kResumedStartDelayMs; the work is passive.
    vTaskDelay(pdMS_TO_TICKS(resumed_ ? bit::limits::kResumedStartDelayMs : 200U));
    startup_snapshot();
    engine_.begin(resumed_ ? bit::RunKind::Passive : bit::RunKind::Startup, ms_now());
    for (;;) {
        const std::uint32_t now = ms_now();
        service_faults(now);
        const bool pad = flight_state_machine_.state() == FlightStateMachine::State::PadIdle;
        if (pad && !in_pad_idle_) { in_pad_idle_ = true; pad_idle_since_ms_ = now; }
        if (!pad) in_pad_idle_ = false;
        if (run_pending_.load()) {
            Request req;
            portENTER_CRITICAL(&request_lock_);
            req = request_;
            portEXIT_CRITICAL(&request_lock_);
            process_request(req, now);
            run_pending_.store(false);
        }
        poll_samples(now);
        const std::uint32_t period = (!resumed_ && now < bit::limits::kFinalDeadlineMs) ? kStartupEvalMs : kMonitorEvalMs;
        if (now - last_eval_ms_ >= period) {
            last_eval_ms_ = now;
            evaluate(bit::RunKind::Passive, now);
        }
        vTaskDelay(pdMS_TO_TICKS(kPollPeriodMs));
    }
}
