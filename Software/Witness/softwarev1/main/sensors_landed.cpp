// Landed low-power mode of the sensor task (see landed_logic.h for the
// timings and FIRMWARE_HANDOFF.md "Landed low-power mode" for the design).
//
// While the flight state is Landed the sensor task stops its 500 Hz loop and
// runs run_landed() instead:
//   - the IMU is powered down and the barometer is no longer polled;
//   - the radio stays in continuous receive (SX1262 SetRx), so every radio
//     command still arrives; DIO1 (RxDone/TxDone) interrupts the task and
//     wakes the MCU from light sleep;
//   - the only packet sent on its own is the 0x09 landed position packet,
//     once per minute at the top of the GPS minute (plus command acks, and
//     one reply per ping);
//   - the GPS sleeps and is woken every 10 minutes to refresh the position;
//   - the CAN link is paused, the housekeeping tasks go quiet, and ESP-IDF
//     automatic light sleep (esp_pm) lets the idle task sleep the MCU whenever
//     every task is blocked and no PM lock is held. A USB host (USB Serial/JTAG
//     connected) keeps the MCU awake.
// Any failure to sleep or to wake the GPS only costs power or position age;
// the position packet schedule never depends on either.

#include "sensors.h"

#include <algorithm>
#include <cstdint>
#include <cstring>

#include "can_link.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "flight_state_machine.h"
#include "hardware_pins.h"
#include "landed_logic.h"
#include "landed_packet.h"
#include "landed_power.h"
#include "spi_data_forwarder.h"
#include "uart_driver.h"
#include "witness_status.h"

namespace {

constexpr char kTag[] = "landed";

// Awake clock while Landed (light sleep is automatic when idle). 80 MHz is
// ample for SPI radio traffic and the 115200 baud GPS UART.
constexpr int kLandedCpuMhz = 80;
constexpr int kFlightCpuMhz = 240;

// ESP-IDF isolates every GPIO pad (output off, no pulls) during light sleep
// (CONFIG_ESP_SLEEP_GPIO_RESET_WORKAROUND, on by default for this chip). Pads
// that must keep their level or function while the MCU sleeps: chip selects,
// radio reset/RF enable (the front end must stay on to receive), the SPI buses,
// NAND hold/WP lines, the GPS UART TX (an edge on it would wake a u-blox), the
// buzzer and LEDs (held low), the CAN transceiver TX and the radio's inputs.
constexpr gpio_num_t kSleepKeepPins[] = {
    HW_PIN_RAD_DIO3, HW_PIN_RAD_DIO2, HW_PIN_RAD_DIO1, HW_PIN_RAD_RESET, HW_PIN_RAD_CS,
    HW_PIN_RAD_BUSY, HW_PIN_RAD_RF_ENABLE,
    HW_PIN_SPI2_IO3, HW_PIN_FLASH_CS, HW_PIN_SPI2_IO0, HW_PIN_SPI2_CLOCK, HW_PIN_SPI2_IO1,
    HW_PIN_SPI2_IO2,
    HW_PIN_SPI3_CLOCK, HW_PIN_SPI3_COPI, HW_PIN_SPI3_CIPO,
    HW_PIN_MS56_CS, HW_PIN_LSM_CS,
    HW_PIN_LED_RED, HW_PIN_LED_GREEN, HW_PIN_LED_BLUE, HW_PIN_BUZZER,
    HW_PIN_CAN_TX, HW_PIN_MCU_UART_TX,
};

void keep_pins_in_sleep(bool keep)
{
    for (const gpio_num_t pin : kSleepKeepPins) {
        (void)(keep ? gpio_sleep_sel_dis(pin) : gpio_sleep_sel_en(pin));
    }
}

void dio1_isr(void *context)
{
    // Level interrupt (it is also the light-sleep wake level): mask it until
    // the task has serviced the radio and re-arms it.
    gpio_intr_disable(HW_PIN_RAD_DIO1);
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(static_cast<TaskHandle_t>(context), &woken);
    if (woken == pdTRUE) portYIELD_FROM_ISR();
}

esp_pm_lock_handle_t create_lock(const char *name)
{
    esp_pm_lock_handle_t handle = nullptr;
    if (esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, name, &handle) != ESP_OK) return nullptr;
    return handle;
}

// A NO_LIGHT_SLEEP lock that is acquired and released idempotently.
struct SleepLock {
    esp_pm_lock_handle_t handle = nullptr;
    bool held = false;
    void init(const char *name) { handle = create_lock(name); }
    void set(bool want)
    {
        if (handle == nullptr || want == held) return;
        if ((want ? esp_pm_lock_acquire(handle) : esp_pm_lock_release(handle)) == ESP_OK) held = want;
    }
    void destroy()
    {
        set(false);
        if (handle != nullptr) esp_pm_lock_delete(handle);
        handle = nullptr;
    }
};

}  // namespace

std::uint64_t Sensors::monotonic_ms()
{
    return static_cast<std::uint64_t>(esp_timer_get_time()) / 1000ULL;
}

void Sensors::note_gps_fix(const NmeaGps::Fix &fix)
{
    if (!fix.valid || fix.sequence == last_seen_fix_sequence_) return;
    last_seen_fix_sequence_ = fix.sequence;
    last_good_fix_ = fix;
    // GGA receive time is the uptime in ms (32-bit); put it on the 64-bit clock.
    const std::uint64_t now = monotonic_ms();
    last_good_fix_local_ms_ = now - static_cast<std::uint32_t>(static_cast<std::uint32_t>(now) - fix.received_ms);
    have_good_fix_ = true;
    if (fix.utc_valid) {
        utc_clock_.observe(last_good_fix_local_ms_, fix.utc_ms_of_day);
        last_utc_local_ms_ = last_good_fix_local_ms_;   // radio time slots (tx_slot.h)
        have_utc_fix_ = true;
    }
}

bool Sensors::request_landed_ping()
{
    if (!landed_mode_active_.load()) return false;
    bool allowed = false;
    portENTER_CRITICAL(&landed_ping_lock_);
    allowed = landed_ping_limiter_.allow(monotonic_ms());
    portEXIT_CRITICAL(&landed_ping_lock_);
    if (!allowed) return false;
    landed_ping_requested_.store(true);
    if (task_handle_ != nullptr) xTaskNotifyGive(task_handle_);
    return true;
}

void Sensors::landed_power_down_sensors()
{
    // LSM6: both accelerometers and the gyro to power-down (ODR 0), FIFO off.
    // The barometer is simply not polled (it idles between conversions).
    (void)write_register(kRegCtrl1, 0x00);
    (void)write_register(kRegCtrl2, 0x00);
    (void)write_register(kRegHighGCtrl1,
                         static_cast<std::uint8_t>(kHighGRegisterOutputEnable | kHighGAccelFullScale));
    (void)write_register(kRegFifoCtrl4, kFifoBypass);
}

void Sensors::landed_restore_sensors()
{
    esp_err_t result = ESP_FAIL;
    for (int attempt = 0; attempt < 3 && result != ESP_OK; ++attempt) {
        result = initialize_lsm6dsv320x();
    }
    if (result != ESP_OK) {
        ESP_LOGE(kTag, "IMU re-initialization after Landed failed: %s", esp_err_to_name(result));
    }
    fifo_slot_open_ = false;
    imu_batch_count_ = 0;
    imu_clock_valid_ = false;
    position_last_predict_us_ = 0;
}

void Sensors::landed_send_usb_state()
{
    // A USB host (bench, export tools) still sees the Landed state at 1 Hz.
    (void)queue_state_packet();
}

void Sensors::run_landed()
{
    const std::uint64_t entry_ms = monotonic_ms();
    const TaskHandle_t self = xTaskGetCurrentTaskHandle();
    ESP_LOGW(kTag, "Landed: low-power mode (position packet every %lu s, GPS refresh every %lu s)",
             static_cast<unsigned long>(landed::kLandedTxPeriodMs / 1000U),
             static_cast<unsigned long>(landed::kGpsWakeIntervalMs / 1000U));
    landed_mode_active_.store(true);

    // ---- Landed location: the last good fix ------------------------------
    landed::Position pos{};
    std::uint64_t pos_local_ms = 0;
    const auto local_ms_of = [](const NmeaGps::Fix &fix) {
        const std::uint64_t now = monotonic_ms();
        return now - static_cast<std::uint32_t>(static_cast<std::uint32_t>(now) - fix.received_ms);
    };
    const auto adopt = [&](const NmeaGps::Fix &fix, std::uint64_t local_ms) {
        pos.valid = true;
        pos.latitude_deg = fix.latitude_degrees;
        pos.longitude_deg = fix.longitude_degrees;
        pos.altitude_m = fix.altitude_meters;
        pos.satellites = fix.satellites;
        pos_local_ms = local_ms;
        if (fix.utc_valid) utc_clock_.observe(local_ms, fix.utc_ms_of_day);
    };
    {
        const NmeaGps::Fix now_fix = gps_uart_.latest_gps_fix();
        if (now_fix.valid) {
            adopt(now_fix, local_ms_of(now_fix));
            pos.stale = false;
        } else if (have_good_fix_) {
            adopt(last_good_fix_, last_good_fix_local_ms_);
            pos.stale = true;   // refreshed (or not) by the GPS cycle that starts now
        } else {
            pos.stale = true;
        }
    }

    landed::TxSchedule schedule;
    schedule.begin(entry_ms);
    if (pos.valid) schedule.on_fix(pos_local_ms);
    landed::GpsDuty duty;
    duty.begin(entry_ms);
    std::uint32_t gps_active_since_ms32 = static_cast<std::uint32_t>(entry_ms) - 5000U;

    // ---- Power down what Landed does not need -----------------------------
    landed_power_down_sensors();
    if (can_link_ != nullptr) can_link_->set_paused(true);

    keep_pins_in_sleep(true);
    SleepLock gps_lock;
    SleepLock usb_lock;
    gps_lock.init("landed_gps");
    usb_lock.init("landed_usb");
    gps_lock.set(true);   // the receiver is running: keep the UART awake

    bool dio1_armed = false;
    if (radio_ready_) {
        (void)gpio_install_isr_service(0);   // ESP_ERR_INVALID_STATE if already installed
        (void)gpio_set_intr_type(HW_PIN_RAD_DIO1, GPIO_INTR_HIGH_LEVEL);
        if (gpio_isr_handler_add(HW_PIN_RAD_DIO1, dio1_isr, self) == ESP_OK &&
            gpio_wakeup_enable(HW_PIN_RAD_DIO1, GPIO_INTR_HIGH_LEVEL) == ESP_OK &&
            esp_sleep_enable_gpio_wakeup() == ESP_OK) {
            dio1_armed = true;
        } else {
            ESP_LOGW(kTag, "DIO1 wake-up could not be armed; the MCU will not light-sleep");
        }
    }
    esp_pm_config_t pm{};
    pm.max_freq_mhz = kLandedCpuMhz;
    pm.min_freq_mhz = kLandedCpuMhz;
    pm.light_sleep_enable = dio1_armed || !radio_ready_;
    const esp_err_t pm_result = esp_pm_configure(&pm);
    if (pm_result != ESP_OK) {
        ESP_LOGW(kTag, "automatic light sleep unavailable (%s); running awake at reduced rate",
                 esp_err_to_name(pm_result));
    }

    // ---- Radio servicing (all on this task, which owns SPI3) --------------
    bool radio_activity = false;
    const auto transmit_position = [&](std::uint64_t now_ms, bool ping) {
        (void)read_battery_voltage();
        std::uint8_t status[2]{};
        witness_status_.write(status);
        landed::Position out = pos;
        if (out.valid) {
            const std::uint64_t age = (now_ms - pos_local_ms) / 1000ULL;
            out.fix_age_s = age > 0xFFFFFFFEULL ? 0xFFFFFFFEUL : static_cast<std::uint32_t>(age);
        }
        std::uint8_t frame[IRIS_PACKET_LANDED_POSITION_FRAME_LENGTH]{};
        landed::encode_position(frame, status, static_cast<std::uint32_t>(now_ms), out,
                                encode_battery_voltage(working_sample_.battery_voltage_mv), ping,
                                !ping && utc_clock_.valid());
        esp_err_t result = ESP_ERR_INVALID_STATE;
        if (radio_ready_ && radio_transmission_enabled_.load()) {
            result = radio_.start_transmit(frame, sizeof(frame));
            ++radio_tx_started_;
            if (result != ESP_OK) {
                ++radio_tx_failed_;
                ESP_LOGW(kTag, "landed position TX failed: %s", esp_err_to_name(result));
            }
        }
        if (usb_serial_jtag_is_connected()) {
            (void)data_forwarder_.queue_packet(IRIS_PACKET_ID_LANDED_POSITION,
                                               frame + IRIS_PACKET_ID_LENGTH,
                                               IRIS_PACKET_LANDED_POSITION_DATA_LENGTH);
        }
        ESP_LOGI(kTag, "position packet%s: %s, age %lu s, %u sats, %s", ping ? " (ping reply)" : "",
                 pos.valid ? "fix" : "no fix", static_cast<unsigned long>(out.fix_age_s),
                 static_cast<unsigned>(pos.satellites), pos.stale ? "stale" : "fresh");
        return result;
    };

    const auto service_radio = [&](std::uint64_t now_ms) {
        if (!radio_ready_) return;
        if (service_radio_transmit_completion()) {   // a frame is on the air
            radio_activity = true;
            return;
        }
        if (gpio_get_level(HW_PIN_RAD_DIO1) != 0) {
            poll_radio_commands();
            radio_activity = true;
        }
        if (!radio_transmission_enabled_.load()) return;
        SpiDataForwarder::RadioPacket packet{};
        if (data_forwarder_.receive_immediate_radio_packet(packet)) {
            // Command acks and replies (and USB relays while a host is attached).
            const esp_err_t result = radio_.start_transmit(packet.data, packet.length);
            ++radio_tx_started_;
            if (result != ESP_OK) ++radio_tx_failed_;
            radio_activity = true;
            return;
        }
        if (landed_ping_requested_.exchange(false)) {
            (void)transmit_position(now_ms, true);
            radio_activity = true;
            return;
        }
        if (schedule.due(utc_clock_, now_ms)) {
            (void)transmit_position(now_ms, false);
            schedule.on_sent(now_ms);   // attempted: the next one is a minute away
            radio_activity = true;
        }
    };

    // ---- Main loop ---------------------------------------------------------
    std::uint64_t linger_until_ms = entry_ms + landed::kRadioLingerMs;
    std::uint64_t last_usb_state_ms = 0;
    constexpr std::uint64_t kUsbGoneMs = 5000;
    std::uint64_t usb_seen_ms = usb_serial_jtag_is_connected() ? entry_ms : 0;
    std::uint32_t dio1_stuck = 0;
    for (;;) {
        const std::uint64_t now = monotonic_ms();
        if (flight_state_machine_.state() != FlightStateMachine::State::Landed || hitl_active()) break;

        // USB host present: debounced. A single false "connected" reading
        // used to release the lock at once; the chip then light-slept with the
        // host attached, the USB PHY stopped answering, the host's
        // re-enumeration attempts (error -71) hit a sleeping chip, and USB
        // never came back (bench 2026-11-15, 9 min into a Landed soak). Now
        // USB counts as gone only after 5 s of continuous "not connected"
        // (polled every 100 ms while present). CONFIG_USJ_NO_AUTO_LS_ON_CONNECTION
        // (sdkconfig.flight) adds the driver's own SOF-based lock.
        if (usb_serial_jtag_is_connected()) usb_seen_ms = now;
        const bool usb = usb_seen_ms != 0U && now - usb_seen_ms < kUsbGoneMs;
        landed_power::set_low_power(!usb);
        usb_lock.set(usb);

        // GPS duty cycle: a fix refreshes the saved location; a timeout keeps
        // the old one and marks it stale. Either way the receiver then sleeps.
        const NmeaGps::Fix fix = gps_uart_.latest_gps_fix();
        const bool good_fix = fix.valid && fix.satellites >= landed::kGpsMinSatellites &&
                              static_cast<std::int32_t>(fix.received_ms - gps_active_since_ms32) >= 0;
        bool fix_ok = false;
        switch (duty.poll(now, good_fix, fix_ok)) {
        case landed::GpsDuty::Action::Sleep:
            if (fix_ok) {
                const std::uint64_t local = local_ms_of(fix);
                adopt(fix, local);
                pos.stale = false;
                schedule.on_fix(local);
                ESP_LOGI(kTag, "landed location refreshed: %.5f %.5f, %u satellites",
                         fix.latitude_degrees, fix.longitude_degrees,
                         static_cast<unsigned>(fix.satellites));
            } else {
                pos.stale = true;
                ESP_LOGW(kTag, "no GPS fix within %lu s; keeping the previous landed location (stale)",
                         static_cast<unsigned long>(landed::kGpsFixTimeoutMs / 1000U));
            }
            gps_uart_.request_gps_sleep();
            gps_lock.set(false);
            break;
        case landed::GpsDuty::Action::Wake:
            gps_active_since_ms32 = static_cast<std::uint32_t>(now);
            gps_uart_.request_gps_wake();
            gps_lock.set(true);
            ESP_LOGI(kTag, "GPS wake (refresh cycle, timeout %lu s)",
                     static_cast<unsigned long>(landed::kGpsFixTimeoutMs / 1000U));
            break;
        case landed::GpsDuty::Action::None:
            break;
        }
        witness_status_.set_state_flag(IRIS_WITNESS_STATUS_GPS_LOCKED_MASK, pos.valid && !pos.stale);

        radio_activity = false;
        service_radio(now);
        if (radio_activity) linger_until_ms = now + landed::kRadioLingerMs;

        if (usb && now - last_usb_state_ms >= 1000U) {
            last_usb_state_ms = now;
            landed_send_usb_state();
        }

        // ---- Wait for the next event --------------------------------------
        const bool tx_possible = radio_ready_ && radio_transmission_enabled_.load();   // off during USB export
        const std::uint64_t tx_ms = tx_possible ? schedule.next_tx_ms(utc_clock_, now) : UINT64_MAX;
        const std::uint64_t event_ms = std::min(tx_ms, duty.next_event_ms());
        std::uint32_t wait_ms = landed::sleep_ms(now, event_ms, landed::kTxLeadMs, 5);
        if (now < linger_until_ms || (radio_ready_ && radio_.transmit_in_progress())) {
            wait_ms = std::min<std::uint32_t>(wait_ms, 10);   // a command may be in flight
        }
        if (duty.phase() == landed::GpsDuty::Phase::Active) wait_ms = std::min<std::uint32_t>(wait_ms, 200);
        if (usb) wait_ms = std::min<std::uint32_t>(wait_ms, 100);

        const bool dio1_high = radio_ready_ && gpio_get_level(HW_PIN_RAD_DIO1) != 0;
        dio1_stuck = dio1_high ? dio1_stuck + 1U : 0U;
        if (dio1_high && dio1_stuck > 3U) wait_ms = std::min<std::uint32_t>(wait_ms, 50);  // never spin
        if (dio1_armed && !(dio1_high && dio1_stuck > 3U)) (void)gpio_intr_enable(HW_PIN_RAD_DIO1);
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait_ms));
    }

    // ---- Leave Landed: restore normal radio, GPS, CAN and CPU behaviour ------
    ESP_LOGW(kTag, "leaving Landed: restoring sensors, GPS, CAN and CPU");
    if (dio1_armed) {
        gpio_intr_disable(HW_PIN_RAD_DIO1);
        (void)gpio_isr_handler_remove(HW_PIN_RAD_DIO1);
        (void)gpio_wakeup_disable(HW_PIN_RAD_DIO1);
        (void)gpio_set_intr_type(HW_PIN_RAD_DIO1, GPIO_INTR_DISABLE);
    }
    keep_pins_in_sleep(false);
    pm.max_freq_mhz = kFlightCpuMhz;
    pm.min_freq_mhz = kFlightCpuMhz;
    pm.light_sleep_enable = false;
    (void)esp_pm_configure(&pm);
    landed_power::set_low_power(false);
    gps_lock.destroy();
    usb_lock.destroy();
    if (can_link_ != nullptr) can_link_->set_paused(false);
    gps_uart_.request_gps_wake();   // no-op unless it is asleep
    landed_restore_sensors();
    witness_status_.set_state_flag(IRIS_WITNESS_STATUS_GPS_LOCKED_MASK, false);
    landed_mode_active_.store(false);
}
