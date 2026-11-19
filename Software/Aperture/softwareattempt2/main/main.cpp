#include <array>
#include <cstddef>
#include <cstdint>

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "comms.h"
#include "command_parser.h"
#include "frame_ring.h"
#include "ground_station_status.h"
#include "hardware.h"
#include "lc86_gps.h"
#include "ra01.h"
#include "tx_slot.h"
#include "uplink_pacing.h"

namespace {

constexpr const char *kLogTag = "aperture";

// Reported in the 0xF0 RSSI byte when the receiver cannot hear (radio not
// initialized, or the RX front end was switched off from the PC). Normal RSSI
// is -127..0 dBm, so -128 is free. There is no spare field in the 29-byte frame.
constexpr std::int8_t kRssiReceiverDown = -128;
// A PC "front end OFF" (0x5250) deafens the receiver; it reverts by itself.
constexpr std::uint32_t kFrontEndOffMaxMs = 30000U;
constexpr std::uint32_t kHealthPeriodMs = 1000U;
constexpr std::uint32_t kStatsLogPeriodMs = 1000U;     // only when something changed
constexpr std::uint32_t kStackLogPeriodMs = 60000U;
constexpr unsigned kInitFailuresBeforeRestart = 60U;   // ~10 min of retries

using UplinkQueue = uplink_pacing::Queue<IRIS_PACKET_COMMAND_FRAME_LENGTH, 64>;

Lc86Gps gps;
Ra01 radio;   // static: keeps the object off the main task stack

// ---- USB output: ring + dedicated task, never blocks the radio loop --------
FrameRing<3072> usb_ring;
portMUX_TYPE usb_lock = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t usb_task_handle = nullptr;

void usb_send(const std::uint8_t *data, std::size_t length)
{
    while (length > 0U) {   // frames up to 255 B each; longer input is chunked
        const std::size_t chunk = length > 255U ? 255U : length;
        portENTER_CRITICAL(&usb_lock);
        (void)usb_ring.push(data, chunk);   // full: oldest frame dropped and counted
        portEXIT_CRITICAL(&usb_lock);
        data += chunk;
        length -= chunk;
    }
    if (usb_task_handle != nullptr) xTaskNotifyGive(usb_task_handle);
}

std::uint32_t usb_drops()
{
    portENTER_CRITICAL(&usb_lock);
    const std::uint32_t d = usb_ring.drops();
    portEXIT_CRITICAL(&usb_lock);
    return d;
}

void usb_task(void *)
{
    std::array<std::uint8_t, 255> frame{};
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
        for (;;) {
            std::uint32_t seq = 0;
            portENTER_CRITICAL(&usb_lock);
            const std::size_t length = usb_ring.peek(frame.data(), frame.size(), seq);
            portEXIT_CRITICAL(&usb_lock);
            if (length == 0U) break;
            // The driver accepts a write whole or not at all (one ring item).
            if (usb_serial_jtag_write_bytes(frame.data(), length, pdMS_TO_TICKS(20)) <= 0) {
                break;   // host not draining: keep the frame, retry on next wake
            }
            portENTER_CRITICAL(&usb_lock);
            (void)usb_ring.pop_if(seq);
            portEXIT_CRITICAL(&usb_lock);
        }
    }
}

std::uint32_t now_ms32()
{
    return static_cast<std::uint32_t>(esp_timer_get_time() / 1000LL);
}

// Ack sent by Aperture itself on USB (never over radio) for a command it
// could not pass on: same frame as Witness's ack, detail kAckDetail* so the
// PC can tell it came from Aperture.
constexpr std::uint16_t kAckDetailUplinkSendFailed = 0xA901U;   // radio TX failed kUplinkAttempts times
constexpr std::uint16_t kAckDetailUplinkQueueFull = 0xA902U;
constexpr std::uint8_t kUplinkAttempts = 3U;

void send_local_ack(const std::uint8_t *command, const std::uint8_t result, const std::uint16_t detail)
{
    std::array<std::uint8_t, IRIS_PACKET_COMMAND_ACK_FRAME_LENGTH> ack{};
    ack[0] = IRIS_PACKET_ID_COMMAND_ACK;
    ack[1] = command[7];
    ack[2] = command[8];
    for (std::size_t i = 0; i < 6U; ++i) ack[3U + i] = command[1U + i];
    ack[9] = result;
    ack[10] = static_cast<std::uint8_t>(detail >> 8);
    ack[11] = static_cast<std::uint8_t>(detail & 0xFFU);
    ack[12] = IRIS_PACKET_EOF_VALUE;
    usb_send(ack.data(), ack.size());
}

// 0x5250 ground-radio command, handled locally and acked on USB.
void handle_ground_radio_power(const std::uint8_t *command, std::uint32_t now,
                               std::uint32_t &front_end_off_since_ms)
{
    const std::uint8_t front_end = command[3];
    const bool ok = front_end <= IRIS_RADIO_FRONT_END_OFF &&
                    radio.set_tx_power_dbm(static_cast<std::int8_t>(command[2]));
    if (ok && front_end != IRIS_RADIO_FRONT_END_UNCHANGED) {
        const bool on = front_end == IRIS_RADIO_FRONT_END_ON;
        radio.set_front_end_enabled(on);
        front_end_off_since_ms = on ? 0U : (now | 1U);   // timer for the auto-revert
    }
    const std::int16_t drive = radio.tx_power_dbm();
    const bool fem = radio.front_end_enabled();
    std::array<std::uint8_t, IRIS_PACKET_COMMAND_ACK_FRAME_LENGTH> ack{};
    ack[0] = IRIS_PACKET_ID_COMMAND_ACK;
    ack[1] = command[7];
    ack[2] = command[8];
    for (std::size_t i = 0; i < 6U; ++i) ack[3U + i] = command[1U + i];
    ack[9] = ok ? IRIS_ACK_OK : IRIS_ACK_BAD_ARGUMENT;
    ack[10] = fem ? 1U : 0U;
    ack[11] = static_cast<std::uint8_t>(drive);
    ack[12] = IRIS_PACKET_EOF_VALUE;
    usb_send(ack.data(), ack.size());
    ESP_LOGD(kLogTag, "ground radio TX drive %s: %d dBm, front end %s",
             ok ? "set" : "refused", static_cast<int>(drive), fem ? "on" : "off");
}

}  // namespace

extern "C" void app_main(void)
{
    static constexpr std::size_t kPacketBufferSize =
        IRIS_RADIO_MAX_PACKET_LENGTH;
    static std::array<std::uint8_t, kPacketBufferSize> packet{};

    // Flight image logs only warnings and errors (sdkconfig: LOG_DEFAULT_LEVEL
    // WARN) because the console shares the binary USB stream. This tag keeps
    // its INFO lines (one per minute, plus a line when a counter changes).
    esp_log_level_set(kLogTag, ESP_LOG_INFO);

    usb_serial_jtag_driver_config_t usb_config{};
    usb_config.tx_buffer_size = 512U;
    // ESP-IDF requires the RX buffer to be larger than 64 bytes. 512 so a PC
    // burst (GPS-assist upload) is not dropped between 10 ms polls.
    usb_config.rx_buffer_size = 512U;
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));
    (void)xTaskCreate(usb_task, "usb_tx", 3072, nullptr, 2, &usb_task_handle);

    // A hung receiver must reboot itself: the main loop feeds the task
    // watchdog (sdkconfig: TWDT 5 s, panic = reboot). No path below blocks
    // that long (BUSY timeout 100 ms, then 5 ms per command while stuck).
    (void)esp_task_wdt_add(nullptr);

    // The GPS runs on its own task; a failure here leaves the bridge running
    // and the 0xF0 GPS fields at their no-fix values.
    const esp_err_t gps_result = gps.start();
    if (gps_result != ESP_OK) {
        ESP_LOGE(kLogTag, "GPS start failed: %s (0x%x)",
                 esp_err_to_name(gps_result),
                 static_cast<unsigned>(gps_result));
    }

    static CommandAssembler assembler;
    static std::array<std::uint8_t, IRIS_PACKET_GROUND_STATION_STATUS_FRAME_LENGTH> status_packet{};
    TickType_t next_status = xTaskGetTickCount();
    // Commands from USB waiting for the uplink window (tx_slot.h); static, not stack.
    static UplinkQueue uplink;
    static uplink_pacing::Pacer pacer;
    bool uplink_sent_before = false;
    std::uint64_t last_uplink_ms = 0U;
    bool slots_active = false;
    const std::uint32_t kCommandAirtimeMs = tx_slot::lora_airtime_ms(
        IRIS_PACKET_COMMAND_FRAME_LENGTH, IRIS_RADIO_SPREADING_FACTOR, IRIS_RADIO_BANDWIDTH_HZ,
        IRIS_RADIO_CODING_RATE, IRIS_RADIO_PREAMBLE_LENGTH);

    bool radio_ready = false;
    unsigned init_failures = 0U;
    std::uint32_t next_init_ms = 0U;
    std::uint32_t next_health_ms = now_ms32() + kHealthPeriodMs;
    unsigned receive_errors = 0U;
    std::uint32_t front_end_off_since_ms = 0U;
    std::uint32_t uplink_dups = 0U, uplink_full = 0U, uplink_failed = 0U;
    std::uint8_t uplink_attempts = 0U;   // failed sends of the command at the queue front
    std::uint32_t radio_failures_total = 0U;
    std::uint32_t last_logged_sum = 0U;
    std::uint32_t next_stats_log_ms = now_ms32() + kStatsLogPeriodMs;
    std::uint32_t next_stack_log_ms = now_ms32() + kStackLogPeriodMs;

    for (;;) {
        (void)esp_task_wdt_reset();
        const std::uint32_t loop_ms = now_ms32();

        // ---- Radio init / retry with backoff (never permanent) --------------
        if (!radio_ready && static_cast<std::int32_t>(loop_ms - next_init_ms) >= 0) {
            const esp_err_t init = radio.initialize();
            if (init == ESP_OK) {
                radio_ready = true;
                init_failures = 0U;
                next_health_ms = loop_ms + kHealthPeriodMs;
                ESP_LOGI(kLogTag, "Radio initialized; transmit power is %d dBm",
                         static_cast<int>(radio.tx_power_dbm()));
            } else {
                ++init_failures;
                ++radio_failures_total;
                // Backoff 1,2,4,10,10... s. Status frames keep flowing with
                // RSSI -128 so the PC shows the receiver as down.
                const std::uint32_t backoff_s = init_failures >= 4U ? 10U : (1U << (init_failures - 1U));
                next_init_ms = loop_ms + backoff_s * 1000U;
                ESP_LOGW(kLogTag, "Radio init failed (#%u): %s; retry in %us",
                         init_failures, esp_err_to_name(init), static_cast<unsigned>(backoff_s));
                if (init_failures >= kInitFailuresBeforeRestart) {
                    ESP_LOGE(kLogTag, "Radio still failing; restarting");
                    esp_restart();
                }
            }
        }

        // ---- Status frame once per second -----------------------------------
        if (xTaskGetTickCount() >= next_status) {
            const bool deaf = !radio_ready || !radio.front_end_enabled();
            build_ground_station_status(status_packet.data(),
                                       deaf ? kRssiReceiverDown : radio.last_packet_rssi_dbm(),
                                       now_ms32(), gps.state());
            usb_send(status_packet.data(), status_packet.size());
            next_status += pdMS_TO_TICKS(1000);
            if (xTaskGetTickCount() > next_status + pdMS_TO_TICKS(5000)) next_status = xTaskGetTickCount();
        }

        // ---- PC commands -----------------------------------------------------
        for (int round = 0; round < 8; ++round) {
            std::uint8_t usb_input[64];
            const int usb_count = usb_serial_jtag_read_bytes(usb_input, sizeof(usb_input), 0);
            if (usb_count <= 0) break;
            // The protocol requires USB input to be echoed byte-for-byte
            // (one non-blocking ring entry per chunk, not one per byte).
            usb_send(usb_input, static_cast<std::size_t>(usb_count));
            for (int index = 0; index < usb_count; ++index) {
                // Frame assembly with resync on the 0x05 marker: one lost byte
                // costs one frame, not two (command_parser.h).
                if (!assembler.feed(usb_input[index])) continue;
                const std::uint8_t *command = assembler.frame();
                const std::uint16_t opcode = static_cast<std::uint16_t>((command[7] << 8U) | command[8]);
                if (opcode == IRIS_COMMAND_RADIO_POWER &&
                    command[1] == IRIS_RADIO_POWER_TARGET_GROUND) {
                    // Ground-station radio power: applied here, acked on
                    // USB (0x06, detail = dBm now in use), never transmitted.
                    handle_ground_radio_power(command, now_ms32(), front_end_off_since_ms);
                    continue;
                }
                // Queued; sent in Witness's listening window (tx_slot.h),
                // paced by uplink_pacing.h. Exact duplicates are dropped.
                switch (uplink.push(command)) {
                    case UplinkQueue::Push::Duplicate:
                        ++uplink_dups;
                        break;
                    case UplinkQueue::Push::Full:
                        ++uplink_full;
                        send_local_ack(command, IRIS_ACK_BUSY, kAckDetailUplinkQueueFull);
                        break;
                    default:
                        break;
                }
            }
        }

        // RX front end OFF is a bench function: never leave the receiver deaf.
        if (front_end_off_since_ms != 0U && !radio.front_end_enabled() &&
            now_ms32() - front_end_off_since_ms >= kFrontEndOffMaxMs) {
            radio.set_front_end_enabled(true);
            front_end_off_since_ms = 0U;
            ESP_LOGW(kLogTag, "RX front end was OFF for %u s: switched back ON",
                     static_cast<unsigned>(kFrontEndOffMaxMs / 1000U));
        }
        if (front_end_off_since_ms != 0U && radio.front_end_enabled()) front_end_off_since_ms = 0U;

        if (radio_ready) {
            // ---- Uplink ------------------------------------------------------
            if (!uplink.empty()) {
                // Read the GPS snapshot first, then the clock, so a PPS/GGA
                // arriving in between cannot give negative/wrapped ages.
                const std::int64_t pps_us = gps.last_pps_us();
                const Lc86Nmea::State gps_state = gps.state();
                const std::int64_t now_us = esp_timer_get_time();
                const std::uint64_t now_ms = static_cast<std::uint64_t>(now_us / 1000LL);
                const bool pps_known =
                    pps_us > 0 && now_us >= pps_us &&
                    now_us - pps_us < static_cast<std::int64_t>(tx_slot::kTimeMaxAgeMs) * 1000LL &&
                    Lc86Nmea::gga_fresh(gps_state, static_cast<std::uint32_t>(now_ms)) &&
                    gps_state.fix_quality > 0U;
                if (pps_known != slots_active) {
                    slots_active = pps_known;
                    ESP_LOGD(kLogTag, "uplink time slots %s", pps_known ? "ON (PPS)" : "off (no GPS fix/PPS)");
                }
                const std::uint64_t since_pps_ms =
                    pps_known ? static_cast<std::uint64_t>((now_us - pps_us) / 1000LL) : 0U;
                if (pacer.allow(now_ms) &&
                    tx_slot::aperture_may_send(now_ms, uplink_sent_before, last_uplink_ms, pps_known,
                                               since_pps_ms, kCommandAirtimeMs)) {
                    const esp_err_t result = radio.transmit(uplink.front(), IRIS_PACKET_COMMAND_FRAME_LENGTH);
                    if (result != ESP_OK) {
                        ++uplink_failed;
                        ++uplink_attempts;
                        ESP_LOGW(kLogTag, "Command send failed (attempt %u of %u): %s (0x%x)",
                                 static_cast<unsigned>(uplink_attempts), static_cast<unsigned>(kUplinkAttempts),
                                 esp_err_to_name(result), static_cast<unsigned>(result));
                    }
                    // A failed send stays queued for the next allowed slot;
                    // after kUplinkAttempts it is dropped and the PC told.
                    if (result == ESP_OK || uplink_attempts >= kUplinkAttempts) {
                        if (result != ESP_OK) {
                            send_local_ack(uplink.front(), IRIS_ACK_BUSY, kAckDetailUplinkSendFailed);
                        }
                        uplink.pop();
                        uplink_attempts = 0U;
                    }
                    uplink_sent_before = true;
                    last_uplink_ms = static_cast<std::uint64_t>(esp_timer_get_time() / 1000LL);
                    pacer.on_sent(last_uplink_ms);
                }
            }

            // ---- Receive -----------------------------------------------------
            std::size_t packet_length = 0;
            const esp_err_t result = radio.receive(packet.data(), packet.size(), packet_length);
            if (result == ESP_OK && packet_length > 0U) {
                usb_send(packet.data(), packet_length);   // non-blocking
                next_health_ms = now_ms32() + kHealthPeriodMs;
                receive_errors = 0U;
            } else if (result == ESP_ERR_NOT_FOUND || result == ESP_ERR_INVALID_CRC ||
                       result == ESP_ERR_INVALID_SIZE || result == ESP_OK) {
                receive_errors = 0U;   // normal: nothing yet / bad packet (counted in radio stats)
            } else {
                ++receive_errors;      // SPI/BUSY failure: escalate to the health check
            }

            // ---- Health check: ~1 s without a packet, or on any fault --------
            const std::uint32_t t = now_ms32();
            if (radio.rx_fault() || receive_errors >= 3U ||
                static_cast<std::int32_t>(t - next_health_ms) >= 0) {
                next_health_ms = t + kHealthPeriodMs;
                const Ra01::Health health = radio.health_check();
                if (health == Ra01::Health::Failed && !radio.ready()) {
                    radio_ready = false;   // re-init failed: back to the retry loop
                    next_init_ms = t + 1000U;
                    ++radio_failures_total;
                } else if (health != Ra01::Health::Ok) {
                    ESP_LOGW(kLogTag, "Radio recovery: %s",
                             health == Ra01::Health::Rearmed ? "SetRx re-issued"
                             : health == Ra01::Health::Reinitialized ? "re-initialized (RESET)"
                                                                     : "check failed");
                }
                receive_errors = 0U;
            }
        }

        // ---- Diagnostics: counters (no spare field in 0xF0) -------------------
        const std::uint32_t t_log = now_ms32();
        if (static_cast<std::int32_t>(t_log - next_stats_log_ms) >= 0) {
            next_stats_log_ms = t_log + kStatsLogPeriodMs;
            const Ra01::Stats s = radio.stats();
            const std::uint32_t drops = usb_drops();
            // CRC errors are normal on a noisy channel: counted, shown in the
            // minute line only. These indicate loss or recovery:
            const std::uint32_t sum = drops + s.rx_overruns + s.rearms + s.reinits + s.busy_timeouts +
                                      s.device_errors + uplink_full + uplink_failed + radio_failures_total;
            if (sum != last_logged_sum) {
                last_logged_sum = sum;
                ESP_LOGW(kLogTag,
                         "counters: usb_drop=%u rx_overrun=%u rearm=%u reinit=%u busy_to=%u dev_err=%u "
                         "up_full=%u up_fail=%u radio_fail=%u",
                         static_cast<unsigned>(drops), static_cast<unsigned>(s.rx_overruns),
                         static_cast<unsigned>(s.rearms), static_cast<unsigned>(s.reinits),
                         static_cast<unsigned>(s.busy_timeouts), static_cast<unsigned>(s.device_errors),
                         static_cast<unsigned>(uplink_full), static_cast<unsigned>(uplink_failed),
                         static_cast<unsigned>(radio_failures_total));
            }
        }
        if (static_cast<std::int32_t>(t_log - next_stack_log_ms) >= 0) {
            next_stack_log_ms = t_log + kStackLogPeriodMs;
            const Ra01::Stats s = radio.stats();
            ESP_LOGI(kLogTag,
                     "stack_free_min main=%uB usb=%uB; crc=%u dup_cmd=%u resync=%u usb_drop=%u",
                     static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)),
                     static_cast<unsigned>(usb_task_handle ? uxTaskGetStackHighWaterMark(usb_task_handle) : 0U),
                     static_cast<unsigned>(s.crc_errors), static_cast<unsigned>(uplink_dups),
                     static_cast<unsigned>(assembler.resyncs()), static_cast<unsigned>(usb_drops()));
        }

        // Sleep until the radio's DIO1 line fires (RxDone/TxDone/CrcErr) or one
        // tick (10 ms) passes, whichever is first.
        if (radio_ready) radio.wait_for_event(1);
        else vTaskDelay(1);
    }
}
