// MAINTENANCE image: Wi-Fi OTA, log download (Wi-Fi HTTP and USB mass
// storage), LoRa command listening and the 0x08 status beacon. No sensors, no
// flight logic. Entered only from the flight image (boot_manager.h) and left
// by restarting; the boot slot is pointed back at flight first thing.
#include <algorithm>
#include <cstring>

#include "boot_manager.h"
#include "comms.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "flash_logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "maint_app.h"
#include "maint_radio.h"
#include "maint_usb.h"
#include "maintenance_mode.h"
#include "nvs_flash.h"
#include "wifi_ota.h"
#include "wifi_ota_logic.h"
#include "witness_status.h"

namespace {
constexpr char kLogTag[] = "maint";
constexpr std::uint32_t kUsbExportMaxMinutes = 120;
constexpr std::uint32_t kStoreMountTimeoutMs = 60000;
constexpr std::uint32_t kVerifyTimeoutMs = 30000;
constexpr std::uint32_t kRadioAckGuardMs = 700;

maint::Shared g_shared;
WifiOta *g_wifi = nullptr;
}  // namespace

namespace maint {

Shared &shared()
{
    return g_shared;
}

void set_flag(const std::uint8_t mask, const bool on)
{
    if (on) {
        g_shared.flags.fetch_or(mask);
    } else {
        g_shared.flags.fetch_and(static_cast<std::uint8_t>(~mask));
    }
}

void leave(const std::uint32_t delay_ms, const char *reason)
{
    ESP_LOGW(kLogTag, "leaving maintenance (%s): restarting in %u ms", reason, static_cast<unsigned>(delay_ms));
    if (!maintenance_mode::restart_after_ms(delay_ms)) esp_restart();
}

void wifi_keep_alive(const std::uint32_t minutes)
{
    if (g_wifi != nullptr) g_wifi->keep_alive(minutes);
}

void send_ack(const bool radio, const std::uint16_t command, const std::uint8_t *data6, const std::uint8_t result,
              const std::uint16_t detail)
{
    std::uint8_t frame[IRIS_PACKET_COMMAND_ACK_FRAME_LENGTH]{};
    frame[0] = IRIS_PACKET_ID_COMMAND_ACK;
    std::uint8_t *payload = frame + IRIS_PACKET_ID_LENGTH;
    payload[0] = static_cast<std::uint8_t>(command >> 8U);
    payload[1] = static_cast<std::uint8_t>(command);
    std::memcpy(payload + 2, data6, 6);
    payload[8] = result;
    payload[9] = static_cast<std::uint8_t>(detail >> 8U);
    payload[10] = static_cast<std::uint8_t>(detail);
    frame[sizeof(frame) - 1] = IRIS_PACKET_EOF_VALUE;
    if (radio) {
        (void)maint_radio::queue_frame(frame, sizeof(frame));
    } else {
        maint_usb::write(frame, sizeof(frame));
    }
}

void handle_command(const std::uint8_t *frame, const std::size_t length, const bool radio)
{
    if (length != IRIS_PACKET_COMMAND_FRAME_LENGTH || frame[0] != IRIS_PACKET_ID_COMMAND) return;
    constexpr std::size_t kOpcodeOffset = IRIS_PACKET_ID_LENGTH + IRIS_PACKET_COMMAND_VALUE_OFFSET;
    const std::uint16_t command =
        static_cast<std::uint16_t>((static_cast<std::uint16_t>(frame[kOpcodeOffset]) << 8U) | frame[kOpcodeOffset + 1U]);
    const std::uint8_t *args = frame + IRIS_PACKET_ID_LENGTH;
    const char *source = radio ? "radio" : "USB";

    if (command == IRIS_COMMAND_WIFI) {
        if (args[0] == 2U) {   // status query
            send_ack(radio, command, args, IRIS_ACK_OK, g_shared.wifi_status.load());
            return;
        }
        const wifi_ota_logic::WifiCommand parsed = wifi_ota_logic::parse_wifi_command(
            args, WifiOta::kDefaultTxPowerDbm, WifiOta::kDefaultAutoOffMinutes);
        if (!parsed.valid) {
            send_ack(radio, command, args, IRIS_ACK_BAD_ARGUMENT);
            return;
        }
        if (parsed.on) {
            // Already in a maintenance session: just extend the Wi-Fi window
            // (a ground station retrying the on command lands here). In USB
            // export mode there is no Wi-Fi.
            if (g_shared.mode.load() != IRIS_MAINT_MODE_WIFI) {
                send_ack(radio, command, args, IRIS_ACK_REFUSED);
                return;
            }
            wifi_keep_alive(parsed.auto_off_minutes);
            send_ack(radio, command, args, IRIS_ACK_OK, g_shared.wifi_status.load());
            ESP_LOGI(kLogTag, "Wi-Fi on over %s: already up, auto-off now %u min", source,
                     static_cast<unsigned>(parsed.auto_off_minutes));
            return;
        }
        // Off = leave maintenance. The ack goes out first.
        send_ack(radio, command, args, IRIS_ACK_OK);
        leave(radio ? kRadioAckGuardMs : 300, radio ? "Wi-Fi off over radio" : "Wi-Fi off over USB");
        return;
    }
    if (command == IRIS_COMMAND_RADIO_POWER) {
        const int requested = static_cast<std::int8_t>(args[1]);
        std::uint8_t result = IRIS_ACK_OK;
        if (args[0] != IRIS_RADIO_POWER_TARGET_WITNESS || !maint_radio::set_power(requested, args[2])) {
            result = IRIS_ACK_BAD_ARGUMENT;
        }
        send_ack(radio, command, args, result,
                 static_cast<std::uint16_t>((maint_radio::front_end_on() ? 0x100U : 0U) |
                                            static_cast<std::uint8_t>(maint_radio::tx_power_dbm())));
        return;
    }
    // Everything else belongs to the flight image.
    ESP_LOGI(kLogTag, "command 0x%04X over %s is not handled in maintenance", command, source);
    send_ack(radio, command, args, IRIS_ACK_UNKNOWN);
}

}  // namespace maint

extern "C" void app_main(void)
{
    // NVS holds the hand-over request and boot bookkeeping (the IMU calibration
    // namespace is never touched, and a damaged NVS is never erased here).
    const esp_err_t nvs_result = nvs_flash_init();
    if (nvs_result != ESP_OK) ESP_LOGE(kLogTag, "NVS init failed: %s", esp_err_to_name(nvs_result));

    maintenance_mode::Request request;
    const bool armed = maintenance_mode::consume(request);
    maint::Shared &shared = maint::shared();
    shared.own_image_id = boot_manager::own_image_id();
    bool recovery = false;
    bool unconfirmed = false;

    if (!armed) {
        // A crash, watchdog, power loss or rollback landed here, or the boot
        // slot was left pointing at us: no maintenance was requested.
        ESP_LOGW(kLogTag, "boot without a maintenance request: returning to flight");
        const int slot = boot_manager::return_to_flight(-1);
        if (slot >= 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
            esp_restart();
        }
        recovery = true;   // no flight image at all: stay up so one can be uploaded
        request = maintenance_mode::Request{};
    } else if (request.verify) {
        // First boot of a freshly uploaded maintenance image: do NOT point the
        // boot slot at flight until the self-test below passes, so a failure
        // rolls back to the previous maintenance image (which returns to
        // flight itself).
        unconfirmed = true;
        shared.flight_slot = request.flight_slot;
    } else {
        shared.flight_slot = boot_manager::return_to_flight(request.flight_slot);
        recovery = shared.flight_slot < 0;
    }
    shared.flight_state = request.flight_state;
    if (recovery) maint::set_flag(IRIS_MAINT_FLAG_RECOVERY, true);
    if (unconfirmed) maint::set_flag(IRIS_MAINT_FLAG_UNCONFIRMED, true);
    {
        const int fslot = shared.flight_slot >= 0 ? shared.flight_slot : boot_manager::last_good_flight();
        shared.flight_image_id =
            fslot >= 0 ? boot_manager::image_id(boot_manager::slot_partition(boot_manager::Kind::Flight, fslot)) : 0;
    }
    const bool usb_export = request.kind == maintenance_mode::Kind::UsbExport && !recovery;
    shared.mode.store(usb_export ? IRIS_MAINT_MODE_USB_EXPORT : IRIS_MAINT_MODE_WIFI);
    ESP_LOGW(kLogTag, "MAINTENANCE image %s (%s): mode %s%s%s; flight slot %d; logging OFF, NOT FLIGHT READY",
             esp_app_get_description()->version, esp_ota_get_running_partition()->label,
             usb_export ? "USB export" : "Wi-Fi", recovery ? ", RECOVERY (no flight image)" : "",
             unconfirmed ? ", UNCONFIRMED (verification boot)" : "", shared.flight_slot);

    // Read-only store (the write path does not exist in this image).
    static WitnessStatus witness_status;
    static FlashLogger flash_logger(witness_status);
    const esp_err_t flash_result = flash_logger.start_export_only();
    if (flash_result != ESP_OK) ESP_LOGE(kLogTag, "log store task start failed: %s", esp_err_to_name(flash_result));

    const esp_err_t radio_result = maint_radio::start(request.radio_dbm, request.radio_front_end);
    if (radio_result != ESP_OK) ESP_LOGW(kLogTag, "radio start: %s", esp_err_to_name(radio_result));

    static WifiOta wifi(flash_logger);
    g_wifi = &wifi;
    wifi.set_return_slot(shared.flight_slot);
    if (usb_export) {
        // Gives up after an upper bound so a forgotten export cannot keep the
        // rocket out of flight mode indefinitely.
        const esp_err_t usb_result = maint_usb::start_export(flash_logger, kStoreMountTimeoutMs);
        if (usb_result != ESP_OK) ESP_LOGE(kLogTag, "USB export start failed: %s", esp_err_to_name(usb_result));
    } else {
        const esp_err_t usb_result = maint_usb::start_serial();
        if (usb_result != ESP_OK) ESP_LOGW(kLogTag, "USB serial start failed: %s", esp_err_to_name(usb_result));
        const std::uint32_t minutes = recovery ? 0U
                                               : (request.minutes != 0 ? request.minutes : WifiOta::kDefaultAutoOffMinutes);
        const esp_err_t wifi_result = wifi.start(request.tx_dbm, minutes);
        if (wifi_result != ESP_OK) ESP_LOGE(kLogTag, "Wi-Fi task start failed: %s", esp_err_to_name(wifi_result));
    }

    const std::uint32_t boot_ms = static_cast<std::uint32_t>(esp_timer_get_time() / 1000);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        const std::uint32_t now_ms = static_cast<std::uint32_t>(esp_timer_get_time() / 1000);
        const bool mounted = flash_logger.frozen();
        maint::set_flag(IRIS_MAINT_FLAG_STORE_MOUNTED, mounted);
        if (mounted) {
            shared.log_files.store(static_cast<std::uint8_t>(
                std::min<std::size_t>(flash_logger.export_file_count(), 255U)));
        }
        const bool disk = usb_export && mounted;
        maint::set_flag(IRIS_MAINT_FLAG_USB_DISK, disk);

        if (unconfirmed) {
            // Self-test of a freshly uploaded maintenance image: radio up, log
            // store mounted, Wi-Fi AP up (Wi-Fi mode). Pass: confirm it (the
            // flight image may now pick it) and make flight the next boot.
            const bool ok = maint_radio::ready() && mounted && (usb_export || wifi.ap_up());
            if (ok) {
                shared.flight_slot = boot_manager::return_to_flight(request.flight_slot);
                recovery = shared.flight_slot < 0;
                if (recovery) maint::set_flag(IRIS_MAINT_FLAG_RECOVERY, true);
                boot_manager::note_maintenance_confirmed(boot_manager::own_info().build_id);
                maint::set_flag(IRIS_MAINT_FLAG_UNCONFIRMED, false);
                wifi.set_return_slot(shared.flight_slot);
                unconfirmed = false;
                ESP_LOGW(kLogTag, "verification boot passed at %u ms: this maintenance image is confirmed",
                         static_cast<unsigned>(now_ms));
            } else if (now_ms - boot_ms >= kVerifyTimeoutMs) {
                ESP_LOGE(kLogTag, "verification boot failed (radio %d, store %d, AP %d): rolling back",
                         maint_radio::ready(), mounted, usb_export || wifi.ap_up());
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_ota_mark_app_invalid_rollback_and_reboot();
                esp_restart();   // only reached if there is nothing to roll back to
            }
        }
        if (usb_export && now_ms - boot_ms >= kUsbExportMaxMinutes * 60000U) {
            maint::leave(100, "USB export time limit");
        }
    }
}
