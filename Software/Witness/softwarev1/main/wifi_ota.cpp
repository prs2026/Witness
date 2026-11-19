#include "wifi_ota.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <strings.h>

#include "boot_manager.h"
#include "comms.h"
#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "flash_logger.h"
#include "image_info.h"
#include "maint_app.h"
#include "task_cores.h"
#include "wifi_ota_logic.h"

namespace {
constexpr char kLogTag[] = "wifi_ota";
constexpr char kProjectName[] = "softwarev1";
constexpr TickType_t kLogWaitTimeout = pdMS_TO_TICKS(30000);

static_assert(sizeof(CONFIG_WITNESS_WIFI_PASSWORD) >= 9 && sizeof(CONFIG_WITNESS_WIFI_PASSWORD) <= 64,
              "CONFIG_WITNESS_WIFI_PASSWORD must be 8..63 characters (WPA2)");

void wifi_event_handler(void *, esp_event_base_t base, std::int32_t id, void *data)
{
    if (base != WIFI_EVENT) return;
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        const auto *event = static_cast<wifi_event_ap_staconnected_t *>(data);
        ESP_LOGI(kLogTag, "station %02X:%02X:%02X:%02X:%02X:%02X joined", event->mac[0], event->mac[1],
                 event->mac[2], event->mac[3], event->mac[4], event->mac[5]);
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        const auto *event = static_cast<wifi_event_ap_stadisconnected_t *>(data);
        ESP_LOGI(kLogTag, "station %02X:%02X:%02X:%02X:%02X:%02X left", event->mac[0], event->mac[1],
                 event->mac[2], event->mac[3], event->mac[4], event->mac[5]);
    }
}

const char *state_text(const esp_partition_t *partition)
{
    esp_ota_img_states_t state;
    if (partition == nullptr || esp_ota_get_state_partition(partition, &state) != ESP_OK) return "unknown";
    switch (state) {
    case ESP_OTA_IMG_NEW: return "new";
    case ESP_OTA_IMG_PENDING_VERIFY: return "pending";
    case ESP_OTA_IMG_VALID: return "valid";
    case ESP_OTA_IMG_INVALID: return "invalid";
    case ESP_OTA_IMG_ABORTED: return "aborted";
    default: return "undefined";
    }
}

// {"slot":"flight_a","version":"...","build":N} or {"slot":"flight_a","version":null}
std::size_t slot_json(char *out, const std::size_t cap, const boot_manager::Kind kind, const int index)
{
    const esp_partition_t *partition = boot_manager::slot_partition(kind, index);
    image_info::Info info{};
    esp_app_desc_t desc;
    if (partition == nullptr) return static_cast<std::size_t>(std::snprintf(out, cap, "null"));
    if (!boot_manager::read_info(partition, kind, info) ||
        esp_ota_get_partition_description(partition, &desc) != ESP_OK) {
        return static_cast<std::size_t>(
            std::snprintf(out, cap, "{\"slot\":\"%s\",\"version\":null}", partition->label));
    }
    char version[40];
    wifi_ota_logic::json_escape(version, sizeof(version), desc.version);
    return static_cast<std::size_t>(std::snprintf(
        out, cap, "{\"slot\":\"%s\",\"version\":\"%s\",\"build\":%u,\"store\":%u,\"command_set\":%u}",
        partition->label, version, static_cast<unsigned>(info.build_id), static_cast<unsigned>(info.store_version),
        static_cast<unsigned>(info.command_set_version)));
}
}  // namespace

WifiOta::WifiOta(FlashLogger &flash_logger) : flash_logger_(flash_logger) {}

std::uint32_t WifiOta::now_ms() const
{
    return static_cast<std::uint32_t>(esp_timer_get_time() / 1000);
}

void WifiOta::touch()
{
    last_activity_ms_.store(now_ms());
}

void WifiOta::keep_alive(const std::uint32_t minutes)
{
    auto_off_ms_.store(minutes * 60000U);
    touch();
}

esp_err_t WifiOta::start(const int tx_dbm, const std::uint32_t auto_off_minutes)
{
    if (task_ != nullptr) return ESP_ERR_INVALID_STATE;
    log_mutex_ = xSemaphoreCreateMutex();
    if (log_mutex_ == nullptr) return ESP_ERR_NO_MEM;
    tx_power_dbm_ = tx_dbm != 0 ? tx_dbm : kDefaultTxPowerDbm;
    auto_off_ms_.store(auto_off_minutes * 60000U);
    touch();
    return xTaskCreatePinnedToCore(task_entry, "wifi_ota", kTaskStackSize, this, kTaskPriority, &task_,
                                   task_cores::kSystem) == pdPASS
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}

void WifiOta::task_entry(void *context)
{
    static_cast<WifiOta *>(context)->run();
}

void WifiOta::run()
{
    const esp_err_t result = start_wifi();
    if (result != ESP_OK) {
        // Nothing to serve: the beacon (status word, "AP down") still tells the
        // ground station, and the operator can send Wi-Fi off over LoRa.
        // The auto-off below only runs with the AP up, so without this the
        // board would stay in maintenance (no sensors, no logging) until a
        // Wi-Fi off command or a power cycle: return to flight after
        // kFailedStartLeaveMs instead (audit 2026-11-19 02 M1).
        ESP_LOGE(kLogTag, "Wi-Fi start failed: %s; back to flight in %u s", esp_err_to_name(result),
                 static_cast<unsigned>(kFailedStartLeaveMs / 1000U));
        failed_.store(true);
    }
    const std::uint32_t failed_at_ms = now_ms();
    bool failed_leave_sent = false;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(kPollIntervalMs));
        maint::shared().wifi_status.store(status_word());
        // Not in recovery (no flight image: auto-off 0), where staying up is the point.
        if (failed_.load() && !failed_leave_sent && auto_off_ms_.load() != 0 &&
            now_ms() - failed_at_ms >= kFailedStartLeaveMs) {
            maint::leave(200, "Wi-Fi start failed");
            failed_leave_sent = true;
        }
        const std::uint32_t window = auto_off_ms_.load();
        if (enabled_.load() && window != 0 && now_ms() - last_activity_ms_.load() >= window) {
            maint::leave(200, "Wi-Fi auto-off, no HTTP activity");
            auto_off_ms_.store(0);   // once
        }
    }
}

esp_err_t WifiOta::start_wifi()
{
    ESP_LOGW(kLogTag, "starting Wi-Fi AP: max TX %d dBm, auto-off %u min", tx_power_dbm_,
             static_cast<unsigned>(auto_off_ms_.load() / 60000U));
    const auto step = [this](const std::uint8_t n, const char *what) {
        start_step_.store(n);
        ESP_LOGW(kLogTag, "start step %u: %s (free heap %u, internal %u)", n, what,
                 static_cast<unsigned>(esp_get_free_heap_size()),
                 static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
    };

    esp_err_t result = ESP_OK;
    bool wifi_inited = false;
    bool wifi_started = false;
    do {
        if (!netif_ready_) {
            step(1, "esp_netif_init");
            result = esp_netif_init();
            if (result != ESP_OK) break;
            step(2, "esp_event_loop_create_default");
            result = esp_event_loop_create_default();
            if (result == ESP_ERR_INVALID_STATE) result = ESP_OK;   // already created
            if (result != ESP_OK) break;
            netif_ready_ = true;
        }
        step(3, "esp_netif_create_default_wifi_ap");
        ap_netif_ = esp_netif_create_default_wifi_ap();   // 192.168.4.1/24 with DHCP
        if (ap_netif_ == nullptr) {
            result = ESP_ERR_NO_MEM;
            break;
        }
        const wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
        step(4, "esp_wifi_init");
        result = esp_wifi_init(&init_config);
        if (result != ESP_OK) break;
        wifi_inited = true;
        (void)esp_wifi_set_storage(WIFI_STORAGE_RAM);   // never touch NVS (IMU calibration lives there)
        step(5, "event handler register");
        result = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr);
        if (result != ESP_OK) break;

        std::uint8_t mac[6]{};
        if (esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP) != ESP_OK) {
            ESP_LOGW(kLogTag, "MAC read failed; SSID suffix is 0000 (set WITNESS_WIFI_SSID_OVERRIDE)");
        }
        wifi_config_t config{};
        const std::size_t ssid_length = wifi_ota_logic::format_ssid(
            reinterpret_cast<char *>(config.ap.ssid), sizeof(config.ap.ssid), mac,
            CONFIG_WITNESS_WIFI_SSID_OVERRIDE);
        config.ap.ssid_len = static_cast<std::uint8_t>(ssid_length);
        std::strncpy(reinterpret_cast<char *>(config.ap.password), CONFIG_WITNESS_WIFI_PASSWORD,
                     sizeof(config.ap.password) - 1);
        config.ap.channel = CONFIG_WITNESS_WIFI_CHANNEL;
        config.ap.authmode = WIFI_AUTH_WPA2_PSK;
        config.ap.max_connection = 2;
        config.ap.pairwise_cipher = WIFI_CIPHER_TYPE_CCMP;
        config.ap.beacon_interval = 100;
        step(6, "esp_wifi_set_mode");
        result = esp_wifi_set_mode(WIFI_MODE_AP);
        if (result != ESP_OK) break;
        step(7, "esp_wifi_set_config");
        result = esp_wifi_set_config(WIFI_IF_AP, &config);
        if (result != ESP_OK) break;
        step(8, "esp_wifi_start");
        result = esp_wifi_start();
        if (result != ESP_OK) break;
        wifi_started = true;
        step(9, "esp_wifi_set_max_tx_power");
        result = esp_wifi_set_max_tx_power(
            static_cast<std::int8_t>(wifi_ota_logic::tx_power_to_wifi_units(tx_power_dbm_)));
        if (result != ESP_OK) break;

        httpd_config_t http = HTTPD_DEFAULT_CONFIG();
        http.core_id = task_cores::kSystem;
        http.task_priority = 2;
        http.stack_size = 6144;
        http.max_open_sockets = 3;
        http.max_uri_handlers = 8;
        http.lru_purge_enable = true;
        http.recv_wait_timeout = 10;
        http.send_wait_timeout = 10;
        http.uri_match_fn = httpd_uri_match_wildcard;
        step(10, "httpd_start");
        result = httpd_start(&server_, &http);
        if (result != ESP_OK) {
            server_ = nullptr;
            break;
        }
        struct Route {
            const char *uri;
            httpd_method_t method;
            esp_err_t (*handler)(httpd_req_t *);
        };
        const Route routes[] = {
            {"/info", HTTP_GET, handle_info},
            {"/ota", HTTP_POST, handle_ota},
            {"/logs", HTTP_GET, handle_logs},
            {"/logs/*", HTTP_GET, handle_log_file},
            {"/reboot", HTTP_POST, handle_reboot},
            {"/wifi/off", HTTP_POST, handle_wifi_off},
        };
        for (const Route &route : routes) {
            httpd_uri_t entry{};
            entry.uri = route.uri;
            entry.method = route.method;
            entry.handler = route.handler;
            entry.user_ctx = this;
            result = httpd_register_uri_handler(server_, &entry);
            if (result != ESP_OK) break;
        }
        if (result != ESP_OK) break;

        step(12, "AP up");
        enabled_.store(true);
        maint::set_flag(IRIS_MAINT_FLAG_AP_UP, true);
        ESP_LOGI(kLogTag, "Wi-Fi AP \"%s\" up on channel %d at 192.168.4.1 (stack free %u B)",
                 reinterpret_cast<const char *>(config.ap.ssid), CONFIG_WITNESS_WIFI_CHANNEL,
                 static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
        return ESP_OK;
    } while (false);

    // Failure: unwind whatever was brought up.
    ESP_LOGE(kLogTag, "Wi-Fi start failed at step %u: %s", start_step_.load(), esp_err_to_name(result));
    start_step_.store(0xEE);
    if (server_ != nullptr) {
        httpd_stop(server_);
        server_ = nullptr;
    }
    (void)esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler);
    if (wifi_started) esp_wifi_stop();
    if (wifi_inited) esp_wifi_deinit();
    if (ap_netif_ != nullptr) {
        esp_netif_destroy_default_wifi(ap_netif_);
        ap_netif_ = nullptr;
    }
    return result;
}

esp_err_t WifiOta::send_json(httpd_req_t *req, const char *status, const char *json)
{
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

// Error replies close the connection: a request body may be unread.
esp_err_t WifiOta::send_error(httpd_req_t *req, const char *status, const char *message)
{
    char escaped[128];
    char body[192];
    wifi_ota_logic::json_escape(escaped, sizeof(escaped), message);
    std::snprintf(body, sizeof(body), "{\"ok\":false,\"error\":\"%s\"}", escaped);
    httpd_resp_set_hdr(req, "Connection", "close");
    send_json(req, status, body);
    return ESP_FAIL;
}

esp_err_t WifiOta::wait_logs()
{
    xSemaphoreTake(log_mutex_, portMAX_DELAY);
    // The maintenance image mounts the store read-only at boot; this just
    // waits for the one-time scan to finish.
    const esp_err_t result = flash_logger_.prepare_mass_storage(kLogWaitTimeout);
    xSemaphoreGive(log_mutex_);
    return result;
}

esp_err_t WifiOta::handle_info(httpd_req_t *req)
{
    auto *self = static_cast<WifiOta *>(req->user_ctx);
    self->touch();
    const esp_app_desc_t *desc = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    char version[40];
    wifi_ota_logic::json_escape(version, sizeof(version), desc->version);
    char idf[40];
    wifi_ota_logic::json_escape(idf, sizeof(idf), desc->idf_ver);
    char flight[2][160];
    char maint_slots[2][160];
    for (int i = 0; i < 2; ++i) {
        slot_json(flight[i], sizeof(flight[i]), boot_manager::Kind::Flight, i);
        slot_json(maint_slots[i], sizeof(maint_slots[i]), boot_manager::Kind::Maintenance, i);
    }
    const maint::Shared &shared = maint::shared();
    const esp_partition_t *return_slot =
        self->return_slot_ >= 0 ? boot_manager::slot_partition(boot_manager::Kind::Flight, self->return_slot_)
                                : nullptr;
    char body[1100];
    std::snprintf(body, sizeof(body),
                  "{\"project\":\"%s\",\"kind\":\"maintenance\",\"version\":\"%s\",\"idf\":\"%s\","
                  "\"running\":\"%s\",\"app_state\":\"%s\",\"uptime_ms\":%llu,\"mode\":\"%s\","
                  "\"wifi_tx_dbm\":%d,\"free_heap\":%u,\"store_mounted\":%s,\"log_files\":%u,"
                  "\"recovery\":%s,\"unconfirmed\":%s,\"flight_return\":\"%s\","
                  "\"flight\":[%s,%s],\"maintenance\":[%s,%s],\"confirmed_build\":%u}",
                  desc->project_name, version, idf, running != nullptr ? running->label : "", state_text(running),
                  static_cast<unsigned long long>(esp_timer_get_time() / 1000),
                  shared.mode.load() == IRIS_MAINT_MODE_USB_EXPORT ? "usb_export" : "wifi", self->tx_power_dbm_,
                  static_cast<unsigned>(esp_get_free_heap_size()),
                  (shared.flags.load() & IRIS_MAINT_FLAG_STORE_MOUNTED) != 0 ? "true" : "false",
                  static_cast<unsigned>(shared.log_files.load()),
                  (shared.flags.load() & IRIS_MAINT_FLAG_RECOVERY) != 0 ? "true" : "false",
                  (shared.flags.load() & IRIS_MAINT_FLAG_UNCONFIRMED) != 0 ? "true" : "false",
                  return_slot != nullptr ? return_slot->label : "", flight[0], flight[1], maint_slots[0],
                  maint_slots[1], static_cast<unsigned>(boot_manager::confirmed_maintenance_build()));
    return self->send_json(req, "200 OK", body);
}

esp_err_t WifiOta::handle_ota(httpd_req_t *req)
{
    auto *self = static_cast<WifiOta *>(req->user_ctx);
    self->touch();
    bool expected = false;
    if (!self->ota_busy_.compare_exchange_strong(expected, true)) {
        return self->send_error(req, "409 Conflict", "another update is in progress");
    }
    struct Release {
        std::atomic_bool &flag;
        ~Release() { flag.store(false); }
    } release{self->ota_busy_};

    if ((maint::shared().flags.load() & IRIS_MAINT_FLAG_UNCONFIRMED) != 0) {
        return self->send_error(req, "409 Conflict", "this maintenance image is still unconfirmed");
    }
    const std::size_t total = req->content_len;
    if (total == 0) return self->send_error(req, "411 Length Required", "Content-Length required");
    constexpr std::size_t kHead = image_info::kHeadLength;
    static_assert(kHead >= wifi_ota_logic::kImageHeadLength, "image-info head covers the app descriptor");
    if (total < kHead) return self->send_error(req, "400 Bad Request", "image too short");

    auto *buffer = static_cast<std::uint8_t *>(std::malloc(kTransferChunk));
    if (buffer == nullptr) return self->send_error(req, "503 Service Unavailable", "out of memory");
    struct Free {
        void *p;
        ~Free() { std::free(p); }
    } free_buffer{buffer};

    // Read until the image header, application descriptor and image-info
    // record are in hand.
    std::size_t have = 0;
    std::size_t received = 0;
    int stalls = 0;
    while (have < kHead) {
        const int n = httpd_req_recv(req, reinterpret_cast<char *>(buffer) + have,
                                     std::min(kTransferChunk - have, total - received));
        if (n == HTTPD_SOCK_ERR_TIMEOUT && ++stalls < 3) continue;
        if (n <= 0) return self->send_error(req, "408 Request Timeout", "upload stalled");
        have += static_cast<std::size_t>(n);
        received += static_cast<std::size_t>(n);
    }
    const wifi_ota_logic::ImageCheck check = wifi_ota_logic::check_image_head(buffer, have, kProjectName);
    if (check != wifi_ota_logic::ImageCheck::Ok) {
        ESP_LOGW(kLogTag, "OTA rejected: %s", wifi_ota_logic::image_check_message(check));
        return self->send_error(req, "422 Unprocessable Entity", wifi_ota_logic::image_check_message(check));
    }
    image_info::Info up_info{};
    if (!image_info::parse(buffer, have, up_info) ||
        (up_info.kind != IRIS_IMAGE_KIND_FLIGHT && up_info.kind != IRIS_IMAGE_KIND_MAINTENANCE)) {
        return self->send_error(req, "422 Unprocessable Entity",
                                "no Witness image-info record (flight/maintenance kind unknown)");
    }
    const bool is_flight = up_info.kind == IRIS_IMAGE_KIND_FLIGHT;
    esp_app_desc_t new_desc;
    std::memcpy(&new_desc, buffer + wifi_ota_logic::kAppDescOffset, sizeof(new_desc));

    // The slot of the matching kind that is not in use.
    const int running_slot = boot_manager::running_slot();
    int target_index = 0;
    if (is_flight) {
        target_index = self->return_slot_ >= 0 ? image_info::inactive_slot(self->return_slot_) : 0;
    } else {
        target_index = image_info::inactive_slot(running_slot);
    }
    const esp_partition_t *slot = boot_manager::slot_partition(
        is_flight ? boot_manager::Kind::Flight : boot_manager::Kind::Maintenance, target_index);
    if (slot == nullptr) return self->send_error(req, "500 Internal Server Error", "no OTA slot");
    if (total > slot->size) return self->send_error(req, "413 Payload Too Large", "image larger than the slot");

    // Compatibility with the OTHER image kind: the flight image refuses to
    // switch to an incompatible maintenance image, so never install a pair
    // that would strand the operator. X-Witness-Force: 1 overrides (coordinated
    // updates of both images; update maintenance first).
    char force_header[8]{};
    const bool force = httpd_req_get_hdr_value_str(req, "X-Witness-Force", force_header, sizeof(force_header)) ==
                           ESP_OK &&
                       force_header[0] == '1';
    image_info::Compat compat = image_info::Compat::Ok;
    if (is_flight) {
        compat = image_info::check_maintenance(&boot_manager::own_info(), up_info);
    } else if (self->return_slot_ >= 0) {
        image_info::Info flight_info{};
        if (boot_manager::read_info(boot_manager::slot_partition(boot_manager::Kind::Flight, self->return_slot_),
                                    boot_manager::Kind::Flight, flight_info)) {
            compat = image_info::check_maintenance(&up_info, flight_info);
        }
    }
    if (compat != image_info::Compat::Ok && !force) {
        char message[192];
        std::snprintf(message, sizeof(message),
                      "incompatible with the %s image (refusal code 0x%04X); update the other image first or "
                      "send X-Witness-Force: 1",
                      is_flight ? "installed maintenance" : "installed flight",
                      static_cast<unsigned>(image_info::refuse_code(compat)));
        ESP_LOGW(kLogTag, "OTA rejected: %s", message);
        return self->send_error(req, "409 Conflict", message);
    }

    esp_ota_handle_t handle = 0;
    esp_err_t result = esp_ota_begin(slot, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (result != ESP_OK) {
        return self->send_error(req, "500 Internal Server Error", esp_err_to_name(result));
    }
    ESP_LOGW(kLogTag, "OTA (%s image) to %s: %u bytes, new version %.31s", is_flight ? "flight" : "maintenance",
             slot->label, static_cast<unsigned>(total), new_desc.version);
    const std::uint32_t started_ms = self->now_ms();
    std::size_t written = 0;
    const char *failure = nullptr;
    for (;;) {
        if (have > 0) {
            result = esp_ota_write(handle, buffer, have);
            if (result != ESP_OK) {
                failure = esp_err_to_name(result);
                break;
            }
            written += have;
            have = 0;
        }
        if (written >= total) break;
        if (!self->enabled_.load()) {
            failure = "aborted: Wi-Fi stopping";
            break;
        }
        const int n = httpd_req_recv(req, reinterpret_cast<char *>(buffer), std::min(kTransferChunk, total - written));
        if (n == HTTPD_SOCK_ERR_TIMEOUT && ++stalls < 6) continue;
        if (n <= 0) {
            failure = "upload stalled";
            break;
        }
        stalls = 0;
        have = static_cast<std::size_t>(n);
        self->touch();
    }
    if (failure != nullptr) {
        esp_ota_abort(handle);
        ESP_LOGE(kLogTag, "OTA failed: %s", failure);
        return self->send_error(req, "500 Internal Server Error", failure);
    }
    result = esp_ota_end(handle);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "OTA image invalid: %s", esp_err_to_name(result));
        return self->send_error(req, "422 Unprocessable Entity", "image failed validation");
    }
    // Flight: boot it now; its self-test confirms it or the bootloader rolls
    // back (to the maintenance image, which returns to the old flight slot).
    // Maintenance: boot it once as a verification boot (see boot_manager.h).
    if (is_flight) {
        result = boot_manager::boot_new_flight(slot);
    } else {
        result = boot_manager::boot_other_maintenance_for_verify(
            slot, self->return_slot_, self->tx_power_dbm_,
            static_cast<std::uint32_t>(CONFIG_WITNESS_WIFI_CONFIRM_MINUTES > 0 ? CONFIG_WITNESS_WIFI_CONFIRM_MINUTES
                                                                                : 1));
    }
    if (result != ESP_OK) {
        return self->send_error(req, "500 Internal Server Error", esp_err_to_name(result));
    }
    ESP_LOGW(kLogTag, "OTA complete in %u ms; next boot %s", static_cast<unsigned>(self->now_ms() - started_ms),
             slot->label);
    char version[40];
    wifi_ota_logic::json_escape(version, sizeof(version), new_desc.version);
    char body[320];
    std::snprintf(body, sizeof(body),
                  "{\"ok\":true,\"kind\":\"%s\",\"bytes\":%u,\"slot\":\"%s\",\"version\":\"%s\",\"reboot_ms\":%u,"
                  "\"next\":\"%s\"}",
                  is_flight ? "flight" : "maintenance", static_cast<unsigned>(written), slot->label, version,
                  static_cast<unsigned>(kRestartDelayMs),
                  is_flight ? "flight self-test" : "maintenance verification boot");
    const esp_err_t sent = self->send_json(req, "200 OK", body);
    // maint::leave only schedules the restart; the boot partition is already
    // the new image (above), so this restart goes there, not to the old flight.
    maint::leave(kRestartDelayMs, "OTA complete");
    return sent;
}

esp_err_t WifiOta::handle_logs(httpd_req_t *req)
{
    auto *self = static_cast<WifiOta *>(req->user_ctx);
    self->touch();
    if (self->wait_logs() != ESP_OK) {
        return self->send_error(req, "503 Service Unavailable", "log store not mounted");
    }
    const std::size_t count = std::min(self->flash_logger_.export_file_count(), FlashLogger::kMaximumExportFiles);
    httpd_resp_set_type(req, "application/json");
    char piece[128];
    std::snprintf(piece, sizeof(piece), "{\"logging_frozen\":true,\"count\":%u,\"files\":[",
                  static_cast<unsigned>(count));
    esp_err_t result = httpd_resp_send_chunk(req, piece, HTTPD_RESP_USE_STRLEN);
    for (std::size_t i = 0; i < count && result == ESP_OK; ++i) {
        const FlashLogger::ExportFile &file = self->flash_logger_.export_file(i);
        char name[32];
        wifi_ota_logic::json_escape(name, sizeof(name), file.name);
        std::snprintf(piece, sizeof(piece), "%s{\"name\":\"%s\",\"size\":%u,\"session\":%u,\"slot\":%u}",
                      i == 0 ? "" : ",", name, static_cast<unsigned>(file.size),
                      static_cast<unsigned>(file.generation), static_cast<unsigned>(file.slot));
        result = httpd_resp_send_chunk(req, piece, HTTPD_RESP_USE_STRLEN);
        self->touch();
    }
    if (result == ESP_OK) result = httpd_resp_send_chunk(req, "]}", HTTPD_RESP_USE_STRLEN);
    if (result == ESP_OK) result = httpd_resp_send_chunk(req, nullptr, 0);
    return result;
}

esp_err_t WifiOta::handle_log_file(httpd_req_t *req)
{
    auto *self = static_cast<WifiOta *>(req->user_ctx);
    self->touch();
    constexpr char kPrefix[] = "/logs/";
    char name[16]{};
    const char *uri = req->uri;
    if (std::strncmp(uri, kPrefix, sizeof(kPrefix) - 1) != 0) {
        return self->send_error(req, "404 Not Found", "unknown path");
    }
    uri += sizeof(kPrefix) - 1;
    const std::size_t length = strcspn(uri, "?");
    if (length == 0 || length >= sizeof(name)) return self->send_error(req, "404 Not Found", "no such log");
    std::memcpy(name, uri, length);
    if (!wifi_ota_logic::valid_log_name(name)) return self->send_error(req, "400 Bad Request", "bad file name");
    if (self->wait_logs() != ESP_OK) {
        return self->send_error(req, "503 Service Unavailable", "log store not mounted");
    }
    const std::size_t count = std::min(self->flash_logger_.export_file_count(), FlashLogger::kMaximumExportFiles);
    std::size_t index = count;
    for (std::size_t i = 0; i < count; ++i) {
        if (strcasecmp(self->flash_logger_.export_file(i).name, name) == 0) {
            index = i;
            break;
        }
    }
    if (index == count) return self->send_error(req, "404 Not Found", "no such log");
    const std::uint32_t size = self->flash_logger_.export_file(index).size;

    auto *buffer = static_cast<std::uint8_t *>(std::malloc(kTransferChunk));
    if (buffer == nullptr) return self->send_error(req, "503 Service Unavailable", "out of memory");
    struct Free {
        void *p;
        ~Free() { std::free(p); }
    } free_buffer{buffer};

    char size_text[16];
    std::snprintf(size_text, sizeof(size_text), "%u", static_cast<unsigned>(size));
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "X-File-Size", size_text);
    ESP_LOGI(kLogTag, "sending %s (%u bytes)", name, static_cast<unsigned>(size));
    std::uint32_t offset = 0;
    while (offset < size) {
        if (!self->enabled_.load()) return ESP_FAIL;
        const std::size_t amount = std::min<std::size_t>(kTransferChunk, size - offset);
        xSemaphoreTake(self->log_mutex_, portMAX_DELAY);
        const esp_err_t read = self->flash_logger_.read_export_file(index, offset, buffer, amount);
        xSemaphoreGive(self->log_mutex_);
        if (read != ESP_OK) {
            ESP_LOGE(kLogTag, "read of %s failed at %u: %s", name, static_cast<unsigned>(offset),
                     esp_err_to_name(read));
            return ESP_FAIL;   // truncated chunked reply tells the client
        }
        if (httpd_resp_send_chunk(req, reinterpret_cast<const char *>(buffer), amount) != ESP_OK) {
            return ESP_FAIL;
        }
        offset += static_cast<std::uint32_t>(amount);
        self->touch();
    }
    return httpd_resp_send_chunk(req, nullptr, 0);
}

esp_err_t WifiOta::handle_reboot(httpd_req_t *req)
{
    auto *self = static_cast<WifiOta *>(req->user_ctx);
    self->touch();
    const esp_err_t sent = self->send_json(req, "200 OK", "{\"ok\":true,\"reboot_ms\":1000}");
    maint::leave(kRestartDelayMs, "HTTP /reboot");
    return sent;
}

esp_err_t WifiOta::handle_wifi_off(httpd_req_t *req)
{
    auto *self = static_cast<WifiOta *>(req->user_ctx);
    // Leaving Wi-Fi mode = leaving maintenance: restart into flight, after the
    // reply is out.
    const esp_err_t sent = self->send_json(req, "200 OK", "{\"ok\":true,\"wifi\":\"off\",\"reboot_ms\":1000}");
    maint::leave(kRestartDelayMs, "HTTP /wifi/off");
    return sent;
}
