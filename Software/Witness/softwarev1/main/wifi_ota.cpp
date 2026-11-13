#include "wifi_ota.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <strings.h>

#include "comms.h"
#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include "flash_logger.h"
#include "maintenance_mode.h"
#include "flight_state_machine.h"
#include "sensors.h"
#include "task_cores.h"
#include "wifi_ota_logic.h"
#include "witness_status.h"

namespace {
constexpr char kLogTag[] = "wifi_ota";
constexpr char kProjectName[] = "softwarev1";
constexpr std::uint64_t kStartWatchdogUs = 15000000ULL;
// Time between a Wi-Fi stop that froze logging and the reboot that thaws it;
// long enough for a radio ack of the off command to go out first.
constexpr std::uint32_t kThawRestartDelayMs = 2000;
constexpr TickType_t kLogFreezeTimeout = pdMS_TO_TICKS(15000);

static_assert(sizeof(CONFIG_WITNESS_WIFI_PASSWORD) >= 9 && sizeof(CONFIG_WITNESS_WIFI_PASSWORD) <= 64,
              "CONFIG_WITNESS_WIFI_PASSWORD must be 8..63 characters (WPA2)");

std::atomic_bool g_wifi_on{false};

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
}  // namespace

const char *wifi_ota_state_text()
{
    return g_wifi_on.load() ? "on" : "off";
}

WifiOta::WifiOta(Sensors &sensors, FlashLogger &flash_logger,
                 FlightStateMachine &flight_state_machine, WitnessStatus &witness_status)
    : sensors_(sensors),
      flash_logger_(flash_logger),
      flight_state_machine_(flight_state_machine),
      witness_status_(witness_status)
{
}

std::uint32_t WifiOta::now_ms() const
{
    return static_cast<std::uint32_t>(esp_timer_get_time() / 1000);
}

bool WifiOta::state_allows() const
{
    return wifi_ota_logic::state_allows_wifi(
        static_cast<std::uint8_t>(flight_state_machine_.state()));
}

void WifiOta::touch()
{
    last_activity_ms_.store(now_ms());
}

esp_err_t WifiOta::start()
{
    if (task_ != nullptr) return ESP_ERR_INVALID_STATE;
    queue_ = xQueueCreate(4, sizeof(Request));
    done_ = xSemaphoreCreateBinary();
    log_mutex_ = xSemaphoreCreateMutex();
    if (queue_ == nullptr || done_ == nullptr || log_mutex_ == nullptr) return ESP_ERR_NO_MEM;

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t image_state = ESP_OTA_IMG_UNDEFINED;
    if (running != nullptr && esp_ota_get_state_partition(running, &image_state) == ESP_OK &&
        image_state == ESP_OTA_IMG_PENDING_VERIFY) {
        self_test_pending_ = true;
        ESP_LOGW(kLogTag, "new image is pending verification: self-test within %u s or roll back",
                 static_cast<unsigned>(kSelfTestWindowMs / 1000));
    }
    ESP_LOGI(kLogTag, "running %s (%s), app state %s; %s",
             running != nullptr ? running->label : "?", esp_app_get_description()->version,
             app_state_text(),
             maintenance_mode::active() ? "MAINTENANCE boot: Wi-Fi coming up, NOT logging, NOT flight ready"
                                        : "Wi-Fi off until commanded (a maintenance reboot)");

    if (xTaskCreatePinnedToCore(task_entry, "wifi_ota", kTaskStackSize, this, kTaskPriority,
                                &task_, task_cores::kSystem) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    if (maintenance_mode::active()) {
        const int tx = maintenance_mode::requested_tx_dbm();
        const std::uint32_t minutes = maintenance_mode::requested_minutes();
        const Request enable{RequestKind::Enable,
                             static_cast<std::int8_t>(tx != 0 ? tx : kDefaultTxPowerDbm),
                             static_cast<std::uint16_t>(minutes != 0 ? minutes : kDefaultAutoOffMinutes), 0, false};
        (void)xQueueSend(queue_, &enable, 0);
    }
    return ESP_OK;
}

void WifiOta::task_entry(void *context)
{
    static_cast<WifiOta *>(context)->run();
}

void WifiOta::run()
{
    for (;;) {
        Request request{};
        if (xQueueReceive(queue_, &request, pdMS_TO_TICKS(kPollIntervalMs)) == pdTRUE) {
            handle_request(request);
        }
        self_test_step();
        monitor();
    }
}

std::uint8_t WifiOta::request_enable(const int tx_dbm, const std::uint32_t minutes, const bool radio)
{
    if (!state_allows()) return IRIS_ACK_REFUSED;
    if (mass_storage_active_.load()) return IRIS_ACK_REFUSED;
    if (restart_scheduled_.load() || task_ == nullptr) return IRIS_ACK_BUSY;
    if (!maintenance_mode::active()) {
        // Normal mode has no RAM for Wi-Fi: reboot into a maintenance boot,
        // after the caller's ack has gone out (radio: ack guard time).
        maintenance_mode::request(tx_dbm, minutes);
        const Request restart{RequestKind::Restart, 0, 0,
                              static_cast<std::uint16_t>(radio ? kRadioAckGuardMs + 400 : 400), false};
        ESP_LOGW(kLogTag, "Wi-Fi requested: rebooting into MAINTENANCE mode (not flight ready)");
        return xQueueSend(queue_, &restart, 0) == pdTRUE ? IRIS_ACK_OK : IRIS_ACK_BUSY;
    }
    const Request request{RequestKind::Enable, static_cast<std::int8_t>(tx_dbm),
                          static_cast<std::uint16_t>(minutes),
                          static_cast<std::uint16_t>(radio ? kRadioAckGuardMs : 0), false};
    return xQueueSend(queue_, &request, 0) == pdTRUE ? IRIS_ACK_OK : IRIS_ACK_BUSY;
}

bool WifiOta::request_disable_and_wait(const TickType_t timeout)
{
    if (task_ == nullptr) return true;
    if (!enabled_.load() && uxQueueMessagesWaiting(queue_) == 0) return true;
    (void)xSemaphoreTake(done_, 0);
    const Request request{RequestKind::Disable, 0, 0, 0, true};
    if (xQueueSend(queue_, &request, pdMS_TO_TICKS(100)) != pdTRUE) return false;
    return xSemaphoreTake(done_, timeout) == pdTRUE;
}

void WifiOta::note_mass_storage_active()
{
    mass_storage_active_.store(true);
}

void WifiOta::handle_request(const Request &request)
{
    switch (request.kind) {
    case RequestKind::Enable:
        if (request.delay_ms > 0) vTaskDelay(pdMS_TO_TICKS(request.delay_ms));
        if (mass_storage_active_.load() || !state_allows()) {
            ESP_LOGW(kLogTag, "enable dropped: state or mass storage changed");
        } else if (enabled_.load()) {
            auto_off_ms_ = static_cast<std::uint32_t>(request.minutes) * 60000U;
            touch();
            ESP_LOGI(kLogTag, "already on; auto-off now %u min", static_cast<unsigned>(request.minutes));
        } else {
            const esp_err_t result = start_wifi(request.tx_dbm, request.minutes);
            if (result != ESP_OK) {
                ESP_LOGE(kLogTag, "Wi-Fi start failed: %s", esp_err_to_name(result));
            }
        }
        break;
    case RequestKind::Disable:
        if (request.delay_ms > 0) vTaskDelay(pdMS_TO_TICKS(request.delay_ms));
        if (enabled_.load()) stop_wifi("commanded off");
        break;
    case RequestKind::Restart:
        restart_at_ms_.store(now_ms() + request.delay_ms);
        restart_scheduled_.store(true);
        ESP_LOGW(kLogTag, "reboot in %u ms", static_cast<unsigned>(request.delay_ms));
        break;
    }
    if (request.wake_waiter) (void)xSemaphoreGive(done_);
}

void WifiOta::monitor()
{
    if (restart_scheduled_.load() && static_cast<std::int32_t>(now_ms() - restart_at_ms_.load()) >= 0) {
        // Leaving maintenance (back to flight-ready) is always allowed; any
        // other reboot only on the pad or landed.
        if (state_allows() || (maintenance_mode::active() && !enabled_.load())) {
            ESP_LOGW(kLogTag, "rebooting");
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
        }
        restart_scheduled_.store(false);
        ESP_LOGW(kLogTag, "reboot cancelled: flight state is not Pad Idle/Landed");
    }
    if (!enabled_.load()) return;
    if (mass_storage_active_.load()) {
        stop_wifi("USB mass storage started");
    } else if (!state_allows()) {
        stop_wifi("flight state change");
    } else if (now_ms() - last_activity_ms_.load() >= auto_off_ms_) {
        stop_wifi("auto-off, no HTTP activity");
    }
}

bool WifiOta::self_test_passed() const
{
    const std::uint8_t flags = witness_status_.flags();
    constexpr std::uint8_t kImuReady =
        IRIS_WITNESS_STATUS_LOW_G_ACCEL_READY_MASK | IRIS_WITNESS_STATUS_GYRO_READY_MASK;
    constexpr std::uint8_t kFlashFailed =
        IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK | IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK;
    return (flags & kImuReady) == kImuReady && (flags & kFlashFailed) == 0 && sensors_.radio_ready();
}

void WifiOta::self_test_step()
{
    if (!self_test_pending_) return;
    if (self_test_passed()) {
        const esp_err_t result = esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGI(kLogTag, "self-test passed at %u ms: image marked valid (%s)",
                 static_cast<unsigned>(now_ms()), esp_err_to_name(result));
        self_test_pending_ = false;
        // The post-OTA boot is a maintenance boot (handle_ota requests it), so
        // Wi-Fi is already up for the host to confirm; nothing to open here.
        if (false) {
            ESP_LOGW(kLogTag, "first boot after OTA: Wi-Fi opens for %d min so the host can confirm",
                     CONFIG_WITNESS_WIFI_CONFIRM_MINUTES);
            const Request enable{RequestKind::Enable, static_cast<std::int8_t>(kDefaultTxPowerDbm),
                                 static_cast<std::uint16_t>(CONFIG_WITNESS_WIFI_CONFIRM_MINUTES), 0, false};
            (void)xQueueSend(queue_, &enable, 0);
        }
    } else if (now_ms() >= kSelfTestWindowMs) {
        ESP_LOGE(kLogTag, "self-test failed (status 0x%02X, radio %s): rolling back",
                 witness_status_.flags(), sensors_.radio_ready() ? "ok" : "not ready");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_ota_mark_app_invalid_rollback_and_reboot();
        self_test_pending_ = false;   // only reached if there is no previous image
    }
}

esp_err_t WifiOta::start_wifi(const int tx_dbm, const std::uint32_t minutes)
{
    tx_power_dbm_ = tx_dbm;
    auto_off_ms_ = minutes * 60000U;
    touch();
    ESP_LOGW(kLogTag, "starting Wi-Fi AP: LoRa transmission off, max TX %d dBm, auto-off %u min", tx_dbm,
             static_cast<unsigned>(minutes));
    // Watchdog: if the AP is not up in kStartWatchdogUs (a call below hangs),
    // LoRa transmission comes back on from the esp_timer task, so a stuck
    // Wi-Fi start can never leave Witness silent.
    watchdog_fired_.store(false);
    if (start_watchdog_ == nullptr) {
        esp_timer_create_args_t args{};
        args.callback = &WifiOta::start_watchdog_expired;
        args.arg = this;
        args.name = "wifi_start_wd";
        esp_timer_handle_t timer = nullptr;
        if (esp_timer_create(&args, &timer) == ESP_OK) start_watchdog_ = timer;
    }
    if (start_watchdog_ != nullptr) {
        auto *timer = static_cast<esp_timer_handle_t>(start_watchdog_);
        (void)esp_timer_stop(timer);
        (void)esp_timer_start_once(timer, kStartWatchdogUs);
    }
    const auto step = [this](const std::uint8_t n, const char *what) {
        start_step_.store(n);
        ESP_LOGW(kLogTag, "start step %u: %s (free heap %u, internal %u)", n, what,
                 static_cast<unsigned>(esp_get_free_heap_size()),
                 static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
    };
    sensors_.set_radio_transmission_enabled(false);
    sensors_.set_radio_beacon_only(true);

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
            static_cast<std::int8_t>(wifi_ota_logic::tx_power_to_wifi_units(tx_dbm)));
        if (result != ESP_OK) break;

        httpd_config_t http = HTTPD_DEFAULT_CONFIG();
        http.core_id = task_cores::kSystem;
        http.task_priority = 2;     // below the NAND logger and USB tasks
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
        if (start_watchdog_ != nullptr) (void)esp_timer_stop(static_cast<esp_timer_handle_t>(start_watchdog_));
        enabled_.store(true);
        g_wifi_on.store(true);
        ESP_LOGI(kLogTag, "Wi-Fi AP \"%s\" up on channel %d at 192.168.4.1 (stack free %u B)",
                 reinterpret_cast<const char *>(config.ap.ssid), CONFIG_WITNESS_WIFI_CHANNEL,
                 static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
        return ESP_OK;
    } while (false);

    // Failure: unwind whatever was brought up.
    ESP_LOGE(kLogTag, "Wi-Fi start failed at step %u: %s", start_step_.load(), esp_err_to_name(result));
    start_step_.store(0xEE);
    if (start_watchdog_ != nullptr) (void)esp_timer_stop(static_cast<esp_timer_handle_t>(start_watchdog_));
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
    if (!mass_storage_active_.load()) sensors_.set_radio_transmission_enabled(true);
    return result;
}

void WifiOta::stop_wifi(const char *reason)
{
    ESP_LOGW(kLogTag, "stopping Wi-Fi: %s", reason);
    if (server_ != nullptr) {
        httpd_stop(server_);
        server_ = nullptr;
    }
    (void)esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler);
    esp_wifi_stop();
    esp_wifi_deinit();
    if (ap_netif_ != nullptr) {
        esp_netif_destroy_default_wifi(ap_netif_);
        ap_netif_ = nullptr;
    }
    enabled_.store(false);
    g_wifi_on.store(false);
    if (!mass_storage_active_.load()) {
        sensors_.set_radio_transmission_enabled(true);
    }
    sensors_.set_radio_beacon_only(false);
    if (maintenance_mode::active() && !restart_scheduled_.load()) {
        maintenance_mode::cancel();
        restart_at_ms_.store(now_ms() + kThawRestartDelayMs);
        restart_scheduled_.store(true);
        ESP_LOGW(kLogTag, "leaving maintenance: rebooting into flight mode in %u ms",
                 static_cast<unsigned>(kThawRestartDelayMs));
        return;
    }
    // A log download froze the flash logger until reset; reboot (only when the
    // rocket is on the pad or landed) so logging resumes.
    if (logs_were_frozen_ && !restart_scheduled_.load() && state_allows()) {
        restart_at_ms_.store(now_ms() + kThawRestartDelayMs);
        restart_scheduled_.store(true);
        ESP_LOGW(kLogTag, "logging was frozen for log download: rebooting in %u ms",
                 static_cast<unsigned>(kThawRestartDelayMs));
    }
}

void WifiOta::start_watchdog_expired(void *context)
{
    auto *self = static_cast<WifiOta *>(context);
    if (self->enabled_.load()) return;
    self->watchdog_fired_.store(true);
    if (!self->mass_storage_active_.load()) self->sensors_.set_radio_transmission_enabled(true);
    ESP_LOGE(kLogTag, "Wi-Fi start stuck at step %u for %u s: LoRa transmission restored",
             self->start_step_.load(), static_cast<unsigned>(kStartWatchdogUs / 1000000U));
}

const char *WifiOta::app_state_text()
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (running == nullptr || esp_ota_get_state_partition(running, &state) != ESP_OK) return "unknown";
    switch (state) {
    case ESP_OTA_IMG_NEW: return "new";
    case ESP_OTA_IMG_PENDING_VERIFY: return "pending";
    case ESP_OTA_IMG_VALID: return "valid";
    case ESP_OTA_IMG_INVALID: return "invalid";
    case ESP_OTA_IMG_ABORTED: return "aborted";
    default: return "undefined";
    }
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
    char escaped[96];
    char body[160];
    wifi_ota_logic::json_escape(escaped, sizeof(escaped), message);
    std::snprintf(body, sizeof(body), "{\"ok\":false,\"error\":\"%s\"}", escaped);
    httpd_resp_set_hdr(req, "Connection", "close");
    send_json(req, status, body);
    return ESP_FAIL;
}

esp_err_t WifiOta::freeze_logs()
{
    // Freezing stops flash logging until a reboot, and the thaw reboot is
    // never done in flight. On the pad a launch could follow before that
    // reboot and the flight would go unlogged, so downloads (which freeze)
    // are allowed only in Landed. On the bench: set the flight state to
    // Landed first (0x05 0x73 0x04), or use USB mass storage.
    if (!flash_logger_.frozen() && !maintenance_mode::active() &&
        flight_state_machine_.state() != FlightStateMachine::State::Landed) {
        ESP_LOGW(kLogTag, "log download refused: only in Landed (it freezes flash logging)");
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(log_mutex_, portMAX_DELAY);
    esp_err_t result = ESP_OK;
    if (!flash_logger_.frozen()) {
        ESP_LOGW(kLogTag, "freezing flash logging for log download (reboot restores it)");
        result = flash_logger_.prepare_mass_storage(kLogFreezeTimeout);
        if (result == ESP_OK) {
            logs_were_frozen_ = true;
        } else {
            ESP_LOGE(kLogTag, "cannot freeze logging: %s", esp_err_to_name(result));
        }
    }
    xSemaphoreGive(log_mutex_);
    return result;
}

esp_err_t WifiOta::handle_info(httpd_req_t *req)
{
    auto *self = static_cast<WifiOta *>(req->user_ctx);
    self->touch();
    const esp_app_desc_t *desc = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(nullptr);
    const bool frozen = self->flash_logger_.frozen();
    char version[40];
    wifi_ota_logic::json_escape(version, sizeof(version), desc->version);
    char idf[40];
    wifi_ota_logic::json_escape(idf, sizeof(idf), desc->idf_ver);
    char body[512];
    std::snprintf(body, sizeof(body),
                  "{\"project\":\"%s\",\"version\":\"%s\",\"idf\":\"%s\",\"running\":\"%s\","
                  "\"next\":\"%s\",\"app_state\":\"%s\",\"uptime_ms\":%llu,\"flight_state\":\"%s\","
                  "\"wifi_tx_dbm\":%d,\"free_heap\":%u,\"logging_frozen\":%s,\"log_files\":%d}",
                  desc->project_name, version, idf, running != nullptr ? running->label : "",
                  next != nullptr ? next->label : "", app_state_text(),
                  static_cast<unsigned long long>(esp_timer_get_time() / 1000),
                  FlightStateMachine::state_name(self->flight_state_machine_.state()),
                  self->tx_power_dbm_, static_cast<unsigned>(esp_get_free_heap_size()),
                  frozen ? "true" : "false",
                  frozen ? static_cast<int>(self->flash_logger_.export_file_count()) : -1);
    return self->send_json(req, "200 OK", body);
}

esp_err_t WifiOta::handle_ota(httpd_req_t *req)
{
    auto *self = static_cast<WifiOta *>(req->user_ctx);
    self->touch();
    if (!self->state_allows()) return self->send_error(req, "409 Conflict", "not in Pad Idle or Landed");
    bool expected = false;
    if (!self->ota_busy_.compare_exchange_strong(expected, true)) {
        return self->send_error(req, "409 Conflict", "another update is in progress");
    }
    struct Release {
        std::atomic_bool &flag;
        ~Release() { flag.store(false); }
    } release{self->ota_busy_};

    const std::size_t total = req->content_len;
    if (total == 0) return self->send_error(req, "411 Length Required", "Content-Length required");
    const esp_partition_t *slot = esp_ota_get_next_update_partition(nullptr);
    if (slot == nullptr) return self->send_error(req, "500 Internal Server Error", "no OTA slot");
    if (total > slot->size) return self->send_error(req, "413 Payload Too Large", "image larger than the slot");
    if (total < wifi_ota_logic::kImageHeadLength) {
        return self->send_error(req, "400 Bad Request", "image too short");
    }

    auto *buffer = static_cast<std::uint8_t *>(std::malloc(kTransferChunk));
    if (buffer == nullptr) return self->send_error(req, "503 Service Unavailable", "out of memory");
    struct Free {
        void *p;
        ~Free() { std::free(p); }
    } free_buffer{buffer};

    // Read until the image header and application descriptor are in hand.
    std::size_t have = 0;
    std::size_t received = 0;
    int stalls = 0;
    while (have < wifi_ota_logic::kImageHeadLength) {
        const int n = httpd_req_recv(req, reinterpret_cast<char *>(buffer) + have,
                                     std::min(kTransferChunk - have, total - received));
        if (n == HTTPD_SOCK_ERR_TIMEOUT && ++stalls < 3) continue;
        if (n <= 0) return self->send_error(req, "408 Request Timeout", "upload stalled");
        have += static_cast<std::size_t>(n);
        received += static_cast<std::size_t>(n);
    }
    const wifi_ota_logic::ImageCheck check =
        wifi_ota_logic::check_image_head(buffer, have, kProjectName);
    if (check != wifi_ota_logic::ImageCheck::Ok) {
        ESP_LOGW(kLogTag, "OTA rejected: %s", wifi_ota_logic::image_check_message(check));
        return self->send_error(req, "422 Unprocessable Entity", wifi_ota_logic::image_check_message(check));
    }
    esp_app_desc_t new_desc;
    std::memcpy(&new_desc, buffer + wifi_ota_logic::kAppDescOffset, sizeof(new_desc));

    esp_ota_handle_t handle = 0;
    esp_err_t result = esp_ota_begin(slot, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (result != ESP_OK) {
        return self->send_error(req, "500 Internal Server Error", esp_err_to_name(result));
    }
    ESP_LOGW(kLogTag, "OTA to %s: %u bytes, new version %.31s (flash writes stall both cores briefly)",
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
        if (!self->state_allows() || !self->enabled_.load()) {
            failure = "aborted: flight state changed or Wi-Fi stopping";
            break;
        }
        const int n = httpd_req_recv(req, reinterpret_cast<char *>(buffer),
                                     std::min(kTransferChunk, total - written));
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
    result = esp_ota_set_boot_partition(slot);
    if (result != ESP_OK) {
        return self->send_error(req, "500 Internal Server Error", esp_err_to_name(result));
    }
    ESP_LOGW(kLogTag, "OTA complete in %u ms; boot partition %s", static_cast<unsigned>(self->now_ms() - started_ms),
             slot->label);
    char version[40];
    wifi_ota_logic::json_escape(version, sizeof(version), new_desc.version);
    char body[160];
    std::snprintf(body, sizeof(body),
                  "{\"ok\":true,\"bytes\":%u,\"slot\":\"%s\",\"version\":\"%s\",\"reboot_ms\":%u}",
                  static_cast<unsigned>(written), slot->label, version,
                  static_cast<unsigned>(kRestartDelayMs));
    const esp_err_t sent = self->send_json(req, "200 OK", body);
    // The new image boots into maintenance (Wi-Fi up) so the host can see it
    // pass its self-test; Wi-Fi off from there reboots it into flight mode.
    maintenance_mode::request(self->tx_power_dbm_, 0);
    const Request restart{RequestKind::Restart, 0, 0, static_cast<std::uint16_t>(kRestartDelayMs), false};
    (void)xQueueSend(self->queue_, &restart, 0);
    return sent;
}

esp_err_t WifiOta::handle_logs(httpd_req_t *req)
{
    auto *self = static_cast<WifiOta *>(req->user_ctx);
    self->touch();
    if (self->freeze_logs() != ESP_OK) {
        return self->send_error(req, self->flight_state_machine_.state() == FlightStateMachine::State::Landed ? "503 Service Unavailable" : "409 Conflict", self->flight_state_machine_.state() == FlightStateMachine::State::Landed ? "cannot freeze flash logging" : "log download only in Landed (it freezes flash logging until reboot)");
    }
    const std::size_t count = std::min(self->flash_logger_.export_file_count(),
                                       FlashLogger::kMaximumExportFiles);
    httpd_resp_set_type(req, "application/json");
    char piece[128];
    std::snprintf(piece, sizeof(piece), "{\"logging_frozen\":true,\"count\":%u,\"files\":[",
                  static_cast<unsigned>(count));
    esp_err_t result = httpd_resp_send_chunk(req, piece, HTTPD_RESP_USE_STRLEN);
    for (std::size_t i = 0; i < count && result == ESP_OK; ++i) {
        const FlashLogger::ExportFile &file = self->flash_logger_.export_file(i);
        char name[32];
        wifi_ota_logic::json_escape(name, sizeof(name), file.name);
        std::snprintf(piece, sizeof(piece),
                      "%s{\"name\":\"%s\",\"size\":%u,\"session\":%u,\"slot\":%u}", i == 0 ? "" : ",",
                      name, static_cast<unsigned>(file.size), static_cast<unsigned>(file.generation),
                      static_cast<unsigned>(file.slot));
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
    if (self->freeze_logs() != ESP_OK) {
        return self->send_error(req, self->flight_state_machine_.state() == FlightStateMachine::State::Landed ? "503 Service Unavailable" : "409 Conflict", self->flight_state_machine_.state() == FlightStateMachine::State::Landed ? "cannot freeze flash logging" : "log download only in Landed (it freezes flash logging until reboot)");
    }
    const std::size_t count = std::min(self->flash_logger_.export_file_count(),
                                       FlashLogger::kMaximumExportFiles);
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
        if (!self->enabled_.load() || !self->state_allows()) return ESP_FAIL;
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
    if (!self->state_allows()) return self->send_error(req, "409 Conflict", "not in Pad Idle or Landed");
    const esp_err_t sent = self->send_json(req, "200 OK", "{\"ok\":true,\"reboot_ms\":1000}");
    const Request restart{RequestKind::Restart, 0, 0, static_cast<std::uint16_t>(kRestartDelayMs), false};
    (void)xQueueSend(self->queue_, &restart, 0);
    return sent;
}

esp_err_t WifiOta::handle_wifi_off(httpd_req_t *req)
{
    auto *self = static_cast<WifiOta *>(req->user_ctx);
    const esp_err_t sent = self->send_json(req, "200 OK", "{\"ok\":true,\"wifi\":\"off\"}");
    // Delayed so the reply is out before the access point goes away.
    const Request off{RequestKind::Disable, 0, 0, 300, false};
    (void)xQueueSend(self->queue_, &off, 0);
    return sent;
}
