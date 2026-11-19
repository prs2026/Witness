#pragma once

#include <atomic>
#include <cstdint>

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

class FlashLogger;

// Wi-Fi access point with a minimal HTTP server for firmware update (OTA) and
// flight-log download. MAINTENANCE image only (the flight image links no Wi-Fi
// code at all). The AP comes up when the maintenance boot starts in Wi-Fi mode
// and stays up until the maintenance session ends (restart into flight:
// Wi-Fi off command over LoRa/USB, HTTP /wifi/off or /reboot, auto-off after
// no HTTP activity, or any reset).
//
// OTA: POST /ota with the raw .bin. The image-info record (image_info.h) says
// whether it is a flight or a maintenance image; it is written to the slot of
// that kind that is NOT in use (flight: not the one the boot returns to;
// maintenance: not the running one) and compatibility with the other kind is
// checked first. See boot_manager.h for how each kind is verified and rolled
// back.
class WifiOta final {
public:
    static constexpr UBaseType_t kTaskPriority = 3;
    static constexpr std::uint32_t kTaskStackSize = 6144;
    static constexpr std::uint32_t kPollIntervalMs = 250;
    // AP failed to start: return to the flight image after this long.
    static constexpr std::uint32_t kFailedStartLeaveMs = 60'000;
    static constexpr std::uint32_t kRestartDelayMs = 1000;
    static constexpr std::size_t kTransferChunk = 4096;
    static constexpr int kDefaultTxPowerDbm = CONFIG_WITNESS_WIFI_TX_POWER_DBM;
    static constexpr std::uint32_t kDefaultAutoOffMinutes = CONFIG_WITNESS_WIFI_AUTO_OFF_MINUTES;

    explicit WifiOta(FlashLogger &flash_logger);
    WifiOta(const WifiOta &) = delete;
    WifiOta &operator=(const WifiOta &) = delete;

    // Starts the task that brings the AP and HTTP server up. auto_off_minutes
    // 0 = never (recovery mode).
    esp_err_t start(int tx_dbm, std::uint32_t auto_off_minutes);
    // Extends the auto-off window (minutes; 0 = never) and counts as activity.
    void keep_alive(std::uint32_t minutes);
    bool ap_up() const { return enabled_.load(); }
    // IRIS_COMMAND_WIFI d0=2 status word: bit 8 = AP up, bit 9 = start failed,
    // low byte = last start step reached (0xEE = start returned an error).
    std::uint16_t status_word() const
    {
        return static_cast<std::uint16_t>((enabled_.load() ? 0x100U : 0U) | (failed_.load() ? 0x200U : 0U) |
                                          start_step_.load());
    }
    // Flight slot the next boot returns to (OTA keeps it out of the way).
    void set_return_slot(int slot) { return_slot_ = slot; }

private:
    static void task_entry(void *context);
    void run();
    esp_err_t start_wifi();
    void touch();
    std::uint32_t now_ms() const;

    // HTTP handlers (httpd task).
    static esp_err_t handle_info(httpd_req_t *req);
    static esp_err_t handle_ota(httpd_req_t *req);
    static esp_err_t handle_logs(httpd_req_t *req);
    static esp_err_t handle_log_file(httpd_req_t *req);
    static esp_err_t handle_reboot(httpd_req_t *req);
    static esp_err_t handle_wifi_off(httpd_req_t *req);
    esp_err_t send_json(httpd_req_t *req, const char *status, const char *json);
    esp_err_t send_error(httpd_req_t *req, const char *status, const char *message);
    esp_err_t wait_logs();

    FlashLogger &flash_logger_;
    TaskHandle_t task_ = nullptr;
    SemaphoreHandle_t log_mutex_ = nullptr;   // serializes export-table reads

    std::atomic_bool enabled_{false};
    std::atomic_bool failed_{false};
    std::atomic_bool ota_busy_{false};
    std::atomic_uint32_t last_activity_ms_{0};
    std::atomic_uint32_t auto_off_ms_{0};
    std::atomic<std::uint8_t> start_step_{0};
    int tx_power_dbm_ = 0;
    int return_slot_ = -1;
    bool netif_ready_ = false;
    httpd_handle_t server_ = nullptr;
    esp_netif_t *ap_netif_ = nullptr;
};
