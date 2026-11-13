#pragma once

#include <atomic>
#include <cstdint>

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

class Sensors;
class FlashLogger;
class FlightStateMachine;
class WitnessStatus;

// Short state text for periodic log lines ("off", "on"); safe from any task.
const char *wifi_ota_state_text();

// Wi-Fi access point with a minimal HTTP server for firmware update (OTA) and
// flight-log download, plus the boot-time OTA self-test that confirms a new
// image or rolls it back. Wi-Fi never starts by itself: IRIS_COMMAND_WIFI
// (USB or radio, Pad Idle or Landed) turns it on. One core-0 task owns every
// Wi-Fi start/stop so the 960 Hz sensor task on core 1 is never involved.
// While Wi-Fi is on all LoRa transmission is stopped (reception continues).
class WifiOta final {
public:
    static constexpr UBaseType_t kTaskPriority = 3;
    static constexpr std::uint32_t kTaskStackSize = 5120;
    static constexpr std::uint32_t kPollIntervalMs = 250;
    // The new image must pass its self-test within this time after boot or the
    // bootloader rolls back to the previous slot.
    static constexpr std::uint32_t kSelfTestWindowMs = 30000;
    static constexpr std::uint32_t kRestartDelayMs = 1000;
    // Delay between acknowledging a radio enable command and cutting LoRa
    // transmission, so the ack is on the air first.
    static constexpr std::uint32_t kRadioAckGuardMs = 600;
    static constexpr std::size_t kTransferChunk = 4096;
    static constexpr int kDefaultTxPowerDbm = CONFIG_WITNESS_WIFI_TX_POWER_DBM;
    static constexpr std::uint32_t kDefaultAutoOffMinutes = CONFIG_WITNESS_WIFI_AUTO_OFF_MINUTES;

    WifiOta(Sensors &sensors, FlashLogger &flash_logger,
            FlightStateMachine &flight_state_machine, WitnessStatus &witness_status);
    WifiOta(const WifiOta &) = delete;
    WifiOta &operator=(const WifiOta &) = delete;

    // Starts the control task (and the OTA self-test when this boot is a
    // pending-verify image). Wi-Fi itself stays off.
    esp_err_t start();

    // Validates and queues an enable (IRIS_ACK_OK, REFUSED, BUSY); tx_dbm and
    // minutes are already resolved (wifi_ota_logic::parse_wifi_command). radio: the command came over LoRa, so the
    // start is delayed until the caller's ack has been transmitted.
    std::uint8_t request_enable(int tx_dbm, std::uint32_t minutes, bool radio);
    // Turns Wi-Fi off and waits (up to timeout) until it is off and LoRa
    // transmission is restored. True when off.
    bool request_disable_and_wait(TickType_t timeout);
    // Mass storage took over USB (and stopped the radio): Wi-Fi is stopped and
    // never re-enables LoRa transmission.
    void note_mass_storage_active();

    bool enabled() const { return enabled_.load(); }
    // Diagnostics, readable without the control task (IRIS_COMMAND_WIFI d0=2):
    // bit 8 = AP up, bit 9 = start watchdog fired, low byte = last start step
    // reached (StartStep), 0xEE = start returned an error.
    std::uint16_t status_word() const
    {
        return static_cast<std::uint16_t>((enabled_.load() ? 0x100U : 0U) |
                                          (watchdog_fired_.load() ? 0x200U : 0U) | start_step_.load());
    }

private:
    enum class RequestKind : std::uint8_t { Enable, Disable, Restart };
    struct Request {
        RequestKind kind;
        std::int8_t tx_dbm;
        std::uint16_t minutes;
        std::uint16_t delay_ms;
        bool wake_waiter;
    };

    static void task_entry(void *context);
    void run();
    void handle_request(const Request &request);
    void monitor();
    void self_test_step();
    bool self_test_passed() const;
    esp_err_t start_wifi(int tx_dbm, std::uint32_t minutes);
    void stop_wifi(const char *reason);
    void touch();
    std::uint32_t now_ms() const;
    bool state_allows() const;

    // HTTP handlers (core 0, httpd task).
    static esp_err_t handle_info(httpd_req_t *req);
    static esp_err_t handle_ota(httpd_req_t *req);
    static esp_err_t handle_logs(httpd_req_t *req);
    static esp_err_t handle_log_file(httpd_req_t *req);
    static esp_err_t handle_reboot(httpd_req_t *req);
    static esp_err_t handle_wifi_off(httpd_req_t *req);
    esp_err_t send_json(httpd_req_t *req, const char *status, const char *json);
    esp_err_t send_error(httpd_req_t *req, const char *status, const char *message);
    esp_err_t freeze_logs();
    static const char *app_state_text();

    Sensors &sensors_;
    FlashLogger &flash_logger_;
    FlightStateMachine &flight_state_machine_;
    WitnessStatus &witness_status_;

    TaskHandle_t task_ = nullptr;
    QueueHandle_t queue_ = nullptr;
    SemaphoreHandle_t done_ = nullptr;       // given when a waited request finishes
    SemaphoreHandle_t log_mutex_ = nullptr;  // serializes export-table reads

    std::atomic_bool enabled_{false};
    std::atomic_bool mass_storage_active_{false};
    std::atomic_bool ota_busy_{false};
    std::atomic_bool restart_scheduled_{false};
    std::atomic_uint32_t last_activity_ms_{0};
    std::atomic_uint32_t restart_at_ms_{0};
    std::atomic<std::uint8_t> start_step_{0};
    std::atomic_bool watchdog_fired_{false};
    void *start_watchdog_ = nullptr;   // esp_timer_handle_t
    static void start_watchdog_expired(void *context);
    std::uint32_t auto_off_ms_ = 0;
    int tx_power_dbm_ = 0;
    bool netif_ready_ = false;
    bool logs_were_frozen_ = false;   // a download session froze logging
    bool self_test_pending_ = false;
    httpd_handle_t server_ = nullptr;
    esp_netif_t *ap_netif_ = nullptr;
};
