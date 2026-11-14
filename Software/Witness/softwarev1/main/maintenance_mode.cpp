#include "maintenance_mode.h"

#include "esp_attr.h"
#include "nvs.h"
#include "esp_system.h"
#include "esp_timer.h"

namespace {

constexpr std::uint32_t kMagic = 0x4D41494FU;   // "MAIO"
constexpr char kNamespace[] = "wboot";
constexpr char kKey[] = "req";

// The request is passed in NVS, not RTC memory: the two images do not share an
// RTC layout (RTC_NOINIT placement depends on what else each image links), but
// both read the same NVS namespace. Written by the flight image right before
// the restart, erased by the maintenance image as the first thing it reads.
struct Blob {
    std::uint32_t magic;
    std::uint32_t kind;
    std::int32_t tx_dbm;
    std::uint32_t minutes;
    std::uint32_t verify;
    std::int32_t flight_slot;
    std::uint32_t flight_state;
    std::int32_t radio_dbm;
    std::uint32_t radio_front_end;
    std::uint32_t check;   // ~magic ^ kind ^ minutes
};

std::uint32_t check_of(const Blob &b)
{
    return ~b.magic ^ b.kind ^ b.minutes ^ static_cast<std::uint32_t>(b.flight_slot);
}

esp_timer_handle_t s_restart_timer = nullptr;

void restart_now(void *)
{
    esp_restart();
}

}  // namespace

namespace maintenance_mode {

void arm(const Request &request)
{
    Blob blob{};
    blob.magic = kMagic;
    blob.kind = static_cast<std::uint32_t>(request.kind);
    blob.tx_dbm = request.tx_dbm;
    blob.minutes = request.minutes;
    blob.verify = request.verify ? 1U : 0U;
    blob.flight_slot = request.flight_slot;
    blob.flight_state = request.flight_state;
    blob.radio_dbm = request.radio_dbm;
    blob.radio_front_end = request.radio_front_end ? 1U : 0U;
    blob.check = check_of(blob);
    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READWRITE, &handle) != ESP_OK) return;
    if (nvs_set_blob(handle, kKey, &blob, sizeof(blob)) == ESP_OK) (void)nvs_commit(handle);
    nvs_close(handle);
}

bool consume(Request &out)
{
    // Read and erase first, so a request is never honoured twice.
    Blob blob{};
    bool armed = false;
    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READWRITE, &handle) == ESP_OK) {
        std::size_t size = sizeof(blob);
        armed = nvs_get_blob(handle, kKey, &blob, &size) == ESP_OK && size == sizeof(blob) &&
                blob.magic == kMagic && blob.check == check_of(blob);
        if (nvs_erase_key(handle, kKey) == ESP_OK) (void)nvs_commit(handle);
        nvs_close(handle);
    }
    // Only a deliberate esp_restart() may enter maintenance: a request left
    // armed (e.g. a cancelled reboot) must not turn a later panic, watchdog or
    // brownout reset into a maintenance session.
    const bool honoured = armed && esp_reset_reason() == ESP_RST_SW;
    if (honoured) {
        out.kind = blob.kind == static_cast<std::uint32_t>(Kind::UsbExport) ? Kind::UsbExport : Kind::Wifi;
        out.tx_dbm = static_cast<int>(blob.tx_dbm);
        out.minutes = blob.minutes;
        out.verify = blob.verify != 0U;
        out.flight_slot = (blob.flight_slot == 0 || blob.flight_slot == 1) ? static_cast<int>(blob.flight_slot) : -1;
        out.flight_state = static_cast<std::uint8_t>(blob.flight_state);
        out.radio_dbm = static_cast<int>(blob.radio_dbm);
        out.radio_front_end = blob.radio_front_end != 0U;
    }
    return honoured;
}

void cancel()
{
    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READWRITE, &handle) != ESP_OK) return;
    if (nvs_erase_key(handle, kKey) == ESP_OK) (void)nvs_commit(handle);
    nvs_close(handle);
}

bool restart_after_ms(const std::uint32_t delay_ms)
{
    if (s_restart_timer == nullptr) {
        esp_timer_create_args_t args{};
        args.callback = &restart_now;
        args.name = "maint_restart";
        if (esp_timer_create(&args, &s_restart_timer) != ESP_OK) return false;
    }
    (void)esp_timer_stop(s_restart_timer);
    return esp_timer_start_once(s_restart_timer, static_cast<std::uint64_t>(delay_ms) * 1000U) == ESP_OK;
}

}  // namespace maintenance_mode
