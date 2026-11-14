#include "boot_manager.h"

#include <cstring>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "hardware_pins.h"
#include "nvs.h"
#include "sdkconfig.h"

#ifndef WITNESS_BUILD_ID
#define WITNESS_BUILD_ID 0
#endif

namespace {
constexpr char kLogTag[] = "boot_mgr";
constexpr char kNamespace[] = "wboot";
constexpr char kKeyFlightSlot[] = "fslot";
constexpr char kKeyMaintConfirmed[] = "mconf";

#if CONFIG_WITNESS_IMAGE_MAINTENANCE
constexpr std::uint8_t kOwnKind = IRIS_IMAGE_KIND_MAINTENANCE;
#else
constexpr std::uint8_t kOwnKind = IRIS_IMAGE_KIND_FLIGHT;
#endif

// Directly behind esp_app_desc_t (see image_info.h): the same offset in every
// image, which is how the other image (and the OTA handler) finds it.
__attribute__((section(".rodata_custom_desc"), used)) const image_info::Info g_image_info =
    image_info::make_info(kOwnKind, static_cast<std::uint32_t>(WITNESS_BUILD_ID), IRIS_RADIO_FREQUENCY_HZ,
                          IRIS_RADIO_BANDWIDTH_HZ, IRIS_RADIO_SPREADING_FACTOR, IRIS_RADIO_CODING_RATE,
                          IRIS_RADIO_PREAMBLE_LENGTH);

esp_partition_subtype_t subtype_for(const boot_manager::Kind kind, const int slot)
{
    const int base = kind == boot_manager::Kind::Flight ? 0 : 2;
    return static_cast<esp_partition_subtype_t>(ESP_PARTITION_SUBTYPE_APP_OTA_0 + base + (slot == 0 ? 0 : 1));
}

bool get_u32(const char *key, std::uint32_t &value)
{
    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READONLY, &handle) != ESP_OK) return false;
    const esp_err_t result = nvs_get_u32(handle, key, &value);
    nvs_close(handle);
    return result == ESP_OK;
}

bool put_u32(const char *key, const std::uint32_t value)
{
    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READWRITE, &handle) != ESP_OK) return false;
    esp_err_t result = nvs_set_u32(handle, key, value);
    if (result == ESP_OK) result = nvs_commit(handle);
    nvs_close(handle);
    return result == ESP_OK;
}

void load_slots(const boot_manager::Kind kind, image_info::Slot out[2])
{
    for (int i = 0; i < 2; ++i) {
        const esp_partition_t *partition = boot_manager::slot_partition(kind, i);
        out[i] = image_info::Slot{};
        out[i].present = partition != nullptr;
        out[i].has_info = partition != nullptr && boot_manager::read_info(partition, kind, out[i].info);
    }
}
}  // namespace

namespace boot_manager {

const image_info::Info &own_info()
{
    return g_image_info;
}

bool running_flight()
{
    return kOwnKind == IRIS_IMAGE_KIND_FLIGHT;
}

const esp_partition_t *slot_partition(const Kind kind, const int slot)
{
    return esp_partition_find_first(ESP_PARTITION_TYPE_APP, subtype_for(kind, slot), nullptr);
}

int slot_index(const esp_partition_t *partition)
{
    if (partition == nullptr || partition->type != ESP_PARTITION_TYPE_APP) return -1;
    const int n = static_cast<int>(partition->subtype) - static_cast<int>(ESP_PARTITION_SUBTYPE_APP_OTA_0);
    if (n < 0 || n > 3) return -1;
    return n & 1;
}

int running_slot()
{
    return slot_index(esp_ota_get_running_partition());
}

bool read_info(const esp_partition_t *partition, const Kind kind, image_info::Info &out)
{
    if (partition == nullptr) return false;
    std::uint8_t head[image_info::kHeadLength];
    if (esp_partition_read(partition, 0, head, sizeof(head)) != ESP_OK) return false;
    if (head[0] != 0xE9) return false;   // ESP image header magic
    if (!image_info::parse(head, sizeof(head), out)) return false;
    const std::uint8_t want = kind == Kind::Flight ? IRIS_IMAGE_KIND_FLIGHT : IRIS_IMAGE_KIND_MAINTENANCE;
    return out.kind == want;
}

std::uint16_t image_id(const esp_partition_t *partition)
{
    esp_app_desc_t desc;
    if (partition == nullptr || esp_ota_get_partition_description(partition, &desc) != ESP_OK) return 0;
    return image_info::version_id(desc.version, sizeof(desc.version));
}

std::uint16_t own_image_id()
{
    return image_info::version_id(esp_app_get_description()->version, sizeof(esp_app_get_description()->version));
}

int last_good_flight()
{
    std::uint32_t v = 0;
    if (!get_u32(kKeyFlightSlot, v) || v > 1) return -1;
    return static_cast<int>(v);
}

void note_flight_good()
{
    const int slot = running_slot();
    if (slot < 0 || last_good_flight() == slot) return;
    ESP_LOGI(kLogTag, "flight slot %d recorded as last good", slot);
    (void)put_u32(kKeyFlightSlot, static_cast<std::uint32_t>(slot));
}

std::uint32_t confirmed_maintenance_build()
{
    std::uint32_t v = 0;
    return get_u32(kKeyMaintConfirmed, v) ? v : 0;
}

void note_maintenance_confirmed(const std::uint32_t build_id)
{
    if (confirmed_maintenance_build() == build_id) return;
    ESP_LOGI(kLogTag, "maintenance image build %u confirmed", static_cast<unsigned>(build_id));
    (void)put_u32(kKeyMaintConfirmed, build_id);
}

image_info::Compat check_target(int *slot_out)
{
    image_info::Slot slots[2];
    load_slots(Kind::Maintenance, slots);
    const int pick = image_info::select_maintenance_slot(slots, confirmed_maintenance_build());
    if (slot_out != nullptr) *slot_out = pick;
    if (pick < 0) return image_info::Compat::NoImage;
    return image_info::check_maintenance(&slots[pick].info, own_info());
}

SwitchResult prepare_switch(const maintenance_mode::Kind kind, const int tx_dbm, const std::uint32_t minutes,
                            const std::uint8_t flight_state, const int radio_dbm, const bool radio_front_end)
{
    SwitchResult result;
    int pick = -1;
    const image_info::Compat compat = check_target(&pick);
    if (compat != image_info::Compat::Ok) {
        result.refuse_detail = image_info::refuse_code(compat);
        ESP_LOGW(kLogTag, "switch to maintenance refused: check %u (detail 0x%04X)",
                 static_cast<unsigned>(compat), static_cast<unsigned>(result.refuse_detail));
        return result;
    }
    const esp_partition_t *target = slot_partition(Kind::Maintenance, pick);
    maintenance_mode::Request request;
    request.kind = kind;
    request.tx_dbm = tx_dbm;
    request.minutes = minutes;
    request.flight_slot = running_slot();
    request.flight_state = flight_state;
    request.radio_dbm = radio_dbm;
    request.radio_front_end = radio_front_end;
    maintenance_mode::arm(request);
    const esp_err_t set = esp_ota_set_boot_partition(target);   // validates the image (SHA-256)
    if (set != ESP_OK) {
        maintenance_mode::cancel();
        result.refuse_detail = IRIS_MAINT_REFUSE_SWITCH_FAILED;
        ESP_LOGE(kLogTag, "cannot select %s: %s", target->label, esp_err_to_name(set));
        return result;
    }
    result.ok = true;
    result.target = target;
    ESP_LOGW(kLogTag, "next boot: maintenance slot %s (flight slot %d to return to)", target->label,
             request.flight_slot);
    return result;
}

bool running_pending_verify()
{
    esp_ota_img_states_t state;
    const esp_partition_t *running = esp_ota_get_running_partition();
    return running != nullptr && esp_ota_get_state_partition(running, &state) == ESP_OK &&
           state == ESP_OTA_IMG_PENDING_VERIFY;
}

void mark_running_valid()
{
    // Acts on the newest otadata entry, which is this image's until
    // return_to_flight() has selected the flight slot.
    const esp_err_t result = esp_ota_mark_app_valid_cancel_rollback();
    if (result != ESP_OK) ESP_LOGW(kLogTag, "mark valid: %s", esp_err_to_name(result));
}

int return_to_flight(const int flight_slot_hint)
{
    // 1. This (maintenance) image is the newest otadata entry: confirm it, so
    //    its entry never stays "pending verify" (esp_ota_begin refuses to
    //    write while the running image is pending).
    mark_running_valid();
    // 2. Pick the flight slot: the one that requested us, else the recorded
    //    last-good one, else the newest valid flight image.
    image_info::Slot slots[2];
    load_slots(Kind::Flight, slots);
    int pick = -1;
    if (flight_slot_hint == 0 || flight_slot_hint == 1) {
        if (slots[flight_slot_hint].present && slots[flight_slot_hint].has_info) pick = flight_slot_hint;
    }
    if (pick < 0) pick = image_info::select_flight_slot(slots, last_good_flight());
    if (pick < 0) {
        ESP_LOGE(kLogTag, "no valid flight image installed: staying in maintenance (recovery)");
        return -1;
    }
    const esp_partition_t *flight = slot_partition(Kind::Flight, pick);
    esp_err_t result = esp_ota_set_boot_partition(flight);
    if (result != ESP_OK) {
        // The recorded slot is corrupt: try the other one.
        const int other = image_info::inactive_slot(pick);
        if (slots[other].present && slots[other].has_info) {
            result = esp_ota_set_boot_partition(slot_partition(Kind::Flight, other));
            if (result == ESP_OK) pick = other;
        }
    }
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "cannot select a flight slot: %s", esp_err_to_name(result));
        return -1;
    }
    // The flight image was good before it handed over: mark its fresh entry
    // valid so it does not boot "pending verify" (a crash during the first
    // seconds would roll back).
    mark_running_valid();
    ESP_LOGW(kLogTag, "boot partition restored to flight slot %d: any reset returns to flight", pick);
    return pick;
}

esp_err_t boot_other_maintenance_for_verify(const esp_partition_t *target, const int flight_slot, const int tx_dbm,
                                            const std::uint32_t minutes)
{
    maintenance_mode::Request request;
    request.kind = maintenance_mode::Kind::Wifi;
    request.tx_dbm = tx_dbm;
    request.minutes = minutes;
    request.verify = true;
    request.flight_slot = flight_slot;
    maintenance_mode::arm(request);
    const esp_err_t result = esp_ota_set_boot_partition(target);
    if (result != ESP_OK) maintenance_mode::cancel();
    return result;
}

esp_err_t boot_new_flight(const esp_partition_t *target)
{
    maintenance_mode::cancel();
    return esp_ota_set_boot_partition(target);
}

}  // namespace boot_manager
