#include "radio_power_store.h"

#include "esp_log.h"
#include "hardware_pins.h"
#include "nvs.h"

namespace radio_power_store {
namespace {
constexpr const char *kLogTag = "radio_pwr";
constexpr const char *kNamespace = "radio";
constexpr const char *kKey = "tx_dbm";
constexpr bool in_range(const int dbm)
{
    return dbm >= IRIS_RADIO_TX_POWER_MIN_DBM && dbm <= IRIS_RADIO_TX_POWER_MAX_DBM;
}
}  // namespace

bool load(std::int8_t &tx_dbm)
{
    nvs_handle_t handle = 0;
    if (nvs_open(kNamespace, NVS_READONLY, &handle) != ESP_OK) return false;
    std::int8_t value = 0;
    const esp_err_t result = nvs_get_i8(handle, kKey, &value);
    nvs_close(handle);
    if (result != ESP_OK) return false;
    if (!in_range(value)) {
        ESP_LOGW(kLogTag, "stored TX power %d dBm out of range, ignored", value);
        return false;
    }
    tx_dbm = value;
    return true;
}

esp_err_t save(const std::int8_t tx_dbm)
{
    if (!in_range(tx_dbm)) return ESP_ERR_INVALID_ARG;
    std::int8_t existing = 0;
    if (load(existing) && existing == tx_dbm) return ESP_OK;
    nvs_handle_t handle = 0;
    esp_err_t result = nvs_open(kNamespace, NVS_READWRITE, &handle);
    if (result != ESP_OK) return result;
    result = nvs_set_i8(handle, kKey, tx_dbm);
    if (result == ESP_OK) result = nvs_commit(handle);
    nvs_close(handle);
    if (result != ESP_OK) ESP_LOGE(kLogTag, "save failed: %s", esp_err_to_name(result));
    return result;
}

}  // namespace radio_power_store
