#include "imu_calibration_store.h"

#include <atomic>
#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "task_cores.h"

namespace imu_calibration {
namespace {

constexpr const char *kLogTag = "imu_cal";
constexpr const char *kNamespace = "imu_cal";
constexpr const char *kKey = "cal";
constexpr std::uint32_t kLayoutVersion = 1;

struct Stored {
    std::uint32_t version = kLayoutVersion;
    Calibration calibration{};
};

std::atomic_bool busy{false};
Stored pending{};
bool pending_erase = false;

void save_task(void *)
{
    nvs_handle_t handle = 0;
    esp_err_t result = nvs_open(kNamespace, NVS_READWRITE, &handle);
    if (result == ESP_OK) {
        result = pending_erase ? nvs_erase_key(handle, kKey)
                               : nvs_set_blob(handle, kKey, &pending, sizeof(pending));
        if (result == ESP_ERR_NVS_NOT_FOUND && pending_erase) result = ESP_OK;
        if (result == ESP_OK) result = nvs_commit(handle);
        nvs_close(handle);
    }
    if (result == ESP_OK) {
        ESP_LOGI(kLogTag, "calibration %s NVS", pending_erase ? "erased from" : "saved to");
    } else {
        ESP_LOGE(kLogTag, "calibration NVS %s failed: %s",
                 pending_erase ? "erase" : "save", esp_err_to_name(result));
    }
    busy.store(false);
    vTaskDelete(nullptr);
}

esp_err_t start(const bool erase, const Calibration *calibration)
{
    bool expected = false;
    if (!busy.compare_exchange_strong(expected, true)) {
        return ESP_ERR_INVALID_STATE;
    }
    pending_erase = erase;
    pending = Stored{};
    if (calibration != nullptr) pending.calibration = *calibration;
    if (xTaskCreatePinnedToCore(save_task, "imu_cal_save", 3072, nullptr, 1,
                                nullptr, task_cores::kSystem) != pdPASS) {
        busy.store(false);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

}  // namespace

esp_err_t init_storage()
{
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES ||
        result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(kLogTag, "NVS partition reformatted (%s)", esp_err_to_name(result));
        result = nvs_flash_erase();
        if (result == ESP_OK) result = nvs_flash_init();
    }
    return result;
}

esp_err_t load(Calibration &calibration)
{
    nvs_handle_t handle = 0;
    esp_err_t result = nvs_open(kNamespace, NVS_READONLY, &handle);
    if (result != ESP_OK) {
        return result == ESP_ERR_NVS_NOT_FOUND ? ESP_ERR_NOT_FOUND : result;
    }
    Stored stored{};
    std::size_t length = sizeof(stored);
    result = nvs_get_blob(handle, kKey, &stored, &length);
    nvs_close(handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) return ESP_ERR_NOT_FOUND;
    if (result != ESP_OK) return result;
    if (length != sizeof(stored) || stored.version != kLayoutVersion) {
        return ESP_ERR_NOT_FOUND;
    }
    calibration = stored.calibration;
    return ESP_OK;
}

esp_err_t save_async(const Calibration &calibration)
{
    return start(false, &calibration);
}

esp_err_t erase_async()
{
    return start(true, nullptr);
}

}  // namespace imu_calibration
