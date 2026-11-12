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
// Layout 2 added the high-g scale. A layout-1 blob (no scale) still loads,
// with the scale at 1, so an existing gyro/accelerometer calibration survives
// the update.
constexpr std::uint32_t kLayoutVersion = 2;

struct Stored {
    std::uint32_t version = kLayoutVersion;
    Calibration calibration{};
};

struct CalibrationV1 {
    float gyro_bias_mdps[3]{};
    float accel_offset_mg[3]{};
    float accel_scale[3]{1.0F, 1.0F, 1.0F};
    float high_g_offset_mg[3]{};
    bool gyro_valid = false;
    bool accel_valid = false;
};

struct StoredV1 {
    std::uint32_t version = 1;
    CalibrationV1 calibration{};
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
    std::size_t length = 0;
    result = nvs_get_blob(handle, kKey, nullptr, &length);
    if (result != ESP_OK) {
        nvs_close(handle);
        return result == ESP_ERR_NVS_NOT_FOUND ? ESP_ERR_NOT_FOUND : result;
    }
    if (length == sizeof(Stored)) {
        Stored stored{};
        result = nvs_get_blob(handle, kKey, &stored, &length);
        nvs_close(handle);
        if (result != ESP_OK) return result;
        if (stored.version != kLayoutVersion) return ESP_ERR_NOT_FOUND;
        calibration = stored.calibration;
        return ESP_OK;
    }
    if (length == sizeof(StoredV1)) {
        StoredV1 old{};
        result = nvs_get_blob(handle, kKey, &old, &length);
        nvs_close(handle);
        if (result != ESP_OK) return result;
        if (old.version != 1) return ESP_ERR_NOT_FOUND;
        calibration = Calibration{};
        for (int axis = 0; axis < 3; ++axis) {
            calibration.gyro_bias_mdps[axis] = old.calibration.gyro_bias_mdps[axis];
            calibration.accel_offset_mg[axis] = old.calibration.accel_offset_mg[axis];
            calibration.accel_scale[axis] = old.calibration.accel_scale[axis];
            calibration.high_g_offset_mg[axis] = old.calibration.high_g_offset_mg[axis];
        }
        calibration.gyro_valid = old.calibration.gyro_valid;
        calibration.accel_valid = old.calibration.accel_valid;
        ESP_LOGI(kLogTag, "calibration layout 1 loaded (high-g scale 1 until the next six-face calibration)");
        return ESP_OK;
    }
    nvs_close(handle);
    return ESP_ERR_NOT_FOUND;
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
