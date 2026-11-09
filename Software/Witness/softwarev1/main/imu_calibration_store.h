#pragma once

// NVS persistence for imu_calibration::Calibration (namespace "imu_cal").

#include "esp_err.h"
#include "imu_calibration.h"

namespace imu_calibration {

// Initializes NVS (erasing it if its format is from another IDF version).
esp_err_t init_storage();
// ESP_ERR_NOT_FOUND when nothing (or an older layout) is stored.
esp_err_t load(Calibration &calibration);
// Writes from a short-lived core-0 task so the flash write never runs in the
// core-1 sensor task. Returns ESP_ERR_INVALID_STATE while a save is running.
esp_err_t save_async(const Calibration &calibration);
// Removes the stored calibration (also asynchronous, same task).
esp_err_t erase_async();

}  // namespace imu_calibration
