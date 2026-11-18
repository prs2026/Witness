#pragma once

// NVS persistence for imu_calibration::Calibration (namespace "imu_cal").

#include <cstdint>

#include "esp_err.h"
#include "imu_calibration.h"

namespace imu_calibration {

// Initializes NVS. When the partition is full/unreadable (NO_FREE_PAGES) or
// from another IDF version (NEW_VERSION_FOUND) it is reformatted ONLY if
// allow_erase is true. A flight image that has resumed an in-flight state
// passes false: the erase costs the calibration and the boot bookkeeping and
// freezes both cores for a long time. NVS then stays unavailable (every user
// tolerates that) until the next cold boot, where the reformat happens on the
// pad. Returns the NVS error when it was not reformatted.
esp_err_t init_storage(bool allow_erase = true);
// ESP_ERR_NOT_FOUND when nothing (or an older layout) is stored. A stored
// calibration that fails sanitize() is partly or wholly replaced by identity
// (see below) and counted in rejected_sections().
esp_err_t load(Calibration &calibration);
// Number of calibration sections (gyro / accelerometer) rejected on load.
std::uint32_t rejected_sections();

// Sanity limits for a LOADED calibration (looser than the capture solver's
// Limits, so anything the solver accepted is accepted here).
struct SanityLimits {
    float max_gyro_bias_mdps = 30000.0F;
    float max_accel_offset_mg = 500.0F;
    float min_scale = 0.8F;
    float max_scale = 1.25F;
    float max_high_g_offset_mg = 6000.0F;
};

namespace detail {
inline bool finite_in(const float value, const float low, const float high)
{
    return value >= low && value <= high;  // false for NaN
}
}  // namespace detail

// Replaces any section that holds a non-finite, out-of-range scale or an
// implausible offset by identity (valid flag cleared) and returns how many
// sections (0..2; gyro, accelerometer incl. high-g) were rejected. A scale of
// 0 or NaN would otherwise divide the flight data by zero.
inline std::uint32_t sanitize(Calibration &c, const SanityLimits &limits = SanityLimits{})
{
    std::uint32_t rejected = 0;
    bool gyro_ok = true;
    bool accel_ok = true;
    for (int axis = 0; axis < 3; ++axis) {
        gyro_ok = gyro_ok && detail::finite_in(c.gyro_bias_mdps[axis], -limits.max_gyro_bias_mdps,
                                               limits.max_gyro_bias_mdps);
        accel_ok = accel_ok &&
                   detail::finite_in(c.accel_offset_mg[axis], -limits.max_accel_offset_mg,
                                     limits.max_accel_offset_mg) &&
                   detail::finite_in(c.accel_scale[axis], limits.min_scale, limits.max_scale) &&
                   detail::finite_in(c.high_g_offset_mg[axis], -limits.max_high_g_offset_mg,
                                     limits.max_high_g_offset_mg) &&
                   detail::finite_in(c.high_g_scale[axis], limits.min_scale, limits.max_scale);
    }
    if (!gyro_ok) {
        for (int axis = 0; axis < 3; ++axis) c.gyro_bias_mdps[axis] = 0.0F;
        c.gyro_valid = false;
        ++rejected;
    }
    if (!accel_ok) {
        for (int axis = 0; axis < 3; ++axis) {
            c.accel_offset_mg[axis] = 0.0F;
            c.accel_scale[axis] = 1.0F;
            c.high_g_offset_mg[axis] = 0.0F;
            c.high_g_scale[axis] = 1.0F;
        }
        c.accel_valid = false;
        ++rejected;
    }
    return rejected;
}
// Writes from a short-lived core-0 task so the flash write never runs in the
// core-1 sensor task. Returns ESP_ERR_INVALID_STATE while a save is running.
esp_err_t save_async(const Calibration &calibration);
// Removes the stored calibration (also asynchronous, same task).
esp_err_t erase_async();

}  // namespace imu_calibration
