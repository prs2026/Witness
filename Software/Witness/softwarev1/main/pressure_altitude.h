#pragma once

#include <cmath>
#include <cstdint>

// Barometric altitude used by both the MS5607 driver and HITL pressure input,
// so simulated pressure produces exactly the altitude the real sensor would.
// Pressure is centi-millibar (numerically pascals). The standard atmosphere
// is referenced to 1013.25 mbar, not to a launch-site zero.
constexpr float kStandardSeaLevelPressureMbar = 1013.25F;

// Returns false (and leaves altitude unchanged) for non-positive pressure.
inline bool pressure_altitude_meters(const std::int32_t pressure_centi_mbar,
                                     float &altitude_meters)
{
    const float pressure_mbar = static_cast<float>(pressure_centi_mbar) / 100.0F;
    if (!(pressure_mbar > 0.0F)) {
        return false;
    }
    altitude_meters = 44330.0F *
        (1.0F - std::pow(pressure_mbar / kStandardSeaLevelPressureMbar, 0.19029495F));
    return true;
}
