#pragma once

#include <cmath>
#include <cstddef>
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
    // ISA (1976) layers to 47 km (geopotential), inverted per layer. The single-layer
    // troposphere formula used before read about 1.1 km low at 20 km.
    struct Layer {
        float base_m, base_k, lapse_k_per_m, base_pa;
    };
    static constexpr Layer kLayers[] = {
        {0.0F, 288.15F, -0.0065F, 101325.0F},
        {11000.0F, 216.65F, 0.0F, 22632.06F},
        {20000.0F, 216.65F, 0.001F, 5474.889F},
        {32000.0F, 228.65F, 0.0028F, 868.0187F},
        {47000.0F, 270.65F, 0.0F, 110.9063F},
    };
    constexpr float kGasOverG = 287.05287F / 9.80665F;  // R / g0, m/K
    // Scale so the 1013.25 mbar reference maps to 0 m exactly.
    const float pressure_pa =
        pressure_mbar * (101325.0F / (kStandardSeaLevelPressureMbar * 100.0F)) * 100.0F;
    std::size_t i = 0;
    while (i + 1 < sizeof(kLayers) / sizeof(kLayers[0]) &&
           pressure_pa < kLayers[i + 1].base_pa) {
        ++i;
    }
    const Layer &layer = kLayers[i];
    const float ratio = pressure_pa / layer.base_pa;
    float geopotential_m = 0.0F;
    if (layer.lapse_k_per_m == 0.0F) {
        geopotential_m = layer.base_m - kGasOverG * layer.base_k * std::log(ratio);
    } else {
        geopotential_m = layer.base_m +
            layer.base_k / layer.lapse_k_per_m *
                (std::pow(ratio, -layer.lapse_k_per_m * kGasOverG) - 1.0F);
    }
    // The ISA is defined in geopotential altitude; GPS and the position
    // filter use geometric altitude. h = r H / (r - H) adds about h^2 / r:
    // 0.3 m at 1.4 km, 45 m at 17 km, 250 m at 38 km.
    constexpr float kEarthRadiusM = 6356766.0F;
    altitude_meters = kEarthRadiusM * geopotential_m / (kEarthRadiusM - geopotential_m);
    return true;
}
