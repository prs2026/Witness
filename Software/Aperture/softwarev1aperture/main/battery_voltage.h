#pragma once

#include <cstdint>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

class BatteryVoltage final {
public:
    BatteryVoltage() = default;
    ~BatteryVoltage();

    BatteryVoltage(const BatteryVoltage &) = delete;
    BatteryVoltage &operator=(const BatteryVoltage &) = delete;

    esp_err_t initialize();

    // Returns the divider input (battery) voltage, not the ADC pin voltage.
    esp_err_t read_millivolts(std::uint32_t &battery_millivolts);

private:
    static constexpr adc_atten_t kAttenuation = ADC_ATTEN_DB_12;

    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    adc_cali_handle_t calibration_handle_ = nullptr;
    adc_unit_t unit_ = ADC_UNIT_1;
    adc_channel_t channel_ = ADC_CHANNEL_0;
};
