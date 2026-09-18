#include "battery_voltage.h"

#include <cstdint>

#include "esp_adc/adc_cali_scheme.h"
#include "hardware.h"
#include "soc/soc_caps.h"

BatteryVoltage::~BatteryVoltage()
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (calibration_handle_ != nullptr) {
        (void)adc_cali_delete_scheme_curve_fitting(calibration_handle_);
    }
#endif
    if (adc_handle_ != nullptr) {
        (void)adc_oneshot_del_unit(adc_handle_);
    }
}

esp_err_t BatteryVoltage::initialize()
{
    if (adc_handle_ != nullptr || calibration_handle_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t result = adc_oneshot_io_to_channel(
        IRIS_PIN_VSENSE, &unit_, &channel_);
    if (result != ESP_OK) {
        return result;
    }

    adc_oneshot_unit_init_cfg_t unit_config{};
    unit_config.unit_id = unit_;
    result = adc_oneshot_new_unit(&unit_config, &adc_handle_);
    if (result != ESP_OK) {
        adc_handle_ = nullptr;
        return result;
    }

    adc_oneshot_chan_cfg_t channel_config{};
    channel_config.atten = kAttenuation;
    channel_config.bitwidth = ADC_BITWIDTH_DEFAULT;
    result = adc_oneshot_config_channel(
        adc_handle_, channel_, &channel_config);
    if (result != ESP_OK) {
        (void)adc_oneshot_del_unit(adc_handle_);
        adc_handle_ = nullptr;
        return result;
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t calibration_config{};
    calibration_config.unit_id = unit_;
    calibration_config.chan = channel_;
    calibration_config.atten = kAttenuation;
    calibration_config.bitwidth = ADC_BITWIDTH_DEFAULT;
    result = adc_cali_create_scheme_curve_fitting(
        &calibration_config, &calibration_handle_);
    if (result != ESP_OK) {
        calibration_handle_ = nullptr;
        (void)adc_oneshot_del_unit(adc_handle_);
        adc_handle_ = nullptr;
    }
    return result;
#else
    (void)adc_oneshot_del_unit(adc_handle_);
    adc_handle_ = nullptr;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t BatteryVoltage::read_millivolts(
    std::uint32_t &battery_millivolts)
{
    if (adc_handle_ == nullptr || calibration_handle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    std::int64_t raw_sum = 0;
    for (std::uint32_t sample = 0;
         sample < IRIS_BATTERY_ADC_SAMPLE_COUNT;
         ++sample) {
        int raw = 0;
        const esp_err_t result =
            adc_oneshot_read(adc_handle_, channel_, &raw);
        if (result != ESP_OK) {
            return result;
        }
        raw_sum += raw;
    }

    const int average_raw = static_cast<int>(
        (raw_sum + IRIS_BATTERY_ADC_SAMPLE_COUNT / 2U) /
        IRIS_BATTERY_ADC_SAMPLE_COUNT);
    int divider_millivolts = 0;
    const esp_err_t result = adc_cali_raw_to_voltage(
        calibration_handle_, average_raw, &divider_millivolts);
    if (result != ESP_OK) {
        return result;
    }

    constexpr std::uint32_t kDividerNumerator =
        IRIS_BATTERY_DIVIDER_HIGH_OHMS + IRIS_BATTERY_DIVIDER_LOW_OHMS;
    constexpr std::uint32_t kDividerDenominator =
        IRIS_BATTERY_DIVIDER_LOW_OHMS;
    battery_millivolts =
        (static_cast<std::uint32_t>(divider_millivolts) *
             kDividerNumerator +
         kDividerDenominator / 2U) /
        kDividerDenominator;
    return ESP_OK;
}
