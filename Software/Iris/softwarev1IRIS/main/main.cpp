#include <cstdint>

#include "battery_voltage.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hardware.h"
#include "mcp23008.h"
#include "pac1931.h"
#include "usb_serial_echo.h"

extern "C" void app_main(void)
{
    static constexpr char kLogTag[] = "main";
    static constexpr std::int64_t kBatteryReadIntervalUs =
        static_cast<std::int64_t>(
            IRIS_BATTERY_VOLTAGE_READ_INTERVAL_MS) * 1000;

    Mcp23008 gpio_expander;
    ESP_ERROR_CHECK(gpio_expander.initialize());

    constexpr std::uint8_t output_pins[] = {
        IRIS_MCP23008_PIN_OUT1_ENABLE,
        IRIS_MCP23008_PIN_OUT2_ENABLE,
        IRIS_MCP23008_PIN_OUT3_ENABLE,
        IRIS_MCP23008_PIN_5V_ENABLE,
        IRIS_MCP23008_PIN_LED_BLUE,
    };
    for (const std::uint8_t pin : output_pins) {
        ESP_ERROR_CHECK(gpio_expander.configure_output(pin, false));
    }

    Pac1931 current_monitor;
    const esp_err_t current_monitor_result =
        current_monitor.initialize(gpio_expander.bus_handle());
    if (current_monitor_result != ESP_OK) {
        ESP_LOGE(kLogTag, "PAC193x initialization failed: %s",
                 esp_err_to_name(current_monitor_result));
    }

    BatteryVoltage battery_voltage;
    ESP_ERROR_CHECK(battery_voltage.initialize());
    std::int64_t last_battery_read_us = esp_timer_get_time();

    UsbSerialEcho usb_serial_echo(gpio_expander, current_monitor);
    ESP_ERROR_CHECK(usb_serial_echo.initialize());

    for (;;) {
        const std::int64_t now_us = esp_timer_get_time();
        const std::int64_t time_since_battery_read_us =
            now_us - last_battery_read_us;
        if (time_since_battery_read_us >= kBatteryReadIntervalUs) {
            last_battery_read_us = now_us;
            std::uint32_t reading_mv = 0;
            const bool reading_valid =
                battery_voltage.read_millivolts(reading_mv) == ESP_OK;
            usb_serial_echo.update_battery_voltage(
                reading_mv, reading_valid);
        }

        usb_serial_echo.poll();
        vTaskDelay(1);
    }
}
