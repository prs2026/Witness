#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hardware.h"
#include "mcp23008.h"
#include "usb_serial_echo.h"

extern "C" void app_main(void)
{
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

    UsbSerialEcho usb_serial_echo(gpio_expander);
    ESP_ERROR_CHECK(usb_serial_echo.initialize());

    for (;;) {
        usb_serial_echo.poll();
        vTaskDelay(1);
    }
}
