#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb_serial_echo.h"

extern "C" void app_main(void)
{
    UsbSerialEcho usb_serial_echo;
    ESP_ERROR_CHECK(usb_serial_echo.initialize());

    for (;;) {
        usb_serial_echo.poll();
        vTaskDelay(1);
    }
}
