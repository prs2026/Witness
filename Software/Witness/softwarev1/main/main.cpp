#include "esp_err.h"
#include "heartbeat.h"
#include "usb_serial_echo.h"

extern "C" void app_main(void)
{
    static UsbSerialEcho usb_serial_echo;
    static Heartbeat heartbeat;

    ESP_ERROR_CHECK(usb_serial_echo.start());
    ESP_ERROR_CHECK(heartbeat.start());
}
