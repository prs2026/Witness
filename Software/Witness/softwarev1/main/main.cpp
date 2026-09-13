#include "esp_err.h"
#include "usb_serial_echo.h"

extern "C" void app_main(void)
{
    static UsbSerialEcho usb_serial_echo;
    ESP_ERROR_CHECK(usb_serial_echo.start());
}
