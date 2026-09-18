#include "esp_err.h"
#include "heartbeat.h"
#include "sensors.h"
#include "spi_data_forwarder.h"
#include "usb_serial_echo.h"

extern "C" void app_main(void)
{
    static SpiDataForwarder spi_data_forwarder;
    static Heartbeat heartbeat(spi_data_forwarder);
    static UsbSerialEcho usb_serial_echo(heartbeat, spi_data_forwarder);
    static Sensors sensors(spi_data_forwarder);

    ESP_ERROR_CHECK(spi_data_forwarder.start());
    ESP_ERROR_CHECK(usb_serial_echo.start());
    ESP_ERROR_CHECK(sensors.start());
    ESP_ERROR_CHECK(heartbeat.start());
}
