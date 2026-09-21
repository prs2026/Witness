#include "esp_err.h"
#include "flash_logger.h"
#include "heartbeat.h"
#include "sensors.h"
#include "spi_data_forwarder.h"
#include "usb_serial_echo.h"
#include "witness_status.h"

extern "C" void app_main(void)
{
    static SpiDataForwarder spi_data_forwarder;
    static WitnessStatus witness_status;
    static Heartbeat heartbeat(spi_data_forwarder, witness_status);
    static Sensors sensors(spi_data_forwarder, witness_status);
    static FlashLogger flash_logger(sensors, witness_status);
    static UsbSerialEcho usb_serial_echo(
        heartbeat, sensors, spi_data_forwarder, flash_logger);

    ESP_ERROR_CHECK(spi_data_forwarder.start());
    ESP_ERROR_CHECK(usb_serial_echo.start());
    ESP_ERROR_CHECK(sensors.start());
    ESP_ERROR_CHECK(flash_logger.start());
    ESP_ERROR_CHECK(heartbeat.start());
}
