#include "esp_err.h"
#include "flash_logger.h"
#include "flight_state_machine.h"
#include "heartbeat.h"
#include "sensors.h"
#include "spi_data_forwarder.h"
#include "twai_driver.h"
#include "usb_serial_echo.h"
#include "witness_status.h"

extern "C" void app_main(void)
{
    static SpiDataForwarder spi_data_forwarder;
    static WitnessStatus witness_status;
    static FlightStateMachine flight_state_machine(witness_status);
    static Heartbeat heartbeat(spi_data_forwarder, witness_status);
    static Sensors sensors(spi_data_forwarder, witness_status);
    static TwaiDriver twai_driver(spi_data_forwarder);
    static FlashLogger flash_logger(
        sensors, witness_status, flight_state_machine);
    static UsbSerialEcho usb_serial_echo(
        heartbeat,
        sensors,
        spi_data_forwarder,
        flash_logger,
        flight_state_machine);

    ESP_ERROR_CHECK(spi_data_forwarder.start());
    ESP_ERROR_CHECK(usb_serial_echo.start());
    // Install the diagnostic log sink before the remaining subsystems start so
    // their initialization output is included in this boot's TXT file.
    ESP_ERROR_CHECK(flash_logger.start());
    ESP_ERROR_CHECK(twai_driver.start());
    ESP_ERROR_CHECK(sensors.start());
    ESP_ERROR_CHECK(heartbeat.start());
}
