#include "command_bridge.h"
#include "esp_err.h"
#include "flash_logger.h"
#include "flight_state_machine.h"
#include "heartbeat.h"
#include "sensors.h"
#include "spi_data_forwarder.h"
#include "uart_driver.h"
#include "usb_serial_echo.h"
#include "witness_status.h"

extern "C" void app_main(void)
{
    static SpiDataForwarder spi_data_forwarder;
    static WitnessStatus witness_status;
    static FlightStateMachine flight_state_machine(witness_status);
    static Heartbeat heartbeat(spi_data_forwarder, witness_status);
    static UartDriver uart_driver(spi_data_forwarder);
    static CommandBridge command_bridge(uart_driver);
    static Sensors sensors(
        spi_data_forwarder, witness_status, command_bridge);
    static FlashLogger flash_logger(
        sensors, witness_status, flight_state_machine);
    static UsbSerialEcho usb_serial_echo(
        heartbeat,
        sensors,
        spi_data_forwarder,
        flash_logger,
        flight_state_machine,
        command_bridge);

    ESP_ERROR_CHECK(spi_data_forwarder.start());
    ESP_ERROR_CHECK(usb_serial_echo.start());
    // Install the diagnostic log sink before the remaining subsystems start so
    // their initialization output is included in this boot's TXT file.
    ESP_ERROR_CHECK(flash_logger.start());
    ESP_ERROR_CHECK(uart_driver.start());
    ESP_ERROR_CHECK(sensors.start());
    ESP_ERROR_CHECK(heartbeat.start());
}
