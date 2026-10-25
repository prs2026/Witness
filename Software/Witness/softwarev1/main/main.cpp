#include "command_bridge.h"
#include "esp_err.h"
#include "flash_logger.h"
#include "flight_state_machine.h"
#include "heartbeat.h"
#include "hardware_pins.h"
#include "uart_driver.h"
#include "sensors.h"
#include "spi_data_forwarder.h"
#include "esp_log.h"
#include "usb_serial_echo.h"
#include "witness_status.h"

extern "C" void app_main(void)
{
    static SpiDataForwarder spi_data_forwarder;
    static WitnessStatus witness_status;
    static FlightStateMachine flight_state_machine(witness_status);
    static Heartbeat heartbeat(spi_data_forwarder, witness_status);
    static UartDriver gps_uart(spi_data_forwarder, UART_NUM_1,
                               HW_PIN_MCU_UART_TX, HW_PIN_MCU_UART_RX);
    static CommandBridge command_bridge(gps_uart, spi_data_forwarder);
    static Sensors sensors(
        spi_data_forwarder,
        witness_status,
        command_bridge,
        flight_state_machine, gps_uart);
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
    ESP_ERROR_CHECK(command_bridge.start());
    ESP_ERROR_CHECK(usb_serial_echo.start());
    // Install the diagnostic log sink before the remaining subsystems start so
    // their initialization output is included in this boot's TXT file.
    ESP_ERROR_CHECK(flash_logger.start());
    gps_uart.set_log_sink(flash_logger);
    const esp_err_t gps_result = gps_uart.start();
    if (gps_result != ESP_OK)
        ESP_LOGW("main", "GPS UART1 start failed: %s; other logging continues",
                 esp_err_to_name(gps_result));
    ESP_ERROR_CHECK(sensors.start());
    ESP_ERROR_CHECK(heartbeat.start());
}
