#include "command_bridge.h"
#include "cpu_load.h"
#include "can_link.h"
#include "esp_err.h"
#include "flash_logger.h"
#include "flight_state_machine.h"
#include "flight_state_store.h"
#include "heartbeat.h"
#include "imu_calibration_store.h"
#include "maintenance_mode.h"
#include "hardware_pins.h"
#include "uart_driver.h"
#include "sensors.h"
#include "spi_data_forwarder.h"
#include "esp_log.h"
#include "usb_serial_echo.h"
#include "wifi_ota.h"
#include "witness_status.h"

extern "C" void app_main(void)
{
    // Decide first whether this boot resumes an in-flight state after an MCU
    // reset (see flight_state_store.h). A resumed flight must never enter a
    // maintenance boot, so any armed maintenance request is dropped before it
    // is consumed.
    flight_state_store::Snapshot resume_snapshot{};
    const bool resumed = flight_state_store::take_boot_snapshot(resume_snapshot);
    if (resumed) {
        maintenance_mode::cancel();
    }
    // Consume the one-shot maintenance request (see maintenance_mode.h).
    maintenance_mode::consume_boot_request();
    const bool maintenance = maintenance_mode::active();
    static SpiDataForwarder spi_data_forwarder;
    static WitnessStatus witness_status;
    static FlightStateMachine flight_state_machine(witness_status);
    if (resumed) {
        flight_state_machine.restore(
            static_cast<FlightStateMachine::State>(resume_snapshot.state),
            resume_snapshot.maximum_altitude_m, resume_snapshot.maximum_valid);
        witness_status.set_state_flag(IRIS_WITNESS_STATUS_RESUMED_MASK, true);
        ESP_LOGE("main", "RESET IN FLIGHT: resumed state '%s' (max altitude %.1f m)",
                 FlightStateMachine::state_name(flight_state_machine.state()),
                 static_cast<double>(resume_snapshot.maximum_altitude_m));
    }
    flight_state_machine.set_persist_callback(flight_state_store::save);
    static Heartbeat heartbeat(spi_data_forwarder, witness_status);
    static UartDriver gps_uart(spi_data_forwarder, UART_NUM_1,
                               HW_PIN_MCU_UART_TX, HW_PIN_MCU_UART_RX);
    static CanLink iris_can(spi_data_forwarder, witness_status);
    static CommandBridge command_bridge(iris_can, spi_data_forwarder);
    static Sensors sensors(
        spi_data_forwarder,
        witness_status,
        command_bridge,
        flight_state_machine, gps_uart);
    static FlashLogger flash_logger(
        sensors, witness_status, flight_state_machine);
    static WifiOta wifi_ota(
        sensors, flash_logger, flight_state_machine, witness_status);
    static UsbSerialEcho usb_serial_echo(
        heartbeat,
        sensors,
        spi_data_forwarder,
        flash_logger,
        flight_state_machine,
        command_bridge,
        wifi_ota);

    if (maintenance) {
        witness_status.set_state_flag(IRIS_WITNESS_STATUS_MAINTENANCE_MASK, true);
        ESP_LOGW("main", "MAINTENANCE BOOT: Wi-Fi OTA/log download; flight logging OFF; NOT FLIGHT READY");
    }
    ESP_ERROR_CHECK(spi_data_forwarder.start());
    const esp_err_t can_result = iris_can.start();
    if (can_result != ESP_OK)
        ESP_LOGW("main", "Iris CAN start failed: %s; GPS and sensing continue",
                 esp_err_to_name(can_result));
    ESP_ERROR_CHECK(command_bridge.start());
    ESP_ERROR_CHECK(usb_serial_echo.start());
    // Install the diagnostic log sink before the remaining subsystems start so
    // their initialization output is included in this boot's TXT file.
    // Maintenance: read-only export mount; the write path (and its RAM) is
    // never created, which leaves room for Wi-Fi.
    ESP_ERROR_CHECK(maintenance ? flash_logger.start_export_only() : flash_logger.start());
    iris_can.set_log_sink(&flash_logger);
    gps_uart.set_log_sink(flash_logger);
    const esp_err_t gps_result = gps_uart.start();
    if (gps_result != ESP_OK)
        ESP_LOGW("main", "GPS UART1 start failed: %s; other logging continues",
                 esp_err_to_name(gps_result));
    sensors.set_imu_log_sink(&flash_logger);
    {
        // IMU calibration lives in NVS; without it the IMU runs uncorrected.
        const esp_err_t nvs_result = imu_calibration::init_storage();
        if (nvs_result != ESP_OK) {
            ESP_LOGE("main", "NVS init failed: %s; IMU calibration unavailable",
                     esp_err_to_name(nvs_result));
        }
    }
    ESP_ERROR_CHECK(sensors.start());
    ESP_ERROR_CHECK(heartbeat.start());
    // Wi-Fi stays off until commanded; this starts the control task and the
    // pending-verify self-test of a freshly updated image.
    const esp_err_t wifi_result = wifi_ota.start();
    if (wifi_result != ESP_OK)
        ESP_LOGW("main", "Wi-Fi OTA task start failed: %s", esp_err_to_name(wifi_result));
    static CpuLoadMonitor cpu_load;
    const esp_err_t cpu_result = cpu_load.start();
    if (cpu_result != ESP_OK)
        ESP_LOGW("main", "CPU load monitor start failed: %s", esp_err_to_name(cpu_result));
}
