#include "bit_monitor.h"
#include "command_bridge.h"
#include "cpu_load.h"
#include "can_link.h"
#include "esp_err.h"
#include "esp_system.h"
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
#include "flight_selftest.h"
#include "witness_status.h"

namespace {
// Boot-time starts must degrade, never abort: an abort() in app_main is a
// reboot loop with no sensors, no logging and no telemetry (audit W5). Log the
// failure and let the rest of the flight computer come up.
void note_start(const char *what, const esp_err_t result)
{
    if (result != ESP_OK) {
        ESP_LOGE("main", "%s start failed: %s; continuing without it", what,
                 esp_err_to_name(result));
    }
}
}  // namespace

extern "C" void app_main(void)
{
    // Decide first whether this boot resumes an in-flight state after an MCU
    // reset (see flight_state_store.h).
    flight_state_store::Snapshot resume_snapshot{};
    const bool resumed = flight_state_store::take_boot_snapshot(resume_snapshot);
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
    static UsbSerialEcho usb_serial_echo(
        heartbeat,
        sensors,
        spi_data_forwarder,
        flash_logger,
        flight_state_machine,
        command_bridge);

    note_start("data forwarder", spi_data_forwarder.start());
    const esp_err_t can_result = iris_can.start();
    if (can_result != ESP_OK)
        ESP_LOGW("main", "Iris CAN start failed: %s; GPS and sensing continue",
                 esp_err_to_name(can_result));
    note_start("command bridge", command_bridge.start());
    note_start("USB serial", usb_serial_echo.start());
    // Install the diagnostic log sink before the remaining subsystems start so
    // their initialization output is included in this boot's TXT file.
    note_start("flash logger", flash_logger.start());
    iris_can.set_log_sink(&flash_logger);
    // NVS first: the GPS task reads its stored pin orientation as soon as it
    // starts (bench 2026-11-17: it ran before the init and never saw the value).
    {
        // IMU calibration lives in NVS; without it the IMU runs uncorrected.
        // In a resumed flight never erase NVS (calibration + boot bookkeeping);
        // on a cold boot (pad) a full/foreign NVS is reformatted as before.
        const esp_err_t nvs_result = imu_calibration::init_storage(!resumed);
        if (nvs_result != ESP_OK) {
            ESP_LOGE("main", "NVS init failed: %s; IMU calibration unavailable",
                     esp_err_to_name(nvs_result));
        }
        // A flight image never runs a maintenance request (the maintenance
        // image consumes it): drop a stale one.
        maintenance_mode::cancel();
    }
    gps_uart.set_log_sink(flash_logger);
    const esp_err_t gps_result = gps_uart.start();
    if (gps_result != ESP_OK)
        ESP_LOGW("main", "GPS UART1 start failed: %s; other logging continues",
                 esp_err_to_name(gps_result));
    sensors.set_imu_log_sink(&flash_logger);
    sensors.set_can_link(&iris_can);
    // Sensors::start only fails on resource exhaustion (mutex, SPI bus, sensor
    // task); an IMU, ADC or radio problem is handled inside. Retry a few
    // times (it cleans up after itself), then bring the rest up and restart.
    esp_err_t sensors_result = ESP_FAIL;
    for (int attempt = 0; attempt < 3 && sensors_result != ESP_OK; ++attempt) {
        if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(500));
        sensors_result = sensors.start();
        note_start("sensors", sensors_result);
    }
    note_start("heartbeat", heartbeat.start());
    // Boot confirmation (rollback) and last-good flight slot bookkeeping.
    const esp_err_t selftest_result = flight_selftest::start(witness_status, sensors, flight_state_machine);
    if (selftest_result != ESP_OK)
        ESP_LOGW("main", "flight self-test task start failed: %s", esp_err_to_name(selftest_result));
    // Built-in self test: a low-priority core-0 task that only reports (BIT_PLAN_2026-11-18.md).
    static BitMonitor bit_monitor(sensors, flash_logger, flight_state_machine, gps_uart, witness_status,
                                  spi_data_forwarder, resumed);
    usb_serial_echo.set_bit_monitor(&bit_monitor);
    note_start("BIT", bit_monitor.start());
    static CpuLoadMonitor cpu_load;
    const esp_err_t cpu_result = cpu_load.start();
    if (cpu_result != ESP_OK)
        ESP_LOGW("main", "CPU load monitor start failed: %s", esp_err_to_name(cpu_result));
    if (sensors_result != ESP_OK) {
        // No sensor task means no flight functions at all: the only recovery
        // is a fresh boot. Wait so the log above is seen/recorded (short when
        // resuming an in-flight state), then restart; the state resumes.
        const uint32_t delay_ms = resumed ? 2000U : 10000U;
        ESP_LOGE("main", "sensor task could not be created (%s): restarting in %u ms",
                 esp_err_to_name(sensors_result), static_cast<unsigned>(delay_ms));
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        esp_restart();
    }
}
