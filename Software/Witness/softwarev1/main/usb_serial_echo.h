#pragma once

#include <cstddef>
#include <cstdint>

#include "comms.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb_mass_storage.h"

class Heartbeat;
class Sensors;
class SpiDataForwarder;
class FlashLogger;
class FlightStateMachine;

class UsbSerialEcho final {
public:
    UsbSerialEcho(
        Heartbeat &heartbeat,
        Sensors &sensors,
        SpiDataForwarder &spi_interface,
        FlashLogger &flash_logger,
        FlightStateMachine &flight_state_machine);

    UsbSerialEcho(const UsbSerialEcho &) = delete;
    UsbSerialEcho &operator=(const UsbSerialEcho &) = delete;

    // Installs the built-in USB Serial/JTAG driver and starts the echo task.
    esp_err_t start();

private:
    static constexpr UBaseType_t kTaskPriority = 3;
    static constexpr std::size_t kBufferSize = 512;
    // "TX " plus a packet ID and the largest payload as space-separated hex.
    static constexpr std::size_t kCommandBufferSize =
        4 + 3 * (IRIS_PACKET_ID_LENGTH + IRIS_PACKET_MAX_DATA_LENGTH);
    static constexpr std::uint32_t kTaskStackSize = 3072;

    static void task_entry(void *context);
    void echo_bytes(const std::uint8_t *data, std::size_t length);
    void consume_command_bytes(const std::uint8_t *data, std::size_t length);
    void process_command();
    void process_protocol_command(std::uint16_t command);
    void process_tx_command();
    void start_mass_storage();
    void erase_log_sessions();
    void set_flight_state(std::uint8_t state);
    void set_camera_state(std::size_t camera_index, bool enabled);
    void run();

    Heartbeat &heartbeat_;
    Sensors &sensors_;
    SpiDataForwarder &spi_interface_;
    FlashLogger &flash_logger_;
    FlightStateMachine &flight_state_machine_;
    UsbMassStorage mass_storage_;
    TaskHandle_t task_handle_ = nullptr;
    char command_buffer_[kCommandBufferSize]{};
    std::size_t command_length_ = 0;
    bool command_overflow_ = false;
    bool camera_enabled_[2]{};
    bool mass_storage_started_ = false;
};
