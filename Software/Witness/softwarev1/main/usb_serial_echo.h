#pragma once

#include <cstddef>
#include <cstdint>

#include "comms.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps_assist.h"
#include "hitl_input.h"
#include "usb_mass_storage.h"

class Heartbeat;
class Sensors;
class SpiDataForwarder;
class FlashLogger;
class FlightStateMachine;
class CommandBridge;
class WifiOta;

class UsbSerialEcho final {
public:
    UsbSerialEcho(
        Heartbeat &heartbeat,
        Sensors &sensors,
        SpiDataForwarder &spi_interface,
        FlashLogger &flash_logger,
        FlightStateMachine &flight_state_machine,
        CommandBridge &command_bridge,
        WifiOta &wifi_ota);

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
    static constexpr std::uint32_t kBinaryCommandIdleMs = 20;
    static constexpr std::uint32_t kHitlErrorReportIntervalMs = 1000;
    static constexpr std::uint32_t kWifiOffWaitMs = 15000;
    static constexpr std::uint32_t kHitlIgnoredReportIntervalMs = 5000;

    static void task_entry(void *context);
    void echo_bytes(const std::uint8_t *data, std::size_t length);
    // Splits USB input into 0x0C HITL frames (never echoed, never forwarded)
    // and everything else (echoed, then parsed as before).
    void consume_input(const std::uint8_t *data, std::size_t length);
    void echo_and_consume_commands(const std::uint8_t *data, std::size_t length);
    void handle_hitl_frame(const std::uint8_t *frame);
    void report_hitl_frame_error(const char *reason);
    void consume_command_bytes(const std::uint8_t *data, std::size_t length);
    void finish_binary_command();
    void discard_partial_text_line();
    void process_compact_binary_command(
        std::uint8_t command,
        std::uint8_t argument);
    void process_canonical_binary_command(
        const std::uint8_t *data,
        std::size_t length);
    void process_queued_radio_commands();
    void process_radio_command(
        const std::uint8_t *data,
        std::size_t length);
    void process_command();
    void forward_usb_command(
        const std::uint8_t *data,
        std::size_t length);
    // Parses a whitespace-separated hexadecimal 0x05 command and forwards
    // the decoded bytes to UART1. Returns true when a command was forwarded.
    bool forward_text_command_if_present();
    void process_protocol_command(std::uint16_t command);
    void process_tx_command();
    void start_mass_storage();
    void erase_log_sessions();
    void clear_log_sessions();
    // GPS assistance chunks (gps_assist.h); data = the six data bytes.
    // radio: the command came over LoRa (replies go back by radio), else USB.
    void handle_gps_assist(std::uint16_t command, const std::uint8_t *data, const char *source,
                           bool radio);
    void handle_pad_heading(const std::uint8_t *data, const char *source, bool radio);
    void handle_radio_power(const std::uint8_t *data, const char *source, bool radio);
    // IRIS_COMMAND_WIFI (comms.h): d0 on/off, d1 TX dBm, d2 auto-off minutes.
    void handle_wifi(const std::uint8_t *data, const char *source, bool radio);
    // Applies command_policy for the source; returns ESP_ERR_NOT_ALLOWED when refused.
    esp_err_t set_flight_state(std::uint8_t state, bool radio);
    // Mass storage / erase / clear (0x6868/9/A) gated by command_policy.
    // Runs the command when allowed; returns the ack result (OK or REFUSED).
    std::uint8_t run_log_maintenance(std::uint16_t command, bool radio);
    // 0x06 command acknowledgement / 0x07 transfer status (comms.h), sent on
    // the link the command arrived on. data6 = the command's six data bytes.
    void send_ack(bool radio, std::uint16_t command, const std::uint8_t *data6, std::uint8_t result,
                  std::uint16_t detail = 0);
    void send_transfer_status(bool radio, const std::uint8_t *data6);
    void send_reply(bool radio, std::uint8_t packet_id, const std::uint8_t *payload,
                    std::size_t length);
    void set_camera_state(std::size_t camera_index, bool enabled);
    void run();

    Heartbeat &heartbeat_;
    Sensors &sensors_;
    SpiDataForwarder &spi_interface_;
    FlashLogger &flash_logger_;
    FlightStateMachine &flight_state_machine_;
    CommandBridge &command_bridge_;
    WifiOta &wifi_ota_;
    UsbMassStorage mass_storage_;
    TaskHandle_t task_handle_ = nullptr;
    char command_buffer_[kCommandBufferSize]{};
    std::size_t command_length_ = 0;
    bool command_overflow_ = false;
    bool binary_command_active_ = false;
    TickType_t binary_command_last_byte_time_ = 0;
    bool camera_enabled_[2]{};
    bool mass_storage_started_ = false;
    // Set just before the Serial/JTAG driver is uninstalled for export.
    bool usb_driver_removed_ = false;
    hitl::FrameAssembler hitl_frames_{};
    // One assistance transfer at a time, from USB or radio (same task).
    gps_assist::Assembler gps_assist_{};
    std::uint32_t gps_assist_rejects_ = 0;
    TickType_t hitl_last_byte_time_ = 0;
    std::uint32_t hitl_frame_errors_ = 0;
    TickType_t hitl_last_error_report_ = 0;
    std::uint32_t hitl_ignored_frames_ = 0;
    TickType_t hitl_last_ignored_report_ = 0;
};
