#include "usb_serial_echo.h"
#include "maintenance_mode.h"
#include "task_cores.h"

#include <cstddef>
#include <cstdlib>
#include <cstdint>
#include <cstring>

#include "bit_monitor.h"
#include "command_bridge.h"
#include "command_policy.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "flash_logger.h"
#include "flight_state_machine.h"
#include "heartbeat.h"
#include "sdkconfig.h"
#include "sensors.h"
#include "spi_data_forwarder.h"
#include "uart_driver.h"
#include "boot_manager.h"
#include "wifi_ota_logic.h"

// Persistent radio power level (radio agent, 2026-11-15): optional until that
// header lands; the call below is compiled in as soon as it exists.
#if __has_include("radio_power_store.h")
#include "radio_power_store.h"
#define WITNESS_HAVE_RADIO_POWER_STORE 1
#endif

static_assert(command_policy::kRadioPowerMinDbm == IRIS_RADIO_TX_POWER_MIN_DBM &&
                  command_policy::kRadioPowerMaxDbm == IRIS_RADIO_TX_POWER_MAX_DBM,
              "command_policy radio power range must match hardware_pins.h");
static_assert(command_policy::kCh1LoadOff == IRIS_COMMAND_CH1_LOAD_OFF &&
                  command_policy::kCh1LoadOn == IRIS_COMMAND_CH1_LOAD_ON &&
                  command_policy::kCh2LoadOff == IRIS_COMMAND_CH2_LOAD_OFF &&
                  command_policy::kCh2LoadOn == IRIS_COMMAND_CH2_LOAD_ON &&
                  command_policy::kCh3LoadOff == IRIS_COMMAND_CH3_LOAD_OFF &&
                  command_policy::kCh3LoadOn == IRIS_COMMAND_CH3_LOAD_ON &&
                  command_policy::kRegulator5vOff == IRIS_COMMAND_5V_REGULATOR_OFF &&
                  command_policy::kRegulator5vOn == IRIS_COMMAND_5V_REGULATOR_ON,
              "command_policy load opcodes must match comms.h");
static_assert(command_policy::kBenchRestart == IRIS_COMMAND_BENCH_RESTART &&
                  command_policy::kBenchHang == IRIS_COMMAND_BENCH_HANG &&
                  command_policy::kBenchIrisReset == IRIS_COMMAND_BENCH_IRIS_RESET,
              "command_policy bench opcodes must match comms.h");

namespace {

constexpr char kLogTag[] = "serial_command";

std::uint32_t now_ms()
{
    return static_cast<std::uint32_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS;
}

// Flight state for the CommandBridge load gate.
std::uint8_t read_state_for_bridge(void *context)
{
    return static_cast<std::uint8_t>(static_cast<FlightStateMachine *>(context)->state());
}

int hex_nibble(const char character)
{
    if (character >= '0' && character <= '9') {
        return character - '0';
    }
    if (character >= 'A' && character <= 'F') {
        return character - 'A' + 10;
    }
    return -1;
}

bool parse_hex_byte(const char *&cursor, std::uint8_t &value)
{
    while (*cursor == ' ') {
        ++cursor;
    }

    const int high = hex_nibble(cursor[0]);
    const int low = cursor[0] == '\0' ? -1 : hex_nibble(cursor[1]);
    if (high < 0 || low < 0 ||
        (cursor[2] != '\0' && cursor[2] != ' ')) {
        return false;
    }

    value = static_cast<std::uint8_t>((high << 4) | low);
    cursor += 2;
    return true;
}

std::size_t normalize_command(char *command, const std::size_t length)
{
    std::size_t read_index = 0;
    std::size_t write_index = 0;
    bool pending_space = false;

    while (read_index < length &&
           (command[read_index] == ' ' || command[read_index] == '\t')) {
        ++read_index;
    }

    for (; read_index < length; ++read_index) {
        char character = command[read_index];

        if (character == ' ' || character == '\t') {
            pending_space = write_index > 0;
            continue;
        }

        if (pending_space) {
            command[write_index++] = ' ';
            pending_space = false;
        }

        if (character >= 'a' && character <= 'z') {
            character = static_cast<char>(character - ('a' - 'A'));
        }
        command[write_index++] = character;
    }

    command[write_index] = '\0';
    return write_index;
}

}  // namespace

UsbSerialEcho::UsbSerialEcho(
    Heartbeat &heartbeat,
    Sensors &sensors,
    SpiDataForwarder &spi_interface,
    FlashLogger &flash_logger,
    FlightStateMachine &flight_state_machine,
    CommandBridge &command_bridge)
    : heartbeat_(heartbeat),
      sensors_(sensors),
      spi_interface_(spi_interface),
      flash_logger_(flash_logger),
      flight_state_machine_(flight_state_machine),
      command_bridge_(command_bridge)
{
}

esp_err_t UsbSerialEcho::start()
{
    if (task_handle_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    // The bridge refuses load/5 V OFF commands in flight before they reach
    // Iris or the radio (command_policy::allow_iris_load_command).
    command_bridge_.set_state_reader(&read_state_for_bridge, &flight_state_machine_);

    usb_serial_jtag_driver_config_t usb_config{};
    usb_config.tx_buffer_size = kBufferSize;
    usb_config.rx_buffer_size = kBufferSize;

    const esp_err_t result = usb_serial_jtag_driver_install(&usb_config);
    if (result != ESP_OK) {
        return result;
    }

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    // The USB console initially uses polling I/O. Route it through the
    // interrupt-driven driver shared with the echo task.
    usb_serial_jtag_vfs_use_driver();
#endif

    const BaseType_t task_created = xTaskCreatePinnedToCore(
        task_entry,
        "usb_serial_echo",
        kTaskStackSize,
        this,
        kTaskPriority,
        &task_handle_,
        task_cores::kSystem);

    if (task_created != pdPASS) {
        task_handle_ = nullptr;
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
        usb_serial_jtag_vfs_use_nonblocking();
#endif
        usb_serial_jtag_driver_uninstall();
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void UsbSerialEcho::task_entry(void *context)
{
    static_cast<UsbSerialEcho *>(context)->run();
}

void UsbSerialEcho::echo_bytes(const std::uint8_t *data, const std::size_t length)
{
    // The echo is a convenience for a terminal: it must never hold up command
    // handling (radio commands run on this task). No host, or a host that is
    // not draining, means the echo is dropped (audit 07 #5; was an endless
    // wait plus a yield loop that could starve IDLE).
    const TickType_t now = xTaskGetTickCount();
    if (echo_muted_) {
        if (static_cast<std::int32_t>(now - echo_mute_until_) < 0) return;
        echo_muted_ = false;
    }
    if (!usb_serial_jtag_is_connected()) return;

    const TickType_t deadline = now + pdMS_TO_TICKS(kEchoBudgetMs);
    std::size_t bytes_echoed = 0;
    while (bytes_echoed < length) {
        const TickType_t remaining = static_cast<TickType_t>(deadline - xTaskGetTickCount());
        if (static_cast<std::int32_t>(remaining) <= 0) break;
        const int bytes_written = usb_serial_jtag_write_bytes(
            data + bytes_echoed,
            length - bytes_echoed,
            remaining);
        if (bytes_written <= 0) break;
        bytes_echoed += static_cast<std::size_t>(bytes_written);
    }
    if (bytes_echoed < length) {
        // Host not reading: stop echoing for a moment instead of waiting
        // again on every chunk.
        echo_muted_ = true;
        echo_mute_until_ = xTaskGetTickCount() + pdMS_TO_TICKS(kEchoMuteMs);
    }
}

void UsbSerialEcho::consume_input(
    const std::uint8_t *data,
    const std::size_t length)
{
    // A 0x0C byte starts a HITL frame unless a binary 0x05 command is in
    // progress. Text commands are printable, so a 0x0C abandons any partial
    // text line (for example stale echoed log text with no newline yet).
    // Once started, the next 46 bytes belong to the frame whatever their value.
    //
    // While HITL is NOT active a 0x0C/0x0E byte only starts a candidate: the
    // bytes are held, and if they do not form a valid frame (EOF and decode)
    // they are replayed as ordinary input, so a stray 0x0C can no longer
    // swallow the next 46 (or 149) bytes, which may be a real command
    // (audit 07 #15 / 04_sensors). A real frame from a host that is still
    // streaming after an exit is still dropped quietly.
    std::size_t run_start = 0;
    for (std::size_t index = 0; index < length; ++index) {
        const std::uint8_t byte = data[index];
        if (hitl_hold_length_ != 0U) {
            hitl_hold_[hitl_hold_length_++] = byte;
            hitl_last_byte_time_ = xTaskGetTickCount();
            run_start = index + 1U;
            if (hitl_hold_length_ >= hitl_hold_expected_) {
                finish_hitl_hold();
            }
            continue;
        }
        if (!hitl_frames_.active()) {
            if (byte != IRIS_PACKET_ID_HITL_SENSORS && byte != IRIS_PACKET_ID_HITL_IMU) {
                continue;
            }
            // The boundary depends on the bytes before this one.
            echo_and_consume_commands(data + run_start, index - run_start);
            run_start = index;
            if (binary_command_active_) {
                continue;
            }
            if (!sensors_.hitl_active()) {
                run_start = index + 1U;
                hitl_hold_[0] = byte;
                hitl_hold_length_ = 1U;
                hitl_hold_expected_ = byte == IRIS_PACKET_ID_HITL_SENSORS ? hitl::kFrameLength
                                                                          : hitl::kImuFrameLength;
                hitl_last_byte_time_ = xTaskGetTickCount();
                continue;
            }
            discard_partial_text_line();
        }

        run_start = index + 1U;
        hitl_last_byte_time_ = xTaskGetTickCount();
        switch (hitl_frames_.push(byte)) {
            case hitl::FrameAssembler::Result::Frame:
                handle_hitl_frame(hitl_frames_.frame());
                break;
            case hitl::FrameAssembler::Result::BadEof:
                report_hitl_frame_error("invalid EOF; resynchronizing");
                break;
            default:
                break;
        }
    }
    echo_and_consume_commands(data + run_start, length - run_start);
}

void UsbSerialEcho::finish_hitl_hold()
{
    const std::size_t length = hitl_hold_length_;
    bool valid = length > 0U && hitl_hold_[length - 1U] == IRIS_PACKET_EOF_VALUE;
    if (valid) {
        if (hitl_hold_[0] == IRIS_PACKET_ID_HITL_IMU) {
            hitl::ImuBatch batch{};
            valid = hitl::decode_imu_frame(hitl_hold_, length, batch);
        } else {
            hitl::Sample sample{};
            valid = hitl::decode_frame(hitl_hold_, length, sample);
        }
    }
    if (valid) {
        hitl_hold_length_ = 0U;
        handle_hitl_frame(hitl_hold_);   // HITL off: counted and dropped
        return;
    }
    replay_hitl_hold();
}

void UsbSerialEcho::replay_hitl_hold()
{
    // Not a frame: drop the stray ID byte and treat the rest as input.
    std::uint8_t copy[hitl::kMaxFrameLength];
    const std::size_t length = hitl_hold_length_ > 0U ? hitl_hold_length_ - 1U : 0U;
    std::memcpy(copy, hitl_hold_ + 1, length);
    hitl_hold_length_ = 0U;
    if (length != 0U) {
        consume_input(copy, length);
    }
}

void UsbSerialEcho::echo_and_consume_commands(
    const std::uint8_t *data,
    const std::size_t length)
{
    if (length == 0U) {
        return;
    }
    echo_bytes(data, length);
    consume_command_bytes(data, length);
}

void UsbSerialEcho::handle_hitl_frame(const std::uint8_t *const frame)
{
    if (frame[0] == IRIS_PACKET_ID_HITL_IMU) {
        hitl::ImuBatch batch{};
        if (!hitl::decode_imu_frame(frame, hitl::kImuFrameLength, batch)) {
            report_hitl_frame_error("0x0E decode failed");
            return;
        }
        if (!sensors_.submit_hitl_imu_batch(batch)) {
            ++hitl_ignored_frames_;
        }
        return;
    }
    hitl::Sample sample{};
    if (!hitl::decode_frame(frame, hitl::kFrameLength, sample)) {
        report_hitl_frame_error("decode failed");
        return;
    }
    if (sensors_.submit_hitl_sample(sample)) {
        return;
    }
    // Typically a host still streaming after an exit or timeout.
    ++hitl_ignored_frames_;
    const TickType_t now = xTaskGetTickCount();
    if (hitl_ignored_frames_ == 1U ||
        now - hitl_last_ignored_report_ >=
            pdMS_TO_TICKS(kHitlIgnoredReportIntervalMs)) {
        ESP_LOGW(kLogTag, "%lu 0x0C HITL frame(s) ignored: HITL mode is not active",
                 static_cast<unsigned long>(hitl_ignored_frames_));
        hitl_ignored_frames_ = 0;
        hitl_last_ignored_report_ = now;
    }
}

void UsbSerialEcho::report_hitl_frame_error(const char *const reason)
{
    ++hitl_frame_errors_;
    const TickType_t now = xTaskGetTickCount();
    if (hitl_frame_errors_ == 1U ||
        now - hitl_last_error_report_ >=
            pdMS_TO_TICKS(kHitlErrorReportIntervalMs)) {
        ESP_LOGW(kLogTag, "0x0C HITL frame rejected: %s (%lu since last report)",
                 reason, static_cast<unsigned long>(hitl_frame_errors_));
        hitl_frame_errors_ = 0;
        hitl_last_error_report_ = now;
    }
}

void UsbSerialEcho::consume_command_bytes(
    const std::uint8_t *data,
    const std::size_t length)
{
    for (std::size_t index = 0; index < length; ++index) {
        const std::uint8_t byte = data[index];

        if (binary_command_active_) {
            if (command_length_ < IRIS_PACKET_COMMAND_FRAME_LENGTH) {
                command_buffer_[command_length_++] =
                    static_cast<char>(byte);
                binary_command_last_byte_time_ = xTaskGetTickCount();
            }

            if (command_length_ == IRIS_PACKET_COMMAND_FRAME_LENGTH) {
                finish_binary_command();
            }
            continue;
        }

        if (byte == IRIS_PACKET_ID_COMMAND) {
            // Text commands are printable, so 0x05 always starts a binary
            // command, even if unterminated text (such as echoed log output)
            // is pending. Otherwise that text would swallow the command.
            discard_partial_text_line();
            binary_command_active_ = true;
            binary_command_last_byte_time_ = xTaskGetTickCount();
            command_buffer_[command_length_++] = static_cast<char>(byte);
            continue;
        }

        const char character = static_cast<char>(data[index]);

        if (character == '\r' || character == '\n') {
            if (command_overflow_) {
                ESP_LOGW(kLogTag, "command discarded: maximum length is %u bytes",
                         static_cast<unsigned>(kCommandBufferSize - 1));
            } else if (command_length_ > 0) {
                process_command();
            }

            command_length_ = 0;
            command_overflow_ = false;
            continue;
        }

        if (command_overflow_) {
            continue;
        }

        if (command_length_ < kCommandBufferSize - 1) {
            command_buffer_[command_length_++] = character;
        } else {
            command_overflow_ = true;
        }
    }
}

void UsbSerialEcho::discard_partial_text_line()
{
    if (command_length_ != 0U || command_overflow_) {
        ESP_LOGD(kLogTag, "discarded %u bytes of unterminated text before a binary frame",
                 static_cast<unsigned>(command_length_));
    }
    command_length_ = 0;
    command_overflow_ = false;
}

void UsbSerialEcho::finish_binary_command()
{
    std::size_t packet_length = command_length_;

    // CR/LF is a terminal delimiter for compact commands, but byte 9 of a
    // canonical command is the required 0x0A application EOF and is retained.
    if (packet_length < IRIS_PACKET_COMMAND_FRAME_LENGTH) {
        while (packet_length > 1U &&
               (static_cast<std::uint8_t>(command_buffer_[packet_length - 1U]) ==
                    '\r' ||
                static_cast<std::uint8_t>(command_buffer_[packet_length - 1U]) ==
                    '\n')) {
            --packet_length;
        }
    }

    const std::uint16_t frame_opcode = static_cast<std::uint16_t>(
        (static_cast<std::uint8_t>(command_buffer_[7]) << 8U) |
        static_cast<std::uint8_t>(command_buffer_[8]));
    const bool assist_frame =
        packet_length == IRIS_PACKET_COMMAND_FRAME_LENGTH &&
        (gps_assist::is_assist_command(frame_opcode) ||
         frame_opcode == IRIS_COMMAND_GPS_COLD_START ||
         command_policy::is_bench_command(frame_opcode) ||
         bit::policy::is_bit_command(frame_opcode));   // BIT is local to Witness: never forwarded
    if (packet_length > 1U) {
        if (!assist_frame) {
            forward_usb_command(
                reinterpret_cast<const std::uint8_t *>(command_buffer_),
                packet_length);
        }

        if (packet_length == 3U) {
            process_compact_binary_command(
                static_cast<std::uint8_t>(command_buffer_[1]),
                static_cast<std::uint8_t>(command_buffer_[2]));
        } else if (packet_length == IRIS_PACKET_COMMAND_FRAME_LENGTH) {
            process_canonical_binary_command(
                reinterpret_cast<const std::uint8_t *>(command_buffer_),
                packet_length);
        }
    }

    command_length_ = 0;
    command_overflow_ = false;
    binary_command_active_ = false;
}

void UsbSerialEcho::process_canonical_binary_command(
    const std::uint8_t *const data,
    const std::size_t length)
{
    if (data == nullptr || length != IRIS_PACKET_COMMAND_FRAME_LENGTH ||
        data[0] != IRIS_PACKET_ID_COMMAND) {
        return;
    }
    if (data[IRIS_PACKET_COMMAND_FRAME_LENGTH - 1U] !=
        IRIS_PACKET_EOF_VALUE) {
        ESP_LOGW(kLogTag, "USB command rejected: invalid EOF");
        return;
    }

    constexpr std::size_t kCommandFrameOffset =
        IRIS_PACKET_ID_LENGTH + IRIS_PACKET_COMMAND_VALUE_OFFSET;
    const std::uint16_t command = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(data[kCommandFrameOffset]) << 8U) |
        data[kCommandFrameOffset + 1U]);
    const std::uint8_t *const args = data + IRIS_PACKET_ID_LENGTH;
    if (command == IRIS_COMMAND_HITL_ENTER) {
        const esp_err_t r = sensors_.enter_hitl(hitl::Source::Usb);
        send_ack(false, command, args, r == ESP_OK ? IRIS_ACK_OK : IRIS_ACK_REFUSED);
        return;
    }
    if (command == IRIS_COMMAND_HITL_EXIT) {
        const esp_err_t r = sensors_.exit_hitl(hitl::Source::Usb);
        send_ack(false, command, args, r == ESP_OK ? IRIS_ACK_OK : IRIS_ACK_REFUSED);
        return;
    }
    if (command_policy::is_bench_command(command)) {
        // Bench-only commands (restart 0x4252, hang 0x4248, Iris reset 0x4249):
        // canonical USB frames while HITL is active. HITL can only be entered
        // over USB and only in Pad Idle, so a real flight cannot reach them;
        // radio and the USB-forward path refuse them (command_bridge.cpp).
        if (!command_policy::allow_bench_command(command_policy::Source::Usb, sensors_.hitl_active(),
                                                    sensors_.resumed_after_reset())) {
            ESP_LOGW(kLogTag, "bench command 0x%04X refused: HITL not active and not resumed after a reset", command);
            send_ack(false, command, args, IRIS_ACK_REFUSED);
            return;
        }
        const std::uint8_t state = static_cast<std::uint8_t>(flight_state_machine_.state());
        if (command == IRIS_COMMAND_BENCH_HANG) {
            ESP_LOGW(kLogTag, "bench hang in flight state %u: the sensor task stops, the watchdog resets in ~8 s",
                     static_cast<unsigned>(state));
            send_ack(false, command, args, IRIS_ACK_OK);
            (void)spi_interface_.flush(pdMS_TO_TICKS(200));
            sensors_.bench_hang();
            return;
        }
        if (command == IRIS_COMMAND_BENCH_IRIS_RESET) {
            const esp_err_t sent = command_bridge_.send_bench_iris_reset();
            ESP_LOGW(kLogTag, "bench Iris warm reset in flight state %u: CAN send %s",
                     static_cast<unsigned>(state), esp_err_to_name(sent));
            send_ack(false, command, args, sent == ESP_OK ? IRIS_ACK_OK : IRIS_ACK_BUSY);
            return;
        }
        // IRIS_COMMAND_BENCH_RESTART: a warm reset in any flight state (tests
        // the RTC resume).
        const std::uint8_t mode = args[0];
        if (mode != IRIS_BENCH_RESTART_SOFTWARE && mode != IRIS_BENCH_RESTART_PANIC) {
            send_ack(false, command, args, IRIS_ACK_BAD_ARGUMENT);
            return;
        }
        ESP_LOGW(kLogTag, "bench restart (%s) in flight state %u",
                 mode == IRIS_BENCH_RESTART_PANIC ? "panic" : "software", static_cast<unsigned>(state));
        send_ack(false, command, args, IRIS_ACK_OK);
        // The ack must be out before the reset (queued before the flush, so the
        // flush covers it).
        (void)spi_interface_.flush(pdMS_TO_TICKS(200));
        if (mode == IRIS_BENCH_RESTART_PANIC) abort();
        esp_restart();
    }
    if (bit::policy::is_bit_command(command)) {
        if (bit_monitor_ != nullptr) bit_monitor_->handle_command(command, args, false);
        else send_ack(false, command, args, IRIS_ACK_UNKNOWN);
        return;
    }
    if (command == IRIS_COMMAND_WITNESS_DEBUG_START) {
        // Canonical 05 .. 00 E0 0A, as the HITL tool sends it. The compact
        // handler has no 0x00E0 form, so without this the request was ignored.
        process_protocol_command(command);
        send_ack(false, command, args, IRIS_ACK_OK);
        return;
    }
    if (gps_assist::is_assist_command(command)) {
        handle_gps_assist(command, args, "USB", false);
        return;
    }
    if (command == IRIS_COMMAND_PAD_HEADING) {
        handle_pad_heading(args, "USB", false);
        return;
    }
    if (command == IRIS_COMMAND_RADIO_POWER) {
        handle_radio_power(args, "USB", false);
        return;
    }
    if (command == IRIS_COMMAND_WIFI) {
        handle_wifi(args, "USB", false);
        return;
    }
    if (command == IRIS_COMMAND_GPS_COLD_START) {
        // Only on the pad: a cold start loses the fix for 30 s or more.
        if (!command_policy::allow_gps_cold_start(static_cast<std::uint8_t>(flight_state_machine_.state()))) {
            ESP_LOGW(kLogTag, "GPS cold start over USB refused: not in Pad Idle");
            send_ack(false, command, args, IRIS_ACK_REFUSED);
            return;
        }
        ESP_LOGW(kLogTag, "GPS cold start requested over USB");
        sensors_.gps_uart().request_gps_cold_start();
        send_ack(false, command, args, IRIS_ACK_OK);
        return;
    }
    if (command_policy::is_log_maintenance_command(command)) {
        // The switch to the maintenance image (0x6868) is decided first so a
        // refusal can carry its reason; the restart it schedules runs after
        // this ack.
        std::uint16_t detail = 0;
        const std::uint8_t result = run_log_maintenance(command, false, detail);
        send_ack(false, command, args, result, detail);
        return;
    }
    if ((command >> 8U) == IRIS_COMMAND_SET_FLIGHT_STATE) {
        const esp_err_t r = set_flight_state(static_cast<std::uint8_t>(command), false);
        send_ack(false, command, args, r == ESP_OK ? IRIS_ACK_OK : IRIS_ACK_REFUSED);
        return;
    }
    if (command_policy::is_iris_load_command(command)) {
        // Already forwarded (or refused) by the bridge just before this call.
        const bool allowed = command_policy::allow_iris_load_command(
            command, static_cast<std::uint8_t>(flight_state_machine_.state()));
        send_ack(false, command, args,
                 !allowed ? IRIS_ACK_REFUSED : last_forward_result_ == ESP_OK ? IRIS_ACK_OK : IRIS_ACK_BUSY);
        return;
    }
    // The compact handler acts on (or forwards) the rest; the ack confirms
    // reception (it does not report a result for these).
    send_ack(false, command, args, IRIS_ACK_OK);
    process_compact_binary_command(
        data[kCommandFrameOffset], data[kCommandFrameOffset + 1U]);
}

void UsbSerialEcho::process_compact_binary_command(
    const std::uint8_t command,
    const std::uint8_t argument)
{
    const std::uint16_t opcode = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(command) << 8U) | argument);
    if (command_policy::is_log_maintenance_command(opcode)) {
        std::uint16_t detail = 0;
        (void)run_log_maintenance(opcode, false, detail);
    } else if (command == IRIS_COMMAND_SET_FLIGHT_STATE) {
        (void)set_flight_state(argument, false);
    } else if (command == IRIS_COMMAND_IMU_CALIBRATION) {
        (void)sensors_.request_calibration(argument);
    } else if (command == static_cast<std::uint8_t>(IRIS_COMMAND_HITL_ENTER >> 8) &&
               (argument == static_cast<std::uint8_t>(IRIS_COMMAND_HITL_ENTER) ||
                argument == static_cast<std::uint8_t>(IRIS_COMMAND_HITL_EXIT))) {
        ESP_LOGW(kLogTag, "HITL command ignored: send it as a canonical 10-byte 0x05 frame");
    }
}

void UsbSerialEcho::process_queued_radio_commands()
{
    std::uint8_t command[IRIS_PACKET_COMMAND_FRAME_LENGTH]{};
    std::size_t command_length = 0;
    esp_err_t forward_result = ESP_OK;
    while (command_bridge_.receive_radio_command(
        command, sizeof(command), command_length, &forward_result)) {
        process_radio_command(command, command_length, forward_result);
    }
}

void UsbSerialEcho::process_radio_command(
    const std::uint8_t *const data,
    const std::size_t length,
    const esp_err_t forward_result)
{
    if (data == nullptr || data[0] != IRIS_PACKET_ID_COMMAND) {
        return;
    }
    (void)flash_logger_.log_packet(FlashLogger::kPacketSourceRadioRx, data, length);

    std::uint16_t command = 0;
    if (length == 3U) {
        command = static_cast<std::uint16_t>(
            (static_cast<std::uint16_t>(data[1]) << 8U) | data[2]);
    } else if (length == IRIS_PACKET_COMMAND_FRAME_LENGTH) {
        if (data[IRIS_PACKET_COMMAND_FRAME_LENGTH - 1U] !=
            IRIS_PACKET_EOF_VALUE) {
            ESP_LOGW(kLogTag, "radio command rejected: invalid EOF");
            return;
        }
        constexpr std::size_t kCommandFrameOffset =
            IRIS_PACKET_ID_LENGTH + IRIS_PACKET_COMMAND_VALUE_OFFSET;
        command = static_cast<std::uint16_t>(
            (static_cast<std::uint16_t>(data[kCommandFrameOffset]) << 8U) |
            data[kCommandFrameOffset + 1U]);
    } else {
        ESP_LOGW(
            kLogTag,
            "radio command rejected: unsupported %u-byte format",
            static_cast<unsigned>(length));
        return;
    }

    // Acknowledgements only for canonical 10-byte frames (the 3-byte compact
    // form has no data bytes to echo).
    const bool canonical = length == IRIS_PACKET_COMMAND_FRAME_LENGTH;
    const std::uint8_t *const args = data + IRIS_PACKET_ID_LENGTH;
    const auto ack = [&](std::uint8_t result, std::uint16_t detail = 0) {
        if (canonical) send_ack(true, command, args, result, detail);
    };
    if (gps_assist::is_assist_command(command)) {
        // Chunked upload: logged per transfer, not per frame.
        if (canonical) handle_gps_assist(command, args, "radio", true);
        return;
    }
    if (bit::policy::is_bit_command(command)) {
        if (canonical && bit_monitor_ != nullptr) bit_monitor_->handle_command(command, args, true);
        else ack(IRIS_ACK_UNKNOWN);
        return;
    }
    if (command == IRIS_COMMAND_PAD_HEADING) {
        if (canonical) handle_pad_heading(args, "radio", true);
        return;
    }
    if (command == IRIS_COMMAND_RADIO_POWER) {
        if (canonical) handle_radio_power(args, "radio", true);
        return;
    }
    if (command == IRIS_COMMAND_WIFI) {
        if (canonical) handle_wifi(args, "radio", true);
        return;
    }
    if (command == IRIS_COMMAND_GPS_COLD_START) {
        // Only on the pad: a cold start loses the fix for 30 s or more.
        if (!command_policy::allow_gps_cold_start(static_cast<std::uint8_t>(flight_state_machine_.state()))) {
            ESP_LOGW(kLogTag, "GPS cold start over radio refused: not in Pad Idle");
            ack(IRIS_ACK_REFUSED);
            return;
        }
        ESP_LOGW(kLogTag, "GPS cold start requested over radio");
        sensors_.gps_uart().request_gps_cold_start();
        ack(IRIS_ACK_OK);
        return;
    }
    ESP_LOGI(kLogTag, "acting on radio command 0x%04X", command);
    switch (command) {
        case IRIS_COMMAND_WITNESS_DEBUG_START:
            process_protocol_command(command);
            ack(IRIS_ACK_OK);
            break;
        case IRIS_COMMAND_MASS_STORAGE_START:
            // USB export is USB only (it needs the USB disk).
            ESP_LOGW(kLogTag, "radio command 0x%04X refused: USB export is USB-only", command);
            ack(IRIS_ACK_REFUSED);
            break;
        case IRIS_COMMAND_ERASE_LOG_SESSIONS:
        case IRIS_COMMAND_CLEAR_LOG_SESSIONS: {
            // Over radio too (Pad Idle/Landed): the only way to free a store
            // whose two slots hold flights once Witness is on the backplane.
            // Two sends needed (command_policy::EraseConfirm): the first one is
            // answered IRIS_ACK_BUSY + IRIS_ACK_DETAIL_CONFIRM_NEEDED and does nothing.
            std::uint16_t detail = 0;
            const std::uint8_t result = run_log_maintenance(command, true, detail);
            ack(result, detail);
            break;
        }
        case IRIS_COMMAND_HITL_ENTER:
            // Rejected and logged: HITL can only be entered over USB.
            (void)sensors_.enter_hitl(hitl::Source::Radio);
            ack(IRIS_ACK_REFUSED);
            break;
        case IRIS_COMMAND_HITL_EXIT: {
            const esp_err_t r = sensors_.exit_hitl(hitl::Source::Radio);
            ack(r == ESP_OK ? IRIS_ACK_OK : IRIS_ACK_REFUSED);
            break;
        }
        case 0x7300U:
        case 0x7301U:
        case 0x7302U:
        case 0x7303U:
        case 0x7304U:
            ack(set_flight_state(static_cast<std::uint8_t>(command), true) == ESP_OK ? IRIS_ACK_OK
                                                                                     : IRIS_ACK_REFUSED);
            break;
        case IRIS_COMMAND_CH1_LOAD_OFF:
        case IRIS_COMMAND_CH1_LOAD_ON:
        case IRIS_COMMAND_CH2_LOAD_OFF:
        case IRIS_COMMAND_CH2_LOAD_ON:
        case IRIS_COMMAND_CH3_LOAD_OFF:
        case IRIS_COMMAND_CH3_LOAD_ON:
        case IRIS_COMMAND_5V_REGULATOR_OFF:
        case IRIS_COMMAND_5V_REGULATOR_ON:
            // These commands are implemented by the external device. The
            // CommandBridge already forwarded them (or refused them: OFF in
            // Boost/Coast/Descent, command_policy). OK only when the CAN
            // queue accepted the frame, so the ground can retry otherwise.
            if (!command_policy::allow_iris_load_command(
                    command, static_cast<std::uint8_t>(flight_state_machine_.state()))) {
                ESP_LOGW(kLogTag, "radio load command 0x%04X refused in flight", command);
                ack(IRIS_ACK_REFUSED);
            } else if (forward_result != ESP_OK) {
                ESP_LOGW(kLogTag, "radio load command 0x%04X not forwarded to Iris: %s", command,
                         esp_err_to_name(forward_result));
                ack(IRIS_ACK_BUSY);
            } else {
                ack(IRIS_ACK_OK);
            }
            break;
        case IRIS_COMMAND_PING:
            // Ground ping (already forwarded to Iris, which answers PONG over CAN).
            // In Landed Witness also replies with the landed position packet at
            // once (rate limited); in any other state it is unknown, as before.
            if (landed::kLowPowerModeEnabled &&
                flight_state_machine_.state() == FlightStateMachine::State::Landed) {
                ack(sensors_.request_landed_ping() ? IRIS_ACK_OK : IRIS_ACK_BUSY);
            } else {
                ack(IRIS_ACK_UNKNOWN);
            }
            break;
        default:
            ESP_LOGW(kLogTag, "radio command 0x%04X has no local handler",
                     command);
            ack(IRIS_ACK_UNKNOWN);
            break;
    }
}

void UsbSerialEcho::process_command()
{
    if (command_length_ > 0U &&
        static_cast<std::uint8_t>(command_buffer_[0]) ==
            IRIS_PACKET_ID_COMMAND) {
        forward_usb_command(
            reinterpret_cast<const std::uint8_t *>(command_buffer_),
            command_length_);
        return;
    }

    if (command_length_ == 1U &&
        static_cast<std::uint8_t>(command_buffer_[0]) ==
            IRIS_COMMAND_WITNESS_DEBUG_START) {
        process_protocol_command(IRIS_COMMAND_WITNESS_DEBUG_START);
        return;
    }

    if (normalize_command(command_buffer_, command_length_) == 0) {
        return;
    }

    const bool command_forwarded = forward_text_command_if_present();

    if (std::strncmp(command_buffer_, "TX ", 3) == 0) {
        process_tx_command();
    } else if (std::strcmp(command_buffer_, "05 68 68") == 0) {
        std::uint16_t detail = 0;
        (void)run_log_maintenance(IRIS_COMMAND_MASS_STORAGE_START, false, detail);
    } else if (std::strcmp(command_buffer_, "05 68 69") == 0) {
        std::uint16_t detail = 0;
        (void)run_log_maintenance(IRIS_COMMAND_ERASE_LOG_SESSIONS, false, detail);
    } else if (std::strcmp(command_buffer_, "05 68 6A") == 0 ||
               std::strcmp(command_buffer_, "05 68 6a") == 0) {
        std::uint16_t detail = 0;
        (void)run_log_maintenance(IRIS_COMMAND_CLEAR_LOG_SESSIONS, false, detail);
    } else if (std::strlen(command_buffer_) == 8U &&
               std::strncmp(command_buffer_, "05 73 0", 7) == 0 &&
               hex_nibble(command_buffer_[7]) >= 0) {
        (void)set_flight_state(
            static_cast<std::uint8_t>(hex_nibble(command_buffer_[7])), false);
    } else if (std::strcmp(command_buffer_, "05 48 49") == 0 ||
               std::strcmp(command_buffer_, "05 48 58") == 0) {
        ESP_LOGW(kLogTag, "HITL command ignored: send it as a canonical 10-byte 0x05 frame");
    } else if (std::strcmp(command_buffer_, "E0") == 0 ||
               std::strcmp(command_buffer_, "0XE0") == 0) {
        process_protocol_command(IRIS_COMMAND_WITNESS_DEBUG_START);
    } else if (std::strcmp(command_buffer_, "LED 1") == 0 ||
        std::strcmp(command_buffer_, "LED ON") == 0) {
        const esp_err_t result = heartbeat_.set_led(true);
        if (result == ESP_OK) {
            ESP_LOGI(kLogTag, "LED ON");
        } else {
            ESP_LOGE(kLogTag, "LED ON failed: %s", esp_err_to_name(result));
        }
    } else if (std::strcmp(command_buffer_, "LED 0") == 0 ||
               std::strcmp(command_buffer_, "LED OFF") == 0) {
        const esp_err_t result = heartbeat_.set_led(false);
        if (result == ESP_OK) {
            ESP_LOGI(kLogTag, "LED OFF");
        } else {
            ESP_LOGE(kLogTag, "LED OFF failed: %s", esp_err_to_name(result));
        }
    } else if (std::strcmp(command_buffer_, "CAM 1 ON") == 0) {
        set_camera_state(0, true);
    } else if (std::strcmp(command_buffer_, "CAM 1 OFF") == 0) {
        set_camera_state(0, false);
    } else if (std::strcmp(command_buffer_, "CAM 2 ON") == 0) {
        set_camera_state(1, true);
    } else if (std::strcmp(command_buffer_, "CAM 2 OFF") == 0) {
        set_camera_state(1, false);
    } else if (command_forwarded) {
        // The external bridge owns generic compact commands and complete
        // canonical 10-byte command packets that have no local action.
        return;
    } else {
        ESP_LOGW(kLogTag, "unknown command: %s", command_buffer_);
    }
}

void UsbSerialEcho::forward_usb_command(
    const std::uint8_t *const data,
    const std::size_t length)
{
    const esp_err_t result = command_bridge_.submit_usb_command(data, length);
    last_forward_result_ = result;
    if (result == ESP_ERR_NOT_ALLOWED) {
        ESP_LOGW(kLogTag, "USB command refused in this flight state: not forwarded");
    } else if (result != ESP_OK) {
        ESP_LOGW(
            kLogTag,
            "external command forwarding failed: %s",
            esp_err_to_name(result));
    }
}

bool UsbSerialEcho::forward_text_command_if_present()
{
    if (command_buffer_[0] != '0' || command_buffer_[1] != '5' ||
        (command_buffer_[2] != '\0' && command_buffer_[2] != ' ')) {
        return false;
    }

    std::uint8_t bytes[
        IRIS_PACKET_ID_LENGTH + IRIS_PACKET_MAX_DATA_LENGTH]{};
    std::size_t byte_count = 0;
    const char *cursor = command_buffer_;
    while (*cursor != '\0' && byte_count < sizeof(bytes)) {
        if (!parse_hex_byte(cursor, bytes[byte_count])) {
            return false;
        }
        ++byte_count;
    }
    while (*cursor == ' ') {
        ++cursor;
    }

    if (*cursor != '\0' || byte_count == 0U) {
        return false;
    }

    if (byte_count == IRIS_PACKET_COMMAND_FRAME_LENGTH &&
        bytes[IRIS_PACKET_COMMAND_FRAME_LENGTH - 1U] !=
            IRIS_PACKET_EOF_VALUE) {
        ESP_LOGW(
            kLogTag,
            "10-byte 0x05 packet has invalid EOF 0x%02X; forwarding unchanged",
            bytes[IRIS_PACKET_COMMAND_FRAME_LENGTH - 1U]);
    }

    forward_usb_command(bytes, byte_count);
    if (byte_count == IRIS_PACKET_COMMAND_FRAME_LENGTH) {
        process_canonical_binary_command(bytes, byte_count);
    }
    return true;
}

esp_err_t UsbSerialEcho::erase_log_sessions()
{
    ESP_LOGW(kLogTag, "protected erase command accepted; erasing all log sessions");
    // Bounded: this task also serves the radio commands, it must not hang if
    // the logger task is gone (audit 07 #6).
    const esp_err_t result = flash_logger_.erase_all_sessions(pdMS_TO_TICKS(kEraseWaitMs));
    if (result == ESP_OK) {
        ESP_LOGI(kLogTag, "all flash log sessions erased");
    } else {
        ESP_LOGE(kLogTag, "log erase failed: %s", esp_err_to_name(result));
    }
    return result;
}

esp_err_t UsbSerialEcho::clear_log_sessions()
{
    ESP_LOGW(kLogTag, "protected clear command accepted; deleting all log sessions");
    const esp_err_t result = flash_logger_.clear_all_sessions(pdMS_TO_TICKS(kEraseWaitMs));
    if (result == ESP_OK) {
        ESP_LOGI(kLogTag, "all flash log sessions cleared");
    } else {
        ESP_LOGE(kLogTag, "log clear failed: %s", esp_err_to_name(result));
    }
    return result;
}

void UsbSerialEcho::handle_gps_assist(const std::uint16_t command, const std::uint8_t *data,
                                      const char *source, const bool radio)
{
    using gps_assist::Assembler;
    if (!command_policy::allow_gps_assist(command,
                                          static_cast<std::uint8_t>(flight_state_machine_.state()))) {
        if (++gps_assist_rejects_ % 50U == 1U) {
            ESP_LOGW(kLogTag, "GPS assist 0x%04X over %s refused: Pad Idle/Landed only", command, source);
        }
        if (command != gps_assist::kCommandData) send_ack(radio, command, data, IRIS_ACK_REFUSED);
        return;
    }
    const Assembler::Result result = gps_assist_.handle(command, data);
    const bool data_chunk = command == gps_assist::kCommandData;
    switch (result) {
        case Assembler::Result::Accepted:
            if (command == gps_assist::kCommandBegin) {
                ESP_LOGI(kLogTag, "GPS assist upload from %s: %u bytes in %u chunks (%u already here)",
                         source, static_cast<unsigned>(gps_assist_.size()),
                         static_cast<unsigned>(gps_assist_.chunks_expected()),
                         static_cast<unsigned>(gps_assist_.chunks_received()));
            }
            // DATA chunks are confirmed in bulk by the 0x07 status bitmap.
            if (!data_chunk) send_ack(radio, command, data, IRIS_ACK_OK);
            return;
        case Assembler::Result::Rejected:
            if (++gps_assist_rejects_ % 50U == 1U) {
                ESP_LOGW(kLogTag, "GPS assist frame 0x%04X rejected (%lu so far): no matching transfer",
                         command, static_cast<unsigned long>(gps_assist_rejects_));
            }
            if (!data_chunk) send_ack(radio, command, data, IRIS_ACK_BAD_ARGUMENT);
            return;
        case Assembler::Result::Incomplete:
            ESP_LOGW(kLogTag, "GPS assist commit: %u of %u chunks received; resend the missing ones",
                     static_cast<unsigned>(gps_assist_.chunks_received()),
                     static_cast<unsigned>(gps_assist_.chunks_expected()));
            send_ack(radio, command, data, IRIS_ACK_INCOMPLETE,
                     static_cast<std::uint16_t>(gps_assist_.chunks_received()));
            return;
        case Assembler::Result::BadCrc:
            ESP_LOGW(kLogTag, "GPS assist commit: CRC mismatch; upload discarded");
            send_ack(radio, command, data, IRIS_ACK_BAD_CRC);
            return;
        case Assembler::Result::BadFormat:
            ESP_LOGW(kLogTag, "GPS assist commit: records do not parse; upload discarded");
            send_ack(radio, command, data, IRIS_ACK_BAD_FORMAT);
            return;
        case Assembler::Result::Status:
            ESP_LOGI(kLogTag, "GPS assist transfer: %s, %u/%u chunks",
                     gps_assist_.active() ? "receiving" : "idle",
                     static_cast<unsigned>(gps_assist_.chunks_received()),
                     static_cast<unsigned>(gps_assist_.chunks_expected()));
            sensors_.gps_uart().request_gps_assist_status();
            send_transfer_status(radio, data);
            return;
        case Assembler::Result::AlreadyComplete:
            send_ack(radio, command, data, IRIS_ACK_OK);
            return;
        case Assembler::Result::Complete: {
            const esp_err_t r = sensors_.gps_uart().submit_gps_assist(gps_assist_.blob(),
                                                                       gps_assist_.size());
            if (r == ESP_OK) {
                ESP_LOGI(kLogTag, "GPS assist received: %u records, CRC ok; passing to the GPS",
                         static_cast<unsigned>(gps_assist_.records()));
            } else {
                ESP_LOGW(kLogTag, "GPS assist could not be queued: %s", esp_err_to_name(r));
                // Reopen the transfer so a repeated COMMIT submits again instead of
                // being answered OK for a blob that never reached the GPS (audit G10).
                gps_assist_.handover_failed();
            }
            send_ack(radio, command, data, r == ESP_OK ? IRIS_ACK_OK : IRIS_ACK_BUSY);
            return;
        }
        default:
            return;
    }
}

void UsbSerialEcho::send_reply(const bool radio, const std::uint8_t packet_id,
                               const std::uint8_t *payload, const std::size_t length)
{
    if (!radio) {
        // Acks wait briefly for room: a full USB queue used to drop them silently.
        const esp_err_t queued = spi_interface_.queue_packet_wait(packet_id, payload, length, pdMS_TO_TICKS(50));
        if (queued != ESP_OK) {
            ESP_LOGW(kLogTag, "USB reply 0x%02X not queued: %s", packet_id, esp_err_to_name(queued));
        }
        return;
    }
    std::uint8_t frame[SpiDataForwarder::kMaximumFrameLength]{};
    if (length + IRIS_PACKET_ID_LENGTH + IRIS_PACKET_EOF_LENGTH > sizeof(frame)) return;
    frame[0] = packet_id;
    std::memcpy(frame + IRIS_PACKET_ID_LENGTH, payload, length);
    frame[IRIS_PACKET_ID_LENGTH + length] = IRIS_PACKET_EOF_VALUE;
    const esp_err_t queued = spi_interface_.queue_immediate_radio_packet(
        frame, length + IRIS_PACKET_ID_LENGTH + IRIS_PACKET_EOF_LENGTH);
    if (queued != ESP_OK) {
        // The radio reply queue is full (a very long burst): never silent.
        ESP_LOGW(kLogTag, "radio reply 0x%02X not queued: %s (%lu frames lost so far)", packet_id,
                 esp_err_to_name(queued), static_cast<unsigned long>(spi_interface_.immediate_radio_dropped()));
    }
}

void UsbSerialEcho::send_ack(const bool radio, const std::uint16_t command, const std::uint8_t *data6,
                             const std::uint8_t result, const std::uint16_t detail)
{
    std::uint8_t payload[IRIS_PACKET_COMMAND_ACK_DATA_LENGTH]{};
    payload[0] = static_cast<std::uint8_t>(command >> 8U);
    payload[1] = static_cast<std::uint8_t>(command);
    std::memcpy(payload + 2, data6, 6);
    payload[8] = result;
    payload[9] = static_cast<std::uint8_t>(detail >> 8U);
    payload[10] = static_cast<std::uint8_t>(detail);
    send_reply(radio, IRIS_PACKET_ID_COMMAND_ACK, payload, sizeof(payload));
}

void UsbSerialEcho::set_bit_monitor(BitMonitor *monitor)
{
    bit_monitor_ = monitor;
    if (monitor != nullptr) monitor->set_ack_sink(&UsbSerialEcho::bit_ack_sink, this);
}

void UsbSerialEcho::bit_ack_sink(void *context, const bool radio, const std::uint16_t command,
                                 const std::uint8_t *data6, const std::uint8_t result, const std::uint16_t detail)
{
    static_cast<UsbSerialEcho *>(context)->send_ack(radio, command, data6, result, detail);
}

void UsbSerialEcho::send_transfer_status(const bool radio, const std::uint8_t *data6)
{
    // d0-d1 of the STATUS command: first chunk of the bitmap window, 0xFFFF
    // (or zero data, as older hosts send) = start at the first missing chunk.
    std::size_t start = static_cast<std::size_t>((data6[0] << 8U) | data6[1]);
    if (start == 0xFFFFU || start == 0U) start = gps_assist_.first_missing_chunk();
    std::uint8_t payload[IRIS_PACKET_TRANSFER_STATUS_DATA_LENGTH]{};
    const auto put16 = [&payload](std::size_t at, std::size_t v) {
        payload[at] = static_cast<std::uint8_t>(v >> 8U);
        payload[at + 1] = static_cast<std::uint8_t>(v);
    };
    payload[0] = gps_assist_.transfer_id();
    payload[1] = static_cast<std::uint8_t>(gps_assist_.state());
    put16(2, gps_assist_.size());
    put16(4, gps_assist_.chunks_expected());
    put16(6, gps_assist_.chunks_received());
    put16(8, start);
    gps_assist_.bitmap_window(start, payload + 10, IRIS_PACKET_TRANSFER_STATUS_BITMAP_BYTES);
    send_reply(radio, IRIS_PACKET_ID_TRANSFER_STATUS, payload, sizeof(payload));
}

void UsbSerialEcho::handle_pad_heading(const std::uint8_t *data, const char *source, const bool radio)
{
    // d0 axis code (0xFF clears), d1-d2 azimuth in 0.01 deg (comms.h).
    const std::uint8_t axis = data[0];
    const std::uint16_t centideg = static_cast<std::uint16_t>((data[1] << 8U) | data[2]);
    const esp_err_t r = sensors_.set_pad_heading(axis, centideg);
    send_ack(radio, IRIS_COMMAND_PAD_HEADING, data,
             r == ESP_OK ? IRIS_ACK_OK
             : r == ESP_ERR_INVALID_STATE ? IRIS_ACK_REFUSED : IRIS_ACK_BAD_ARGUMENT);
    if (r == ESP_OK) {
        if (axis == 0xFFU) {
            ESP_LOGI(kLogTag, "pad heading cleared over %s", source);
        } else {
            ESP_LOGI(kLogTag, "pad heading over %s: body %s at %.2f deg; applied on the pad", source,
                     axis == 0 ? "+Y" : axis == 1 ? "-Y" : axis == 2 ? "+Z" : "-Z",
                     static_cast<double>(centideg) / 100.0);
        }
    } else {
        ESP_LOGW(kLogTag, "pad heading over %s refused (%s): Pad Idle only, axis 0-3 or 0xFF, "
                          "azimuth 0-35999",
                 source, esp_err_to_name(r));
    }
}

void UsbSerialEcho::handle_radio_power(const std::uint8_t *data, const char *source, const bool radio)
{
    // d0 target, d1 drive dBm (int8) (comms.h). The ack goes out before the
    // new level takes effect only if it is already queued; either way it
    // reports the level now in use in detail (int16).
    const int requested = static_cast<std::int8_t>(data[1]);
    const std::uint8_t front_end = data[2];
    const std::uint8_t state = static_cast<std::uint8_t>(flight_state_machine_.state());
    const bool front_end_off = front_end == IRIS_RADIO_FRONT_END_OFF;
    std::uint8_t result = IRIS_ACK_OK;
    if (!command_policy::allow_radio_power(state)) {
        result = IRIS_ACK_REFUSED;        // never in Boost (any source)
    } else if (data[0] != IRIS_RADIO_POWER_TARGET_WITNESS || front_end > IRIS_RADIO_FRONT_END_OFF ||
               !command_policy::radio_power_in_range(requested)) {
        result = IRIS_ACK_BAD_ARGUMENT;   // GROUND is Aperture's; it never forwards it
    } else if (!command_policy::allow_radio_power_change(state, requested, sensors_.radio_tx_power_dbm(),
                                                         front_end_off)) {
        result = IRIS_ACK_REFUSED;        // Coast/Descent: no front end off, no lowering
    } else if (!sensors_.set_radio_tx_power_dbm(requested)) {
        result = IRIS_ACK_BAD_ARGUMENT;
    } else if (front_end == IRIS_RADIO_FRONT_END_ON) {
        sensors_.set_radio_front_end_enabled(true);
        front_end_off_pending_ = false;   // a later ON overrides a pending OFF
    }
    // The front end is switched OFF only after the ack is queued (the ack
    // reports the state it is about to have); over radio it then waits for
    // the ack to be on the air and the flight state is checked again.
    const bool switch_off = result == IRIS_ACK_OK && front_end_off;
    const int now = sensors_.radio_tx_power_dbm();
    const bool fem = switch_off ? false : sensors_.radio_front_end_enabled();
    send_ack(radio, IRIS_COMMAND_RADIO_POWER, data, result,
             static_cast<std::uint16_t>((fem ? 0x100U : 0U) | static_cast<std::uint8_t>(now)));
    if (switch_off) {
        if (radio) {
            front_end_off_pending_ = true;
            front_end_off_due_ = xTaskGetTickCount() + pdMS_TO_TICKS(kRadioAckGuardMs);
        } else {
            sensors_.set_radio_front_end_enabled(false);
        }
    }
#ifdef WITNESS_HAVE_RADIO_POWER_STORE
    // Keep the chosen drive level across resets (a reset in flight otherwise
    // drops the downlink to -17 dBm). Front-end on/off is never stored.
    // Ground only: an NVS write stalls both cores for a few ms.
    if (result == IRIS_ACK_OK && command_policy::allow_persist_radio_power(state)) {
        const esp_err_t saved = radio_power_store::save(static_cast<std::int8_t>(now));
        if (saved != ESP_OK) ESP_LOGW(kLogTag, "radio power not stored: %s", esp_err_to_name(saved));
    }
#else
    // TODO(radio_power_store): radio_power_store::save(now) goes here once
    // main/radio_power_store.h exists (audit W1). Not wired in this build.
#endif
    if (result == IRIS_ACK_OK) {
        ESP_LOGI(kLogTag, "radio TX drive %d dBm over %s, front end %s (%s)", now, source,
                 fem ? "on" : "OFF", fem ? "~+26 dB" : "PA unpowered, bench only");
    } else {
        ESP_LOGW(kLogTag, "radio TX power over %s refused%s: target %u, %d dBm (allowed -17..%d); still %d dBm",
                 source, result == IRIS_ACK_REFUSED ? " (Boost; or Coast/Descent lowering / front end off)" : "",
                 data[0], requested,
                 IRIS_RADIO_TX_POWER_MAX_DBM, now);
    }
}

void UsbSerialEcho::handle_wifi(const std::uint8_t *data, const char *source, const bool radio)
{
    if (data[0] == 2U) {
        // Status query: the flight image has no Wi-Fi (the maintenance image
        // answers this command when it is running); AP down = status word 0.
        send_ack(radio, IRIS_COMMAND_WIFI, data, IRIS_ACK_OK, 0);
        return;
    }
    const wifi_ota_logic::WifiCommand command = wifi_ota_logic::parse_wifi_command(
        data, CONFIG_WITNESS_WIFI_TX_POWER_DBM, CONFIG_WITNESS_WIFI_AUTO_OFF_MINUTES);
    if (!command.valid) {
        send_ack(radio, IRIS_COMMAND_WIFI, data, IRIS_ACK_BAD_ARGUMENT);
        ESP_LOGW(kLogTag, "Wi-Fi command over %s: bad argument (on/off 0-1, %d..%d dBm, <= %u min)",
                 source, wifi_ota_logic::kMinTxPowerDbm, wifi_ota_logic::kMaxTxPowerDbm,
                 static_cast<unsigned>(wifi_ota_logic::kMaxAutoOffMinutes));
        return;
    }
    if (!command.on) {
        // Already off: this image has no Wi-Fi.
        send_ack(radio, IRIS_COMMAND_WIFI, data, IRIS_ACK_OK);
        return;
    }
    if (!wifi_ota_logic::state_allows_wifi(static_cast<std::uint8_t>(flight_state_machine_.state()))) {
        ESP_LOGW(kLogTag, "Wi-Fi on over %s refused: not in Pad Idle/Landed", source);
        send_ack(radio, IRIS_COMMAND_WIFI, data, IRIS_ACK_REFUSED);
        return;
    }
    // Hand over to the maintenance image (it restarts after this ack).
    std::uint16_t detail = 0;
    const std::uint8_t result = switch_to_maintenance(
        static_cast<int>(maintenance_mode::Kind::Wifi), command.tx_power_dbm, command.auto_off_minutes, radio,
        detail);
    send_ack(radio, IRIS_COMMAND_WIFI, data, result, detail);
    ESP_LOGW(kLogTag, "Wi-Fi on over %s: %s (detail 0x%04X)", source,
             result == IRIS_ACK_OK ? "switching to the maintenance image" : "refused",
             static_cast<unsigned>(detail));
}

std::uint8_t UsbSerialEcho::switch_to_maintenance(const int kind, const int tx_dbm, const std::uint32_t minutes,
                                                  const bool radio, std::uint16_t &detail)
{
    detail = IRIS_MAINT_REFUSE_NONE;
    // Verifies the maintenance image (present, readable, compatible), selects
    // its slot as the next boot and arms the request. Flash writes stall both
    // cores for a few ms: only allowed on the pad / after landing (the callers
    // check the flight state first).
    const boot_manager::SwitchResult result = boot_manager::prepare_switch(
        static_cast<maintenance_mode::Kind>(kind), tx_dbm, minutes,
        static_cast<std::uint8_t>(flight_state_machine_.state()), sensors_.radio_tx_power_dbm(),
        sensors_.radio_front_end_enabled());
    if (!result.ok) {
        detail = result.refuse_detail;
        return IRIS_ACK_REFUSED;
    }
    if (!schedule_restart(command_policy::PendingRestart::Maintenance, radio)) {
        // prepare_switch already selected the maintenance slot: put the boot
        // partition back, or a later reset would boot maintenance first
        // (audit B10).
        (void)boot_manager::revert_switch();
        detail = IRIS_MAINT_REFUSE_BUSY;
        return IRIS_ACK_BUSY;
    }
    ESP_LOGW(kLogTag, "restarting into the maintenance image (%s); any reset returns to flight",
             result.target != nullptr ? result.target->label : "?");
    return IRIS_ACK_OK;
}

std::uint8_t UsbSerialEcho::run_log_maintenance(const std::uint16_t command, const bool radio,
                                                std::uint16_t &detail)
{
    detail = IRIS_MAINT_REFUSE_NONE;
    const std::uint8_t current = static_cast<std::uint8_t>(flight_state_machine_.state());
    if (!command_policy::allow_log_maintenance(
            radio ? command_policy::Source::Radio : command_policy::Source::Usb, current, command)) {
        ESP_LOGW(kLogTag, "command 0x%04X over %s refused in flight state 0x%02X "
                          "(Pad Idle/Landed only; USB export USB only)",
                 command, radio ? "radio" : "USB", current);
        return IRIS_ACK_REFUSED;
    }
    if (command == IRIS_COMMAND_MASS_STORAGE_START) {
        // USB export runs in the maintenance image (the flight image links no
        // USB-disk code): hand over to it. Any later reset returns to flight.
        const std::uint8_t result = switch_to_maintenance(
            static_cast<int>(maintenance_mode::Kind::UsbExport), 0, 0, radio, detail);
        if (result == IRIS_ACK_OK) {
            ESP_LOGW(kLogTag, "USB export requested: restarting into the maintenance image (USB disk)");
        }
        return result;
    }
    // Over radio, erase/clear need the same command twice within 10 s
    // (accident / replay protection). USB stays single shot.
    if (radio && !erase_confirm_.accept(command, now_ms())) {
        ESP_LOGW(kLogTag, "radio command 0x%04X: send it again within %u s to confirm", command,
                 static_cast<unsigned>(command_policy::kEraseConfirmWindowMs / 1000U));
        detail = IRIS_ACK_DETAIL_CONFIRM_NEEDED;
        return IRIS_ACK_BUSY;
    }
    const esp_err_t erased = command == IRIS_COMMAND_ERASE_LOG_SESSIONS ? erase_log_sessions()
                                                                         : clear_log_sessions();
    if (erased == ESP_OK) return IRIS_ACK_OK;
    detail = static_cast<std::uint16_t>(erased);
    // Timeout / logger busy: try again later; anything else: not done.
    return (erased == ESP_ERR_TIMEOUT || erased == ESP_ERR_INVALID_STATE) ? IRIS_ACK_BUSY : IRIS_ACK_REFUSED;
}

esp_err_t UsbSerialEcho::set_flight_state(const std::uint8_t state, const bool radio)
{
    const std::uint8_t current = static_cast<std::uint8_t>(flight_state_machine_.state());
    if (!command_policy::allow_set_flight_state(
            radio ? command_policy::Source::Radio : command_policy::Source::Usb, current, state)) {
        ESP_LOGW(kLogTag, "flight state 0x%02X -> 0x%02X over %s refused", current, state,
                 radio ? "radio" : "USB");
        return ESP_ERR_NOT_ALLOWED;
    }
    if (command_policy::is_reset_to_pad(current, state)) {
        // The flight states are one-way: the way back to Pad Idle is a restart
        // (a Landed record is not resumed, so the next boot is Pad Idle with a
        // fresh position filter). The caller acknowledges after this returns;
        // the restart timer lets that ack go out first. Any pending
        // maintenance-image request is dropped so this restart boots flight.
        if (pending_restart_ == command_policy::PendingRestart::Maintenance) {
            (void)boot_manager::revert_switch();   // also drops the request
        } else {
            maintenance_mode::cancel();
        }
        if (!schedule_restart(command_policy::PendingRestart::ResetToPad, radio)) {
            ESP_LOGE(kLogTag, "reset to Pad Idle: restart timer failed");
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGW(kLogTag, "Landed -> Pad Idle over %s: restarting (flight states are one-way)",
                 radio ? "radio" : "USB");
        return ESP_OK;
    }
    const bool commanded_launch =
        flight_state_machine_.state() == FlightStateMachine::State::PadIdle &&
        state == static_cast<std::uint8_t>(FlightStateMachine::State::Boost);
    if (commanded_launch) {
        ESP_LOGI(kLogTag, "manual launch trigger accepted");
    }
    const esp_err_t result = commanded_launch
        ? flight_state_machine_.trigger_launch()
        : flight_state_machine_.set_state(state);
    if (result != ESP_OK) {
        ESP_LOGW(kLogTag, "flight state 0x%02X rejected: %s",
                 state, esp_err_to_name(result));
    }
    return result;
}

bool UsbSerialEcho::schedule_restart(const command_policy::PendingRestart kind, const bool radio)
{
    // The timer only raises a flag; service_pending_restart() (run loop)
    // checks the flight state again before it restarts (audit B2 / W11).
    pending_restart_ = kind;
    if (!maintenance_mode::restart_deferred_after_ms(radio ? kRadioAckGuardMs + kSwitchRestartDelayMs
                                                           : kSwitchRestartDelayMs)) {
        pending_restart_ = command_policy::PendingRestart::None;
        return false;
    }
    return true;
}

void UsbSerialEcho::service_pending_restart()
{
    if (pending_restart_ == command_policy::PendingRestart::None || !maintenance_mode::deferred_restart_due()) {
        return;
    }
    const command_policy::PendingRestart kind = pending_restart_;
    pending_restart_ = command_policy::PendingRestart::None;
    maintenance_mode::deferred_restart_clear();
    const std::uint8_t state = static_cast<std::uint8_t>(flight_state_machine_.state());
    if (command_policy::restart_still_allowed(kind, state)) {
        ESP_LOGW(kLogTag, "restarting now (%s)",
                 kind == command_policy::PendingRestart::Maintenance ? "maintenance image" : "reset to Pad Idle");
        // The ack went out 400 ms ago over USB; make sure it left the chip.
        (void)spi_interface_.flush(pdMS_TO_TICKS(200));
        maintenance_mode::restart();
    }
    ESP_LOGE(kLogTag, "restart abandoned: flight state is now 0x%02X", state);
    if (kind == command_policy::PendingRestart::Maintenance) {
        (void)boot_manager::revert_switch();   // boot partition back to the flight image
    } else {
        maintenance_mode::cancel();
    }
}

void UsbSerialEcho::service_deferred_front_end_off()
{
    if (!front_end_off_pending_ || static_cast<std::int32_t>(xTaskGetTickCount() - front_end_off_due_) < 0) return;
    front_end_off_pending_ = false;
    const std::uint8_t state = static_cast<std::uint8_t>(flight_state_machine_.state());
    const int current = sensors_.radio_tx_power_dbm();
    if (!command_policy::allow_radio_power_change(state, current, current, true)) {
        ESP_LOGW(kLogTag, "radio front end stays on: flight state is now 0x%02X", state);
        return;
    }
    sensors_.set_radio_front_end_enabled(false);
    ESP_LOGW(kLogTag, "radio front end OFF (PA unpowered, bench only)");
}

void UsbSerialEcho::log_stack_if_due()
{
    const TickType_t now = xTaskGetTickCount();
    if (static_cast<TickType_t>(now - last_stack_log_) < pdMS_TO_TICKS(kStackLogIntervalMs)) return;
    last_stack_log_ = now;
    const unsigned free_bytes = static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr));
    if (free_bytes < 1024U) {
        ESP_LOGW(kLogTag, "stack high-water: only %u of %u bytes ever free", free_bytes,
                 static_cast<unsigned>(kTaskStackSize));
    } else {
        ESP_LOGI(kLogTag, "stack high-water: %u of %u bytes never used", free_bytes,
                 static_cast<unsigned>(kTaskStackSize));
    }
}

void UsbSerialEcho::process_protocol_command(const std::uint16_t command)
{
    const esp_err_t result = sensors_.handle_command(command);
    if (result != ESP_OK) {
        ESP_LOGW(kLogTag, "command 0x%04X rejected: %s",
                 static_cast<unsigned>(command), esp_err_to_name(result));
    }
}

void UsbSerialEcho::process_tx_command()
{
    std::uint8_t bytes[
        IRIS_PACKET_ID_LENGTH + IRIS_PACKET_MAX_DATA_LENGTH];
    std::size_t byte_count = 0;
    const char *cursor = command_buffer_ + 3;

    while (*cursor != '\0' && byte_count < sizeof(bytes)) {
        if (!parse_hex_byte(cursor, bytes[byte_count])) {
            ESP_LOGW(kLogTag, "TX requires two-digit hexadecimal bytes");
            return;
        }
        ++byte_count;
    }

    while (*cursor == ' ') {
        ++cursor;
    }

    if (*cursor != '\0' || byte_count == 0U) {
        ESP_LOGW(kLogTag, "TX requires two-digit hexadecimal bytes");
        return;
    }

    if (bytes[0] == IRIS_PACKET_ID_COMMAND) {
        forward_usb_command(bytes, byte_count);
        return;
    }
    if (bytes[0] == IRIS_PACKET_ID_HITL_SENSORS) {
        ESP_LOGW(kLogTag, "TX rejected: 0x0C HITL input is USB-local and never forwarded");
        return;
    }

    if (byte_count < IRIS_PACKET_ID_LENGTH + 6U) {
        ESP_LOGW(kLogTag, "TX requires an ID and 6 to %u payload bytes",
                 static_cast<unsigned>(IRIS_PACKET_MAX_DATA_LENGTH));
        return;
    }

    const esp_err_t result = spi_interface_.queuecommand(
        bytes[0], &bytes[1], byte_count - 1);
    if (result != ESP_OK) {
        ESP_LOGW(kLogTag, "TX command rejected: %s", esp_err_to_name(result));
    }
}

void UsbSerialEcho::set_camera_state(
    const std::size_t camera_index,
    const bool enabled)
{
    camera_enabled_[camera_index] = enabled;
    ESP_LOGI(kLogTag, "CAM %u %s",
             static_cast<unsigned>(camera_index + 1),
             enabled ? "ON" : "OFF");
}

void UsbSerialEcho::run()
{
    std::uint8_t buffer[kBufferSize];
    const TickType_t read_timeout =
        pdMS_TO_TICKS(kBinaryCommandIdleMs) > 0
            ? pdMS_TO_TICKS(kBinaryCommandIdleMs)
            : 1;

    for (;;) {
        const int bytes_read = usb_serial_jtag_read_bytes(
            buffer, sizeof(buffer), read_timeout);

        if (bytes_read > 0) {
            consume_input(buffer, static_cast<std::size_t>(bytes_read));
        } else if (binary_command_active_ &&
                   (xTaskGetTickCount() - binary_command_last_byte_time_) >=
                       read_timeout) {
            finish_binary_command();
        } else if (hitl_frames_.active() &&
                   (xTaskGetTickCount() - hitl_last_byte_time_) >=
                       read_timeout) {
            // A host writes each frame at once; a stalled partial frame is
            // dropped so it cannot absorb the next command.
            hitl_frames_.reset();
            report_hitl_frame_error("incomplete frame followed by idle input");
        } else if (hitl_hold_length_ != 0U &&
                   (xTaskGetTickCount() - hitl_last_byte_time_) >= read_timeout) {
            // A stray 0x0C/0x0E that never became a frame: hand the bytes
            // after it back to the command parser.
            replay_hitl_hold();
        }

        process_queued_radio_commands();
        service_deferred_front_end_off();
        service_pending_restart();
        log_stack_if_due();
    }
}
