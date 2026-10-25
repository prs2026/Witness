#include "uart_driver.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "comms.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hardware_pins.h"
#include "flash_logger.h"
#include "spi_data_forwarder.h"

namespace {

constexpr char kTag[] = "uart";
constexpr std::array<std::uint8_t, 2> kLoopbackRequest = {0x05, 0x22};
constexpr std::uint32_t kRadioFrameInterByteTimeoutMs = 20;

std::size_t canonical_frame_length(const std::uint8_t packet_id)
{
    switch (packet_id) {
        case IRIS_PACKET_ID_SENSORS:
            return IRIS_PACKET_SENSORS_FRAME_LENGTH;
        case IRIS_PACKET_ID_STATE:
            return IRIS_PACKET_STATE_FRAME_LENGTH;
        case IRIS_PACKET_ID_CAMERA:
            return IRIS_PACKET_CAMERA_FRAME_LENGTH;
        case IRIS_PACKET_ID_COMMAND:
            return IRIS_PACKET_COMMAND_FRAME_LENGTH;
        case IRIS_PACKET_ID_HEARTBEAT:
            return IRIS_PACKET_HEARTBEAT_FRAME_LENGTH;
        case IRIS_PACKET_ID_WITNESS_DEBUG:
            return IRIS_PACKET_WITNESS_DEBUG_FRAME_LENGTH;
        case IRIS_PACKET_ID_IRIS_DEBUG:
            return IRIS_PACKET_IRIS_DEBUG_FRAME_LENGTH;
        default:
            return 0U;
    }
}

}  // namespace

UartDriver::UartDriver(SpiDataForwarder &serial_forwarder, uart_port_t port,
                       gpio_num_t tx_pin, gpio_num_t rx_pin)
    : serial_forwarder_(serial_forwarder), port_(port),
      tx_pin_(tx_pin), rx_pin_(rx_pin)
{
}

UartDriver::~UartDriver()
{
    (void)shutdown();
}

esp_err_t UartDriver::start()
{
    if (initialized_ || task_handle_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t result = initialize();
    if (result != ESP_OK) {
        return result;
    }

    if (xTaskCreate(
            task_entry,
            port_ == UART_NUM_1 ? "uart1" : "uart2",
            kTaskStackSize,
            this,
            kTaskPriority,
            &task_handle_) != pdPASS) {
        task_handle_ = nullptr;
        (void)shutdown();
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t UartDriver::initialize()
{
    uart_config_t config{};
    config.baud_rate = static_cast<int>(kBaudRate);
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.rx_flow_ctrl_thresh = 0;
    config.source_clk = UART_SCLK_DEFAULT;

    esp_err_t result = uart_param_config(port_, &config);
    if (result != ESP_OK) {
        return result;
    }

    result = uart_set_pin(
        port_,
        tx_pin_,
        rx_pin_,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE);
    if (result != ESP_OK) {
        return result;
    }

    result = uart_driver_install(
        port_,
        kDriverBufferSize,
        kDriverBufferSize,
        0,
        nullptr,
        0);
    if (result == ESP_OK) {
        initialized_ = true;
    }
    return result;
}

esp_err_t UartDriver::shutdown()
{
    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }

    if (!initialized_) {
        return ESP_OK;
    }

    const esp_err_t result = uart_driver_delete(port_);
    if (result == ESP_OK) {
        initialized_ = false;
    }
    return result;
}

esp_err_t UartDriver::transmit(
    const std::uint8_t *const data,
    const std::size_t length,
    const TickType_t timeout_ticks)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == nullptr || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const int written = uart_write_bytes(port_, data, length);
    if (written < 0) {
        return ESP_FAIL;
    }
    if (static_cast<std::size_t>(written) != length) {
        return ESP_ERR_TIMEOUT;
    }
    return timeout_ticks == 0 ? ESP_OK
                              : uart_wait_tx_done(port_, timeout_ticks);
}

esp_err_t UartDriver::send_loopback_request()
{
    return transmit(
        kLoopbackRequest.data(),
        kLoopbackRequest.size(),
        pdMS_TO_TICKS(100));
}

void UartDriver::task_entry(void *const context)
{
    static_cast<UartDriver *>(context)->run();
}

bool UartDriver::gps_send(void *context, const std::uint8_t *data,
                          const std::size_t length)
{
    return static_cast<UartDriver *>(context)->transmit(
               data, length, pdMS_TO_TICKS(100)) == ESP_OK;
}

bool UartDriver::gps_set_baud(void *context, const std::uint32_t baud)
{
    auto *self = static_cast<UartDriver *>(context);
    return uart_set_baudrate(self->port_, baud) == ESP_OK;
}

void UartDriver::run()
{
    NmeaGps nmea;
    ESP_LOGI(
        kTag,
        "UART%d ready at %" PRIu32 " baud on TX=%d RX=%d",
        static_cast<int>(port_),
        kBaudRate,
        static_cast<int>(tx_pin_),
        static_cast<int>(rx_pin_));

    bool waiting_for_loopback_reply = false;
    bool previous_byte_was_05 = false;
    TickType_t loopback_deadline = 0;
    const TickType_t task_delay_ticks =
        std::max<TickType_t>(1, pdMS_TO_TICKS(kTaskTickMs));

    if constexpr (kStartupLoopbackTestEnabled) {
        const esp_err_t test_result = send_loopback_request();
        if (test_result == ESP_OK) {
            waiting_for_loopback_reply = true;
            loopback_deadline = xTaskGetTickCount() +
                                pdMS_TO_TICKS(kLoopbackReplyTimeoutMs);
            ESP_LOGI(kTag, "sent loopback request: 05 22");
        } else {
            ESP_LOGE(kTag, "could not send loopback request: %s",
                     esp_err_to_name(test_result));
        }
    }

    std::array<std::uint8_t, kReadBufferSize> receive_buffer{};
    std::array<std::uint8_t, SpiDataForwarder::kMaximumFrameLength>
        radio_frame{};
    std::size_t radio_frame_length = 0;
    std::size_t expected_radio_frame_length = 0;
    TickType_t last_radio_frame_byte_time = 0;
    std::uint32_t lost_log_bytes = 0;
    TickType_t last_log_warning = 0;
    TickType_t last_usb_warning = 0;
    std::uint32_t total_rx_bytes = 0;
    std::uint32_t last_rx_ms = 0;
    std::uint32_t last_diagnostic_ms = 0;
    auto last_gps_status = gps_.status();
    if (last_gps_status == Px1120sGps::Status::Configuring)
        ESP_LOGI(kTag, "PX1120S startup: configuring airborne navigation; UART fixed at %lu baud",
                 static_cast<unsigned long>(kBaudRate));
    for (;;) {
        gps_.service(static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL));
        const int bytes_read = uart_read_bytes(
            port_,
            receive_buffer.data(),
            receive_buffer.size(),
            task_delay_ticks);

        if (bytes_read > 0) {
            const std::size_t received = static_cast<std::size_t>(bytes_read);
            total_rx_bytes += received;
            last_rx_ms = static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL);
            const TickType_t receive_time = xTaskGetTickCount();
            // Persist before USB forwarding or protocol parsing. GPS/NMEA,
            // binary receiver messages, embedded zeros and CR/LF all survive.
            if (logger_) {
                const auto result = logger_->log_uart_bytes(
                    receive_buffer.data(), received);
                if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
                    lost_log_bytes += received;
                    if (receive_time - last_log_warning >= pdMS_TO_TICKS(1000)) {
                        ESP_LOGE(kTag, "UART%d flash input overflow/error: %lu bytes lost",
                                 static_cast<int>(port_),
                                 static_cast<unsigned long>(lost_log_bytes));
                        last_log_warning = receive_time;
                    }
                }
            }
            gps_.consume(receive_buffer.data(), received,
                         static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL));
            const auto previous_gga_count = nmea.gga_sentence_count();
            nmea.consume(receive_buffer.data(), received, last_rx_ms);
            if (nmea.gga_sentence_count() != previous_gga_count) {
                gps_.observe_nmea();
            }
            const auto fix = nmea.latest_fix();
            portENTER_CRITICAL(&fix_lock_);
            latest_fix_ = fix;
            portEXIT_CRITICAL(&fix_lock_);
            const esp_err_t forward_result =
                serial_forwarder_.queue_raw_bytes(
                    receive_buffer.data(), received);
            if (forward_result != ESP_OK &&
                receive_time - last_usb_warning >= pdMS_TO_TICKS(1000)) {
                last_usb_warning = receive_time;
                ESP_LOGW(
                    kTag,
                    "could not forward %u UART bytes: %s",
                    static_cast<unsigned>(received),
                    esp_err_to_name(forward_result));
            }

            // UART is a byte stream, so reconstruct complete canonical frames
            // before handing externally received packets to the radio. USB
            // still receives the original UART bytes without modification.
            for (std::size_t index = 0; index < received; ++index) {
                if (!serial_forwarder_.radio_available()) {
                    radio_frame_length = 0;
                    expected_radio_frame_length = 0;
                    break;
                }
                const std::uint8_t byte = receive_buffer[index];
                if (radio_frame_length != 0U &&
                    (receive_time - last_radio_frame_byte_time) >
                        pdMS_TO_TICKS(kRadioFrameInterByteTimeoutMs)) {
                    radio_frame_length = 0U;
                    expected_radio_frame_length = 0U;
                }
                if (radio_frame_length == 0U) {
                    expected_radio_frame_length =
                        canonical_frame_length(byte);
                    if (expected_radio_frame_length == 0U) {
                        continue;
                    }
                }

                radio_frame[radio_frame_length++] = byte;
                last_radio_frame_byte_time = receive_time;
                if (radio_frame_length < expected_radio_frame_length) {
                    continue;
                }

                if (byte == IRIS_PACKET_EOF_VALUE) {
                    const esp_err_t radio_result =
                        serial_forwarder_.queue_immediate_radio_packet(
                            radio_frame.data(), radio_frame_length);
                    if (radio_result != ESP_OK) {
                        ESP_LOGW(
                            kTag,
                            "could not queue UART packet for radio: %s",
                            esp_err_to_name(radio_result));
                    }
                } else {
                    ESP_LOGW(
                        kTag,
                        "discarded UART packet 0x%02X with invalid EOF",
                        radio_frame[0]);
                }
                radio_frame_length = 0U;
                expected_radio_frame_length = 0U;
            }

            if (waiting_for_loopback_reply) {
                for (std::size_t index = 0; index < received; ++index) {
                    const std::uint8_t byte = receive_buffer[index];
                    if (previous_byte_was_05 && byte == 0x22) {
                        waiting_for_loopback_reply = false;
                        ESP_LOGI(kTag, "loopback reply received: 05 22");
                        break;
                    }
                    previous_byte_was_05 = byte == 0x05;
                }
            }
        } else if (bytes_read < 0) {
            ESP_LOGW(kTag, "UART receive failed");
        }

        if (gps_.status() != last_gps_status) {
            last_gps_status = gps_.status();
            if (last_gps_status == Px1120sGps::Status::Ready) {
                ESP_LOGI(kTag, "PX1120S airborne mode %u confirmed at %lu baud (4 g receiver)",
                         Px1120sGps::kNavigationMode, static_cast<unsigned long>(kBaudRate));
            } else if (last_gps_status == Px1120sGps::Status::Failed) {
                ESP_LOGW(kTag, "PX1120S startup failed: %s; raw logging continues at %lu baud",
                         gps_.failure_reason(), static_cast<unsigned long>(kBaudRate));
            }
        }

        const auto now_ms = static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL);
        if (now_ms - last_diagnostic_ms >= kGpsDiagnosticIntervalMs) {
            last_diagnostic_ms = now_ms;
            std::uint32_t baud = 0;
            (void)uart_get_baudrate(port_, &baud);
            ESP_LOGI(kTag, "GPS RX: baud=%lu bytes=%lu GGA=%lu fix=%s idle_ms=%lu",
                     static_cast<unsigned long>(baud),
                     static_cast<unsigned long>(total_rx_bytes),
                     static_cast<unsigned long>(nmea.gga_sentence_count()),
                     NmeaGps::fresh(nmea.latest_fix(), now_ms) ? "valid" : "none",
                     static_cast<unsigned long>(now_ms - last_rx_ms));
            if (now_ms - last_rx_ms >= kGpsDiagnosticIntervalMs) {
                ESP_LOGW(kTag, "GPS UART input silent; reception and raw logging remain enabled");
            }
        }

        if (waiting_for_loopback_reply &&
            static_cast<std::int32_t>(
                xTaskGetTickCount() - loopback_deadline) >= 0) {
            waiting_for_loopback_reply = false;
            ESP_LOGW(
                kTag,
                "no 05 22 loopback reply within %" PRIu32 " ms",
                kLoopbackReplyTimeoutMs);
        }
    }
}

NmeaGps::Fix UartDriver::latest_gps_fix() const
{
    portENTER_CRITICAL(&fix_lock_);
    const auto fix = latest_fix_;
    portEXIT_CRITICAL(&fix_lock_);
    return NmeaGps::fresh(fix, static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL))
        ? fix : NmeaGps::Fix{};
}
