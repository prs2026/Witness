#include "uart_driver.h"
#include "task_cores.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "comms.h"
#include "esp_log.h"
#include "hal/uart_ll.h"
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

    if (xTaskCreatePinnedToCore(
            task_entry,
            port_ == UART_NUM_1 ? "uart1" : "uart2",
            kTaskStackSize,
            this,
            kTaskPriority,
            &task_handle_,
            task_cores::kSystem) != pdPASS) {
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

    // RX only; TX is connected by the first transmit (see tx_connected_).
    gpio_reset_pin(tx_pin_);
    result = uart_set_pin(
        port_,
        UART_PIN_NO_CHANGE,
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

    if (!tx_connected_) {
        if (uart_set_pin(port_, current_tx(), current_rx(), UART_PIN_NO_CHANGE,
                         UART_PIN_NO_CHANGE) != ESP_OK)
            return ESP_FAIL;
        tx_connected_ = true;
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

UartDriver::RxProbe UartDriver::probe_rx(const std::uint32_t window_ms)
{
    RxProbe probe{};
    uart_dev_t *const hw = UART_LL_GET_HW(port_);
    (void)uart_get_sclk_freq(UART_SCLK_DEFAULT, &probe.sclk_hz);
    // Clearing autobaud_en resets the edge and pulse counters.
    uart_ll_set_autobaud_en(hw, false);
    uart_ll_set_autobaud_en(hw, true);
    vTaskDelay(pdMS_TO_TICKS(window_ms));
    probe.edges = uart_ll_get_rxd_edge_cnt(hw);
    probe.low_min_cycles = uart_ll_get_low_pulse_cnt(hw);
    probe.high_min_cycles = uart_ll_get_high_pulse_cnt(hw);
    uart_ll_set_autobaud_en(hw, false);
    // A single-bit pulse is the shortest in either level for NMEA text.
    const std::uint32_t shortest = std::min(probe.low_min_cycles, probe.high_min_cycles);
    // Valid only with real pulses of both levels (counters at 0 or 4095 mean
    // a glitch from reconfiguring the pin, or no complete pulse).
    if (probe.edges >= kProbeMinimumEdges && probe.low_min_cycles > 0 && probe.high_min_cycles > 0 &&
        std::max(probe.low_min_cycles, probe.high_min_cycles) < 4095U && probe.sclk_hz)
        probe.baud_estimate = (probe.sclk_hz + shortest / 2U) / shortest;
    else
        probe.edges = probe.edges >= kProbeMinimumEdges ? 0 : probe.edges;   // not a signal
    return probe;
}

void UartDriver::select_gps_pins()
{
    // Try the configured RX pin, then the swapped one; the receiver's TX is
    // the pin with traffic. Pins and the rate hint go to the GPS startup.
    const gpio_num_t pins[2][2] = {{tx_pin_, rx_pin_}, {rx_pin_, tx_pin_}};
    RxProbe probes[2]{};
    for (int i = 0; i < 2; ++i) {
        listen_on(i == 1);   // listen only: nothing drives either line
        probes[i] = probe_rx(kProbeWindowMs);
        const RxProbe &p = probes[i];
        ESP_LOGI(kTag, "GPS RX probe on GPIO%d (TX GPIO%d): %lu edges in %lu ms, shortest low %lu / high %lu "
                       "clocks at %lu Hz -> about %lu baud",
                 static_cast<int>(pins[i][1]), static_cast<int>(pins[i][0]),
                 static_cast<unsigned long>(p.edges), static_cast<unsigned long>(kProbeWindowMs),
                 static_cast<unsigned long>(p.low_min_cycles), static_cast<unsigned long>(p.high_min_cycles),
                 static_cast<unsigned long>(p.sclk_hz), static_cast<unsigned long>(p.baud_estimate));
    }
    const bool swap = probes[0].edges < kProbeMinimumEdges && probes[1].edges >= kProbeMinimumEdges;
    const int use = swap ? 1 : 0;
    listen_on(swap);
    if (swap) {
        ESP_LOGW(kTag, "GPS TX found on GPIO%d: RX/TX swapped from the configured pins",
                 static_cast<int>(pins[use][1]));
    } else if (probes[0].edges < kProbeMinimumEdges) {
        ESP_LOGW(kTag, "GPS RX probe: no signal on either pin; identification scans both orientations");
    }
    const std::uint32_t estimate = probes[use].baud_estimate;
    const std::uint32_t hint = GpsReceiver::nearest_scan_baud(estimate);
    if (estimate != 0) {
        gps_.set_baud_hint(estimate);
        if (hint != 0) {
            ESP_LOGI(kTag, "GPS identification starts at %lu baud (measured %lu)",
                     static_cast<unsigned long>(hint), static_cast<unsigned long>(estimate));
        } else {
            ESP_LOGW(kTag, "GPS RX bit rate %lu baud matches no standard rate; trying it first",
                     static_cast<unsigned long>(estimate));
        }
    }
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

void UartDriver::listen_on(const bool swapped)
{
    // Both pins back to plain GPIO (no output), then route only RX.
    gpio_reset_pin(tx_pin_);
    gpio_reset_pin(rx_pin_);
    pins_swapped_ = swapped;
    tx_connected_ = false;
    (void)uart_set_pin(port_, UART_PIN_NO_CHANGE, current_rx(), UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
}

bool UartDriver::gps_swap_pins(void *context)
{
    auto *self = static_cast<UartDriver *>(context);
    self->listen_on(!self->pins_swapped_);
    ESP_LOGW(kTag, "GPS: listening on RX GPIO%d (TX GPIO%d once the receiver is heard)",
             static_cast<int>(self->current_rx()), static_cast<int>(self->current_tx()));
    return true;
}

bool UartDriver::gps_set_baud(void *context, const std::uint32_t baud)
{
    auto *self = static_cast<UartDriver *>(context);
    return uart_set_baudrate(self->port_, baud) == ESP_OK;
}

void UartDriver::run()
{
    NmeaGps nmea;
    select_gps_pins();
    ESP_LOGI(
        kTag,
        "UART%d ready at %" PRIu32 " baud on TX=%d RX=%d (before the GPS pin probe)",
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
    auto last_gps_model = gps_.model();
    auto last_gps_baud = gps_.baud();
    if (last_gps_status == GpsReceiver::Status::Configuring)
        ESP_LOGI(kTag, "GPS startup: identifying PX1120S or u-blox M8 at %lu baud",
                 static_cast<unsigned long>(kBaudRate));
    GpsReceiver::AssistProgress last_assist{};
    for (;;) {
        {
            // Pick up an assistance upload handed over by the USB/radio task.
            static std::uint8_t blob[gps_assist::kMaximumBlob];
            std::size_t length = 0;
            portENTER_CRITICAL(&assist_lock_);
            if (assist_pending_length_ != 0U) {
                length = assist_pending_length_;
                std::memcpy(blob, assist_pending_, length);
                assist_pending_length_ = 0;
            }
            portEXIT_CRITICAL(&assist_lock_);
            if (length != 0U) {
                const auto now = static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL);
                if (gps_.begin_assist(blob, length, now)) {
                    ESP_LOGI(kTag, "GPS assist: writing %u ephemeris records to the %s",
                             static_cast<unsigned>(gps_.assist_progress().total), gps_.model_name());
                } else {
                    ESP_LOGW(kTag, "GPS assist refused: %s", gps_.assist_refusal());
                }
            }
        }
        if (cold_start_requested_) {
            cold_start_requested_ = false;
            if (gps_.cold_start(static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL))) {
                ESP_LOGW(kTag, "GPS cold start sent to the %s (ephemerides, almanac, position and "
                               "time cleared); reconfiguring in %lu ms",
                         gps_.model_name(), static_cast<unsigned long>(GpsReceiver::kRestartDelayMs));
            } else {
                ESP_LOGW(kTag, "GPS cold start refused: no receiver identified at %lu baud",
                         static_cast<unsigned long>(GpsReceiver::kOperationalBaud));
            }
        }
        gps_.service(static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL));
        {
            const auto progress = gps_.assist_progress();
            if (last_assist.active && !progress.active) {
                ESP_LOGI(kTag, "GPS assist done: %u/%u acknowledged, %u rejected, %u unanswered",
                         static_cast<unsigned>(progress.acked), static_cast<unsigned>(progress.total),
                         static_cast<unsigned>(progress.nacked),
                         static_cast<unsigned>(progress.timed_out));
            }
            if (assist_status_requested_) {
                assist_status_requested_ = false;
                ESP_LOGI(kTag, "GPS assist status: %s, %u/%u sent, %u acknowledged, %u rejected, "
                               "%u unanswered, %lu uploads since boot; receiver %s %s",
                         progress.active ? "uploading" : "idle",
                         static_cast<unsigned>(progress.sent), static_cast<unsigned>(progress.total),
                         static_cast<unsigned>(progress.acked), static_cast<unsigned>(progress.nacked),
                         static_cast<unsigned>(progress.timed_out),
                         static_cast<unsigned long>(progress.uploads), gps_.model_name(),
                         gps_.status() == GpsReceiver::Status::Ready ? "ready" : "not ready");
            }
            last_assist = progress;
        }
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
                receive_time - last_usb_warning >= pdMS_TO_TICKS(10000)) {
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

        if (gps_.baud() != last_gps_baud) {
            last_gps_baud = gps_.baud();
            ESP_LOGI(kTag, "GPS UART host rate now %lu baud",
                     static_cast<unsigned long>(last_gps_baud));
        }
        if (gps_.model() != last_gps_model) {
            last_gps_model = gps_.model();
            ESP_LOGI(kTag, "GPS identified as %s at %lu baud%s", gps_.model_name(),
                     static_cast<unsigned long>(gps_.baud()),
                     gps_.model_assumed() ? " (assumed from NMEA; no binary reply)" : "");
        }
        if (gps_.status() != last_gps_status) {
            last_gps_status = gps_.status();
            if (last_gps_status == GpsReceiver::Status::Ready) {
                ESP_LOGI(kTag, "%s airborne 4 g mode and %lu Hz GGA+RMC confirmed at %lu baud",
                         gps_.model_name(),
                         static_cast<unsigned long>(gps_.update_rate_hz()),
                         static_cast<unsigned long>(gps_.baud()));
                if (gps_.optional_rejects() != 0U) {
                    ESP_LOGW(kTag, "%s rejected %u optional setting(s), last: %s",
                             gps_.model_name(), gps_.optional_rejects(),
                             gps_.last_optional_reject());
                }
            } else if (last_gps_status == GpsReceiver::Status::Failed) {
                ESP_LOGW(kTag, "%s startup failed: %s; raw logging continues at %lu baud",
                         gps_.model_name(), gps_.failure_reason(),
                         static_cast<unsigned long>(gps_.baud()));
            }
        }

        const auto now_ms = static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL);
        if (now_ms - last_diagnostic_ms >= kGpsDiagnosticIntervalMs) {
            last_diagnostic_ms = now_ms;
            std::uint32_t baud = 0;
            (void)uart_get_baudrate(port_, &baud);
            const auto gga = nmea.latest_fix();
            ESP_LOGI(kTag, "GPS RX: baud=%lu bytes=%lu GGA=%lu fix=%s sats=%u idle_ms=%lu",
                     static_cast<unsigned long>(baud),
                     static_cast<unsigned long>(total_rx_bytes),
                     static_cast<unsigned long>(nmea.gga_sentence_count()),
                     NmeaGps::fresh(gga, now_ms) ? "valid" : "none",
                     NmeaGps::gga_fresh(gga, now_ms) ? static_cast<unsigned>(gga.satellites) : 0U,
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

esp_err_t UartDriver::submit_gps_assist(const std::uint8_t *blob, const std::size_t length)
{
    if (blob == nullptr || length == 0U || length > sizeof(assist_pending_))
        return ESP_ERR_INVALID_ARG;
    esp_err_t result = ESP_OK;
    portENTER_CRITICAL(&assist_lock_);
    if (assist_pending_length_ != 0U) {
        result = ESP_ERR_INVALID_STATE;
    } else {
        std::memcpy(assist_pending_, blob, length);
        assist_pending_length_ = length;
    }
    portEXIT_CRITICAL(&assist_lock_);
    return result;
}

NmeaGps::Fix UartDriver::latest_gps_fix() const
{
    portENTER_CRITICAL(&fix_lock_);
    const auto fix = latest_fix_;
    portEXIT_CRITICAL(&fix_lock_);
    // A fresh no-fix GGA still reports its satellite count; position fields
    // are already zero and valid is false for it.
    return NmeaGps::gga_fresh(fix, static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL))
        ? fix : NmeaGps::Fix{};
}
