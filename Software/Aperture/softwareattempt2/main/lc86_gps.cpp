#include "lc86_gps.h"

#include <array>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hardware.h"

namespace {

constexpr const char *kTag = "gps";
constexpr int kUartRxBufferSize = 2048;
constexpr int kUartTxBufferSize = 256;
constexpr std::uint32_t kTaskStackSize = 4096;
constexpr UBaseType_t kTaskPriority = tskIDLE_PRIORITY + 2;
constexpr std::uint32_t kResetPulseMs = 100;      // LC86G hardware design: >= 100 ms.
constexpr std::uint32_t kBootWaitMs = 3000;       // Wait this long for first NMEA.
constexpr std::uint32_t kAckTimeoutMs = 500;
constexpr int kCommandAttempts = 3;               // LC86G: resend until $PAIR001.
constexpr std::uint32_t kReadSliceMs = 50;
constexpr std::uint32_t kDiagnosticIntervalMs = 30000;

// PAIR result codes (LC26G/LC76G/LC86G GNSS Protocol Specification, $PAIR001).
constexpr std::uint8_t kPairOk = 0;
constexpr std::uint8_t kPairProcessing = 1;
constexpr std::uint8_t kPairNotSupported = 3;
// PMTK_ACK flags (L76-LB/L26-LB/LC86L GNSS Protocol Specification, $PMTK001).
constexpr std::uint8_t kPmtkNotSupported = 1;
constexpr std::uint8_t kPmtkOk = 3;

std::uint32_t now_ms() {
    return static_cast<std::uint32_t>(esp_timer_get_time() / 1000LL);
}

void set_reset(bool asserted) {
    const bool level = IRIS_GPS_RESET_ACTIVE_LOW ? !asserted : asserted;
    (void)gpio_set_level(IRIS_PIN_GPS_RESET, level ? 1U : 0U);
}

}  // namespace

esp_err_t Lc86Gps::start() {
    if (task_ != nullptr) return ESP_ERR_INVALID_STATE;

    // RESET_N has an internal 1.8 V pull-up and the LC86G hardware design
    // forbids an external one, so the active-low line is driven open-drain:
    // low to reset, released (pulled high by the module) to run.
    gpio_config_t reset{};
    reset.pin_bit_mask = 1ULL << IRIS_PIN_GPS_RESET;
    reset.mode = IRIS_GPS_RESET_ACTIVE_LOW ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT;
    reset.pull_up_en = GPIO_PULLUP_DISABLE;
    reset.pull_down_en = GPIO_PULLDOWN_DISABLE;
    reset.intr_type = GPIO_INTR_DISABLE;
    set_reset(false);
    esp_err_t result = gpio_config(&reset);
    if (result != ESP_OK) return result;
    set_reset(false);

    uart_config_t config{};
    config.baud_rate = static_cast<int>(IRIS_GPS_BAUD_RATE);
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_DEFAULT;
    result = uart_driver_install(IRIS_GPS_UART_PORT, kUartRxBufferSize,
                                 kUartTxBufferSize, 0, nullptr, 0);
    if (result != ESP_OK) return result;
    result = uart_param_config(IRIS_GPS_UART_PORT, &config);
    if (result == ESP_OK)
        result = uart_set_pin(IRIS_GPS_UART_PORT, IRIS_PIN_GPS_UART_TX,
                              IRIS_PIN_GPS_UART_RX, UART_PIN_NO_CHANGE,
                              UART_PIN_NO_CHANGE);
    if (result != ESP_OK) {
        (void)uart_driver_delete(IRIS_GPS_UART_PORT);
        return result;
    }

    // PPS is optional diagnostics; a failure here does not stop the GPS.
    gpio_config_t pps{};
    pps.pin_bit_mask = 1ULL << IRIS_PIN_GPS_PPS;
    pps.mode = GPIO_MODE_INPUT;
    pps.pull_up_en = GPIO_PULLUP_DISABLE;
    pps.pull_down_en = GPIO_PULLDOWN_ENABLE;
    pps.intr_type = GPIO_INTR_POSEDGE;
    esp_err_t pps_result = gpio_config(&pps);
    if (pps_result == ESP_OK) {
        pps_result = gpio_install_isr_service(0);
        if (pps_result == ESP_ERR_INVALID_STATE) pps_result = ESP_OK;  // Already installed.
    }
    if (pps_result == ESP_OK)
        pps_result = gpio_isr_handler_add(IRIS_PIN_GPS_PPS, pps_isr, this);
    if (pps_result != ESP_OK)
        ESP_LOGW(kTag, "PPS input unavailable: %s", esp_err_to_name(pps_result));

    if (xTaskCreate(task_entry, "gps", kTaskStackSize, this, kTaskPriority, &task_) != pdPASS) {
        task_ = nullptr;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

Lc86Nmea::State Lc86Gps::state() const {
    portENTER_CRITICAL(&lock_);
    const Lc86Nmea::State copy = published_;
    portEXIT_CRITICAL(&lock_);
    return copy;
}

std::uint32_t Lc86Gps::pps_count() const {
    portENTER_CRITICAL(&lock_);
    const std::uint32_t count = pps_count_;
    portEXIT_CRITICAL(&lock_);
    return count;
}

std::int64_t Lc86Gps::last_pps_us() const {
    portENTER_CRITICAL(&lock_);
    const std::int64_t time = last_pps_us_;
    portEXIT_CRITICAL(&lock_);
    return time;
}

void IRAM_ATTR Lc86Gps::pps_isr(void *context) {
    auto *self = static_cast<Lc86Gps *>(context);
    const std::int64_t time = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&self->lock_);
    self->pps_count_ = self->pps_count_ + 1U;
    self->last_pps_us_ = time;
    portEXIT_CRITICAL_ISR(&self->lock_);
}

void Lc86Gps::task_entry(void *context) {
    static_cast<Lc86Gps *>(context)->run();
}

void Lc86Gps::pump(const std::uint32_t wait_ms) {
    std::array<std::uint8_t, 256> buffer{};
    const int count = uart_read_bytes(IRIS_GPS_UART_PORT, buffer.data(), buffer.size(),
                                      pdMS_TO_TICKS(wait_ms));
    if (count <= 0) return;
    const std::uint32_t time = now_ms();
    rx_bytes_ += static_cast<std::uint32_t>(count);
    last_rx_ms_ = time;
    parser_.consume(buffer.data(), static_cast<std::size_t>(count), time);
    const Lc86Nmea::State &latest = parser_.state();
    portENTER_CRITICAL(&lock_);
    published_ = latest;
    portEXIT_CRITICAL(&lock_);
}

bool Lc86Gps::send_command(const char *body, const bool pair,
                           const std::uint16_t command, const bool not_supported_ok) {
    char sentence[96];
    const std::size_t length = Lc86Nmea::build_sentence(body, sentence, sizeof(sentence));
    if (length == 0) return false;
    for (int attempt = 0; attempt < kCommandAttempts; ++attempt) {
        const std::uint32_t ack_count = parser_.last_ack().count;
        if (uart_write_bytes(IRIS_GPS_UART_PORT, sentence, length) != static_cast<int>(length))
            continue;
        const std::uint32_t start = now_ms();
        while (now_ms() - start < kAckTimeoutMs) {
            pump(kReadSliceMs);
            const Lc86Nmea::Ack &ack = parser_.last_ack();
            if (ack.count == ack_count || ack.pair != pair || ack.command != command)
                continue;
            if (pair) {
                if (ack.result == kPairOk) return true;
                if (ack.result == kPairNotSupported) return not_supported_ok;
                if (ack.result == kPairProcessing) continue;  // Final ack follows.
            } else {
                if (ack.result == kPmtkOk) return true;
                if (ack.result == kPmtkNotSupported) return not_supported_ok;
            }
            break;  // Rejected; resend.
        }
    }
    return false;
}

void Lc86Gps::configure() {
    // PAIR (Airoha, LC86G): disable every standard sentence except GGA and RMC,
    // output those on every fix, then set a 100 ms fix interval. Settings go to
    // receiver RAM only and are reapplied each boot.
    struct Command { const char *body; std::uint16_t id; bool optional; };
    static constexpr Command kPair[] = {
        {"PAIR062,1,0", 62, false},  // GLL off
        {"PAIR062,2,0", 62, false},  // GSA off
        {"PAIR062,3,0", 62, false},  // GSV off
        {"PAIR062,5,0", 62, false},  // VTG off
        {"PAIR062,6,0", 62, true},   // ZDA off
        {"PAIR062,7,0", 62, true},   // GRS off
        {"PAIR062,8,0", 62, true},   // GST off
        {"PAIR062,9,0", 62, true},   // GNS off (unsupported on early firmware)
        {"PAIR062,0,1", 62, false},  // GGA every fix
        {"PAIR062,4,1", 62, false},  // RMC every fix
        {"PAIR050,100", 50, false},  // 100 ms fix interval = 10 Hz
    };
    const std::uint32_t acks_before = parser_.last_ack().count;
    bool all_ok = true;
    for (const Command &command : kPair) {
        const bool ok = send_command(command.body, true, command.id, command.optional);
        if (!ok) {
            all_ok = false;
            ESP_LOGW(kTag, "$%s not acknowledged", command.body);
        }
        // No PAIR reply at all to the first command: not a PAIR receiver.
        if (parser_.last_ack().count == acks_before) break;
    }
    if (parser_.last_ack().count != acks_before && parser_.last_ack().pair) {
        if (all_ok) ESP_LOGI(kTag, "LC86 configured (PAIR): 10 Hz GGA + RMC");
        else ESP_LOGW(kTag, "LC86 PAIR configuration incomplete; parsing whatever arrives");
        return;
    }

    // PMTK (MediaTek, LC86L) fallback: GGA + RMC every fix, 100 ms interval.
    const bool nmea_ok = send_command(
        "PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0", false, 314, false);
    const bool rate_ok = send_command("PMTK220,100", false, 220, false);
    if (nmea_ok && rate_ok) ESP_LOGI(kTag, "LC86 configured (PMTK): 10 Hz GGA + RMC");
    else ESP_LOGW(kTag, "LC86 configuration not acknowledged; parsing whatever arrives");
}

void Lc86Gps::log_diagnostic(const std::uint32_t time) {
    const Lc86Nmea::State &s = parser_.state();
    const bool fresh = Lc86Nmea::gga_fresh(s, time);
    ESP_LOGI(kTag,
             "rx=%lu B sentences=%lu bad=%lu quality=%u sats=%u hdop=%u fix_age_ms=%ld pps=%lu",
             static_cast<unsigned long>(rx_bytes_),
             static_cast<unsigned long>(parser_.sentence_count()),
             static_cast<unsigned long>(parser_.checksum_error_count()),
             fresh ? s.fix_quality : 0U, fresh ? s.satellites : 0U,
             fresh ? s.hdop_centi : Lc86Nmea::kHdopUnknown,
             s.fix ? static_cast<long>(time - s.fix_ms) : -1L,
             static_cast<unsigned long>(pps_count()));
    if (rx_bytes_ == 0 || time - last_rx_ms_ >= kDiagnosticIntervalMs)
        ESP_LOGW(kTag, "no UART data from LC86 at %lu baud",
                 static_cast<unsigned long>(IRIS_GPS_BAUD_RATE));
}

void Lc86Gps::run() {
    set_reset(true);
    vTaskDelay(pdMS_TO_TICKS(kResetPulseMs));
    set_reset(false);
    (void)uart_flush_input(IRIS_GPS_UART_PORT);

    // Wait for the receiver to boot (first checksum-valid sentence) before
    // configuring it; configure anyway after the timeout.
    const std::uint32_t boot_start = now_ms();
    while (parser_.sentence_count() == 0 && now_ms() - boot_start < kBootWaitMs)
        pump(kReadSliceMs);
    if (parser_.sentence_count() == 0)
        ESP_LOGW(kTag, "no NMEA within %lu ms of reset", static_cast<unsigned long>(kBootWaitMs));
    configure();

    std::uint32_t last_diagnostic = now_ms();
    bool had_fresh_fix = false;
    for (;;) {
        pump(kReadSliceMs);
        const std::uint32_t time = now_ms();
        const Lc86Nmea::State &s = parser_.state();
        const bool fresh_fix = Lc86Nmea::gga_fresh(s, time) && s.fix && s.fix_ms == s.gga_ms;
        if (fresh_fix != had_fresh_fix) {
            had_fresh_fix = fresh_fix;
            if (fresh_fix)
                ESP_LOGI(kTag, "fix: %.7f, %.7f, %.1f m MSL, %u sats",
                         s.latitude_degrees, s.longitude_degrees,
                         static_cast<double>(s.altitude_meters), s.satellites);
            else
                ESP_LOGW(kTag, "fix lost");
        }
        if (time - last_diagnostic >= kDiagnosticIntervalMs) {
            last_diagnostic = time;
            log_diagnostic(time);
        }
    }
}
