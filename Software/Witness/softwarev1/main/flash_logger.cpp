#include "flash_logger.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "comms.h"
#include "esp_timer.h"
#include "sensors.h"
#include "witness_status.h"

namespace {
constexpr char kLogTag[] = "flash_logger";

void write_u16_be(std::uint8_t *p, std::uint16_t v) {
    p[0] = static_cast<std::uint8_t>(v >> 8); p[1] = static_cast<std::uint8_t>(v);
}
void write_u32_be(std::uint8_t *p, std::uint32_t v) {
    p[0] = static_cast<std::uint8_t>(v >> 24); p[1] = static_cast<std::uint8_t>(v >> 16);
    p[2] = static_cast<std::uint8_t>(v >> 8); p[3] = static_cast<std::uint8_t>(v);
}
std::uint16_t read_u16_be(const std::uint8_t *p) {
    return static_cast<std::uint16_t>((p[0] << 8) | p[1]);
}
std::uint32_t read_u32_be(const std::uint8_t *p) {
    return (static_cast<std::uint32_t>(p[0]) << 24) |
           (static_cast<std::uint32_t>(p[1]) << 16) |
           (static_cast<std::uint32_t>(p[2]) << 8) | p[3];
}
void write_float_be(std::uint8_t *p, float v) {
    std::uint32_t bits; std::memcpy(&bits, &v, sizeof(bits)); write_u32_be(p, bits);
}
} // namespace

FlashLogger *FlashLogger::log_sink_ = nullptr;

FlashLogger::FlashLogger(Sensors &s, WitnessStatus &w) : sensors_(s), witness_status_(w) {}

esp_err_t FlashLogger::start() {
    if (task_handle_) return ESP_ERR_INVALID_STATE;
    text_queue_ = xQueueCreate(32, sizeof(LogLine));
    frozen_semaphore_ = xSemaphoreCreateBinary();
    if (!text_queue_ || !frozen_semaphore_) return ESP_ERR_NO_MEM;
    install_log_capture();
    if (xTaskCreate(task_entry, "flash_logger", kTaskStackSize, this,
                    kTaskPriority, &task_handle_) != pdPASS) {
        task_handle_ = nullptr; return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void FlashLogger::install_log_capture() {
    log_sink_ = this;
    previous_vprintf_ = esp_log_set_vprintf(log_vprintf);
    capture_logs_.store(true);
}

int FlashLogger::log_vprintf(const char *format, va_list args) {
    FlashLogger *self = log_sink_;
    int result = 0;
    if (self && self->console_output_enabled_.load() && self->previous_vprintf_) {
        va_list console_args; va_copy(console_args, args);
        result = self->previous_vprintf_(format, console_args);
        va_end(console_args);
    }
    if (self && self->capture_logs_.load() && self->text_queue_) {
        LogLine line{};
        va_list copy; va_copy(copy, args);
        const int count = std::vsnprintf(line.data, sizeof(line.data), format, copy);
        va_end(copy);
        if (count > 0) {
            line.length = static_cast<std::uint16_t>(
                std::min<int>(count, sizeof(line.data) - 1));
            xQueueSend(self->text_queue_, &line, 0);
        }
    }
    return result;
}

void FlashLogger::task_entry(void *context) { static_cast<FlashLogger *>(context)->run(); }

esp_err_t FlashLogger::scan_sessions(bool build_exports) {
    std::uint32_t newest = 0;
    std::uint8_t newest_slot = 0;
    bool found = false;
    if (build_exports) export_file_count_ = 0;
    for (std::uint8_t slot = 0; slot < kSessionCount; ++slot) {
        std::uint8_t header[9]{};
        const std::uint32_t page = slot * kBlocksPerSession * W25n01kv::kPagesPerBlock;
        esp_err_t r = flash_.read(page, 0, header, sizeof(header));
        if (r != ESP_OK || read_u32_be(header) != kSessionMagic || header[8] != kStoreVersion) continue;
        const std::uint32_t generation = read_u32_be(header + 4);
        if (!found || static_cast<std::int32_t>(generation - newest) > 0) {
            newest = generation; newest_slot = slot; found = true;
        }
        if (build_exports && export_file_count_ + 2 <= kMaximumExportFiles) {
            for (std::uint8_t type : {kBinaryType, kTextType}) {
                ExportFile &file = export_files_[export_file_count_];
                std::snprintf(file.name, sizeof(file.name), "S%07lX.%s",
                              static_cast<unsigned long>(generation),
                              type == kBinaryType ? "BIN" : "TXT");
                file.generation = generation; file.slot = slot; file.type = type;
                r = scan_file_size(slot, type, file.size);
                if (r != ESP_OK) return r;
                ++export_file_count_;
            }
        }
    }
    current_generation_ = found ? newest + 1 : 1;
    current_slot_ = found ? static_cast<std::uint8_t>((newest_slot + 1) % kSessionCount) : 0;
    return ESP_OK;
}

esp_err_t FlashLogger::create_session() {
    const std::uint32_t base = current_slot_ * kBlocksPerSession;
    esp_err_t r = flash_.erase_block(base);
    if (r == ESP_OK) r = flash_.erase_block(base + kBinaryBlocksPerSession);
    if (r != ESP_OK) return r;
    std::uint8_t header[9]{};
    write_u32_be(header, kSessionMagic); write_u32_be(header + 4, current_generation_);
    header[8] = kStoreVersion;
    r = flash_.program(base * W25n01kv::kPagesPerBlock, 0, header, sizeof(header));
    if (r != ESP_OK) return r;
    binary_writer_ = {kBinaryType, base, kBinaryBlocksPerSession, 1, base};
    text_writer_ = {kTextType, base + kBinaryBlocksPerSession,
                    kTextBlocksPerSession, 0, base + kBinaryBlocksPerSession};
    return ESP_OK;
}

esp_err_t FlashLogger::append_bytes(Writer &writer, const std::uint8_t *data, std::size_t length) {
    while (length) {
        if (writer.page_index >= writer.block_count * W25n01kv::kPagesPerBlock) return ESP_ERR_NO_MEM;
        const std::uint32_t block = writer.first_block + writer.page_index / W25n01kv::kPagesPerBlock;
        if (block != writer.last_erased_block) {
            esp_err_t r = flash_.erase_block(block); if (r != ESP_OK) return r;
            writer.last_erased_block = block;
        }
        const std::size_t chunk = std::min(length, kPagePayloadSize);
        std::uint8_t page[W25n01kv::kPageDataSize]; std::memset(page, 0xFF, sizeof(page));
        write_u32_be(page, kPageMagic); page[4] = writer.type; page[5] = kStoreVersion;
        write_u16_be(page + 6, static_cast<std::uint16_t>(chunk));
        std::memcpy(page + kPageHeaderSize, data, chunk);
        const std::uint32_t absolute_page = writer.first_block * W25n01kv::kPagesPerBlock + writer.page_index;
        esp_err_t r = flash_.program(absolute_page, 0, page, sizeof(page));
        if (r != ESP_OK) return r;
        ++writer.page_index; data += chunk; length -= chunk;
    }
    return ESP_OK;
}

void FlashLogger::serialize_record(std::uint8_t *d) {
    Sensors::Sample sample{};
    const bool available = sensors_.latest_sample(sample, 0) == ESP_OK;
    std::memset(d, 0, kRecordSize); write_u16_be(d, kRecordMagic);
    d[2] = kRecordVersion; d[3] = witness_status_.flags(); write_u32_be(d + 4, sequence_++);
    write_u32_be(d + 8, static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL));
    if (!available) return;
    std::size_t o = 12;
    for (auto v : sample.low_g_accel_raw) { write_u16_be(d + o, static_cast<std::uint16_t>(v)); o += 2; }
    for (auto v : sample.high_g_accel_raw) { write_u16_be(d + o, static_cast<std::uint16_t>(v)); o += 2; }
    for (auto v : sample.gyro_raw) { write_u16_be(d + o, static_cast<std::uint16_t>(v)); o += 2; }
    write_u16_be(d + 30, static_cast<std::uint16_t>(sample.temperature_raw));
    write_u32_be(d + 32, static_cast<std::uint32_t>(sample.pressure_centi_mbar));
    write_u32_be(d + 36, static_cast<std::uint32_t>(sample.ms5607_temperature_centi_celsius));
    write_float_be(d + 40, sample.barometric_altitude_meters);
    write_u32_be(d + 44, sample.battery_voltage_mv);
}

esp_err_t FlashLogger::append_sample() {
    if (binary_buffer_used_ + kRecordSize > sizeof(binary_buffer_)) {
        esp_err_t r = flush_binary(); if (r != ESP_OK) return r;
    }
    serialize_record(binary_buffer_ + binary_buffer_used_); binary_buffer_used_ += kRecordSize;
    return ESP_OK;
}

esp_err_t FlashLogger::flush_binary() {
    if (!binary_buffer_used_) return ESP_OK;
    esp_err_t r = append_bytes(binary_writer_, binary_buffer_, binary_buffer_used_);
    if (r == ESP_OK) binary_buffer_used_ = 0;
    return r;
}

esp_err_t FlashLogger::flush_text() {
    LogLine line{};
    while (xQueueReceive(text_queue_, &line, 0) == pdTRUE) {
        esp_err_t r = append_bytes(text_writer_, reinterpret_cast<const std::uint8_t *>(line.data), line.length);
        if (r != ESP_OK) return r;
    }
    return ESP_OK;
}

esp_err_t FlashLogger::scan_file_size(std::uint8_t slot, std::uint8_t type, std::uint32_t &size) {
    size = 0;
    const std::uint32_t session_base = slot * kBlocksPerSession;
    const std::uint32_t first_block = session_base + (type == kBinaryType ? 0 : kBinaryBlocksPerSession);
    const std::uint32_t block_count = type == kBinaryType ? kBinaryBlocksPerSession : kTextBlocksPerSession;
    std::uint32_t index = type == kBinaryType ? 1 : 0;
    std::uint8_t h[kPageHeaderSize]{};
    for (; index < block_count * W25n01kv::kPagesPerBlock; ++index) {
        esp_err_t r = flash_.read(first_block * W25n01kv::kPagesPerBlock + index, 0, h, sizeof(h));
        if (r != ESP_OK) return r;
        const std::uint16_t n = read_u16_be(h + 6);
        if (read_u32_be(h) != kPageMagic || h[4] != type || h[5] != kStoreVersion || n > kPagePayloadSize) break;
        size += n;
    }
    return ESP_OK;
}

esp_err_t FlashLogger::read_file_data(const ExportFile &f, std::uint32_t offset,
                                      std::uint8_t *destination, std::size_t length) {
    const std::uint32_t session_base = f.slot * kBlocksPerSession;
    const std::uint32_t first_block = session_base + (f.type == kBinaryType ? 0 : kBinaryBlocksPerSession);
    const std::uint32_t block_count = f.type == kBinaryType ? kBinaryBlocksPerSession : kTextBlocksPerSession;
    std::uint32_t page_index = f.type == kBinaryType ? 1 : 0;
    std::uint32_t logical = 0;
    std::uint8_t page[W25n01kv::kPageDataSize];
    while (length && page_index < block_count * W25n01kv::kPagesPerBlock) {
        esp_err_t r = flash_.read(first_block * W25n01kv::kPagesPerBlock + page_index++, 0, page, sizeof(page));
        if (r != ESP_OK) return r;
        const std::uint16_t n = read_u16_be(page + 6);
        if (read_u32_be(page) != kPageMagic || page[4] != f.type || n > kPagePayloadSize) return ESP_ERR_INVALID_RESPONSE;
        if (offset >= logical + n) { logical += n; continue; }
        const std::size_t in_page = offset > logical ? offset - logical : 0;
        const std::size_t take = std::min<std::size_t>(length, n - in_page);
        std::memcpy(destination, page + kPageHeaderSize + in_page, take);
        destination += take; length -= take; offset += take; logical += n;
    }
    return length == 0 ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

esp_err_t FlashLogger::prepare_mass_storage(TickType_t timeout) {
    if (frozen_.load()) return ESP_OK;
    if (!task_handle_) return ESP_ERR_INVALID_STATE;
    export_requested_.store(true);
    return xSemaphoreTake(frozen_semaphore_, timeout) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

std::size_t FlashLogger::export_file_count() const { return export_file_count_; }
const FlashLogger::ExportFile &FlashLogger::export_file(std::size_t i) const { return export_files_[i]; }
esp_err_t FlashLogger::read_export_file(std::size_t i, std::uint32_t o, std::uint8_t *d, std::size_t n) {
    if (!frozen_.load() || i >= export_file_count_ || !d || o + n > export_files_[i].size) return ESP_ERR_INVALID_ARG;
    return read_file_data(export_files_[i], o, d, n);
}
void FlashLogger::disable_console_output() { console_output_enabled_.store(false); }

void FlashLogger::run() {
    esp_err_t r = flash_.initialize();
    if (r == ESP_OK) r = scan_sessions(false);
    if (r == ESP_OK) r = create_session();
    if (r != ESP_OK) {
        witness_status_.set(IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK, true);
        ESP_LOGE(kLogTag, "flash initialization failed: %s", esp_err_to_name(r));
        capture_logs_.store(false); task_handle_ = nullptr; vTaskDelete(nullptr); return;
    }
    witness_status_.set(IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK, false);
    ESP_LOGI(kLogTag, "session %lu started in slot %u; flush interval %lu ms",
             static_cast<unsigned long>(current_generation_), current_slot_,
             static_cast<unsigned long>(kFlushIntervalMs));
    TickType_t next = xTaskGetTickCount();
    TickType_t last_flush = next;
    for (;;) {
        if (export_requested_.load()) {
            capture_logs_.store(false);
            r = flush_binary(); if (r == ESP_OK) r = flush_text();
            if (r == ESP_OK) r = scan_sessions(true);
            if (r == ESP_OK) {
                frozen_.store(true); xSemaphoreGive(frozen_semaphore_);
                for (;;) vTaskDelay(portMAX_DELAY);
            }
        }
        r = append_sample();
        const TickType_t now = xTaskGetTickCount();
        if (r == ESP_OK && now - last_flush >= pdMS_TO_TICKS(kFlushIntervalMs)) {
            r = flush_binary(); if (r == ESP_OK) r = flush_text(); last_flush = now;
        }
        if (r != ESP_OK) {
            witness_status_.set(IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK, true);
            ESP_LOGE(kLogTag, "flash logging stopped: %s", esp_err_to_name(r));
            capture_logs_.store(false); task_handle_ = nullptr; vTaskDelete(nullptr); return;
        }
        vTaskDelayUntil(&next, kLogPeriod);
    }
}
