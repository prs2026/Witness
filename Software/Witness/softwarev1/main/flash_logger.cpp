#include "flash_logger.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "comms.h"
#include "esp_timer.h"
#include "flight_state_machine.h"
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

FlashLogger::FlashLogger(Sensors &s,
                         WitnessStatus &w,
                         FlightStateMachine &state_machine)
    : sensors_(s),
      witness_status_(w),
      flight_state_machine_(state_machine)
{
}

esp_err_t FlashLogger::start() {
    if (task_handle_) return ESP_ERR_INVALID_STATE;
    text_queue_ = xQueueCreate(32, sizeof(LogLine));
    binary_record_queue_ =
        xQueueCreate(kRecordQueueDepth, sizeof(BinaryRecord));
    frozen_semaphore_ = xSemaphoreCreateBinary();
    erase_semaphore_ = xSemaphoreCreateBinary();
    if (!text_queue_ || !binary_record_queue_ || !frozen_semaphore_ ||
        !erase_semaphore_)
        return ESP_ERR_NO_MEM;
    install_log_capture();
    if (xTaskCreate(capture_task_entry, "flash_capture",
                    kCaptureTaskStackSize, this, kCaptureTaskPriority,
                    &capture_task_handle_) != pdPASS) {
        capture_task_handle_ = nullptr; return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(task_entry, "flash_logger", kTaskStackSize, this,
                    kTaskPriority, &task_handle_) != pdPASS) {
        task_handle_ = nullptr;
        vTaskDelete(capture_task_handle_);
        capture_task_handle_ = nullptr;
        return ESP_ERR_NO_MEM;
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
void FlashLogger::capture_task_entry(void *context) { static_cast<FlashLogger *>(context)->run_capture(); }

esp_err_t FlashLogger::run_startup_self_test() {
    std::uint32_t block = W25n01kv::kBlockCount;
    for (std::uint32_t candidate = W25n01kv::kBlockCount;
         candidate-- > W25n01kv::kBlockCount - kReservedBlocks;) {
        bool bad = false;
        esp_err_t r = flash_.is_bad_block(candidate, bad);
        if (r != ESP_OK) return r;
        if (!bad) { block = candidate; break; }
    }
    if (block == W25n01kv::kBlockCount) return ESP_ERR_NOT_FOUND;

    std::uint8_t expected[32]{};
    std::uint8_t actual[32]{};
    for (std::size_t i = 0; i < sizeof(expected); ++i)
        expected[i] = static_cast<std::uint8_t>(0xA5U ^ (i * 0x1DU));
    esp_err_t r = flash_.erase_block(block);
    const std::uint32_t page = block * W25n01kv::kPagesPerBlock;
    if (r == ESP_OK) r = flash_.program(page, 0, expected, sizeof(expected));
    if (r == ESP_OK) r = flash_.read(page, 0, actual, sizeof(actual));
    if (r != ESP_OK) return r;
    ESP_LOGI(kLogTag, "startup test block %lu wrote/read:",
             static_cast<unsigned long>(block));
    ESP_LOG_BUFFER_HEX_LEVEL(kLogTag, actual, sizeof(actual), ESP_LOG_INFO);
    return std::memcmp(expected, actual, sizeof(expected)) == 0
               ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t FlashLogger::scan_sessions(bool build_exports) {
    std::uint32_t newest = 0;
    std::uint8_t newest_slot = 0;
    bool found = false;
    if (build_exports) {
        export_file_count_ = 0;
        invalidate_export_read_cache();
    }
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

esp_err_t FlashLogger::erase_session_blocks()
{
    esp_err_t first_error = ESP_OK;
    constexpr std::uint32_t kLogBlockCount =
        kSessionCount * kBlocksPerSession;

    for (std::uint32_t block = 0; block < kLogBlockCount; ++block) {
        bool bad = false;
        esp_err_t result = flash_.is_bad_block(block, bad);
        if (result == ESP_OK && !bad) {
            result = flash_.erase_block(block);
        }
        if (result != ESP_OK && first_error == ESP_OK) {
            first_error = result;
        }
    }
    if (first_error != ESP_OK) {
        return first_error;
    }

    current_slot_ = 0;
    current_generation_ = 1;
    binary_buffer_used_ = 0;
    text_buffer_used_ = 0;
    export_file_count_ = 0;
    xQueueReset(text_queue_);
    xQueueReset(binary_record_queue_);
    ++prelaunch_reset_generation_;
    return create_session();
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

esp_err_t FlashLogger::append_record(const BinaryRecord &record) {
    if (binary_buffer_used_ + kRecordSize > sizeof(binary_buffer_)) {
        esp_err_t r = flush_binary(); if (r != ESP_OK) return r;
    }
    std::memcpy(
        binary_buffer_ + binary_buffer_used_, record.data, kRecordSize);
    binary_buffer_used_ += kRecordSize;
    return ESP_OK;
}

esp_err_t FlashLogger::drain_binary_records()
{
    BinaryRecord record{};
    while (xQueueReceive(binary_record_queue_, &record, 0) == pdTRUE) {
        const esp_err_t result = append_record(record);
        if (result != ESP_OK) {
            return result;
        }
    }
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
        if (text_buffer_used_ + line.length > sizeof(text_buffer_)) {
            esp_err_t r = append_bytes(text_writer_, text_buffer_, text_buffer_used_);
            if (r != ESP_OK) return r;
            text_buffer_used_ = 0;
        }
        std::memcpy(text_buffer_ + text_buffer_used_, line.data, line.length);
        text_buffer_used_ += line.length;
    }
    if (!text_buffer_used_) return ESP_OK;
    const esp_err_t r = append_bytes(text_writer_, text_buffer_, text_buffer_used_);
    if (r == ESP_OK) text_buffer_used_ = 0;
    return r;
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

void FlashLogger::invalidate_export_read_cache()
{
    export_read_cache_valid_ = false;
    export_read_file_index_ = kMaximumExportFiles;
    export_read_page_index_ = 0;
    export_read_logical_offset_ = 0;
    export_read_payload_length_ = 0;
}

esp_err_t FlashLogger::load_export_page(
    const std::size_t file_index,
    const std::uint32_t page_index,
    const std::uint32_t logical_offset)
{
    const ExportFile &file = export_files_[file_index];
    const std::uint32_t session_base = file.slot * kBlocksPerSession;
    const std::uint32_t first_block =
        session_base +
        (file.type == kBinaryType ? 0 : kBinaryBlocksPerSession);
    const std::uint32_t block_count =
        file.type == kBinaryType
            ? kBinaryBlocksPerSession
            : kTextBlocksPerSession;
    if (page_index >= block_count * W25n01kv::kPagesPerBlock) {
        return ESP_ERR_INVALID_SIZE;
    }

    const esp_err_t result = flash_.read(
        first_block * W25n01kv::kPagesPerBlock + page_index,
        0,
        export_read_page_,
        sizeof(export_read_page_));
    if (result != ESP_OK) return result;

    const std::uint16_t payload_length =
        read_u16_be(export_read_page_ + 6);
    if (read_u32_be(export_read_page_) != kPageMagic ||
        export_read_page_[4] != file.type ||
        export_read_page_[5] != kStoreVersion ||
        payload_length > kPagePayloadSize) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    export_read_file_index_ = file_index;
    export_read_page_index_ = page_index;
    export_read_logical_offset_ = logical_offset;
    export_read_payload_length_ = payload_length;
    export_read_cache_valid_ = true;
    return ESP_OK;
}

esp_err_t FlashLogger::read_file_data(
    const std::size_t file_index,
    std::uint32_t offset,
    std::uint8_t *destination,
    std::size_t length)
{
    const ExportFile &f = export_files_[file_index];
    const std::uint32_t block_count = f.type == kBinaryType ? kBinaryBlocksPerSession : kTextBlocksPerSession;
    const std::uint32_t first_page = f.type == kBinaryType ? 1 : 0;
    const std::uint32_t page_limit =
        block_count * W25n01kv::kPagesPerBlock;
    // Rewind only for a non-sequential request that precedes the cached page.
    if (!export_read_cache_valid_ ||
        export_read_file_index_ != file_index ||
        offset < export_read_logical_offset_) {
        invalidate_export_read_cache();
        esp_err_t result = load_export_page(file_index, first_page, 0);
        if (result != ESP_OK) return result;
    }

    while (length > 0) {
        const std::uint32_t cached_end =
            export_read_logical_offset_ + export_read_payload_length_;
        if (offset >= cached_end) {
            if (export_read_page_index_ + 1U >= page_limit) {
                return ESP_ERR_INVALID_SIZE;
            }
            const esp_err_t result = load_export_page(
                file_index,
                export_read_page_index_ + 1U,
                cached_end);
            if (result != ESP_OK) return result;
            continue;
        }

        const std::size_t in_page =
            offset - export_read_logical_offset_;
        const std::size_t amount = std::min<std::size_t>(
            length, export_read_payload_length_ - in_page);
        std::memcpy(
            destination,
            export_read_page_ + kPageHeaderSize + in_page,
            amount);
        destination += amount;
        offset += amount;
        length -= amount;
    }
    return ESP_OK;
}

esp_err_t FlashLogger::prepare_mass_storage(TickType_t timeout) {
    if (frozen_.load()) return ESP_OK;
    if (!task_handle_) return ESP_ERR_INVALID_STATE;
    export_requested_.store(true);
    return xSemaphoreTake(frozen_semaphore_, timeout) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t FlashLogger::erase_all_sessions(const TickType_t timeout)
{
    if (frozen_.load() || !task_handle_) return ESP_ERR_INVALID_STATE;
    bool expected = false;
    if (!erase_requested_.compare_exchange_strong(expected, true)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(erase_semaphore_, timeout) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return erase_result_;
}

std::size_t FlashLogger::export_file_count() const { return export_file_count_; }
const FlashLogger::ExportFile &FlashLogger::export_file(std::size_t i) const { return export_files_[i]; }
esp_err_t FlashLogger::read_export_file(std::size_t i, std::uint32_t o, std::uint8_t *d, std::size_t n) {
    if (!frozen_.load() || i >= export_file_count_ || !d || o + n > export_files_[i].size) return ESP_ERR_INVALID_ARG;
    return read_file_data(i, o, d, n);
}
void FlashLogger::disable_console_output() { console_output_enabled_.store(false); }

void FlashLogger::run_capture()
{
    TickType_t next = xTaskGetTickCount();
    std::uint32_t observed_reset_generation =
        prelaunch_reset_generation_.load();

    for (;;) {
        vTaskDelayUntil(&next, kLogPeriod);

        const std::uint32_t reset_generation =
            prelaunch_reset_generation_.load();
        if (reset_generation != observed_reset_generation) {
            observed_reset_generation = reset_generation;
            prelaunch_write_index_ = 0;
            prelaunch_record_count_ = 0;
            prelaunch_dump_pending_ = true;
            sequence_ = 0;
        }

        if (!capture_binary_records_.load()) {
            continue;
        }

        BinaryRecord record{};
        serialize_record(record.data);
        const FlightStateMachine::State state = flight_state_machine_.state();

        if (state == FlightStateMachine::State::PadIdle) {
            prelaunch_records_[prelaunch_write_index_] = record;
            prelaunch_write_index_ =
                (prelaunch_write_index_ + 1U) % kPrelaunchRecordCount;
            prelaunch_record_count_ = std::min(
                prelaunch_record_count_ + 1U, kPrelaunchRecordCount);
            prelaunch_dump_pending_ = true;
            continue;
        }

        if (!flight_state_machine_.binary_logging_enabled()) {
            continue;
        }

        if (prelaunch_dump_pending_) {
            const std::size_t oldest_record =
                (prelaunch_write_index_ + kPrelaunchRecordCount -
                 prelaunch_record_count_) %
                kPrelaunchRecordCount;
            for (std::size_t offset = 0; offset < prelaunch_record_count_;
                 ++offset) {
                const std::size_t index =
                    (oldest_record + offset) % kPrelaunchRecordCount;
                (void)xQueueSend(
                    binary_record_queue_, &prelaunch_records_[index],
                    portMAX_DELAY);
            }
            ESP_LOGI(
                kLogTag,
                "queued %u prelaunch records (%lu ms) for flash",
                static_cast<unsigned>(prelaunch_record_count_),
                static_cast<unsigned long>(
                    prelaunch_record_count_ * 1000U / kLogRateHz));
            prelaunch_record_count_ = 0;
            prelaunch_dump_pending_ = false;
        }

        // The live record is queued after the prelaunch snapshot. The writer
        // may be programming NAND concurrently, so samples continue to be
        // captured at 50 Hz while the snapshot drains.
        (void)xQueueSend(binary_record_queue_, &record, portMAX_DELAY);
    }
}

void FlashLogger::run() {
    esp_err_t r = flash_.initialize();
    if (r == ESP_OK) r = run_startup_self_test();
    if (r == ESP_OK) r = scan_sessions(false);
    if (r == ESP_OK) r = create_session();
    if (r != ESP_OK) {
        capture_binary_records_.store(false);
        witness_status_.set(IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK, true);
        ESP_LOGE(kLogTag, "flash initialization failed: %s", esp_err_to_name(r));
        capture_logs_.store(false); task_handle_ = nullptr; vTaskDelete(nullptr); return;
    }
    witness_status_.set(IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK, false);
    witness_status_.set(IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK, false);
    ESP_LOGI(kLogTag, "session %lu started in slot %u; flush interval %lu ms",
             static_cast<unsigned long>(current_generation_), current_slot_,
             static_cast<unsigned long>(kFlushIntervalMs));
    TickType_t next = xTaskGetTickCount();
    TickType_t last_flush = next;
    for (;;) {
        if (erase_requested_.exchange(false)) {
            capture_binary_records_.store(false);
            // Let an in-progress capture finish, then discard both queued and
            // pre-trigger records as part of the requested full erase.
            vTaskDelay(kLogPeriod);
            xQueueReset(binary_record_queue_);
            capture_logs_.store(false);
            erase_result_ = erase_session_blocks();
            capture_logs_.store(erase_result_ == ESP_OK);
            capture_binary_records_.store(erase_result_ == ESP_OK);
            xSemaphoreGive(erase_semaphore_);
            if (erase_result_ != ESP_OK) {
                r = erase_result_;
            } else {
                ESP_LOGI(kLogTag, "all stored sessions erased; new session 1 created");
                last_flush = xTaskGetTickCount();
            }
        }
        if (export_requested_.load()) {
            capture_binary_records_.store(false);
            // Drain once to release any producer waiting for queue space, wait
            // for the capture task to observe the stop flag, then drain again
            // so the exported file includes every record captured beforehand.
            r = drain_binary_records();
            vTaskDelay(kLogPeriod);
            if (r == ESP_OK) r = drain_binary_records();
            capture_logs_.store(false);
            if (r == ESP_OK) r = flush_binary();
            if (r == ESP_OK) r = flush_text();
            if (r == ESP_OK) r = scan_sessions(true);
            if (r == ESP_OK) {
                frozen_.store(true); xSemaphoreGive(frozen_semaphore_);
                for (;;) vTaskDelay(portMAX_DELAY);
            }
        }
        if (r == ESP_OK) r = drain_binary_records();
        const TickType_t now = xTaskGetTickCount();
        if (r == ESP_OK && now - last_flush >= pdMS_TO_TICKS(kFlushIntervalMs)) {
            r = flush_binary(); if (r == ESP_OK) r = flush_text(); last_flush = now;
        }
        if (r != ESP_OK) {
            capture_binary_records_.store(false);
            witness_status_.set(IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK, true);
            ESP_LOGE(kLogTag, "flash logging stopped: %s", esp_err_to_name(r));
            capture_logs_.store(false); task_handle_ = nullptr; vTaskDelete(nullptr); return;
        }
        vTaskDelayUntil(&next, kLogPeriod);
    }
}
