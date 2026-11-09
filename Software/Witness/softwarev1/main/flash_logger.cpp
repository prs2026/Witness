#include "flash_logger.h"
#include "task_cores.h"

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

// Flight states in which sensor and IMU records are written.
bool logged_state(std::uint8_t state) {
    return state == static_cast<std::uint8_t>(FlightStateMachine::State::Boost) ||
           state == static_cast<std::uint8_t>(FlightStateMachine::State::Coast) ||
           state == static_cast<std::uint8_t>(FlightStateMachine::State::Descent);
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

FlashLogger::ImuRecord FlashLogger::make_imu_record(
    const std::uint32_t timestamp_us, const std::int16_t low_g[3],
    const std::int16_t high_g[3], const std::int16_t gyro[3],
    const std::uint8_t status)
{
    ImuRecord record{};
    std::uint8_t *d = record.data;
    write_u32_be(d, timestamp_us);
    for (int i = 0; i < 3; ++i) {
        write_u16_be(d + 4 + 2 * i, static_cast<std::uint16_t>(low_g[i]));
        write_u16_be(d + 10 + 2 * i, static_cast<std::uint16_t>(high_g[i]));
        write_u16_be(d + 16 + 2 * i, static_cast<std::uint16_t>(gyro[i]));
    }
    d[22] = status;
    d[23] = 0;  // sequence, set by log_imu_record
    return record;
}

esp_err_t FlashLogger::start() {
    if (task_handle_) return ESP_ERR_INVALID_STATE;
    text_queue_ = xQueueCreate(32, sizeof(LogLine));
    binary_record_queue_ =
        xQueueCreate(kRecordQueueDepth, sizeof(BinaryRecord));
    imu_record_queue_ = xQueueCreate(kImuQueueDepth, sizeof(ImuRecord));
    uart_capture_mutex_ = xSemaphoreCreateMutex();
    uart_queue_ = xStreamBufferCreate(kUartReceiveBufferSize, 1);
    frozen_semaphore_ = xSemaphoreCreateBinary();
    erase_semaphore_ = xSemaphoreCreateBinary();
    if (!text_queue_ || !binary_record_queue_ || !imu_record_queue_ ||
        !uart_capture_mutex_ || !uart_queue_ || !frozen_semaphore_ ||
        !erase_semaphore_)
        return ESP_ERR_NO_MEM;
    install_log_capture();
    if (xTaskCreatePinnedToCore(capture_task_entry, "flash_capture",
                    kCaptureTaskStackSize, this, kCaptureTaskPriority,
                    &capture_task_handle_, task_cores::kSystem) != pdPASS) {
        capture_task_handle_ = nullptr; return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(task_entry, "flash_logger", kTaskStackSize, this,
                    kTaskPriority, &task_handle_, task_cores::kSystem) != pdPASS) {
        task_handle_ = nullptr;
        vTaskDelete(capture_task_handle_);
        capture_task_handle_ = nullptr;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t FlashLogger::log_external_bytes(
    const std::uint8_t *data,
    const std::size_t length)
{
    if (!data || !length) return ESP_ERR_INVALID_ARG;
    if (!uart_capture_mutex_) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(uart_capture_mutex_, portMAX_DELAY);
    esp_err_t result = ESP_ERR_INVALID_STATE;
    if (capture_uart_) {
        // All-or-nothing admission keeps overflow explicit to the caller.
        if (xStreamBufferSpacesAvailable(uart_queue_) >= length) {
            result = xStreamBufferSend(uart_queue_, data, length, 0)
                         == length ? ESP_OK : ESP_FAIL;
        } else {
            result = ESP_ERR_NO_MEM;
            witness_status_.set(IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK, true);
        }
    }
    xSemaphoreGive(uart_capture_mutex_);
    return result;
}

esp_err_t FlashLogger::log_imu_record(ImuRecord record)
{
    if (!imu_record_queue_ || !capture_imu_records_.load()) return ESP_ERR_INVALID_STATE;
    record.data[23] = imu_sequence_++;
    if (xQueueSend(imu_record_queue_, &record, 0) != pdTRUE) {
        imu_records_dropped_.fetch_add(1);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void FlashLogger::set_uart_capture_enabled(const bool enabled)
{
    xSemaphoreTake(uart_capture_mutex_, portMAX_DELAY);
    capture_uart_ = enabled;
    xSemaphoreGive(uart_capture_mutex_);
}

FlashLogger::FileLayout FlashLogger::file_layout(const std::uint8_t slot,
                                                 const std::uint8_t type)
{
    const std::uint32_t base = slot_base(slot) + kHeaderBlocks;
    switch (type) {
        case kImuType: return {base, kImuBlocks};
        case kBinaryType: return {base + kImuBlocks, kBinaryBlocks};
        case kTextType: return {base + kImuBlocks + kBinaryBlocks, kTextBlocks};
        default: return {base + kImuBlocks + kBinaryBlocks + kTextBlocks, kUartBlocks};
    }
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

esp_err_t FlashLogger::read_slot_info(const std::uint8_t slot, SlotInfo &info)
{
    info = {};
    const std::uint32_t first_page = slot_base(slot) * W25n01kv::kPagesPerBlock;
    std::uint8_t header[9]{};
    esp_err_t r = flash_.read(first_page, 0, header, sizeof(header));
    if (r != ESP_OK) return r;
    if (read_u32_be(header) != kSessionMagic || header[8] != kStoreVersion) return ESP_OK;
    info.valid = true;
    info.generation = read_u32_be(header + 4);
    std::uint8_t marker[8]{};
    r = flash_.read(first_page + kFlightMarkerPage, 0, marker, sizeof(marker));
    if (r != ESP_OK) return r;
    info.flight = read_u32_be(marker) == kFlightMagic &&
                  read_u32_be(marker + 4) == info.generation;
    return ESP_OK;
}

esp_err_t FlashLogger::scan_sessions(bool build_exports) {
    if (build_exports) {
        export_file_count_ = 0;
        invalidate_export_read_cache();
    }
    SlotInfo info[kSlotCount]{};
    for (std::uint8_t slot = 0; slot < kSlotCount; ++slot) {
        const esp_err_t r = read_slot_info(slot, info[slot]);
        if (r != ESP_OK) return r;
    }

    // Choose the slot for this boot's session: never the one holding the
    // newest flight; otherwise the empty or older slot.
    int newest_flight = -1;
    int newest = -1;
    for (int slot = 0; slot < static_cast<int>(kSlotCount); ++slot) {
        if (!info[slot].valid) continue;
        if (newest < 0 || static_cast<std::int32_t>(info[slot].generation - info[newest].generation) > 0)
            newest = slot;
        if (info[slot].flight &&
            (newest_flight < 0 ||
             static_cast<std::int32_t>(info[slot].generation - info[newest_flight].generation) > 0))
            newest_flight = slot;
    }
    if (newest_flight >= 0) {
        current_slot_ = static_cast<std::uint8_t>(1 - newest_flight);
    } else if (!info[0].valid) {
        current_slot_ = 0;
    } else if (!info[1].valid) {
        current_slot_ = 1;
    } else {
        current_slot_ = static_cast<std::uint8_t>(1 - newest);
    }
    current_generation_ = newest >= 0 ? info[newest].generation + 1 : 1;

    if (!build_exports) return ESP_OK;
    for (std::uint8_t slot = 0; slot < kSlotCount; ++slot) {
        if (!info[slot].valid) continue;
        for (std::uint8_t type : {kImuType, kBinaryType, kTextType, kUart1Type}) {
            if (export_file_count_ == kMaximumExportFiles) return ESP_ERR_NO_MEM;
            ExportFile &file = export_files_[export_file_count_];
            const char prefix = type == kImuType ? 'I' : type == kUart1Type ? 'U' : 'S';
            std::snprintf(file.name, sizeof(file.name), "%c%07lX.%s", prefix,
                          static_cast<unsigned long>(info[slot].generation),
                          type == kTextType ? "TXT" : "BIN");
            file.generation = info[slot].generation; file.slot = slot; file.type = type;
            file.store_version = kStoreVersion;
            const esp_err_t r = scan_file(slot, type, is_ring_type(type),
                                          info[slot].generation, file.tagged,
                                          file.size, file.start_page);
            if (r != ESP_OK) return r;
            ++export_file_count_;
        }
    }
    return ESP_OK;
}

esp_err_t FlashLogger::create_session() {
    const std::uint32_t base = slot_base(current_slot_);
    esp_err_t r = flash_.erase_block(base);  // header and flight marker
    for (std::uint8_t type : {kImuType, kBinaryType, kTextType, kUart1Type}) {
        if (r == ESP_OK) r = flash_.erase_block(file_layout(current_slot_, type).first_block);
    }
    if (r != ESP_OK) return r;
    std::uint8_t header[9]{};
    write_u32_be(header, kSessionMagic); write_u32_be(header + 4, current_generation_);
    header[8] = kStoreVersion;
    r = flash_.program(base * W25n01kv::kPagesPerBlock, 0, header, sizeof(header));
    if (r != ESP_OK) return r;
    auto writer = [this](std::uint8_t type, bool ring) {
        const auto layout = file_layout(current_slot_, type);
        Writer w{};
        w.type = type; w.first_block = layout.first_block; w.block_count = layout.block_count;
        w.page_index = 0; w.last_erased_block = layout.first_block; w.ring = ring;
        w.sequence = sequence_base(current_generation_);
        return w;
    };
    imu_writer_ = writer(kImuType, false);
    binary_writer_ = writer(kBinaryType, false);
    text_writer_ = writer(kTextType, is_ring_type(kTextType));
    uart_writer_ = writer(kUart1Type, is_ring_type(kUart1Type));
    flight_marker_written_ = false;
    flight_marker_pending_.store(false);
    return ESP_OK;
}

esp_err_t FlashLogger::write_flight_marker()
{
    std::uint8_t marker[8]{};
    write_u32_be(marker, kFlightMagic);
    write_u32_be(marker + 4, current_generation_);
    const std::uint32_t page = slot_base(current_slot_) * W25n01kv::kPagesPerBlock + kFlightMarkerPage;
    const esp_err_t r = flash_.program(page, 0, marker, sizeof(marker));
    if (r == ESP_OK) {
        flight_marker_written_ = true;
        ESP_LOGI(kLogTag, "flight marked in slot %c; this session is kept until the next flight",
                 'A' + current_slot_);
    }
    return r;
}

esp_err_t FlashLogger::erase_session_blocks()
{
    esp_err_t first_error = ESP_OK;
    constexpr std::uint32_t kLogBlockCount = kSlotCount * kBlocksPerSlot;

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
    imu_buffer_used_ = 0;
    text_buffer_used_ = 0;
    export_file_count_ = 0;
    xQueueReset(text_queue_);
    xQueueReset(binary_record_queue_);
    xQueueReset(imu_record_queue_);
    xStreamBufferReset(uart_queue_);
    uart_buffer_used_ = 0;
    imu_prelaunch_count_ = 0;
    imu_dump_pending_ = true;
    imu_descent_counter_ = 0;
    ++prelaunch_reset_generation_;
    return create_session();
}

esp_err_t FlashLogger::append_bytes(Writer &writer, const std::uint8_t *data, std::size_t length) {
    if (writer.full) return ESP_OK;
    const std::uint32_t capacity = writer.block_count * W25n01kv::kPagesPerBlock;
    const std::uint32_t pages_needed =
        static_cast<std::uint32_t>((length + kPagePayloadSize - 1) / kPagePayloadSize);
    if (!writer.ring && pages_needed > capacity - writer.page_index) {
        // Exhausting one allocation must not stop the other streams, e.g.
        // a full sensor file must not terminate an ongoing GPS recording.
        writer.full = true;
        witness_status_.set(IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK, true);
        if (writer.type == kBinaryType) capture_binary_records_.store(false);
        if (writer.type == kImuType) capture_imu_records_.store(false);
        if (writer.type == kTextType) capture_logs_.store(false);
        ESP_LOGE(kLogTag, "session log type %u is full; other streams continue", writer.type);
        return ESP_OK;
    }
    while (length) {
        if (writer.page_index >= capacity) {
            if (!writer.ring) return ESP_ERR_NO_MEM;
            // Circular stream: overwrite from the start, oldest block first.
            writer.page_index = 0;
            if (!writer.wrapped) {
                writer.wrapped = true;
                ESP_LOGW(kLogTag, "%s log ring wrapped; its oldest data is now overwritten",
                         writer.type == kTextType ? "text" : "GPS");
            }
        }
        const std::uint32_t block = writer.first_block + writer.page_index / W25n01kv::kPagesPerBlock;
        if (block != writer.last_erased_block) {
            esp_err_t r = flash_.erase_block(block); if (r != ESP_OK) return r;
            writer.last_erased_block = block;
        }
        const std::size_t chunk = std::min(length, kPagePayloadSize);
        std::uint8_t page[W25n01kv::kPageDataSize]; std::memset(page, 0xFF, sizeof(page));
        write_u32_be(page, kPageMagic); page[4] = writer.type; page[5] = kStoreVersion;
        write_u16_be(page + 6, static_cast<std::uint16_t>(chunk));
        write_u32_be(page + 8, writer.sequence++);
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

esp_err_t FlashLogger::append_imu_bytes(const std::uint8_t *data)
{
    if (imu_buffer_used_ + kImuRecordSize > sizeof(imu_buffer_)) {
        const esp_err_t r = flush_imu(); if (r != ESP_OK) return r;
    }
    std::memcpy(imu_buffer_ + imu_buffer_used_, data, kImuRecordSize);
    imu_buffer_used_ += kImuRecordSize;
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

esp_err_t FlashLogger::drain_imu_records()
{
    ImuRecord record{};
    while (xQueueReceive(imu_record_queue_, &record, 0) == pdTRUE) {
        const std::uint8_t state = record.data[22] & kImuStateMask;
        if (state == static_cast<std::uint8_t>(FlightStateMachine::State::PadIdle)) {
            // Keep the newest two seconds before launch.
            imu_prelaunch_[imu_prelaunch_write_] = record;
            imu_prelaunch_write_ = (imu_prelaunch_write_ + 1U) % kImuPrelaunchRecordCount;
            imu_prelaunch_count_ = std::min(imu_prelaunch_count_ + 1U, kImuPrelaunchRecordCount);
            imu_dump_pending_ = true;
            imu_descent_counter_ = 0;
            continue;
        }
        if (!logged_state(state)) continue;
        if (imu_dump_pending_) {
            const std::size_t oldest = (imu_prelaunch_write_ + kImuPrelaunchRecordCount -
                                        imu_prelaunch_count_) % kImuPrelaunchRecordCount;
            for (std::size_t i = 0; i < imu_prelaunch_count_; ++i) {
                const esp_err_t r = append_imu_bytes(
                    imu_prelaunch_[(oldest + i) % kImuPrelaunchRecordCount].data);
                if (r != ESP_OK) return r;
            }
            ESP_LOGI(kLogTag, "queued %u prelaunch IMU records (%lu ms) for flash",
                     static_cast<unsigned>(imu_prelaunch_count_),
                     static_cast<unsigned long>(imu_prelaunch_count_ * 1000U / kImuRateHz));
            imu_prelaunch_count_ = 0;
            imu_dump_pending_ = false;
            flight_marker_pending_.store(true);
        }
        // After apogee the IMU log keeps every kImuDescentDecimation-th record.
        if (state == static_cast<std::uint8_t>(FlightStateMachine::State::Descent) &&
            (imu_descent_counter_++ % kImuDescentDecimation) != 0) {
            continue;
        }
        const esp_err_t r = append_imu_bytes(record.data);
        if (r != ESP_OK) return r;
    }
    return ESP_OK;
}

esp_err_t FlashLogger::flush_binary() {
    if (!binary_buffer_used_) return ESP_OK;
    esp_err_t r = append_bytes(binary_writer_, binary_buffer_, binary_buffer_used_);
    if (r == ESP_OK) binary_buffer_used_ = 0;
    return r;
}

esp_err_t FlashLogger::flush_imu() {
    if (!imu_buffer_used_) return ESP_OK;
    const esp_err_t r = append_bytes(imu_writer_, imu_buffer_, imu_buffer_used_);
    if (r == ESP_OK) imu_buffer_used_ = 0;
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

esp_err_t FlashLogger::drain_uart_records(const bool flush)
{
    // Bound each pass so continuous input cannot starve other log streams.
    std::size_t remaining = xStreamBufferBytesAvailable(uart_queue_);
    while (remaining || (flush && uart_buffer_used_)) {
        auto &used = uart_buffer_used_;
        const auto amount = std::min(remaining, sizeof(uart_buffer_) - used);
        if (amount) {
            const auto received = xStreamBufferReceive(
                uart_queue_, uart_buffer_ + used, amount, 0);
            used += received;
            remaining -= received;
        }
        if (used == sizeof(uart_buffer_) || (flush && !remaining && used)) {
            const esp_err_t r = append_bytes(uart_writer_, uart_buffer_, used);
            if (r != ESP_OK) return r;
            used = 0;
        }
    }
    return ESP_OK;
}

esp_err_t FlashLogger::scan_file(const std::uint8_t slot, const std::uint8_t type,
                                 const bool ring, const std::uint32_t generation,
                                 bool &tagged, std::uint32_t &size,
                                 std::uint32_t &start_page)
{
    const auto layout = file_layout(slot, type);
    const std::uint32_t capacity = layout.block_count * W25n01kv::kPagesPerBlock;
    const std::uint32_t tag = sequence_tag(generation);
    std::uint8_t h[kPageHeaderSize]{};
    // Pass 1 counts only this session's pages. A file with none of them was
    // written before tagging (or is empty); pass 2 then reads it untagged, as
    // before, so sessions recorded by older firmware stay exportable.
    for (int pass = 0; pass < 2; ++pass) {
        tagged = pass == 0;
        size = 0;
        start_page = 0;
        bool found = false;
        std::uint32_t oldest_sequence = 0;
        for (std::uint32_t index = 0; index < capacity; ++index) {
            esp_err_t r = flash_.read(layout.first_block * W25n01kv::kPagesPerBlock + index, 0, h, sizeof(h));
            if (r != ESP_OK) return r;
            const std::uint16_t n = read_u16_be(h + 6);
            const std::uint32_t sequence = read_u32_be(h + 8);
            const bool valid = read_u32_be(h) == kPageMagic && h[4] == type &&
                               h[5] == kStoreVersion && n <= kPagePayloadSize &&
                               (!tagged || (sequence >> kSequenceTagShift) == tag);
            if (!valid) {
                if (!ring) break;  // linear files end at the first page not theirs
                continue;
            }
            size += n;
            if (!found || static_cast<std::int32_t>(sequence - oldest_sequence) < 0) {
                oldest_sequence = sequence;
                start_page = index;
                found = true;
            }
        }
        if (found) break;
    }
    return ESP_OK;
}

void FlashLogger::invalidate_export_read_cache()
{
    export_read_cache_valid_ = false;
    export_read_file_index_ = kMaximumExportFiles;
    export_read_step_ = 0;
    export_read_logical_offset_ = 0;
    export_read_payload_length_ = 0;
}

esp_err_t FlashLogger::load_export_page(
    const std::size_t file_index,
    std::uint32_t step,
    const std::uint32_t logical_offset)
{
    const ExportFile &file = export_files_[file_index];
    const auto layout = file_layout(file.slot, file.type);
    const std::uint32_t capacity = layout.block_count * W25n01kv::kPagesPerBlock;
    const bool ring = is_ring_type(file.type);
    // A ring file is read oldest page first, wrapping, skipping the erased
    // pages around the write position.
    for (; step < capacity; ++step) {
        const std::uint32_t index = ring ? (file.start_page + step) % capacity : step;
        const esp_err_t result = flash_.read(
            layout.first_block * W25n01kv::kPagesPerBlock + index, 0,
            export_read_page_, sizeof(export_read_page_));
        if (result != ESP_OK) return result;
        const std::uint16_t payload_length = read_u16_be(export_read_page_ + 6);
        const bool valid = read_u32_be(export_read_page_) == kPageMagic &&
                           export_read_page_[4] == file.type &&
                           export_read_page_[5] == file.store_version &&
                           payload_length <= kPagePayloadSize &&
                           (!file.tagged ||
                            (read_u32_be(export_read_page_ + 8) >> kSequenceTagShift) ==
                                sequence_tag(file.generation));
        if (!valid) {
            if (!ring) return ESP_ERR_INVALID_RESPONSE;
            continue;
        }
        export_read_file_index_ = file_index;
        export_read_step_ = step;
        export_read_logical_offset_ = logical_offset;
        export_read_payload_length_ = payload_length;
        export_read_cache_valid_ = true;
        return ESP_OK;
    }
    return ESP_ERR_INVALID_SIZE;
}

esp_err_t FlashLogger::read_file_data(
    const std::size_t file_index,
    std::uint32_t offset,
    std::uint8_t *destination,
    std::size_t length)
{
    // Rewind only for a non-sequential request that precedes the cached page.
    if (!export_read_cache_valid_ ||
        export_read_file_index_ != file_index ||
        offset < export_read_logical_offset_) {
        invalidate_export_read_cache();
        esp_err_t result = load_export_page(file_index, 0, 0);
        if (result != ESP_OK) return result;
    }

    while (length > 0) {
        const std::uint32_t cached_end =
            export_read_logical_offset_ + export_read_payload_length_;
        if (offset >= cached_end) {
            const esp_err_t result = load_export_page(
                file_index, export_read_step_ + 1U, cached_end);
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
            flight_marker_pending_.store(true);
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
        set_uart_capture_enabled(false);
        capture_binary_records_.store(false);
        capture_imu_records_.store(false);
        witness_status_.set(IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK, true);
        ESP_LOGE(kLogTag, "flash initialization failed: %s", esp_err_to_name(r));
        capture_logs_.store(false); task_handle_ = nullptr; vTaskDelete(nullptr); return;
    }
    capture_imu_records_.store(true);
    witness_status_.set(IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK, false);
    witness_status_.set(IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK, false);
    ESP_LOGI(kLogTag,
             "session %lu in slot %c (store v%u); blocks: IMU %lu, sensors %lu, text %lu, "
             "GPS ring %lu; flush interval %lu ms",
             static_cast<unsigned long>(current_generation_), 'A' + current_slot_,
             kStoreVersion, static_cast<unsigned long>(kImuBlocks),
             static_cast<unsigned long>(kBinaryBlocks), static_cast<unsigned long>(kTextBlocks),
             static_cast<unsigned long>(kUartBlocks), static_cast<unsigned long>(kFlushIntervalMs));
    TickType_t next = xTaskGetTickCount();
    TickType_t last_flush = next;
    std::uint32_t reported_drops = 0;
    for (;;) {
        if (erase_requested_.exchange(false)) {
            set_uart_capture_enabled(false);
            capture_binary_records_.store(false);
            capture_imu_records_.store(false);
            // Let an in-progress capture finish, then discard both queued and
            // pre-trigger records as part of the requested full erase.
            vTaskDelay(kLogPeriod);
            xQueueReset(binary_record_queue_);
            capture_logs_.store(false);
            erase_result_ = erase_session_blocks();
            capture_logs_.store(erase_result_ == ESP_OK);
            capture_binary_records_.store(erase_result_ == ESP_OK);
            capture_imu_records_.store(erase_result_ == ESP_OK);
            set_uart_capture_enabled(erase_result_ == ESP_OK);
            xSemaphoreGive(erase_semaphore_);
            if (erase_result_ != ESP_OK) {
                r = erase_result_;
            } else {
                ESP_LOGI(kLogTag, "all stored sessions erased; new session 1 created");
                last_flush = xTaskGetTickCount();
            }
        }
        if (export_requested_.load()) {
            set_uart_capture_enabled(false);
            capture_binary_records_.store(false);
            capture_imu_records_.store(false);
            // Drain once to release any producer waiting for queue space, wait
            // for the capture task to observe the stop flag, then drain again
            // so the exported file includes every record captured beforehand.
            r = drain_binary_records();
            if (r == ESP_OK) r = drain_imu_records();
            vTaskDelay(kLogPeriod);
            if (r == ESP_OK) r = drain_binary_records();
            if (r == ESP_OK) r = drain_imu_records();
            capture_logs_.store(false);
            if (r == ESP_OK && flight_marker_pending_.load() && !flight_marker_written_)
                r = write_flight_marker();
            if (r == ESP_OK) r = flush_binary();
            if (r == ESP_OK) r = flush_imu();
            if (r == ESP_OK) r = flush_text();
            if (r == ESP_OK) r = drain_uart_records(true);
            if (r == ESP_OK) r = scan_sessions(true);
            if (r == ESP_OK) {
                frozen_.store(true); xSemaphoreGive(frozen_semaphore_);
                for (;;) vTaskDelay(portMAX_DELAY);
            }
        }
        if (r == ESP_OK) r = drain_binary_records();
        if (r == ESP_OK) r = drain_imu_records();
        if (r == ESP_OK && flight_marker_pending_.load() && !flight_marker_written_)
            r = write_flight_marker();
        if (r == ESP_OK) r = drain_uart_records(false);
        const TickType_t now = xTaskGetTickCount();
        if (r == ESP_OK && now - last_flush >= pdMS_TO_TICKS(kFlushIntervalMs)) {
            r = flush_binary();
            if (r == ESP_OK) r = flush_imu();
            if (r == ESP_OK) r = flush_text();
            if (r == ESP_OK) r = drain_uart_records(true);
            last_flush = now;
            const std::uint32_t drops = imu_records_dropped_.load();
            if (drops != reported_drops) {
                ESP_LOGW(kLogTag, "IMU log queue full: %lu records dropped so far",
                         static_cast<unsigned long>(drops));
                reported_drops = drops;
            }
        }
        if (r != ESP_OK) {
            set_uart_capture_enabled(false);
            capture_binary_records_.store(false);
            capture_imu_records_.store(false);
            witness_status_.set(IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK, true);
            ESP_LOGE(kLogTag, "flash logging stopped: %s", esp_err_to_name(r));
            capture_logs_.store(false); task_handle_ = nullptr; vTaskDelete(nullptr); return;
        }
        vTaskDelayUntil(&next, kLogPeriod);
    }
}
