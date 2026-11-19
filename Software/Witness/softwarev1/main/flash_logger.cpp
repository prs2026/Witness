#include "flash_logger.h"
#include "landed_logic.h"
#include "landed_power.h"
#include "task_cores.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "comms.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "flight_state_machine.h"
#include "radio_edge_logic.h"
#if !CONFIG_WITNESS_IMAGE_MAINTENANCE
#include "sensors.h"
#endif
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
    : sensors_(&s),
      witness_status_(w),
      flight_state_machine_(&state_machine)
{
}

FlashLogger::FlashLogger(WitnessStatus &w)
    : sensors_(nullptr),
      witness_status_(w),
      flight_state_machine_(nullptr)
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
    // main() restored the flight state before this call: a reset in flight is
    // known here, before any sensor task can change the state.
    resumed_in_flight_ = flight_state_machine_ != nullptr &&
                         logged_state(static_cast<std::uint8_t>(flight_state_machine_->state()));
    text_queue_ = xQueueCreate(32, sizeof(LogLine));
    binary_record_queue_ =
        xQueueCreate(kRecordQueueDepth, sizeof(BinaryRecord));
    imu_record_queue_ = xQueueCreate(kImuQueueDepth, sizeof(ImuRecord));
    packet_queue_ = xQueueCreate(kPacketQueueDepth, sizeof(PacketRecord));
    uart_capture_mutex_ = xSemaphoreCreateMutex();
    uart_queue_ = xStreamBufferCreate(kUartReceiveBufferSize, 1);
    frozen_semaphore_ = xSemaphoreCreateBinary();
    erase_semaphore_ = xSemaphoreCreateBinary();
    if (!text_queue_ || !binary_record_queue_ || !imu_record_queue_ || !packet_queue_ ||
        !uart_capture_mutex_ || !uart_queue_ || !frozen_semaphore_ ||
        !erase_semaphore_)
        return ESP_ERR_NO_MEM;
    install_log_capture();
    (void)esp_register_shutdown_handler(&FlashLogger::restart_flush_handler);
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

esp_err_t FlashLogger::log_packet(const std::uint8_t source,
                                  const std::uint8_t *frame, const std::size_t length)
{
    if (!frame || length == 0 || length > kMaxPacketFrame) return ESP_ERR_INVALID_ARG;
    if (!packet_queue_ || !capture_packets_.load()) return ESP_ERR_INVALID_STATE;
    PacketRecord record{};
    write_u32_be(record.data, static_cast<std::uint32_t>(esp_timer_get_time()));
    record.data[4] = source;
    record.data[5] = static_cast<std::uint8_t>(length);
    std::memcpy(record.data + kPacketRecordHeader, frame, length);
    record.length = static_cast<std::uint8_t>(kPacketRecordHeader + length);
    if (xQueueSend(packet_queue_, &record, 0) != pdTRUE) {
        packets_dropped_.fetch_add(1);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t FlashLogger::drain_packet_records()
{
    PacketRecord record{};
    while (xQueueReceive(packet_queue_, &record, 0) == pdTRUE) {
        if (packet_buffer_used_ + record.length > sizeof(packet_buffer_)) {
            const esp_err_t r = append_bytes(packet_writer_, packet_buffer_, packet_buffer_used_);
            if (r != ESP_OK) return r;
            packet_buffer_used_ = 0;
        }
        std::memcpy(packet_buffer_ + packet_buffer_used_, record.data, record.length);
        packet_buffer_used_ += record.length;
    }
    return ESP_OK;
}

esp_err_t FlashLogger::flush_packets()
{
    if (!packet_buffer_used_) return ESP_OK;
    const esp_err_t r = append_bytes(packet_writer_, packet_buffer_, packet_buffer_used_);
    if (r == ESP_OK) packet_buffer_used_ = 0;
    return r;
}

void FlashLogger::protect_rings_from_launch()
{
    for (Writer *writer : {&text_writer_, &uart_writer_, &packet_writer_}) {
        writer->launch_protected = true;
        writer->pages_since_launch = 0;
    }
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
                                                 const std::uint8_t type,
                                                 const std::uint8_t version)
{
    const std::uint32_t base = slot_base(slot) + kHeaderBlocks;
    const std::uint32_t after_text = base + kImuBlocks + kBinaryBlocks + kTextBlocks;
    switch (type) {
        case kImuType: return {base, kImuBlocks};
        case kBinaryType: return {base + kImuBlocks, kBinaryBlocks};
        case kTextType: return {base + kImuBlocks + kBinaryBlocks, kTextBlocks};
        case kPacketType:
            return version == kLegacyStoreVersion ? FileLayout{after_text, 0}
                                                  : FileLayout{after_text, kPacketBlocks};
        default:
            return version == kLegacyStoreVersion
                       ? FileLayout{after_text, kV3UartBlocks}
                       : FileLayout{after_text + kPacketBlocks, kUartBlocks};
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
void FlashLogger::export_task_entry(void *context) { static_cast<FlashLogger *>(context)->run_export_only(); }

esp_err_t FlashLogger::start_export_only() {
    if (task_handle_) return ESP_ERR_INVALID_STATE;
    frozen_semaphore_ = xSemaphoreCreateBinary();
    if (!frozen_semaphore_) return ESP_ERR_NO_MEM;
    // Idle priority: the one-time NAND scan takes seconds without blocking;
    // above idle it starved IDLE0 and tripped the task watchdog.
    if (xTaskCreatePinnedToCore(export_task_entry, "flash_export", kTaskStackSize, this,
                    tskIDLE_PRIORITY, &task_handle_, task_cores::kSystem) != pdPASS) {
        task_handle_ = nullptr;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void FlashLogger::run_export_only() {
    esp_err_t r = flash_.initialize();
    if (r == ESP_OK) r = scan_sessions(true);
    if (r != ESP_OK) {
        witness_status_.set(IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK, true);
        ESP_LOGE(kLogTag, "maintenance: flash read-only mount failed: %s", esp_err_to_name(r));
    } else {
        ESP_LOGW(kLogTag, "maintenance: flash mounted read-only, %u export file(s), NOT logging",
                 static_cast<unsigned>(export_file_count_));
        frozen_.store(true);
        xSemaphoreGive(frozen_semaphore_);
    }
    for (;;) vTaskDelay(portMAX_DELAY);
}
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

esp_err_t FlashLogger::read_page_retry(const std::uint32_t page, const std::uint16_t column,
                                       std::uint8_t *data, const std::size_t length)
{
    esp_err_t r = ESP_FAIL;
    for (int attempt = 0; attempt < 3; ++attempt) {
        r = flash_.read(page, column, data, length);
        // Uncorrectable ECC is a property of the page, not transient.
        if (r == ESP_OK || r == ESP_ERR_INVALID_CRC) break;
        if (attempt == 1) (void)flash_.recover();
    }
    return r;
}

// A header or marker page that is uncorrectable (torn by a power loss during its
// erase/program) must not fail the mount: the slot is decided from the rest of
// the evidence, and a torn header block is simply erased before it is reused.
//  - header torn, marker intact: the slot IS a flight (the marker holds the
//    generation); its header is rebuilt in RAM, the data stays readable.
//  - header torn, no marker: not a session (dirty): erased on reuse.
//  - marker torn, header intact: treated as a flight (the marker is only ever
//    programmed at launch), so the data is kept.
// Only a persistent non-ECC bus error is returned (the caller retries).
esp_err_t FlashLogger::read_slot_info(const std::uint8_t slot, SlotInfo &info)
{
    info = {};
    const std::uint32_t first_page = slot_base(slot) * W25n01kv::kPagesPerBlock;
    std::uint8_t header[9]{};
    std::uint8_t marker[8]{};
    esp_err_t r = read_page_retry(first_page, 0, header, sizeof(header));
    if (r == ESP_ERR_INVALID_CRC) {
        ++unreadable_pages_;
        info.dirty = true;
        r = read_page_retry(first_page + kFlightMarkerPage, 0, marker, sizeof(marker));
        if (r == ESP_OK && read_u32_be(marker) == kFlightMagic) {
            info.valid = true;
            info.flight = true;
            info.generation = read_u32_be(marker + 4);
            info.version = kStoreVersion;
            std::uint8_t page_header[kPageHeaderSize]{};
            const FileLayout imu = file_layout(slot, kImuType, kStoreVersion);
            if (read_page_retry(imu.first_block * W25n01kv::kPagesPerBlock, 0, page_header,
                                sizeof(page_header)) == ESP_OK &&
                read_u32_be(page_header) == kPageMagic && page_header[5] == kLegacyStoreVersion)
                info.version = kLegacyStoreVersion;
            ESP_LOGW(kLogTag, "slot %c header unreadable; rebuilt from the flight marker (generation %lu)",
                     'A' + slot, static_cast<unsigned long>(info.generation));
            return ESP_OK;
        }
        if (r == ESP_OK || r == ESP_ERR_INVALID_CRC) {
            ESP_LOGW(kLogTag, "slot %c header unreadable and no flight marker: slot treated as empty", 'A' + slot);
            return ESP_OK;
        }
        return r;
    }
    if (r != ESP_OK) return r;
    if (read_u32_be(header) != kSessionMagic ||
        (header[8] != kStoreVersion && header[8] != kLegacyStoreVersion)) return ESP_OK;
    info.valid = true;
    info.version = header[8];
    info.generation = read_u32_be(header + 4);
    r = read_page_retry(first_page + kFlightMarkerPage, 0, marker, sizeof(marker));
    if (r == ESP_ERR_INVALID_CRC) {
        ++unreadable_pages_;
        info.dirty = true;
        info.flight = true;
        ESP_LOGW(kLogTag, "slot %c flight marker unreadable: slot kept as a flight", 'A' + slot);
        return ESP_OK;
    }
    if (r != ESP_OK) return r;
    info.flight = read_u32_be(marker) == kFlightMagic &&
                  read_u32_be(marker + 4) == info.generation;
    return ESP_OK;
}

// Applies log_slot::choose() to the scanned slots.
void FlashLogger::choose_slot(const bool allow_append)
{
    log_slot::SlotState states[kSlotCount]{};
    for (std::uint8_t s = 0; s < kSlotCount; ++s) {
        states[s].valid = slot_info_[s].valid;
        states[s].flight = slot_info_[s].flight;
        states[s].appendable = slot_info_[s].version == kStoreVersion;
        states[s].generation = slot_info_[s].generation;
        states[s].unusable = slot_unusable_[s];
    }
    const log_slot::Choice c = log_slot::choose(states, allow_append && resumed_in_flight_);
    choice_action_ = c.action;
    slots_full_ = c.action == log_slot::Action::SlotsFull;
    current_slot_ = c.slot;
    current_generation_ = c.action == log_slot::Action::Append
        ? slot_info_[c.slot].generation
        : next_usable_generation(c.have_newest ? c.newest_generation + 1 : 1);
}

esp_err_t FlashLogger::scan_sessions(bool build_exports) {
    if (build_exports) {
        export_file_count_ = 0;
        unreadable_pages_ = 0;
        invalidate_export_read_cache();
    }
    SlotInfo info[kSlotCount]{};
    for (std::uint8_t slot = 0; slot < kSlotCount; ++slot) {
        const esp_err_t r = read_slot_info(slot, info[slot]);
        if (r != ESP_OK) return r;
    }
    for (std::uint8_t slot = 0; slot < kSlotCount; ++slot) slot_info_[slot] = info[slot];

    // Slot choice (pure logic, tested): a pad boot never uses a slot holding a
    // flight and logs nothing when both do; a boot that resumes a flight in
    // progress appends to its slot (resume_session). The export scan after
    // logging stopped must not change the choice.
    if (!build_exports) {
        choose_slot(true);
        return ESP_OK;
    }
    for (std::uint8_t slot = 0; slot < kSlotCount; ++slot) {
        if (!info[slot].valid) continue;
        for (std::uint8_t type : {kImuType, kBinaryType, kTextType, kPacketType, kUart1Type}) {
            if (type == kPacketType && info[slot].version == kLegacyStoreVersion) continue;
            if (export_file_count_ == kMaximumExportFiles) return ESP_ERR_NO_MEM;
            ExportFile &file = export_files_[export_file_count_];
            const char prefix = type == kImuType     ? 'I'
                                : type == kUart1Type  ? 'U'
                                : type == kPacketType ? 'P'
                                                      : 'S';
            std::snprintf(file.name, sizeof(file.name), "%c%07lX.%s", prefix,
                          static_cast<unsigned long>(info[slot].generation),
                          type == kTextType ? "TXT" : "BIN");
            file.generation = info[slot].generation; file.slot = slot; file.type = type;
            file.store_version = info[slot].version;
            const esp_err_t r = scan_file(slot, type, info[slot].version, is_ring_type(type),
                                          info[slot].generation, file.tagged,
                                          file.size, file.start_page);
            if (r != ESP_OK) return r;
            ++export_file_count_;
        }
    }
    if (unreadable_pages_ != 0)
        ESP_LOGW(kLogTag, "export scan skipped %lu unreadable page(s)",
                 static_cast<unsigned long>(unreadable_pages_));
    return ESP_OK;
}

bool FlashLogger::block_session_bad(const std::uint32_t block) const
{
    for (std::size_t i = 0; i < session_bad_count_; ++i)
        if (session_bad_[i] == block) return true;
    return false;
}

void FlashLogger::mark_session_bad(const std::uint32_t block)
{
    if (!block_session_bad(block) && session_bad_count_ < sizeof(session_bad_) / sizeof(session_bad_[0]))
        session_bad_[session_bad_count_++] = block;
}

// Erases `block` for use, skipping factory-bad blocks and blocks that already
// failed this boot; transient errors are retried (with a chip reset in the
// middle). False: the block cannot be used, the caller moves on.
bool FlashLogger::prepare_block(const std::uint32_t block)
{
    if (block_session_bad(block)) return false;
    bool bad = false;
    if (flash_.is_bad_block(block, bad) == ESP_OK && bad) {
        mark_session_bad(block);
        ESP_LOGW(kLogTag, "block %lu is a factory bad block: skipped", static_cast<unsigned long>(block));
        return false;
    }
    esp_err_t r = ESP_FAIL;
    for (unsigned attempt = 0; attempt < log_slot::kEraseAttempts; ++attempt) {
        r = flash_.erase_block(block);
        if (r == ESP_OK) return true;
        if (r == ESP_ERR_TIMEOUT) (void)flash_.wait_idle(200);
        if (attempt == 1) (void)flash_.recover();
    }
    mark_session_bad(block);
    ESP_LOGW(kLogTag, "block %lu: erase failed (%s): skipped this boot",
             static_cast<unsigned long>(block), esp_err_to_name(r));
    return false;
}

// Programs one page; transient errors are retried. A program failure (P_FAIL)
// is final for the page. After a timeout the chip is waited for and the page
// header read back, so a program that finished late is not repeated.
esp_err_t FlashLogger::program_page_robust(const std::uint32_t page, const std::uint8_t *data,
                                           const std::size_t length)
{
    esp_err_t r = ESP_FAIL;
    for (unsigned attempt = 0; attempt < log_slot::kProgramAttempts; ++attempt) {
        r = flash_.program(page, 0, data, length);
        if (r == ESP_OK || r == ESP_FAIL) return r;
        if (r == ESP_ERR_TIMEOUT) {
            (void)flash_.wait_idle(200);
            std::uint8_t readback[16]{};
            const std::size_t n = std::min<std::size_t>(length, sizeof(readback));
            if (read_page_retry(page, 0, readback, n) == ESP_OK &&
                std::memcmp(readback, data, n) == 0) return ESP_OK;
        }
        if (attempt == 1) (void)flash_.recover();
    }
    return r;
}

bool FlashLogger::any_stream_full() const
{
    return imu_writer_.full || binary_writer_.full || text_writer_.full ||
           uart_writer_.full || packet_writer_.full;
}

// FLASH_LOG_FAILED follows the NAND write health: set while blocks are failing,
// cleared once writes succeed again (unless a stream is full, which keeps it).
void FlashLogger::update_nand_status()
{
    const bool failing = write_health_.failing();
    if (failing && !nand_status_set_) {
        nand_status_set_ = true;
        witness_status_.set(IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK, true);
    } else if (!failing && nand_status_set_) {
        nand_status_set_ = false;
        if (!any_stream_full())
            witness_status_.set(IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK, false);
        ESP_LOGW(kLogTag, "NAND writes recovered (%lu block(s) skipped so far)",
                 static_cast<unsigned long>(write_health_.total_failed_blocks()));
    }
}

// Erase and clear only run on the ground (Pad Idle / Landed).
bool FlashLogger::ground_state() const
{
    if (flight_state_machine_ == nullptr) return true;
    const FlightStateMachine::State s = flight_state_machine_->state();
    return s == FlightStateMachine::State::PadIdle || s == FlightStateMachine::State::Landed;
}

// A reset in flight: continue the flight's own session instead of starting
// another. Per stream the last written page is found (log_slot::stream_end),
// the writer continues after it (never erasing or overwriting flight pages;
// a torn page is skipped), and the ring streams are launch-protected. Fails
// (caller falls back to the normal slot rule) on a read error or when no
// stream has any room left.
esp_err_t FlashLogger::resume_session()
{
    constexpr std::uint32_t kNoBlock = 0xFFFFFFFFU;
    const std::uint32_t tag = sequence_tag(current_generation_);
    bool probe_failed = false;
    bool any_room = false;
    struct Stream { std::uint8_t type; Writer *writer; };
    const Stream streams[] = {{kImuType, &imu_writer_}, {kBinaryType, &binary_writer_},
                              {kTextType, &text_writer_}, {kPacketType, &packet_writer_},
                              {kUart1Type, &uart_writer_}};
    Writer built[5]{};
    int index = 0;
    for (const Stream &s : streams) {
        const FileLayout layout = file_layout(current_slot_, s.type);
        const std::uint32_t capacity = layout.block_count * W25n01kv::kPagesPerBlock;
        const std::uint32_t base_page = layout.first_block * W25n01kv::kPagesPerBlock;
        const std::uint8_t type = s.type;
        auto probe = [&](const std::uint32_t i) {
            log_slot::PageProbe p{};
            std::uint8_t h[kPageHeaderSize]{};
            const esp_err_t r = read_page_retry(base_page + i, 0, h, sizeof(h));
            if (r == ESP_ERR_INVALID_CRC) { p.kind = log_slot::PageKind::Unreadable; return p; }
            if (r != ESP_OK) { probe_failed = true; p.kind = log_slot::PageKind::Foreign; return p; }
            bool erased = true;
            for (const std::uint8_t b : h) if (b != 0xFF) erased = false;
            if (erased) { p.kind = log_slot::PageKind::Erased; return p; }
            const std::uint32_t sequence = read_u32_be(h + 8);
            if (read_u32_be(h) == kPageMagic && h[4] == type && h[5] == kStoreVersion &&
                read_u16_be(h + 6) <= kPagePayloadSize &&
                (sequence >> kSequenceTagShift) == tag) {
                p.kind = log_slot::PageKind::Valid;
                p.sequence = sequence & log_slot::kSequenceMask;
            } else {
                p.kind = log_slot::PageKind::Foreign;
            }
            return p;
        };
        const bool ring = is_ring_type(type);
        const log_slot::StreamEnd end = log_slot::stream_end(capacity, ring, probe);
        const std::uint32_t next = log_slot::first_free_page(end.page_index, capacity, probe);
        if (probe_failed) return ESP_FAIL;
        Writer w{};
        w.type = type; w.first_block = layout.first_block; w.block_count = layout.block_count;
        w.page_index = next; w.ring = ring; w.wrapped = end.wrapped;
        // The blocks up to the write position were erased when entered; the
        // block after a boundary (or an empty stream) is erased on entry.
        if (next % W25n01kv::kPagesPerBlock == 0)
            w.last_erased_block = next == 0 ? kNoBlock
                                            : layout.first_block + (next - 1) / W25n01kv::kPagesPerBlock;
        else
            w.last_erased_block = layout.first_block + next / W25n01kv::kPagesPerBlock;
        w.launch_protected = ring && slot_info_[current_slot_].flight;
        w.sequence = sequence_base(current_generation_) | end.next_sequence;
        if (!ring && next >= capacity) w.full = true; else any_room = true;
        ESP_LOGI(kLogTag, "resume: stream %u continues at page %lu of %lu%s", type,
                 static_cast<unsigned long>(next), static_cast<unsigned long>(capacity),
                 w.full ? " (full)" : "");
        built[index++] = w;
    }
    if (!any_room) return ESP_ERR_NO_MEM;
    for (int i = 0; i < 5; ++i) *streams[i].writer = built[i];
    packet_buffer_used_ = 0;
    flight_marker_written_ = slot_info_[current_slot_].flight;
    flight_marker_pending_.store(false);
    appended_ = true;
    return ESP_OK;
}

esp_err_t FlashLogger::create_session() {
    constexpr std::uint32_t kNoBlock = 0xFFFFFFFFU;
    appended_ = false;
    const std::uint32_t base = slot_base(current_slot_);
    // The header block cannot be skipped: if it will not erase, this slot is
    // unusable and the next attempt (run init loop) takes the other one.
    if (!prepare_block(base)) {
        slot_unusable_[current_slot_] = true;
        return ESP_FAIL;
    }
    auto writer = [this](std::uint8_t type, bool ring) {
        const auto layout = file_layout(current_slot_, type);
        Writer w{};
        w.type = type; w.first_block = layout.first_block; w.block_count = layout.block_count;
        w.page_index = 0; w.ring = ring;
        // A first block that will not erase is skipped by append_bytes.
        w.last_erased_block = prepare_block(layout.first_block) ? layout.first_block : kNoBlock;
        w.sequence = sequence_base(current_generation_);
        return w;
    };
    imu_writer_ = writer(kImuType, false);
    binary_writer_ = writer(kBinaryType, false);
    text_writer_ = writer(kTextType, is_ring_type(kTextType));
    uart_writer_ = writer(kUart1Type, is_ring_type(kUart1Type));
    packet_writer_ = writer(kPacketType, is_ring_type(kPacketType));
    std::uint8_t header[9]{};
    write_u32_be(header, kSessionMagic); write_u32_be(header + 4, current_generation_);
    header[8] = kStoreVersion;
    const esp_err_t r = program_page_robust(base * W25n01kv::kPagesPerBlock, header, sizeof(header));
    if (r != ESP_OK) {
        slot_unusable_[current_slot_] = true;
        return r;
    }
    packet_buffer_used_ = 0;
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
    const esp_err_t r = program_page_robust(page, marker, sizeof(marker));
    if (r == ESP_OK) {
        flight_marker_written_ = true;
        protect_rings_from_launch();
        ESP_LOGI(kLogTag, "flight marked in slot %c; this session is kept until the next flight",
                 'A' + current_slot_);
    }
    return r;
}

// Background wipe after a full erase (erase_wipe.h): a few blocks of the slot
// that is not the current session per logger pass, ground states only.
void FlashLogger::service_background_wipe()
{
    if (!wipe_.active()) return;
    std::uint32_t first = 0;
    std::uint32_t count = 0;
    if (!wipe_.next(current_slot_, ground_state(), first, count)) {
        if (!wipe_.active()) {
            ESP_LOGI(kLogTag, "background wipe ended: %lu blocks erased in %lu ms",
                     static_cast<unsigned long>(wipe_.erased()),
                     static_cast<unsigned long>(
                         static_cast<std::uint32_t>(esp_timer_get_time() / 1000) - wipe_start_ms_));
        }
        return;
    }
    for (std::uint32_t block = first; block < first + count; ++block) {
        task_supervision::feed();
        bool bad = false;
        if (flash_.is_bad_block(block, bad) == ESP_OK && !bad) (void)flash_.erase_block(block);
    }
}

esp_err_t FlashLogger::clear_session_headers()
{
    // Block 0 of each slot holds its session header and flight marker; with
    // both erased no session is listed and no flight is protected.
    esp_err_t first_error = ESP_OK;
    for (std::uint8_t slot = 0; slot < kSlotCount; ++slot) {
        bool bad = false;
        esp_err_t result = flash_.is_bad_block(slot_base(slot), bad);
        if (result == ESP_OK && !bad) result = flash_.erase_block(slot_base(slot));
        if (result != ESP_OK && first_error == ESP_OK) first_error = result;
    }
    // Keep counting: stale pages carry older tags and never match this one.
    current_slot_ = 0;
    // Both slots are empty now: the slot choice made at start-up (SlotsFull) is stale, and BIT
    // check 6.1 reads it (it kept failing after an erase until the next reboot).
    choice_action_ = log_slot::Action::NewSession;
    current_generation_ = next_usable_generation(current_generation_ + 1);
    session_bad_count_ = 0;
    slot_unusable_[0] = slot_unusable_[1] = false;
    reset_session_buffers();
    const esp_err_t created = create_session();
    session_ready_ = created == ESP_OK;
    return first_error != ESP_OK ? first_error : created;
}

void FlashLogger::reset_session_buffers()
{
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
}

// Appends one chunk as pages. A NAND error never ends logging: transient
// errors are retried; a block that cannot be erased or programmed is skipped
// for this boot (the reader tolerates the gap: it accepts this session's pages
// wherever they are) and the chunk is rewritten in the next block. Only after
// kBackoffAfterBlocks consecutive failed blocks do writes pause and retry every
// few seconds (ESP_ERR_NOT_FINISHED: callers keep their buffers).
esp_err_t FlashLogger::append_bytes(Writer &writer, const std::uint8_t *data, std::size_t length) {
    if (writer.full) return ESP_OK;
    const auto now_ms = [] { return static_cast<std::uint32_t>(esp_timer_get_time() / 1000); };
    if (write_health_.backing_off(now_ms())) return ESP_ERR_NOT_FINISHED;
    const std::uint32_t capacity = writer.block_count * W25n01kv::kPagesPerBlock;
    const auto mark_full = [this, &writer] {
        // Exhausting one allocation must not stop the other streams, e.g.
        // a full sensor file must not terminate an ongoing GPS recording.
        writer.full = true;
        witness_status_.set(IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK, true);
        if (writer.type == kBinaryType) capture_binary_records_.store(false);
        if (writer.type == kImuType) capture_imu_records_.store(false);
        if (writer.type == kTextType) capture_logs_.store(false);
        ESP_LOGE(kLogTag, "session log type %u is full; other streams continue", writer.type);
    };
    const std::uint32_t pages_needed =
        static_cast<std::uint32_t>((length + kPagePayloadSize - 1) / kPagePayloadSize);
    if (!writer.ring && pages_needed > capacity - writer.page_index) {
        mark_full();
        return ESP_OK;
    }
    unsigned skips = 0;
    while (length) {
        task_supervision::feed();
        if (writer.ring && writer.launch_protected &&
            writer.pages_since_launch >= capacity) {
            // Everything since launch fills the ring; keep it all.
            writer.full = true;
            ESP_LOGW(kLogTag, "log type %u: ring full since launch; stopped to keep the flight",
                     writer.type);
            return ESP_OK;
        }
        if (writer.page_index >= capacity) {
            if (!writer.ring) { mark_full(); return ESP_OK; }  // skipped blocks used the room
            // Circular stream: overwrite from the start, oldest block first.
            writer.page_index = 0;
            if (!writer.wrapped) {
                writer.wrapped = true;
                ESP_LOGW(kLogTag, "%s log ring wrapped; its oldest data is now overwritten",
                         writer.type == kTextType ? "text" : "GPS");
            }
        }
        const std::uint32_t block = writer.first_block + writer.page_index / W25n01kv::kPagesPerBlock;
        const std::uint32_t next_block_page =
            (writer.page_index / W25n01kv::kPagesPerBlock + 1U) * W25n01kv::kPagesPerBlock;
        if (block != writer.last_erased_block) {
            if (!prepare_block(block)) {
                write_health_.on_block_failed(now_ms());
                update_nand_status();
                writer.page_index = next_block_page;
                if (++skips > log_slot::kMaxBlockSkipsPerChunk) {
                    ++chunks_dropped_;
                    ESP_LOGE(kLogTag, "log type %u: %u blocks unusable in a row, chunk dropped",
                             writer.type, skips);
                    return ESP_OK;
                }
                if (write_health_.backing_off(now_ms())) return ESP_ERR_NOT_FINISHED;
                continue;
            }
            writer.last_erased_block = block;
        }
        const std::size_t chunk = std::min(length, kPagePayloadSize);
        std::uint8_t page[W25n01kv::kPageDataSize]; std::memset(page, 0xFF, sizeof(page));
        write_u32_be(page, kPageMagic); page[4] = writer.type; page[5] = kStoreVersion;
        write_u16_be(page + 6, static_cast<std::uint16_t>(chunk));
        write_u32_be(page + 8, writer.sequence++);
        std::memcpy(page + kPageHeaderSize, data, chunk);
        const std::uint32_t absolute_page = writer.first_block * W25n01kv::kPagesPerBlock + writer.page_index;
        const esp_err_t r = program_page_robust(absolute_page, page, sizeof(page));
        if (r != ESP_OK) {
            // The page (and so the block) is abandoned for this boot; the chunk
            // is rewritten in the next block with a fresh sequence number.
            mark_session_bad(block);
            write_health_.on_block_failed(now_ms());
            update_nand_status();
            writer.page_index = next_block_page;
            if (++skips > log_slot::kMaxBlockSkipsPerChunk) {
                ++chunks_dropped_;
                ESP_LOGE(kLogTag, "log type %u: program failed in %u blocks, chunk dropped",
                         writer.type, skips);
                return ESP_OK;
            }
            if (write_health_.backing_off(now_ms())) return ESP_ERR_NOT_FINISHED;
            continue;
        }
        write_health_.on_success();
        ++writer.page_index; data += chunk; length -= chunk;
        if (writer.launch_protected) ++writer.pages_since_launch;
    }
    update_nand_status();
    return ESP_OK;
}

// WL record v2 (24 bytes, big-endian), 50 Hz. Only what no other stream
// has: IMU samples are in I*.BIN (960 Hz) and baro altitude, battery, GPS,
// attitude and position in the 50 Hz state packets of P*.BIN.
//   0 magic "WL", 2 version 2, 3-4 Witness status bytes 0 and 1 (byte 1
//   carries the flight state), 5 reserved, 6 u32 sequence, 10 u32 uptime ms,
//   14 int16 LSM6 temperature raw, 16 int32 pressure Pa (centi-mbar),
//   20 int32 MS5607 temperature centi-degC.
void FlashLogger::serialize_record(std::uint8_t *d) {
#if CONFIG_WITNESS_IMAGE_MAINTENANCE
    const bool available = false;   // read-only export image: no sensors
#else
    Sensors::Sample sample{};
    const bool available = sensors_ != nullptr && sensors_->latest_sample(sample, 0) == ESP_OK;
#endif
    std::memset(d, 0, kRecordSize); write_u16_be(d, kRecordMagic);
    d[2] = kRecordVersion;
    witness_status_.write(d + 3);
    write_u32_be(d + 6, sequence_++);
    write_u32_be(d + 10, static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL));
    if (!available) return;
#if !CONFIG_WITNESS_IMAGE_MAINTENANCE
    write_u16_be(d + 14, static_cast<std::uint16_t>(sample.temperature_raw));
    write_u32_be(d + 16, static_cast<std::uint32_t>(sample.pressure_centi_mbar));
    write_u32_be(d + 20, static_cast<std::uint32_t>(sample.ms5607_temperature_centi_celsius));
#endif
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
                                 const std::uint8_t version,
                                 const bool ring, const std::uint32_t generation,
                                 bool &tagged, std::uint32_t &size,
                                 std::uint32_t &start_page)
{
    const auto layout = file_layout(slot, type, version);
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
            if ((index & 255U) == 0) task_supervision::feed();
            // An unreadable page (uncorrectable ECC, e.g. torn by a power loss)
            // is counted and skipped, never fatal to the mount.
            const esp_err_t r = read_page_retry(layout.first_block * W25n01kv::kPagesPerBlock + index, 0, h, sizeof(h));
            if (r != ESP_OK) { ++unreadable_pages_; continue; }
            const std::uint16_t n = read_u16_be(h + 6);
            const std::uint32_t sequence = read_u32_be(h + 8);
            const bool valid = read_u32_be(h) == kPageMagic && h[4] == type &&
                               h[5] == version && n <= kPagePayloadSize &&
                               (sequence >> kSequenceTagShift) == (tagged ? tag : 0U);
            if (!valid) {
                // Untagged (pre-tag) linear files end at the first page not
                // theirs. A tagged file may have gaps (blocks the writer skipped
                // after a NAND error) and is read to the end of its allocation:
                // only this session's pages carry its tag.
                if (!ring && !tagged) break;
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
    const auto layout = file_layout(file.slot, file.type, file.store_version);
    const std::uint32_t capacity = layout.block_count * W25n01kv::kPagesPerBlock;
    const bool ring = is_ring_type(file.type);
    // A ring file is read oldest page first, wrapping, skipping the erased
    // pages around the write position.
    for (; step < capacity; ++step) {
        const std::uint32_t index = ring ? (file.start_page + step) % capacity : step;
        const esp_err_t result = read_page_retry(
            layout.first_block * W25n01kv::kPagesPerBlock + index, 0,
            export_read_page_, sizeof(export_read_page_));
        if (result != ESP_OK) { ++unreadable_pages_; continue; }  // same rule as scan_file
        const std::uint16_t payload_length = read_u16_be(export_read_page_ + 6);
        const bool valid = read_u32_be(export_read_page_) == kPageMagic &&
                           export_read_page_[4] == file.type &&
                           export_read_page_[5] == file.store_version &&
                           payload_length <= kPagePayloadSize &&
                           (read_u32_be(export_read_page_ + 8) >> kSequenceTagShift) ==
                               (file.tagged ? sequence_tag(file.generation) : 0U);
        if (!valid) {
            if (!ring && !file.tagged) return ESP_ERR_INVALID_RESPONSE;
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

esp_err_t FlashLogger::flush_for_restart(const TickType_t timeout)
{
    if (!task_handle_ || frozen_.load() || !session_ready_ || slots_full_) return ESP_ERR_INVALID_STATE;
    const std::uint32_t target = flush_requested_.fetch_add(1) + 1U;
    const TickType_t start = xTaskGetTickCount();
    while (!radio_edge::sequence_reached(flush_done_.load(), target)) {
        if (xTaskGetTickCount() - start >= timeout) return ESP_ERR_TIMEOUT;
        vTaskDelay(1);
    }
    return ESP_OK;
}

// esp_restart() runs this before it resets (software restarts: the bench
// restart, the Landed -> Pad Idle reset, the maintenance switch).
void FlashLogger::restart_flush_handler()
{
    FlashLogger *self = log_sink_;
    if (self != nullptr) (void)self->flush_for_restart(pdMS_TO_TICKS(150));
}

// Posts an erase (1) or fast clear (2) and waits for the logger task, at most
// kEraseWaitMs when the caller passes portMAX_DELAY (full erase worst case is
// ~12 s). A request the task has not started is cancelled on timeout.
esp_err_t FlashLogger::request_erase(const std::uint8_t mode, const TickType_t timeout)
{
    if (frozen_.load() || !task_handle_) return ESP_ERR_INVALID_STATE;
    const TickType_t wait = timeout == portMAX_DELAY ? pdMS_TO_TICKS(kEraseWaitMs) : timeout;
    // Drop a completion left by an earlier call that timed out.
    if (erase_request_.load() == 0 && !erase_busy_.load()) (void)xSemaphoreTake(erase_semaphore_, 0);
    std::uint8_t expected = 0;
    if (!erase_request_.compare_exchange_strong(expected, mode)) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(erase_semaphore_, wait) != pdTRUE) {
        std::uint8_t pending = mode;
        (void)erase_request_.compare_exchange_strong(pending, 0);
        return ESP_ERR_TIMEOUT;
    }
    return erase_result_;
}

esp_err_t FlashLogger::erase_all_sessions(const TickType_t timeout)
{
    return request_erase(1, timeout);
}

esp_err_t FlashLogger::clear_all_sessions(const TickType_t timeout)
{
    return request_erase(2, timeout);
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

    landed_power::register_current_task();
    for (;;) {
        if (landed_power::low_power()) {
            // Landed low-power mode: nothing is captured in Landed (see
            // FLIGHT_LOG_BIN_LANDED); wait for the flag to clear.
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(landed::kLowPowerTaskPeriodMs));
            next = xTaskGetTickCount();
            continue;
        }
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
        const FlightStateMachine::State state = flight_state_machine_->state();

        if (state == FlightStateMachine::State::PadIdle) {
            prelaunch_records_[prelaunch_write_index_] = record;
            prelaunch_write_index_ =
                (prelaunch_write_index_ + 1U) % kPrelaunchRecordCount;
            prelaunch_record_count_ = std::min(
                prelaunch_record_count_ + 1U, kPrelaunchRecordCount);
            prelaunch_dump_pending_ = true;
            continue;
        }

        if (!flight_state_machine_->binary_logging_enabled()) {
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
    esp_err_t r = ESP_FAIL;
    bool chip_ready = false;
    bool self_test_done = false;
    unsigned init_failures = 0;
    const auto now_ms = [] { return static_cast<std::uint32_t>(esp_timer_get_time() / 1000); };

    // Startup never gives up: a NAND error (even a torn header page) is
    // retried every 2 s, with a chip reset every third time, and logging starts
    // as soon as the chip answers. Telemetry does not depend on this task.
    for (;;) {
        r = ESP_OK;
        if (!chip_ready) {
            r = flash_.initialize();
            chip_ready = r == ESP_OK;
        }
        // After a warm reset in flight the NAND start-up test is skipped (owner decision, BIT plan Q13):
        // it erases and programs a block before logging resumes, and in flight every data point counts.
        if (r == ESP_OK && !self_test_done && resumed_in_flight_) {
            self_test_done = true;
            ESP_LOGW(kLogTag, "resumed in flight: NAND start-up test skipped");
        }
        if (r == ESP_OK && !self_test_done) {
            r = run_startup_self_test();
            if (r != ESP_OK && init_failures >= 2) {
                // The chip answers but the reserved test block does not
                // verify: log anyway, the writer reports real failures.
                ESP_LOGW(kLogTag, "startup self test failed (%s): continuing", esp_err_to_name(r));
                r = ESP_OK;
            }
            self_test_done = r == ESP_OK;
        }
        if (r == ESP_OK) r = scan_sessions(false);
        if (r == ESP_OK && choice_action_ == log_slot::Action::Append) {
            if (resume_session() != ESP_OK) {
                ESP_LOGW(kLogTag, "cannot append to the flight slot: normal slot rule");
                choose_slot(false);
            }
        }
        if (r == ESP_OK && choice_action_ != log_slot::Action::Append && !slots_full_) {
            r = create_session();
            if (r != ESP_OK) choose_slot(false);  // slot_unusable_ steers the next try
        }
        if (r == ESP_OK) break;
        ++init_failures;
        set_uart_capture_enabled(false);
        capture_binary_records_.store(false);
        capture_imu_records_.store(false);
        witness_status_.set(IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK, true);
        ESP_LOGE(kLogTag, "flash initialization failed: %s (retrying)", esp_err_to_name(r));
        vTaskDelay(pdMS_TO_TICKS(2000));
        if (chip_ready && (init_failures % 3) == 0) (void)flash_.recover();
    }
    // A session exists (created or appended) unless both slots hold flights.
    // BIT checks 6.1/6.2/5.2/6.3 and the commanded NAND slice read this flag.
    session_ready_ = !slots_full_;
    set_uart_capture_enabled(true);
    capture_binary_records_.store(true);
    if (slots_full_) {
        // Both slots hold flights: never erase either. Nothing is written to
        // NAND (all capture stays off) until an erase/clear command (USB or
        // radio, Pad Idle/Landed) frees the store.
        set_uart_capture_enabled(false);
        capture_binary_records_.store(false);
        capture_imu_records_.store(false);
        capture_packets_.store(false);
        capture_logs_.store(false);
        witness_status_.set_state_flag(IRIS_WITNESS_STATUS_LOG_SLOTS_FULL_MASK, true);
        witness_status_.set(IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK, false);
        ESP_LOGW(kLogTag, "both log slots hold a flight: NOT LOGGING (nothing is erased). "
                          "Download the flights, then erase/clear (0x6869/0x686A) to log again");
        landed_power::register_current_task();
        // Long idle wait (not subscribed to the task watchdog here).
        while (erase_request_.load() == 0) {
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(landed_power::low_power()
                                                             ? landed::kLowPowerTaskPeriodMs : 200));
        }
        // The normal loop below runs the erase; the writers exist after it.
    }
    capture_imu_records_.store(true);
    capture_packets_.store(true);
    witness_status_.set(IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK, false);
    witness_status_.set(IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK, false);
    if (appended_) {
        // Streams with no room left keep their capture off (as at runtime).
        if (imu_writer_.full) capture_imu_records_.store(false);
        if (binary_writer_.full) capture_binary_records_.store(false);
        if (text_writer_.full) capture_logs_.store(false);
        if (any_stream_full()) witness_status_.set(IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK, true);
        // Marker record in the packet stream: kind 1 = resumed after an
        // in-flight reset, flight state, session generation (u32 BE).
        std::uint8_t marker[6]{};
        marker[0] = 1;
        marker[1] = flight_state_machine_ != nullptr
                        ? static_cast<std::uint8_t>(flight_state_machine_->state()) : 0;
        write_u32_be(marker + 2, current_generation_);
        (void)log_packet(kPacketSourceLogMarker, marker, sizeof(marker));
        ESP_LOGW(kLogTag, "RESUMED after an in-flight reset: appending to flight session %lu in slot %c",
                 static_cast<unsigned long>(current_generation_), 'A' + current_slot_);
    } else if (!slots_full_) {
        ESP_LOGI(kLogTag,
             "session %lu in slot %c (store v%u); blocks: IMU %lu, sensors %lu, text %lu, "
             "packets %lu, GPS ring %lu; flush interval %lu ms",
             static_cast<unsigned long>(current_generation_), 'A' + current_slot_,
             kStoreVersion, static_cast<unsigned long>(kImuBlocks),
             static_cast<unsigned long>(kBinaryBlocks), static_cast<unsigned long>(kTextBlocks),
             static_cast<unsigned long>(kPacketBlocks),
             static_cast<unsigned long>(kUartBlocks), static_cast<unsigned long>(kFlushIntervalMs));
    }
    TickType_t next = xTaskGetTickCount();
    TickType_t last_flush = next;
    landed_power::register_current_task();
    std::uint32_t reported_drops = 0;
    std::uint32_t reported_packet_drops = 0;
    // Task watchdog: subscribed only while the loop runs (init and the slots-full
    // wait above can take longer than the timeout); fed every pass and inside
    // the long erase/scan/append loops; unsubscribed before the frozen wait.
    task_supervision::subscribe();
    for (;;) {
        task_supervision::feed();
        r = ESP_OK;
        const std::uint8_t erase_req = erase_request_.exchange(0);
        if (erase_req != 0) erase_busy_.store(true);
        if (erase_req != 0 && !ground_state()) {
            // Accepted on the ground, but the rocket has launched since: the
            // flight data must not be erased.
            erase_result_ = ESP_ERR_INVALID_STATE;
            xSemaphoreGive(erase_semaphore_);
            erase_busy_.store(false);
            ESP_LOGW(kLogTag, "erase/clear refused: flight state is no longer Pad Idle/Landed");
        } else if (erase_req != 0) {
            set_uart_capture_enabled(false);
            capture_binary_records_.store(false);
            capture_imu_records_.store(false);
            // Let an in-progress capture finish, then discard both queued and
            // pre-trigger records as part of the requested full erase.
            vTaskDelay(kLogPeriod);
            xQueueReset(binary_record_queue_);
            capture_logs_.store(false);
            const bool quick = erase_req == 2;
            const std::uint32_t erase_start_ms = now_ms();
            // Both erase modes clear the slot headers at once (about 40 ms) and
            // start a new session; the full erase then wipes the other slot in
            // the background (service_background_wipe), so the logger, the
            // packet queue and the USB command task are never held for the 4.3 s
            // the old all-blocks erase needed.
            wipe_.cancel();
            erase_result_ = clear_session_headers();
            if (!quick && session_ready_) {
                wipe_.start(kBlocksPerSlot, kSlotCount);
                wipe_start_ms_ = now_ms();
            }
            const std::uint32_t erase_ms = now_ms() - erase_start_ms;
            capture_logs_.store(session_ready_);
            capture_binary_records_.store(session_ready_);
            capture_imu_records_.store(session_ready_);
            set_uart_capture_enabled(session_ready_);
            xSemaphoreGive(erase_semaphore_);
            erase_busy_.store(false);
            if (erase_result_ != ESP_OK) {
                ESP_LOGE(kLogTag, "erase/clear finished with error: %s", esp_err_to_name(erase_result_));
            }
            if (session_ready_) {
                if (slots_full_) {
                    slots_full_ = false;
                    capture_packets_.store(true);
                    witness_status_.set_state_flag(IRIS_WITNESS_STATUS_LOG_SLOTS_FULL_MASK, false);
                    ESP_LOGI(kLogTag, "log store freed: logging resumed");
                }
                if (quick) {
                    ESP_LOGI(kLogTag, "all stored sessions cleared (headers only, %lu ms); new session %lu",
                             static_cast<unsigned long>(erase_ms),
                             static_cast<unsigned long>(current_generation_));
                } else {
                    ESP_LOGI(kLogTag, "all stored sessions erased (headers %lu ms, the other slot is wiped in "
                                      "the background); new session %lu created",
                             static_cast<unsigned long>(erase_ms),
                             static_cast<unsigned long>(current_generation_));
                }
                last_flush = xTaskGetTickCount();
            }
        }
        if (export_requested_.load()) {
            set_uart_capture_enabled(false);
            capture_binary_records_.store(false);
            capture_imu_records_.store(false);
            capture_packets_.store(false);
            // Drain once to release any producer waiting for queue space, wait
            // for the capture task to observe the stop flag, then drain again
            // so the exported file includes every record captured beforehand.
            // Every step runs even if an earlier one failed: the export must
            // freeze with as much data on NAND as possible.
            esp_err_t fr = ESP_OK;
            const auto keep = [&fr](const esp_err_t e) {
                if (fr == ESP_OK && e != ESP_OK) fr = e;
            };
            keep(drain_binary_records());
            keep(drain_imu_records());
            vTaskDelay(kLogPeriod);
            keep(drain_binary_records());
            keep(drain_imu_records());
            capture_logs_.store(false);
            if (flight_marker_pending_.load() && !flight_marker_written_)
                keep(write_flight_marker());
            keep(flush_binary());
            keep(flush_imu());
            keep(drain_packet_records());
            keep(flush_packets());
            keep(flush_text());
            keep(drain_uart_records(true));
            if (fr != ESP_OK)
                ESP_LOGE(kLogTag, "export: some buffered data could not be written: %s",
                         esp_err_to_name(fr));
            r = scan_sessions(true);
            if (r == ESP_OK) {
                frozen_.store(true); xSemaphoreGive(frozen_semaphore_);
                task_supervision::unsubscribe();
                for (;;) vTaskDelay(portMAX_DELAY);
            }
        }
        if (r == ESP_OK && write_health_.backing_off(now_ms())) {
            // Too many consecutive failed blocks: pause the NAND writes and
            // retry every kBackoffMs forever (queues overflow and count drops).
            backoff_active_ = true;
            vTaskDelay(kLogPeriod);
            continue;
        }
        if (backoff_active_) {
            backoff_active_ = false;
            ESP_LOGW(kLogTag, "retrying NAND writes after back-off");
            (void)flash_.recover();
        }
        if (r == ESP_OK) r = drain_binary_records();
        if (r == ESP_OK) r = drain_imu_records();
        if (r == ESP_OK && flight_marker_pending_.load() && !flight_marker_written_)
            r = write_flight_marker();
        if (r == ESP_OK) r = drain_packet_records();
        if (r == ESP_OK) r = drain_uart_records(false);
        if (r == ESP_OK) service_background_wipe();
        const TickType_t now = xTaskGetTickCount();
        // A restart handler asked for everything to be written now.
        const std::uint32_t flush_target = flush_requested_.load();
        const bool restart_flush = !radio_edge::sequence_reached(flush_done_.load(), flush_target);
        if (r == ESP_OK && (restart_flush || now - last_flush >= pdMS_TO_TICKS(kFlushIntervalMs))) {
            r = flush_binary();
            if (r == ESP_OK) r = flush_imu();
            if (r == ESP_OK) r = flush_packets();
            if (r == ESP_OK) r = flush_text();
            if (r == ESP_OK) r = drain_uart_records(true);
            last_flush = now;
            if (restart_flush && r == ESP_OK) flush_done_.store(flush_target);
            const std::uint32_t packet_drops = packets_dropped_.load();
            if (packet_drops != reported_packet_drops) {
                ESP_LOGW(kLogTag, "packet log queue full: %lu packets dropped so far",
                         static_cast<unsigned long>(packet_drops));
                reported_packet_drops = packet_drops;
            }
            const std::uint32_t drops = imu_records_dropped_.load();
            if (drops != reported_drops) {
                ESP_LOGW(kLogTag, "IMU log queue full: %lu records dropped so far",
                         static_cast<unsigned long>(drops));
                reported_drops = drops;
            }
        }
#if !CONFIG_WITNESS_IMAGE_MAINTENANCE
        // BIT active-test slice (commanded runs only; a no-op otherwise).
        if (r == ESP_OK) bit_slice(last_flush == now);
#endif
        if (r != ESP_OK) {
            // Never end the task on an error (previously: capture off, task
            // deleted, logging lost until power cycle). Buffers keep their data
            // and are retried on the next pass; ESP_ERR_NOT_FINISHED is the
            // back-off pause and is not a new failure.
            if (r != ESP_ERR_NOT_FINISHED) {
                write_health_.on_block_failed(now_ms());
                update_nand_status();
                if ((write_health_.total_failed_blocks() & 15U) == 1U)
                    ESP_LOGE(kLogTag, "flash write error: %s (retrying)", esp_err_to_name(r));
            }
            r = ESP_OK;
        }
        if (landed_power::low_power()) {
            // Landed low-power mode: the queues stay empty, so wake only for the
            // housekeeping period (text and GPS window bytes flush on it).
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(landed::kLowPowerTaskPeriodMs));
            next = xTaskGetTickCount();
            continue;
        }
        vTaskDelayUntil(&next, kLogPeriod);
    }
}
