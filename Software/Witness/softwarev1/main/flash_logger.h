#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdarg>

#include "comms.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "w25n01kv.h"
#include "log_slot_logic.h"
#include "erase_wipe.h"

class Sensors;
class WitnessStatus;
class FlightStateMachine;

// Append-only log store for the raw SPI NAND (store version 4).
//
// The usable NAND (1,004 blocks) is two session slots, A and B, of 502 blocks.
// Each slot holds one boot's session:
//
//   block 0        session header (page 0) and flight marker (page 1)
//   blocks 1-96    I*.BIN  IMU stream: 24-byte records, 960 Hz until Descent,
//                  then every 10th record (96 Hz); prelaunch 2 s included
//   blocks 97-136  S*.BIN  sensor stream: 24-byte WL v2 records at 50 Hz
//                  (status, uptime, pressure and temperatures only; IMU
//                  samples are in I*.BIN, altitude/battery in P*.BIN)
//   blocks 137-160 S*.TXT  diagnostic text (circular)
//   blocks 161-280 P*.BIN  packet stream (circular): Witness state packets at
//                  50 Hz and every packet sent or received over CAN, plus
//                  radio commands; records [u32 us][u8 source][u8 length]
//                  [complete frame], big-endian
//   blocks 281-501 U*.BIN  raw GPS UART bytes (circular)
//
// Circular streams wrap freely on the pad. From launch (the flight marker)
// they stop overwriting: everything written since launch is kept, and the
// stream stops when it would overwrite the launch point.
// Store version 3 sessions (no packet stream, 341 GPS blocks) still export.
//
// The flight marker is written once, when sensor logging first starts
// (Boost). A new session always takes the slot that does NOT hold the newest
// flight, so bench power-ups keep reusing one slot and the last flight is kept
// until another flight replaces it. A read-only FAT volume is generated from
// the export table by UsbMassStorage; a desktop OS never writes the NAND.
class FlashLogger final {
public:
    static constexpr UBaseType_t kTaskPriority = 3;
    static constexpr UBaseType_t kCaptureTaskPriority = 4;
    // 50 Hz: the MS5607 converts pressure at about 50 Hz, so faster records
    // only repeat it.
    static constexpr std::uint32_t kLogRateHz = 50;
    static constexpr std::uint32_t kPrelaunchBufferSeconds = 2;
    static constexpr std::uint32_t kFlushIntervalMs = 1000;
    static constexpr std::uint32_t kTaskStackSize = 7168;
    // 3072 was tight: the Boost-transition ESP_LOGI (formatting + the 256 B
    // LogLine in log_vprintf + console printf) needs ~2 KB. +2 KB of heap.
    static constexpr std::uint32_t kCaptureTaskStackSize = 5120;
    // Erase/clear callers wait at most this long (full erase worst case is
    // 1004 blocks x ~12 ms = ~12 s) instead of forever.
    static constexpr std::uint32_t kEraseWaitMs = 30000;
    static constexpr std::size_t kMaximumExportFiles = 48;
    static constexpr std::size_t kUartReceiveBufferSize = 32768;

    // Packet stream sources (record byte 4).
    static constexpr std::uint8_t kPacketSourceState = 1;   // Witness 0x02, 50 Hz
    static constexpr std::uint8_t kPacketSourceCanRx = 2;   // from Iris over CAN
    static constexpr std::uint8_t kPacketSourceCanTx = 3;   // to Iris over CAN
    static constexpr std::uint8_t kPacketSourceRadioRx = 4; // LoRa command
    static constexpr std::uint8_t kPacketSourceKalman = 5;  // 0xE2 filter log, 25 Hz
    // Logger marker, written once when a reset in flight resumed in this slot:
    // frame = [u8 kind=1 (resumed)] [u8 flight state] [u32 BE session generation].
    // Timestamps (u32 us), IMU sequence and WL sequence restart after it.
    static constexpr std::uint8_t kPacketSourceLogMarker = 6;
    static constexpr std::uint8_t kPacketSourceBit = 7;     // BIT 0x0B / 0x10 frames and the 6.3 marker
    static constexpr std::size_t kPacketRecordHeader = 6;   // u32 us, source, length
    static constexpr std::size_t kMaxPacketFrame = 103;     // intercomm limit
    // Any task may call this; never blocks (a full queue drops and counts).
    esp_err_t log_packet(std::uint8_t source, const std::uint8_t *frame,
                         std::size_t length);

    // IMU stream: one record per fresh IMU sample from the sensor loop.
    static constexpr std::uint32_t kImuRateHz = 960;
    static constexpr std::uint32_t kImuDescentDecimation = 10; // 960 -> 96 Hz
    static constexpr std::size_t kImuRecordSize = 24;
    // IMU record status byte (offset 22).
    static constexpr std::uint8_t kImuStateMask = 0x07;      // flight state
    static constexpr std::uint8_t kImuHitlFlag = 0x08;
    static constexpr std::uint8_t kImuLowGFreshFlag = 0x10;
    static constexpr std::uint8_t kImuHighGFreshFlag = 0x20;
    static constexpr std::uint8_t kImuGyroFreshFlag = 0x40;
    // Filtered acceleration was taken from the high-g sensor (low-g near
    // saturation) for this sample.
    static constexpr std::uint8_t kImuHighGActiveFlag = 0x80;

    struct ImuRecord {
        std::uint8_t data[kImuRecordSize];
    };

    // Big-endian layout: u32 timestamp_us (esp_timer, low 32 bits), int16
    // low-g xyz, int16 high-g xyz, int16 gyro xyz (native LSM6 counts), u8
    // status (kImu* flags), u8 sequence (assigned by log_imu_record).
    static ImuRecord make_imu_record(std::uint32_t timestamp_us,
                                     const std::int16_t low_g[3],
                                     const std::int16_t high_g[3],
                                     const std::int16_t gyro[3],
                                     std::uint8_t status);

    struct ExportFile {
        char name[13];                 // DOS 8.3 name, including terminator
        std::uint32_t size;
        std::uint32_t generation;
        std::uint8_t slot;
        std::uint8_t type;
        std::uint8_t store_version;
        std::uint32_t start_page;      // first page in reading order (ring files)
        // Pages carry the session tag (see kSequenceTagShift); false only for
        // ring files written before tagging, which are read untagged.
        bool tagged;
    };

    FlashLogger(Sensors &sensors,
                WitnessStatus &witness_status,
                FlightStateMachine &flight_state_machine);
    // Maintenance image: read-only export (start_export_only) with no sensor or
    // flight-state source; the write path must not be started on this object.
    explicit FlashLogger(WitnessStatus &witness_status);
    FlashLogger(const FlashLogger &) = delete;
    FlashLogger &operator=(const FlashLogger &) = delete;

    esp_err_t start();
    // Maintenance boot: mount the NAND read-only for export (log download)
    // without creating the write path (queues, capture task, console hook),
    // which is what frees the RAM Wi-Fi needs. Logging calls are no-ops and
    // frozen() turns true once the export table is built.
    esp_err_t start_export_only();
    // Exact external GPS bytes, recorded continuously regardless of flight state.
    // No framing or timestamps are added.
    esp_err_t log_external_bytes(const std::uint8_t *data, std::size_t length);
    esp_err_t log_uart_bytes(const std::uint8_t *data, std::size_t length) {
        return log_external_bytes(data, length);
    }
    // Called from the sensor task once per fresh IMU sample. Never blocks;
    // a full queue drops the record and counts it.
    esp_err_t log_imu_record(ImuRecord record);

    // Flushes the current session and permanently freezes logging until reset.
    // This must complete before exposing the read-only USB disk.
    esp_err_t prepare_mass_storage(TickType_t timeout = portMAX_DELAY);
    // Destructively erases both session slots, then creates a fresh empty
    // session. NAND access remains owned by the logger task throughout.
    esp_err_t erase_all_sessions(TickType_t timeout = portMAX_DELAY);
    // Fast clear: deletes every stored session by erasing only the two slot
    // headers, then creates a fresh session whose number continues upward
    // (never reset), so stale data pages carry other session tags and are
    // never exported. Data blocks are erased lazily before they are rewritten.
    esp_err_t clear_all_sessions(TickType_t timeout = portMAX_DELAY);
    // Flushes everything buffered (sensor, IMU, packet, text and GPS records) to
    // NAND and returns once the logger task has done it, at most `timeout`.
    // Used by the restart handler so a software restart loses well under the 1 s
    // flush interval of log. Panics and watchdog resets cannot run it. Safe to
    // call from any task; ESP_ERR_INVALID_STATE when nothing is being logged.
    esp_err_t flush_for_restart(TickType_t timeout);
    std::size_t export_file_count() const;
    const ExportFile &export_file(std::size_t index) const;
    esp_err_t read_export_file(
        std::size_t index,
        std::uint32_t offset,
        std::uint8_t *destination,
        std::size_t length);
    void disable_console_output();
    // True once prepare_mass_storage() has frozen logging (until reset).
    bool frozen() const { return frozen_.load(); }
    // Pages skipped as unreadable (uncorrectable ECC) by scans and exports.
    std::uint32_t unreadable_page_count() const { return unreadable_pages_; }

private:
    friend class BitProbe;   // read-only BIT probes and the BIT request/result words (bit_monitor.cpp)
    static constexpr TickType_t kLogPeriod =
        pdMS_TO_TICKS(1000U / kLogRateHz);
    static constexpr std::size_t kRecordSize = 24;
    static constexpr std::size_t kPrelaunchRecordCount =
        kLogRateHz * kPrelaunchBufferSeconds;
    static constexpr std::size_t kImuPrelaunchRecordCount =
        kImuRateHz * kPrelaunchBufferSeconds;
    // Holds the complete prelaunch dump plus more than three seconds of new
    // live records while the NAND writer catches up.
    static constexpr std::size_t kRecordQueueDepth = 256;
    // About two seconds of 960 Hz IMU records between writer passes.
    static constexpr std::size_t kImuQueueDepth = 2048;
    // magic(4) type(1) version(1) length(2) page sequence(4)
    static constexpr std::size_t kPageHeaderSize = 12;
    static constexpr std::size_t kPagePayloadSize =
        W25n01kv::kPageDataSize - kPageHeaderSize;
    static constexpr std::uint32_t kReservedBlocks = 20;
    static constexpr std::uint32_t kSlotCount = 2;
    static constexpr std::uint32_t kBlocksPerSlot = 502;
    static constexpr std::uint32_t kHeaderBlocks = 1;
    static constexpr std::uint32_t kImuBlocks = 96;
    static constexpr std::uint32_t kBinaryBlocks = 40;
    static constexpr std::uint32_t kTextBlocks = 24;
    // 120 blocks = 15.6 MB, about 70 min at ~3.6 KB/s (state at 50 Hz plus CAN).
    static constexpr std::uint32_t kPacketBlocks = 120;
    static constexpr std::uint32_t kUartBlocks =
        kBlocksPerSlot - kHeaderBlocks - kImuBlocks - kBinaryBlocks - kTextBlocks -
        kPacketBlocks;
    // Store version 3 layout (read-only, for exporting older sessions).
    static constexpr std::uint8_t kLegacyStoreVersion = 3;
    static constexpr std::uint32_t kV3UartBlocks =
        kBlocksPerSlot - kHeaderBlocks - kImuBlocks - kBinaryBlocks - kTextBlocks;
    static constexpr std::uint32_t kFlightMarkerPage = 1;  // within block 0
    static constexpr std::uint32_t kSessionMagic = 0x574C5331U; // WLS1
    static constexpr std::uint32_t kFlightMagic = 0x574C4654U;  // WLFT
    static constexpr std::uint32_t kPageMagic = 0x574C5047U;    // WLPG
    static constexpr std::uint8_t kStoreVersion = 4;
    static_assert(kStoreVersion == IRIS_MAINT_STORE_FORMAT_VERSION,
                  "comms.h IRIS_MAINT_STORE_FORMAT_VERSION must follow kStoreVersion "
                  "(the image-info record advertises it to the other image)");
    static constexpr std::uint8_t kBinaryType = 1;
    static constexpr std::uint8_t kTextType = 2;
    static constexpr std::uint8_t kUart1Type = 3;
    // Circular streams overwrite their oldest block instead of filling up:
    // raw GPS and the diagnostic text (it is written from power-on in every
    // state, so a long pad wait must not leave no room for the flight).
    static constexpr bool is_ring_type(std::uint8_t type)
    {
        return type == kUart1Type || type == kTextType || type == kPacketType;
    }
    // A new session erases only the first block of each stream, so the rest
    // of a slot still holds valid pages from older sessions. The page
    // sequence number therefore carries the session generation in its top
    // 12 bits (20 bits count pages: 2 GB per stream), and the scan and
    // export ignore pages from any other session.
    static constexpr unsigned kSequenceTagShift = 20;
    static constexpr std::uint32_t sequence_tag(std::uint32_t generation)
    {
        return generation & 0xFFFU;
    }
    // Pages written before tagging carry tag 0, so no session uses it.
    static constexpr std::uint32_t next_usable_generation(std::uint32_t generation)
    {
        return sequence_tag(generation) == 0 ? generation + 1 : generation;
    }
    static constexpr std::uint32_t sequence_base(std::uint32_t generation)
    {
        return sequence_tag(generation) << kSequenceTagShift;
    }
    static constexpr std::uint8_t kImuType = 4;
    static constexpr std::uint8_t kPacketType = 5;
    static constexpr std::size_t kPacketQueueDepth = 128;
    static constexpr std::uint16_t kRecordMagic = 0x574C;       // WL
    static constexpr std::uint8_t kRecordVersion = 2;

    struct LogLine {
        std::uint16_t length;
        char data[254];
    };

    struct BinaryRecord {
        std::uint8_t data[kRecordSize];
    };

    struct PacketRecord {
        std::uint8_t length;  // header + frame bytes
        std::uint8_t data[kPacketRecordHeader + kMaxPacketFrame];
    };

    struct Writer {
        std::uint8_t type;
        std::uint32_t first_block;
        std::uint32_t block_count;
        std::uint32_t page_index;
        std::uint32_t last_erased_block;
        bool ring = false;       // wrap and overwrite instead of stopping
        bool full = false;
        bool wrapped = false;
        // Ring streams after launch: pages written since the launch point;
        // the stream stops once it would overwrite that point.
        bool launch_protected = false;
        std::uint32_t pages_since_launch = 0;
        std::uint32_t sequence = 0;
    };
    struct FileLayout {
        std::uint32_t first_block;
        std::uint32_t block_count;
    };
    struct SlotInfo {
        bool valid = false;
        bool flight = false;
        bool dirty = false;    // header/marker page was uncorrectable
        std::uint8_t version = 0;
        std::uint32_t generation = 0;
    };
    static FileLayout file_layout(std::uint8_t slot, std::uint8_t type,
                                  std::uint8_t version = kStoreVersion);
    static std::uint32_t slot_base(std::uint8_t slot) { return slot * kBlocksPerSlot; }

    static_assert(kLogRateHz > 0 && (1000U % kLogRateHz) == 0);
    static_assert(kPrelaunchRecordCount > 0);
    static_assert(kRecordQueueDepth > kPrelaunchRecordCount);
    static_assert(kFlushIntervalMs == 500 || kFlushIntervalMs == 1000,
                  "Use a 0.5 s or 1 s NAND flush interval");
    static_assert(kSlotCount * kBlocksPerSlot <=
                  W25n01kv::kBlockCount - kReservedBlocks);
    static_assert(kUartBlocks > 0 && kUartBlocks < kBlocksPerSlot);
    static_assert(kImuRateHz % kImuDescentDecimation == 0);

    static void task_entry(void *context);
    static void export_task_entry(void *context);
    void run_export_only();
    static void capture_task_entry(void *context);
    static int log_vprintf(const char *format, va_list arguments);
    static FlashLogger *log_sink_;

    esp_err_t read_slot_info(std::uint8_t slot, SlotInfo &info);
    // Page read with a few retries for transient errors (an uncorrectable ECC
    // result is deterministic and returned at once).
    esp_err_t read_page_retry(std::uint32_t page, std::uint16_t column,
                              std::uint8_t *data, std::size_t length);
    void choose_slot(bool allow_append);
    esp_err_t resume_session();
    bool block_session_bad(std::uint32_t block) const;
    void mark_session_bad(std::uint32_t block);
    bool prepare_block(std::uint32_t block);
    esp_err_t program_page_robust(std::uint32_t page, const std::uint8_t *data, std::size_t length);
    void update_nand_status();
    bool any_stream_full() const;
    bool ground_state() const;
    esp_err_t request_erase(std::uint8_t mode, TickType_t timeout);
    esp_err_t scan_sessions(bool build_export_table);
    esp_err_t run_startup_self_test();
    // Full erase (0x6869): header clear now, wipe of the other slot afterwards.
    void service_background_wipe();
    static void restart_flush_handler();
    esp_err_t clear_session_headers();
    void reset_session_buffers();
    esp_err_t create_session();
    esp_err_t write_flight_marker();
    esp_err_t append_bytes(Writer &writer,
                           const std::uint8_t *data,
                           std::size_t length);
    esp_err_t flush_binary();
    esp_err_t flush_imu();
    esp_err_t flush_text();
    esp_err_t drain_uart_records(bool flush);
    void set_uart_capture_enabled(bool enabled);
    esp_err_t append_record(const BinaryRecord &record);
    esp_err_t append_imu_bytes(const std::uint8_t *data);
    esp_err_t drain_binary_records();
    esp_err_t drain_imu_records();
    esp_err_t drain_packet_records();
    esp_err_t flush_packets();
    void protect_rings_from_launch();
    esp_err_t scan_file(std::uint8_t slot, std::uint8_t type, std::uint8_t version,
                        bool ring,
                        std::uint32_t generation, bool &tagged,
                        std::uint32_t &size, std::uint32_t &start_page);
    esp_err_t read_file_data(std::size_t file_index,
                             std::uint32_t offset,
                             std::uint8_t *destination,
                             std::size_t length);
    esp_err_t load_export_page(std::size_t file_index,
                               std::uint32_t step,
                               std::uint32_t logical_offset);
    void invalidate_export_read_cache();
    void serialize_record(std::uint8_t *destination);
    void install_log_capture();
    void run_capture();
    void run();
    // BIT active-test slices (flash_logger_bit.cpp, flight image): requested by the BIT
    // task through bit_request_, run here between the normal drain and flush steps.
    static constexpr std::uint8_t kBitRequestNone = 0, kBitRequestEndToEnd = 1,
                                  kBitRequestNandWrite = 2, kBitRequestCancel = 0xFF;
    static constexpr std::uint8_t kBitStateIdle = 0, kBitStatePending = 1, kBitStatePass = 2,
                                  kBitStateFail = 3, kBitStateError = 4;
    void bit_slice(bool flushed);
    void bit_finish(std::uint8_t state, std::uint16_t value);
    std::atomic<std::uint8_t> bit_request_{0};
    std::atomic<std::uint8_t> bit_state_{0};
    std::atomic<std::uint16_t> bit_value_{0};
    std::atomic<std::uint32_t> bit_passes_{0};      // loop passes: the logger heartbeat (check 6.5)
    std::uint8_t bit_nonce_[4]{};
    std::uint8_t bit_phase_ = 0;                    // logger task only
    std::uint32_t bit_started_ms_ = 0;
    std::uint32_t bit_pages_before_ = 0;
    std::uint32_t bit_marker_start_page_ = 0;       // packet stream page the writer was on when the marker was queued

    Sensors *sensors_;
    WitnessStatus &witness_status_;
    FlightStateMachine *flight_state_machine_;
    W25n01kv flash_{};
    TaskHandle_t task_handle_ = nullptr;
    TaskHandle_t capture_task_handle_ = nullptr;
    QueueHandle_t text_queue_ = nullptr;
    QueueHandle_t binary_record_queue_ = nullptr;
    QueueHandle_t imu_record_queue_ = nullptr;
    QueueHandle_t packet_queue_ = nullptr;
    std::atomic<bool> capture_packets_{false};
    // Both slots hold a flight: no session, nothing logged (never erased
    // automatically) until an erase/clear request.
    bool slots_full_ = false;
    std::atomic<std::uint32_t> packets_dropped_{0};
    std::uint8_t packet_buffer_[2 * kPagePayloadSize]{};
    std::size_t packet_buffer_used_ = 0;
    StreamBufferHandle_t uart_queue_ = nullptr;
    SemaphoreHandle_t uart_capture_mutex_ = nullptr;
    bool capture_uart_ = true; // protected by uart_capture_mutex_
    SemaphoreHandle_t frozen_semaphore_ = nullptr;
    SemaphoreHandle_t erase_semaphore_ = nullptr;
    vprintf_like_t previous_vprintf_ = nullptr;
    std::atomic<bool> capture_logs_{false};
    std::atomic<bool> console_output_enabled_{true};
    std::atomic<bool> export_requested_{false};
    // 0 none, 1 full erase, 2 fast clear (one word: no mode/request race).
    std::atomic<std::uint8_t> erase_request_{0};
    std::atomic<bool> erase_busy_{false};  // the logger task is executing an erase
    // Chosen at scan: what this boot does with the slots.
    SlotInfo slot_info_[kSlotCount]{};
    log_slot::Action choice_action_ = log_slot::Action::NewSession;
    bool resumed_in_flight_ = false;       // set in start(): reset during Boost/Coast/Descent
    bool appended_ = false;                // this session continues a flight already in the slot
    bool session_ready_ = false;
    bool slot_unusable_[kSlotCount]{};
    log_slot::WriteHealth write_health_{};
    bool nand_status_set_ = false;
    bool backoff_active_ = false;
    std::uint32_t session_bad_[32]{};      // blocks given up on this boot
    std::size_t session_bad_count_ = 0;
    std::uint32_t chunks_dropped_ = 0;
    std::uint32_t unreadable_pages_ = 0;
    std::atomic<bool> frozen_{false};
    std::atomic<bool> capture_binary_records_{true};
    std::atomic<bool> capture_imu_records_{false};
    std::atomic<bool> flight_marker_pending_{false};
    std::atomic<std::uint32_t> imu_records_dropped_{0};
    std::atomic<std::uint32_t> prelaunch_reset_generation_{0};
    esp_err_t erase_result_ = ESP_OK;
    erase_wipe::BackgroundWipe wipe_{};
    std::uint32_t wipe_start_ms_ = 0;
    std::atomic<std::uint32_t> flush_requested_{0};
    std::atomic<std::uint32_t> flush_done_{0};
    std::uint8_t binary_buffer_[4096]{};
    std::size_t binary_buffer_used_ = 0;
    std::uint8_t imu_buffer_[2 * kPagePayloadSize]{};
    std::size_t imu_buffer_used_ = 0;
    std::uint8_t text_buffer_[4096]{};
    std::size_t text_buffer_used_ = 0;
    // Two complete NAND payloads avoid wasting a third page for 16 bytes.
    std::uint8_t uart_buffer_[2 * kPagePayloadSize]{};
    std::size_t uart_buffer_used_ = 0;
    BinaryRecord prelaunch_records_[kPrelaunchRecordCount]{};
    std::size_t prelaunch_write_index_ = 0;
    std::size_t prelaunch_record_count_ = 0;
    bool prelaunch_dump_pending_ = true;
    // IMU prelaunch ring, owned by the writer task.
    ImuRecord imu_prelaunch_[kImuPrelaunchRecordCount]{};
    std::size_t imu_prelaunch_write_ = 0;
    std::size_t imu_prelaunch_count_ = 0;
    bool imu_dump_pending_ = true;
    std::uint32_t imu_descent_counter_ = 0;
    std::uint8_t imu_sequence_ = 0;            // sensor task only
    bool flight_marker_written_ = false;       // writer task only
    Writer binary_writer_{};
    Writer imu_writer_{};
    Writer text_writer_{};
    Writer uart_writer_{};
    Writer packet_writer_{};
    ExportFile export_files_[kMaximumExportFiles]{};
    std::size_t export_file_count_ = 0;
    // USB hosts read files in small sequential chunks. Retaining the current
    // NAND page prevents every 512-byte request from rescanning the file from
    // its beginning, which otherwise eventually causes host-side timeouts.
    std::uint8_t export_read_page_[W25n01kv::kPageDataSize]{};
    std::size_t export_read_file_index_ = kMaximumExportFiles;
    std::uint32_t export_read_step_ = 0;       // pages from the file's start
    std::uint32_t export_read_logical_offset_ = 0;
    std::uint16_t export_read_payload_length_ = 0;
    bool export_read_cache_valid_ = false;
    std::uint8_t current_slot_ = 0;
    std::uint32_t current_generation_ = 1;
    std::uint32_t sequence_ = 0;
};
