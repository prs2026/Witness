#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdarg>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "w25n01kv.h"

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
    static constexpr std::uint32_t kCaptureTaskStackSize = 3072;
    static constexpr std::size_t kMaximumExportFiles = 48;
    static constexpr std::size_t kUartReceiveBufferSize = 32768;

    // Packet stream sources (record byte 4).
    static constexpr std::uint8_t kPacketSourceState = 1;   // Witness 0x02, 50 Hz
    static constexpr std::uint8_t kPacketSourceCanRx = 2;   // from Iris over CAN
    static constexpr std::uint8_t kPacketSourceCanTx = 3;   // to Iris over CAN
    static constexpr std::uint8_t kPacketSourceRadioRx = 4; // LoRa command
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
    FlashLogger(const FlashLogger &) = delete;
    FlashLogger &operator=(const FlashLogger &) = delete;

    esp_err_t start();
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
    std::size_t export_file_count() const;
    const ExportFile &export_file(std::size_t index) const;
    esp_err_t read_export_file(
        std::size_t index,
        std::uint32_t offset,
        std::uint8_t *destination,
        std::size_t length);
    void disable_console_output();

private:
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
    static void capture_task_entry(void *context);
    static int log_vprintf(const char *format, va_list arguments);
    static FlashLogger *log_sink_;

    esp_err_t read_slot_info(std::uint8_t slot, SlotInfo &info);
    esp_err_t scan_sessions(bool build_export_table);
    esp_err_t run_startup_self_test();
    esp_err_t erase_session_blocks();
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

    Sensors &sensors_;
    WitnessStatus &witness_status_;
    FlightStateMachine &flight_state_machine_;
    W25n01kv flash_{};
    TaskHandle_t task_handle_ = nullptr;
    TaskHandle_t capture_task_handle_ = nullptr;
    QueueHandle_t text_queue_ = nullptr;
    QueueHandle_t binary_record_queue_ = nullptr;
    QueueHandle_t imu_record_queue_ = nullptr;
    QueueHandle_t packet_queue_ = nullptr;
    std::atomic<bool> capture_packets_{false};
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
    std::atomic<bool> erase_requested_{false};
    std::atomic<bool> frozen_{false};
    std::atomic<bool> capture_binary_records_{true};
    std::atomic<bool> capture_imu_records_{false};
    std::atomic<bool> flight_marker_pending_{false};
    std::atomic<std::uint32_t> imu_records_dropped_{0};
    std::atomic<std::uint32_t> prelaunch_reset_generation_{0};
    esp_err_t erase_result_ = ESP_OK;
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
