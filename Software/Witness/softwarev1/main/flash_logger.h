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
#include "w25n01kv.h"

class Sensors;
class WitnessStatus;

// Append-only log store for the raw SPI NAND. A read-only FAT volume is
// generated from its export table by UsbMassStorage; a desktop OS never writes
// FAT metadata directly to the NAND.
class FlashLogger final {
public:
    static constexpr UBaseType_t kTaskPriority = 3;
    static constexpr std::uint32_t kLogRateHz = 50;
    static constexpr std::uint32_t kFlushIntervalMs = 1000;
    static constexpr std::uint32_t kTaskStackSize = 7168;
    static constexpr std::size_t kMaximumExportFiles = 32;

    struct ExportFile {
        char name[13];                 // DOS 8.3 name, including terminator
        std::uint32_t size;
        std::uint32_t generation;
        std::uint8_t slot;
        std::uint8_t type;
    };

    FlashLogger(Sensors &sensors, WitnessStatus &witness_status);
    FlashLogger(const FlashLogger &) = delete;
    FlashLogger &operator=(const FlashLogger &) = delete;

    esp_err_t start();

    // Flushes the current session and permanently freezes logging until reset.
    // This must complete before exposing the read-only USB disk.
    esp_err_t prepare_mass_storage(TickType_t timeout = portMAX_DELAY);
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
    static constexpr std::size_t kRecordSize = 48;
    static constexpr std::size_t kPageHeaderSize = 8;
    static constexpr std::size_t kPagePayloadSize =
        W25n01kv::kPageDataSize - kPageHeaderSize;
    static constexpr std::uint32_t kReservedBlocks = 20;
    static constexpr std::uint32_t kSessionCount = 16;
    static constexpr std::uint32_t kBlocksPerSession = 62;
    static constexpr std::uint32_t kBinaryBlocksPerSession = 50;
    static constexpr std::uint32_t kTextBlocksPerSession =
        kBlocksPerSession - kBinaryBlocksPerSession;
    static constexpr std::uint32_t kSessionMagic = 0x574C5331U; // WLS1
    static constexpr std::uint32_t kPageMagic = 0x574C5047U;    // WLPG
    static constexpr std::uint8_t kStoreVersion = 1;
    static constexpr std::uint8_t kBinaryType = 1;
    static constexpr std::uint8_t kTextType = 2;
    static constexpr std::uint16_t kRecordMagic = 0x574C;       // WL
    static constexpr std::uint8_t kRecordVersion = 1;

    struct LogLine {
        std::uint16_t length;
        char data[254];
    };

    struct Writer {
        std::uint8_t type;
        std::uint32_t first_block;
        std::uint32_t block_count;
        std::uint32_t page_index;
        std::uint32_t last_erased_block;
    };

    static_assert(kLogRateHz > 0 && (1000U % kLogRateHz) == 0);
    static_assert(kFlushIntervalMs == 500 || kFlushIntervalMs == 1000,
                  "Use a 0.5 s or 1 s NAND flush interval");
    static_assert(kSessionCount * kBlocksPerSession <=
                  W25n01kv::kBlockCount - kReservedBlocks);

    static void task_entry(void *context);
    static int log_vprintf(const char *format, va_list arguments);
    static FlashLogger *log_sink_;

    esp_err_t scan_sessions(bool build_export_table);
    esp_err_t create_session();
    esp_err_t append_bytes(Writer &writer,
                           const std::uint8_t *data,
                           std::size_t length);
    esp_err_t flush_binary();
    esp_err_t flush_text();
    esp_err_t append_sample();
    esp_err_t scan_file_size(std::uint8_t slot,
                             std::uint8_t type,
                             std::uint32_t &size);
    esp_err_t read_file_data(const ExportFile &file,
                             std::uint32_t offset,
                             std::uint8_t *destination,
                             std::size_t length);
    void serialize_record(std::uint8_t *destination);
    void install_log_capture();
    void run();

    Sensors &sensors_;
    WitnessStatus &witness_status_;
    W25n01kv flash_{};
    TaskHandle_t task_handle_ = nullptr;
    QueueHandle_t text_queue_ = nullptr;
    SemaphoreHandle_t frozen_semaphore_ = nullptr;
    vprintf_like_t previous_vprintf_ = nullptr;
    std::atomic<bool> capture_logs_{false};
    std::atomic<bool> console_output_enabled_{true};
    std::atomic<bool> export_requested_{false};
    std::atomic<bool> frozen_{false};
    std::uint8_t binary_buffer_[4096]{};
    std::size_t binary_buffer_used_ = 0;
    Writer binary_writer_{};
    Writer text_writer_{};
    ExportFile export_files_[kMaximumExportFiles]{};
    std::size_t export_file_count_ = 0;
    std::uint8_t current_slot_ = 0;
    std::uint32_t current_generation_ = 1;
    std::uint32_t sequence_ = 0;
};
