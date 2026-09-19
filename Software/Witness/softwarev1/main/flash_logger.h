#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "w25n01gw.h"

class Sensors;
class WitnessStatus;

class FlashLogger final {
public:
    static constexpr UBaseType_t kTaskPriority = 3;
    static constexpr std::uint32_t kLogRateHz = 50;
    static constexpr std::uint32_t kTaskStackSize = 4096;

    FlashLogger(Sensors &sensors, WitnessStatus &witness_status);
    FlashLogger(const FlashLogger &) = delete;
    FlashLogger &operator=(const FlashLogger &) = delete;

    esp_err_t start();

private:
    static constexpr TickType_t kLogPeriod =
        pdMS_TO_TICKS(1000U / kLogRateHz);
    // Big-endian 48-byte record: magic[2], version, status, sequence[4],
    // uptime_ms[4], low-g[3x2], high-g[3x2], gyro[3x2], IMU temp[2],
    // pressure[4], barometer temp[4], altitude float32[4], battery_mV[4].
    static constexpr std::size_t kRecordSize = 48;
    static constexpr std::uint32_t kFirstLogBlock = 0;
    // Reserve the highest 20 blocks for the destructive startup self-test and
    // future bad-block replacement. Normal logging never writes these blocks.
    static constexpr std::uint32_t kFirstReservedBlock =
        W25n01gw::kBlockCount - 20U;
    static constexpr std::uint16_t kRecordMagic = 0x574C;  // "WL"
    static constexpr std::uint8_t kRecordVersion = 1;

    static_assert(kLogRateHz > 0 && (1000U % kLogRateHz) == 0,
                  "Flash logging rate must divide evenly into 1000 ms");

    static void task_entry(void *context);
    esp_err_t run_startup_self_test();
    esp_err_t select_and_erase_log_block(std::uint32_t start_block);
    esp_err_t append_sample();
    esp_err_t flush_page();
    void serialize_record(std::uint8_t *destination);
    void run();

    Sensors &sensors_;
    WitnessStatus &witness_status_;
    W25n01gw flash_{};
    TaskHandle_t task_handle_ = nullptr;
    std::uint8_t page_buffer_[W25n01gw::kPageDataSize]{};
    std::size_t page_buffer_used_ = 0;
    std::uint32_t current_block_ = kFirstLogBlock;
    std::uint32_t current_page_in_block_ = 0;
    std::uint32_t sequence_ = 0;
};
