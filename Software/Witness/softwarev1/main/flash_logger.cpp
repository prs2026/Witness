#include "flash_logger.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "comms.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sensors.h"
#include "witness_status.h"

namespace {

constexpr char kLogTag[] = "flash_logger";

void write_u16_be(std::uint8_t *destination, const std::uint16_t value)
{
    destination[0] = static_cast<std::uint8_t>(value >> 8);
    destination[1] = static_cast<std::uint8_t>(value);
}

void write_i16_be(std::uint8_t *destination, const std::int16_t value)
{
    write_u16_be(destination, static_cast<std::uint16_t>(value));
}

void write_u32_be(std::uint8_t *destination, const std::uint32_t value)
{
    destination[0] = static_cast<std::uint8_t>(value >> 24);
    destination[1] = static_cast<std::uint8_t>(value >> 16);
    destination[2] = static_cast<std::uint8_t>(value >> 8);
    destination[3] = static_cast<std::uint8_t>(value);
}

void write_i32_be(std::uint8_t *destination, const std::int32_t value)
{
    write_u32_be(destination, static_cast<std::uint32_t>(value));
}

void write_float_be(std::uint8_t *destination, const float value)
{
    std::uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    write_u32_be(destination, bits);
}

}  // namespace

FlashLogger::FlashLogger(
    Sensors &sensors,
    WitnessStatus &witness_status)
    : sensors_(sensors),
      witness_status_(witness_status)
{
}

esp_err_t FlashLogger::start()
{
    if (task_handle_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xTaskCreate(
            task_entry,
            "flash_logger",
            kTaskStackSize,
            this,
            kTaskPriority,
            &task_handle_) != pdPASS) {
        task_handle_ = nullptr;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void FlashLogger::task_entry(void *context)
{
    static_cast<FlashLogger *>(context)->run();
}

esp_err_t FlashLogger::run_startup_self_test()
{
    std::uint32_t test_block = W25n01gw::kBlockCount;
    for (std::uint32_t block = W25n01gw::kBlockCount;
         block-- > kFirstReservedBlock;) {
        bool bad = false;
        const esp_err_t result = flash_.is_bad_block(block, bad);
        if (result != ESP_OK) {
            return result;
        }
        if (!bad) {
            test_block = block;
            break;
        }
    }
    if (test_block == W25n01gw::kBlockCount) {
        return ESP_ERR_NOT_FOUND;
    }

    constexpr std::size_t kTestLength = 32;
    std::uint8_t before[kTestLength]{};
    std::uint8_t expected[kTestLength]{};
    std::uint8_t actual[kTestLength]{};
    const std::uint32_t test_page = test_block * W25n01gw::kPagesPerBlock;

    esp_err_t result = flash_.read(test_page, 0, before, sizeof(before));
    if (result != ESP_OK) {
        return result;
    }
    ESP_LOGI(kLogTag, "startup test block %lu before erase:",
             static_cast<unsigned long>(test_block));
    ESP_LOG_BUFFER_HEX_LEVEL(kLogTag, before, sizeof(before), ESP_LOG_INFO);

    result = flash_.erase_block(test_block);
    if (result != ESP_OK) {
        return result;
    }
    for (std::size_t index = 0; index < sizeof(expected); ++index) {
        expected[index] = static_cast<std::uint8_t>(
            0xA5U ^ static_cast<std::uint8_t>(index * 0x1DU));
    }
    result = flash_.program(test_page, 0, expected, sizeof(expected));
    if (result != ESP_OK) {
        return result;
    }
    result = flash_.read(test_page, 0, actual, sizeof(actual));
    if (result != ESP_OK) {
        return result;
    }

    ESP_LOGI(kLogTag, "startup test wrote:");
    ESP_LOG_BUFFER_HEX_LEVEL(kLogTag, expected, sizeof(expected), ESP_LOG_INFO);
    ESP_LOGI(kLogTag, "startup test read back:");
    ESP_LOG_BUFFER_HEX_LEVEL(kLogTag, actual, sizeof(actual), ESP_LOG_INFO);
    if (std::memcmp(expected, actual, sizeof(expected)) != 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    ESP_LOGI(kLogTag, "startup erase/program/read-back test passed");
    return ESP_OK;
}

esp_err_t FlashLogger::select_and_erase_log_block(
    const std::uint32_t start_block)
{
    const std::uint32_t log_block_count =
        kFirstReservedBlock - kFirstLogBlock;
    for (std::uint32_t offset = 0; offset < log_block_count; ++offset) {
        const std::uint32_t block =
            kFirstLogBlock +
            ((start_block - kFirstLogBlock + offset) % log_block_count);
        bool bad = false;
        esp_err_t result = flash_.is_bad_block(block, bad);
        if (result != ESP_OK) {
            return result;
        }
        if (bad) {
            ESP_LOGW(kLogTag, "skipping factory-bad block %lu",
                     static_cast<unsigned long>(block));
            continue;
        }
        result = flash_.erase_block(block);
        if (result == ESP_OK) {
            current_block_ = block;
            current_page_in_block_ = 0;
            ESP_LOGI(kLogTag, "logging to block %lu at %u Hz",
                     static_cast<unsigned long>(block),
                     static_cast<unsigned>(kLogRateHz));
            return ESP_OK;
        }
        ESP_LOGW(kLogTag, "erase failed for block %lu: %s",
                 static_cast<unsigned long>(block), esp_err_to_name(result));
    }
    return ESP_ERR_NOT_FOUND;
}

void FlashLogger::serialize_record(std::uint8_t *destination)
{
    Sensors::Sample sample{};
    const bool sample_available =
        sensors_.latest_sample(sample, 0) == ESP_OK;

    std::memset(destination, 0, kRecordSize);
    write_u16_be(&destination[0], kRecordMagic);
    destination[2] = kRecordVersion;
    destination[3] = witness_status_.flags();
    write_u32_be(&destination[4], sequence_++);
    write_u32_be(
        &destination[8],
        static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL));

    if (!sample_available) {
        return;
    }

    std::size_t offset = 12;
    for (std::size_t axis = 0; axis < 3; ++axis, offset += 2) {
        write_i16_be(&destination[offset], sample.low_g_accel_raw[axis]);
    }
    for (std::size_t axis = 0; axis < 3; ++axis, offset += 2) {
        write_i16_be(&destination[offset], sample.high_g_accel_raw[axis]);
    }
    for (std::size_t axis = 0; axis < 3; ++axis, offset += 2) {
        write_i16_be(&destination[offset], sample.gyro_raw[axis]);
    }
    write_i16_be(&destination[30], sample.temperature_raw);
    write_i32_be(&destination[32], sample.pressure_centi_mbar);
    write_i32_be(
        &destination[36], sample.ms5607_temperature_centi_celsius);
    write_float_be(&destination[40], sample.barometric_altitude_meters);
    write_u32_be(&destination[44], sample.battery_voltage_mv);
}

esp_err_t FlashLogger::append_sample()
{
    if (page_buffer_used_ + kRecordSize > sizeof(page_buffer_)) {
        const esp_err_t result = flush_page();
        if (result != ESP_OK) {
            return result;
        }
    }
    serialize_record(&page_buffer_[page_buffer_used_]);
    page_buffer_used_ += kRecordSize;
    return ESP_OK;
}

esp_err_t FlashLogger::flush_page()
{
    if (page_buffer_used_ == 0) {
        return ESP_OK;
    }
    std::memset(
        &page_buffer_[page_buffer_used_],
        0xFF,
        sizeof(page_buffer_) - page_buffer_used_);
    const std::uint32_t page =
        current_block_ * W25n01gw::kPagesPerBlock + current_page_in_block_;
    esp_err_t result = flash_.program(page, 0, page_buffer_, sizeof(page_buffer_));
    if (result != ESP_OK) {
        return result;
    }

    page_buffer_used_ = 0;
    ++current_page_in_block_;
    if (current_page_in_block_ >= W25n01gw::kPagesPerBlock) {
        const std::uint32_t next_block =
            current_block_ + 1U >= kFirstReservedBlock
                ? kFirstLogBlock
                : current_block_ + 1U;
        result = select_and_erase_log_block(next_block);
    }
    return result;
}

void FlashLogger::run()
{
    esp_err_t result = flash_.initialize();
    if (result == ESP_OK) {
        result = run_startup_self_test();
    }
    if (result == ESP_OK) {
        result = select_and_erase_log_block(kFirstLogBlock);
    }
    if (result != ESP_OK) {
        witness_status_.set(
            IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK, true);
        ESP_LOGE(kLogTag, "flash initialization failed: %s",
                 esp_err_to_name(result));
        flash_.release();
        task_handle_ = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    witness_status_.set(IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK, false);
    witness_status_.set(IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK, false);
    TickType_t next_wake_time = xTaskGetTickCount();
    for (;;) {
        result = append_sample();
        if (result != ESP_OK) {
            witness_status_.set(
                IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK, true);
            ESP_LOGE(kLogTag, "flash logging stopped: %s",
                     esp_err_to_name(result));
            flash_.release();
            task_handle_ = nullptr;
            vTaskDelete(nullptr);
            return;
        }
        vTaskDelayUntil(&next_wake_time, kLogPeriod);
    }
}
