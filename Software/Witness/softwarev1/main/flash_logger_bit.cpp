// BIT active-test slices of the flash logger (flight image only). They run in
// the logger task, the only user of SPI2, between its normal drain and flush
// steps, and only when the BIT task asked for them (commanded run, Pad Idle or
// Landed, still gate passed). Nothing here runs at start-up or after a warm
// reset. See BIT_PLAN_2026-11-18.md checks 5.2 and 6.3.
#include <cstring>

#include "bit_limits.h"
#include "bit_marker_scan.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "flash_logger.h"

namespace {
constexpr char kLogTag[] = "flash_logger";
constexpr std::size_t kMarkerLength = 12;
}  // namespace

void FlashLogger::bit_finish(const std::uint8_t state, const std::uint16_t value)
{
    bit_value_.store(value, std::memory_order_relaxed);
    bit_phase_ = 0;
    bit_request_.store(kBitRequestNone, std::memory_order_relaxed);
    bit_state_.store(state, std::memory_order_release);   // last: the BIT task polls it
}

void FlashLogger::bit_slice(const bool flushed)
{
    bit_passes_.fetch_add(1, std::memory_order_relaxed);
    const std::uint8_t request = bit_request_.load(std::memory_order_acquire);
    if (request == kBitRequestNone && bit_phase_ == 0) return;
    const std::uint32_t now_ms = static_cast<std::uint32_t>(esp_timer_get_time() / 1000);

    if (request == kBitRequestCancel) {
        bit_phase_ = 0;
        bit_request_.store(kBitRequestNone, std::memory_order_relaxed);
        bit_state_.store(kBitStateIdle, std::memory_order_release);
        return;
    }
    // Active tests never run once the flight state is no longer on the ground
    // or after a warm reset in flight (the BIT task checked too; this is the owner's own check).
    if (!ground_state() || resumed_in_flight_ || !session_ready_ || export_requested_.load()) {
        bit_finish(kBitStateError, 0xFF);
        return;
    }

    if (bit_phase_ == 0 && request == kBitRequestNandWrite) {
        // 5.2 extended: erase a reserved block, program a full page, read it back and compare.
        std::uint32_t block = W25n01kv::kBlockCount;
        for (std::uint32_t candidate = W25n01kv::kBlockCount;
             candidate-- > W25n01kv::kBlockCount - kReservedBlocks;) {
            bool bad = false;
            if (flash_.is_bad_block(candidate, bad) != ESP_OK) { bit_finish(kBitStateFail, 1); return; }
            if (!bad) { block = candidate; break; }
        }
        if (block == W25n01kv::kBlockCount) { bit_finish(kBitStateFail, 2); return; }
        std::uint8_t *pattern = export_read_page_;   // unused in flight; the cache is invalidated below
        invalidate_export_read_cache();
        for (std::size_t i = 0; i < W25n01kv::kPageDataSize; ++i)
            pattern[i] = static_cast<std::uint8_t>((i * 0x1DU) ^ 0xA5U ^ (i >> 8));
        const std::uint32_t page = block * W25n01kv::kPagesPerBlock;
        const std::int64_t t0 = esp_timer_get_time();
        esp_err_t r = flash_.erase_block(block);
        const std::int64_t t1 = esp_timer_get_time();
        if (r == ESP_OK) r = flash_.program(page, 0, pattern, W25n01kv::kPageDataSize);
        const std::int64_t t2 = esp_timer_get_time();
        std::uint16_t code = 0;
        if (r != ESP_OK) code = 3;
        std::uint8_t chunk[256];
        for (std::size_t offset = 0; code == 0 && offset < W25n01kv::kPageDataSize; offset += sizeof(chunk)) {
            r = flash_.read(page, static_cast<std::uint16_t>(offset), chunk, sizeof(chunk));
            if (r == ESP_ERR_INVALID_CRC) code = 4;          // uncorrectable ECC
            else if (r != ESP_OK) code = 5;
            else if (std::memcmp(chunk, pattern + offset, sizeof(chunk)) != 0) code = 6;
        }
        invalidate_export_read_cache();
        const std::uint32_t erase_us = static_cast<std::uint32_t>(t1 - t0);
        const std::uint32_t program_us = static_cast<std::uint32_t>(t2 - t1);
        ESP_LOGI(kLogTag, "BIT NAND write test block %lu: erase %lu us, program %lu us (datasheet max %lu / %lu us): %s",
                 static_cast<unsigned long>(block), static_cast<unsigned long>(erase_us),
                 static_cast<unsigned long>(program_us), static_cast<unsigned long>(bit::limits::kNandEraseMaxUs),
                 static_cast<unsigned long>(bit::limits::kNandProgramMaxUs), code == 0 ? "ok" : "FAILED");
        // Pass value: erase time in 0.1 ms units (timing is reported; only data errors fail the check).
        bit_finish(code == 0 ? kBitStatePass : kBitStateFail,
                   code == 0 ? static_cast<std::uint16_t>(erase_us / 100U > 65535U ? 65535U : erase_us / 100U) : code);
        return;
    }

    if (bit_phase_ == 0 && request == kBitRequestEndToEnd) {
        // 6.3 phase A: a fresh marker through the normal packet path.
        std::uint8_t marker[kMarkerLength]{};
        marker[0] = 'B'; marker[1] = 'I'; marker[2] = 'T'; marker[3] = 1;
        std::memcpy(marker + 4, bit_nonce_, 4);
        marker[8] = 'E'; marker[9] = 'N'; marker[10] = 'D'; marker[11] = 0x0A;
        bit_pages_before_ = packet_writer_.sequence;
        bit_marker_start_page_ = packet_writer_.page_index;
        const esp_err_t r = log_packet(kPacketSourceBit, marker, sizeof(marker));
        if (r == ESP_ERR_INVALID_STATE) { bit_finish(kBitStateFail, 1); return; }   // packet capture is off
        if (r != ESP_OK) { bit_finish(kBitStateFail, 2); return; }                  // queue full
        bit_started_ms_ = now_ms;
        bit_phase_ = 1;
        return;
    }

    if (bit_phase_ == 1) {
        // 6.3 phase B: after the next flush, find the marker in the last pages written to the
        // packet stream by reading them back from the NAND (not from a buffer).
        if (now_ms - bit_started_ms_ > bit::limits::kEndToEndTimeoutMs) { bit_finish(kBitStateFail, 5); return; }
        if (!flushed || packet_buffer_used_ != 0 || packet_writer_.sequence == bit_pages_before_) return;
        // The marker is somewhere after the page the writer was on when it was queued (behind
        // the traffic queued before it, in front of the traffic after it).
        invalidate_export_read_cache();
        std::uint8_t needle[bit_marker::kNeedleLength] = {'B', 'I', 'T', 1};
        std::memcpy(needle + 4, bit_nonce_, 4);
        bit_marker::Scanner scanner(needle);
        const std::uint32_t capacity = packet_writer_.block_count * W25n01kv::kPagesPerBlock;
        const std::uint32_t pages = bit_marker::pages_to_scan(bit_marker_start_page_, packet_writer_.page_index, capacity);
        bool found = false;
        std::uint16_t code = 0;
        std::uint32_t scanned = 0;
        for (; scanned < pages && !found; ++scanned) {
            const std::uint32_t page = packet_writer_.first_block * W25n01kv::kPagesPerBlock +
                                       (bit_marker_start_page_ + scanned) % capacity;
            const esp_err_t r = read_page_retry(page, 0, export_read_page_, W25n01kv::kPageDataSize);
            if (r != ESP_OK) {   // unreadable or an abandoned page: no continuity, keep looking
                if (code == 0) code = r == ESP_ERR_INVALID_CRC ? 3 : 4;
                scanner.gap();
                continue;
            }
            const std::uint8_t *h = export_read_page_;
            const std::uint32_t magic = (static_cast<std::uint32_t>(h[0]) << 24) | (static_cast<std::uint32_t>(h[1]) << 16) |
                                        (static_cast<std::uint32_t>(h[2]) << 8) | h[3];
            if (magic != kPageMagic) { scanner.gap(); continue; }   // not written
            std::size_t length = (static_cast<std::size_t>(h[6]) << 8) | h[7];
            if (length > kPagePayloadSize) length = kPagePayloadSize;
            found = scanner.feed(export_read_page_ + kPageHeaderSize, length);
        }
        invalidate_export_read_cache();
        if (found) {
            const std::uint32_t latency = now_ms - bit_started_ms_;
            bit_finish(kBitStatePass, static_cast<std::uint16_t>(latency > 65535U ? 65535U : latency));
        } else {
            ESP_LOGW(kLogTag, "BIT end-to-end: marker not found in %lu of %lu page(s) after the marker was queued (page %lu to %lu)",
                     static_cast<unsigned long>(scanned), static_cast<unsigned long>(pages),
                     static_cast<unsigned long>(bit_marker_start_page_), static_cast<unsigned long>(packet_writer_.page_index));
            bit_finish(kBitStateFail, code != 0 ? code : 4);
        }
        return;
    }
}
