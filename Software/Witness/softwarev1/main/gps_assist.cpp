#include "gps_assist.h"

#include <cstring>

namespace gps_assist {

std::uint16_t crc16(const std::uint8_t *data, const std::size_t length)
{
    std::uint16_t crc = 0xFFFF;  // CRC-16/CCITT-FALSE: poly 0x1021, no reflection
    for (std::size_t i = 0; i < length; ++i) {
        crc ^= static_cast<std::uint16_t>(data[i]) << 8U;
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000U) ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                                  : static_cast<std::uint16_t>(crc << 1U);
    }
    return crc;
}

bool Assembler::parse_records(const std::uint8_t *blob, const std::size_t length,
                              std::size_t &records)
{
    records = 0;
    std::size_t offset = 0;
    while (offset < length) {
        if (offset + 2 > length) return false;
        const std::size_t n = (static_cast<std::size_t>(blob[offset]) << 8U) | blob[offset + 1];
        if (n == 0 || n > kMaximumRecord || offset + 2 + n > length) return false;
        offset += 2 + n;
        ++records;
    }
    return records > 0;
}

Assembler::Result Assembler::handle(const std::uint16_t command, const std::uint8_t data[6])
{
    switch (command) {
        case kCommandBegin: {
            const std::size_t length = (static_cast<std::size_t>(data[0]) << 8U) | data[1];
            if (length == 0 || length > kMaximumBlob || data[4] != kFormatSkytraq) {
                active_ = false;
                return Result::Rejected;
            }
            length_ = length;
            crc_ = static_cast<std::uint16_t>((data[2] << 8U) | data[3]);
            format_ = data[4];
            id_ = data[5];
            received_ = 0;
            records_ = 0;
            std::memset(have_, 0, sizeof(have_));
            std::memset(blob_, 0, sizeof(blob_));
            active_ = true;
            return Result::Accepted;
        }
        case kCommandData: {
            if (!active_) return Result::Rejected;
            const std::size_t offset = (static_cast<std::size_t>(data[0]) << 8U) | data[1];
            if (offset % kChunkBytes != 0 || offset >= length_) return Result::Rejected;
            const std::size_t chunk = offset / kChunkBytes;
            const std::size_t n = length_ - offset < kChunkBytes ? length_ - offset : kChunkBytes;
            std::memcpy(blob_ + offset, data + 2, n);
            const std::uint8_t mask = static_cast<std::uint8_t>(1U << (chunk % 8U));
            if ((have_[chunk / 8U] & mask) == 0) {
                have_[chunk / 8U] |= mask;
                ++received_;
            }
            return Result::Accepted;
        }
        case kCommandCommit: {
            if (!active_ || data[0] != id_) return Result::Rejected;
            if (received_ != chunk_count()) return Result::Incomplete;
            if (crc16(blob_, length_) != crc_) {
                active_ = false;
                return Result::BadCrc;
            }
            active_ = false;
            if (!parse_records(blob_, length_, records_)) return Result::BadFormat;
            return Result::Complete;
        }
        case kCommandStatus:
            return Result::Status;
        default:
            return Result::Ignored;
    }
}

bool RecordCursor::next(const std::uint8_t *&body, std::size_t &length)
{
    if (offset_ + 2 > length_) return false;
    const std::size_t n = (static_cast<std::size_t>(blob_[offset_]) << 8U) | blob_[offset_ + 1];
    if (n == 0 || offset_ + 2 + n > length_) return false;
    body = blob_ + offset_ + 2;
    length = n;
    offset_ += 2 + n;
    ++index_;
    return true;
}

}  // namespace gps_assist
