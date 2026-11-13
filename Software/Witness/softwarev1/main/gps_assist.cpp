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
            const std::uint16_t crc = static_cast<std::uint16_t>((data[2] << 8U) | data[3]);
            if (active_ && length == length_ && crc == crc_ && data[4] == format_ && data[5] == id_) {
                // The same transfer again (BEGIN re-sent after a lost ack or
                // a lost link): keep the chunks already received, so the
                // upload resumes instead of starting over.
                return Result::Accepted;
            }
            length_ = length;
            crc_ = crc;
            format_ = data[4];
            id_ = data[5];
            received_ = 0;
            records_ = 0;
            std::memset(have_, 0, sizeof(have_));
            std::memset(blob_, 0, sizeof(blob_));
            active_ = true;
            state_ = State::Receiving;
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
            if (!active_ && state_ == State::Complete && data[0] == id_) {
                // The ack of the first COMMIT was lost: confirm again, but
                // the records are not passed to the receiver a second time.
                return Result::AlreadyComplete;
            }
            if (!active_ || data[0] != id_) return Result::Rejected;
            if (received_ != chunk_count()) return Result::Incomplete;
            if (crc16(blob_, length_) != crc_) {
                active_ = false;
                state_ = State::Failed;
                return Result::BadCrc;
            }
            active_ = false;
            if (!parse_records(blob_, length_, records_)) {
                state_ = State::Failed;
                return Result::BadFormat;
            }
            state_ = State::Complete;
            return Result::Complete;
        }
        case kCommandStatus:
            return Result::Status;
        default:
            return Result::Ignored;
    }
}

std::size_t Assembler::first_missing_chunk() const
{
    const std::size_t n = chunk_count();
    for (std::size_t chunk = 0; chunk < n; ++chunk) {
        if ((have_[chunk / 8U] & (1U << (chunk % 8U))) == 0) return chunk;
    }
    return n;
}

void Assembler::bitmap_window(const std::size_t start, std::uint8_t *out, const std::size_t bytes) const
{
    const std::size_t n = chunk_count();
    for (std::size_t i = 0; i < bytes * 8U; ++i) {
        const std::size_t chunk = start + i;
        const bool have = chunk >= n || (have_[chunk / 8U] & (1U << (chunk % 8U))) != 0;
        if (i % 8U == 0) out[i / 8U] = 0;
        if (have) out[i / 8U] = static_cast<std::uint8_t>(out[i / 8U] | (1U << (i % 8U)));
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
