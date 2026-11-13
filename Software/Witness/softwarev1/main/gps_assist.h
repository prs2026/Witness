#pragma once

// GPS assistance upload: the ground station sends a blob of receiver messages
// (for the SkyTraq PX1120S: "Set GPS Ephemeris" 0x41 bodies) in ordinary
// 10-byte 0x05 command frames, so the same path works over USB, the radio and
// CAN. Witness reassembles and checks the blob, then GpsReceiver writes the
// messages to the receiver one at a time and counts ACK/NACK.
//
// Command frames: 05 d0 d1 d2 d3 d4 d5 HH LL 0A (opcode HHLL, data d0..d5):
//   0x4741 BEGIN   d0-1 total length (u16 BE), d2-3 CRC-16/CCITT-FALSE of the
//                  blob (u16 BE), d4 format, d5 transfer id
//   0x4744 DATA    d0-1 byte offset (u16 BE, a multiple of 4), d2-5 four bytes
//                  (the last chunk is zero padded)
//   0x4745 COMMIT  d0 transfer id: verify every chunk and the CRC, then hand
//                  the blob to the receiver
//   0x4751 STATUS  logs the transfer and receiver progress (no data)
// Chunks may arrive in any order and more than once (radio retries).
//
// Blob format kFormatSkytraq: records of [length u16 BE][SkyTraq message
// body: message ID + payload], each written to the receiver as
// A0 A1 <len> <body> <xor> 0D 0A. Pure logic, no ESP-IDF (host-tested).

#include <cstddef>
#include <cstdint>

namespace gps_assist {

inline constexpr std::uint16_t kCommandBegin = 0x4741;   // 'GA'
inline constexpr std::uint16_t kCommandData = 0x4744;    // 'GD'
inline constexpr std::uint16_t kCommandCommit = 0x4745;  // 'GE'
inline constexpr std::uint16_t kCommandStatus = 0x4751;  // 'GQ'
inline constexpr std::uint8_t kFormatSkytraq = 1;
inline constexpr std::size_t kMaximumBlob = 4096;
inline constexpr std::size_t kChunkBytes = 4;
inline constexpr std::size_t kMaximumRecord = 128;

inline bool is_assist_command(const std::uint16_t command)
{
    return command == kCommandBegin || command == kCommandData ||
           command == kCommandCommit || command == kCommandStatus;
}

std::uint16_t crc16(const std::uint8_t *data, std::size_t length);

class Assembler final {
public:
    enum class Result : std::uint8_t {
        Ignored,      // not an assist frame
        Accepted,     // frame applied
        Rejected,     // bad arguments, wrong transfer id, or no transfer
        Complete,     // COMMIT verified: blob() is ready
        Incomplete,   // COMMIT with chunks missing
        BadCrc,       // COMMIT with every chunk but a CRC mismatch
        BadFormat,    // COMMIT: records do not parse
        Status,       // STATUS request
        AlreadyComplete,  // COMMIT repeated for the transfer that already completed
    };
    enum class State : std::uint8_t { Idle = 0, Receiving = 1, Complete = 2, Failed = 3 };

    // data: the six data bytes of the command frame (bytes 1..6).
    Result handle(std::uint16_t command, const std::uint8_t data[6]);

    const std::uint8_t *blob() const { return blob_; }
    std::size_t size() const { return length_; }
    std::uint8_t format() const { return format_; }
    bool active() const { return active_; }
    std::size_t chunks_received() const { return received_; }
    std::size_t chunks_expected() const { return chunk_count(); }
    std::uint8_t transfer_id() const { return id_; }
    State state() const { return state_; }
    // First chunk not yet received (chunks_expected() when none is missing).
    std::size_t first_missing_chunk() const;
    // Received-chunk bitmap from `start`: bit i of out (LSB first per byte)
    // is chunk start + i; `bytes` bytes are written (chunks past the end read 1).
    void bitmap_window(std::size_t start, std::uint8_t *out, std::size_t bytes) const;
    // Number of records in a complete blob (valid after Complete).
    std::size_t records() const { return records_; }

    // Validates the record framing of a blob; counts records.
    static bool parse_records(const std::uint8_t *blob, std::size_t length,
                              std::size_t &records);

private:
    std::size_t chunk_count() const { return (length_ + kChunkBytes - 1) / kChunkBytes; }
    std::uint8_t blob_[kMaximumBlob]{};
    std::uint8_t have_[kMaximumBlob / kChunkBytes / 8]{};
    std::size_t length_ = 0;
    std::size_t received_ = 0;
    std::size_t records_ = 0;
    std::uint16_t crc_ = 0;
    std::uint8_t format_ = 0;
    std::uint8_t id_ = 0;
    bool active_ = false;
    State state_ = State::Idle;
};

// Iterates the records of a verified blob.
class RecordCursor final {
public:
    RecordCursor(const std::uint8_t *blob, std::size_t length) : blob_(blob), length_(length) {}
    // Next record body; false at the end.
    bool next(const std::uint8_t *&body, std::size_t &length);
    std::size_t index() const { return index_; }

private:
    const std::uint8_t *blob_;
    std::size_t length_;
    std::size_t offset_ = 0;
    std::size_t index_ = 0;
};

}  // namespace gps_assist
