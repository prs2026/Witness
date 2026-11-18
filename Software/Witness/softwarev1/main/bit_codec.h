#pragma once

#include <cstddef>
#include <cstdint>

#include "bit_logic.h"
#include "comms.h"

// Wire encoding of the BIT packets 0x0B (summary) and 0x10 (detail) and of the
// command ack detail. Pure; golden bytes are shared with Horizon's test
// (Software/Horizon/tests/test_bit.py).
namespace bit::codec {

static_assert(IRIS_PACKET_BIT_SUMMARY_DATA_LENGTH == 12U, "summary layout");
static_assert(IRIS_PACKET_BIT_DETAIL_DATA_LENGTH ==
                  IRIS_PACKET_BIT_DETAIL_HEADER_LENGTH +
                      IRIS_PACKET_BIT_DETAIL_RECORDS * IRIS_PACKET_BIT_DETAIL_RECORD_LENGTH,
              "detail layout");
static_assert(IRIS_PACKET_BIT_DETAIL_DATA_LENGTH <= IRIS_PACKET_MAX_DATA_LENGTH,
              "the detail frame must fit the radio/forwarder maximum");
// IDs avoided on purpose: 0x0A is the end-of-frame byte, 0x0D is CR.
static_assert(IRIS_PACKET_ID_BIT_SUMMARY != IRIS_PACKET_EOF_VALUE && IRIS_PACKET_ID_BIT_SUMMARY != 0x0DU, "id");
static_assert(IRIS_PACKET_ID_BIT_DETAIL != IRIS_PACKET_EOF_VALUE && IRIS_PACKET_ID_BIT_DETAIL != 0x0DU, "id");

constexpr std::size_t kSummaryFrameLength = IRIS_PACKET_BIT_SUMMARY_FRAME_LENGTH;
constexpr std::size_t kDetailFrameLength = IRIS_PACKET_BIT_DETAIL_FRAME_LENGTH;
constexpr std::size_t kDetailPageCount =
    (kCheckCount + IRIS_PACKET_BIT_DETAIL_RECORDS - 1U) / IRIS_PACKET_BIT_DETAIL_RECORDS;

// Writes the 0x0B frame (14 bytes) into frame[].
inline std::size_t encode_summary(const Summary &s, std::uint8_t *frame)
{
    std::uint8_t *d = frame + IRIS_PACKET_ID_LENGTH;
    frame[0] = IRIS_PACKET_ID_BIT_SUMMARY;
    d[0] = static_cast<std::uint8_t>((IRIS_PACKET_BIT_SUMMARY_VERSION << 4U) |
                                     (static_cast<std::uint8_t>(s.verdict) & 0x07U) |
                                     (s.bench ? IRIS_BIT_SUMMARY_BENCH_FLAG : 0U));
    d[1] = static_cast<std::uint8_t>((static_cast<std::uint8_t>(s.kind) & 0x03U) |
                                     ((s.flight_state & 0x07U) << 2U) |
                                     (s.core_done ? IRIS_BIT_SUMMARY_CORE_DONE_FLAG : 0U) |
                                     (s.settled ? IRIS_BIT_SUMMARY_SETTLED_FLAG : 0U) |
                                     (s.active_ran ? IRIS_BIT_SUMMARY_ACTIVE_FLAG : 0U));
    d[2] = static_cast<std::uint8_t>(s.blocking_fail_mask >> 8U);
    d[3] = static_cast<std::uint8_t>(s.blocking_fail_mask);
    d[4] = static_cast<std::uint8_t>(s.degraded_fail_mask >> 8U);
    d[5] = static_cast<std::uint8_t>(s.degraded_fail_mask);
    d[6] = static_cast<std::uint8_t>(s.not_evaluated_mask >> 8U);
    d[7] = static_cast<std::uint8_t>(s.not_evaluated_mask);
    d[8] = s.first_fail_id;
    d[9] = s.sequence;
    d[10] = static_cast<std::uint8_t>(s.age_s >> 8U);
    d[11] = static_cast<std::uint8_t>(s.age_s);
    frame[IRIS_PACKET_ID_LENGTH + IRIS_PACKET_BIT_SUMMARY_DATA_LENGTH] = IRIS_PACKET_EOF_VALUE;
    return kSummaryFrameLength;
}

// Decoder (the ground side implements the same in Python; this one is for the
// golden tests and for the log tool).
inline bool decode_summary(const std::uint8_t *frame, const std::size_t length, Summary &s)
{
    if (length != kSummaryFrameLength || frame[0] != IRIS_PACKET_ID_BIT_SUMMARY ||
        frame[kSummaryFrameLength - 1U] != IRIS_PACKET_EOF_VALUE)
        return false;
    const std::uint8_t *d = frame + IRIS_PACKET_ID_LENGTH;
    if ((d[0] >> 4U) != IRIS_PACKET_BIT_SUMMARY_VERSION) return false;
    s = Summary{};
    s.verdict = static_cast<Verdict>(d[0] & 0x07U);
    s.bench = (d[0] & IRIS_BIT_SUMMARY_BENCH_FLAG) != 0U;
    s.kind = static_cast<RunKind>(d[1] & 0x03U);
    s.flight_state = static_cast<std::uint8_t>((d[1] >> 2U) & 0x07U);
    s.core_done = (d[1] & IRIS_BIT_SUMMARY_CORE_DONE_FLAG) != 0U;
    s.settled = (d[1] & IRIS_BIT_SUMMARY_SETTLED_FLAG) != 0U;
    s.active_ran = (d[1] & IRIS_BIT_SUMMARY_ACTIVE_FLAG) != 0U;
    s.blocking_fail_mask = static_cast<std::uint16_t>((d[2] << 8U) | d[3]);
    s.degraded_fail_mask = static_cast<std::uint16_t>((d[4] << 8U) | d[5]);
    s.not_evaluated_mask = static_cast<std::uint16_t>((d[6] << 8U) | d[7]);
    s.first_fail_id = d[8];
    s.sequence = d[9];
    s.age_s = static_cast<std::uint16_t>((d[10] << 8U) | d[11]);
    return true;
}

inline std::uint8_t pack_flags(const Eval &e)
{
    return static_cast<std::uint8_t>((static_cast<std::uint8_t>(e.result) & 0x07U) |
                                     ((static_cast<std::uint8_t>(e.cls) & 0x03U) << 3U) |
                                     (e.bench ? IRIS_BIT_DETAIL_BENCH_FLAG : 0U));
}

// Writes detail page `page` (0-based) of the engine's current results into
// frame[] (IRIS_PACKET_BIT_DETAIL_FRAME_LENGTH bytes). False for a page that
// does not exist.
inline bool encode_detail(const Engine &engine, const std::uint8_t page, std::uint8_t *frame)
{
    if (page >= kDetailPageCount) return false;
    frame[0] = IRIS_PACKET_ID_BIT_DETAIL;
    std::uint8_t *d = frame + IRIS_PACKET_ID_LENGTH;
    d[0] = page;
    d[1] = static_cast<std::uint8_t>(kDetailPageCount);
    d[2] = engine.sequence();
    for (std::size_t r = 0; r < IRIS_PACKET_BIT_DETAIL_RECORDS; ++r) {
        std::uint8_t *rec = d + IRIS_PACKET_BIT_DETAIL_HEADER_LENGTH + r * IRIS_PACKET_BIT_DETAIL_RECORD_LENGTH;
        const std::size_t index = static_cast<std::size_t>(page) * IRIS_PACKET_BIT_DETAIL_RECORDS + r;
        if (index >= kCheckCount) {
            rec[0] = IRIS_BIT_DETAIL_UNUSED_ID; rec[1] = 0; rec[2] = 0; rec[3] = 0;
            continue;
        }
        const Eval &e = engine.slot(index).shown;
        rec[0] = kChecks[index].id;
        rec[1] = pack_flags(e);
        rec[2] = static_cast<std::uint8_t>(e.value >> 8U);
        rec[3] = static_cast<std::uint8_t>(e.value);
    }
    frame[IRIS_PACKET_ID_LENGTH + IRIS_PACKET_BIT_DETAIL_DATA_LENGTH] = IRIS_PACKET_EOF_VALUE;
    return true;
}

struct DetailRecord {
    std::uint8_t id = 0;
    Result result = Result::NotYet;
    Class cls = Class::Info;
    bool bench = false;
    std::uint16_t value = 0;
};

// Decodes a 0x10 frame; fills records[] (up to 24) and returns how many are in use.
inline int decode_detail(const std::uint8_t *frame, const std::size_t length, std::uint8_t &page,
                         std::uint8_t &pages, std::uint8_t &sequence, DetailRecord *records)
{
    if (length != kDetailFrameLength || frame[0] != IRIS_PACKET_ID_BIT_DETAIL ||
        frame[kDetailFrameLength - 1U] != IRIS_PACKET_EOF_VALUE)
        return -1;
    const std::uint8_t *d = frame + IRIS_PACKET_ID_LENGTH;
    page = d[0]; pages = d[1]; sequence = d[2];
    int used = 0;
    for (std::size_t r = 0; r < IRIS_PACKET_BIT_DETAIL_RECORDS; ++r) {
        const std::uint8_t *rec = d + IRIS_PACKET_BIT_DETAIL_HEADER_LENGTH + r * IRIS_PACKET_BIT_DETAIL_RECORD_LENGTH;
        if (rec[0] == IRIS_BIT_DETAIL_UNUSED_ID) continue;
        DetailRecord &o = records[used++];
        o.id = rec[0];
        o.result = static_cast<Result>(rec[1] & 0x07U);
        o.cls = static_cast<Class>((rec[1] >> 3U) & 0x03U);
        o.bench = (rec[1] & IRIS_BIT_DETAIL_BENCH_FLAG) != 0U;
        o.value = static_cast<std::uint16_t>((rec[2] << 8U) | rec[3]);
    }
    return used;
}

// Ack detail of a BIT run: verdict in the high byte, number of groups with a
// blocking failure in the low byte (so the radio operator gets go/no-go from
// the ack alone).
constexpr std::uint16_t ack_detail(const Summary &s)
{
    return static_cast<std::uint16_t>((static_cast<std::uint16_t>(s.verdict) << 8U) | s.blocking_fail_groups);
}

}  // namespace bit::codec
