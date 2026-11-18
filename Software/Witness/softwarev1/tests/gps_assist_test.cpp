#include "gps_assist.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <vector>

using namespace gps_assist;

static void frame(Assembler &a, std::uint16_t command, std::initializer_list<std::uint8_t> bytes,
                  Assembler::Result expected) {
    std::uint8_t data[6]{};
    std::size_t i = 0;
    for (auto b : bytes) data[i++] = b;
    assert(a.handle(command, data) == expected);
}

static void send_chunk(Assembler &a, const std::vector<std::uint8_t> &blob, std::size_t off) {
    std::uint8_t d[6]{static_cast<std::uint8_t>(off >> 8), static_cast<std::uint8_t>(off)};
    for (std::size_t k = 0; k < 4 && off + k < blob.size(); ++k) d[2 + k] = blob[off + k];
    assert(a.handle(kCommandData, d) == Assembler::Result::Accepted);
}

static std::vector<std::uint8_t> make_blob(int records, std::uint8_t id = 0x41, std::size_t len = 87) {
    std::vector<std::uint8_t> blob;
    for (int r = 0; r < records; ++r) {
        blob.push_back(static_cast<std::uint8_t>(len >> 8));
        blob.push_back(static_cast<std::uint8_t>(len));
        blob.push_back(id);
        blob.push_back(static_cast<std::uint8_t>(r));
        for (std::size_t k = 2; k < len; ++k) blob.push_back(static_cast<std::uint8_t>(k));
    }
    return blob;
}

static void begin(Assembler &a, const std::vector<std::uint8_t> &blob, std::uint8_t id,
                  Assembler::Result expected = Assembler::Result::Accepted) {
    const std::uint16_t crc = crc16(blob.data(), blob.size());
    frame(a, kCommandBegin, {static_cast<std::uint8_t>(blob.size() >> 8), static_cast<std::uint8_t>(blob.size()),
                             static_cast<std::uint8_t>(crc >> 8), static_cast<std::uint8_t>(crc),
                             kFormatSkytraq, id}, expected);
}

int main() {
    const char *check = "123456789";
    assert(crc16(reinterpret_cast<const std::uint8_t *>(check), 9) == 0x29B1);

    const auto blob = make_blob(2);   // 2 x (2 + 87) = 178 bytes, 45 chunks
    Assembler a;
    std::uint8_t none[6]{};
    assert(a.handle(0x1234, none) == Assembler::Result::Ignored);
    frame(a, kCommandData, {0, 0, 1, 2, 3, 4}, Assembler::Result::Rejected);  // no transfer
    begin(a, blob, 7);
    assert(a.chunks_expected() == 45);
    // Reverse order, a duplicate, and chunk 2 (offset 8) missing.
    for (std::size_t off = 176; off >= 4; off -= 4)
        if (off != 8) send_chunk(a, blob, off);
    send_chunk(a, blob, 0);
    send_chunk(a, blob, 4);   // duplicate
    frame(a, kCommandData, {0, 2, 0, 0, 0, 0}, Assembler::Result::Rejected);    // unaligned
    frame(a, kCommandData, {0, 180, 0, 0, 0, 0}, Assembler::Result::Rejected);  // past the end
    frame(a, kCommandCommit, {9}, Assembler::Result::Rejected);                 // wrong id
    frame(a, kCommandCommit, {7}, Assembler::Result::Incomplete);
    assert(a.chunks_received() == 44);
    send_chunk(a, blob, 8);
    frame(a, kCommandCommit, {7}, Assembler::Result::Complete);
    assert(a.size() == blob.size() && std::memcmp(a.blob(), blob.data(), blob.size()) == 0);
    assert(a.records() == 2 && !a.active());
    RecordCursor cursor(a.blob(), a.size());
    const std::uint8_t *body;
    std::size_t length;
    int n = 0;
    while (cursor.next(body, length)) {
        assert(length == 87 && body[0] == 0x41 && body[1] == n);
        ++n;
    }
    assert(n == 2);

    // G10: the hand-over was refused (BUSY): the transfer is reopened, so a
    // repeated COMMIT verifies and hands over again; it is never answered
    // AlreadyComplete (= OK) for a blob that was never submitted.
    a.handover_failed();
    assert(a.active());
    frame(a, kCommandCommit, {7}, Assembler::Result::Complete);
    frame(a, kCommandCommit, {7}, Assembler::Result::AlreadyComplete);   // handed over: lost-ack case

    // G4 whitelist: only 0x41 with 87 bytes. Baud (0x05), factory defaults
    // (0x04), restart (0x01), flash-attribute messages, wrong length: BadFormat.
    for (std::uint8_t id : {0x05, 0x04, 0x01, 0x0B, 0x64, 0x08, 0x0E, 0x00, 0x42}) {
        const auto bad_blob = make_blob(1, id);
        Assembler b;
        begin(b, bad_blob, 1);
        for (std::size_t off = 0; off < bad_blob.size(); off += 4) send_chunk(b, bad_blob, off);
        frame(b, kCommandCommit, {1}, Assembler::Result::BadFormat);
        std::size_t records = 0;
        assert(!Assembler::parse_records(bad_blob.data(), bad_blob.size(), records));
    }
    {
        std::size_t records = 0;
        const auto short_blob = make_blob(1, 0x41, 5);   // wrong length for 0x41
        assert(!Assembler::parse_records(short_blob.data(), short_blob.size(), records));
        auto mixed = make_blob(1);
        const auto w = make_blob(1, 0x05);
        mixed.insert(mixed.end(), w.begin(), w.end());   // one good, one baud record
        assert(!Assembler::parse_records(mixed.data(), mixed.size(), records));
        const auto ok = make_blob(3);
        assert(Assembler::parse_records(ok.data(), ok.size(), records) && records == 3);
    }

    // CRC mismatch discards the transfer.
    frame(a, kCommandBegin, {0, 4, 0, 0, kFormatSkytraq, 1}, Assembler::Result::Accepted);
    frame(a, kCommandData, {0, 0, 0, 2, 0x41, 0x42}, Assembler::Result::Accepted);
    frame(a, kCommandCommit, {1}, Assembler::Result::BadCrc);
    assert(!a.active());
    // Bad record framing.
    std::uint8_t bad[4] = {0, 9, 1, 2};
    const std::uint16_t bad_crc = crc16(bad, 4);
    frame(a, kCommandBegin, {0, 4, static_cast<std::uint8_t>(bad_crc >> 8),
                             static_cast<std::uint8_t>(bad_crc), kFormatSkytraq, 2},
          Assembler::Result::Accepted);
    frame(a, kCommandData, {0, 0, 0, 9, 1, 2}, Assembler::Result::Accepted);
    frame(a, kCommandCommit, {2}, Assembler::Result::BadFormat);
    // Unknown format and oversize are refused at BEGIN.
    frame(a, kCommandBegin, {0, 4, 0, 0, 9, 3}, Assembler::Result::Rejected);
    frame(a, kCommandBegin, {0x20, 0x01, 0, 0, kFormatSkytraq, 3}, Assembler::Result::Rejected);
    frame(a, kCommandStatus, {}, Assembler::Result::Status);

    // Resume and acknowledgement support.
    {
        Assembler r;
        begin(r, blob, 5);
        assert(r.state() == Assembler::State::Receiving && r.transfer_id() == 5);
        for (std::size_t off : {0u, 8u, 16u}) send_chunk(r, blob, off);
        begin(r, blob, 5);                          // the same transfer again: resume
        assert(r.chunks_received() == 3);
        assert(r.first_missing_chunk() == 1);
        std::uint8_t bits[2]{};
        r.bitmap_window(0, bits, 2);
        // chunks 0, 2, 4 received; 1, 3, 5, 6, 7 missing
        assert(bits[0] == static_cast<std::uint8_t>(0x01 | 0x04 | 0x10));
        assert(bits[1] == 0x00);
        for (std::size_t off = 0; off < blob.size(); off += 4) send_chunk(r, blob, off);
        assert(r.first_missing_chunk() == r.chunks_expected());
        frame(r, kCommandCommit, {5}, Assembler::Result::Complete);
        assert(r.state() == Assembler::State::Complete);
        frame(r, kCommandCommit, {5}, Assembler::Result::AlreadyComplete);
        frame(r, kCommandCommit, {6}, Assembler::Result::Rejected);
        begin(r, blob, 6);                          // a different transfer starts over
        assert(r.chunks_received() == 0 && r.state() == Assembler::State::Receiving);
    }
    std::puts("gps_assist tests passed");
}
