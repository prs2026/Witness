#include "gps_assist.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace gps_assist;

static void frame(Assembler &a, std::uint16_t command, std::initializer_list<std::uint8_t> bytes,
                  Assembler::Result expected) {
    std::uint8_t data[6]{};
    std::size_t i = 0;
    for (auto b : bytes) data[i++] = b;
    assert(a.handle(command, data) == expected);
}

int main() {
    // CRC-16/CCITT-FALSE check value.
    const char *check = "123456789";
    assert(crc16(reinterpret_cast<const std::uint8_t *>(check), 9) == 0x29B1);

    std::vector<std::uint8_t> blob;
    for (int r = 0; r < 3; ++r) {
        blob.push_back(0);
        blob.push_back(5);
        for (int k = 0; k < 5; ++k) blob.push_back(static_cast<std::uint8_t>(0x41 + r * 10 + k));
    }
    const std::uint16_t crc = crc16(blob.data(), blob.size());
    Assembler a;
    std::uint8_t none[6]{};
    assert(a.handle(0x1234, none) == Assembler::Result::Ignored);
    frame(a, kCommandData, {0, 0, 1, 2, 3, 4}, Assembler::Result::Rejected);  // no transfer
    frame(a, kCommandBegin, {0, static_cast<std::uint8_t>(blob.size()),
                             static_cast<std::uint8_t>(crc >> 8), static_cast<std::uint8_t>(crc),
                             kFormatSkytraq, 7}, Assembler::Result::Accepted);
    assert(a.chunks_expected() == 6);  // 21 bytes
    // Out of order, with a duplicate and one missing.
    for (std::size_t off : {20u, 4u, 0u, 4u, 12u, 16u}) {
        std::uint8_t d[6]{0, static_cast<std::uint8_t>(off)};
        for (std::size_t k = 0; k < 4 && off + k < blob.size(); ++k) d[2 + k] = blob[off + k];
        assert(a.handle(kCommandData, d) == Assembler::Result::Accepted);
    }
    frame(a, kCommandData, {0, 2, 0, 0, 0, 0}, Assembler::Result::Rejected);   // unaligned
    frame(a, kCommandData, {0, 24, 0, 0, 0, 0}, Assembler::Result::Rejected);  // past the end
    frame(a, kCommandCommit, {9}, Assembler::Result::Rejected);                // wrong id
    frame(a, kCommandCommit, {7}, Assembler::Result::Incomplete);
    assert(a.chunks_received() == 5);
    {
        std::uint8_t d[6]{0, 8, blob[8], blob[9], blob[10], blob[11]};
        assert(a.handle(kCommandData, d) == Assembler::Result::Accepted);
    }
    frame(a, kCommandCommit, {7}, Assembler::Result::Complete);
    assert(a.size() == blob.size() && std::memcmp(a.blob(), blob.data(), blob.size()) == 0);
    assert(a.records() == 3 && !a.active());
    RecordCursor cursor(a.blob(), a.size());
    const std::uint8_t *body;
    std::size_t length;
    int n = 0;
    while (cursor.next(body, length)) {
        assert(length == 5 && body[0] == 0x41 + n * 10);
        ++n;
    }
    assert(n == 3);

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
    std::puts("gps_assist tests passed");
}
