#include "can_transport.h"
#include <cassert>
#include <iostream>
#include <vector>

using namespace intercomm;
static std::vector<Frame> frames(std::size_t length, std::uint8_t seq = 1) {
    std::vector<std::uint8_t> bytes(length);
    for (std::size_t i = 0; i < length; ++i) bytes[i] = static_cast<std::uint8_t>(i);
    std::vector<Frame> result(fragment_count(length));
    for (std::size_t i = 0; i < result.size(); ++i) {
        assert(encode_fragment(bytes.data(), length, seq, i, result[i]));
        result[i].identifier = kIrisToWitnessId;
    }
    return result;
}
int main() {
    for (std::size_t n = 2; n <= kMaxPacketLength; ++n) {
        Reassembler rx(kIrisToWitnessId);
        auto tx = frames(n);
        for (std::size_t i = 0; i < tx.size(); ++i)
            assert(rx.accept(tx[i], static_cast<std::uint32_t>(i)) == (i + 1 == tx.size()));
        assert(rx.size() == n);
        for (std::size_t i = 0; i < n; ++i) assert(rx.data()[i] == i);
        if (tx.size() > 1) assert(!rx.accept(tx.back(), 100)); // orphan tail
    }
    Reassembler rx(kIrisToWitnessId);
    auto tx = frames(19);
    assert(!rx.accept(tx[0], 0));
    assert(!rx.accept(tx[2], 1)); // missing fragment
    assert(!rx.accept(tx[3], 2));
    assert(!rx.accept(tx[0], 3));
    assert(!rx.accept(tx[1], 253)); // total transfer timeout
    assert(!rx.accept(tx[0], 260));
    auto wrong = tx[1]; wrong.data[0]++;
    assert(!rx.accept(wrong, 261));
    assert(!rx.accept(tx[2], 262));
    assert(!rx.accept(tx[0], 270));
    wrong = tx[1]; wrong.identifier = kWitnessToIrisId;
    assert(!rx.accept(wrong, 271)); // other traffic does not break transfer
    wrong = tx[1]; wrong.extended = true;
    assert(!rx.accept(wrong, 271));
    wrong = tx[1]; wrong.remote = true;
    assert(!rx.accept(wrong, 271));
    assert(!rx.accept(tx[1], 272));
    assert(!rx.accept(tx[2], 273));
    assert(rx.accept(tx[3], 274));
    for (unsigned dlc = 0; dlc <= 255; ++dlc) {
        wrong = tx[0]; wrong.length = static_cast<std::uint8_t>(dlc);
        Reassembler malformed(kIrisToWitnessId);
        assert(!malformed.accept(wrong, 0));
    }
    assert(!rx.accept(tx[0], UINT32_MAX - 10));
    assert(!rx.accept(tx[1], 0)); // timestamp wrap
    assert(!rx.accept(tx[2], 1));
    assert(rx.accept(tx[3], 2));
    // A new transfer's first fragment supersedes an abandoned transfer.
    assert(!rx.accept(tx[0], 300));
    auto next = frames(10, 255);
    assert(!rx.accept(next[0], 301));
    assert(rx.accept(next[1], 302));
    Frame out;
    assert(!encode_fragment(nullptr, 10, 0, 0, out));
    std::uint8_t bytes[104]{};
    assert(!encode_fragment(bytes, 104, 0, 0, out));
    assert(!encode_fragment(bytes, 10, 0, 2, out));
    std::cout << "CAN transport tests passed\n";
}
