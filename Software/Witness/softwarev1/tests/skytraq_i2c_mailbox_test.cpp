#include "skytraq_i2c_mailbox.h"

#include <array>
#include <cassert>
#include <cstdio>
#include <vector>

struct Device {
    std::array<std::uint8_t, 9> registers{};
    bool io_ok = true;
    std::vector<std::uint8_t> written;
    static bool read(void *context, std::uint8_t reg, std::uint8_t &value) {
        auto &self = *static_cast<Device *>(context);
        assert(reg < self.registers.size());
        if (!self.io_ok) return false;
        value = self.registers[reg];
        return true;
    }
    static bool write(void *context, std::uint8_t reg, std::uint8_t value) {
        auto &self = *static_cast<Device *>(context);
        assert(reg < self.registers.size());
        if (!self.io_ok) return false;
        self.registers[reg] = value;
        if (reg == 8 && (value & 1)) {
            const auto count = (value >> 2) & 7U;
            assert(count <= 3);
            for (unsigned i = 0; i < count; ++i)
                self.written.push_back(self.registers[5 + i]);
        }
        return true;
    }
    SkytraqI2cMailbox::Transport transport() { return {this, read, write}; }
};

int main() {
    using Result = SkytraqI2cMailbox::Result;
    static_assert(SkytraqI2cMailbox::kAddress == (0x78 >> 1));
    Device device;
    SkytraqI2cMailbox mailbox(device.transport());
    std::uint8_t data[5]{};
    std::size_t size = 99;
    assert(mailbox.read(data, size) == Result::Busy && size == 0);
    // Five-byte raw payloads, including embedded zero and CR/LF, remain intact.
    const std::array<std::uint8_t, 5> expected{{0xA0, 0x00, 0x0D, 0x0A, 0xFF}};
    for (unsigned i = 0; i < 5; ++i) device.registers[i] = expected[i];
    device.registers[8] = (5 << 5) | 2;
    assert(mailbox.read(data, size) == Result::Ok && size == 5);
    for (unsigned i = 0; i < 5; ++i) assert(data[i] == expected[i]);
    assert(device.registers[8] & 2); // Reading alone must not acknowledge data.
    assert(mailbox.acknowledge_read() == Result::Ok);
    assert(device.registers[8] == (5 << 5));
    assert(mailbox.acknowledge_read() == Result::Ok); // Safe ACK retry.

    // A GPS-owned busy RX mailbox must not be overwritten or reasserted by TX ACK.
    device.registers[8] = (5 << 5) | (3 << 2) | 3;
    std::size_t sent = 99;
    assert(mailbox.send_fragment(expected.data(), 5, sent) == Result::Busy && sent == 0);
    assert(mailbox.acknowledge_read() == Result::Busy);
    device.registers[8] &= ~1U; // Receiver consumes host data.
    assert(mailbox.acknowledge_read() == Result::Ok);
    assert((device.registers[8] & 3) == 0);

    // An entire SkyTraq frame survives fragmentation into 3-byte mailboxes.
    const std::vector<std::uint8_t> command{
        0xA0, 0xA1, 0x00, 0x04, 0x64, 0x17, 0x05, 0x00, 0x76, 0x0D, 0x0A};
    device.written.clear();
    device.registers[8] = (2 << 5) | 2; // Preserve concurrent GPS->host ready/count.
    std::size_t offset = 0;
    while (offset < command.size()) {
        assert(mailbox.send_fragment(command.data() + offset, command.size() - offset, sent) == Result::Ok);
        assert(sent <= 3 && sent != 0);
        assert((device.registers[8] & 0xE2) == ((2 << 5) | 2));
        offset += sent;
        device.registers[8] &= ~1U;
    }
    assert(device.written == command);

    // Reject invalid live sizes before reading outside the mailbox registers.
    device.registers[8] = (6 << 5) | 2;
    assert(mailbox.read(data, size) == Result::InvalidControl && size == 0);
    device.registers[8] = (4 << 2) | 1;
    assert(mailbox.send_fragment(data, 1, sent) == Result::InvalidControl);
    device.registers[8] = 0xFC; // Undefined sizes are allowed while both ready bits are clear.
    assert(mailbox.read(data, size) == Result::Busy);
    device.io_ok = false;
    assert(mailbox.read(data, size) == Result::IoError);
    assert(mailbox.acknowledge_read() == Result::IoError);
    assert(mailbox.send_fragment(data, 1, sent) == Result::IoError);
    std::puts("SkyTraq I2C mailbox tests passed");
}
