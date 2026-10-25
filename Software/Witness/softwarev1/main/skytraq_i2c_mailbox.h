#pragma once

#include <cstddef>
#include <cstdint>

// SkyTraq AN0031 v3 mailbox protocol. The example's 0x78/0x79 wire
// addresses correspond to 7-bit slave address 0x3C. PX1120S firmware
// support must be checked on hardware; AN0031 explicitly targets Venus 8.
class SkytraqI2cMailbox final {
public:
    static constexpr std::uint8_t kAddress = 0x3C;
    static constexpr std::uint8_t kControlRegister = 8;
    static constexpr std::size_t kReceiveSize = 5;
    static constexpr std::size_t kTransmitSize = 3;
    enum class Result { Ok, Busy, IoError, InvalidControl };
    struct Transport {
        void *context;
        bool (*read_register)(void *, std::uint8_t, std::uint8_t &);
        bool (*write_register)(void *, std::uint8_t, std::uint8_t);
    };
    explicit SkytraqI2cMailbox(Transport transport) : transport_(transport) {}
    Result read(std::uint8_t *data, std::size_t &length);
    // Acknowledge only after the caller has buffered the received bytes.
    Result acknowledge_read();
    Result send_fragment(const std::uint8_t *data, std::size_t length,
                         std::size_t &sent);

private:
    Result control(std::uint8_t &value);
    static constexpr std::uint8_t kTxReady = 0x02; // GPS -> host
    static constexpr std::uint8_t kRxReady = 0x01; // host -> GPS
    Transport transport_;
};
