#include "skytraq_i2c_mailbox.h"

#include <algorithm>

SkytraqI2cMailbox::Result SkytraqI2cMailbox::control(std::uint8_t &value)
{
    if (!transport_.read_register(transport_.context, kControlRegister, value))
        return Result::IoError;
    // Size fields are undefined until their ready bit is asserted.
    if (((value & kTxReady) && (value >> 5) > kReceiveSize) ||
        ((value & kRxReady) && ((value >> 2) & 7U) > kTransmitSize))
        return Result::InvalidControl;
    return Result::Ok;
}

SkytraqI2cMailbox::Result SkytraqI2cMailbox::read(std::uint8_t *data,
                                               std::size_t &length)
{
    length = 0;
    std::uint8_t status = 0;
    auto result = control(status);
    if (result != Result::Ok) return result;
    if (!(status & kTxReady)) return Result::Busy;
    const std::size_t count = status >> 5;
    for (std::size_t i = 0; i < count; ++i) {
        if (!transport_.read_register(transport_.context,
                                      static_cast<std::uint8_t>(i), data[i]))
            return Result::IoError;
    }
    length = count;
    return Result::Ok;
}

SkytraqI2cMailbox::Result SkytraqI2cMailbox::acknowledge_read()
{
    std::uint8_t status = 0;
    auto result = control(status);
    if (result != Result::Ok) return result;
    // Do not restore an RX-ready bit that GPS might clear concurrently.
    if (status & kRxReady) return Result::Busy;
    if (!(status & kTxReady)) return Result::Ok;
    return transport_.write_register(transport_.context, kControlRegister,
                                     status & ~kTxReady)
               ? Result::Ok : Result::IoError;
}

SkytraqI2cMailbox::Result SkytraqI2cMailbox::send_fragment(
    const std::uint8_t *data, const std::size_t length, std::size_t &sent)
{
    sent = 0;
    if (length == 0) return Result::Ok;
    std::uint8_t status = 0;
    auto result = control(status);
    if (result != Result::Ok) return result;
    if (status & kRxReady) return Result::Busy;
    const auto count = std::min(length, kTransmitSize);
    for (std::size_t i = 0; i < count; ++i) {
        if (!transport_.write_register(transport_.context,
                                       static_cast<std::uint8_t>(5 + i), data[i]))
            return Result::IoError;
    }
    // Refresh GPS->host state before publishing the command fragment.
    result = control(status);
    if (result != Result::Ok) return result;
    const auto next = static_cast<std::uint8_t>(
        (status & ~0x1CU) | (count << 2U) | kRxReady);
    if (!transport_.write_register(transport_.context, kControlRegister, next))
        return Result::IoError;
    sent = count;
    return Result::Ok;
}
