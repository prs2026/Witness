#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

class UartDriver;

// Common command egress for commands received by USB now and by the radio in
// the future. Commands are kept byte-for-byte intact when sent over UART2.
class CommandBridge final {
public:
    explicit CommandBridge(UartDriver &uart);

    CommandBridge(const CommandBridge &) = delete;
    CommandBridge &operator=(const CommandBridge &) = delete;

    esp_err_t submit_usb_command(
        const std::uint8_t *data,
        std::size_t length);

    // Radio receive scaffold. Future radio polling code should pass each
    // complete received command packet here. Radio polling is intentionally
    // not implemented yet.
    esp_err_t submit_radio_command(
        const std::uint8_t *data,
        std::size_t length);

private:
    esp_err_t forward_command(
        const std::uint8_t *data,
        std::size_t length);

    UartDriver &uart_;
};
