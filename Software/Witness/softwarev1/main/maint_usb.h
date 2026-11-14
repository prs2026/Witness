#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

class FlashLogger;

// USB side of the maintenance image.
namespace maint_usb {

// Wi-Fi mode: installs the USB Serial/JTAG driver and a reader task that
// accepts canonical 10-byte 0x05 command frames (maint::handle_command).
esp_err_t start_serial();
// Writes bytes to the USB serial port if a host is connected (never blocks).
void write(const std::uint8_t *data, std::size_t length);

// USB export mode: waits (up to timeout_ms) for the read-only log store to
// mount, then turns the USB port into a read-only mass-storage disk. The
// serial port is gone from then on (LoRa still works).
esp_err_t start_export(FlashLogger &flash_logger, std::uint32_t timeout_ms);

}  // namespace maint_usb
