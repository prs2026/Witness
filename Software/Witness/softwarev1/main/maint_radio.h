#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

// LoRa side of the maintenance image: the RA-01 radio on SPI3, command
// reception, ack transmission and the 0x08 beacon (also written to USB).
namespace maint_radio {

// Initialises the SPI3 bus and the radio and starts the task. dbm 127 = boot
// default. The beacon (USB) runs even if the radio is missing.
esp_err_t start(int dbm, bool front_end);
bool ready();
// Queues a complete frame (ID .. EOF) for transmission (acks); false if the
// radio is not ready or the queue is full.
bool queue_frame(const std::uint8_t *frame, std::size_t length);
// Applies IRIS_COMMAND_RADIO_POWER arguments; returns false if out of range.
bool set_power(int dbm, std::uint8_t front_end_code);
int tx_power_dbm();
bool front_end_on();

}  // namespace maint_radio
