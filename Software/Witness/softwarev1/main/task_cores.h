#pragma once

#include "freertos/FreeRTOS.h"

// Core assignment for every Witness application task.
//
// Core 1 (APP CPU) is reserved for the 500 Hz sensor/attitude loop and the
// planned Kalman filter: nothing else is pinned there, and the SPI3 traffic it
// issues uses polling transactions, so it does not wait on interrupts serviced
// by core 0. Core 0 (PRO CPU) runs everything else: USB, NAND logging, GPS
// UART, CAN, radio-command handling, heartbeat, the CPU-load monitor, and the
// ESP-IDF system tasks (esp_timer, IPC) that are pinned to core 0 by default.
namespace task_cores {
inline constexpr BaseType_t kEstimation = 1;  // sensors (+ future Kalman filter)
inline constexpr BaseType_t kSystem = 0;      // everything else
}  // namespace task_cores
