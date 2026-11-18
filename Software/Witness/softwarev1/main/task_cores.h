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

#include "esp_task_wdt.h"

// Task-watchdog supervision (flight image only; sdkconfig.flight sets the
// timeout and CONFIG_ESP_TASK_WDT_PANIC=y). A subscribed task that does not
// call feed() within the timeout reboots the MCU; the reset reason
// (ESP_RST_TASK_WDT) is a Warm class in flight_state_store.cpp, so an in-flight
// state is resumed. Usage inside a task: subscribe() once at the top, feed()
// at least once per pass of the main loop, and unsubscribe() before a task
// ends or before an operation that legitimately takes longer than the timeout
// (re-subscribe afterwards). All three are cheap and safe to call when the
// watchdog is not initialised (they just return an error that is ignored).
namespace task_supervision {
inline void subscribe() { (void)esp_task_wdt_add(nullptr); }
inline void feed() { (void)esp_task_wdt_reset(); }
inline void unsubscribe() { (void)esp_task_wdt_delete(nullptr); }
}  // namespace task_supervision
