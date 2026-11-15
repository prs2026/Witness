#pragma once

#include <atomic>
#include <cstddef>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Shared flag for the Landed low-power mode (landed_logic.h). The sensor task
// sets it while the flight state is Landed and no USB host is connected; the
// housekeeping tasks (heartbeat, flash capture, flash writer) then stop their
// fast periodic work and block on a task notification with a long timeout, so
// the idle task can enter light sleep. Clearing the flag notifies them so they
// resume immediately. Flight logic never reads this flag.
namespace landed_power {

inline std::atomic<bool> g_low_power{false};
inline TaskHandle_t g_tasks[4]{};
inline std::atomic<std::size_t> g_task_count{0};

inline bool low_power() { return g_low_power.load(std::memory_order_relaxed); }

// Call from the task itself, once, before its loop.
inline void register_current_task()
{
    const std::size_t index = g_task_count.fetch_add(1);
    if (index < sizeof(g_tasks) / sizeof(g_tasks[0])) g_tasks[index] = xTaskGetCurrentTaskHandle();
}

inline void set_low_power(bool on)
{
    if (g_low_power.exchange(on) == on) return;
    const std::size_t count = g_task_count.load();
    for (std::size_t i = 0; i < count && i < sizeof(g_tasks) / sizeof(g_tasks[0]); ++i) {
        if (g_tasks[i] != nullptr) xTaskNotifyGive(g_tasks[i]);
    }
}

}  // namespace landed_power
