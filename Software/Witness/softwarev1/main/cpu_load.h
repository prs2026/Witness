#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Periodic CPU-load report from FreeRTOS run-time stats (esp_timer clock).
//
// Every kReportIntervalMs it logs, at INFO:
//   cpu: core0 B0% busy, core1 B1% busy; heap free F (min M); top: name P%[cN] ...
// Per-core busy is 100% minus that core's idle task share of the interval;
// per-task % is the share of one core's time. [cN] is the task's pinned core
// ("-" when unpinned).
class CpuLoadMonitor final {
public:
    static constexpr std::uint32_t kReportIntervalMs = 10'000;
    static constexpr UBaseType_t kTaskPriority = 1;
    static constexpr std::uint32_t kTaskStackSize = 4096;
    static constexpr BaseType_t kCore = 0;
    static constexpr std::size_t kMaxTasks = 32;
    static constexpr std::size_t kTopTasks = 8;

    esp_err_t start();

private:
    static void task_entry(void *context);
    void run();
    void report();

    struct Previous {
        TaskHandle_t handle = nullptr;
        configRUN_TIME_COUNTER_TYPE runtime = 0;
    };

    TaskHandle_t task_ = nullptr;
    TaskStatus_t status_[kMaxTasks]{};
    Previous previous_[kMaxTasks]{};
    std::size_t previous_count_ = 0;
    configRUN_TIME_COUNTER_TYPE previous_total_ = 0;
};
