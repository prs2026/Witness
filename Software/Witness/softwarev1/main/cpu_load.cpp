#include "cpu_load.h"

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstring>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/idf_additions.h"
#include "wifi_ota.h"

// Repeated every report (not only at boot): after a reset the USB console
// reconnects late, so a one-off boot line is usually lost. BROWNOUT here
// means the supply sagged, e.g. under the RA-01SH-P transmit current.
static const char *reset_reason_name(const esp_reset_reason_t reason)
{
    switch (reason) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    case ESP_RST_USB: return "USB";
    case ESP_RST_JTAG: return "JTAG";
    default: return "UNKNOWN";
    }
}

namespace {
constexpr char kLogTag[] = "cpu";
}  // namespace

esp_err_t CpuLoadMonitor::start()
{
    if (task_ != nullptr) return ESP_ERR_INVALID_STATE;
    return xTaskCreatePinnedToCore(task_entry, "cpu_load", kTaskStackSize, this,
                                   kTaskPriority, &task_, kCore) == pdPASS
               ? ESP_OK : ESP_ERR_NO_MEM;
}

void CpuLoadMonitor::task_entry(void *context)
{
    static_cast<CpuLoadMonitor *>(context)->run();
}

void CpuLoadMonitor::run()
{
    report();  // Prime the previous snapshot; the first real report follows.
    TickType_t next = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(kReportIntervalMs));
        report();
    }
}

void CpuLoadMonitor::report()
{
    configRUN_TIME_COUNTER_TYPE total = 0;
    const UBaseType_t count = uxTaskGetSystemState(status_, kMaxTasks, &total);
    if (count == 0) {
        ESP_LOGW(kLogTag, "more than %u tasks; load not measured",
                 static_cast<unsigned>(kMaxTasks));
        return;
    }
    const configRUN_TIME_COUNTER_TYPE elapsed = total - previous_total_;
    const bool primed = previous_total_ != 0 && elapsed > 0;

    // Delta runtime per task since the last report (new tasks count from 0).
    struct Share {
        const char *name;
        double percent;
        BaseType_t core;
        configSTACK_DEPTH_TYPE stack_free;
    };
    Share shares[kMaxTasks]{};
    double idle_percent[2] = {0.0, 0.0};
    for (UBaseType_t i = 0; i < count; ++i) {
        configRUN_TIME_COUNTER_TYPE before = 0;
        for (std::size_t j = 0; j < previous_count_; ++j) {
            if (previous_[j].handle == status_[i].xHandle) {
                before = previous_[j].runtime;
                break;
            }
        }
        const configRUN_TIME_COUNTER_TYPE delta =
            status_[i].ulRunTimeCounter >= before ? status_[i].ulRunTimeCounter - before : 0;
        const double percent = primed ? 100.0 * static_cast<double>(delta) /
                                            static_cast<double>(elapsed)
                                      : 0.0;
        // Pinned core of the task (tskNO_AFFINITY when unpinned).
        const BaseType_t core = xTaskGetCoreID(status_[i].xHandle);
        shares[i] = {status_[i].pcTaskName, percent, core, status_[i].usStackHighWaterMark};
        if (std::strncmp(status_[i].pcTaskName, "IDLE", 4) == 0 && core >= 0 && core < 2) {
            idle_percent[core] = percent;
        }
    }

    previous_count_ = count;
    for (UBaseType_t i = 0; i < count; ++i) {
        previous_[i] = {status_[i].xHandle, status_[i].ulRunTimeCounter};
    }
    previous_total_ = total;
    if (!primed) return;

    std::sort(shares, shares + count, [](const Share &a, const Share &b) {
        return a.percent > b.percent;
    });
    ESP_LOGI(kLogTag,
             "core0 %.1f%% busy, core1 %.1f%% busy; heap free %u (min %u), internal free %u; last reset %s; wifi %s",
             100.0 - idle_percent[0], 100.0 - idle_percent[1],
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_DEFAULT)),
             static_cast<unsigned>(heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT)),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
             reset_reason_name(esp_reset_reason()), wifi_ota_state_text());
    // One short line per task: the USB stream interleaves GPS bytes, so a
    // long single line is easily split.
    std::size_t listed = 0;
    for (UBaseType_t i = 0; i < count && listed < kTopTasks; ++i) {
        if (std::strncmp(shares[i].name, "IDLE", 4) == 0) continue;
        const int core = shares[i].core >= 0 && shares[i].core < 2 ? static_cast<int>(shares[i].core) : -1;
        ESP_LOGI(kLogTag, "task %-15s %5.1f%% core %d, stack free %u B", shares[i].name,
                 shares[i].percent, core, static_cast<unsigned>(shares[i].stack_free));
        ++listed;
    }
}
