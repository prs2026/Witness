#include <cstdint>

#include "battery_voltage.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hardware.h"
#include "load_state.h"
#include "mcp23008.h"
#include "pac1931.h"
#include "sdkconfig.h"
#include "usb_serial_echo.h"

namespace {

constexpr char kLogTag[] = "main";
// Pins driven as outputs: OUT1..3, 5V, blue LED.
constexpr std::uint8_t kExpanderOutputMask =
    IRIS_MCP23008_MASK_OUT1_ENABLE | IRIS_MCP23008_MASK_OUT2_ENABLE |
    IRIS_MCP23008_MASK_OUT3_ENABLE | IRIS_MCP23008_MASK_5V_ENABLE |
    IRIS_MCP23008_MASK_LED_BLUE;

// Bring the expander up (bus + outputs). Retries transient I2C errors and
// never aborts: a failure here must not become a reboot loop that keeps the
// loads off. Returns true once the outputs are configured.
bool bring_up_expander(Mcp23008 &expander, const bool pulse_reset,
                       const std::uint8_t load_mask, const int attempts)
{
    for (int attempt = 0; attempt < attempts; ++attempt) {
        esp_err_t result = ESP_OK;
        if (!expander.bus_ready()) {
            result = expander.initialize(pulse_reset);
        }
        if (result == ESP_OK) {
            result = expander.configure_outputs(kExpanderOutputMask, load_mask);
        }
        if (result == ESP_OK) {
            return true;
        }
        ESP_LOGE(kLogTag, "expander bring-up attempt %d/%d failed: %s",
                 attempt + 1, attempts, esp_err_to_name(result));
        vTaskDelay(1);
    }
    return false;
}

// I3: stack high-water bookkeeping. The minimum free main-task stack seen so
// far is kept in RTC memory so the next boot can report it even though
// application logging is switched off once the USB stream is running.
constexpr std::uint32_t kStackMagic = 0x53544B31U;  // "STK1"
RTC_NOINIT_ATTR std::uint32_t g_stack_magic;
RTC_NOINIT_ATTR std::uint32_t g_stack_min_free;
// Per-minute lines would corrupt the binary USB stream if left on forever, so
// they are printed only for the first minutes after boot (bench measurement);
// the minimum is always kept and reported at the next boot.
constexpr std::int64_t kStackLogPeriodUs = 60000000;
constexpr int kStackLogCount = 10;

}  // namespace

// The command/telemetry object holds the CAN queues, TWAI slots, reassembler
// and the 2 x 512 B receive buffers (several KB). It used to be a local of
// app_main on a 3584 B stack (audit I3); it is static, in .bss.

extern "C" void app_main(void)
{
    static constexpr std::int64_t kBatteryReadIntervalUs =
        static_cast<std::int64_t>(
            IRIS_BATTERY_VOLTAGE_READ_INTERVAL_MS) * 1000;

    // Warm reset (software, panic, watchdog, brownout) with a valid RTC record:
    // restore the commanded loads first. Any other reset: all loads OFF.
    const load_state::BootResult boot = load_state::begin_boot();

    if (g_stack_magic == kStackMagic) {
        ESP_LOGW(kLogTag, "previous run: minimum free main stack %lu B",
                 static_cast<unsigned long>(g_stack_min_free));
    }
    g_stack_magic = kStackMagic;
    g_stack_min_free = 0xFFFFFFFFU;

    static Mcp23008 gpio_expander;
    bool expander_ok = bring_up_expander(
        gpio_expander, !boot.restore, boot.mask, 25);
    if (expander_ok && boot.restore) {
        ESP_LOGW(kLogTag, "LOADS RESTORED after warm reset: mask 0x%02X "
                          "at +%lld ms since app start",
                 boot.mask,
                 static_cast<long long>(esp_timer_get_time() / 1000));
    } else if (!expander_ok) {
        ESP_LOGE(kLogTag, "expander not ready; will keep retrying in the main loop");
    }

    static Pac1931 current_monitor;
    if (gpio_expander.bus_ready()) {
        const esp_err_t current_monitor_result =
            current_monitor.initialize(gpio_expander.bus_handle());
        if (current_monitor_result != ESP_OK) {
            ESP_LOGE(kLogTag, "PAC193x initialization failed: %s",
                     esp_err_to_name(current_monitor_result));
        }
    }

    BatteryVoltage battery_voltage;
    const esp_err_t battery_result = battery_voltage.initialize();
    if (battery_result != ESP_OK) {
        // Non-fatal: reads fail and the battery-monitor error flag is set.
        ESP_LOGE(kLogTag, "battery ADC init failed: %s",
                 esp_err_to_name(battery_result));
    }
    std::int64_t last_battery_read_us = esp_timer_get_time();

    static UsbSerialEcho usb_serial_echo(gpio_expander, current_monitor);
    if (boot.restore) {
        usb_serial_echo.set_restored_load_mask(boot.mask);
    }
    const esp_err_t echo_result = usb_serial_echo.initialize();
    if (echo_result != ESP_OK) {
        // poll() is a no-op if this failed, so loads are still held but
        // commands are not serviced; make that visible rather than looping.
        ESP_LOGE(kLogTag, "command/telemetry init failed: %s",
                 esp_err_to_name(echo_result));
    }

    std::int64_t next_expander_retry_us = 0;
    std::int64_t next_stack_log_us = esp_timer_get_time() + kStackLogPeriodUs;
    int stack_logs_left = kStackLogCount;
    for (;;) {
        const std::int64_t now_us = esp_timer_get_time();
        const std::int64_t time_since_battery_read_us =
            now_us - last_battery_read_us;
        if (time_since_battery_read_us >= kBatteryReadIntervalUs) {
            last_battery_read_us = now_us;
            std::uint32_t reading_mv = 0;
            const bool reading_valid =
                battery_voltage.read_millivolts(reading_mv) == ESP_OK;
            usb_serial_echo.update_battery_voltage(
                reading_mv, reading_valid);
        }

        if (!expander_ok && now_us >= next_expander_retry_us) {
            next_expander_retry_us = now_us + 250000;
            // Never pulse the expander reset here: apply the newest COMMANDED
            // mask (also covers a command that arrived while it was down).
            expander_ok = bring_up_expander(
                gpio_expander, false, load_state::commanded_mask(), 1);
            if (!expander_ok && gpio_expander.bus_ready()) {
                // Still failing on an existing bus: clear a possibly stuck bus.
                (void)gpio_expander.recover_bus();
            }
        }

        usb_serial_echo.poll();
        // One-time erase of the stale NVS load key (no load state is stored in flash).
        load_state::service(now_us);

        if (now_us >= next_stack_log_us) {
            next_stack_log_us = now_us + kStackLogPeriodUs;
            const std::uint32_t free_bytes = static_cast<std::uint32_t>(
                uxTaskGetStackHighWaterMark(nullptr));
            if (free_bytes < g_stack_min_free) {
                g_stack_min_free = free_bytes;
            }
            if (stack_logs_left > 0) {
                --stack_logs_left;
                esp_log_level_set(kLogTag, ESP_LOG_INFO);
                ESP_LOGI(kLogTag, "main stack high-water: %lu B free of %d",
                         static_cast<unsigned long>(free_bytes),
                         CONFIG_ESP_MAIN_TASK_STACK_SIZE);
                esp_log_level_set(kLogTag, ESP_LOG_NONE);
            }
        }
        vTaskDelay(1);
    }
}
