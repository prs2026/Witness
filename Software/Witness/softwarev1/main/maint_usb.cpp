#include "maint_usb.h"

#include <cstring>

#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_log.h"
#include "flash_logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "maint_app.h"
#include "task_cores.h"
#include "usb_mass_storage.h"

namespace {
constexpr char kLogTag[] = "maint_usb";
bool g_serial = false;
UsbMassStorage *g_storage = nullptr;

void reader_task(void *)
{
    // Sliding 10-byte window: a canonical command is 05 <6 data bytes> <op hi>
    // <op lo> 0A. Anything else (including text) is ignored.
    constexpr std::size_t kFrame = IRIS_PACKET_COMMAND_FRAME_LENGTH;
    std::uint8_t window[kFrame]{};
    std::uint8_t buffer[64];
    for (;;) {
        const int n = usb_serial_jtag_read_bytes(buffer, sizeof(buffer), pdMS_TO_TICKS(50));
        for (int i = 0; i < n; ++i) {
            std::memmove(window, window + 1, kFrame - 1);
            window[kFrame - 1] = buffer[i];
            if (window[0] == IRIS_PACKET_ID_COMMAND && window[kFrame - 1] == IRIS_PACKET_EOF_VALUE) {
                maint::handle_command(window, kFrame, false);
                std::memset(window, 0, sizeof(window));
            }
        }
    }
}
}  // namespace

namespace maint_usb {

esp_err_t start_serial()
{
    usb_serial_jtag_driver_config_t config{};
    config.tx_buffer_size = 512;
    config.rx_buffer_size = 512;
    const esp_err_t result = usb_serial_jtag_driver_install(&config);
    if (result != ESP_OK) return result;
    g_serial = true;
    return xTaskCreatePinnedToCore(reader_task, "maint_usb", 3072, nullptr, 3, nullptr, task_cores::kSystem) ==
                   pdPASS
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}

void write(const std::uint8_t *data, const std::size_t length)
{
    if (!g_serial || !usb_serial_jtag_is_connected()) return;
    (void)usb_serial_jtag_write_bytes(data, length, 0);
}

esp_err_t start_export(FlashLogger &flash_logger, const std::uint32_t timeout_ms)
{
    // Waits for the read-only export table (the one-time NAND scan takes seconds).
    esp_err_t result = flash_logger.prepare_mass_storage(pdMS_TO_TICKS(timeout_ms));
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "log store not mounted: %s", esp_err_to_name(result));
        return result;
    }
    ESP_LOGW(kLogTag, "starting the USB log disk; reset or LoRa Wi-Fi-off returns to flight");
    flash_logger.disable_console_output();
    // Console output goes through the USB Serial/JTAG FIFO, which stops
    // draining once TinyUSB owns the PHY: make it non-blocking first.
    usb_serial_jtag_vfs_use_nonblocking();
    vTaskDelay(pdMS_TO_TICKS(250));
    static UsbMassStorage storage(flash_logger);
    g_storage = &storage;
    result = storage.start();
    if (result != ESP_OK) g_storage = nullptr;
    return result;
}

}  // namespace maint_usb
