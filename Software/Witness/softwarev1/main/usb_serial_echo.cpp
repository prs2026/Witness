#include "usb_serial_echo.h"

#include <cstddef>
#include <cstdint>

#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "sdkconfig.h"

esp_err_t UsbSerialEcho::start()
{
    if (task_handle_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    usb_serial_jtag_driver_config_t usb_config{};
    usb_config.tx_buffer_size = kBufferSize;
    usb_config.rx_buffer_size = kBufferSize;

    const esp_err_t result = usb_serial_jtag_driver_install(&usb_config);
    if (result != ESP_OK) {
        return result;
    }

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    // The USB console initially uses polling I/O. Route it through the
    // interrupt-driven driver shared with the echo task.
    usb_serial_jtag_vfs_use_driver();
#endif

    const BaseType_t task_created = xTaskCreate(
        task_entry,
        "usb_serial_echo",
        kTaskStackSize,
        this,
        kTaskPriority,
        &task_handle_);

    if (task_created != pdPASS) {
        task_handle_ = nullptr;
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
        usb_serial_jtag_vfs_use_nonblocking();
#endif
        usb_serial_jtag_driver_uninstall();
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void UsbSerialEcho::task_entry(void *context)
{
    static_cast<UsbSerialEcho *>(context)->run();
}

void UsbSerialEcho::run()
{
    std::uint8_t buffer[kBufferSize];

    for (;;) {
        const int bytes_read = usb_serial_jtag_read_bytes(
            buffer, sizeof(buffer), portMAX_DELAY);

        if (bytes_read > 0) {
            std::size_t bytes_echoed = 0;
            const std::size_t bytes_to_echo =
                static_cast<std::size_t>(bytes_read);

            // A USB write can be partial. Keep writing from the same buffer
            // until every byte has been echoed, including embedded zeroes.
            while (bytes_echoed < bytes_to_echo) {
                const int bytes_written = usb_serial_jtag_write_bytes(
                    buffer + bytes_echoed,
                    bytes_to_echo - bytes_echoed,
                    portMAX_DELAY);

                if (bytes_written > 0) {
                    bytes_echoed += static_cast<std::size_t>(bytes_written);
                } else {
                    taskYIELD();
                }
            }
        }
    }
}
