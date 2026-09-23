#include "command_bridge.h"

#include "comms.h"
#include "freertos/FreeRTOS.h"
#include "uart_driver.h"

CommandBridge::CommandBridge(UartDriver &uart)
    : uart_(uart)
{
}

esp_err_t CommandBridge::submit_usb_command(
    const std::uint8_t *const data,
    const std::size_t length)
{
    return forward_command(data, length);
}

esp_err_t CommandBridge::submit_radio_command(
    const std::uint8_t *const data,
    const std::size_t length)
{
    return forward_command(data, length);
}

esp_err_t CommandBridge::forward_command(
    const std::uint8_t *const data,
    const std::size_t length)
{
    if (data == nullptr || length == 0 || data[0] != IRIS_PACKET_ID_COMMAND) {
        return ESP_ERR_INVALID_ARG;
    }

    return uart_.transmit(data, length, pdMS_TO_TICKS(100));
}
