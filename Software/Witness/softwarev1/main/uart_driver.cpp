#include "uart_driver.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "esp_log.h"
#include "hardware_pins.h"
#include "spi_data_forwarder.h"

namespace {

constexpr char kTag[] = "uart2";
constexpr std::array<std::uint8_t, 2> kLoopbackRequest = {0x05, 0x22};

}  // namespace

UartDriver::UartDriver(SpiDataForwarder &serial_forwarder)
    : serial_forwarder_(serial_forwarder)
{
}

UartDriver::~UartDriver()
{
    (void)shutdown();
}

esp_err_t UartDriver::start()
{
    if (initialized_ || task_handle_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t result = initialize();
    if (result != ESP_OK) {
        return result;
    }

    if (xTaskCreate(
            task_entry,
            "uart2",
            kTaskStackSize,
            this,
            kTaskPriority,
            &task_handle_) != pdPASS) {
        task_handle_ = nullptr;
        (void)shutdown();
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t UartDriver::initialize()
{
    uart_config_t config{};
    config.baud_rate = static_cast<int>(kBaudRate);
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.rx_flow_ctrl_thresh = 0;
    config.source_clk = UART_SCLK_DEFAULT;

    esp_err_t result = uart_param_config(kPort, &config);
    if (result != ESP_OK) {
        return result;
    }

    result = uart_set_pin(
        kPort,
        HW_PIN_MCU_UART_TX,
        HW_PIN_MCU_UART_RX,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE);
    if (result != ESP_OK) {
        return result;
    }

    result = uart_driver_install(
        kPort,
        kDriverBufferSize,
        kDriverBufferSize,
        0,
        nullptr,
        0);
    if (result == ESP_OK) {
        initialized_ = true;
    }
    return result;
}

esp_err_t UartDriver::shutdown()
{
    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }

    if (!initialized_) {
        return ESP_OK;
    }

    const esp_err_t result = uart_driver_delete(kPort);
    if (result == ESP_OK) {
        initialized_ = false;
    }
    return result;
}

esp_err_t UartDriver::transmit(
    const std::uint8_t *const data,
    const std::size_t length,
    const TickType_t timeout_ticks)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == nullptr || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const int written = uart_write_bytes(kPort, data, length);
    if (written < 0) {
        return ESP_FAIL;
    }
    if (static_cast<std::size_t>(written) != length) {
        return ESP_ERR_TIMEOUT;
    }
    return timeout_ticks == 0 ? ESP_OK
                              : uart_wait_tx_done(kPort, timeout_ticks);
}

esp_err_t UartDriver::send_loopback_request()
{
    return transmit(
        kLoopbackRequest.data(),
        kLoopbackRequest.size(),
        pdMS_TO_TICKS(100));
}

void UartDriver::task_entry(void *const context)
{
    static_cast<UartDriver *>(context)->run();
}

void UartDriver::run()
{
    ESP_LOGI(
        kTag,
        "ready at %" PRIu32 " baud on TX=%d RX=%d",
        kBaudRate,
        static_cast<int>(HW_PIN_MCU_UART_TX),
        static_cast<int>(HW_PIN_MCU_UART_RX));

    bool waiting_for_loopback_reply = false;
    bool previous_byte_was_05 = false;
    TickType_t loopback_deadline = 0;
    const TickType_t task_delay_ticks =
        std::max<TickType_t>(1, pdMS_TO_TICKS(kTaskTickMs));

    const esp_err_t test_result = send_loopback_request();
    if (test_result == ESP_OK) {
        waiting_for_loopback_reply = true;
        loopback_deadline = xTaskGetTickCount() +
                            pdMS_TO_TICKS(kLoopbackReplyTimeoutMs);
        ESP_LOGI(kTag, "sent loopback request: 05 22");
    } else {
        ESP_LOGE(
            kTag,
            "could not send loopback request: %s",
            esp_err_to_name(test_result));
    }

    std::array<std::uint8_t, kReadBufferSize> receive_buffer{};
    for (;;) {
        const int bytes_read = uart_read_bytes(
            kPort,
            receive_buffer.data(),
            receive_buffer.size(),
            task_delay_ticks);

        if (bytes_read > 0) {
            const std::size_t received = static_cast<std::size_t>(bytes_read);
            const esp_err_t forward_result =
                serial_forwarder_.queue_raw_bytes(
                    receive_buffer.data(), received);
            if (forward_result != ESP_OK) {
                ESP_LOGW(
                    kTag,
                    "could not forward %u UART bytes: %s",
                    static_cast<unsigned>(received),
                    esp_err_to_name(forward_result));
            }

            if (waiting_for_loopback_reply) {
                for (std::size_t index = 0; index < received; ++index) {
                    const std::uint8_t byte = receive_buffer[index];
                    if (previous_byte_was_05 && byte == 0x22) {
                        waiting_for_loopback_reply = false;
                        ESP_LOGI(kTag, "loopback reply received: 05 22");
                        break;
                    }
                    previous_byte_was_05 = byte == 0x05;
                }
            }
        } else if (bytes_read < 0) {
            ESP_LOGW(kTag, "UART receive failed");
        }

        if (waiting_for_loopback_reply &&
            static_cast<std::int32_t>(
                xTaskGetTickCount() - loopback_deadline) >= 0) {
            waiting_for_loopback_reply = false;
            ESP_LOGW(
                kTag,
                "no 05 22 loopback reply within %" PRIu32 " ms",
                kLoopbackReplyTimeoutMs);
        }
    }
}
