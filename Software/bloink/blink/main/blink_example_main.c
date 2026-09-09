/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO 2
#define ECHO_BUFFER_SIZE 256
#define UART_RX_BUFFER_SIZE 1024

static uint8_t s_led_state = 0;

static void serial_echo_task(void *arg)
{
    const uart_port_t uart_port = (uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM;
    uint8_t data[ECHO_BUFFER_SIZE];

    while (1) {
        const int bytes_read = uart_read_bytes(uart_port, data, sizeof(data), portMAX_DELAY);
        if (bytes_read > 0) {
            uart_write_bytes(uart_port, data, bytes_read);
        }
    }
}

static void configure_serial_echo(void)
{
    const uart_port_t uart_port = (uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM;
    const uart_config_t uart_config = {
        .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(uart_port, UART_RX_BUFFER_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(xTaskCreate(serial_echo_task, "serial_echo", 2048, NULL, 10, NULL) == pdPASS
                        ? ESP_OK
                        : ESP_ERR_NO_MEM);
    ESP_LOGI(TAG, "Serial echo ready on UART%d at %d baud", uart_port,
             CONFIG_ESP_CONSOLE_UART_BAUDRATE);
}

#ifdef CONFIG_BLINK_LED_STRIP


#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();
    configure_serial_echo();

    while (1) {
        ESP_LOGI(TAG, "Turning the LED %s! NEW PROGRAm", s_led_state == true ? "ON" : "OFF");
        blink_led();
        /* Toggle the LED state */
        s_led_state = !s_led_state;
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
