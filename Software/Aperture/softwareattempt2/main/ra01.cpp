#include "ra01.h"

#include <algorithm>
#include <array>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hardware.h"

namespace {

constexpr const char *kLogTag = "ra01";

constexpr std::uint8_t kSetStandby = 0x80;
constexpr std::uint8_t kSetPacketType = 0x8A;
constexpr std::uint8_t kSetRfFrequency = 0x86;
constexpr std::uint8_t kSetPaConfig = 0x95;
constexpr std::uint8_t kSetDio2AsRfSwitchCtrl = 0x9D;
constexpr std::uint8_t kSetModulationParams = 0x8B;
constexpr std::uint8_t kSetPacketParams = 0x8C;
constexpr std::uint8_t kSetTxParams = 0x8E;
constexpr std::uint8_t kSetDioIrqParams = 0x08;
constexpr std::uint8_t kSetBufferBaseAddress = 0x8F;
constexpr std::uint8_t kWriteBuffer = 0x0E;
constexpr std::uint8_t kSetRx = 0x82;
constexpr std::uint8_t kSetTx = 0x83;
constexpr std::uint8_t kGetIrqStatus = 0x12;
constexpr std::uint8_t kClearIrqStatus = 0x02;
constexpr std::uint8_t kGetRxBufferStatus = 0x13;
constexpr std::uint8_t kReadBuffer = 0x1E;

constexpr std::uint16_t kIrqRxDone = 0x0002;
constexpr std::uint16_t kIrqTxDone = 0x0001;
constexpr std::uint16_t kIrqCrcError = 0x0040;
static_assert(IRIS_RADIO_TX_POWER_DBM >= -9 && IRIS_RADIO_TX_POWER_DBM <= 22,
              "IRIS_RADIO_TX_POWER_DBM must be between -9 and +22 dBm");

void store_u32_be(std::uint8_t *data, const std::uint32_t value)
{
    data[0] = static_cast<std::uint8_t>(value >> 24U);
    data[1] = static_cast<std::uint8_t>(value >> 16U);
    data[2] = static_cast<std::uint8_t>(value >> 8U);
    data[3] = static_cast<std::uint8_t>(value);
}

}  // namespace

Ra01::~Ra01()
{
    if (spi_device_ != nullptr) {
        (void)spi_bus_remove_device(spi_device_);
        (void)spi_bus_free(SPI2_HOST);
    }
}

esp_err_t Ra01::initialize()
{
    if (initialized_) return ESP_ERR_INVALID_STATE;

    gpio_config_t outputs{};
    outputs.pin_bit_mask = (1ULL << IRIS_PIN_RADIO_RESET) |
                           (1ULL << IRIS_PIN_RADIO_RF_ENABLE);
    outputs.mode = GPIO_MODE_OUTPUT;
    outputs.intr_type = GPIO_INTR_DISABLE;
    esp_err_t result = gpio_config(&outputs);
    if (result != ESP_OK) return result;

    gpio_config_t inputs{};
    inputs.pin_bit_mask = (1ULL << IRIS_PIN_RADIO_BUSY) |
                          (1ULL << IRIS_PIN_RADIO_DIO1);
#if IRIS_RADIO_DIO2_MCU_INPUT
    inputs.pin_bit_mask |= (1ULL << IRIS_PIN_RADIO_DIO2);
#endif
    inputs.mode = GPIO_MODE_INPUT;
    inputs.intr_type = GPIO_INTR_DISABLE;
    result = gpio_config(&inputs);
    if (result != ESP_OK) return result;

    ESP_LOGI(kLogTag, "Pins: RESET=%d BUSY=%d RF_EN=%d CS=%d",
             IRIS_PIN_RADIO_RESET, IRIS_PIN_RADIO_BUSY,
             IRIS_PIN_RADIO_RF_ENABLE, IRIS_PIN_RADIO_CS);

    // Keep the FEM disabled while the SX1262 is held in reset. This avoids
    // switching the RF front end before the transceiver is ready.
    gpio_set_level(IRIS_PIN_RADIO_RF_ENABLE, 0);
    gpio_set_level(IRIS_PIN_RADIO_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(IRIS_PIN_RADIO_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(IRIS_PIN_RADIO_RF_ENABLE, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(kLogTag, "After reset: RESET=%d RF_EN=%d BUSY=%d",
             gpio_get_level(IRIS_PIN_RADIO_RESET),
             gpio_get_level(IRIS_PIN_RADIO_RF_ENABLE),
             gpio_get_level(IRIS_PIN_RADIO_BUSY));

    spi_bus_config_t bus{};
    bus.mosi_io_num = IRIS_PIN_RADIO_SPI_MOSI;
    bus.miso_io_num = IRIS_PIN_RADIO_SPI_MISO;
    bus.sclk_io_num = IRIS_PIN_RADIO_SPI_SCK;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    result = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED);
    if (result != ESP_OK) return result;

    spi_device_interface_config_t device{};
    device.clock_speed_hz = kSpiClockHz;
    device.mode = 0;
    device.spics_io_num = IRIS_PIN_RADIO_CS;
    device.queue_size = 1;
    result = spi_bus_add_device(SPI2_HOST, &device, &spi_device_);
    if (result != ESP_OK) {
        (void)spi_bus_free(SPI2_HOST);
        return result;
    }

    const std::uint8_t standby[] = {0x00U};
    const std::uint8_t packet_type[] = {0x01U};
    result = write_command(kSetStandby, standby, sizeof(standby));
    if (result != ESP_OK) return result;
    result = write_command(kSetPacketType, packet_type, sizeof(packet_type));
    if (result != ESP_OK) return result;

    const std::uint32_t frequency_word = static_cast<std::uint32_t>(
        (static_cast<std::uint64_t>(IRIS_RADIO_FREQUENCY_HZ) << 25U) /
        32000000ULL);
    std::array<std::uint8_t, 4> frequency{};
    store_u32_be(frequency.data(), frequency_word);
    result = write_command(kSetRfFrequency, frequency.data(), frequency.size());
    if (result != ESP_OK) return result;

    // SX1262 high-power PA configuration. The RA-01SH-P front end is driven
    // with the requested 2 dBm from SetTxParams below; this configuration is
    // still required to select the SX1262 PA path.
    const std::uint8_t pa_config[] = {0x04U, 0x07U, 0x00U, 0x01U};
    result = write_command(kSetPaConfig, pa_config, sizeof(pa_config));
    if (result != ESP_OK) return result;

    // Let the SX1262 drive DIO2 automatically during RX/TX for an external
    // RF switch. The inversion setting selects the opposite control state;
    // the SX1262 itself does not provide a separate polarity parameter.
    constexpr std::uint8_t dio2_switch_enable =
        static_cast<std::uint8_t>(IRIS_RADIO_DIO2_RF_SWITCH_ENABLE != 0);
    constexpr std::uint8_t dio2_switch_invert =
        static_cast<std::uint8_t>(IRIS_RADIO_DIO2_RF_SWITCH_INVERT != 0);
    const std::uint8_t dio2_switch[] = {
        static_cast<std::uint8_t>(dio2_switch_enable ^ dio2_switch_invert)};
    result = write_command(kSetDio2AsRfSwitchCtrl, dio2_switch,
                           sizeof(dio2_switch));
    if (result != ESP_OK) return result;

    const std::uint8_t modulation[] = {
        static_cast<std::uint8_t>(IRIS_RADIO_SPREADING_FACTOR),
        0x04U,  // 125 kHz bandwidth
        static_cast<std::uint8_t>(IRIS_RADIO_CODING_RATE), 0x00U};
    result = write_command(kSetModulationParams, modulation,
                           sizeof(modulation));
    if (result != ESP_OK) return result;

    const std::uint8_t packet_params[] = {
        static_cast<std::uint8_t>(IRIS_RADIO_PREAMBLE_LENGTH >> 8U),
        static_cast<std::uint8_t>(IRIS_RADIO_PREAMBLE_LENGTH), 0x00U, 0xFFU,
        0x01U, 0x00U};  // explicit header, CRC on, normal IQ
    result = write_command(kSetPacketParams, packet_params,
                           sizeof(packet_params));
    if (result != ESP_OK) return result;

    const std::uint16_t enabled_irqs = kIrqRxDone | kIrqTxDone | kIrqCrcError;
    const std::uint8_t irq_params[] = {
        static_cast<std::uint8_t>(enabled_irqs >> 8U),
        static_cast<std::uint8_t>(enabled_irqs),
        static_cast<std::uint8_t>(enabled_irqs >> 8U),
        static_cast<std::uint8_t>(enabled_irqs),
        0x00U, 0x00U, 0x00U, 0x00U};
    result = write_command(kSetDioIrqParams, irq_params, sizeof(irq_params));
    if (result != ESP_OK) return result;

    const std::uint8_t buffer_base[] = {0x00U, 0x00U};
    result = write_command(kSetBufferBaseAddress, buffer_base,
                           sizeof(buffer_base));
    if (result != ESP_OK) return result;

    const std::uint8_t clear_irq[] = {0xFFU, 0xFFU};
    result = write_command(kClearIrqStatus, clear_irq, sizeof(clear_irq));
    if (result != ESP_OK) return result;

    const std::uint8_t continuous_rx[] = {0xFFU, 0xFFU, 0xFFU};
    result = write_command(kSetRx, continuous_rx, sizeof(continuous_rx));
    if (result == ESP_OK) initialized_ = true;
    return result;
}

esp_err_t Ra01::transmit(const std::uint8_t *data, const std::size_t length)
{
    if (!initialized_ || data == nullptr || length == 0U || length > 255U) {
        return ESP_ERR_INVALID_ARG;
    }

    const std::uint8_t standby[] = {0x00U};
    esp_err_t result = write_command(kSetStandby, standby, sizeof(standby));
    if (result != ESP_OK) return result;

    // The SX1262 power byte is a signed dBm value encoded as uint8_t.
    const std::uint8_t tx_params[] = {
        static_cast<std::uint8_t>(static_cast<std::int8_t>(IRIS_RADIO_TX_POWER_DBM)),
        0x04U};
    result = write_command(kSetTxParams, tx_params, sizeof(tx_params));
    if (result != ESP_OK) return result;

    const std::uint8_t packet_params[] = {
        static_cast<std::uint8_t>(IRIS_RADIO_PREAMBLE_LENGTH >> 8U),
        static_cast<std::uint8_t>(IRIS_RADIO_PREAMBLE_LENGTH), 0x00U,
        static_cast<std::uint8_t>(length), 0x01U, 0x00U};
    result = write_command(kSetPacketParams, packet_params,
                           sizeof(packet_params));
    if (result != ESP_OK) return result;

    result = write_buffer(data, length);
    if (result != ESP_OK) return result;

    const std::uint8_t clear_irq[] = {0xFFU, 0xFFU};
    result = write_command(kClearIrqStatus, clear_irq, sizeof(clear_irq));
    if (result != ESP_OK) return result;

    // SetTx timeout units are 15.625 us. 0x00FA00 is exactly 1 second.
    const std::uint8_t tx_timeout[] = {0x00U, 0xFAU, 0x00U};
    result = write_command(kSetTx, tx_timeout, sizeof(tx_timeout));
    if (result != ESP_OK) return result;

    const TickType_t deadline = xTaskGetTickCount() +
                                pdMS_TO_TICKS(kTransmitTimeoutMs);
    for (;;) {
        std::uint8_t irq_data[2]{};
        result = read_command(kGetIrqStatus, irq_data, sizeof(irq_data));
        if (result != ESP_OK) break;
        const std::uint16_t irq = static_cast<std::uint16_t>(
            (static_cast<std::uint16_t>(irq_data[0]) << 8U) | irq_data[1]);
        if ((irq & kIrqTxDone) != 0U) {
            result = ESP_OK;
            break;
        }
        if (xTaskGetTickCount() >= deadline) {
            result = ESP_ERR_TIMEOUT;
            break;
        }
        vTaskDelay(1);
    }

    (void)write_command(kClearIrqStatus, clear_irq, sizeof(clear_irq));
    const std::uint8_t continuous_rx[] = {0xFFU, 0xFFU, 0xFFU};
    const esp_err_t rx_result = write_command(kSetRx, continuous_rx,
                                               sizeof(continuous_rx));
    return result == ESP_OK ? rx_result : result;
}

esp_err_t Ra01::receive(std::uint8_t *data, const std::size_t capacity,
                        std::size_t &length)
{
    length = 0;
    if (!initialized_ || data == nullptr) return ESP_ERR_INVALID_STATE;

    std::uint8_t irq_data[2]{};
    esp_err_t result = read_command(kGetIrqStatus, irq_data, sizeof(irq_data));
    if (result != ESP_OK) return result;
    const std::uint16_t irq = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(irq_data[0]) << 8U) | irq_data[1]);

    if ((irq & kIrqCrcError) != 0U) {
        const std::uint8_t clear[] = {0x00U, 0xFFU};
        (void)write_command(kClearIrqStatus, clear, sizeof(clear));
        return ESP_ERR_INVALID_CRC;
    }
    if ((irq & kIrqRxDone) == 0U) return ESP_ERR_NOT_FOUND;

    std::uint8_t status[2]{};
    result = read_command(kGetRxBufferStatus, status, sizeof(status));
    if (result != ESP_OK) return result;
    const std::size_t packet_length = status[0];
    if (packet_length > capacity) {
        const std::uint8_t clear[] = {0x00U, 0xFFU};
        (void)write_command(kClearIrqStatus, clear, sizeof(clear));
        return ESP_ERR_INVALID_SIZE;
    }

    result = read_buffer(status[1], data, packet_length);
    const std::uint8_t clear[] = {0x00U, 0xFFU};
    (void)write_command(kClearIrqStatus, clear, sizeof(clear));
    if (result == ESP_OK) length = packet_length;
    return result;
}

esp_err_t Ra01::wait_until_ready()
{
    const TickType_t deadline = xTaskGetTickCount() +
                                pdMS_TO_TICKS(kBusyTimeoutMs);
    while (gpio_get_level(IRIS_PIN_RADIO_BUSY) != 0) {
        if (xTaskGetTickCount() >= deadline) {
            ESP_LOGE(kLogTag, "BUSY timeout: GPIO%d remains high",
                     IRIS_PIN_RADIO_BUSY);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }
    return ESP_OK;
}

esp_err_t Ra01::write_command(const std::uint8_t command,
                              const std::uint8_t *data,
                              const std::size_t length)
{
    esp_err_t result = wait_until_ready();
    if (result != ESP_OK) return result;
    std::array<std::uint8_t, 258> buffer{};
    if (length + 1U > buffer.size()) return ESP_ERR_INVALID_SIZE;
    buffer[0] = command;
    std::copy(data, data + length, buffer.begin() + 1);
    spi_transaction_t transaction{};
    transaction.length = (length + 1U) * 8U;
    transaction.tx_buffer = buffer.data();
    return spi_device_transmit(spi_device_, &transaction);
}

esp_err_t Ra01::write_buffer(const std::uint8_t *data, const std::size_t length)
{
    esp_err_t result = wait_until_ready();
    if (result != ESP_OK) return result;
    std::array<std::uint8_t, 257> buffer{};
    if (length + 2U > buffer.size()) return ESP_ERR_INVALID_SIZE;
    buffer[0] = kWriteBuffer;
    buffer[1] = 0x00U;
    std::copy(data, data + length, buffer.begin() + 2);
    spi_transaction_t transaction{};
    transaction.length = (length + 2U) * 8U;
    transaction.tx_buffer = buffer.data();
    return spi_device_transmit(spi_device_, &transaction);
}

esp_err_t Ra01::read_command(const std::uint8_t command, std::uint8_t *data,
                             const std::size_t length)
{
    esp_err_t result = wait_until_ready();
    if (result != ESP_OK) return result;
    std::array<std::uint8_t, 258> tx{};
    std::array<std::uint8_t, 258> rx{};
    if (length + 2U > tx.size()) return ESP_ERR_INVALID_SIZE;
    tx[0] = command;
    spi_transaction_t transaction{};
    transaction.length = (length + 2U) * 8U;
    transaction.rxlength = transaction.length;
    transaction.tx_buffer = tx.data();
    transaction.rx_buffer = rx.data();
    result = spi_device_transmit(spi_device_, &transaction);
    if (result == ESP_OK) std::copy(rx.begin() + 2,
                                    rx.begin() + 2 + length, data);
    return result;
}

esp_err_t Ra01::read_buffer(const std::uint8_t offset, std::uint8_t *data,
                            const std::size_t length)
{
    // Leave margin below the ESP32-S3 host's 64-byte no-DMA transfer limit.
    // ReadBuffer needs three command bytes, so use 60-byte chunks.
    constexpr std::size_t kMaxPayloadPerTransfer = 60U;
    std::size_t remaining = length;
    std::size_t copied = 0;

    while (remaining != 0U) {
        const std::size_t chunk =
            std::min(remaining, kMaxPayloadPerTransfer);
        esp_err_t result = wait_until_ready();
        if (result != ESP_OK) return result;

        std::array<std::uint8_t, 64> tx{};
        std::array<std::uint8_t, 64> rx{};
        tx[0] = kReadBuffer;
        tx[1] = static_cast<std::uint8_t>(offset + copied);

        spi_transaction_t transaction{};
        transaction.length = (chunk + 3U) * 8U;
        transaction.rxlength = transaction.length;
        transaction.tx_buffer = tx.data();
        transaction.rx_buffer = rx.data();
        result = spi_device_transmit(spi_device_, &transaction);
        if (result != ESP_OK) return result;

        std::copy(rx.begin() + 3, rx.begin() + 3 + chunk,
                  data + copied);
        copied += chunk;
        remaining -= chunk;
    }
    return ESP_OK;
}
