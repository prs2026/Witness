#include "mcp23008.h"

#include <cstdint>

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "hardware.h"

esp_err_t Mcp23008::initialize()
{
    if (bus_handle_ != nullptr || device_handle_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    gpio_config_t reset_config{};
    reset_config.pin_bit_mask = 1ULL << IRIS_PIN_MCP23008_RESET;
    reset_config.mode = GPIO_MODE_OUTPUT;
    reset_config.pull_up_en = GPIO_PULLUP_DISABLE;
    reset_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    reset_config.intr_type = GPIO_INTR_DISABLE;

    esp_err_t result = gpio_config(&reset_config);
    if (result != ESP_OK) {
        return result;
    }

    // The MCP23008 requires RESET low for at least 1 us. Resetting also puts
    // every port in input mode and clears the output latch.
    result = gpio_set_level(IRIS_PIN_MCP23008_RESET, 0);
    if (result != ESP_OK) {
        return result;
    }
    esp_rom_delay_us(2);
    result = gpio_set_level(IRIS_PIN_MCP23008_RESET, 1);
    if (result != ESP_OK) {
        return result;
    }

    i2c_master_bus_config_t bus_config{};
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.sda_io_num = IRIS_PIN_I2C_SDA;
    bus_config.scl_io_num = IRIS_PIN_I2C_SCL;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = false;

    result = i2c_new_master_bus(&bus_config, &bus_handle_);
    if (result != ESP_OK) {
        bus_handle_ = nullptr;
        return result;
    }

    i2c_device_config_t device_config{};
    device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device_config.device_address = IRIS_MCP23008_I2C_ADDRESS;
    device_config.scl_speed_hz = kBusFrequencyHz;

    result = i2c_master_bus_add_device(
        bus_handle_, &device_config, &device_handle_);
    if (result != ESP_OK) {
        device_handle_ = nullptr;
        (void)i2c_del_master_bus(bus_handle_);
        bus_handle_ = nullptr;
        return result;
    }

    direction_ = 0xFF;
    output_latch_ = 0x00;
    return ESP_OK;
}

esp_err_t Mcp23008::configure_output(
    const std::uint8_t pin,
    const bool initial_level)
{
    if (device_handle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (pin >= kPinCount) {
        return ESP_ERR_INVALID_ARG;
    }

    const std::uint8_t mask = static_cast<std::uint8_t>(1U << pin);
    const std::uint8_t new_latch = initial_level
        ? static_cast<std::uint8_t>(output_latch_ | mask)
        : static_cast<std::uint8_t>(output_latch_ & ~mask);

    // Program the latch before enabling the output driver to prevent a
    // momentary pulse at the opposite level.
    esp_err_t result = write_register(kRegisterOlat, new_latch);
    if (result != ESP_OK) {
        return result;
    }
    output_latch_ = new_latch;

    const std::uint8_t new_direction =
        static_cast<std::uint8_t>(direction_ & ~mask);
    result = write_register(kRegisterIodir, new_direction);
    if (result == ESP_OK) {
        direction_ = new_direction;
    }
    return result;
}

esp_err_t Mcp23008::write_pin(
    const std::uint8_t pin,
    const bool level)
{
    if (device_handle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (pin >= kPinCount) {
        return ESP_ERR_INVALID_ARG;
    }

    const std::uint8_t mask = static_cast<std::uint8_t>(1U << pin);
    if ((direction_ & mask) != 0) {
        return ESP_ERR_INVALID_STATE;
    }

    const std::uint8_t new_latch = level
        ? static_cast<std::uint8_t>(output_latch_ | mask)
        : static_cast<std::uint8_t>(output_latch_ & ~mask);
    if (new_latch == output_latch_) {
        return ESP_OK;
    }

    const esp_err_t result = write_register(kRegisterOlat, new_latch);
    if (result == ESP_OK) {
        output_latch_ = new_latch;
    }
    return result;
}

esp_err_t Mcp23008::write_register(
    const std::uint8_t address,
    const std::uint8_t value)
{
    const std::uint8_t transaction[2] = {address, value};
    return i2c_master_transmit(
        device_handle_,
        transaction,
        sizeof(transaction),
        kTransactionTimeoutMs);
}

