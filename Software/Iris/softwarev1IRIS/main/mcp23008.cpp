#include "mcp23008.h"

#include <cstdint>

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hardware.h"

esp_err_t Mcp23008::initialize(const bool pulse_reset)
{
    if (bus_handle_ != nullptr || device_handle_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    // After a CPU/system reset the pad may still be held (see below). Keep the
    // hold until the level is latched and the driver enabled, then release it,
    // so the pad never changes level.
    // Latch the pin level before enabling the driver so GPIO9 never glitches
    // low (the board has a 10k pulldown on this net, R55).
    if (!pulse_reset) {
        (void)gpio_set_level(IRIS_PIN_MCP23008_RESET, 1);
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
    // Output enabled and (when not pulsing) level already high: hand the pad
    // back from the hold left by the previous run (no-op if not held). A
    // deliberate expander reset needs the pad free to move as well.
    (void)gpio_hold_dis(IRIS_PIN_MCP23008_RESET);

    if (pulse_reset) {
        // The MCP23008 requires RESET low for at least 1 us. Resetting also
        // puts every port in input mode and clears the output latch.
        result = gpio_set_level(IRIS_PIN_MCP23008_RESET, 0);
        if (result != ESP_OK) {
            return result;
        }
        esp_rom_delay_us(2);
    }
    result = gpio_set_level(IRIS_PIN_MCP23008_RESET, 1);
    if (result != ESP_OK) {
        return result;
    }
    // I2: hold GPIO9 high through any CPU/system reset (panic, watchdog,
    // software, EXT/USB). Without this the pad is released at reset and R55
    // pulls the MCP23008 RESET low, which turns every output into an input
    // and cuts the cameras until the app has booted again (hundreds of ms).
    // With the pad held the expander is never reset and keeps its OLAT/IODIR,
    // so loads stay powered across a warm reset. The hold is in the RTC
    // domain: it does not survive a power cycle or a brownout that resets the
    // RTC domain (the expander loses its supply then anyway).
    (void)gpio_hold_en(IRIS_PIN_MCP23008_RESET);

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
    configured_ = false;
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

esp_err_t Mcp23008::configure_outputs(
    const std::uint8_t output_mask,
    const std::uint8_t latch)
{
    if (device_handle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    const std::uint8_t new_latch = static_cast<std::uint8_t>(latch & output_mask);
    const std::uint8_t new_direction = static_cast<std::uint8_t>(~output_mask);
    esp_err_t result = ESP_FAIL;
    for (int attempt = 0; attempt < 3; ++attempt) {
        // Latch first so an output never momentarily drives the wrong level.
        result = write_register(kRegisterOlat, new_latch);
        if (result == ESP_OK) {
            result = write_register(kRegisterIodir, new_direction);
        }
        if (result == ESP_OK) {
            break;
        }
        vTaskDelay(1);
    }
    if (result == ESP_OK) {
        output_latch_ = new_latch;
        direction_ = new_direction;
        configured_ = true;
    }
    return result;
}

esp_err_t Mcp23008::resync()
{
    if (device_handle_ == nullptr || !configured_) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t result = write_register(kRegisterOlat, output_latch_);
    if (result == ESP_OK) {
        result = write_register(kRegisterIodir, direction_);
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
    // Always write (no "unchanged" early-out) and always adopt the intended
    // latch: if the transaction failed after the chip latched it, the cache
    // must not drift from what the chip may hold; resync() re-pushes it.
    output_latch_ = new_latch;
    return write_register(kRegisterOlat, new_latch);
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

esp_err_t Mcp23008::verify_and_heal(bool &healed)
{
    healed = false;
    if (device_handle_ == nullptr || !configured_) {
        return ESP_ERR_INVALID_STATE;
    }
    // Read IODIR and OLAT back (two 1-byte reads, ~100 us at 400 kHz). An
    // expander-only reset (3V3 dip, GPIO9 glitch) shows up as IODIR = 0xFF
    // (all inputs, loads off): re-push the cached state at once instead of
    // waiting for the 1 Hz blind resync.
    std::uint8_t iodir = 0;
    std::uint8_t olat = 0;
    esp_err_t result = read_register(kRegisterIodir, iodir);
    if (result == ESP_OK) {
        result = read_register(kRegisterOlat, olat);
    }
    if (result != ESP_OK) {
        return result;  // never touch the cache on an I2C error
    }
    const std::uint8_t out_mask = static_cast<std::uint8_t>(~direction_);
    if (iodir != direction_ ||
        static_cast<std::uint8_t>(olat & out_mask) !=
            static_cast<std::uint8_t>(output_latch_ & out_mask)) {
        result = resync();
        healed = result == ESP_OK;
    }
    return result;
}

esp_err_t Mcp23008::recover_bus()
{
    if (bus_handle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    // ESP-IDF resets the I2C controller FSM and clocks SCL (9 pulses plus a
    // STOP) to release a slave that is holding SDA low.
    const esp_err_t result = i2c_master_bus_reset(bus_handle_);
    return result;
}

esp_err_t Mcp23008::read_register(
    const std::uint8_t address,
    std::uint8_t &value)
{
    return i2c_master_transmit_receive(
        device_handle_, &address, 1, &value, 1, kTransactionTimeoutMs);
}
