#include "gps_i2c_driver.h"

#include <cstring>

#include "esp_log.h"
#include "esp_timer.h"
#include "flash_logger.h"
#include "hardware_pins.h"
#include "spi_data_forwarder.h"

namespace {
constexpr char kTag[] = "gps_i2c";
std::uint32_t milliseconds() {
    return static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL);
}
}

esp_err_t GpsI2cDriver::start()
{
    if (bus_ || task_) return ESP_ERR_INVALID_STATE;
    commands_ = xQueueCreate(16, sizeof(Command));
    if (!commands_) return ESP_ERR_NO_MEM;
    i2c_master_bus_config_t bus{};
    bus.i2c_port = I2C_NUM_0;
    bus.sda_io_num = HW_PIN_GPS_I2C_SDA;
    bus.scl_io_num = HW_PIN_GPS_I2C_SCL;
    bus.clk_source = I2C_CLK_SRC_DEFAULT;
    bus.glitch_ignore_cnt = 7;
    bus.flags.enable_internal_pullup = true;
    esp_err_t result = i2c_new_master_bus(&bus, &bus_);
    if (result == ESP_OK) {
        i2c_device_config_t device{};
        device.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        device.device_address = kAddress;
        device.scl_speed_hz = kClockHz;
        result = i2c_master_bus_add_device(bus_, &device, &device_);
    }
    if (result == ESP_OK && xTaskCreate(task_entry, "gps_i2c", kTaskStackSize,
                                       this, kTaskPriority, &task_) != pdPASS)
        result = ESP_ERR_NO_MEM;
    if (result != ESP_OK) {
        if (device_) i2c_master_bus_rm_device(device_);
        if (bus_) i2c_del_master_bus(bus_);
        device_ = nullptr;
        bus_ = nullptr;
        vQueueDelete(commands_);
        commands_ = nullptr;
    }
    return result;
}

esp_err_t GpsI2cDriver::transmit(const std::uint8_t *data, const std::size_t length,
                                const TickType_t timeout_ticks)
{
    if (!data || !length || length > kCommandSize) return ESP_ERR_INVALID_ARG;
    if (!commands_ || !connected_.load()) return ESP_ERR_INVALID_STATE;
    Command command{};
    command.length = length;
    std::memcpy(command.data, data, length);
    return xQueueSend(commands_, &command, timeout_ticks) == pdTRUE
               ? ESP_OK : ESP_ERR_NO_MEM;
}

bool GpsI2cDriver::read_register(void *context, const std::uint8_t address,
                                std::uint8_t &value)
{
    auto *self = static_cast<GpsI2cDriver *>(context);
    return i2c_master_transmit_receive(self->device_, &address, 1, &value, 1,
                                       kTransactionTimeoutMs) == ESP_OK;
}

bool GpsI2cDriver::write_register(void *context, const std::uint8_t address,
                                 const std::uint8_t value)
{
    const std::uint8_t bytes[] = {address, value};
    auto *self = static_cast<GpsI2cDriver *>(context);
    return i2c_master_transmit(self->device_, bytes, sizeof(bytes),
                              kTransactionTimeoutMs) == ESP_OK;
}

bool GpsI2cDriver::send_command(void *context, const std::uint8_t *data,
                               const std::size_t length)
{
    return static_cast<GpsI2cDriver *>(context)->transmit(data, length) == ESP_OK;
}

void GpsI2cDriver::flush_usb()
{
    if (usb_used_) {
        // USB backpressure must not block the flash capture path.
        (void)forwarder_.queue_raw_bytes(usb_, usb_used_);
        usb_used_ = 0;
    }
}

bool GpsI2cDriver::exchange(const std::uint32_t now_ms, bool &progress)
{
    using Result = SkytraqI2cMailbox::Result;
    progress = false;
    if (!receive_pending_) {
        const auto result = mailbox_.read(receive_, receive_length_);
        if (result == Result::IoError || result == Result::InvalidControl) return false;
        if (result == Result::Ok) {
            receive_pending_ = true;
            receive_logged_ = false;
        }
    }
    if (receive_pending_) {
        if (!receive_logged_) {
            if (receive_length_ && logger_) {
                const auto result = logger_->log_external_bytes(receive_, receive_length_);
                // Leave the mailbox unread until the RAM queue has room.
                if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) return true;
            }
            receive_logged_ = true;
            if (usb_used_ + receive_length_ > sizeof(usb_)) flush_usb();
            std::memcpy(usb_ + usb_used_, receive_, receive_length_);
            usb_used_ += receive_length_;
            gps_.consume(receive_, receive_length_, now_ms);
        }
        const auto result = mailbox_.acknowledge_read();
        if (result == Result::IoError || result == Result::InvalidControl) return false;
        if (result == Result::Ok) {
            receive_pending_ = false;
            progress = true;
        }
    }
    if (!transmitting_.length &&
        xQueueReceive(commands_, &transmitting_, 0) == pdTRUE) {
        transmit_offset_ = 0;
        transmit_started_ms_ = now_ms;
    }
    if (transmitting_.length) {
        if (now_ms - transmit_started_ms_ >= kCommandTimeoutMs) return false;
        std::size_t sent = 0;
        const auto result = mailbox_.send_fragment(
            transmitting_.data + transmit_offset_,
            transmitting_.length - transmit_offset_, sent);
        if (result == Result::IoError || result == Result::InvalidControl) return false;
        if (result == Result::Ok) {
            transmit_offset_ += sent;
            if (transmit_offset_ == transmitting_.length) transmitting_.length = 0;
            progress = true;
        }
    }
    return true;
}

void GpsI2cDriver::disconnect(const char *reason)
{
    connected_.store(false);
    ESP_LOGW(kTag, "%s; sensor/flash logging continues, retrying GPS", reason);
    // Commands interrupted by a bus fault cannot safely be replayed halfway.
    xQueueReset(commands_);
    transmitting_.length = 0;
    // Retain a captured packet whose mailbox ACK failed. On reconnection,
    // retry the ACK without logging that same packet for a second time.
    flush_usb();
}

void GpsI2cDriver::task_entry(void *context) {
    static_cast<GpsI2cDriver *>(context)->run();
}

void GpsI2cDriver::run()
{
    ESP_LOGI(kTag, "SDA=%d SCL=%d, %lu Hz, 7-bit address 0x%02X (AN0031 mailbox)",
             HW_PIN_GPS_I2C_SDA, HW_PIN_GPS_I2C_SCL,
             static_cast<unsigned long>(kClockHz), kAddress);
    vTaskDelay(pdMS_TO_TICKS(Px1120sGps::kStartupDelayMs));
    auto last_status = gps_.status();
    bool warned_missing = false;
    for (;;) {
        if (!connected_.load()) {
            if (i2c_master_probe(bus_, kAddress, kTransactionTimeoutMs) != ESP_OK) {
                if (!warned_missing) {
                    ESP_LOGW(kTag, "no GPS at 0x%02X; check wiring and I2C-enabled firmware", kAddress);
                    warned_missing = true;
                }
                vTaskDelay(pdMS_TO_TICKS(kReconnectIntervalMs));
                continue;
            }
            gps_ = Px1120sGps({this, send_command, nullptr});
            last_status = gps_.status();
            connected_.store(true);
            warned_missing = false;
            ESP_LOGI(kTag, "I2C device found; validating mailbox and configuring airborne mode");
        }
        gps_.service(milliseconds());
        for (unsigned i = 0; i < kMaximumExchangesPerPass; ++i) {
            bool progress = false;
            if (!exchange(milliseconds(), progress)) {
                disconnect("I2C mailbox error or command timeout");
                vTaskDelay(pdMS_TO_TICKS(kReconnectIntervalMs));
                break;
            }
            if (!progress) break;
        }
        flush_usb();
        if (gps_.status() != last_status) {
            last_status = gps_.status();
            if (last_status == Px1120sGps::Status::Ready)
                ESP_LOGI(kTag, "airborne mode %u verified over I2C; raw GPS logging active",
                         Px1120sGps::kNavigationMode);
            else if (last_status == Px1120sGps::Status::Failed)
                ESP_LOGW(kTag, "configuration unverified: %s; raw GPS logging continues",
                         gps_.failure_reason());
        }
        vTaskDelay(1);
    }
}
