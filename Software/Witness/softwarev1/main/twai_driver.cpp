#include "twai_driver.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "esp_log.h"
#include "esp_twai_onchip.h"
#include "hardware_pins.h"
#include "spi_data_forwarder.h"

namespace {

constexpr char kTag[] = "twai";
constexpr std::uint8_t kLoopbackRequest[] = {0x05, 0x22};

}  // namespace

TwaiDriver::TwaiDriver(SpiDataForwarder &serial_forwarder)
    : serial_forwarder_(serial_forwarder)
{
}

TwaiDriver::~TwaiDriver()
{
    (void)shutdown();
}

esp_err_t TwaiDriver::start()
{
    if (task_handle_ != nullptr || node_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t result = initialize();
    if (result != ESP_OK) {
        return result;
    }

    if (xTaskCreate(
            task_entry,
            "twai",
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

esp_err_t TwaiDriver::initialize()
{
    if (node_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    twai_onchip_node_config_t config{};
    config.io_cfg.tx = HW_PIN_CAN_TX;
    config.io_cfg.rx = HW_PIN_CAN_RX;
    config.io_cfg.quanta_clk_out = GPIO_NUM_NC;
    config.io_cfg.bus_off_indicator = GPIO_NUM_NC;
    config.bit_timing.bitrate = kBitrate;
    config.timestamp_resolution_hz = 1000000U;
    config.fail_retry_cnt = 3;
    config.tx_queue_depth = kTransmitQueueDepth;

    esp_err_t result = twai_new_node_onchip(&config, &node_);
    if (result != ESP_OK) {
        node_ = nullptr;
        return result;
    }

    twai_event_callbacks_t callbacks{};
    callbacks.on_tx_done = on_transmit_done;
    callbacks.on_rx_done = on_receive;
    callbacks.on_state_change = on_state_change;
    result = twai_node_register_event_callbacks(node_, &callbacks, this);
    if (result != ESP_OK) {
        (void)twai_node_delete(node_);
        node_ = nullptr;
        return result;
    }

    result = twai_node_enable(node_);
    if (result != ESP_OK) {
        (void)twai_node_delete(node_);
        node_ = nullptr;
    }
    return result;
}

esp_err_t TwaiDriver::shutdown()
{
    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }

    if (node_ == nullptr) {
        return ESP_OK;
    }

    const esp_err_t disable_result = twai_node_disable(node_);
    if (disable_result != ESP_OK && disable_result != ESP_ERR_INVALID_STATE) {
        return disable_result;
    }

    const esp_err_t delete_result = twai_node_delete(node_);
    if (delete_result == ESP_OK) {
        node_ = nullptr;
    }
    return delete_result;
}

void TwaiDriver::task_entry(void *const context)
{
    static_cast<TwaiDriver *>(context)->run();
}

esp_err_t TwaiDriver::send_loopback_request()
{
    Frame request{};
    request.identifier = kLoopbackCanIdentifier;
    request.length = sizeof(kLoopbackRequest);
    std::copy_n(
        std::begin(kLoopbackRequest),
        request.length,
        request.data.begin());
    return transmit(request);
}

void TwaiDriver::process_received_frame(
    const Frame &frame,
    bool &waiting_for_loopback_reply)
{
    if (!frame.remote && frame.length > 0) {
        const esp_err_t result = serial_forwarder_.queue_raw_bytes(
            frame.data.data(), frame.length);
        if (result != ESP_OK) {
            ESP_LOGW(
                kTag,
                "could not forward CAN frame 0x%03" PRIX32 ": %s",
                frame.identifier,
                esp_err_to_name(result));
        }
    }

    if (waiting_for_loopback_reply && !frame.remote && frame.length >= 2 &&
        frame.data[0] == kLoopbackRequest[0] &&
        frame.data[1] == kLoopbackRequest[1]) {
        waiting_for_loopback_reply = false;
        ESP_LOGI(
            kTag,
            "loopback reply received on CAN ID 0x%03" PRIX32,
            frame.identifier);
    }
}

void TwaiDriver::run()
{
    ESP_LOGI(
        kTag,
        "ready at %" PRIu32 " bit/s on TX=%d RX=%d",
        kBitrate,
        static_cast<int>(HW_PIN_CAN_TX),
        static_cast<int>(HW_PIN_CAN_RX));

    bool waiting_for_loopback_reply = false;
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

    for (;;) {
        const esp_err_t poll_result = poll();
        if (poll_result != ESP_OK && poll_result != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(kTag, "bus recovery failed: %s", esp_err_to_name(poll_result));
        }

        Frame frame{};
        while (receive(frame)) {
            process_received_frame(frame, waiting_for_loopback_reply);
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

        vTaskDelay(task_delay_ticks);
    }
}

esp_err_t TwaiDriver::transmit(const Frame &frame)
{
    if (node_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (frame.length > kMaxDataLength ||
        (!frame.extended && frame.identifier > TWAI_STD_ID_MASK) ||
        (frame.extended && frame.identifier > TWAI_EXT_ID_MASK)) {
        return ESP_ERR_INVALID_ARG;
    }

    TransmitSlot *slot = nullptr;
    portENTER_CRITICAL(&lock_);
    for (TransmitSlot &candidate : transmit_slots_) {
        if (!candidate.in_use) {
            candidate.in_use = true;
            slot = &candidate;
            break;
        }
    }
    portEXIT_CRITICAL(&lock_);
    if (slot == nullptr) {
        return ESP_ERR_TIMEOUT;
    }

    std::copy_n(frame.data.begin(), frame.length, slot->data.begin());
    slot->native_frame = {};
    slot->native_frame.header.id = frame.identifier;
    slot->native_frame.header.dlc = frame.length;
    slot->native_frame.header.ide = frame.extended;
    slot->native_frame.header.rtr = frame.remote;
    slot->native_frame.buffer = slot->data.data();
    slot->native_frame.buffer_len = frame.remote ? 0U : frame.length;

    const esp_err_t result = twai_node_transmit(node_, &slot->native_frame, 0);
    if (result != ESP_OK) {
        portENTER_CRITICAL(&lock_);
        slot->in_use = false;
        portEXIT_CRITICAL(&lock_);
    }
    return result;
}

bool TwaiDriver::receive(Frame &frame)
{
    portENTER_CRITICAL(&lock_);
    if (receive_read_index_ == receive_write_index_) {
        portEXIT_CRITICAL(&lock_);
        return false;
    }

    const ReceiveSlot &slot = receive_queue_[receive_read_index_];
    frame.identifier = slot.header.id;
    frame.length = static_cast<std::uint8_t>(
        std::min<std::uint16_t>(slot.header.dlc, kMaxDataLength));
    frame.extended = slot.header.ide;
    frame.remote = slot.header.rtr;
    frame.timestamp_us = slot.header.timestamp;
    frame.data.fill(0);
    if (!frame.remote) {
        std::copy_n(slot.data.begin(), frame.length, frame.data.begin());
    }
    receive_read_index_ = (receive_read_index_ + 1U) % kReceiveQueueDepth;
    portEXIT_CRITICAL(&lock_);
    return true;
}

esp_err_t TwaiDriver::poll()
{
    if (node_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    bool start_recovery = false;
    portENTER_CRITICAL(&lock_);
    if (bus_off_ && !recovery_started_) {
        recovery_started_ = true;
        start_recovery = true;
    }
    portEXIT_CRITICAL(&lock_);

    if (!start_recovery) {
        return ESP_OK;
    }

    const esp_err_t result = twai_node_recover(node_);
    if (result != ESP_OK) {
        portENTER_CRITICAL(&lock_);
        recovery_started_ = false;
        portEXIT_CRITICAL(&lock_);
    }
    return result;
}

esp_err_t TwaiDriver::get_status(
    twai_node_status_t &status,
    twai_node_record_t &record) const
{
    if (node_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    return twai_node_get_info(node_, &status, &record);
}

std::uint32_t TwaiDriver::dropped_receive_frames() const
{
    portENTER_CRITICAL(&lock_);
    const std::uint32_t result = dropped_receive_frames_;
    portEXIT_CRITICAL(&lock_);
    return result;
}

std::uint32_t TwaiDriver::failed_transmit_frames() const
{
    portENTER_CRITICAL(&lock_);
    const std::uint32_t result = failed_transmit_frames_;
    portEXIT_CRITICAL(&lock_);
    return result;
}

bool IRAM_ATTR TwaiDriver::on_receive(
    const twai_node_handle_t node,
    const twai_rx_done_event_data_t *,
    void *const context)
{
    return static_cast<TwaiDriver *>(context)->handle_receive_from_isr(node);
}

bool IRAM_ATTR TwaiDriver::on_transmit_done(
    const twai_node_handle_t,
    const twai_tx_done_event_data_t *const event,
    void *const context)
{
    static_cast<TwaiDriver *>(context)->handle_transmit_done_from_isr(*event);
    return false;
}

bool IRAM_ATTR TwaiDriver::on_state_change(
    const twai_node_handle_t,
    const twai_state_change_event_data_t *const event,
    void *const context)
{
    static_cast<TwaiDriver *>(context)->handle_state_change_from_isr(*event);
    return false;
}

bool TwaiDriver::handle_receive_from_isr(const twai_node_handle_t node)
{
    portENTER_CRITICAL_ISR(&lock_);
    const std::size_t write_index = receive_write_index_;
    const std::size_t next_write =
        (write_index + 1U) % kReceiveQueueDepth;
    const bool queue_full = next_write == receive_read_index_;
    portEXIT_CRITICAL_ISR(&lock_);

    if (queue_full) {
        std::uint8_t discarded_data[kMaxDataLength]{};
        twai_frame_t discarded_frame{};
        discarded_frame.buffer = discarded_data;
        discarded_frame.buffer_len = sizeof(discarded_data);
        (void)twai_node_receive_from_isr(node, &discarded_frame);

        portENTER_CRITICAL_ISR(&lock_);
        ++dropped_receive_frames_;
        portEXIT_CRITICAL_ISR(&lock_);
        return false;
    }

    ReceiveSlot &slot = receive_queue_[write_index];
    twai_frame_t native_frame{};
    native_frame.buffer = slot.data.data();
    native_frame.buffer_len = slot.data.size();
    if (twai_node_receive_from_isr(node, &native_frame) != ESP_OK) {
        return false;
    }

    slot.header = native_frame.header;
    portENTER_CRITICAL_ISR(&lock_);
    receive_write_index_ = next_write;
    portEXIT_CRITICAL_ISR(&lock_);
    return false;
}

void TwaiDriver::handle_transmit_done_from_isr(
    const twai_tx_done_event_data_t &event)
{
    portENTER_CRITICAL_ISR(&lock_);
    for (TransmitSlot &slot : transmit_slots_) {
        if (&slot.native_frame == event.done_tx_frame) {
            slot.in_use = false;
            break;
        }
    }
    if (!event.is_tx_success) {
        ++failed_transmit_frames_;
    }
    portEXIT_CRITICAL_ISR(&lock_);
}

void TwaiDriver::handle_state_change_from_isr(
    const twai_state_change_event_data_t &event)
{
    portENTER_CRITICAL_ISR(&lock_);
    bus_off_ = event.new_sta == TWAI_ERROR_BUS_OFF;
    if (!bus_off_) {
        recovery_started_ = false;
    }
    portEXIT_CRITICAL_ISR(&lock_);
}
