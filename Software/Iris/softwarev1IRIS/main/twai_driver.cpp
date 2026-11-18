#include "twai_driver.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "esp_twai_onchip.h"
#include "esp_timer.h"
#include "hardware.h"

TwaiDriver::~TwaiDriver()
{
    (void)shutdown();
}

esp_err_t TwaiDriver::initialize()
{
    if (node_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    twai_onchip_node_config_t config{};
    config.io_cfg.tx = IRIS_PIN_CAN_TX;
    config.io_cfg.rx = IRIS_PIN_CAN_RX;
    config.io_cfg.quanta_clk_out = GPIO_NUM_NC;
    config.io_cfg.bus_off_indicator = GPIO_NUM_NC;
    config.bit_timing.bitrate = IRIS_TWAI_BITRATE;
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
    const std::int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&lock_);
    bool any_busy = false;
    for (const TransmitSlot &candidate : transmit_slots_) {
        any_busy = any_busy || candidate.in_use;
    }
    for (TransmitSlot &candidate : transmit_slots_) {
        if (!candidate.in_use) {
            candidate.in_use = true;
            slot = &candidate;
            break;
        }
    }
    // Idle -> busy: start the "no progress" clock from now.
    if (slot != nullptr && !any_busy) {
        last_tx_progress_us_ = now_us;
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

esp_err_t TwaiDriver::reset_node()
{
    ++node_resets_;
    (void)shutdown();  // disables (bus-off: already disabled) and deletes
    if (node_ != nullptr) {
        // Could not delete: retry on the next poll (progress clock restarts
        // so we do not spin).
        portENTER_CRITICAL(&lock_);
        last_tx_progress_us_ = esp_timer_get_time();
        portEXIT_CRITICAL(&lock_);
        return ESP_FAIL;
    }
    portENTER_CRITICAL(&lock_);
    for (TransmitSlot &slot : transmit_slots_) {
        slot.in_use = false;
    }
    bus_off_ = false;
    recovery_started_ = false;
    last_tx_progress_us_ = esp_timer_get_time();
    portEXIT_CRITICAL(&lock_);
    return initialize();
}

esp_err_t TwaiDriver::poll()
{
    if (node_ == nullptr) {
        // A failed rebuild leaves no node: keep trying (once per second).
        const std::int64_t now_us = esp_timer_get_time();
        if (now_us - last_tx_progress_us_ < 1000000) {
            return ESP_ERR_INVALID_STATE;
        }
        last_tx_progress_us_ = now_us;
        return initialize();
    }

    // I6: if every transmit slot is in use and nothing has completed for
    // kTxStuckTimeoutUs (frames lost across a bus-off without a tx-done
    // event), rebuild the node, which frees the driver queue and all slots.
    {
        int in_use = 0;
        std::int64_t progress_us = 0;
        portENTER_CRITICAL(&lock_);
        for (const TransmitSlot &slot : transmit_slots_) {
            in_use += slot.in_use ? 1 : 0;
        }
        progress_us = last_tx_progress_us_;
        portEXIT_CRITICAL(&lock_);
        if (iris_recovery::tx_slots_stuck(
                in_use, static_cast<int>(kTransmitQueueDepth),
                esp_timer_get_time(), progress_us, kTxStuckTimeoutUs)) {
            return reset_node();
        }
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
    last_tx_progress_us_ = esp_timer_get_time();
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
