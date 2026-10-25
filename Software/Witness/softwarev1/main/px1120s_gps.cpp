#include "px1120s_gps.h"

bool Px1120sGps::send_payload(const std::uint8_t *payload, const std::size_t length)
{
    if (length == 0 || length > 4) return false;
    std::array<std::uint8_t, 11> packet{{0xA0, 0xA1, 0x00,
                                      static_cast<std::uint8_t>(length)}};
    std::uint8_t checksum = 0;
    for (std::size_t i = 0; i < length; ++i) {
        packet[4 + i] = payload[i];
        checksum ^= payload[i];
    }
    packet[4 + length] = checksum;
    packet[5 + length] = 0x0D;
    packet[6 + length] = 0x0A;
    return transport_.send(transport_.context, packet.data(), length + 7);
}

void Px1120sGps::begin_step(const Step step, const std::uint32_t now_ms)
{
    step_ = step;
    step_time_ms_ = now_ms;
    attempts_ = 0;
    waiting_ = false;
}

void Px1120sGps::fail(const char *reason)
{
    failure_reason_ = reason;
    status_ = Status::Failed;
    // Logging must always continue at the requested operational baud rate.
    if (transport_.set_baud)
        (void)transport_.set_baud(transport_.context, kBaudRate);
}

void Px1120sGps::service(const std::uint32_t now_ms)
{
    if (status_ != Status::Configuring) return;
    if (!started_) {
        started_ = true;
        step_time_ms_ = now_ms;
    }
    if (step_ == Step::BootDelay) {
        if (now_ms - step_time_ms_ >= kStartupDelayMs)
            begin_step(Step::Probe, now_ms);
        return;
    }
    if (waiting_ && now_ms - sent_time_ms_ < kReplyTimeoutMs) return;
    if (attempts_ >= kMaximumAttempts) {
        if (step_ == Step::Probe && operational_stream_seen_) {
            // A receiver can output good NMEA without answering this binary
            // query. Never disrupt the stream by hunting for another baud.
            // Still attempt airborne configuration and require mode readback.
            begin_step(Step::Configure, now_ms);
        } else if (step_ == Step::Configure) {
            // Some firmware misses an ACK; actual mode readback is decisive.
            begin_step(Step::Verify, now_ms);
        } else {
            fail(step_ == Step::Probe ? "GPS navigation probe timed out"
                                      : "navigation mode verification timed out");
        }
        return;
    }

    const std::uint8_t configure[] = {
        0x64, 0x17, kNavigationMode,
        static_cast<std::uint8_t>(kSaveSettingsToFlash ? 1 : 0)};
    const std::uint8_t query[] = {0x64, 0x18};
    const bool sent = step_ == Step::Configure
                          ? send_payload(configure, sizeof(configure))
                          : send_payload(query, sizeof(query));
    if (!sent) {
        fail("GPS command transmission failed");
        return;
    }
    ++attempts_;
    waiting_ = true;
    sent_time_ms_ = now_ms;
}

void Px1120sGps::consume(const std::uint8_t *data, const std::size_t length,
                        const std::uint32_t now_ms)
{
    if (status_ != Status::Configuring) return;
    if (frame_used_ && now_ms - last_byte_ms_ > kFrameIdleTimeoutMs)
        frame_used_ = frame_expected_ = 0;
    last_byte_ms_ = now_ms;

    for (std::size_t i = 0; i < length; ++i) {
        const auto byte = data[i];
        if (frame_used_ == 0) {
            if (byte == 0xA0) frame_[frame_used_++] = byte;
            continue;
        }
        if (frame_used_ == 1 && byte != 0xA1) {
            frame_used_ = byte == 0xA0 ? 1 : 0;
            continue;
        }
        frame_[frame_used_++] = byte;
        if (frame_used_ == 4) {
            const std::size_t payload_size = (frame_[2] << 8U) | frame_[3];
            if (payload_size == 0 || payload_size > frame_.size() - 7) {
                frame_used_ = frame_expected_ = 0;
                continue;
            }
            frame_expected_ = payload_size + 7;
        }
        if (!frame_expected_ || frame_used_ != frame_expected_) continue;
        std::uint8_t checksum = 0;
        for (std::size_t j = 4; j < frame_used_ - 3; ++j) checksum ^= frame_[j];
        if (checksum == frame_[frame_used_ - 3] &&
            frame_[frame_used_ - 2] == 0x0D && frame_[frame_used_ - 1] == 0x0A)
            handle_frame(now_ms);
        frame_used_ = frame_expected_ = 0;
        if (status_ != Status::Configuring) break;
    }
}

void Px1120sGps::handle_frame(const std::uint32_t now_ms)
{
    if (!waiting_) return;
    const std::size_t length = frame_expected_ - 7;
    const auto *payload = frame_.data() + 4;
    if (length >= 2 && payload[0] == 0x84 && payload[1] == 0x64) {
        fail("GPS rejected navigation command (NACK)");
        return;
    }
    if (step_ == Step::Configure && length >= 2 && payload[0] == 0x83 &&
        payload[1] == 0x64 && (length == 2 || payload[2] == 0x17)) {
        begin_step(Step::Verify, now_ms);
        return;
    }
    if (length != 3 || payload[0] != 0x64 || payload[1] != 0x8B) return;
    if (step_ == Step::Probe) {
        begin_step(Step::Configure, now_ms);
    } else if (step_ == Step::Verify) {
        if (payload[2] == kNavigationMode) status_ = Status::Ready;
        else fail("GPS navigation readback did not match airborne mode");
    }
}
