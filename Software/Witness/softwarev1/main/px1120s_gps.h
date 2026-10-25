#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// SkyTraq PX1120S startup configuration, serviced by its transport task.
// Protocol: SkyTraq AN0037, navigation 0x64/0x17,
// query navigation 0x64/0x18 and response 0x64/0x8B.
class Px1120sGps final {
public:
    static constexpr bool kConfigureAtStartup = true;
    // Keep host and receiver at the hardware-confirmed rate. Startup never
    // changes receiver baud or probes another rate while it is streaming.
    static constexpr std::uint32_t kBaudRate = 115200;
    static constexpr std::uint32_t kFactoryBaudRate = 115200;
    static constexpr std::uint8_t kNavigationMode = 5; // airborne; PX1120S rated for 4 g
    // Reapply SRAM settings each boot instead of wearing GPS flash on startup.
    static constexpr bool kSaveSettingsToFlash = false;
    static constexpr std::uint32_t kStartupDelayMs = 1500;
    static constexpr std::uint32_t kReplyTimeoutMs = 1000;
    static constexpr std::uint32_t kFrameIdleTimeoutMs = 200;
    static constexpr unsigned kMaximumAttempts = 3;

    struct Transport {
        void *context;
        bool (*send)(void *, const std::uint8_t *, std::size_t);
        // Null for I2C; UART failure recovery reapplies the configured host rate.
        bool (*set_baud)(void *, std::uint32_t);
    };
    enum class Status { Disabled, Configuring, Ready, Failed };

    explicit Px1120sGps(Transport transport) : transport_(transport) {}
    void service(std::uint32_t now_ms);
    // Observes a copy of the received stream; never removes bytes from logging.
    void consume(const std::uint8_t *data, std::size_t length,
                 std::uint32_t now_ms);
    // Call only after checksum-valid NMEA is decoded at the current UART baud.
    // No-fix sentences also prove that the serial link is working.
    void observe_nmea() {
        operational_stream_seen_ = true;
    }
    Status status() const { return status_; }
    const char *failure_reason() const { return failure_reason_; }

private:
    enum class Step { BootDelay, Probe, Configure, Verify };
    bool send_payload(const std::uint8_t *payload, std::size_t length);
    void begin_step(Step step, std::uint32_t now_ms);
    void handle_frame(std::uint32_t now_ms);
    void fail(const char *reason);

    static_assert(kMaximumAttempts > 0);
    Transport transport_;
    Status status_ = kConfigureAtStartup ? Status::Configuring : Status::Disabled;
    Step step_ = Step::BootDelay;
    bool started_ = false;
    bool waiting_ = false;
    bool operational_stream_seen_ = false;
    unsigned attempts_ = 0;
    std::uint32_t step_time_ms_ = 0;
    std::uint32_t sent_time_ms_ = 0;
    std::uint32_t last_byte_ms_ = 0;
    std::array<std::uint8_t, 64> frame_{};
    std::size_t frame_used_ = 0;
    std::size_t frame_expected_ = 0;
    const char *failure_reason_ = "";
};
