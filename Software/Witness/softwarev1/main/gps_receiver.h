#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// Identifies the UART GPS receiver and configures it for flight.
//
// Supported receivers:
//   SkyTraq PX1120S  - SkyTraq binary (AN0037): A0 A1 len payload xor 0D 0A
//   u-blox NEO-M8N   - UBX: B5 62 class id len(LE) payload ck_a ck_b
//
// Identification sends a harmless poll in each protocol and waits for a
// checksum-valid reply. A receiver that is already streaming NMEA at the
// operational baud is never disturbed by baud probing; only a silent link is
// retried at the u-blox factory rate, and a u-blox found there is moved to the
// operational rate. All settings are written to receiver RAM and reapplied on
// every boot, so neither receiver's flash is worn.
//
// Both receivers are set to their airborne (<= 4 g) dynamics model, their
// maximum position update rate, and NMEA output reduced to GGA + RMC so the
// higher rate fits the UART bandwidth.
class GpsReceiver final {
public:
    static constexpr bool kConfigureAtStartup = true;
    static constexpr std::uint32_t kOperationalBaud = 115200;
    static constexpr std::uint32_t kUbloxFactoryBaud = 9600;
    static constexpr std::uint32_t kStartupDelayMs = 1500;
    static constexpr std::uint32_t kReplyTimeoutMs = 1000;
    static constexpr std::uint32_t kFrameIdleTimeoutMs = 200;
    // Time for a u-blox to finish its ACK and switch rate after CFG-PRT.
    static constexpr std::uint32_t kBaudSwitchDelayMs = 150;
    static constexpr unsigned kMaximumAttempts = 3;

    // PX1120S: navigation mode 5 is airborne; the receiver is rated for 4 g.
    // 25 Hz is the maximum for the standard PX1120S (50/100 Hz are -H only).
    static constexpr std::uint8_t kSkytraqAirborneMode = 5;
    static constexpr std::uint8_t kSkytraqUpdateRateHz = 25;
    // NEO-M8N: dynamic model 8 is "airborne < 4 g". 10 Hz is the maximum
    // navigation rate with concurrent GNSS (its default GPS + GLONASS).
    static constexpr std::uint8_t kUbloxAirborne4gModel = 8;
    static constexpr std::uint16_t kUbloxMeasurementPeriodMs = 100;

    enum class Model { Unknown, Px1120s, UbloxM8 };
    enum class Status { Disabled, Configuring, Ready, Failed };

    struct Transport {
        void *context;
        // Must not return until the bytes have left the UART.
        bool (*send)(void *, const std::uint8_t *, std::size_t);
        bool (*set_baud)(void *, std::uint32_t);
    };

    explicit GpsReceiver(Transport transport) : transport_(transport) {}

    void service(std::uint32_t now_ms);
    // Observes a copy of the received stream; never removes bytes from logging.
    void consume(const std::uint8_t *data, std::size_t length,
                 std::uint32_t now_ms);
    // Call after checksum-valid NMEA is decoded at the current host baud.
    void observe_nmea() { nmea_seen_ = true; }

    Status status() const { return status_; }
    Model model() const { return model_; }
    const char *model_name() const;
    const char *failure_reason() const { return failure_reason_; }
    std::uint32_t baud() const { return baud_; }
    std::uint32_t update_rate_hz() const;
    // True when the model was assumed from NMEA because no binary reply came.
    bool model_assumed() const { return model_assumed_; }

private:
    enum class Step {
        BootDelay,
        Identify,
        UbloxSwitchBaud,
        UbloxConfirmBaud,
        Configure,
        Verify,
    };
    static constexpr std::size_t kSkytraqFrameCapacity = 64;
    static constexpr std::size_t kUbloxPayloadCapacity = 48;
    static constexpr std::size_t kUbloxMaximumPayload = 1024;

    void begin_step(Step step, std::uint32_t now_ms);
    void identified(Model model, std::uint32_t now_ms);
    void fail(const char *reason);
    bool send_skytraq(const std::uint8_t *payload, std::size_t length);
    bool send_ubx(std::uint8_t message_class, std::uint8_t id,
                  const std::uint8_t *payload, std::size_t length);
    bool send_current(std::uint32_t now_ms);
    std::size_t configure_count() const;
    std::size_t verify_count() const;
    void advance(std::uint32_t now_ms);
    void consume_skytraq(std::uint8_t byte, std::uint32_t now_ms);
    void consume_ubx(std::uint8_t byte, std::uint32_t now_ms);
    void handle_skytraq(const std::uint8_t *payload, std::size_t length,
                        std::uint32_t now_ms);
    void handle_ubx(std::uint8_t message_class, std::uint8_t id,
                    const std::uint8_t *payload, std::size_t length,
                    std::uint32_t now_ms);

    Transport transport_;
    Status status_ = kConfigureAtStartup ? Status::Configuring : Status::Disabled;
    Model model_ = Model::Unknown;
    Step step_ = Step::BootDelay;
    std::size_t index_ = 0;  // command within Configure / Verify
    std::uint32_t baud_ = kOperationalBaud;
    bool started_ = false;
    bool waiting_ = false;
    bool nmea_seen_ = false;
    bool model_assumed_ = false;
    unsigned attempts_ = 0;
    std::uint32_t step_time_ms_ = 0;
    std::uint32_t sent_time_ms_ = 0;
    const char *failure_reason_ = "";

    std::uint32_t last_byte_ms_ = 0;
    std::array<std::uint8_t, kSkytraqFrameCapacity> skytraq_{};
    std::size_t skytraq_used_ = 0;
    std::size_t skytraq_expected_ = 0;

    std::size_t ubx_state_ = 0;  // bytes of the current UBX frame consumed
    std::uint8_t ubx_class_ = 0;
    std::uint8_t ubx_id_ = 0;
    std::size_t ubx_length_ = 0;
    std::uint8_t ubx_ck_a_ = 0;
    std::uint8_t ubx_ck_b_ = 0;
    std::uint8_t ubx_received_ck_a_ = 0;
    std::array<std::uint8_t, kUbloxPayloadCapacity> ubx_payload_{};
};
