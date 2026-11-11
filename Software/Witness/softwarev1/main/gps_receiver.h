#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "gps_assist.h"

// Identifies the UART GPS receiver and configures it for flight. Nothing is
// sent until checksum-valid NMEA has been heard at the current rate and pin
// orientation, so a wrongly guessed orientation never drives the receiver's
// TX line (both supported receivers stream NMEA from power-up).
//
// Supported receivers:
//   SkyTraq PX1120S  - SkyTraq binary (AN0037): A0 A1 len payload xor 0D 0A
//   u-blox NEO-M8N   - UBX: B5 62 class id len(LE) payload ck_a ck_b
//
// Identification sends a harmless poll in each protocol and waits for a
// checksum-valid reply. A receiver that is already streaming NMEA at the
// operational baud is never disturbed by baud probing. A link without valid
// NMEA at 115200 is scanned through kScanBauds (u-blox factory 9600 first,
// then the other common rates); a receiver found at another rate is moved to
// the operational rate in its RAM (u-blox CFG-PRT, SkyTraq 0x05), so one left
// at an odd rate in its flash still works without a flash write. All settings are written to receiver RAM and reapplied on
// every boot, so neither receiver's flash is worn.
//
// Both receivers are set to their airborne (<= 4 g) dynamics model, their
// maximum position update rate, and NMEA output reduced to GGA + RMC every fix
// plus GSA (PDOP/HDOP/VDOP) every kGsaEveryFixes fixes, so the higher rate
// fits the UART bandwidth.
//
// Assistance (PX1120S): once Ready, begin_assist() takes a verified blob of
// SkyTraq message bodies (gps_assist.h; "Set GPS Ephemeris" 0x41) and writes
// them one at a time, waiting for each ACK/NACK, so a cold receiver can use
// uploaded ephemerides instead of decoding them from the sky (18-30 s).
class GpsReceiver final {
public:
    static constexpr bool kConfigureAtStartup = true;
    static constexpr std::uint32_t kOperationalBaud = 115200;
    static constexpr std::uint32_t kUbloxFactoryBaud = 9600;
    // Identification order: operational, u-blox factory, then the others.
    static constexpr std::uint32_t kScanBauds[] = {115200, 9600, 230400, 460800, 921600,
                                                   57600, 38400, 19200, 4800};
    static constexpr std::size_t kScanBaudCount = sizeof(kScanBauds) / sizeof(kScanBauds[0]);
    // SkyTraq 0x05 baud index for kOperationalBaud (5 = 115200).
    static constexpr std::uint8_t kSkytraqOperationalBaudIndex = 5;
    static constexpr std::uint32_t kStartupDelayMs = 1500;
    static constexpr std::uint32_t kReplyTimeoutMs = 1000;
    static constexpr std::uint32_t kFrameIdleTimeoutMs = 200;
    // Time for a u-blox to finish its ACK and switch rate after CFG-PRT.
    static constexpr std::uint32_t kBaudSwitchDelayMs = 150;
    static constexpr unsigned kMaximumAttempts = 3;
    // GSA (dilution of precision) interval: every 5 s on the PX1120S (its
    // NMEA intervals above 1 are seconds); every 5th fix on the u-blox.
    static constexpr std::uint8_t kGsaEveryFixes = 5;
    static constexpr unsigned kAssistAttempts = 2;

    // PX1120S: navigation mode 5 is airborne; the receiver is rated for 4 g.
    // 25 Hz is the maximum for the standard PX1120S (50/100 Hz are -H only).
    static constexpr std::uint8_t kSkytraqAirborneMode = 5;
    static constexpr std::uint8_t kSkytraqUpdateRateHz = 25;
    // Lowest elevation mask the PX1120S accepts (AN0037 0x2B: 3..85 deg).
    static constexpr std::uint8_t kSkytraqElevationMaskDeg = 3;
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
        // Optional: exchange the host RX/TX pins (harness wired either way).
        bool (*swap_pins)(void *) = nullptr;
    };

    explicit GpsReceiver(Transport transport) : transport_(transport) {}

    void service(std::uint32_t now_ms);
    // Observes a copy of the received stream; never removes bytes from logging.
    void consume(const std::uint8_t *data, std::size_t length,
                 std::uint32_t now_ms);
    // Call after checksum-valid NMEA is decoded at the current host baud.
    void observe_nmea() { nmea_seen_ = true; }
    // Rate measured on the RX line before identification (0 = unknown). The
    // nearest kScanBauds entry is tried first, then the rest in order; a
    // rate near no standard one (2400..1M) is tried as measured, first.
    void set_baud_hint(std::uint32_t measured_baud);
    // Nearest scan rate within 8%, or 0.
    static std::uint32_t nearest_scan_baud(std::uint32_t measured_baud);

    Status status() const { return status_; }
    Model model() const { return model_; }
    const char *model_name() const;
    const char *failure_reason() const { return failure_reason_; }
    std::uint32_t baud() const { return baud_; }
    bool pins_swapped() const { return pins_swapped_; }
    std::uint32_t update_rate_hz() const;
    // True when the model was assumed from NMEA because no binary reply came.
    bool model_assumed() const { return model_assumed_; }
    // Optional configuration commands the receiver rejected (NACK), and the
    // last one's name; configuration carries on without them.
    unsigned optional_rejects() const { return optional_rejects_; }
    const char *last_optional_reject() const { return last_optional_reject_; }

    // Assistance upload (PX1120S, after Ready). Copies the blob; false if it
    // cannot start (wrong receiver, not ready, upload already running).
    bool begin_assist(const std::uint8_t *blob, std::size_t length, std::uint32_t now_ms);
    struct AssistProgress {
        bool active = false;
        std::uint16_t total = 0;     // records in the current/last upload
        std::uint16_t sent = 0;      // records written (each counted once)
        std::uint16_t acked = 0;
        std::uint16_t nacked = 0;
        std::uint16_t timed_out = 0;
        std::uint32_t uploads = 0;   // completed uploads since boot
    };
    AssistProgress assist_progress() const { return assist_; }
    // Cold start: the receiver forgets ephemerides, almanac, position and
    // time; Witness then waits kRestartDelayMs and configures it again (the
    // RAM settings may be lost). False if no receiver is identified.
    bool cold_start(std::uint32_t now_ms);
    static constexpr std::uint32_t kRestartDelayMs = 2000;
    std::uint32_t cold_starts() const { return cold_starts_; }
    // Why begin_assist() refused, or "".
    const char *assist_refusal() const { return assist_refusal_; }

private:
    enum class Step {
        BootDelay,
        Identify,
        UbloxSwitchBaud,
        UbloxConfirmBaud,
        SkytraqSwitchBaud,
        SkytraqConfirmBaud,
        RestartWait,          // after a cold start, before reconfiguring
        Configure,
        Verify,
    };
    static constexpr std::size_t kSkytraqFrameCapacity = 64;
    static constexpr std::size_t kSkytraqTxCapacity = gps_assist::kMaximumRecord + 7;
    static constexpr std::size_t kUbloxPayloadCapacity = 48;
    static constexpr std::size_t kUbloxMaximumPayload = 1024;

    void begin_step(Step step, std::uint32_t now_ms);
    void identified(Model model, std::uint32_t now_ms);
    void fail(const char *reason);
    bool send_skytraq(const std::uint8_t *payload, std::size_t length);
    void service_assist(std::uint32_t now_ms);
    void finish_assist_record(std::uint32_t now_ms);
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
    std::size_t scan_index_ = 0;             // scan_order_ entry being tried
    // kScanBauds, optionally preceded by a measured non-standard rate.
    std::array<std::uint32_t, sizeof(kScanBauds) / sizeof(kScanBauds[0]) + 1> scan_order_{
        115200, 9600, 230400, 460800, 921600, 57600, 38400, 19200, 4800};
    std::size_t scan_count_ = sizeof(kScanBauds) / sizeof(kScanBauds[0]);
    bool pins_swapped_ = false;   // the second pin orientation is being scanned
    std::uint32_t found_baud_ = kOperationalBaud;  // where the receiver answered
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

    // Assistance upload state.
    std::array<std::uint8_t, gps_assist::kMaximumBlob> assist_blob_{};
    std::size_t assist_length_ = 0;
    std::size_t assist_offset_ = 0;       // offset of the current record
    std::uint8_t assist_message_id_ = 0;  // ID of the record awaiting a reply
    unsigned assist_attempts_ = 0;
    bool assist_waiting_ = false;
    std::uint32_t assist_sent_ms_ = 0;
    AssistProgress assist_{};
    std::uint32_t cold_starts_ = 0;
    unsigned optional_rejects_ = 0;
    const char *last_optional_reject_ = "";
    const char *assist_refusal_ = "";
};
