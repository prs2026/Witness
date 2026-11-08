#pragma once

#include <cstddef>
#include <cstdint>

#include "comms.h"

// Hardware-in-the-loop (HITL) input: decoding of host 0x0C sensor frames, a
// byte-stream frame assembler, and the enter/exit/timeout state. Pure logic
// with injected time, independent of ESP-IDF and FreeRTOS for host testing.
// Callers provide their own locking.
namespace hitl {

constexpr std::size_t kFrameLength = IRIS_PACKET_HITL_FRAME_LENGTH;
constexpr std::uint32_t kInputTimeoutMs = IRIS_HITL_INPUT_TIMEOUT_MS;

// One decoded 0x0C frame: native counts exactly as sent, plus the engineering
// values Witness derives from them with the real sensors' scale factors.
struct Sample {
    std::uint32_t sim_time_ms = 0;
    std::int16_t low_g_accel_raw[3]{};
    std::int16_t high_g_accel_raw[3]{};
    std::int16_t gyro_raw[3]{};
    std::int16_t temperature_raw = 0;
    std::uint32_t pressure_pa = 0;  // Numerically equal to centi-mbar.
    std::int16_t barometer_temperature_centi_celsius = 0;
    std::int32_t latitude_e4 = 0;   // 0.0001 degree counts.
    std::int32_t longitude_e4 = 0;
    float gps_altitude_meters = 0.0F;
    std::uint8_t satellites = 0;
    std::uint8_t fix_quality = 0;
    std::uint16_t battery_mv = 0;

    float low_g_accel_mg[3]{};
    float high_g_accel_mg[3]{};
    float gyro_mdps[3]{};
    float temperature_celsius = 0.0F;
    // Same formula and sea-level reference as the MS5607 driver; zero and
    // barometer_valid false when pressure is zero.
    float barometric_altitude_meters = 0.0F;
    bool barometer_valid = false;
    double latitude_degrees = 0.0;  // Zero unless gps_fix.
    double longitude_degrees = 0.0;
    // GGA quality 1..8 with a finite altitude, as the NMEA parser requires.
    bool gps_fix = false;
};

// Decodes a complete frame: length 47, ID 0x0C and EOF 0x0A at offset 46.
bool decode_frame(const std::uint8_t *frame, std::size_t length, Sample &sample);

// Collects fixed-length 0x0C frames from a byte stream. The caller decides
// whether a 0x0C byte starts a frame (only at a command boundary); once
// started, every byte belongs to the frame until it completes or fails.
class FrameAssembler final {
public:
    enum class Result : std::uint8_t {
        NotFramed,  // Inactive and the byte is not a frame ID; not consumed.
        Pending,    // Byte consumed; frame incomplete.
        Frame,      // frame() holds a complete frame with a valid EOF.
        BadEof,     // Final byte was not 0x0A. The assembler resynchronized
                    // on the next 0x0C inside the rejected bytes, if any.
    };

    Result push(std::uint8_t byte);
    bool active() const { return length_ > 0U; }
    std::size_t length() const { return length_; }
    const std::uint8_t *frame() const { return buffer_; }
    std::uint8_t rejected_eof() const { return rejected_eof_; }
    void reset() { length_ = 0U; }

private:
    std::uint8_t buffer_[kFrameLength]{};
    std::size_t length_ = 0U;
    std::uint8_t rejected_eof_ = 0U;
};

enum class Source : std::uint8_t { Usb, Radio };

// HITL mode state. Starts inactive (a reboot always starts with HITL off).
class Controller final {
public:
    enum class EnterResult : std::uint8_t {
        Entered,
        AlreadyActive,
        RejectedSource,       // Not from USB.
        RejectedFlightState,  // Not in Pad Idle.
    };

    EnterResult enter(Source source, bool pad_idle, std::uint32_t now_ms);
    // Returns true if HITL was active. Any source may exit.
    bool exit();
    // Accepts a sample while active (refreshing the input timeout); returns
    // false and ignores it otherwise.
    bool submit(const Sample &sample, std::uint32_t now_ms);
    // Exits when no sample has arrived for kInputTimeoutMs since entry or the
    // last sample. Returns true only on the call that caused the exit.
    bool expire(std::uint32_t now_ms);
    // Copies the latest sample if one arrived since the previous take.
    bool take_sample(Sample &sample);

    bool active() const { return active_; }
    bool has_sample() const { return has_sample_; }
    // Incremented on every entry, so a consumer can detect exit+re-entry.
    std::uint32_t session() const { return session_; }
    std::uint32_t samples_received() const { return samples_received_; }

private:
    Sample latest_{};
    std::uint32_t last_input_ms_ = 0U;
    std::uint32_t session_ = 0U;
    std::uint32_t samples_received_ = 0U;
    bool active_ = false;
    bool has_sample_ = false;
    bool sample_pending_ = false;
};

}  // namespace hitl
