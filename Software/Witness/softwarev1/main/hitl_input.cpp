#include "hitl_input.h"

#include <cmath>
#include <cstring>

#include "pressure_altitude.h"

namespace hitl {
namespace {

static_assert(kFrameLength == 47U, "HITL frame length must match comms.h");

constexpr std::size_t kId = IRIS_PACKET_ID_LENGTH;  // Payload starts here.

std::uint16_t u16(const std::uint8_t *p)
{
    return static_cast<std::uint16_t>((p[0] << 8) | p[1]);
}

std::int16_t i16(const std::uint8_t *p)
{
    return static_cast<std::int16_t>(u16(p));
}

std::uint32_t u24(const std::uint8_t *p)
{
    return (static_cast<std::uint32_t>(p[0]) << 16) |
           (static_cast<std::uint32_t>(p[1]) << 8) | p[2];
}

std::int32_t i24(const std::uint8_t *p)
{
    const std::uint32_t raw = u24(p);
    return (raw & 0x00800000U) != 0U
        ? static_cast<std::int32_t>(raw) - 0x01000000
        : static_cast<std::int32_t>(raw);
}

std::uint32_t u32(const std::uint8_t *p)
{
    return (static_cast<std::uint32_t>(p[0]) << 24) |
           (static_cast<std::uint32_t>(p[1]) << 16) |
           (static_cast<std::uint32_t>(p[2]) << 8) | p[3];
}

float f32(const std::uint8_t *p)
{
    const std::uint32_t bits = u32(p);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

constexpr float kLowGMgPerCount =
    static_cast<float>(IRIS_FIELD_LSM6_LOW_ACCEL_MICRO_G_PER_COUNT) / 1000.0F;
constexpr float kHighGMgPerCount =
    static_cast<float>(IRIS_FIELD_LSM6_HIGH_ACCEL_MICRO_G_PER_COUNT) / 1000.0F;
constexpr float kGyroMdpsPerCount = static_cast<float>(
    IRIS_FIELD_LSM6_GYRO_MILLIDEGREES_PER_SECOND_PER_COUNT);

}  // namespace

bool decode_frame(const std::uint8_t *frame, const std::size_t length,
                  Sample &sample)
{
    if (frame == nullptr || length != kFrameLength ||
        frame[0] != IRIS_PACKET_ID_HITL_SENSORS ||
        frame[kId + IRIS_PACKET_HITL_EOF_OFFSET] != IRIS_PACKET_EOF_VALUE) {
        return false;
    }
    const std::uint8_t *payload = frame + kId;
    Sample s{};
    s.sim_time_ms = u32(payload + IRIS_PACKET_HITL_SIM_TIME_OFFSET);
    for (std::size_t axis = 0; axis < 3U; ++axis) {
        s.low_g_accel_raw[axis] =
            i16(payload + IRIS_PACKET_HITL_LOW_ACCEL_X_OFFSET + 2U * axis);
        s.high_g_accel_raw[axis] =
            i16(payload + IRIS_PACKET_HITL_HIGH_ACCEL_X_OFFSET + 2U * axis);
        s.gyro_raw[axis] = i16(payload + IRIS_PACKET_HITL_GYRO_X_OFFSET + 2U * axis);
        s.low_g_accel_mg[axis] =
            static_cast<float>(s.low_g_accel_raw[axis]) * kLowGMgPerCount;
        s.high_g_accel_mg[axis] =
            static_cast<float>(s.high_g_accel_raw[axis]) * kHighGMgPerCount;
        s.gyro_mdps[axis] = static_cast<float>(s.gyro_raw[axis]) * kGyroMdpsPerCount;
    }
    s.temperature_raw = i16(payload + IRIS_PACKET_HITL_IMU_TEMPERATURE_OFFSET);
    s.temperature_celsius = static_cast<float>(s.temperature_raw) / 256.0F + 25.0F;
    s.pressure_pa = u24(payload + IRIS_PACKET_HITL_PRESSURE_OFFSET);
    s.barometer_temperature_centi_celsius =
        i16(payload + IRIS_PACKET_HITL_BARO_TEMPERATURE_OFFSET);
    s.barometer_valid = pressure_altitude_meters(
        static_cast<std::int32_t>(s.pressure_pa), s.barometric_altitude_meters);
    s.latitude_e4 = i24(payload + IRIS_PACKET_HITL_LATITUDE_OFFSET);
    s.longitude_e4 = i24(payload + IRIS_PACKET_HITL_LONGITUDE_OFFSET);
    s.gps_altitude_meters = f32(payload + IRIS_PACKET_HITL_GPS_ALTITUDE_OFFSET);
    s.satellites = payload[IRIS_PACKET_HITL_GPS_SATELLITES_OFFSET];
    s.fix_quality = payload[IRIS_PACKET_HITL_GPS_FIX_QUALITY_OFFSET];
    s.battery_mv = u16(payload + IRIS_PACKET_HITL_BATTERY_VOLTAGE_OFFSET);
    s.gps_fix = s.fix_quality >= 1U && s.fix_quality <= 8U &&
                std::isfinite(s.gps_altitude_meters);
    if (s.gps_fix) {
        s.latitude_degrees = static_cast<double>(s.latitude_e4) /
                             static_cast<double>(IRIS_FIELD_GPS_COORDINATE_SCALE);
        s.longitude_degrees = static_cast<double>(s.longitude_e4) /
                              static_cast<double>(IRIS_FIELD_GPS_COORDINATE_SCALE);
    } else {
        s.gps_altitude_meters = 0.0F;  // Matches the NMEA no-fix behaviour.
    }
    sample = s;
    return true;
}

FrameAssembler::Result FrameAssembler::push(const std::uint8_t byte)
{
    if (length_ == 0U) {
        if (byte != IRIS_PACKET_ID_HITL_SENSORS) {
            return Result::NotFramed;
        }
        buffer_[length_++] = byte;
        return Result::Pending;
    }
    buffer_[length_++] = byte;
    if (length_ < kFrameLength) {
        return Result::Pending;
    }
    if (byte == IRIS_PACKET_EOF_VALUE) {
        length_ = 0U;
        return Result::Frame;
    }
    // Payload bytes may equal 0x0C, so a later 0x0C in the rejected bytes is
    // the earliest place a real frame can start.
    rejected_eof_ = byte;
    std::size_t start = 1U;
    while (start < kFrameLength && buffer_[start] != IRIS_PACKET_ID_HITL_SENSORS) {
        ++start;
    }
    length_ = kFrameLength - start;
    if (length_ > 0U) {
        std::memmove(buffer_, buffer_ + start, length_);
    }
    return Result::BadEof;
}

Controller::EnterResult Controller::enter(const Source source,
                                          const bool pad_idle,
                                          const std::uint32_t now_ms)
{
    if (source != Source::Usb) {
        return EnterResult::RejectedSource;
    }
    if (active_) {
        return EnterResult::AlreadyActive;
    }
    if (!pad_idle) {
        return EnterResult::RejectedFlightState;
    }
    active_ = true;
    has_sample_ = false;
    sample_pending_ = false;
    samples_received_ = 0U;
    last_input_ms_ = now_ms;
    ++session_;
    return EnterResult::Entered;
}

bool Controller::exit()
{
    const bool was_active = active_;
    active_ = false;
    has_sample_ = false;
    sample_pending_ = false;
    return was_active;
}

bool Controller::submit(const Sample &sample, const std::uint32_t now_ms)
{
    if (!active_) {
        return false;
    }
    latest_ = sample;
    last_input_ms_ = now_ms;
    has_sample_ = true;
    sample_pending_ = true;
    ++samples_received_;
    return true;
}

bool Controller::expire(const std::uint32_t now_ms)
{
    if (!active_ || now_ms - last_input_ms_ < kInputTimeoutMs) {
        return false;
    }
    (void)exit();
    return true;
}

bool Controller::take_sample(Sample &sample)
{
    if (!active_ || !sample_pending_) {
        return false;
    }
    sample = latest_;
    sample_pending_ = false;
    return true;
}

}  // namespace hitl
