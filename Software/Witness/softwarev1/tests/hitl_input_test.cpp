#include "hitl_input.h"
#include "pressure_altitude.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

struct Input {
    std::uint32_t sim_time_ms = 0;
    std::int16_t low[3]{};
    std::int16_t high[3]{};
    std::int16_t gyro[3]{};
    std::int16_t imu_temperature = 0;
    std::uint32_t pressure_pa = 0;
    std::int16_t baro_temperature = 0;
    std::int32_t latitude = 0;
    std::int32_t longitude = 0;
    float gps_altitude = 0.0F;
    std::uint8_t satellites = 0;
    std::uint8_t quality = 0;
    std::uint16_t battery_mv = 0;
};

void put(std::vector<std::uint8_t> &f, std::size_t offset, std::uint32_t value,
         std::size_t bytes)
{
    for (std::size_t i = 0; i < bytes; ++i) {
        f[1 + offset + i] =
            static_cast<std::uint8_t>(value >> (8U * (bytes - 1U - i)));
    }
}

std::vector<std::uint8_t> encode(const Input &in)
{
    std::vector<std::uint8_t> f(hitl::kFrameLength, 0);
    f[0] = IRIS_PACKET_ID_HITL_SENSORS;
    f[1] = 0xA4;  // Sender status: bits other than 0 (GPS from receiver) are ignored.
    f[2] = 0x5A;
    put(f, IRIS_PACKET_HITL_SIM_TIME_OFFSET, in.sim_time_ms, 4);
    for (std::size_t a = 0; a < 3; ++a) {
        put(f, IRIS_PACKET_HITL_LOW_ACCEL_X_OFFSET + 2 * a,
            static_cast<std::uint16_t>(in.low[a]), 2);
        put(f, IRIS_PACKET_HITL_HIGH_ACCEL_X_OFFSET + 2 * a,
            static_cast<std::uint16_t>(in.high[a]), 2);
        put(f, IRIS_PACKET_HITL_GYRO_X_OFFSET + 2 * a,
            static_cast<std::uint16_t>(in.gyro[a]), 2);
    }
    put(f, IRIS_PACKET_HITL_IMU_TEMPERATURE_OFFSET,
        static_cast<std::uint16_t>(in.imu_temperature), 2);
    put(f, IRIS_PACKET_HITL_PRESSURE_OFFSET, in.pressure_pa, 3);
    put(f, IRIS_PACKET_HITL_BARO_TEMPERATURE_OFFSET,
        static_cast<std::uint16_t>(in.baro_temperature), 2);
    put(f, IRIS_PACKET_HITL_LATITUDE_OFFSET,
        static_cast<std::uint32_t>(in.latitude) & 0xFFFFFFU, 3);
    put(f, IRIS_PACKET_HITL_LONGITUDE_OFFSET,
        static_cast<std::uint32_t>(in.longitude) & 0xFFFFFFU, 3);
    std::uint32_t bits = 0;
    std::memcpy(&bits, &in.gps_altitude, sizeof(bits));
    put(f, IRIS_PACKET_HITL_GPS_ALTITUDE_OFFSET, bits, 4);
    put(f, IRIS_PACKET_HITL_GPS_SATELLITES_OFFSET, in.satellites, 1);
    put(f, IRIS_PACKET_HITL_GPS_FIX_QUALITY_OFFSET, in.quality, 1);
    put(f, IRIS_PACKET_HITL_BATTERY_VOLTAGE_OFFSET, in.battery_mv, 2);
    f[hitl::kFrameLength - 1] = IRIS_PACKET_EOF_VALUE;
    return f;
}

bool near(double a, double b, double tolerance) { return std::fabs(a - b) <= tolerance; }

Input typical()
{
    Input in;
    in.sim_time_ms = 0x01020304U;
    in.low[0] = 2049; in.low[1] = -2049; in.low[2] = 0x0A0C;  // Embedded 0A/0C.
    in.high[0] = 960; in.high[1] = -1; in.high[2] = 32767;
    in.gyro[0] = -32768; in.gyro[1] = 100; in.gyro[2] = 0;
    in.imu_temperature = -512;
    in.pressure_pa = 101325;
    in.baro_temperature = -1234;
    in.latitude = 481173;
    in.longitude = -1210087;
    in.gps_altitude = 545.4F;
    in.satellites = 9;
    in.quality = 1;
    in.battery_mv = 7400;
    return in;
}

void test_decode()
{
    const Input in = typical();
    const auto f = encode(in);
    hitl::Sample s;
    assert(hitl::decode_frame(f.data(), f.size(), s));
    assert(s.sim_time_ms == 0x01020304U);
    assert(s.low_g_accel_raw[0] == 2049 && s.low_g_accel_raw[1] == -2049 &&
           s.low_g_accel_raw[2] == 0x0A0C);
    assert(near(s.low_g_accel_mg[0], 2049 * 0.488, 1e-3));   // ~1 g.
    assert(near(s.low_g_accel_mg[1], -2049 * 0.488, 1e-3));
    assert(s.high_g_accel_raw[0] == 960 && s.high_g_accel_raw[1] == -1 &&
           s.high_g_accel_raw[2] == 32767);
    assert(near(s.high_g_accel_mg[0], 960 * 10.417, 1e-2));  // ~10 g.
    assert(s.gyro_raw[0] == -32768 && s.gyro_raw[1] == 100);
    assert(near(s.gyro_mdps[0], -32768.0 * 70.0, 1e-3));
    assert(near(s.gyro_mdps[1], 7000.0, 1e-3));
    assert(s.temperature_raw == -512 && near(s.temperature_celsius, 23.0, 1e-6));
    assert(s.pressure_pa == 101325 && s.barometer_valid);
    assert(s.barometer_temperature_centi_celsius == -1234);
    float expected_altitude = 0.0F;
    assert(pressure_altitude_meters(101325, expected_altitude));
    assert(s.barometric_altitude_meters == expected_altitude);  // Same formula.
    assert(near(s.barometric_altitude_meters, 0.0, 0.5));
    assert(s.latitude_e4 == 481173 && s.longitude_e4 == -1210087);
    assert(s.gps_fix && near(s.latitude_degrees, 48.1173, 1e-12));
    assert(near(s.longitude_degrees, -121.0087, 1e-12));
    assert(s.gps_altitude_meters == 545.4F);
    assert(s.satellites == 9 && s.fix_quality == 1 && s.battery_mv == 7400);

    // Standard atmosphere: ~898.75 hPa is ~1000 m; 24-bit pressure maximum.
    Input high = in;
    high.pressure_pa = 89875;
    assert(hitl::decode_frame(encode(high).data(), hitl::kFrameLength, s));
    assert(near(s.barometric_altitude_meters, 1000.0, 1.0));
    high.pressure_pa = 0xFFFFFF;
    assert(hitl::decode_frame(encode(high).data(), hitl::kFrameLength, s));
    assert(s.pressure_pa == 0xFFFFFFU && s.barometer_valid &&
           s.barometric_altitude_meters < 0.0F);

    // Zero pressure: barometer invalid, altitude zero.
    Input no_baro = in;
    no_baro.pressure_pa = 0;
    assert(hitl::decode_frame(encode(no_baro).data(), hitl::kFrameLength, s));
    assert(!s.barometer_valid && s.barometric_altitude_meters == 0.0F);

    // No fix: satellites still reported, position zero, like a no-fix GGA.
    Input no_fix = in;
    no_fix.quality = 0;
    no_fix.satellites = 3;
    assert(hitl::decode_frame(encode(no_fix).data(), hitl::kFrameLength, s));
    assert(!s.gps_fix && s.satellites == 3 && s.fix_quality == 0);
    assert(s.latitude_degrees == 0.0 && s.longitude_degrees == 0.0 &&
           s.gps_altitude_meters == 0.0F);
    no_fix.quality = 9;  // Outside the NMEA parser's accepted 1..8.
    assert(hitl::decode_frame(encode(no_fix).data(), hitl::kFrameLength, s));
    assert(!s.gps_fix);
    no_fix.quality = 1;
    no_fix.gps_altitude = NAN;
    assert(hitl::decode_frame(encode(no_fix).data(), hitl::kFrameLength, s));
    assert(!s.gps_fix);

    // int24 extremes survive the state-packet re-encoding lround(deg * 1e4).
    for (const std::int32_t raw : {-8388608, -8388607, -1, 0, 1, 900000, 8388607}) {
        Input g = in;
        g.latitude = raw;
        g.longitude = -raw - 1;
        assert(hitl::decode_frame(encode(g).data(), hitl::kFrameLength, s));
        assert(s.latitude_e4 == raw && s.longitude_e4 == -raw - 1);
        assert(std::lround(s.latitude_degrees * IRIS_FIELD_GPS_COORDINATE_SCALE) == raw);
        assert(std::lround(s.longitude_degrees * IRIS_FIELD_GPS_COORDINATE_SCALE) ==
               -raw - 1);
    }

    // Rejections: length, ID, EOF, null.
    auto bad = f;
    assert(!hitl::decode_frame(bad.data(), bad.size() - 1, s));
    bad[0] = IRIS_PACKET_ID_COMMAND;
    assert(!hitl::decode_frame(bad.data(), bad.size(), s));
    bad = f;
    bad[hitl::kFrameLength - 1] = 0x0D;
    assert(!hitl::decode_frame(bad.data(), bad.size(), s));
    assert(!hitl::decode_frame(nullptr, hitl::kFrameLength, s));
}

// Feeds a stream as the USB task does at command boundaries: a 0x0C starts a
// frame when idle; other idle bytes are not framed.
std::vector<std::uint32_t> run_stream(const std::vector<std::uint8_t> &stream,
                                      std::size_t &bad_eofs, std::size_t &unframed)
{
    hitl::FrameAssembler assembler;
    std::vector<std::uint32_t> times;
    bad_eofs = 0;
    unframed = 0;
    for (const std::uint8_t byte : stream) {
        switch (assembler.push(byte)) {
        case hitl::FrameAssembler::Result::Frame: {
            hitl::Sample s;
            assert(hitl::decode_frame(assembler.frame(), hitl::kFrameLength, s));
            times.push_back(s.sim_time_ms);
            break;
        }
        case hitl::FrameAssembler::Result::BadEof: ++bad_eofs; break;
        case hitl::FrameAssembler::Result::NotFramed: ++unframed; break;
        case hitl::FrameAssembler::Result::Pending: break;
        }
    }
    return times;
}

void append(std::vector<std::uint8_t> &stream, const std::vector<std::uint8_t> &f)
{
    stream.insert(stream.end(), f.begin(), f.end());
}

std::vector<std::uint8_t> frame_at(std::uint32_t t)
{
    Input in = typical();
    in.sim_time_ms = t;
    return encode(in);
}

// No 0x0A or 0x0C byte after the ID (for t other than 10 and 12), so the
// resynchronization paths below are deterministic. With no CRC, a resync
// through payload bytes can accept one misaligned frame whose final byte
// happens to be 0x0A; that is a protocol limitation, not tested here.
std::vector<std::uint8_t> clean_frame_at(std::uint32_t t)
{
    Input in = typical();
    in.sim_time_ms = t;
    in.low[2] = 0;
    in.gps_altitude = 512.0F;
    const auto f = encode(in);
    for (std::size_t i = 1; i + 1 < f.size(); ++i) {
        assert(f[i] != IRIS_PACKET_EOF_VALUE && f[i] != IRIS_PACKET_ID_HITL_SENSORS);
    }
    return f;
}

void test_assembler()
{
    std::size_t bad = 0, unframed = 0;

    // Back-to-back frames whose payloads contain 0x0A and 0x0C bytes.
    std::vector<std::uint8_t> stream;
    for (std::uint32_t t = 10; t <= 50; t += 10) append(stream, frame_at(t));
    auto times = run_stream(stream, bad, unframed);
    assert((times == std::vector<std::uint32_t>{10, 20, 30, 40, 50}));
    assert(bad == 0 && unframed == 0);

    // Byte-at-a-time state.
    hitl::FrameAssembler a;
    assert(!a.active());
    assert(a.push('L') == hitl::FrameAssembler::Result::NotFramed && !a.active());
    const auto f = frame_at(7);
    for (std::size_t i = 0; i + 1 < f.size(); ++i) {
        assert(a.push(f[i]) == hitl::FrameAssembler::Result::Pending);
        assert(a.active() && a.length() == i + 1);
    }
    assert(a.push(f.back()) == hitl::FrameAssembler::Result::Frame && !a.active());
    a.push(IRIS_PACKET_ID_HITL_SENSORS);
    a.reset();
    assert(!a.active());

    // Truncated frame (a byte lost): the next frame's ID is the final byte of
    // the misaligned 47, so the stream resynchronizes on it immediately.
    stream.clear();
    append(stream, clean_frame_at(1));
    auto lost = clean_frame_at(2);
    lost.erase(lost.begin() + 20);
    append(stream, lost);
    append(stream, clean_frame_at(3));
    append(stream, clean_frame_at(4));
    times = run_stream(stream, bad, unframed);
    assert((times == std::vector<std::uint32_t>{1, 3, 4}) && bad == 1 && unframed == 0);

    // Corrupt EOF on a frame whose payload holds a 0x0C byte: that 0x0C is
    // tried first and fails, then the following real frame is recovered.
    stream.clear();
    auto corrupt = clean_frame_at(5);
    corrupt[1 + IRIS_PACKET_HITL_LOW_ACCEL_Z_OFFSET + 1] = IRIS_PACKET_ID_HITL_SENSORS;
    corrupt[hitl::kFrameLength - 1] = 0x00;
    append(stream, corrupt);
    append(stream, clean_frame_at(6));
    append(stream, clean_frame_at(7));
    times = run_stream(stream, bad, unframed);
    assert((times == std::vector<std::uint32_t>{6, 7}) && bad == 2 && unframed == 0);

    // A corrupt frame without any 0x0C after its ID returns to idle.
    auto no_id = clean_frame_at(5);
    no_id[hitl::kFrameLength - 1] = 0x00;
    hitl::FrameAssembler b;
    hitl::FrameAssembler::Result last = hitl::FrameAssembler::Result::Pending;
    for (const auto byte : no_id) last = b.push(byte);
    assert(last == hitl::FrameAssembler::Result::BadEof && !b.active());
    assert(b.rejected_eof() == 0x00);

    // Inserted garbage between frames is not framed (the USB task passes it
    // to the command parser instead).
    stream.clear();
    append(stream, frame_at(8));
    stream.push_back('x');
    stream.push_back('\n');
    append(stream, frame_at(9));
    times = run_stream(stream, bad, unframed);
    assert((times == std::vector<std::uint32_t>{8, 9}) && bad == 0 && unframed == 2);
}

void test_controller()
{
    using Result = hitl::Controller::EnterResult;
    hitl::Controller c;
    hitl::Sample s{};
    assert(!c.active() && c.session() == 0);  // Boot: HITL off.

    // Entry rules.
    assert(c.enter(hitl::Source::Radio, true, 0) == Result::RejectedSource);
    assert(c.enter(hitl::Source::Usb, false, 0) == Result::RejectedFlightState);
    assert(!c.active());
    assert(!c.submit(s, 0));  // Inactive: samples ignored.
    assert(!c.take_sample(s));
    assert(!c.exit());

    assert(c.enter(hitl::Source::Usb, true, 1000) == Result::Entered);
    assert(c.active() && c.session() == 1 && !c.has_sample());
    assert(!c.take_sample(s));  // No sample yet: nothing to substitute.
    assert(c.enter(hitl::Source::Usb, false, 1100) == Result::AlreadyActive);
    assert(c.enter(hitl::Source::Radio, true, 1100) == Result::RejectedSource);

    // Timeout from entry when no sample ever arrives.
    assert(!c.expire(1999));
    assert(c.expire(2000) && !c.active());
    assert(!c.expire(2000));  // Reported once.

    // Samples refresh the timeout; each is taken once.
    assert(c.enter(hitl::Source::Usb, true, 3000) == Result::Entered);
    assert(c.session() == 2);
    s.sim_time_ms = 11;
    assert(c.submit(s, 3500));
    assert(c.has_sample() && c.samples_received() == 1);
    hitl::Sample out{};
    assert(c.take_sample(out) && out.sim_time_ms == 11);
    assert(!c.take_sample(out));
    s.sim_time_ms = 12;
    assert(c.submit(s, 4400));
    s.sim_time_ms = 13;
    assert(c.submit(s, 4490));  // Newer sample replaces the untaken one.
    assert(c.take_sample(out) && out.sim_time_ms == 13);
    assert(!c.expire(5489));
    assert(c.expire(5490) && !c.active() && !c.has_sample());

    // Exit by command from any source; pending samples are dropped.
    assert(c.enter(hitl::Source::Usb, true, 6000) == Result::Entered);
    assert(c.submit(s, 6010));
    assert(c.exit() && !c.active() && !c.take_sample(out));
    assert(!c.exit());
    assert(c.samples_received() == 1);

    // Millisecond clock wrap.
    assert(c.enter(hitl::Source::Usb, true, 0xFFFFFF00U) == Result::Entered);
    assert(!c.expire(0x00000100U));             // 512 ms later.
    assert(c.expire(0xFFFFFF00U + 1000U));      // Wrapped, 1000 ms later.
}

std::vector<std::uint8_t> imu_frame(std::uint32_t first_index, std::int16_t base)
{
    std::vector<std::uint8_t> f(hitl::kImuFrameLength, 0);
    f[0] = IRIS_PACKET_ID_HITL_IMU;
    put(f, 0, first_index, 4);
    for (std::size_t i = 0; i < hitl::kImuSamplesPerFrame; ++i) {
        for (std::size_t c = 0; c < 9; ++c) {
            const auto v = static_cast<std::uint16_t>(
                static_cast<std::int16_t>(base + 100 * static_cast<int>(i) + static_cast<int>(c)));
            put(f, 4 + 18 * i + 2 * c, v, 2);
        }
    }
    f.back() = IRIS_PACKET_EOF_VALUE;
    return f;
}

void test_imu_batch()
{
    static_assert(hitl::kImuFrameLength == 150, "0x0D frame is 150 bytes");
    assert(hitl::frame_length_for(IRIS_PACKET_ID_HITL_SENSORS) == 47);
    assert(hitl::frame_length_for(IRIS_PACKET_ID_HITL_IMU) == 150);
    assert(hitl::frame_length_for(0x05) == 0);

    // Decode: index and every channel of every sample, including negatives.
    const auto f = imu_frame(123456U, -200);
    hitl::ImuBatch batch{};
    assert(hitl::decode_imu_frame(f.data(), f.size(), batch));
    assert(batch.first_index == 123456U);
    assert(batch.low_g[0][0] == -200 && batch.low_g[0][2] == -198);
    assert(batch.high_g[3][1] == -200 + 300 + 4);
    assert(batch.gyro[7][2] == -200 + 700 + 8);
    auto bad = f;
    bad.back() = 0;
    assert(!hitl::decode_imu_frame(bad.data(), bad.size(), batch));
    assert(!hitl::decode_imu_frame(f.data(), f.size() - 1, batch));

    // Assembler: 0x0C and 0x0D frames interleaved, each completed at its own
    // length (base 1000 keeps 0x0A/0x0C/0x0E out of the IMU payload bytes).
    std::vector<std::uint8_t> stream;
    append(stream, imu_frame(0, 1000));
    append(stream, clean_frame_at(5));
    append(stream, imu_frame(8, 1000));
    hitl::FrameAssembler a;
    std::vector<std::pair<std::uint8_t, std::size_t>> got;
    for (const auto byte : stream) {
        if (a.push(byte) == hitl::FrameAssembler::Result::Frame) {
            got.emplace_back(a.frame()[0], a.frame_length());
        }
    }
    assert(got.size() == 3);
    assert(got[0].first == IRIS_PACKET_ID_HITL_IMU && got[0].second == 150);
    assert(got[1].first == IRIS_PACKET_ID_HITL_SENSORS && got[1].second == 47);
    assert(got[2].first == IRIS_PACKET_ID_HITL_IMU && got[2].second == 150);

    // A truncated 0x0E followed by a good 0x0C: the 0x0C is recovered by the
    // resync (the bad frame's last byte is not 0x0A).
    stream.clear();
    auto cut = imu_frame(16, 1000);
    cut.resize(60);
    append(stream, cut);
    append(stream, clean_frame_at(7));
    std::vector<std::uint8_t> filler(150 - 60 - 47, 0x55);
    append(stream, filler);
    append(stream, clean_frame_at(9));
    hitl::FrameAssembler b2;
    std::size_t frames = 0, bad_eof = 0;
    for (const auto byte : stream) {
        const auto r = b2.push(byte);
        frames += r == hitl::FrameAssembler::Result::Frame;
        bad_eof += r == hitl::FrameAssembler::Result::BadEof;
    }
    assert(bad_eof >= 1);
    assert(frames >= 1);  // at least the final 0x0C after the junk
}

}  // namespace

int main()
{
    test_decode();
    test_assembler();
    test_controller();
    test_imu_batch();
    {
        // Status bit 0: GPS from the real receiver (RF simulation).
        auto f = encode(typical());
        hitl::Sample s;
        assert(hitl::decode_frame(f.data(), f.size(), s) && !s.gps_from_receiver);
        f[1 + IRIS_PACKET_HITL_STATUS_OFFSET] = IRIS_HITL_STATUS_GPS_FROM_RECEIVER;
        assert(hitl::decode_frame(f.data(), f.size(), s) && s.gps_from_receiver);
    }
    std::puts("HITL input tests passed");
}
