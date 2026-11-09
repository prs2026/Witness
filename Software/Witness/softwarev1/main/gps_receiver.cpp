#include "gps_receiver.h"

namespace {

// ---- SkyTraq PX1120S (AN0037) ------------------------------------------------

constexpr std::uint8_t kSkytraqAck = 0x83;
constexpr std::uint8_t kSkytraqNack = 0x84;

struct SkytraqCommand {
    const char *name;
    std::uint8_t payload[9];
    std::uint8_t length;
};

// Each command is acknowledged by 83 <id> (plus sub-id for 0x64 commands).
// Settings are written to SRAM only (final attribute byte 0).
constexpr SkytraqCommand kSkytraqConfigure[] = {
    // 0x64/0x17 navigation mode: airborne.
    {"airborne navigation mode",
     {0x64, 0x17, GpsReceiver::kSkytraqAirborneMode, 0x00}, 4},
    // 0x08 NMEA intervals per fix: GGA GSA GSV GLL RMC VTG ZDA, attributes.
    // Reduce output to GGA + RMC before raising the rate, so the stream fits
    // 115200 baud at the maximum update rate.
    {"NMEA output GGA+RMC",
     {0x08, 1, 0, 0, 0, 1, 0, 0, 0x00}, 9},
    // 0x0E position update rate.
    {"position update rate",
     {0x0E, GpsReceiver::kSkytraqUpdateRateHz, 0x00}, 3},
};

constexpr SkytraqCommand kSkytraqVerify[] = {
    {"navigation mode readback", {0x64, 0x18}, 2},  // reply 64 8B <mode>
    {"update rate readback", {0x10}, 1},            // reply 86 <rate>
};

// ---- u-blox M8 (UBX) ---------------------------------------------------------

constexpr std::uint8_t kUbxClassAck = 0x05;
constexpr std::uint8_t kUbxIdAck = 0x01;
constexpr std::uint8_t kUbxIdNak = 0x00;
constexpr std::uint8_t kUbxClassCfg = 0x06;
constexpr std::uint8_t kUbxIdCfgMsg = 0x01;
constexpr std::uint8_t kUbxIdCfgPrt = 0x00;
constexpr std::uint8_t kUbxIdCfgRate = 0x08;
constexpr std::uint8_t kUbxIdCfgNav5 = 0x24;
constexpr std::uint8_t kUbxClassMon = 0x0A;
constexpr std::uint8_t kUbxIdMonVer = 0x04;
constexpr std::size_t kUbxNav5Length = 36;
constexpr std::size_t kUbxRateLength = 6;

struct UbxCommand {
    const char *name;
    std::uint8_t message_class;
    std::uint8_t id;
    std::uint8_t payload[kUbxNav5Length];
    std::uint8_t length;
};

constexpr std::uint8_t kNmeaClass = 0xF0;

// CFG-MSG (3-byte form) sets the rate on the port the command arrives on.
// CFG-NAV5 mask 0x0001 applies only the dynamic model. CFG-RATE sets the
// measurement period, one navigation solution per measurement, GPS time.
constexpr UbxCommand kUbxConfigure[] = {
    {"disable NMEA GLL", kUbxClassCfg, kUbxIdCfgMsg, {kNmeaClass, 0x01, 0}, 3},
    {"disable NMEA GSA", kUbxClassCfg, kUbxIdCfgMsg, {kNmeaClass, 0x02, 0}, 3},
    {"disable NMEA GSV", kUbxClassCfg, kUbxIdCfgMsg, {kNmeaClass, 0x03, 0}, 3},
    {"disable NMEA VTG", kUbxClassCfg, kUbxIdCfgMsg, {kNmeaClass, 0x05, 0}, 3},
    {"enable NMEA GGA", kUbxClassCfg, kUbxIdCfgMsg, {kNmeaClass, 0x00, 1}, 3},
    {"enable NMEA RMC", kUbxClassCfg, kUbxIdCfgMsg, {kNmeaClass, 0x04, 1}, 3},
    {"airborne <4 g dynamic model", kUbxClassCfg, kUbxIdCfgNav5,
     {0x01, 0x00, GpsReceiver::kUbloxAirborne4gModel}, kUbxNav5Length},
    {"navigation rate", kUbxClassCfg, kUbxIdCfgRate,
     {static_cast<std::uint8_t>(GpsReceiver::kUbloxMeasurementPeriodMs & 0xFF),
      static_cast<std::uint8_t>(GpsReceiver::kUbloxMeasurementPeriodMs >> 8),
      0x01, 0x00, 0x01, 0x00},
     kUbxRateLength},
};

// Polls: an empty CFG payload returns the current configuration.
constexpr UbxCommand kUbxVerify[] = {
    {"dynamic model readback", kUbxClassCfg, kUbxIdCfgNav5, {}, 0},
    {"navigation rate readback", kUbxClassCfg, kUbxIdCfgRate, {}, 0},
};

// CFG-PRT for UART1: 8N1, operational baud, UBX + NMEA in and out.
constexpr std::uint8_t kUbxPrtUart1[20] = {
    0x01, 0x00, 0x00, 0x00,
    0xD0, 0x08, 0x00, 0x00,
    static_cast<std::uint8_t>(GpsReceiver::kOperationalBaud & 0xFF),
    static_cast<std::uint8_t>((GpsReceiver::kOperationalBaud >> 8) & 0xFF),
    static_cast<std::uint8_t>((GpsReceiver::kOperationalBaud >> 16) & 0xFF),
    static_cast<std::uint8_t>((GpsReceiver::kOperationalBaud >> 24) & 0xFF),
    0x03, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x00,
};

template <typename T, std::size_t N>
constexpr std::size_t count_of(const T (&)[N]) { return N; }

}  // namespace

const char *GpsReceiver::model_name() const
{
    switch (model_) {
        case Model::Px1120s: return "PX1120S";
        case Model::UbloxM8: return "u-blox M8";
        default: return "unknown GPS";
    }
}

std::uint32_t GpsReceiver::update_rate_hz() const
{
    switch (model_) {
        case Model::Px1120s: return kSkytraqUpdateRateHz;
        case Model::UbloxM8: return 1000U / kUbloxMeasurementPeriodMs;
        default: return 0;
    }
}

bool GpsReceiver::send_skytraq(const std::uint8_t *payload, const std::size_t length)
{
    std::array<std::uint8_t, 16> packet{};
    if (length == 0 || length + 7 > packet.size()) return false;
    packet[0] = 0xA0;
    packet[1] = 0xA1;
    packet[2] = 0x00;
    packet[3] = static_cast<std::uint8_t>(length);
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

bool GpsReceiver::send_ubx(const std::uint8_t message_class, const std::uint8_t id,
                           const std::uint8_t *payload, const std::size_t length)
{
    std::array<std::uint8_t, 8 + kUbxNav5Length> packet{};
    if (length > kUbxNav5Length) return false;
    packet[0] = 0xB5;
    packet[1] = 0x62;
    packet[2] = message_class;
    packet[3] = id;
    packet[4] = static_cast<std::uint8_t>(length & 0xFF);
    packet[5] = static_cast<std::uint8_t>(length >> 8);
    for (std::size_t i = 0; i < length; ++i) packet[6 + i] = payload[i];
    std::uint8_t ck_a = 0;
    std::uint8_t ck_b = 0;
    for (std::size_t i = 2; i < 6 + length; ++i) {
        ck_a = static_cast<std::uint8_t>(ck_a + packet[i]);
        ck_b = static_cast<std::uint8_t>(ck_b + ck_a);
    }
    packet[6 + length] = ck_a;
    packet[7 + length] = ck_b;
    return transport_.send(transport_.context, packet.data(), length + 8);
}

void GpsReceiver::begin_step(const Step step, const std::uint32_t now_ms)
{
    step_ = step;
    step_time_ms_ = now_ms;
    index_ = 0;
    attempts_ = 0;
    waiting_ = false;
}

void GpsReceiver::fail(const char *reason)
{
    failure_reason_ = reason;
    status_ = Status::Failed;
    // An unidentified receiver leaves the host at the operational rate so raw
    // logging continues where a receiver is most likely to be talking.
    if (model_ == Model::Unknown && baud_ != kOperationalBaud && transport_.set_baud) {
        if (transport_.set_baud(transport_.context, kOperationalBaud))
            baud_ = kOperationalBaud;
    }
}

void GpsReceiver::identified(const Model model, const std::uint32_t now_ms)
{
    model_ = model;
    if (model == Model::UbloxM8 && baud_ != kOperationalBaud)
        begin_step(Step::UbloxSwitchBaud, now_ms);
    else
        begin_step(Step::Configure, now_ms);
}

std::size_t GpsReceiver::configure_count() const
{
    return model_ == Model::UbloxM8 ? count_of(kUbxConfigure) : count_of(kSkytraqConfigure);
}

std::size_t GpsReceiver::verify_count() const
{
    return model_ == Model::UbloxM8 ? count_of(kUbxVerify) : count_of(kSkytraqVerify);
}

void GpsReceiver::advance(const std::uint32_t now_ms)
{
    ++index_;
    attempts_ = 0;
    waiting_ = false;
    if (step_ == Step::Configure && index_ >= configure_count()) {
        begin_step(Step::Verify, now_ms);
    } else if (step_ == Step::Verify && index_ >= verify_count()) {
        status_ = Status::Ready;
    }
}

bool GpsReceiver::send_current(const std::uint32_t now_ms)
{
    (void)now_ms;
    switch (step_) {
        case Step::Identify: {
            // Both polls are ignored by the other receiver family.
            const std::uint8_t skytraq_query[] = {0x64, 0x18};
            const bool skytraq = send_skytraq(skytraq_query, sizeof(skytraq_query));
            const bool ubx = send_ubx(kUbxClassMon, kUbxIdMonVer, nullptr, 0);
            return skytraq && ubx;
        }
        case Step::UbloxConfirmBaud:
            return send_ubx(kUbxClassMon, kUbxIdMonVer, nullptr, 0);
        case Step::Configure:
            if (model_ == Model::UbloxM8) {
                const auto &command = kUbxConfigure[index_];
                return send_ubx(command.message_class, command.id,
                                command.payload, command.length);
            } else {
                const auto &command = kSkytraqConfigure[index_];
                return send_skytraq(command.payload, command.length);
            }
        case Step::Verify:
            if (model_ == Model::UbloxM8) {
                const auto &command = kUbxVerify[index_];
                return send_ubx(command.message_class, command.id,
                                command.payload, command.length);
            } else {
                const auto &command = kSkytraqVerify[index_];
                return send_skytraq(command.payload, command.length);
            }
        default:
            return false;
    }
}

void GpsReceiver::service(const std::uint32_t now_ms)
{
    if (status_ != Status::Configuring) return;
    if (!started_) {
        started_ = true;
        step_time_ms_ = now_ms;
    }

    if (step_ == Step::BootDelay) {
        if (now_ms - step_time_ms_ >= kStartupDelayMs)
            begin_step(Step::Identify, now_ms);
        return;
    }

    if (step_ == Step::UbloxSwitchBaud) {
        if (!waiting_) {
            if (!send_ubx(kUbxClassCfg, kUbxIdCfgPrt, kUbxPrtUart1, sizeof(kUbxPrtUart1))) {
                fail("GPS command transmission failed");
                return;
            }
            waiting_ = true;
            sent_time_ms_ = now_ms;
            return;
        }
        if (now_ms - sent_time_ms_ < kBaudSwitchDelayMs) return;
        if (!transport_.set_baud ||
            !transport_.set_baud(transport_.context, kOperationalBaud)) {
            fail("could not change host UART baud");
            return;
        }
        baud_ = kOperationalBaud;
        nmea_seen_ = false;
        begin_step(Step::UbloxConfirmBaud, now_ms);
        return;
    }

    if (waiting_ && now_ms - sent_time_ms_ < kReplyTimeoutMs) return;

    if (attempts_ >= kMaximumAttempts) {
        switch (step_) {
            case Step::Identify:
                if (baud_ == kOperationalBaud && nmea_seen_) {
                    // Valid NMEA but no binary reply: keep the stream untouched
                    // and treat it as the PX1120S, whose mode query has been
                    // seen to go unanswered. Mode readback still decides.
                    model_assumed_ = true;
                    identified(Model::Px1120s, now_ms);
                } else if (baud_ == kOperationalBaud && transport_.set_baud) {
                    // A silent link: try the u-blox factory rate.
                    if (!transport_.set_baud(transport_.context, kUbloxFactoryBaud)) {
                        fail("could not change host UART baud");
                        return;
                    }
                    baud_ = kUbloxFactoryBaud;
                    nmea_seen_ = false;
                    begin_step(Step::Identify, now_ms);
                } else {
                    fail("no GPS replied at 115200 or 9600 baud");
                }
                return;
            case Step::UbloxConfirmBaud:
                // The receiver most likely kept its old rate; follow it there.
                if (transport_.set_baud &&
                    transport_.set_baud(transport_.context, kUbloxFactoryBaud))
                    baud_ = kUbloxFactoryBaud;
                fail("u-blox did not answer after switching to 115200 baud");
                return;
            case Step::Configure:
                if (model_ == Model::Px1120s) {
                    // Some PX1120S firmware misses an ACK; readback is decisive.
                    advance(now_ms);
                } else {
                    fail("u-blox did not acknowledge configuration");
                }
                return;
            case Step::Verify:
                fail(model_ == Model::UbloxM8 ? "u-blox configuration readback timed out"
                                              : "PX1120S configuration readback timed out");
                return;
            default:
                return;
        }
    }

    if (!send_current(now_ms)) {
        fail("GPS command transmission failed");
        return;
    }
    ++attempts_;
    waiting_ = true;
    sent_time_ms_ = now_ms;
}

void GpsReceiver::consume(const std::uint8_t *data, const std::size_t length,
                          const std::uint32_t now_ms)
{
    if (status_ != Status::Configuring) return;
    if ((skytraq_used_ || ubx_state_) && now_ms - last_byte_ms_ > kFrameIdleTimeoutMs) {
        skytraq_used_ = skytraq_expected_ = 0;
        ubx_state_ = 0;
    }
    last_byte_ms_ = now_ms;
    for (std::size_t i = 0; i < length && status_ == Status::Configuring; ++i) {
        consume_skytraq(data[i], now_ms);
        consume_ubx(data[i], now_ms);
    }
}

void GpsReceiver::consume_skytraq(const std::uint8_t byte, const std::uint32_t now_ms)
{
    if (skytraq_used_ == 0) {
        if (byte == 0xA0) skytraq_[skytraq_used_++] = byte;
        return;
    }
    if (skytraq_used_ == 1 && byte != 0xA1) {
        skytraq_used_ = byte == 0xA0 ? 1 : 0;
        return;
    }
    skytraq_[skytraq_used_++] = byte;
    if (skytraq_used_ == 4) {
        const std::size_t payload_size = (skytraq_[2] << 8U) | skytraq_[3];
        if (payload_size == 0 || payload_size > skytraq_.size() - 7) {
            skytraq_used_ = skytraq_expected_ = 0;
            return;
        }
        skytraq_expected_ = payload_size + 7;
    }
    if (!skytraq_expected_ || skytraq_used_ != skytraq_expected_) return;
    std::uint8_t checksum = 0;
    for (std::size_t j = 4; j < skytraq_used_ - 3; ++j) checksum ^= skytraq_[j];
    const bool valid = checksum == skytraq_[skytraq_used_ - 3] &&
                       skytraq_[skytraq_used_ - 2] == 0x0D &&
                       skytraq_[skytraq_used_ - 1] == 0x0A;
    const std::size_t payload_length = skytraq_expected_ - 7;
    skytraq_used_ = skytraq_expected_ = 0;
    if (valid) handle_skytraq(skytraq_.data() + 4, payload_length, now_ms);
}

void GpsReceiver::consume_ubx(const std::uint8_t byte, const std::uint32_t now_ms)
{
    const auto add = [this](std::uint8_t value) {
        ubx_ck_a_ = static_cast<std::uint8_t>(ubx_ck_a_ + value);
        ubx_ck_b_ = static_cast<std::uint8_t>(ubx_ck_b_ + ubx_ck_a_);
    };
    switch (ubx_state_) {
        case 0:
            if (byte == 0xB5) ubx_state_ = 1;
            return;
        case 1:
            ubx_state_ = byte == 0x62 ? 2 : (byte == 0xB5 ? 1 : 0);
            ubx_ck_a_ = ubx_ck_b_ = 0;
            return;
        case 2:
            ubx_class_ = byte;
            add(byte);
            ubx_state_ = 3;
            return;
        case 3:
            ubx_id_ = byte;
            add(byte);
            ubx_state_ = 4;
            return;
        case 4:
            ubx_length_ = byte;
            add(byte);
            ubx_state_ = 5;
            return;
        case 5:
            ubx_length_ |= static_cast<std::size_t>(byte) << 8U;
            add(byte);
            ubx_state_ = ubx_length_ > kUbloxMaximumPayload ? 0 : 6;
            return;
        default:
            break;
    }
    const std::size_t offset = ubx_state_ - 6;
    if (offset < ubx_length_) {
        if (offset < ubx_payload_.size()) ubx_payload_[offset] = byte;
        add(byte);
        ++ubx_state_;
    } else if (offset == ubx_length_) {
        ubx_received_ck_a_ = byte;
        ++ubx_state_;
    } else {
        const bool valid = ubx_received_ck_a_ == ubx_ck_a_ && byte == ubx_ck_b_;
        ubx_state_ = 0;
        if (valid) {
            const std::size_t stored = ubx_length_ < ubx_payload_.size()
                                           ? ubx_length_ : ubx_payload_.size();
            handle_ubx(ubx_class_, ubx_id_, ubx_payload_.data(), stored, now_ms);
        }
    }
}

void GpsReceiver::handle_skytraq(const std::uint8_t *payload, const std::size_t length,
                                 const std::uint32_t now_ms)
{
    if (!waiting_ || length == 0) return;
    if (step_ == Step::Identify) {
        // Any checksum-valid SkyTraq frame (mode reply, ACK or NACK) identifies it.
        identified(Model::Px1120s, now_ms);
        return;
    }
    if (model_ != Model::Px1120s) return;
    if (step_ == Step::Configure) {
        const auto &command = kSkytraqConfigure[index_];
        const std::uint8_t id = command.payload[0];
        const bool matches = length >= 2 && payload[1] == id &&
                             (id != 0x64 || length == 2 || payload[2] == command.payload[1]);
        if (!matches) return;
        if (payload[0] == kSkytraqNack) fail("PX1120S rejected a configuration command");
        else if (payload[0] == kSkytraqAck) advance(now_ms);
        return;
    }
    if (step_ != Step::Verify) return;
    if (index_ == 0 && length == 3 && payload[0] == 0x64 && payload[1] == 0x8B) {
        if (payload[2] == kSkytraqAirborneMode) advance(now_ms);
        else fail("PX1120S navigation readback did not match airborne mode");
    } else if (index_ == 1 && length == 2 && payload[0] == 0x86) {
        if (payload[1] == kSkytraqUpdateRateHz) advance(now_ms);
        else fail("PX1120S update-rate readback did not match");
    } else if (length >= 2 && payload[0] == kSkytraqNack &&
               payload[1] == kSkytraqVerify[index_].payload[0]) {
        fail("PX1120S rejected a configuration query");
    }
}

void GpsReceiver::handle_ubx(const std::uint8_t message_class, const std::uint8_t id,
                             const std::uint8_t *payload, const std::size_t length,
                             const std::uint32_t now_ms)
{
    if (!waiting_) return;
    if (step_ == Step::Identify) {
        // Any checksum-valid UBX frame identifies a u-blox receiver.
        identified(Model::UbloxM8, now_ms);
        return;
    }
    if (step_ == Step::UbloxConfirmBaud) {
        begin_step(Step::Configure, now_ms);
        return;
    }
    if (model_ != Model::UbloxM8) return;
    if (step_ == Step::Configure) {
        const auto &command = kUbxConfigure[index_];
        if (message_class != kUbxClassAck || length < 2 ||
            payload[0] != command.message_class || payload[1] != command.id)
            return;
        if (id == kUbxIdAck) advance(now_ms);
        else if (id == kUbxIdNak) fail("u-blox rejected a configuration command (NAK)");
        return;
    }
    if (step_ != Step::Verify || message_class != kUbxClassCfg) return;
    if (index_ == 0 && id == kUbxIdCfgNav5 && length >= 3) {
        if (payload[2] == kUbloxAirborne4gModel) advance(now_ms);
        else fail("u-blox dynamic model readback did not match airborne <4 g");
    } else if (index_ == 1 && id == kUbxIdCfgRate && length >= 2) {
        const std::uint16_t period = static_cast<std::uint16_t>(payload[0] | (payload[1] << 8U));
        if (period == kUbloxMeasurementPeriodMs) advance(now_ms);
        else fail("u-blox navigation rate readback did not match");
    }
}
