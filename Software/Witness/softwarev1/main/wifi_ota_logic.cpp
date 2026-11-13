#include "wifi_ota_logic.h"

#include <cstdio>
#include <cstring>

namespace wifi_ota_logic {

WifiCommand parse_wifi_command(const std::uint8_t data[3], const int default_tx_dbm,
                               const std::uint32_t default_auto_off_minutes)
{
    WifiCommand command;
    if (data[0] > 1U) return command;
    command.on = data[0] == 1U;
    const int requested = static_cast<std::int8_t>(data[1]);
    if (requested == 0) {
        command.tx_power_dbm = default_tx_dbm;
    } else if (requested < kMinTxPowerDbm || requested > kMaxTxPowerDbm) {
        return command;
    } else {
        command.tx_power_dbm = requested;
    }
    command.auto_off_minutes = data[2] == 0U ? default_auto_off_minutes : data[2];
    if (command.auto_off_minutes > kMaxAutoOffMinutes) return command;
    command.valid = true;
    return command;
}

std::size_t format_ssid(char *out, const std::size_t cap, const std::uint8_t mac[6],
                        const char *override_ssid)
{
    if (out == nullptr || cap == 0) return 0;
    int n;
    if (override_ssid != nullptr && override_ssid[0] != '\0') {
        n = std::snprintf(out, cap, "%.32s", override_ssid);
    } else {
        n = std::snprintf(out, cap, "witness-%02X%02X", mac[4], mac[5]);
    }
    if (n <= 0 || static_cast<std::size_t>(n) >= cap) {
        out[0] = '\0';
        return 0;
    }
    return static_cast<std::size_t>(n);
}

ImageCheck check_image_head(const std::uint8_t *data, const std::size_t length,
                            const char *project_name)
{
    if (data == nullptr || length < kImageHeadLength) return ImageCheck::TooShort;
    if (data[kImageMagicOffset] != kImageMagic) return ImageCheck::BadMagic;
    const std::uint16_t chip = static_cast<std::uint16_t>(
        data[kImageChipIdOffset] | (data[kImageChipIdOffset + 1] << 8));
    if (chip != kChipIdEsp32S3) return ImageCheck::WrongChip;
    const std::uint8_t *desc = data + kAppDescOffset;
    const std::uint32_t magic = static_cast<std::uint32_t>(desc[0]) |
                                (static_cast<std::uint32_t>(desc[1]) << 8) |
                                (static_cast<std::uint32_t>(desc[2]) << 16) |
                                (static_cast<std::uint32_t>(desc[3]) << 24);
    if (magic != kAppDescMagic) return ImageCheck::BadDescriptor;
    char name[kAppDescProjectLength + 1]{};
    std::memcpy(name, desc + kAppDescProjectOffset, kAppDescProjectLength);
    if (project_name == nullptr || std::strcmp(name, project_name) != 0) {
        return ImageCheck::WrongProject;
    }
    return ImageCheck::Ok;
}

const char *image_check_message(const ImageCheck check)
{
    switch (check) {
    case ImageCheck::Ok: return "ok";
    case ImageCheck::TooShort: return "image too short for a header";
    case ImageCheck::BadMagic: return "not an ESP32 application image";
    case ImageCheck::WrongChip: return "image is for a different chip (need ESP32-S3)";
    case ImageCheck::BadDescriptor: return "application descriptor missing";
    case ImageCheck::WrongProject: return "image is a different project";
    }
    return "invalid image";
}

bool valid_log_name(const char *name)
{
    if (name == nullptr) return false;
    const std::size_t length = std::strlen(name);
    if (length == 0 || length > 12) return false;
    int dots = 0;
    for (std::size_t i = 0; i < length; ++i) {
        const char c = name[i];
        if (c == '.') {
            if (++dots > 1 || i == 0) return false;
        } else if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') ||
                     (c >= 'a' && c <= 'z') || c == '_' || c == '-')) {
            return false;
        }
    }
    return true;
}

std::size_t json_escape(char *out, const std::size_t cap, const char *in)
{
    if (out == nullptr || cap == 0) return 0;
    std::size_t w = 0;
    for (; in != nullptr && *in != '\0'; ++in) {
        const unsigned char c = static_cast<unsigned char>(*in);
        char piece[8];
        std::size_t n;
        if (c == '"' || c == '\\') {
            piece[0] = '\\';
            piece[1] = static_cast<char>(c);
            n = 2;
        } else if (c < 0x20) {
            n = static_cast<std::size_t>(std::snprintf(piece, sizeof(piece), "\\u%04x", c));
        } else {
            piece[0] = static_cast<char>(c);
            n = 1;
        }
        if (w + n >= cap) break;
        std::memcpy(out + w, piece, n);
        w += n;
    }
    out[w] = '\0';
    return w;
}

}  // namespace wifi_ota_logic
