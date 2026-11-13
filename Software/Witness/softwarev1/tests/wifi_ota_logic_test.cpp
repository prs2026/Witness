#include "wifi_ota_logic.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace wifi_ota_logic;

static std::vector<std::uint8_t> make_image(const char *project, std::uint16_t chip = kChipIdEsp32S3)
{
    std::vector<std::uint8_t> image(kImageHeadLength + 64, 0);
    image[0] = kImageMagic;
    image[kImageChipIdOffset] = static_cast<std::uint8_t>(chip);
    image[kImageChipIdOffset + 1] = static_cast<std::uint8_t>(chip >> 8);
    image[kAppDescOffset + 0] = 0x32;
    image[kAppDescOffset + 1] = 0x54;
    image[kAppDescOffset + 2] = 0xCD;
    image[kAppDescOffset + 3] = 0xAB;
    std::strncpy(reinterpret_cast<char *>(&image[kAppDescOffset + kAppDescProjectOffset]), project,
                 kAppDescProjectLength);
    return image;
}

int main()
{
    // Flight-state gate: only Pad Idle and Landed.
    assert(state_allows_wifi(0x00) && state_allows_wifi(0x04));
    for (std::uint8_t s = 1; s <= 3; ++s) assert(!state_allows_wifi(s));
    assert(!state_allows_wifi(0x05));

    // Command parsing: on/off, power default and range, auto-off default.
    {
        const std::uint8_t on_default[3] = {1, 0, 0};
        auto c = parse_wifi_command(on_default, 8, 15);
        assert(c.valid && c.on && c.tx_power_dbm == 8 && c.auto_off_minutes == 15);
        const std::uint8_t on_custom[3] = {1, 12, 30};
        c = parse_wifi_command(on_custom, 8, 15);
        assert(c.valid && c.on && c.tx_power_dbm == 12 && c.auto_off_minutes == 30);
        const std::uint8_t off[3] = {0, 0, 0};
        c = parse_wifi_command(off, 8, 15);
        assert(c.valid && !c.on);
        const std::uint8_t bad_onoff[3] = {2, 0, 0};
        assert(!parse_wifi_command(bad_onoff, 8, 15).valid);
        const std::uint8_t too_high[3] = {1, 21, 0};
        assert(!parse_wifi_command(too_high, 8, 15).valid);
        const std::uint8_t negative[3] = {1, 0xFB, 0};   // -5 dBm
        assert(!parse_wifi_command(negative, 8, 15).valid);
        const std::uint8_t too_long[3] = {1, 0, 250};
        assert(!parse_wifi_command(too_long, 8, 15).valid);
        assert(tx_power_to_wifi_units(8) == 32);
    }

    // SSID from the MAC tail, or the override.
    {
        const std::uint8_t mac[6] = {0x34, 0x85, 0x18, 0xAA, 0x0B, 0xCD};
        char ssid[33];
        assert(format_ssid(ssid, sizeof(ssid), mac, "") == 12);
        assert(std::strcmp(ssid, "witness-0BCD") == 0);
        assert(format_ssid(ssid, sizeof(ssid), mac, nullptr) == 12);
        assert(format_ssid(ssid, sizeof(ssid), mac, "pad-7") == 5);
        assert(std::strcmp(ssid, "pad-7") == 0);
        char small[8];
        assert(format_ssid(small, sizeof(small), mac, "") == 0 && small[0] == '\0');
    }

    // Image header validation.
    {
        auto ok = make_image("softwarev1");
        assert(check_image_head(ok.data(), ok.size(), "softwarev1") == ImageCheck::Ok);
        assert(check_image_head(ok.data(), 100, "softwarev1") == ImageCheck::TooShort);
        assert(check_image_head(nullptr, 0, "softwarev1") == ImageCheck::TooShort);
        auto other = make_image("softwareattempt2");
        assert(check_image_head(other.data(), other.size(), "softwarev1") == ImageCheck::WrongProject);
        auto prefix = make_image("softwarev10");
        assert(check_image_head(prefix.data(), prefix.size(), "softwarev1") == ImageCheck::WrongProject);
        auto c3 = make_image("softwarev1", 5);
        assert(check_image_head(c3.data(), c3.size(), "softwarev1") == ImageCheck::WrongChip);
        auto bad_magic = ok;
        bad_magic[0] = 0x00;
        assert(check_image_head(bad_magic.data(), bad_magic.size(), "softwarev1") == ImageCheck::BadMagic);
        auto no_desc = ok;
        no_desc[kAppDescOffset] = 0;
        assert(check_image_head(no_desc.data(), no_desc.size(), "softwarev1") == ImageCheck::BadDescriptor);
        // A 32-character name with no terminator is not "softwarev1".
        auto full = make_image("softwarev1xxxxxxxxxxxxxxxxxxxxxxx");
        assert(check_image_head(full.data(), full.size(), "softwarev1") == ImageCheck::WrongProject);
    }

    // Log names.
    assert(valid_log_name("I0001.BIN") && valid_log_name("S_2-1.TXT") && valid_log_name("NODOT"));
    assert(!valid_log_name("") && !valid_log_name(nullptr) && !valid_log_name("../x"));
    assert(!valid_log_name("a/b.bin") && !valid_log_name("a..b") && !valid_log_name(".hidden"));
    assert(!valid_log_name("TOOLONGNAME.BINX") && !valid_log_name("a b.bin"));

    // JSON escaping and truncation.
    {
        char out[16];
        assert(json_escape(out, sizeof(out), "a\"b\\c\n") == 13);
        assert(std::strcmp(out, "a\\\"b\\\\c\\u000a") == 0);
        char tiny[4];
        assert(json_escape(tiny, sizeof(tiny), "abcdef") == 3 && std::strcmp(tiny, "abc") == 0);
    }

    std::puts("wifi_ota_logic_test passed");
    return 0;
}
