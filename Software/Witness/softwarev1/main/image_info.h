#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "comms.h"

// Image-info record embedded in BOTH Witness images (flight and maintenance)
// and the pure rules built on it. No ESP-IDF dependency (host test
// tests/image_info_test.cpp).
//
// The record sits in the ".rodata_custom_desc" section, which the ESP-IDF
// linker script places directly behind esp_app_desc_t, so it is at a fixed
// offset in every app image: 24-byte image header + 8-byte first segment
// header + 256-byte esp_app_desc_t = 288. A running image reads the other
// slot's record with esp_partition_read; the OTA handler reads it from the
// first bytes of an upload.
namespace image_info {

constexpr std::uint32_t kMagic = 0x474D4957U;          // "WIMG" little-endian
constexpr std::size_t kOffset = 32 + 256;             // behind esp_app_desc_t
constexpr std::size_t kHeadLength = kOffset + 48;      // bytes needed to judge an image
constexpr std::size_t kAppDescVersionOffset = 32 + 16; // esp_app_desc_t.version[32] in the image
constexpr std::size_t kAppDescVersionLength = 32;

struct Info {
    std::uint32_t magic;
    std::uint8_t struct_version;
    std::uint8_t kind;                  // IRIS_IMAGE_KIND_*
    std::uint8_t store_version;         // newest FlashLogger format this image reads/writes
    std::uint8_t store_min_version;     // oldest format it can read
    std::uint8_t command_set_version;
    std::uint8_t reserved0;
    std::uint8_t radio_sf;
    std::uint8_t radio_cr;
    std::uint32_t radio_freq_hz;
    std::uint32_t radio_bw_hz;
    std::uint16_t radio_preamble;
    std::uint16_t radio_sync_word;
    std::uint32_t build_id;             // build time, seconds since the Unix epoch
    std::uint8_t radio_settings_version;
    std::uint8_t reserved1[19];
};
static_assert(sizeof(Info) == 48, "image info layout is part of the flash format");

// Why a switch to the maintenance image is (not) allowed. Values map to the
// IRIS_MAINT_REFUSE_* ack detail codes through refuse_code().
enum class Compat : std::uint8_t {
    Ok = 0,
    NoImage,
    BadInfo,
    InfoVersion,
    StoreVersion,
    RadioSettings,
    CommandSet,
};

constexpr std::uint16_t refuse_code(const Compat c)
{
    switch (c) {
    case Compat::Ok: return IRIS_MAINT_REFUSE_NONE;
    case Compat::NoImage: return IRIS_MAINT_REFUSE_NO_IMAGE;
    case Compat::BadInfo: return IRIS_MAINT_REFUSE_BAD_INFO;
    case Compat::InfoVersion: return IRIS_MAINT_REFUSE_INFO_VERSION;
    case Compat::StoreVersion: return IRIS_MAINT_REFUSE_STORE_VERSION;
    case Compat::RadioSettings: return IRIS_MAINT_REFUSE_RADIO_SETTINGS;
    case Compat::CommandSet: return IRIS_MAINT_REFUSE_COMMAND_SET;
    }
    return IRIS_MAINT_REFUSE_BAD_INFO;
}

// The record this build carries (settings come from the build's constants).
constexpr Info make_info(const std::uint8_t kind, const std::uint32_t build_id,
                         const std::uint32_t radio_freq_hz, const std::uint32_t radio_bw_hz,
                         const std::uint8_t sf, const std::uint8_t cr, const std::uint16_t preamble)
{
    Info info{};
    info.magic = kMagic;
    info.struct_version = IRIS_MAINT_INFO_STRUCT_VERSION;
    info.kind = kind;
    info.store_version = IRIS_MAINT_STORE_FORMAT_VERSION;
    info.store_min_version = IRIS_MAINT_STORE_FORMAT_MIN_VERSION;
    info.command_set_version = IRIS_MAINT_COMMAND_SET_VERSION;
    info.radio_sf = sf;
    info.radio_cr = cr;
    info.radio_freq_hz = radio_freq_hz;
    info.radio_bw_hz = radio_bw_hz;
    info.radio_preamble = preamble;
    info.radio_sync_word = IRIS_MAINT_RADIO_SYNC_WORD;
    info.build_id = build_id;
    info.radio_settings_version = IRIS_MAINT_RADIO_SETTINGS_VERSION;
    return info;
}

// Parses the record from the first bytes of an app image (at least
// kHeadLength). False if too short or the magic is missing.
inline bool parse(const std::uint8_t *image, const std::size_t length, Info &out)
{
    if (image == nullptr || length < kOffset + sizeof(Info)) return false;
    std::memcpy(&out, image + kOffset, sizeof(Info));
    return out.magic == kMagic;
}

// May the flight image `self` hand over to the maintenance image `maint`?
// `maint` is nullptr when the slot holds no readable record.
inline Compat check_maintenance(const Info *maint, const Info &self)
{
    if (maint == nullptr) return Compat::NoImage;
    if (maint->magic != kMagic || maint->kind != IRIS_IMAGE_KIND_MAINTENANCE) return Compat::BadInfo;
    if (maint->struct_version != self.struct_version) return Compat::InfoVersion;
    if (self.store_version < maint->store_min_version || self.store_version > maint->store_version) {
        return Compat::StoreVersion;
    }
    if (maint->radio_settings_version != self.radio_settings_version ||
        maint->radio_freq_hz != self.radio_freq_hz || maint->radio_bw_hz != self.radio_bw_hz ||
        maint->radio_sf != self.radio_sf || maint->radio_cr != self.radio_cr ||
        maint->radio_preamble != self.radio_preamble || maint->radio_sync_word != self.radio_sync_word) {
        return Compat::RadioSettings;
    }
    if (maint->command_set_version != self.command_set_version) return Compat::CommandSet;
    return Compat::Ok;
}

// One app slot as seen by slot selection.
struct Slot {
    bool present;        // partition exists
    bool has_info;       // readable record of the expected kind
    Info info;
};

// Maintenance slot to boot. `confirmed_build` is the build id of the newest
// maintenance image that has booted and passed its self-test (0 = none known,
// e.g. a freshly USB-flashed board). A slot is eligible when it has a valid
// record and is not newer than the confirmed build; the newest eligible slot
// wins. A freshly uploaded image newer than the confirmed one is booted only
// by the maintenance OTA handler's explicit verification boot (see
// boot_manager.h), so a broken upload is never what a pad-side switch lands in
// while a confirmed image exists. If nothing is eligible (a USB re-flash with
// a newer image than the recorded confirmation, or the confirmed slot is
// gone) the newest valid slot is used; it confirms itself when it boots.
// Returns -1 if no slot holds a valid record.
inline int select_maintenance_slot(const Slot slots[2], const std::uint32_t confirmed_build)
{
    int best = -1;
    int newest = -1;
    for (int i = 0; i < 2; ++i) {
        if (!slots[i].present || !slots[i].has_info) continue;
        if (newest < 0 || slots[i].info.build_id > slots[newest].info.build_id) newest = i;
        if (confirmed_build != 0 && slots[i].info.build_id > confirmed_build) continue;
        if (best < 0 || slots[i].info.build_id > slots[best].info.build_id) best = i;
    }
    return best >= 0 ? best : newest;
}

// Flight slot a maintenance boot returns to: the slot recorded as last good
// (`last_good`, 0/1, or -1 unknown), if it still holds a valid image;
// otherwise the newest valid flight image; -1 if none.
inline int select_flight_slot(const Slot slots[2], const int last_good)
{
    if (last_good >= 0 && last_good < 2 && slots[last_good].present && slots[last_good].has_info) {
        return last_good;
    }
    int best = -1;
    for (int i = 0; i < 2; ++i) {
        if (!slots[i].present || !slots[i].has_info) continue;
        if (best < 0 || slots[i].info.build_id > slots[best].info.build_id) best = i;
    }
    return best;
}

// Slot an OTA writes within a kind: the one that is not `current` (0/1).
constexpr int inactive_slot(const int current) { return current == 0 ? 1 : 0; }

// Short image id for the beacon: FNV-1a over a version string, folded to 16
// bits, never 0 for a non-empty string.
inline std::uint16_t version_id(const char *text, const std::size_t max_length)
{
    if (text == nullptr) return 0;
    std::uint32_t hash = 2166136261U;
    std::size_t n = 0;
    for (; n < max_length && text[n] != '\0'; ++n) {
        hash ^= static_cast<std::uint8_t>(text[n]);
        hash *= 16777619U;
    }
    if (n == 0) return 0;
    const std::uint16_t folded = static_cast<std::uint16_t>((hash >> 16) ^ (hash & 0xFFFFU));
    return folded == 0 ? 1 : folded;
}

// What an uploaded image is, judged from its first bytes (OTA handler).
enum class UploadKind : std::uint8_t { Flight, Maintenance, Unknown };
inline UploadKind upload_kind(const std::uint8_t *head, const std::size_t length)
{
    Info info{};
    if (!parse(head, length, info)) return UploadKind::Unknown;
    if (info.kind == IRIS_IMAGE_KIND_FLIGHT) return UploadKind::Flight;
    if (info.kind == IRIS_IMAGE_KIND_MAINTENANCE) return UploadKind::Maintenance;
    return UploadKind::Unknown;
}

}  // namespace image_info
