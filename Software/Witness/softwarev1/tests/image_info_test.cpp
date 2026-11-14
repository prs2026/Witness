#include "image_info.h"
#include "maint_beacon.h"

#include <cassert>
#include <cstring>

using namespace image_info;

static Info flight_info()
{
    return make_info(IRIS_IMAGE_KIND_FLIGHT, 1000, 926000000U, 500000U, 7, 1, 8);
}
static Info maint_info(const std::uint32_t build = 2000)
{
    return make_info(IRIS_IMAGE_KIND_MAINTENANCE, build, 926000000U, 500000U, 7, 1, 8);
}

int main()
{
    // --- compatibility decision ---
    const Info flight = flight_info();
    Info maint = maint_info();
    assert(check_maintenance(&maint, flight) == Compat::Ok);
    assert(check_maintenance(nullptr, flight) == Compat::NoImage);
    Info bad = maint;
    bad.magic = 0;
    assert(check_maintenance(&bad, flight) == Compat::BadInfo);
    bad = flight_info();   // wrong kind
    assert(check_maintenance(&bad, flight) == Compat::BadInfo);
    bad = maint;
    bad.struct_version = 9;
    assert(check_maintenance(&bad, flight) == Compat::InfoVersion);
    // Storage: maintenance reads MIN..current; the flight version must be inside.
    bad = maint;
    bad.store_version = flight.store_version - 1;   // maintenance too old for a newer flight store
    assert(check_maintenance(&bad, flight) == Compat::StoreVersion);
    bad = maint;
    bad.store_min_version = flight.store_version + 1;
    assert(check_maintenance(&bad, flight) == Compat::StoreVersion);
    bad = maint;
    bad.store_version = flight.store_version + 1;   // newer maintenance still reads the older store
    assert(check_maintenance(&bad, flight) == Compat::Ok);
    bad = maint;
    bad.radio_sf = 8;
    assert(check_maintenance(&bad, flight) == Compat::RadioSettings);
    bad = maint;
    bad.radio_freq_hz += 1;
    assert(check_maintenance(&bad, flight) == Compat::RadioSettings);
    bad = maint;
    bad.radio_settings_version += 1;
    assert(check_maintenance(&bad, flight) == Compat::RadioSettings);
    bad = maint;
    bad.command_set_version += 1;
    assert(check_maintenance(&bad, flight) == Compat::CommandSet);
    assert(refuse_code(Compat::NoImage) == IRIS_MAINT_REFUSE_NO_IMAGE);
    assert(refuse_code(Compat::RadioSettings) == IRIS_MAINT_REFUSE_RADIO_SETTINGS);
    assert(refuse_code(Compat::Ok) == IRIS_MAINT_REFUSE_NONE);

    // --- maintenance slot selection ---
    Slot slots[2]{};
    assert(select_maintenance_slot(slots, 0) == -1);
    slots[0] = {true, true, maint_info(2000)};
    assert(select_maintenance_slot(slots, 0) == 0);       // fresh USB flash: nothing confirmed yet
    slots[1] = {true, true, maint_info(3000)};
    assert(select_maintenance_slot(slots, 0) == 1);       // none confirmed: newest
    assert(select_maintenance_slot(slots, 2000) == 0);    // 3000 is unconfirmed: skipped
    assert(select_maintenance_slot(slots, 3000) == 1);
    slots[1].has_info = false;                             // corrupt slot ignored
    assert(select_maintenance_slot(slots, 3000) == 0);
    slots[0].present = false;
    assert(select_maintenance_slot(slots, 3000) == -1);
    slots[0] = {true, true, maint_info(5000)};            // only newer than confirmed
    slots[1] = {true, false, {}};
    assert(select_maintenance_slot(slots, 4000) == 0);    // nothing eligible: newest valid (re-flash case)
    slots[0].has_info = false;
    assert(select_maintenance_slot(slots, 4000) == -1);

    // --- flight slot selection ---
    Slot f[2]{};
    f[0] = {true, true, make_info(IRIS_IMAGE_KIND_FLIGHT, 10, 1, 1, 7, 1, 8)};
    f[1] = {true, true, make_info(IRIS_IMAGE_KIND_FLIGHT, 20, 1, 1, 7, 1, 8)};
    assert(select_flight_slot(f, 0) == 0);    // recorded last-good wins over newer
    assert(select_flight_slot(f, 1) == 1);
    assert(select_flight_slot(f, -1) == 1);   // unknown: newest
    f[0].has_info = false;
    assert(select_flight_slot(f, 0) == 1);    // last-good invalid: fall back
    f[1].present = false;
    assert(select_flight_slot(f, 0) == -1);
    assert(inactive_slot(0) == 1 && inactive_slot(1) == 0);

    // --- info record location and upload kind ---
    std::uint8_t image[kHeadLength] = {};
    assert(upload_kind(image, sizeof(image)) == UploadKind::Unknown);
    const Info m = maint_info();
    std::memcpy(image + kOffset, &m, sizeof(m));
    assert(upload_kind(image, sizeof(image)) == UploadKind::Maintenance);
    assert(upload_kind(image, kOffset) == UploadKind::Unknown);   // too short
    const Info fl = flight_info();
    std::memcpy(image + kOffset, &fl, sizeof(fl));
    assert(upload_kind(image, sizeof(image)) == UploadKind::Flight);
    Info parsed{};
    assert(parse(image, sizeof(image), parsed) && parsed.build_id == 1000);

    // --- version id ---
    assert(version_id("", 32) == 0 && version_id(nullptr, 32) == 0);
    assert(version_id("bd4d2e3", 32) != 0);
    assert(version_id("bd4d2e3", 32) != version_id("bd4d2e4", 32));
    assert(version_id("bd4d2e3", 32) == version_id("bd4d2e3", 32));

    // --- beacon encode/decode ---
    maint_beacon::Status s;
    s.mode = IRIS_MAINT_MODE_USB_EXPORT;
    s.flags = IRIS_MAINT_FLAG_USB_DISK | IRIS_MAINT_FLAG_STORE_MOUNTED | IRIS_MAINT_FLAG_RADIO_OK;
    s.maint_image_id = 0xBEEF;
    s.flight_image_id = 0x1234;
    s.battery_mv = 7412;
    s.uptime_ms = 0x01020304U;
    s.log_files = 12;
    std::uint8_t frame[maint_beacon::kFrameLength + 4] = {};
    assert(maint_beacon::encode(s, frame) == maint_beacon::kFrameLength);
    static_assert(maint_beacon::kFrameLength <= 24, "beacon must stay small");
    assert(frame[0] == 0x08 && frame[maint_beacon::kFrameLength - 1] == 0x0A);
    maint_beacon::Status d;
    assert(maint_beacon::decode(frame, maint_beacon::kFrameLength, d));
    assert(d.mode == s.mode && d.flags == s.flags && d.maint_image_id == 0xBEEF &&
           d.flight_image_id == 0x1234 && d.battery_mv == 7412 && d.uptime_ms == 0x01020304U &&
           d.log_files == 12 && d.store_version == IRIS_MAINT_STORE_FORMAT_VERSION);
    assert(!maint_beacon::decode(frame, maint_beacon::kFrameLength - 1, d));
    frame[maint_beacon::kFrameLength - 1] = 0;
    assert(!maint_beacon::decode(frame, maint_beacon::kFrameLength, d));
    frame[maint_beacon::kFrameLength - 1] = 0x0A;
    frame[1] = 2;   // unknown beacon version
    assert(!maint_beacon::decode(frame, maint_beacon::kFrameLength, d));
    return 0;
}
