#pragma once

#include <cstdint>

#include "esp_err.h"
#include "esp_partition.h"
#include "image_info.h"
#include "maintenance_mode.h"

// Two-image boot (flash layout in partitions.csv):
//   flight slots  ota_0 / ota_1  (flight_a, flight_b)  -- FLIGHT image
//   maint slots   ota_2 / ota_3  (maint_a, maint_b)    -- MAINTENANCE image
// Both images link this file. The flight image uses prepare_switch() to hand
// over to the maintenance image; the maintenance image uses return_to_flight()
// first thing at every boot, so any reset (crash, watchdog, power loss)
// returns to flight.
//
// Persistent bookkeeping lives in NVS namespace "wboot" (the IMU calibration
// namespace is never touched): "fslot" = flight slot that last passed its
// self-test, "mconf" = build id of the newest maintenance image that booted
// and passed its self-test.
namespace boot_manager {

enum class Kind : std::uint8_t { Flight, Maintenance };

// This image's own record (embedded in .rodata_custom_desc, image_info_embed.cpp).
const image_info::Info &own_info();
bool running_flight();   // true in the flight image

// Partition of a slot (0 / 1) of a kind; nullptr if the table lacks it.
const esp_partition_t *slot_partition(Kind kind, int slot);
// Index (0/1) of a partition within its kind, -1 if it is neither.
int slot_index(const esp_partition_t *partition);
// Slot (0/1) of the running image within its kind.
int running_slot();
// Reads a slot's record (also checks the image header byte); false if the
// slot is empty, erased or not a Witness image of that kind.
bool read_info(const esp_partition_t *partition, Kind kind, image_info::Info &out);
// 16-bit id of a slot's image for the beacon (hash of app-desc version), 0 if none.
std::uint16_t image_id(const esp_partition_t *partition);
std::uint16_t own_image_id();

// --- bookkeeping (NVS; callers must have run nvs_flash_init) ---
int last_good_flight();                       // -1 unknown
void note_flight_good();                      // flight image: record the running slot (writes only on change)
std::uint32_t confirmed_maintenance_build();  // 0 unknown
void note_maintenance_confirmed(std::uint32_t build_id);

// --- flight image ---
struct SwitchResult {
    bool ok = false;
    std::uint16_t refuse_detail = IRIS_MAINT_REFUSE_NONE;   // IRIS_MAINT_REFUSE_* when !ok
    const esp_partition_t *target = nullptr;
};
// Checks that a compatible maintenance image exists, selects the newest
// confirmed slot, arms the RTC request and sets it as the next boot. The
// caller acknowledges, then restarts (maintenance_mode::restart_after_ms).
// On failure nothing is armed and the boot slot is unchanged.
SwitchResult prepare_switch(maintenance_mode::Kind kind, int tx_dbm, std::uint32_t minutes,
                            std::uint8_t flight_state, int radio_dbm, bool radio_front_end);
// The compatibility verdict alone (no side effects), for logging/tests.
image_info::Compat check_target(int *slot_out);

// Flight image: puts the boot partition back to the running (flight) image and
// drops any maintenance request. Used when a switch is abandoned after
// prepare_switch (restart timer failure, or the flight state changed during
// the restart delay), so no later reset can boot the maintenance image.
esp_err_t revert_switch();

// --- maintenance image ---
// Confirms this (running) image, then makes `flight_slot` (or the best flight
// slot when -1) the next boot and marks it valid, so any later reset returns
// to flight. Returns the flight slot (0/1), or -1 if no valid flight image
// exists (recovery mode).
int return_to_flight(int flight_slot_hint);
// Boots the other maintenance slot once for verification after an OTA there.
esp_err_t boot_other_maintenance_for_verify(const esp_partition_t *target, int flight_slot,
                                            int tx_dbm, std::uint32_t minutes);
// Boots a freshly uploaded flight slot (pending verify; the flight image's
// self-test confirms it, otherwise the bootloader rolls back).
esp_err_t boot_new_flight(const esp_partition_t *target);
// Marks this running maintenance image valid (cancels pending-verify rollback
// for it), which esp_ota_begin requires before an OTA write.
void mark_running_valid();
// True if the running image is still pending verification.
bool running_pending_verify();

}  // namespace boot_manager
