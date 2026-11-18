#pragma once

#include "esp_err.h"

class FlightStateMachine;
class Sensors;
class WitnessStatus;

// Flight-image boot confirmation (rollback). A flight image that boots as
// "pending verify" (first boot after a maintenance-side OTA, or after a
// return from the maintenance image) must pass its self-test (IMU ready, flash
// log healthy, radio up) within kWindowMs or the bootloader rolls back. On a
// pass the image is marked valid and recorded as the last-good flight slot
// (boot_manager::note_flight_good). A boot that is already valid just records
// the slot. Needs NVS (nvs_flash_init) to have run.
namespace flight_selftest {

constexpr unsigned kWindowMs = 30000;

// A failed self-test rolls the image back (reboot) ONLY while the flight state
// is Pad Idle; in any other state the image stays pending-verify and flying.
esp_err_t start(const WitnessStatus &witness_status, const Sensors &sensors,
                const FlightStateMachine &flight_state_machine);

}  // namespace flight_selftest
