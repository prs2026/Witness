#pragma once

#include <cstdint>

#include "comms.h"
#include "lc86_nmea.h"

// Builds the USB-only 0xF0 ground-station status frame
// (IRIS_PACKET_GROUND_STATION_STATUS_FRAME_LENGTH bytes) from the last LoRa
// RSSI, the station uptime and the LC86 GPS state. Pure C++ for host tests.
//
// now_ms is the station uptime (esp_timer milliseconds) and the clock used for
// the GPS receive timestamps in gps.
void build_ground_station_status(std::uint8_t *frame, std::int8_t rssi_dbm,
                                 std::uint32_t now_ms,
                                 const Lc86Nmea::State &gps);
