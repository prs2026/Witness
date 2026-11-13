#pragma once

// Shared AMA command and telemetry protocol. Keep the four comms.h copies
// identical; workbook values may describe an older protocol revision.
//
// Every transport uses the same canonical application packet:
//
//     packet ID | fixed-layout payload | 0x0A end-of-frame delimiter
//
// The delimiter is not escaped and may occur in binary payload data. A receiver
// must therefore determine the payload length from the packet ID and verify the
// delimiter at the resulting fixed offset; it must not scan for 0x0A. Transport
// framing, integrity checks, and CAN fragmentation are outside this schema.
// Multi-byte fields are big-endian on the wire.

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

#define IRIS_COMMAND_ARGUMENT_LENGTH 0U

#define IRIS_COMMAND_STATUS_FLAG_SET 0x20U
#define IRIS_COMMAND_STATUS_FLAG_CLEAR 0x21U
#define IRIS_COMMAND_PING 0x22U
#define IRIS_COMMAND_PONG 0x23U

#define IRIS_COMMAND_CH1_LOAD_OFF 0x30U
#define IRIS_COMMAND_CH1_LOAD_ON 0x31U
#define IRIS_COMMAND_CH2_LOAD_OFF 0x32U
#define IRIS_COMMAND_CH2_LOAD_ON 0x33U
#define IRIS_COMMAND_CH3_LOAD_OFF 0x34U
#define IRIS_COMMAND_CH3_LOAD_ON 0x35U

#define IRIS_COMMAND_5V_REGULATOR_OFF 0x50U
#define IRIS_COMMAND_5V_REGULATOR_ON 0x51U

#define IRIS_COMMAND_IRIS_DEBUG_START 0xE1U

// Enables the 10 Hz Witness Debug packet stream (packet ID 0xE0).
#define IRIS_COMMAND_WITNESS_DEBUG_START 0xE0U
// Packet ID 0x05 followed by this big-endian value switches USB to a
// read-only log-export disk until reset.
#define IRIS_COMMAND_MASS_STORAGE_START 0x6868U
// Protected destructive command: erase every stored flash log session.
#define IRIS_COMMAND_ERASE_LOG_SESSIONS 0x6869U
// Protected fast clear: deletes every stored session by erasing only the two
// session headers (2 blocks, not 1,004). Data pages stay on the chip but carry
// older session tags, so no export ever shows them; each block is erased
// again just before it is rewritten. The session number keeps counting up.
#define IRIS_COMMAND_CLEAR_LOG_SESSIONS 0x686AU
// GPS assistance upload (ephemeris for the receiver), chunked in ordinary
// 10-byte 0x05 frames over USB or radio; see gps_assist.h. Never forwarded.
#define IRIS_COMMAND_GPS_ASSIST_BEGIN 0x4741U
#define IRIS_COMMAND_GPS_ASSIST_DATA 0x4744U
#define IRIS_COMMAND_GPS_ASSIST_COMMIT 0x4745U
#define IRIS_COMMAND_GPS_ASSIST_STATUS 0x4751U
// GPS cold start: the receiver discards its ephemerides, almanac, position
// and time (SkyTraq 0x01 mode 3; u-blox CFG-RST, all BBR sections), then
// Witness reapplies its configuration. Witness-local, USB or radio, Pad Idle only.
#define IRIS_COMMAND_GPS_COLD_START 0x4752U
// Pad heading: d0 = the body axis that points to the azimuth (0 +Y, 1 -Y,
// 2 +Z, 3 -Z; 0xFF clears the setting), d1-d2 = its azimuth in 0.01 deg
// clockwise from true north (u16 BE, 0-35999), d3-d5 0. Witness turns its
// attitude about the vertical so that axis points there (no magnetometer: the
// pad leaves heading unknown) and re-applies it after every attitude
// re-initialization on the pad. Witness-local, USB or radio, Pad Idle only.
#define IRIS_COMMAND_PAD_HEADING 0x4844U
// Radio transmit power (RAM only; every boot starts at the bench-safe
// IRIS_RADIO_TX_POWER_DBM). d0 = target, d1 = SX1262 drive in dBm as int8
// (-17..+3; below -9 the SX1262 low-power PA is used; the Ra-01SH-P front end
// adds ~26 dB). Boot default -17 dBm.
// d2 = front end (RF_EN): 0 unchanged, 1 on (normal), 2 off (bench: no PA
// current; the SX1262 output passes the disabled FEM attenuated).
// Target WITNESS: Witness applies it (USB or radio). Target GROUND: Aperture
// applies it to its own radio, acks on USB and does not transmit the frame.
// Ack detail: high byte = front end on (1/0), low byte = drive dBm (int8).
// Out of range -> IRIS_ACK_BAD_ARGUMENT.
#define IRIS_COMMAND_RADIO_POWER 0x5250U
#define IRIS_RADIO_POWER_TARGET_WITNESS 0x00U
#define IRIS_RADIO_POWER_TARGET_GROUND 0x01U
#define IRIS_RADIO_FRONT_END_UNCHANGED 0x00U
#define IRIS_RADIO_FRONT_END_ON 0x01U
#define IRIS_RADIO_FRONT_END_OFF 0x02U
// Wi-Fi OTA / log download (Witness only; USB or radio; Pad Idle or Landed
// only, otherwise IRIS_ACK_REFUSED). d0 = 0 off / 1 on, d1 = Wi-Fi maximum TX
// power in dBm as int8 (0 keeps the default, +8; allowed 2..20), d2 =
// auto-off minutes (0 = default 15; 1..240). Never starts at boot. While Wi-Fi
// is on, Witness stops all LoRa transmission (telemetry and acks) but keeps
// receiving, so an off command over radio still works; its ack is sent after
// Wi-Fi is off. Wi-Fi also turns off by itself after the auto-off time without
// HTTP activity and on any flight state change out of Pad Idle/Landed.
// Out-of-range arguments -> IRIS_ACK_BAD_ARGUMENT; busy/rebooting -> IRIS_ACK_BUSY.
#define IRIS_COMMAND_WIFI 0x5746U
// Three-byte state command: 0x05, 0x73, then one of these state values.
#define IRIS_COMMAND_SET_FLIGHT_STATE 0x73U
#define IRIS_FLIGHT_STATE_PAD_IDLE 0x00U
#define IRIS_FLIGHT_STATE_BOOST 0x01U
#define IRIS_FLIGHT_STATE_COAST 0x02U
#define IRIS_FLIGHT_STATE_DESCENT 0x03U
#define IRIS_FLIGHT_STATE_LANDED 0x04U
// Hardware-in-the-loop (HITL) test mode. ENTER is accepted only from USB and
// only in Pad Idle; Witness then takes IMU, barometer, GPS and battery values
// from host 0x0C packets instead of its sensors. EXIT (USB or radio), a reboot,
// or no 0x0C packet for IRIS_HITL_INPUT_TIMEOUT_MS returns to real sensors.
#define IRIS_COMMAND_HITL_ENTER 0x4849U
#define IRIS_COMMAND_HITL_EXIT 0x4858U
// IMU calibration (USB only, Pad Idle, real sensors): 0x05, 0x43, then one of
// these arguments. GYRO captures 2 s still and stores the gyro bias; FACE
// captures 2 s on one face (+/-X, +/-Y, +/-Z, any order) and, after all six,
// solves and stores accelerometer offset/scale and high-g offset; CLEAR erases
// the stored calibration; STATUS logs it. Results are ESP log lines (tag
// sensors) and persist in NVS across reboots.
#define IRIS_COMMAND_IMU_CALIBRATION 0x43U
#define IRIS_IMU_CALIBRATION_GYRO 0x47U   /* 'G' */
#define IRIS_IMU_CALIBRATION_FACE 0x41U   /* 'A' */
#define IRIS_IMU_CALIBRATION_CLEAR 0x58U  /* 'X' */
#define IRIS_IMU_CALIBRATION_STATUS 0x53U /* 'S' */
#define IRIS_HITL_INPUT_TIMEOUT_MS 1000U

// Producer-defined IRIS status-byte flags.
#define IRIS_STATUS_CURRENT_MONITOR_ERROR_FLAG 0x01U
#define IRIS_STATUS_BATTERY_MONITOR_ERROR_FLAG 0x02U
#define IRIS_STATUS_COMMAND_CONTROLLED_FLAG 0x04U
#define IRIS_STATUS_OUT1_ENABLED_FLAG 0x08U
#define IRIS_STATUS_OUT2_ENABLED_FLAG 0x10U
#define IRIS_STATUS_OUT3_ENABLED_FLAG 0x20U
#define IRIS_STATUS_5V_ENABLED_FLAG 0x40U

// ---------------------------------------------------------------------------
// Canonical field catalog
// ---------------------------------------------------------------------------

// Witness status: two raw bytes. Byte 0 contains the following active-high
// flags. Bits 0..2 of byte 1 contain the current flight state; the remaining
// bits of byte 1 are reserved and must be zero.
#define IRIS_FIELD_WITNESS_STATUS_LENGTH 2U
// Byte 0, bit 0: toggles once per heartbeat period.
#define IRIS_WITNESS_STATUS_HEARTBEAT_MASK (1U << 0)
// Byte 0, bit 1: a valid LSM6 low-g acceleration sample is available.
#define IRIS_WITNESS_STATUS_LOW_G_ACCEL_READY_MASK (1U << 1)
// Byte 0, bit 2: a valid LSM6 gyroscope sample is available.
#define IRIS_WITNESS_STATUS_GYRO_READY_MASK (1U << 2)
// Byte 0, bit 3: a valid LSM6 temperature sample is available.
#define IRIS_WITNESS_STATUS_IMU_TEMPERATURE_READY_MASK (1U << 3)
// Byte 0, bit 4: the W25N01GW failed initialization or startup self-test.
#define IRIS_WITNESS_STATUS_FLASH_INIT_FAILED_MASK (1U << 4)
// Byte 0, bit 5: a valid LSM6 high-g acceleration sample is available.
#define IRIS_WITNESS_STATUS_HIGH_G_ACCEL_READY_MASK (1U << 5)
// Byte 0, bit 6: a valid MS5607 sample is available.
#define IRIS_WITNESS_STATUS_BAROMETER_READY_MASK (1U << 6)
// Byte 0, bit 7: flash initialized, but logging later stopped on an error.
#define IRIS_WITNESS_STATUS_FLASH_LOG_FAILED_MASK (1U << 7)
// Status byte 1, bits 0..2: flight state encoded using IRIS_FLIGHT_STATE_*.
#define IRIS_WITNESS_STATUS_FLIGHT_STATE_BYTE_OFFSET 1U
#define IRIS_WITNESS_STATUS_FLIGHT_STATE_SHIFT 0U
#define IRIS_WITNESS_STATUS_FLIGHT_STATE_MASK 0x07U
// Status byte 1, bit 3: a fresh GPS fix (GGA quality >= 1) is available.
#define IRIS_WITNESS_STATUS_GPS_LOCKED_BYTE_OFFSET 1U
#define IRIS_WITNESS_STATUS_GPS_LOCKED_MASK (1U << 3)
// Status byte 1, bit 4: HITL mode is active; sensor values are simulated.
#define IRIS_WITNESS_STATUS_HITL_ACTIVE_BYTE_OFFSET 1U
#define IRIS_WITNESS_STATUS_HITL_ACTIVE_MASK (1U << 4)
// Status byte 1, bit 5: MAINTENANCE boot (Wi-Fi OTA / log download). Flight
// logging is NOT running: Witness is NOT ready for flight. Cleared by the
// reboot back to normal mode (Wi-Fi off, auto-off, or power cycle).
#define IRIS_WITNESS_STATUS_MAINTENANCE_BYTE_OFFSET 1U
#define IRIS_WITNESS_STATUS_MAINTENANCE_MASK (1U << 5)
// Status byte 1, bits 6..7: reserved and transmitted as zero.
#define IRIS_WITNESS_STATUS_RESERVED_MASK 0xC0U

// Witness uptime: big-endian uint32, milliseconds since boot. The counter
// wraps after approximately 49.7 days.
#define IRIS_FIELD_WITNESS_UPTIME_LENGTH 4U

// Witness MCU temperature: signed int8, 1 degree Celsius/count.
#define IRIS_FIELD_WITNESS_MCU_TEMPERATURE_LENGTH 1U

// Witness battery voltage: big-endian uint16, 1 millivolt/count.
#define IRIS_FIELD_WITNESS_BATTERY_VOLTAGE_LENGTH 2U

// IRIS uptime: big-endian uint32, milliseconds since boot. The counter wraps
// after approximately 49.7 days.
#define IRIS_FIELD_IRIS_UPTIME_LENGTH 4U

// IRIS status: two raw flag bytes. Byte 0 contains producer-defined active
// flags and byte 1 is reserved. This is not a scaled numeric quantity.
#define IRIS_FIELD_IRIS_STATUS_LENGTH 2U

// IRIS MCU temperature: signed int8, 1 degree Celsius/count.
#define IRIS_FIELD_IRIS_MCU_TEMPERATURE_LENGTH 1U

// IRIS battery voltage: big-endian uint16, 1 millivolt/count.
#define IRIS_FIELD_IRIS_BATTERY_VOLTAGE_LENGTH 2U

// Filtered acceleration: big-endian signed int24, 40 micro-g/count. Physical
// acceleration in g = raw * 0.000040. Range is approximately +/-335.54 g.
#define IRIS_FIELD_FILTERED_ACCEL_X_LENGTH 3U
#define IRIS_FIELD_FILTERED_ACCEL_Y_LENGTH 3U
#define IRIS_FIELD_FILTERED_ACCEL_Z_LENGTH 3U

// Orientation quaternion: big-endian IEEE-754 float32, unitless normalized
// quaternion components in X, Y, Z, W order (scalar last).
// Convention: Hamilton quaternion rotating BODY vectors into the WORLD frame
// (v_world = q * v_body * q^-1). WORLD is local ENU: +X east, +Y north,
// +Z up. BODY is the LSM6 sensor frame, with +X along the rocket's nose axis
// (an upright rocket on the pad has body +X pointing to world +Z). Heading
// (rotation about world +Z) is not observable without a magnetometer; it is
// fixed arbitrarily at initialization by the shortest-arc tilt alignment.
// All four components zero means attitude is not available.
#define IRIS_FIELD_ORIENTATION_QUAT_X_LENGTH 4U
#define IRIS_FIELD_ORIENTATION_QUAT_Y_LENGTH 4U
#define IRIS_FIELD_ORIENTATION_QUAT_Z_LENGTH 4U
#define IRIS_FIELD_ORIENTATION_QUAT_W_LENGTH 4U

// Position: big-endian IEEE-754 float32, meters in the estimator local frame.
// The local-frame origin and axis convention are estimator-defined.
#define IRIS_FIELD_POSITION_X_LENGTH 4U
#define IRIS_FIELD_POSITION_Y_LENGTH 4U
#define IRIS_FIELD_POSITION_Z_LENGTH 4U

// Velocity: big-endian IEEE-754 float32, meters/second in the estimator local
// frame. Axis convention matches the position field.
#define IRIS_FIELD_VELOCITY_X_LENGTH 4U
#define IRIS_FIELD_VELOCITY_Y_LENGTH 4U
#define IRIS_FIELD_VELOCITY_Z_LENGTH 4U

// LSM6 low-g acceleration: big-endian signed int16 native sensor samples,
// 0.488 milli-g/count at the configured +/-16 g full scale.
#define IRIS_FIELD_LSM6_LOW_ACCEL_X_LENGTH 2U
#define IRIS_FIELD_LSM6_LOW_ACCEL_Y_LENGTH 2U
#define IRIS_FIELD_LSM6_LOW_ACCEL_Z_LENGTH 2U

// LSM6 temperature: big-endian signed int16 native sensor sample.
// Temperature in degrees Celsius = 25 + raw / 256.
#define IRIS_FIELD_LSM6_TEMPERATURE_LENGTH 2U

// LSM6 high-g acceleration: big-endian signed int16 native sensor samples,
// 10.417 milli-g/count at the configured +/-320 g full scale.
#define IRIS_FIELD_LSM6_HIGH_ACCEL_X_LENGTH 2U
#define IRIS_FIELD_LSM6_HIGH_ACCEL_Y_LENGTH 2U
#define IRIS_FIELD_LSM6_HIGH_ACCEL_Z_LENGTH 2U

// LSM6 angular rate: big-endian signed int16 native sensor samples,
// 70 milli-degrees/second/count at the configured +/-2000 dps full scale.
#define IRIS_FIELD_LSM6_GYRO_X_LENGTH 2U
#define IRIS_FIELD_LSM6_GYRO_Y_LENGTH 2U
#define IRIS_FIELD_LSM6_GYRO_Z_LENGTH 2U

// MS5607 pressure: big-endian unsigned uint24, 1 pascal/count. One pascal is
// equivalent to 0.01 millibar, matching the driver's centi-mbar value.
#define IRIS_FIELD_MS5607_PRESSURE_LENGTH 3U

// MS5607 temperature: big-endian signed int16, 0.01 degree Celsius/count.
#define IRIS_FIELD_MS5607_TEMPERATURE_LENGTH 2U

// MS5607 altitude: big-endian IEEE-754 float32, meters relative to the
// configured sea-level pressure reference (currently 1013.25 mbar).
#define IRIS_FIELD_MS5607_ALTITUDE_LENGTH 4U

// Latitude and longitude: big-endian signed int24, 0.0001 degree/count.
// Degrees = raw / 10,000; north/east are positive and south/west negative.
#define IRIS_FIELD_LATITUDE_LENGTH 3U
#define IRIS_FIELD_LONGITUDE_LENGTH 3U

// GPS time: big-endian uint32 Unix timestamp, seconds since 1970-01-01 UTC.
#define IRIS_FIELD_GPS_TIME_LENGTH 4U

// GPS altitude: big-endian IEEE-754 float32, meters above mean sea level.
#define IRIS_FIELD_GPS_ALTITUDE_LENGTH 4U

// GPS satellites: unsigned uint8, satellites used in the solution (NMEA GGA
// field 7) from the most recent GGA within the fix timeout; 0 when stale.
#define IRIS_FIELD_GPS_SATELLITES_LENGTH 1U

// GPS fix quality: unsigned uint8, NMEA GGA field 6 (0 = no fix, 1 = GPS,
// 2 = DGPS, ...). Used by the HITL input packet.
#define IRIS_FIELD_GPS_FIX_QUALITY_LENGTH 1U

// Channel current: unsigned uint8, 0.02 ampere/count (20 mA/count).
// Representable range is 0.00 to 5.10 A. Camera reports one-second peaks;
// IRIS debug reports the most recent sample.
#define IRIS_FIELD_CH1_CURRENT_LENGTH 1U
#define IRIS_FIELD_CH2_CURRENT_LENGTH 1U
#define IRIS_FIELD_CH3_CURRENT_LENGTH 1U

// Command: big-endian uint16 enumeration/opcode. No numeric scale or unit.
#define IRIS_FIELD_COMMAND_LENGTH 2U

// Machine-readable fixed-point scales corresponding to the comments above.
#define IRIS_FIELD_FILTERED_ACCEL_MICRO_G_PER_COUNT 40U
#define IRIS_FIELD_BATTERY_VOLTAGE_MILLIVOLTS_PER_COUNT 1U
#define IRIS_FIELD_GPS_COORDINATE_SCALE 10000L
#define IRIS_FIELD_MCU_TEMPERATURE_CELSIUS_PER_COUNT 1U
#define IRIS_FIELD_LSM6_LOW_ACCEL_MICRO_G_PER_COUNT 488U
#define IRIS_FIELD_LSM6_HIGH_ACCEL_MICRO_G_PER_COUNT 10417U
#define IRIS_FIELD_LSM6_GYRO_MILLIDEGREES_PER_SECOND_PER_COUNT 70U
#define IRIS_FIELD_LSM6_TEMPERATURE_RAW_COUNTS_PER_CELSIUS 256U
#define IRIS_FIELD_LSM6_TEMPERATURE_OFFSET_CENTICELSIUS 2500
#define IRIS_FIELD_MS5607_PRESSURE_PASCALS_PER_COUNT 1U
#define IRIS_FIELD_MS5607_TEMPERATURE_CENTICELSIUS_PER_COUNT 1U
#define IRIS_FIELD_CURRENT_MILLIAMPS_PER_COUNT 20U

// ---------------------------------------------------------------------------
// Common packet definitions
// ---------------------------------------------------------------------------

#define IRIS_PACKET_ID_LENGTH 1U
#define IRIS_PACKET_EOF_VALUE 0x0AU
#define IRIS_PACKET_EOF_LENGTH 1U
#define IRIS_PACKET_TRANSPORT_CRC_LENGTH 0U
#define IRIS_CAN_FRAME_DATA_LENGTH 8U
// Intercomm/can_transport.h reserves sequence/index/total header bytes.
#define IRIS_CAN_FRAGMENT_PAYLOAD_LENGTH 5U
#define IRIS_CAN_FRAME_COUNT(application_packet_length) \
    (((application_packet_length) + IRIS_CAN_FRAGMENT_PAYLOAD_LENGTH - 1U) / \
     IRIS_CAN_FRAGMENT_PAYLOAD_LENGTH)
// Packet-level status and uptime aliases use the Witness encodings documented
// above: two raw flag bytes and big-endian uint32 milliseconds, respectively.
#define IRIS_PACKET_STATUS_LENGTH IRIS_FIELD_WITNESS_STATUS_LENGTH
#define IRIS_PACKET_UPTIME_LENGTH IRIS_FIELD_WITNESS_UPTIME_LENGTH

#define IRIS_PACKET_VECTOR_COMPONENTS 3U
#define IRIS_PACKET_QUATERNION_COMPONENTS 4U
#define IRIS_PACKET_CAMERA_CURRENT_CHANNELS 3U

// ---------------------------------------------------------------------------
// Sensor packet (0x01)
// ---------------------------------------------------------------------------

#define IRIS_PACKET_ID_SENSORS 0x01U

#define IRIS_PACKET_SENSORS_STATUS_OFFSET 0U
#define IRIS_PACKET_SENSORS_STATUS_LENGTH IRIS_FIELD_WITNESS_STATUS_LENGTH
#define IRIS_PACKET_SENSORS_UPTIME_OFFSET \
    (IRIS_PACKET_SENSORS_STATUS_OFFSET + IRIS_PACKET_SENSORS_STATUS_LENGTH)
#define IRIS_PACKET_SENSORS_UPTIME_LENGTH IRIS_FIELD_WITNESS_UPTIME_LENGTH
#define IRIS_PACKET_SENSORS_ACCEL_OFFSET \
    (IRIS_PACKET_SENSORS_UPTIME_OFFSET + IRIS_PACKET_SENSORS_UPTIME_LENGTH)
#define IRIS_PACKET_SENSORS_ACCEL_COMPONENT_LENGTH \
    IRIS_FIELD_FILTERED_ACCEL_X_LENGTH
#define IRIS_PACKET_SENSORS_ACCEL_LENGTH \
    (IRIS_PACKET_VECTOR_COMPONENTS * \
     IRIS_PACKET_SENSORS_ACCEL_COMPONENT_LENGTH)
#define IRIS_PACKET_SENSORS_ACCEL_X_OFFSET IRIS_PACKET_SENSORS_ACCEL_OFFSET
#define IRIS_PACKET_SENSORS_ACCEL_Y_OFFSET \
    (IRIS_PACKET_SENSORS_ACCEL_X_OFFSET + \
     IRIS_PACKET_SENSORS_ACCEL_COMPONENT_LENGTH)
#define IRIS_PACKET_SENSORS_ACCEL_Z_OFFSET \
    (IRIS_PACKET_SENSORS_ACCEL_Y_OFFSET + \
     IRIS_PACKET_SENSORS_ACCEL_COMPONENT_LENGTH)
#define IRIS_PACKET_SENSORS_GYRO_OFFSET \
    (IRIS_PACKET_SENSORS_ACCEL_OFFSET + IRIS_PACKET_SENSORS_ACCEL_LENGTH)
#define IRIS_PACKET_SENSORS_GYRO_COMPONENT_LENGTH \
    IRIS_FIELD_LSM6_GYRO_X_LENGTH
#define IRIS_PACKET_SENSORS_GYRO_LENGTH \
    (IRIS_PACKET_VECTOR_COMPONENTS * \
     IRIS_PACKET_SENSORS_GYRO_COMPONENT_LENGTH)
#define IRIS_PACKET_SENSORS_GYRO_X_OFFSET IRIS_PACKET_SENSORS_GYRO_OFFSET
#define IRIS_PACKET_SENSORS_GYRO_Y_OFFSET \
    (IRIS_PACKET_SENSORS_GYRO_X_OFFSET + \
     IRIS_PACKET_SENSORS_GYRO_COMPONENT_LENGTH)
#define IRIS_PACKET_SENSORS_GYRO_Z_OFFSET \
    (IRIS_PACKET_SENSORS_GYRO_Y_OFFSET + \
     IRIS_PACKET_SENSORS_GYRO_COMPONENT_LENGTH)
#define IRIS_PACKET_SENSORS_DATA_LENGTH \
    (IRIS_PACKET_SENSORS_STATUS_LENGTH + \
     IRIS_PACKET_SENSORS_UPTIME_LENGTH + \
     IRIS_PACKET_SENSORS_ACCEL_LENGTH + \
     IRIS_PACKET_SENSORS_GYRO_LENGTH)
#define IRIS_PACKET_SENSORS_EOF_OFFSET IRIS_PACKET_SENSORS_DATA_LENGTH
#define IRIS_PACKET_SENSORS_FRAME_LENGTH \
    (IRIS_PACKET_ID_LENGTH + IRIS_PACKET_SENSORS_DATA_LENGTH + \
     IRIS_PACKET_EOF_LENGTH)
#define IRIS_PACKET_SENSORS_CAN_FRAME_COUNT \
    IRIS_CAN_FRAME_COUNT(IRIS_PACKET_SENSORS_FRAME_LENGTH)

// ---------------------------------------------------------------------------
// Estimated-state packet (0x02)
// ---------------------------------------------------------------------------

#define IRIS_PACKET_ID_STATE 0x02U

#define IRIS_PACKET_STATE_STATUS_OFFSET 0U
#define IRIS_PACKET_STATE_STATUS_LENGTH IRIS_FIELD_WITNESS_STATUS_LENGTH
#define IRIS_PACKET_STATE_UPTIME_OFFSET \
    (IRIS_PACKET_STATE_STATUS_OFFSET + IRIS_PACKET_STATE_STATUS_LENGTH)
#define IRIS_PACKET_STATE_UPTIME_LENGTH IRIS_FIELD_WITNESS_UPTIME_LENGTH
#define IRIS_PACKET_STATE_ORIENTATION_OFFSET \
    (IRIS_PACKET_STATE_UPTIME_OFFSET + IRIS_PACKET_STATE_UPTIME_LENGTH)
#define IRIS_PACKET_STATE_ORIENTATION_COMPONENT_LENGTH \
    IRIS_FIELD_ORIENTATION_QUAT_X_LENGTH
#define IRIS_PACKET_STATE_ORIENTATION_LENGTH \
    (IRIS_PACKET_QUATERNION_COMPONENTS * \
     IRIS_PACKET_STATE_ORIENTATION_COMPONENT_LENGTH)
#define IRIS_PACKET_STATE_ORIENTATION_X_OFFSET \
    IRIS_PACKET_STATE_ORIENTATION_OFFSET
#define IRIS_PACKET_STATE_ORIENTATION_Y_OFFSET \
    (IRIS_PACKET_STATE_ORIENTATION_X_OFFSET + \
     IRIS_PACKET_STATE_ORIENTATION_COMPONENT_LENGTH)
#define IRIS_PACKET_STATE_ORIENTATION_Z_OFFSET \
    (IRIS_PACKET_STATE_ORIENTATION_Y_OFFSET + \
     IRIS_PACKET_STATE_ORIENTATION_COMPONENT_LENGTH)
#define IRIS_PACKET_STATE_ORIENTATION_W_OFFSET \
    (IRIS_PACKET_STATE_ORIENTATION_Z_OFFSET + \
     IRIS_PACKET_STATE_ORIENTATION_COMPONENT_LENGTH)
#define IRIS_PACKET_STATE_POSITION_OFFSET \
    (IRIS_PACKET_STATE_ORIENTATION_OFFSET + \
     IRIS_PACKET_STATE_ORIENTATION_LENGTH)
#define IRIS_PACKET_STATE_POSITION_COMPONENT_LENGTH IRIS_FIELD_POSITION_X_LENGTH
#define IRIS_PACKET_STATE_POSITION_LENGTH \
    (IRIS_PACKET_VECTOR_COMPONENTS * \
     IRIS_PACKET_STATE_POSITION_COMPONENT_LENGTH)
#define IRIS_PACKET_STATE_POSITION_X_OFFSET IRIS_PACKET_STATE_POSITION_OFFSET
#define IRIS_PACKET_STATE_POSITION_Y_OFFSET \
    (IRIS_PACKET_STATE_POSITION_X_OFFSET + \
     IRIS_PACKET_STATE_POSITION_COMPONENT_LENGTH)
#define IRIS_PACKET_STATE_POSITION_Z_OFFSET \
    (IRIS_PACKET_STATE_POSITION_Y_OFFSET + \
     IRIS_PACKET_STATE_POSITION_COMPONENT_LENGTH)
#define IRIS_PACKET_STATE_VELOCITY_OFFSET \
    (IRIS_PACKET_STATE_POSITION_OFFSET + IRIS_PACKET_STATE_POSITION_LENGTH)
#define IRIS_PACKET_STATE_VELOCITY_COMPONENT_LENGTH IRIS_FIELD_VELOCITY_X_LENGTH
#define IRIS_PACKET_STATE_VELOCITY_LENGTH \
    (IRIS_PACKET_VECTOR_COMPONENTS * \
     IRIS_PACKET_STATE_VELOCITY_COMPONENT_LENGTH)
#define IRIS_PACKET_STATE_VELOCITY_X_OFFSET IRIS_PACKET_STATE_VELOCITY_OFFSET
#define IRIS_PACKET_STATE_VELOCITY_Y_OFFSET \
    (IRIS_PACKET_STATE_VELOCITY_X_OFFSET + \
     IRIS_PACKET_STATE_VELOCITY_COMPONENT_LENGTH)
#define IRIS_PACKET_STATE_VELOCITY_Z_OFFSET \
    (IRIS_PACKET_STATE_VELOCITY_Y_OFFSET + \
     IRIS_PACKET_STATE_VELOCITY_COMPONENT_LENGTH)
#define IRIS_PACKET_STATE_LATITUDE_OFFSET \
    (IRIS_PACKET_STATE_VELOCITY_OFFSET + IRIS_PACKET_STATE_VELOCITY_LENGTH)
#define IRIS_PACKET_STATE_LATITUDE_LENGTH IRIS_FIELD_LATITUDE_LENGTH
#define IRIS_PACKET_STATE_LONGITUDE_OFFSET \
    (IRIS_PACKET_STATE_LATITUDE_OFFSET + IRIS_PACKET_STATE_LATITUDE_LENGTH)
#define IRIS_PACKET_STATE_LONGITUDE_LENGTH IRIS_FIELD_LONGITUDE_LENGTH
#define IRIS_PACKET_STATE_LAT_LON_OFFSET IRIS_PACKET_STATE_LATITUDE_OFFSET
#define IRIS_PACKET_STATE_LAT_LON_LENGTH \
    (IRIS_PACKET_STATE_LATITUDE_LENGTH + IRIS_PACKET_STATE_LONGITUDE_LENGTH)
#define IRIS_PACKET_STATE_GPS_ALTITUDE_OFFSET \
    (IRIS_PACKET_STATE_LONGITUDE_OFFSET + IRIS_PACKET_STATE_LONGITUDE_LENGTH)
#define IRIS_PACKET_STATE_GPS_ALTITUDE_LENGTH IRIS_FIELD_GPS_ALTITUDE_LENGTH
#define IRIS_PACKET_STATE_BAROMETRIC_ALTITUDE_OFFSET \
    (IRIS_PACKET_STATE_GPS_ALTITUDE_OFFSET + \
     IRIS_PACKET_STATE_GPS_ALTITUDE_LENGTH)
#define IRIS_PACKET_STATE_BAROMETRIC_ALTITUDE_LENGTH \
    IRIS_FIELD_MS5607_ALTITUDE_LENGTH
// This slot carries the Kalman filter altitude (float32 m above the pad, the
// filter's up position), not the raw barometer altitude. Raw MS5607 pressure
// stays in the sensors packet and the flash sensor log (S*.BIN), from which
// the ISA barometric altitude can be recomputed.
#define IRIS_PACKET_STATE_KALMAN_ALTITUDE_OFFSET \
    IRIS_PACKET_STATE_BAROMETRIC_ALTITUDE_OFFSET
#define IRIS_PACKET_STATE_BATTERY_VOLTAGE_OFFSET \
    (IRIS_PACKET_STATE_BAROMETRIC_ALTITUDE_OFFSET + \
     IRIS_PACKET_STATE_BAROMETRIC_ALTITUDE_LENGTH)
#define IRIS_PACKET_STATE_BATTERY_VOLTAGE_LENGTH \
    IRIS_FIELD_WITNESS_BATTERY_VOLTAGE_LENGTH
#define IRIS_PACKET_STATE_GPS_SATELLITES_OFFSET \
    (IRIS_PACKET_STATE_BATTERY_VOLTAGE_OFFSET + \
     IRIS_PACKET_STATE_BATTERY_VOLTAGE_LENGTH)
#define IRIS_PACKET_STATE_GPS_SATELLITES_LENGTH IRIS_FIELD_GPS_SATELLITES_LENGTH
#define IRIS_PACKET_STATE_DATA_LENGTH \
    (IRIS_PACKET_STATE_STATUS_LENGTH + \
     IRIS_PACKET_STATE_UPTIME_LENGTH + \
     IRIS_PACKET_STATE_ORIENTATION_LENGTH + \
     IRIS_PACKET_STATE_POSITION_LENGTH + \
     IRIS_PACKET_STATE_VELOCITY_LENGTH + \
     IRIS_PACKET_STATE_LATITUDE_LENGTH + \
     IRIS_PACKET_STATE_LONGITUDE_LENGTH + \
     IRIS_PACKET_STATE_GPS_ALTITUDE_LENGTH + \
     IRIS_PACKET_STATE_BAROMETRIC_ALTITUDE_LENGTH + \
     IRIS_PACKET_STATE_BATTERY_VOLTAGE_LENGTH + \
     IRIS_PACKET_STATE_GPS_SATELLITES_LENGTH)
#define IRIS_PACKET_STATE_EOF_OFFSET IRIS_PACKET_STATE_DATA_LENGTH
#define IRIS_PACKET_STATE_FRAME_LENGTH \
    (IRIS_PACKET_ID_LENGTH + IRIS_PACKET_STATE_DATA_LENGTH + \
     IRIS_PACKET_EOF_LENGTH)
#define IRIS_PACKET_STATE_CAN_FRAME_COUNT \
    IRIS_CAN_FRAME_COUNT(IRIS_PACKET_STATE_FRAME_LENGTH)

// ---------------------------------------------------------------------------
// Camera telemetry packet (0x03)
// ---------------------------------------------------------------------------

#define IRIS_PACKET_ID_CAMERA 0x03U

#define IRIS_PACKET_CAMERA_FC_STATUS_OFFSET 0U
#define IRIS_PACKET_CAMERA_FC_STATUS_LENGTH IRIS_FIELD_WITNESS_STATUS_LENGTH
#define IRIS_PACKET_CAMERA_FC_UPTIME_OFFSET \
    (IRIS_PACKET_CAMERA_FC_STATUS_OFFSET + \
     IRIS_PACKET_CAMERA_FC_STATUS_LENGTH)
#define IRIS_PACKET_CAMERA_FC_UPTIME_LENGTH IRIS_FIELD_WITNESS_UPTIME_LENGTH
#define IRIS_PACKET_CAMERA_STATUS_OFFSET \
    (IRIS_PACKET_CAMERA_FC_UPTIME_OFFSET + \
     IRIS_PACKET_CAMERA_FC_UPTIME_LENGTH)
#define IRIS_PACKET_CAMERA_STATUS_LENGTH IRIS_FIELD_IRIS_STATUS_LENGTH
#define IRIS_PACKET_CAMERA_UPTIME_OFFSET \
    (IRIS_PACKET_CAMERA_STATUS_OFFSET + IRIS_PACKET_CAMERA_STATUS_LENGTH)
#define IRIS_PACKET_CAMERA_UPTIME_LENGTH IRIS_FIELD_IRIS_UPTIME_LENGTH
#define IRIS_PACKET_CAMERA_CURRENT_SENSE_OFFSET \
    (IRIS_PACKET_CAMERA_UPTIME_OFFSET + IRIS_PACKET_CAMERA_UPTIME_LENGTH)
#define IRIS_PACKET_CAMERA_CURRENT_SENSE_COMPONENT_LENGTH \
    IRIS_FIELD_CH1_CURRENT_LENGTH
#define IRIS_PACKET_CAMERA_CURRENT_SENSE_LENGTH \
    (IRIS_PACKET_CAMERA_CURRENT_CHANNELS * \
     IRIS_PACKET_CAMERA_CURRENT_SENSE_COMPONENT_LENGTH)
#define IRIS_PACKET_CAMERA_CH1_CURRENT_OFFSET \
    IRIS_PACKET_CAMERA_CURRENT_SENSE_OFFSET
#define IRIS_PACKET_CAMERA_CH2_CURRENT_OFFSET \
    (IRIS_PACKET_CAMERA_CH1_CURRENT_OFFSET + \
     IRIS_PACKET_CAMERA_CURRENT_SENSE_COMPONENT_LENGTH)
#define IRIS_PACKET_CAMERA_CH3_CURRENT_OFFSET \
    (IRIS_PACKET_CAMERA_CH2_CURRENT_OFFSET + \
     IRIS_PACKET_CAMERA_CURRENT_SENSE_COMPONENT_LENGTH)
#define IRIS_PACKET_CAMERA_BATTERY_VOLTAGE_OFFSET \
    (IRIS_PACKET_CAMERA_CURRENT_SENSE_OFFSET + \
     IRIS_PACKET_CAMERA_CURRENT_SENSE_LENGTH)
#define IRIS_PACKET_CAMERA_BATTERY_VOLTAGE_LENGTH \
    IRIS_FIELD_IRIS_BATTERY_VOLTAGE_LENGTH
#define IRIS_PACKET_CAMERA_DATA_LENGTH \
    (IRIS_PACKET_CAMERA_FC_STATUS_LENGTH + \
     IRIS_PACKET_CAMERA_FC_UPTIME_LENGTH + \
     IRIS_PACKET_CAMERA_STATUS_LENGTH + \
     IRIS_PACKET_CAMERA_UPTIME_LENGTH + \
     IRIS_PACKET_CAMERA_CURRENT_SENSE_LENGTH + \
     IRIS_PACKET_CAMERA_BATTERY_VOLTAGE_LENGTH)
#define IRIS_PACKET_CAMERA_EOF_OFFSET IRIS_PACKET_CAMERA_DATA_LENGTH
#define IRIS_PACKET_CAMERA_FRAME_LENGTH \
    (IRIS_PACKET_ID_LENGTH + IRIS_PACKET_CAMERA_DATA_LENGTH + \
     IRIS_PACKET_EOF_LENGTH)
#define IRIS_PACKET_CAMERA_CAN_FRAME_COUNT \
    IRIS_CAN_FRAME_COUNT(IRIS_PACKET_CAMERA_FRAME_LENGTH)

// ---------------------------------------------------------------------------
// Command packet (0x05)
// ---------------------------------------------------------------------------

#define IRIS_PACKET_ID_COMMAND 0x05U

#define IRIS_PACKET_COMMAND_STATUS_OFFSET 0U
#define IRIS_PACKET_COMMAND_STATUS_LENGTH IRIS_PACKET_STATUS_LENGTH
#define IRIS_PACKET_COMMAND_UPTIME_OFFSET \
    (IRIS_PACKET_COMMAND_STATUS_OFFSET + IRIS_PACKET_COMMAND_STATUS_LENGTH)
#define IRIS_PACKET_COMMAND_UPTIME_LENGTH IRIS_PACKET_UPTIME_LENGTH
#define IRIS_PACKET_COMMAND_VALUE_OFFSET \
    (IRIS_PACKET_COMMAND_UPTIME_OFFSET + IRIS_PACKET_COMMAND_UPTIME_LENGTH)
#define IRIS_PACKET_COMMAND_VALUE_LENGTH IRIS_FIELD_COMMAND_LENGTH
#define IRIS_PACKET_COMMAND_DATA_LENGTH \
    (IRIS_PACKET_COMMAND_STATUS_LENGTH + \
     IRIS_PACKET_COMMAND_UPTIME_LENGTH + \
     IRIS_PACKET_COMMAND_VALUE_LENGTH)
#define IRIS_PACKET_COMMAND_EOF_OFFSET IRIS_PACKET_COMMAND_DATA_LENGTH
#define IRIS_PACKET_COMMAND_FRAME_LENGTH \
    (IRIS_PACKET_ID_LENGTH + IRIS_PACKET_COMMAND_DATA_LENGTH + \
     IRIS_PACKET_EOF_LENGTH)
#define IRIS_PACKET_COMMAND_CAN_FRAME_COUNT \
    IRIS_CAN_FRAME_COUNT(IRIS_PACKET_COMMAND_FRAME_LENGTH)

// ---------------------------------------------------------------------------
// Kalman filter log packet (0xE2): flash log only, never sent over LoRa/CAN
// ---------------------------------------------------------------------------
// Witness writes one per position-filter update cycle (25 Hz) to the P*.BIN
// packet stream (record source 5) for post-flight review. All multi-byte
// fields are big-endian; floats are IEEE-754 float32. Axes are E, N, U (m,
// m/s relative to the pad). Variances: P00 position (m^2), P11 velocity
// ((m/s)^2), P01 position-velocity covariance (m^2/s), after the cycle's
// updates. Innovations are measurement - prediction (m) with S = P00 + R
// (m^2) for the barometer; NaN when that measurement was not offered this
// cycle. GPS R = (2.5 m horizontal / 5 m vertical, doubled below 6
// satellites)^2. Counters are the filter's running totals, mod 65536.

#define IRIS_PACKET_ID_KALMAN_LOG 0xE2U

#define IRIS_PACKET_KALMAN_LOG_PHASE_OFFSET 0U             // uint8 flight state
#define IRIS_PACKET_KALMAN_LOG_FLAGS_OFFSET 1U             // uint8, IRIS_KALMAN_FLAG_*
#define IRIS_PACKET_KALMAN_LOG_GPS_SATELLITES_OFFSET 2U    // uint8, last fix
#define IRIS_PACKET_KALMAN_LOG_POSITION_OFFSET 3U          // 3 x float32 m
#define IRIS_PACKET_KALMAN_LOG_VELOCITY_OFFSET 15U         // 3 x float32 m/s
#define IRIS_PACKET_KALMAN_LOG_POSITION_VARIANCE_OFFSET 27U  // 3 x float32 P00
#define IRIS_PACKET_KALMAN_LOG_VELOCITY_VARIANCE_OFFSET 39U  // 3 x float32 P11
#define IRIS_PACKET_KALMAN_LOG_COVARIANCE_OFFSET 51U       // 3 x float32 P01
#define IRIS_PACKET_KALMAN_LOG_BARO_INNOVATION_OFFSET 63U  // float32 m
#define IRIS_PACKET_KALMAN_LOG_BARO_INNOVATION_VARIANCE_OFFSET 67U  // float32 m^2
#define IRIS_PACKET_KALMAN_LOG_BARO_SIGMA_OFFSET 71U       // float32 m, last R^0.5
#define IRIS_PACKET_KALMAN_LOG_GPS_INNOVATION_OFFSET 75U   // 3 x float32 m
#define IRIS_PACKET_KALMAN_LOG_MACH_OFFSET 87U             // float32
#define IRIS_PACKET_KALMAN_LOG_BARO_UPDATES_OFFSET 91U     // uint16
#define IRIS_PACKET_KALMAN_LOG_BARO_GATED_OFFSET 93U       // uint16 (Mach gate)
#define IRIS_PACKET_KALMAN_LOG_BARO_REJECTS_OFFSET 95U     // uint16 (5 sigma gate)
#define IRIS_PACKET_KALMAN_LOG_GPS_UPDATES_OFFSET 97U      // uint16
#define IRIS_PACKET_KALMAN_LOG_GPS_REJECTS_OFFSET 99U      // uint16 (per axis)
#define IRIS_PACKET_KALMAN_LOG_DATA_LENGTH 101U
#define IRIS_PACKET_KALMAN_LOG_EOF_OFFSET IRIS_PACKET_KALMAN_LOG_DATA_LENGTH
#define IRIS_PACKET_KALMAN_LOG_FRAME_LENGTH \
    (IRIS_PACKET_ID_LENGTH + IRIS_PACKET_KALMAN_LOG_DATA_LENGTH + \
     IRIS_PACKET_EOF_LENGTH)

#define IRIS_KALMAN_FLAG_INITIALIZED 0x01U     // predicting (attitude known once)
#define IRIS_KALMAN_FLAG_IMU_IGNORED 0x02U     // attitude lost: no-IMU predict
#define IRIS_KALMAN_FLAG_PAD_HOLD 0x04U        // pad still: state pinned to 0
#define IRIS_KALMAN_FLAG_BARO_GATED 0x08U      // Mach >= 0.8 gate active
#define IRIS_KALMAN_FLAG_BARO_ACCEPTED 0x10U   // baro update used this cycle
#define IRIS_KALMAN_FLAG_GPS_ACCEPTED 0x20U    // >= 1 GPS axis used this cycle
#define IRIS_KALMAN_FLAG_GPS_ORIGIN_SET 0x40U  // GPS origin (pad) established
#define IRIS_KALMAN_FLAG_HITL 0x80U            // HITL simulation active

// ---------------------------------------------------------------------------
// Ground-station USB status packet (not transmitted over LoRa)
// ---------------------------------------------------------------------------

#define IRIS_PACKET_ID_GROUND_STATION_STATUS 0xF0U
#define IRIS_PACKET_GROUND_STATION_STATUS_RSSI_OFFSET 0U
#define IRIS_PACKET_GROUND_STATION_STATUS_RSSI_LENGTH 1U
#define IRIS_PACKET_GROUND_STATION_STATUS_UPTIME_OFFSET 1U
#define IRIS_PACKET_GROUND_STATION_STATUS_UPTIME_LENGTH 4U
// Aperture's own GPS (LC86 on UART): the launch pad position when the
// Featherweight ground station's location is not available.
#define IRIS_PACKET_GROUND_STATION_STATUS_GPS_FIX_QUALITY_OFFSET 5U
#define IRIS_PACKET_GROUND_STATION_STATUS_GPS_FIX_QUALITY_LENGTH 1U
#define IRIS_PACKET_GROUND_STATION_STATUS_GPS_SATELLITES_OFFSET 6U
#define IRIS_PACKET_GROUND_STATION_STATUS_GPS_SATELLITES_LENGTH 1U
#define IRIS_PACKET_GROUND_STATION_STATUS_LATITUDE_OFFSET 7U
#define IRIS_PACKET_GROUND_STATION_STATUS_LATITUDE_LENGTH 4U
#define IRIS_PACKET_GROUND_STATION_STATUS_LONGITUDE_OFFSET 11U
#define IRIS_PACKET_GROUND_STATION_STATUS_LONGITUDE_LENGTH 4U
#define IRIS_PACKET_GROUND_STATION_STATUS_GPS_ALTITUDE_OFFSET 15U
#define IRIS_PACKET_GROUND_STATION_STATUS_GPS_ALTITUDE_LENGTH 4U
#define IRIS_PACKET_GROUND_STATION_STATUS_GPS_TIME_OFFSET 19U
#define IRIS_PACKET_GROUND_STATION_STATUS_GPS_TIME_LENGTH 4U
#define IRIS_PACKET_GROUND_STATION_STATUS_GPS_HDOP_OFFSET 23U
#define IRIS_PACKET_GROUND_STATION_STATUS_GPS_HDOP_LENGTH 2U
#define IRIS_PACKET_GROUND_STATION_STATUS_GPS_FIX_AGE_OFFSET 25U
#define IRIS_PACKET_GROUND_STATION_STATUS_GPS_FIX_AGE_LENGTH 2U
#define IRIS_PACKET_GROUND_STATION_STATUS_DATA_LENGTH 27U
#define IRIS_PACKET_GROUND_STATION_STATUS_EOF_OFFSET \
    IRIS_PACKET_GROUND_STATION_STATUS_DATA_LENGTH
#define IRIS_PACKET_GROUND_STATION_STATUS_FRAME_LENGTH 29U
// Pre-GPS Aperture firmware sent an 11-byte payload (RSSI, uptime, six zero
// bytes); hosts should still accept it.
#define IRIS_PACKET_GROUND_STATION_STATUS_LEGACY_DATA_LENGTH 11U

// RSSI is a signed int8 in dBm. Fix quality is NMEA GGA field 6 (0 = no fix).
// Latitude/longitude: big-endian signed int32, 1e-7 degree/count (north/east
// positive). GPS altitude: float32 m MSL. GPS time: uint32 Unix seconds (0 =
// unknown). HDOP: uint16, 0.01/count (0xFFFF = unknown). Fix age: uint16 ms
// since the last fix with quality >= 1 (0xFFFF = none, or 65.5 s or older).
// Position fields are zero while no fix has been seen.

// ---------------------------------------------------------------------------
// Heartbeat packet (0xFF)
// ---------------------------------------------------------------------------
// 0x06 Command acknowledgement (Witness -> host, on the link the command came
// in on). Sent for every canonical 0x05 command Witness handles except GPS
// assist DATA chunks (those are confirmed in bulk by 0x07). Payload:
//   [0..1] command opcode (u16 BE)
//   [2..7] the command's six data bytes, echoed (frame bytes 1..6)
//   [8]    result (IRIS_ACK_*)
//   [9..10] detail (u16 BE; e.g. chunks received for INCOMPLETE)
// The host matches an ack to its command by opcode + data and retries a
// command that is not acknowledged.
#define IRIS_PACKET_ID_COMMAND_ACK 0x06U
#define IRIS_PACKET_COMMAND_ACK_DATA_LENGTH 11U
#define IRIS_PACKET_COMMAND_ACK_FRAME_LENGTH     (IRIS_PACKET_ID_LENGTH + IRIS_PACKET_COMMAND_ACK_DATA_LENGTH + IRIS_PACKET_EOF_LENGTH)
#define IRIS_ACK_OK 0x00U            // applied / accepted
#define IRIS_ACK_REFUSED 0x01U       // not allowed now (flight state, mode)
#define IRIS_ACK_BAD_ARGUMENT 0x02U
#define IRIS_ACK_UNKNOWN 0x03U       // command not recognized
#define IRIS_ACK_BUSY 0x04U          // try again later
#define IRIS_ACK_INCOMPLETE 0x05U    // assist COMMIT with chunks missing (detail = received)
#define IRIS_ACK_BAD_CRC 0x06U       // assist COMMIT: CRC mismatch, transfer discarded
#define IRIS_ACK_BAD_FORMAT 0x07U    // assist COMMIT: records do not parse

// 0x07 Transfer status (Witness -> host), the reply to a GPS assist STATUS
// command (0x4751). d0-d1 of the STATUS command select the first chunk of the
// bitmap window (0xFFFF: start at the first missing chunk). Payload:
//   [0]    transfer id
//   [1]    state: 0 idle, 1 receiving, 2 complete (last transfer committed
//          and passed to the receiver), 3 failed (CRC or format)
//   [2..3] blob length (u16 BE)   [4..5] chunks expected   [6..7] chunks received
//   [8..9] first chunk of the bitmap window (u16 BE)
//   [10..73] bitmap: bit i (LSB first in each byte) = chunk window+i received
// A transfer survives a lost link: re-sending the same BEGIN (same id,
// length and CRC) keeps the chunks already received, so an upload resumes.
#define IRIS_PACKET_ID_TRANSFER_STATUS 0x07U
#define IRIS_PACKET_TRANSFER_STATUS_DATA_LENGTH 74U
#define IRIS_PACKET_TRANSFER_STATUS_BITMAP_BYTES 64U
#define IRIS_PACKET_TRANSFER_STATUS_FRAME_LENGTH     (IRIS_PACKET_ID_LENGTH + IRIS_PACKET_TRANSFER_STATUS_DATA_LENGTH + IRIS_PACKET_EOF_LENGTH)

// ---------------------------------------------------------------------------

#define IRIS_PACKET_ID_HEARTBEAT 0xFFU

#define IRIS_PACKET_HEARTBEAT_STATUS_OFFSET 0U
#define IRIS_PACKET_HEARTBEAT_STATUS_LENGTH IRIS_PACKET_STATUS_LENGTH
#define IRIS_PACKET_HEARTBEAT_UPTIME_OFFSET \
    (IRIS_PACKET_HEARTBEAT_STATUS_OFFSET + \
     IRIS_PACKET_HEARTBEAT_STATUS_LENGTH)
#define IRIS_PACKET_HEARTBEAT_UPTIME_LENGTH IRIS_PACKET_UPTIME_LENGTH
#define IRIS_PACKET_HEARTBEAT_DATA_LENGTH \
    (IRIS_PACKET_HEARTBEAT_STATUS_LENGTH + \
     IRIS_PACKET_HEARTBEAT_UPTIME_LENGTH)
#define IRIS_PACKET_HEARTBEAT_EOF_OFFSET IRIS_PACKET_HEARTBEAT_DATA_LENGTH
#define IRIS_PACKET_HEARTBEAT_FRAME_LENGTH \
    (IRIS_PACKET_ID_LENGTH + IRIS_PACKET_HEARTBEAT_DATA_LENGTH + \
     IRIS_PACKET_EOF_LENGTH)
#define IRIS_PACKET_HEARTBEAT_CAN_FRAME_COUNT \
    IRIS_CAN_FRAME_COUNT(IRIS_PACKET_HEARTBEAT_FRAME_LENGTH)

// ---------------------------------------------------------------------------
// Witness debug packet (0xE0)
// ---------------------------------------------------------------------------

#define IRIS_PACKET_ID_WITNESS_DEBUG 0xE0U

#define IRIS_PACKET_WITNESS_DEBUG_STATUS_OFFSET 0U
#define IRIS_PACKET_WITNESS_DEBUG_STATUS_LENGTH \
    IRIS_FIELD_WITNESS_STATUS_LENGTH
#define IRIS_PACKET_WITNESS_DEBUG_UPTIME_OFFSET 2U
#define IRIS_PACKET_WITNESS_DEBUG_UPTIME_LENGTH \
    IRIS_FIELD_WITNESS_UPTIME_LENGTH
#define IRIS_PACKET_WITNESS_DEBUG_MCU_TEMPERATURE_OFFSET 6U
#define IRIS_PACKET_WITNESS_DEBUG_MCU_TEMPERATURE_LENGTH \
    IRIS_FIELD_WITNESS_MCU_TEMPERATURE_LENGTH
#define IRIS_PACKET_WITNESS_DEBUG_BATTERY_VOLTAGE_OFFSET 7U
#define IRIS_PACKET_WITNESS_DEBUG_BATTERY_VOLTAGE_LENGTH \
    IRIS_FIELD_WITNESS_BATTERY_VOLTAGE_LENGTH
#define IRIS_PACKET_WITNESS_DEBUG_FILTERED_ACCEL_X_OFFSET 9U
#define IRIS_PACKET_WITNESS_DEBUG_FILTERED_ACCEL_Y_OFFSET 12U
#define IRIS_PACKET_WITNESS_DEBUG_FILTERED_ACCEL_Z_OFFSET 15U
#define IRIS_PACKET_WITNESS_DEBUG_ORIENTATION_X_OFFSET 18U
#define IRIS_PACKET_WITNESS_DEBUG_ORIENTATION_Y_OFFSET 22U
#define IRIS_PACKET_WITNESS_DEBUG_ORIENTATION_Z_OFFSET 26U
#define IRIS_PACKET_WITNESS_DEBUG_ORIENTATION_W_OFFSET 30U
#define IRIS_PACKET_WITNESS_DEBUG_POSITION_X_OFFSET 34U
#define IRIS_PACKET_WITNESS_DEBUG_POSITION_Y_OFFSET 38U
#define IRIS_PACKET_WITNESS_DEBUG_POSITION_Z_OFFSET 42U
#define IRIS_PACKET_WITNESS_DEBUG_VELOCITY_X_OFFSET 46U
#define IRIS_PACKET_WITNESS_DEBUG_VELOCITY_Y_OFFSET 50U
#define IRIS_PACKET_WITNESS_DEBUG_VELOCITY_Z_OFFSET 54U
#define IRIS_PACKET_WITNESS_DEBUG_LOW_ACCEL_X_OFFSET 58U
#define IRIS_PACKET_WITNESS_DEBUG_LOW_ACCEL_Y_OFFSET 60U
#define IRIS_PACKET_WITNESS_DEBUG_LOW_ACCEL_Z_OFFSET 62U
#define IRIS_PACKET_WITNESS_DEBUG_LSM6_TEMPERATURE_OFFSET 64U
#define IRIS_PACKET_WITNESS_DEBUG_HIGH_ACCEL_X_OFFSET 66U
#define IRIS_PACKET_WITNESS_DEBUG_HIGH_ACCEL_Y_OFFSET 68U
#define IRIS_PACKET_WITNESS_DEBUG_HIGH_ACCEL_Z_OFFSET 70U
#define IRIS_PACKET_WITNESS_DEBUG_GYRO_X_OFFSET 72U
#define IRIS_PACKET_WITNESS_DEBUG_GYRO_Y_OFFSET 74U
#define IRIS_PACKET_WITNESS_DEBUG_GYRO_Z_OFFSET 76U
#define IRIS_PACKET_WITNESS_DEBUG_PRESSURE_OFFSET 78U
#define IRIS_PACKET_WITNESS_DEBUG_MS5607_TEMPERATURE_OFFSET 81U
#define IRIS_PACKET_WITNESS_DEBUG_BAROMETRIC_ALTITUDE_OFFSET 83U
// Kalman filter altitude, m above the pad (same as the state packet slot).
#define IRIS_PACKET_WITNESS_DEBUG_KALMAN_ALTITUDE_OFFSET \
    IRIS_PACKET_WITNESS_DEBUG_BAROMETRIC_ALTITUDE_OFFSET
#define IRIS_PACKET_WITNESS_DEBUG_LATITUDE_OFFSET 87U
#define IRIS_PACKET_WITNESS_DEBUG_LONGITUDE_OFFSET 90U
#define IRIS_PACKET_WITNESS_DEBUG_GPS_TIME_OFFSET 93U
#define IRIS_PACKET_WITNESS_DEBUG_GPS_ALTITUDE_OFFSET 97U
#define IRIS_PACKET_WITNESS_DEBUG_DATA_LENGTH 101U
#define IRIS_PACKET_WITNESS_DEBUG_EOF_OFFSET \
    IRIS_PACKET_WITNESS_DEBUG_DATA_LENGTH
#define IRIS_PACKET_WITNESS_DEBUG_FRAME_LENGTH \
    (IRIS_PACKET_ID_LENGTH + IRIS_PACKET_WITNESS_DEBUG_DATA_LENGTH + \
     IRIS_PACKET_EOF_LENGTH)
#define IRIS_PACKET_WITNESS_DEBUG_CAN_FRAME_COUNT \
    IRIS_CAN_FRAME_COUNT(IRIS_PACKET_WITNESS_DEBUG_FRAME_LENGTH)

// ---------------------------------------------------------------------------
// IRIS debug packet (0xE1)
// ---------------------------------------------------------------------------

#define IRIS_PACKET_ID_IRIS_DEBUG 0xE1U

#define IRIS_PACKET_IRIS_DEBUG_STATUS_OFFSET 0U
#define IRIS_PACKET_IRIS_DEBUG_STATUS_LENGTH IRIS_FIELD_IRIS_STATUS_LENGTH
#define IRIS_PACKET_IRIS_DEBUG_UPTIME_OFFSET 2U
#define IRIS_PACKET_IRIS_DEBUG_UPTIME_LENGTH IRIS_FIELD_IRIS_UPTIME_LENGTH
#define IRIS_PACKET_IRIS_DEBUG_MCU_TEMPERATURE_OFFSET 6U
#define IRIS_PACKET_IRIS_DEBUG_MCU_TEMPERATURE_LENGTH \
    IRIS_FIELD_IRIS_MCU_TEMPERATURE_LENGTH
#define IRIS_PACKET_IRIS_DEBUG_BATTERY_VOLTAGE_OFFSET 7U
#define IRIS_PACKET_IRIS_DEBUG_BATTERY_VOLTAGE_LENGTH \
    IRIS_FIELD_IRIS_BATTERY_VOLTAGE_LENGTH
#define IRIS_PACKET_IRIS_DEBUG_CH1_CURRENT_OFFSET 9U
#define IRIS_PACKET_IRIS_DEBUG_CH2_CURRENT_OFFSET 10U
#define IRIS_PACKET_IRIS_DEBUG_CH3_CURRENT_OFFSET 11U
#define IRIS_PACKET_IRIS_DEBUG_DATA_LENGTH 12U
#define IRIS_PACKET_IRIS_DEBUG_EOF_OFFSET IRIS_PACKET_IRIS_DEBUG_DATA_LENGTH
#define IRIS_PACKET_IRIS_DEBUG_FRAME_LENGTH \
    (IRIS_PACKET_ID_LENGTH + IRIS_PACKET_IRIS_DEBUG_DATA_LENGTH + \
     IRIS_PACKET_EOF_LENGTH)
#define IRIS_PACKET_IRIS_DEBUG_CAN_FRAME_COUNT \
    IRIS_CAN_FRAME_COUNT(IRIS_PACKET_IRIS_DEBUG_FRAME_LENGTH)

// ---------------------------------------------------------------------------
// HITL sensor input packet (0x0C), host -> Witness over USB only
// ---------------------------------------------------------------------------
// Never bridged to CAN or radio. Field encodings match the Witness debug
// packet's native sensor counts; offsets are payload-relative. Witness
// computes barometric altitude from the pressure, as it does for the MS5607.

#define IRIS_PACKET_ID_HITL_SENSORS 0x0CU

#define IRIS_PACKET_HITL_STATUS_OFFSET 0U             // sender flags (below)
// 0x0C status flag: take GPS from Witness's own receiver (fed an RF GPS
// simulation, e.g. gps-sdr-sim + HackRF) instead of the packet's GPS fields.
#define IRIS_HITL_STATUS_GPS_FROM_RECEIVER 0x01U
#define IRIS_PACKET_HITL_SIM_TIME_OFFSET 2U           // uint32 simulation ms
#define IRIS_PACKET_HITL_LOW_ACCEL_X_OFFSET 6U        // int16, 0.488 mg/count
#define IRIS_PACKET_HITL_LOW_ACCEL_Y_OFFSET 8U
#define IRIS_PACKET_HITL_LOW_ACCEL_Z_OFFSET 10U
#define IRIS_PACKET_HITL_HIGH_ACCEL_X_OFFSET 12U      // int16, 10.417 mg/count
#define IRIS_PACKET_HITL_HIGH_ACCEL_Y_OFFSET 14U
#define IRIS_PACKET_HITL_HIGH_ACCEL_Z_OFFSET 16U
#define IRIS_PACKET_HITL_GYRO_X_OFFSET 18U            // int16, 70 mdps/count
#define IRIS_PACKET_HITL_GYRO_Y_OFFSET 20U
#define IRIS_PACKET_HITL_GYRO_Z_OFFSET 22U
#define IRIS_PACKET_HITL_IMU_TEMPERATURE_OFFSET 24U   // int16, 25 + raw/256 C
#define IRIS_PACKET_HITL_PRESSURE_OFFSET 26U          // uint24, Pa
#define IRIS_PACKET_HITL_BARO_TEMPERATURE_OFFSET 29U  // int16, centi-C
#define IRIS_PACKET_HITL_LATITUDE_OFFSET 31U          // int24, 0.0001 deg
#define IRIS_PACKET_HITL_LONGITUDE_OFFSET 34U         // int24, 0.0001 deg
#define IRIS_PACKET_HITL_GPS_ALTITUDE_OFFSET 37U      // float32, m MSL
#define IRIS_PACKET_HITL_GPS_SATELLITES_OFFSET 41U    // uint8
#define IRIS_PACKET_HITL_GPS_FIX_QUALITY_OFFSET 42U   // uint8, GGA quality
#define IRIS_PACKET_HITL_BATTERY_VOLTAGE_OFFSET 43U   // uint16, mV
#define IRIS_PACKET_HITL_DATA_LENGTH 45U
#define IRIS_PACKET_HITL_EOF_OFFSET IRIS_PACKET_HITL_DATA_LENGTH
#define IRIS_PACKET_HITL_FRAME_LENGTH \
    (IRIS_PACKET_ID_LENGTH + IRIS_PACKET_HITL_DATA_LENGTH + \
     IRIS_PACKET_EOF_LENGTH)

// ---------------------------------------------------------------------------
// HITL IMU batch packet (0x0E), host -> Witness over USB only
// ---------------------------------------------------------------------------
// Eight consecutive IMU samples at IRIS_PACKET_HITL_IMU_RATE_HZ, so in HITL
// the attitude estimator, filtered acceleration, position filter and NAND IMU
// log run at the real sensor's 960 Hz instead of the 100 Hz 0x0C rate. While
// these arrive, the IMU fields of 0x0C frames are ignored; 0x0C still supplies
// barometer, GPS, temperature and battery. Sample indices count from 0 in each
// HITL run. Native counts as in 0x0C. Never forwarded, so it is not included in
// IRIS_PACKET_MAX_DATA_LENGTH.

// 0x0E (ASCII SO): non-printing and never typed, unlike 0x0D (CR).
#define IRIS_PACKET_ID_HITL_IMU 0x0EU
#define IRIS_PACKET_HITL_IMU_RATE_HZ 960U
#define IRIS_PACKET_HITL_IMU_SAMPLES 8U
#define IRIS_PACKET_HITL_IMU_INDEX_OFFSET 0U           // uint32 first sample index
#define IRIS_PACKET_HITL_IMU_SAMPLE_OFFSET 4U
// Per sample: int16 low-g x,y,z, high-g x,y,z, gyro x,y,z.
#define IRIS_PACKET_HITL_IMU_SAMPLE_LENGTH 18U
#define IRIS_PACKET_HITL_IMU_DATA_LENGTH \
    (IRIS_PACKET_HITL_IMU_SAMPLE_OFFSET + \
     IRIS_PACKET_HITL_IMU_SAMPLES * IRIS_PACKET_HITL_IMU_SAMPLE_LENGTH)
#define IRIS_PACKET_HITL_IMU_FRAME_LENGTH \
    (IRIS_PACKET_ID_LENGTH + IRIS_PACKET_HITL_IMU_DATA_LENGTH + \
     IRIS_PACKET_EOF_LENGTH)

// Largest payload currently defined by the workbook. This excludes both the
// packet ID and EOF delimiter.
#define IRIS_PACKET_MAX_DATA_LENGTH IRIS_PACKET_WITNESS_DEBUG_DATA_LENGTH

// Catch accidental schema drift from the workbook totals. Payload lengths do
// not include the packet ID or EOF delimiter; frame lengths include both.
#if IRIS_PACKET_SENSORS_DATA_LENGTH != 21U
#error "Sensor packet payload must be 21 bytes"
#endif
#if IRIS_PACKET_STATE_DATA_LENGTH != 63U
#error "State packet payload must be 63 bytes"
#endif
#if IRIS_PACKET_HITL_BATTERY_VOLTAGE_OFFSET + \
        IRIS_FIELD_WITNESS_BATTERY_VOLTAGE_LENGTH != \
    IRIS_PACKET_HITL_DATA_LENGTH
#error "HITL packet field offsets do not fill the payload"
#endif
#if IRIS_PACKET_HITL_FRAME_LENGTH != 47U
#error "HITL application packet must be 47 bytes"
#endif
#if IRIS_PACKET_CAMERA_DATA_LENGTH != 17U
#error "Camera packet payload must be 17 bytes"
#endif
#if IRIS_PACKET_COMMAND_DATA_LENGTH != 8U
#error "Command packet payload must be 8 bytes"
#endif
#if IRIS_PACKET_HEARTBEAT_DATA_LENGTH != 6U
#error "Heartbeat packet payload must be 6 bytes"
#endif
#if IRIS_PACKET_WITNESS_DEBUG_DATA_LENGTH != 101U
#error "Witness debug packet payload must be 101 bytes"
#endif
#if IRIS_PACKET_IRIS_DEBUG_DATA_LENGTH != 12U
#error "IRIS debug packet payload must be 12 bytes"
#endif
#if IRIS_PACKET_SENSORS_GYRO_Z_OFFSET + \
        IRIS_PACKET_SENSORS_GYRO_COMPONENT_LENGTH != \
    IRIS_PACKET_SENSORS_DATA_LENGTH
#error "Sensor packet field offsets do not fill the payload"
#endif
#if IRIS_PACKET_STATE_GPS_SATELLITES_OFFSET + \
        IRIS_PACKET_STATE_GPS_SATELLITES_LENGTH != \
    IRIS_PACKET_STATE_DATA_LENGTH
#error "State packet field offsets do not fill the payload"
#endif
#if IRIS_PACKET_CAMERA_BATTERY_VOLTAGE_OFFSET + \
        IRIS_PACKET_CAMERA_BATTERY_VOLTAGE_LENGTH != \
    IRIS_PACKET_CAMERA_DATA_LENGTH
#error "Camera packet field offsets do not fill the payload"
#endif
#if IRIS_PACKET_COMMAND_VALUE_OFFSET + IRIS_PACKET_COMMAND_VALUE_LENGTH != \
    IRIS_PACKET_COMMAND_DATA_LENGTH
#error "Command packet field offsets do not fill the payload"
#endif
#if IRIS_PACKET_HEARTBEAT_UPTIME_OFFSET + \
        IRIS_PACKET_HEARTBEAT_UPTIME_LENGTH != \
    IRIS_PACKET_HEARTBEAT_DATA_LENGTH
#error "Heartbeat packet field offsets do not fill the payload"
#endif
#if IRIS_PACKET_WITNESS_DEBUG_GPS_ALTITUDE_OFFSET + \
        IRIS_FIELD_GPS_ALTITUDE_LENGTH != \
    IRIS_PACKET_WITNESS_DEBUG_DATA_LENGTH
#error "Witness debug packet field offsets do not fill the payload"
#endif
#if IRIS_PACKET_IRIS_DEBUG_CH3_CURRENT_OFFSET + \
        IRIS_FIELD_CH3_CURRENT_LENGTH != \
    IRIS_PACKET_IRIS_DEBUG_DATA_LENGTH
#error "IRIS debug packet field offsets do not fill the payload"
#endif
#if IRIS_PACKET_SENSORS_FRAME_LENGTH != 23U
#error "Sensor application packet must be 23 bytes"
#endif
#if IRIS_PACKET_STATE_FRAME_LENGTH != 65U
#error "State application packet must be 65 bytes"
#endif
#if IRIS_PACKET_CAMERA_FRAME_LENGTH != 19U
#error "Camera application packet must be 19 bytes"
#endif
#if IRIS_PACKET_COMMAND_FRAME_LENGTH != 10U
#error "Command application packet must be 10 bytes"
#endif
#if IRIS_PACKET_HEARTBEAT_FRAME_LENGTH != 8U
#error "Heartbeat application packet must be 8 bytes"
#endif
#if IRIS_PACKET_WITNESS_DEBUG_FRAME_LENGTH != 103U
#error "Witness debug application packet must be 103 bytes"
#endif
#if IRIS_PACKET_KALMAN_LOG_FRAME_LENGTH != 103U
#error "Kalman log packet must be 103 bytes (flash packet record limit)"
#endif
#if IRIS_PACKET_GROUND_STATION_STATUS_FRAME_LENGTH != \
    (IRIS_PACKET_ID_LENGTH + IRIS_PACKET_GROUND_STATION_STATUS_DATA_LENGTH + \
     IRIS_PACKET_EOF_LENGTH)
#error "Ground-station status frame length mismatch"
#endif
#if IRIS_PACKET_IRIS_DEBUG_FRAME_LENGTH != 14U
#error "IRIS debug application packet must be 14 bytes"
#endif
#if IRIS_PACKET_SENSORS_CAN_FRAME_COUNT != 5U || \
    IRIS_PACKET_STATE_CAN_FRAME_COUNT != 13U || \
    IRIS_PACKET_CAMERA_CAN_FRAME_COUNT != 4U || \
    IRIS_PACKET_COMMAND_CAN_FRAME_COUNT != 2U || \
    IRIS_PACKET_HEARTBEAT_CAN_FRAME_COUNT != 2U || \
    IRIS_PACKET_WITNESS_DEBUG_CAN_FRAME_COUNT != 21U || \
    IRIS_PACKET_IRIS_DEBUG_CAN_FRAME_COUNT != 3U
#error "CAN fragmentation counts must match the shared five-byte transport"
#endif
