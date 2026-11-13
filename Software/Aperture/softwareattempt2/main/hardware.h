#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"

// RA-01SH-P (SX1262) connections on the Adafruit ESP32-S3 Feather.
#define IRIS_PIN_RADIO_SPI_SCK GPIO_NUM_36
#define IRIS_PIN_RADIO_SPI_MOSI GPIO_NUM_35
#define IRIS_PIN_RADIO_SPI_MISO GPIO_NUM_37
#define IRIS_PIN_RADIO_CS GPIO_NUM_4
#define IRIS_PIN_RADIO_RESET GPIO_NUM_12
#define IRIS_PIN_RADIO_BUSY GPIO_NUM_34
#define IRIS_PIN_RADIO_DIO1 GPIO_NUM_10
#define IRIS_PIN_RADIO_DIO2 GPIO_NUM_11
#define IRIS_PIN_RADIO_DIO3 GPIO_NUM_9
#define IRIS_PIN_RADIO_RF_ENABLE GPIO_NUM_3

// These settings must match the transmitting radio.
#define IRIS_RADIO_FREQUENCY_HZ 926000000U
// SX1262 drive level into the Ra-01SH-P front end (PA gain ~26 dB, +29 dBm
// saturated). Ai-Thinker: the FEM input must not exceed +3 dBm (0 dBm with the
// antenna port unloaded) or the PA is damaged; 0..+3 dBm is recommended for
// full output. The SX1262's own -9..+22 dBm range does NOT apply here.
#define IRIS_RADIO_TX_POWER_DBM -17  // boot default: SX1262 low-power PA, bench-safe; raise with IRIS_COMMAND_RADIO_POWER
#define IRIS_RADIO_TX_POWER_MIN_DBM -17  // below -9 the SX1262 low-power PA is used
#define IRIS_RADIO_TX_POWER_MAX_DBM 3
static_assert(IRIS_RADIO_TX_POWER_DBM >= IRIS_RADIO_TX_POWER_MIN_DBM && IRIS_RADIO_TX_POWER_DBM <= IRIS_RADIO_TX_POWER_MAX_DBM,
              "Ra-01SH-P: SX1262 TX power must be -17..+3 dBm (FEM input limit)");
#define IRIS_RADIO_DIO2_RF_SWITCH_ENABLE 1
// The SX1262 command has no separate polarity bit. Set to 1 to select the
// opposite DIO2 control state for an RF switch with opposite polarity.
#define IRIS_RADIO_DIO2_RF_SWITCH_INVERT 0
// DIO2 is also wired to the MCU, so it may be monitored as an input.
#define IRIS_RADIO_DIO2_MCU_INPUT 1
#define IRIS_RADIO_MAX_PACKET_LENGTH 128U
#define IRIS_RADIO_SPREADING_FACTOR 7U
#define IRIS_RADIO_BANDWIDTH_HZ 500000U  // SF7/500: ~6 % downlink duty, tolerant of untrimmed crystals (no TCXO); see HITL/LORA_MODULATION_OPTIONS.md
#define IRIS_RADIO_CODING_RATE 1U // 4/5
#define IRIS_RADIO_PREAMBLE_LENGTH 8U

// Quectel LC86 GPS (L80-R footprint) on the MCU UART. RX/TX are named from the
// ESP32-S3 side: MCU RX <- GPS TX, MCU TX -> GPS RX.
#define IRIS_GPS_UART_PORT UART_NUM_1
#define IRIS_GPS_BAUD_RATE 115200U // LC86G factory default, 8N1
#define IRIS_PIN_GPS_UART_RX GPIO_NUM_38
#define IRIS_PIN_GPS_UART_TX GPIO_NUM_39
#define IRIS_PIN_GPS_PPS GPIO_NUM_18
#define IRIS_PIN_GPS_RESET GPIO_NUM_14
// RESET_N is active low with an internal 1.8 V pull-up: the line is driven
// open-drain, low for 100 ms at boot, then released. Set to 0 only if the
// board drives RESET_N through an inverting transistor (then GPIO14 is
// push-pull and high means reset).
#define IRIS_GPS_RESET_ACTIVE_LOW 1
