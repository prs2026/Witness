#pragma once

#include "driver/gpio.h"

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
#define IRIS_RADIO_FREQUENCY_HZ 915000000U
#define IRIS_RADIO_TX_POWER_DBM 0 // SX1262 range: -9 through +22 dBm
#define IRIS_RADIO_DIO2_RF_SWITCH_ENABLE 1
// The SX1262 command has no separate polarity bit. Set to 1 to select the
// opposite DIO2 control state for an RF switch with opposite polarity.
#define IRIS_RADIO_DIO2_RF_SWITCH_INVERT 0
// DIO2 is also wired to the MCU, so it may be monitored as an input.
#define IRIS_RADIO_DIO2_MCU_INPUT 1
#define IRIS_RADIO_SPREADING_FACTOR 7U
#define IRIS_RADIO_BANDWIDTH_HZ 125000U
#define IRIS_RADIO_CODING_RATE 1U // 4/5
#define IRIS_RADIO_PREAMBLE_LENGTH 8U
