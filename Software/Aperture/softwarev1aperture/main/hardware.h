#pragma once

#include "driver/gpio.h"

// Hardware pin map for the custom board built around an Adafruit Feather
// HUZZAH32 (ESP32), an LC86 GPS receiver, and an RA-01-SH-P radio module.
// Keep board wiring here so application code does not depend on raw GPIO
// numbers.

// ---------------------------------------------------------------------------
// LC86 GPS
// ---------------------------------------------------------------------------

// GPS TXD1 -> MCU RX and MCU TX -> GPS RXD1.
#define IRIS_PIN_GPS_UART_RX GPIO_NUM_16
#define IRIS_PIN_GPS_UART_TX GPIO_NUM_17

// GPS one-pulse-per-second output and reset input.
#define IRIS_PIN_GPS_PPS GPIO_NUM_26
#define IRIS_PIN_GPS_RESET GPIO_NUM_36

// ---------------------------------------------------------------------------
// RA-01-SH-P radio
// ---------------------------------------------------------------------------

// SPI3 bus shared with the RA-01 module.
#define IRIS_PIN_RADIO_SPI_SCK GPIO_NUM_5
#define IRIS_PIN_RADIO_SPI_MOSI GPIO_NUM_18
#define IRIS_PIN_RADIO_SPI_MISO GPIO_NUM_19
#define IRIS_PIN_RADIO_CS GPIO_NUM_22

// Radio control and interrupt lines.
#define IRIS_PIN_RADIO_RESET GPIO_NUM_33
#define IRIS_PIN_RADIO_BUSY GPIO_NUM_13
#define IRIS_PIN_RADIO_RF_ENABLE GPIO_NUM_23
#define IRIS_PIN_RADIO_DIO1 GPIO_NUM_15
#define IRIS_PIN_RADIO_DIO2 GPIO_NUM_32
#define IRIS_PIN_RADIO_DIO3 GPIO_NUM_14

// GPIO13 is connected to the Feather D13/LED net as well as RAD_BUSY in the
// schematic. It must be treated as the radio BUSY input, not as a software-
// controlled heartbeat LED output.

// The schematic does not show an ESP32-connected battery-voltage divider,
// PAC1931 current monitor, MCP23008 expander, CAN transceiver, or external
// load-switch outputs. Those devices intentionally have no definitions here.

