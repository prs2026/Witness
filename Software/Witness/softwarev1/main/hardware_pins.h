#pragma once

#include "driver/gpio.h"

// Witness ESP32-S3FN8 board pin map. Keep physical board assignments here so
// application modules do not contain literal GPIO numbers.

// General-purpose and monitoring signals.
#define HW_PIN_GPIO0          GPIO_NUM_0
#define HW_PIN_VSENSE         GPIO_NUM_1
#define HW_PIN_GPIO3          GPIO_NUM_3
#define HW_PIN_GPIO45         GPIO_NUM_45
#define HW_PIN_GPIO46         GPIO_NUM_46

// Radio interface.
#define HW_PIN_RAD_DIO3       GPIO_NUM_2
#define HW_PIN_RAD_DIO2       GPIO_NUM_6
#define HW_PIN_RAD_DIO1       GPIO_NUM_7
#define HW_PIN_RAD_RESET      GPIO_NUM_8
#define HW_PIN_RAD_CS         GPIO_NUM_16
#define HW_PIN_RAD_BUSY       GPIO_NUM_39
#define HW_PIN_RAD_RF_ENABLE  GPIO_NUM_40

// Broken-out MCU UART1 interface (115200 baud, 8N1 by default).
// Normal wiring is ESP32 RX = GPIO4, TX = GPIO5. Set to 1 to exchange them for
// a GPS harness or breakout whose RX/TX are crossed the other way.
#define HW_MCU_UART_SWAP_RX_TX 0
#if HW_MCU_UART_SWAP_RX_TX
#define HW_PIN_MCU_UART_TX    GPIO_NUM_4
#define HW_PIN_MCU_UART_RX    GPIO_NUM_5
#else
#define HW_PIN_MCU_UART_TX    GPIO_NUM_5
#define HW_PIN_MCU_UART_RX    GPIO_NUM_4
#endif
// GPS I2C reuses the same broken-out pair. UART1 must not run concurrently.
#define HW_PIN_GPS_I2C_SDA    GPIO_NUM_4
#define HW_PIN_GPS_I2C_SCL    GPIO_NUM_5
// Set to 0 on reduced boards to skip all radio initialization and service.
#define HW_RADIO_ENABLED      1

// SPI2 interface.
#define HW_PIN_SPI2_IO3       GPIO_NUM_9
#define HW_PIN_FLASH_CS       GPIO_NUM_10
#define HW_PIN_SPI2_IO0       GPIO_NUM_11
#define HW_PIN_SPI2_CLOCK     GPIO_NUM_12
#define HW_PIN_SPI2_IO1       GPIO_NUM_13
#define HW_PIN_SPI2_IO2       GPIO_NUM_14

// SPI3 interface.
#define HW_PIN_SPI3_CLOCK     GPIO_NUM_15
#define HW_PIN_SPI3_COPI      GPIO_NUM_17
#define HW_PIN_SPI3_CIPO      GPIO_NUM_18

// Built-in USB Serial/JTAG connection.
#define HW_PIN_USB_D_MINUS    GPIO_NUM_19
#define HW_PIN_USB_D_PLUS     GPIO_NUM_20

// Pressure sensor chip select.
#define HW_PIN_MS56_CS        GPIO_NUM_21

// Indicators and buzzer.
#define HW_PIN_LED_RED        GPIO_NUM_33
#define HW_PIN_LED_GREEN      GPIO_NUM_34
#define HW_PIN_LED_BLUE       GPIO_NUM_47
#define HW_PIN_BUZZER         GPIO_NUM_48

// LSM inertial sensor.
#define HW_PIN_LSM_CS         GPIO_NUM_35
#define HW_PIN_LSM_INT1       GPIO_NUM_41
#define HW_PIN_LSM_INT2       GPIO_NUM_42

// CAN transceiver.
#define HW_PIN_CAN_TX         GPIO_NUM_37
#define HW_PIN_CAN_RX         GPIO_NUM_36
#define HW_CAN_BITRATE        500000U



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
#define IRIS_RADIO_SPREADING_FACTOR 7U
#define IRIS_RADIO_BANDWIDTH_HZ 500000U  // SF7/500: ~6 % downlink duty, tolerant of untrimmed crystals (no TCXO); see HITL/LORA_MODULATION_OPTIONS.md
#define IRIS_RADIO_CODING_RATE 1U // 4/5
#define IRIS_RADIO_PREAMBLE_LENGTH 8U
