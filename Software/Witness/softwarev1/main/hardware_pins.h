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

// MCU UART interface.
#define HW_PIN_MCU_UART_TX    GPIO_NUM_4
#define HW_PIN_MCU_UART_RX    GPIO_NUM_5

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
