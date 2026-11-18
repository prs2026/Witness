#pragma once

#include <cstdint>

#include "esp_err.h"

// Persistent LoRa drive level (dBm), so a reset in flight does not drop the
// downlink back to the -17 dBm bench default (audit W1/F6). NVS namespace
// "radio", key "tx_dbm". The front-end ON/OFF state is deliberately NOT stored:
// an "off" must never survive a reset.
namespace radio_power_store {

// True and *tx_dbm set only when a value is stored and inside
// IRIS_RADIO_TX_POWER_MIN_DBM..MAX_DBM. False when absent, out of range, or
// NVS is not initialised.
bool load(std::int8_t &tx_dbm);

// Stores the level (range checked, ESP_ERR_INVALID_ARG outside). Skips the
// flash write when the stored value is already equal. Blocking flash write
// (a few ms, up to ~50 ms if NVS must erase a page): call from the command
// task on the pad, never from the sensor task. ESP_ERR_NVS_NOT_INITIALIZED if
// NVS is not up.
esp_err_t save(std::int8_t tx_dbm);

}  // namespace radio_power_store
