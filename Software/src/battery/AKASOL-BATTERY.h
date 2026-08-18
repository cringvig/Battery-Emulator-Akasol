#ifndef AKASOL_BATTERY_H
#define AKASOL_BATTERY_H

#include <Arduino.h>
#include "CanBattery.h"

/* ============================================================================
 * AKASOL AKASYSTEM (15 OEM 50 PRC) battery driver
 * Single-tray (BMM01) configuration.
 *
 * Based on:
 *  - PublicCAN.dbc supplied by the user
 *  - AKASOL User Manual AKASYSTEM 15 OEM 50 PRC, Doc No. 3-024001TEN_0001, v1.2
 *
 * IMPORTANT - THIS IS UNTESTED AGAINST REAL HARDWARE.
 * Verify every value against your own CAN bus traffic before trusting it
 * for anything that touches contactors or high voltage.
 * ========================================================================= */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// GPIO CONFIGURATION - PLACEHOLDER PINS, YOU MUST ADJUST THESE FOR YOUR BOARD
// The Akasol battery needs three discrete 12V/24V signals in addition to CAN:
//   - KL30       : permanent LV supply to the BMU (24V)
//   - KL15/Wake  : "Battery Wake" - wakes the BMU CPUs and starts CAN
//   - KL30_safe  : safety-loop supply (must be bounce-free BEFORE req_batuse=1,
//                  otherwise the battery gets stuck in Init mode)
// These are normally driven through a relay/MOSFET from a free GPIO on your
// board (Stark/LilyGo/etc). Pick pins that are free on your hardware.
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#define AKASOL_PIN_KL30 16       // Expansion header IO16 - drives KL30 relay
#define AKASOL_PIN_KL15_WAKE 47  // Expansion header IO47 - drives Battery Wake relay
#define AKASOL_PIN_KL30_SAFE 15  // Expansion header IO15 - drives KL30_safe / eStop loop relay
// If AKASOL_PIN_x is left at -1, the driver will NOT attempt to drive that
// pin, and you must supply the signal by some other means (e.g. permanently
// wired if your setup allows it) - useful for bench testing on CAN alone.

class AkasolBattery : public CanBattery {
 public:
  AkasolBattery() : CanBattery(CAN_Speed::CAN_SPEED_250KBPS) {}

  virtual void setup();
  virtual void update_values();
  virtual void handle_incoming_can_frame(CAN_frame rx_frame);
  virtual void transmit_can(unsigned long currentMillis);
  virtual const char* interface_name() { return "AKASOL"; }

  static constexpr const char* Name = "AKASOL AKASYSTEM 15 OEM 50 PRC";

 private:
  // --- CAN IDs (decoded from PublicCAN.dbc, 29-bit extended) ---------------
  // TX: emulator (VCU) -> battery (BMM01)
  static const uint32_t AKASOL_VCU1_TO_BMM01 = 0x0CEFF3E3;

  // RX: battery (BMM01) -> emulator (VCU)
  static const uint32_t AKASOL_BMM01_STATE = 0x0CFF10F3;
  static const uint32_t AKASOL_BMM01_CELLVOLTAGES = 0x18FF11F3;
  static const uint32_t AKASOL_BMM01_CELLTEMPERATURES = 0x18FF12F3;
  static const uint32_t AKASOL_BMM01_ELECTRICS = 0x18FF14F3;
  static const uint32_t AKASOL_BMM01_LIMITS1 = 0x0CFF1AF3;  // charge/discharge current limits
  static const uint32_t AKASOL_BMM01_LIMITS2 = 0x0CFF1BF3;  // voltage/power limits
  static const uint32_t AKASOL_BMM01_CAPACITY = 0x18FCEAF3;  // SOC (Ah based), Ah capacity
  static const uint32_t AKASOL_BMM01_SOH = 0x18FF1CF3;

  // --- Wake / init state machine (see chapter 6.2.1 "Start Up" of manual) --
  enum class AkasolState {
    INIT_HW,      // driving GPIOs low, nothing started yet
    WAKING_UP,    // GPIOs asserted, waiting before requesting battery use
    REQUEST_USE,  // sending req_batuse=1, waiting for Operational
    RUNNING       // battery reported Operational at least once
  };
  AkasolState akasol_state = AkasolState::INIT_HW;
  unsigned long state_entry_time = 0;

  // --- Parsed raw values from CAN --------------------------------------
  int16_t battery_current_dA = 0;      // 0.1A resolution, already matches Battery-Emulator unit
  int16_t battery_voltage_dV = 0;      // 0.1V resolution
  uint16_t cell_voltage_max_mV = 0;
  uint16_t cell_voltage_min_mV = 0;
  int16_t cell_temperature_max_dC = 0;  // 0.1 degC
  int16_t cell_temperature_min_dC = 0;
  uint16_t soc_pptt = 0;               // 0.01% resolution (basis points)
  uint16_t soh_pptt = 0;
  uint16_t max_avail_capacity_Ah_x10 = 0;
  uint16_t lim_charge_curr_A_x10 = 0;
  uint16_t lim_discharge_curr_A_x10 = 0;
  uint16_t lim_max_volt_V_x10 = 0;
  uint16_t lim_min_volt_V_x10 = 0;

  bool stat_operational = false;
  bool stat_standby = false;
  bool stat_error = false;
  bool stat_precharge = false;
  bool contactor_pos = false;
  bool contactor_neg = false;

  uint8_t bmm_alive_counter_mirror = 0;  // BMM01_cnt_MirroredAliveCounter (echo of our alive counter)

  bool CAN_battery_still_alive_frames_received = false;

  // --- Our TX alive counter ---
  uint8_t vcu_alive_counter = 0;

  // --- Helper: drive the three discrete GPIO signals ---
  void set_kl30(bool state);
  void set_kl15_wake(bool state);
  void set_kl30_safe(bool state);
};

#endif  // AKASOL_BATTERY_H
