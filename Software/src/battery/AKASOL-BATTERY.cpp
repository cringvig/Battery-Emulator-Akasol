#include "AKASOL-BATTERY.h"
#include "../datalayer/datalayer.h"
#include "../devboard/utils/events.h"

/* ============================================================================
 * AKASOL AKASYSTEM 15 OEM 50 PRC - single tray (BMM01) driver
 *
 * Reference data taken directly from the supplied PublicCAN.dbc and from the
 * AKASOL User Manual (doc 3-024001TEN_0001 v1.2):
 *   - Nominal voltage 655V, range 540-756V, 50Ah, 33kWh, 15 modules x 12 cells
 *     = 180 cells in series (12s1p per module, 15s modules => 180s1p pack)
 *   - Public CAN runs at 250 kbit/s (SAE J1939)
 *   - Startup sequence per chapter 6.2.1 "Start Up"
 * ========================================================================= */

#define AKASOL_NOMINAL_CAPACITY_AH 50
#define AKASOL_NOMINAL_ENERGY_WH 33000
#define AKASOL_NUMBER_OF_CELLS 180  // 15 modules * 12 cells, all in series
#define AKASOL_MAX_PACK_VOLTAGE_DV 7560  // 756.0V
#define AKASOL_MIN_PACK_VOLTAGE_DV 5400  // 540.0V

// Time (ms) to hold KL30/KL30_safe on before asserting Battery Wake.
// Manual does not give an exact value; this is a conservative placeholder.
#define AKASOL_T_KL30_SETTLE_MS 500

// How often to transmit the VCU1_to_BMM01 message. Manual does not specify an
// exact cyclic rate; 100ms is a common BMS heartbeat rate and safely inside
// any reasonable alive-counter timeout.
#define AKASOL_TX_INTERVAL_MS 100

void AkasolBattery::set_kl30(bool state) {
#if AKASOL_PIN_KL30 >= 0
  pinMode(AKASOL_PIN_KL30, OUTPUT);
  digitalWrite(AKASOL_PIN_KL30, state ? HIGH : LOW);
#endif
}

void AkasolBattery::set_kl15_wake(bool state) {
#if AKASOL_PIN_KL15_WAKE >= 0
  pinMode(AKASOL_PIN_KL15_WAKE, OUTPUT);
  digitalWrite(AKASOL_PIN_KL15_WAKE, state ? HIGH : LOW);
#endif
}

void AkasolBattery::set_kl30_safe(bool state) {
#if AKASOL_PIN_KL30_SAFE >= 0
  pinMode(AKASOL_PIN_KL30_SAFE, OUTPUT);
  digitalWrite(AKASOL_PIN_KL30_SAFE, state ? HIGH : LOW);
#endif
}

void AkasolBattery::setup() {
  strncpy(datalayer.system.info.battery_protocol, Name, 63);

  datalayer.battery.info.chemistry = battery_chemistry_enum::NMC;
  datalayer.battery.info.number_of_cells = AKASOL_NUMBER_OF_CELLS;
  datalayer.battery.info.total_capacity_Wh = AKASOL_NOMINAL_ENERGY_WH;
  datalayer.battery.info.max_design_voltage_dV = AKASOL_MAX_PACK_VOLTAGE_DV;
  datalayer.battery.info.min_design_voltage_dV = AKASOL_MIN_PACK_VOLTAGE_DV;

  // Do not allow contactor closing / current flow until the state machine
  // below has taken the battery through Init -> Standby -> Operational.
 datalayer.battery.status.real_bms_status = BMS_STANDBY;

  akasol_state = AkasolState::INIT_HW;
  state_entry_time = millis();

  set_kl30(false);
  set_kl15_wake(false);
  set_kl30_safe(false);
}

void AkasolBattery::handle_incoming_can_frame(CAN_frame rx_frame) {
  switch (rx_frame.ID) {
    case AKASOL_BMM01_STATE: {
      uint8_t b0 = rx_frame.data.u8[0];
      stat_precharge = (b0 & 0x04) != 0;   // bit 2
      stat_operational = (b0 & 0x08) != 0;  // bit 3
      stat_standby = (b0 & 0x02) != 0;      // bit 1
      stat_error = (b0 & 0x40) != 0;        // bit 6

      uint8_t b2 = rx_frame.data.u8[2];
      contactor_pos = (b2 & 0x08) != 0;  // bit 19 -> byte2 bit3
      contactor_neg = (b2 & 0x10) != 0;  // bit 20 -> byte2 bit4

      bmm_alive_counter_mirror = rx_frame.data.u8[7];  // byte 56-63

      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      CAN_battery_still_alive_frames_received = true;
      break;
    }
    case AKASOL_BMM01_ELECTRICS: {
      battery_current_dA = (int16_t)(rx_frame.data.u8[0] | (rx_frame.data.u8[1] << 8));
      battery_voltage_dV = (int16_t)(rx_frame.data.u8[2] | (rx_frame.data.u8[3] << 8));
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    }
    case AKASOL_BMM01_CELLVOLTAGES: {
      cell_voltage_max_mV = rx_frame.data.u8[0] | (rx_frame.data.u8[1] << 8);
      cell_voltage_min_mV = rx_frame.data.u8[2] | (rx_frame.data.u8[3] << 8);
      break;
    }
    case AKASOL_BMM01_CELLTEMPERATURES: {
      cell_temperature_max_dC = (int16_t)(rx_frame.data.u8[0] | (rx_frame.data.u8[1] << 8));
      cell_temperature_min_dC = (int16_t)(rx_frame.data.u8[2] | (rx_frame.data.u8[3] << 8));
      break;
    }
    case AKASOL_BMM01_LIMITS1: {
      // bytes 4-5: charge current limit (0.1A), bytes 6-7: discharge current limit (0.1A)
      lim_charge_curr_A_x10 = rx_frame.data.u8[4] | (rx_frame.data.u8[5] << 8);
      lim_discharge_curr_A_x10 = rx_frame.data.u8[6] | (rx_frame.data.u8[7] << 8);
      break;
    }
    case AKASOL_BMM01_LIMITS2: {
      lim_max_volt_V_x10 = rx_frame.data.u8[0] | (rx_frame.data.u8[1] << 8);
      lim_min_volt_V_x10 = rx_frame.data.u8[2] | (rx_frame.data.u8[3] << 8);
      break;
    }
    case AKASOL_BMM01_CAPACITY: {
      // SOC scale factor is 0.0025% per bit -> multiply by 25 to get 0.01% (pptt)
      uint16_t raw_soc = rx_frame.data.u8[0] | (rx_frame.data.u8[1] << 8);
      soc_pptt = (uint16_t)((uint32_t)raw_soc * 25 / 100);
      max_avail_capacity_Ah_x10 = rx_frame.data.u8[2] | (rx_frame.data.u8[3] << 8);
      break;
    }
    case AKASOL_BMM01_SOH: {
      // 0.1% resolution -> multiply by 10 to get pptt (0.01%)... value is already *10 vs %,
      // dbc scale 0.1 means raw*0.1 = percent, so pptt = raw*10
      uint16_t raw_soh = rx_frame.data.u8[0] | (rx_frame.data.u8[1] << 8);
      soh_pptt = raw_soh * 10;
      break;
    }
    default:
      break;
  }
}

void AkasolBattery::update_values() {
  datalayer.battery.status.voltage_dV = battery_voltage_dV;
  datalayer.battery.status.current_dA = battery_current_dA;
  datalayer.battery.status.real_soc = soc_pptt;
  datalayer.battery.status.soh_pptt = soh_pptt > 0 ? soh_pptt : 10000;

  datalayer.battery.status.cell_max_voltage_mV = cell_voltage_max_mV;
  datalayer.battery.status.cell_min_voltage_mV = cell_voltage_min_mV;

  datalayer.battery.status.temperature_max_dC = cell_temperature_max_dC;
  datalayer.battery.status.temperature_min_dC = cell_temperature_min_dC;

  // Battery reports its own dynamic charge/discharge current limits (BMM01_limits1).
  // Use them directly rather than recomputing from SOC/temperature tables.
  uint32_t charge_current_dA = lim_charge_curr_A_x10;
  uint32_t discharge_current_dA = lim_discharge_curr_A_x10;

  datalayer.battery.status.max_charge_power_W =
      (uint32_t)((uint64_t)charge_current_dA * battery_voltage_dV / 100);
  datalayer.battery.status.max_discharge_power_W =
      (uint32_t)((uint64_t)discharge_current_dA * battery_voltage_dV / 100);

  datalayer.battery.status.remaining_capacity_Wh =
      (uint32_t)((uint64_t)datalayer.battery.info.total_capacity_Wh * soc_pptt / 10000);

  // Do not allow HV use to be requested (or reported "ready") until we have
  // actually completed the wake-up handshake and the battery reports itself
  // Operational. Until then, force limits to zero as a safety default.
  if (akasol_state != AkasolState::RUNNING || stat_error || !stat_operational) {
    datalayer.battery.status.max_charge_power_W = 0;
    datalayer.battery.status.max_discharge_power_W = 0;
  }

  if (stat_error) {
    datalayer.battery.status.real_bms_status = BMS_FAULT;
    set_event(EVENT_BATTERY_CAUTION, 0);
  } else if (stat_operational) {
    datalayer.battery.status.real_bms_status = BMS_ACTIVE;
  } else {
    datalayer.battery.status.real_bms_status = BMS_STANDBY;
  }
}


void AkasolBattery::transmit_can(unsigned long currentMillis) {
  static unsigned long last_tx = 0;

  // --- GPIO / wake-up state machine (manual chapter 6.2.1 Start Up) --------
  switch (akasol_state) {
    case AkasolState::INIT_HW:
      set_kl30(true);
      set_kl30_safe(true);  // Must be bounce-free ON before req_batuse=1, see manual.
      set_kl15_wake(false);
      state_entry_time = currentMillis;
      akasol_state = AkasolState::WAKING_UP;
      break;

    case AkasolState::WAKING_UP:
      if (currentMillis - state_entry_time >= AKASOL_T_KL30_SETTLE_MS) {
        set_kl15_wake(true);  // "Battery Wake" -> starts BMU CPUs and CAN
        state_entry_time = currentMillis;
        akasol_state = AkasolState::REQUEST_USE;
      }
      break;

    case AkasolState::REQUEST_USE:
      // Wait until we see the battery in Standby (CAN alive) before asking for
      // HV use. If we never hear from it, we simply keep sending req_batuse=0.
      if (CAN_battery_still_alive_frames_received && stat_standby) {
        akasol_state = AkasolState::RUNNING;  // proceed to request HV use below
      }
      break;

    case AkasolState::RUNNING:
      // Nothing to do here; message content below handles req_batuse=1.
      break;
  }

  // --- Cyclic VCU1_to_BMM01 message ----------------------------------------
  if (currentMillis - last_tx < AKASOL_TX_INTERVAL_MS) {
    return;
  }
  last_tx = currentMillis;

  bool request_use = (akasol_state == AkasolState::RUNNING) && !stat_error;

  CAN_frame AKASOL_VCU_frame = {.FD = false,
                                 .ext_ID = true,
                                 .DLC = 8,
                                 .ID = AKASOL_VCU1_TO_BMM01,
                                 .data = {0, 0, 0, 0, 0, 0, 0, 0}};

  uint8_t byte0 = 0;
  if (request_use) {
    byte0 |= 0x01;  // bit0 use_req
  }
  // bit4 isolated_bmu: 0 = LV ground connected to chassis (road use).
  // Set to 1 instead if your LV ground is insulated from chassis (marine use).
  byte0 |= 0x00;  // isolated_bmu = 0
  // bit5 req_iso_meas, bit7 iso_disable: leave at 0 unless you implement the
  // insulation-monitoring handshake described in manual chapter 6.2.5.

  AKASOL_VCU_frame.data.u8[0] = byte0;
  AKASOL_VCU_frame.data.u8[2] = vcu_alive_counter;  // bits 16-23
  AKASOL_VCU_frame.data.u8[6] = 0xAA;               // val_code low byte  -> 0x55AA
  AKASOL_VCU_frame.data.u8[7] = 0x55;               // val_code high byte

  transmit_can_frame(&AKASOL_VCU_frame);

  vcu_alive_counter++;
}
