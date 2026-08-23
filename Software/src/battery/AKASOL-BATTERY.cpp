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

// Time (ms) to hold KL30_safe alone before asserting KL30.
// Manual does not give an exact value; this is a conservative placeholder.
#define AKASOL_T_KL30_SETTLE_MS 500

// Time (ms) to hold KL30_safe+KL30 before asserting Battery Wake - matches
// the "1s pause" step of the sequence manually verified to work on real
// hardware (KL30_safe -> KL30 -> 1s -> Wake).
#define AKASOL_T_KL30_TO_WAKE_MS 1000

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
      stat_init = (b0 & 0x01) != 0;          // bit 0
      stat_standby = (b0 & 0x02) != 0;       // bit 1
      stat_precharge = (b0 & 0x04) != 0;     // bit 2
      stat_operational = (b0 & 0x08) != 0;   // bit 3
      stat_disabling = (b0 & 0x10) != 0;     // bit 4
      stat_error = (b0 & 0x40) != 0;         // bit 6

      uint8_t b1 = rx_frame.data.u8[1];
      flag_drive = (b1 & 0x01) != 0;   // bit 8
      flag_charge = (b1 & 0x02) != 0;  // bit 9

      uint8_t b2 = rx_frame.data.u8[2];
      flag_wake = (b2 & 0x01) != 0;         // bit 16
      flag_isodisable = (b2 & 0x02) != 0;   // bit 17
      contactor_pos = (b2 & 0x08) != 0;     // bit 19 -> byte2 bit3
      contactor_neg = (b2 & 0x10) != 0;     // bit 20 -> byte2 bit4
      flag_contactor_precha = (b2 & 0x20) != 0;  // bit 21
      flag_chargecomplete = (b2 & 0x40) != 0;    // bit 22
      flag_extshutdownreq = (b2 & 0x80) != 0;    // bit 23

      uint8_t b3 = rx_frame.data.u8[3];
      // NOTE: per PublicCAN.dbc, both HVILState and eStopLoopClosed are documented as
      // "0 = closed, 1 = open" - i.e. the RAW bit is 1 when the loop is OPEN, not closed.
      // The variable names here mean "is closed", so we invert the raw bit to match.
      flag_internalkl30safe = (b3 & 0x01) != 0;  // bit 24 - DBC comment doesn't state polarity, left as raw bit
      flag_hvilstate = (b3 & 0x40) == 0;         // bit 30 - HV interlock loop (0 = closed, 1 = open per DBC)
      flag_estoploopclosed = (b3 & 0x80) == 0;   // bit 31 - e-stop/safety loop (0 = closed, 1 = open per DBC)

      uint8_t b4 = rx_frame.data.u8[4];
      flag_warning = (b4 & 0x01) != 0;  // bit 32
      flag_alarm = (b4 & 0x02) != 0;    // bit 33
      flag_auxcont = (b4 & 0x04) != 0;  // bit 34
      flag_firerisk = (b4 & 0x80) != 0; // bit 39

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
    case AKASOL_BMM01_ERRORFLAG: {
      error_flags_raw = 0;
      for (int i = 0; i < 8; i++) {
        error_flags_raw |= ((uint64_t)rx_frame.data.u8[i]) << (8 * i);
      }
      break;
    }
    case AKASOL_BMM01_ALARMFLAG: {
      alarm_flags_raw = 0;
      for (int i = 0; i < 8; i++) {
        alarm_flags_raw |= ((uint64_t)rx_frame.data.u8[i]) << (8 * i);
      }
      break;
    }
    case AKASOL_BMM01_WARNINGFLAG: {
      warning_flags_raw = 0;
      for (int i = 0; i < 8; i++) {
        warning_flags_raw |= ((uint64_t)rx_frame.data.u8[i]) << (8 * i);
      }
      break;
    }
    case AKASOL_BMM01_ERROR_INFO: {
      const uint8_t* d = rx_frame.data.u8;
      errinfo_value = (int16_t)(d[0] | (d[1] << 8));                  // bits 0-15, signed
      uint32_t bits16_31 = d[2] | (d[3] << 8);                        // bits 16-31 packed
      errinfo_srccompnr = bits16_31 & 0x7FF;                          // bits 16-26 (11 bits)
      errinfo_srcsubcompclass = (uint8_t)((bits16_31 >> 11) & 0x3);   // bits 27-28 (2 bits)
      errinfo_srccompclass = (uint8_t)((bits16_31 >> 13) & 0x7);      // bits 29-31 (3 bits)
      errinfo_errornumber = d[4] | (d[5] << 8);                       // bits 32-47
      errinfo_detectdevice = d[6];                                    // bits 48-55
      break;
    }
    default:
      break;
  }
}

void AkasolBattery::update_values() {
  // Guard against publishing zero/uninitialized readings to the datalayer
  // before any real CAN frame has ever been received. battery_voltage_dV,
  // cell_voltage_min_mV/max_mV etc. all default to 0 (see header) and this
  // function is called on its own timer, independent of whether the BMU has
  // said anything yet - so without this guard, there is a real window at
  // boot where update_values() runs first and briefly publishes voltage=0V
  // / cell_min_voltage=0mV to the datalayer. The framework's own generic
  // safety layer polls the datalayer on its own schedule and reacted to
  // that transient zero by permanently latching
  // CELL_CRITICAL_UNDER_VOLTAGE/CELL_UNDER_VOLTAGE/BATTERY_UNDERVOLTAGE and
  // system_status=FAULT (seen on real hardware once contactors closed
  // cleanly) even though real, healthy cell voltages (~3.5V) arrived a
  // fraction of a second later and the AKASOL driver itself reported zero
  // faults. Skip publishing entirely until we've heard from the BMU at
  // least once.
  if (!CAN_battery_still_alive_frames_received) {
    return;
  }

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
  // lim_charge_curr_A_x10 / lim_discharge_curr_A_x10 are already in "amps x10"
  // i.e. exactly the same 0.1A resolution as max_charge_current_dA /
  // max_discharge_current_dA expect (deci-amps) - no rescaling needed.
  uint32_t charge_current_dA = lim_charge_curr_A_x10;
  uint32_t discharge_current_dA = lim_discharge_curr_A_x10;

  // SECOND BUG, found from Solax's own compatibility page
  // (kb.solaxpower.com/solution/detail/2c9fa4148ceddee5018e94039f8026a0):
  // the X1/X3 Hybrid G4 battery PORT has a hardware ceiling of "Max. charge /
  // Discharge current [A]: 30" - identical for both models, regardless of how
  // many kWh of battery is behind it. AKASOL's own BMM01_limits1 message
  // reports what the 50Ah AKASOL pack itself can do, which is on a totally
  // different scale (seen on the bus: ~137A charge / ~180A discharge - normal
  // for a pack this size, has nothing to do with what Solax's port hardware
  // can swallow). Forwarding that raw AKASOL number unclamped (which is
  // exactly what the *first* current_dA fix did) tells the inverter "charge/
  // discharge me at 137-180A", 4-6x past its own declared port limit - a
  // second, opposite-extreme way to feed it an invalid current value
  // (first bug: always 0A/too low; this one: uncapped/way too high). Clamp to
  // the inverter's real hardware limit before it ever reaches the datalayer.
  #define AKASOL_SOLAX_PORT_MAX_CURRENT_DA 300  // 30.0A, per SolaX's own spec
  if (charge_current_dA > AKASOL_SOLAX_PORT_MAX_CURRENT_DA) {
    charge_current_dA = AKASOL_SOLAX_PORT_MAX_CURRENT_DA;
  }
  if (discharge_current_dA > AKASOL_SOLAX_PORT_MAX_CURRENT_DA) {
    discharge_current_dA = AKASOL_SOLAX_PORT_MAX_CURRENT_DA;
  }

  // BUG FIX: these two datalayer fields were never being set anywhere in this
  // driver (only the *_power_W siblings were). They default to 0 and stay 0
  // forever. SOLAX-CAN.cpp's BMS_Limits frame (0x1872, bytes 4-7) sends
  // max_charge_current_dA/max_discharge_current_dA verbatim to the inverter,
  // and it does so starting in the very first BATTERY_ANNOUNCE handshake -
  // before contactors ever close. That means the SolaX was being told, from
  // the first frame onward, "this battery allows 0A charge and 0A discharge"
  // even while voltage/SOC/temperature all read out correctly - a battery
  // that reports a real pack voltage but zero allowed current is exactly the
  // kind of internally-inconsistent limits data that a battery-voltage/limits
  // sanity check performed during the inverter's init/checking phase (IE07
  // BatVoltFault) would plausibly reject, and it would explain why E07
  // persisted even after the earlier datalayer-guard fix (that fix only
  // stopped a transient 0V publish at boot - it never touched these two
  // fields, which were unconditionally 0 the entire time, guard or no guard).
  datalayer.battery.status.max_charge_current_dA = (uint16_t)charge_current_dA;
  datalayer.battery.status.max_discharge_current_dA = (uint16_t)discharge_current_dA;

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
    datalayer.battery.status.max_charge_current_dA = 0;
    datalayer.battery.status.max_discharge_current_dA = 0;
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
  // Sequenced KL30_safe -> KL30 -> (settle) -> Wake, matching the exact order
  // manually verified to work on real hardware. Previously KL30 and
  // KL30_safe were asserted in the same instant, which likely prevented the
  // BMU from ever seeing KL30_safe as bounce-free/stable before other
  // signals came up (manual: "must be bounce-free BEFORE req_batuse=1,
  // otherwise the battery gets stuck") - a plausible cause of a persistent
  // stat_Error / HVIL-not-seen condition even though Wake and CAN comms
  // came up fine.
  switch (akasol_state) {
    case AkasolState::INIT_HW:
      set_kl30(false);
      set_kl15_wake(false);
      set_kl30_safe(true);  // KL30_safe asserted alone first.
      state_entry_time = currentMillis;
      akasol_state = AkasolState::KL30_SAFE_ON;
      break;

    case AkasolState::KL30_SAFE_ON:
      if (currentMillis - state_entry_time >= AKASOL_T_KL30_SETTLE_MS) {
        set_kl30(true);  // KL30_safe stays asserted; KL30 comes up next.
        state_entry_time = currentMillis;
        akasol_state = AkasolState::KL30_ON;
      }
      break;

    case AkasolState::KL30_ON:
      if (currentMillis - state_entry_time >= AKASOL_T_KL30_TO_WAKE_MS) {
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
  // bit4 isolated_bmu - REVERTED back to 0 after testing.
  // Tested isolated_bmu=1 on real hardware (canlog_0d00h01m30s.txt): bit63/
  // SCUError and Error_info (detectDevice=163) were IDENTICAL to the
  // isolated_bmu=0 case - no change at all. So this bit is NOT the cause of
  // the persistent SCUError/detectDevice=163 signature; that theory is ruled
  // out empirically.
  // More importantly, manual chapter 6.2.5 revealed isolated_bmu=1 is not
  // just an informational flag - it is a HARDWARE variant declaration, and
  // setting it commits you to a mandatory procedure: an insulated-BMU system
  // must, during Init mode, temporarily bond LV ground to chassis ground via
  // a relay AND send req_iso_meas=1 within 2 seconds, then release both
  // before Standby. We are NOT doing that dance. The manual's own words:
  // "Noncompliance or work arounds pose the risk of severe injuries or
  // death in case of low insulation between HV and LV." Since it bought us
  // nothing and the described physical setup (KL31_GND only wired to the
  // control supply's own negative, nothing to the battery's chassis output
  // reported) looks like a standard, non-insulated BMU, 0 is the safer
  // default until/unless a real isolated-BMU hardware variant is confirmed
  // and the full init-mode grounding procedure is actually implemented.
#define AKASOL_ISOLATED_BMU 0  // reverted - see comment above
  byte0 |= (AKASOL_ISOLATED_BMU << 4);
  // bit5 req_iso_meas, bit7 iso_disable: leave at 0 unless you implement the
  // insulation-monitoring handshake described in manual chapter 6.2.5.

  AKASOL_VCU_frame.data.u8[0] = byte0;
  AKASOL_VCU_frame.data.u8[2] = vcu_alive_counter;  // bits 16-23
  AKASOL_VCU_frame.data.u8[6] = 0xAA;               // val_code low byte  -> 0x55AA
  AKASOL_VCU_frame.data.u8[7] = 0x55;               // val_code high byte

  // >>> TEMPORARY DIAGNOSTIC: rule out our own VCU1_to_BMM01 CAN message as
  // the trigger for the persistent SCUError/detectDevice=163 fault. With this
  // set to 1, we still receive and parse everything normally (status page
  // keeps working), and the KL30/KL30_safe/Wake discrete signals still get
  // driven exactly as before (that's hardware, not a CAN command) - we simply
  // stop putting our own frame on the bus at all. If SCUError/detectDevice
  // still shows up identically with zero bytes ever sent by us, that proves
  // the fault is not a reaction to anything we transmit (bad alive counter,
  // val_code, isolated_bmu bit, address, etc.) and is coming from inside the
  // battery independent of the vehicle CAN side entirely.
  // Diagnostic test complete (canlog_0d00h03m32s.txt: SCUError/detectDevice
  // identical with zero bytes ever transmitted by us - ruled out our own CAN
  // content as the trigger). Re-enabled now that AKASOL_BMU_ADDR is back to
  // 0xF3/Tray01 (see AKASOL-BATTERY.h) for a full retest with the bridge
  // refitted.
#define AKASOL_DISABLE_CAN_TX 0
#if !AKASOL_DISABLE_CAN_TX
  transmit_can_frame(&AKASOL_VCU_frame);
#endif

  vcu_alive_counter++;
}

// Names and plain-English meaning of all 64 bits shared by BMM01_ErrorFlag,
// BMM01_AlarmFlag and BMM01_WarningFlag (same bit layout in all three
// messages, just ef_/af_/wf_ prefixes in the DBC). Descriptions are my best
// reading of the AKASOL signal names - the exact internal logic behind each
// is not publicly documented, but the names themselves are official.
struct AkasolFaultBit {
  const char* name;
  const char* description;
};

// Descriptions below are AKASOL's own official comments, taken verbatim from
// the PublicCAN.sym file (PCAN Symbol Editor format) supplied by AKASOL -
// these are more precise than plain guesses from the signal names alone
// (e.g. bit22/23 are specifically about the HVIL loop, not a generic current
// sense loop; bit57 is explicitly tied to KL30/KL30Safe).
static const AkasolFaultBit AKASOL_FAULT_BITS[64] = {
    /*0*/ {"CellVoltageMax", "Maximum cell voltage above the specified limit"},
    /*1*/ {"CellVoltageMin", "Minimum cell voltage below the specified limit"},
    /*2*/ {"SysVoltageMax", "System voltage above the specified limit"},
    /*3*/ {"SysVoltageMin", "System voltage below the specified limit"},
    /*4*/ {"SysVoltageSum", "System voltage sum outside limit"},
    /*5*/ {"ModVoltageSum", "Module voltage sum outside limit"},
    /*6*/ {"ModVoltageRefMax", "Module reference voltage above specified limit"},
    /*7*/ {"ModVoltageRefMin", "Module reference voltage below specified limit"},
    /*8*/ {"IsoFaultBattNeg", "Isolation fault on negative battery terminal"},
    /*9*/ {"IsoFaultBattPos", "Isolation fault on positive battery terminal"},
    /*10*/ {"ContactorNegStuck", "Main contactor on negative battery terminal is stuck"},
    /*11*/ {"ContactorPosStuck", "Main contactor on positive battery terminal is stuck"},
    /*12*/ {"LVSupplyMax", "Supply voltage above the specified limit"},
    /*13*/ {"LVSupplyMin", "Supply voltage below the specified limit"},
    /*14*/ {"TerminalTempMax", "Maximum terminal temperature above the specified limit"},
    /*15*/ {"TerminalTempMin", "Minimum terminal temperature below the specified limit"},
    /*16*/ {"CellTempChargMax", "Maximum cell temperature when charging above the specified limit"},
    /*17*/ {"CellTempChargMin", "Minimum cell temperature when charging below the specified limit"},
    /*18*/ {"CellTempDischMax", "Maximum cell temperature when discharging above the specified limit"},
    /*19*/ {"CellTempDischMin", "Minimum cell temperature when discharging below the specified limit"},
    /*20*/ {"SysCurCha", "Charge current above the absolute current limit in charge direction"},
    /*21*/ {"SysCurDis", "Discharge current above the absolute current limit in discharge direction"},
    /*22*/ {"HVCurrLoopMax", "HVIL (HV interlock loop) short circuit"},
    /*23*/ {"HVCurrLoopMin", "HVIL (HV interlock loop) open circuit"},
    /*24*/ {"eStopCurrLoopMax", "eStop loop short circuit"},
    /*25*/ {"eStopCurrLoopMin", "eStop loop open circuit"},
    /*26*/ {"SysCurLimitCha", "Charge current above the actual current limit in charge direction"},
    /*27*/ {"SysCurLimitDis", "Discharge current above the actual current limit in discharge direction"},
    /*28*/ {"SysPrechargeFailed", "Precharge not successful"},
    /*29*/ {"BMMBattCom", "Failure in battery communication (battery internal or between battery and VCU)"},
    /*30*/ {"SysInitTimeout", "Failure in Init phase"},
    /*31*/ {"SCUConfig", "Faulty configuration of battery system"},
    /*32*/ {"DeepDischProt",
            "Deep-discharge-protection against self-discharge of battery at low SOC - SCU disconnected from CAN"},
    /*33*/ {"WDReset", "A watchdog reset has occurred"},
    /*34*/ {"ExtCommunicationTout", "Timeout for public CAN messages"},
    /*35*/ {"ContactorCoilCurrMax", "The contactor coil current is too high"},
    /*36*/ {"CellVoltUnbalance", "Cell voltages are unbalanced"},
    /*37*/ {"CellTempUnbalance", "Cell temperatures are unbalanced"},
    /*38*/ {"TerminalTempUnbalance", "Terminal temperatures are unbalanced"},
    /*39*/ {"FanError", "Fan is not working accordingly"},
    /*40*/ {"KL15Max", "KL15 voltage is too high"},
    /*41*/ {"KL30SafeCurrentMax", "Current through the safety channels is too high"},
    /*42*/ {"StuckAtTemp", "At least one cell temperature measured value isn't updated any more"},
    /*43*/ {"StuckAtVolt", "At least one cell voltage measured value isn't updated any more"},
    /*44*/ {"TaskGuardianError", "Unexpected behaviour was detected by the task guardian"},
    /*45*/ {"MemoryFault", "Error during memory operation"},
    /*46*/ {"Rack_U_Unbalance",
            "At least one battery rack/module voltage is unbalanced (parallel-connected system topology)"},
    /*47*/ {"Module_Disconnected",
            "At least one battery rack/module is electrically not connected (parallel-connected system topology)"},
    /*48*/ {"InvalidData", "Invalid value detected"},
    /*49*/ {"ContactorWrongState", "Contactor state doesn't match request"},
    /*50*/ {"CurrSens", "Current sensor error occurred"},
    /*51*/ {"ContDam", "Wear-out of contactors too high - no further operation allowed"},
    /*52*/ {"RefVoltErrorMax", "SCU reference voltage above specified limit"},
    /*53*/ {"RefVoltErrorMin", "SCU reference voltage below specified limit"},
    /*54*/ {"Kl30SafeMax", "Supply voltage of KL30safe above the specified limit"},
    /*55*/ {"Kl30SafeMin", "Supply voltage of KL30safe below the specified limit"},
    /*56*/ {"SCUSupply", "SCU CPU supply under low limit"},
    /*57*/ {"SCUPowerProtection", "Fault from KL30/KL30Safe power protection detected"},
    /*58*/ {"Kl30SafeOff", "BMS-M-SafePower was switched off (HW detection)"},
    /*59*/ {"Dew_Sensor", "Tray dew value out of range"},
    /*60*/ {"ContactorDropOut", "A contactor drop-out occurred"},
    /*61*/ {"ECUBoardTemp", "Board temperature exceeds a limit"},
    /*62*/ {"Valve", "Valve error occurred"},
    /*63*/ {"SCUError", "Indicates that the error was detected by the SCU"},
};

String AkasolBattery::get_status_html() {
  // Surfaces internal state that's already parsed from BMM01_State but was
  // previously invisible in the web UI - useful while confirming the wake
  // sequence / contactor closing behaves as expected on real hardware.
  String content;
  content.reserve(9000);

  content +=
      "<h4 style='margin-top:20px;color:#27b06c;border-bottom:2px solid #27b06c;padding-bottom:5px;'>"
      "AKASOL internal state (from BMM01_State)</h4>";

  auto flag_row = [&content](const char* label, bool value) {
    content += "<p style='margin:4px 0;'>";
    content += label;
    content += ": ";
    content += value ? "<span style='color:#69f0ae;'>yes</span>" : "<span style='color:#bbb;'>no</span>";
    content += "</p>";
  };

  flag_row("Init", stat_init);
  flag_row("Standby", stat_standby);
  flag_row("Precharge", stat_precharge);
  flag_row("Operational (contactors closed)", stat_operational);
  flag_row("Disabling", stat_disabling);
  flag_row("Error flag", stat_error);

  content +=
      "<h4 style='margin-top:16px;color:#27b06c;border-bottom:2px solid #27b06c;padding-bottom:5px;'>"
      "Contactors &amp; safety loop</h4>";
  flag_row("Contactor + closed", contactor_pos);
  flag_row("Contactor - closed", contactor_neg);
  flag_row("Precharge contactor closed", flag_contactor_precha);
  flag_row("HV interlock loop (HVIL) closed", flag_hvilstate);
  flag_row("E-stop / safety loop closed", flag_estoploopclosed);
  flag_row("Internal KL30_safe seen by BMU", flag_internalkl30safe);
  flag_row("External shutdown requested", flag_extshutdownreq);

  content +=
      "<h4 style='margin-top:16px;color:#27b06c;border-bottom:2px solid #27b06c;padding-bottom:5px;'>"
      "Warning / Alarm (separate from generic Error above)</h4>";
  flag_row("Warning", flag_warning);
  flag_row("Alarm", flag_alarm);
  flag_row("Fire risk", flag_firerisk);

  content +=
      "<h4 style='margin-top:16px;color:#27b06c;border-bottom:2px solid #27b06c;padding-bottom:5px;'>"
      "Other flags</h4>";
  flag_row("Drive enabled", flag_drive);
  flag_row("Charge enabled", flag_charge);
  flag_row("Wake", flag_wake);
  flag_row("Isolation monitoring disabled", flag_isodisable);
  flag_row("Charge complete", flag_chargecomplete);
  flag_row("Aux contactor", flag_auxcont);

  content += "<p style='margin:12px 0 4px;color:#ccc;'>BMM01 mirrored alive counter: " +
             String(bmm_alive_counter_mirror) + "</p>";
  content += "<p style='margin:4px 0;color:#ccc;'>Our VCU alive counter (last sent): " +
             String((int)(vcu_alive_counter - 1) & 0xFF) + "</p>";

  // --- Full 64-bit named fault breakdown (BMM01_ErrorFlag/AlarmFlag/WarningFlag) ---
  // This is the ONLY place that identifies which specific condition is
  // behind the generic "Error flag"/"Warning"/"Alarm" shown above. Every one
  // of the 64 defined bits is listed, with active ones highlighted, so
  // nothing gets missed even without AKASOL support available to ask.
  content +=
      "<h4 style='margin-top:20px;color:#27b06c;border-bottom:2px solid #27b06c;padding-bottom:5px;'>"
      "Detailed fault bits (BMM01_ErrorFlag / AlarmFlag / WarningFlag)</h4>";
  content +=
      "<table style='width:100%;border-collapse:collapse;font-size:0.85em;'>"
      "<tr style='color:#9be7c4;text-align:left;'>"
      "<th style='padding:3px 6px;'>#</th>"
      "<th style='padding:3px 6px;'>Signal</th>"
      "<th style='padding:3px 6px;'>Meaning</th>"
      "<th style='padding:3px 6px;text-align:center;'>Err</th>"
      "<th style='padding:3px 6px;text-align:center;'>Alm</th>"
      "<th style='padding:3px 6px;text-align:center;'>Wrn</th>"
      "</tr>";
  for (int i = 0; i < 64; i++) {
    bool e = ((error_flags_raw >> i) & 1ULL) != 0;
    bool a = ((alarm_flags_raw >> i) & 1ULL) != 0;
    bool w = ((warning_flags_raw >> i) & 1ULL) != 0;
    bool active = e || a || w;
    content += "<tr style='";
    if (active) {
      content += "background:#3a2323;";
    }
    content += "border-bottom:1px solid #333;'>";
    content += "<td style='padding:3px 6px;color:#888;'>" + String(i) + "</td>";
    content += "<td style='padding:3px 6px;'><b>" + String(AKASOL_FAULT_BITS[i].name) + "</b></td>";
    content += "<td style='padding:3px 6px;color:#ccc;'>" + String(AKASOL_FAULT_BITS[i].description) + "</td>";
    content += "<td style='padding:3px 6px;text-align:center;'>" +
               String(e ? "<span style='color:#ff6b6b;font-weight:bold;'>ERROR</span>" : "-") + "</td>";
    content += "<td style='padding:3px 6px;text-align:center;'>" +
               String(a ? "<span style='color:#ffd166;font-weight:bold;'>ALARM</span>" : "-") + "</td>";
    content += "<td style='padding:3px 6px;text-align:center;'>" +
               String(w ? "<span style='color:#9be7c4;font-weight:bold;'>WARN</span>" : "-") + "</td>";
    content += "</tr>";
  }
  content += "</table>";

  content +=
      "<p style='margin:10px 0 2px;color:#ccc;font-size:0.9em;'>BMM01_Error_info (internal AKASOL codes, "
      "not publicly documented - quote these verbatim if contacting AKASOL support): value=" +
      String(errinfo_value) + ", errornumber=" + String(errinfo_errornumber) +
      ", srcCompClass=" + String(errinfo_srccompclass) + ", srcSubcompClass=" + String(errinfo_srcsubcompclass) +
      ", srcCompNr=" + String(errinfo_srccompnr) + ", detectDevice=" + String(errinfo_detectdevice) + "</p>";

  return content;
}
