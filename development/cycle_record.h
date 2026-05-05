#pragma once
#include <stdint.h>

// One record per completed charge cycle; appended to /cycles.csv on FFat.
// Defined in a header (not in the .ino body) so that Arduino IDE's automatic
// forward-declaration pass sees the type before it emits the prototype for
// appendCycleRecord() — same reason HttpCtx lives in https_ctx.h.
//
// Fields marked _x100 / _x10 store fixed-point values to avoid floats in
// the struct (reduces size; Python training reads them as plain CSV floats).
struct CycleRecord {
  char     timestamp[20];   // "2026-05-04T14:23:11" (NTP) or "" if no NTP yet
  uint8_t  preset_pct;      // 70 / 80 / 90 / 100
  uint16_t start_v_dv;      // pack voltage dV at CC entry
  uint16_t end_v_dv;        // pack voltage dV at DONE entry
  uint8_t  start_soc;       // BMS SOC at CC entry  (255 = not available)
  uint8_t  end_soc;         // BMS SOC at DONE entry (255 = not available)
  int8_t   start_temp;      // monolithMaxTemp (°C) at CC entry
  int8_t   end_temp;        // monolithMaxTemp (°C) at DONE entry
  uint16_t total_ah_x100;   // session.chargeAh × 100  (2 dp fixed-point)
  uint32_t total_wh_x10;    // session.energyWh × 10   (1 dp fixed-point)
  uint16_t bulk_min;        // minutes in PHASE_CC
  uint16_t absorption_min;  // minutes in PHASE_CV
  uint8_t  charger_count;   // ctrl.chargerCount at CC entry
  uint8_t  abort_reason;    // 0 = current_taper   1 = cv_timeout
};
