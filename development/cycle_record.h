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
  int8_t   start_temp;      // monolithMaxTemp (°C) at CC entry; -128 = no valid reading
  int8_t   end_temp;        // monolithMaxTemp (°C) at DONE entry; -128 = no valid reading
  // Per-cycle totals (this cycle only), NOT the session-to-date counters:
  // rampTask totals them tick by tick from CC entry to DONE (cycleAh/cycleWh),
  // never derived from the session accumulators, so Reset Session cannot
  // affect them. uint32_t, not uint16_t — at x100 a uint16_t wrapped silently
  // past 655.35 Ah.
  uint32_t total_ah_x100;   // Ah delivered this cycle × 100  (2 dp fixed-point)
  uint32_t total_wh_x10;    // Wh delivered this cycle × 10   (1 dp fixed-point)
  uint16_t bulk_min;        // minutes in PHASE_CC
  uint16_t absorption_min;  // minutes in PHASE_CV
  uint8_t  charger_count;   // ctrl.chargerCount at CC entry
  // How absorption ended. 0 is the only "ran to completion" value; 1 and 2 both
  // mean absorption was cut short, for different reasons, and Stage 2 training
  // should weight them differently from a natural finish.
  uint8_t  abort_reason;    // 0 = current_taper   1 = cv_timeout
                            // 2 = above_target (pack went above the ceiling, or
                            //     the ceiling was lowered below the pack)
};
