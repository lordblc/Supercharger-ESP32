#include <stdint.h>

struct cutback_entry {
  // SIGNED: temperature thresholds can legitimately be below zero, and the
  // COLD_CUTBACK table now carries a sub-zero "do not charge" entry. This was
  // uint32_t, which forced find_cutback() to cast to (int) on every comparison
  // and made a negative threshold a narrowing-conversion error at the
  // initialiser. Voltage thresholds (900-1164 dV) fit an int32_t trivially.
  int32_t  threshold;
  uint32_t c_rate_thousandths;
};

enum CutbackLimitType {
  CUTBACK_AT_OR_ABOVE,
  CUTBACK_AT_OR_BELOW
};

// Find the most restrictive applicable cutback threshold from [entries],
// or UINT32_MAX if no cutback is necessary.
uint32_t find_cutback(
  int measurement,
  CutbackLimitType mode,
  const cutback_entry *entries,
  size_t entry_count
) {
  // `threshold` and `measurement` are both signed, so these comparisons are
  // plain signed arithmetic. (Historically threshold was uint32_t and each
  // comparison needed an explicit (int) cast — omitting it would have promoted
  // a negative measurement to a huge unsigned value, matching the HIGHEST
  // HOT_CUTBACK entry and clamping a merely-cold pack to zero power.)
  if (mode == CUTBACK_AT_OR_ABOVE) {
    for (ssize_t i = (ssize_t)entry_count - 1; i >= 0; i--) {
      if (entries[i].threshold <= (int32_t)measurement) {
        return entries[i].c_rate_thousandths;
      }
    }
  } else {
    for (ssize_t i = 0; i < (ssize_t)entry_count; i++) {
      if (entries[i].threshold >= (int32_t)measurement) {
        return entries[i].c_rate_thousandths;
      }
    }
  }
  return UINT32_MAX;
}

// Convenience template to automatically supply length to find_cutback.
template <int N>
inline uint32_t find_cutback(
  int measurement,
  CutbackLimitType mode,
  const cutback_entry (&entries)[N]
) {
  return find_cutback(measurement, mode, entries, N);
}

// Voltage cutback table. Threshold is decivolts of lifted pack voltage.
/*
const cutback_entry VOLTAGE_CUTBACK[] = {
  { .threshold =  900, .c_rate_thousandths = 3000 },
  { .threshold =  980, .c_rate_thousandths = 2800 },
  { .threshold = 1000, .c_rate_thousandths = 2500 },
  { .threshold = 1010, .c_rate_thousandths = 2300 },
  { .threshold = 1020, .c_rate_thousandths = 2000 },
  { .threshold = 1060, .c_rate_thousandths = 1800 },
  { .threshold = 1100, .c_rate_thousandths = 1300 },
  { .threshold = 1140, .c_rate_thousandths = 1000 },
  { .threshold = 1150, .c_rate_thousandths =  700 },
  { .threshold = 1160, .c_rate_thousandths =  600 }
};
*/

// Much more aggressive curve to avoid voltage overshoot
const cutback_entry VOLTAGE_CUTBACK[] = {
  { .threshold =  900, .c_rate_thousandths = 3000 },
  { .threshold =  980, .c_rate_thousandths = 2800 },
  { .threshold = 1000, .c_rate_thousandths = 2500 },
  { .threshold = 1010, .c_rate_thousandths = 2300 },
  { .threshold = 1020, .c_rate_thousandths = 2000 },
  { .threshold = 1060, .c_rate_thousandths = 1800 },
  { .threshold = 1100, .c_rate_thousandths = 1000 },
  { .threshold = 1110, .c_rate_thousandths = 700 },
  { .threshold = 1120, .c_rate_thousandths = 500 },
  { .threshold = 1130, .c_rate_thousandths = 400 },
  { .threshold = 1140, .c_rate_thousandths = 300 },
  { .threshold = 1145, .c_rate_thousandths = 200 },
  { .threshold = 1150, .c_rate_thousandths = 100 },
  { .threshold = 1153, .c_rate_thousandths = 50 },
  { .threshold = 1156, .c_rate_thousandths = 30 },
  { .threshold = 1160, .c_rate_thousandths = 20 },
  { .threshold = 1162, .c_rate_thousandths = 10 },
  { .threshold = 1164, .c_rate_thousandths =  5 }  // 116.4 V design max — 0.005 C ceiling (~570 mA on 114 Ah)
};

// ---------------------------------------------------------------------------
// Cold / hot cutback tables — reshaped to the Farasis IMP06160230P25A envelope.
//
// Datasheet limits (Farasis Energy V5, Aug 2011):
//     Max Charge Current   25 A on a 25 Ah cell  = 1.0 C
//     Charging Temp.       0 °C to 45 °C
//
// The previous tables predate the cell being identified and topped out at
// 3.0 C — three times the cell's rated charge current — with HOT_CUTBACK
// applying no limit whatsoever below 50 °C, i.e. no limit across the entire
// 45-50 °C band that is already outside the datasheet's charging range. In
// practice the 13.2 kW system ceiling (~1 C at this pack size) meant the 3.0 C
// entries never bound, so this is not a "we were charging at 3 C" fix — but the
// tables read as safety limits and were not functioning as any.
//
// Division of responsibility now:
//   * The hard inhibit in rampTask owns the 0 °C / 45 °C BOUNDARY.
//   * These tables shape the C-rate INSIDE that window, capped at 1.0 C.
//   * The out-of-range entries below are kept as an independent second layer,
//     so a future refactor that loses the gate still cannot charge hot or cold.
// ---------------------------------------------------------------------------

// Battery-cold cutback table. Threshold is degrees C, as measured
// at the coldest thermocouple of each battery pack.
// CUTBACK_AT_OR_BELOW: returns the first entry whose threshold >= measurement,
// so the table must stay ascending and colder measurements pick lower C-rates.
const cutback_entry COLD_CUTBACK[] = {
  // Sub-zero entries are unreachable in normal operation — the rampTask gate
  // inhibits charging below CHARGE_TEMP_MIN_C. Retained as a backstop: if the
  // gate is ever bypassed, a freezing pack still gets 0 C-rate rather than the
  // 0.2 C the old table handed out at -20 °C.
  { .threshold = -1, .c_rate_thousandths =    0 },  // below 0 °C: do not charge
  { .threshold =  0, .c_rate_thousandths =  200 },  // 0 °C — lowest permitted
  { .threshold =  1, .c_rate_thousandths =  300 },
  { .threshold =  5, .c_rate_thousandths =  400 },
  { .threshold =  6, .c_rate_thousandths =  500 },
  { .threshold =  8, .c_rate_thousandths =  600 },
  { .threshold =  9, .c_rate_thousandths =  700 },
  { .threshold = 10, .c_rate_thousandths =  800 },
  { .threshold = 12, .c_rate_thousandths =  900 },
  // 13-45 °C: full 1.0 C, the cell's rated maximum charge current. Previously
  // this band ramped on up to 3.0 C.
  { .threshold = 45, .c_rate_thousandths = 1000 }
};

// Battery-hot cutback table. Threshold is degrees C, as measured
// at the hottest thermocouple of each battery pack.
// CUTBACK_AT_OR_ABOVE: scanned from the END backwards, returns the first entry
// whose threshold <= measurement, so the table must stay ascending and hotter
// measurements pick lower C-rates. Returns UINT32_MAX (no limit) below the
// lowest threshold — that is fine here because COLD_CUTBACK supplies the 1.0 C
// cap across the whole permitted range.
const cutback_entry HOT_CUTBACK[] = {
  { .threshold = 35, .c_rate_thousandths = 1000 },  // ≥35 °C: still full 1.0 C
  { .threshold = 40, .c_rate_thousandths =  700 },  // begin tapering before the
  { .threshold = 43, .c_rate_thousandths =  500 },  //   45 °C datasheet limit
  // 46 °C and above: past the datasheet charging maximum. The rampTask gate
  // should already have inhibited charging; these are the backstop layer.
  { .threshold = 46, .c_rate_thousandths =    0 },
  { .threshold = 60, .c_rate_thousandths =    0 },
  { .threshold = 75, .c_rate_thousandths =    0 }
};
