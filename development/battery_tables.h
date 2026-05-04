#include <stdint.h>

struct cutback_entry {
  uint32_t threshold;
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
  if (mode == CUTBACK_AT_OR_ABOVE) {
    for (ssize_t i = entry_count - 1; i >= 0; i--) {
      if (entries[i].threshold <= measurement) {
        return entries[i].c_rate_thousandths;
      }
    }
  } else {
    for (ssize_t i = 0; i < entry_count; i++) {
      if (entries[i].threshold >= measurement) {
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

// Battery-cold cutback table. Threshold is degrees C, as measured
// at the coldest thermocouple of each battery pack.
const cutback_entry COLD_CUTBACK[] = {
  { .threshold =  0, .c_rate_thousandths =  200 },
  { .threshold =  1, .c_rate_thousandths =  300 },
  { .threshold =  5, .c_rate_thousandths =  400 },
  { .threshold =  6, .c_rate_thousandths =  500 },
  { .threshold =  8, .c_rate_thousandths =  600 },
  { .threshold =  9, .c_rate_thousandths =  700 },
  { .threshold = 10, .c_rate_thousandths =  800 },
  { .threshold = 12, .c_rate_thousandths =  900 },
  { .threshold = 14, .c_rate_thousandths = 1000 },
  { .threshold = 16, .c_rate_thousandths = 1100 },
  { .threshold = 18, .c_rate_thousandths = 1200 },
  { .threshold = 20, .c_rate_thousandths = 1300 },
  { .threshold = 21, .c_rate_thousandths = 1400 },
  { .threshold = 22, .c_rate_thousandths = 1500 },
  { .threshold = 23, .c_rate_thousandths = 1600 },
  { .threshold = 24, .c_rate_thousandths = 1700 },
  { .threshold = 25, .c_rate_thousandths = 1800 },
  { .threshold = 26, .c_rate_thousandths = 1900 },
  { .threshold = 27, .c_rate_thousandths = 2000 },
  { .threshold = 28, .c_rate_thousandths = 2200 },
  { .threshold = 29, .c_rate_thousandths = 2300 },
  { .threshold = 30, .c_rate_thousandths = 2400 },
  { .threshold = 31, .c_rate_thousandths = 2500 },
  { .threshold = 33, .c_rate_thousandths = 2600 },
  { .threshold = 34, .c_rate_thousandths = 2700 },
  { .threshold = 35, .c_rate_thousandths = 2800 },
  { .threshold = 37, .c_rate_thousandths = 2900 },
  { .threshold = 40, .c_rate_thousandths = 3000 }
};

// Battery-hot cutback table. Threshold is degrees C, as measured
// at the hottest thermocouple of each battery pack.
const cutback_entry HOT_CUTBACK[] = {
  { .threshold = 50, .c_rate_thousandths = 2500 },
  { .threshold = 60, .c_rate_thousandths = 1000 },
  { .threshold = 70, .c_rate_thousandths =  300 },
  { .threshold = 75, .c_rate_thousandths =    0 }
};
