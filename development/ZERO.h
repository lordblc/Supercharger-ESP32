#ifndef ZERO_H
#define ZERO_H

#include <Arduino.h>
#include "ZeroNetwork.h"
#include "unions.h"

// Sentinel returned by highestTemp() / lowestTemp() when the frame carries no
// usable reading — either the BMS "sensor disconnected" marker (0x7F) or a
// frame too short to contain the byte. It must sit far outside any physical
// temperature so callers can test for "no data" rather than silently treating
// a dead thermistor as 0 °C: 0 °C reads as a perfectly normal pack and removes
// both the hot cutback and the 45 °C charge inhibit. Callers in the firmware
// use the TEMP_INVALID_THRESHOLD test below, which this value fails by design.
#define ZERO_TEMP_INVALID (-32768)

// Single validity threshold for every pack-temperature consumer. A reading is
// usable only if it is STRICTLY GREATER than this value. -1000 is far below any
// physically possible pack temperature and far above ZERO_TEMP_INVALID, so it
// catches the sentinel (and any other absurd decode) without ever rejecting a
// real thermistor. Previously the same test was open-coded as `> -100` in
// rampTask and `<= -1000` in fmtTempJson / the MQTT publish macro — three
// literals that could drift apart. Use this macro everywhere instead.
#define TEMP_INVALID_THRESHOLD (-1000)


class Zero {
  public:
    Zero();
    ~Zero();

    static byte messageLength;

    byte hasVoltage(short &canId);
    byte hasMonolithVoltage(short &canId);
    byte hasPowerTankVoltage(short &canId);

    byte hasAmps(short &canId);
    byte hasMonolithAmps(short &canId);
    byte hasPowerTankAmps(short &canId);

    byte hasMaxCRate(short &canId);
    byte hasMonolithMaxCRate(short &canId);
    byte hasPowerTankMaxCRate(short &canId);

    byte hasPackTime(short &canId);
    byte hasMonolithPackTime(short &canId);
    byte hasPowerTankPackTime(short &canId);

    byte hasPackConfig(short &canId);
    byte hasMonolithPackConfig(short &canId);
    byte hasPowerTankPackConfig(short &canId);

    byte hasPackActiveData(short &canId);
    byte hasMonolithPackActiveData(short &canId);
    byte hasPowerTankPackActiveData(short &canId);

    // BMS_PACK_STATUS (0x188 / 0x189) carries the BMS's own SoC % at byte 0.
    // Decoded byte layout (verified against the EMF-thread reverse engineering):
    //   byte 0: SoC %, uint8 (0..100)
    //   byte 1: always 0
    //   byte 2: 66 = riding / 3 = charging
    //   byte 3-4: charge cycles, uint16 LE
    //   byte 5: cell-balance flags
    //   byte 6: always 0
    //   byte 7: brick count (?)
    byte hasPackStatus(short &canId);
    byte hasMonolithPackStatus(short &canId);
    byte hasPowerTankPackStatus(short &canId);

    byte hasThrottle(short &canId);

    long voltage(byte &len,byte buf[]);
    short sagAdjust(byte &len,byte buf[]);

    short amps(byte &len,byte buf[]);

    short throttle(byte &len,byte buf[]);

    short maxCRate(byte &len,byte buf[]);

    short AH(byte &len,byte buf[]);

    short highestTemp(byte &len,byte buf[]);
    short lowestTemp(byte &len,byte buf[]);

    // BMS-reported state of charge as a 0..100 percentage (255 = bad/unknown frame)
    byte stateOfCharge(byte &len,byte buf[]);

    void logInit();
    void logRaw(byte &len,byte buf[],short &canId);

};

#endif
