#include "ZERO.h"

Zero::Zero(){

}

Zero::~Zero(){

}

byte Zero::messageLength=ZERO_MESSAGE_LENGTH;

byte Zero::hasVoltage(short &canId){
  if(canId == CALEX_CHARGE_CONTROL.id
    || canId == BMS_CELL_VOLTAGE.id
    || canId == BMS1_CELL_VOLTAGE.id ) {

      return 1;

  }
    return 0;
}

byte Zero::hasMonolithVoltage(short &canId){
  if(canId == BMS_CELL_VOLTAGE.id) {

      return 1;

  }
    return 0;
}

byte Zero::hasPowerTankVoltage(short &canId){
  if(canId == BMS1_CELL_VOLTAGE.id ) {

      return 1;

  }
    return 0;
}

byte Zero::hasAmps(short &canId){
  if(canId == BMS_PACK_ACTIVE_DATA.id
    || canId == BMS1_PACK_ACTIVE_DATA.id ) {

      return 1;

  }
    return 0;
}

byte Zero::hasMonolithAmps(short &canId){
  if(canId == BMS_PACK_ACTIVE_DATA.id) {

      return 1;

  }
    return 0;
}

byte Zero::hasPowerTankAmps(short &canId){
  if(canId == BMS1_PACK_ACTIVE_DATA.id) {

      return 1;

  }
    return 0;
}

byte Zero::hasMaxCRate(short &canId){
  if(canId == BMS_PACK_TIME.id
    || canId == BMS1_PACK_TIME.id ) {

      return 1;

  }
    return 0;
}

byte Zero::hasMonolithMaxCRate(short &canId){
  if(canId == BMS_PACK_TIME.id) {

      return 1;

  }
    return 0;
}

byte Zero::hasPowerTankMaxCRate(short &canId){
  if(canId == BMS1_PACK_TIME.id) {

      return 1;

  }
    return 0;
}

byte Zero::hasPackTime(short &canId){
  if(canId == BMS_PACK_TIME.id
    || canId == BMS1_PACK_TIME.id ) {

      return 1;

  }
    return 0;
}

byte Zero::hasMonolithPackTime(short &canId){
  if(canId == BMS_PACK_TIME.id) {

      return 1;

  }
    return 0;
}

byte Zero::hasPowerTankPackTime(short &canId){
  if(canId == BMS1_PACK_TIME.id) {

      return 1;

  }
    return 0;
}

byte Zero::hasPackConfig(short &canId){
  if(canId == BMS_PACK_CONFIG.id
    || canId == BMS1_PACK_CONFIG.id ) {

      return 1;

  }
    return 0;
}

byte Zero::hasMonolithPackConfig(short &canId){
  if(canId == BMS_PACK_CONFIG.id) {

      return 1;

  }
    return 0;
}

byte Zero::hasPowerTankPackConfig(short &canId){
  if(canId == BMS1_PACK_CONFIG.id) {

      return 1;

  }
    return 0;
}

byte Zero::hasPackActiveData(short &canId){
  if(canId == BMS_PACK_ACTIVE_DATA.id
    || canId == BMS1_PACK_ACTIVE_DATA.id ) {

      return 1;

  }
    return 0;
}

byte Zero::hasMonolithPackActiveData(short &canId){
  if(canId == BMS_PACK_ACTIVE_DATA.id) {

      return 1;

  }
    return 0;
}

byte Zero::hasPowerTankPackActiveData(short &canId){
  if(canId == BMS1_PACK_ACTIVE_DATA.id) {

      return 1;

  }
    return 0;
}

byte Zero::hasThrottle(short &canId){
  if(canId == CONTROLLER_RPM_THROTTLE_MOT_TEMP.id) {

      return 1;

  }
    return 0;
}

// BMS_PACK_STATUS (0x188 / 0x189) — SoC % at byte 0 (uint8 0..100).
// See ZERO.h for the full byte layout.  The matching frame from the
// secondary "PowerTank" pack uses ID 0x189.
byte Zero::hasPackStatus(short &canId){
  if(canId == BMS_PACK_STATUS.id
    || canId == BMS1_PACK_STATUS.id ) {

      return 1;

  }
    return 0;
}

byte Zero::hasMonolithPackStatus(short &canId){
  if(canId == BMS_PACK_STATUS.id) {

      return 1;

  }
    return 0;
}

byte Zero::hasPowerTankPackStatus(short &canId){
  if(canId == BMS1_PACK_STATUS.id) {

      return 1;

  }
    return 0;
}

short Zero::throttle(byte &len,byte buf[ZERO_MESSAGE_LENGTH]){
  ByteToShort throttleVoltage;

  throttleVoltage.bytes[0]=buf[4];
  throttleVoltage.bytes[1]=buf[5];

  //Serial.println(throttleVoltage.value);

  return throttleVoltage.value;
}

short Zero::amps(byte &len,byte buf[ZERO_MESSAGE_LENGTH]){
  ByteToShort amps;

  amps.bytes[0]=buf[3];
  amps.bytes[1]=buf[4];

  //Serial.println(amps.value);

  return amps.value;
}

long Zero::voltage(byte &len,byte buf[ZERO_MESSAGE_LENGTH]){
  ByteToLong voltage;

  voltage.bytes[0]=buf[3];
  voltage.bytes[1]=buf[4];
  voltage.bytes[2]=buf[5];
  voltage.bytes[3]=buf[6];

  //Serial.println(voltage.value);

  return voltage.value;
}

short Zero::sagAdjust(byte &len,byte buf[ZERO_MESSAGE_LENGTH]){
  ByteToShort sagAdjust;

  sagAdjust.bytes[0]=buf[0];
  sagAdjust.bytes[1]=buf[1];

  //Serial.println(sagAdjust.value);

  return sagAdjust.value;
}

short Zero::maxCRate(byte &len,byte buf[ZERO_MESSAGE_LENGTH]){
  ByteToShort CRate;

  // Bytes 0-3 of BMS_PACK_TIME (0x508) are a 32-bit runtime counter.
  // Max charge current reported by BMS is at bytes 6-7 (not 4-5).
  CRate.bytes[0]=buf[6];
  CRate.bytes[1]=buf[7];

  return CRate.value;
}

short Zero::AH(byte &len,byte buf[ZERO_MESSAGE_LENGTH]){
  ByteToShort AH;

  AH.bytes[0]=buf[5];
  AH.bytes[1]=buf[6];

  //Serial.println(AH.value);

  return AH.value;
}

// Cell-temperature decoders for BMS_PACK_ACTIVE_DATA (0x408) and the BMS1
// equivalent (0x409). Layout, confirmed against the SCv2 reference sketches
// (v2.5 / v2.6 / v3 / 13kw_force) and the original Arduino library:
//
//   byte 1: highest cell temp (signed int8, °C)
//   byte 2: lowest  cell temp (signed int8, °C)
//   byte 3-4: pack amps (signed int16, centiamps) — handled by amps()
//
// The previous "scan all 8 bytes and filter by [-20, 85]" approach was a
// workaround for decoding the wrong CAN ID — 0x488 BMS_PACK_TEMP_DATA, which
// on real bikes carries other BMS internals (cell-balance flags, individual
// sensor positions with 0x7F sentinels, etc.). That filter let random
// non-temperature bytes through (e.g. byte 0xF5 reads as -11°C as int8) and
// produced bogus min readings down around -11..0 °C. Temps now come straight
// off the ACTIVE_DATA frame, same place the working SCv2 sketches use.
//
// 0x7F (127) is the BMS "sensor disconnected" sentinel — guarded against so
// a single dead thermistor doesn't push the max to +127°C.

short Zero::highestTemp(byte &len, byte buf[ZERO_MESSAGE_LENGTH]) {
  if (len < 2) return 0;
  if ((uint8_t)buf[1] == 0x7F) return 0;   // disconnected sensor
  return (short)(int8_t)buf[1];
}

short Zero::lowestTemp(byte &len, byte buf[ZERO_MESSAGE_LENGTH]) {
  if (len < 3) return 0;
  if ((uint8_t)buf[2] == 0x7F) return 0;   // disconnected sensor
  return (short)(int8_t)buf[2];
}

// Pull the BMS-reported State of Charge out of a BMS_PACK_STATUS (0x188 / 0x189)
// frame.  Byte 0 is the SoC as a uint8 percent (0..100).  Returns 255 as a
// "bad / unknown frame" sentinel so callers can distinguish "no data yet"
// from a real 0 % reading on a fully-discharged pack.
byte Zero::stateOfCharge(byte &len, byte buf[ZERO_MESSAGE_LENGTH]) {
  if (len < 1) return 255;
  uint8_t soc = (uint8_t)buf[0];
  if (soc > 100) return 255;
  return (byte)soc;
}

void Zero::logInit(){
  // Do nothing on ESP32
//  if(Serial){
//    Serial.begin(SERIAL_BAUD);
//  }
}

void Zero::logRaw(byte &len,byte buf[ZERO_MESSAGE_LENGTH],short &canId){
  if(!Serial){
    return;
  }
  byte i;
  byte knownMessage=0;
  byte show=0;

  for(i=0; i<TOTAL_MESSAGES; i++){
    if(canId==Zero_Messages[i].id){
      knownMessage=1;
      show=1;
      Serial.println("-----------------------------");
      Serial.println(Zero_Messages[i].name);
      break;
    }
  }


  Serial.print(" ID: ");
  Serial.println(canId,HEX);
  Serial.println("-----------------------------");
  for(int i = 0; i<len; i++){
      Serial.print(buf[i], HEX);
      Serial.print("\t");
  }

  Serial.println();

}
