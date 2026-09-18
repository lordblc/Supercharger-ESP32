// ==========================================================================
// Supercharger controller for Zero bike based on LilyGo T-2CAN (ESP32-S3)
//
// Thanks to RIEvangelist, Skonk and BrianTRice. 
// Without the work these people have done, i would not be able to put
// together this controller. Their openly available work online is the 
// basis for a lot of what this controller does. I just put it together
// and added some other features.
// (There are a lot more people involved here and I thank you all. If you
// want your name in this list, do not hesitate to get in contact with me)
// 
// Hardware info:
//  CAN:
//   Charger: MCP2515 via SPI  (CS=10, SCLK=12, MOSI=11, MISO=13, RST=9)
//   Bike: ESP32-S3 TWAI    (TX=7, RX=6)
//
// Written for 1-4 x Elcon TC HK-J 3300W chargers (the units supplied in the
// DigiNow supercharger kit). They all ship with the same CAN instance ID
// 0x18FF50E5, so one broadcast command drives every unit and charger
// auto-detect reports 1 no matter how many are wired.
//
// Other functions:
//   Web dashboard (live status, polled via /api/status JSON)
//   WiFi - connects to saved network, falls back to its own AP for setup
//   OTA firmware update via browser (/update) - the image must carry a valid
//     HMAC-SHA256 trailer (sign_ota.py), and the page is behind the same
//     form-login session cookie as the rest of the UI. NOT HTTP Basic Auth.
//   MQTT communication (for Home Assistant)
//   Optional HTTPS on port 443, per-cycle CSV logging to FFat
//
// Home Assistant integration:
//   MQTT discovery prefix : homeassistant/
//   Device base topic     : supercharger/<HOSTNAME>/
//   Sensors (read-only)   : monolith_v, monolith_a, monolith_tmin,
//                           monolith_tmax, monolith_soc, monolith_ah_avail,
//                           powertank_v, powertank_a, powertank_tmin,
//                           powertank_tmax, current_power_w, session_wh,
//                           session_ah, target_preset_pct, thermal_throttle,
//                           ramp_phase, eta_minutes, cycle_count,
//                           cell_balance_mv, cell_avg_mv, bms_board_temp,
//                           odometer_km
//   Diagnostics           : wifi_rssi, uptime, firmware
//   Controls (read/write) : target_power_w (number), charger_count (number),
//                           ramp_rate_wps (number), target_volt_v (number),
//                           charging_enabled (switch)
//   Buttons               : preset_70 / preset_80 / preset_90 / preset_100
//                           (one per TARGET_VOLT_PRESETS entry), reset_session
//
// Required libraries (Arduino Library Manager):
//   mcp_can          by coryjfowler (important. Do not use alternatives, it will fail the code.)
//   PubSubClient     by Nick O'Leary
//   ArduinoJson      by Benoit Blanchon - this code uses the v6 API
//                    (StaticJsonDocument / containsKey). The library installed
//                    on the build host is 7.x, which still compiles it but
//                    warns on every use. Pin v6 or migrate the call sites.
//   ESPmDNS          } built into Espressif ESP32 Arduino core - no install needed
//   driver/twai.h    } built into Espressif ESP32 Arduino core - no install needed
//   WiFiClientSecure } built into Espressif ESP32 Arduino core - used for MQTT-TLS client
//   esp_https_server } built into Espressif ESP32 Arduino core - used for HTTPS port-443 serving
//   (No BLE: the Zero app's Bluetooth is a wireless bridge for these same CAN
//    frames, so there is nothing BLEDevice could add here.)
// ==========================================================================

#define VERSION 202609131746

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <arduino_secrets.h>
// Preferred names (2026-09): SECRET_AP_SSID/SECRET_AP_PASS = this device's own
// access point; SECRET_WIFI_SSID/SECRET_WIFI_PASS = the home network joined as a
// station. Older secrets files use SECRET_SSID/SECRET_PASS (AP) and
// SECRET_MQTT_SSID/SECRET_MQTT_PASS (station); map them so both still build.
// (SECRET_MQTT_HOST / _USER / _BROKER_PASS are the MQTT broker login and are
// unrelated to either WiFi pair.)
#ifndef SECRET_AP_SSID
  #define SECRET_AP_SSID SECRET_SSID
#endif
#ifndef SECRET_AP_PASS
  #define SECRET_AP_PASS SECRET_PASS
#endif
#ifndef SECRET_WIFI_SSID
  #define SECRET_WIFI_SSID SECRET_MQTT_SSID
#endif
#ifndef SECRET_WIFI_PASS
  #define SECRET_WIFI_PASS SECRET_MQTT_PASS
#endif
// Fixed AP-mode IP defines are optional — guard so a build host whose
// arduino_secrets.h predates this feature still compiles. Empty = disabled.
#ifndef SECRET_AP_IP
  #define SECRET_AP_IP ""
#endif
#ifndef SECRET_AP_GATEWAY
  #define SECRET_AP_GATEWAY ""
#endif
#ifndef SECRET_AP_SUBNET
  #define SECRET_AP_SUBNET ""
#endif
#include "supercharger.h"
#include "battery_tables.h"
#include "ZERO.h"
#include "Network.h"
#include "unions.h"
#include <Update.h>

#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SPI.h>
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <esp_task_wdt.h>    // task watchdog — rampTask + chargerBusTask subscribe (STAB-5)
#include <esp_system.h>      // esp_restart() — stack-overflow hook
#include <rom/ets_sys.h>     // ets_printf() — allocation-free printf for the overflow hook
#include <lwip/sockets.h>    // select() writability probe + SO_SNDTIMEO — SSE log stream
#include <PubSubClient.h>
#include <ArduinoJson.h>     // v6 — used by /api/settings, /api/control body parsing
#include <WiFiClientSecure.h>// MQTT-over-TLS path (mqttTls=true)
#include <mbedtls/md.h>      // HMAC-SHA256 for signed OTA verification
#include "ota_secret.h"      // OTA_HMAC_SECRET[32] — gitignored, generated per build host
// HttpCtx struct + ESP-IDF httpd_ssl request helpers.
// Must come before any .ino function definition so Arduino's prototype
// generator can see the struct type.
#include "https_ctx.h"
// CycleRecord struct — must be in a header so Arduino's auto-prototype pass
// sees the type before it emits the forward declaration for appendCycleRecord().
#include "cycle_record.h"
// LoginOutcome / LoginResult — same reason: tryLogin() returns LoginOutcome.
#include "auth_types.h"
#include <FFat.h>
#include <time.h>      // getLocalTime, struct tm — NTP timestamp for cycle records
#include <lwip/sockets.h>  // lwip_getpeername — per-IP login lockout on the HTTPS path

// ---------------------------------------------------------------------------
// Logging — ring buffer mirrored to hardware serial and SSE stream
//
// logPrintf() and logPushBuf() are plain free functions.
// NOT marked IRAM_ATTR: they call vsnprintf and Serial.write which live
// in flash, so an IRAM_ATTR tag would crash if flash cache were disabled
// (e.g. during an NVS write on the other core).
//
// Usage: LOG("fmt", ...) from any core or task. Call logBegin(baud) once.
// ---------------------------------------------------------------------------

#define LOG_BUF_SIZE 4096   // must be power of 2

static SemaphoreHandle_t logMutex = nullptr;
static char     logBuf[LOG_BUF_SIZE];
static uint32_t logHead = 0;
static uint32_t logSeq  = 0;
// Set true by logBegin() once HardwareSerial is initialised.
// Core 0 tasks must not call Serial.write() before this is true.
static volatile bool logReady = false;

static void logPushBuf(const char* buf, size_t len) {
  if (!logMutex || xSemaphoreTake(logMutex, 0) != pdTRUE) return;
  for (size_t i = 0; i < len; i++) {
    logBuf[logHead & (LOG_BUF_SIZE - 1)] = buf[i];
    logHead++;
  }
  logSeq += len;
  xSemaphoreGive(logMutex);
}

void logPrintf(const char* fmt, ...) {
  char tmp[256];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(tmp, sizeof(tmp), fmt, args);
  va_end(args);
  if (n > 0) {
    // Only write to HardwareSerial once it's been initialised.
    // logPushBuf() is always safe regardless of core or init state.
    if (logReady) Serial.write((const uint8_t*)tmp, (size_t)n);
    logPushBuf(tmp, (size_t)n);
  }
}

void logBegin(unsigned long baud) {
  Serial.begin(baud);
  logMutex = xSemaphoreCreateMutex();
  logReady = true;  // must be last — gates Core 0 Serial access
}

uint32_t logReadFrom(uint32_t fromSeq, char* out, size_t maxLen) {
  if (!logMutex || xSemaphoreTake(logMutex, pdMS_TO_TICKS(5)) != pdTRUE)
    return fromSeq;
  uint32_t oldest = (logHead > LOG_BUF_SIZE) ? (logHead - LOG_BUF_SIZE) : 0;
  if (fromSeq < oldest) fromSeq = oldest;
  uint32_t written = 0;
  while (fromSeq < logHead && written < maxLen) {
    out[written++] = logBuf[fromSeq & (LOG_BUF_SIZE - 1)];
    fromSeq++;
  }
  xSemaphoreGive(logMutex);
  return fromSeq;
}

#define LOG(...) logPrintf(__VA_ARGS__)

// ---------------------------------------------------------------------------
// Device state
// ---------------------------------------------------------------------------

enum DeviceState {
  STATE_CONNECTING,    // STA-only, waiting for initial association
  STATE_CONNECTED,     // STA-only, associated with home WiFi
  STATE_AP_RETRYING,   // AP+STA, AP up, STA reconnects in background (may be up or down)
  STATE_SETUP_MODE     // AP-only, no STA credentials available to retry
};

DeviceState currentState = STATE_CONNECTING;
Preferences preferences;
WebServer server(80);

unsigned long lastWifiCheck = 0;
const unsigned long wifiCheckInterval = 5000;
// While STATE_CONNECTING, poll WiFi status fast (not every 5 s) so the
// STATE_CONNECTED transition — mDNS register, NTP kickoff, home-profile apply —
// happens as soon as the link is actually up, instead of up to 5 s later.
const unsigned long wifiConnectingPollMs = 250;

// Tracks which credential source is currently being attempted, so
// monitorWifiStatus() knows whether to try the secrets fallback next
// or go straight to AP+STA retry mode.
enum WifiSource { WIFI_SRC_NONE, WIFI_SRC_PREFS, WIFI_SRC_SECRETS };
WifiSource wifiSource = WIFI_SRC_NONE;

// Timestamp of when WiFi.begin() was last called — used to enforce a
// connect timeout before declaring failure and moving to the next source.
unsigned long wifiConnectStartMs    = 0;
const unsigned long wifiConnectTimeoutMs = 15000; // 15 s per attempt

// True when the in-progress STA attempt used the cached-BSSID fast-connect
// path (directed WiFi.begin, no channel scan). If such an attempt times out,
// monitorWifiStatus() drops the stale cache and retries the same prefs creds
// once with a full scan before giving up — so a moved/roamed AP self-heals.
static bool wifiFastAttempt = false;

// STATE_AP_RETRYING bookkeeping. retryStaSsid/Pass hold the credentials we're
// continuously trying to reach; populated when entering the state from either
// a failed boot connect or a runtime drop. The driver's auto-reconnect handles
// most of the work, but we explicitly nudge it every staRetryIntervalMs as a
// belt-and-braces measure for cases where the driver gives up.
static char retryStaSsid[33] = "";
static char retryStaPass[65] = "";
const unsigned long staRetryIntervalMs = 30000UL;
static unsigned long lastStaRetryAt    = 0;

// AP grace period: once STA has been continuously up for AP_GRACE_MS while we
// were in STATE_AP_RETRYING, tear down the SoftAP and return to STA-only.
// Any STA drop during the grace window resets the timer (and AP keeps serving
// the dashboard). 0 == not currently stable.
const unsigned long AP_GRACE_MS = 90000UL;       // 90 s of stable STA
static unsigned long apStaStableSinceMs = 0;

// STA up/down edge latch for STATE_AP_RETRYING. File scope (not a function
// static) so enterApRetrying() can clear it on every entry — a function static
// survived the state change and made the first poll after re-entry look like
// "still up", skipping the reconnect bookkeeping (NET-18a).
static bool apRetryLastStaUp = false;

// Flap limiter (NET-18c): count CONNECTED → AP_RETRYING entries. More than
// AP_FLAP_MAX inside AP_FLAP_WINDOW_MS means the link is flapping, and cycling
// the SoftAP (and mDNS with it) once per flap costs more than it buys — so we
// keep the AP up and let the 30 s WiFi.reconnect() nudge do the work.
//
// Lifetime (B2): once engaged the limiter HOLDS for AP_FLAP_WINDOW_MS measured
// from the moment it engaged, then releases and normal SoftAP teardown resumes.
// It used to be timed from apFlapWindowStart — the start of the counting
// window — which is only ever written in the STATE_CONNECTED drop branch. That
// branch stops running the moment the limiter engages (the state machine sits
// in AP_RETRYING from then on), so the window start froze at the FIRST drop and
// the "expiry" test could already be satisfied seconds later, releasing the
// hold immediately and defeating the whole mechanism.
const unsigned long AP_FLAP_WINDOW_MS = 600000UL;  // 10 min
const uint8_t       AP_FLAP_MAX       = 3;         // entries allowed per window
static uint8_t       apFlapCount          = 0;
static unsigned long apFlapWindowStart    = 0;
static bool          apFlapLimited        = false;
static unsigned long apFlapLimitedSinceMs = 0;   // millis() when the hold engaged

// ---------------------------------------------------------------------------
// Live data — written by bikeBusTask() on Core 0, read by /api/status on Core 1
// Protected by liveMutex. All raw values follow Zero CAN library scaling:
//   voltage  : decivolts  (div 10  = V)
//   amps     : centiamps  (div 100 = A)
//   AH       : centi-Ah   (div 100 = Ah)
//   temp     : degrees C raw (no scaling)
//   maxCRate : thousandths of C (div 1000 = C-rate)
// ---------------------------------------------------------------------------

SemaphoreHandle_t liveMutex = nullptr;

struct LiveData {
  // Monolith (BMS0)
  long  monolithVoltageDv   = 0;
  short monolithSagAdjDv    = 0;  // sagAdjust from BMS_PACK_CONFIG (0x288) bytes 0-1
  short monolithAmps        = 0;
  short monolithAH          = 0;
  // Temperatures start INVALID, not 0 (audit 2026-09). 0 is a perfectly normal
  // pack reading, so a struct that has never received a 0x408 frame used to be
  // indistinguishable from a healthy 0 °C pack: the hot cutback saw "cold", the
  // 45 °C inhibit saw "fine", and the temp-unknown inhibit never armed. The
  // sentinel makes "no data yet" explicit to every consumer (all of which test
  // against TEMP_INVALID_THRESHOLD).
  short monolithMinTemp     = ZERO_TEMP_INVALID;
  short monolithMaxTemp     = ZERO_TEMP_INVALID;
  short monolithMaxCRate    = 0;
  // BMS-reported SoC % from BMS_PACK_STATUS (0x188 byte 0). 255 = no frame
  // received yet; falls back to voltage-curve estimate in that case.
  byte  monolithBmsSoc      = 255;

  // Per-cell voltages decoded from 0x388 rotating frame:
  //   byte 0 = cell index (0..27), bytes 1-2 = cell voltage uint16 LE (mV)
  // The BMS cycles through all 28 cells at ~10 Hz, so full coverage arrives
  // in about 3 s. cellSeenMask bit i is set once cell i has been received at
  // least once. cellBalanceMv = max − min across all seen cells (mV), 0 while
  // fewer than 2 cells have been seen.
  uint16_t cellVoltsMv[28]  = {};   // per-cell mV; 0 = not yet received
  uint32_t cellSeenMask     = 0;    // bitmask — bit i set when cell i received
  uint16_t cellBalanceMv    = 0;    // max − min (mV); 0 = insufficient data

  // BMS board temperature — 0x488 byte 1 (signed °C).
  // ZeroSpy labels this "Controller Temp". -128 = not yet received.
  int8_t   bmsBoardTempC    = -128;

  // Per-cell average voltage — 0x488 bytes 6-7 (uint16 LE, mV).
  // Tracks pack_voltage / 28 within ±2 mV in practice (confirmed from a full
  // 100% charge capture); BMS likely computes it independently rather than
  // exposing the raw arithmetic mean. Useful as a single "cell health" number
  // alongside cellBalanceMv. 0 = not yet received.
  uint16_t cellAvgMv        = 0;

  // Odometer (total distance) from DASH_ODO_FROM_DASH (0x2C0). Stored in
  // HECTOMETRES (0.1 km units) as decoded from the dash frame — km = /10.
  // 0 = not yet received. See the decode branch + the [CAN] +-2C0 diagnostic
  // log line for the byte layout and how to verify/adjust it.
  uint32_t odometerHm       = 0;

  // PowerTank (BMS1)
  long  powerTankVoltageDv  = 0;
  short powerTankSagAdjDv   = 0;  // sagAdjust from BMS1_PACK_CONFIG (0x289) bytes 0-1
  short powerTankAmps       = 0;
  short powerTankAH         = 0;
  // Same rationale as the monolith pair above — see that comment.
  short powerTankMinTemp    = ZERO_TEMP_INVALID;
  short powerTankMaxTemp    = ZERO_TEMP_INVALID;
  short powerTankMaxCRate   = 0;
  byte  powerTankBmsSoc     = 255;  // BMS1_PACK_STATUS (0x189 byte 0)

  // Presence / freshness flags
  bool  dataFresh           = false; // true once first BMS0 frame received
  bool  powerTankPresent    = false; // true once first BMS1 frame received
  bool  powerTankDecided    = false; // true once detection window has closed

  // Detection window timestamps (ms) — not sent in JSON
  unsigned long bms0FirstMs = 0;
  unsigned long bms1FirstMs = 0;
  // Timestamp of the most recent monolith voltage frame (0x388). Used by
  // rampTask as a liveness/staleness check: if the bike CAN bus drops
  // mid-charge this stops advancing and charging is halted. 0 = none yet.
  unsigned long bms0LastMs  = 0;
} live;

// Detection window: if no BMS1 frame arrives within this many ms of the
// first BMS0 frame, PowerTank is declared absent and the dashboard hides it.
static const unsigned long POWERTANK_DETECT_WINDOW_MS = 10000;

// Zero protocol decoder — stateless helper, safe to use from any task
Zero zeroDecoder;

// ---------------------------------------------------------------------------
// Charger CAN bus data — written by chargerBusTask(), read by /api/status
// Protected by chargerMutex, separate from liveMutex so the two tasks
// never contend on the same lock.
//
// Charger protocol (Elcon / J1939-based, extended 29-bit IDs):
//   Command frame  0x1806E5F4  BMS -> Charger  (heartbeat, 1 Hz)
//   Status frame   0x18FF50Ex  Charger -> BMS  (x = charger instance ID)
//
// Voltage/current in status frames: unit is 0.1 V / 0.1 A
// We store as raw 16-bit words and divide at JSON serialisation time.
//
// STATUS byte bitfield (byte 4 of status frame). Meanings below are per the
// Elcon TC charger CAN protocol document; they are NOT yet validated against
// this hardware, so treat them as the best available reading of the spec:
//   0x01  Hardware failure                      — ACTED ON (latches g_chargerFault)
//   0x02  Charger over-temperature              — ACTED ON (latches g_chargerFault)
//   0x04  AC input voltage out of range         — display only
//   0x08  Battery not detected / starting state — display only
//   0x10  Communication receive timeout         — display only
//
// Only 0x03 (hardware fault | overtemperature) is acted on; see the fault scan
// in rampTask. The other three are decoded and shown on the dashboard but never
// stop a charge:
//   * 0x04 asserts transiently on any mains dip and self-clears — latching a
//     stop on it would turn a flicker into a manual re-arm.
//   * 0x08 is the charger's own "no battery / still starting" state, which is
//     true for the first moments of every session before the output closes.
//   * 0x10 is the Elcon comm-timeout bit: the charger raises it by design after
//     ~5 s without a command frame, i.e. it is set on every idle unit and on
//     every unit that has just been powered up. Acting on it (the old 0x1B mask
//     did) latched a charger fault against a perfectly healthy idle charger.
// ---------------------------------------------------------------------------

SemaphoreHandle_t chargerMutex = nullptr;

// Charger status IDs 0x18FF50E0..EF — the low nibble is the unit's instance
// index and is used to index chargers[] (hence MAX_CHARGERS = 16). Units with
// distinct instance IDs (e.g. 5,7,8,9) are counted separately; units that share
// the factory default 0x18FF50E5 all land in slot 5 and count as ONE (see the
// HARDWARE NOTE below MAX_ACTIVE_CHARGERS).
static const uint32_t CHARGER_CMD_ID         = 0x1806E5F4UL;
static const uint32_t CHARGER_STATUS_ID_BASE = 0x18FF50E0UL; // mask low nibble
static const uint32_t CHARGER_STATUS_ID_MASK = 0x1FFFFFF0UL; // top 28 bits

// Maximum distinct chargers we track (nibbles 0–15).
// MUST stay 16: chargers[] is indexed by the raw CAN instance nibble, and the
// Elcon factory IDs use nibbles 5 / 7 / 8 / 9 (see the ID comment above). A
// smaller array would make processChargerFrame()'s `nibble >= MAX_CHARGERS`
// guard reject every real charger.
#define MAX_CHARGERS 16

// Largest charger count the rest of the system supports. The preset power table
// has 4 rows and ctrl.chargerCount is validated to 1..4, so rampTask's divisor —
// max(ctrl.chargerCount, chargerBus.chargerCount) — can never exceed 4. A fifth
// unit on the bus would therefore be divided for by only four: all units share
// one command frame, so each of the five would deliver a quarter-share and the
// pack would receive 5/4 of the commanded current. Detection is clamped here
// rather than in the nibble guard so extra units are noticed and logged
// (g_chargerCountClamped) instead of silently ignored.
//
// HARDWARE NOTE: the Elcon units on this bike all answer on the SAME instance
// ID (0x18FF50E5), so only one nibble is ever populated and the detected count
// is 1 no matter how many units are wired. The count-mismatch protection
// (g_chargerCountMismatch) is therefore INERT on this hardware — it can only
// ever fire if a unit is re-strapped to a different instance ID. It is kept
// because it costs nothing and is the correct behaviour for mixed IDs.
static const uint8_t MAX_ACTIVE_CHARGERS = 4;

struct ChargerUnit {
  bool     present   = false;
  uint16_t voltDv    = 0;    // actual output voltage * 10 (0.1 V units → dV)
  uint16_t ampsDa    = 0;    // actual output current * 10 (0.1 A units → dA)
  uint8_t  status    = 0;    // raw status bitfield
  // Consecutive status frames seen for this instance since it was last absent.
  // A unit is only declared `present` (and counted) on the CHARGER_SEEN_MIN_FRAMES'th
  // frame — one corrupted/aliased ID would otherwise invent a phantom charger
  // that inflates the divisor and trips the count-mismatch stop. Reset to 0 by
  // the presence-decay sweep so a returning unit re-qualifies from scratch.
  uint8_t  seenCount = 0;
  unsigned long lastSeenMs = 0;
};

// Status frames required before a charger instance counts as really there.
// 2 is enough: the Elcons broadcast status at ~1 Hz, so a genuine unit
// qualifies within ~1 s, while a single bit-flipped ID never does.
static const uint8_t CHARGER_SEEN_MIN_FRAMES = 2;

struct ChargerBusData {
  ChargerUnit chargers[MAX_CHARGERS]; // indexed by low nibble of status ID
  uint8_t     chargerCount  = 0;     // number of units seen in last 10 s
  bool        heartbeatOk   = false; // true while heartbeat is being sent

  // Command values written here by charging logic (round 3).
  // chargerBusTask reads these each heartbeat cycle.
  // Default: STOP with 0 V / 0 A — safe until explicitly commanded.
  uint16_t    cmdVoltDv     = 0;     // target voltage in 0.1 V units
  uint16_t    cmdAmpsDa     = 0;     // target current in 0.1 A units
  bool        cmdStart      = false; // false = STOP (0x01), true = START (0x00)
} chargerBus;

// Charge phase exposed to the API without a mutex (uint8_t write is atomic on ARM).
// 0 = CC / Absorption,  1 = CV / Float,  2 = Done / Complete
static volatile uint8_t g_rampPhase = 0;
// Dead-man liveness counter — rampTask bumps this once per tick (~1 Hz).
// chargerBusTask watches it: if it stops advancing, rampTask is wedged and the
// charger is forced to STOP rather than left charging on the last command
// forever. 32-bit read/write is atomic on the ESP32, so no mutex needed.
static volatile uint32_t g_rampHeartbeat = 0;
// Unconditional failsafe STOP request (STAB-2). Every rampTask decision that
// means "the chargers must not be running" raises this BEFORE it attempts the
// chargerMutex write that zeroes chargerBus.cmd*. Those writes are best-effort:
// they sit behind a 10 ms xSemaphoreTake with no else branch, so a contended
// mutex used to leave the previous START command in place — and because
// g_rampHeartbeat is bumped earlier in the tick, the dead-man in sendHeartbeat()
// could not catch it either. sendHeartbeat() therefore forces start=false /
// amps=0 whenever this flag is set, with no lock of its own (bool read/write is
// atomic on the ESP32). Cleared only by the command-update path at the end of a
// tick that actually wrote a valid START under the mutex — and only while
// g_shuttingDown is false (see below).
static volatile bool g_forceStop = false;
// Set by stopChargerForRestart() for the whole of its 1.5 s "let one STOP frame
// reach the wire" wait, and never cleared (the CPU is about to go away).
// rampTask keeps ticking during that wait — vTaskDelay() yields — so without
// this flag a single CC or CV tick landing inside the window would write a
// fresh START under chargerMutex and clear g_forceStop, and the reboot would
// leave the chargers running on that START until their own ~5 s heartbeat
// timeout. rampTask therefore only clears g_forceStop when this is false.
static volatile bool g_shuttingDown = false;
// True while CC-phase power is being clamped by the hot-temperature cutback.
// Read by the dashboard to display a "thermal throttling" banner.
static volatile bool    g_thermalThrottle = false;
// Estimated minutes remaining to reach the user's chosen target voltage.
// Updated by rampTask each tick using a coulomb-counting estimate plus EMA
// smoothing.  -1 = unknown / not charging / target already reached.
// 16-bit so the dashboard can display "—" when negative; capped at 24 h.
static volatile int16_t g_etaMinutes = -1;

// Why charging is currently inhibited outright, if it is. Distinct from
// g_thermalThrottle, which means "still charging, just at reduced power".
// An inhibit means zero current: the pack is outside the cell's safe charging
// envelope (Farasis datasheet: 0-45 °C, and see PACK_V_CHARGE_FLOOR_DV).
// Latched by rampTask with hysteresis so a cell sitting on a limit cannot
// chatter the chargers on and off once per second. 8-bit read/write is atomic
// on the ESP32, so no mutex needed.
enum ChargeInhibit : uint8_t {
  INHIBIT_NONE         = 0,
  INHIBIT_TOO_COLD     = 1,   // coldest cell below CHARGE_TEMP_MIN_C
  INHIBIT_TOO_HOT      = 2,   // hottest cell above CHARGE_TEMP_MAX_C
  INHIBIT_PACK_LOW     = 3,   // pack below PACK_V_CHARGE_FLOOR_DV
  // At least ONE of the two pack sensors (hottest / coldest) has had no usable
  // reading (ZERO_TEMP_INVALID) for longer than TEMP_INVALID_INHIBIT_MS. Either
  // one going dark is enough: the hot sensor guards the 45 °C limit and the hot
  // cutback, the cold sensor guards the 0 °C plating limit and COLD_CUTBACK, so
  // losing either leaves one half of the envelope unprotected. Without this the
  // gate silently stops protecting anything — a failed sensor used to decode as
  // 0 °C, which is inside the permitted window.
  INHIBIT_TEMP_UNKNOWN = 4,
};
static volatile uint8_t g_chargeInhibit = INHIBIT_NONE;

// Human-readable form for logs and the status API.
static const char* chargeInhibitName(uint8_t r) {
  switch (r) {
    case INHIBIT_TOO_COLD:     return "pack too cold";
    case INHIBIT_TOO_HOT:      return "pack too hot";
    case INHIBIT_PACK_LOW:     return "pack voltage too low";
    case INHIBIT_TEMP_UNKNOWN: return "pack temperature unknown";
    default:                   return "none";
  }
}

// ---------------------------------------------------------------------------
// Protection-state reporting for the dashboard banner.
//
// Each flag below is owned by the code path that detects the condition. They
// are deliberately NOT combined at the point of detection — the priority
// ordering lives in exactly one place (activeProtection()) so the firmware,
// the dashboard and any future MQTT sensor can never disagree about which
// condition is "the" current one. The dashboard shows a single banner for the
// highest-severity active state; duplicating this ordering in JavaScript would
// be a second source of truth waiting to drift.
// 8/32-bit read/write is atomic on the ESP32, so none of these need a mutex.
// ---------------------------------------------------------------------------

// Bike BMS telemetry older than BMS_STALE_TIMEOUT_MS while charging is enabled.
// Charging is stopped and waiting for fresh frames. Owned by rampTask.
static volatile bool g_bmsStale = false;

// More chargers answered on the CAN bus than MAX_ACTIVE_CHARGERS. The count was
// clamped, so the per-charger current divisor no longer matches reality and
// every unit would over-deliver. Owned by the charger-bus recount paths.
static volatile bool g_chargerCountClamped = false;

// More chargers are answering on the CAN bus than ctrl.chargerCount says are
// installed (STAB-1). The per-charger current divisor is the configured count,
// so every extra unit would deliver a full share on top of what was asked for.
// Charging is stopped until the setting matches. Owned by rampTask, recomputed
// every tick, so it clears itself once the two agree.
static volatile bool g_chargerCountMismatch = false;

// A charger reported a hardware-failure or over-temperature status bit (mask
// 0x03 — see the status bitfield table above), or output voltage well above the
// commanded ceiling (STAB-4). LATCHED: cleared only when the user switches
// charging off and on again, so a unit that faults, drops off the bus and comes
// back cannot silently resume. Owned by rampTask.
//
// The latch lives in RAM only, so a reboot (deliberate or a crash) also clears
// it. That is deliberate for now: with the default boot profiles (charging off)
// a reboot re-runs the whole start sequence and the user still has to press
// Charge, which is the same manual re-arm the latch demands. Caveat: if
// def_chg_en / home_chg_en are set to ON, a reboot both clears the latch and
// re-enables charging without a human in the loop. Persisting
// it to NVS would survive a power cycle of the chargers themselves — which is
// the usual way a user clears a genuine charger fault — and would need its own
// "clear latched fault" UI. Revisit if a fault is ever seen to recur across a
// reboot without the user noticing.
static volatile bool g_chargerFault = false;

// The absolute 1.0 C ceiling (CELL_MAX_CHARGE_C × pack Ah) is actively limiting
// commanded current, i.e. something upstream asked for more than the cells are
// rated to accept. Owned by rampTask's amp calculation.
static volatile bool g_currentClamped = false;

// millis() of the most recent CAN frame rejected as implausible (pack voltage
// outside the 28S window). 0 = none since boot. A timestamp rather than a
// sticky flag so the banner self-clears once the bus is healthy again.
static volatile unsigned long g_lastBadFrameMs = 0;
static const unsigned long BAD_FRAME_BANNER_MS = 10000UL;  // "recent" window

// Ordered worst-first. The numeric values are not persisted anywhere, so this
// list can be reordered freely; only the relative order matters.
enum ProtState : uint8_t {
  PROT_NONE = 0,
  PROT_TOO_COLD,           // charging inhibited — below datasheet minimum
  PROT_TOO_HOT,            // charging inhibited — above datasheet maximum
  PROT_PACK_LOW,           // charging inhibited — risk of copper dissolution
  PROT_TEMP_UNKNOWN,       // charging inhibited — no valid pack temperature
  PROT_BMS_STALE,          // charging stopped — no fresh bike telemetry
  PROT_CHARGER_FAULT,      // charging stopped — a charger reports a fault
  PROT_CHARGER_MISMATCH,   // charging stopped — more chargers than configured
  PROT_CHARGER_CLAMPED,    // wiring/config fault — too many chargers
  PROT_THERMAL_THROTTLE,   // still charging, power reduced by hot cutback
  PROT_CURRENT_CLAMPED,    // still charging, power reduced by the 1 C ceiling
  PROT_FRAMES_REJECTED,    // informational — bad CAN frames being dropped
};

// Stable machine-readable keys. The dashboard maps these to copy, so changing a
// string here means changing the matching key in HTML_DASHBOARD's JS.
static const char* protStateKey(uint8_t s) {
  switch (s) {
    case PROT_TOO_COLD:          return "too_cold";
    case PROT_TOO_HOT:           return "too_hot";
    case PROT_PACK_LOW:          return "pack_low";
    case PROT_TEMP_UNKNOWN:      return "temp_unknown";
    case PROT_BMS_STALE:         return "bms_stale";
    case PROT_CHARGER_FAULT:     return "charger_fault";
    case PROT_CHARGER_MISMATCH:  return "charger_mismatch";
    case PROT_CHARGER_CLAMPED:   return "charger_clamped";
    case PROT_THERMAL_THROTTLE:  return "thermal_throttle";
    case PROT_CURRENT_CLAMPED:   return "current_clamped";
    case PROT_FRAMES_REJECTED:   return "frames_rejected";
    default:                     return "none";
  }
}

// The single highest-severity active protection state. An outright inhibit
// outranks everything: if the pack is too cold to charge, saying "power reduced"
// as well would be actively misleading.
static uint8_t activeProtection() {
  switch (g_chargeInhibit) {
    case INHIBIT_TOO_COLD:     return PROT_TOO_COLD;
    case INHIBIT_TOO_HOT:      return PROT_TOO_HOT;
    case INHIBIT_PACK_LOW:     return PROT_PACK_LOW;
    case INHIBIT_TEMP_UNKNOWN: return PROT_TEMP_UNKNOWN;
    default: break;
  }
  if (g_bmsStale)             return PROT_BMS_STALE;
  if (g_chargerFault)         return PROT_CHARGER_FAULT;
  if (g_chargerCountMismatch) return PROT_CHARGER_MISMATCH;
  if (g_chargerCountClamped)  return PROT_CHARGER_CLAMPED;
  if (g_thermalThrottle)      return PROT_THERMAL_THROTTLE;
  if (g_currentClamped)       return PROT_CURRENT_CLAMPED;
  if (g_lastBadFrameMs != 0 &&
      (millis() - g_lastBadFrameMs) < BAD_FRAME_BANNER_MS)
    return PROT_FRAMES_REJECTED;
  return PROT_NONE;
}

// Charger is considered gone if no status frame received within this window
static const unsigned long CHARGER_TIMEOUT_MS = 10000;

// Consecutive failed heartbeat transmits before we re-initialise the MCP2515
// (M2). At HEARTBEAT_INTERVAL_MS this is the recovery latency, so keep it well
// above CHARGER_TIMEOUT_MS / HEARTBEAT_INTERVAL_MS — a reset is itself
// disruptive and we only want it for a genuinely wedged controller, not for
// "no charger plugged in yet", which also fails to ACK.
static const uint16_t MCP_TX_FAIL_RESET_THRESHOLD = 30;

// Heartbeat interval — must stay well under the charger's 5 s cutoff
static const unsigned long HEARTBEAT_INTERVAL_MS = 1000;

// ---------------------------------------------------------------------------
// Charging control — written by web handlers (Core 1) and ramp task (Core 1)
// Read by chargerBusTask (Core 0) under chargerMutex.
//
// Max charge voltage derived from the last (highest) entry of VOLTAGE_CUTBACK.
// That entry represents the hardest cutback point — we never command beyond it.
// ---------------------------------------------------------------------------

// Preset power table [charger_count-1][preset_index], watts
// Each row = charger_count × [500, 1000, 1650, 2200, 3300] rounded to nice numbers.
// Zero-terminated: JS filters out trailing zeroes.
static const uint16_t POWER_PRESETS[][5] = {
  {  500, 1000, 1650, 2200,  3300 },  // 1 charger  (max  3.3 kW)
  { 1000, 2000, 3300, 4400,  6600 },  // 2 chargers (max  6.6 kW)
  { 1500, 3000, 5000, 6600,  9900 },  // 3 chargers (max  9.9 kW)
  { 2000, 4000, 6600, 8800, 13200 },  // 4 chargers (max 13.2 kW)
};
static const int MAX_PRESETS_PER_ROW = 5;
static const int MAX_CHARGER_ROWS = 4;

// Max charge voltage (dV) — last (highest) threshold in VOLTAGE_CUTBACK table.
// Evaluated at runtime init, not compile time, because VOLTAGE_CUTBACK is
// const not constexpr. Value is fixed after startup and never changes.
static const uint16_t MAX_CHARGE_VOLTAGE_DV =
  VOLTAGE_CUTBACK[ (sizeof(VOLTAGE_CUTBACK)/sizeof(VOLTAGE_CUTBACK[0])) - 1 ].threshold;

// ---------------------------------------------------------------------------
// Cell-derived limits — Farasis IMP06160230P25A, 25 Ah NMC pouch, 28S monolith
// (datasheet Farasis Energy V5, Aug 2011):
//
//   Nominal voltage        3.65 V      → 28S nominal 102.2 V
//   Constant voltage       4.15-4.20 V → 28S CV      116.2-117.6 V
//   Discharge V (min)      2.00 V      → 28S floor    56.0 V
//   Max charge current     25 A on a 25 Ah cell = 1.0 C
//   Charging temp          0 to 45 °C
//
// Three DIFFERENT thresholds live below. Keeping them separate matters:
//
//   1. PLAUSIBILITY  — "could a working BMS have sent this?" Purpose is to
//      reject corrupt CAN frames. Deliberately WIDE, so a genuinely flat pack
//      still reads correctly and can be reported as flat rather than
//      disappearing into a "BMS data stale" failsafe.
//   2. CHARGE FLOOR  — "is it safe to push current into this pack?" This is a
//      protection limit, and it is much higher than the plausibility floor.
//   3. CHARGE TARGET — MAX_CHARGE_VOLTAGE_DV above, what we aim for.
//
// Collapsing 1 and 2 into one constant is the tempting mistake: it makes an
// over-discharged pack look like a comms fault.
static const uint8_t  PACK_SERIES_COUNT = 28;    // 28S — matches cellVoltsMv[28]

// (1) Plausibility window. 1.5 V/cell is below anything a healthy cell reaches
// but still a decodable reading, so a deeply discharged pack is *visible*.
// 4.20 V/cell is the datasheet CV maximum — nothing above it is physical.
static const uint16_t CELL_V_PLAUSIBLE_MIN_MV = 1500;
static const uint16_t CELL_V_PLAUSIBLE_MAX_MV = 4200;

// (2) Charge-inhibit floor. Below ~2.5 V/cell the copper anode current
// collector begins to dissolve; on recharge the dissolved copper can plate out
// as an internal short. This is a distinct failure mode from the lithium
// plating that COLD_CUTBACK guards against, and it is not recoverable by
// charging normally — industry guidance is that a cell taken below 2.5 V should
// only ever see a ≤0.05 C recovery charge under supervision, not a 1 C CC ramp.
// The Farasis sheet lists 2.0 V as "Discharge V (min)", which is the absolute
// floor, not a safe recharge point; the Zero BMS should itself cut out around
// 3.0 V/cell, so reaching this gate at all means something is already wrong.
// We therefore refuse to charge rather than attempt recovery.
static const uint16_t CELL_V_CHARGE_FLOOR_MV = 2500;

// Derived, so the series count and the per-cell limits can never drift apart.
//   mV × cells / 100 = dV
static const uint16_t PACK_V_MIN_DV =
  (uint16_t)((uint32_t)CELL_V_PLAUSIBLE_MIN_MV * PACK_SERIES_COUNT / 100);  //  420 dV
static const uint16_t PACK_V_MAX_DV =
  (uint16_t)((uint32_t)CELL_V_PLAUSIBLE_MAX_MV * PACK_SERIES_COUNT / 100);  // 1176 dV
static const uint16_t PACK_V_CHARGE_FLOOR_DV =
  (uint16_t)((uint32_t)CELL_V_CHARGE_FLOOR_MV  * PACK_SERIES_COUNT / 100);  //  700 dV

// Cell max charge current is 25 A on a 25 Ah cell, i.e. exactly 1.0 C. Because
// the C-rate is per-cell and packAH scales with the parallel count, 1.0 C of
// the BMS-reported pack Ah is the correct total-current ceiling regardless of
// how many cells sit in parallel — no need to hard-code the P count.
static const float CELL_MAX_CHARGE_C = 1.0f;

// (4) Pack-capacity plausibility (STAB-3). The BMS-reported pack Ah from
// BMS_PACK_CONFIG (0x288 bytes 5-6) is the multiplier on the 1.0 C ceiling
// above, on every cutback table's power limit, on the CV current floor and on
// the ETA — an implausibly large value quietly lifts the "absolute" current
// ceiling out of the way entirely. A stock Zero monolith reports ~114 Ah and a
// monolith + PowerTank is still well under 200 Ah, so this window is wide
// enough to never reject a real pack while catching a corrupt frame.
static const short PACK_AH_PLAUSIBLE_MIN = 20;
static const short PACK_AH_PLAUSIBLE_MAX = 400;
// Same ceiling as a float, applied again in rampTask as a belt-and-braces clamp
// before packAH is used — so the two can never drift apart.
static constexpr float PACK_AH_MAX = (float)PACK_AH_PLAUSIBLE_MAX;

// ---------------------------------------------------------------------------
// Charging temperature window — Farasis datasheet "Charging Temp. 0°C to 45°C".
// (Operating range is -20..60 °C, but that is DISCHARGE; charging is narrower.)
//
// Enforced as a hard inhibit in rampTask, with hysteresis so a cell hovering on
// the boundary doesn't chatter the chargers on and off once per second. The
// COLD/HOT cutback tables shape the C-rate *inside* this window; they are not
// the boundary itself, and they are kept overlapping it as a second,
// independent layer in case the gate is ever bypassed by a future refactor.
static const short CHARGE_TEMP_MIN_C        = 0;   // datasheet minimum
static const short CHARGE_TEMP_MAX_C        = 45;  // datasheet maximum
static const short CHARGE_TEMP_HYSTERESIS_C = 2;   // re-arm 2 °C inside the limit

// Ramp rate: default 100 W per ramp tick (1 s), slider step 100 W
static const uint16_t DEFAULT_RAMP_STEP_W = 100;
static const uint16_t SLIDER_STEP_W = 100;

// Voltage-to-SOC lookup table (decivolts → percent)
// Based on Zero monolith pack open-circuit voltage curve (28S Li-ion NMC)
struct VoltSocEntry { uint16_t dv; uint8_t soc; };
static const VoltSocEntry VOLT_SOC_TABLE[] = {
  {  820,   0 },
  {  870,  10 },
  {  900,  20 },
  {  930,  30 },
  {  960,  40 },
  {  990,  50 },
  { 1020,  60 },
  { 1060,  70 },
  { 1100,  80 },
  { 1132,  90 },
  { 1164, 100 }
};
static const int VOLT_SOC_COUNT = sizeof(VOLT_SOC_TABLE) / sizeof(VOLT_SOC_TABLE[0]);

// Target voltage presets (percent → decivolts)
struct TargetVoltPreset { uint8_t pct; uint16_t dv; };
static const TargetVoltPreset TARGET_VOLT_PRESETS[] = {
  {  70, 1060 },
  {  80, 1100 },
  {  90, 1132 },
  { 100, 1164 }
};
static const int TARGET_VOLT_PRESET_COUNT = sizeof(TARGET_VOLT_PRESETS) / sizeof(TARGET_VOLT_PRESETS[0]);

// Calculate SOC from voltage (decivolts) using linear interpolation
int calcSocFromVoltage(long voltageDv) {
  if (voltageDv <= VOLT_SOC_TABLE[0].dv) return 0;
  if (voltageDv >= VOLT_SOC_TABLE[VOLT_SOC_COUNT - 1].dv) return 100;
  for (int i = 1; i < VOLT_SOC_COUNT; i++) {
    if (voltageDv <= VOLT_SOC_TABLE[i].dv) {
      long v0 = VOLT_SOC_TABLE[i-1].dv, v1 = VOLT_SOC_TABLE[i].dv;
      int  s0 = VOLT_SOC_TABLE[i-1].soc, s1 = VOLT_SOC_TABLE[i].soc;
      return s0 + (int)((voltageDv - v0) * (s1 - s0) / (v1 - v0));
    }
  }
  return 100;
}

SemaphoreHandle_t controlMutex = nullptr;

struct ChargingControl {
  uint16_t targetPowerW  = 0;     // desired end-state power, set by web UI
  uint16_t currentPowerW = 0;     // actual commanded power, ramped each second
  bool     enabled       = false; // set from defaultEnabled at boot in rampInit()
  uint8_t  chargerCount  = 3;     // manual charger count for per-unit current division
  uint16_t rampStepW     = DEFAULT_RAMP_STEP_W; // W per second ramp rate, configurable
  uint16_t targetVoltDv  = 1100;  // target charge voltage (dV), overwritten from NVS in rampInit()
  // Boot-time defaults (AP / road mode) — loaded from NVS in rampInit(), applied immediately
  bool     defaultEnabled        = false; // charging on/off state at boot
  uint8_t  defaultPowerPresetIdx = 1;     // POWER_PRESETS column at boot (idx 1 = 1/2/3/4 kW by charger count)
  uint16_t defaultTargetVoltDv   = 1100;  // target voltage at boot (dV); 1100 = 80%
  // Home WiFi defaults — loaded from NVS in rampInit(), applied once STA first connects
  bool     homeDefaultEnabled        = false;
  uint8_t  homeDefaultPowerPresetIdx = 1;
  uint16_t homeDefaultTargetVoltDv   = 1100;
} ctrl;

// Set to true the first time home WiFi defaults are applied (STATE_CONNECTING →
// STATE_CONNECTED transition). Prevents re-applying on every WiFi reconnect.
//
// C3 — three tasks write this flag, so it is volatile:
//   * loopTask       — onStaUp() / applyHomeWifiBootDefaults()
//   * the httpd task — /api/control, after a successful controlMutex take
//   * mqttTask       — mqttCallback(), likewise only on a command that applied
// No mutex guards it and none is needed. It is a single byte that is only ever
// written with the value `true` and only ever read as "has anyone latched it
// yet" — the write is atomic on this target, there is no read-modify-write to
// tear, and no ordering between the writers matters because they all agree on
// the value. volatile is what stops the compiler from caching the read in
// applyHomeWifiBootDefaults() across a call that a different task could have
// raced. The worst case remains a benign race the design already tolerates:
// a user command landing in the same instant as the STA-up edge may or may not
// beat the profile, which is exactly the ambiguity the flag exists to bound.
static volatile bool homeDefaultsApplied = false;

// Session energy tracking — accumulated in rampTask, reset on boot or manual reset.
// Guarded by sessionMutex: rampTask does read-modify-write `+= delta` each tick
// while /api/control and the MQTT reset command write zero across all three
// fields. Without a mutex, a reset can land between rampTask's load and store,
// and a subsequent accumulator step then re-introduces a non-zero energyWh
// while chargeAh is already zero — visibly inconsistent on the dashboard.
struct SessionData {
  float energyWh  = 0.0f;  // cumulative Wh delivered this session
  float chargeAh  = 0.0f;  // cumulative Ah delivered this session
  unsigned long startMs = 0; // session start timestamp
} session;

// CycleRecord struct is in cycle_record.h (included at top with other headers).

SemaphoreHandle_t sessionMutex = nullptr;

// Helpers for cross-task session access. The mutex is created in rampInit();
// before that runs there's only setup() executing on Core 1, so a null mutex
// means "no contention possible" — fall through to a direct read/write.
static inline void sessionAddCC(float wh, float ah) {
  if (sessionMutex && xSemaphoreTake(sessionMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    session.energyWh += wh;
    session.chargeAh += ah;
    xSemaphoreGive(sessionMutex);
  } else {
    session.energyWh += wh;
    session.chargeAh += ah;
  }
}

static inline void sessionReset() {
  if (sessionMutex && xSemaphoreTake(sessionMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    session.energyWh = 0.0f;
    session.chargeAh = 0.0f;
    session.startMs  = millis();
    xSemaphoreGive(sessionMutex);
  } else {
    session.energyWh = 0.0f;
    session.chargeAh = 0.0f;
    session.startMs  = millis();
  }
}

// Atomic read of the two accumulators together — guarantees the dashboard /
// MQTT publisher never sees a half-reset state (energy=0, charge=non-zero).
static inline void sessionSnapshot(float& wh, float& ah) {
  if (sessionMutex && xSemaphoreTake(sessionMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    wh = session.energyWh;
    ah = session.chargeAh;
    xSemaphoreGive(sessionMutex);
  } else {
    wh = session.energyWh;
    ah = session.chargeAh;
  }
}

static TaskHandle_t rampTaskHandle = nullptr;

// ---------------------------------------------------------------------------
// System stats — CPU load per core and free heap
// Sampled every 1 s by sysStatsTask on Core 1.
// Protected by sysStatsMutex.
// ---------------------------------------------------------------------------

SemaphoreHandle_t sysStatsMutex = nullptr;

struct SysStats {
  uint8_t  load0    = 0;   // Core 0 load 0-100 %
  uint8_t  load1    = 0;   // Core 1 load 0-100 %
  uint32_t freeHeap = 0;   // bytes
} sysStats;

static TaskHandle_t sysStatsTaskHandle = nullptr;

// ---------------------------------------------------------------------------
// MQTT — Home Assistant integration via PubSubClient
//
// Topic layout:
//   supercharger/<hostname>/state                     online / offline (LWT)
//   supercharger/<hostname>/sensor/<name>             published sensor values
//   supercharger/<hostname>/command/target_power_w    inbound: uint16 watts
//   supercharger/<hostname>/command/charging_enabled  inbound: "true" / "false"
//   supercharger/<hostname>/command/charger_count     inbound: 1-4
//   supercharger/<hostname>/command/ramp_rate_wps     inbound: 10-500 W/s
//   supercharger/<hostname>/command/target_volt_v     inbound: V  e.g. 106.0/110.0/113.2/116.4
//   supercharger/<hostname>/command/reset_session     inbound: any payload triggers reset
//
// HA Discovery topics:
//   homeassistant/<component>/supercharger_<hostname>/<name>/config
//
// Publishes on change, with a 10 s keepalive republish to maintain
// HA availability even when values are stable.
// ---------------------------------------------------------------------------

// Snapshot of last-published values — used to detect changes.
// Float fields must match the type used in PUB_IF_CHANGED_F comparisons
// to avoid truncation causing every cycle to look "changed".
struct MqttSnapshot {
  float   monolithVoltageDv  = -1;
  float   monolithAmps       = -1;
  short   monolithMinTemp    = -1;
  short   monolithMaxTemp    = -1;
  int     monolithSoc        = -1;
  float   powerTankVoltageDv = -1;
  float   powerTankAmps      = -1;
  short   powerTankMinTemp   = -1;
  short   powerTankMaxTemp   = -1;
  uint8_t chargerCount       = 255;
  uint16_t currentPowerW     = 65535;
  uint16_t targetPowerW      = 65535;
  bool    enabled            = false;
  bool    enabledForced      = true;  // true after reset → guarantees one publish even when enabled==false
  bool    powerTankPresent   = false;
  float   sessionWh          = -1;
  float   sessionAh          = -1;
  uint16_t rampStepW         = 65535;
  float    targetVoltDv      = -1.0f;  // stored as V (dV/10) for float comparison
  bool     thermalThrottle   = false;
  bool     thermalThrottleForced = true; // guarantee one publish after boot
  uint8_t  targetPresetPct   = 255;     // 255 = sentinel, forces first publish
  uint8_t  rampPhase         = 255;     // 255 = sentinel, forces first publish
  int16_t  etaMinutes        = -2;      // -2 = sentinel (firmware uses -1 for "unknown"); forces first publish
  float    monolithAhAvail   = -1.0f;   // forces first publish
  uint16_t cycleCount        = 65535;   // 65535 = sentinel, forces first publish
  uint16_t cellBalanceMv     = 65535;   // sentinel, forces first publish
  int16_t  bmsBoardTempC     = -256;    // sentinel (outside int8 range), forces first publish
  uint16_t cellAvgMv         = 65535;   // sentinel, forces first publish
  uint32_t odometerHm        = 0xFFFFFFFFUL; // sentinel, forces first publish
} mqttLast;

// MQTT transport — both a plaintext and a TLS client live in BSS so we can
// hot-swap PubSubClient's underlying transport when the user toggles SSL on
// the settings page. Only one is "active" at a time (selected by mqttTls).
// Memory cost of the TLS client is ~6 KB even when unused, but on ESP32-S3
// that's not a concern.
static WiFiClient        mqttWifiClient;
static WiFiClientSecure  mqttTlsClient;
static PubSubClient      mqttClient(mqttWifiClient);
static TaskHandle_t      mqttTaskHandle = nullptr;
static char              mqttHostname[32] = "supercharger";

// Runtime-configurable AP and MQTT settings.
// Loaded from NVS at boot, falling back to arduino_secrets.h, then defaults.
// Written by /api/settings POST handler; read by startAPMode() and mqttTask().
static char     apSSID[33]         = "Supercharger";
static char     apPass[65]         = "12345678";

// Fixed AP-mode IP — optional override of the ESP32 default 192.168.4.1.
// When enabled, WiFi.softAPConfig() is called before WiFi.softAP() so AP mode
// serves the dashboard at a chosen fixed IP (lets one iOS home-screen shortcut
// work both at home and in AP mode). Strings hold dotted-quad IPv4 — 15 chars
// max + NUL. Loaded from NVS at boot, falling back to SECRET_AP_* defines.
static bool     apStaticIpEnabled  = false;
static char     apStaticIp[16]     = "";
static char     apStaticGateway[16]= "";
static char     apStaticSubnet[16] = "";

static char     mqttHost[64]       = "";
static uint16_t mqttPort           = 1883;
static char     mqttUser[33]       = "";
static char     mqttBrokerPass[65] = "";

// MQTT-TLS configuration. mqttTls=true requires mqttCaCert to be a valid PEM
// blob (broker's CA cert or self-signed cert). Saved as NVS preferences
// "mqtt_tls" (bool) and "mqtt_ca" (string blob, max ~2.5 KB). When TLS is on
// we pass the CA cert via WiFiClientSecure::setCACert() before each connect
// attempt; without a valid cert, mqttConnect() refuses to connect rather
// than silently downgrading to setInsecure() — pretending to be secure is
// worse than plaintext.
static bool     mqttTls            = false;
// Cert blob — sized to fit a typical X.509 CA (~1.5 KB) plus headroom.
// Stored as a String to avoid yet another fixed-size buffer, but persisted
// as a regular preferences string. PEM format with BEGIN/END markers.
static String   mqttCaCert         = "";

// MCP2515 SPI pins (LilyGo T-2CAN hardware)
#define MCP_CS_PIN    10
#define MCP_SCLK_PIN  12
#define MCP_MOSI_PIN  11
#define MCP_MISO_PIN  13
#define MCP_RST_PIN    9

// BOOT button on the LilyGo T-2CAN (GPIO 0). Strapping pin — must read HIGH
// at boot for normal mode, so leave INPUT_PULLUP and only sample after setup().
// Pressed = LOW (button shorts to GND).
#define BOOT_BUTTON_PIN 0
#define BTN_HOLD_AP_RESET_MS    5000UL   // 5 s : clear WiFi creds + ALL login
                                         //       sessions → AP mode on next boot
#define BTN_HOLD_FACTORY_MS    10000UL   // 10 s: wipe entire NVS namespace

// ---------------------------------------------------------------------------
// HTTPS / TLS server — optional port-443 server (ESP-IDF httpd_ssl).
//
// Implementation: ESP-IDF's esp_https_server (httpd_ssl_*). Built into the
// ESP32 Arduino core; runs in its own FreeRTOS task — no loop() polling.
//
// HttpCtx struct, IDF→ctx adapter (initFromIDFReq), URL decode, and
// parseKVPairs are in https_ctx.h (included above).
//
// When the HTTPS server actually came up (g_httpsRunning):
//   • Port 80  WebServer serves the HTTP-only routes — /login GET+POST,
//              /logout POST, /update GET+POST (OTA), /log, /api/log/stream,
//              /save, /api/tls (escape hatch so the user can disable HTTPS
//              over HTTP if cert upload was misconfigured) — and redirects
//              everything else to https:// on the same host.
//   • Port 443 httpd_ssl_server runs in its own task. Each registered URI
//              hits a thin idf_*() wrapper that builds an HttpCtx and calls
//              the same httpsHandle*() function used elsewhere.
//
// Keep-alive is handled internally by httpd_ssl. Dashboard 2 s polling stays
// on a single TLS session.
//
// Routes intentionally HTTP-only (not registered on 443):
//   /update OTA, /api/log/stream SSE, /log, /save — the IDF path needs a
//   chunked/streaming or multipart-upload story before they can move over.
//   SEC-4/NET-2: they are registered on port 80 in BOTH states, so enabling
//   HTTPS no longer makes OTA and the log viewer vanish from both ports.
//   /login is likewise on both ports: the 443 cookie carries Secure and is
//   never sent over HTTP, so these routes need their own (non-Secure) login.
//   (/api/cycles GET+DELETE moved to 443 in the 2026-07 audit — chunked
//   file streaming turned out to be enough for it.)
// ---------------------------------------------------------------------------

// HTTPS server state.
//   httpsEnabled   — the PERSISTED INTENT (NVS key "https_en"). What the
//                    Settings page shows and what the next boot will try.
//   g_httpsRunning — the LIVE FACT: true only after startHTTPSServer()
//                    actually succeeded in setup(). SEC-15/NET-6: the port-80
//                    redirect and the port-80 route table consult this and
//                    NEVER httpsEnabled, so toggling the setting on (which
//                    only takes effect after a reboot) can't start bouncing
//                    HTTP clients to a port 443 that nothing is listening on.
static bool            httpsEnabled   = false;
static bool            g_httpsRunning = false;
static httpd_handle_t  g_httpsServer  = nullptr;

// FFat cycle-data logger state
static bool     g_fatReady   = false;  // true after FFat.begin() succeeds
static uint16_t g_cycleCount = 0;      // number of records in /cycles.csv (cached)

// ---------------------------------------------------------------------------
// Login page — standard HTML form so password managers can autofill.
// The %s slot is replaced with an empty string (no error) or an error
// fragment; use snprintf(buf, sizeof(buf), HTML_LOGIN, errFrag).
//
// The fragment carries three cases, all firmware-generated (UX-10): the
// ?err=1 bad-credentials line, the 423 hard-lock banner and the 429
// rate-limit banner, the last two built by authBuildRetryFrag() with the
// same Retry-After countdown the header carries. Nothing derived from the
// request is ever interpolated here, so the page needs no HTML escaping —
// keep it that way if the fragment ever grows a fourth case.
// Note: literal % in CSS must be written %% so snprintf doesn't choke.
// ---------------------------------------------------------------------------

const char HTML_LOGIN[] PROGMEM =
  "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
  "<title>Supercharger - Login</title>"
  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<style>"
  "body{font-family:sans-serif;background:#0a0a1a;color:#ccc;"
  "display:flex;flex-direction:column;align-items:center;"
  "justify-content:center;min-height:100vh;margin:0}"
  "h1{color:#e94560;margin-bottom:24px;font-size:1.4rem}"
  "form{background:#16213e;padding:28px 32px;border-radius:10px;"
  "min-width:260px;display:flex;flex-direction:column;gap:12px}"
  "label{font-size:.85rem;color:#aaa}"
  "input{background:#0f3460;border:1px solid #334;color:#eee;"
  "padding:9px 12px;border-radius:6px;font-size:1rem;"
  "width:100%%;box-sizing:border-box}"
  "input:focus{border-color:#e94560;outline:none}"
  "button{background:#e94560;color:#fff;border:none;padding:10px;"
  "border-radius:6px;font-size:1rem;cursor:pointer;font-weight:bold}"
  "button:hover{background:#c73652}"
  ".err{color:#e94560;font-size:.9rem;text-align:center;margin:0}"
  ".hint{color:#8a8aa0;font-size:.78rem;text-align:center;margin:-4px 0 0;"
  "line-height:1.4;white-space:pre-line}"
  ".rem{display:flex;align-items:center;gap:8px;cursor:pointer;font-size:.85rem;color:#aaa}"
  ".rem input{width:auto}"
  "</style></head><body>"
  "<h1>&#9889; Supercharger</h1>"
  "<form method='POST' action='/login' autocomplete='on'>"
  "<label for='u'>Username</label>"
  "<input id='u' name='username' type='text' autocomplete='username' required>"
  "<label for='p'>Password</label>"
  "<input id='p' name='password' type='password' autocomplete='current-password' required>"
  "<label class='rem'><input type='checkbox' name='remember' value='1' checked>"
  "<span>Keep me signed in on this device</span></label>"
  "%s"   // error fragment — empty string or <p class='err'>…</p>
  "<button type='submit'>Log in</button>"
  "</form></body></html>";

// ---------------------------------------------------------------------------
// WiFi setup page — only served in AP / setup mode
// ---------------------------------------------------------------------------

const char HTML_SETTINGS[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
  <meta name="apple-mobile-web-app-capable" content="yes">
  <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
  <meta name="theme-color" content="#1a1a2e">
  <title>Settings — Supercharger</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;padding:16px;max-width:600px;margin:auto;
         padding-top:calc(16px + env(safe-area-inset-top));
         padding-bottom:calc(16px + env(safe-area-inset-bottom));
         padding-left:calc(16px + env(safe-area-inset-left));
         padding-right:calc(16px + env(safe-area-inset-right))}
    h1{color:#e94560;font-size:1.4em;margin-bottom:6px}
    nav{display:flex;flex-wrap:wrap}
    /* UX-11: 44 px minimum touch target. */
    nav a{color:#aaa;font-size:0.85em;text-decoration:none;margin-right:14px;
          display:inline-flex;align-items:center;min-height:44px}
    nav a:hover{color:#e94560}
    .section{margin-top:22px;font-size:0.78em;color:#666;text-transform:uppercase;
             letter-spacing:.08em;border-bottom:1px solid #0f3460;padding-bottom:4px;margin-bottom:12px}
    .box{background:#16213e;padding:20px;border-radius:10px;margin-bottom:16px;
         box-shadow:0 4px 12px rgba(0,0,0,.4)}
    label{display:block;font-size:0.82em;color:#aaa;margin-bottom:3px;margin-top:10px}
    label:first-child{margin-top:0}
    input[type=text],input[type=password],input[type=number],select{
      width:100%;padding:9px;border:1px solid #0f3460;border-radius:5px;
      background:#0f3460;color:#eee;font-size:14px}
    .row2{display:grid;grid-template-columns:1fr 1fr;gap:12px}
    button{background:#e94560;color:#fff;border:none;padding:11px 24px;
           border-radius:5px;font-size:15px;cursor:pointer;margin-top:14px;width:100%;
           min-height:44px}
    button:hover{opacity:0.9}
    button:disabled{opacity:.45;cursor:progress}
    .btn-secondary{background:#0f3460;border:1px solid #e94560}
    /* UX-1: the message used to sit at the very bottom of a long scrolling
       page, so a save result was usually off-screen on a phone. Pin it. */
    .msg{text-align:center;padding:14px;border-radius:8px;display:none;
         position:fixed;left:12px;right:12px;
         bottom:calc(14px + env(safe-area-inset-bottom));z-index:1000;
         max-width:576px;margin:auto;font-weight:bold;
         box-shadow:0 6px 20px rgba(0,0,0,.55)}
    .msg.ok{display:block;background:#0a3d2a;color:#7ef0b0;border:1px solid #4ade80}
    .msg.err{display:block;background:#4a0d0d;color:#ffd4d4;border:1px solid #f87171}
    /* D4: retry affordance inside the load-failure message. */
    .msg-reload{width:auto;margin-top:0;margin-left:6px;padding:8px 16px;min-height:36px;
                font-size:0.85em;background:#0f3460;border:1px solid #f87171;color:#ffd4d4}
    .hint{font-size:0.75em;color:#666;margin-top:3px}
    /* UX-9: per-field show/hide toggle for password inputs. */
    .pw-wrap{display:flex;gap:8px;align-items:stretch}
    .pw-wrap input{flex:1}
    .pw-toggle{background:#0f3460;color:#aaa;border:1px solid #0f3460;border-radius:5px;
               font-size:0.78em;cursor:pointer;margin-top:0;width:auto;min-width:64px;
               min-height:44px;padding:0 10px;flex:0 0 auto}
    .pw-toggle:hover{color:#eee;border-color:#e94560}
    /* In-page confirm (UX-3) — same component as the dashboard. */
    .cfm-overlay{position:fixed;top:0;left:0;right:0;bottom:0;
                 background:rgba(0,0,0,.72);z-index:1100;
                 display:flex;align-items:center;justify-content:center;padding:20px}
    .cfm-box{background:#16213e;border:1px solid #0f3460;border-radius:10px;
             padding:20px;max-width:380px;width:100%;
             box-shadow:0 10px 30px rgba(0,0,0,.6)}
    .cfm-msg{font-size:1em;line-height:1.5;color:#eee;margin-bottom:18px}
    .cfm-row{display:flex;gap:10px}
    .cfm-btn{flex:1;min-height:48px;border:none;border-radius:6px;font-size:1em;
             font-weight:bold;cursor:pointer;padding:12px;margin-top:0;width:auto}
    .cfm-no {background:#0f3460;color:#eee;border:1px solid #444}
    .cfm-yes{background:#e94560;color:#fff}
  </style>
</head>
<body>
  <h1>&#9881; Settings</h1>
  <nav><a href="/">&#8592; Dashboard</a></nav>

  <div class="section">WiFi Network</div>
  <div class="box">
    <label for="wifiSSID">SSID</label>
    <input type="text" id="wifiSSID" name="wifi_ssid" placeholder="Network name"
           autocomplete="off" autocapitalize="none" autocorrect="off" spellcheck="false">
    <label for="wifiPass">Password</label>
    <!-- UX-9: these are the *device's* credentials for someone else's network,
         never the user's own login, so autocomplete is off and the field is
         marked new-password to stop a manager offering the site password. -->
    <div class="pw-wrap">
      <input type="password" id="wifiPass" name="wifi_pass" placeholder="Password"
             autocomplete="new-password" autocapitalize="none" autocorrect="off"
             spellcheck="false">
      <button type="button" class="pw-toggle" data-pw="wifiPass"
              onclick="togglePw('wifiPass', this)" aria-label="Show password">Show</button>
    </div>
    <button type="button" onclick="saveWifi()">Save WiFi &amp; Restart</button>
    <div class="hint">Connects to this network on boot. Falls back to AP mode if unavailable.</div>
  </div>

  <div class="section">Access Point (Fallback)</div>
  <div class="box">
    <label for="apSSID">AP Name</label>
    <input type="text" id="apSSID" name="ap_ssid" placeholder="Supercharger"
           autocomplete="off" autocapitalize="none" autocorrect="off" spellcheck="false">
    <label for="apPass">AP Password</label>
    <div class="pw-wrap">
      <input type="password" id="apPass" name="ap_pass" placeholder="Min 8 characters"
             autocomplete="new-password" autocapitalize="none" autocorrect="off"
             spellcheck="false">
      <button type="button" class="pw-toggle" data-pw="apPass"
              onclick="togglePw('apPass', this)" aria-label="Show password">Show</button>
    </div>
    <label style="display:flex;align-items:center;gap:8px;cursor:pointer;margin-top:14px">
      <input type="checkbox" id="apStaticEn" name="ap_static_en" onchange="onApStaticToggle()">
      <span>Use a fixed AP IP address</span>
    </label>
    <div id="apStaticWrap" style="display:none;margin-top:8px">
      <label for="apStaticIp">AP IP Address</label>
      <input type="text" id="apStaticIp" name="ap_ip" placeholder="192.168.1.50"
             inputmode="decimal" autocomplete="off" spellcheck="false">
      <label for="apStaticGw">Gateway</label>
      <input type="text" id="apStaticGw" name="ap_gw" placeholder="192.168.1.1"
             inputmode="decimal" autocomplete="off" spellcheck="false">
      <label for="apStaticSn">Subnet Mask</label>
      <input type="text" id="apStaticSn" name="ap_sn" placeholder="255.255.255.0"
             inputmode="decimal" autocomplete="off" spellcheck="false">
      <div class="hint">AP mode normally serves the dashboard at 192.168.4.1.
        Set a fixed IP that mirrors your home network so one iOS home-screen
        shortcut works both at home and on the road. Applies on next reboot.</div>
    </div>
    <button type="button" onclick="saveAP()">Save AP Settings</button>
    <div class="hint">Used when WiFi is unavailable. Name and password for the local hotspot.</div>
  </div>

  <div class="section">MQTT Broker</div>
  <div id="mqttApBanner" class="box" style="display:none;text-align:center;color:#888">
    <span style="font-size:1.1em">&#128268;</span>
    MQTT settings are not available when running in AP mode.<br>
    <span style="font-size:0.82em;color:#666">Configure WiFi above, then MQTT will be available after connecting.</span>
  </div>
  <div id="mqttForm" class="box">
    <div class="row2">
      <div><label for="mqttHost">Host / IP</label>
        <input type="text" id="mqttHost" name="mqtt_host" placeholder="192.168.1.100"
               autocomplete="off" autocapitalize="none" autocorrect="off" spellcheck="false"></div>
      <div><label for="mqttPort">Port</label>
        <input type="number" id="mqttPort" name="mqtt_port" placeholder="1883"
               inputmode="numeric" autocomplete="off"></div>
    </div>
    <div class="row2">
      <div><label for="mqttUser">Username</label>
        <input type="text" id="mqttUser" name="mqtt_user" placeholder="(optional)"
               autocomplete="off" autocapitalize="none" autocorrect="off" spellcheck="false"></div>
      <div><label for="mqttPass">Password</label>
        <div class="pw-wrap">
          <input type="password" id="mqttPass" name="mqtt_pass" placeholder="(optional)"
                 autocomplete="new-password" autocapitalize="none" autocorrect="off"
                 spellcheck="false">
          <button type="button" class="pw-toggle" data-pw="mqttPass"
                  onclick="togglePw('mqttPass', this)" aria-label="Show password">Show</button>
        </div></div>
    </div>
    <div style="margin-top:8px">
      <label style="display:flex;align-items:center;gap:8px;cursor:pointer">
        <input type="checkbox" id="mqttTls" name="mqtt_tls" onchange="onTlsToggle()">
        <span>Use SSL/TLS (encrypted broker connection)</span>
      </label>
      <div class="hint">Ticking this auto-switches the port to 8883.
        You'll also need to upload the broker's CA certificate (PEM) below;
        without a cert the device refuses to connect rather than disable
        certificate verification.</div>
    </div>
    <div id="mqttCaWrap" style="display:none;margin-top:10px">
      <label for="mqttCa">Broker CA Certificate (PEM)</label>
      <textarea id="mqttCa" name="mqtt_ca" rows="6" spellcheck="false"
        style="width:100%;font-family:monospace;font-size:11px;background:#0f0f1e;
               color:#eee;border:1px solid #0f3460;border-radius:5px;padding:8px"
        placeholder="-----BEGIN CERTIFICATE-----&#10;...&#10;-----END CERTIFICATE-----"></textarea>
      <div class="hint" id="mqttCaStatus">No certificate uploaded yet.</div>
    </div>
    <button type="button" onclick="saveMQTT()">Save MQTT &amp; Reconnect</button>
  </div>

  <div class="section">Charger Hardware</div>
  <div class="box">
    <label for="ccCount">Number of Chargers (1–4)</label>
    <input type="number" id="ccCount" name="charger_count" min="1" max="4" value="3"
           inputmode="numeric" autocomplete="off">
    <!-- UX-12: same wording as the dashboard hint. -->
    <div class="hint">Sets the preset table and slider range only.
      Does not change what the chargers physically do.</div>
    <button type="button" onclick="saveChargerCount()">Save Charger Count</button>
  </div>

  <div class="section">Charging Behaviour</div>
  <div class="box">
    <label for="rampRate">Ramp Rate (W/s, 10–500)</label>
    <input type="number" id="rampRate" name="ramp_rate_wps" min="10" max="500" value="50"
           inputmode="numeric" autocomplete="off">
    <div class="hint">Power increases/decreases by this many watts per second when ramping. Lower = gentler ramp. Default: 100 W/s.</div>
    <button type="button" onclick="saveRampRate()">Save Ramp Rate</button>
  </div>

  <div class="section">Boot Defaults &mdash; AP / Road Mode</div>
  <div class="box">
    <div class="hint" style="margin-top:0;margin-bottom:10px">Applied when the device boots without a home WiFi connection (AP mode or no saved network).</div>
    <label style="display:flex;align-items:center;gap:8px;cursor:pointer;margin-top:0">
      <input type="checkbox" id="defChgEnabled" name="charging_enabled_default">
      <span>Start with charging enabled</span>
    </label>
    <label style="margin-top:14px" for="defPowerPreset">Default charge speed preset</label>
    <select id="defPowerPreset" name="power_preset_default">
      <option value="0">Preset 1 &mdash; 0.5 / 1 / 1.5 / 2 kW (1&ndash;4 chargers)</option>
      <option value="1">Preset 2 &mdash; 1 / 2 / 3 / 4 kW (1&ndash;4 chargers)</option>
      <option value="2">Preset 3 &mdash; 1.65 / 3.3 / 5 / 6.6 kW (1&ndash;4 chargers)</option>
      <option value="3">Preset 4 &mdash; 2.2 / 4.4 / 6.6 / 8.8 kW (1&ndash;4 chargers)</option>
      <option value="4">Preset 5 &mdash; 3.3 / 6.6 / 9.9 / 13.2 kW (1&ndash;4 chargers)</option>
    </select>
    <div class="hint">Actual watts depend on charger count. Preset 2 = 2 kW with 2 chargers.</div>
    <label style="margin-top:14px" for="defTargetVolt">Default target voltage</label>
    <select id="defTargetVolt" name="target_volt_default">
      <option value="1060">70% &mdash; 106.0 V</option>
      <option value="1100">80% &mdash; 110.0 V</option>
      <option value="1132">90% &mdash; 113.2 V</option>
      <option value="1164">100% &mdash; 116.4 V</option>
    </select>
    <div class="hint">Applied at boot. Can be changed per session from the dashboard.</div>
    <button type="button" onclick="saveBootDefaults()">Save AP / Road Defaults</button>
  </div>

  <div class="section">Boot Defaults &mdash; Home WiFi</div>
  <div class="box">
    <div class="hint" style="margin-top:0;margin-bottom:10px">Applied once when the device first connects to your home WiFi network after boot. Overrides the AP / Road defaults above.</div>
    <label style="display:flex;align-items:center;gap:8px;cursor:pointer;margin-top:0">
      <input type="checkbox" id="homeDefChgEnabled" name="home_charging_enabled_default">
      <span>Start with charging enabled</span>
    </label>
    <label style="margin-top:14px" for="homeDefPowerPreset">Default charge speed preset</label>
    <select id="homeDefPowerPreset" name="home_power_preset_default">
      <option value="0">Preset 1 &mdash; 0.5 / 1 / 1.5 / 2 kW (1&ndash;4 chargers)</option>
      <option value="1">Preset 2 &mdash; 1 / 2 / 3 / 4 kW (1&ndash;4 chargers)</option>
      <option value="2">Preset 3 &mdash; 1.65 / 3.3 / 5 / 6.6 kW (1&ndash;4 chargers)</option>
      <option value="3">Preset 4 &mdash; 2.2 / 4.4 / 6.6 / 8.8 kW (1&ndash;4 chargers)</option>
      <option value="4">Preset 5 &mdash; 3.3 / 6.6 / 9.9 / 13.2 kW (1&ndash;4 chargers)</option>
    </select>
    <div class="hint">Actual watts depend on charger count. Preset 2 = 2 kW with 2 chargers.</div>
    <label style="margin-top:14px" for="homeDefTargetVolt">Default target voltage</label>
    <select id="homeDefTargetVolt" name="home_target_volt_default">
      <option value="1060">70% &mdash; 106.0 V</option>
      <option value="1100">80% &mdash; 110.0 V</option>
      <option value="1132">90% &mdash; 113.2 V</option>
      <option value="1164">100% &mdash; 116.4 V</option>
    </select>
    <div class="hint">Applied at boot. Can be changed per session from the dashboard.</div>
    <button type="button" onclick="saveHomeDefaults()">Save Home WiFi Defaults</button>
  </div>

  <div class="section">HTTPS / TLS</div>
  <div class="box">
    <div class="hint" style="margin-top:0;margin-bottom:10px">
      Upload a certificate and private key (PEM format) to enable HTTPS on port 443.
      When enabled, port 80 redirects to HTTPS. Requires restart to take effect.
      Use a self-signed cert for local LAN use; a publicly-trusted cert if exposed externally.
    </div>
    <div id="tlsCertStatus" class="hint" style="margin-bottom:10px">No certificate uploaded yet.</div>
    <label for="tlsCert">Certificate (PEM)</label>
    <textarea id="tlsCert" name="tls_cert" rows="5" spellcheck="false"
      style="width:100%;font-family:monospace;font-size:11px;background:#0f0f1e;
             color:#eee;border:1px solid #0f3460;border-radius:5px;padding:8px"
      placeholder="-----BEGIN CERTIFICATE-----&#10;...&#10;-----END CERTIFICATE-----"></textarea>
    <label style="margin-top:10px" for="tlsKey">Private Key (PEM)</label>
    <textarea id="tlsKey" name="tls_key" rows="5" spellcheck="false"
      style="width:100%;font-family:monospace;font-size:11px;background:#0f0f1e;
             color:#eee;border:1px solid #0f3460;border-radius:5px;padding:8px"
      placeholder="-----BEGIN RSA PRIVATE KEY-----&#10;...&#10;-----END RSA PRIVATE KEY-----"></textarea>
    <button type="button" onclick="saveTlsCert()" style="margin-top:10px">Upload Cert &amp; Key</button>
    <div style="margin-top:14px">
      <label style="display:flex;align-items:center;gap:8px;cursor:pointer">
        <input type="checkbox" id="httpsEnabled" name="https_enabled" onchange="saveHttpsToggle()">
        <span>Enable HTTPS (redirect port 80 → 443)</span>
      </label>
      <div class="hint">A certificate must be uploaded before enabling. Reboot required after changing.</div>
    </div>
  </div>

  <div id="msgBox" class="msg" role="status" aria-live="polite"></div>

  <script>
    // /log and /update live only on port 80 — see the note in the dashboard.
    if (location.protocol === 'https:') {
      document.querySelectorAll('nav a[href="/log"], nav a[href="/update"]').forEach(function(a){
        a.href = 'http://' + location.hostname + a.getAttribute('href');
      });
    }

    var msgTimer = null;
    function showMsg(text, ok) {
      var m = document.getElementById('msgBox');
      m.textContent = text;
      m.className = 'msg ' + (ok ? 'ok' : 'err');
      if (msgTimer) clearTimeout(msgTimer);
      msgTimer = setTimeout(function(){ m.className = 'msg'; }, 4000);
    }

    // ---- In-page confirm (UX-3) -----------------------------------------
    // Same component as the dashboard: window.confirm on iOS prefixes the
    // hostname and can't be styled, and in a home-screen web app it looks
    // like a browser error rather than part of the page.
    function uiConfirm(msg, onYes, onNo) {
      var ov  = document.createElement('div'); ov.className  = 'cfm-overlay';
      var box = document.createElement('div'); box.className = 'cfm-box';
      var p   = document.createElement('div'); p.className   = 'cfm-msg';
      p.textContent = msg;
      var row = document.createElement('div'); row.className = 'cfm-row';
      var no  = document.createElement('button');
      no.type = 'button'; no.className = 'cfm-btn cfm-no';  no.textContent  = 'Cancel';
      var yes = document.createElement('button');
      yes.type = 'button'; yes.className = 'cfm-btn cfm-yes'; yes.textContent = 'Confirm';
      var closed = false;
      function close(cancelled) {
        if (closed) return; closed = true;
        if (ov.parentNode) ov.parentNode.removeChild(ov);
        document.removeEventListener('keydown', onKey);
        if (cancelled && onNo) onNo();
      }
      function onKey(e){ if (e.key === 'Escape') close(true); }
      no.onclick  = function(){ close(true); };
      yes.onclick = function(){ close(false); if (onYes) onYes(); };
      ov.onclick  = function(e){ if (e.target === ov) close(true); };
      document.addEventListener('keydown', onKey);
      row.appendChild(no); row.appendChild(yes);
      box.appendChild(p);  box.appendChild(row);
      ov.appendChild(box);
      document.body.appendChild(ov);
      yes.focus();
    }

    // ---- Password show/hide (UX-9) ---------------------------------------
    function togglePw(id, btn) {
      var el = document.getElementById(id);
      if (!el) return;
      var show = (el.type === 'password');
      el.type = show ? 'text' : 'password';
      btn.textContent = show ? 'Hide' : 'Show';
      btn.setAttribute('aria-label', show ? 'Hide password' : 'Show password');
    }

    // ---- Unsaved-changes guard (UX-15) -----------------------------------
    // dirty is set by any input/change event AFTER the initial load has
    // populated the form, and cleared by every successful save.
    var formLoaded = false;
    var dirty      = false;
    function markDirty(e){
      if (!formLoaded) return;
      // The HTTPS checkbox saves itself on change, so it is never an unsaved
      // edit — counting it would leave a stale warning behind a cancelled
      // confirm.
      if (e && e.target && e.target.id === 'httpsEnabled') return;
      dirty = true;
    }
    function clearDirty(){ dirty = false; }
    document.addEventListener('input',  markDirty, true);
    document.addEventListener('change', markDirty, true);
    window.addEventListener('beforeunload', function(e){
      if (!dirty) return;
      e.preventDefault();
      e.returnValue = '';   // required by Chrome/Safari to show the prompt
      return '';
    });

    // D4: a failed load leaves every field holding its HTML default, which is
    // NOT what the device is running. Leaving the Save buttons live would let
    // one tap write preset 1 / 1060 dV / 3 chargers / 50 W/s that the user
    // never chose. So on any load failure: say what went wrong, keep it on
    // screen (no 4 s auto-hide), offer a Reload, disable every Save, and leave
    // formLoaded false so the dirty guard stays quiet too.
    function loadFailed(text) {
      formLoaded = false;
      clearDirty();
      setSaveDisabled(true);
      if (msgTimer) { clearTimeout(msgTimer); msgTimer = null; }
      var m = document.getElementById('msgBox');
      m.textContent = '';
      m.className = 'msg err';
      var span = document.createElement('span');
      span.textContent = text + ' ';
      var btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'msg-reload';
      btn.textContent = 'Reload';
      btn.onclick = function(){ location.reload(); };
      m.appendChild(span);
      m.appendChild(btn);
    }

    // Load current settings on page load.
    // D5: read the body BEFORE judging the status, so the server error text
    // survives. The builder answers {"ok":false,...} with HTTP 500 when the
    // JSON overflowed its buffer; populating the form from that would blank
    // every field, so it takes the same disabled-form path as a transport
    // failure rather than being a dead branch behind an earlier throw.
    fetch('/api/settings', {credentials:'same-origin'}).then(function(r){
      return r.text().then(function(txt){
        if (r.status === 401) { window.location.href = '/login'; throw new Error('__auth__'); }
        var d = null;
        try { d = JSON.parse(txt); } catch (e) {}
        if (!d)               throw new Error(txt || ('HTTP ' + r.status));
        if (d.ok === false)   throw new Error(d.error || 'Settings unavailable');
        if (!r.ok)            throw new Error(d.error || ('HTTP ' + r.status));
        return d;
      });
    }).then(function(d){
      document.getElementById('wifiSSID').value = d.wifi_ssid || '';
      document.getElementById('apSSID').value   = d.ap_ssid   || '';
      if (d.ap_pass_set) {
        document.getElementById('apPass').placeholder = '(unchanged — leave blank to keep)';
      }
      document.getElementById('apStaticEn').checked = !!d.ap_static_en;
      document.getElementById('apStaticIp').value   = d.ap_ip || '';
      document.getElementById('apStaticGw').value   = d.ap_gw || '';
      document.getElementById('apStaticSn').value   = d.ap_sn || '';
      onApStaticToggle();
      document.getElementById('ccCount').value    = d.charger_count || 3;
      document.getElementById('rampRate').value   = d.ramp_rate_wps || 50;
      document.getElementById('defChgEnabled').checked     = !!d.charging_enabled_default;
      document.getElementById('defPowerPreset').value      = (d.power_preset_default !== undefined) ? d.power_preset_default : 1;
      document.getElementById('defTargetVolt').value       = (d.target_volt_default   !== undefined) ? d.target_volt_default  : 1100;
      document.getElementById('homeDefChgEnabled').checked = !!d.home_charging_enabled_default;
      document.getElementById('homeDefPowerPreset').value  = (d.home_power_preset_default !== undefined) ? d.home_power_preset_default : 1;
      document.getElementById('homeDefTargetVolt').value   = (d.home_target_volt_default   !== undefined) ? d.home_target_volt_default  : 1100;
      // Toggle MQTT section based on AP mode
      if (d.ap_mode) {
        document.getElementById('mqttForm').style.display = 'none';
        document.getElementById('mqttApBanner').style.display = 'block';
      } else {
        document.getElementById('mqttForm').style.display = 'block';
        document.getElementById('mqttApBanner').style.display = 'none';
        document.getElementById('mqttHost').value  = d.mqtt_host || '';
        document.getElementById('mqttPort').value  = d.mqtt_port || 1883;
        document.getElementById('mqttUser').value  = d.mqtt_user || '';
        document.getElementById('mqttTls').checked = !!d.mqtt_tls;
        document.getElementById('mqttCaWrap').style.display = d.mqtt_tls ? 'block' : 'none';
        // Cert blob itself is never returned (potentially large + privacy);
        // the API just reports whether one is currently stored.
        var s = document.getElementById('mqttCaStatus');
        if (s) s.textContent = d.mqtt_ca_set
          ? ('Certificate stored on device (' + d.mqtt_ca_bytes + ' bytes). Paste a new one to replace.')
          : 'No certificate uploaded yet.';
      }
      // HTTPS section
      document.getElementById('httpsEnabled').checked = !!d.https_enabled;
      var ts = document.getElementById('tlsCertStatus');
      if (ts) ts.textContent = d.https_cert_set
        ? 'Certificate stored on device. Paste new PEM to replace.'
        : 'No certificate uploaded yet.';
      // Passwords are never returned from the API for security
      // UX-15: only start watching for edits once the form holds real values,
      // so the population above doesn't itself count as an unsaved change.
      formLoaded = true;
      clearDirty();
    }).catch(function(e){
      if (e && e.message === '__auth__') return;   // navigating to /login
      loadFailed('Could not load settings — ' + ((e && e.message) || 'connection error') + '.');
    });

    // Auto-switch port when SSL is toggled. Only overwrites the port field
    // if it currently holds the OTHER protocol's default — we don't clobber
    // a user-customised port (e.g. 8884 for an EMQX cluster).
    function onTlsToggle() {
      var on = document.getElementById('mqttTls').checked;
      document.getElementById('mqttCaWrap').style.display = on ? 'block' : 'none';
      var portEl = document.getElementById('mqttPort');
      var cur = parseInt(portEl.value);
      if (on  && (cur === 1883 || isNaN(cur))) portEl.value = 8883;
      if (!on && (cur === 8883 || isNaN(cur))) portEl.value = 1883;
    }

    // ---- Shared POST helper (UX-1 / UX-4) --------------------------------
    // Disables every button on the page while a save is in flight, checks the
    // status BEFORE parsing the body (a 401 answers text/plain, which
    // r.json() would reject as a generic connection error and so hide the
    // fact that the session had simply expired), and surfaces the error text
    // the server itself sent.
    var saveBusy = false;
    var savePrev = null;
    function setSaveDisabled(on) {
      if (on) {
        var btns = document.querySelectorAll('button');
        savePrev = [];
        for (var i = 0; i < btns.length; i++) {
          var cn = btns[i].className || '';
          if (cn.indexOf('cfm-btn') >= 0) continue;      // confirm dialog
          if (cn.indexOf('msg-reload') >= 0) continue;   // D4 retry affordance
          if (cn.indexOf('pw-toggle') >= 0) continue;    // show/hide is not a save
          savePrev.push([btns[i], btns[i].disabled]);
          btns[i].disabled = true;
        }
      } else if (savePrev) {
        for (var j = 0; j < savePrev.length; j++) savePrev[j][0].disabled = savePrev[j][1];
        savePrev = null;
      }
    }

    // url/data → promise resolving to the parsed body. Rejects with an Error
    // whose message is fit to show the user. A message of __auth__ means we
    // are already navigating to the login page, so callers stay silent on it.
    function postJson(url, data) {
      if (saveBusy) return Promise.reject(new Error('__busy__'));
      saveBusy = true;
      setSaveDisabled(true);
      return fetch(url, {
        method:'POST',
        credentials:'same-origin',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify(data)
      }).then(function(r){
        return r.text().then(function(txt){
          if (r.status === 401) { window.location.href = '/login'; throw new Error('__auth__'); }
          var d = null;
          try { d = JSON.parse(txt); } catch (e) {}
          if (r.status === 503) throw new Error('Controller busy — try again');
          if (!r.ok) throw new Error((d && d.error) || txt || ('HTTP ' + r.status));
          if (!d || d.ok !== true) throw new Error((d && d.error) || 'Save failed');
          return d;
        });
      }).then(function(d){
        saveBusy = false; setSaveDisabled(false);
        return d;
      }, function(e){
        if (!e || e.message !== '__busy__') { saveBusy = false; setSaveDisabled(false); }
        throw e;
      });
    }

    function postSettings(data, successMsg, restart) {
      postJson('/api/settings', data).then(function(){
        showMsg(successMsg, true);
        clearDirty();                       // UX-15: saved, so no longer dirty
        if (restart) {
          setSaveDisabled(true);            // nothing useful to press while it reboots
          setTimeout(function(){ location.href='/'; }, 3000);
        }
      }).catch(function(e){
        if (e && (e.message === '__auth__' || e.message === '__busy__')) return;
        showMsg((e && e.message) || 'Connection error', false);
      });
    }

    // UX-3: this POST reboots the controller, which stops any charge in
    // progress — confirm before it fires.
    function saveWifi() {
      var ssid = document.getElementById('wifiSSID').value.trim();
      var pass = document.getElementById('wifiPass').value;
      if (!ssid) { showMsg('SSID cannot be empty', false); return; }
      uiConfirm('Save and restart the controller? Charging will stop.', function(){
        postSettings({wifi_ssid: ssid, wifi_pass: pass}, 'WiFi saved — restarting...', true);
      });
    }
    function onApStaticToggle() {
      var on = document.getElementById('apStaticEn').checked;
      document.getElementById('apStaticWrap').style.display = on ? 'block' : 'none';
    }
    function saveAP() {
      var ssid = document.getElementById('apSSID').value.trim();
      var pass = document.getElementById('apPass').value;
      if (!ssid) { showMsg('AP name cannot be empty', false); return; }
      if (pass.length > 0 && pass.length < 8) { showMsg('AP password must be 8+ characters', false); return; }
      var data = {ap_ssid: ssid};
      if (pass.length > 0) data.ap_pass = pass; // only send if user entered a new password
      // Fixed AP IP. Always send the three fields so toggling off then on
      // again keeps the typed values; only validate them when enabling.
      var apEn = document.getElementById('apStaticEn').checked;
      var aip  = document.getElementById('apStaticIp').value.trim();
      var agw  = document.getElementById('apStaticGw').value.trim();
      var asn  = document.getElementById('apStaticSn').value.trim();
      if (apEn) {
        var ipRe = /^(\d{1,3}\.){3}\d{1,3}$/;
        if (!ipRe.test(aip)) { showMsg('AP IP address looks invalid', false); return; }
        if (!ipRe.test(agw)) { showMsg('Gateway looks invalid', false); return; }
        if (!ipRe.test(asn)) { showMsg('Subnet mask looks invalid', false); return; }
      }
      data.ap_static_en = apEn;
      data.ap_ip = aip; data.ap_gw = agw; data.ap_sn = asn;
      postSettings(data, 'AP settings saved', false);
    }
    function saveMQTT() {
      var tls = document.getElementById('mqttTls').checked;
      var ca  = document.getElementById('mqttCa').value.trim();
      var data = {
        mqtt_host: document.getElementById('mqttHost').value.trim(),
        mqtt_port: parseInt(document.getElementById('mqttPort').value) || (tls ? 8883 : 1883),
        mqtt_user: document.getElementById('mqttUser').value.trim(),
        mqtt_pass: document.getElementById('mqttPass').value,
        mqtt_tls:  tls
      };
      // Sanity-check the PEM client-side before sending so the user gets
      // immediate feedback on a paste error. Server re-validates anyway.
      if (ca.length > 0) {
        if (ca.indexOf('-----BEGIN CERTIFICATE-----') < 0 ||
            ca.indexOf('-----END CERTIFICATE-----')   < 0) {
          showMsg('CA cert must be PEM with BEGIN/END CERTIFICATE markers', false);
          return;
        }
        data.mqtt_ca = ca;
      }
      postSettings(data, 'MQTT saved — reconnecting...', false);
    }
    // UX-7: parseInt('') is NaN, and NaN fails every comparison — so the old
    // "cc < 1 || cc > 4" check passed an empty field straight through to the
    // POST. Test for an actual integer instead.
    function saveChargerCount() {
      var cc = parseInt(document.getElementById('ccCount').value, 10);
      if (!Number.isInteger(cc) || cc < 1 || cc > 4) { showMsg('Enter 1-4', false); return; }
      postSettings({charger_count: cc}, 'Charger count saved', false);
    }
    function saveRampRate() {
      var rr = parseInt(document.getElementById('rampRate').value, 10);
      if (!Number.isInteger(rr) || rr < 10 || rr > 500) {
        showMsg('Ramp rate must be 10–500 W/s', false); return;
      }
      // Ramp rate lives on /api/control, not /api/settings — same 401 and
      // error handling via postJson() (UX-4).
      postJson('/api/control', {ramp_rate_wps: rr}).then(function(){
        showMsg('Ramp rate saved', true);
        clearDirty();
      }).catch(function(e){
        if (e && (e.message === '__auth__' || e.message === '__busy__')) return;
        showMsg((e && e.message) || 'Connection error', false);
      });
    }
    function saveBootDefaults() {
      var ce  = document.getElementById('defChgEnabled').checked;
      var pi  = parseInt(document.getElementById('defPowerPreset').value);
      var tvd = parseInt(document.getElementById('defTargetVolt').value);
      if (isNaN(pi)  || pi  < 0 || pi  > 4)       { showMsg('Invalid preset index',   false); return; }
      if (isNaN(tvd) || tvd < 1060 || tvd > 1164)  { showMsg('Invalid target voltage', false); return; }
      postSettings({
        charging_enabled_default: ce,
        power_preset_default:     pi,
        target_volt_default:      tvd
      }, 'AP / Road defaults saved', false);
    }
    function saveHomeDefaults() {
      var ce  = document.getElementById('homeDefChgEnabled').checked;
      var pi  = parseInt(document.getElementById('homeDefPowerPreset').value);
      var tvd = parseInt(document.getElementById('homeDefTargetVolt').value);
      if (isNaN(pi)  || pi  < 0 || pi  > 4)       { showMsg('Invalid preset index',   false); return; }
      if (isNaN(tvd) || tvd < 1060 || tvd > 1164)  { showMsg('Invalid target voltage', false); return; }
      postSettings({
        home_charging_enabled_default: ce,
        home_power_preset_default:     pi,
        home_target_volt_default:      tvd
      }, 'Home WiFi defaults saved', false);
    }
    function saveTlsCert() {
      var cert = document.getElementById('tlsCert').value.trim();
      var key  = document.getElementById('tlsKey').value.trim();
      if (!cert || !key) { showMsg('Both certificate and private key are required', false); return; }
      if (cert.indexOf('-----BEGIN CERTIFICATE-----') < 0) {
        showMsg('Certificate must be PEM with BEGIN CERTIFICATE marker', false); return;
      }
      postJson('/api/tls', {cert: cert, key: key}).then(function(){
        showMsg('Certificate uploaded. Enable HTTPS below and reboot.', true);
        document.getElementById('tlsCertStatus').textContent =
          'Certificate stored on device. Paste new PEM to replace.';
        document.getElementById('tlsCert').value = '';
        document.getElementById('tlsKey').value  = '';
        clearDirty();
      }).catch(function(e){
        if (e && (e.message === '__auth__' || e.message === '__busy__')) return;
        showMsg((e && e.message) || 'Connection error', false);
      });
    }
    // UX-3: flipping this checkbox changes boot-time state and only takes
    // effect after a restart — which stops charging and moves the dashboard
    // to a different scheme/port. Confirm first; put the checkbox back if
    // the user cancels. (The POST itself does NOT reboot — the firmware
    // answers "Saved. Reboot the controller for this to take effect." — so
    // the wording says restart-required rather than restarting-now.)
    function saveHttpsToggle() {
      var box = document.getElementById('httpsEnabled');
      var en  = box.checked;
      uiConfirm((en ? 'Enable' : 'Disable') + ' HTTPS? It takes effect when the ' +
                'controller is restarted, and restarting stops charging.', function(){
        postJson('/api/tls', {enabled: en}).then(function(){
          showMsg('HTTPS ' + (en ? 'enabled' : 'disabled') + ' — reboot to apply', true);
          clearDirty();
        }).catch(function(e){
          if (e && e.message === '__auth__') return;
          box.checked = !en;                 // failed → revert the checkbox
          if (e && e.message === '__busy__') return;
          showMsg((e && e.message) || 'Connection error', false);
        });
      }, function(){
        box.checked = !en;                   // cancelled → revert the checkbox
      });
    }
  </script>
</body>
</html>
)rawliteral";

// ---------------------------------------------------------------------------
// D11 — what the Arduino preprocessor can and cannot survive inside these
// R"rawliteral(...)" page strings. This note lives in C comment space, outside
// every literal, because that is the one place it cannot itself trip the thing
// it describes.
//
// The ctags 5.8 that ships with the Arduino toolchain predates C++11 raw
// string literals. It has no concept of R"delim(...)delim", so it walks the
// page bodies as ordinary C source and tracks quoting and comments as it goes.
// The auto-prototype pass is driven by its output, so when its state desyncs it
// emits no prototypes at all — and the build fails with a cascade of bogus
// "'foo' was not declared in this scope" errors pointing at perfectly good
// functions far below. That error signature is the tell: suspect the page
// literals, not the code the compiler is pointing at.
//
// Two constructs have actually been observed to desync it:
//   * an unpaired double quote — easy to introduce in HTML attribute syntax,
//     where single and double quotes get mixed freely;
//   * a data: URL, whose "//" ctags may take for the start of a comment.
// The favicon note that used to be repeated inside all four page literals was
// describing this second case.
//
// What is NOT a problem, despite the caution the old note invited: apostrophes
// in ordinary prose. Fourteen such lines are in these literals today and the
// sketch builds clean — ctags only cares about the double quote. Write "don't"
// and "the pack's voltage" normally.
//
// Rule of thumb when editing the pages: keep double quotes balanced on every
// line, prefer single quotes for HTML attributes, and do not inline a data: URL
// (add a real route and link to it instead).
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Dashboard — served at / when connected to a network
// Polls /api/status every 2 s and updates fields in-place.
// The page itself is fully static — no server-side template substitution.
// ---------------------------------------------------------------------------

const char HTML_DASHBOARD[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
  <meta name="apple-mobile-web-app-capable" content="yes">
  <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
  <meta name="theme-color" content="#1a1a2e">
  <title>Supercharger</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;padding:16px;
         padding-top:calc(16px + env(safe-area-inset-top));
         padding-bottom:calc(16px + env(safe-area-inset-bottom));
         padding-left:calc(16px + env(safe-area-inset-left));
         padding-right:calc(16px + env(safe-area-inset-right))}
    h1{color:#e94560;font-size:1.4em;margin-bottom:6px}
    /* UX-11: nav links are a 44 px touch target, and wrap rather than
       overflowing a 375 px-wide phone screen. */
    nav{display:flex;flex-wrap:wrap;row-gap:2px}
    nav a{color:#aaa;font-size:0.85em;text-decoration:none;margin-right:14px;
          display:inline-flex;align-items:center;min-height:44px}
    nav a:hover{color:#e94560}
    .section{margin-top:18px;font-size:0.78em;color:#666;text-transform:uppercase;
             letter-spacing:.08em;border-bottom:1px solid #0f3460;padding-bottom:4px;
             margin-bottom:10px}
    .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:10px}
    .card{background:#16213e;border-radius:8px;padding:14px;
          box-shadow:0 2px 8px rgba(0,0,0,.35)}
    .card .label{font-size:0.70em;color:#888;text-transform:uppercase;
                 letter-spacing:.05em;margin-bottom:4px}
    .card .val{font-size:1.45em;font-weight:bold;color:#e94560}
    .card .unit{font-size:0.72em;color:#aaa;margin-left:2px}
    .badge{display:inline-block;padding:2px 9px;border-radius:10px;
           font-size:0.72em;font-weight:bold;vertical-align:middle;margin-left:8px}
    .ok   {background:#1a4a1a;color:#4caf50}
    .stale{background:#3a1a1a;color:#e94560}
    footer{margin-top:18px;font-size:0.70em;color:#444;text-align:center}
    .ctrl-box{background:#16213e;border-radius:8px;padding:16px;margin-top:2px}
    .ctrl-row{display:flex;align-items:center;gap:12px;flex-wrap:wrap;margin-bottom:14px}
    .big-btn{background:#e94560;color:#fff;border:none;padding:16px 28px;border-radius:6px;
             font-size:1.15em;font-weight:bold;cursor:pointer;min-width:180px}
    .big-btn.off{background:#0f3460}
    .pwr-display{text-align:center;min-width:70px;display:flex;flex-direction:column;
                 align-items:center;align-self:flex-start;padding-top:3px}
    .pwr-label{font-size:0.65em;color:#888;text-transform:uppercase;letter-spacing:.05em;
               margin-bottom:3px}
    .pwr-val{font-size:1.4em;font-weight:bold;color:#e94560;line-height:1.2}
    .pwr-unit{font-size:0.72em;color:#aaa;margin-left:2px}
    .phase-badge{font-size:1em;font-weight:bold;padding:2px 10px;border-radius:4px;
                 border:1px solid currentColor;white-space:nowrap;line-height:1.4}
    .phase-bulk      {color:#e9a020;border-color:#e9a020}
    .phase-absorption{color:#4ab4f8;border-color:#4ab4f8}
    .phase-float     {color:#4caf82;border-color:#4caf82}
    .preset-label{font-size:0.70em;color:#888;text-transform:uppercase;
                  letter-spacing:.05em;margin-bottom:6px}
    .preset-row{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:14px}
    /* UX-11: 44 px minimum touch height on every tap target. */
    .preset-btn{background:#0f3460;color:#eee;border:1px solid #e94560;padding:7px 14px;
                border-radius:5px;font-size:0.85em;cursor:pointer;min-height:44px}
    .preset-btn:hover{background:#e94560}
    .preset-btn.active{background:#e94560}
    .slider-wrap{display:flex;align-items:center;gap:10px}
    input[type=range]{flex:1;accent-color:#e94560;height:6px}
    .slider-readout{min-width:70px;text-align:right;font-size:0.9em;color:#eee}
    .cc-row{display:flex;align-items:center;gap:10px;margin-top:14px;flex-wrap:wrap}
    .cc-row .preset-label{margin-bottom:0}
    .cc-btn{background:#0f3460;color:#eee;border:1px solid #0f3460;padding:6px 14px;
            border-radius:5px;font-size:0.85em;cursor:pointer;min-width:44px;
            min-height:44px;text-align:center}
    .cc-btn.active{background:#e94560;border-color:#e94560}
    .cc-hint{font-size:0.72em;color:#666;margin-top:6px;line-height:1.4}
    /* SOC card */
    .soc-card{grid-column:span 2}
    .soc-val{font-size:2.2em!important}
    .soc-bar-wrap{background:#0f3460;border-radius:4px;height:6px;margin-top:8px;overflow:hidden}
    .soc-bar{height:100%;border-radius:4px;background:#e94560;width:0%;transition:width 0.6s ease}
    /* Session data grid */
    .session-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(140px,1fr));gap:10px;margin-top:0}
    /* Target voltage card */
    .tgt-volt-box{background:#16213e;border-radius:8px;padding:16px;margin-top:2px}
    .tgt-volt-row{display:flex;flex-wrap:wrap;gap:10px;margin-top:8px}
    .tgt-btn{background:#0f3460;color:#eee;border:1px solid #e94560;padding:10px 18px;
             border-radius:6px;font-size:0.9em;font-weight:bold;cursor:pointer;flex:1;
             min-width:80px;text-align:center;line-height:1.3}
    .tgt-btn:hover{background:#e94560}
    .tgt-btn.active{background:#e94560}
    .tgt-btn .tgt-v{font-size:0.72em;color:rgba(255,255,255,0.65);display:block;
                    font-weight:normal;margin-top:2px}
    .tgt-btn.active .tgt-v{color:rgba(255,255,255,0.85)}
    /* Reset-session button (UX-23) — a real button element, not a clickable div. */
    .mini-btn{background:#0f3460;color:#eee;border:1px solid #0f3460;border-radius:5px;
              font-size:1em;cursor:pointer;padding:8px 12px;min-height:44px;width:100%;
              text-align:left;margin-top:2px}
    .mini-btn:hover{border-color:#e94560}
    /* Disabled state while a control POST is in flight (UX-1). */
    button:disabled,input:disabled{opacity:.45;cursor:progress}
    /* Toast (UX-1) — bottom of the screen, high contrast, above the
       home-indicator inset, auto-hides after 4 s. */
    .toast{position:fixed;left:12px;right:12px;
           bottom:calc(14px + env(safe-area-inset-bottom));
           padding:14px 16px;border-radius:8px;font-size:0.95em;font-weight:bold;
           text-align:center;z-index:1000;opacity:0;pointer-events:none;
           transform:translateY(12px);transition:opacity .18s,transform .18s;
           box-shadow:0 6px 20px rgba(0,0,0,.55)}
    .toast.show{opacity:1;transform:translateY(0)}
    .toast.ok {background:#0a3d2a;color:#7ef0b0;border:1px solid #4ade80}
    .toast.err{background:#4a0d0d;color:#ffd4d4;border:1px solid #f87171}
    /* In-page confirm (UX-2/UX-3) — replaces window.confirm, which iOS renders
       with the page hostname and no styling control. */
    .cfm-overlay{position:fixed;top:0;left:0;right:0;bottom:0;
                 background:rgba(0,0,0,.72);z-index:1100;
                 display:flex;align-items:center;justify-content:center;padding:20px}
    .cfm-box{background:#16213e;border:1px solid #0f3460;border-radius:10px;
             padding:20px;max-width:380px;width:100%;
             box-shadow:0 10px 30px rgba(0,0,0,.6)}
    .cfm-msg{font-size:1em;line-height:1.5;color:#eee;margin-bottom:18px}
    .cfm-row{display:flex;gap:10px}
    .cfm-btn{flex:1;min-height:48px;border:none;border-radius:6px;font-size:1em;
             font-weight:bold;cursor:pointer;padding:12px}
    .cfm-no {background:#0f3460;color:#eee;border:1px solid #444}
    .cfm-yes{background:#e94560;color:#fff}
  </style>
</head>
<body>
  <h1>&#9889; Supercharger
    <span class="badge stale" id="badge" role="status" aria-live="polite">NO DATA</span>
  </h1>
  <nav><a href="/settings">&#9881; Settings</a><a href="/update">&#128190; OTA Update</a><a href="/log">&#128220; Log</a><a href="/api/cycles" download="cycles.csv">&#11015; Cycles</a><a href="#" onclick="logout();return false">&#128274; Logout</a></nav>

  <div id="apBanner" style="display:none;background:#0f3460;border:1px solid #e94560;
       border-radius:8px;padding:12px 16px;margin-top:12px;text-align:center">
    <span style="color:#e94560;font-weight:bold">&#9888; No WiFi</span>
    <span style="color:#aaa"> — Running in AP mode.</span>
    <a href="/settings" style="color:#e94560;margin-left:8px">Configure WiFi &#8594;</a>
  </div>

  <!-- Protection banner. ONE element for every protection state rather than one
       div per condition: the firmware already decided which state is most
       severe (activeProtection()), so the UI only ever has one thing to say.
       Colour, icon and copy are all set from d.protection in updateStatus().
       Replaces the former separate thermalBanner / inhibitBanner. -->
  <div id="protBanner" role="status" aria-live="polite"
       style="display:none;border-radius:8px;padding:12px 16px;
       margin-top:12px;text-align:center;border:1px solid #888;background:#222">
    <span id="protTitle" style="font-weight:bold"></span>
    <span id="protMsg" style="color:#aaa"></span>
  </div>

  <div class="section">Monolith Pack</div>
  <div class="grid">
    <div class="card"><div class="label">Voltage</div>
      <span class="val" id="mV">—</span></div>
    <div class="card"><div class="label">Current</div>
      <span class="val" id="mA">—</span></div>
    <div class="card"><div class="label">Capacity <span style="font-size:0.7em;color:#888;font-weight:normal">avail / total</span></div>
      <span class="val" id="mAH">—</span></div>
    <div class="card soc-card"><div class="label">State of Charge <span id="mSOCSrc" style="font-size:0.7em;color:#888;font-weight:normal"></span></div>
      <span class="val soc-val" id="mSOC">—</span>
      <div class="soc-bar-wrap"><div class="soc-bar" id="mSOCBar"></div></div></div>
    <div class="card"><div class="label">Temp Min / Max</div>
      <span class="val" id="mT">—</span></div>
    <div class="card"><div class="label">Cell Balance <span style="font-size:0.7em;color:#888;font-weight:normal">max&#8722;min</span></div>
      <span class="val" id="mCellBal">—</span></div>
    <div class="card"><div class="label">Cell Avg <span style="font-size:0.7em;color:#888;font-weight:normal">BMS</span></div>
      <span class="val" id="mCellAvg">—</span></div>
    <div class="card"><div class="label">BMS Temp</div>
      <span class="val" id="mBmsTemp">—</span></div>
  </div>

  <div id="ptSection" style="display:none">
    <div class="section">PowerTank Pack</div>
    <div class="grid">
      <div class="card"><div class="label">Voltage</div>
        <span class="val" id="pV">—</span></div>
      <div class="card"><div class="label">Current</div>
        <span class="val" id="pA">—</span></div>
      <div class="card"><div class="label">Capacity <span style="font-size:0.7em;color:#888;font-weight:normal">avail / total</span></div>
        <span class="val" id="pAH">—</span></div>
      <div class="card"><div class="label">Temp Min / Max</div>
        <span class="val" id="pT">—</span></div>
    </div>
  </div>

  <div class="section">Chargers
    <span class="badge stale" id="hbBadge" style="margin-left:8px">HB: —</span>
  </div>
  <div id="chargerGrid" class="grid"></div>

  <div class="section">Session</div>
  <div class="session-grid">
    <div class="card"><div class="label">Energy Delivered</div>
      <span class="val" id="sessWh">—</span><span class="unit">Wh</span></div>
    <div class="card"><div class="label">Charge Delivered</div>
      <span class="val" id="sessAh">—</span><span class="unit">Ah</span></div>
    <div class="card"><div class="label">Time Remaining</div>
      <span class="val" id="etaTime">—</span></div>
    <div class="card"><div class="label">Ramp Rate</div>
      <span class="val" id="rampRate">—</span><span class="unit">W/s</span></div>
    <div class="card">
      <div class="label">Reset Session</div>
      <button type="button" id="btnReset" class="mini-btn" onclick="resetSession()">
        &#8635; Reset</button></div>
  </div>

  <div class="section">Charging Control</div>
  <div class="ctrl-box">

    <div class="ctrl-row">
      <button type="button" class="big-btn" id="btnEnable" onclick="toggleEnable()">&#9654; Charging ON</button>
      <div class="pwr-display">
        <div class="pwr-label">Current</div>
        <span class="pwr-val" id="curPwr">—</span><span class="pwr-unit">W</span>
      </div>
      <div class="pwr-display">
        <div class="pwr-label">Target</div>
        <span class="pwr-val" id="tgtPwr">—</span><span class="pwr-unit">W</span>
      </div>
      <div class="pwr-display">
        <div class="pwr-label">Mode</div>
        <span id="chgMode" class="phase-badge phase-bulk">—</span>
      </div>
    </div>

    <div class="preset-label">Presets</div>
    <div id="presetBtns" class="preset-row"></div>

    <div class="slider-wrap">
      <input type="range" id="pwrSlider" min="0" max="13200" step="100" value="0"
             oninput="onSliderInput(this.value)" onchange="onSliderCommit(this.value)">
      <div class="slider-readout">
        <span id="sliderVal">0</span><span> W</span>
      </div>
    </div>

    <div class="cc-row">
      <span class="preset-label">Chargers</span>
      <button type="button" class="cc-btn" onclick="setChargerCount(1)">1</button>
      <button type="button" class="cc-btn" onclick="setChargerCount(2)">2</button>
      <button type="button" class="cc-btn active" onclick="setChargerCount(3)">3</button>
      <button type="button" class="cc-btn" onclick="setChargerCount(4)">4</button>
    </div>
    <div class="cc-hint">Sets the preset table and slider range only.
      Does not change what the chargers physically do.</div>
  </div>

  <div class="section">Target Voltage</div>
  <div class="tgt-volt-box">
    <div id="tgtVoltBtns" class="tgt-volt-row"></div>
  </div>

  <div class="section">System
    <a id="sysToggle" href="#" onclick="toggleSys();return false;"
       style="font-size:0.75em;color:#e94560;text-transform:none;letter-spacing:0;
              font-weight:bold;margin-left:10px;text-decoration:none">show</a>
  </div>
  <div id="sysGrid" class="grid" style="display:none">
    <div class="card"><div class="label">Uptime</div>
      <span class="val" id="uptime">—</span></div>
    <div class="card"><div class="label">WiFi RSSI</div>
      <span class="val" id="rssi">—</span></div>
    <div class="card"><div class="label">Firmware</div>
      <span class="val" style="font-size:0.85em" id="ver">—</span></div>
    <div class="card"><div class="label">CPU Core 0</div>
      <span class="val" id="cpu0">—</span></div>
    <div class="card"><div class="label">CPU Core 1</div>
      <span class="val" id="cpu1">—</span></div>
    <div class="card"><div class="label">Free Heap</div>
      <span class="val" id="heap">—</span></div>
    <div class="card"><div class="label">Cycles Logged</div>
      <span class="val" id="cycleCount">—</span></div>
    <div class="card"><div class="label">Odometer</div>
      <span class="val" id="odo">—</span></div>
  </div>

  <footer id="footer">Waiting for first update...</footer>

  <script>
    // /log and /update are served ONLY on port 80 (multipart OTA and the SSE
    // log stream have no equivalent on the IDF TLS server). Reached over
    // https:// they resolve to port 443, where nothing is registered, and the
    // user gets a bare 404. Point them at http:// explicitly when this page
    // itself arrived over TLS.
    if (location.protocol === 'https:') {
      document.querySelectorAll('nav a[href="/log"], nav a[href="/update"]').forEach(function(a){
        a.href = 'http://' + location.hostname + a.getAttribute('href');
      });
    }

    var fmt = function(v, d){
      return (v === null || v === undefined || v === '' || isNaN(v)) ? '—' : (+v).toFixed(d);
    };
    // UX-13: fmt() + a unit suffix, but never "— V" / "undefined A" — a missing
    // field renders as a bare em dash.
    var unit = function(v, d, u){ var s = fmt(v, d); return s === '—' ? s : (s + u); };
    // Raw (already-integer) value with a suffix, or an em dash when absent.
    var rawUnit = function(v, u){
      return (v === null || v === undefined || v === '' || isNaN(v)) ? '—' : (v + u);
    };
    // D10: a min/max pair with BOTH sensors dead read as "— / — °C", which
    // looks like two readings. One dash means "no data" much more clearly.
    // A half-dead pair still shows both halves so the live one is visible.
    var pairUnit = function(a, b, dec, u){
      var sa = fmt(a, dec), sb = fmt(b, dec);
      if (sa === '—' && sb === '—') return '—';
      return sa + ' / ' + sb + u;
    };

    // ---- Toast (UX-1) ----------------------------------------------------
    // One reusable element, created on first use, auto-hidden after 4 s.
    var toastTimer = null;
    function toast(msg, ok) {
      var t = document.getElementById('toast');
      if (!t) {
        t = document.createElement('div');
        t.id = 'toast';
        t.setAttribute('role', 'status');
        t.setAttribute('aria-live', 'polite');
        document.body.appendChild(t);
      }
      t.textContent = msg;
      t.className = 'toast show ' + (ok ? 'ok' : 'err');
      if (toastTimer) clearTimeout(toastTimer);
      toastTimer = setTimeout(function(){ t.className = 'toast ' + (ok ? 'ok' : 'err'); }, 4000);
    }

    // ---- In-page confirm (UX-2) -----------------------------------------
    // Built from DOM nodes (textContent, never innerHTML) so a message can
    // never become an injection sink.
    function uiConfirm(msg, onYes, onNo) {
      var ov  = document.createElement('div'); ov.className  = 'cfm-overlay';
      var box = document.createElement('div'); box.className = 'cfm-box';
      var p   = document.createElement('div'); p.className   = 'cfm-msg';
      p.textContent = msg;
      var row = document.createElement('div'); row.className = 'cfm-row';
      var no  = document.createElement('button');
      no.type = 'button'; no.className = 'cfm-btn cfm-no';  no.textContent  = 'Cancel';
      var yes = document.createElement('button');
      yes.type = 'button'; yes.className = 'cfm-btn cfm-yes'; yes.textContent = 'Confirm';
      var closed = false;
      function close(cancelled) {
        if (closed) return; closed = true;
        if (ov.parentNode) ov.parentNode.removeChild(ov);
        document.removeEventListener('keydown', onKey);
        if (cancelled && onNo) onNo();
      }
      function onKey(e){ if (e.key === 'Escape') close(true); }
      no.onclick  = function(){ close(true); };
      yes.onclick = function(){ close(false); if (onYes) onYes(); };
      ov.onclick  = function(e){ if (e.target === ov) close(true); };
      document.addEventListener('keydown', onKey);
      row.appendChild(no); row.appendChild(yes);
      box.appendChild(p);  box.appendChild(row);
      ov.appendChild(box);
      document.body.appendChild(ov);
      yes.focus();
    }

    // Preset table — mirrors POWER_PRESETS on the ESP32.
    // Each row = charger_count * [500, 1000, 1650, 2200, 3300] rounded.
    var PRESETS = [
      [  500, 1000, 1650, 2200,  3300 ],
      [ 1000, 2000, 3300, 4400,  6600 ],
      [ 1500, 3000, 5000, 6600,  9900 ],
      [ 2000, 4000, 6600, 8800, 13200 ]
    ];

    // Target voltage presets — mirrors TARGET_VOLT_PRESETS on the ESP32
    var TARGET_VOLT_PRESETS = [
      { pct:  70, dv: 1060 },
      { pct:  80, dv: 1100 },
      { pct:  90, dv: 1132 },
      { pct: 100, dv: 1164 }
    ];
    var tgtVoltBuilt = false;

    var chargingEnabled = false; // updated from server on first refresh
    var lastChargerCount = -1;  // track when count changes to rebuild presets
    var curTargetDv = 0;        // last target voltage seen from the server
    var redirecting = false;    // true once we've started navigating to /login

    // Status bitfield labels
    var STATUS_BITS = [
      [0x01, 'HW Fault'],
      [0x02, 'Overtemp'],
      [0x04, 'AC Fault'],
      [0x08, 'Starting/No Batt'],
      [0x10, 'Comm Timeout']
    ];
    // 0x08 and 0x10 are informational (Elcon TC protocol: starting state /
    // command-frame timeout) and are normal while idle; the firmware only
    // acts on 0x01 and 0x02 — keep these labels in step with the C table.
    function statusText(s) {
      if (s === 0) return 'OK';
      var out = [];
      for (var i = 0; i < STATUS_BITS.length; i++) {
        if (s & STATUS_BITS[i][0]) out.push(STATUS_BITS[i][1]);
      }
      return out.join(', ');
    }
    function statusClass(s) { return s === 0 ? 'ok' : 'stale'; }

    // ---- Control helpers ----
    //
    // UX-1: every /api/control POST goes through postControl(). It
    //   * disables every control while the request is in flight, so a second
    //     tap can't race the first,
    //   * treats a non-2xx *or* {"ok":false} as a failure and shows the
    //     server's own error text in a toast,
    //   * maps 401 → /login and 503 → "Controller busy — try again"
    //     (applyApiControlBody() returns 503 when controlMutex timed out and
    //     NOTHING was applied — the old fire-and-forget code silently showed
    //     the user a stop/start that never happened), and
    //   * forces an immediate /api/status poll on both paths, so an optimistic
    //     button state is always overwritten by the controller's real state.

    var ctrlBusy = false;
    var ctrlPrev = null;   // [element, wasDisabled] pairs, for exact restore

    function controlEls() {
      var out = [];
      var one = document.getElementById('btnEnable');
      if (one) out.push(one);
      var sel = document.querySelectorAll('.preset-btn, .tgt-btn, .cc-btn, #pwrSlider, #btnReset');
      for (var i = 0; i < sel.length; i++) out.push(sel[i]);
      return out;
    }

    // Restores the prior disabled state rather than blanket-enabling — the
    // slider is legitimately disabled when the charger count is out of range.
    function setCtrlDisabled(on) {
      if (on) {
        var els = controlEls();
        ctrlPrev = [];
        for (var i = 0; i < els.length; i++) {
          ctrlPrev.push([els[i], els[i].disabled]);
          els[i].disabled = true;
        }
      } else if (ctrlPrev) {
        for (var j = 0; j < ctrlPrev.length; j++) ctrlPrev[j][0].disabled = ctrlPrev[j][1];
        ctrlPrev = null;
      }
    }

    function postControl(body, onOk) {
      if (ctrlBusy) return;
      ctrlBusy = true;
      setCtrlDisabled(true);
      var settle = function(){
        ctrlBusy = false;
        // On the 401 path we are already navigating to /login — clear the
        // busy latch, but leave the controls inert rather than re-arming a
        // page that is on its way out.
        if (redirecting) return;
        setCtrlDisabled(false);
        refreshSoon();          // re-sync the UI from the controller's truth
      };
      fetch('/api/control', {
        method: 'POST',
        credentials: 'same-origin',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify(body)
      }).then(function(r){
        return r.text().then(function(txt){
          if (r.status === 401) { redirecting = true; window.location.href = '/login';
                                  throw new Error('__auth__'); }
          // The HTTP path answers "No body" as text/plain, everything else as
          // JSON — parse defensively rather than assuming r.json() succeeds.
          var d = null;
          try { d = JSON.parse(txt); } catch (e) {}
          if (r.status === 503) throw new Error('Controller busy — try again');
          if (!r.ok) throw new Error((d && d.error) || txt || ('HTTP ' + r.status));
          if (!d || d.ok !== true) throw new Error((d && d.error) || 'Command not applied');
          return d;
        });
      }).then(function(d){
        // D8: a throw inside onOk is a rendering bug, not a failed command —
        // don't report it to the user as one. Log it and carry on.
        if (onOk) {
          try { onOk(d); }
          catch (err) { if (window.console) console.error('onOk failed', err); }
        }
      }).catch(function(e){
        if (e && e.message === '__auth__') return;   // navigating away
        toast((e && e.message) ? e.message : 'Connection error', false);
      }).then(settle, settle);   // D8: settle runs on every path
    }

    function setPreset(w) {
      var slider = document.getElementById('pwrSlider');
      slider.value = w;
      document.getElementById('sliderVal').textContent = w;
      postControl({target_w: w, enabled: chargingEnabled});
    }

    // Live readout while dragging — don't send until release
    function onSliderInput(v) {
      document.getElementById('sliderVal').textContent = v;
    }

    // Commit on mouse/touch release
    function onSliderCommit(v) {
      postControl({target_w: parseInt(v), enabled: chargingEnabled});
    }

    // Human-readable current target, for the start-charging confirm text.
    function targetPctText() {
      for (var i = 0; i < TARGET_VOLT_PRESETS.length; i++) {
        if (TARGET_VOLT_PRESETS[i].dv === curTargetDv) return TARGET_VOLT_PRESETS[i].pct + '%';
      }
      return (curTargetDv > 0) ? ((curTargetDv / 10).toFixed(1) + ' V') : 'the current target';
    }

    // UX-2: starting and stopping a charge are both confirmed. The optimistic
    // button flip stays, but a failed POST is reverted by the forced poll in
    // postControl()'s settle().
    function toggleEnable() {
      var slider = document.getElementById('pwrSlider');
      var want   = !chargingEnabled;
      var msg    = want ? ('Start charging at ' + (parseInt(slider.value) || 0) +
                           ' W to ' + targetPctText() + '?')
                        : 'Stop charging?';
      uiConfirm(msg, function(){
        // D9: the confirm is modal to this page only. Home Assistant, a second
        // browser, or the ramp task reaching Float can all change the state
        // while the dialog is open, and the poll keeps running behind it — so
        // re-read everything at accept time instead of acting on the snapshot
        // taken when the dialog opened.
        if (chargingEnabled !== !want) {
          toast('State changed — try again', false);
          return;
        }
        var w = parseInt(document.getElementById('pwrSlider').value) || 0;
        chargingEnabled = want;
        updateEnableBtn();
        postControl({target_w: w, enabled: want}, function(){
          toast(want ? 'Charging started' : 'Charging stopped', true);
        });
      });
    }

    function setChargerCount(n) {
      postControl({charger_count: n}, function(){
        var btns = document.querySelectorAll('.cc-btn');
        for (var i = 0; i < btns.length; i++) {
          btns[i].className = 'cc-btn' + (i + 1 === n ? ' active' : '');
        }
        lastChargerCount = -1; // force preset rebuild
        rebuildPresets(n);
      });
    }

    function updateEnableBtn() {
      var btn = document.getElementById('btnEnable');
      if (chargingEnabled) {
        btn.textContent = '\u25B6 Charging ON';
        btn.className = 'big-btn';
      } else {
        btn.textContent = '\u25A0 Charging OFF';
        btn.className = 'big-btn off';
      }
    }

    // ---- Target voltage ----

    function buildTargetVoltBtns() {
      if (tgtVoltBuilt) return;
      tgtVoltBuilt = true;
      var row = document.getElementById('tgtVoltBtns');
      row.innerHTML = '';
      for (var i = 0; i < TARGET_VOLT_PRESETS.length; i++) {
        (function(p) {
          var btn = document.createElement('button');
          btn.className = 'tgt-btn';
          btn.id = 'tvBtn_' + p.dv;
          btn.innerHTML = p.pct + '%<span class="tgt-v">' +
                          (p.dv / 10).toFixed(1) + ' V</span>';
          btn.onclick = function() { setTargetVolt(p.dv); };
          row.appendChild(btn);
        })(TARGET_VOLT_PRESETS[i]);
      }
    }

    function setTargetVolt(dv) {
      postControl({target_volt_dv: dv}, function(){
        curTargetDv = dv;
        // Optimistic highlight — only once the controller confirmed it. A
        // failed POST leaves the old highlight, and the forced poll in
        // postControl() re-asserts the real target either way.
        for (var i = 0; i < TARGET_VOLT_PRESETS.length; i++) {
          var b = document.getElementById('tvBtn_' + TARGET_VOLT_PRESETS[i].dv);
          if (b) b.className = 'tgt-btn' + (TARGET_VOLT_PRESETS[i].dv === dv ? ' active' : '');
        }
      });
    }

    function syncTargetVoltBtns(dv) {
      buildTargetVoltBtns();
      for (var i = 0; i < TARGET_VOLT_PRESETS.length; i++) {
        var b = document.getElementById('tvBtn_' + TARGET_VOLT_PRESETS[i].dv);
        if (b) b.className = 'tgt-btn' + (TARGET_VOLT_PRESETS[i].dv === dv ? ' active' : '');
      }
    }

    // ---- Session reset ----

    function resetSession() {
      uiConfirm('Reset the session Wh/Ah counters? This cannot be undone.', function(){
        postControl({reset_session: true}, function(){ toast('Session counters reset', true); });
      });
    }

    // ---- SOC bar colour ----

    function socColour(pct) {
      if (pct >= 80) return '#4caf50';
      if (pct >= 40) return '#e9a000';
      return '#e94560';
    }

    // Rebuild preset buttons and update slider range when charger count changes
    function rebuildPresets(count) {
      if (count === lastChargerCount) return;
      lastChargerCount = count;

      var row = document.getElementById('presetBtns');
      row.innerHTML = '';
      var slider = document.getElementById('pwrSlider');

      if (count < 1 || count > 4) {
        row.innerHTML = '<span style="color:#666;font-size:0.85em">Set charger count</span>';
        slider.disabled = true;
        return;
      }

      var presets = PRESETS[count - 1];
      var maxW = presets[presets.length - 1];

      slider.min      = 0;
      slider.max      = maxW;
      slider.step     = 100;
      slider.disabled = false;

      // Clamp slider if current value exceeds new max
      if (parseInt(slider.value) > maxW) {
        slider.value = maxW;
        document.getElementById('sliderVal').textContent = maxW;
      }

      for (var i = 0; i < presets.length; i++) {
        (function(w){
          var btn = document.createElement('button');
          btn.className   = 'preset-btn';
          var kw = w / 1000;
          btn.textContent = (w >= 1000 ? (kw % 1 === 0 ? kw.toFixed(0) : kw.toFixed(1)) + 'kW' : w + 'W');
          btn.onclick = function(){ setPreset(w); };
          row.appendChild(btn);
        })(presets[i]);
      }
    }

    function logout(){
      // Use a real form submit so the browser navigates away and renders
      // the server's "logged out" page directly.  A fetch() + redirect
      // would follow the 3xx back through Digest auto-auth and end up
      // silently re-logging the user in (looks like a page refresh).
      var f = document.createElement('form');
      f.method = 'POST'; f.action = '/logout';
      document.body.appendChild(f); f.submit();
    }

    // ---- Polling (UX-14) ------------------------------------------------
    // Self-rescheduling: the next request is only issued 2 s AFTER the
    // previous one settled, so a slow or hung link can't stack up requests
    // (the old setInterval fired regardless). Each request is aborted after
    // 5 s, and two consecutive failures flip the existing badge to NO DATA.
    var pollTimer  = null;
    var pollDue    = 0;      // timestamp the pending poll is scheduled for
    var pollFails  = 0;
    var refreshing = false;

    // D7: never push a pending poll further out. An in-flight poll that
    // settles just after a control POST called refreshSoon() used to replace
    // the forced 250 ms resync with its own 2 s one — so the UI sat in the
    // optimistic state, and a payload captured before the command landed
    // could snap the button back. Anything already queued sooner wins.
    function scheduleRefresh(ms) {
      if (redirecting) return;
      var due = Date.now() + ms;
      if (pollTimer && pollDue <= due) return;     // something sooner is queued
      if (pollTimer) clearTimeout(pollTimer);
      pollDue   = due;
      pollTimer = setTimeout(function(){ pollTimer = null; refresh(); }, ms);
    }
    // Called after a control POST so the UI re-syncs immediately instead of
    // sitting in an optimistic state for up to 2 s.
    function refreshSoon(){ scheduleRefresh(250); }

    function refresh(){
      if (redirecting) return;
      if (refreshing) { scheduleRefresh(2000); return; }   // previous still in flight
      refreshing = true;

      var opts = {credentials:'same-origin'};
      var ctl = null, tmr = null;
      if (window.AbortController) {
        ctl = new AbortController();
        opts.signal = ctl.signal;
        tmr = setTimeout(function(){ try { ctl.abort(); } catch(e) {} }, 5000);
      }
      var settle = function(){
        refreshing = false;
        if (tmr) { clearTimeout(tmr); tmr = null; }
        scheduleRefresh(2000);
      };

      fetch('/api/status', opts)
        .then(function(r){
          // Session expired (or never existed) → bounce to /login.
          // The cookie is HttpOnly so we can't inspect it from JS —
          // a 401 from /api/status is our cue to redirect.
          if (r.status === 401) {
            redirecting = true;
            window.location.href = '/login';
            throw new Error('__auth__');
          }
          if (!r.ok) throw new Error('HTTP ' + r.status);
          return r.json();
        })
        .then(function(d){
          pollFails = 0;
          var b = document.getElementById('badge');
          b.textContent = d.fresh ? 'LIVE' : 'NO DATA';
          b.className   = 'badge ' + (d.fresh ? 'ok' : 'stale');

          // AP mode banner
          document.getElementById('apBanner').style.display = d.ap_mode ? 'block' : 'none';

          // ── Protection banner ────────────────────────────────────────────
          // d.protection is the single highest-severity active state, already
          // prioritised by activeProtection() in the firmware. Keys here MUST
          // match protStateKey(). Three severities:
          //   stop  (red)   — no current is flowing
          //   warn  (amber) — still charging, but limited
          //   info  (blue)  — advisory only, charging unaffected
          var PROT = {
            too_cold: { sev:'stop', icon:'❄', title:'Charging paused — pack too cold',
              msg:' Pack is below 0 °C. Charging a cold lithium cell plates metallic lithium on the anode, which permanently reduces capacity and can eventually short the cell. Charging will resume on its own once the pack warms above 2 °C — riding it, or moving the bike somewhere warmer, is the quickest way.' },
            too_hot: { sev:'stop', icon:'🔥', title:'Charging paused — pack too hot',
              msg:' Pack is above 45 °C, the cell manufacturer’s charging limit. Charging will resume on its own once it cools below 43 °C. If this happens without hard riding beforehand, check for blocked airflow around the pack.' },
            pack_low: { sev:'stop', icon:'⚠', title:'Charging blocked — pack voltage too low',
              msg:' Pack is below 70 V (2.5 V per cell). Below this level the copper inside the cells starts to dissolve, and charging normally can create an internal short. This should not happen in normal use — the bike’s BMS cuts out well above it. Have the pack checked by a technician before charging; do not force it.' },
            temp_unknown: { sev:'stop', icon:'🌡', title:'Pack temperature unknown',
              msg:' The BMS is not reporting a valid pack temperature. Charging is inhibited until temperature data returns.' },
            bms_stale: { sev:'stop', icon:'🔌', title:'Charging stopped — no data from bike',
              msg:' The controller has lost contact with the bike’s battery management system, so it cannot see pack voltage or temperature and will not charge blind. Check that the charge connector is fully seated and the bike is powered on. Charging resumes automatically within a second of data returning.' },
            charger_fault: { sev:'stop', icon:'⛔', title:'Charger fault',
              msg:' A charger reports a fault (see charger status). Charging stopped — fix the cause, then press Stop and Charge again to re-arm.' },
            charger_mismatch: { sev:'stop', icon:'⚠', title:'Charger count mismatch',
              msg:' More chargers detected on the bus than configured. Charging stopped — set the charger count in Settings to match.' },
            charger_clamped: { sev:'stop', icon:'⚠', title:'Wiring problem — too many chargers',
              msg:' More than four chargers are answering on the CAN bus. The controller divides the requested current between the chargers it knows about, so an extra unit would make every charger deliver more than intended. Disconnect the extra charger, or set distinct CAN instance IDs, before charging.' },
            thermal_throttle: { sev:'warn', icon:'🌡', title:'Reduced power — pack warm',
              msg:' Pack temperature is high, so charge power has been cut back. This is normal on a hot day or after a fast ride. Charging continues safely at a lower rate and will speed up as the pack cools — nothing to do.' },
            current_clamped: { sev:'warn', icon:'🛡', title:'Reduced power — at cell current limit',
              msg:' The requested power needs more current than the cells are rated to accept (1 C), so it has been capped. Charging continues safely at the maximum the pack allows. Lower the power setting if you’d rather not sit at the limit.' },
            frames_rejected: { sev:'info', icon:'ℹ', title:'Ignoring bad data from bike',
              msg:' Some CAN messages from the battery are arriving corrupted and are being discarded rather than acted on. Charging is unaffected while valid messages keep coming. If this persists, check the CAN wiring and connector for damage or a loose shield.' }
          };
          var SEV = {
            stop: { bg:'#3a0a0a', border:'#e94560', fg:'#e94560' },
            warn: { bg:'#3a1f0a', border:'#ff8c00', fg:'#ff8c00' },
            info: { bg:'#0a2a3a', border:'#4aa3c7', fg:'#4aa3c7' }
          };
          var pb = document.getElementById('protBanner');
          var p  = PROT[d.protection];
          if (p) {
            var s = SEV[p.sev];
            pb.style.background  = s.bg;
            pb.style.borderColor = s.border;
            var t = document.getElementById('protTitle');
            t.style.color   = s.fg;
            t.textContent   = p.icon + ' ' + p.title;
            // textContent, not innerHTML — these strings are firmware
            // constants today, but keeping the sink safe means a future
            // dynamic detail can't turn into script injection.
            document.getElementById('protMsg').textContent = p.msg;
            pb.style.display = 'block';
          } else {
            pb.style.display = 'none';
          }

          // Pack current display:
          //  - When charging is enabled, current is flowing INTO the pack on
          //    every phase (Bulk/Absorption/Float). Zero's BMS reports this as
          //    a negative number (discharge convention); the negative is
          //    confusing in a "you are charging right now" UI, so flip the
          //    sign for display. The Mode badge already conveys phase, so no
          //    information is lost.
          //  - When charging is disabled (idle / bike being ridden) the sign
          //    remains as the BMS reports — useful for seeing discharge
          //    currents during a ride.
          // In absorption specifically we also swap the charger-reported W on
          // the "Current" power readout for V*I at the pack, since the
          // charger figure becomes meaningless as it tapers.
          var phase = d.ramp_phase || 'bulk';
          var inAbsorption = (phase === 'absorption');
          var charging     = !!d.charging_enabled;
          var dispMonoA    = charging ? Math.abs(d.monolith_a) : d.monolith_a;
          var dispPtA      = charging ? Math.abs(d.powertank_a) : d.powertank_a;

          // UX-13: every field below goes through fmt()/unit()/rawUnit(), so a
          // key missing from /api/status renders "—" rather than "undefined V".
          document.getElementById('mV').textContent  = unit(d.monolith_v, 1, ' V');
          document.getElementById('mA').textContent  = unit(dispMonoA, 1, ' A');
          // Capacity card: shows "available / total Ah" so the user sees both
          // how much is actually left in the pack right now (avail = total × SoC)
          // and the BMS's nominal capacity constant.
          var mAvail  = (d.monolith_ah_avail !== undefined) ? d.monolith_ah_avail : null;
          var mAhTot  = rawUnit(d.monolith_ah, '');   // keeps the BMS's own precision
          document.getElementById('mAH').textContent = (mAvail !== null)
            ? (fmt(mAvail, 1) + ' / ' + mAhTot + ' Ah')
            : (mAhTot === '—' ? '—' : (mAhTot + ' Ah'));
          // Pack temps arrive as JSON null when the BMS reports a disconnected
          // thermistor; fmt() already maps null/undefined to an em dash, so a
          // dead sensor shows a dash rather than a bogus reading.
          document.getElementById('mT').textContent  =
            pairUnit(d.monolith_tmin, d.monolith_tmax, 0, ' \u00B0C');

          // Cell balance tile \u2014 colour-coded: green <10 mV, amber 10\u201350 mV, red >50 mV
          var balEl = document.getElementById('mCellBal');
          if (d.cell_balance_mv !== undefined && d.cell_balance_mv > 0) {
            var balMv = d.cell_balance_mv;
            balEl.textContent = balMv + ' mV';
            balEl.style.color = balMv < 10 ? '#4caf50' : balMv < 50 ? '#ff9800' : '#f44336';
          } else {
            balEl.textContent = '\u2014';
            balEl.style.color = '';
          }

          // BMS board temperature (0x488 byte 1 \u2014 "Controller Temp" in ZeroSpy)
          var bmsEl = document.getElementById('mBmsTemp');
          bmsEl.textContent = (d.bms_board_temp !== undefined && d.bms_board_temp !== -128)
            ? (d.bms_board_temp + ' \u00B0C') : '\u2014';

          // Cell average voltage from BMS (0x488 bytes 6-7) \u2014 shown in V with 3 decimals
          var avgEl = document.getElementById('mCellAvg');
          avgEl.textContent = (d.cell_avg_mv !== undefined && d.cell_avg_mv > 0)
            ? ((d.cell_avg_mv / 1000).toFixed(3) + ' V') : '\u2014';

          // SOC card
          var soc = (d.monolith_soc !== undefined && d.monolith_soc !== null &&
                     !isNaN(d.monolith_soc)) ? d.monolith_soc : null;
          var socEl  = document.getElementById('mSOC');
          var socBar = document.getElementById('mSOCBar');
          if (soc !== null) {
            socEl.textContent = soc + ' %';
            socBar.style.width = soc + '%';
            socBar.style.background = socColour(soc);
          } else {
            socEl.textContent  = '—';
            socBar.style.width = '0%';
          }
          // SOC source label: blank when reading the BMS (the truth — same number
          // the bike's dashboard uses), "(est.)" while we're still falling back
          // to the voltage-curve estimate.
          var srcEl = document.getElementById('mSOCSrc');
          if (srcEl) srcEl.textContent = (d.monolith_soc_source === 'voltage') ? '(est.)' : '';

          // Session data
          document.getElementById('sessWh').textContent   = fmt(d.session_wh, 1);
          document.getElementById('sessAh').textContent   = fmt(d.session_ah, 2);
          // Time remaining: -1 = unknown / not charging, render as "—".
          // Otherwise format minutes as "Xh Ym" (or just "Ym" under an hour).
          var etaMin = d.eta_minutes;
          var etaEl  = document.getElementById('etaTime');
          if (etaMin === undefined || etaMin === null || etaMin < 0) {
            etaEl.textContent = '—';
          } else if (etaMin < 60) {
            etaEl.textContent = etaMin + ' min';
          } else {
            var h = Math.floor(etaMin / 60);
            var m = etaMin % 60;
            etaEl.textContent = h + 'h ' + (m < 10 ? '0' + m : m) + 'm';
          }
          document.getElementById('rampRate').textContent = rawUnit(d.ramp_rate_wps, '');

          // Target voltage buttons
          curTargetDv = d.target_volt_dv || 0;
          syncTargetVoltBtns(d.target_volt_dv);

          var ptSec = document.getElementById('ptSection');
          if (d.powertank_decided && d.powertank_present) {
            ptSec.style.display = 'block';
            document.getElementById('pV').textContent  = unit(d.powertank_v, 1, ' V');
            document.getElementById('pA').textContent  = unit(dispPtA, 1, ' A');
            var pAvail = (d.powertank_ah_avail !== undefined) ? d.powertank_ah_avail : null;
            var pAhTot = rawUnit(d.powertank_ah, '');
            document.getElementById('pAH').textContent = (pAvail !== null)
              ? (fmt(pAvail, 1) + ' / ' + pAhTot + ' Ah')
              : (pAhTot === '—' ? '—' : (pAhTot + ' Ah'));
            // null (disconnected thermistor) renders as a dash via fmt();
            // both dead collapses to a single dash (D10).
            document.getElementById('pT').textContent  =
              pairUnit(d.powertank_tmin, d.powertank_tmax, 0, ' \u00B0C');
          } else if (d.powertank_decided && !d.powertank_present) {
            ptSec.style.display = 'none';
          }

          // Charger section
          var hb = document.getElementById('hbBadge');
          hb.textContent = d.heartbeat_ok ? 'HB: OK' : 'HB: STOPPED';
          hb.className   = 'badge ' + (d.heartbeat_ok ? 'ok' : 'stale');

          var grid = document.getElementById('chargerGrid');
          var chargers = d.chargers || [];
          if (chargers.length > 0) {
            var totalA = 0; var avgV = 0; var worstStatus = 0;
            for (var i = 0; i < chargers.length; i++) {
              // UX-13: a charger entry missing a field must not poison the
              // sum into NaN and blank the whole tile.
              var ca = +chargers[i].a, cv = +chargers[i].v, cs = +chargers[i].status;
              totalA += isNaN(ca) ? 0 : ca;
              avgV   += isNaN(cv) ? 0 : cv;
              if (!isNaN(cs) && cs > worstStatus) worstStatus = cs;
            }
            avgV = avgV / chargers.length;
            grid.innerHTML =
              '<div class="card">' +
                '<div class="label">Chargers</div>' +
                '<span class="val" id="cv_sum">' + fmt(avgV, 1) + ' V</span> ' +
                '<span class="val" id="ca_sum">' + fmt(totalA, 1) + ' A</span>' +
                '<div style="margin-top:6px;font-size:0.75em">' +
                  '<span class="badge ' + statusClass(worstStatus) + '">' +
                    statusText(worstStatus) + '</span></div>' +
              '</div>';
          } else {
            grid.innerHTML = '<div class="card"><div class="label">Chargers</div>' +
              '<span class="val" style="font-size:1em;color:#666">No chargers seen</span></div>';
          }

          // Control section — sync from server state
          rebuildPresets(d.charger_count || 0);
          chargingEnabled = !!d.charging_enabled;
          updateEnableBtn();

          // Sync charger count button highlight from server
          var ccBtns = document.querySelectorAll('.cc-btn');
          for (var i = 0; i < ccBtns.length; i++) {
            ccBtns[i].className = 'cc-btn' + (i + 1 === d.charger_count ? ' active' : '');
          }

          // Current power: in absorption, derive from BMS (V*|I| at the pack)
          // rather than the charger-reported figure, and tint it the same blue
          // as the absorption mode badge to flag the swap.
          var curPwrEl = document.getElementById('curPwr');
          if (inAbsorption) {
            // UX-13: any missing BMS field would make this NaN — fall back to
            // the charger-reported figure instead of printing "NaN".
            var pwrW = Math.abs(+d.monolith_a) * (+d.monolith_v);
            if (d.powertank_decided && d.powertank_present) {
              pwrW += Math.abs(+d.powertank_a) * (+d.powertank_v);
            }
            curPwrEl.textContent = isNaN(pwrW) ? (d.current_power_w || 0) : Math.round(pwrW);
            curPwrEl.style.color = isNaN(pwrW) ? '' : '#4ab4f8';
          } else {
            curPwrEl.textContent = d.current_power_w || 0;
            curPwrEl.style.color = '';
          }
          document.getElementById('tgtPwr').textContent = d.target_power_w  || 0;
          var modeEl = document.getElementById('chgMode');
          var modeLabels = {bulk:'Bulk', absorption:'Absorption', float:'Float'};
          modeEl.textContent  = modeLabels[phase] || phase;
          modeEl.className    = 'phase-badge phase-' + phase;
          // Only update slider from server if user isn't dragging
          // (slider fires oninput while dragging, onchange on release)
          if (document.activeElement !== document.getElementById('pwrSlider')) {
            var slider = document.getElementById('pwrSlider');
            slider.value = d.target_power_w || slider.min;
            document.getElementById('sliderVal').textContent = slider.value;
          }

          document.getElementById('uptime').textContent = rawUnit(d.uptime_s, ' s');
          document.getElementById('rssi').textContent   = rawUnit(d.rssi, ' dBm');
          document.getElementById('ver').textContent    = (d.version === undefined ||
                                                           d.version === null ||
                                                           d.version === '') ? '—' : d.version;
          document.getElementById('cpu0').textContent   = rawUnit(d.cpu0, ' %');
          document.getElementById('cpu1').textContent   = rawUnit(d.cpu1, ' %');
          document.getElementById('heap').textContent       = d.free_heap_kb !== undefined
                                                            ? fmt(d.free_heap_kb, 1) + ' kB' : '—';
          document.getElementById('cycleCount').textContent = d.cycle_count !== undefined
                                                            ? d.cycle_count : '—';
          // Odometer: -1 (or absent) means no dash frame decoded yet → "—".
          document.getElementById('odo').textContent =
            (d.odometer_km !== undefined && d.odometer_km >= 0)
              ? (fmt(d.odometer_km, 1) + ' km') : '—';
          document.getElementById('footer').textContent =
            'Last update: ' + new Date().toLocaleTimeString();
          settle();
        })
        .catch(function(e){
          if (e && e.message === '__auth__') return;   // navigating to /login
          // UX-14: one dropped poll on a phone that just roamed is normal —
          // only call it stale after two consecutive failures.
          pollFails++;
          if (pollFails >= 2) {
            var b = document.getElementById('badge');
            b.textContent = 'NO DATA'; b.className = 'badge stale';
            document.getElementById('footer').textContent =
              'No response from controller since ' + new Date().toLocaleTimeString();
          }
          settle();
        });
    }
    // ---- Collapsible System section ----
    // Default: hidden.  User's choice persisted in localStorage key 'sysVis'.
    function toggleSys() {
      var grid = document.getElementById('sysGrid');
      var lnk  = document.getElementById('sysToggle');
      var vis  = grid.style.display !== 'none';
      grid.style.display = vis ? 'none' : '';
      lnk.textContent    = vis ? 'show' : 'hide';
      try { localStorage.setItem('sysVis', vis ? '0' : '1'); } catch(e) {}
    }
    // Restore saved state on load
    (function(){
      try {
        if (localStorage.getItem('sysVis') === '1') {
          document.getElementById('sysGrid').style.display = '';
          document.getElementById('sysToggle').textContent = 'hide';
        }
      } catch(e) {}
    })();

    // UX-14: the loop reschedules itself from refresh()'s settle(); no
    // setInterval, so a stalled request can never stack up behind itself.
    refresh();
  </script>
</body>
</html>
)rawliteral";

// ---------------------------------------------------------------------------
// OTA page — protected, linked from dashboard nav
// ---------------------------------------------------------------------------

const char HTML_OTA[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
  <meta name="apple-mobile-web-app-capable" content="yes">
  <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
  <meta name="theme-color" content="#1a1a2e">
  <title>OTA Update</title>
  <style>
    body{font-family:Arial;text-align:center;background:#1a1a2e;color:#eee;padding:20px;
         padding-top:calc(20px + env(safe-area-inset-top));
         padding-bottom:calc(20px + env(safe-area-inset-bottom));
         padding-left:calc(20px + env(safe-area-inset-left));
         padding-right:calc(20px + env(safe-area-inset-right))}
    h2{color:#e94560}
    .box{background:#16213e;padding:30px;border-radius:10px;
         box-shadow:0 4px 12px rgba(0,0,0,.4);max-width:480px;margin:auto}
    input[type="file"]{width:90%;padding:8px;margin:12px 0;border:1px solid #0f3460;
                       border-radius:5px;background:#0f3460;color:#eee;font-size:14px}
    button{background:#e94560;color:#fff;border:none;padding:12px 30px;
           border-radius:5px;font-size:16px;cursor:pointer;margin-top:10px}
    button:disabled{background:#555;cursor:not-allowed}
    #bar-wrap{background:#0f3460;border-radius:5px;height:22px;
              margin-top:16px;display:none}
    #bar{background:#e94560;height:100%;border-radius:5px;
         width:0%;transition:width .3s}
    #status{margin-top:14px;font-size:15px;min-height:22px}
    .ver{font-size:12px;color:#555;margin-top:20px}
    nav{margin-bottom:14px;display:flex;flex-wrap:wrap}
    /* UX-11: 44 px minimum touch target. */
    nav a{color:#aaa;font-size:0.85em;text-decoration:none;
          display:inline-flex;align-items:center;min-height:44px}
    nav a:hover{color:#e94560}
  </style>
</head>
<body>
  <div class="box">
    <nav><a href="/">&#8592; Dashboard</a></nav>
    <h2>&#128190; Firmware Update</h2>
    <p>Select a compiled <code>.bin</code> file to upload.</p>
    <p style="font-size:0.78em;color:#888">This page is always served over plain
      HTTP on port 80. With HTTPS enabled it needs its own login, separate from
      the one used on the secure dashboard.</p>
    <input type="file" id="bin" accept=".bin"><br>
    <button id="btn" onclick="upload()">Upload Firmware</button>
    <div id="bar-wrap"><div id="bar"></div></div>
    <div id="status"></div>
    <div class="ver" id="ver">Loading version...</div>
  </div>
  <script>
    // /update is registered on port 80 only. If this page was somehow reached
    // over TLS, keep the upload on http:// rather than POSTing to a port 443
    // that has no /update handler.
    var httpBase = (location.protocol === 'https:') ? 'http://' + location.hostname : '';
    if (location.protocol === 'https:') {
      document.querySelectorAll('nav a[href="/log"], nav a[href="/update"]').forEach(function(a){
        a.href = 'http://' + location.hostname + a.getAttribute('href');
      });
    }

    // UX-5 / C6: the version read-out is a nicety — the Upload button must
    // stay usable whatever happens to it.
    //   * 401 → the session expired, go and log in.
    //   * anything else (including the cross-origin failure you get when this
    //     page was loaded over TLS and httpBase points at http://) → say
    //     "unavailable" once and stop. Deliberately no retry loop: on the
    //     https: → http: hop the browser blocks the request every time.
    (function(){
      var verEl = document.getElementById('ver');
      function unavailable(){ verEl.textContent = 'Firmware version unavailable'; }
      fetch(httpBase + '/api/status', {credentials:'same-origin'})
        .then(function(r){
          // D13: same login target as the upload path — this page and /update
          // both live on port 80, so follow httpBase when it is set.
          if (r.status === 401) {
            window.location.href = (httpBase || '') + '/login';
            throw new Error('__auth__');
          }
          if (!r.ok) throw new Error('HTTP ' + r.status);
          return r.json();
        })
        .then(function(d){
          verEl.textContent = (d && d.version)
            ? ('Current firmware: ' + d.version) : 'Firmware version unavailable';
        })
        .catch(function(e){
          if (e && e.message === '__auth__') return;
          unavailable();
        });
    })();

    function upload(){
      var file = document.getElementById('bin').files[0];
      if(!file){ document.getElementById('status').innerText='No file selected.'; return; }
      var btn  = document.getElementById('btn');
      var bar  = document.getElementById('bar');
      var wrap = document.getElementById('bar-wrap');
      var stat = document.getElementById('status');
      btn.disabled=true; wrap.style.display='block'; stat.innerText='Uploading...';

      // Use FormData so WebServer receives a proper multipart/form-data upload.
      // Do NOT set Content-Type manually — the browser must set it with the
      // multipart boundary string, otherwise the server cannot find the payload.
      var fd = new FormData();
      fd.append('firmware', file, file.name);

      var xhr = new XMLHttpRequest();
      xhr.open('POST', httpBase + '/update', true);
      xhr.upload.onprogress=function(e){
        if(e.lengthComputable){
          var p=Math.round(e.loaded/e.total*100);
          bar.style.width=p+'%'; stat.innerText='Uploading... '+p+'%';
        }
      };
      xhr.onload=function(){
        if(xhr.status===200){
          bar.style.width='100%';
          var secs=6;
          function tick(){
            // Plain ASCII "..." here, deliberately. This string used to carry a
            // literal U+2026 ellipsis while the page declared no charset, so
            // browsers fell back to their locale default (usually windows-1252)
            // and rendered its three UTF-8 bytes as mojibake -- the "Rebooting"
            // message looked corrupted. The meta charset added above fixes the
            // root cause; ASCII here means this particular string cannot break
            // again regardless of how the encoding is resolved.
            stat.innerText='Success! Rebooting... returning to dashboard in '+secs+' s';
            if(secs<=0){ window.location.href='/'; return; }
            secs--;
            setTimeout(tick,1000);
          }
          tick();
        } else if(xhr.status===401){
          // UX-5: the OTA session expired mid-upload — /update is HTTP-only, so
          // send the user to the port-80 login rather than showing a raw 401.
          stat.innerText='Session expired — signing in again...';
          window.location.href = (httpBase || '') + '/login';
        } else {
          stat.innerText='Failed (HTTP '+xhr.status+'): '+xhr.responseText;
          btn.disabled=false;
        }
      };
      xhr.onerror=function(){ stat.innerText='Network error.'; btn.disabled=false; };
      xhr.send(fd);
    }
  </script>
</body>
</html>
)rawliteral";

// ---------------------------------------------------------------------------
// Log viewer page — streams serial output via SSE
// ---------------------------------------------------------------------------

const char HTML_LOG[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
  <meta name="apple-mobile-web-app-capable" content="yes">
  <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
  <meta name="theme-color" content="#1a1a2e">
  <title>Serial Log</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;
         padding:16px;display:flex;flex-direction:column;height:100vh;
         padding-top:calc(16px + env(safe-area-inset-top));
         padding-bottom:calc(16px + env(safe-area-inset-bottom));
         padding-left:calc(16px + env(safe-area-inset-left));
         padding-right:calc(16px + env(safe-area-inset-right))}
    h1{color:#e94560;font-size:1.2em;margin-bottom:6px;flex-shrink:0}
    nav{font-size:0.85em;margin-bottom:8px;flex-shrink:0;display:flex;flex-wrap:wrap}
    /* UX-11: 44 px minimum touch target on links and toolbar buttons. */
    nav a{color:#aaa;text-decoration:none;margin-right:14px;
          display:inline-flex;align-items:center;min-height:44px}
    nav a:hover{color:#e94560}
    .toolbar{display:flex;gap:8px;margin-bottom:8px;flex-shrink:0;flex-wrap:wrap}
    button{background:#e94560;color:#fff;border:none;padding:6px 14px;
           border-radius:5px;font-size:0.8em;cursor:pointer;min-height:44px;min-width:64px}
    button.inactive{background:#0f3460}
    #status{font-size:0.75em;color:#888;margin-left:auto;align-self:center}
    #log{flex:1;background:#0a0a1a;border:1px solid #0f3460;border-radius:6px;
         padding:10px;overflow-y:auto;font-family:monospace;font-size:0.78em;
         line-height:1.5;white-space:pre-wrap;word-break:break-all}
    .line-err{color:#e94560}
    .line-warn{color:#ffb300}
    .line-ok{color:#4caf50}
    .line-info{color:#eee}
  </style>
</head>
<body>
  <h1>&#128220; Serial Log</h1>
  <nav><a href="/">&#8592; Dashboard</a></nav>
  <div class="toolbar">
    <button type="button" id="btnPause" onclick="togglePause()">Pause</button>
    <button type="button" onclick="clearLog()">Clear</button>
    <span id="status" role="status" aria-live="polite">Connecting...</span>
  </div>
  <div style="font-size:0.72em;color:#888;margin-bottom:6px">Served over plain
    HTTP on port 80; with HTTPS enabled this page needs its own login.</div>
  <div id="log"></div>
  <script>
    // /api/log/stream is registered on port 80 only — the IDF TLS server has
    // no long-lived SSE story. Keep the stream on http:// if this page was
    // somehow loaded over TLS, and fix the nav links for the same reason.
    var httpBase = (location.protocol === 'https:') ? 'http://' + location.hostname : '';
    if (location.protocol === 'https:') {
      document.querySelectorAll('nav a[href="/log"], nav a[href="/update"]').forEach(function(a){
        a.href = 'http://' + location.hostname + a.getAttribute('href');
      });
    }

    var paused   = false;
    var autoScroll = true;
    var logEl    = document.getElementById('log');
    var statusEl = document.getElementById('status');
    var es       = null;

    function colorClass(line) {
      var l = line.toLowerCase();
      if (l.indexOf('error') >= 0 || l.indexOf('fail') >= 0 || l.indexOf('fault') >= 0)
        return 'line-err';
      if (l.indexOf('warn') >= 0 || l.indexOf('timeout') >= 0)
        return 'line-warn';
      if (l.indexOf('ok') >= 0 || l.indexOf('started') >= 0 || l.indexOf('connected') >= 0)
        return 'line-ok';
      return 'line-info';
    }

    function appendLine(text) {
      if (paused) return;
      var span = document.createElement('span');
      span.className = colorClass(text);
      span.textContent = text;
      logEl.appendChild(span);
      // Keep at most 2000 child nodes to avoid unbounded DOM growth
      while (logEl.childNodes.length > 2000) logEl.removeChild(logEl.firstChild);
      if (autoScroll) logEl.scrollTop = logEl.scrollHeight;
    }

    var lastEvt = 0;
    var esFails = 0;        // consecutive failed/aborted connections
    var esDead  = false;    // true once we've stopped retrying
    var MAX_ES_FAILS = 5;

    function connect() {
      if (esDead) return;
      if (es) es.close();
      es = new EventSource(httpBase + '/api/log/stream');
      lastEvt = Date.now();

      es.onopen = function() {
        statusEl.textContent = 'Connected';
        esFails = 0;            // a successful open clears the retry budget
        lastEvt = Date.now();
      };

      es.onmessage = function(e) {
        // Each SSE message is one line of log output
        lastEvt = Date.now();
        appendLine(e.data + '\n');
      };

      // Server emits "event: ping" every ~20 s while the log is quiet.
      // Not shown in the log — it only feeds the staleness watchdog below.
      es.addEventListener('ping', function() { lastEvt = Date.now(); });

      // UX-6: EventSource hides the HTTP status, so a 401 (session expired)
      // is indistinguishable here from a dropped link — it just errors. The
      // pre-flight below catches the 401 case; this only has to stop the
      // endless 3 s retry loop that otherwise hammers the ESP32 forever.
      es.onerror = function() {
        es.close();
        if (++esFails >= MAX_ES_FAILS) {
          esDead = true;
          statusEl.textContent = 'Disconnected — reload to retry';
          return;
        }
        statusEl.textContent = 'Reconnecting... (' + esFails + '/' + MAX_ES_FAILS + ')';
        setTimeout(connect, 3000);
      };
    }

    // UX-6: pre-flight the session before opening the stream. /api/status is
    // the cheapest authenticated GET and answers 401 with a readable status,
    // which EventSource would have swallowed.
    function startLog() {
      fetch(httpBase + '/api/status', {credentials:'same-origin'})
        .then(function(r){
          if (r.status === 401) { window.location.href = (httpBase || '') + '/login'; return; }
          // Any other status (including a cross-origin failure on the
          // https: → http: hop) is not proof of a dead session — try the
          // stream anyway and let the retry cap above handle it.
          connect();
        })
        .catch(function(){ connect(); });
    }

    // Staleness watchdog: a half-open TCP stream (WiFi drop, device reboot,
    // firewall idle-kill on a routed path) leaves EventSource silently "open"
    // forever — onerror never fires. With server pings every 20 s, more than
    // 45 s of total silence means the stream is dead: force a reconnect.
    setInterval(function() {
      if (esDead) return;
      if (es && lastEvt && Date.now() - lastEvt > 45000) {
        statusEl.textContent = 'Stale - reconnecting...';
        connect();
      }
    }, 5000);

    function togglePause() {
      paused = !paused;
      document.getElementById('btnPause').textContent = paused ? 'Resume' : 'Pause';
      document.getElementById('btnPause').className   = paused ? 'inactive' : '';
      if (!paused) logEl.scrollTop = logEl.scrollHeight;
    }

    function clearLog() { logEl.innerHTML = ''; }

    // Detect manual scroll — disable auto-scroll until user scrolls back to bottom
    logEl.addEventListener('scroll', function() {
      autoScroll = logEl.scrollTop + logEl.clientHeight >= logEl.scrollHeight - 5;
    });

    startLog();
  </script>
</body>
</html>
)rawliteral";

// SSE client state — only one concurrent log viewer supported.
// WiFiClient is the underlying TCP connection; sseReadPos tracks how far
// through the ring buffer we've sent to this client.
static WiFiClient sseClient;
static uint32_t   sseReadPos  = 0;
static bool       sseActive   = false;
// Last time anything was written to the SSE client — drives the idle ping.
static unsigned long sseLastWriteMs = 0;

// ---------------------------------------------------------------------------
// Route handlers
// ---------------------------------------------------------------------------

// GET /log — serves the log viewer page (protected)
void handleLogPage() {
  if (!requireAuth(false)) return;  // HTML route — 302 to /login on no session
  server.send_P(200, "text/html; charset=utf-8", HTML_LOG);
}

// GET /api/log/stream — opens an SSE connection (protected).
// The response headers are sent here; data is pushed from loop() via
// sseFlush() so the connection stays open without blocking handleClient().
void handleLogStream() {
  if (!requireAuth(true)) return;   // SSE/API — 401 on no session
  // Close any existing SSE client before accepting a new one
  if (sseActive) {
    sseClient.stop();
    sseActive = false;
  }

  sseClient  = server.client();
  // Bound how long a single write may block if the viewer's TCP window fills
  // mid-write. Together with the select() probe in sseFlush() this preserves
  // the finding #15 guarantee: a stalled log viewer can't wedge loop().
  {
    struct timeval sndTo = { 0, 200000 };  // 200 ms
    sseClient.setSocketOption(SOL_SOCKET, SO_SNDTIMEO, &sndTo, sizeof(sndTo));
  }
  // Start the SSE response — keep-alive, no content-length
  sseClient.print(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/event-stream\r\n"
    "Cache-Control: no-cache\r\n"
    "Connection: keep-alive\r\n"
    "\r\n"
  );
  // Send the whole buffer so the viewer shows history on connect,
  // then track position for incremental updates
  sseReadPos = (logHead > LOG_BUF_SIZE) ? (logHead - LOG_BUF_SIZE) : 0;
  sseActive  = true;
  sseLastWriteMs = millis();
}

// Zero-timeout select(): true when the SSE socket's send buffer can accept
// data right now. This replaces the earlier availableForWrite() >= 1460 guard:
// core 3.x's NetworkClient does NOT implement availableForWrite(), so the
// Print base-class default returned 0 and the guard skipped EVERY round —
// the stream carried headers and then nothing, forever (the "log page shows
// nothing" bug). select() writability is the real lwIP-level check.
static bool sseSocketWritable() {
  int fd = sseClient.fd();
  if (fd < 0) return false;
  fd_set wfds;
  FD_ZERO(&wfds);
  FD_SET(fd, &wfds);
  struct timeval tv = { 0, 0 };   // poll — never blocks loop()
  return select(fd + 1, nullptr, &wfds, nullptr, &tv) > 0;
}

// Called from loop() — pushes any new log lines to the SSE client.
// Splits the ring buffer content on newlines and emits one SSE message
// per line so the browser receives clean line events.
void sseFlush() {
  if (!sseActive) return;
  if (!sseClient.connected()) {
    sseActive = false;
    return;
  }
  // Don't write unless the socket can take data without blocking. A stalled
  // log viewer must never block the whole WebServer on a slow TCP write
  // (finding #15). If the send buffer is backed up we skip this round and retry
  // next loop(); worst case the viewer misses a few log lines, never the
  // dashboard. The 200 ms SO_SNDTIMEO set at accept bounds the residual risk
  // of a window filling mid-write.
  if (!sseSocketWritable()) return;
  if (sseReadPos >= logHead) {
    // Nothing new. Ping every 20 s of silence — three jobs in one:
    //  - keeps stateful firewalls from silently dropping the idle flow
    //    (cross-VLAN viewers lose quiet streams to FW idle timeouts),
    //  - forces lwIP to detect a black-holed peer (the write eventually
    //    errors → connected() goes false → the single SSE slot is freed),
    //  - feeds the log page's staleness watchdog ("event: ping" listener).
    if (millis() - sseLastWriteMs >= 20000UL) {
      sseClient.print("event: ping\ndata: 1\n\n");
      sseLastWriteMs = millis();
    }
    return;
  }

  // Read up to 256 bytes at a time to keep loop() responsive
  char tmp[257];
  uint32_t newPos = logReadFrom(sseReadPos, tmp, 256);
  if (newPos == sseReadPos) return;

  size_t len = newPos - sseReadPos;
  sseReadPos = newPos;

  // Emit SSE messages — one per line
  // Accumulate partial line across calls using a static buffer
  static char lineBuf[256];
  static size_t lineLen = 0;

  for (size_t i = 0; i < len; i++) {
    char c = tmp[i];
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        sseClient.print("data: ");
        sseClient.print(lineBuf);
        sseClient.print("\n\n");
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
  sseLastWriteMs = millis();  // real lines went out — idle ping timer resets
}

// ---------------------------------------------------------------------------
// Authentication — session-cookie model with form-based login + hard-lock.
// ---------------------------------------------------------------------------
// Threat model: single-user device on a LAN. Need to (a) keep brute-force
// guesses expensive without making the dashboard sluggish, and (b) make
// remote brute-force completely unsurvivable past a small threshold.
//
// Three layers, evaluated in this order on every protected route:
//
// 1. **Session cookie** (`scs=<32-hex>`): on a successful login, the server
//    mints a random 16-byte token, stores it in a small in-memory table,
//    and returns it as `Set-Cookie: scs=...; HttpOnly; SameSite=Lax`.
//    Subsequent requests just look up the cookie in the table. No Digest,
//    no rate-limit logic, no delay — the dashboard's 2 s polling is cheap.
//    Ordinary sessions are RAM-only with a 6 h idle timeout. When the user
//    ticks "keep me signed in", the session is flagged `persistent`: it is
//    mirrored to NVS (key `auth_sess`) so it survives the reboot that
//    happens every time the chargers are power-cycled, and it has no idle
//    timeout — it lives until logout, table eviction, or factory reset.
//
// 2. **Immediate-reject rate limit** on POST /login only: failed attempts
//    increment a per-client-IP counter and set that client's next-allowed
//    time. A subsequent attempt before then is rejected with 429 +
//    Retry-After *immediately* — no vTaskDelay, no blocked WebServer task.
//    Client sees the rate-limit, doesn't time out, dashboard polls (which
//    use the cookie path) keep working uninterrupted.
//
// 3. **State-based hard lock**: after AUTH_HARD_LOCK_THRESHOLD failed
//    attempts FROM ONE CLIENT IP, that IP's slot becomes hard-locked
//    (finding #9 — tracked per IP so one bad LAN device can't lock everyone
//    out). Login attempts from it return 423 Locked until either
//    (a) HARD_LOCK_AUTO_CLEAR_MS elapses, (b) a BOOT-button hold of 3–5 s
//    clears ALL clients, or (c) the device is rebooted. Existing valid
//    sessions are NOT invalidated by the lock — only NEW logins are blocked.
//    Defensible because brute-forcers can't have a valid session (no cookie
//    without the password), so honouring active sessions during a lock can't
//    aid an attacker.
//
// 4. **Device-wide backstop** (audit 2026-07): the per-IP slots are
//    LRU-evicted, so an attacker rotating source addresses could keep getting
//    fresh counters. A global sliding window counts failures across ALL
//    clients; AUTH_GLOBAL_FAIL_MAX failures within AUTH_GLOBAL_WINDOW_MS
//    trips a lock on new logins from everyone for HARD_LOCK_AUTO_CLEAR_MS.
//    Cleared by the same BOOT-button hold / reboot as the per-IP lock.
//
// Login form vs Digest: the form sends credentials in plaintext over HTTP
// (same as every consumer router / embedded device login page on a LAN).
// The advantage is that password managers (Bitwarden, etc.) can autofill
// standard <input type=password> fields; they cannot autofill Digest
// browser dialogs. Session security is unchanged post-login.
//
// History: an earlier Digest design required the Authorization header to
// be round-tripped on every /login, and a vTaskDelay-based rate-limiter
// blocked the WebServer task and made the dashboard sluggish. Both have
// been replaced by this form + cookie design.
// ---------------------------------------------------------------------------


// Login rate-limit + hard-lock state — tracked PER CLIENT IP (finding #9) so
// one misbehaving LAN device can't lock everyone else out of new logins.
// Each slot holds the back-off/hard-lock state for one client address. The
// table is small and LRU-evicted; an unknown/undeterminable IP (0) shares a
// single slot, which only ever makes the limit STRICTER, never weaker.
static const uint8_t AUTH_GRACE                = 2;   // free attempts before back-off
static const uint8_t AUTH_HARD_LOCK_THRESHOLD  = 5;   // failures that trip the hard lock
static const unsigned long HARD_LOCK_AUTO_CLEAR_MS = 15UL * 60UL * 1000UL;

// AuthIpSlot is defined in auth_types.h (included at the top) so the Arduino
// auto-prototype pass sees the type before it emits prototypes for the helpers
// that take/return it.
static const int  MAX_AUTH_IPS = 8;
static AuthIpSlot authIpSlots[MAX_AUTH_IPS];

// Device-wide failure backstop (audit 2026-07). The per-IP slots above are
// LRU-evicted, and a freshly (re)allocated slot starts with failCount=0 — so
// an attacker rotating through >MAX_AUTH_IPS source addresses could reset
// their counters indefinitely and never trip the per-IP hard lock. This
// global sliding window counts ALL failed logins regardless of source: too
// many failures across the whole device within the window trips a global
// lock on NEW logins (existing sessions unaffected, same rationale as the
// per-IP lock). Cleared by the BOOT-button hold, reboot, or auto-expiry.
// All three variables are guarded by authMutex.
static const uint8_t       AUTH_GLOBAL_FAIL_MAX  = 15;      // failures within window
static const unsigned long AUTH_GLOBAL_WINDOW_MS = 60000UL; // 60 s sliding window
static uint8_t       authGlobalFails       = 0;
static unsigned long authGlobalWindowStart = 0;
static unsigned long authGlobalLockUntil   = 0;  // 0 = no global lock active

// Constant-time string compare (finding #10) — no early-out on first mismatch,
// so an attacker can't learn how many leading characters were correct from the
// response timing. The loop length is the SECRET's length (constant per build),
// not the attacker-supplied input's, so timing doesn't vary with their guess.
static bool authConstTimeStrEqual(const char* a, const char* b) {
  size_t la = strlen(a), lb = strlen(b);
  // M3: this was `(uint8_t)((la - lb) | (lb - la))`. Both operands are size_t,
  // so the cast keeps only the low byte — and when la - lb is any nonzero
  // multiple of 256 BOTH low bytes are zero, leaving diff == 0. Since the loop
  // below only covers lb bytes, the correct secret followed by exactly 256
  // extra characters compared equal. Not a bypass (the secret is still
  // required) but the length guard did nothing in that case. Reduce the
  // comparison to a single branchless 0/1 instead: it is independent of the
  // secret's content, so no timing signal is introduced.
  uint8_t diff = (la == lb) ? 0 : 1;
  for (size_t i = 0; i < lb; i++) {
    uint8_t ca = (i < la) ? (uint8_t)a[i] : 0;
    diff |= (uint8_t)(ca ^ (uint8_t)b[i]);
  }
  return diff == 0;
}

// Constant-time compare of two 32-char session tokens (caller has already
// length-checked to 32; length is not secret).
static bool authConstTimeTokenEqual(const char* a, const char* b) {
  uint8_t diff = 0;
  for (int i = 0; i < 32; i++) diff |= (uint8_t)(a[i] ^ b[i]);
  return diff == 0;
}

// Session store — fixed-size RAM table. Slots flagged `persistent` are ALSO
// mirrored to NVS so they survive a reboot (the controller cold-boots every
// time the chargers are powered on). Non-persistent slots behave as before:
// RAM-only, 6 h idle timeout.
struct AuthSession {
  char          token[33];     // 32 hex + NUL; token[0]=='\0' → slot free
  unsigned long lastUsedMs;
  bool          persistent;    // true → "keep me signed in": NVS-backed, no idle expiry
  uint32_t      mintedEpoch;   // SEC-5: wall-clock mint time, 0 = unknown (no NTP)
};
static const int           MAX_SESSIONS              = 4;
static const unsigned long SESSION_IDLE_TIMEOUT_MS   = 6UL * 60UL * 60UL * 1000UL; // 6 h

// SEC-5(a) — absolute lifetime cap for "keep me signed in" sessions.
//
// Persistent sessions have no idle timeout and survive reboots, so without
// this they lived forever. millis() can't express that (it restarts at every
// power-on, which on this device is every charge session), so the cap is
// measured in WALL-CLOCK time: the mint epoch is stored next to the token in
// NVS as "token:epoch" and checked against time(nullptr) on load and on every
// touch. Anything older than 30 days is forgotten.
//
// Documented limitation: if NTP never becomes valid — an AP-mode-only device,
// or a LAN with no route out — time(nullptr) stays near 0, no epoch can be
// recorded or compared, and persistent sessions do NOT expire. That is the
// pre-existing behaviour, not a regression; logout, eviction, the BOOT-button
// WiFi reset and factory reset all still clear them.
static const uint32_t SESSION_PERSIST_MAX_AGE_S = 30UL * 24UL * 60UL * 60UL;  // 30 days
// time(nullptr) below this means the clock was never set (NTP not yet valid).
static const uint32_t NTP_EPOCH_VALID_MIN = 1000000000UL;  // 2001-09-09

// Current wall-clock epoch, or 0 when the clock has not been set by NTP yet.
static uint32_t authNowEpoch() {
  time_t t = time(nullptr);
  return ((uint32_t)t >= NTP_EPOCH_VALID_MIN) ? (uint32_t)t : 0;
}

// True when `mintedEpoch` is old enough that the session must be dropped.
// Returns false whenever either clock reading is unavailable — fail open on
// time, never on credentials.
static bool authPersistExpired(uint32_t mintedEpoch) {
  if (mintedEpoch == 0) return false;
  uint32_t now = authNowEpoch();
  if (now == 0) return false;
  return (now > mintedEpoch) && ((now - mintedEpoch) > SESSION_PERSIST_MAX_AGE_S);
}
static AuthSession         authSessions[MAX_SESSIONS];

// Set-Cookie Max-Age values. Short = ordinary login (6 h, matches the RAM
// idle timeout). Long = "keep me signed in" (30 days). HttpOnly server-set
// cookies are not subject to Safari ITP's 7-day script-cookie cap, so the
// 30-day value sticks for the iOS home-screen shortcut.
static const long SESSION_COOKIE_MAXAGE_SHORT = 21600L;     // 6 h
static const long SESSION_COOKIE_MAXAGE_LONG  = 2592000L;   // 30 days
// NVS key holding the comma-joined list of persistent session tokens.
static const char* NVS_KEY_AUTH_SESS = "auth_sess";

// Mutex protecting authSessions[] and the per-IP authIpSlots[] lockout table.
// Once HTTPS is enabled the same auth state is touched by BOTH the Arduino
// WebServer task (Core 1) AND the IDF esp_https_server worker thread, so
// concurrent mutation is real. Created in setup() before any HTTP/HTTPS server
// starts. Hold for short critical sections only — the table scans are
// O(MAX_SESSIONS) / O(MAX_AUTH_IPS) = trivial.
static SemaphoreHandle_t authMutex = nullptr;

static inline bool authLock() {
  // pdMS_TO_TICKS(200) is generous — actual hold time is microseconds. We
  // accept timeout-failed return rather than block forever to keep the
  // WebServer responsive in the (impossible) case of a stuck holder.
  return authMutex && xSemaphoreTake(authMutex, pdMS_TO_TICKS(200)) == pdTRUE;
}
static inline void authUnlock() {
  if (authMutex) xSemaphoreGive(authMutex);
}

// ---------------------------------------------------------------------------
// Settings mutex (findings #7, #8) — serialises mutation of the shared config
// globals (mqttHost/apSSID/mqttCaCert/… and the NVS writes around them). When
// HTTPS is enabled the Arduino WebServer task AND the IDF httpd task can both
// run a settings/control/TLS POST concurrently, and those globals are plain
// char arrays / Strings (NVS itself is thread-safe; the buffers are not). The
// settings/control/TLS handlers take this around the whole apply; mqttTask
// takes it briefly to snapshot the broker config.
static SemaphoreHandle_t settingsMutex = nullptr;

static inline bool settingsLock() {
  return settingsMutex && xSemaphoreTake(settingsMutex, pdMS_TO_TICKS(200)) == pdTRUE;
}
static inline void settingsUnlock() {
  if (settingsMutex) xSemaphoreGive(settingsMutex);
}

// Set by the settings handlers when broker config changed; consumed by
// mqttTask, which is the ONLY task allowed to touch mqttClient (finding #7).
// Replaces the old cross-task mqttClient.disconnect() calls from the web tasks.
static volatile bool mqttReconnectRequested = false;

// ---------------------------------------------------------------------------
// Capped request-body collection for port-80 POST routes (audit 2026-07,
// reworked into RawBodyHandler 2026-09 — SEC-1).
//
// The Arduino WebServer's default body handling malloc()s the ENTIRE request
// body (Content-Length sized) into RAM before the route handler — and thus
// its auth check — ever runs, so an unauthenticated client could POST a huge
// body and force large allocations. Every body-consuming POST/DELETE route is
// therefore served by RawBodyHandler (defined next to OtaUpdateHandler, near
// setup()), which declares canRaw and collects the body in RAW streaming mode:
// it arrives in fixed ~1.4 KB chunks and we accumulate at most RAW_BODY_CAP
// bytes into g_rawBody. Anything larger is drained off the socket through the
// fixed chunk buffer and discarded; the completion handler answers 413.
// Unmatched URIs are covered by WebBodyGuardHandler (registered last in
// setup()).
//
// Side effect of RAW mode: the WebServer no longer parses the body into
// server.arg() — completion handlers read g_rawBody directly (JSON routes)
// or parse it with parseKVPairs() from https_ctx.h (form routes: /login,
// /save). Query-string args are also NOT parsed in RAW mode; none of these
// POST routes use them.
//
// Single-threaded by construction: the WebServer serves one client at a time
// from loop(), so one static buffer suffices. RAW_START resets it per request.
//
// multipart/form-data on these routes: RawBodyHandler::canUpload() returns
// false, so the core streams-and-discards the parts and RawBodyHandler::handle()
// answers 415 without ever invoking the route function. That both closes the
// SEC-1 null-deref (see the class comment) and keeps the route from acting on
// a body it never captured. The core's _parseForm() still buffers non-file
// FIELD values into an unbounded String before handle() runs — that residual
// heap-pressure gap can only be closed by patching the bundled WebServer
// library; accepted for a LAN device whose worst case is a reboot into
// charging-off defaults.
// ---------------------------------------------------------------------------
static const size_t RAW_BODY_CAP = 8192;
static String g_rawBody;
static bool   g_rawTooLarge = false;

// Standardised response strings for the locked / rate-limited paths so both
// HTTP and HTTPS handlers say the same thing.
static const char* AUTH_LOCK_MSG =
  "Locked: too many failed login attempts.\n\n"
  "To unlock: hold the BOOT button on the device for 3-5 seconds, "
  "reboot the device, or wait for the cooldown to expire.\n";
static const char* AUTH_RATE_LIMIT_MSG =
  "Too many failed attempts. Please wait and try again.\n";

// UX-10 — the 423 (hard lock) and 429 (rate limit) responses used to be raw
// text/plain dumps of the two constants above: a white page of unstyled text
// with no indication of how long the wait actually is, reached from a styled
// login form. Both paths now re-render HTML_LOGIN with a banner that states
// the reason AND the countdown, taken from the same value that populates the
// Retry-After header, so the page and the header can never disagree. Status
// codes and headers are unchanged — only the body is.
//
// Size of the buffer the fragment is built into. The widest fragment is the
// hard-lock one at 251 bytes: AUTH_LOCK_MSG's 179 plus 72 of headline and
// markup. 320 leaves comfortable headroom without pretending to a budget the
// page buffer cannot honour (D6 — the earlier 384 paired with a 2048-byte page
// buffer described an invariant that was arithmetically false: HTML_LOGIN
// alone formats to ~1668 bytes, so 2048 could not have absorbed a 384-byte
// fragment. The page buffers are 2304 for exactly that reason: 1668 + 320
// = 1988, with 316 to spare. The handlers check snprintf's return anyway —
// see AUTH_PAGE_BUF below — so a future edit to either constant or to the page
// itself gets a log line instead of a silently half-rendered form.)
static const size_t AUTH_FRAG_BUF = 320;
// Size of the buffer the whole login page is formatted into. See the
// arithmetic above; keep the two in step if HTML_LOGIN grows.
static const size_t AUTH_PAGE_BUF = 2304;

// Render a Retry-After second count as something a human reads at a glance.
// Under a minute stays in seconds; a minute-plus but under two shows both
// parts; anything longer rounds UP to whole minutes (the tail of a 15-minute
// hard lock reads "15 min", not "14 min 57 s" — the extra precision is noise,
// and rounding up is what guarantees the page never advises coming back before
// the lock has actually cleared).
static void authFormatRetry(char* out, size_t outSz, unsigned long sec) {
  if (sec == 0)        snprintf(out, outSz, "a moment");
  else if (sec < 60)   snprintf(out, outSz, "%lu s", sec);
  else if (sec < 120)  snprintf(out, outSz, "1 min %lu s", sec - 60);
  else                 snprintf(out, outSz, "%lu min", (sec + 59) / 60);
}

// Build the login-page error fragment for a locked / rate-limited response.
// Every byte written here is a firmware constant or a number this device
// computed — no request data reaches it, which is why HTML_LOGIN can stay
// escaping-free. `hardLocked` picks the 423 wording, otherwise the 429 one.
static void authBuildRetryFrag(char* out, size_t outSz, bool hardLocked,
                               unsigned long retrySec) {
  char when[32];
  authFormatRetry(when, sizeof(when), retrySec);
  if (hardLocked)
    snprintf(out, outSz,
             "<p class='err'>&#128274; Locked &mdash; try again in %s.</p>"
             "<p class='hint'>%s</p>", when, AUTH_LOCK_MSG);
  else
    snprintf(out, outSz,
             "<p class='err'>&#9203; Too many attempts &mdash; try again in %s.</p>"
             "<p class='hint'>%s</p>", when, AUTH_RATE_LIMIT_MSG);
}

// Format HTML_LOGIN into `out` around the given fragment. Single point of
// truth for the truncation check (D6) so all four login handlers — HTTP and
// HTTPS, GET and POST — get it without four copies that could drift apart.
// A truncated page is a half-rendered form, which fails in a confusing way for
// the user and silently for us; the log line is what makes it diagnosable.
static void authRenderLoginPage(char* out, size_t outSz, const char* frag) {
  int n = snprintf(out, outSz, HTML_LOGIN, frag);
  if (n < 0 || (size_t)n >= outSz)
    LOG("[AUTH] login page truncated (%d)\n", n);
}

// D16 — buffers the HTTPS login handlers format into, deliberately NOT on the
// stack. esp_http_server services requests from a single worker task, one at a
// time, so the two HTTPS login handlers can never run concurrently and one
// shared pair is safe. Worth the ~2.6 KB of BSS because cfg.httpd.stack_size is
// 10240 and the live TLS session already sits on that stack — the page
// formatting is the part that does not need to be there. The HTTP twins keep
// their buffers automatic: loopTask has the room and no TLS state competing
// for it.
// ASSUMPTION: single httpd worker task. If esp_http_server is ever configured
// for more than one, these must go back on the stack or gain a mutex.
static char g_httpsLoginPage[AUTH_PAGE_BUF];
static char g_httpsLoginFrag[AUTH_FRAG_BUF];

// LoginOutcome / LoginResult — returned by tryLogin(). Definitions live in
// auth_types.h to dodge the Arduino IDE 2.x auto-prototype generator (which
// emits `static LoginOutcome tryLogin(...);` near the top of the file,
// before any inline struct here would be visible). On OK, `token` carries
// the freshly-minted 32-hex session token for the Set-Cookie header. On
// LOCKED / RATE_LIMITED, `retryAfterSec` populates the Retry-After header.

// Format a Set-Cookie value for a session token. `secure` adds the Secure
// flag (set only when responding over HTTPS). `maxAgeSec` is the cookie
// lifetime — SESSION_COOKIE_MAXAGE_SHORT for an ordinary login or
// SESSION_COOKIE_MAXAGE_LONG when "keep me signed in" was ticked.
static void sessionBuildSetCookie(char* out, size_t outSz, const char* token,
                                  bool secure, long maxAgeSec) {
  snprintf(out, outSz,
           "scs=%s; Path=/; HttpOnly;%s SameSite=Lax; Max-Age=%ld",
           token, secure ? " Secure;" : "", maxAgeSec);
}
// Format a Set-Cookie value for clearing the session (logout). SameSite must
// match the live cookie (Lax) so the browser pairs them up correctly across
// versions.
static void sessionBuildClearCookie(char* out, size_t outSz, bool secure) {
  snprintf(out, outSz,
           "scs=; Path=/; HttpOnly;%s SameSite=Lax; Max-Age=0",
           secure ? " Secure;" : "");
}

// Returns seconds to wait before the next login attempt is permitted, given
// how many consecutive failures have already accumulated. Schedule:
//   1st-2nd : 0 s    (free)
//   3rd     : 2 s
//   4th     : 5 s
//   5th     : 15 s   (also trips hard lock — but the rate-limit value still applies)
//   6th     : 30 s
//   7th+    : 60 s
static unsigned long authDelaySec(uint8_t fails) {
  if (fails < AUTH_GRACE) return 0;
  switch (fails) {
    case 2:  return 2;
    case 3:  return 5;
    case 4:  return 15;
    case 5:  return 30;
    default: return 60;
  }
}

// Find (or allocate) the lockout slot for a client IP. Internal — caller must
// hold authMutex. Reuses an existing slot for the IP, else a free slot, else
// evicts the least-recently-used one. Never returns nullptr.
static AuthIpSlot* authIpSlot_nolock(uint32_t ip) {
  unsigned long now = millis();
  int free = -1, lru = -1;
  for (int i = 0; i < MAX_AUTH_IPS; i++) {
    if (authIpSlots[i].inUse && authIpSlots[i].ip == ip) {
      authIpSlots[i].lastSeenMs = now;
      return &authIpSlots[i];
    }
    if (!authIpSlots[i].inUse) {
      if (free < 0) free = i;
    } else if (lru < 0 || authIpSlots[i].lastSeenMs < authIpSlots[lru].lastSeenMs) {
      lru = i;
    }
  }
  int idx = (free >= 0) ? free : lru;
  AuthIpSlot& s = authIpSlots[idx];
  s.inUse = true; s.ip = ip; s.failCount = 0; s.nextAllowedAttemptMs = 0;
  s.hardLocked = false; s.hardLockSinceMs = 0; s.lastSeenMs = now;
  return &s;
}

// Auto-clear one slot's hard lock if its cooldown has elapsed. Internal —
// caller must hold authMutex.
static void hardLockMaybeAutoExpire_nolock(AuthIpSlot* s) {
  if (!s->hardLocked) return;
  if ((millis() - s->hardLockSinceMs) >= HARD_LOCK_AUTO_CLEAR_MS) {
    s->hardLocked           = false;
    s->failCount            = 0;
    s->nextAllowedAttemptMs = 0;
    LOG("[AUTH] Hard lock auto-cleared after cooldown (ip slot)\n");
  }
}

// Device-wide backstop: is the global login lock active? Internal — caller
// must hold authMutex. Lazily clears an expired lock (mirrors the per-IP
// auto-expire pattern). remSecOut = seconds until auto-clear when locked.
static bool globalLockActive_nolock(unsigned long now, unsigned long& remSecOut) {
  remSecOut = 0;
  if (authGlobalLockUntil == 0) return false;
  long rem = (long)(authGlobalLockUntil - now);   // rollover-safe signed diff
  if (rem <= 0) {
    authGlobalLockUntil = 0;
    authGlobalFails     = 0;
    LOG("[AUTH] Global login lock auto-cleared after cooldown\n");
    return false;
  }
  remSecOut = ((unsigned long)rem + 999UL) / 1000UL;
  return true;
}

// Count one failed login into the device-wide sliding window. Internal —
// caller must hold authMutex. Returns true if this failure tripped the
// global lock (caller logs after releasing the mutex).
static bool globalFailCount_nolock(unsigned long now) {
  if (authGlobalFails == 0 ||
      (now - authGlobalWindowStart) > AUTH_GLOBAL_WINDOW_MS) {
    authGlobalWindowStart = now;
    authGlobalFails       = 1;
    return false;
  }
  if (authGlobalFails < 255) authGlobalFails++;
  if (authGlobalFails >= AUTH_GLOBAL_FAIL_MAX && authGlobalLockUntil == 0) {
    authGlobalLockUntil = now + HARD_LOCK_AUTO_CLEAR_MS;
    if (authGlobalLockUntil == 0) authGlobalLockUntil = 1;  // 0 = "off" sentinel
    return true;
  }
  return false;
}

// Atomic read of "is this client's hard lock active?" + remaining seconds.
// Used by the /login GET handlers so the auto-expire + read pair can't race
// against a concurrent failed-login attempt that just tripped the lock.
// Also reports the device-wide global lock (whichever lasts longer).
static bool peekLockStatus(uint32_t ip, unsigned long& remSecOut) {
  remSecOut = 0;
  if (!authLock()) return false;
  AuthIpSlot* s = authIpSlot_nolock(ip);
  hardLockMaybeAutoExpire_nolock(s);
  bool locked = s->hardLocked;
  if (locked) {
    unsigned long lockedForMs = millis() - s->hardLockSinceMs;
    unsigned long remMs       = (HARD_LOCK_AUTO_CLEAR_MS > lockedForMs)
                                  ? (HARD_LOCK_AUTO_CLEAR_MS - lockedForMs) : 0;
    remSecOut = (remMs + 999UL) / 1000UL;
  }
  unsigned long gRem = 0;
  if (globalLockActive_nolock(millis(), gRem)) {
    locked = true;
    if (gRem > remSecOut) remSecOut = gRem;
  }
  authUnlock();
  return locked;
}

// True if ANY client currently has an active hard lock or accumulated
// failures. Used by the BOOT-button handler to decide whether a manual clear
// has anything to do.
static bool authAnyLockOrFails() {
  if (!authLock()) return false;
  bool any = (authGlobalFails > 0 || authGlobalLockUntil != 0);
  for (int i = 0; !any && i < MAX_AUTH_IPS; i++) {
    if (authIpSlots[i].inUse &&
        (authIpSlots[i].hardLocked || authIpSlots[i].failCount > 0)) {
      any = true;
    }
  }
  authUnlock();
  return any;
}

// Manual clear path (called from BOOT button handler). Clears the lockout state
// for ALL client IPs — the physical button is a deliberate global unlock.
static void hardLockManualClear(const char* reason) {
  if (!authLock()) return;
  bool any = false;
  for (int i = 0; i < MAX_AUTH_IPS; i++) {
    AuthIpSlot& s = authIpSlots[i];
    if (s.inUse && (s.hardLocked || s.failCount > 0 || s.nextAllowedAttemptMs)) {
      s.hardLocked = false; s.failCount = 0; s.nextAllowedAttemptMs = 0;
      any = true;
    }
  }
  // Device-wide backstop clears with the same deliberate physical action.
  if (authGlobalFails > 0 || authGlobalLockUntil != 0) {
    authGlobalFails       = 0;
    authGlobalWindowStart = 0;
    authGlobalLockUntil   = 0;
    any = true;
  }
  authUnlock();
  if (any) LOG("[AUTH] Hard lock + fail counters cleared for all clients (%s)\n", reason);
}

// Pull the session token out of a Cookie header string. Returns empty
// string if the string doesn't contain an scs= entry.
static String sessionParseCookieTokenStr(const String& cookie) {
  // L4: previously this took the FIRST "scs=" occurrence and, if it wasn't at a
  // cookie boundary, gave up entirely. Any unrelated cookie whose *value*
  // happened to contain the substring "scs=" (e.g. `theme=scs=dark; scs=<tok>`)
  // therefore locked the user out. Keep scanning past non-boundary matches
  // instead of returning empty on the first one.
  int from = 0;
  while (true) {
    int idx = cookie.indexOf("scs=", from);
    if (idx < 0) return String();
    // Accept only at start-of-header or immediately after a "; " separator,
    // so "xscs=foo" and "theme=scs=dark" don't false-match.
    bool atBoundary = (idx == 0) ||
                      (cookie.charAt(idx - 1) == ' ') ||
                      (cookie.charAt(idx - 1) == ';');
    if (!atBoundary) { from = idx + 4; continue; }
    int valStart = idx + 4;
    int valEnd   = cookie.indexOf(';', valStart);
    if (valEnd < 0) valEnd = cookie.length();
    String token = cookie.substring(valStart, valEnd);
    token.trim();
    return token;
  }
}

// Pull the session token out of the WebServer request's Cookie header.
static String sessionParseCookieToken() {
  if (!server.hasHeader("Cookie")) return String();
  return sessionParseCookieTokenStr(server.header("Cookie"));
}

// Forward declaration — the touch path below re-syncs NVS when it drops a
// persistent session that hit the 30-day cap, and the definition sits further
// down. The Arduino IDE's auto-prototype pass does not emit prototypes for
// `static` free functions, so this has to be written out by hand.
static void sessionsPersistToNvs_nolock();

// Core session-touch logic — takes a token string directly. Internal — caller
// must hold authMutex. The public wrapper below takes the lock.
static bool sessionTouchOrFailToken_nolock(const String& token) {
  if (token.length() != 32) return false;
  unsigned long now = millis();
  for (int i = 0; i < MAX_SESSIONS; i++) {
    if (authSessions[i].token[0] == '\0') continue;
    // Lazy expiry sweep — persistent ("keep me signed in") sessions never
    // idle-expire; they live until logout, eviction, factory reset, or the
    // 30-day absolute cap below.
    if (!authSessions[i].persistent &&
        (now - authSessions[i].lastUsedMs) > SESSION_IDLE_TIMEOUT_MS) {
      authSessions[i].token[0] = '\0';
      continue;
    }
    // SEC-5(a): absolute age cap for persistent sessions, wall-clock based.
    if (authSessions[i].persistent &&
        authPersistExpired(authSessions[i].mintedEpoch)) {
      authSessions[i].token[0]   = '\0';
      authSessions[i].persistent = false;
      LOG("[AUTH] Session slot %d expired (30-day cap for remembered device)\n", i);
      sessionsPersistToNvs_nolock();   // drop it from the NVS copy too
      continue;
    }
    if (authConstTimeTokenEqual(token.c_str(), authSessions[i].token)) {
      authSessions[i].lastUsedMs = now;
      // SEC-5(a): a persistent session with no mint epoch came either from a
      // pre-SEC-5 NVS blob or from a mint that happened before NTP was up.
      // Stamp it the first time we see a valid clock so its 30-day cap starts
      // ticking instead of never applying.
      if (authSessions[i].persistent && authSessions[i].mintedEpoch == 0) {
        uint32_t nowEpoch = authNowEpoch();
        if (nowEpoch != 0) {
          authSessions[i].mintedEpoch = nowEpoch;
          sessionsPersistToNvs_nolock();
        }
      }
      return true;
    }
  }
  return false;
}

// Public wrapper — locks authMutex around the table scan.
static bool sessionTouchOrFailToken(const String& token) {
  if (!authLock()) return false;
  bool r = sessionTouchOrFailToken_nolock(token);
  authUnlock();
  return r;
}

// If the request's Cookie maps to a non-expired session slot, refresh its
// lastUsedMs and return true. Otherwise return false. Also lazily evicts
// any session it encounters that's already past the idle timeout.
static bool sessionTouchOrFail() {
  return sessionTouchOrFailToken(sessionParseCookieToken());
}

// HTTPS-path variant: checks the cookie string from the TLS request headers.
static bool sessionTouchOrFailStr(const String& cookieHeader) {
  return sessionTouchOrFailToken(sessionParseCookieTokenStr(cookieHeader));
}

// Mint a new session — find a free slot, or evict. Writes the new token into
// `outToken` (must be ≥33 chars) and flags the slot `persistent`. Internal —
// caller must hold authMutex (the slot scan and snprintf-into-slot must be
// atomic to prevent a concurrent sessionTouchOrFailToken_nolock from matching
// against a half-written token).
//
// Eviction prefers the oldest NON-persistent slot, so a "keep me signed in"
// device isn't silently kicked out by an ordinary login on another device.
// Only if every slot is persistent does it evict the oldest persistent one.
//
// Returns the slot index used, so the caller can log which slot was minted
// without logging any part of the token itself (SEC-12).
static int sessionMint_nolock(char* outToken, bool persistent) {
  unsigned long now = millis();
  int           slot         = -1;
  unsigned long oldestAny    = ULONG_MAX;  int oldestAnySlot = 0;
  unsigned long oldestNp     = ULONG_MAX;  int oldestNpSlot  = -1;
  for (int i = 0; i < MAX_SESSIONS; i++) {
    if (authSessions[i].token[0] == '\0') { slot = i; break; }
    if (authSessions[i].lastUsedMs < oldestAny) {
      oldestAny = authSessions[i].lastUsedMs; oldestAnySlot = i;
    }
    if (!authSessions[i].persistent && authSessions[i].lastUsedMs < oldestNp) {
      oldestNp = authSessions[i].lastUsedMs; oldestNpSlot = i;
    }
  }
  if (slot < 0) {
    slot = (oldestNpSlot >= 0) ? oldestNpSlot : oldestAnySlot;
    LOG("[AUTH] Session table full — evicting slot %d (%s)\n",
        slot, (oldestNpSlot >= 0) ? "non-persistent" : "oldest persistent");
  }
  // Build the token in a local buffer first, then publish atomically to the
  // slot. Prevents a concurrent reader from seeing a half-written token (we
  // hold the mutex, but defensive: a future refactor that drops the lock
  // here won't introduce a subtle TOCTOU).
  char tmp[33];
  for (int i = 0; i < 16; i++) {
    uint8_t b = (uint8_t)(esp_random() & 0xFF);
    snprintf(&tmp[i * 2], 3, "%02x", b);
  }
  tmp[32] = '\0';
  authSessions[slot].lastUsedMs  = now;
  authSessions[slot].persistent  = persistent;
  authSessions[slot].mintedEpoch = authNowEpoch();  // 0 when NTP isn't up yet
  memcpy(authSessions[slot].token, tmp, 33);  // publishes new token
  strncpy(outToken, tmp, 33);
  return slot;
}

// Write every persistent session token to NVS as one comma-joined blob.
// Internal — caller must hold authMutex. Called after any mint or after a
// persistent slot is cleared, so the NVS copy always matches the RAM table.
// NVS putString() de-dups identical writes, so a redundant call is cheap.
//
// Entry format (SEC-5a): "<32-hex token>:<mint epoch>", or the bare token when
// no epoch is known. Writing the bare form in that case keeps the blob
// byte-identical to what pre-SEC-5 builds wrote, and sessionsLoadFromNvs()
// accepts both, so the two formats interoperate in either direction.
static void sessionsPersistToNvs_nolock() {
  String blob;
  for (int i = 0; i < MAX_SESSIONS; i++) {
    if (authSessions[i].token[0] != '\0' && authSessions[i].persistent) {
      if (blob.length()) blob += ',';
      blob += authSessions[i].token;
      if (authSessions[i].mintedEpoch != 0) {
        blob += ':';
        blob += String((unsigned long)authSessions[i].mintedEpoch);
      }
    }
  }
  preferences.putString(NVS_KEY_AUTH_SESS, blob);
}

// Restore persistent session tokens from NVS into the RAM table. Called once
// from setup() before any server task starts — single-threaded, so no lock.
//
// SEC-5(a): each entry may be "token" (written by an older build) or
// "token:epoch". Entries past the 30-day cap are dropped here rather than
// loaded; entries with no epoch are stamped with the CURRENT epoch, so an
// upgrade starts their 30-day clock from first boot on the new firmware
// instead of expiring them all at once.
//
// B7 — when the work here actually happens: this runs early in setup(), long
// before SNTP has answered, so authNowEpoch() returns a usable value only on a
// WARM restart, where the RTC kept running and libc's clock is already set
// (software reset, OTA reboot, settings-save reboot). On a COLD boot the clock
// starts at 0, nothing can be compared or stamped here, and the work falls to
// the touch path instead: sessionTouchOrFailToken_nolock() stamps a 0-epoch
// persistent session and drops an over-age one the first time it is used after
// NTP becomes valid. Either way an expired remembered device stops working
// within one request of the clock being right; this function is just the early
// opportunity, not the only one.
static void sessionsLoadFromNvs() {
  String blob = preferences.getString(NVS_KEY_AUTH_SESS, "");
  if (blob.length() == 0) return;
  unsigned long now      = millis();
  uint32_t      nowEpoch = authNowEpoch();
  int slot = 0, start = 0, dropped = 0, malformed = 0, rewritten = 0;
  while (start < (int)blob.length() && slot < MAX_SESSIONS) {
    int comma = blob.indexOf(',', start);
    if (comma < 0) comma = blob.length();
    String entry = blob.substring(start, comma);
    entry.trim();
    start = comma + 1;
    // An empty field (a stray or trailing comma) is nothing to salvage and
    // nothing to complain about — skip it without counting it as corruption,
    // so it can't trigger a pointless NVS rewrite on every boot.
    if (entry.length() == 0) continue;

    String   tok   = entry;
    uint32_t epoch = 0;
    int      colon = entry.indexOf(':');
    if (colon >= 0) {
      tok   = entry.substring(0, colon);
      epoch = (uint32_t)strtoul(entry.substring(colon + 1).c_str(), nullptr, 10);
      tok.trim();
    }
    // B8: length alone is not enough — mint only ever produces LOWERCASE hex,
    // so anything else in the blob is corruption or a hand-edited NVS value.
    // Reject it rather than load a token that can never legitimately match.
    //
    // C8: the test is spelled out rather than using isxdigit(), which also
    // accepts A-F and so contradicted the comment: an uppercased copy of a real
    // token would have loaded into a slot where the constant-time compare could
    // never match it, silently burning one of the four session slots until the
    // 30-day cap expired it. A rejected entry is now COUNTED, not silently
    // skipped: the count is what forces the rewrite at the end of this
    // function, so the junk does not survive into the next boot (and every
    // boot after that) unnoticed.
    bool hexOk = (tok.length() == 32);
    for (int c = 0; c < 32 && hexOk; c++) {
      char ch = tok.charAt(c);
      if (!((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f'))) hexOk = false;
    }
    if (!hexOk) { malformed++; continue; }

    if (authPersistExpired(epoch)) { dropped++; continue; }
    if (epoch == 0 && nowEpoch != 0) { epoch = nowEpoch; rewritten++; }

    strncpy(authSessions[slot].token, tok.c_str(), 33);
    authSessions[slot].token[32]   = '\0';
    authSessions[slot].lastUsedMs  = now;
    authSessions[slot].persistent  = true;
    authSessions[slot].mintedEpoch = epoch;
    slot++;
  }
  if (slot > 0)
    LOG("[AUTH] Restored %d persistent session(s) from NVS\n", slot);
  if (dropped > 0)
    LOG("[AUTH] Dropped %d persistent session(s) past the 30-day cap\n", dropped);
  if (malformed > 0)
    LOG("[AUTH] Dropped %d malformed persistent session entr%s from NVS\n",
        malformed, malformed == 1 ? "y" : "ies");
  // Re-sync so a dropped, malformed or freshly-stamped entry is reflected in
  // NVS (C8 — a malformed entry used to be skipped without being counted, so
  // it stayed in the blob and was re-parsed and re-rejected on every boot).
  // Runs single-threaded from setup(), so the _nolock form is correct here.
  if (dropped > 0 || malformed > 0 || rewritten > 0) sessionsPersistToNvs_nolock();
}

// Invalidate the session matching the given token string. Internal — caller
// must hold authMutex.
static void sessionForgetToken_nolock(const String& token) {
  if (token.length() != 32) return;
  for (int i = 0; i < MAX_SESSIONS; i++) {
    if (authSessions[i].token[0] != '\0' &&
        authConstTimeTokenEqual(token.c_str(), authSessions[i].token)) {
      bool wasPersistent = authSessions[i].persistent;
      authSessions[i].token[0]    = '\0';
      authSessions[i].persistent  = false;
      authSessions[i].mintedEpoch = 0;
      LOG("[AUTH] Session slot %d invalidated by logout\n", i);
      // Drop it from the NVS copy too, so it doesn't come back on reboot.
      if (wasPersistent) sessionsPersistToNvs_nolock();
      return;
    }
  }
}

// Public wrapper — locks authMutex around the logout-side slot clear.
static void sessionForgetToken(const String& token) {
  if (!authLock()) return;
  sessionForgetToken_nolock(token);
  authUnlock();
}

// Invalidate the session matching the WebServer request's cookie (logout path).
static void sessionForgetCurrent() {
  sessionForgetToken(sessionParseCookieToken());
}

// HTTPS-path variant.
static void sessionForgetStr(const String& cookieHeader) {
  sessionForgetToken(sessionParseCookieTokenStr(cookieHeader));
}

// SEC-5(b) — invalidate EVERY session, RAM table and NVS copy alike.
//
// Used by the BOOT-button 5-10 s "clear WiFi creds" hold. That gesture is how
// a user hands the device to someone else or recovers it from a network they
// no longer control, and leaving remembered ("keep me signed in") sessions
// alive across it meant the previous owner's browser cookie still opened the
// dashboard on the new network. Factory reset (10 s+) already cleared them via
// preferences.clear(); this makes the 5-10 s hold consistent with it.
//
// Takes authMutex itself. Safe to call from loopTask.
static void sessionsClearAll(const char* reason) {
  if (!authLock()) {
    // Can't take the lock (should be impossible) — still drop the NVS copy so
    // nothing comes back after the reboot that follows this call.
    preferences.remove(NVS_KEY_AUTH_SESS);
    LOG("[AUTH] Session clear (%s): mutex busy, NVS copy removed only\n", reason);
    return;
  }
  int cleared = 0;
  for (int i = 0; i < MAX_SESSIONS; i++) {
    if (authSessions[i].token[0] != '\0') cleared++;
    authSessions[i].token[0]    = '\0';
    authSessions[i].persistent  = false;
    authSessions[i].mintedEpoch = 0;
  }
  preferences.remove(NVS_KEY_AUTH_SESS);
  authUnlock();
  LOG("[AUTH] All %d session(s) invalidated and NVS copy removed (%s)\n",
      cleared, reason);
}

// Gate a request on a valid session. Used by every protected handler.
//   isApi == true  → respond with 401 on failure (JS catches and redirects)
//   isApi == false → respond with 302 → /login (browser navigates directly)
//
// Hard-lock note: this does NOT block authed sessions. Only NEW logins
// (handleLogin) are gated by the hard lock. An attacker can't have a
// session without the password, so respecting active sessions through a
// lock window doesn't widen the attack surface.
static bool requireAuth(bool isApi) {
  if (sessionTouchOrFail()) return true;

  if (isApi) {
    server.send(401, "text/plain", "Authentication required");
  } else {
    server.sendHeader("Location", "/login");
    server.send(302, "text/plain", "");
  }
  return false;
}

// Silent variant for upload chunk handlers (handleOTAUpload). WebServer
// invokes those once per multipart chunk before the POST completion handler
// runs; sending a 401 response mid-upload would corrupt the request stream.
// Returns true iff the request bears a valid session cookie.
static bool authValidNoChallenge() {
  return sessionTouchOrFail();
}

// tryLogin — shared between the HTTP and HTTPS login handlers. Performs all
// auth-state checks and mutations atomically under authMutex:
//   1. lazy-expire any active hard lock
//   2. reject if still hard-locked
//   3. reject if within the per-attempt rate-limit window
//   4. validate credentials
//   5. on success: clear fail counter, mint session token
//   6. on failure: increment counter (with overflow guard), trip lock or
//      schedule next-allowed time with jitter
//
// All counter mutations, lock trips, AND token mints happen under one lock
// acquisition. The caller is just a thin response-dispatcher — see
// handleLoginPost / httpsHandleLoginPost.
static LoginOutcome tryLogin(const String& username, const String& password,
                             bool remember, uint32_t clientIp) {
  LoginOutcome out{};
  out.result = LOGIN_BAD_CREDS;

  if (!authLock()) {
    // Mutex timed out — should never happen with healthy tasks. Treat as a
    // bad-creds response (fail closed) rather than letting the caller assume
    // success or leak any state.
    return out;
  }

  AuthIpSlot* s = authIpSlot_nolock(clientIp);   // per-IP lockout state (#9)
  hardLockMaybeAutoExpire_nolock(s);

  // Device-wide backstop (audit 2026-07): blocks NEW logins from every client
  // when too many failures accumulated across all IPs — closes the "rotate
  // source IPs to evict per-IP slots" gap. Existing sessions stay valid, and
  // the BOOT-button 3-5 s hold clears it just like the per-IP hard lock.
  unsigned long gRem = 0;
  if (globalLockActive_nolock(millis(), gRem)) {
    out.result        = LOGIN_HARD_LOCKED;
    out.retryAfterSec = gRem;
    authUnlock();
    return out;
  }

  if (s->hardLocked) {
    unsigned long lockedForMs = millis() - s->hardLockSinceMs;
    unsigned long remMs       = (HARD_LOCK_AUTO_CLEAR_MS > lockedForMs)
                                  ? (HARD_LOCK_AUTO_CLEAR_MS - lockedForMs) : 0;
    out.result         = LOGIN_HARD_LOCKED;
    out.retryAfterSec  = (remMs + 999UL) / 1000UL;
    authUnlock();
    return out;
  }

  unsigned long now = millis();
  // Rollover-safe deadline check (finding #11): signed difference instead of
  // `now < deadline`, so a millis() wrap during the back-off window can't make
  // the limiter mis-evaluate.
  if (s->failCount >= AUTH_GRACE && (long)(s->nextAllowedAttemptMs - now) > 0) {
    out.result         = LOGIN_RATE_LIMITED;
    out.retryAfterSec  = (s->nextAllowedAttemptMs - now + 999UL) / 1000UL;
    authUnlock();
    return out;
  }

  // Credential check — constant-time (finding #10). Bitwise & (not &&) so both
  // comparisons always run and neither short-circuits on a length/char mismatch.
  bool userOk = authConstTimeStrEqual(username.c_str(), SECRET_OTA_USER);
  bool passOk = authConstTimeStrEqual(password.c_str(), SECRET_OTA_PASS);
  bool ok = userOk & passOk;

  if (!ok) {
    if (s->failCount < 255) s->failCount++;
    // Count into the device-wide window as well (audit 2026-07); log the
    // trip after the mutex is released, matching the per-IP pattern.
    bool gTripped = globalFailCount_nolock(millis());
    if (s->failCount >= AUTH_HARD_LOCK_THRESHOLD && !s->hardLocked) {
      s->hardLocked      = true;
      s->hardLockSinceMs = millis();
      uint8_t fc = s->failCount;
      authUnlock();
      LOG("[AUTH] HARD LOCK tripped after %d failures (one client IP) — manual "
          "unlock required (hold BOOT 3-5s, reboot, or wait %lu min)\n",
          (int)fc, HARD_LOCK_AUTO_CLEAR_MS / 60000UL);
    } else {
      unsigned long delaySec = authDelaySec(s->failCount);
      delaySec += (unsigned long)(esp_random() % 3);  // 0–2 s jitter
      s->nextAllowedAttemptMs = millis() + delaySec * 1000UL;
      uint8_t fc = s->failCount;
      authUnlock();
      LOG("[AUTH] Failed login #%d (one client IP) — next attempt allowed in %lu s\n",
          (int)fc, delaySec);
    }
    if (gTripped) {
      LOG("[AUTH] GLOBAL LOCK tripped: %d failed logins across all clients "
          "within %lu s — new logins blocked for %lu min (hold BOOT 3-5s to clear)\n",
          (int)AUTH_GLOBAL_FAIL_MAX, AUTH_GLOBAL_WINDOW_MS / 1000UL,
          HARD_LOCK_AUTO_CLEAR_MS / 60000UL);
    }
    out.result = LOGIN_BAD_CREDS;
    return out;
  }

  // Success — clear this client's fail state and mint session token, still
  // under lock so a concurrent failed-login attempt can't increment the
  // counter against this just-succeeded user.
  uint8_t prevFails = s->failCount;
  s->failCount            = 0;
  s->nextAllowedAttemptMs = 0;
  s->hardLocked           = false;
  // A successful login also resets the device-wide window — an attacker can't
  // reach this branch, so it only ever forgives the owner's own typos.
  authGlobalFails       = 0;
  authGlobalWindowStart = 0;
  int mintedSlot = sessionMint_nolock(out.token, remember);
  // Always re-sync NVS after a mint: writes the persistent set if `remember`,
  // and also catches the case where minting evicted a persistent slot.
  // putString() de-dups identical blobs so a no-op call costs nothing.
  sessionsPersistToNvs_nolock();
  authUnlock();

  if (prevFails > 0) {
    LOG("[AUTH] Login OK — clearing fail counter (was %d)\n", (int)prevFails);
  }
  // SEC-12: no part of the token goes in the log. The serial console and the
  // /log SSE stream are both readable by anyone who can already reach them,
  // but the log buffer also survives in RAM and gets pasted into bug reports —
  // and 6 of 32 hex characters is 6 characters an attacker no longer has to
  // guess. The slot index identifies the session just as well for debugging.
  LOG("[AUTH] Login OK — session minted in slot %d (%s)\n",
      mintedSlot, remember ? "persistent" : "6 h");
  out.result = LOGIN_OK;
  return out;
}

// HTTPS-path auth gate. isApi behaves the same as requireAuth().
static bool requireAuthCtx(HttpCtx& ctx, bool isApi) {
  if (sessionTouchOrFailStr(ctx.header("Cookie"))) return true;
  if (isApi) {
    ctx.send(401, "text/plain", "Authentication required");
  } else {
    ctx.addRespHdr("Location", "/login");
    ctx.send(302, "text/plain", "");
  }
  return false;
}

// Client IP helpers for the per-IP login lockout (finding #9).
//   webClientIp() — Arduino WebServer (HTTP) path.
//   idfClientIp() — ESP-IDF httpd_ssl (HTTPS) path; reads the socket peer.
// Both return a uint32_t used purely as a table key (its byte order is
// irrelevant as long as it's consistent). 0 means "couldn't determine" and is
// treated as a single shared bucket — that only tightens the limit, never
// loosens it.
static uint32_t webClientIp() {
  return (uint32_t)server.client().remoteIP();
}
static uint32_t idfClientIp(httpd_req_t* req) {
  if (!req) return 0;
  int fd = httpd_req_to_sockfd(req);
  if (fd < 0) return 0;
  struct sockaddr_in6 sa;
  socklen_t sl = sizeof(sa);
  if (lwip_getpeername(fd, (struct sockaddr*)&sa, &sl) != 0) return 0;
  if (sa.sin6_family == AF_INET) {
    return ((struct sockaddr_in*)&sa)->sin_addr.s_addr;
  }
  // IPv4-mapped IPv6 (::ffff:a.b.c.d) — the IPv4 address is the low 4 bytes.
  const uint8_t* b = (const uint8_t*)&sa.sin6_addr;
  return ((uint32_t)b[12])       | ((uint32_t)b[13] << 8) |
         ((uint32_t)b[14] << 16) | ((uint32_t)b[15] << 24);
}

// GET /login — serve the HTML login form.
// Redirects to / if a valid session is already present.
// Shows a locked page if the hard lock is active.
// Shows an error paragraph when the query string contains ?err=1.
void handleLoginGet() {
  if (sessionTouchOrFail()) {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
    return;
  }

  // HTML_LOGIN formats to ~1668 bytes; the widest fragment is 251. See
  // AUTH_PAGE_BUF / AUTH_FRAG_BUF for the arithmetic. authRenderLoginPage()
  // logs if this ever stops being true.
  char buf[AUTH_PAGE_BUF];

  unsigned long lockRemSec = 0;
  if (peekLockStatus(webClientIp(), lockRemSec)) {
    char retryHdr[16];
    snprintf(retryHdr, sizeof(retryHdr), "%lu", lockRemSec);
    server.sendHeader("Retry-After", retryHdr);
    // UX-10: styled page, same 423 and same Retry-After as before.
    char frag[AUTH_FRAG_BUF];
    authBuildRetryFrag(frag, sizeof(frag), /*hardLocked=*/true, lockRemSec);
    authRenderLoginPage(buf, sizeof(buf), frag);
    server.send(423, "text/html; charset=utf-8", buf);
    return;
  }

  const char* errFrag = server.hasArg("err")
    ? "<p class='err'>&#10006; Invalid username or password.</p>"
    : "";
  authRenderLoginPage(buf, sizeof(buf), errFrag);
  server.send(200, "text/html; charset=utf-8", buf);
}

// POST /login — thin response dispatcher around tryLogin(). All auth-state
// mutations happen atomically inside tryLogin under authMutex.
void handleLoginPost() {
  if (sessionTouchOrFail()) {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
    return;
  }

  // RAW body mode (RawBodyHandler): the WebServer no longer parses the
  // urlencoded form into server.arg() — parse the capped g_rawBody here with
  // the same helpers the HTTPS path uses. An oversized body yields empty
  // credentials, which fail closed through tryLogin.
  HttpCtx form;
  parseKVPairs(g_rawBody, form);

  // L1: more fields than args[] holds means we cannot trust that "username" /
  // "password" / "remember" are the ones the user actually sent — reject rather
  // than authenticate against a partially-parsed form.
  if (form.argsOverflow) {
    LOG("[AUTH] Login form had more than %d fields — rejected\n", HTTPS_MAX_ARGS);
    server.send(400, "text/plain", "Malformed login form\n");
    return;
  }

  // The "remember" checkbox only appears in the POST body when ticked.
  bool remember  = form.hasArg("remember");
  LoginOutcome o = tryLogin(form.arg("username"), form.arg("password"),
                            remember, webClientIp());
  char retryHdr[16];
  char cookieHdr[140];

  switch (o.result) {
    // UX-10: one arm for both refusals — identical mechanics, the only
    // differences are the status code and the wording, and both are derived
    // from o.result. Status codes (423 / 429) and Retry-After are unchanged;
    // the body is now the styled login page carrying the same countdown.
    case LOGIN_HARD_LOCKED:
    case LOGIN_RATE_LIMITED: {
      const bool hardLocked = (o.result == LOGIN_HARD_LOCKED);
      snprintf(retryHdr, sizeof(retryHdr), "%lu", o.retryAfterSec);
      server.sendHeader("Retry-After", retryHdr);
      char frag[AUTH_FRAG_BUF];
      authBuildRetryFrag(frag, sizeof(frag), hardLocked, o.retryAfterSec);
      char page[AUTH_PAGE_BUF];
      authRenderLoginPage(page, sizeof(page), frag);
      server.send(hardLocked ? 423 : 429, "text/html; charset=utf-8", page);
      return;
    }

    case LOGIN_BAD_CREDS:
      // Redirect back to the form with an error flag rather than re-serving
      // the page here — avoids a browser "resubmit form?" warning on refresh.
      server.sendHeader("Location", "/login?err=1");
      server.send(303, "text/plain", "");
      return;

    case LOGIN_OK:
      // Cookie attributes set by sessionBuildSetCookie:
      //  - HttpOnly      — JS can't read it (XSS defense)
      //  - SameSite=Lax  — sent on top-level GET navigations (bookmarks, tab
      //                    restore, iOS home-screen shortcut) but NOT on
      //                    cross-site POST (CSRF safe).
      //  - Path=/        — sent on every path
      //  - Max-Age       — 30 days when "keep me signed in" was ticked
      //                    (session is NVS-backed, survives reboot), else 6 h
      //  - No Secure     — plain HTTP on a LAN
      sessionBuildSetCookie(cookieHdr, sizeof(cookieHdr), o.token, /*secure=*/false,
        remember ? SESSION_COOKIE_MAXAGE_LONG : SESSION_COOKIE_MAXAGE_SHORT);
      server.sendHeader("Set-Cookie", cookieHdr);
      server.sendHeader("Location",   "/");
      server.send(302, "text/plain", "");
      return;
  }
}

// POST /logout — invalidates the current session and clears the cookie.
// POST (not GET) so that link previews / prefetchers don't accidentally
// log the user out.
void handleLogout() {
  // SEC-8 — no state changes without a valid token, and nothing is logged in
  // that case. sessionForgetCurrent() → sessionForgetToken_nolock() returns
  // immediately unless the cookie carries a 32-char token that constant-time
  // matches a live slot; only that branch clears a slot, re-writes NVS and
  // emits the "[AUTH] Session slot N invalidated by logout" line. An
  // unauthenticated POST /logout therefore just gets the cookie-clear and the
  // page below, which is the correct response for a browser holding a stale or
  // forged cookie.
  sessionForgetCurrent();
  // Clear the cookie and return a 200 page — NOT a redirect.
  // A redirect to /login would be followed by the browser's cached Digest
  // credentials, silently minting a new session and bouncing back to the
  // dashboard.  Serving a page here breaks that loop: the browser shows
  // "logged out" and the user must take a deliberate action to log back in.
  // SameSite=Lax matches the live cookie minted at login — some browsers
  // pair Set-Cookie clears with the matching SameSite attribute, so using
  // Strict here could leave a stale cookie in the browser. Server-side the
  // session is already gone (sessionForgetCurrent above).
  char clearHdr[140];
  sessionBuildClearCookie(clearHdr, sizeof(clearHdr), /*secure=*/false);
  server.sendHeader("Set-Cookie", clearHdr);
  server.send(200, "text/html; charset=utf-8",
    "<!DOCTYPE html><html><head><title>Logged out</title>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<style>"
    "body{font-family:sans-serif;background:#0a0a1a;color:#ccc;"
    "display:flex;flex-direction:column;align-items:center;"
    "justify-content:center;min-height:100vh;margin:0;text-align:center}"
    "h2{color:#e94560;margin-bottom:8px}"
    "p{color:#aaa;margin:4px 0}"
    "a{display:inline-block;margin-top:24px;padding:10px 28px;"
    "background:#e94560;color:#fff;border-radius:6px;text-decoration:none;"
    "font-weight:bold}"
    "a:hover{background:#c73652}"
    "</style></head><body>"
    "<h2>&#128274; Logged out</h2>"
    "<p>Your session has been cleared.</p>"
    "<a href='/login'>Log in again &rarr;</a>"
    "</body></html>");
}

void handleRoot() {
  if (!requireAuth(false)) return;  // HTML route — 302 to /login on no session
  server.send_P(200, "text/html; charset=utf-8", HTML_DASHBOARD);
}

// GET /settings — settings page (protected)
void handleSettingsPage() {
  if (!requireAuth(false)) return;  // HTML route
  server.send_P(200, "text/html; charset=utf-8", HTML_SETTINGS);
}

// ---------------------------------------------------------------------------
// HTTPS handler functions — mirror the WebServer handlers but use HttpCtx.
// Only the routes that a user needs over HTTPS are duplicated here; OTA
// upload (/update) remains HTTP-only because multipart streaming over a
// manually-parsed TLS connection is impractical.
// ---------------------------------------------------------------------------

// settingsNeedRestart — set by applyApiSettingsBody() when a WiFi change
// requires a reboot. Declared here (before the HTTPS handlers that read it)
// and initialised to false. Do NOT redeclare this below.
static bool settingsNeedRestart = false;

// GET / (HTTPS)
static void httpsHandleRoot(HttpCtx& ctx) {
  if (!requireAuthCtx(ctx, false)) return;
  ctx.sendProgmem(200, "text/html; charset=utf-8", HTML_DASHBOARD);
}

// GET /settings (HTTPS)
static void httpsHandleSettingsPage(HttpCtx& ctx) {
  if (!requireAuthCtx(ctx, false)) return;
  ctx.sendProgmem(200, "text/html; charset=utf-8", HTML_SETTINGS);
}

// GET /login (HTTPS)
static void httpsHandleLoginGet(HttpCtx& ctx) {
  if (sessionTouchOrFailStr(ctx.header("Cookie"))) {
    ctx.addRespHdr("Location", "/");
    ctx.send(302, "text/plain", "");
    return;
  }
  // D16 — shared static buffers, not stack: the TLS session already occupies
  // much of this task's 10240-byte stack. See g_httpsLoginPage.
  unsigned long lockRemSec = 0;
  if (peekLockStatus(idfClientIp(ctx.idfReq), lockRemSec)) {
    char retryHdr[16];
    snprintf(retryHdr, sizeof(retryHdr), "%lu", lockRemSec);
    ctx.addRespHdr("Retry-After", String(retryHdr));
    // UX-10 — same styled 423 page as the HTTP twin (handleLoginGet).
    authBuildRetryFrag(g_httpsLoginFrag, sizeof(g_httpsLoginFrag),
                       /*hardLocked=*/true, lockRemSec);
    authRenderLoginPage(g_httpsLoginPage, sizeof(g_httpsLoginPage),
                        g_httpsLoginFrag);
    ctx.send(423, "text/html; charset=utf-8", String(g_httpsLoginPage));
    return;
  }
  const char* errFrag = ctx.hasArg("err")
    ? "<p class='err'>&#10006; Invalid username or password.</p>" : "";
  authRenderLoginPage(g_httpsLoginPage, sizeof(g_httpsLoginPage), errFrag);
  ctx.send(200, "text/html; charset=utf-8", String(g_httpsLoginPage));
}

// POST /login (HTTPS) — uses the same tryLogin() helper as the HTTP path.
// Only differences from handleLoginPost: ctx.* instead of server.*, and the
// Secure flag on the cookie because the response is always HTTPS.
static void httpsHandleLoginPost(HttpCtx& ctx) {
  if (sessionTouchOrFailStr(ctx.header("Cookie"))) {
    ctx.addRespHdr("Location", "/");
    ctx.send(302, "text/plain", "");
    return;
  }

  // L1 — see handleLoginPost.
  if (ctx.argsOverflow) {
    LOG("[AUTH] Login form had more than %d fields — rejected (TLS path)\n",
        HTTPS_MAX_ARGS);
    ctx.send(400, "text/plain", "Malformed login form\n");
    return;
  }

  bool remember  = ctx.hasArg("remember");
  LoginOutcome o = tryLogin(ctx.arg("username"), ctx.arg("password"),
                            remember, idfClientIp(ctx.idfReq));
  char retryHdr[16];
  char cookieHdr[160];

  switch (o.result) {
    // UX-10 — mirrors handleLoginPost: one arm, both status codes preserved.
    case LOGIN_HARD_LOCKED:
    case LOGIN_RATE_LIMITED: {
      const bool hardLocked = (o.result == LOGIN_HARD_LOCKED);
      snprintf(retryHdr, sizeof(retryHdr), "%lu", o.retryAfterSec);
      ctx.addRespHdr("Retry-After", String(retryHdr));
      // D16 — shared static buffers (see g_httpsLoginPage).
      authBuildRetryFrag(g_httpsLoginFrag, sizeof(g_httpsLoginFrag),
                         hardLocked, o.retryAfterSec);
      authRenderLoginPage(g_httpsLoginPage, sizeof(g_httpsLoginPage),
                          g_httpsLoginFrag);
      ctx.send(hardLocked ? 423 : 429, "text/html; charset=utf-8",
               String(g_httpsLoginPage));
      return;
    }

    case LOGIN_BAD_CREDS:
      ctx.addRespHdr("Location", "/login?err=1");
      ctx.send(303, "text/plain", "");
      return;

    case LOGIN_OK:
      sessionBuildSetCookie(cookieHdr, sizeof(cookieHdr), o.token, /*secure=*/true,
        remember ? SESSION_COOKIE_MAXAGE_LONG : SESSION_COOKIE_MAXAGE_SHORT);
      ctx.addRespHdr("Set-Cookie", String(cookieHdr));
      ctx.addRespHdr("Location",   "/");
      ctx.send(302, "text/plain", "");
      return;
  }
}

// POST /logout (HTTPS)
static void httpsHandleLogout(HttpCtx& ctx) {
  // SEC-8: same no-token-no-state-change guarantee as handleLogout — both go
  // through sessionForgetToken_nolock(). See the note there.
  sessionForgetStr(ctx.header("Cookie"));
  // SameSite=Lax matches the live cookie — see note in handleLogout.
  char clearHdr[160];
  sessionBuildClearCookie(clearHdr, sizeof(clearHdr), /*secure=*/true);
  ctx.addRespHdr("Set-Cookie", String(clearHdr));
  ctx.send(200, "text/html; charset=utf-8",
    "<!DOCTYPE html><html><head><title>Logged out</title>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<style>"
    "body{font-family:sans-serif;background:#0a0a1a;color:#ccc;"
    "display:flex;flex-direction:column;align-items:center;"
    "justify-content:center;min-height:100vh;margin:0;text-align:center}"
    "h2{color:#e94560;margin-bottom:8px}"
    "p{color:#aaa;margin:4px 0}"
    "a{display:inline-block;margin-top:24px;padding:10px 28px;"
    "background:#e94560;color:#fff;border-radius:6px;text-decoration:none;"
    "font-weight:bold}"
    "a:hover{background:#c73652}"
    "</style></head><body>"
    "<h2>&#128274; Logged out</h2>"
    "<p>Your session has been cleared.</p>"
    "<a href='/login'>Log in again &rarr;</a>"
    "</body></html>");
}

// GET /api/status (HTTPS) — identical logic to WebServer version but writes to ctx.
// Forward-declare the function that builds the JSON so we can call it from both paths.
static String buildApiStatusJson();  // defined near handleApiStatus

static void httpsHandleApiStatus(HttpCtx& ctx) {
  if (!requireAuthCtx(ctx, true)) return;
  String json = buildApiStatusJson();
  ctx.send(200, "application/json", json);
}

// GET /api/settings (HTTPS) — forward-declare builder.
static String buildApiSettingsJson();
// NET-13 — defined just above setup(); every ESP.restart() path calls it first
// so the chargers are commanded STOP before the heartbeat disappears.
// Declared here explicitly rather than relying on the IDE's auto-prototype pass,
// which does not emit prototypes for `static` free functions.
static void stopChargerForRestart(const char* reason);

static void httpsHandleApiSettingsGet(HttpCtx& ctx) {
  if (!requireAuthCtx(ctx, true)) return;
  String json = buildApiSettingsJson();
  // B9 — see handleApiSettingsGet.
  int code = json.startsWith("{\"ok\":false") ? 500 : 200;
  ctx.send(code, "application/json", json);
}

// POST /api/settings (HTTPS) — reuse the logic factored out of handleApiSettingsPost.
static String applyApiSettingsBody(const String& body);  // returns JSON ack

static void httpsHandleApiSettingsPost(HttpCtx& ctx) {
  if (!requireAuthCtx(ctx, true)) return;
  if (ctx.bodyTooLarge) {
    ctx.send(413, "application/json", "{\"ok\":false,\"error\":\"Payload too large\"}");
    return;
  }
  if (ctx.body.length() == 0) {
    ctx.send(400, "application/json", "{\"ok\":false,\"error\":\"No body\"}");
    return;
  }
  // NET-12: 503 rather than apply unlocked — see handleApiSettingsPost.
  if (!settingsLock()) {   // serialise vs the HTTP WebServer task (#8)
    ctx.send(503, "application/json", "{\"ok\":false,\"error\":\"busy, retry\"}");
    return;
  }
  String result = applyApiSettingsBody(ctx.body);
  // B12: snapshot inside the lock — see handleApiSettingsPost.
  bool needRestart = settingsNeedRestart;
  settingsUnlock();
  // Determine HTTP status from result JSON
  int code = (result.indexOf("\"ok\":true") >= 0) ? 200 : 400;
  if (result.indexOf("\"error\":\"Payload too large\"") >= 0) code = 413;
  ctx.send(code, "application/json", result);
  // NET-4: reboot only on a successful save — see handleApiSettingsPost.
  if (code == 200 && needRestart) {
    LOG("[SETTINGS] Restarting for WiFi changes (TLS path)...\n");
    // Response is already sent above; the helper's 1.5 s wait replaces the
    // delay that used to sit here and also lets the STOP frame go out.
    stopChargerForRestart("settings save, HTTPS");
    ESP.restart();
  }
}

// POST /api/control (HTTPS) — forward-declare applier.
static String applyApiControlBody(const String& body);

static void httpsHandleApiControl(HttpCtx& ctx) {
  if (!requireAuthCtx(ctx, true)) return;
  if (ctx.bodyTooLarge) {
    ctx.send(413, "application/json", "{\"ok\":false,\"error\":\"Payload too large\"}");
    return;
  }
  if (ctx.body.length() == 0) {
    ctx.send(400, "application/json", "{\"ok\":false,\"error\":\"No body\"}");
    return;
  }
  String result = applyApiControlBody(ctx.body);
  // M1: 503 for "controller busy, nothing applied" — see handleApiControl.
  int code = (result.indexOf("\"ok\":true") >= 0) ? 200
           : (result.indexOf("Controller busy") >= 0) ? 503 : 400;
  ctx.send(code, "application/json", result);
}

// GET /api/cycles (HTTPS) — stream the cycle CSV as a download (audit 2026-07:
// previously HTTP-only, so enabling HTTPS silently lost the dashboard's
// "Cycles" download link). Streamed in chunks — the file can be large.
static void httpsHandleApiCyclesGet(HttpCtx& ctx) {
  if (!requireAuthCtx(ctx, true)) return;
  if (!g_fatReady) { ctx.send(503, "text/plain", "FAT not mounted"); return; }
  File f = FFat.open("/cycles.csv", FILE_READ);
  if (!f) { ctx.send(404, "text/plain", "No data yet"); return; }
  ctx.addRespHdr("Content-Disposition", "attachment; filename=\"cycles.csv\"");
  ctx.flushRespHdrs(200, "text/csv");
  char buf[512];
  while (f.available()) {
    size_t n = f.read((uint8_t*)buf, sizeof(buf));
    if (n == 0) break;
    if (httpd_resp_send_chunk(ctx.idfReq, buf, (ssize_t)n) != ESP_OK) break;
  }
  f.close();
  httpd_resp_send_chunk(ctx.idfReq, nullptr, 0);  // terminate chunked transfer
}

// DELETE /api/cycles (HTTPS) — same semantics as the HTTP handler.
static void httpsHandleApiCyclesDelete(HttpCtx& ctx) {
  if (!requireAuthCtx(ctx, true)) return;
  if (!g_fatReady) { ctx.send(503, "text/plain", "FAT not mounted"); return; }
  FFat.remove("/cycles.csv");
  g_cycleCount = 0;
  ctx.send(200, "text/plain", "Cleared");
  LOG("[FAT] cycles.csv deleted by user (TLS path)\n");
}

// POST /api/tls (HTTPS and HTTP) — cert/key upload.
// Body JSON: {"cert":"<PEM>","key":"<PEM>"} (both required) or {"enabled":bool}.
static void handleApiTlsPost(HttpCtx& ctx) {
  // Authentication
  if (ctx.isWS) { if (!requireAuth(true)) return; }
  else          { if (!requireAuthCtx(ctx, true)) return; }

  // Oversized IDF body (finding #19) — the body was never read, so check the
  // flag, not body.length(), before falling through to "Invalid JSON".
  // The WS path uses the capped RAW collector (audit 2026-07): g_rawTooLarge
  // is its equivalent of ctx.bodyTooLarge.
  if (ctx.isWS ? g_rawTooLarge : ctx.bodyTooLarge) {
    if (ctx.isWS) server.send(413, "application/json", "{\"ok\":false,\"error\":\"Payload too large\"}");
    else ctx.send(413, "application/json", "{\"ok\":false,\"error\":\"Payload too large\"}");
    return;
  }
  String body = ctx.isWS ? g_rawBody : ctx.body;
  // One response sink for both transports — the HTTP and HTTPS paths differ
  // only in which object sends the bytes.
  auto respond = [&](int code, const char* json) {
    if (ctx.isWS) server.send(code, "application/json", json);
    else          ctx.send  (code, "application/json", json);
  };

  DynamicJsonDocument doc(body.length() + 512);
  if (deserializeJson(doc, body)) {
    respond(400, "{\"ok\":false,\"error\":\"Invalid JSON\"}");
    return;
  }

  // B6: there is no "changed but no reboot needed" case — both mutable fields
  // (cert/key and the enable flag) are only read at boot — so `changed` alone
  // decides the response and a separate rebootNeeded flag was dead weight.
  bool changed = false;
  if (doc.containsKey("cert") && doc.containsKey("key")) {
    const char* cert = doc["cert"] | "";
    const char* key  = doc["key"]  | "";
    size_t certLen = strlen(cert);
    size_t keyLen  = strlen(key);
    // NVS string values cap around 4000 bytes — reject above 4096 explicitly
    // rather than letting putString() fail silently mid-write (which can
    // leave the namespace in an indeterminate state).
    const size_t PEM_MIN = 64;
    const size_t PEM_MAX = 4096;
    auto reject = [&](const char* msg) { respond(400, msg); };
    if (certLen < PEM_MIN || certLen > PEM_MAX ||
        strstr(cert, "-----BEGIN CERTIFICATE-----") == nullptr) {
      reject("{\"ok\":false,\"error\":\"Invalid cert PEM (need BEGIN CERTIFICATE marker, 64-4096 bytes)\"}");
      return;
    }
    // Private-key marker — accept PKCS#1 (BEGIN RSA PRIVATE KEY), PKCS#8
    // (BEGIN PRIVATE KEY), SEC1 EC (BEGIN EC PRIVATE KEY), and the rare
    // ENCRYPTED PRIVATE KEY form. All carry the suffix " PRIVATE KEY-----"
    // after a "-----BEGIN " prefix.
    if (keyLen < PEM_MIN || keyLen > PEM_MAX ||
        strstr(key, "-----BEGIN ")        == nullptr ||
        strstr(key, " PRIVATE KEY-----")  == nullptr) {
      reject("{\"ok\":false,\"error\":\"Invalid key PEM (need BEGIN ... PRIVATE KEY marker, 64-4096 bytes)\"}");
      return;
    }
    // Mutations under settingsLock (audit 2026-07) — this handler is reachable
    // from BOTH the WebServer task and the IDF httpd task, same as
    // /api/settings, so it must serialise the same way (#8). Validation above
    // stays outside the lock; only the NVS writes are inside.
    // NET-12: a failed lock is a hard 503 now — proceeding unlocked was the
    // whole hazard the mutex exists to prevent.
    if (!settingsLock()) {
      respond(503, "{\"ok\":false,\"error\":\"busy, retry\"}");
      return;
    }
    preferences.putString("tls_cert", cert);
    preferences.putString("tls_key",  key);
    settingsUnlock();
    changed = true;
    LOG("[TLS] Cert+key stored in NVS (%u / %u bytes)\n",
        (unsigned)certLen, (unsigned)keyLen);
  }
  if (doc.containsKey("enabled")) {
    bool en = doc["enabled"] | false;
    if (!settingsLock()) {   // NET-12
      respond(503, "{\"ok\":false,\"error\":\"busy, retry\"}");
      return;
    }
    // SEC-15: refuse to arm HTTPS when NVS has no cert/key pair to serve it
    // with. Without this the next boot finds "https_en" true, fails to start
    // the TLS server, and — before g_httpsRunning existed — port 80 would
    // still have been redirecting to a port nothing listens on. The check runs
    // after the cert/key branch above, so uploading and enabling in one POST
    // still works. Note this only writes the persisted INTENT: the live
    // g_httpsRunning is set once, in setup().
    if (en) {
      bool haveCert = preferences.getString("tls_cert", "").length() > 0;
      bool haveKey  = preferences.getString("tls_key",  "").length() > 0;
      if (!haveCert || !haveKey) {
        settingsUnlock();
        LOG("[TLS] Refused to enable HTTPS — no cert/key in NVS\n");
        respond(400, "{\"ok\":false,\"error\":\"upload a certificate and key first\"}");
        return;
      }
    }
    httpsEnabled = en;
    preferences.putBool("https_en", en);
    settingsUnlock();
    changed = true;
    LOG("[TLS] HTTPS %s (takes effect on next reboot)\n", en ? "enabled" : "disabled");
  }
  if (!changed) {
    respond(400, "{\"ok\":false,\"error\":\"No recognized fields\"}");
    return;
  }
  // Anything accepted here changes only boot-time state, so the answer is
  // always "saved, now reboot".
  respond(200, "{\"ok\":true,\"reboot_required\":true,"
               "\"message\":\"Saved. Reboot the controller for this to take effect.\"}");
}

// Wrapper for WebServer route registration
static void handleApiTlsPostWS() {
  HttpCtx ctx; ctx.isWS = true;
  handleApiTlsPost(ctx);
}

// ---------------------------------------------------------------------------
// IDF URI handler wrappers — one per HTTPS route.
//
// Each builds an HttpCtx from the IDF request (parses query string + form
// body into ctx.args[]), calls the existing httpsHandle*() function, and
// returns ESP_OK. The handlers themselves are unchanged from the legacy
// path: they read ctx.arg() / ctx.header() and emit via ctx.send() /
// ctx.sendProgmem(), which dispatch on ctx.isIDF inside HttpCtx.
// ---------------------------------------------------------------------------

static esp_err_t idf_root_get(httpd_req_t* req) {
  HttpCtx ctx; initFromIDFReq(req, ctx);
  httpsHandleRoot(ctx);
  return ESP_OK;
}
static esp_err_t idf_settings_get(httpd_req_t* req) {
  HttpCtx ctx; initFromIDFReq(req, ctx);
  httpsHandleSettingsPage(ctx);
  return ESP_OK;
}
static esp_err_t idf_login_get(httpd_req_t* req) {
  HttpCtx ctx; initFromIDFReq(req, ctx);
  httpsHandleLoginGet(ctx);
  return ESP_OK;
}
static esp_err_t idf_login_post(httpd_req_t* req) {
  HttpCtx ctx; initFromIDFReq(req, ctx);
  httpsHandleLoginPost(ctx);
  return ESP_OK;
}
static esp_err_t idf_logout_post(httpd_req_t* req) {
  HttpCtx ctx; initFromIDFReq(req, ctx);
  httpsHandleLogout(ctx);
  return ESP_OK;
}
static esp_err_t idf_api_status_get(httpd_req_t* req) {
  HttpCtx ctx; initFromIDFReq(req, ctx);
  httpsHandleApiStatus(ctx);
  return ESP_OK;
}
static esp_err_t idf_api_settings_get(httpd_req_t* req) {
  HttpCtx ctx; initFromIDFReq(req, ctx);
  httpsHandleApiSettingsGet(ctx);
  return ESP_OK;
}
static esp_err_t idf_api_settings_post(httpd_req_t* req) {
  HttpCtx ctx; initFromIDFReq(req, ctx);
  httpsHandleApiSettingsPost(ctx);
  return ESP_OK;
}
static esp_err_t idf_api_control_post(httpd_req_t* req) {
  HttpCtx ctx; initFromIDFReq(req, ctx);
  httpsHandleApiControl(ctx);
  return ESP_OK;
}
static esp_err_t idf_api_tls_post(httpd_req_t* req) {
  HttpCtx ctx; initFromIDFReq(req, ctx);
  handleApiTlsPost(ctx);
  return ESP_OK;
}
static esp_err_t idf_api_cycles_get(httpd_req_t* req) {
  HttpCtx ctx; initFromIDFReq(req, ctx);
  httpsHandleApiCyclesGet(ctx);
  return ESP_OK;
}
static esp_err_t idf_api_cycles_delete(httpd_req_t* req) {
  HttpCtx ctx; initFromIDFReq(req, ctx);
  httpsHandleApiCyclesDelete(ctx);
  return ESP_OK;
}

// ---------------------------------------------------------------------------
// startHTTPSServer / stopHTTPSServer
//
// Cert and key strings must outlive httpd_ssl_start; static String storage
// here keeps the c_str() pointers valid for the server's lifetime. Stack
// size is raised to 10 KB (default 4 KB is too small for TLS + ArduinoJson).
// ---------------------------------------------------------------------------

static bool startHTTPSServer(const String& cert, const String& key) {
  if (g_httpsServer) return true;  // already running

  static String s_cert, s_key;
  s_cert = cert;
  s_key  = key;

  httpd_ssl_config_t cfg = HTTPD_SSL_CONFIG_DEFAULT();
  // NET-1: the SERVER certificate goes in servercert/servercert_len. In IDF
  // 5.x cacert_pem/cacert_len is the CLIENT CA (the CA used to verify client
  // certificates in mutual-TLS) — this code used to put the server cert there,
  // leaving the handshake with a private key and no certificate to match it.
  // Lengths are +1 so the trailing NUL is included, which mbedTLS requires for
  // PEM input (same convention as prvtkey_len below).
  cfg.servercert             = (const uint8_t*)s_cert.c_str();
  cfg.servercert_len         = s_cert.length() + 1;
  cfg.cacert_pem             = nullptr;   // no client-cert auth
  cfg.cacert_len             = 0;
  cfg.prvtkey_pem            = (const uint8_t*)s_key.c_str();
  cfg.prvtkey_len            = s_key.length() + 1;
  cfg.port_secure            = 443;
  cfg.httpd.stack_size       = 10240;
  cfg.httpd.max_uri_handlers = 14;   // 12 routes registered below + headroom

  esp_err_t err = httpd_ssl_start(&g_httpsServer, &cfg);
  if (err != ESP_OK) {
    g_httpsServer = nullptr;
    LOG("[TLS] httpd_ssl_start failed (%d) — falling back to HTTP\n", (int)err);
    return false;
  }

  // Helper avoids C++ aggregate-init pitfalls if httpd_uri_t gains fields.
  // NET-1: a failed registration used to be silent — the route then answered
  // 404 over TLS with nothing in the log to explain it. Log the URI instead.
  auto reg = [&](const char* uri, httpd_method_t method,
                 esp_err_t (*handler)(httpd_req_t*)) {
    httpd_uri_t u = {};
    u.uri      = uri;
    u.method   = method;
    u.handler  = handler;
    u.user_ctx = nullptr;
    esp_err_t rerr = httpd_register_uri_handler(g_httpsServer, &u);
    if (rerr != ESP_OK) {
      LOG("[TLS] register %s failed (%d) — route unavailable over HTTPS\n",
          uri, (int)rerr);
    }
  };
  reg("/",              HTTP_GET,  idf_root_get);
  reg("/settings",      HTTP_GET,  idf_settings_get);
  reg("/login",         HTTP_GET,  idf_login_get);
  reg("/login",         HTTP_POST, idf_login_post);
  reg("/logout",        HTTP_POST, idf_logout_post);
  reg("/api/status",    HTTP_GET,  idf_api_status_get);
  reg("/api/settings",  HTTP_GET,  idf_api_settings_get);
  reg("/api/settings",  HTTP_POST, idf_api_settings_post);
  reg("/api/control",   HTTP_POST, idf_api_control_post);
  reg("/api/tls",       HTTP_POST, idf_api_tls_post);
  reg("/api/cycles",    HTTP_GET,    idf_api_cycles_get);
  reg("/api/cycles",    HTTP_DELETE, idf_api_cycles_delete);

  LOG("[TLS] HTTPS server started on port 443 (cert %u bytes)\n",
      (unsigned)s_cert.length());
  return true;
}

// UNUSED — nothing calls this. HTTPS is only ever started once, from setup();
// the /api/tls "enabled" toggle persists the intent and asks for a reboot
// rather than tearing the TLS server down under live connections (SEC-15).
// Kept as the counterpart to startHTTPSServer() should a runtime stop ever be
// wanted; delete both together if that never happens.
__attribute__((unused))
static void stopHTTPSServer() {
  if (!g_httpsServer) return;
  httpd_ssl_stop(g_httpsServer);
  g_httpsServer  = nullptr;
  g_httpsRunning = false;
  LOG("[TLS] HTTPS server stopped\n");
}

// Port-80 redirect handler — installed as onNotFound when g_httpsRunning, and
// invoked by WebBodyGuardHandler for every port-80 URI that has no explicit
// route. It therefore only ever sees the URIs that moved to 443; the HTTP-only
// routes (/login, /logout, /update, /log, /api/log/stream, /save, /api/tls) are
// registered on port 80 in both states (SEC-4/NET-2) and never land here.
static void handleHTTPSRedirect() {
  String host = server.hostHeader();
  // Strip port number from host if present
  int col = host.lastIndexOf(':');
  if (col > 0) host = host.substring(0, col);
  // Sanitise the Host header before reflecting it into the Location URL
  // (finding #12 — open-redirect / header-injection hardening). Keep only
  // characters legal in a hostname or IPv4/IPv6 literal; anything else
  // (including CR/LF, '/', '@', whitespace) means a crafted Host, so we fall
  // back to the device's own STA IP rather than honour it.
  bool hostOk = host.length() > 0 && host.length() <= 253;
  for (unsigned int i = 0; hostOk && i < host.length(); i++) {
    char c = host.charAt(i);
    bool legal = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                 (c >= '0' && c <= '9') || c == '.' || c == '-' || c == ':';
    if (!legal) hostOk = false;
  }
  if (!hostOk) host = WiFi.localIP().toString();
  // server.uri() is the path the WebServer already parsed (no scheme/host), so
  // it can't smuggle a different origin. The query string is intentionally not
  // reflected — it isn't needed to land on the HTTPS dashboard and reflecting
  // raw arg values would be another injection vector.
  String url = "https://" + host + server.uri();
  server.sendHeader("Location", url);
  // M5: 302, not 301. Browsers cache a 301 indefinitely and ignore the origin
  // afterwards, so if HTTPS is later turned off (bad cert, expired cert,
  // factory reset) every previously-visited browser keeps forcing https:// and
  // the dashboard becomes unreachable until the user clears their cache — on a
  // device whose only recovery path is the physical BOOT button. A 302 gets the
  // same redirect behaviour with none of the stickiness.
  server.send(302, "text/plain", "");
}

// Build the /api/settings JSON string. Called by both HTTP and HTTPS handlers.
static String buildApiSettingsJson() {
  // Read the shared config globals under settingsLock so we can't snapshot a
  // half-written mqttHost/apStatic*/mqttCaCert while the other server task is
  // mutating it (#8).
  // NET-12: best-effort by design here — this is a read-only snapshot, so on a
  // lock timeout we serve a possibly-torn field rather than fail the settings
  // page. The mutating paths (/api/settings POST, /api/tls) answer 503 instead.
  bool locked = settingsLock();
  String ssid = preferences.getString("ssid", "");
  String aps  = preferences.getString("ap_ssid", String(apSSID));
  String mh   = String(mqttHost);
  String mu   = String(mqttUser);
  uint8_t cc  = ctrl.chargerCount;
  bool hasApPass = (strlen(apPass) > 0 ||
                    preferences.getString("ap_pass", "").length() > 0);

  // SEC-9: control characters (< 0x20) are illegal raw inside a JSON string —
  // an SSID or MQTT username carrying one used to be copied through verbatim
  // and broke JSON.parse() in the settings page. Escape them the way the spec
  // does: the named forms where they exist, \u00XX otherwise.
  auto jsonEscape = [](const String& in) -> String {
    String out;
    out.reserve(in.length() + 8);
    for (unsigned int i = 0; i < in.length(); i++) {
      char c = in.charAt(i);
      if (c == '"' || c == '\\') { out += '\\'; out += c; continue; }
      switch (c) {
        case '\n': out += "\\n"; continue;
        case '\r': out += "\\r"; continue;
        case '\t': out += "\\t"; continue;
        case '\b': out += "\\b"; continue;
        case '\f': out += "\\f"; continue;
        default: break;
      }
      if ((uint8_t)c < 0x20) {
        char u[7];
        snprintf(u, sizeof(u), "\\u%04x", (unsigned)(uint8_t)c);
        out += u;
        continue;
      }
      out += c;
    }
    return out;
  };

  String jSsid = jsonEscape(ssid);
  String jAps  = jsonEscape(aps);
  String jMh   = jsonEscape(mh);
  String jMu   = jsonEscape(mu);
  // SEC-10: the AP address strings come from NVS and are emitted into the same
  // JSON — escape them too. They should always be dotted quads (the POST path
  // parse-checks them now), but a blob written by an older build or a direct
  // NVS edit must not be able to break the document.
  String jAip  = jsonEscape(String(apStaticIp));
  String jAgw  = jsonEscape(String(apStaticGateway));
  String jAsn  = jsonEscape(String(apStaticSubnet));

  bool   caSet   = mqttCaCert.length() > 0;
  size_t caBytes = mqttCaCert.length();
  // TLS cert presence (for HTTPS section in settings UI)
  bool   tlsCertSet = preferences.getString("tls_cert", "").length() > 0;

  char buf[1100];
  int jsonLen = snprintf(buf, sizeof(buf),
    "{\"ap_mode\":%s,\"wifi_ssid\":\"%s\",\"ap_ssid\":\"%s\",\"ap_pass_set\":%s,"
    "\"ap_static_en\":%s,\"ap_ip\":\"%s\",\"ap_gw\":\"%s\",\"ap_sn\":\"%s\","
    "\"mqtt_host\":\"%s\",\"mqtt_port\":%d,\"mqtt_user\":\"%s\","
    "\"mqtt_tls\":%s,\"mqtt_ca_set\":%s,\"mqtt_ca_bytes\":%u,"
    "\"charger_count\":%d,\"ramp_rate_wps\":%d,"
    "\"charging_enabled_default\":%s,\"power_preset_default\":%d,\"target_volt_default\":%d,"
    "\"home_charging_enabled_default\":%s,\"home_power_preset_default\":%d,\"home_target_volt_default\":%d,"
    "\"https_enabled\":%s,\"https_cert_set\":%s}",
    apIsBroadcasting() ? "true" : "false",
    jSsid.c_str(), jAps.c_str(), hasApPass ? "true" : "false",
    apStaticIpEnabled ? "true" : "false",
    jAip.c_str(), jAgw.c_str(), jAsn.c_str(),
    jMh.c_str(), (int)mqttPort, jMu.c_str(),
    mqttTls ? "true" : "false", caSet ? "true" : "false", (unsigned)caBytes,
    (int)cc, (int)ctrl.rampStepW,
    ctrl.defaultEnabled ? "true" : "false",
    (int)ctrl.defaultPowerPresetIdx,
    (int)ctrl.defaultTargetVoltDv,
    ctrl.homeDefaultEnabled ? "true" : "false",
    (int)ctrl.homeDefaultPowerPresetIdx,
    (int)ctrl.homeDefaultTargetVoltDv,
    httpsEnabled ? "true" : "false",
    tlsCertSet   ? "true" : "false"
  );
  if (locked) settingsUnlock();
  // SEC-11: same guard buildApiStatusJson() carries. A long SSID / MQTT host
  // (each up to 64 chars, and escaping can double them) can overrun buf, and
  // snprintf would hand back a silently truncated — therefore unparseable —
  // document that the settings page renders as a blank form.
  if (jsonLen < 0 || jsonLen >= (int)sizeof(buf)) {
    LOG("[WEB] settings JSON truncated (%d >= %u) — sending error stub\n",
        jsonLen, (unsigned)sizeof(buf));
    return String("{\"ok\":false,\"error\":\"settings buffer overflow\"}");
  }
  return String(buf);
}

// GET /api/settings — returns current settings as JSON (protected)
void handleApiSettingsGet() {
  if (!requireAuth(true)) return;   // API route
  String json = buildApiSettingsJson();
  // B9: the overflow stub (SEC-11) is an error, not a settings document —
  // answering 200 made it indistinguishable from real data to anything but the
  // page's own JS. 500 is the honest code: the request was fine, we failed.
  int code = json.startsWith("{\"ok\":false") ? 500 : 200;
  server.send(code, "application/json", json);
}

// Apply a /api/settings JSON body. Returns a JSON ack string.
// Sets settingsNeedRestart=true if the caller should restart after responding.
//
// NET-4 — two passes, strictly separated:
//   Pass 1 parses and validates EVERY field present into locals and returns
//          the error JSON on the first failure, having written nothing.
//   Pass 2 commits the validated values to NVS and to the RAM globals.
// Before this split the function wrote each field as it went and returned on
// the first bad one, so a body with a good wifi_ssid and a bad mqtt_ca left
// new WiFi credentials in NVS, answered 400, and — because settingsNeedRestart
// had already been latched by the wifi_ssid branch — still rebooted the
// controller into them. Callers additionally gate the reboot on a 200 now.
static String applyApiSettingsBody(const String& body) {
  settingsNeedRestart = false;

  if (body.length() > 8192)
    return "{\"ok\":false,\"error\":\"Payload too large\"}";

  DynamicJsonDocument doc(body.length() + 1024);
  if (deserializeJson(doc, body))
    return "{\"ok\":false,\"error\":\"Invalid JSON\"}";

  // Helper: validate length and copy string to fixed buffer.
  // Returns empty string on success; returns an error JSON string on failure.
  auto copyChecked = [](const char* src, char* dst, size_t maxLen,
                        const char* fieldLabel) -> String {
    if (!src) src = "";
    size_t l = strlen(src);
    if (l >= maxLen) {
      char err[96];
      snprintf(err, sizeof(err),
               "{\"ok\":false,\"error\":\"%s too long (max %u chars)\"}",
               fieldLabel, (unsigned)(maxLen - 1));
      return String(err);
    }
    strncpy(dst, src, maxLen - 1);
    dst[maxLen - 1] = '\0';
    return String();
  };

  String err;

  // ─────────────────────────────────────────────────────────────────────────
  // PASS 1 — parse + validate into locals. No NVS write, no global mutation,
  // no settingsNeedRestart latch. Any `return err` here leaves the device
  // exactly as it was.
  // ─────────────────────────────────────────────────────────────────────────

  // WiFi credentials.
  bool hasWifi = false;
  char vSsid[65]     = {0};
  char vWifiPass[65] = {0};
  if (doc.containsKey("wifi_ssid")) {
    const char* ssid = doc["wifi_ssid"] | "";
    if (strlen(ssid) == 0) return "{\"ok\":false,\"error\":\"SSID empty\"}";
    err = copyChecked(ssid, vSsid, sizeof(vSsid), "wifi_ssid");
    if (err.length()) return err;
    const char* pass = doc["wifi_pass"] | "";
    err = copyChecked(pass, vWifiPass, sizeof(vWifiPass), "wifi_pass");
    if (err.length()) return err;
    hasWifi = true;
  }

  // AP credentials. An empty ap_ssid is ignored (kept from the original), but
  // ap_pass is still honoured alongside it.
  bool hasApBlock = doc.containsKey("ap_ssid");
  bool hasApSsid  = false;
  bool hasApPass  = false;
  char vApSsid[sizeof(apSSID)] = {0};
  char vApPass[sizeof(apPass)] = {0};
  if (hasApBlock) {
    const char* aps = doc["ap_ssid"] | "";
    if (strlen(aps) > 0) {
      err = copyChecked(aps, vApSsid, sizeof(vApSsid), "ap_ssid");
      if (err.length()) return err;
      hasApSsid = true;
    }
    if (doc.containsKey("ap_pass")) {
      const char* app = doc["ap_pass"] | "";
      size_t apl = strlen(app);
      // SEC-6: an EMPTY ap_pass used to pass this check and then be stored,
      // silently turning the access point open. The settings JS only sends the
      // key when the user typed something (saveAP: `if (pass.length > 0)`), so
      // rejecting empty costs the UI nothing — verified, no UI change needed.
      if (apl < 8)
        return "{\"ok\":false,\"error\":\"AP password must be 8+ chars\"}";
      err = copyChecked(app, vApPass, sizeof(vApPass), "ap_pass");
      if (err.length()) return err;
      hasApPass = true;
    }
  }

  // Fixed AP-mode IP. ap_static_en gates the feature; ap_ip/ap_gw/ap_sn are
  // always persisted (even when disabled) so toggling off then on again keeps
  // the user's typed values.
  // SEC-10: each address is now parse-checked whenever it is present and
  // non-empty, not only while enabling — storing garbage that only blows up at
  // the next AP bring-up was a booby trap. Empty still means "not configured"
  // and is accepted while the feature is off; enabling requires all three.
  bool hasApStatic = doc.containsKey("ap_static_en");
  bool vApEn = false;
  char vAip[16] = {0}, vAgw[16] = {0}, vAsn[16] = {0};
  if (hasApStatic) {
    vApEn = doc["ap_static_en"] | false;
    const char* aip = doc["ap_ip"] | "";
    const char* agw = doc["ap_gw"] | "";
    const char* asn = doc["ap_sn"] | "";
    IPAddress t;
    if ((vApEn || strlen(aip) > 0) && !t.fromString(aip))
      return "{\"ok\":false,\"error\":\"AP IP address invalid\"}";
    if ((vApEn || strlen(agw) > 0) && !t.fromString(agw))
      return "{\"ok\":false,\"error\":\"AP gateway invalid\"}";
    if ((vApEn || strlen(asn) > 0) && !t.fromString(asn))
      return "{\"ok\":false,\"error\":\"AP subnet mask invalid\"}";
    err = copyChecked(aip, vAip, sizeof(vAip), "ap_ip");  if (err.length()) return err;
    err = copyChecked(agw, vAgw, sizeof(vAgw), "ap_gw");  if (err.length()) return err;
    err = copyChecked(asn, vAsn, sizeof(vAsn), "ap_sn");  if (err.length()) return err;
  }

  // MQTT broker settings. mqtt_port / mqtt_user / mqtt_pass are only read
  // alongside mqtt_host, as before. An out-of-range port is ignored rather
  // than rejected — unchanged behaviour.
  bool hasMqttHost = doc.containsKey("mqtt_host");
  bool hasMqttPort = false, hasMqttUser = false, hasMqttPass = false;
  char     vMqttHost[sizeof(mqttHost)]       = {0};
  char     vMqttUser[sizeof(mqttUser)]       = {0};
  char     vMqttPass[sizeof(mqttBrokerPass)] = {0};
  uint16_t vMqttPort = 0;
  if (hasMqttHost) {
    const char* mh = doc["mqtt_host"] | "";
    err = copyChecked(mh, vMqttHost, sizeof(vMqttHost), "mqtt_host");
    if (err.length()) return err;

    int port = doc["mqtt_port"] | 0;
    if (port > 0 && port <= 65535) { vMqttPort = (uint16_t)port; hasMqttPort = true; }

    if (doc.containsKey("mqtt_user")) {
      const char* mu = doc["mqtt_user"] | "";
      err = copyChecked(mu, vMqttUser, sizeof(vMqttUser), "mqtt_user");
      if (err.length()) return err;
      hasMqttUser = true;
    }
    if (doc.containsKey("mqtt_pass")) {
      const char* mp = doc["mqtt_pass"] | "";
      err = copyChecked(mp, vMqttPass, sizeof(vMqttPass), "mqtt_pass");
      if (err.length()) return err;
      hasMqttPass = true;
    }
  }

  // MQTT TLS toggle + CA cert.
  bool   hasCa    = doc.containsKey("mqtt_ca");
  bool   vCaClear = false;
  String vCa;
  if (hasCa) {
    const char* ca = doc["mqtt_ca"] | "";
    size_t calen = strlen(ca);
    if (calen == 0) {
      vCaClear = true;
    } else {
      if (calen > 4096)
        return "{\"ok\":false,\"error\":\"CA cert too large (max 4 KB)\"}";
      if (!strstr(ca, "-----BEGIN CERTIFICATE-----") ||
          !strstr(ca, "-----END CERTIFICATE-----"))
        return "{\"ok\":false,\"error\":\"CA cert must be PEM (BEGIN/END markers required)\"}";
      vCa = ca;
    }
  }
  bool hasMqttTls = doc.containsKey("mqtt_tls");
  bool vMqttTls   = false;
  if (hasMqttTls) {
    vMqttTls = doc["mqtt_tls"] | false;
    // Validate against the CA this same body would leave in place, not just
    // the one currently stored — upload-and-enable in one POST still works,
    // and clear-and-enable is still refused.
    size_t effCaLen = hasCa ? (vCaClear ? 0 : vCa.length()) : mqttCaCert.length();
    if (vMqttTls && effCaLen == 0)
      return "{\"ok\":false,\"error\":\"Cannot enable TLS without a CA cert. Upload one first.\"}";
  }

  // Charger count and the two boot-default profiles. Out-of-range values are
  // silently ignored here exactly as they were before — not an error.
  bool    hasCc = false; uint8_t vCc = 0;
  if (doc.containsKey("charger_count")) {
    int cc = doc["charger_count"] | -1;
    if (cc >= 1 && cc <= 4) { vCc = (uint8_t)cc; hasCc = true; }
  }
  bool    hasDefEn = doc.containsKey("charging_enabled_default");
  bool    vDefEn   = hasDefEn ? (doc["charging_enabled_default"] | false) : false;
  bool    hasDefPwr = false; uint8_t vDefPwr = 0;
  if (doc.containsKey("power_preset_default")) {
    int pi = doc["power_preset_default"] | -1;
    if (pi >= 0 && pi < MAX_PRESETS_PER_ROW) { vDefPwr = (uint8_t)pi; hasDefPwr = true; }
  }
  bool    hasDefTgt = false; uint16_t vDefTgt = 0;
  if (doc.containsKey("target_volt_default")) {
    int tvd = doc["target_volt_default"] | -1;
    if (tvd >= (int)TARGET_VOLT_PRESETS[0].dv && tvd <= (int)MAX_CHARGE_VOLTAGE_DV) {
      vDefTgt = (uint16_t)tvd; hasDefTgt = true;
    }
  }
  bool    hasHomeEn = doc.containsKey("home_charging_enabled_default");
  bool    vHomeEn   = hasHomeEn ? (doc["home_charging_enabled_default"] | false) : false;
  bool    hasHomePwr = false; uint8_t vHomePwr = 0;
  if (doc.containsKey("home_power_preset_default")) {
    int pi = doc["home_power_preset_default"] | -1;
    if (pi >= 0 && pi < MAX_PRESETS_PER_ROW) { vHomePwr = (uint8_t)pi; hasHomePwr = true; }
  }
  bool    hasHomeTgt = false; uint16_t vHomeTgt = 0;
  if (doc.containsKey("home_target_volt_default")) {
    int tvd = doc["home_target_volt_default"] | -1;
    if (tvd >= (int)TARGET_VOLT_PRESETS[0].dv && tvd <= (int)MAX_CHARGE_VOLTAGE_DV) {
      vHomeTgt = (uint16_t)tvd; hasHomeTgt = true;
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  // PASS 2 — commit. Everything below is validated; no path returns an error.
  // ─────────────────────────────────────────────────────────────────────────

  if (hasWifi) {
    preferences.putString("ssid", vSsid);
    preferences.putString("pass", vWifiPass);
    LOG("[SETTINGS] WiFi credentials saved\n");
    settingsNeedRestart = true;
  }

  if (hasApSsid) {
    preferences.putString("ap_ssid", vApSsid);
    strncpy(apSSID, vApSsid, sizeof(apSSID) - 1);
    apSSID[sizeof(apSSID) - 1] = '\0';
  }
  if (hasApPass) {
    preferences.putString("ap_pass", vApPass);
    strncpy(apPass, vApPass, sizeof(apPass) - 1);
    apPass[sizeof(apPass) - 1] = '\0';
  }
  if (hasApBlock) {
    LOG("[SETTINGS] AP credentials saved: \"%s\"\n", apSSID);
  }

  if (hasApStatic) {
    preferences.putBool("ap_ip_en", vApEn);
    preferences.putString("ap_ip", vAip);
    preferences.putString("ap_gw", vAgw);
    preferences.putString("ap_sn", vAsn);
    apStaticIpEnabled = vApEn;
    strncpy(apStaticIp,      vAip, sizeof(apStaticIp) - 1);
    apStaticIp[sizeof(apStaticIp) - 1] = '\0';
    strncpy(apStaticGateway, vAgw, sizeof(apStaticGateway) - 1);
    apStaticGateway[sizeof(apStaticGateway) - 1] = '\0';
    strncpy(apStaticSubnet,  vAsn, sizeof(apStaticSubnet) - 1);
    apStaticSubnet[sizeof(apStaticSubnet) - 1] = '\0';
    LOG("[SETTINGS] AP fixed IP %s: %s gw %s mask %s\n",
        vApEn ? "enabled" : "disabled", vAip, vAgw, vAsn);
  }

  if (hasMqttHost) {
    preferences.putString("mqtt_host", vMqttHost);
    strncpy(mqttHost, vMqttHost, sizeof(mqttHost) - 1);
    mqttHost[sizeof(mqttHost) - 1] = '\0';
    if (hasMqttPort) {
      preferences.putUShort("mqtt_port", vMqttPort);
      mqttPort = vMqttPort;
    }
    if (hasMqttUser) {
      preferences.putString("mqtt_user", vMqttUser);
      strncpy(mqttUser, vMqttUser, sizeof(mqttUser) - 1);
      mqttUser[sizeof(mqttUser) - 1] = '\0';
    }
    if (hasMqttPass) {
      preferences.putString("mqtt_pass", vMqttPass);
      strncpy(mqttBrokerPass, vMqttPass, sizeof(mqttBrokerPass) - 1);
      mqttBrokerPass[sizeof(mqttBrokerPass) - 1] = '\0';
    }
    LOG("[SETTINGS] MQTT settings saved: %s:%d\n", mqttHost, mqttPort);
    mqttReconnectRequested = true;  // mqttTask owns mqttClient — don't touch it here (#7)
  }

  if (hasCa) {
    if (vCaClear) {
      mqttCaCert = "";
      preferences.remove("mqtt_ca");
      LOG("[SETTINGS] MQTT CA cert cleared\n");
    } else {
      mqttCaCert = vCa;
      preferences.putString("mqtt_ca", mqttCaCert);
      LOG("[SETTINGS] MQTT CA cert saved (%u bytes)\n", (unsigned)vCa.length());
    }
  }
  if (hasMqttTls) {
    mqttTls = vMqttTls;
    preferences.putBool("mqtt_tls", vMqttTls);
    LOG("[SETTINGS] MQTT TLS: %s\n", vMqttTls ? "ON" : "off");
  }
  if (hasCa || hasMqttTls) mqttReconnectRequested = true;  // consumed by mqttTask (#7)

  if (hasCc) {
    if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      ctrl.chargerCount = vCc;
      xSemaphoreGive(controlMutex);
    }
    preferences.putUChar("charger_count", vCc);
    LOG("[SETTINGS] Charger count: %d\n", (int)vCc);
  }

  // Boot defaults.
  if (hasDefEn) {
    ctrl.defaultEnabled = vDefEn;
    preferences.putBool("def_chg_en", vDefEn);
    LOG("[SETTINGS] Boot charging default: %s\n", vDefEn ? "ON" : "OFF");
  }
  if (hasDefPwr) {
    ctrl.defaultPowerPresetIdx = vDefPwr;
    preferences.putUChar("def_pwr_idx", vDefPwr);
    LOG("[SETTINGS] Boot power preset index: %d\n", (int)vDefPwr);
  }
  if (hasDefTgt) {
    ctrl.defaultTargetVoltDv = vDefTgt;
    preferences.putUShort("def_tgt_dv", vDefTgt);
    LOG("[SETTINGS] Boot target volt default: %d dV\n", (int)vDefTgt);
  }

  // Home WiFi boot defaults.
  if (hasHomeEn) {
    ctrl.homeDefaultEnabled = vHomeEn;
    preferences.putBool("home_chg_en", vHomeEn);
    LOG("[SETTINGS] Home boot charging default: %s\n", vHomeEn ? "ON" : "OFF");
  }
  if (hasHomePwr) {
    ctrl.homeDefaultPowerPresetIdx = vHomePwr;
    preferences.putUChar("home_pwr_idx", vHomePwr);
    LOG("[SETTINGS] Home boot power preset index: %d\n", (int)vHomePwr);
  }
  if (hasHomeTgt) {
    ctrl.homeDefaultTargetVoltDv = vHomeTgt;
    preferences.putUShort("home_tgt_dv", vHomeTgt);
    LOG("[SETTINGS] Home boot target volt default: %d dV\n", (int)vHomeTgt);
  }

  return "{\"ok\":true}";
}

// POST /api/settings — saves settings to NVS (protected).
// Body parsed via ArduinoJson (replaces a hand-rolled indexOf-based parser
// that could match keys inside string values and silently truncate strings
// larger than the destination buffer). All length-constrained string fields
// now reject oversized input with a 400 instead of being silently clipped.
void handleApiSettingsPost() {
  if (!requireAuth(true)) return;   // API route
  if (g_rawTooLarge) {
    server.send(413, "application/json", "{\"ok\":false,\"error\":\"Payload too large\"}");
    return;
  }
  if (g_rawBody.length() == 0) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"No body\"}");
    return;
  }
  // NET-12: serialise vs the HTTPS server task (#8). A failed take used to fall
  // through and apply the body unlocked, which is precisely the race the mutex
  // exists to stop — answer 503 and let the client retry instead.
  if (!settingsLock()) {
    server.send(503, "application/json", "{\"ok\":false,\"error\":\"busy, retry\"}");
    return;
  }
  String result = applyApiSettingsBody(g_rawBody);
  // B12: snapshot the flag INSIDE the lock. settingsNeedRestart is a plain
  // file-scope bool shared with the HTTPS handler, and applyApiSettingsBody
  // clears it on entry — so once the mutex is released, a concurrent TLS-side
  // save can reset or set it before we read it, and this request would either
  // miss its own reboot or take someone else's.
  bool needRestart = settingsNeedRestart;
  settingsUnlock();
  int code = 200;
  if (result.indexOf("\"ok\":false") >= 0)
    code = (result.indexOf("Payload too large") >= 0) ? 413 : 400;
  server.send(code, "application/json", result);
  // NET-4: reboot only on a successful save. applyApiSettingsBody latches the
  // flag in its commit pass, which a 4xx never reaches — the code check is the
  // belt to that braces.
  if (code == 200 && needRestart) {
    LOG("[SETTINGS] Restarting for WiFi changes...\n");
    // Response already sent; helper's 1.5 s wait replaces the old delay().
    stopChargerForRestart("settings save");
    ESP.restart();
  }
}

// Legacy POST /save — kept for backwards compatibility with the original
// setup-page form. Auth-gated: without this check, any device on the LAN
// (or any malicious page the owner visits in a tab on the same network —
// form-encoded POSTs are CORS-simple, no preflight) could rewrite the WiFi
// credentials and force a reboot, hijacking the controller.
void handleSave() {
  if (!requireAuth(false)) return;  // legacy HTML form-POST, returns HTML
  // RAW body mode (RawBodyHandler): parse the urlencoded body ourselves.
  HttpCtx form;
  parseKVPairs(g_rawBody, form);
  // L1: reject rather than persist WiFi credentials parsed from a form we only
  // partially captured — a silently dropped "pass" field would save an SSID
  // with an empty password and then reboot into it.
  if (form.argsOverflow) {
    LOG("[SETTINGS] /save form had more than %d fields — rejected\n", HTTPS_MAX_ARGS);
    server.send(400, "text/plain", "Malformed form\n");
    return;
  }
  if (form.hasArg("ssid")) {
    String newSSID = form.arg("ssid");
    String newPass = form.arg("pass");
    // SEC-13: apply the /api/settings length limit, plus a control-character
    // rejection /api/settings does not have. Both reject rather than truncate:
    // silently clipping a 70-char SSID stored credentials that can never
    // associate, and the device reboots straight into them with no way back
    // except the BOOT button. Control characters cannot be typed into a real
    // network name and would land raw in the /api/settings JSON.
    if (newSSID.length() > 64) {
      server.send(400, "text/plain", "SSID too long (max 64 chars)\n");
      return;
    }
    if (newPass.length() > 64) {
      server.send(400, "text/plain", "Password too long (max 64 chars)\n");
      return;
    }
    for (unsigned int i = 0; i < newSSID.length(); i++) {
      if ((uint8_t)newSSID.charAt(i) < 0x20 || (uint8_t)newSSID.charAt(i) == 0x7F) {
        server.send(400, "text/plain", "SSID contains control characters\n");
        return;
      }
    }
    if (!newSSID.isEmpty()) {
      preferences.putString("ssid", newSSID);
      preferences.putString("pass", newPass);
      server.send(200, "text/html; charset=utf-8",
        "<div style='font-family:Arial;text-align:center;padding:40px;"
        "background:#1a1a2e;color:#eee'>"
        "<h2 style='color:#e94560'>Saved!</h2><p>Restarting...</p></div>");
      // 500 ms for the browser to finish reading the page, then the helper's
      // 1.5 s STOP window (2 s total — same as the delay this replaces).
      delay(500);
      stopChargerForRestart("WiFi creds saved via /save");
      ESP.restart();
      return;
    }
  }
  server.send(400, "text/plain", "Missing SSID");
}

// GET /api/status — JSON endpoint polled by the dashboard every 2 s.
// Auth-gated: pack voltage, SoC and energy figures shouldn't be readable
// by any device on the LAN. The browser caches the Basic Auth header from
// the dashboard load and re-uses it on every poll without re-prompting.
// Format one pack temperature for JSON. A disconnected/shorted thermistor is
// stored raw as ZERO_TEMP_INVALID (-32768, ZERO.h) so the firmware can tell
// "no sensor" from a genuine 0 °C; emitting that number would put -32768 °C on
// the dashboard, so anything absurdly low becomes JSON null instead and the
// page renders "—". The threshold is TEMP_INVALID_THRESHOLD (ZERO.h) — far
// below any real pack temperature and far above the sentinel — shared with
// rampTask's validity test and the MQTT publish macro.
static void fmtTempJson(char* out, size_t n, short v) {
  if (v <= TEMP_INVALID_THRESHOLD) snprintf(out, n, "null");
  else                             snprintf(out, n, "%d", (int)v);
}

// Build the /api/status JSON string. Called by both HTTP and HTTPS handlers.
static String buildApiStatusJson() {
  // Snapshot both data structs under their respective mutexes
  LiveData      liveSnap;
  ChargerBusData chargerSnap;
  SysStats      statsSnap;

  if (xSemaphoreTake(liveMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    liveSnap = live;
    xSemaphoreGive(liveMutex);
  }
  if (xSemaphoreTake(chargerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    chargerSnap = chargerBus;
    xSemaphoreGive(chargerMutex);
  }
  if (xSemaphoreTake(sysStatsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    statsSnap = sysStats;
    xSemaphoreGive(sysStatsMutex);
  }

  // Control state under its mutex too (audit 2026-07) — previously read from
  // ctrl.* directly. Aligned ≤16-bit reads are atomic on Xtensa so it was
  // benign, but snapshotting keeps the JSON internally consistent and matches
  // how every other shared struct is handled here.
  ChargingControl ctrlSnap;
  if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    ctrlSnap = ctrl;
    xSemaphoreGive(controlMutex);
  }

  // State of charge: prefer the BMS's own value (BMS_PACK_STATUS 0x188 byte 0)
  // because it's coulomb-counted inside the BMS — same number the bike's
  // dashboard derives from. Fall back to a voltage-curve estimate until the
  // first 0x188 frame arrives. soc_source is exposed in the JSON so the
  // dashboard / HA can show whether we're reading the truth or the estimate.
  int         monolithSoc       = (liveSnap.monolithBmsSoc <= 100)
                                    ? (int)liveSnap.monolithBmsSoc
                                    : calcSocFromVoltage(liveSnap.monolithVoltageDv);
  const char* monolithSocSource = (liveSnap.monolithBmsSoc <= 100) ? "bms" : "voltage";

  // Available Ah: how much capacity is actually left in the pack right now.
  // Nominal pack-AH (114 on the monolith) × current SoC %. Updates in lockstep
  // with the SoC card; gives the user a "real Ah remaining" feel-good number.
  float monolithAhAvail = (liveSnap.monolithAH > 0)
                            ? (float)liveSnap.monolithAH * monolithSoc / 100.0f
                            : 0.0f;

  // Same for the optional PowerTank pack — uses the BMS1 SoC if available,
  // otherwise the voltage estimate from the secondary pack voltage.
  int   powerTankSoc      = (liveSnap.powerTankBmsSoc <= 100)
                              ? (int)liveSnap.powerTankBmsSoc
                              : calcSocFromVoltage(liveSnap.powerTankVoltageDv);
  float powerTankAhAvail  = (liveSnap.powerTankAH > 0)
                              ? (float)liveSnap.powerTankAH * powerTankSoc / 100.0f
                              : 0.0f;

  // Snapshot session data — atomic read pair via sessionMutex.
  float sessWh = 0.0f, sessAh = 0.0f;
  sessionSnapshot(sessWh, sessAh);

  // Fixed fields — use a stack buffer for the bulk of the response. Sized with
  // headroom over the worst-case formatted length (finding #14); the return
  // value is checked below so a future field addition can't silently truncate
  // the JSON into something unparseable.
  // 1536 -> 1600 when charge_inhibit was added (worst case ~40 bytes:
  // "charge_inhibit":"pack voltage too low",). The guard below still backs this
  // up, but keep the headroom real rather than relying on the error stub.
  // Pack temperatures are emitted as a number OR as literal null when the
  // thermistor is missing — see fmtTempJson() above.
  char tMonMin[8], tMonMax[8], tPtMin[8], tPtMax[8];
  fmtTempJson(tMonMin, sizeof(tMonMin), liveSnap.monolithMinTemp);
  fmtTempJson(tMonMax, sizeof(tMonMax), liveSnap.monolithMaxTemp);
  fmtTempJson(tPtMin,  sizeof(tPtMin),  liveSnap.powerTankMinTemp);
  fmtTempJson(tPtMax,  sizeof(tPtMax),  liveSnap.powerTankMaxTemp);

  char buf[1600];
  int jsonLen = snprintf(buf, sizeof(buf),
    "{"
      "\"fresh\":%s,"
      "\"ap_mode\":%s,"
      "\"monolith_v\":%.1f,"
      "\"monolith_a\":%.0f,"
      "\"monolith_ah\":%.0f,"
      "\"monolith_ah_avail\":%.1f,"
      "\"monolith_tmin\":%s,"
      "\"monolith_tmax\":%s,"
      "\"monolith_crate\":%.3f,"
      "\"monolith_soc\":%d,"
      "\"monolith_soc_source\":\"%s\","
      "\"powertank_present\":%s,"
      "\"powertank_decided\":%s,"
      "\"powertank_v\":%.1f,"
      "\"powertank_a\":%.0f,"
      "\"powertank_ah\":%.0f,"
      "\"powertank_ah_avail\":%.1f,"
      "\"powertank_tmin\":%s,"
      "\"powertank_tmax\":%s,"
      "\"powertank_crate\":%.3f,"
      "\"session_wh\":%.1f,"
      "\"session_ah\":%.2f,"
      "\"ramp_rate_wps\":%d,"
      "\"target_volt_dv\":%d,"
      "\"uptime_s\":%lu,"
      "\"rssi\":%d,"
      "\"version\":%lld,"
      "\"cpu0\":%d,"
      "\"cpu1\":%d,"
      "\"free_heap_kb\":%.1f,"
      "\"heartbeat_ok\":%s,"
      "\"ramp_phase\":\"%s\","
      "\"eta_minutes\":%d,"
      "\"charger_count\":%d,"
      "\"charging_enabled\":%s,"
      "\"target_power_w\":%d,"
      "\"current_power_w\":%d,"
      "\"thermal_throttle\":%s,"
      // Outright charge inhibit (outside the cell's safe envelope). Distinct
      // from thermal_throttle, which means "charging, but at reduced power".
      // Emitted as a fixed string from chargeInhibitName() — no user-controlled
      // data, so no escaping needed.
      "\"charge_inhibit\":\"%s\","
      // Highest-severity active protection state, for the dashboard banner.
      // Priority lives in activeProtection() so the firmware and the UI can't
      // disagree. Fixed key from protStateKey() — no user data, no escaping.
      "\"protection\":\"%s\","
      "\"cycle_count\":%d,"
      "\"chargers\":[",
    liveSnap.dataFresh         ? "true" : "false",
    apIsBroadcasting()         ? "true" : "false",
    liveSnap.monolithVoltageDv  / 10.0f,
    (float)liveSnap.monolithAmps,
    (float)liveSnap.monolithAH,
    monolithAhAvail,
    tMonMin,
    tMonMax,
    liveSnap.monolithMaxCRate   / 10.0f,
    monolithSoc,
    monolithSocSource,
    liveSnap.powerTankPresent  ? "true" : "false",
    liveSnap.powerTankDecided  ? "true" : "false",
    liveSnap.powerTankVoltageDv / 10.0f,
    (float)liveSnap.powerTankAmps,
    (float)liveSnap.powerTankAH,
    powerTankAhAvail,
    tPtMin,
    tPtMax,
    liveSnap.powerTankMaxCRate  / 10.0f,
    sessWh,
    sessAh,
    (int)ctrlSnap.rampStepW,
    (int)ctrlSnap.targetVoltDv,
    millis() / 1000UL,
    (int)WiFi.RSSI(),
    (long long)VERSION,
    (int)statsSnap.load0,
    (int)statsSnap.load1,
    statsSnap.freeHeap / 1024.0f,
    chargerSnap.heartbeatOk ? "true" : "false",
    g_rampPhase == 1 ? "absorption" : g_rampPhase == 2 ? "float" : "bulk",
    (int)g_etaMinutes,
    (int)ctrlSnap.chargerCount,
    ctrlSnap.enabled ? "true" : "false",
    (int)ctrlSnap.targetPowerW,
    (int)ctrlSnap.currentPowerW,
    g_thermalThrottle ? "true" : "false",
    chargeInhibitName(g_chargeInhibit),
    protStateKey(activeProtection()),
    (int)g_cycleCount
  );

  // Guard against truncation: snprintf returns the length it WOULD have written.
  // If that meets/exceeds the buffer the fixed block was clipped — emit a small
  // valid JSON error instead of a malformed body the dashboard can't parse.
  if (jsonLen < 0 || jsonLen >= (int)sizeof(buf)) {
    LOG("[API] status JSON truncated (%d >= %u) — sending error stub\n",
        jsonLen, (unsigned)sizeof(buf));
    return String("{\"fresh\":false,\"error\":\"status buffer overflow\"}");
  }

  // Build the charger array dynamically — one entry per present charger
  String response = String(buf);
  bool first = true;
  unsigned long now = millis();
  for (int i = 0; i < MAX_CHARGERS; i++) {
    const ChargerUnit& c = chargerSnap.chargers[i];
    if (!c.present) continue;
    // Treat charger as gone if no frame received within timeout window
    if ((now - c.lastSeenMs) > CHARGER_TIMEOUT_MS) continue;
    char entry[100];
    snprintf(entry, sizeof(entry),
      "%s{\"id\":%d,\"v\":%.1f,\"a\":%.1f,\"status\":%d}",
      first ? "" : ",",
      i,
      c.voltDv / 10.0f,
      c.ampsDa / 10.0f,
      (int)c.status
    );
    response += entry;
    first = false;
  }
  // Append cell balance, BMS board temp, per-cell avg, odometer, then close.
  // odometer_km is -1 when no dash frame has been decoded yet (dashboard shows "—").
  char cellBuf[128];
  snprintf(cellBuf, sizeof(cellBuf),
    "],\"cell_balance_mv\":%u,\"bms_board_temp\":%d,\"cell_avg_mv\":%u,\"odometer_km\":%.1f}",
    (unsigned)liveSnap.cellBalanceMv,
    (int)liveSnap.bmsBoardTempC,
    (unsigned)liveSnap.cellAvgMv,
    liveSnap.odometerHm > 0 ? liveSnap.odometerHm / 10.0f : -1.0f);
  response += cellBuf;
  return response;
}

void handleApiStatus() {
  if (!requireAuth(true)) return;   // API route — JS-side handles 401 redirect
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buildApiStatusJson());
}

// POST /api/control — sets target power, enabled state, charger count,
//                     ramp rate, and target voltage (protected).
// Body: {"target_w": 3300, "enabled": true, "charger_count": 3,
//        "ramp_rate_wps": 50, "target_volt_dv": 1100, "reset_session": true}
// Body parsed via ArduinoJson (replaces an indexOf-based parser that could
// match keys appearing inside string values elsewhere in the payload).
// Only fields actually present in the body are applied; missing fields are
// left unchanged.
// Apply a /api/control JSON body string. Returns JSON ack.
static String applyApiControlBody(const String& body) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, body))
    return "{\"ok\":false,\"error\":\"Invalid JSON\"}";

  bool hasEn   = doc.containsKey("enabled");
  bool en      = doc["enabled"]        | false;
  int  tw      = doc["target_w"]       | -1;
  int  cc      = doc["charger_count"]  | -1;
  int  rr      = doc["ramp_rate_wps"]  | -1;
  int  tvd     = doc["target_volt_dv"] | -1;
  bool resetSession = doc["reset_session"] | false;

  // M1: a controlMutex timeout used to fall straight through to
  // {"ok":true} — the caller was told the command landed when nothing had been
  // written. That silently swallowed {"enabled":false}, i.e. a stop-charging
  // request. Report the failure so the client (or Home Assistant) can retry.
  if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
    LOG("[CTRL] controlMutex timeout — command NOT applied "
        "(target=%d enabled=%s chargers=%d ramp=%d tgtV=%d)\n",
        tw, hasEn ? (en ? "true" : "false") : "(unchanged)", cc, rr, tvd);
    return "{\"ok\":false,\"error\":\"Controller busy, command not applied — retry\"}";
  }
  // B3: the user is driving the charge now, so the home WiFi profile must not
  // overwrite it if STA comes up later (slow router → SoftAP session → STA
  // associates mid-charge). Latching the flag here makes the choice sticky for
  // the rest of the boot. charger_count and ramp_rate are deliberately NOT
  // counted — they are configuration, not a charge decision.
  if (tw >= 0 || hasEn || (tvd >= TARGET_VOLT_PRESETS[0].dv &&
                           tvd <= (int)MAX_CHARGE_VOLTAGE_DV)) {
    homeDefaultsApplied = true;
  }
  if (tw >= 0) ctrl.targetPowerW = (uint16_t)constrain(tw, 0, 13200);
  if (hasEn) {
    ctrl.enabled = en;
    if (!en) ctrl.currentPowerW = 0;
  }
  if (cc >= 1 && cc <= (int)MAX_ACTIVE_CHARGERS) {
    ctrl.chargerCount = (uint8_t)cc;
    preferences.putUChar("charger_count", (uint8_t)cc);
  }
  if (rr >= 10 && rr <= 500) {
    ctrl.rampStepW = (uint16_t)rr;
    preferences.putUShort("ramp_step_w", (uint16_t)rr);
  }
  if (tvd >= TARGET_VOLT_PRESETS[0].dv && tvd <= (int)MAX_CHARGE_VOLTAGE_DV) {
    ctrl.targetVoltDv = (uint16_t)tvd;
    preferences.putUShort("target_volt_dv", (uint16_t)tvd);
  }
  xSemaphoreGive(controlMutex);

  if (resetSession) {
    sessionReset();
    LOG("[CTRL] Session reset\n");
  }
  LOG("[CTRL] target=%dW enabled=%s chargers=%d ramp=%dW/s tgtV=%ddV\n",
      tw, hasEn ? (en ? "true" : "false") : "(unchanged)",
      cc > 0 ? cc : (int)ctrl.chargerCount,
      rr > 0 ? rr : (int)ctrl.rampStepW,
      tvd > 0 ? tvd : (int)ctrl.targetVoltDv);
  return "{\"ok\":true}";
}

void handleApiControl() {
  if (!requireAuth(true)) return;   // API route
  if (g_rawTooLarge) {
    server.send(413, "application/json", "{\"ok\":false,\"error\":\"Payload too large\"}");
    return;
  }
  if (g_rawBody.length() == 0) {
    server.send(400, "text/plain", "No body");
    return;
  }
  String result = applyApiControlBody(g_rawBody);
  // M1: distinguish "your request was malformed" (400) from "the controller was
  // busy and applied nothing" (503) — the latter is retryable and the dashboard
  // must not treat it as success.
  int code = (result.indexOf("\"ok\":true") >= 0) ? 200
           : (result.indexOf("Controller busy") >= 0) ? 503 : 400;
  server.send(code, "application/json", result);
}

// ---------------------------------------------------------------------------
// HMAC-signed OTA — verification state and helpers (security pass round 2)
//
//   The .bin produced by the build hook has a 32-byte HMAC-SHA256 trailer
//   appended after the firmware image. The trailer is over the *image* bytes
//   only, keyed with OTA_HMAC_SECRET[32] from ota_secret.h.
//
//   On upload we stream all bytes through mbedtls_md_hmac_update() except the
//   trailing 32, which we hold in a sliding window. When the upload ends we
//   pop the 32 bytes from the window (those are the candidate HMAC), feed any
//   image bytes still in the window through the HMAC, finalize, and compare
//   against the candidate using a constant-time check. Mismatch → Update.abort().
//
//   This blocks the "OTA creds leaked → arbitrary firmware flashed" attack:
//   without the secret an attacker can't produce a valid trailer, so an
//   unsigned (or wrongly-signed) .bin is rejected before any flash write
//   commits.
//
//   Limitation: an attacker with physical access can extract the secret from
//   flash. They can already reflash via USB though, so the secret being in
//   flash is not widening the attack surface.
//
//   This block lives ABOVE the OTA handlers so handleOTAPost can read the
//   sticky failure flag set by handleOTAUpload during the multipart stream.
// ---------------------------------------------------------------------------
static mbedtls_md_context_t otaHmacCtx;
static bool                 otaHmacInited = false;
// Sliding window of the most recent OTA_HMAC_LEN bytes. Anything older has
// already been hashed AND written to the partition. At UPLOAD_FILE_END this
// window holds exactly the candidate HMAC trailer.
static const size_t OTA_HMAC_LEN = 32;
static uint8_t otaHmacWin[OTA_HMAC_LEN];
static size_t  otaHmacWinFill   = 0;     // 0..OTA_HMAC_LEN
static bool    otaUploadFailed  = false; // sticky: any error bails out the rest
static size_t  otaImageBytes    = 0;     // bytes actually flashed (image, no HMAC)
// NET-13 interlock: set at UPLOAD_FILE_START when charging is enabled. Distinct
// from otaUploadFailed so handleOTAPost can answer 409 + a specific message
// rather than the generic "signature invalid" 403.
static bool    otaBlockedCharging = false;

static void otaResetHmacState() {
  if (otaHmacInited) {
    mbedtls_md_free(&otaHmacCtx);
    otaHmacInited = false;
  }
  otaHmacWinFill     = 0;
  otaUploadFailed    = false;
  otaBlockedCharging = false;
  otaImageBytes      = 0;
}

// True if the compiled-in OTA secret is still the all-zero placeholder shipped
// in the repo copy of ota_secret.h (finding #13). A real build host overwrites
// it with 32 random bytes. If it's left as zeros, signed-OTA is meaningless —
// anyone could sign an image with the public all-zero key — so we refuse OTA
// entirely rather than "verify" against a known key.
static bool otaSecretIsPlaceholder() {
  uint8_t acc = 0;
  for (size_t i = 0; i < sizeof(OTA_HMAC_SECRET); i++) acc |= OTA_HMAC_SECRET[i];
  return acc == 0;
}

static bool otaHmacBegin() {
  if (otaSecretIsPlaceholder()) {
    LOG("[OTA] REFUSED: OTA_HMAC_SECRET is the all-zero placeholder. Generate a "
        "real key in ota_secret.h + sign_ota.py before using OTA.\n");
    return false;
  }
  mbedtls_md_init(&otaHmacCtx);
  const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (!info) return false;
  if (mbedtls_md_setup(&otaHmacCtx, info, 1 /* HMAC */) != 0) return false;
  if (mbedtls_md_hmac_starts(&otaHmacCtx,
                             OTA_HMAC_SECRET, sizeof(OTA_HMAC_SECRET)) != 0) {
    mbedtls_md_free(&otaHmacCtx);
    return false;
  }
  otaHmacInited = true;
  return true;
}

// Constant-time byte compare — avoids leaking match progress via timing.
static bool otaConstTimeEqual(const uint8_t* a, const uint8_t* b, size_t n) {
  uint8_t diff = 0;
  for (size_t i = 0; i < n; i++) diff |= (a[i] ^ b[i]);
  return diff == 0;
}

// Process one chunk of upload bytes. We keep the last OTA_HMAC_LEN bytes of the
// stream in otaHmacWin (the candidate HMAC trailer); everything before that is
// committed image — fed through the HMAC and flashed via Update.write().
//
// Finding #18: emit in CONTIGUOUS BLOCKS rather than one byte at a time (the
// old version called mbedtls_md_hmac_update()/Update.write() per byte). The
// exact same byte ranges are hashed and written, so HMAC and flash content
// always agree — a bug here can only ever cause an HMAC mismatch (→ rejected,
// no flash commit), never a corrupt or partial image.
static bool otaProcessChunk(const uint8_t* buf, size_t n) {
  // Conceptual stream = otaHmacWin[0..winFill) followed by buf[0..n).
  // We must end with the LAST OTA_HMAC_LEN bytes back in otaHmacWin.
  size_t total = otaHmacWinFill + n;
  if (total <= OTA_HMAC_LEN) {
    // Not enough yet to commit anything — just grow the window.
    memcpy(otaHmacWin + otaHmacWinFill, buf, n);
    otaHmacWinFill += n;
    return true;
  }

  size_t emit = total - OTA_HMAC_LEN;  // bytes to hash+flash this call

  // Block 1: bytes that come from the existing window (its oldest `fromWin`).
  size_t fromWin = (emit < otaHmacWinFill) ? emit : otaHmacWinFill;
  if (fromWin > 0) {
    if (mbedtls_md_hmac_update(&otaHmacCtx, otaHmacWin, fromWin) != 0) return false;
    if (Update.write(otaHmacWin, fromWin) != fromWin) return false;
    otaImageBytes += fromWin;
  }
  // Block 2: remaining emitted bytes come from the head of buf.
  size_t fromBuf = emit - fromWin;
  if (fromBuf > 0) {
    if (mbedtls_md_hmac_update(&otaHmacCtx, buf, fromBuf) != 0) return false;
    if (Update.write((uint8_t*)buf, fromBuf) != (int)fromBuf) return false;
    otaImageBytes += fromBuf;
  }

  // Rebuild the window with the trailing OTA_HMAC_LEN bytes, in order:
  //   leftover window bytes [fromWin..winFill)  ++  leftover buf bytes [fromBuf..n)
  uint8_t newWin[OTA_HMAC_LEN];
  size_t pos = 0;
  for (size_t i = fromWin; i < otaHmacWinFill; i++) newWin[pos++] = otaHmacWin[i];
  for (size_t i = fromBuf; i < n; i++)              newWin[pos++] = buf[i];
  // pos == OTA_HMAC_LEN by construction (total - emit == OTA_HMAC_LEN).
  memcpy(otaHmacWin, newWin, pos);
  otaHmacWinFill = pos;
  return true;
}

// GET /update — OTA page (session-cookie protected)
void handleOTAGet() {
  if (!requireAuth(false)) return;  // HTML route
  server.send_P(200, "text/html; charset=utf-8", HTML_OTA);
}

// POST /update completion handler
void handleOTAPost() {
  if (!requireAuth(false)) return;  // browser form-context, return code <-> redirect
  // NET-13 interlock first: the upload handler refused the image because
  // charging was enabled. Answer with a specific 409 — the OTA page's
  // xhr.onload prints xhr.responseText for any non-200, so the user sees this
  // sentence verbatim instead of the generic signature-failure text.
  if (otaBlockedCharging) {
    otaBlockedCharging = false;
    server.send(409, "text/plain",
                "Charging is enabled — stop charging before updating firmware.");
    LOG("[OTA] FAILED: charging enabled (409)\n");
    return;
  }
  // Check the HMAC-verification flag before Update.hasError() — the upload
  // handler may have called Update.abort() on a signature mismatch, and
  // hasError() doesn't always reflect an explicit abort.
  if (otaUploadFailed) {
    server.send(403, "text/plain",
                "Update rejected: signature invalid or upload incomplete.\n");
    LOG("[OTA] FAILED: rejected by HMAC verification\n");
    return;
  }
  if (Update.hasError()) {
    server.send(500, "text/plain",
                String("Update failed: ") + Update.errorString());
    LOG("[OTA] FAILED: %s\n", Update.errorString());
  } else {
    server.send(200, "text/plain", "OK");
    LOG("[OTA] Success. Restarting...\n");
    delay(500);
    stopChargerForRestart("OTA complete");
    ESP.restart();
  }
}

// POST /update body handler — called by WebServer as multipart chunks arrive.
// Auth MUST be checked here — WebServer calls this for each chunk BEFORE
// the POST completion handler (handleOTAPost) runs. We use the silent variant
// so we don't try to send a 401 response in the middle of a multipart stream.
// HMAC verification state and helpers are declared above handleOTAGet.
void handleOTAUpload() {
  if (!authValidNoChallenge()) return;
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    otaResetHmacState();
    LOG("[OTA] Start: field='%s' file='%s'\n",
                  upload.name.c_str(), upload.filename.c_str());

    // NET-13 interlock — never flash while the chargers are commanded on.
    // Update.end() reboots into the new image; doing that mid-charge means the
    // last CAN command the DigiNow units heard was a START, and the flash write
    // stalls this task long enough that the 1 Hz heartbeat (and with it the
    // dead-man STOP) is unreliable. Refuse the upload outright instead.
    // A controlMutex timeout is treated as "enabled" — if we can't prove
    // charging is off, we don't flash.
    bool chargingOn = true;
    if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      chargingOn = ctrl.enabled;
      xSemaphoreGive(controlMutex);
    } else {
      LOG("[OTA] controlMutex timeout — assuming charging is enabled\n");
    }
    if (chargingOn) {
      LOG("[OTA] Rejected: charging is enabled — stop charging first\n");
      otaBlockedCharging = true;
      otaUploadFailed    = true;   // makes UPLOAD_FILE_WRITE/END bail out too
      return;                      // NOT calling Update.begin()
    }

    if (!otaHmacBegin()) {
      LOG("[OTA] HMAC init failed — aborting upload\n");
      otaUploadFailed = true;
      return;
    }
    // Defensively clear any stale Update state from a prior interrupted upload
    // (browser disconnect, network drop). Without this, the new ESP32 core's
    // Update.begin() returns false with errorString()=="No Error" because its
    // internal _size is still non-zero from the prior attempt.
    Update.abort();
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      LOG("[OTA] begin() error: %s\n", Update.errorString());
      otaUploadFailed = true;
      return;
    }

  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (otaUploadFailed) return;
    if (!otaProcessChunk(upload.buf, upload.currentSize)) {
      LOG("[OTA] chunk processing failed — aborting\n");
      Update.abort();
      otaUploadFailed = true;
    }

  } else if (upload.status == UPLOAD_FILE_END) {
    if (otaUploadFailed) {
      // Already aborted earlier; nothing more to do here.
      return;
    }
    // The sliding window now holds exactly the candidate HMAC trailer.
    // (Upload smaller than 32 bytes? Then the window is short and we have
    // no room for an image — definitely not a valid signed firmware.)
    if (otaHmacWinFill < OTA_HMAC_LEN) {
      LOG("[OTA] Upload too small (%u bytes total) to contain HMAC trailer — "
          "rejecting\n", (unsigned)upload.totalSize);
      Update.abort();
      otaUploadFailed = true;
      return;
    }
    uint8_t calc[OTA_HMAC_LEN];
    if (mbedtls_md_hmac_finish(&otaHmacCtx, calc) != 0) {
      LOG("[OTA] HMAC finalize failed — rejecting\n");
      Update.abort();
      otaUploadFailed = true;
      return;
    }
    if (!otaConstTimeEqual(calc, otaHmacWin, OTA_HMAC_LEN)) {
      LOG("[OTA] HMAC mismatch — image is not signed by this build host. "
          "Rejecting (image=%u bytes).\n", (unsigned)otaImageBytes);
      Update.abort();
      otaUploadFailed = true;
      return;
    }
    LOG("[OTA] HMAC verified — finalising flash (%u bytes)\n",
        (unsigned)otaImageBytes);
    if (Update.end(true)) {
      LOG("[OTA] Done. %u bytes written.\n", (unsigned)otaImageBytes);
    } else {
      LOG("[OTA] end() error: %s\n", Update.errorString());
      otaUploadFailed = true;
    }
  }
}

// ---------------------------------------------------------------------------
// BOOT button (GPIO 0) handler
//
// Polled from loop(). Two recognised hold patterns:
//   ≥ 5 s   on release  → wipe saved WiFi credentials AND every login session
//                         (RAM table + the NVS "auth_sess" blob behind "keep
//                         me signed in" — SEC-5b), restart into AP mode for
//                         re-provisioning. Keeps AP/MQTT/charger-count
//                         settings intact.
//   ≥ 10 s  on threshold → factory reset: clear the entire "wifi-config" NVS
//                          namespace and restart. Acts immediately so the
//                          user gets a reboot as feedback without releasing.
//
// Releasing before 5 s does nothing (the < 1 s "toggle charging" pattern was
// intentionally skipped to avoid accidental triggers from packaging knocks).
// ---------------------------------------------------------------------------

static unsigned long bootBtnPressedAt   = 0;     // millis() when press began, 0 = idle
static bool          bootBtnFactoryDone = false; // latched after 10 s threshold to suppress release-handler

void checkBootButton() {
  bool pressed = (digitalRead(BOOT_BUTTON_PIN) == LOW);
  unsigned long now = millis();

  if (pressed && bootBtnPressedAt == 0) {
    // Press started
    bootBtnPressedAt   = now;
    bootBtnFactoryDone = false;
    return;
  }

  if (pressed && bootBtnPressedAt > 0) {
    // Still held — check 10 s threshold and act immediately if reached
    if (!bootBtnFactoryDone && (now - bootBtnPressedAt) >= BTN_HOLD_FACTORY_MS) {
      bootBtnFactoryDone = true;
      LOG("[BTN] BOOT held %lus → FACTORY RESET (clearing NVS)\n",
          (now - bootBtnPressedAt) / 1000UL);
      preferences.clear();   // wipes the entire "wifi-config" namespace
      delay(300);            // give the log line time to flush over serial / SSE
      stopChargerForRestart("factory reset");
      ESP.restart();
    }
    return;
  }

  if (!pressed && bootBtnPressedAt > 0) {
    // Released — decide based on hold duration
    unsigned long heldMs = now - bootBtnPressedAt;
    bootBtnPressedAt = 0;
    if (bootBtnFactoryDone) return;  // already restarted; can't actually reach here
    if (heldMs >= BTN_HOLD_AP_RESET_MS) {
      // SEC-5(b): also drop every login session — RAM table and the NVS
      // "auth_sess" blob that backs "keep me signed in". This hold is the
      // hand-the-device-on / lost-network recovery gesture, so a previously
      // remembered browser must not still be authenticated after it.
      LOG("[BTN] BOOT held %lus → clearing WiFi creds + all login sessions, "
          "restarting into AP mode\n", heldMs / 1000UL);
      preferences.remove("ssid");
      preferences.remove("pass");
      sessionsClearAll("BOOT button held 5-10s");
      delay(300);
      stopChargerForRestart("WiFi creds cleared, AP mode");
      ESP.restart();
    } else if (heldMs >= 3000UL) {
      // 3–5 s window: clear the auth hard-lock for ALL client IPs if any is
      // active. This is the documented manual-unlock path for users locked out
      // of the dashboard after too many failed login attempts. Note: only takes
      // effect on release, so a press that grows past 5 s into AP-reset
      // territory wins instead. Sessions persist; only the lock + fail counters
      // clear. authAnyLockOrFails()/hardLockManualClear() both take authMutex
      // internally, so this is race-safe against a concurrent login attempt.
      if (authAnyLockOrFails()) {
        hardLockManualClear("BOOT button held 3-5s");
      } else {
        LOG("[BTN] BOOT pressed %lums (no auth lock to clear)\n", heldMs);
      }
    } else if (heldMs >= 1000UL) {
      // 1–3 s window: too short for any action. Log it so the user sees the
      // press registered.
      LOG("[BTN] BOOT pressed %lums (no action — hold ≥3s for auth unlock, "
          "≥5s for AP reset, ≥10s for factory)\n", heldMs);
    }
  }
}

// ---------------------------------------------------------------------------
// WiFi helpers
// ---------------------------------------------------------------------------

// mDNS hostname — controller is reachable at http://supercharger.local/
// from any machine on the same L2 segment that supports mDNS / Bonjour.
// Works in both STA and AP mode (clients joined to the AP can use it too).
static const char MDNS_HOSTNAME[] = "supercharger";

static void startMdns() {
  // MDNS.begin() is safe to call repeatedly only after MDNS.end(); without
  // the end() the second begin() leaks the prior service registration.
  MDNS.end();
  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    LOG("[MDNS] Started: http://%s.local/\n", MDNS_HOSTNAME);
  } else {
    LOG("[MDNS] Failed to start\n");
  }
}

// Apply the fixed SoftAP IP configuration if enabled and valid. MUST be called
// before WiFi.softAP(). When disabled, or any of the three stored strings
// don't parse as IPv4, does nothing — the ESP32 default (192.168.4.1) stands.
static void applyApStaticIp() {
  if (!apStaticIpEnabled) return;
  IPAddress ip, gw, sn;
  if (!ip.fromString(apStaticIp) ||
      !gw.fromString(apStaticGateway) ||
      !sn.fromString(apStaticSubnet)) {
    LOG("[AP] Fixed IP config invalid (%s / %s / %s) — using default\n",
        apStaticIp, apStaticGateway, apStaticSubnet);
    return;
  }
  if (WiFi.softAPConfig(ip, gw, sn)) {
    LOG("[AP] Fixed SoftAP IP %s (gw %s, mask %s)\n",
        apStaticIp, apStaticGateway, apStaticSubnet);
  } else {
    LOG("[AP] softAPConfig() rejected %s — using default\n", apStaticIp);
  }
}

// Persist the BSSID + primary channel of the current STA association so the
// next cold boot can use the directed WiFi.begin() fast-connect path and skip
// the all-channel scan (~1-2 s faster). Write-if-changed to avoid needless NVS
// churn on every reconnect. Stored in the open "wifi-config" namespace.
void saveWifiFastConnect() {
  uint8_t* bssid = WiFi.BSSID();
  int      ch    = WiFi.channel();
  if (bssid == nullptr || ch < 1 || ch > 14) return;
  uint8_t cur[6];
  bool same = ((int)preferences.getUChar("wifi_ch", 0) == ch) &&
              (preferences.getBytes("wifi_bssid", cur, sizeof(cur)) == sizeof(cur));
  if (same) {
    for (int i = 0; i < 6; i++) if (cur[i] != bssid[i]) { same = false; break; }
  }
  if (same) return;
  preferences.putBytes("wifi_bssid", bssid, 6);
  preferences.putUChar("wifi_ch", (uint8_t)ch);
  LOG("[WIFI] Cached AP %02X:%02X:%02X:%02X:%02X:%02X ch %d for fast-connect\n",
      bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5], ch);
}

// Start a STA connection to the given creds. If a BSSID+channel cached by a
// previous successful connect is present in NVS, use the directed
// WiFi.begin(ssid, pass, channel, bssid) form to skip the channel scan.
// Returns true if the fast/directed form was used (caller stores this so a
// timeout can fall back to a scan).
bool beginStaConnect(const char* ssid, const char* pass) {
  int     ch = (int)preferences.getUChar("wifi_ch", 0);
  uint8_t bssid[6];
  if (ch >= 1 && ch <= 14 &&
      preferences.getBytes("wifi_bssid", bssid, sizeof(bssid)) == sizeof(bssid)) {
    LOG("[WIFI] Fast-connect via cached AP %02X:%02X:%02X:%02X:%02X:%02X ch %d (no scan)\n",
        bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5], ch);
    WiFi.begin(ssid, pass, ch, bssid);
    return true;
  }
  LOG("[WIFI] Connecting with full channel scan (no cached AP yet)\n");
  WiFi.begin(ssid, pass);
  return false;
}

// Human-readable names for the WiFi disconnect reason codes we actually see
// in the field. Anything else is logged numerically.
static const char* wifiDisconnectReasonName(uint8_t reason) {
  switch (reason) {
    case 2:   return "AUTH_EXPIRE";
    case 3:   return "AUTH_LEAVE";
    case 4:   return "ASSOC_EXPIRE";           // AP dropped us for inactivity
    case 8:   return "ASSOC_LEAVE";
    case 15:  return "4WAY_HANDSHAKE_TIMEOUT"; // usually wrong password
    case 200: return "BEACON_TIMEOUT";         // we stopped hearing the AP
    case 201: return "NO_AP_FOUND";
    case 202: return "AUTH_FAIL";
    case 203: return "ASSOC_FAIL";
    case 204: return "HANDSHAKE_TIMEOUT";
    default:  return "?";
  }
}

// STA disconnect event — logs WHY the link dropped (the state machine only
// notices THAT it dropped). Runs in the WiFi event task; LOG() is ring-buffer
// + mutex so cross-task use is fine. Rate-limited to 1 line per 5 s with a
// suppressed-count so an overnight outage can't flood the web log.
void onWifiStaDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  static unsigned long lastLogMs = 0;
  static uint32_t      suppressed = 0;
  uint8_t reason = info.wifi_sta_disconnected.reason;
  unsigned long now = millis();
  if (now - lastLogMs < 5000UL) { suppressed++; return; }
  if (suppressed > 0) {
    LOG("[WIFI] STA disconnected: reason %u (%s) — +%lu more in last 5 s\n",
        reason, wifiDisconnectReasonName(reason), (unsigned long)suppressed);
  } else {
    // (No RSSI here — by the time this event fires the link is down and
    // WiFi.RSSI() reads 0. Watch the dashboard's RSSI card while connected.)
    LOG("[WIFI] STA disconnected: reason %u (%s)\n",
        reason, wifiDisconnectReasonName(reason));
  }
  lastLogMs = now;
  suppressed = 0;
}

void startAPMode() {
  WiFi.disconnect(true);
  // Hostname must be set BEFORE softAP() so it goes into the AP DHCP server's
  // client-name advertisement (clients that join the AP see this name).
  WiFi.softAPsetHostname(MDNS_HOSTNAME);
  applyApStaticIp();  // softAPConfig() — must precede softAP()
  WiFi.softAP(apSSID, apPass);
  LOG("[AP] Started \"%s\" — IP: %s\n", apSSID, WiFi.softAPIP().toString().c_str());
  startMdns();
}

// Attempt connection using the compile-time station credentials
// SECRET_WIFI_SSID / SECRET_WIFI_PASS (the home network).
// Credentials are intentionally NOT written to preferences here —
// secrets remain a read-only bootstrap; the /save page is the only
// path that persists credentials.
void trySecretsConnect() {
  const char* ssid = SECRET_WIFI_SSID;
  const char* pass = SECRET_WIFI_PASS;
  if (ssid == nullptr || strlen(ssid) == 0) {
    // No compile-time station SSID. Do NOT drop straight into SETUP_MODE —
    // this path is also reached after a prefs connect times out (router still
    // booting after a power cut), and SETUP_MODE never retries, so perfectly
    // good NVS credentials would sit unused until the next reboot (NET-3).
    // enterApRetrying() re-reads NVS, keeps retrying in the background, and
    // falls back to SETUP_MODE itself if there really are no credentials.
    LOG("[WIFI] Secrets SSID empty — entering AP+STA retry\n");
    enterApRetrying();
    return;
  }
  LOG("[WIFI] No saved credentials. Trying secrets SSID \"%s\"...\n", ssid);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // modem sleep OFF — see comment at the setup() call
  WiFi.setHostname(MDNS_HOSTNAME);
  WiFi.begin(ssid, pass);
  wifiSource         = WIFI_SRC_SECRETS;
  wifiConnectStartMs = millis();
  currentState       = STATE_CONNECTING;
}

// Bring up AP+STA mode and start the background STA retry loop.
// Pulls credentials from NVS first, then SECRET_WIFI_SSID/PASS as fallback.
// If neither is available there's nothing to retry — drop to AP-only setup mode.
void enterApRetrying() {
  String s = preferences.getString("ssid", "");
  String p = preferences.getString("pass", "");
  if (s.length() == 0 && strlen(SECRET_WIFI_SSID) > 0) {
    s = SECRET_WIFI_SSID;
    p = SECRET_WIFI_PASS;
  }
  if (s.length() == 0) {
    LOG("[WIFI] No STA creds to retry — staying in AP-only setup mode\n");
    startAPMode();
    currentState = STATE_SETUP_MODE;
    return;
  }
  s.toCharArray(retryStaSsid, sizeof(retryStaSsid));
  p.toCharArray(retryStaPass, sizeof(retryStaPass));

  // If the SoftAP is already broadcasting, do NOT tear the radio down and
  // re-softAP() — that kicks every client joined to the AP and re-registers
  // mDNS for no gain. Just make sure the STA interface exists and re-kick it
  // (NET-18c).
  //
  // B1: ask the RADIO, not the state machine. This used to be
  // apIsBroadcasting(), which derives from currentState — and every call site
  // reaches here from STATE_CONNECTING or STATE_CONNECTED, so it was always
  // false and the "already up" branch was dead code. WiFi.getMode() reports
  // what the driver actually has running, which is the fact this decision
  // needs.
  //
  // C7 — to be precise about the branch's reachability: on today's call graph
  // it still cannot be taken. Every caller is in STATE_CONNECTING or
  // STATE_CONNECTED, and neither leaves a SoftAP broadcasting, so apAlreadyUp
  // is false on every path that exists right now. The branch is kept
  // correct-by-construction for the re-entry the flap limiter is expected to
  // introduce (returning here from STATE_AP_RETRYING with the AP deliberately
  // still up), and because sourcing the answer from the radio rather than from
  // currentState is right regardless of who calls. Do not delete it as dead
  // code — deleting it would reintroduce the tear-down it prevents the moment
  // that caller lands.
  const bool apAlreadyUp = (WiFi.getMode() & WIFI_MODE_AP) != 0;
  if (!apAlreadyUp) {
    // AP+STA: both interfaces share the radio. When STA associates, the SoftAP
    // is force-moved to the STA's channel (clients on the AP may briefly drop).
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);  // modem sleep OFF — see comment at the setup() call
    WiFi.softAPsetHostname(MDNS_HOSTNAME);
    applyApStaticIp();  // softAPConfig() — must precede softAP()
    WiFi.softAP(apSSID, apPass);
  } else if (WiFi.getMode() != WIFI_AP_STA) {
    // Came from AP-only SETUP_MODE — add the STA interface without disturbing
    // the running SoftAP.
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);  // modem sleep OFF — see comment at the setup() call
  }
  // Deliberately OUTSIDE the branch above: the STA is (re)started with the
  // credentials on BOTH paths, so the kept-AP path still nudges the station
  // interface back at the router (B1). Without this, keeping the AP up would
  // mean never re-issuing the association request.
  WiFi.setHostname(MDNS_HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(retryStaSsid, retryStaPass);
  if (!apAlreadyUp) startMdns();
  lastStaRetryAt     = millis();
  apStaStableSinceMs = 0;          // grace window starts only after STA is up
  apRetryLastStaUp   = false;      // fresh edge latch for this AP_RETRYING run
  currentState = STATE_AP_RETRYING;
  LOG("[WIFI] AP+STA retry mode — AP \"%s\" %s at %s, STA → \"%s\"\n",
      apSSID, apAlreadyUp ? "kept" : "up",
      WiFi.softAPIP().toString().c_str(), retryStaSsid);
}

// Work that must run on EVERY path to a usable station link, not just the
// first one. There are two such edges: CONNECTING → CONNECTED (boot connect
// succeeded) and the STA false → true edge inside STATE_AP_RETRYING (link came
// back after a drop). Until NET-5 this lived only on the first edge, so a
// device that reached the network via AP_RETRYING never started SNTP — every
// CycleRecord was stamped 1970 — and never applied the home WiFi profile.
// applyHomeWifiBootDefaults() is idempotent (homeDefaultsApplied), so calling
// it from both edges is safe.
//
// firstConnect: true on the boot-time CONNECTING → CONNECTED edge, false on
// the AP_RETRYING reconnect edge. The "charge already running" guard below is
// applied ONLY on the reconnect edge (audit 2026-09, C1): on the first edge
// ctrl.enabled merely reflects the AP/Road boot profile (rampInit copies
// def_chg_en into it before loop() runs), not a charge a human started, and
// treating it as one would permanently skip the Home profile for every user
// whose road profile has charging ON.
static void onStaUp(bool firstConnect) {
  // Kick off NTP sync (non-blocking; result arrives in ~2 s via SNTP task).
  // Timestamps are used by the cycle data logger (appendCycleRecord).
  configTime(0, 0, "pool.ntp.org", "time.cloudflare.com");
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1); // Europe/Oslo
  tzset();

  // B3: do not apply the home profile over a charge that is already running.
  // The AP_RETRYING edge reaches here with homeDefaultsApplied still false
  // whenever the router was slow to come up — the owner meanwhile joined the
  // SoftAP and started charging by hand, and applying the profile at that
  // moment would silently rewrite their target power, target voltage, and
  // possibly stop the charge outright (home profile default is charging OFF).
  // The user-command paths also latch homeDefaultsApplied now, so this is a
  // second line of defence for a charge started before those paths ran (e.g.
  // the AP/road boot profile had charging enabled).
  if (firstConnect) {
    applyHomeWifiBootDefaults();
    return;
  }
  bool charging     = false;
  bool haveCtrlSnap = false;
  if (controlMutex != nullptr &&
      xSemaphoreTake(controlMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    charging     = ctrl.enabled;
    haveCtrlSnap = true;
    xSemaphoreGive(controlMutex);
  }
  if (!haveCtrlSnap) {
    // Couldn't read the control state — skip applying rather than guess. The
    // profile is a convenience; clobbering a live charge is not recoverable.
    LOG("[WIFI] STA up — controlMutex timeout, home profile not applied\n");
    return;
  }
  if (charging) {
    homeDefaultsApplied = true;   // don't re-attempt on a later edge either
    LOG("[WIFI] STA up mid-session — home profile not applied\n");
    return;
  }
  applyHomeWifiBootDefaults();  // apply home WiFi profile once per boot
}

void monitorWifiStatus() {
  unsigned long now = millis();

  if (currentState == STATE_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      LOG("[WIFI] Connected (%s). IP: %s\n",
                    wifiSource == WIFI_SRC_SECRETS ? "secrets" : "prefs",
                    WiFi.localIP().toString().c_str());
      // Capture hostname for MQTT client ID and topic prefix. Write-if-changed
      // (audit 2026-07): mqttTask reads this char array without a lock, so
      // don't rewrite identical bytes on every reconnect — in practice the
      // hostname never changes after the first association.
      const char* hn = WiFi.getHostname();
      if (hn && strlen(hn) > 0 && strcmp(hn, mqttHostname) != 0)
        snprintf(mqttHostname, sizeof(mqttHostname), "%s", hn);
      wifiFastAttempt = false;
      saveWifiFastConnect();  // remember BSSID+channel for next cold boot's fast-connect
      startMdns();
      currentState = STATE_CONNECTED;
      onStaUp(true);   // NTP + TZ + home WiFi profile (first connect: profile applies unconditionally)
      return;
    }
    // Not yet connected — check whether we've exceeded the timeout.
    if (now - wifiConnectStartMs >= wifiConnectTimeoutMs) {
      if (wifiSource == WIFI_SRC_PREFS && wifiFastAttempt) {
        // The directed (cached-BSSID) attempt timed out — the AP likely changed
        // channel or we'd roamed to a different one. Drop the stale cache and
        // retry the same prefs creds once with a full scan before falling
        // through to secrets / AP retry.
        LOG("[WIFI] Fast-connect timed out — clearing cached AP, retrying with scan\n");
        preferences.remove("wifi_bssid");
        preferences.remove("wifi_ch");
        wifiFastAttempt = false;
        String s = preferences.getString("ssid", "");
        String p = preferences.getString("pass", "");
        WiFi.disconnect(true);
        WiFi.mode(WIFI_STA);
        // Modem sleep OFF — same reason as the setup() call (see the comment
        // there): WIFI_PS_MIN_MODEM is what caused the "connects then drops"
        // bug. WiFi.mode() can restore the driver default, so re-assert it on
        // every bring-up path, this one included (NET-17).
        WiFi.setSleep(false);
        WiFi.setHostname(MDNS_HOSTNAME);
        WiFi.begin(s.c_str(), p.c_str());
        wifiConnectStartMs = millis();   // restart the timeout for the scan attempt
      } else if (wifiSource == WIFI_SRC_PREFS) {
        // Preferences failed — try secrets if available, else AP+STA retry.
        LOG("[WIFI] Prefs connect timed out.\n");
        trySecretsConnect();
      } else {
        // Both prefs and secrets failed (or only secrets were tried).
        // If we still have credentials anywhere, switch to AP+STA retry so
        // the user can reach the dashboard via the AP while STA keeps trying.
        LOG("[WIFI] Initial STA connect timed out — entering AP+STA retry\n");
        enterApRetrying();
      }
    }
    // Still within timeout window — keep waiting, do nothing.

  } else if (currentState == STATE_CONNECTED) {
    if (WiFi.status() != WL_CONNECTED) {
      // STA dropped after a previously successful connection. Bring up AP so
      // the dashboard stays reachable while the driver works on reconnecting.
      // Count the flap first (NET-18c) — the limiter, once engaged, keeps the
      // SoftAP up across the next entries instead of cycling it each time.
      if (apFlapWindowStart == 0 || (now - apFlapWindowStart) >= AP_FLAP_WINDOW_MS) {
        apFlapWindowStart    = now;   // start a fresh 10-minute window
        apFlapCount          = 0;
        apFlapLimited        = false;
        apFlapLimitedSinceMs = 0;     // B2: keep the two in step
      }
      if (apFlapCount < 255) apFlapCount++;
      if (apFlapCount > AP_FLAP_MAX && !apFlapLimited) {
        apFlapLimited        = true;
        apFlapLimitedSinceMs = now;   // B2: hold is timed from HERE, not from
                                      // the start of the counting window
        LOG("[WIFI] Link flapping (%u drops in %lu s) — holding SoftAP up, "
            "STA retry nudges continue every %lu s\n",
            (unsigned)apFlapCount,
            (unsigned long)((now - apFlapWindowStart) / 1000UL),
            (unsigned long)(staRetryIntervalMs / 1000UL));
      }
      LOG("[WIFI] STA dropped — switching to AP+STA retry mode\n");
      enterApRetrying();
    }

  } else if (currentState == STATE_AP_RETRYING) {
    // Track STA up/down transitions; AP keeps serving the dashboard until
    // STA has been stable for AP_GRACE_MS, at which point we drop AP and
    // return to STA-only (STATE_CONNECTED).
    bool staUp = (WiFi.status() == WL_CONNECTED);

    if (staUp && !apRetryLastStaUp) {
      LOG("[WIFI] STA reconnected. IP: %s (AP stays up; grace window started)\n",
          WiFi.localIP().toString().c_str());
      const char* hn = WiFi.getHostname();
      if (hn && strlen(hn) > 0 && strcmp(hn, mqttHostname) != 0)
        snprintf(mqttHostname, sizeof(mqttHostname), "%s", hn);
      // Re-bind mDNS — STA-side service registration needs the new IP.
      startMdns();
      saveWifiFastConnect();  // refresh cached BSSID+channel after a reconnect
      onStaUp(false);         // NTP + TZ + home WiFi profile unless a charge is running (NET-5/B3)
      apStaStableSinceMs = now;    // begin counting toward AP teardown
    } else if (!staUp && apRetryLastStaUp) {
      LOG("[WIFI] STA dropped again — grace timer reset, driver will retry\n");
      apStaStableSinceMs = 0;      // reset: must be continuously up
    } else if (staUp && apStaStableSinceMs == 0) {
      // Safety net (NET-18b): STA is up but the grace timer was never armed —
      // the false→true edge was missed (e.g. the link came up between polls of
      // a state entry). Without this the SoftAP would stay up forever.
      LOG("[WIFI] STA up but grace timer unarmed — arming now\n");
      apStaStableSinceMs = now;
    }
    apRetryLastStaUp = staUp;

    // Flap limiter expiry (NET-18c, corrected in B2): the hold lasts
    // AP_FLAP_WINDOW_MS from the moment it ENGAGED. Timing it from
    // apFlapWindowStart was wrong — that variable is only written in the
    // STATE_CONNECTED drop branch, which never executes again once we are
    // parked in AP_RETRYING, so it could never advance and the hold expired
    // almost immediately. Resetting the counter state here too means the next
    // flap starts a clean window rather than resuming an exhausted one.
    if (apFlapLimited && (now - apFlapLimitedSinceMs) >= AP_FLAP_WINDOW_MS) {
      apFlapLimited        = false;
      apFlapLimitedSinceMs = 0;
      apFlapCount          = 0;
      apFlapWindowStart    = 0;
      LOG("[WIFI] Flap limiter expired — SoftAP teardown re-enabled\n");
    }

    // AP teardown: STA has been continuously up for the full grace window.
    // Suppressed while the flap limiter is engaged — on a flapping link the
    // AP would otherwise be torn down and brought straight back up again.
    if (staUp && !apFlapLimited && apStaStableSinceMs != 0 &&
        (now - apStaStableSinceMs) >= AP_GRACE_MS) {
      LOG("[WIFI] STA stable for %lu ms — tearing down SoftAP, returning to STA-only\n",
          (unsigned long)(now - apStaStableSinceMs));
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_STA);
      apStaStableSinceMs = 0;
      currentState = STATE_CONNECTED;
      // Re-register mDNS now that the AP-side iface is gone, so service
      // records advertise only the STA IP. Safe to call repeatedly.
      startMdns();
      return;
    }

    // Belt-and-braces: nudge the driver every staRetryIntervalMs while STA is down.
    // Arduino-ESP32 auto-reconnects on its own, but some failure modes leave the
    // STA stuck disconnected. WiFi.reconnect() kicks it again with the same creds.
    if (!staUp && (now - lastStaRetryAt) >= staRetryIntervalMs) {
      LOG("[WIFI] Nudging STA reconnect to \"%s\"...\n", retryStaSsid);
      WiFi.reconnect();
      lastStaRetryAt = now;
    }
  }
  // STATE_SETUP_MODE: terminal by design, and since NET-3 it is only ever
  // reached when NO credentials exist anywhere (neither NVS nor compile-time
  // secrets) — so there is nothing to retry. Wait for /save to write creds and
  // restart. Any state that still has credentials uses AP_RETRYING instead.
}

// True whenever the SoftAP is currently broadcasting (covers both the
// "no creds" setup mode and the "creds present, retrying" recovery mode).
// Used by the dashboard's ap_mode JSON field so the UI can hint that the
// AP fallback is reachable.
static inline bool apIsBroadcasting() {
  return currentState == STATE_SETUP_MODE || currentState == STATE_AP_RETRYING;
}

// ---------------------------------------------------------------------------
// Cycle data API — download / clear /cycles.csv from FFat
// ---------------------------------------------------------------------------

// GET /api/cycles — stream the cycle history CSV as a file download.
// Returns 404 if no cycles have been recorded yet.
void handleApiCyclesGet() {
  if (!requireAuth(true)) return;
  if (!g_fatReady) { server.send(503, "text/plain", "FAT not mounted"); return; }
  File f = FFat.open("/cycles.csv", FILE_READ);
  if (!f) { server.send(404, "text/plain", "No data yet"); return; }
  server.sendHeader("Content-Disposition", "attachment; filename=\"cycles.csv\"");
  server.streamFile(f, "text/csv");
  f.close();
}

// DELETE /api/cycles — wipe cycle history (creates a clean slate for next cycle).
void handleApiCyclesDelete() {
  if (!requireAuth(true)) return;
  if (!g_fatReady) { server.send(503, "text/plain", "FAT not mounted"); return; }
  FFat.remove("/cycles.csv");
  g_cycleCount = 0;
  server.send(200, "text/plain", "Cleared");
  LOG("[FAT] cycles.csv deleted by user\n");
}

// ---------------------------------------------------------------------------
// Request-handler shims (audit 2026-07)
//
// WebBodyGuardHandler — catch-all registered LAST in setup() so it only sees
// requests no explicit route matched. It (a) reproduces the previous
// not-found behaviour (404, or the port-80→443 redirect when g_httpsRunning),
// and (b) declares canRaw so the WebServer streams-and-discards the body of
// any unmatched POST/PUT/PATCH/DELETE through its fixed ~1.4 KB chunk buffer
// instead of malloc()ing the whole Content-Length into RAM before any auth
// check could run.
//
// OtaUpdateHandler — replaces the plain server.on("/update", POST, fn, ufn)
// registration. On ESP32 core 3.x a route with an upload function is ALSO
// raw-capable, so a non-multipart POST to /update used to invoke
// handleOTAUpload() through the raw path, where server.upload() dereferences
// a null unique_ptr → LoadProhibited crash. This handler sends multipart
// bodies to the real upload handler and quietly drains anything else,
// flagging the attempt so the completion handler answers 403.
//
// RawBodyHandler — replaces server.on(uri, method, fn, rawBodyCollect) on
// every body-consuming route (SEC-1). The mirror image of the /update bug:
// FunctionRequestHandler uses ONE _ufn for both upload() and raw(), and its
// canUpload() returns true for any POST once _ufn is set
// (RequestHandlersImpl.h). A multipart POST carrying a filename part therefore
// took Parsing.cpp's _parseForm() path → upload() → the collector →
// server.raw() → *_currentRaw, a unique_ptr that is only reset() on the
// non-form branch (Parsing.cpp) → null deref → panic, reachable with no
// credentials on /login, /save, /api/settings, /api/control, /api/tls,
// /logout and DELETE /api/cycles. RawBodyHandler declares canUpload() false
// (so the core never routes a multipart part into the collector), keeps
// canRaw() for the streaming collector, and answers 415 in handle() when the
// request announced a multipart Content-Type.
// ---------------------------------------------------------------------------

class WebBodyGuardHandler : public RequestHandler {
public:
  bool canHandle(HTTPMethod, const String&) override { return true; }
  bool canHandle(WebServer&, HTTPMethod, const String&) override { return true; }
  bool canRaw(const String&) override { return true; }
  bool canRaw(WebServer&, const String&) override { return true; }
  bool handle(WebServer& srv, HTTPMethod, const String&) override {
    // g_httpsRunning, not httpsEnabled (SEC-15/NET-6): redirect only when a
    // server is actually listening on 443. httpsEnabled can be true with no
    // live TLS server (toggle saved but not yet rebooted, or start failed).
    if (g_httpsRunning) handleHTTPSRedirect();
    else srv.send(404, "text/plain", "Not found");
    return true;
  }
  void raw(WebServer&, const String&, HTTPRaw&) override { /* discard body */ }
};

class OtaUpdateHandler : public RequestHandler {
public:
  bool canHandle(HTTPMethod m, const String& uri) override {
    return m == HTTP_POST && uri == "/update";
  }
  bool canHandle(WebServer&, HTTPMethod m, const String& uri) override {
    return m == HTTP_POST && uri == "/update";
  }
  bool canUpload(const String& uri) override { return uri == "/update"; }
  bool canUpload(WebServer&, const String& uri) override { return uri == "/update"; }
  bool canRaw(const String&) override { return true; }
  bool canRaw(WebServer&, const String&) override { return true; }
  bool handle(WebServer&, HTTPMethod, const String&) override {
    handleOTAPost();
    return true;
  }
  void upload(WebServer&, const String&, HTTPUpload&) override { handleOTAUpload(); }
  void raw(WebServer&, const String&, HTTPRaw& r) override {
    // Non-multipart POST to /update is never a valid OTA upload. The body
    // drains through the fixed chunk buffer; flag it so handleOTAPost 403s
    // instead of misreading stale Update state as success.
    if (r.status == RAW_START) {
      otaUploadFailed = true;
      LOG("[OTA] Non-multipart POST to /update — rejected\n");
    }
  }
};

class RawBodyHandler : public RequestHandler {
public:
  RawBodyHandler(const char* uri, HTTPMethod method, void (*fn)())
    : _uri(uri), _method(method), _fn(fn) {}

  bool canHandle(HTTPMethod m, const String& uri) override {
    return m == _method && uri == _uri;
  }
  bool canHandle(WebServer&, HTTPMethod m, const String& uri) override {
    return m == _method && uri == _uri;
  }
  // SEC-1: never claim the multipart upload path — that is the whole point of
  // this class. Returning false here makes the core stream-and-discard file
  // parts without ever calling back into us.
  bool canUpload(const String&) override { return false; }
  bool canUpload(WebServer&, const String&) override { return false; }
  // RAW streaming stays on, for the non-multipart bodies these routes expect.
  bool canRaw(const String& uri) override { return uri == _uri; }
  bool canRaw(WebServer&, const String& uri) override { return uri == _uri; }

  bool handle(WebServer& srv, HTTPMethod m, const String& uri) override {
    if (!canHandle(srv, m, uri)) return false;
    // A multipart body never reached the collector, so g_rawBody holds either
    // nothing or the previous request's content. Refuse rather than let the
    // route function act on it. Content-Type is in the collectHeaders() list
    // set up in setup(); the core keeps no public accessor of its own.
    if (srv.header("Content-Type").startsWith("multipart/")) {
      LOG("[WEB] multipart POST to %s — rejected (415)\n", _uri);
      srv.send(415, "text/plain", "multipart not accepted on this route\n");
      return true;
    }
    _fn();
    return true;
  }

  // The capped RAW body collector (was the free function rawBodyCollect()).
  // See the RAW_BODY_CAP block near the auth globals for the rationale.
  void raw(WebServer& srv, const String&, HTTPRaw& r) override {
    switch (r.status) {
      case RAW_START: {
        int cl = srv.clientContentLength();
        g_rawBody     = String();
        g_rawTooLarge = (cl > (int)RAW_BODY_CAP);
        if (!g_rawTooLarge && cl > 0) g_rawBody.reserve(cl + 1);
        break;
      }
      case RAW_WRITE:
        if (!g_rawTooLarge) {
          if (g_rawBody.length() + r.currentSize > RAW_BODY_CAP) {
            // Body exceeded the cap despite the Content-Length check (lying
            // header). Flip to discard mode; chunks keep draining harmlessly.
            g_rawTooLarge = true;
            g_rawBody = String();
          } else {
            g_rawBody.concat((const char*)r.buf, r.currentSize);
          }
        }
        break;
      case RAW_END:
        break;
      case RAW_ABORTED:
        g_rawBody     = String();
        g_rawTooLarge = false;
        break;
    }
  }

private:
  const char* _uri;
  HTTPMethod  _method;
  void      (*_fn)();
};

// ---------------------------------------------------------------------------
// Setup & Loop
// ---------------------------------------------------------------------------

// NET-13: assert a charger STOP before any deliberate reboot.
//
// A reboot takes the 1 Hz heartbeat away, and the DigiNow units do eventually
// self-stop when it goes missing — but only after their own ~5 s timeout, and
// only if the last frame they saw wasn't a START that the boot sequence then
// re-issues. Every restart path therefore disables charging at the source
// (ctrl), raises g_forceStop (which makes sendHeartbeat() transmit STOP
// unconditionally, no mutex involved), zeroes the command struct best-effort,
// and then waits long enough for at least one heartbeat to carry that STOP onto
// the wire before the CPU goes away.
//
// Order matters. rampTask keeps running throughout — vTaskDelay() yields rather
// than spins, which is exactly what has to happen for the STOP to reach the bus
// — and it ticks once a second, so one or two of its ticks land INSIDE the
// 1.5 s wait. With charging still enabled in ctrl, such a tick would take
// chargerMutex, write a fresh START into chargerBus and clear g_forceStop, and
// the reboot would hand the chargers a START instead of the STOP this function
// exists to send. Clearing ctrl.enabled first sends those ticks down rampTask's
// disabled branch (which asserts the STOP itself), and g_shuttingDown stops any
// tick already past that point from clearing g_forceStop behind us.
//
// Safe to call from loopTask (BOOT button) and from the web server tasks alike.
static void stopChargerForRestart(const char* reason) {
  // 1. Disable charging at the source, so rampTask's own ticks during the wait
  //    below command STOP rather than re-asserting the running setpoint.
  if (controlMutex != nullptr &&
      xSemaphoreTake(controlMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    ctrl.enabled       = false;
    ctrl.currentPowerW = 0;
    xSemaphoreGive(controlMutex);
  }
  // 2. Failsafe flags. g_shuttingDown is checked by rampTask at both of its
  //    "a valid START landed under the mutex" clear sites, so from here on
  //    nothing can lower g_forceStop again.
  g_shuttingDown = true;
  g_forceStop    = true;
  // 3. chargerMutex is created in chargerBusInit(); a restart requested before
  //    that point (there is no such path today, but be defensive) just skips the
  //    struct write — g_forceStop alone already forces STOP in sendHeartbeat().
  if (chargerMutex != nullptr &&
      xSemaphoreTake(chargerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    chargerBus.cmdVoltDv = 0;
    chargerBus.cmdAmpsDa = 0;
    chargerBus.cmdStart  = false;
    xSemaphoreGive(chargerMutex);
  }
  LOG("[BOOT] restart (%s) — charger STOP asserted, waiting for heartbeat\n",
      reason ? reason : "?");
  // HEARTBEAT_INTERVAL_MS is 1 s, so 1.5 s guarantees at least one STOP frame.
  vTaskDelay(pdMS_TO_TICKS(1500));
}

void setup() {
  // logBegin() initialises hardware serial and creates the ring buffer mutex.




  logBegin(115200);
  LOG("\n[BOOT] Supercharger firmware %lld\n", (long long)VERSION);
  if (otaSecretIsPlaceholder()) {
    LOG("[BOOT] WARNING: OTA_HMAC_SECRET is the all-zero placeholder — signed "
        "OTA is DISABLED until you generate a real key (ota_secret.h + "
        "sign_ota.py). USB flashing still works.\n");
  }

  // authMutex protects authSessions[] and the per-IP authIpSlots[] lockout
  // table. Must exist BEFORE any HTTP or HTTPS handler can run — both call into
  // the session/lockout helpers, which assume the mutex is available.
  // settingsMutex serialises config mutation across the HTTP and HTTPS server
  // tasks (findings #7/#8). Create it before any handler can run.
  settingsMutex = xSemaphoreCreateMutex();
  if (settingsMutex == nullptr) {
    LOG("[BOOT] Failed to create settingsMutex — halting\n");
    while (true) { delay(1000); }
  }

  authMutex = xSemaphoreCreateMutex();
  if (authMutex == nullptr) {
    LOG("[BOOT] Failed to create authMutex — halting\n");
    while (true) { delay(1000); }
  }

  // BOOT button (GPIO 0). Strapping pin — safe to configure as input AFTER
  // boot completes. Internal pullup is enough; the button shorts to GND.
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  preferences.begin("wifi-config", false);
  String savedSSID = preferences.getString("ssid", "");
  String savedPass = preferences.getString("pass", "");

  // Restore "keep me signed in" sessions from NVS into the RAM session table.
  // Safe to do unlocked — setup() is single-threaded, no server task running.
  sessionsLoadFromNvs();

  // Load persistent charger count (default 3 if not set)
  uint8_t savedCC = (uint8_t)preferences.getUChar("charger_count", 3);
  if (savedCC >= 1 && savedCC <= 4) ctrl.chargerCount = savedCC;
  LOG("[BOOT] Charger count: %d (from NVS)\n", ctrl.chargerCount);

  // Load AP settings from NVS, fallback to secrets, then defaults
  {
    String s = preferences.getString("ap_ssid", "");
    if (s.length() > 0) {
      s.toCharArray(apSSID, sizeof(apSSID));
    } else if (strlen(SECRET_AP_SSID) > 0) {
      strncpy(apSSID, SECRET_AP_SSID, sizeof(apSSID) - 1);
      apSSID[sizeof(apSSID) - 1] = '\0';
    }
    // else keep default "Supercharger"
    String p = preferences.getString("ap_pass", "");
    if (p.length() > 0) {
      p.toCharArray(apPass, sizeof(apPass));
    } else if (strlen(SECRET_AP_PASS) > 0) {
      strncpy(apPass, SECRET_AP_PASS, sizeof(apPass) - 1);
      apPass[sizeof(apPass) - 1] = '\0';
    }
    LOG("[BOOT] AP: \"%s\"\n", apSSID);
  }

  // Load fixed AP-mode IP config: NVS first, then SECRET_AP_* compile-time
  // fallback. A non-empty SECRET_AP_IP is itself the opt-in (enabled=true).
  // The Settings page (NVS) always overrides the compile-time values.
  {
    String ip = preferences.getString("ap_ip", "");
    String gw = preferences.getString("ap_gw", "");
    String sn = preferences.getString("ap_sn", "");
    bool   en = preferences.getBool("ap_ip_en", false);
    if (ip.length() == 0 && strlen(SECRET_AP_IP) > 0) {
      // Nothing saved via Settings — fall back to arduino_secrets.h
      ip = SECRET_AP_IP;
      gw = SECRET_AP_GATEWAY;
      sn = SECRET_AP_SUBNET;
      en = true;   // defining SECRET_AP_IP enables the feature
    }
    ip.toCharArray(apStaticIp,      sizeof(apStaticIp));
    gw.toCharArray(apStaticGateway, sizeof(apStaticGateway));
    sn.toCharArray(apStaticSubnet,  sizeof(apStaticSubnet));
    apStaticIpEnabled = en;
    if (apStaticIpEnabled)
      LOG("[BOOT] AP fixed IP: %s gw %s mask %s\n",
          apStaticIp, apStaticGateway, apStaticSubnet);
  }

  // Load MQTT settings from NVS, fallback to secrets
  {
    String h = preferences.getString("mqtt_host", "");
    if (h.length() > 0) {
      h.toCharArray(mqttHost, sizeof(mqttHost));
    } else {
      strncpy(mqttHost, SECRET_MQTT_HOST, sizeof(mqttHost) - 1);
      mqttHost[sizeof(mqttHost) - 1] = '\0';
    }
    mqttPort = preferences.getUShort("mqtt_port", 1883);
    // TLS toggle + CA cert blob. Default: TLS off (preserves the unencrypted
    // installation path). Cert blob is stored as a NUL-terminated PEM string
    // in NVS — Preferences caps a single string at ~3.5 KB which comfortably
    // fits a CA certificate (typically 1.5–2 KB).
    mqttTls    = preferences.getBool("mqtt_tls", false);
    mqttCaCert = preferences.getString("mqtt_ca", "");
    LOG("[BOOT] MQTT TLS: %s, CA cert: %u bytes\n",
        mqttTls ? "ON" : "off", (unsigned)mqttCaCert.length());
    String u = preferences.getString("mqtt_user", "");
    if (u.length() > 0) {
      u.toCharArray(mqttUser, sizeof(mqttUser));
    } else {
      strncpy(mqttUser, SECRET_MQTT_USER, sizeof(mqttUser) - 1);
      mqttUser[sizeof(mqttUser) - 1] = '\0';
    }
    String bp = preferences.getString("mqtt_pass", "");
    if (bp.length() > 0) {
      bp.toCharArray(mqttBrokerPass, sizeof(mqttBrokerPass));
    } else {
      strncpy(mqttBrokerPass, SECRET_MQTT_BROKER_PASS, sizeof(mqttBrokerPass) - 1);
      mqttBrokerPass[sizeof(mqttBrokerPass) - 1] = '\0';
    }
    LOG("[BOOT] MQTT: %s:%d user=%s\n", mqttHost, mqttPort, mqttUser);
  }

  // Log the driver's reason code whenever the STA link drops — the state
  // machine only sees THAT it dropped, this tells us WHY (beacon timeout,
  // AP kicked us, auth failure...). Register before any begin() so the very
  // first failed attempt is captured too.
  WiFi.onEvent(onWifiStaDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  if (savedSSID.length() > 0) {
    // Priority 1: saved preferences
    LOG("[WIFI] Connecting to saved SSID \"%s\"...\n", savedSSID.c_str());
    // Hostname must be set BEFORE begin() so it's included in the DHCP DISCOVER.
    // Routers that auto-register DHCP client names into local DNS (OpenWRT,
    // pfSense, UniFi, etc.) will then expose the controller as
    // supercharger.<your-domain> — independent of mDNS / .local resolution.
    WiFi.mode(WIFI_STA);
    // Modem power save OFF. The ESP32 default (WIFI_PS_MIN_MODEM) naps the
    // radio between DTIM beacons: pings jump to 100-2000 ms, mDNS gets flaky,
    // and APs that expect prompt ACKs deauth us for inactivity — the observed
    // "connects then drops shortly after" failure. This device is powered from
    // the charger/bike, so the ~60 mA saving is irrelevant. The flag persists
    // across mode changes but is re-asserted at every bring-up path anyway.
    WiFi.setSleep(false);
    WiFi.setHostname(MDNS_HOSTNAME);
    wifiFastAttempt    = beginStaConnect(savedSSID.c_str(), savedPass.c_str());
    wifiSource         = WIFI_SRC_PREFS;
    wifiConnectStartMs = millis();
    currentState       = STATE_CONNECTING;
  } else if (strlen(SECRET_WIFI_SSID) > 0) {
    // Priority 2: compile-time station secrets (not persisted)
    trySecretsConnect();
  } else {
    // Priority 3: no credentials at all — AP setup mode
    LOG("[WIFI] No credentials available. Starting AP mode.\n");
    startAPMode();
    currentState = STATE_SETUP_MODE;
  }

  // Load HTTPS / TLS config from NVS and start the IDF httpd_ssl server.
  // httpsEnabled is the PERSISTED INTENT; g_httpsRunning is the live fact
  // (SEC-15/NET-6) and is the only thing the port-80 redirect may consult.
  httpsEnabled = preferences.getBool("https_en", false);
  if (httpsEnabled) {
    String cert = preferences.getString("tls_cert", "");
    String key  = preferences.getString("tls_key",  "");
    if (cert.length() > 0 && key.length() > 0) {
      g_httpsRunning = startHTTPSServer(cert, key);
      if (!g_httpsRunning) {
        LOG("[TLS] HTTPS start failed — serving HTTP only this boot\n");
      }
    } else {
      LOG("[TLS] HTTPS enabled but no cert/key in NVS — falling back to HTTP\n");
    }
  }

  // ── Port-80 route table (SEC-4 / NET-2) ──────────────────────────────────
  // These routes are registered UNCONDITIONALLY, because they exist only on
  // port 80 — the IDF httpd_ssl server has no story for multipart OTA upload
  // or for a long-lived SSE stream, and /save is the legacy setup form:
  //   /login GET+POST, /logout POST, /update GET + OtaUpdateHandler,
  //   /log GET, /api/log/stream GET, /save POST, /api/tls POST.
  // Before this change they were registered ONLY in the HTTPS-off branch, so
  // turning HTTPS on left /update, /log, /api/log/stream and /save reachable on
  // NEITHER port — OTA and the log viewer simply disappeared.
  // /login is registered on 80 for the same reason: the 443 cookie is minted
  // with the Secure flag and is therefore never sent over HTTP, so the
  // HTTP-only routes need a login of their own to mint a non-Secure cookie.
  // /api/tls stays the escape hatch for disabling HTTPS after a bad cert.
  //
  // Everything else (/, /settings, /api/status, /api/settings, /api/control,
  // /api/cycles) is registered on 80 only when HTTPS is NOT running; when it
  // is, those URIs fall through to WebBodyGuardHandler, which redirects to 443.
  //
  // Every body-consuming route is served by RawBodyHandler (SEC-1): capped
  // RAW streaming so an oversized body can't be malloc()ed into RAM before the
  // auth check, and multipart bodies are refused with 415 instead of taking
  // the core's null-deref upload path. Handlers read g_rawBody, never
  // server.arg("plain").
  server.on("/login",           HTTP_GET,  handleLoginGet);   // serve the login form
  server.addHandler(new RawBodyHandler("/login",  HTTP_POST, handleLoginPost));
  // POST (not GET) so prefetchers can't trigger a logout.
  server.addHandler(new RawBodyHandler("/logout", HTTP_POST, handleLogout));
  server.addHandler(new RawBodyHandler("/save",   HTTP_POST, handleSave));
  server.on("/api/log/stream",  HTTP_GET,  handleLogStream);
  server.on("/log",             HTTP_GET,  handleLogPage);
  server.on("/update",          HTTP_GET,  handleOTAGet);
  // Custom handler: multipart → handleOTAUpload, anything else drained +
  // rejected (fixes the raw-path server.upload() null-deref, audit 2026-07).
  server.addHandler(new OtaUpdateHandler());
  server.addHandler(new RawBodyHandler("/api/tls", HTTP_POST, handleApiTlsPostWS));

  if (g_httpsRunning) {
    // Belt-and-braces only: WebBodyGuardHandler (added below) matches first
    // and issues the redirect itself; onNotFound would only fire if the
    // catch-all were ever removed.
    server.onNotFound(handleHTTPSRedirect);
  } else {
    server.on("/",                HTTP_GET,  handleRoot);
    server.on("/settings",        HTTP_GET,  handleSettingsPage);
    server.on("/api/settings",    HTTP_GET,  handleApiSettingsGet);
    server.addHandler(new RawBodyHandler("/api/settings", HTTP_POST, handleApiSettingsPost));
    server.on("/api/status",      HTTP_GET,  handleApiStatus);
    server.addHandler(new RawBodyHandler("/api/control",  HTTP_POST, handleApiControl));
    server.on("/api/cycles",      HTTP_GET,    handleApiCyclesGet);
    server.addHandler(new RawBodyHandler("/api/cycles", HTTP_DELETE, handleApiCyclesDelete));
  }
  // Catch-all — MUST be registered last (handlers match in registration
  // order): 404s / redirects unmatched URIs and stream-discards their bodies.
  server.addHandler(new WebBodyGuardHandler());

  // Tell WebServer to keep these request headers — by default it discards
  // anything not on this list, so server.header()/hasHeader() return empty.
  //   - Cookie       : needed by sessionTouchOrFail() on every protected route
  //                    to look up the current scs= session token.
  //   - Content-Type : needed by RawBodyHandler::handle() to spot a multipart
  //                    POST (SEC-1). The core parses Content-Type internally
  //                    but exposes no accessor for it, so it has to be
  //                    collected explicitly.
  static const char* collectedHeaders[] = { "Cookie", "Content-Type" };
  server.collectHeaders(collectedHeaders, 2);

  // Mount the FFat partition for cycle data logging (and later model storage).
  // true = format partition if blank (first-boot only; no-op on subsequent boots).
  g_fatReady = FFat.begin(true);
  if (g_fatReady) {
    g_cycleCount = countCycleRecords();
    LOG("[FAT] Mounted. Free: %lu KB, %d cycles logged\n",
        (unsigned long)(FFat.freeBytes() / 1024UL), (int)g_cycleCount);
  } else {
    LOG("[FAT] Mount failed — cycle logging disabled\n");
  }

  server.begin();

  // Start bike CAN bus task on Core 0
  bikeBusInit();

  // Start charger CAN bus task on Core 0 at higher priority
  chargerBusInit();

  // Brief yield — ensures logBegin() (and thus logReady=true) has been
  // seen by Core 0 before the CAN tasks call LOG() for the first time.
  delay(50);

  // Start power ramp task on Core 1
  rampInit();

  // Start system stats sampling task on Core 1
  sysStatsInit();

  // Start MQTT task on Core 1
  mqttInit();

  // ── Task watchdog (STAB-5 / NET-19) ──────────────────────────────────────
  // The ESP32 core already brings the TWDT up for us (CONFIG_ESP_TASK_WDT_INIT),
  // so this is a reconfigure, not an init: 10 s timeout (10 rampTask ticks /
  // 2000 chargerBusTask passes — long enough that a slow NVS write or a flash
  // cache stall can't false-positive) and trigger_panic so a genuine hang gives
  // us a panic backtrace and a reboot rather than a silent stall with the
  // chargers still running.
  //
  // idle_core_mask = BIT(0) — keep IDLE0 watched. The stock Arduino-ESP32 TWDT
  // config watches IDLE0 at a 5 s timeout; passing 0 here (as this code used to)
  // silently DROPPED that monitoring as a side effect of raising the timeout, so
  // a Core 0 task spinning without yielding would no longer be caught at all.
  // The net effect of this block is therefore: IDLE0 stays watched, the timeout
  // goes 5 s → 10 s, and rampTask + chargerBusTask are added (they subscribe
  // themselves, at the top of each task).
  //
  // IDLE1 stays UNWATCHED deliberately: loopTask runs on Core 1 and OTA flashing
  // and streamFile() legitimately block it for tens of seconds without yielding,
  // so watching IDLE1 would panic the device during a perfectly normal firmware
  // upload.
  {
    esp_task_wdt_config_t wdtCfg = {};
    wdtCfg.timeout_ms     = 10000;
    wdtCfg.idle_core_mask = (1u << 0);   // IDLE0 only
    wdtCfg.trigger_panic  = true;
    esp_err_t werr = esp_task_wdt_reconfigure(&wdtCfg);
    if (werr == ESP_ERR_INVALID_STATE) {
      // TWDT not running (a core build with CONFIG_ESP_TASK_WDT_INIT off) —
      // bring it up ourselves with the same settings.
      werr = esp_task_wdt_init(&wdtCfg);
    }
    if (werr == ESP_OK) {
      LOG("[WDT] task watchdog 10 s, IDLE0 watched, panic on timeout\n");
    } else {
      LOG("[WDT] watchdog config failed (%d) — continuing unwatched\n", (int)werr);
    }
  }
}

void loop() {
  unsigned long now = millis();

  // Poll fast while associating so the STATE_CONNECTED transition fires as soon
  // as the link is up; back off to the steady-state interval once connected.
  unsigned long wifiInterval = (currentState == STATE_CONNECTING)
                                 ? wifiConnectingPollMs : wifiCheckInterval;
  if (now - lastWifiCheck >= wifiInterval) {
    lastWifiCheck = now;
    monitorWifiStatus();
  }

  server.handleClient();
  // HTTPS (port 443) runs in its own httpd_ssl FreeRTOS task — no polling here.
  sseFlush();
  handleCANBusTask();
  checkBootButton();
}

// ---------------------------------------------------------------------------
// Bike CAN bus — TWAI (ESP32-S3 built-in), 500 kbps, Core 0
//
// Architecture note:
//   bikeBusTask() runs on Core 0 at priority 5, leaving Core 1 free for
//   the Arduino loop() (WiFi stack + web server) at priority 1.
//   All writes to `live` are guarded by liveMutex (10 ms timeout).
//   When the charger CAN bus (MCP2515) is added in round 2 it gets its
//   own task also on Core 0 at higher priority so it always preempts this.
//
// TWAI filter:
//   We accept all 11-bit IDs here (code=0, mask=0x7FF accept-all) and
//   let the software dispatcher below ignore non-BMS frames. A tighter
//   dual-filter mode (ESP32 TWAI supports two separate acceptance windows)
//   could reduce ISR load but requires splitting BMS0 and BMS1 IDs across
//   two filter banks — the gain is marginal on a lightly-loaded bus.
// ---------------------------------------------------------------------------

#define TWAI_TX_PIN  GPIO_NUM_7
#define TWAI_RX_PIN  GPIO_NUM_6

static TaskHandle_t bikeBusTaskHandle = nullptr;

// Initialise the TWAI peripheral. Called once from setup().
// Returns true on success.
bool twaiInit() {
  twai_general_config_t gCfg = {
    .mode             = TWAI_MODE_LISTEN_ONLY,  // read-only for this round
    .tx_io            = TWAI_TX_PIN,
    .rx_io            = TWAI_RX_PIN,
    .clkout_io        = TWAI_IO_UNUSED,
    .bus_off_io       = TWAI_IO_UNUSED,
    .tx_queue_len     = 0,    // no TX in listen-only mode
    .rx_queue_len     = 16,   // enough to absorb a burst while task is busy
    .alerts_enabled   = TWAI_ALERT_BUS_ERROR | TWAI_ALERT_BUS_RECOVERED,
    .clkout_divider   = 0,
    .intr_flags       = ESP_INTR_FLAG_LEVEL1
  };

  twai_timing_config_t tCfg = TWAI_TIMING_CONFIG_500KBITS();

  // Accept all frames; software filters below select BMS messages
  twai_filter_config_t fCfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&gCfg, &tCfg, &fCfg) != ESP_OK) {
    LOG("[TWAI] Driver install failed\n");
    return false;
  }
  if (twai_start() != ESP_OK) {
    LOG("[TWAI] Start failed\n");
    return false;
  }
  LOG("[TWAI] Bike bus started (500 kbps, listen-only)\n");
  return true;
}

// Decode one received TWAI frame and write into live under mutex.
// Called only from bikeBusTask() — no other task touches live without mutex.
void processBikeFrame(const twai_message_t &msg) {
  short id  = (short)msg.identifier;
  byte  len = (byte)msg.data_length_code;
  // Cast away const for Zero library signatures (it doesn't modify the buffer)
  byte* buf = (byte*)msg.data;

  // ---------------------------------------------------------------------------
  // Raw-frame diagnostic logging — web log only, rate-limited to 1 per 5 s
  // per frame ID so the log stays readable.
  // Frames logged:
  //   0x388 BMS_CELL_VOLTAGE  — voltage (bytes 3-6), confirm bytes 0-1 are NOT sagAdjust
  //   0x288 BMS_PACK_CONFIG   — sagAdjust (bytes 0-1), AH candidate (bytes 5-6)
  //   0x488 BMS_PACK_TEMP_DATA— verify temp encoding (single byte vs int16)
  //   0x508 BMS_PACK_TIME     — verify C-rate byte position
  //   BMS1 equivalents: 0x389, 0x289, 0x489, 0x509
  // ---------------------------------------------------------------------------
  {
    static unsigned long lastLog388  = 0, lastLog288  = 0, lastLog488  = 0, lastLog508  = 0;
    static unsigned long lastLog389  = 0, lastLog289  = 0, lastLog489  = 0, lastLog509  = 0;
    static unsigned long lastLog308  = 0;
    static unsigned long lastLog188  = 0, lastLog189  = 0;
    static unsigned long lastLog2C0  = 0, lastLog3C0  = 0;
    unsigned long* slot = nullptr;
    const char*    name = nullptr;

    if      (id == 0x2C0) { slot = &lastLog2C0; name = "DASH_ODO_FROM_DASH(0x2C0)"; }
    else if (id == 0x3C0) { slot = &lastLog3C0; name = "DASH_ODO_TO_DASH  (0x3C0)"; }
    else if (id == 0x388) { slot = &lastLog388; name = "BMS_CELL_VOLTAGE  (0x388)"; }
    else if (id == 0x288) { slot = &lastLog288; name = "BMS_PACK_CONFIG   (0x288)"; }
    else if (id == 0x488) { slot = &lastLog488; name = "BMS_PACK_TEMP_DATA(0x488)"; }
    else if (id == 0x508) { slot = &lastLog508; name = "BMS_PACK_TIME     (0x508)"; }
    else if (id == 0x188) { slot = &lastLog188; name = "BMS_PACK_STATUS   (0x188)"; }
    else if (id == 0x389) { slot = &lastLog389; name = "BMS1_CELL_VOLTAGE (0x389)"; }
    else if (id == 0x289) { slot = &lastLog289; name = "BMS1_PACK_CONFIG  (0x289)"; }
    else if (id == 0x489) { slot = &lastLog489; name = "BMS1_PACK_TEMP    (0x489)"; }
    else if (id == 0x509) { slot = &lastLog509; name = "BMS1_PACK_TIME    (0x509)"; }
    else if (id == 0x189) { slot = &lastLog189; name = "BMS1_PACK_STATUS  (0x189)"; }
    else if (id == BMS_PACK_STATS.id)    { slot = &lastLog308; name = "BMS_PACK_STATS"; }

    unsigned long nowMs = millis();
    if (slot && (nowMs - *slot >= 5000)) {
      *slot = nowMs;
      LOG("[CAN] %s len=%d  %02X %02X %02X %02X %02X %02X %02X %02X\n",
          name, (int)len,
          buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);

      // ── Extra interpretation for frames under active byte-mapping ────────────
      // Cross-reference these against ZeroSpy to identify cell voltage / temp
      // fields. All candidate readings printed so we can pick the right one.

      if (id == 0x388 || id == 0x389) {
        // Candidates for cell voltage min/max (ZeroSpy shows ~4065-4068 mV idle):
        //   b[0-1] as uint16 LE   — likely cell min mV
        //   b[2]   as uint8       — possible delta (max-min), fits in 1 byte
        //   b[2-3] as uint16 LE   — possible cell max mV (conflicts with pack b[3])
        //   pack confirmed b[3-6] as uint32 LE (mV) and alt b[4-7]
        uint16_t b01 = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
        uint16_t b23 = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
        uint32_t b36 = (uint32_t)buf[3] | ((uint32_t)buf[4] << 8)
                     | ((uint32_t)buf[5] << 16) | ((uint32_t)buf[6] << 24);
        uint32_t b47 = (uint32_t)buf[4] | ((uint32_t)buf[5] << 8)
                     | ((uint32_t)buf[6] << 16) | ((uint32_t)buf[7] << 24);
        LOG("[CAN]  +-388 b[0-1]=%u mV  b[2]=%u  b[2-3]=%u mV  "
            "pack(b3-6)=%u mV  pack(b4-7)=%u mV\n",
            b01, (unsigned)buf[2], b23, b36, b47);
      }

      if (id == 0x2C0 || id == 0x3C0) {
        // Odometer candidates. Community Zero reverse-engineering says the value
        // is a little-endian integer in HECTOMETRES (0.1 km) → km = raw/10.
        // Print a few byte-range interpretations so the layout can be confirmed
        // against the bike's displayed odometer (~26600 km expected here).
        uint32_t b02 = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8)
                     | ((uint32_t)buf[2] << 16);                       // bytes 0-2 LE
        uint32_t b03 = b02 | ((uint32_t)buf[3] << 24);                 // bytes 0-3 LE
        uint32_t b47 = (uint32_t)buf[4] | ((uint32_t)buf[5] << 8)
                     | ((uint32_t)buf[6] << 16) | ((uint32_t)buf[7] << 24); // bytes 4-7 LE
        LOG("[CAN]  +-%03X odo: b[0-2]=%lu (%.1f km)  b[0-3]=%lu (%.1f km)  "
            "b[4-7]=%lu (%.1f km)\n",
            (unsigned)(id & 0xFFF),
            (unsigned long)b02, b02 / 10.0f,
            (unsigned long)b03, b03 / 10.0f,
            (unsigned long)b47, b47 / 10.0f);
      }

      if (id == 0x488 || id == 0x489) {
        // 0x488 BMS_PACK_TEMP_DATA — layout unknown; print all interpretations.
        // Signed bytes: possible temperature fields (°C, signed).
        // uint16 LE pairs: possible mV / dV fields.
        LOG("[CAN]  +-488 signed:  %4d %4d %4d %4d %4d %4d %4d %4d\n",
            (int)(int8_t)buf[0], (int)(int8_t)buf[1],
            (int)(int8_t)buf[2], (int)(int8_t)buf[3],
            (int)(int8_t)buf[4], (int)(int8_t)buf[5],
            (int)(int8_t)buf[6], (int)(int8_t)buf[7]);
        uint16_t w01 = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
        uint16_t w23 = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
        uint16_t w45 = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
        uint16_t w67 = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
        LOG("[CAN]  +-488 u16LE:   %5u  %5u  %5u  %5u\n", w01, w23, w45, w67);
      }
    }
  }

  unsigned long now = millis();

  if (xSemaphoreTake(liveMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    return; // drop frame rather than block the task
  }

  // --- Monolith (BMS0) ---
  if (zeroDecoder.hasMonolithVoltage(id)) {
    // BMS_CELL_VOLTAGE (0x388) frame layout (confirmed from CAN capture):
    //   byte 0      : cell index 0..27 (BMS cycles through all cells at ~10 Hz)
    //   bytes 1-2   : cell voltage, uint16 LE, mV  (e.g. E2 0F → 4066 mV)
    //   bytes 3-6   : pack voltage, uint32 LE, mV  → dV stored in monolithVoltageDv
    //   byte 7      : 0x00 padding
    // Plausibility-gate the pack voltage before storing it (H1). The cell
    // decode 10 lines below has always had a sanity window; the *pack* voltage
    // did not, and it is the divisor in the I = P/V calculation that produces
    // the charger current command. A corrupt frame decoding to e.g. 1 dV made
    // rampTask compute a five-figure amp command (and, past 6553.5 A, wrap the
    // uint16_t). PACK_V_MIN_DV/PACK_V_MAX_DV come from the Farasis cell
    // datasheet × 28S — see their definition near MAX_CHARGE_VOLTAGE_DV.
    //
    // A rejected frame deliberately does NOT refresh bms0LastMs, so persistent
    // garbage trips the existing BMS_STALE_TIMEOUT_MS failsafe in rampTask and
    // the chargers are commanded to STOP. Fail-safe by omission.
    long vDv = zeroDecoder.voltage(len, buf) / 100;
    if (vDv < (long)PACK_V_MIN_DV || vDv > (long)PACK_V_MAX_DV) {
      g_lastBadFrameMs = now;   // raises the dashboard banner for a short window
      static unsigned long lastBadMonoVLog = 0;
      if (lastBadMonoVLog == 0 || now - lastBadMonoVLog >= 5000) {
        lastBadMonoVLog = now;
        LOG("[CAN] Implausible monolith pack voltage %ld dV (valid %u-%u) — "
            "frame rejected\n", vDv,
            (unsigned)PACK_V_MIN_DV, (unsigned)PACK_V_MAX_DV);
      }
    } else {
      live.monolithVoltageDv = vDv;
      live.bms0LastMs = now;   // liveness timestamp for rampTask staleness guard
      if (!live.dataFresh) {
        live.dataFresh   = true;
        live.bms0FirstMs = now;
      }
    }
    // Decode rotating cell voltage and update per-cell array + balance
    if (len >= 3 && buf[0] < 28) {
      uint8_t  cellIdx = buf[0];
      uint16_t cellMv  = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
      if (cellMv > 1000 && cellMv < 6000) {  // sanity: 1–6 V per cell
        live.cellVoltsMv[cellIdx]  = cellMv;
        live.cellSeenMask         |= (1UL << cellIdx);
        // Recompute balance (max − min) from all seen cells
        uint16_t cMin = 0xFFFF, cMax = 0;
        for (int ci = 0; ci < 28; ci++) {
          if (live.cellSeenMask & (1UL << ci)) {
            if (live.cellVoltsMv[ci] < cMin) cMin = live.cellVoltsMv[ci];
            if (live.cellVoltsMv[ci] > cMax) cMax = live.cellVoltsMv[ci];
          }
        }
        if (cMin != 0xFFFF) live.cellBalanceMv = cMax - cMin;
      }
    }
  }
  else if (zeroDecoder.hasMonolithAmps(id)) {
    // BMS_PACK_ACTIVE_DATA (0x408): one frame carries multiple fields:
    //   byte 1: highest cell temp (signed °C)
    //   byte 2: lowest cell temp  (signed °C)
    //   byte 3-4: pack amps (signed int16 centiamps)
    // This is the layout used by every working SCv2 sketch (v2.5 / v2.6 / v3 /
    // 13kw_force). The 0x488 "BMS_PACK_TEMP_DATA" frame appears to carry other
    // BMS internals (cell-balance flags, individual sensor positions with 0x7F
    // sentinels) and is NOT a reliable source for high/low cell temps.
    live.monolithAmps    = zeroDecoder.amps(len, buf);
    live.monolithMaxTemp = zeroDecoder.highestTemp(len, buf);
    live.monolithMinTemp = zeroDecoder.lowestTemp(len, buf);
  }
  else if (zeroDecoder.hasMonolithPackConfig(id)) {
    // BMS_PACK_CONFIG (0x288): sagAdjust in bytes 0-1, AH candidate in bytes 5-6
    live.monolithSagAdjDv = zeroDecoder.sagAdjust(len, buf);
    // Plausibility-gate the pack capacity (STAB-3), for the same reason the
    // pack voltage above is gated: packAH is a multiplier, not a display value.
    // It scales the absolute 1 C current ceiling, every cutback table's power
    // limit, the CV current figure and the ETA — so a corrupt frame decoding to
    // e.g. 4000 Ah quietly removes the current ceiling rather than producing an
    // obviously wrong number. A rejected frame keeps the previous value and
    // raises the same bad-frame banner the voltage gate uses.
    {
      short ah = zeroDecoder.AH(len, buf);
      if (ah < PACK_AH_PLAUSIBLE_MIN || ah > PACK_AH_PLAUSIBLE_MAX) {
        g_lastBadFrameMs = now;
        static unsigned long lastBadMonoAhLog = 0;
        if (lastBadMonoAhLog == 0 || now - lastBadMonoAhLog >= 5000) {
          lastBadMonoAhLog = now;
          LOG("[CAN] BMS Ah %d implausible — ignored\n", (int)ah);
        }
      } else {
        live.monolithAH = ah;
      }
    }
  }
  else if (zeroDecoder.hasMonolithMaxCRate(id)) {
    // BMS_PACK_TIME (0x508) — C-rate assumed bytes 4-5; verify via raw log
    live.monolithMaxCRate = zeroDecoder.maxCRate(len, buf);
  }
  else if (zeroDecoder.hasMonolithPackStatus(id)) {
    // BMS_PACK_STATUS (0x188): byte 0 is the BMS-reported State of Charge in
    // percent (0..100).  Mirrors what the BMS hands the MBB before the MBB
    // applies its own dashboard filter.  Reverse-engineered from EMF thread
    // 8799 (https://www.electricmotorcycleforum.com/boards/index.php?topic=8799.0).
    byte soc = zeroDecoder.stateOfCharge(len, buf);
    if (soc != 255) live.monolithBmsSoc = soc;  // 255 = bad/short frame, keep last good
  }
  else if (id == 0x488) {
    // BMS_PACK_TEMP_DATA (0x488) — partial decode (confirmed via captures):
    //   byte 0     : rotating counter 0..7
    //   byte 1     : BMS board temperature, signed int8, °C
    //                (ZeroSpy: "Controller Temp")
    //   bytes 2-3  : usually 0x7FFE sentinel; otherwise descending counter.
    //                Hypothesis: time-to-full estimate. Unconfirmed.
    //   byte 4     : small rotating index 0..0x20 — cell index candidate but
    //                range exceeds 27. Unconfirmed.
    //   byte 5     : always 0x00 (padding/reserved)
    //   bytes 6-7  : uint16 LE, mV → per-cell average voltage. Tracks
    //                pack_voltage / 28 to within ±2 mV across an entire
    //                100 % charge capture. Confirmed.
    if (len >= 2) live.bmsBoardTempC = (int8_t)buf[1];
    if (len >= 8) {
      uint16_t avgMv = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
      // Sanity-gate the same range as the 0x388 cell decode
      if (avgMv > 1000 && avgMv < 6000) live.cellAvgMv = avgMv;
    }
  }

  // --- PowerTank (BMS1) ---
  else if (zeroDecoder.hasPowerTankVoltage(id)) {
    // BMS1_CELL_VOLTAGE (0x389): same layout as BMS0 — same plausibility gate
    // (H1). The PowerTank sits in parallel with the monolith on stock Zeros, so
    // it shares the 28S window. A rejected frame also leaves powerTankPresent
    // alone: a pack that only ever sends garbage is never declared present, so
    // rampTask won't fold its temperatures into the cutback decisions.
    long ptVDv = zeroDecoder.voltage(len, buf) / 100;
    if (ptVDv < (long)PACK_V_MIN_DV || ptVDv > (long)PACK_V_MAX_DV) {
      g_lastBadFrameMs = now;   // raises the dashboard banner for a short window
      static unsigned long lastBadPtVLog = 0;
      if (lastBadPtVLog == 0 || now - lastBadPtVLog >= 5000) {
        lastBadPtVLog = now;
        LOG("[CAN] Implausible PowerTank pack voltage %ld dV (valid %u-%u) — "
            "frame rejected\n", ptVDv,
            (unsigned)PACK_V_MIN_DV, (unsigned)PACK_V_MAX_DV);
      }
    } else {
      live.powerTankVoltageDv = ptVDv;
      if (!live.powerTankPresent) {
        live.powerTankPresent = true;
        live.powerTankDecided = true;
        live.bms1FirstMs      = now;
      }
    }
  }
  else if (zeroDecoder.hasPowerTankAmps(id)) {
    // BMS1_PACK_ACTIVE_DATA (0x409): same layout as BMS0 0x408 — temps in
    // bytes 1/2, amps in bytes 3-4.
    live.powerTankAmps    = zeroDecoder.amps(len, buf);
    live.powerTankMaxTemp = zeroDecoder.highestTemp(len, buf);
    live.powerTankMinTemp = zeroDecoder.lowestTemp(len, buf);
  }
  else if (zeroDecoder.hasPowerTankPackConfig(id)) {
    // BMS1_PACK_CONFIG (0x289): sagAdjust bytes 0-1, AH bytes 5-6
    live.powerTankSagAdjDv = zeroDecoder.sagAdjust(len, buf);
    // Same plausibility gate as the monolith Ah above (STAB-3).
    {
      short ah = zeroDecoder.AH(len, buf);
      if (ah < PACK_AH_PLAUSIBLE_MIN || ah > PACK_AH_PLAUSIBLE_MAX) {
        g_lastBadFrameMs = now;
        static unsigned long lastBadPtAhLog = 0;
        if (lastBadPtAhLog == 0 || now - lastBadPtAhLog >= 5000) {
          lastBadPtAhLog = now;
          LOG("[CAN] BMS Ah %d implausible — ignored\n", (int)ah);
        }
      } else {
        live.powerTankAH = ah;
      }
    }
  }
  else if (zeroDecoder.hasPowerTankMaxCRate(id)) {
    // BMS1_PACK_TIME (0x509) — C-rate assumed bytes 4-5; verify via raw log
    live.powerTankMaxCRate = zeroDecoder.maxCRate(len, buf);
  }
  else if (zeroDecoder.hasPowerTankPackStatus(id)) {
    // BMS1_PACK_STATUS (0x189): same byte-0 SoC layout as the monolith 0x188.
    byte soc = zeroDecoder.stateOfCharge(len, buf);
    if (soc != 255) live.powerTankBmsSoc = soc;
  }
  else if (id == 0x2C0) {
    // DASH_ODO_FROM_DASH (0x2C0) — total odometer. Per the community Zero CAN
    // reverse-engineering the value is a 3-byte little-endian integer in
    // HECTOMETRES (0.1 km): e.g. bytes 07 5D 02 → 0x025D07 = 154887 →
    // 15488.7 km. We read bytes 0-2 LE (a 3-byte odometer covers up to
    // ~1.67M km) and store hectometres. Sanity-gated so a corrupt frame can't
    // publish a wild value.
    //
    // NOTE: verify against the bike's displayed odometer using the
    // "[CAN] +-2C0 odo:" diagnostic log line (it prints the b[0-2], b[0-3] and
    // b[4-7] interpretations). If none matches, the source ID may be 0x3C0 on
    // this bike, or the scale differs — adjust the byte range / divisor here.
    if (len >= 3) {
      uint32_t hm = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8)
                  | ((uint32_t)buf[2] << 16);
      if (hm > 0 && hm < 16000000UL)   // 0 < km < 1,600,000 (3-byte range)
        live.odometerHm = hm;
    }
  }
  // 0x489 (BMS1_PACK_TEMP_DATA) intentionally NOT decoded for temps — see 0x408 note.

  // --- PowerTank detection window close ---
  // Once BMS0 data is fresh and the window has elapsed without any BMS1
  // frame, mark PowerTank as absent so the dashboard stops waiting.
  if (live.dataFresh && !live.powerTankDecided) {
    if ((now - live.bms0FirstMs) >= POWERTANK_DETECT_WINDOW_MS) {
      live.powerTankDecided = true;
      live.powerTankPresent = false;
      LOG("[TWAI] PowerTank not detected — section hidden\n");
    }
  }

  xSemaphoreGive(liveMutex);
}

// FreeRTOS task — pinned to Core 0, priority 5.
// Blocks on twai_receive() for up to 10 ms so it yields when the bus is
// quiet. This leaves Core 0 headroom for the charger bus task (round 2)
// which will run at priority 6 and preempt this task when needed.
void bikeBusTask(void* /*pvParameters*/) {
  LOG("[TWAI] bikeBusTask running on core %d\n", xPortGetCoreID());

  twai_message_t msg;
  uint32_t       alerts;

  for (;;) {
    // Check for bus-level alerts (off-bus, recovered, etc.)
    if (twai_read_alerts(&alerts, 0) == ESP_OK) {
      if (alerts & TWAI_ALERT_BUS_ERROR) {
        LOG("[TWAI] Bus error alert\n");
      }
      if (alerts & TWAI_ALERT_BUS_RECOVERED) {
        LOG("[TWAI] Bus recovered\n");
        twai_start();
      }
    }

    // Block for up to 10 ms waiting for a frame
    esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(10));
    if (err == ESP_OK) {
      // Ignore remote frames and frames with non-standard (extended) IDs
      if (!msg.rtr && !msg.extd) {
        processBikeFrame(msg);
      }
    } else if (err != ESP_ERR_TIMEOUT) {
      // ERR_TIMEOUT is normal when bus is quiet — log anything else
      LOG("[TWAI] receive error: 0x%x\n", err);
    }
  }
}

// Called from setup() — creates mutex, inits TWAI, launches task on Core 0
void bikeBusInit() {
  liveMutex = xSemaphoreCreateMutex();
  if (liveMutex == nullptr) {
    LOG("[TWAI] Failed to create mutex — halting\n");
    while (true) { delay(1000); }
  }

  if (!twaiInit()) {
    LOG("[TWAI] Init failed — bike bus disabled\n");
    return;
  }

  xTaskCreatePinnedToCore(
    bikeBusTask,        // task function
    "bikeBusTask",      // name (for debugging)
    4096,               // stack in bytes — enough for Zero decoder + local vars
    nullptr,            // parameter
    5,                  // priority (loop() runs at 1)
    &bikeBusTaskHandle, // handle
    0                   // Core 0
  );
}

// Empty stub — loop() calls this but all real CAN work is in FreeRTOS tasks.
void handleCANBusTask() {}

// ---------------------------------------------------------------------------
// Charger CAN bus — MCP2515 via SPI, extended 29-bit IDs, Core 0
//
// Priority 6 > bikeBusTask priority 5: the charger heartbeat always preempts
// the bike bus decode loop. This ensures the 1 s heartbeat deadline is met
// even if the bike bus is receiving a burst of frames.
// Bus speed: 250 kbps, 8 MHz crystal on MCP2515.
//
// Heartbeat frame (BMS -> Charger):
//   ID   : 0x1806E5F4  (extended, 29-bit)
//   Byte 0-1 : target voltage, 0.1 V/LSB, big-endian
//   Byte 2-3 : target current, 0.1 A/LSB, big-endian
//   Byte 4   : 0x00 = START, 0x01 = STOP
//   Byte 5-7 : 0x00 reserved
//
// Status frame (Charger -> BMS):
//   ID   : 0x18FF50Ex  (x = charger instance nibble: 5, 7, 8, 9 seen in wild)
//   Byte 0-1 : actual output voltage, 0.1 V/LSB, big-endian
//   Byte 2-3 : actual output current, 0.1 A/LSB, big-endian
//   Byte 4   : status bitfield (see ChargerBusData comment)
//   Byte 5-7 : reserved
// ---------------------------------------------------------------------------

// MCP2515 SPI bus — named instance so the library uses our pin mapping
// instead of the ESP32 default SPI pins.
static SPIClass mcpSPI(HSPI);
static MCP_CAN  mcpCan(&mcpSPI, MCP_CS_PIN);
static TaskHandle_t chargerBusTaskHandle = nullptr;

// Set to 1 to enable listen-only mode for bus diagnostics.
// In this mode no heartbeat is sent, but all received frames are logged.
// Set back to 0 once baud rate and wiring are confirmed.
#define MCP_LISTEN_ONLY 0

// Initialise the MCP2515. Returns true on success.
bool mcpInit() {
  pinMode(MCP_RST_PIN, OUTPUT);
  digitalWrite(MCP_RST_PIN, LOW);
  delay(10);
  digitalWrite(MCP_RST_PIN, HIGH);
  delay(10);

  mcpSPI.begin(MCP_SCLK_PIN, MCP_MISO_PIN, MCP_MOSI_PIN);

  for (int attempt = 1; attempt <= 3; attempt++) {
    if (mcpCan.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) == CAN_OK) {
#if MCP_LISTEN_ONLY
      mcpCan.setMode(MCP_LISTENONLY);
      LOG("[MCP] Charger bus started (250 kbps, LISTEN-ONLY diagnostic mode)\n");
      LOG("[MCP] All received frames will be logged. Set MCP_LISTEN_ONLY 0 for normal operation.\n");
#else
      mcpCan.setMode(MCP_NORMAL);
      LOG("[MCP] Charger bus started (250 kbps)\n");
#endif
      return true;
    }
    LOG("[MCP] Init attempt %d failed, retrying...\n", attempt);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  LOG("[MCP] Init failed — charger bus disabled\n");
  return false;
}

// Build and send one heartbeat frame to all chargers.
void sendHeartbeat() {
  uint16_t vCmd = 0;
  uint16_t aCmd = 0;
  bool     start = false;

  if (xSemaphoreTake(chargerMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    vCmd  = chargerBus.cmdVoltDv;
    aCmd  = chargerBus.cmdAmpsDa;
    start = chargerBus.cmdStart;
    xSemaphoreGive(chargerMutex);
  }

  // Failsafe STOP override (STAB-2). Applies to BOTH branches above — the
  // successful read and the mutex timeout (which leaves start=false anyway).
  // rampTask raises g_forceStop before it attempts the mutex write that zeroes
  // the command, so if that write is lost to contention the stale START in
  // chargerBus never reaches the bus: this tick transmits a STOP frame with
  // zero current regardless. No lock needed — a bool is atomic here, and the
  // whole point is to work when the lock is unavailable. vCmd is left as read
  // (or 0 on timeout); byte 4 of the frame is what the charger obeys.
  if (g_forceStop) { start = false; aCmd = 0; }

  // Dead-man check (finding #6): rampTask owns the command values and is the
  // only thing that supervises voltage/temperature/phase. If it stops
  // advancing g_rampHeartbeat (mutex wedge, stack-overflow park, crash) the
  // last command would otherwise be re-sent at 1 Hz forever, charging the pack
  // unsupervised. If the counter hasn't moved for RAMP_DEADMAN_MS, force STOP.
  static uint32_t      lastRampHb   = 0;
  static unsigned long lastRampHbMs = 0;
  static bool          rampHbSeen   = false;
  const unsigned long  RAMP_DEADMAN_MS = 5000UL;  // 5 missed 1 Hz ramp ticks
  uint32_t hbNow  = g_rampHeartbeat;
  unsigned long tn = millis();
  if (!rampHbSeen || hbNow != lastRampHb) {
    lastRampHb = hbNow; lastRampHbMs = tn; rampHbSeen = true;
  }
  if (start && (tn - lastRampHbMs) > RAMP_DEADMAN_MS) {
    vCmd = 0; aCmd = 0; start = false;
    static unsigned long lastDeadmanLog = 0;
    if (lastDeadmanLog == 0 || tn - lastDeadmanLog >= 5000UL) {
      lastDeadmanLog = tn;
      LOG("[MCP] rampTask not advancing (%lus) — forcing charger STOP (dead-man)\n",
          (tn - lastRampHbMs) / 1000UL);
    }
  }

  byte data[8] = {
    (byte)(vCmd >> 8),
    (byte)(vCmd & 0xFF),
    (byte)(aCmd >> 8),
    (byte)(aCmd & 0xFF),
    (byte)(start ? 0x00 : 0x01),
    0x00, 0x00, 0x00
  };

  byte rc = mcpCan.sendMsgBuf(CHARGER_CMD_ID, 1 /*extended*/, 8, data);

  // Rate-limited error logging — print at most once every 5 s so the
  // serial monitor stays readable when no charger is connected.
  static unsigned long lastErrLogMs = 0;
  // M2: consecutive TX failures, used as the bus-off proxy below.
  static uint16_t consecTxErr = 0;
  if (rc != CAN_OK) {
    unsigned long now = millis();
    // `lastErrLogMs == 0` is the file's rate-limiter idiom AND what this
    // limiter's own reset-to-0-on-success (below) has always intended: it is
    // reset so that the next error after a working period logs immediately,
    // which `now - 0 >= 5000` does not deliver while millis() is still small.
    if (lastErrLogMs == 0 || now - lastErrLogMs >= 5000) {
      // Error 6 = TX buffer timeout (no ACK from bus — expected with nothing connected)
      // Error 7 = send msg timeout
      LOG("[MCP] Heartbeat TX error %d (no charger connected?)\n", rc);
      lastErrLogMs = now;
    }

    // ── Controller reset on sustained TX failure (M2) ───────────────────────
    // This used to be `if (mcpCan.checkError() == CAN_CTRLERROR)`, which is
    // NOT a bus-off test. checkError() returns:
    //     (eflg & MCP_EFLG_ERRORMASK) ? CAN_CTRLERROR : CAN_OK
    // and MCP_EFLG_ERRORMASK is 0xF8 — RX1OVR | RX0OVR | TXBO | TXEP | RXEP.
    // Bus-off (TXBO) is only bit 5. So a transient RX overflow or a brief
    // error-passive excursion re-initialised the controller mid-charge,
    // dropping the bus long enough for the chargers to hit
    // CHARGER_TIMEOUT_MS. Upstream coryjfowler has the identical
    // implementation, so this cannot be fixed by updating the library, and
    // mcp2515_readRegister()/MCP_EFLG are private — EFLG cannot be read
    // through the public API without patching the library (which would become
    // an undocumented build dependency).
    //
    // Instead: count consecutive heartbeat TX failures. TXBO means >255
    // consecutive TX errors, so sustained failure of an unconditional 1 Hz
    // transmit is the observable equivalent, and unlike the EFLG test it
    // cannot be tripped by a receive-side hiccup. A single successful send
    // clears the counter.
    if (consecTxErr < 65535) consecTxErr++;
#if MCP_LISTEN_ONLY
    // Bus-off shouldn't occur in listen-only mode — log but don't spam
    if (consecTxErr >= MCP_TX_FAIL_RESET_THRESHOLD) {
      static unsigned long lastBusOffLog = 0;
      if (lastBusOffLog == 0 || millis() - lastBusOffLog >= 10000) {
        lastBusOffLog = millis();
        LOG("[MCP] Sustained TX failure in listen-only mode — unexpected, "
            "check wiring\n");
      }
      consecTxErr = 0;
    }
#else
    if (consecTxErr >= MCP_TX_FAIL_RESET_THRESHOLD) {
      LOG("[MCP] %u consecutive heartbeat TX failures — resetting MCP2515\n",
          (unsigned)consecTxErr);
      mcpCan.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ);
      mcpCan.setMode(MCP_NORMAL);
      consecTxErr = 0;
    }
#endif
  } else {
    lastErrLogMs = 0; // reset so next error after a working period logs immediately
    consecTxErr  = 0; // a single good send means the controller is not bus-off
  }

  if (xSemaphoreTake(chargerMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    chargerBus.heartbeatOk = (rc == CAN_OK);
    xSemaphoreGive(chargerMutex);
  }
}

// Decode one received MCP2515 status frame and update chargerBus.
void processChargerFrame(uint32_t id, byte len, byte* buf) {
  // Only accept status frames: top 28 bits must match CHARGER_STATUS_ID_BASE
  if ((id & CHARGER_STATUS_ID_MASK) != CHARGER_STATUS_ID_BASE) return;

  uint8_t nibble = (uint8_t)(id & 0x0F); // charger instance index
  if (nibble >= MAX_CHARGERS) return;

  // Guard against short / malformed frames — the buf[] passed in is an
  // 8-byte stack array that's only filled up to `len`; reading past `len`
  // would yield uninitialised garbage that we'd then act on as charger volts
  // / amps. A real Elcon TC HK status frame is always 8 bytes; anything
  // shorter is a protocol violation and we drop it.
  if (len < 4) return;

  uint16_t voltDv = ((uint16_t)buf[0] << 8) | buf[1]; // 0.1 V units
  uint16_t ampsDa = ((uint16_t)buf[2] << 8) | buf[3]; // 0.1 A units
  uint8_t  status = (len >= 5) ? buf[4] : 0;

  if (xSemaphoreTake(chargerMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    ChargerUnit& c = chargerBus.chargers[nibble];
    bool wasPresent = c.present;
    // Phantom-charger debounce (audit 2026-09). A single frame whose ID arrived
    // corrupted — or aliased off another node — used to be enough to declare a
    // whole extra charger present. That inflates chargerCount, which inflates
    // rampTask's divisor (so every real unit under-delivers) and trips the
    // count-mismatch stop against a charger that does not exist. Require
    // CHARGER_SEEN_MIN_FRAMES status frames within one CHARGER_TIMEOUT_MS
    // window instead (not strictly consecutive — the count only resets on
    // staleness decay); a real unit broadcasts at ~1 Hz so it qualifies in
    // about a second.
    if (c.seenCount < CHARGER_SEEN_MIN_FRAMES) c.seenCount++;
    c.present    = (c.seenCount >= CHARGER_SEEN_MIN_FRAMES);
    c.voltDv     = voltDv;
    c.ampsDa     = ampsDa;
    c.status     = status;
    c.lastSeenMs = millis();

    // Recount active chargers whenever a new one qualifies. Gated on the
    // present false→true edge, so the first (not-yet-qualified) frame of a new
    // unit does not recount and the qualifying frame does exactly once.
    if (!wasPresent && c.present) {
      uint8_t count = 0;
      for (int i = 0; i < MAX_CHARGERS; i++) {
        if (chargerBus.chargers[i].present) count++;
      }
      // Clamp to what the current divisor / preset table support (L6). More
      // units on the bus than this means the per-charger division would
      // under-divide and every charger would over-deliver, so refuse to
      // report a count we can't safely command.
      if (count > MAX_ACTIVE_CHARGERS) {
        LOG("[MCP] WARNING: %d chargers detected but only %d supported — "
            "count clamped. Per-charger current would be under-divided.\n",
            count, (int)MAX_ACTIVE_CHARGERS);
        count = MAX_ACTIVE_CHARGERS;
        g_chargerCountClamped = true;   // raises the dashboard banner
      }
      chargerBus.chargerCount = count;
      LOG("[MCP] Charger %d seen — total: %d\n", nibble, count);
    }
    xSemaphoreGive(chargerMutex);
  }
}

// FreeRTOS task — Core 0, priority 6.
// Sends heartbeat every HEARTBEAT_INTERVAL_MS, drains incoming frames
// between beats. The tight 5 ms poll keeps receive latency low without
// starving other tasks.
void chargerBusTask(void* /*pvParameters*/) {
  LOG("[MCP] chargerBusTask running on core %d\n", xPortGetCoreID());

  // *** CRITICAL: Initialise MCP2515 HERE, on Core 0, so the SPI HAL
  //     handle is allocated and used on the same core.  Previously this
  //     was done from setup() on Core 1, and the internal spi_t* handle
  //     became invalid when accessed from Core 0 — causing a
  //     LoadProhibited crash in SPIClass::beginTransaction().
  if (!mcpInit()) {
    LOG("[MCP] Charger bus disabled (init failed on core %d)\n", xPortGetCoreID());
    vTaskDelete(nullptr);      // kill this task cleanly
    return;                    // never reached, but keeps compiler happy
  }

  // Subscribe to the task watchdog (STAB-5). Deliberately AFTER the mcpInit()
  // bail-out above: that path calls vTaskDelete() on ourselves, and a subscribed
  // task that deletes itself leaves a dangling handle in the TWDT's list.
  if (esp_task_wdt_add(nullptr) != ESP_OK) {
    LOG("[WDT] chargerBusTask could not subscribe to the task watchdog\n");
  }

  unsigned long lastHeartbeatMs = 0;
  unsigned long lastDecayMs     = 0;
  // Rate-limit raw frame logging in listen-only mode
  static unsigned long lastRawLogMs = 0;
  static uint32_t      frameCount   = 0;

  for (;;) {
    unsigned long now = millis();

    // --- Decay stale chargers (finding #16) ---
    // present[]/chargerCount only ever grew; a charger that stopped sending
    // status frames stayed "present" forever, so cmdStart gating (and the
    // dashboard count) could act on a charger that has actually gone away.
    // Once per second, clear present for any unit not seen within the timeout
    // window and recompute chargerCount from what's genuinely live.
    if (now - lastDecayMs >= 1000) {
      lastDecayMs = now;
      if (xSemaphoreTake(chargerMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        uint8_t count = 0;
        for (int i = 0; i < MAX_CHARGERS; i++) {
          ChargerUnit& c = chargerBus.chargers[i];
          // Decay on staleness whatever `present` says. A unit that logged one
          // qualifying frame and then vanished has to re-earn its place from
          // zero — otherwise one stray frame now plus another an hour later
          // would still add up to CHARGER_SEEN_MIN_FRAMES and invent a phantom
          // charger. seenCount > 0 keeps never-seen slots (lastSeenMs == 0) out.
          if (c.seenCount > 0 && (now - c.lastSeenMs) > CHARGER_TIMEOUT_MS) {
            if (c.present) LOG("[MCP] Charger %d timed out — marking absent\n", i);
            c.present   = false;
            c.seenCount = 0;
          }
          if (c.present) count++;
        }
        // L6 clamp. Recomputed here every second, so the banner clears itself
        // once the extra unit stops answering (unplugged / powered down) rather
        // than latching until reboot.
        g_chargerCountClamped = (count > MAX_ACTIVE_CHARGERS);
        if (count > MAX_ACTIVE_CHARGERS) count = MAX_ACTIVE_CHARGERS;
        if (chargerBus.chargerCount != count) chargerBus.chargerCount = count;
        xSemaphoreGive(chargerMutex);
      }
    }

#if !MCP_LISTEN_ONLY
    // --- Heartbeat (normal mode only) ---
    if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
      lastHeartbeatMs = now;
      sendHeartbeat();
    }
#endif

    // --- Drain incoming frames ---
    // Bounded per pass (audit 2026-07): a babbling/flooded bus used to keep
    // checkReceive() permanently true, spinning this priority-6 loop forever —
    // heartbeats stopped and bikeBusTask (prio 5, same core) starved. 32
    // frames per 5 ms pass (~6400 f/s) is still ~3× the theoretical maximum
    // frame rate of a 250 kbps bus, so no legitimate traffic is ever deferred.
    int drained = 0;
    while (drained < 32 && mcpCan.checkReceive() == CAN_MSGAVAIL) {
      drained++;
      byte     len = 0;
      byte     buf[8];
      uint32_t id  = 0;
      if (mcpCan.readMsgBuf(&id, &len, buf) == CAN_OK) {
        bool extended = (id & 0x80000000UL);
        if (extended) id &= 0x1FFFFFFFUL;

#if MCP_LISTEN_ONLY
        // Log every unique frame type seen; throttle repeated frames
        frameCount++;
        if (now - lastRawLogMs >= 2000) {
          lastRawLogMs = now;
          LOG("[MCP] RX frames in last 2s: %lu | Last: ID=0x%08lX ext=%d len=%d"
                        " [%02X %02X %02X %02X %02X %02X %02X %02X]\n",
                        frameCount, id, extended ? 1 : 0, len,
                        buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
          frameCount = 0;
        }
#endif
        if (extended) processChargerFrame(id, len, buf);
      }
    }

#if MCP_LISTEN_ONLY
    // Log silence so we know the task is alive even with no frames
    if (now - lastRawLogMs >= 5000) {
      lastRawLogMs = now;
      LOG("[MCP] Listen-only: no frames received in 5 s\n");
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(5));
    esp_task_wdt_reset();   // one feed per 5 ms pass (STAB-5)
  }
}

// Called from setup() — creates mutex and launches task on Core 0.
// NOTE: mcpInit() is called INSIDE the task so that the SPI HAL handle
//       is allocated on the same core (0) that performs SPI transactions.
void chargerBusInit() {
  chargerMutex = xSemaphoreCreateMutex();
  if (chargerMutex == nullptr) {
    LOG("[MCP] Failed to create mutex — halting\n");
    while (true) { delay(1000); }
  }

  xTaskCreatePinnedToCore(
    chargerBusTask,        // task function
    "chargerBusTask",      // name
    4096,                  // stack bytes
    nullptr,               // parameter
    6,                     // priority — above bikeBusTask (5)
    &chargerBusTaskHandle, // handle
    0                      // Core 0
  );
}

// ---------------------------------------------------------------------------
// Power ramp task — Core 1, priority 2
//
// Runs every 1 s. Steps ctrl.currentPowerW toward the cutback-limited
// effectiveTarget by at most ctrl.rampStepW (default DEFAULT_RAMP_STEP_W,
// 100 W) per tick — up to 2× that, capped at 500 W, when a cutback is pulling
// the target down, and straight to 0 when a cutback table returns a 0 C-rate.
// It then computes the voltage/current command and writes it to chargerBus
// under chargerMutex.
//
// Voltage selection:
//   Commanded voltage is the CEILING, not the measured pack voltage:
//     CC   -> voltCeiling = min(ctrl.targetVoltDv, MAX_CHARGE_VOLTAGE_DV)
//     CV   -> cvTargetDv (the ceiling for a voltage-triggered CV, the pack
//             voltage at transition for a taper- or plateau-triggered one)
//     DONE -> 0 with cmdStart false
//   The pack voltage used for the phase/cutback decisions (rawPackDv) is the
//   monolith reading plus sagAdjDv when that is positive (BMS_PACK_CONFIG
//   0x288 bytes 0-1, dV — the BMS's estimate of sag under load). The PowerTank
//   voltage is NOT added: it sits in parallel with the monolith, so the two
//   share one rail and summing them would double the reading.
//
// Current calculation:
//   I = P / V  (W / V = A), V being the pack voltage clamped to the ceiling.
//   The total is clamped to CELL_MAX_CHARGE_C × packAH, then divided by
//   max(configured charger count, detected charger count) to get the
//   per-charger figure, stored as dA (0.1 A units) for the heartbeat. A floor
//   of 1 dA is applied so the charger doesn't see a zero-current command while
//   ramping through very low power steps.
//
// Session energy is accumulated each tick from the MEASURED charger output,
// not the commanded power: Wh += (I_actual × V_pack) / 3600, Ah += I_actual / 3600.
//
// Failsafe: every path that decides the chargers must stop raises g_forceStop
// BEFORE attempting its chargerMutex write, so a lost write cannot leave a
// stale START on the bus (see sendHeartbeat()).
// ---------------------------------------------------------------------------

// Count the number of data rows in /cycles.csv (lines minus the header).
// Called once at FFat mount; g_cycleCount is incremented cheaply thereafter.
static uint16_t countCycleRecords() {
  if (!g_fatReady) return 0;
  File f = FFat.open("/cycles.csv", FILE_READ);
  if (!f) return 0;
  // Block reads (audit 2026-07) — the old one-byte-per-read() loop took
  // seconds at boot once the CSV grew to hundreds of KB.
  uint16_t newlines = 0;
  uint8_t  cbuf[256];
  while (f.available()) {
    size_t n = f.read(cbuf, sizeof(cbuf));
    if (n == 0) break;
    for (size_t i = 0; i < n; i++) {
      if (cbuf[i] == '\n') newlines++;
    }
  }
  f.close();
  // Header line accounts for one '\n'; remainder are data rows.
  return (newlines > 0) ? newlines - 1 : 0;
}

// Append one completed charge cycle to /cycles.csv on FFat.
// Called once per cycle from rampTask when phase transitions to PHASE_DONE.
// Thread-safety: only ever called from rampTask; g_cycle is task-local.
static void appendCycleRecord(const CycleRecord& r) {
  if (!g_fatReady) return;
  // Bound growth + surface the full-disk case instead of failing silently
  // (finding #17). Each record is well under 200 bytes; refuse to append once
  // free space drops below a safety margin so the partition can't fill up and
  // start dropping writes invisibly. The user can download + clear via the
  // /api/cycles endpoints to reclaim space.
  const size_t FAT_MIN_FREE = 32768;  // keep 32 KB headroom
  if (FFat.freeBytes() < FAT_MIN_FREE) {
    static unsigned long lastFullLog = 0;
    unsigned long nowMs = millis();
    if (lastFullLog == 0 || nowMs - lastFullLog >= 60000UL) {
      lastFullLog = nowMs;
      LOG("[FAT] cycles.csv not appended — only %lu KB free (clear it via the "
          "dashboard to reclaim space)\n", (unsigned long)(FFat.freeBytes() / 1024UL));
    }
    return;
  }
  File f = FFat.open("/cycles.csv", FILE_APPEND);
  if (!f) { LOG("[FAT] Cannot open /cycles.csv for append\n"); return; }
  // Write CSV header if this is a brand-new file (size 0 before the write)
  if (f.size() == 0) {
    f.println("timestamp,preset_pct,start_v,end_v,start_soc,end_soc,"
              "start_temp,end_temp,total_ah,total_wh,bulk_min,"
              "absorption_min,charger_count,abort_reason");
  }
  char line[180];
  // total_ah / total_wh stay decimal ("12.34" Ah, "1234.5" Wh) so the column
  // keeps the meaning the header advertises — the x100/x10 fixed-point is a
  // storage detail, not the CSV contract. That is also why widening
  // total_ah_x100 to uint32_t (STAB-11) needed no change here: the /100.0f and
  // /10.0f divisions convert both fields to double before they reach the
  // varargs, so %.2f / %.1f are correct for any integer width the struct uses.
  snprintf(line, sizeof(line),
    "%s,%d,%.1f,%.1f,%d,%d,%d,%d,%.2f,%.1f,%d,%d,%d,%d",
    r.timestamp,
    (int)r.preset_pct,
    r.start_v_dv / 10.0f,       r.end_v_dv / 10.0f,
    (int)r.start_soc,            (int)r.end_soc,
    (int)r.start_temp,           (int)r.end_temp,
    r.total_ah_x100 / 100.0f,
    r.total_wh_x10  / 10.0f,
    (int)r.bulk_min,
    (int)r.absorption_min,
    (int)r.charger_count,
    (int)r.abort_reason);
  f.println(line);
  f.close();
  g_cycleCount++;
  LOG("[FAT] Cycle record appended (#%d): %s\n", (int)g_cycleCount, line);
}

// Narrow a raw pack temperature (short °C) to the int8_t a CycleRecord stores.
// A plain (int8_t) cast is wrong twice over: ZERO_TEMP_INVALID (-32768) wraps to
// 0, writing a fabricated "0 °C" into the training data that nothing downstream
// can tell from a real reading, and any out-of-range value wraps to an arbitrary
// in-range one. Map "no reading" to -128 — the single reserved sentinel the CSV
// consumer (and cycle_record.h) treats as missing — and clamp everything else
// into the representable window so a garbled frame can only ever be recorded as
// an implausible-but-honest -127 / 127.
static int8_t cycleTempI8(short v) {
  if (v <= TEMP_INVALID_THRESHOLD) return -128;   // no usable reading
  if (v >  127) return  127;
  if (v < -127) return -127;
  return (int8_t)v;
}

void rampTask(void* /*pvParameters*/) {
  LOG("[RAMP] rampTask running on core %d\n", xPortGetCoreID());
  // Subscribe to the task watchdog (STAB-5) — fed once per 1 s tick below.
  if (esp_task_wdt_add(nullptr) != ESP_OK) {
    LOG("[WDT] rampTask could not subscribe to the task watchdog\n");
  }
  session.startMs = millis();

  // CC / CV / DONE state machine — exposed to users as "Bulk / Absorption / Float"
  // ──────────────────────────────────────────────────────────────────────────────
  // PHASE_CC   ("Bulk"):       Constant-current bulk charge; voltage-based and
  //                            hot cutbacks applied.
  // PHASE_CV   ("Absorption"): Pack voltage has reached the ceiling; charger
  //                            stays on at the ceiling voltage to equalize the
  //                            cells. Transitions to DONE when actual delivered
  //                            current drops below CV_TERM_A (≈ C/20) OR after
  //                            CV_HOLD_MS elapses (safety timeout, 1 h).
  // PHASE_DONE ("Float"):      Cycle complete; charger stopped, cells resting.
  //                            Resets to CC only after the pack has discharged
  //                            below the re-engage floor (see DONE→CC logic).
  enum RampPhase : uint8_t { PHASE_CC = 0, PHASE_CV = 1, PHASE_DONE = 2 };
  static RampPhase phase      = PHASE_CC;
  static unsigned long cvMs   = 0;   // millis() when CV phase was entered
  // TOTAL amps ceiling for CV, in dA, latched at the CC→CV transition. Stored
  // as a total (not per-charger) so the live divisor and the 1 C ceiling can be
  // re-applied on every CV tick — see the write in the charger-bus update.
  static uint16_t cvAmpsTotalDa = 0;
  // CV target voltage — set at CC→CV transition, used by both the charger
  // command and the load-sag (CV→CC) detector. For voltage-triggered CV
  // this is the ceiling. For taper- or plateau-triggered CV this is the
  // ACTUAL pack voltage at transition time (the pack can't be pushed
  // higher under VCB, so commanding the ceiling would immediately trip
  // the load-sag detector and bounce back to CC). 0 = not in CV / unset.
  static uint16_t cvTargetDv  = 0;
  static bool prevEnabled     = false; // for rising-edge detection on `enabled`
  // Current-taper CC→CV trigger: when the chargers can't push more current
  // into the pack despite us commanding power (because the pack has hit its
  // physical ceiling below the configured voltage target), the cycle never
  // leaves CC under the voltage-only trigger. Tracks the timestamp at which
  // measured charger amps first dropped below the taper threshold while
  // we're at our commanded power; cleared whenever amps recover. Once the
  // duration crosses CC_TAPER_MS, we transition to CV regardless of pack
  // voltage. See Bugs 1/2 in [project_charging_bugs_2026-04-26.md].
  static unsigned long ccTaperStartMs = 0;

  // VCB voltage-plateau CC→CV trigger — companion to the current-taper trigger.
  // When the voltage cutback table (VcbON) is actively limiting power AND the
  // pack is below the voltage ceiling, the current-taper trigger won't fire
  // (the charger IS delivering all VCB-allowed current; the pack just can't
  // reach the ceiling physically). Instead watch for the pack voltage to
  // stabilise within ±VCB_PLATEAU_BAND_DV for VCB_PLATEAU_MS; a stable
  // voltage under VcbON means the pack has saturated at the VCB knee.
  static unsigned long voltPlateauStartMs = 0;
  static long          plateauRefDv       = 0;

  // millis() of the first tick on which a pack temperature sensor stopped
  // returning a usable reading; 0 = both valid, or the watch has been reset.
  // Declared at task scope rather than inside the safe-envelope block that uses
  // it so the disabled branch can clear it with the rest of the per-session
  // state: a session that ended with a dead sensor otherwise left the timer
  // armed, and the next session could inherit an immediate INHIBIT_TEMP_UNKNOWN
  // before any telemetry had a chance to arrive.
  static unsigned long tempInvalidSinceMs = 0;

  // millis() of the most recent `enabled` false→true edge. Both the charger
  // fault scan and the overvoltage detector ignore what the chargers report for
  // CHARGER_FAULT_GRACE_MS after this point: an Elcon that has been idle is
  // asserting its comm-timeout bit and reporting a starting-state output until
  // it has seen a few command frames, and reading that as a fault would latch a
  // stop on every single charge start. 0 = no edge seen yet.
  static unsigned long enableEdgeMs = 0;
  const unsigned long  CHARGER_FAULT_GRACE_MS = 5000UL;

  // Rate limiter shared by every "chargerMutex timeout while commanding STOP"
  // report (STAB-2). One timestamp for all of them: if the mutex is wedged all
  // four stop paths hit it, and four separate 5 s limiters would just quadruple
  // the noise without telling us anything more.
  static unsigned long lastStopMutexLogMs = 0;

  // Cycle data logger — record under construction and transition timestamps.
  // All reads/writes are from rampTask only; no mutex needed.
  static CycleRecord   g_cycle    = {};
  static unsigned long ccEntryMs  = 0;   // millis() when current CC phase began
  static unsigned long cvEntryMs  = 0;   // millis() when current CV phase began
  // STAB-11 / D3: this cycle's own energy counters. The session accumulators
  // cannot be used for this, not even as a start/end pair: they are
  // session-to-date, they survive across cycles, and the user can zero them at
  // any moment with Reset Session (/api/control or the MQTT button). A
  // start-snapshot-and-subtract scheme under-reports whenever a reset lands
  // mid-cycle — baseline 5000 Wh, reset to 0, cycle goes on to deliver 6000 Wh,
  // and the subtraction records 1000. Accumulating independently here is
  // immune: these two are zeroed at CC entry and incremented alongside every
  // sessionAddCC() call in the tick, so Reset Session moves the dashboard
  // counters without touching what this cycle has recorded.
  static float         cycleWh = 0.0f;   // Wh delivered since this cycle's CC entry
  static float         cycleAh = 0.0f;   // Ah delivered since this cycle's CC entry
  static RampPhase     lastPhase  = PHASE_DONE; // phase at end of previous tick

  const unsigned long CV_HOLD_MS = 60UL * 60UL * 1000UL; // 1 h absorption safety timeout
  const unsigned long CV_MIN_MS  = 120000UL;              // don't terminate CV before 2 min
  const float         CV_TERM_A  = 2.0f;                  // stop CV when total amps < 2 A
  // Current-taper CC→CV thresholds. Trigger fires when measured charger
  // amps stay below max(CC_TAPER_FLOOR_A, expected_amps * CC_TAPER_FRAC)
  // for CC_TAPER_MS while we're commanding ≥90% of our user setpoint. The
  // floor handles low-target settings (e.g. 100 W trickle where expected
  // amps are tiny anyway); the fractional check handles normal targets
  // where "10% of expected" is a meaningful "the pack's done" signal.
  const float         CC_TAPER_FLOOR_A = 0.5f;
  const float         CC_TAPER_FRAC    = 0.10f;
  const unsigned long CC_TAPER_MS      = 90000UL;   // 90 s sustained low-current
  // VCB voltage-plateau trigger thresholds (see voltPlateauStartMs above).
  const long           VCB_PLATEAU_BAND_DV = 5;        // ±0.5 V stability window
  const unsigned long  VCB_PLATEAU_MS      = 300000UL; // 5 min sustained plateau

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000)); // 1 s ramp tick
    // Feed the task watchdog here, not at the end of the tick: several paths
    // below `continue` (controlMutex timeout, charging disabled, stale BMS
    // telemetry), and a feed further down would be skipped by all of them.
    // This says "the task is still scheduling", which is exactly what the WDT
    // is for; whether a tick did useful work is the g_rampHeartbeat dead-man's
    // job (see the bump after the control snapshot).
    esp_task_wdt_reset();

    // ── Control snapshot ──────────────────────────────────────────────────────
    uint16_t target     = 0;
    uint16_t current    = 0;
    bool     enabled    = false;
    uint8_t  nChargers  = 1;
    uint16_t rampStep   = DEFAULT_RAMP_STEP_W;
    uint16_t targetVolt = MAX_CHARGE_VOLTAGE_DV;

    if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
      // (audit 2026-07) A timeout here used to leave enabled=false, so the
      // disabled branch below ran: phase forced back to CC, cvTargetDv and the
      // taper/plateau watches cleared — one transient contention mid-CV
      // silently restarted the whole cycle (and corrupted the CycleRecord in
      // progress). Instead skip the tick WITHOUT touching any state — and
      // WITHOUT bumping g_rampHeartbeat, so if the mutex stays wedged the
      // dead-man in sendHeartbeat() forces charger STOP within 5 s.
      LOG("[RAMP] controlMutex timeout — tick skipped\n");
      continue;
    }
    target     = ctrl.targetPowerW;
    current    = ctrl.currentPowerW;
    enabled    = ctrl.enabled;
    nChargers  = ctrl.chargerCount;
    rampStep   = ctrl.rampStepW;
    targetVolt = ctrl.targetVoltDv;
    if (nChargers < 1) nChargers = 1;
    xSemaphoreGive(controlMutex);

    // Dead-man heartbeat — bumped once per SUPERVISED tick (after the control
    // snapshot succeeded, before any other `continue`) so chargerBusTask can
    // tell this task is still advancing and still acting on fresh control
    // state. If we ever wedge — including on a stuck controlMutex — the
    // counter freezes and the charger heartbeat falls back to STOP.
    g_rampHeartbeat++;

    // Detect rising edge of `enabled` (off → on) — the start of a charge
    // session, at which point BULK must compare pack voltage against the target
    // before committing to charge.
    //
    // The edge is LATCHED into pendingStartEval rather than acted on directly.
    // prevEnabled has to be updated here (it is the edge detector's own state),
    // but the evaluation itself can only happen ~150 lines further down, after
    // the live snapshot — and three `continue` paths sit in between (charging
    // disabled, BMS telemetry stale, outside the safe charging envelope). Using
    // `justEnabled` directly meant any of those consumed the edge and silently
    // discarded the start check.
    //
    // That is exactly what happened on a boot with charge-on-boot enabled: tick
    // 1 saw the edge, hit the staleness guard because no BMS frame had arrived
    // yet, and continued. From tick 2 on the edge was gone, phase was still
    // PHASE_CC, and once telemetry appeared a pack already above target went
    // straight into the CC→CV voltage trigger and commanded the chargers on.
    // Observed as: target 80 %, bike at 84 %, charging anyway, already showing
    // Absorption a few seconds after power-up.
    //
    // The latch is only cleared once a tick actually reaches the evaluation
    // point with a valid pack voltage — see "Charge-start evaluation" below.
    static bool pendingStartEval = false;
    bool justEnabled = enabled && !prevEnabled;
    prevEnabled = enabled;
    if (justEnabled) {
      pendingStartEval = true;
      // Start of the fault/overvoltage grace window — see enableEdgeMs above.
      enableEdgeMs = millis();
      // The charger-fault latch (STAB-4) requires a manual re-arm: this off→on
      // edge IS that re-arm. Clearing it here rather than on the fault going
      // away means a unit that faults, drops off the bus and reappears cannot
      // resume charging on its own — the user has to press Stop then Charge.
      if (g_chargerFault) {
        LOG("[RAMP] Charge re-enabled — clearing latched charger fault\n");
        g_chargerFault = false;
      }
    }

    // Disabled: reset to CC, stop charger, clear all per-session state, skip tick
    if (!enabled) {
      phase              = PHASE_CC;
      g_rampPhase        = (uint8_t)PHASE_CC;  // keep dashboard/MQTT phase in sync
      g_thermalThrottle  = false;
      g_chargeInhibit    = INHIBIT_NONE;  // not inhibited, just switched off
      // Clear the charging-related protection flags too: with charging off,
      // "charging stopped, waiting for bike CAN" would be a false statement.
      g_bmsStale         = false;
      g_currentClamped   = false;
      // Recomputed every tick from the live charger count while charging, so it
      // would otherwise stay raised after a switch-off. g_chargerFault is
      // deliberately NOT cleared here — it is latched until the next enable
      // edge, and its banner is exactly what the user needs to see meanwhile.
      g_chargerCountMismatch = false;
      // Switching off abandons any pending start evaluation — the next enable
      // edge will raise a fresh one.
      pendingStartEval   = false;
      ccTaperStartMs     = 0;
      voltPlateauStartMs = 0;
      plateauRefDv       = 0;
      cvTargetDv         = 0;
      // D1 — the cycle logger's two transition variables have to be reset here
      // as well, and lastPhase specifically must be dragged along with the
      // `phase = PHASE_CC` above. lastPhase is only assigned at the BOTTOM of
      // the tick, and this branch `continue`s past that point, so a Stop
      // pressed during absorption left lastPhase stuck at PHASE_CV with
      // cvEntryMs still set. The next enable whose start evaluation decided
      // "nothing to charge" went straight to PHASE_DONE, and the DONE hook —
      // which trusts lastPhase == PHASE_CV to mean "a real absorption phase
      // just ended" — appended a fabricated row: the previous cycle's start
      // values, an absorption_min that counted all the off-time in between,
      // and abort_reason 0 (current_taper) for a cycle that never ran.
      // Setting lastPhase here keeps it honest about the phase this branch
      // just forced, which is the same thing the bottom-of-tick assignment
      // would have done had it been reached.
      cvEntryMs          = 0;
      lastPhase          = PHASE_CC;
      // Per-session too: with charging off there is nothing to inhibit, so the
      // "how long have the sensors been dark" timer must not keep running.
      // Leaving it armed across a switch-off meant the next session could be
      // inhibited for INHIBIT_TEMP_UNKNOWN on its very first tick, crediting it
      // with invalidity that accumulated while the charger was idle.
      tempInvalidSinceMs = 0;
      // Zero the commanded power. The ramp task owns currentPowerW, so its
      // disabled branch is the authoritative place to clear it — this covers
      // EVERY disable path (dashboard button, MQTT, auto-disable when the
      // chargers vanish, boot-default re-apply after an auto-enable), not just
      // the two command handlers that also happen to zero it. Without this a
      // stale ramped value (e.g. 800 W) lingers in the "Current" readout and
      // the current_power_w MQTT sensor while charging is OFF.
      if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        ctrl.currentPowerW = 0;
        xSemaphoreGive(controlMutex);
      }
      // Assert the failsafe BEFORE the (best-effort) mutex write, so a lost
      // write still results in a STOP frame on the bus (STAB-2).
      g_forceStop = true;
      if (xSemaphoreTake(chargerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        chargerBus.cmdVoltDv = 0;
        chargerBus.cmdAmpsDa = 0;
        chargerBus.cmdStart  = false;
        xSemaphoreGive(chargerMutex);
      } else {
        unsigned long tms = millis();
        if (lastStopMutexLogMs == 0 || tms - lastStopMutexLogMs >= 5000UL) {
          lastStopMutexLogMs = tms;
          LOG("[RAMP] chargerMutex timeout on STOP — forceStop asserted\n");
        }
      }
      continue;
    }

    // ── Live pack snapshot ────────────────────────────────────────────────────
    long  monolithDv = 0;
    short sagAdjDv   = 0;
    long  ptDv       = 0;
    bool  ptPresent  = false;
    short monolithAH = 114; // nominal fallback
    // Defaults are the INVALID sentinel, not -127: if the liveMutex take below
    // times out these locals are what the rest of the tick sees, and -127 is
    // only "obviously wrong" by convention — it still passes as a number to
    // anything that does not know to look for it. ZERO_TEMP_INVALID is the one
    // value every consumer here already tests for (TEMP_INVALID_THRESHOLD), so
    // a missed snapshot degrades to "no reading" rather than "absurdly cold".
    short maxTemp    = ZERO_TEMP_INVALID;
    short minTemp    = ZERO_TEMP_INVALID; // coldest thermocouple — drives COLD_CUTBACK
    short ptMaxTemp  = ZERO_TEMP_INVALID; // PowerTank hottest thermocouple (finding #4)
    short ptMinTemp  = ZERO_TEMP_INVALID; // PowerTank coldest thermocouple (finding #4)
    unsigned long bmsLastMs = 0; // last monolith voltage frame (staleness, #3)

    if (xSemaphoreTake(liveMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      monolithDv = live.monolithVoltageDv;
      sagAdjDv   = live.monolithSagAdjDv;
      ptDv       = live.powerTankVoltageDv;
      ptPresent  = live.powerTankPresent;
      if (live.monolithAH > 0) monolithAH = live.monolithAH;
      maxTemp    = live.monolithMaxTemp;
      minTemp    = live.monolithMinTemp;
      ptMaxTemp  = live.powerTankMaxTemp;
      ptMinTemp  = live.powerTankMinTemp;
      bmsLastMs  = live.bms0LastMs;
      xSemaphoreGive(liveMutex);
    }

    // Raw (unclamped) pack voltage — all cutback decisions use this.
    // sagAdjDv is the BMS sag-compensation offset (additive, not a replacement).
    // The PowerTank is wired in PARALLEL with the monolith on stock Zeros, so
    // both packs sit at the same rail voltage — the monolith reading already
    // IS the pack voltage and the PowerTank voltage must NOT be added (doing so
    // would double the reading and instantly trip skip-to-DONE / max cutback).
    long rawPackDv = monolithDv + (sagAdjDv > 0 ? (long)sagAdjDv : 0L);

    uint16_t voltCeiling = min(targetVolt, MAX_CHARGE_VOLTAGE_DV);
    const long hyst = 10; // 1.0 V in dV

    // ── Measured charger output snapshot ─────────────────────────────────────
    // Read once up front so that BOTH the current-taper trigger (CC→CV
    // when pack saturates below the voltage ceiling) AND the CC-phase
    // session accumulator can use the actual delivered amps. Previously
    // the CC accumulator added energy at the COMMANDED rate, which kept
    // counting Wh/Ah even when the chargers reported 0 A — exactly the
    // pathology Bug 3 in [project_charging_bugs_2026-04-26.md] describes.
    unsigned long now0             = millis();
    float         actualA          = 0.0f;
    uint8_t       detectedChargers = 0;
    if (xSemaphoreTake(chargerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      for (int i = 0; i < MAX_CHARGERS; i++) {
        if (chargerBus.chargers[i].present &&
            (now0 - chargerBus.chargers[i].lastSeenMs) < CHARGER_TIMEOUT_MS) {
          actualA += chargerBus.chargers[i].ampsDa / 10.0f;
        }
      }
      // Count of units actually answering, maintained (and clamped to
      // MAX_ACTIVE_CHARGERS) by chargerBusTask. 0 while nothing has been seen.
      detectedChargers = chargerBus.chargerCount;
      xSemaphoreGive(chargerMutex);
    }

    // ── Per-charger current divisor (STAB-1) ─────────────────────────────────
    // The divisor used to be ctrl.chargerCount alone — the number the user
    // picked in the UI — while the detected count was only ever tested as a
    // bool ("any charger present?"). All units share one CAN ID and therefore
    // one broadcast command, so each of them delivers the per-charger figure:
    // commanding total/2 with three units on the bus puts 1.5× the intended
    // current into the pack. Dividing by whichever count is LARGER is the
    // fail-safe direction — it can only ever under-deliver.
    //
    // This is a limit on how much damage a wrong setting can do, not a licence
    // to run with one: a detected count above the configured one also raises
    // g_chargerCountMismatch below and stops charging until they agree.
    uint8_t divisor = (detectedChargers > nChargers) ? detectedChargers : nChargers;
    if (divisor < 1) divisor = 1;

    // Detected more units than the user says are installed. Both counts are
    // already clamped to MAX_ACTIVE_CHARGERS, so this is specifically a
    // setting-vs-reality disagreement, not the >4 wiring fault
    // (g_chargerCountClamped) — that one keeps its own banner.
    bool chargerCountMismatch = (detectedChargers > nChargers);
    g_chargerCountMismatch = chargerCountMismatch;
    if (chargerCountMismatch) {
      static unsigned long lastMismatchLogMs = 0;
      // `last == 0 ||` is the file's rate-limiter idiom: without it a first
      // event inside the first N ms of uptime is silently swallowed, because
      // `now - 0 >= N` is false while millis() is still small. That is exactly
      // when a boot-time charger-count mismatch would occur.
      if (lastMismatchLogMs == 0 || now0 - lastMismatchLogMs >= 10000UL) {
        lastMismatchLogMs = now0;
        LOG("[RAMP] %u chargers detected but %u configured — STOP until the "
            "setting matches\n", (unsigned)detectedChargers, (unsigned)nChargers);
      }
    }

    // ── Bike BMS staleness guard (finding #3) ─────────────────────────────────
    // The monolith broadcasts voltage frames (0x388) at ~10 Hz. If the bike CAN
    // bus drops mid-charge (charge connector pulled, bike powered down, wiring
    // fault) live.* would otherwise freeze at its last values and we'd keep
    // commanding the chargers from stale voltage/temperature readings forever —
    // unlike the chargers themselves, which have CHARGER_TIMEOUT_MS. Treat data
    // older than BMS_STALE_TIMEOUT_MS — or never received (bmsLastMs == 0) — as
    // unsafe: command STOP and skip the rest of the tick. This mirrors the
    // charger's own 5 s heartbeat cutoff and is fail-safe. millis() subtraction
    // is rollover-safe. Charging resumes automatically once fresh frames return.
    const unsigned long BMS_STALE_TIMEOUT_MS = 5000UL;
    // `bmsLastMs == 0` is tested explicitly. The comment above has always
    // claimed "never received" counts as stale, but the subtraction alone does
    // not deliver that: with bmsLastMs == 0 the expression reduces to
    // `now0 > BMS_STALE_TIMEOUT_MS`, so for the first 5 seconds of uptime the
    // guard passed with no BMS telemetry whatsoever. Harmless in practice —
    // rawPackDv is 0 then and the downstream `packDv > 0` guards hold — but the
    // code did not do what it documented.
    g_bmsStale = (bmsLastMs == 0) || ((now0 - bmsLastMs) > BMS_STALE_TIMEOUT_MS);
    if (g_bmsStale) {
      g_thermalThrottle  = false;
      // Not an envelope inhibit — we simply have no telemetry to judge with, and
      // the dedicated stale-data message below is the accurate one to surface.
      g_chargeInhibit    = INHIBIT_NONE;
      g_currentClamped   = false;
      g_etaMinutes       = -1;
      ccTaperStartMs     = 0;
      voltPlateauStartMs = 0;
      plateauRefDv       = 0;
      if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        ctrl.currentPowerW = 0;
        xSemaphoreGive(controlMutex);
      }
      // Failsafe first, mutex write second (STAB-2).
      g_forceStop = true;
      if (xSemaphoreTake(chargerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        chargerBus.cmdVoltDv = 0;
        chargerBus.cmdAmpsDa = 0;
        chargerBus.cmdStart  = false;
        xSemaphoreGive(chargerMutex);
      } else if (lastStopMutexLogMs == 0 || now0 - lastStopMutexLogMs >= 5000UL) {
        lastStopMutexLogMs = now0;
        LOG("[RAMP] chargerMutex timeout on STOP — forceStop asserted\n");
      }
      static unsigned long lastStaleLogMs = 0;
      if (lastStaleLogMs == 0 || now0 - lastStaleLogMs >= 5000UL) {
        lastStaleLogMs = now0;
        LOG("[RAMP] Bike BMS data stale (%lus since last frame) — charger STOP, "
            "waiting for bike CAN\n", (now0 - bmsLastMs) / 1000UL);
      }
      continue;
    }

    // ── Worst-case temperatures across both packs ────────────────────────────
    // Moved above the safe-envelope gate below (it needs them) — this used to
    // sit further down with the cutback calculations. When a PowerTank is
    // present and live (reporting voltage — the same "PT is really here" gate
    // the voltage sum uses), fold its hottest / coldest thermocouple in so the
    // hot and cold limits track whichever pack is nearest its limit.
    //
    // Each PowerTank reading is validity-checked before it is folded in. The
    // cold side is why this matters: ZERO_TEMP_INVALID is -32768, so an
    // unguarded `if (ptMinTemp < coldTemp)` adopted the sentinel as the coldest
    // cell the moment a PowerTank was detected but had not yet sent a temp
    // frame (or had a dead thermistor). That poisons coldTemp for the whole
    // tick — COLD_CUTBACK would see an impossibly cold pack. Folding in only
    // valid readings means a PowerTank with no usable temperature simply does
    // not contribute; the monolith's own reading still governs, and if BOTH
    // packs are dark the per-sensor validity test below catches it.
    short hotTemp  = maxTemp;
    short coldTemp = minTemp;
    if (ptPresent && ptDv > 0) {
      bool ptHotValid  = (ptMaxTemp > TEMP_INVALID_THRESHOLD);
      bool ptColdValid = (ptMinTemp > TEMP_INVALID_THRESHOLD);
      // A monolith reading that is itself invalid must not win the comparison
      // either, so take the PowerTank value outright in that case.
      if (ptHotValid  && (hotTemp  <= TEMP_INVALID_THRESHOLD || ptMaxTemp > hotTemp))
        hotTemp  = ptMaxTemp;
      if (ptColdValid && (coldTemp <= TEMP_INVALID_THRESHOLD || ptMinTemp < coldTemp))
        coldTemp = ptMinTemp;
    }

    // Per-sensor validity of the merged worst-case readings. Computed here, at
    // tick scope, because two places need it: the safe-envelope gate just below
    // and the HOT_CUTBACK / COLD_CUTBACK lookups further down. Each limit is
    // applied on the strength of its OWN sensor — one dead thermistor must not
    // disable the other's protection.
    bool coldValid = (coldTemp > TEMP_INVALID_THRESHOLD);
    bool hotValid  = (hotTemp  > TEMP_INVALID_THRESHOLD);

    // ── Cell safe-charging-envelope gate ─────────────────────────────────────
    // Farasis IMP06160230P25A: charging permitted only 0-45 °C, and never below
    // ~2.5 V/cell (copper current-collector dissolution — see
    // CELL_V_CHARGE_FLOOR_MV). None of this was previously enforced:
    //
    //   * COLD_CUTBACK's lowest entry was 0 °C -> 0.2 C, and find_cutback's
    //     AT_OR_BELOW mode returns the first threshold >= measurement, so at
    //     -20 °C it returned 0.2 C and charged happily. Sub-zero charging is
    //     precisely the lithium-plating case the cold table exists to prevent.
    //   * HOT_CUTBACK started at 50 °C, so 46-49 °C — already past the
    //     datasheet maximum — got no limit at all.
    //   * There was no pack-voltage floor of any kind.
    //
    // Unlike a cutback (reduce power, keep charging) this is an outright stop.
    // Hysteresis: once inhibited we require the pack to come back
    // CHARGE_TEMP_HYSTERESIS_C *inside* the limit before charging resumes, so a
    // cell hovering at exactly 0 or 45 °C can't toggle the chargers every tick.
    // Deliberately placed AFTER the staleness guard, so it only ever acts on
    // fresh telemetry, and it re-evaluates every tick — recovery is automatic.
    {
      uint8_t inhibit = INHIBIT_NONE;
      uint8_t prev    = g_chargeInhibit;

      // Temperature limits, with the re-arm band applied only when already
      // inhibited for that same reason.
      short coldLimit = (prev == INHIBIT_TOO_COLD)
                          ? (short)(CHARGE_TEMP_MIN_C + CHARGE_TEMP_HYSTERESIS_C)
                          : CHARGE_TEMP_MIN_C;
      short hotLimit  = (prev == INHIBIT_TOO_HOT)
                          ? (short)(CHARGE_TEMP_MAX_C - CHARGE_TEMP_HYSTERESIS_C)
                          : CHARGE_TEMP_MAX_C;

      // Validity is per SENSOR, not a single combined flag (audit 2026-09).
      // coldTemp/hotTemp carry ZERO_TEMP_INVALID until a temperature frame has
      // arrived, and the decoder returns it for a disconnected sensor or a short
      // frame. The old code required BOTH to be valid before applying EITHER
      // limit, so one dead thermistor disabled the other sensor's protection for
      // the whole 10 s grace window: a genuinely 50 °C pack reported by a working
      // hot sensor was ignored because the cold sensor had failed. Each limit now
      // stands on its own reading. coldValid / hotValid are computed just above
      // this block, at tick scope, because the cutback lookups need them too.

      // ...but "no usable temperature" cannot simply be ignored either (STAB-10).
      // Before ZERO_TEMP_INVALID a dead thermistor decoded as 0 °C, which reads
      // as a healthy pack: the hot cutback stopped limiting and the 45 °C
      // inhibit could never fire. Allow TEMP_INVALID_INHIBIT_MS of continuous
      // invalidity to cover boot and the odd dropped frame, then inhibit.
      // EITHER sensor being dark arms the timer — see INHIBIT_TEMP_UNKNOWN.
      // tempInvalidSinceMs is declared at task scope so the disabled branch can
      // clear it between sessions.
      const unsigned long TEMP_INVALID_INHIBIT_MS = 10000UL;
      bool bothValid = coldValid && hotValid;
      if (bothValid)                       tempInvalidSinceMs = 0;
      else if (tempInvalidSinceMs == 0)    tempInvalidSinceMs = now0;
      bool tempUnknown = !bothValid && tempInvalidSinceMs != 0 &&
                         (now0 - tempInvalidSinceMs) >= TEMP_INVALID_INHIBIT_MS;

      // If the sensor that raised an inhibit goes dark, HOLD that inhibit rather
      // than clearing it (audit 2026-09, B1). Otherwise a hot thermistor failing
      // at 46 °C would drop the TOO_HOT stop on the very next tick and, with the
      // hot cutback also skipped for an invalid sensor, permit full power until
      // INHIBIT_TEMP_UNKNOWN arms 10 s later. Re-evaluation resumes when the
      // sensor reports again.
      if (prev == INHIBIT_TOO_HOT && !hotValid)          inhibit = INHIBIT_TOO_HOT;
      else if (prev == INHIBIT_TOO_COLD && !coldValid)   inhibit = INHIBIT_TOO_COLD;
      else if (coldValid && coldTemp < coldLimit)        inhibit = INHIBIT_TOO_COLD;
      else if (hotValid && hotTemp  > hotLimit)          inhibit = INHIBIT_TOO_HOT;
      // Voltage floor needs no hysteresis: charging raises pack voltage, so it
      // moves monotonically away from the limit once current starts flowing.
      else if (rawPackDv > 0 && rawPackDv < (long)PACK_V_CHARGE_FLOOR_DV)
        inhibit = INHIBIT_PACK_LOW;
      // Ranked last of the four only because the three above are specific
      // diagnoses and this one is "we cannot tell" — all four stop charging
      // outright, so the ordering is about which message the user sees.
      // No hysteresis: the moment both sensors report again, charging resumes.
      else if (tempUnknown) inhibit = INHIBIT_TEMP_UNKNOWN;

      g_chargeInhibit = inhibit;

      if (inhibit != INHIBIT_NONE) {
        g_thermalThrottle  = false;
        g_currentClamped   = false;   // no current at all, so nothing to clamp
        g_etaMinutes       = -1;
        ccTaperStartMs     = 0;
        voltPlateauStartMs = 0;
        plateauRefDv       = 0;
        if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          ctrl.currentPowerW = 0;
          xSemaphoreGive(controlMutex);
        }
        // Failsafe first, mutex write second (STAB-2).
        g_forceStop = true;
        if (xSemaphoreTake(chargerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          chargerBus.cmdVoltDv = 0;
          chargerBus.cmdAmpsDa = 0;
          chargerBus.cmdStart  = false;
          xSemaphoreGive(chargerMutex);
        } else if (lastStopMutexLogMs == 0 || now0 - lastStopMutexLogMs >= 5000UL) {
          lastStopMutexLogMs = now0;
          LOG("[RAMP] chargerMutex timeout on STOP — forceStop asserted\n");
        }
        // Dedicated message for the "no temperature at all" case — the generic
        // line below would print two sentinel values and no elapsed time.
        if (inhibit == INHIBIT_TEMP_UNKNOWN) {
          static unsigned long lastTempUnknownLogMs = 0;
          if (lastTempUnknownLogMs == 0 || now0 - lastTempUnknownLogMs >= 10000UL) {
            lastTempUnknownLogMs = now0;
            LOG("[RAMP] Pack temperature unknown for %lus — charging inhibited\n",
                (now0 - tempInvalidSinceMs) / 1000UL);
          }
        }
        // Log the transition immediately, then rate-limit the repeat.
        static unsigned long lastInhibitLogMs = 0;
        if (prev != inhibit || (now0 - lastInhibitLogMs) >= 30000UL) {
          lastInhibitLogMs = now0;
          LOG("[RAMP] CHARGE INHIBITED (%s) — charger STOP. "
              "cold %d°C / hot %d°C (permitted %d..%d°C), pack %ld dV "
              "(floor %u dV)\n",
              chargeInhibitName(inhibit), (int)coldTemp, (int)hotTemp,
              (int)CHARGE_TEMP_MIN_C, (int)CHARGE_TEMP_MAX_C,
              rawPackDv, (unsigned)PACK_V_CHARGE_FLOOR_DV);
        }
        continue;
      } else if (prev != INHIBIT_NONE) {
        LOG("[RAMP] Charge inhibit cleared (was: %s) — cold %d°C / hot %d°C, "
            "pack %ld dV. Resuming.\n",
            chargeInhibitName(prev), (int)coldTemp, (int)hotTemp, rawPackDv);
      }
    }

    // ── Charge-start evaluation ──────────────────────────────────────────────
    // BULK's first job at the start of a session: decide whether any charge is
    // actually needed. If the pack is already at or above the target there is
    // nothing to do — go straight to FLOAT without ever commanding the chargers.
    //
    // Gated on pendingStartEval (latched at the enable edge, see above) AND on
    // having a real pack voltage. The `rawPackDv > 0` condition is what makes
    // this correct: the latch is not consumed until there is something genuine
    // to compare against, so a boot where telemetry has not arrived yet defers
    // the decision to the next tick instead of skipping it.
    //
    // `enabled` is deliberately left alone. Going to FLOAT hands the pack to the
    // existing DONE→CC re-engage logic, which tops up once the pack sags below
    // the appropriate floor — the same behaviour as finishing a normal cycle.
    // True only on the single tick that actually performed the start evaluation.
    // Downstream hooks that used to key off `justEnabled` use this instead, so
    // they fire on the tick the session really began rather than on an edge that
    // may have been discarded by an early return.
    bool startEvalRan = false;
    if (pendingStartEval && rawPackDv > 0) {
      pendingStartEval = false;
      startEvalRan     = true;
      int packSoc   = calcSocFromVoltage(rawPackDv);
      int targetSoc = calcSocFromVoltage((long)voltCeiling);
      if (rawPackDv >= (long)voltCeiling) {
        phase      = PHASE_DONE;
        cvTargetDv = 0;
        LOG("[RAMP] Start: no charge needed — pack %ld dV (~%d%%) ≥ target %d dV "
            "(~%d%%). FLOAT; will top up if the pack sags.\n",
            rawPackDv, packSoc, voltCeiling, targetSoc);
      } else {
        LOG("[RAMP] Start: BULK — pack %ld dV (~%d%%) → target %d dV (~%d%%)\n",
            rawPackDv, packSoc, voltCeiling, targetSoc);
      }
    }

    // ── Above-target backstop ────────────────────────────────────────────────
    // Covers every route into "pack is past the target" that is NOT an enable
    // edge, none of which the start evaluation above can see:
    //   * the user lowers the target mid-charge (84 % pack, target moved to 80 %)
    //   * the pack was charged externally, or regen lifted it, while enabled
    // Without this the CC→CV voltage trigger fires instead and CV commands the
    // chargers ON at a voltage BELOW the pack's own for at least CV_MIN_MS
    // (2 min) — the Elcons cannot sink, so it achieves nothing while reporting
    // Absorption.
    //
    // `+ hyst` (1.0 V) is what separates this from a legitimate CC arrival: CC
    // stops AT the ceiling, so a pack more than 1 V above it cannot have got
    // there by charging. Preset spacing is 32-40 dV, so any target change clears
    // the margin comfortably while an exact-ceiling arrival still enters CV.
    //
    // DONE→CC cannot immediately undo this: doneReengageDv is always ≤
    // voltCeiling, so a pack above the ceiling is also above the re-engage floor.
    if (phase != PHASE_DONE && rawPackDv > 0 &&
        rawPackDv > (long)voltCeiling + hyst) {
      LOG("[RAMP] Pack %ld dV is above target %d dV by more than %ld dV — "
          "nothing to charge, %s→FLOAT\n",
          rawPackDv, voltCeiling, hyst, (phase == PHASE_CV) ? "ABSORPTION" : "BULK");
      // D2 — a CV→DONE exit through here IS logged as a cycle (the DONE hook
      // only requires lastPhase == PHASE_CV), but it did not end the way the
      // default abort_reason claims. 0 means "current tapered to the finish
      // line"; this is the opposite — absorption was cut short because the
      // target moved below the pack, or the pack was lifted above it from
      // outside. Mark it 2 (above_target) so Stage 2 training can tell a
      // naturally-terminated absorption from an administratively-ended one.
      // Only meaningful on the CV arm: from CC the record is not written at all.
      if (phase == PHASE_CV) g_cycle.abort_reason = 2;
      phase      = PHASE_DONE;
      cvTargetDv = 0;
    }

    // Target lowered while in ABSORPTION, but not far enough to trip the
    // backstop above (e.g. 100 % → 90 % with the pack at 113.5 V). cvTargetDv
    // was latched at the OLD higher ceiling and the CV branch commands it
    // directly, so without this the chargers keep pushing toward the old target
    // until CV_HOLD_MS — one hour — expires. Enforce the invariant
    // cvTargetDv <= voltCeiling unconditionally; it is self-correcting and
    // needs no change-detection state.
    if (phase == PHASE_CV && cvTargetDv > voltCeiling) {
      LOG("[RAMP] Target lowered during ABSORPTION — CV target %u → %d dV\n",
          (unsigned)cvTargetDv, voltCeiling);
      cvTargetDv = voltCeiling;
    }

    // ── Target-raised re-engage ──────────────────────────────────────────────
    // The sag floor below is pinned at the 80 % preset for ANY target above
    // 80 %, because its job is to stop the pack cycling near the top of charge.
    // That makes it the wrong test when the *target* moves rather than the pack
    // draining: sitting in FLOAT at 84 % and raising the target to 90 % left
    // rawPackDv (1112) above doneReengageDv (1100), so nothing happened — the
    // pack had to sag all the way back to 80 % before it would charge to 90 %.
    // Raising to 100 % behaved the same way.
    //
    // Edge-detected on purpose. A plain "resume whenever pack < ceiling - hyst"
    // would also fire on the normal post-cycle sag, re-engaging about 1 V below
    // target every time and defeating the wide hysteresis entirely. Only a
    // deliberate change of target counts.
    //
    // Placed here, after every `continue` path, so the edge cannot be consumed
    // by a tick that returns early — the same trap that swallowed justEnabled.
    static uint16_t prevVoltCeiling = 0;
    bool ceilingRaised = (prevVoltCeiling != 0) && (voltCeiling > prevVoltCeiling);
    prevVoltCeiling = voltCeiling;

    // Covers FLOAT *and* ABSORPTION. The absorption case is the mirror of the
    // lowered-target fix above: sitting in CV at a latched cvTargetDv of 1100
    // (80 % reached) and raising the target to 90 % left the CV→CC sag detector
    // comparing against the OLD 1100 target, so the pack never looked like it
    // had sagged and it finished the cycle at 80 % regardless of the new
    // setting. Dropping back to BULK is the correct response to "charge more".
    if (ceilingRaised && (phase == PHASE_DONE || phase == PHASE_CV) &&
        rawPackDv > 0 && rawPackDv < (long)voltCeiling - hyst) {
      LOG("[RAMP] Target raised to %d dV (~%d%%) — pack %ld dV (~%d%%) is below "
          "it, %s→BULK\n",
          voltCeiling, calcSocFromVoltage((long)voltCeiling),
          rawPackDv, calcSocFromVoltage(rawPackDv),
          (phase == PHASE_CV) ? "ABSORPTION" : "FLOAT");
      phase = PHASE_CC;
      // Discard the CV target latched against the previous, lower ceiling so a
      // future CC→CV transition recomputes it from the new one.
      cvTargetDv = 0;
    }

    // ── Phase transitions ─────────────────────────────────────────────────────
    // DONE → CC re-engage threshold depends on the user's target voltage:
    //   target ≤ 80 % preset (≤ 1100 dV): re-engage at target − 2 V
    //       (frequent top-off is fine in the gentle SoC range)
    //   target  > 80 % preset (> 1100 dV): re-engage only when pack sags
    //       all the way to the 80 % preset voltage (1100 dV)
    //       (wider hysteresis trades more voltage sag per cycle for
    //        far fewer cycles in the high-SoC band — Li-ion lives
    //        longer if it doesn't repeatedly hover near 100 %)
    const long DONE_REENGAGE_FLOOR_DV = (long)TARGET_VOLT_PRESETS[1].dv; // 80 %
    long doneReengageDv = ((long)voltCeiling <= DONE_REENGAGE_FLOOR_DV)
                            ? ((long)voltCeiling - hyst * 2)
                            : DONE_REENGAGE_FLOOR_DV;
    if (phase == PHASE_DONE && rawPackDv > 0 &&
        rawPackDv <= doneReengageDv) {
      phase = PHASE_CC;
      LOG("[RAMP] DONE→CC: pack at %ld dV ≤ floor %ld dV (ceil %d dV)\n",
          rawPackDv, doneReengageDv, voltCeiling);
    }
    // CV → CC : voltage sag under load (bike in use) — re-enter CC to top up.
    // Compare against cvTargetDv, NOT voltCeiling: for taper- and plateau-
    // triggered CV the pack is already below voltCeiling at transition time,
    // so comparing against voltCeiling would immediately re-fire CV→CC every
    // tick (the 100 % preset oscillation bug fixed v202605171800).
    long cvSagFloor = (cvTargetDv > 0) ? (long)cvTargetDv : (long)voltCeiling;
    if (phase == PHASE_CV && rawPackDv > 0 &&
        rawPackDv < (cvSagFloor - hyst)) {
      phase = PHASE_CC;
      LOG("[RAMP] CV→CC: pack dropped to %ld dV (load sag, CV target %ld dV)\n",
          rawPackDv, cvSagFloor);
    }
    // ── Power cutback limits — compute BEFORE taper check ────────────────────
    // effectiveTarget must be known here so the atRamp gate in the taper
    // check below can compare current against the VCB-limited ceiling rather
    // than the user's raw target. Without this, VcbON causes current to be
    // clamped (e.g. 657 W) while target stays at 9900 W, making atRamp
    // permanently FALSE and the taper watch never starts.
    // packAH scales the 1 C current ceiling, every cutback power limit, the CV
    // current figure and the ETA. It is gated on ingestion (STAB-3), but clamp
    // it again here: this is the single place all of those read it, and the
    // cost of the clamp is nil next to the cost of a bad value getting through.
    float packAH = (monolithAH > 0) ? (float)monolithAH : 114.0f;
    if (packAH > PACK_AH_MAX) packAH = PACK_AH_MAX;
    float voltV  = (rawPackDv > 0) ? rawPackDv / 10.0f : voltCeiling / 10.0f;

    uint32_t voltCbK  = find_cutback((int)rawPackDv, CUTBACK_AT_OR_ABOVE, VOLTAGE_CUTBACK);
    uint32_t voltPwrW = (voltCbK == UINT32_MAX) ? UINT32_MAX
                       : (uint32_t)(voltCbK / 1000.0f * packAH * voltV);

    // hotTemp / coldTemp (worst case across both packs, finding #4) are now
    // computed further up — the safe-charging-envelope gate needs them before
    // this point. Values are unchanged.
    //
    // An INVALID reading is never fed to a cutback table. Feeding the sentinel
    // in produces a confidently wrong answer in both directions: -32768 °C sits
    // below every HOT_CUTBACK threshold (AT_OR_ABOVE → no limit, which is what
    // we want but for the wrong reason) and below every COLD_CUTBACK threshold
    // (AT_OR_BELOW → the harshest cold limit in the table, which would throttle
    // a perfectly warm pack to a crawl on a single dropped frame). Skipping the
    // lookup makes "no reading" mean "no limit from this table" explicitly. It
    // is not a safety hole: a sensor that stays dark for TEMP_INVALID_INHIBIT_MS
    // stops the charge outright via INHIBIT_TEMP_UNKNOWN above.
    uint32_t hotCbK   = hotValid
                       ? find_cutback((int)hotTemp,  CUTBACK_AT_OR_ABOVE, HOT_CUTBACK)
                       : UINT32_MAX;
    uint32_t hotPwrW  = (hotCbK  == UINT32_MAX) ? UINT32_MAX
                       : (uint32_t)(hotCbK  / 1000.0f * packAH * voltV);

    // Cold cutback — limit charge C-rate at low pack temperature to avoid
    // lithium plating. Keyed off the COLDEST cell thermocouple across both
    // packs, the conservative choice for a plating limit. COLD_CUTBACK uses
    // CUTBACK_AT_OR_BELOW semantics: colder pack → lower allowed C-rate. The
    // table tops out at 40 °C / 3 C, well above the system's ~1 C ceiling, so
    // it only actually constrains power once a pack is genuinely cold.
    // Skipped when the cold sensor has no usable reading — see the note on the
    // hot lookup above; this is the direction where feeding the sentinel in
    // would actively misbehave (AT_OR_BELOW returns the harshest entry).
    uint32_t coldCbK  = coldValid
                       ? find_cutback((int)coldTemp, CUTBACK_AT_OR_BELOW, COLD_CUTBACK)
                       : UINT32_MAX;
    uint32_t coldPwrW = (coldCbK == UINT32_MAX) ? UINT32_MAX
                       : (uint32_t)(coldCbK / 1000.0f * packAH * voltV);

    // Effective CC target (cutbacks only apply in CC; CV/DONE ramp current to 0)
    uint32_t powerLimit = (uint32_t)target;
    if (phase == PHASE_CC) {
      if (voltPwrW != UINT32_MAX) powerLimit = min(powerLimit, voltPwrW);
      if (hotPwrW  != UINT32_MAX) powerLimit = min(powerLimit, hotPwrW);
      if (coldPwrW != UINT32_MAX) powerLimit = min(powerLimit, coldPwrW);
    } else {
      powerLimit = 0; // CV/DONE: ramp CC current display to 0
    }

    // Update thermal-throttle indicator: true only when the hot cutback is
    // the active limiter in CC phase (drops the requested target below `target`).
    g_thermalThrottle = (phase == PHASE_CC) &&
                        (hotPwrW != UINT32_MAX) &&
                        (hotPwrW <  (uint32_t)target);
    uint16_t effectiveTarget = (uint16_t)min(powerLimit, (uint32_t)UINT16_MAX);
    bool cutbackActive = (phase == PHASE_CC) && (effectiveTarget < target);

    // CC → CV : two triggers
    //  (a) **voltage trigger** — pack reached the configured ceiling. This is
    //      the textbook CC/CV transition.
    //  (b) **current-taper trigger** — pack saturated below the ceiling
    //      (e.g. 100 % preset is 116.4 V but the user's pack physically
    //      tops out at ~115.1 V). Without this we'd stay in CC forever
    //      because the voltage trigger is unreachable. Heuristic: while
    //      we've ramped up to ≥90 % of the effective (VCB-limited) setpoint,
    //      if measured charger amps stay below max(0.5 A, 10 % of expected)
    //      for a sustained 90 s, the pack isn't accepting more current — call
    //      it done with CC and drop into CV. (Note: using effectiveTarget not
    //      target here is critical — when VcbON clamps current below the user
    //      setpoint, atRamp must compare against the clamped ceiling or it
    //      stays permanently FALSE.)
    bool ccTriggerVoltage = (phase == PHASE_CC && rawPackDv > 0 &&
                             rawPackDv >= (long)voltCeiling);

    bool ccTriggerTaper = false;
    if (phase == PHASE_CC && current > 0 && rawPackDv > 0) {
      // 90 %-of-effective-target gate ensures we don't sample taper during
      // the initial ramp-up (when actualA is naturally still climbing).
      // Using effectiveTarget (VCB-limited) not target: when VcbON clamps
      // current to e.g. 657 W, comparing against 9900 W would always be
      // FALSE and the taper watch would never start.
      bool atRamp = (current * 10u >= ((uint32_t)effectiveTarget * 9u));
      if (atRamp) {
        float expectedA  = (float)current / (rawPackDv / 10.0f);
        float threshA    = max(CC_TAPER_FLOOR_A, expectedA * CC_TAPER_FRAC);
        if (actualA < threshA) {
          if (ccTaperStartMs == 0) {
            ccTaperStartMs = now0;
            LOG("[RAMP] CC taper watch: actual %.2fA < %.2fA threshold "
                "(expected %.2fA at %dW)\n",
                actualA, threshA, expectedA, current);
          } else if ((now0 - ccTaperStartMs) >= CC_TAPER_MS) {
            ccTriggerTaper = true;
          }
        } else if (ccTaperStartMs != 0) {
          // Current recovered — abandon the watch
          LOG("[RAMP] CC taper cleared: actual %.2fA recovered above "
              "threshold after %lus\n",
              actualA, (now0 - ccTaperStartMs) / 1000UL);
          ccTaperStartMs = 0;
        }
      } else {
        ccTaperStartMs = 0;  // still ramping — don't start the timer
      }
    } else {
      ccTaperStartMs = 0;
    }

    // (c) **VCB plateau trigger** — voltage-cutback specific saturation check.
    //     Current-taper (b) can't detect the case where VcbON is limiting power
    //     and the pack is accepting all VCB-allowed current but still can't rise
    //     to the voltage ceiling.  Here we use *voltage stability* instead:
    //     when the voltage cutback table is the active limiter (not just thermal)
    //     and the pack is measurably below the ceiling, a ±0.5 V window stable
    //     for 5 min means the pack has physically saturated at the VCB knee.
    bool voltCutbackActive = (phase == PHASE_CC) &&
                             (voltPwrW != UINT32_MAX) &&
                             (voltPwrW < (uint32_t)target);
    bool ccTriggerPlateau = false;
    if (voltCutbackActive && rawPackDv > 0 && rawPackDv < (long)voltCeiling) {
      if (voltPlateauStartMs == 0) {
        // First tick with conditions met — start the plateau watch
        voltPlateauStartMs = now0;
        plateauRefDv       = rawPackDv;
        LOG("[RAMP] VCB plateau watch started: pack %ld dV, "
            "cutback→%u W, ceil %d dV\n",
            rawPackDv, (unsigned)effectiveTarget, voltCeiling);
      } else if (labs(rawPackDv - plateauRefDv) > VCB_PLATEAU_BAND_DV) {
        // Voltage shifted outside the ±0.5 V band — rebase the timer
        plateauRefDv       = rawPackDv;
        voltPlateauStartMs = now0;
      } else if ((now0 - voltPlateauStartMs) >= VCB_PLATEAU_MS) {
        ccTriggerPlateau = true;
      }
    } else {
      if (voltPlateauStartMs != 0) {
        LOG("[RAMP] VCB plateau watch cancelled (conditions no longer met)\n");
      }
      voltPlateauStartMs = 0;
      plateauRefDv       = 0;
    }

    if (ccTriggerVoltage || ccTriggerTaper || ccTriggerPlateau) {
      phase = PHASE_CV;
      cvMs  = millis();
      // CV amps limit, as a TOTAL across all chargers (STAB-6). This is a LIMIT
      // the Elcon regulates under, not a demand: in CV the charger holds voltage
      // and the pack draws whatever it accepts, so a looser figure here does not
      // force current in.
      //
      // Latched as a total, not per-charger, and deliberately so: the value is
      // captured once at this transition but the command is rebuilt from it on
      // every CV tick, where the live divisor and the 1 C ceiling are applied.
      // Latching a per-charger figure meant the transition-time charger count
      // was frozen into an absorption phase that can last an hour, and the
      // write bypassed the 1 C clamp entirely.
      //
      // NOTE (audit 2026-07): the max() below makes C/5 a FLOOR, not a cap — the
      // earlier comment here said "cap at C/5 per charger", which the code has
      // never done. Whichever of ccAmpsTotal / cvMaxTotal is LARGER wins, so
      // entering CV from a tapering CC can raise the limit (e.g. CC tapering at
      // 6 A total → CV limit 22.8 A total on a 114 Ah pack).
      // Deliberately left as-is: changing it to min() would tighten the CV
      // current limit and could shift absorption duration and when the
      // CV→DONE taper fires, which is tuned by bench observation. The absolute
      // 1.0 C ceiling applied at command time backstops it either way.
      // Revisit together with CV tuning, not in isolation.
      long   clampedDv    = min(rawPackDv, (long)voltCeiling);
      float  packV        = clampedDv / 10.0f;
      float  ccAmpsTotal  = (current > 0 && packV > 0) ? (current / packV) : 0.0f;
      float  cvMaxTotal   = packAH * 0.2f;   // C/5 of the (clamped) pack Ah
      float  cvTotalDaF   = max(ccAmpsTotal, cvMaxTotal) * 10.0f;
      if (cvTotalDaF > 65535.0f) cvTotalDaF = 65535.0f;
      cvAmpsTotalDa = (uint16_t)cvTotalDaF;
      // CV target voltage:
      //   - voltage trigger: pack reached the ceiling → hold at the ceiling
      //   - taper / plateau:  pack saturated below ceiling under VCB →
      //                       hold at the achievable plateau voltage so the
      //                       load-sag detector doesn't immediately fire and
      //                       the chargers don't fight an unreachable target.
      if (ccTriggerVoltage) {
        cvTargetDv = voltCeiling;
      } else {
        // clampedDv == min(rawPackDv, voltCeiling); under VCB rawPackDv < ceiling
        cvTargetDv = (uint16_t)clampedDv;
      }
      if (ccTriggerVoltage) {
        LOG("[RAMP] CC→CV (voltage) @ %ld dV (ceil %d dV), "
            "cvTarget=%u dV, cvAmpsTotalDa=%d\n",
            rawPackDv, voltCeiling, (unsigned)cvTargetDv, (int)cvAmpsTotalDa);
      } else if (ccTriggerTaper) {
        LOG("[RAMP] CC→CV (current taper): %lus of %.2fA actual at %dW "
            "commanded, pack at %ld dV (ceil %d dV), "
            "cvTarget=%u dV, cvAmpsTotalDa=%d\n",
            (now0 - ccTaperStartMs) / 1000UL,
            actualA, current, rawPackDv, voltCeiling,
            (unsigned)cvTargetDv, (int)cvAmpsTotalDa);
      } else {
        LOG("[RAMP] CC→CV (VCB plateau): pack %ld dV stable %lus "
            "(ceil %d dV, cutback→%u W), cvTarget=%u dV, cvAmpsTotalDa=%d\n",
            rawPackDv, (now0 - voltPlateauStartMs) / 1000UL,
            voltCeiling, (unsigned)effectiveTarget,
            (unsigned)cvTargetDv, (int)cvAmpsTotalDa);
      }
      ccTaperStartMs     = 0;
      voltPlateauStartMs = 0;
      plateauRefDv       = 0;
    }

    // ── Cycle data logger — phase-entry hooks ─────────────────────────────────
    // CC entry: fire when transitioning into CC from any other phase, OR on the
    // tick the start evaluation ran while already in CC (covers disable/re-enable
    // without a phase change that would update lastPhase).
    //
    // Uses startEvalRan, not justEnabled: lastPhase is not updated on ticks that
    // return early, so a disable/re-enable cycle that stayed in PHASE_CC relies
    // entirely on the enable edge here — and justEnabled could be consumed by an
    // early return before reaching this point, leaving the new cycle's record
    // carrying the previous cycle's start voltage, SoC and temperature.
    //
    // D15 — what "a cycle" means, and it is narrower than "a plug-in": this
    // hook re-fires, and therefore starts a NEW record, on every fresh entry
    // into CC. A CV→CC load-sag excursion, a Stop followed by Charge, and a
    // top-up after the pack has sagged out of Float all reset the record in
    // progress, energy counters included. One plug-in can therefore produce
    // several rows, each covering one bulk→absorption→float leg, and the
    // partial leg that a sag excursion interrupts is simply discarded rather
    // than written. This is deliberate — a leg is the unit the charge
    // algorithm actually reasons about — but it means cycles.csv rows must not
    // be summed to get "energy delivered per plug-in". Documented for users in
    // the manual's cycle-log section.
    if ((phase == PHASE_CC && lastPhase != PHASE_CC) ||
        (startEvalRan && phase == PHASE_CC)) {
      ccEntryMs = millis();
      // Whole-struct value-init: this is what stops a field that is only
      // written at a later transition — bulk_min (CC→CV) and abort_reason
      // (CV→DONE) — from leaking the previous cycle's value into this one
      // (STAB-12). cvEntryMs lives outside the struct, so it has to be cleared
      // by hand here; leaving it set made absorption_min on a cycle that never
      // reached CV count from the *previous* cycle's CV entry.
      g_cycle   = CycleRecord{};
      cvEntryMs = 0;
      // D3: this cycle's energy counters start from zero here, at the same
      // instant as start_v_dv / start_soc, so what they total at DONE covers
      // exactly the span the record's start_* and end_* fields bracket.
      cycleWh   = 0.0f;
      cycleAh   = 0.0f;
      struct tm ti;
      if (getLocalTime(&ti, 100))
        strftime(g_cycle.timestamp, sizeof(g_cycle.timestamp),
                 "%Y-%m-%dT%H:%M:%S", &ti);
      if (xSemaphoreTake(liveMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_cycle.start_v_dv = (uint16_t)live.monolithVoltageDv;
        g_cycle.start_soc  = live.monolithBmsSoc;   // 255 if no BMS frame yet
        g_cycle.start_temp = cycleTempI8(live.monolithMaxTemp);
        xSemaphoreGive(liveMutex);
      }
      // Match target voltage to the nearest named preset percentage
      uint8_t ppct = 0;
      for (int pi = 0; pi < TARGET_VOLT_PRESET_COUNT; pi++)
        if (TARGET_VOLT_PRESETS[pi].dv == targetVolt) { ppct = TARGET_VOLT_PRESETS[pi].pct; break; }
      g_cycle.preset_pct    = ppct;
      g_cycle.charger_count = nChargers;
    }
    // CV entry: transition from CC to CV
    if (phase == PHASE_CV && lastPhase == PHASE_CC) {
      cvEntryMs         = millis();
      g_cycle.bulk_min  = (uint16_t)((cvEntryMs - ccEntryMs) / 60000UL);
    }

    // ── Ramp step ─────────────────────────────────────────────────────────────
    // effectiveTarget and cutbackActive already computed above (before taper check).
    //
    // L3: a cutback table entry of 0 C-rate is a *stop*, not a ramp target. The
    // HOT_CUTBACK table's last entry (75 °C) is c_rate 0, which yields
    // effectiveTarget == 0 — but the normal ramp walks `current` down at
    // rampStep W/s, so at the 100 W/s default it took ~33 s to actually stop
    // from 3300 W while the pack was already 30 °C past the cell's 45 °C
    // charging limit. Zero immediately instead. The chargers see cmdStart=false
    // on this same tick because cmdStart is gated on (current > 0).
    //
    // `target > 0` is deliberate: it distinguishes "a cutback table forced the
    // ceiling to zero" (a protection event — stop now) from "the user dragged
    // the power slider to 0" (an ordinary setpoint change, which should still
    // ramp down smoothly). Without it, moving the slider to 0 would trigger an
    // abrupt stop and a scary HARD CUTOFF log line.
    bool hardCutoff = (phase == PHASE_CC) && (target > 0) &&
                      (effectiveTarget == 0) && (current > 0);
    if (hardCutoff) {
      LOG("[RAMP] HARD CUTOFF: cutback table returned 0 C-rate "
          "(hot %d°C / cold %d°C / pack %lddV) — commanding STOP immediately, "
          "bypassing %dW/s ramp-down from %dW\n",
          (int)hotTemp, (int)coldTemp, rawPackDv, rampStep, current);
      current = 0;
    } else if (current < effectiveTarget) {
      current = (uint16_t)min((uint32_t)(current + rampStep), (uint32_t)effectiveTarget);
    } else if (current > effectiveTarget) {
      uint16_t dn = cutbackActive ? min((uint16_t)(rampStep * 2), (uint16_t)500) : rampStep;
      current = (uint16_t)max((int32_t)(current - dn), (int32_t)effectiveTarget);
    }

    if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      ctrl.currentPowerW = current;
      xSemaphoreGive(controlMutex);
    }

    // ── Amp/voltage calc for CC phase ─────────────────────────────────────────
    // Clamp packDv to ceiling for the I = P/V division.
    long packDv = rawPackDv;
    if (packDv > (long)voltCeiling) packDv = voltCeiling;

    // cmdVoltDv = ceiling (CC/CV ceiling), not current pack voltage.
    uint16_t cmdVoltDv = voltCeiling;

    uint16_t cmdAmpsDa  = 0;
    float    totalAmpsA = 0.0f;
    // Recomputed every tick so the dashboard banner self-clears the moment the
    // ceiling stops binding.
    g_currentClamped = false;
    if (packDv > 0 && current > 0) {
      float v    = packDv / 10.0f;
      totalAmpsA = current / v;

      // ── Absolute current ceiling (H1) ────────────────────────────────────
      // Independent of the cutback tables, which are a shaping curve rather
      // than a hard limit (they permit up to 3.0 C; the cell is rated 1.0 C).
      // The Farasis IMP06160230P25A allows 25 A max charge on a 25 Ah cell =
      // 1.0 C, and because packAH scales with the parallel count, 1.0 C of the
      // BMS-reported pack Ah is the right total ceiling whatever P happens to
      // be. Applied to the TOTAL before dividing per charger.
      float maxTotalA = CELL_MAX_CHARGE_C * packAH;
      if (totalAmpsA > maxTotalA) {
        g_currentClamped = true;
        static unsigned long lastAmpClampLog = 0;
        if (lastAmpClampLog == 0 || now0 - lastAmpClampLog >= 10000UL) {
          lastAmpClampLog = now0;
          LOG("[RAMP] Total current clamped %.1fA -> %.1fA (%.2fC ceiling on "
              "%.0fAh pack)\n", totalAmpsA, maxTotalA, CELL_MAX_CHARGE_C, packAH);
        }
        totalAmpsA = maxTotalA;
      }

      // Per-charger share. `divisor` is max(configured, detected) — see its
      // derivation above; using the configured count alone let an unannounced
      // extra unit multiply the delivered current. The uint16_t cast is
      // unreachable-by-overflow: maxTotalA is bounded by pack Ah (PACK_AH_MAX
      // worst case), so perCh * 10 cannot approach 65535. The explicit clamp
      // documents that invariant rather than relying on it.
      float perChDa = (totalAmpsA / divisor) * 10.0f;
      if (perChDa > 6553.0f) perChDa = 6553.0f;   // uint16_t dA headroom
      cmdAmpsDa = (uint16_t)max(1.0f, perChDa);
    }

    // Session energy (CC phase) — accumulate using measured charger output,
    // not the commanded power. The commanded value keeps growing as the
    // ramp catches up, but if the chargers are saturating (e.g. pack at
    // its physical voltage limit, charger AC-input-limited, etc.) the
    // actually delivered current can be much lower or even zero. Bug 3 in
    // [project_charging_bugs_2026-04-26.md] was the dashboard's session
    // counters continuing to climb at the commanded rate while monolith
    // and chargers both reported 0 A — fixed here by mirroring the CV
    // branch's "actualA × packV" accounting.
    if (phase == PHASE_CC && packDv > 0 && actualA > 0.0f) {
      float actualW  = actualA * (packDv / 10.0f);
      float whDelta  = actualW / 3600.0f;
      float ahDelta  = actualA / 3600.0f;
      sessionAddCC(whDelta, ahDelta);
      // D3: the cycle record's own counters take the same increment. Kept
      // adjacent to the sessionAddCC call deliberately — the two must never
      // drift apart, and the only way to guarantee that is for the same two
      // values to feed both.
      cycleWh += whDelta;
      cycleAh += ahDelta;
    }

    // ── Charger bus update + CV phase management ──────────────────────────────
    // Everything already known to force a stop is asserted BEFORE the mutex is
    // taken, so a failed acquisition cannot drop it (STAB-2). A fault first
    // detected inside the critical section asserts it there instead — still
    // ahead of the command write it guards.
    //
    // `intendStop` is the WHOLE stop intent for this tick, computed here rather
    // than piecemeal inside the critical section (audit 2026-09). It used to be
    // just fault|mismatch, which left three ways for a tick that had decided not
    // to charge to end with the previous START still on the wire: PHASE_DONE, a
    // pack voltage of 0 (no telemetry), and CC with the ramp at 0 W all produce
    // a zeroed command INSIDE the mutex, so if the mutex take failed on that
    // exact tick nothing was written and nothing had raised the failsafe either.
    // Raising it up here means the intent survives a lost mutex; the timeout
    // fallback below now keys off the same flag.
    bool intendStop = g_chargerFault || chargerCountMismatch ||
                      (phase == PHASE_DONE) || (packDv <= 0) ||
                      (phase == PHASE_CC && current == 0);
    if (intendStop) g_forceStop = true;

    // What this tick actually wrote into chargerBus, for the status log at the
    // bottom of the tick. The log used to print the locally COMPUTED cmdVoltDv /
    // cmdAmpsDa, which are what we would have sent had nothing blocked us — so a
    // tick that wrote a zeroed STOP (ccStart false) or lost the mutex entirely
    // still printed a confident "cmd 1100dV/95dA" line. Defaults describe the
    // lost-mutex case: nothing written at all.
    bool     cmdWritten  = false;
    bool     wroteStart  = false;
    uint16_t wroteVoltDv = 0;
    uint16_t wroteAmpsDa = 0;

    if (xSemaphoreTake(chargerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      cmdWritten = true;
      bool chargersPresent = (chargerBus.chargerCount > 0);

      // ── Charger self-reported fault scan (STAB-4) ──────────────────────────
      // The status bitfield (byte 4 of 0x18FF50Ex — see the bit table in the
      // charger-protocol comment block at the top of this file) has been decoded
      // and displayed since the first version, but was long not acted on: a unit
      // could report a hardware failure or an overtemperature and we carried on
      // commanding it at full current.
      //
      // MASK IS 0x03 — hardware failure | over-temperature — and nothing else
      // (audit 2026-09; it was 0x1B). Per the Elcon TC protocol document the two
      // bits that used to be in the mask are not faults at all:
      //   * 0x10 is "communication receive timeout", which the charger asserts
      //     BY DESIGN after ~5 s without a command frame. Every idle unit on the
      //     bus has it set, so the old mask latched a charger fault against
      //     perfectly healthy hardware as soon as a scan ran.
      //   * 0x08 is "battery not detected / starting state", true for the first
      //     moments of every session before the output closes.
      // 0x04 (AC input out of range) stays excluded for the original reason: it
      // asserts transiently on any mains dip and self-clears, so latching a stop
      // on it would turn a flicker into a manual re-arm. All three remain
      // decoded and visible on the dashboard — display only, never a stop.
      //
      // Two further guards keep this from firing on noise:
      //   (a) a 3-consecutive-tick debounce (3 s at the 1 Hz tick) before the
      //       latch closes, so one corrupted status byte cannot end a charge
      //       that then needs a manual re-arm;
      //   (b) a CHARGER_FAULT_GRACE_MS window after the enable edge in which
      //       both the fault bits and the overvoltage test are ignored outright,
      //       because a just-woken Elcon is still reporting its starting state.
      //
      // Only units with a fresh frame are considered: a unit that has gone
      // quiet is handled by the CHARGER_TIMEOUT_MS presence decay, and its last
      // status word is stale by definition.
      bool     wasCommanding = chargerBus.cmdStart;
      bool     faultGraceOver = (enableEdgeMs == 0) ||
                                ((now0 - enableEdgeMs) >= CHARGER_FAULT_GRACE_MS);
      uint8_t  faultBits     = 0;
      uint16_t overVoltDv    = 0;

      // Overvoltage reference. Compare against whichever is higher, the ceiling
      // we asked for or the pack's own measured voltage, plus 6.0 V (audit
      // 2026-09; it was ceiling + 3.0 V). Two reasons the old test misfired:
      // a charger legitimately reads a little above the pack it is pushing into,
      // and the ceiling can drop below the pack voltage the instant the user
      // lowers the target — at which point a correctly-regulating unit sat 30+
      // dV "above" the new ceiling and latched a runaway fault. 60 dV is wider
      // than the widest gap between two presets (40 dV), so a target change can
      // never manufacture one.
      long ovLimit = max((long)voltCeiling, rawPackDv) + 60;  // 6.0 V
      bool ovThisTick = false;

      for (int i = 0; i < MAX_CHARGERS; i++) {
        ChargerUnit& c = chargerBus.chargers[i];
        if (!c.present || (now0 - c.lastSeenMs) >= CHARGER_TIMEOUT_MS) continue;
        if (!faultGraceOver) continue;
        faultBits |= (uint8_t)(c.status & 0x03);
        // Output well above anything we could have asked for means the unit is
        // no longer regulating to the command — a genuine runaway, not ripple.
        // Only meaningful while we are actually commanding START, and never in
        // DONE (where the command is zero and a unit winding down can still be
        // reporting its last output voltage).
        if (wasCommanding && phase != PHASE_DONE && (long)c.voltDv > ovLimit) {
          ovThisTick = true;
          overVoltDv = c.voltDv;
        }
      }

      // Per-tick debounce — both detectors must agree with themselves for
      // FAULT_DEBOUNCE_TICKS consecutive ticks before the latch closes. A clean
      // tick resets the counter, so only a sustained condition latches.
      const uint8_t FAULT_DEBOUNCE_TICKS = 3;
      static uint8_t faultTicks = 0;
      static uint8_t ovTicks    = 0;
      if (faultBits != 0) { if (faultTicks < 255) faultTicks++; } else faultTicks = 0;
      if (ovThisTick)     { if (ovTicks    < 255) ovTicks++;    } else { ovTicks = 0; overVoltDv = 0; }

      bool faultLatch = (faultTicks >= FAULT_DEBOUNCE_TICKS);
      bool ovLatch    = (ovTicks    >= FAULT_DEBOUNCE_TICKS);
      if (faultLatch || ovLatch) {
        g_chargerFault = true;   // latched until the next enable edge
        static unsigned long lastChargerFaultLogMs = 0;
        if (lastChargerFaultLogMs == 0 || now0 - lastChargerFaultLogMs >= 10000UL) {
          lastChargerFaultLogMs = now0;
          if (faultLatch)
            LOG("[RAMP] Charger fault — status bits 0x%02X "
                "(0x01 hardware failure / 0x02 over-temperature) for %u ticks. "
                "STOP; switch charging off and on again to re-arm\n",
                (unsigned)faultBits, (unsigned)faultTicks);
          if (ovLatch)
            LOG("[RAMP] Charger overvoltage — unit reports %u dV against a "
                "%ld dV limit (ceil %d dV, pack %ld dV) for %u ticks. STOP; "
                "switch charging off and on again to re-arm\n",
                (unsigned)overVoltDv, ovLimit, voltCeiling, rawPackDv,
                (unsigned)ovTicks);
        }
      }

      // Any reason this tick must not command START. `intendStop` was computed
      // before the mutex was taken and already covers DONE / no pack voltage /
      // zero commanded power; OR in the fault latch, which may have closed only
      // a few lines above (inside this critical section) and so could not have
      // been part of it. g_forceStop makes the STOP unconditional in
      // sendHeartbeat() even if this very mutex write is the one that gets lost
      // next tick.
      bool stopNow = intendStop || g_chargerFault;
      if (stopNow) g_forceStop = true;

      if (phase == PHASE_CV) {
        // Use the same `actualA` snapshot taken at the top of this tick — no
        // need to re-scan the charger array under the mutex (the value can't
        // have changed by more than one MCP frame and we re-check next tick).
        unsigned long now     = millis();
        unsigned long elapsed = now - cvMs;

        // Termination check (give at least CV_MIN_MS to settle)
        bool terminating = false;
        if (packDv > 0) {
          bool termCurrent = (elapsed >= CV_MIN_MS) && (actualA < CV_TERM_A);
          bool termTimeout = (elapsed >= CV_HOLD_MS);
          if (termCurrent || termTimeout) {
            phase = PHASE_DONE;
            g_cycle.abort_reason = termTimeout ? 1 : 0;  // 0=taper, 1=timeout
            terminating = true;
            LOG("[RAMP] CV→DONE: %s after %lus (%.2fA actual)\n",
                termTimeout ? "timeout" : "current taper",
                elapsed / 1000UL, actualA);
          }
        }
        if (!terminating && packDv > 0 && actualA > 0.0f) {
          float cvPwrW  = actualA * (packDv / 10.0f);
          float whDelta = cvPwrW  / 3600.0f;
          float ahDelta = actualA / 3600.0f;
          sessionAddCC(whDelta, ahDelta);
          cycleWh += whDelta;   // D3 — same increment, same values (see CC arm)
          cycleAh += ahDelta;
        }

        // Keep charger alive in CV mode. Command cvTargetDv (the achievable
        // plateau for taper/plateau-triggered CV, or the ceiling for voltage-
        // triggered CV) — NOT voltCeiling unconditionally. Commanding an
        // unreachable voltage and immediately bouncing back to CC is what the
        // 100 %-preset oscillation looked like prior to v202605171800.
        if (phase == PHASE_CV && chargersPresent && packDv > 0 && !stopNow) {
          // Rebuild the per-charger CV limit from the latched TOTAL every tick
          // (STAB-6). cvAmpsTotalDa was captured at the CC→CV transition; the
          // divisor and the 1 C ceiling are applied HERE so that a charger
          // appearing mid-absorption, or a pack Ah that only arrived after the
          // transition, are both honoured. The old code wrote the latched
          // per-charger figure straight out, bypassing the 1 C clamp entirely.
          float cvTotalA    = cvAmpsTotalDa / 10.0f;
          float cvMaxTotalA = CELL_MAX_CHARGE_C * packAH;
          if (cvTotalA > cvMaxTotalA) cvTotalA = cvMaxTotalA;
          float cvPerChDa = (cvTotalA / divisor) * 10.0f;
          if (cvPerChDa > 6553.0f) cvPerChDa = 6553.0f;  // uint16_t dA headroom
          chargerBus.cmdVoltDv = (cvTargetDv > 0) ? cvTargetDv : voltCeiling;
          chargerBus.cmdAmpsDa = (uint16_t)max(10.0f, cvPerChDa); // floor 1 A/ch
          chargerBus.cmdStart  = true;
          wroteStart  = true;
          wroteVoltDv = chargerBus.cmdVoltDv;
          wroteAmpsDa = chargerBus.cmdAmpsDa;
          // A valid START landed under the mutex — but never lower the failsafe
          // during a shutdown wait (stopChargerForRestart), or this tick would
          // hand the chargers a START on the way to a reboot.
          if (!g_shuttingDown) g_forceStop = false;
        } else {
          // Just transitioned to DONE this tick, chargers absent, no pack
          // voltage, or a fault/mismatch forced the stop — send STOP either way.
          // Raise the failsafe alongside the write: this arm is a stop decision
          // just as much as the pre-mutex ones are, and if the NEXT tick loses
          // the mutex the flag is all that keeps the STOP on the wire.
          g_forceStop = true;
          chargerBus.cmdVoltDv = 0;
          chargerBus.cmdAmpsDa = 0;
          chargerBus.cmdStart  = false;
          // Clear CV target so a future cycle starts fresh
          if (phase == PHASE_DONE) cvTargetDv = 0;
        }
      } else {
        // CC phase: start when current > 0 and pack is known; DONE: current == 0 stops it
        bool ccStart = (current > 0) && chargersPresent && (packDv > 0) && !stopNow;
        chargerBus.cmdVoltDv = ccStart ? cmdVoltDv : 0;
        chargerBus.cmdAmpsDa = ccStart ? cmdAmpsDa : 0;
        chargerBus.cmdStart  = ccStart;
        wroteStart  = ccStart;
        wroteVoltDv = chargerBus.cmdVoltDv;
        wroteAmpsDa = chargerBus.cmdAmpsDa;
        if (ccStart) {
          // See the CV arm — g_shuttingDown wins over a fresh START.
          if (!g_shuttingDown) g_forceStop = false;
        } else {
          // Zeroed command written: same reasoning as the CV else-arm above.
          g_forceStop = true;
        }
      }

      xSemaphoreGive(chargerMutex);
    } else if (intendStop) {
      // The command update is the one chargerMutex write that can also be a
      // STOP decision. If it was lost AND this tick had decided to stop, the
      // failsafe has to stand on its own. `intendStop` (not fault|mismatch) is
      // the test because it carries the full stop intent — DONE, no pack
      // voltage and a zero ramp all end in a zeroed write we never got to make.
      //
      // Note: with the mutex lost the fault scan above did not run this tick, so
      // a NEW charger fault goes undetected until the next tick that gets the
      // lock. That is accepted: detection is deferred, the stop is not — the
      // failsafe below puts a STOP on the wire within one heartbeat regardless.
      g_forceStop = true;
      if (lastStopMutexLogMs == 0 || now0 - lastStopMutexLogMs >= 5000UL) {
        lastStopMutexLogMs = now0;
        LOG("[RAMP] chargerMutex timeout on STOP — forceStop asserted\n");
      }
    }

    // ── Cycle data logger — DONE entry hook ───────────────────────────────────
    // Fires in the same tick that CV→DONE was set (abort_reason already set
    // inside the charger mutex block above). Snapshots end-of-cycle values and
    // writes one CSV row to /cycles.csv.
    //
    // STAB-12: only a CV→DONE entry is a completed charge cycle. DONE is also
    // entered straight from CC — the above-target backstop and the start
    // evaluation's "nothing to charge" path — and those never ran an absorption
    // phase. Recording them produced a row whose absorption_min was measured
    // from the *previous* cycle's cvEntryMs, with that cycle's bulk_min and
    // abort_reason alongside it: fabricated training data, not a short cycle.
    // Skip the append and clear cvEntryMs so nothing downstream can reuse it.
    if (phase == PHASE_DONE && lastPhase != PHASE_DONE) {
      if (lastPhase != PHASE_CV) {
        cvEntryMs = 0;
        LOG("[AI] DONE without absorption — no cycle record\n");
      } else {
        if (xSemaphoreTake(liveMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          g_cycle.end_v_dv = (uint16_t)live.monolithVoltageDv;
          g_cycle.end_soc  = live.monolithBmsSoc;
          g_cycle.end_temp = cycleTempI8(live.monolithMaxTemp);
          xSemaphoreGive(liveMutex);
        }
        // STAB-11 / D3: this cycle's own accumulators, totalled tick by tick
        // since CC entry. Nothing is subtracted and nothing is read from the
        // session counters, so a Reset Session anywhere inside the cycle
        // leaves these figures untouched. Both are non-negative by
        // construction (every increment is a positive delta gated on
        // actualA > 0), so the casts need no clamp.
        g_cycle.total_ah_x100 = (uint32_t)(cycleAh * 100.0f);
        g_cycle.total_wh_x10  = (uint32_t)(cycleWh * 10.0f);
        g_cycle.absorption_min = (cvEntryMs > 0)
          ? (uint16_t)((millis() - cvEntryMs) / 60000UL)
          : 0;
        appendCycleRecord(g_cycle);
        cvEntryMs = 0;      // consumed — the next cycle sets its own
      }
    }

    // Track previous phase for next tick's transition hooks.
    // NOTE: updated AFTER all hooks so each hook sees the true transition.
    lastPhase = phase;

    // Expose phase to API/dashboard (atomic uint8_t write, no mutex needed)
    g_rampPhase = (uint8_t)phase;

    // ── Time-to-target estimate ───────────────────────────────────────────────
    // Mirrors the "time remaining" timer the bike's dashboard shows during
    // charging, but extrapolates toward the user's chosen target voltage
    // (e.g. 70 / 80 / 90 / 100 % preset) instead of always 100 %.
    //
    // Bulk (CC):    coulomb estimate using the present pack current —
    //               minutes = (target_SoC - now_SoC)/100 * monolithAH * 60 / ampsNow
    //               where ampsNow is the magnitude of the current flowing into
    //               the pack (positive while charging).  Suppressed while
    //               current draw is below ~5 A (still ramping or settling) so
    //               the early figure doesn't read in the days.
    // Absorption:   we don't know how long until the cells equalize, but the
    //               CV_HOLD_MS safety timer is the upper bound — show the
    //               remainder of that countdown so the dashboard never says
    //               "0 min" while the charger is still on.
    // Float / off:  -1 sentinel → dashboard shows "—".
    //
    // EMA smoothing (α = 0.2) damps tick-to-tick jitter from current jumps.
    static float etaEmaMin = -1.0f;
    int16_t etaOut = -1;

    if (phase == PHASE_CC) {
      // Target SoC is voltage-derived (user picks a voltage preset). Current
      // SoC: prefer the BMS's coulomb-counted value; fall back to the voltage
      // curve only until the first 0x188 frame arrives.
      byte bmsSoc;
      if (xSemaphoreTake(liveMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        bmsSoc = live.monolithBmsSoc;
        xSemaphoreGive(liveMutex);
      } else {
        bmsSoc = 255;
      }
      int   targetSoc = calcSocFromVoltage((long)voltCeiling);
      int   nowSoc    = (bmsSoc <= 100)        ? (int)bmsSoc
                       : (rawPackDv > 0)       ? calcSocFromVoltage(rawPackDv)
                                               : 0;
      // packAH, not the raw monolithAH: packAH is the same figure re-clamped to
      // PACK_AH_MAX just above, and it is what every other capacity calculation
      // in this tick uses. Reading monolithAH here meant one absurd 0x288 frame
      // that slipped the ingestion gate could stretch the ETA to the 24 h cap
      // while the current ceiling and cutbacks were all working off the clamped
      // value — two different pack sizes inside one tick.
      float ahNeeded  = (float)(targetSoc - nowSoc) / 100.0f * packAH;
      // Use the same actualA snapshot taken at the top of the tick — already
      // includes only chargers with a fresh frame within CHARGER_TIMEOUT_MS.
      float ampsNow   = actualA;
      // Sanity floor — below ~2 A into a 100 Ah pack the ETA blows up.
      if (ahNeeded > 0.0f && ampsNow > 2.0f) {
        float minutesRaw = ahNeeded / ampsNow * 60.0f;
        if (minutesRaw > 24.0f * 60.0f) minutesRaw = 24.0f * 60.0f;
        etaEmaMin = (etaEmaMin < 0.0f) ? minutesRaw
                                       : (etaEmaMin * 0.8f + minutesRaw * 0.2f);
        etaOut = (int16_t)(etaEmaMin + 0.5f);
      } else {
        etaEmaMin = -1.0f; // not enough current to estimate yet
      }
    } else if (phase == PHASE_CV) {
      unsigned long elapsed = millis() - cvMs;
      long remMs = (long)CV_HOLD_MS - (long)elapsed;
      if (remMs < 0) remMs = 0;
      etaOut = (int16_t)(remMs / 60000L);
      etaEmaMin = -1.0f; // reset bulk EMA so a CV→CC sag restarts cleanly
    } else {
      etaEmaMin = -1.0f;
      etaOut = -1; // float / done / disabled
    }
    g_etaMinutes = etaOut;

    // ── Log ───────────────────────────────────────────────────────────────────
    const char* ps = (phase==PHASE_CC) ? "BULK"
                  : (phase==PHASE_CV) ? "ABSORPTION"
                                      : "FLOAT";
    if (phase == PHASE_CV) {
      // Commanded voltage in CV is cvTargetDv, NOT voltCeiling — this line used
      // to print the ceiling twice and label the latched amps "/ch" when they
      // are now a total. cvAmpsTotalDa is the LATCHED total; the per-charger
      // figure actually written is that divided by `divisor` and 1 C-clamped.
      LOG("[RAMP] %s +%lus | cmd %ddV, limit %ddA total /%u ch | raw %lddV ceil %ddV\n",
          ps, (millis()-cvMs)/1000UL,
          (int)((cvTargetDv > 0) ? cvTargetDv : voltCeiling),
          (int)cvAmpsTotalDa, (unsigned)divisor, rawPackDv, voltCeiling);
    } else if (current > 0 || phase == PHASE_DONE) {
      // COLD_CUTBACK returns a value at nearly any temperature (table runs up
      // to 40 °C), so flag it only when it actually limits power below target —
      // unlike HOT, where a non-MAX result already means "genuinely hot".
      //
      // "cmd" is what was actually WRITTEN to chargerBus this tick, not the
      // locally computed cmdVoltDv/cmdAmpsDa: a tick that decided to stop wrote
      // zeros, and a tick that lost chargerMutex wrote nothing at all. Printing
      // the computed values made a STOP tick read exactly like a charging one.
      char cmdStr[40];
      if (!cmdWritten)
        snprintf(cmdStr, sizeof(cmdStr), "(mutex lost, not written)");
      else if (!wroteStart)
        snprintf(cmdStr, sizeof(cmdStr), "STOP 0dV/0dA");
      else
        snprintf(cmdStr, sizeof(cmdStr), "%udV/%udA",
                 (unsigned)wroteVoltDv, (unsigned)wroteAmpsDa);
      LOG("[RAMP] %s %dW(eff) tgt%dW cmd %s raw %lddV%s%s%s\n",
          ps, current, target, cmdStr, rawPackDv,
          voltCbK != UINT32_MAX ? " VcbON" : "",
          hotCbK  != UINT32_MAX ? " HOT"   : "",
          (coldPwrW != UINT32_MAX && coldPwrW < (uint32_t)target) ? " COLD" : "");
    }
  }
}

// Called once when home WiFi (STA) first connects successfully. Overrides the
// AP/road boot defaults that rampInit() loaded with the home WiFi profile.
//
// Guarded by homeDefaultsApplied, which is latched by three things (B3):
//   1. this function itself, so mid-session reconnects can't re-apply;
//   2. any user command that changes enabled / target power / target voltage —
//      /api/control and the MQTT command handler — because once the owner has
//      touched the charge, the profile must not overrule them;
//   3. onStaUp() when it finds a charge already running.
// Between them, the profile can only ever land on an untouched, idle charger,
// which is the only situation it was designed for. The caller (onStaUp) also
// checks ctrl.enabled before calling, so this function does not re-check.
void applyHomeWifiBootDefaults() {
  if (homeDefaultsApplied) return;
  homeDefaultsApplied = true;

  if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    ctrl.enabled = ctrl.homeDefaultEnabled;

    uint8_t cc   = ctrl.chargerCount;
    uint8_t pidx = ctrl.homeDefaultPowerPresetIdx;
    ctrl.targetPowerW = (cc >= 1 && cc <= 4)
                        ? POWER_PRESETS[cc - 1][pidx]
                        : POWER_PRESETS[0][0];

    ctrl.targetVoltDv = ctrl.homeDefaultTargetVoltDv;
    xSemaphoreGive(controlMutex);
  }

  LOG("[BOOT] Home WiFi up — home defaults applied: chg_en=%d pwr=%u W tgt=%u dV\n",
      (int)ctrl.homeDefaultEnabled,
      (unsigned)ctrl.targetPowerW,
      (unsigned)ctrl.homeDefaultTargetVoltDv);
}

// Called from setup() — creates controlMutex and launches ramp task on Core 1
void rampInit() {
  controlMutex = xSemaphoreCreateMutex();
  if (controlMutex == nullptr) {
    LOG("[RAMP] Failed to create mutex — halting\n");
    while (true) { delay(1000); }
  }

  // Session counters are touched from rampTask, /api/control, and mqttCallback —
  // create the mutex before any of those tasks start.
  sessionMutex = xSemaphoreCreateMutex();
  if (sessionMutex == nullptr) {
    LOG("[RAMP] Failed to create session mutex — halting\n");
    while (true) { delay(1000); }
  }

  ctrl.currentPowerW = 0;

  // ---- Load boot-time defaults from NVS ----
  // These three values are user-configurable via /settings and define the
  // state the charger wakes into on every fresh boot.
  {
    ctrl.defaultEnabled = preferences.getBool("def_chg_en", false);

    uint8_t pi = (uint8_t)preferences.getUChar("def_pwr_idx", 1);
    if (pi >= (uint8_t)MAX_PRESETS_PER_ROW) pi = 1;  // clamp to valid column
    ctrl.defaultPowerPresetIdx = pi;

    uint16_t defTgt80 = TARGET_VOLT_PRESETS[1].dv;    // 80% = 1100 dV
    uint16_t defTgt   = preferences.getUShort("def_tgt_dv", defTgt80);
    if (defTgt < TARGET_VOLT_PRESETS[0].dv || defTgt > MAX_CHARGE_VOLTAGE_DV)
      defTgt = defTgt80;
    ctrl.defaultTargetVoltDv = defTgt;

    // Home WiFi profile — same three settings, different NVS keys.
    // Applied once in applyHomeWifiBootDefaults() when STA first connects.
    ctrl.homeDefaultEnabled = preferences.getBool("home_chg_en", false);

    uint8_t hpi = (uint8_t)preferences.getUChar("home_pwr_idx", 1);
    if (hpi >= (uint8_t)MAX_PRESETS_PER_ROW) hpi = 1;
    ctrl.homeDefaultPowerPresetIdx = hpi;

    uint16_t hTgt = preferences.getUShort("home_tgt_dv", defTgt80);
    if (hTgt < TARGET_VOLT_PRESETS[0].dv || hTgt > MAX_CHARGE_VOLTAGE_DV)
      hTgt = defTgt80;
    ctrl.homeDefaultTargetVoltDv = hTgt;

    LOG("[BOOT] AP defaults:   chg_en=%d pwr_idx=%d tgt_dv=%u\n",
        (int)ctrl.defaultEnabled, (int)ctrl.defaultPowerPresetIdx,
        (unsigned)ctrl.defaultTargetVoltDv);
    LOG("[BOOT] Home defaults: chg_en=%d pwr_idx=%d tgt_dv=%u\n",
        (int)ctrl.homeDefaultEnabled, (int)ctrl.homeDefaultPowerPresetIdx,
        (unsigned)ctrl.homeDefaultTargetVoltDv);
  }

  // ---- Apply boot defaults to active control state ----
  ctrl.enabled = ctrl.defaultEnabled;

  // Initial target power: use the configured preset column and charger count.
  // chargerCount is already restored from NVS above (default 3). If out of
  // range for any reason, fall back to the lowest preset of 1-charger row.
  {
    uint8_t cc   = ctrl.chargerCount;
    uint8_t pidx = ctrl.defaultPowerPresetIdx;
    ctrl.targetPowerW = (cc >= 1 && cc <= 4)
                        ? POWER_PRESETS[cc - 1][pidx]
                        : POWER_PRESETS[0][0];  // absolute fallback: 500 W
    LOG("[BOOT] initial target power: %u W (chargers=%d preset_idx=%d)\n",
        (unsigned)ctrl.targetPowerW, (int)cc, (int)pidx);
  }

  // Restore persisted ramp rate (default 100 W/s if never saved)
  ctrl.rampStepW = preferences.getUShort("ramp_step_w", DEFAULT_RAMP_STEP_W);
  if (ctrl.rampStepW < 10 || ctrl.rampStepW > 500) ctrl.rampStepW = DEFAULT_RAMP_STEP_W;

  // Restore persisted target voltage. Priority order:
  //   1. Last explicitly saved session value ("target_volt_dv" key)
  //   2. The user-configured boot default ("def_tgt_dv")
  //   3. Hard-coded 80% fallback
  // This means the target survives reboots but a one-time NVS clear will
  // fall back to the configured default rather than always 100%.
  {
    uint16_t savedTgt = preferences.getUShort("target_volt_dv",
                                               ctrl.defaultTargetVoltDv);
    if (savedTgt < TARGET_VOLT_PRESETS[0].dv || savedTgt > MAX_CHARGE_VOLTAGE_DV)
      savedTgt = ctrl.defaultTargetVoltDv;
    ctrl.targetVoltDv = savedTgt;
    LOG("[BOOT] target_volt_dv restored: %u dV (default %u dV)\n",
        savedTgt, ctrl.defaultTargetVoltDv);
  }

  xTaskCreatePinnedToCore(
    rampTask,         // task function
    "rampTask",       // name
    4096,             // stack bytes
    nullptr,          // parameter
    2,                // priority — above loop() (1), below WiFi stack
    &rampTaskHandle,  // handle
    1                 // Core 1 — same core as web server, avoids cross-core contention
  );

  LOG("[RAMP] Task started. Max charge voltage: %d dV (%.1f V)\n",
                MAX_CHARGE_VOLTAGE_DV, MAX_CHARGE_VOLTAGE_DV / 10.0f);
}

// ---------------------------------------------------------------------------
// System stats task — Core 1, priority 3
//
// Samples CPU load and free heap every 1 s.
//
// CPU load method:
//   FreeRTOS tracks a runtime counter per task (configGENERATE_RUN_TIME_STATS
//   must be enabled — it is by default on ESP32 Arduino).
//   We snapshot all task counters at T0 and T1 (1 s apart), find the idle
//   task for each core (named "IDLE0" / "IDLE1"), compute what fraction of
//   the elapsed runtime they consumed, and subtract from 100%.
//
//   idle_fraction = (idle_delta / total_delta) * 100
//   load = 100 - idle_fraction
//
// We allocate the TaskStatus array on the heap transiently — the array is
// freed immediately after sampling so it doesn't sit in heap permanently.
// ---------------------------------------------------------------------------

void sysStatsTask(void* /*pvParameters*/) {
  LOG("[SYS] sysStatsTask running on core %d\n", xPortGetCoreID());

  const UBaseType_t MAX_TASKS = 20;

  // Previous-tick snapshots for delta calculation
  uint32_t prevTotal  = 0;
  uint32_t prevIdle0  = 0;
  uint32_t prevIdle1  = 0;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    UBaseType_t taskCount = uxTaskGetNumberOfTasks();
    if (taskCount > MAX_TASKS) taskCount = MAX_TASKS;

    TaskStatus_t* taskList = (TaskStatus_t*)pvPortMalloc(
                                taskCount * sizeof(TaskStatus_t));
    if (!taskList) continue;

    uint32_t totalRuntime = 0;
    UBaseType_t filled = uxTaskGetSystemState(taskList, taskCount, &totalRuntime);

    uint32_t idle0 = 0, idle1 = 0;
    for (UBaseType_t i = 0; i < filled; i++) {
      const char* name = taskList[i].pcTaskName;
      if      (strcmp(name, "IDLE0") == 0) idle0 = taskList[i].ulRunTimeCounter;
      else if (strcmp(name, "IDLE1") == 0) idle1 = taskList[i].ulRunTimeCounter;
    }
    vPortFree(taskList);

    uint32_t deltaTotal = totalRuntime - prevTotal;
    uint32_t deltaIdle0 = idle0 - prevIdle0;
    uint32_t deltaIdle1 = idle1 - prevIdle1;

    prevTotal = totalRuntime;
    prevIdle0 = idle0;
    prevIdle1 = idle1;

    // Avoid division by zero on first tick (prevTotal was 0)
    if (deltaTotal == 0) continue;

    // totalRuntime is the sum across both cores, so each core's share
    // is totalRuntime/2. Clamp to 0-100 to absorb any timer skew.
    uint32_t perCoreTicks = deltaTotal / 2;
    uint8_t load0 = 0, load1 = 0;
    if (perCoreTicks > 0) {
      uint32_t idle0Pct = (deltaIdle0 * 100UL) / perCoreTicks;
      uint32_t idle1Pct = (deltaIdle1 * 100UL) / perCoreTicks;
      load0 = (uint8_t)(idle0Pct >= 100 ? 0 : 100 - idle0Pct);
      load1 = (uint8_t)(idle1Pct >= 100 ? 0 : 100 - idle1Pct);
    }

    uint32_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);

    if (xSemaphoreTake(sysStatsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      sysStats.load0    = load0;
      sysStats.load1    = load1;
      sysStats.freeHeap = freeHeap;
      xSemaphoreGive(sysStatsMutex);
    }
  }
}

void sysStatsInit() {
  sysStatsMutex = xSemaphoreCreateMutex();
  if (sysStatsMutex == nullptr) {
    LOG("[SYS] Failed to create mutex\n");
    return;
  }
  xTaskCreatePinnedToCore(
    sysStatsTask,
    "sysStatsTask",
    3072,               // modest stack — only needs temp task list on heap
    nullptr,
    3,                  // priority 3: above rampTask (2) and loop (1)
    &sysStatsTaskHandle,
    1                   // Core 1
  );
}

// ---------------------------------------------------------------------------
// MQTT task — Core 1, priority 2
//
// Owns PubSubClient exclusively. No other code touches mqttClient.
// Inbound commands are dispatched via mqttCallback which writes to ctrl
// under controlMutex — same path as the web API handler.
// ---------------------------------------------------------------------------

// Build base topic prefix into buf. Returns pointer to buf.
static char* mqttBase(char* buf, size_t len) {
  snprintf(buf, len, "supercharger/%s", mqttHostname);
  return buf;
}

// Publish a retained sensor value
static void mqttPublishSensor(const char* name, const char* value) {
  char topic[80];
  snprintf(topic, sizeof(topic), "supercharger/%s/sensor/%s", mqttHostname, name);
  mqttClient.publish(topic, value, true);
}

static void mqttPublishSensorF(const char* name, float value, int decimals) {
  char buf[20];
  dtostrf(value, 1, decimals, buf);
  mqttPublishSensor(name, buf);
}

static void mqttPublishSensorI(const char* name, int value) {
  char buf[12];
  snprintf(buf, sizeof(buf), "%d", value);
  mqttPublishSensor(name, buf);
}

// Publish HA discovery config for one sensor entity
// withAvail (default true): attach the availability (LWT) topic so Home
// Assistant marks the entity "unavailable" when the controller goes offline.
// Pass false for values that should KEEP their last reading when the controller
// powers down (e.g. State of Charge, odometer) instead of flipping to
// "unavailable" — the last value is retained on the broker and HA shows it.
// Two overloads (not a default argument — a default arg on a top-level .ino
// function can collide with the IDE's auto-generated prototype). The 6-arg form
// controls availability; the 5-arg form keeps it on, the common case.
static void mqttDiscoverSensor(
    const char* name, const char* friendlyName,
    const char* unit, const char* deviceClass,
    const char* stateClass, bool withAvail)
{
  static char topic[120];
  static char payload[512];
  static char stateTopic[80];
  static char availTopic[80];
  static char availPart[160];
  static char devId[40];
  static char dcPart[60];
  static char scPart[60];
  static char unitPart[30];

  snprintf(topic, sizeof(topic),
    "homeassistant/sensor/supercharger_%s/%s/config",
    mqttHostname, name);

  snprintf(stateTopic, sizeof(stateTopic),
    "supercharger/%s/sensor/%s", mqttHostname, name);
  snprintf(availTopic, sizeof(availTopic),
    "supercharger/%s/state", mqttHostname);
  snprintf(devId, sizeof(devId),
    "supercharger_%s", mqttHostname);

  availPart[0] = '\0';
  if (withAvail)
    snprintf(availPart, sizeof(availPart),
      ",\"avty_t\":\"%s\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\"",
      availTopic);

  dcPart[0] = '\0';
  if (deviceClass && strlen(deviceClass) > 0)
    snprintf(dcPart, sizeof(dcPart), ",\"dev_cla\":\"%s\"", deviceClass);

  scPart[0] = '\0';
  if (stateClass && strlen(stateClass) > 0)
    snprintf(scPart, sizeof(scPart), ",\"stat_cla\":\"%s\"", stateClass);

  unitPart[0] = '\0';
  if (unit && strlen(unit) > 0)
    snprintf(unitPart, sizeof(unitPart), ",\"unit_of_meas\":\"%s\"", unit);

  snprintf(payload, sizeof(payload),
    "{"
      "\"name\":\"%s\","
      "\"uniq_id\":\"sc_%s_%s\","
      "\"stat_t\":\"%s\""
      "%s"
      "%s%s%s,"
      "\"dev\":{"
        "\"ids\":[\"%s\"],"
        "\"name\":\"Supercharger\","
        "\"mdl\":\"ESP32-S3 Supercharger\","
        "\"mf\":\"DIY\""
      "}"
    "}",
    friendlyName, mqttHostname, name,
    stateTopic, availPart,
    dcPart, scPart, unitPart,
    devId
  );

  mqttClient.publish(topic, payload, true);
}

// 5-arg overload — availability on (default behavior for most sensors).
static void mqttDiscoverSensor(
    const char* name, const char* friendlyName,
    const char* unit, const char* deviceClass,
    const char* stateClass)
{
  mqttDiscoverSensor(name, friendlyName, unit, deviceClass, stateClass, true);
}

// Same as mqttDiscoverSensor but flags the entity as a diagnostic — HA groups
// it under the device's "Diagnostic" section instead of "Sensors". Also
// carries the firmware build in the device block ("sw" → sw_version): it
// merges into HA's device registry, so the device page shows the running
// firmware and updates automatically after every OTA.
static void mqttDiscoverDiagSensor(
    const char* name, const char* friendlyName,
    const char* unit, const char* deviceClass,
    const char* stateClass)
{
  static char topic[120];
  static char payload[512];
  static char stateTopic[80];
  static char availTopic[80];
  static char devId[40];
  static char dcPart[60];
  static char scPart[60];
  static char unitPart[30];

  snprintf(topic, sizeof(topic),
    "homeassistant/sensor/supercharger_%s/%s/config",
    mqttHostname, name);
  snprintf(stateTopic, sizeof(stateTopic),
    "supercharger/%s/sensor/%s", mqttHostname, name);
  snprintf(availTopic, sizeof(availTopic),
    "supercharger/%s/state", mqttHostname);
  snprintf(devId, sizeof(devId),
    "supercharger_%s", mqttHostname);

  dcPart[0] = '\0';
  if (deviceClass && strlen(deviceClass) > 0)
    snprintf(dcPart, sizeof(dcPart), ",\"dev_cla\":\"%s\"", deviceClass);

  scPart[0] = '\0';
  if (stateClass && strlen(stateClass) > 0)
    snprintf(scPart, sizeof(scPart), ",\"stat_cla\":\"%s\"", stateClass);

  unitPart[0] = '\0';
  if (unit && strlen(unit) > 0)
    snprintf(unitPart, sizeof(unitPart), ",\"unit_of_meas\":\"%s\"", unit);

  snprintf(payload, sizeof(payload),
    "{"
      "\"name\":\"%s\","
      "\"uniq_id\":\"sc_%s_%s\","
      "\"stat_t\":\"%s\","
      "\"avty_t\":\"%s\","
      "\"pl_avail\":\"online\","
      "\"pl_not_avail\":\"offline\","
      "\"ent_cat\":\"diagnostic\""
      "%s%s%s,"
      "\"dev\":{"
        "\"ids\":[\"%s\"],"
        "\"name\":\"Supercharger\","
        "\"mdl\":\"ESP32-S3 Supercharger\","
        "\"mf\":\"DIY\","
        "\"sw\":\"%lld\""
      "}"
    "}",
    friendlyName, mqttHostname, name,
    stateTopic, availTopic,
    dcPart, scPart, unitPart,
    devId, (long long)VERSION
  );

  mqttClient.publish(topic, payload, true);
}

static void mqttDiscoverNumber(
    const char* name, const char* friendlyName,
    int minVal, int maxVal, int step, const char* unit)
{
  static char topic[120];
  static char payload[512];
  static char cmdTopic[80], stateTopic[80], availTopic[80], devId[40];

  snprintf(topic,      sizeof(topic),      "homeassistant/number/supercharger_%s/%s/config", mqttHostname, name);
  snprintf(cmdTopic,   sizeof(cmdTopic),   "supercharger/%s/command/%s", mqttHostname, name);
  snprintf(stateTopic, sizeof(stateTopic), "supercharger/%s/sensor/%s",  mqttHostname, name);
  snprintf(availTopic, sizeof(availTopic), "supercharger/%s/state",       mqttHostname);
  snprintf(devId,      sizeof(devId),      "supercharger_%s",             mqttHostname);

  snprintf(payload, sizeof(payload),
    "{"
      "\"name\":\"%s\","
      "\"uniq_id\":\"sc_%s_%s\","
      "\"cmd_t\":\"%s\","
      "\"stat_t\":\"%s\","
      "\"avty_t\":\"%s\","
      "\"pl_avail\":\"online\","
      "\"pl_not_avail\":\"offline\","
      "\"min\":%d,\"max\":%d,\"step\":%d,"
      "\"unit_of_meas\":\"%s\","
      "\"dev\":{\"ids\":[\"%s\"]}"
    "}",
    friendlyName, mqttHostname, name,
    cmdTopic, stateTopic, availTopic,
    minVal, maxVal, step, unit, devId
  );

  mqttClient.publish(topic, payload, true);
}

// Float-range overload — used for target voltage (0.1 V steps)
static void mqttDiscoverNumber(
    const char* name, const char* friendlyName,
    float minVal, float maxVal, float step, const char* unit)
{
  static char topic[120];
  static char payload[512];
  static char cmdTopic[80], stateTopic[80], availTopic[80], devId[40];

  snprintf(topic,      sizeof(topic),      "homeassistant/number/supercharger_%s/%s/config", mqttHostname, name);
  snprintf(cmdTopic,   sizeof(cmdTopic),   "supercharger/%s/command/%s", mqttHostname, name);
  snprintf(stateTopic, sizeof(stateTopic), "supercharger/%s/sensor/%s",  mqttHostname, name);
  snprintf(availTopic, sizeof(availTopic), "supercharger/%s/state",       mqttHostname);
  snprintf(devId,      sizeof(devId),      "supercharger_%s",             mqttHostname);

  snprintf(payload, sizeof(payload),
    "{"
      "\"name\":\"%s\","
      "\"uniq_id\":\"sc_%s_%s\","
      "\"cmd_t\":\"%s\","
      "\"stat_t\":\"%s\","
      "\"avty_t\":\"%s\","
      "\"pl_avail\":\"online\","
      "\"pl_not_avail\":\"offline\","
      "\"min\":%.2f,\"max\":%.2f,\"step\":%.2f,"
      "\"unit_of_meas\":\"%s\","
      "\"dev\":{\"ids\":[\"%s\"]}"
    "}",
    friendlyName, mqttHostname, name,
    cmdTopic, stateTopic, availTopic,
    minVal, maxVal, step, unit, devId
  );

  mqttClient.publish(topic, payload, true);
}

static void mqttDiscoverSwitch(const char* name, const char* friendlyName) {
  static char topic[120];
  static char payload[512];
  static char cmdTopic[80], stateTopic[80], availTopic[80], devId[40];

  snprintf(topic,      sizeof(topic),      "homeassistant/switch/supercharger_%s/%s/config", mqttHostname, name);
  snprintf(cmdTopic,   sizeof(cmdTopic),   "supercharger/%s/command/%s", mqttHostname, name);
  snprintf(stateTopic, sizeof(stateTopic), "supercharger/%s/sensor/%s",  mqttHostname, name);
  snprintf(availTopic, sizeof(availTopic), "supercharger/%s/state",       mqttHostname);
  snprintf(devId,      sizeof(devId),      "supercharger_%s",             mqttHostname);

  snprintf(payload, sizeof(payload),
    "{"
      "\"name\":\"%s\","
      "\"uniq_id\":\"sc_%s_%s\","
      "\"cmd_t\":\"%s\","
      "\"stat_t\":\"%s\","
      "\"avty_t\":\"%s\","
      "\"pl_avail\":\"online\","
      "\"pl_not_avail\":\"offline\","
      "\"pl_on\":\"true\","
      "\"pl_off\":\"false\","
      "\"dev\":{\"ids\":[\"%s\"]}"
    "}",
    friendlyName, mqttHostname, name,
    cmdTopic, stateTopic, availTopic, devId
  );

  mqttClient.publish(topic, payload, true);
}

// Publish HA discovery config for a button entity (e.g. "Reset Session")
static void mqttDiscoverButton(const char* name, const char* friendlyName) {
  static char topic[120];
  static char payload[400];
  static char cmdTopic[80], availTopic[80], devId[40];

  snprintf(topic,      sizeof(topic),      "homeassistant/button/supercharger_%s/%s/config", mqttHostname, name);
  snprintf(cmdTopic,   sizeof(cmdTopic),   "supercharger/%s/command/%s", mqttHostname, name);
  snprintf(availTopic, sizeof(availTopic), "supercharger/%s/state",      mqttHostname);
  snprintf(devId,      sizeof(devId),      "supercharger_%s",            mqttHostname);

  snprintf(payload, sizeof(payload),
    "{"
      "\"name\":\"%s\","
      "\"uniq_id\":\"sc_%s_%s\","
      "\"cmd_t\":\"%s\","
      "\"pl_prs\":\"PRESS\","
      "\"avty_t\":\"%s\","
      "\"pl_avail\":\"online\","
      "\"pl_not_avail\":\"offline\","
      "\"dev\":{\"ids\":[\"%s\"]}"
    "}",
    friendlyName, mqttHostname, name,
    cmdTopic, availTopic, devId
  );

  mqttClient.publish(topic, payload, true);
}

// Publish all discovery payloads — called once after each (re)connect
static void mqttPublishDiscovery() {
  // Bike pack sensors
  mqttDiscoverSensor("monolith_v",      "Monolith Voltage",       "V",        "voltage",     "measurement");
  mqttDiscoverSensor("monolith_a",      "Monolith Current",       "A",        "current",     "measurement");
  mqttDiscoverSensor("monolith_tmin",   "Monolith Temp Min",      "\xB0""C",  "temperature", "measurement");
  mqttDiscoverSensor("monolith_tmax",   "Monolith Temp Max",      "\xB0""C",  "temperature", "measurement");
  // SoC published WITHOUT the availability topic (withAvail=false): when the
  // controller powers down HA keeps showing the last read value instead of
  // flipping to "unavailable" (the last value is retained on the broker).
  mqttDiscoverSensor("monolith_soc",    "Monolith State of Charge", "%",      "battery",     "measurement", false);
  // Available pack capacity right now: nominal AH × SoC. Tracks "how much is
  // actually left in the pack" rather than the constant nominal value.
  mqttDiscoverSensor("monolith_ah_avail","Monolith Capacity Available","Ah",   "",            "measurement");
  mqttDiscoverSensor("powertank_v",     "PowerTank Voltage",      "V",        "voltage",     "measurement");
  mqttDiscoverSensor("powertank_a",     "PowerTank Current",      "A",        "current",     "measurement");
  mqttDiscoverSensor("powertank_tmin",  "PowerTank Temp Min",     "\xB0""C",  "temperature", "measurement");
  mqttDiscoverSensor("powertank_tmax",  "PowerTank Temp Max",     "\xB0""C",  "temperature", "measurement");
  // Charging sensors
  mqttDiscoverSensor("current_power_w", "Charging Power",         "W",        "power",       "measurement");
  // Session sensors
  mqttDiscoverSensor("session_wh",      "Session Energy Delivered","Wh",      "energy",      "measurement");
  mqttDiscoverSensor("session_ah",      "Session Charge Delivered","Ah",      "",            "measurement");
  // Controls (read/write)
  mqttDiscoverNumber("target_power_w",  "Target Charging Power",  0, 13200, 100, "W");
  mqttDiscoverNumber("charger_count",   "Charger Count",          1, 4, 1, "");
  mqttDiscoverNumber("ramp_rate_wps",   "Ramp Rate",              10, 500, 10, "W");
  mqttDiscoverNumber("target_volt_v",   "Target Voltage",         106.0f, 116.4f, 0.1f, "V");
  mqttDiscoverSwitch("charging_enabled","Charging Enabled");
  // Buttons — one per voltage preset, generated from TARGET_VOLT_PRESETS so
  // adding a new preset to that table automatically exposes a new HA button.
  for (int i = 0; i < TARGET_VOLT_PRESET_COUNT; i++) {
    char btnName[20], btnFriendly[40];
    snprintf(btnName,     sizeof(btnName),     "preset_%u",
             TARGET_VOLT_PRESETS[i].pct);
    snprintf(btnFriendly, sizeof(btnFriendly), "Set %u %% (%.1f V)",
             TARGET_VOLT_PRESETS[i].pct,
             TARGET_VOLT_PRESETS[i].dv / 10.0f);
    mqttDiscoverButton(btnName, btnFriendly);
  }
  mqttDiscoverButton("reset_session",   "Reset Charge Session");
  // Active preset readout (0 if current target voltage doesn't match any preset)
  mqttDiscoverSensor("target_preset_pct", "Target Preset", "%", "", "");
  // Thermal-throttle banner state (text "true"/"false")
  mqttDiscoverSensor("thermal_throttle",  "Thermal Throttling", "", "", "");
  // Charging stage — published as "bulk" / "absorption" / "float"
  mqttDiscoverSensor("ramp_phase",        "Charging Stage", "", "", "");
  // Estimated minutes until target voltage is reached (-1 when unknown / not charging)
  mqttDiscoverSensor("eta_minutes",       "Charging Time Remaining", "min", "duration", "measurement");
  // Total charge cycles recorded in the on-device cycle log
  mqttDiscoverSensor("cycle_count",       "Charge Cycles Logged",    "",    "",          "total_increasing");
  // Cell balance (max − min across all 28 cells, mV). Updates as the BMS rotates through cells.
  mqttDiscoverSensor("cell_balance_mv",   "Cell Balance",            "mV",  "",          "measurement");
  // BMS board temperature — byte 1 of 0x488 frame (ZeroSpy: "Controller Temp")
  mqttDiscoverSensor("bms_board_temp",    "BMS Board Temp",          "\xB0""C", "temperature", "measurement");
  // Per-cell average voltage — bytes 6-7 of 0x488 (uint16 LE, mV). Tracks pack_v/28.
  mqttDiscoverSensor("cell_avg_mv",       "Cell Average Voltage",    "mV",  "voltage",    "measurement");
  // Odometer (total distance) from the dash frame 0x2C0. Published WITHOUT the
  // availability topic (like SoC) so HA keeps the last value when the controller
  // is off. total_increasing so HA treats it as a lifetime counter.
  mqttDiscoverSensor("odometer_km",       "Odometer",                "km",  "distance",   "total_increasing", false);
  // Diagnostics — grouped under the HA device's "Diagnostic" section.
  // Published on a fixed 30 s cadence (see mqttPublishChanges), retained.
  // RSSI history is the tool for spotting link trouble; the uptime sawtooth
  // exposes unexpected reboots; firmware pins both to a build.
  mqttDiscoverDiagSensor("wifi_rssi", "WiFi RSSI",        "dBm", "signal_strength", "measurement");
  mqttDiscoverDiagSensor("uptime",    "Uptime",           "s",   "duration",        "measurement");
  mqttDiscoverDiagSensor("firmware",  "Firmware Version", "",    "",                "");

  LOG("[MQTT] Discovery published\n");
}

// Inbound command callback — called by PubSubClient on Core 1 during loop()
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Extract the last path component (command name)
  char* lastSlash = strrchr(topic, '/');
  if (!lastSlash) return;
  const char* cmd = lastSlash + 1;

  // Null-terminate payload into a small stack buffer
  char val[32];
  size_t copyLen = min((unsigned int)(sizeof(val) - 1), length);
  memcpy(val, payload, copyLen);
  val[copyLen] = '\0';

  // M1: MQTT has no response channel, so a dropped command can only be
  // surfaced in the log. Previously every branch logged the command as though
  // it had been applied even when the mutex take failed — most dangerously for
  // charging_enabled=false. Each branch now reports which of the two happened,
  // and the NVS write is skipped when the RAM write didn't land so the two
  // can't diverge.
  // B3: commands that change the charge itself (enabled / target power /
  // target voltage, including the preset buttons) latch homeDefaultsApplied,
  // so a later STA bring-up can't have the home WiFi profile overrule what the
  // user — or a Home Assistant automation — just asked for. charger_count and
  // ramp_rate_wps are configuration rather than charge decisions and are left
  // out, matching /api/control.
  //
  // C2: the latch happens INSIDE the successful-take branch, next to the write
  // it is claiming credit for. It used to sit above the take and fire even when
  // the mutex timed out — i.e. the flag said "the user has taken control of the
  // charge" on the strength of a command that was then dropped, and the home
  // profile it suppressed never got another chance this boot. /api/control
  // latches only after its take succeeds (it returns early on a timeout); these
  // branches now match it, which is what the sentence above assumes.
  if (strcmp(cmd, "target_power_w") == 0) {
    int tw = atoi(val);
    bool ok = (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(20)) == pdTRUE);
    if (ok) {
      ctrl.targetPowerW = (uint16_t)constrain(tw, 0, 13200);
      homeDefaultsApplied = true;   // C2 — only when the write actually landed
      xSemaphoreGive(controlMutex);
    }
    LOG("[MQTT] cmd target_power_w = %d W%s\n",
        tw, ok ? "" : "  *** NOT APPLIED: controlMutex timeout ***");

  } else if (strcmp(cmd, "charging_enabled") == 0) {
    bool en = (strcmp(val, "true") == 0 || strcmp(val, "1") == 0 ||
               strcmp(val, "ON")   == 0 || strcmp(val, "on") == 0);
    bool ok = (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(20)) == pdTRUE);
    if (ok) {
      ctrl.enabled = en;
      if (!en) ctrl.currentPowerW = 0;
      homeDefaultsApplied = true;   // B3, C2
      xSemaphoreGive(controlMutex);
    }
    LOG("[MQTT] cmd charging_enabled = %s%s\n", en ? "true" : "false",
        ok ? "" : "  *** NOT APPLIED: controlMutex timeout ***");

  } else if (strcmp(cmd, "charger_count") == 0) {
    int cc = atoi(val);
    if (cc >= 1 && cc <= (int)MAX_ACTIVE_CHARGERS) {
      bool ok = (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(20)) == pdTRUE);
      if (ok) {
        ctrl.chargerCount = (uint8_t)cc;
        xSemaphoreGive(controlMutex);
        preferences.putUChar("charger_count", (uint8_t)cc);
      }
      LOG("[MQTT] cmd charger_count = %d%s\n",
          cc, ok ? "" : "  *** NOT APPLIED: controlMutex timeout ***");
    }

  } else if (strcmp(cmd, "ramp_rate_wps") == 0) {
    int rr = atoi(val);
    if (rr >= 10 && rr <= 500) {
      bool ok = (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(20)) == pdTRUE);
      if (ok) {
        ctrl.rampStepW = (uint16_t)rr;
        xSemaphoreGive(controlMutex);
        preferences.putUShort("ramp_step_w", (uint16_t)rr);
      }
      LOG("[MQTT] cmd ramp_rate_wps = %d W/s%s\n",
          rr, ok ? "" : "  *** NOT APPLIED: controlMutex timeout ***");
    }

  } else if (strcmp(cmd, "target_volt_v") == 0) {
    // HA number sends value in Volts (e.g. "113.2"); convert to dV internally.
    float tvV = atof(val);
    int   tvd = (int)(tvV * 10.0f + 0.5f); // round to nearest dV
    if (tvd >= TARGET_VOLT_PRESETS[0].dv && tvd <= (int)MAX_CHARGE_VOLTAGE_DV) {
      bool ok = (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(20)) == pdTRUE);
      if (ok) {
        ctrl.targetVoltDv = (uint16_t)tvd;
        homeDefaultsApplied = true;   // B3, C2
        xSemaphoreGive(controlMutex);
        preferences.putUShort("target_volt_dv", (uint16_t)tvd);
      }
      LOG("[MQTT] cmd target_volt_v = %.1f V (%d dV)%s\n", tvV, tvd,
          ok ? "" : "  *** NOT APPLIED: controlMutex timeout ***");
    }

  } else if (strncmp(cmd, "preset_", 7) == 0) {
    // preset_70, preset_80, preset_90, preset_100 — set target voltage to the
    // preset's voltage and persist to NVS (button entities ignore payload).
    int pct = atoi(cmd + 7);
    for (int i = 0; i < TARGET_VOLT_PRESET_COUNT; i++) {
      if (TARGET_VOLT_PRESETS[i].pct == pct) {
        uint16_t dv = TARGET_VOLT_PRESETS[i].dv;
        bool ok = (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(20)) == pdTRUE);
        if (ok) {
          ctrl.targetVoltDv = dv;
          homeDefaultsApplied = true;   // B3, C2
          xSemaphoreGive(controlMutex);
          preferences.putUShort("target_volt_dv", dv);
        }
        LOG("[MQTT] cmd preset_%d → target_volt_v = %.1f V (%u dV)%s\n",
            pct, dv / 10.0f, dv,
            ok ? "" : "  *** NOT APPLIED: controlMutex timeout ***");
        break;
      }
    }

  } else if (strcmp(cmd, "reset_session") == 0) {
    // Any payload (HA button sends "PRESS") triggers the reset
    sessionReset();
    // Force MQTT snapshot reset so new zeroed values publish immediately
    mqttLast.sessionWh = -1;
    mqttLast.sessionAh = -1;
    LOG("[MQTT] cmd reset_session\n");
  }
}

// Connect/reconnect to broker — called from mqttTask when not connected
static bool mqttConnect() {
  // Snapshot broker config into task-local stable storage under settingsLock
  // (findings #7/#8). setServer() stores the host POINTER and setCACert()
  // stores the cert POINTER for the whole TLS session, so they must reference
  // memory only this task owns — never the shared globals another task could
  // reassign mid-handshake. The old code passed mqttCaCert.c_str() directly:
  // if the settings handler reassigned that String during connect(), the
  // pointer dangled (use-after-free). These statics live for the task's life.
  static char     s_host[sizeof(mqttHost)]       = "";
  static char     s_user[sizeof(mqttUser)]       = "";
  static char     s_pass[sizeof(mqttBrokerPass)] = "";
  static String   s_caCert;
  static uint16_t s_port = 1883;
  static bool     s_tls  = false;
  {
    // NET-12: best-effort by design — on a lock timeout we snapshot anyway and
    // risk one torn broker field, because failing here would stall the MQTT
    // task's reconnect loop entirely. A bad snapshot costs one failed connect
    // attempt; the next pass re-reads. The mutating web paths answer 503.
    bool locked = settingsLock();
    strncpy(s_host, mqttHost, sizeof(s_host)); s_host[sizeof(s_host) - 1] = '\0';
    strncpy(s_user, mqttUser, sizeof(s_user)); s_user[sizeof(s_user) - 1] = '\0';
    strncpy(s_pass, mqttBrokerPass, sizeof(s_pass)); s_pass[sizeof(s_pass) - 1] = '\0';
    s_caCert = mqttCaCert;   // deep copy into task-local String
    s_port   = mqttPort;
    s_tls    = mqttTls;
    if (locked) settingsUnlock();
  }

  char baseTopic[80], willTopic[90];
  snprintf(baseTopic, sizeof(baseTopic), "supercharger/%s", mqttHostname);
  snprintf(willTopic, sizeof(willTopic), "%s/state",        baseTopic);

  // Subscribe to both command topics
  char cmdBase[90];
  snprintf(cmdBase, sizeof(cmdBase), "%s/command/#", baseTopic);

  // Pick the underlying transport: WiFiClientSecure (TLS) when the user has
  // enabled SSL on the settings page AND uploaded a CA cert; plaintext
  // WiFiClient otherwise. We refuse to fall back to setInsecure() — without
  // a pinned CA, MQTT-TLS provides confidentiality but no authentication of
  // the broker, which is worse than plaintext (gives a false sense of
  // security and lets MITM attackers transparently relay credentials).
  if (s_tls) {
    if (s_caCert.length() == 0) {
      static unsigned long lastWarnMs = 0;
      if (millis() - lastWarnMs > 30000UL) {
        lastWarnMs = millis();
        LOG("[MQTT] TLS enabled but no CA cert configured — refusing to connect. "
            "Upload a PEM cert on the settings page or disable TLS.\n");
      }
      return false;
    }
    mqttTlsClient.setCACert(s_caCert.c_str());
    mqttClient.setClient(mqttTlsClient);
  } else {
    mqttClient.setClient(mqttWifiClient);
  }
  // Keep the broker host/port in sync with the (possibly-swapped) transport.
  mqttClient.setServer(s_host, s_port);

  bool ok = mqttClient.connect(
    mqttHostname,
    s_user,
    s_pass,
    willTopic,      // LWT topic
    1,              // LWT QoS
    true,           // LWT retained
    "offline"       // LWT payload
  );

  if (ok) {
    mqttClient.publish(willTopic, "online", true);
    mqttClient.subscribe(cmdBase, 1);
    mqttPublishDiscovery();
    // Reset snapshot so all values publish immediately after reconnect
    mqttLast = MqttSnapshot();
    LOG("[MQTT] Connected to %s:%d (%s) as %s\n",
        s_host, s_port, s_tls ? "TLS" : "plain", mqttHostname);
  } else {
    LOG("[MQTT] Connect failed (%s), rc=%d\n",
        s_tls ? "TLS" : "plain", mqttClient.state());
  }
  return ok;
}

// Publish any values that changed since last publish.
// Returns true if anything was published.
static bool mqttPublishChanges() {
  // Snapshot all data sources under their mutexes
  LiveData        ls;
  ChargerBusData  cs;
  ChargingControl ks;

  if (xSemaphoreTake(liveMutex,    pdMS_TO_TICKS(10)) == pdTRUE) { ls = live;       xSemaphoreGive(liveMutex); }
  if (xSemaphoreTake(chargerMutex, pdMS_TO_TICKS(10)) == pdTRUE) { cs = chargerBus; xSemaphoreGive(chargerMutex); }
  if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(10)) == pdTRUE) { ks = ctrl;       xSemaphoreGive(controlMutex); }

  // Session data: rampTask, /api/control and mqttCallback all touch these.
  // Use the helper for an atomic read of the (Wh, Ah) pair so a half-reset
  // is never observable.
  float sessWh = 0.0f, sessAh = 0.0f;
  sessionSnapshot(sessWh, sessAh);

  bool published = false;

  #define PUB_IF_CHANGED_F(field, mqttName, val, dec) \
    if ((val) != mqttLast.field) { \
      mqttPublishSensorF(mqttName, (val), dec); \
      mqttLast.field = (val); published = true; \
    }

  #define PUB_IF_CHANGED_I(field, mqttName, val) \
    if ((val) != mqttLast.field) { \
      mqttPublishSensorI(mqttName, (int)(val)); \
      mqttLast.field = (val); published = true; \
    }

  // Pack temperature variant. A disconnected thermistor is stored raw as
  // ZERO_TEMP_INVALID (-32768); publishing that would write -32768 °C into the
  // HA recorder and wreck every temperature graph. Skip the publish entirely —
  // the retained previous value stays on the broker, mqttLast is left untouched,
  // and the sensor republishes by itself on the first tick with a real reading.
  #define PUB_IF_CHANGED_T(field, mqttName, val) \
    if ((val) > TEMP_INVALID_THRESHOLD) { PUB_IF_CHANGED_I(field, mqttName, val) }

  // Monolith pack
  PUB_IF_CHANGED_F(monolithVoltageDv, "monolith_v",    ls.monolithVoltageDv / 10.0f, 1)
  PUB_IF_CHANGED_I(monolithAmps,      "monolith_a",    ls.monolithAmps)
  PUB_IF_CHANGED_T(monolithMinTemp,   "monolith_tmin", ls.monolithMinTemp)
  PUB_IF_CHANGED_T(monolithMaxTemp,   "monolith_tmax", ls.monolithMaxTemp)
  // Prefer the BMS-reported SoC (0x188 byte 0) — same source as the bike's
  // dashboard. Fall back to the voltage-curve estimate only until the first
  // 0x188 frame arrives.
  int curSoc = (ls.monolithBmsSoc <= 100) ? (int)ls.monolithBmsSoc
                                          : calcSocFromVoltage(ls.monolithVoltageDv);
  PUB_IF_CHANGED_I(monolithSoc,       "monolith_soc",  curSoc)
  // Available capacity right now (nominal AH × SoC). Republish only on
  // ≥ 0.5 Ah change so HA isn't spammed every tick during charging.
  float curAhAvail = (ls.monolithAH > 0) ? (float)ls.monolithAH * curSoc / 100.0f : 0.0f;
  if (fabsf(curAhAvail - mqttLast.monolithAhAvail) >= 0.5f) {
    mqttPublishSensorF("monolith_ah_avail", curAhAvail, 1);
    mqttLast.monolithAhAvail = curAhAvail;
    published = true;
  }

  // PowerTank pack (only publish while present or during the cycle it disappears)
  if (ls.powerTankPresent || mqttLast.powerTankPresent) {
    PUB_IF_CHANGED_F(powerTankVoltageDv, "powertank_v",    ls.powerTankVoltageDv / 10.0f, 1)
    PUB_IF_CHANGED_I(powerTankAmps,      "powertank_a",    ls.powerTankAmps)
    PUB_IF_CHANGED_T(powerTankMinTemp,   "powertank_tmin", ls.powerTankMinTemp)
    PUB_IF_CHANGED_T(powerTankMaxTemp,   "powertank_tmax", ls.powerTankMaxTemp)
    mqttLast.powerTankPresent = ls.powerTankPresent;
  }

  // Charging control state
  PUB_IF_CHANGED_I(chargerCount,  "charger_count",   ks.chargerCount)
  // Charging power. In CC (Bulk) the commanded ramp value is the useful
  // figure. In CV (Absorption) that value ramps to 0 — the controller stops
  // commanding current and the pack simply draws what it will — so publishing
  // it makes HA's "Charging Power" read 0 W while the charger is still
  // delivering. Mirror the dashboard: in absorption publish the actual pack
  // power, V×|I|, summed across monolith + PowerTank.
  {
    int dispPowerW;
    if (g_rampPhase == 1) {  // 1 = absorption / CV
      float p = fabsf((float)ls.monolithAmps) * (ls.monolithVoltageDv / 10.0f);
      if (ls.powerTankPresent)
        p += fabsf((float)ls.powerTankAmps) * (ls.powerTankVoltageDv / 10.0f);
      dispPowerW = (int)(p + 0.5f);
    } else {
      dispPowerW = ks.currentPowerW;
    }
    PUB_IF_CHANGED_I(currentPowerW, "current_power_w", dispPowerW)
  }
  PUB_IF_CHANGED_I(targetPowerW,  "target_power_w",  ks.targetPowerW)
  PUB_IF_CHANGED_I(rampStepW,     "ramp_rate_wps",   ks.rampStepW)
  PUB_IF_CHANGED_F(targetVoltDv,  "target_volt_v",   ks.targetVoltDv / 10.0f, 1)

  // Active preset percentage (0 if current target voltage doesn't match a preset)
  uint8_t presetPct = 0;
  for (int i = 0; i < TARGET_VOLT_PRESET_COUNT; i++) {
    if (TARGET_VOLT_PRESETS[i].dv == ks.targetVoltDv) {
      presetPct = TARGET_VOLT_PRESETS[i].pct;
      break;
    }
  }
  if (presetPct != mqttLast.targetPresetPct) {
    char tmp[6];
    snprintf(tmp, sizeof(tmp), "%u", presetPct);
    mqttPublishSensor("target_preset_pct", tmp);
    mqttLast.targetPresetPct = presetPct;
    published = true;
  }

  if (mqttLast.enabledForced || ks.enabled != mqttLast.enabled) {
    mqttPublishSensor("charging_enabled", ks.enabled ? "true" : "false");
    mqttLast.enabled       = ks.enabled;
    mqttLast.enabledForced = false;
    published = true;
  }

  // Thermal throttling state — published as "true"/"false" string for HA
  bool thermal = g_thermalThrottle;
  if (mqttLast.thermalThrottleForced || thermal != mqttLast.thermalThrottle) {
    mqttPublishSensor("thermal_throttle", thermal ? "true" : "false");
    mqttLast.thermalThrottle       = thermal;
    mqttLast.thermalThrottleForced = false;
    published = true;
  }

  // Charging stage — published as "bulk" / "absorption" / "float"
  uint8_t rp = g_rampPhase;
  if (rp != mqttLast.rampPhase) {
    const char* rps = (rp == 1) ? "absorption"
                    : (rp == 2) ? "float"
                                : "bulk";
    mqttPublishSensor("ramp_phase", rps);
    mqttLast.rampPhase = rp;
    published = true;
  }

  // Time-to-target estimate (minutes). -1 = unknown / not charging.
  // Republish only when the value actually changes — otherwise the broker
  // sees a "min" tick every second.
  int16_t eta = g_etaMinutes;
  if (eta != mqttLast.etaMinutes) {
    mqttPublishSensorI("eta_minutes", (int)eta);
    mqttLast.etaMinutes = eta;
    published = true;
  }

  // Session data — publish whenever value changes by ≥ 0.1 Wh / 0.01 Ah
  // (avoids flooding broker every second while charging)
  if (fabsf(sessWh - mqttLast.sessionWh) >= 0.1f) {
    mqttPublishSensorF("session_wh", sessWh, 1);
    mqttLast.sessionWh = sessWh;
    published = true;
  }
  if (fabsf(sessAh - mqttLast.sessionAh) >= 0.01f) {
    mqttPublishSensorF("session_ah", sessAh, 2);
    mqttLast.sessionAh = sessAh;
    published = true;
  }

  // Cycle count — only changes when a cycle completes or the log is cleared
  PUB_IF_CHANGED_I(cycleCount, "cycle_count", g_cycleCount)

  // Cell balance (max − min across all seen cells). Only publish when we have
  // data (>0 means at least 2 cells seen) to avoid spamming 0 at boot.
  if (ls.cellBalanceMv > 0 && ls.cellBalanceMv != mqttLast.cellBalanceMv) {
    mqttPublishSensorI("cell_balance_mv", (int)ls.cellBalanceMv);
    mqttLast.cellBalanceMv = ls.cellBalanceMv;
    published = true;
  }

  // BMS board temperature. Only publish once a valid reading has arrived
  // (-128 is the "not yet received" sentinel stored in the LiveData int8).
  if (ls.bmsBoardTempC != -128 && (int16_t)ls.bmsBoardTempC != mqttLast.bmsBoardTempC) {
    mqttPublishSensorI("bms_board_temp", (int)ls.bmsBoardTempC);
    mqttLast.bmsBoardTempC = (int16_t)ls.bmsBoardTempC;
    published = true;
  }

  // Per-cell average voltage (0x488 b[6-7]). Only publish once valid (>0).
  if (ls.cellAvgMv > 0 && ls.cellAvgMv != mqttLast.cellAvgMv) {
    mqttPublishSensorI("cell_avg_mv", (int)ls.cellAvgMv);
    mqttLast.cellAvgMv = ls.cellAvgMv;
    published = true;
  }

  // Odometer (dash 0x2C0). Only publish once a value has been decoded (>0) and
  // when it changes. Published in km with one decimal.
  if (ls.odometerHm > 0 && ls.odometerHm != mqttLast.odometerHm) {
    mqttPublishSensorF("odometer_km", ls.odometerHm / 10.0f, 1);
    mqttLast.odometerHm = ls.odometerHm;
    published = true;
  }

  // Diagnostic trio — WiFi RSSI, uptime, firmware. Fixed 30 s cadence instead
  // of change-detection: RSSI jitters a few dB constantly and uptime changes
  // every second, so PUB_IF_CHANGED (with the 10 s snapshot reset) would spam
  // the broker. State topics are retained, so HA always has the latest values
  // after its own restart. First pass publishes immediately on (re)connect.
  {
    static unsigned long lastDiagMs = 0;
    if (lastDiagMs == 0 || millis() - lastDiagMs >= 30000UL) {
      lastDiagMs = millis();
      mqttPublishSensorI("wifi_rssi", (int)WiFi.RSSI());
      mqttPublishSensorI("uptime",    (int)(millis() / 1000UL));
      char fw[16];
      snprintf(fw, sizeof(fw), "%lld", (long long)VERSION);
      mqttPublishSensor("firmware", fw);
      published = true;
    }
  }

  #undef PUB_IF_CHANGED_F
  #undef PUB_IF_CHANGED_T
  #undef PUB_IF_CHANGED_I

  return published;
}

void mqttTask(void* /*pvParameters*/) {
  LOG("[MQTT] mqttTask running on core %d\n", xPortGetCoreID());

  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(768); // discovery payloads need > default 256 bytes;
                                 // diag entities add ent_cat + sw to the payload
  // Note: setServer() is now done inside mqttConnect() from a locked snapshot
  // of the broker config — this task is the ONLY one that touches mqttClient.

  unsigned long lastConnectAttempt = 0;
  unsigned long lastKeepalive      = 0;
  const unsigned long RECONNECT_INTERVAL = 10000; // ms between reconnect attempts
  const unsigned long KEEPALIVE_INTERVAL = 10000; // ms between forced republish

  for (;;) {
    // Only attempt MQTT when STA is actually associated. Works in both
    // STATE_CONNECTED (STA-only) and STATE_AP_RETRYING (AP+STA, STA up).
    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // Consume a reconnect request from the settings handlers (finding #7).
    // mqttClient is owned exclusively by this task, so the disconnect happens
    // here rather than cross-task. Forcing a disconnect makes the block below
    // reconnect with the freshly-saved (locked-snapshot) broker config.
    if (mqttReconnectRequested) {
      mqttReconnectRequested = false;
      if (mqttClient.connected()) {
        LOG("[MQTT] Reconnecting to apply new settings\n");
        mqttClient.disconnect();
      }
      lastConnectAttempt = 0;  // allow an immediate reconnect attempt
    }

    if (!mqttClient.connected()) {
      unsigned long now = millis();
      if (lastConnectAttempt == 0 || now - lastConnectAttempt >= RECONNECT_INTERVAL) {
        lastConnectAttempt = now;
        mqttConnect();   // re-reads broker config under settingsLock
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    mqttClient.loop(); // process inbound messages

    unsigned long now = millis();

    // Publish changed values
    mqttPublishChanges();

    // Keepalive: force full republish every 10 s even with no changes
    if (now - lastKeepalive >= KEEPALIVE_INTERVAL) {
      lastKeepalive = now;
      mqttLast = MqttSnapshot(); // reset snapshot → everything republishes
    }

    vTaskDelay(pdMS_TO_TICKS(200)); // 5 Hz check rate — responsive but not busy
  }
}

void mqttInit() {
  xTaskCreatePinnedToCore(
    mqttTask,
    "mqttTask",
    8192,          // generous stack — static buffers now in BSS, but TCP/TLS needs headroom
    nullptr,
    2,             // same priority as rampTask
    &mqttTaskHandle,
    1              // Core 1
  );
}

// FreeRTOS stack overflow hook — called when a task overflows its stack.
// Requires configCHECK_FOR_STACK_OVERFLOW >= 1 (CONFIG_FREERTOS_CHECK_STACKOVERFLOW_CANARY
// is set in the ESP32 Arduino core's sdkconfig, so this is live).
//
// STAB-5: this runs from the scheduler with a corrupt stack under it, so it must
// not allocate, take a mutex or block — LOG() does all three (logMutex +
// vsnprintf + Serial.write), and the old `while (true) vTaskDelay(...)` halt left
// the device wedged forever with whatever the chargers were last commanded to do.
// ets_printf() writes straight to the ROM UART with no locking and no heap, then
// we reboot. Rebooting is the safe outcome here: it stops the 1 Hz charger
// heartbeat, and the DigiNow units self-stop within ~5 s of losing it, so the
// pack is never left charging unsupervised.
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
  ets_printf("[FATAL] stack overflow in %s — restarting\n", pcTaskName);
  esp_restart();
}
