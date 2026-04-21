// ==========================================================================
// Supercharger controller for Zero bike based on LilyGo T-2CAN (ESP32-S3)
//
// Hardware info:
//  CAN:
//   Charger: MCP2515 via SPI  (CS=10, SCLK=12, MOSI=11, MISO=13, RST=9)
//   Bike: ESP32-S3 TWAI    (TX=7, RX=6)
//
// Written for charger hardware Elcon TC HK-J 3300W
// Other functions:
//   Web dashboard (live status, auto-refresh via /api/status JSON)
//   WiFi – connects to saved network, falls back to AP for setup
//   OTA firmware update via browser (/update, HTTP Basic Auth)
//   MQTT communication (for HA)
//
// MQTT info:
//
// Home Assistant integration:
//   MQTT discovery prefix : homeassistant/
//   Device base topic     : supercharger/<HOSTNAME>/
//   Sensors (read-only)   : actual_volts, target_volts, monolith_volts,
//                           powertank_volts, charge_amps, monolith_amps,
//                           powertank_amps, monolith_ah, powertank_ah,
//                           monolith_min_temp, monolith_max_temp,
//                           powertank_min_temp, powertank_max_temp,
//                           charge_power, target_power_w, max_power_w,
//                           coulombs, watt_hours
//   Controls (read/write) : target_voltage_dv (number), target_power_set (number),
//                           charger_count (number), ramp_rate_wps (number),
//                           c_plus (switch), charging_enabled (switch)
//
// Required libraries (Arduino Library Manager):
//   mcp_can          by coryjfowler
//   PubSubClient     by Nick O'Leary
//   ArduinoJson      by Benoit Blanchon  (v6)
//   BLEDevice        } built into Espressif ESP32 Arduino core - no install needed
//   ESPmDNS          } built into Espressif ESP32 Arduino core - no install needed
//   driver/twai.h    } built into Espressif ESP32 Arduino core - no install needed
// ==========================================================================

#define VERSION 202604211434

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <arduino_secrets.h>
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
#include <PubSubClient.h>

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

// Tracks which credential source is currently being attempted, so
// monitorWifiStatus() knows whether to try the secrets fallback next
// or go straight to AP+STA retry mode.
enum WifiSource { WIFI_SRC_NONE, WIFI_SRC_PREFS, WIFI_SRC_SECRETS };
WifiSource wifiSource = WIFI_SRC_NONE;

// Timestamp of when WiFi.begin() was last called — used to enforce a
// connect timeout before declaring failure and moving to the next source.
unsigned long wifiConnectStartMs    = 0;
const unsigned long wifiConnectTimeoutMs = 15000; // 15 s per attempt

// STATE_AP_RETRYING bookkeeping. retryStaSsid/Pass hold the credentials we're
// continuously trying to reach; populated when entering the state from either
// a failed boot connect or a runtime drop. The driver's auto-reconnect handles
// most of the work, but we explicitly nudge it every staRetryIntervalMs as a
// belt-and-braces measure for cases where the driver gives up.
static char retryStaSsid[33] = "";
static char retryStaPass[65] = "";
const unsigned long staRetryIntervalMs = 30000UL;
static unsigned long lastStaRetryAt    = 0;

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
  short monolithMinTemp     = 0;
  short monolithMaxTemp     = 0;
  short monolithMaxCRate    = 0;

  // PowerTank (BMS1)
  long  powerTankVoltageDv  = 0;
  short powerTankSagAdjDv   = 0;  // sagAdjust from BMS1_PACK_CONFIG (0x289) bytes 0-1
  short powerTankAmps       = 0;
  short powerTankAH         = 0;
  short powerTankMinTemp    = 0;
  short powerTankMaxTemp    = 0;
  short powerTankMaxCRate   = 0;

  // Presence / freshness flags
  bool  dataFresh           = false; // true once first BMS0 frame received
  bool  powerTankPresent    = false; // true once first BMS1 frame received
  bool  powerTankDecided    = false; // true once detection window has closed

  // Detection window timestamps (ms) — not sent in JSON
  unsigned long bms0FirstMs = 0;
  unsigned long bms1FirstMs = 0;
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
// STATUS byte bitfield (byte 4 of status frame):
//   0x01  Hardware fault
//   0x02  Overtemperature
//   0x04  AC input problem
//   0x08  No battery connected
//   0x10  Battery disconnect / reverse / not OK
// ---------------------------------------------------------------------------

SemaphoreHandle_t chargerMutex = nullptr;

// Known charger status IDs — low nibble is the charger index (5,7,8,9).
// Stored as the nibble value for compact bitmask indexing.
// 0x18FF50E5 -> nibble 5, 0x18FF50E7 -> 7, 0x18FF50E8 -> 8, 0x18FF50E9 -> 9
static const uint32_t CHARGER_CMD_ID         = 0x1806E5F4UL;
static const uint32_t CHARGER_STATUS_ID_BASE = 0x18FF50E0UL; // mask low nibble
static const uint32_t CHARGER_STATUS_ID_MASK = 0x1FFFFFF0UL; // top 28 bits

// Maximum distinct chargers we track (nibbles 0–15)
#define MAX_CHARGERS 16

struct ChargerUnit {
  bool     present   = false;
  uint16_t voltDv    = 0;    // actual output voltage * 10 (0.1 V units → dV)
  uint16_t ampsDa    = 0;    // actual output current * 10 (0.1 A units → dA)
  uint8_t  status    = 0;    // raw status bitfield
  unsigned long lastSeenMs = 0;
};

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

// Charger is considered gone if no status frame received within this window
static const unsigned long CHARGER_TIMEOUT_MS = 10000;

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

// Ramp rate: default 50 W per ramp tick (1 s), slider step 100 W
static const uint16_t DEFAULT_RAMP_STEP_W = 50;
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
  bool     enabled       = true;  // default: enabled per spec
  uint8_t  chargerCount  = 3;     // manual charger count for per-unit current division
  uint16_t rampStepW     = DEFAULT_RAMP_STEP_W; // W per second ramp rate, configurable
  uint16_t targetVoltDv  = 1164;  // target charge voltage (dV), default 100% = 116.4V
} ctrl;

// Session energy tracking — accumulated in rampTask, reset on boot or manual reset
struct SessionData {
  float energyWh  = 0.0f;  // cumulative Wh delivered this session
  float chargeAh  = 0.0f;  // cumulative Ah delivered this session
  unsigned long startMs = 0; // session start timestamp
} session;

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
} mqttLast;

static WiFiClient   mqttWifiClient;
static PubSubClient mqttClient(mqttWifiClient);
static TaskHandle_t mqttTaskHandle = nullptr;
static char         mqttHostname[32] = "supercharger";

// Runtime-configurable AP and MQTT settings.
// Loaded from NVS at boot, falling back to arduino_secrets.h, then defaults.
// Written by /api/settings POST handler; read by startAPMode() and mqttTask().
static char     apSSID[33]         = "Supercharger";
static char     apPass[65]         = "12345678";
static char     mqttHost[64]       = "";
static uint16_t mqttPort           = 1883;
static char     mqttUser[33]       = "";
static char     mqttBrokerPass[65] = "";

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
#define BTN_HOLD_AP_RESET_MS    5000UL   // 5 s : clear WiFi creds → AP mode on next boot
#define BTN_HOLD_FACTORY_MS    10000UL   // 10 s: wipe entire NVS namespace

// ---------------------------------------------------------------------------
// WiFi setup page — only served in AP / setup mode
// ---------------------------------------------------------------------------

const char HTML_SETTINGS[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Settings — Supercharger</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;padding:16px;max-width:600px;margin:auto}
    h1{color:#e94560;font-size:1.4em;margin-bottom:6px}
    nav a{color:#aaa;font-size:0.85em;text-decoration:none;margin-right:14px}
    nav a:hover{color:#e94560}
    .section{margin-top:22px;font-size:0.78em;color:#666;text-transform:uppercase;
             letter-spacing:.08em;border-bottom:1px solid #0f3460;padding-bottom:4px;margin-bottom:12px}
    .box{background:#16213e;padding:20px;border-radius:10px;margin-bottom:16px;
         box-shadow:0 4px 12px rgba(0,0,0,.4)}
    label{display:block;font-size:0.82em;color:#aaa;margin-bottom:3px;margin-top:10px}
    label:first-child{margin-top:0}
    input[type=text],input[type=password],input[type=number]{
      width:100%;padding:9px;border:1px solid #0f3460;border-radius:5px;
      background:#0f3460;color:#eee;font-size:14px}
    .row2{display:grid;grid-template-columns:1fr 1fr;gap:12px}
    button{background:#e94560;color:#fff;border:none;padding:11px 24px;
           border-radius:5px;font-size:15px;cursor:pointer;margin-top:14px;width:100%}
    button:hover{opacity:0.9}
    .btn-secondary{background:#0f3460;border:1px solid #e94560}
    .msg{text-align:center;padding:14px;border-radius:8px;margin-top:12px;display:none}
    .msg.ok{display:block;background:#0a3d2a;color:#4ade80}
    .msg.err{display:block;background:#3d0a0a;color:#f87171}
    .hint{font-size:0.75em;color:#666;margin-top:3px}
  </style>
</head>
<body>
  <h1>&#9881; Settings</h1>
  <nav><a href="/">&#8592; Dashboard</a></nav>

  <div class="section">WiFi Network</div>
  <div class="box">
    <label>SSID</label>
    <input type="text" id="wifiSSID" placeholder="Network name">
    <label>Password</label>
    <input type="password" id="wifiPass" placeholder="Password">
    <button onclick="saveWifi()">Save WiFi &amp; Restart</button>
    <div class="hint">Connects to this network on boot. Falls back to AP mode if unavailable.</div>
  </div>

  <div class="section">Access Point (Fallback)</div>
  <div class="box">
    <label>AP Name</label>
    <input type="text" id="apSSID" placeholder="Supercharger">
    <label>AP Password</label>
    <input type="password" id="apPass" placeholder="Min 8 characters">
    <button onclick="saveAP()">Save AP Settings</button>
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
      <div><label>Host / IP</label>
        <input type="text" id="mqttHost" placeholder="192.168.1.100"></div>
      <div><label>Port</label>
        <input type="number" id="mqttPort" placeholder="1883"></div>
    </div>
    <div class="row2">
      <div><label>Username</label>
        <input type="text" id="mqttUser" placeholder="(optional)"></div>
      <div><label>Password</label>
        <input type="password" id="mqttPass" placeholder="(optional)"></div>
    </div>
    <button onclick="saveMQTT()">Save MQTT &amp; Reconnect</button>
  </div>

  <div class="section">Charger Hardware</div>
  <div class="box">
    <label>Number of Chargers (1–4)</label>
    <input type="number" id="ccCount" min="1" max="4" value="3">
    <button onclick="saveChargerCount()">Save Charger Count</button>
  </div>

  <div class="section">Charging Behaviour</div>
  <div class="box">
    <label>Ramp Rate (W/s, 10–500)</label>
    <input type="number" id="rampRate" min="10" max="500" value="50">
    <div class="hint">Power increases/decreases by this many watts per second when ramping. Lower = gentler ramp. Default: 50 W/s.</div>
    <button onclick="saveRampRate()">Save Ramp Rate</button>
  </div>

  <div id="msgBox" class="msg"></div>

  <script>
    function showMsg(text, ok) {
      var m = document.getElementById('msgBox');
      m.textContent = text;
      m.className = 'msg ' + (ok ? 'ok' : 'err');
      setTimeout(function(){ m.className = 'msg'; }, 4000);
    }

    // Load current settings on page load
    fetch('/api/settings').then(function(r){ return r.json(); }).then(function(d){
      document.getElementById('wifiSSID').value = d.wifi_ssid || '';
      document.getElementById('apSSID').value   = d.ap_ssid   || '';
      if (d.ap_pass_set) {
        document.getElementById('apPass').placeholder = '(unchanged — leave blank to keep)';
      }
      document.getElementById('ccCount').value    = d.charger_count || 3;
      document.getElementById('rampRate').value   = d.ramp_rate_wps || 50;
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
      }
      // Passwords are never returned from the API for security
    });

    function postSettings(data, successMsg, restart) {
      fetch('/api/settings', {
        method:'POST',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify(data)
      }).then(function(r){ return r.json(); }).then(function(d){
        if (d.ok) {
          showMsg(successMsg, true);
          if (restart) setTimeout(function(){ location.href='/'; }, 3000);
        } else {
          showMsg(d.error || 'Save failed', false);
        }
      }).catch(function(){ showMsg('Connection error', false); });
    }

    function saveWifi() {
      var ssid = document.getElementById('wifiSSID').value.trim();
      var pass = document.getElementById('wifiPass').value;
      if (!ssid) { showMsg('SSID cannot be empty', false); return; }
      postSettings({wifi_ssid: ssid, wifi_pass: pass}, 'WiFi saved — restarting...', true);
    }
    function saveAP() {
      var ssid = document.getElementById('apSSID').value.trim();
      var pass = document.getElementById('apPass').value;
      if (!ssid) { showMsg('AP name cannot be empty', false); return; }
      if (pass.length > 0 && pass.length < 8) { showMsg('AP password must be 8+ characters', false); return; }
      var data = {ap_ssid: ssid};
      if (pass.length > 0) data.ap_pass = pass; // only send if user entered a new password
      postSettings(data, 'AP settings saved', false);
    }
    function saveMQTT() {
      postSettings({
        mqtt_host: document.getElementById('mqttHost').value.trim(),
        mqtt_port: parseInt(document.getElementById('mqttPort').value) || 1883,
        mqtt_user: document.getElementById('mqttUser').value.trim(),
        mqtt_pass: document.getElementById('mqttPass').value
      }, 'MQTT saved — reconnecting...', false);
    }
    function saveChargerCount() {
      var cc = parseInt(document.getElementById('ccCount').value);
      if (cc < 1 || cc > 4) { showMsg('Must be 1-4', false); return; }
      postSettings({charger_count: cc}, 'Charger count saved', false);
    }
    function saveRampRate() {
      var rr = parseInt(document.getElementById('rampRate').value);
      if (rr < 10 || rr > 500) { showMsg('Ramp rate must be 10–500 W/s', false); return; }
      fetch('/api/control', {
        method: 'POST',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify({ramp_rate_wps: rr})
      }).then(function(r){ return r.json(); }).then(function(d){
        if (d.ok) showMsg('Ramp rate saved', true);
        else showMsg(d.error || 'Save failed', false);
      }).catch(function(){ showMsg('Connection error', false); });
    }
  </script>
</body>
</html>
)rawliteral";

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
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Supercharger</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;padding:16px}
    h1{color:#e94560;font-size:1.4em;margin-bottom:6px}
    nav a{color:#aaa;font-size:0.85em;text-decoration:none;margin-right:14px}
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
    .big-btn{background:#e94560;color:#fff;border:none;padding:12px 22px;border-radius:6px;
             font-size:1em;font-weight:bold;cursor:pointer;min-width:150px}
    .big-btn.off{background:#0f3460}
    .pwr-display{text-align:center;min-width:70px;display:flex;flex-direction:column;
                 align-items:center;align-self:flex-start;padding-top:3px}
    .pwr-label{font-size:0.65em;color:#888;text-transform:uppercase;letter-spacing:.05em;
               margin-bottom:3px}
    .pwr-val{font-size:1.4em;font-weight:bold;color:#e94560;line-height:1.2}
    .pwr-unit{font-size:0.72em;color:#aaa;margin-left:2px}
    .phase-badge{font-size:1em;font-weight:bold;padding:2px 10px;border-radius:4px;
                 border:1px solid currentColor;white-space:nowrap;line-height:1.4}
    .phase-cc  {color:#e9a020;border-color:#e9a020}
    .phase-cv  {color:#4ab4f8;border-color:#4ab4f8}
    .phase-done{color:#4caf82;border-color:#4caf82}
    .preset-label{font-size:0.70em;color:#888;text-transform:uppercase;
                  letter-spacing:.05em;margin-bottom:6px}
    .preset-row{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:14px}
    .preset-btn{background:#0f3460;color:#eee;border:1px solid #e94560;padding:7px 14px;
                border-radius:5px;font-size:0.85em;cursor:pointer}
    .preset-btn:hover{background:#e94560}
    .preset-btn.active{background:#e94560}
    .slider-wrap{display:flex;align-items:center;gap:10px}
    input[type=range]{flex:1;accent-color:#e94560;height:6px}
    .slider-readout{min-width:70px;text-align:right;font-size:0.9em;color:#eee}
    .cc-row{display:flex;align-items:center;gap:10px;margin-top:14px}
    .cc-row .preset-label{margin-bottom:0}
    .cc-btn{background:#0f3460;color:#eee;border:1px solid #0f3460;padding:6px 14px;
            border-radius:5px;font-size:0.85em;cursor:pointer;min-width:32px;text-align:center}
    .cc-btn.active{background:#e94560;border-color:#e94560}
    /* SOC card */
    .soc-card{grid-column:span 2}
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
  </style>
</head>
<body>
  <h1>&#9889; Supercharger
    <span class="badge stale" id="badge">NO DATA</span>
  </h1>
  <nav><a href="/settings">&#9881; Settings</a><a href="/update">&#128190; OTA Update</a><a href="/log">&#128220; Log</a></nav>

  <div id="apBanner" style="display:none;background:#0f3460;border:1px solid #e94560;
       border-radius:8px;padding:12px 16px;margin-top:12px;text-align:center">
    <span style="color:#e94560;font-weight:bold">&#9888; No WiFi</span>
    <span style="color:#aaa"> — Running in AP mode.</span>
    <a href="/settings" style="color:#e94560;margin-left:8px">Configure WiFi &#8594;</a>
  </div>

  <div class="section">Monolith Pack</div>
  <div class="grid">
    <div class="card"><div class="label">Voltage</div>
      <span class="val" id="mV">—</span></div>
    <div class="card"><div class="label">Current</div>
      <span class="val" id="mA">—</span></div>
    <div class="card"><div class="label">Capacity</div>
      <span class="val" id="mAH">—</span></div>
    <div class="card soc-card"><div class="label">State of Charge</div>
      <span class="val" id="mSOC">—</span>
      <div class="soc-bar-wrap"><div class="soc-bar" id="mSOCBar"></div></div></div>
    <div class="card"><div class="label">Temp Min / Max</div>
      <span class="val" id="mT">—</span></div>
    <div class="card"><div class="label">Max C-Rate</div>
      <span class="val" id="mC">—</span></div>
  </div>

  <div id="ptSection" style="display:none">
    <div class="section">PowerTank Pack</div>
    <div class="grid">
      <div class="card"><div class="label">Voltage</div>
        <span class="val" id="pV">—</span></div>
      <div class="card"><div class="label">Current</div>
        <span class="val" id="pA">—</span></div>
      <div class="card"><div class="label">Capacity</div>
        <span class="val" id="pAH">—</span></div>
      <div class="card"><div class="label">Temp Min / Max</div>
        <span class="val" id="pT">—</span></div>
      <div class="card"><div class="label">Max C-Rate</div>
        <span class="val" id="pC">—</span></div>
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
    <div class="card"><div class="label">Ramp Rate</div>
      <span class="val" id="rampRate">—</span><span class="unit">W/s</span></div>
    <div class="card" style="cursor:pointer" onclick="resetSession()" title="Click to reset session">
      <div class="label">Reset Session</div>
      <span class="val" style="font-size:1em;color:#888">&#8635; Reset</span></div>
  </div>

  <div class="section">Charging Control</div>
  <div class="ctrl-box">

    <div class="ctrl-row">
      <button class="big-btn" id="btnEnable" onclick="toggleEnable()">&#9654; Charging ON</button>
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
        <span id="chgMode" class="phase-badge phase-cc">—</span>
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
      <button class="cc-btn" onclick="setChargerCount(1)">1</button>
      <button class="cc-btn" onclick="setChargerCount(2)">2</button>
      <button class="cc-btn active" onclick="setChargerCount(3)">3</button>
      <button class="cc-btn" onclick="setChargerCount(4)">4</button>
    </div>
  </div>

  <div class="section">Target Voltage</div>
  <div class="tgt-volt-box">
    <div id="tgtVoltBtns" class="tgt-volt-row"></div>
  </div>

  <div class="section">System</div>
  <div class="grid">
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
  </div>

  <footer id="footer">Waiting for first update...</footer>

  <script>
    var fmt = function(v, d){ return (v === null || isNaN(v)) ? '—' : (+v).toFixed(d); };

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

    // Status bitfield labels
    var STATUS_BITS = [
      [0x01, 'HW Fault'],
      [0x02, 'Overtemp'],
      [0x04, 'AC Fault'],
      [0x08, 'No Battery'],
      [0x10, 'Batt Fault']
    ];
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

    function sendControl(targetW, enabled) {
      fetch('/api/control', {
        method: 'POST',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify({target_w: targetW, enabled: enabled})
      });
    }

    function setPreset(w) {
      var slider = document.getElementById('pwrSlider');
      slider.value = w;
      document.getElementById('sliderVal').textContent = w;
      sendControl(w, chargingEnabled);
    }

    // Live readout while dragging — don't send until release
    function onSliderInput(v) {
      document.getElementById('sliderVal').textContent = v;
    }

    // Commit on mouse/touch release
    function onSliderCommit(v) {
      sendControl(parseInt(v), chargingEnabled);
    }

    function toggleEnable() {
      chargingEnabled = !chargingEnabled;
      updateEnableBtn();
      var slider = document.getElementById('pwrSlider');
      sendControl(parseInt(slider.value), chargingEnabled);
    }

    function setChargerCount(n) {
      fetch('/api/control', {
        method: 'POST',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify({charger_count: n})
      });
      var btns = document.querySelectorAll('.cc-btn');
      for (var i = 0; i < btns.length; i++) {
        btns[i].className = 'cc-btn' + (i + 1 === n ? ' active' : '');
      }
      lastChargerCount = -1; // force preset rebuild
      rebuildPresets(n);
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
      fetch('/api/control', {
        method: 'POST',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify({target_volt_dv: dv})
      });
      // Optimistic highlight
      for (var i = 0; i < TARGET_VOLT_PRESETS.length; i++) {
        var b = document.getElementById('tvBtn_' + TARGET_VOLT_PRESETS[i].dv);
        if (b) b.className = 'tgt-btn' + (TARGET_VOLT_PRESETS[i].dv === dv ? ' active' : '');
      }
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
      fetch('/api/control', {
        method: 'POST',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify({reset_session: true})
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

    function refresh(){
      fetch('/api/status')
        .then(function(r){ return r.json(); })
        .then(function(d){
          var b = document.getElementById('badge');
          b.textContent = d.fresh ? 'LIVE' : 'NO DATA';
          b.className   = 'badge ' + (d.fresh ? 'ok' : 'stale');

          // AP mode banner
          document.getElementById('apBanner').style.display = d.ap_mode ? 'block' : 'none';

          document.getElementById('mV').textContent  = fmt(d.monolith_v,  1) + ' V';
          document.getElementById('mA').textContent  = d.monolith_a + ' A';
          document.getElementById('mAH').textContent = d.monolith_ah + ' Ah';
          document.getElementById('mT').textContent  = fmt(d.monolith_tmin, 0)
                                               + ' / ' + fmt(d.monolith_tmax, 0) + ' \u00B0C';
          document.getElementById('mC').textContent  = fmt(d.monolith_crate, 3) + ' C';

          // SOC card
          var soc = d.monolith_soc !== undefined ? d.monolith_soc : null;
          var socEl  = document.getElementById('mSOC');
          var socBar = document.getElementById('mSOCBar');
          if (soc !== null) {
            socEl.textContent = soc + ' %';
            socBar.style.width = soc + '%';
            socBar.style.background = socColour(soc);
          }

          // Session data
          document.getElementById('sessWh').textContent   = fmt(d.session_wh, 1);
          document.getElementById('sessAh').textContent   = fmt(d.session_ah, 2);
          document.getElementById('rampRate').textContent = d.ramp_rate_wps !== undefined
                                                            ? d.ramp_rate_wps : '—';

          // Target voltage buttons
          syncTargetVoltBtns(d.target_volt_dv);

          var ptSec = document.getElementById('ptSection');
          if (d.powertank_decided && d.powertank_present) {
            ptSec.style.display = 'block';
            document.getElementById('pV').textContent  = fmt(d.powertank_v,  1) + ' V';
            document.getElementById('pA').textContent  = d.powertank_a + ' A';
            document.getElementById('pAH').textContent = d.powertank_ah + ' Ah';
            document.getElementById('pT').textContent  = fmt(d.powertank_tmin, 0)
                                                 + ' / ' + fmt(d.powertank_tmax, 0) + ' \u00B0C';
            document.getElementById('pC').textContent  = fmt(d.powertank_crate, 3) + ' C';
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
              totalA += chargers[i].a;
              avgV += chargers[i].v;
              if (chargers[i].status > worstStatus) worstStatus = chargers[i].status;
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
          chargingEnabled = d.charging_enabled;
          updateEnableBtn();

          // Sync charger count button highlight from server
          var ccBtns = document.querySelectorAll('.cc-btn');
          for (var i = 0; i < ccBtns.length; i++) {
            ccBtns[i].className = 'cc-btn' + (i + 1 === d.charger_count ? ' active' : '');
          }

          document.getElementById('curPwr').textContent = d.current_power_w || 0;
          document.getElementById('tgtPwr').textContent = d.target_power_w  || 0;
          var phase = d.ramp_phase || 'cc';
          var modeEl = document.getElementById('chgMode');
          var modeLabels = {cc:'Absorption', cv:'Float', done:'Complete'};
          modeEl.textContent  = modeLabels[phase] || phase;
          modeEl.className    = 'phase-badge phase-' + phase;
          // Only update slider from server if user isn't dragging
          // (slider fires oninput while dragging, onchange on release)
          if (document.activeElement !== document.getElementById('pwrSlider')) {
            var slider = document.getElementById('pwrSlider');
            slider.value = d.target_power_w || slider.min;
            document.getElementById('sliderVal').textContent = slider.value;
          }

          document.getElementById('uptime').textContent = d.uptime_s + ' s';
          document.getElementById('rssi').textContent   = d.rssi + ' dBm';
          document.getElementById('ver').textContent    = d.version;
          document.getElementById('cpu0').textContent   = d.cpu0 !== undefined ? d.cpu0 + ' %' : '—';
          document.getElementById('cpu1').textContent   = d.cpu1 !== undefined ? d.cpu1 + ' %' : '—';
          document.getElementById('heap').textContent   = d.free_heap_kb !== undefined
                                                          ? fmt(d.free_heap_kb, 1) + ' kB' : '—';
          document.getElementById('footer').textContent =
            'Last update: ' + new Date().toLocaleTimeString();
        })
        .catch(function(){
          var b = document.getElementById('badge');
          b.textContent = 'OFFLINE'; b.className = 'badge stale';
        });
    }
    refresh();
    setInterval(refresh, 2000);
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
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>OTA Update</title>
  <style>
    body{font-family:Arial;text-align:center;background:#1a1a2e;color:#eee;padding:20px}
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
    nav{margin-bottom:14px}
    nav a{color:#aaa;font-size:0.85em;text-decoration:none}
    nav a:hover{color:#e94560}
  </style>
</head>
<body>
  <div class="box">
    <nav><a href="/">&#8592; Dashboard</a></nav>
    <h2>&#128190; Firmware Update</h2>
    <p>Select a compiled <code>.bin</code> file to upload.</p>
    <input type="file" id="bin" accept=".bin"><br>
    <button id="btn" onclick="upload()">Upload Firmware</button>
    <div id="bar-wrap"><div id="bar"></div></div>
    <div id="status"></div>
    <div class="ver" id="ver">Loading version...</div>
  </div>
  <script>
    fetch('/api/status')
      .then(function(r){ return r.json(); })
      .then(function(d){
        document.getElementById('ver').textContent = 'Current firmware: ' + d.version;
      });

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
      xhr.open('POST','/update',true);
      xhr.upload.onprogress=function(e){
        if(e.lengthComputable){
          var p=Math.round(e.loaded/e.total*100);
          bar.style.width=p+'%'; stat.innerText='Uploading... '+p+'%';
        }
      };
      xhr.onload=function(){
        if(xhr.status===200){
          stat.innerText='Success! Restarting...';
          bar.style.width='100%';
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
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Serial Log</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;
         padding:16px;display:flex;flex-direction:column;height:100vh}
    h1{color:#e94560;font-size:1.2em;margin-bottom:6px;flex-shrink:0}
    nav{font-size:0.85em;margin-bottom:8px;flex-shrink:0}
    nav a{color:#aaa;text-decoration:none;margin-right:14px}
    nav a:hover{color:#e94560}
    .toolbar{display:flex;gap:8px;margin-bottom:8px;flex-shrink:0}
    button{background:#e94560;color:#fff;border:none;padding:6px 14px;
           border-radius:5px;font-size:0.8em;cursor:pointer}
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
    <button id="btnPause" onclick="togglePause()">Pause</button>
    <button onclick="clearLog()">Clear</button>
    <span id="status">Connecting...</span>
  </div>
  <div id="log"></div>
  <script>
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

    function connect() {
      if (es) es.close();
      es = new EventSource('/api/log/stream');

      es.onopen = function() {
        statusEl.textContent = 'Connected';
      };

      es.onmessage = function(e) {
        // Each SSE message is one line of log output
        appendLine(e.data + '\n');
      };

      es.onerror = function() {
        statusEl.textContent = 'Reconnecting...';
        es.close();
        setTimeout(connect, 3000);
      };
    }

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

    connect();
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

// ---------------------------------------------------------------------------
// Route handlers
// ---------------------------------------------------------------------------

// GET /log — serves the log viewer page (protected)
void handleLogPage() {
  if (!requireAuth()) return;
  server.send_P(200, "text/html", HTML_LOG);
}

// GET /api/log/stream — opens an SSE connection (protected).
// The response headers are sent here; data is pushed from loop() via
// sseFlush() so the connection stays open without blocking handleClient().
void handleLogStream() {
  if (!requireAuth()) return;
  // Close any existing SSE client before accepting a new one
  if (sseActive) {
    sseClient.stop();
    sseActive = false;
  }

  sseClient  = server.client();
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
  if (sseReadPos >= logHead) return; // nothing new

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
}

// ---------------------------------------------------------------------------
// Authentication helper.
// Uses OTA credentials (SECRET_OTA_USER / SECRET_OTA_PASS) for all
// protected endpoints. Returns false and sends a 401 if auth fails.
// Dashboard (/) and /api/status are left open — they're read-only and
// needed for the charging UI to function without friction.
// ---------------------------------------------------------------------------
static bool requireAuth() {
  if (!server.authenticate(SECRET_OTA_USER, SECRET_OTA_PASS)) {
    server.requestAuthentication();
    return false;
  }
  return true;
}

void handleRoot() {
  server.send_P(200, "text/html", HTML_DASHBOARD);
}

// GET /settings — settings page (protected)
void handleSettingsPage() {
  if (!requireAuth()) return;
  server.send_P(200, "text/html", HTML_SETTINGS);
}

// GET /api/settings — returns current settings as JSON (protected)
void handleApiSettingsGet() {
  if (!requireAuth()) return;
  String ssid = preferences.getString("ssid", "");
  String aps  = preferences.getString("ap_ssid", String(apSSID));
  String mh   = String(mqttHost);
  String mu   = String(mqttUser);
  uint8_t cc  = ctrl.chargerCount;
  bool hasApPass = (strlen(apPass) > 0 ||
                    preferences.getString("ap_pass", "").length() > 0);

  // Escape strings for safe JSON embedding (handles " and \ characters)
  auto jsonEscape = [](const String& in) -> String {
    String out;
    out.reserve(in.length() + 8);
    for (unsigned int i = 0; i < in.length(); i++) {
      char c = in.charAt(i);
      if (c == '"' || c == '\\') out += '\\';
      out += c;
    }
    return out;
  };

  String jSsid = jsonEscape(ssid);
  String jAps  = jsonEscape(aps);
  String jMh   = jsonEscape(mh);
  String jMu   = jsonEscape(mu);

  char buf[550];
  snprintf(buf, sizeof(buf),
    "{\"ap_mode\":%s,\"wifi_ssid\":\"%s\",\"ap_ssid\":\"%s\",\"ap_pass_set\":%s,"
    "\"mqtt_host\":\"%s\",\"mqtt_port\":%d,\"mqtt_user\":\"%s\","
    "\"charger_count\":%d,\"ramp_rate_wps\":%d}",
    apIsBroadcasting() ? "true" : "false",
    jSsid.c_str(), jAps.c_str(), hasApPass ? "true" : "false",
    jMh.c_str(), (int)mqttPort, jMu.c_str(), (int)cc,
    (int)ctrl.rampStepW
  );
  server.send(200, "application/json", buf);
}

// POST /api/settings — saves settings to NVS (protected)
void handleApiSettingsPost() {
  if (!requireAuth()) return;
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"No body\"}");
    return;
  }
  String body = server.arg("plain");
  bool needRestart = false;

  // Helper: extract string value from JSON by key
  auto extractStr = [&](const char* key, char* out, size_t maxLen) -> bool {
    String search = String("\"") + key + "\"";
    int idx = body.indexOf(search);
    if (idx < 0) return false;
    int colon = body.indexOf(':', idx);
    if (colon < 0) return false;
    int q1 = body.indexOf('"', colon + 1);
    if (q1 < 0) return false;
    int q2 = body.indexOf('"', q1 + 1);
    if (q2 < 0) return false;
    String val = body.substring(q1 + 1, q2);
    val.toCharArray(out, maxLen);
    return true;
  };
  auto extractInt = [&](const char* key) -> int {
    String search = String("\"") + key + "\"";
    int idx = body.indexOf(search);
    if (idx < 0) return -1;
    int colon = body.indexOf(':', idx);
    if (colon < 0) return -1;
    return body.substring(colon + 1).toInt();
  };

  // WiFi credentials
  char tmp[65];
  if (extractStr("wifi_ssid", tmp, sizeof(tmp))) {
    if (strlen(tmp) == 0) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"SSID empty\"}");
      return;
    }
    preferences.putString("ssid", tmp);
    char tmpPass[65] = "";
    extractStr("wifi_pass", tmpPass, sizeof(tmpPass));
    preferences.putString("pass", tmpPass);
    LOG("[SETTINGS] WiFi credentials saved\n");
    needRestart = true;
  }

  // AP credentials
  if (extractStr("ap_ssid", tmp, sizeof(tmp))) {
    if (strlen(tmp) > 0) {
      preferences.putString("ap_ssid", tmp);
      strncpy(apSSID, tmp, sizeof(apSSID) - 1);
      apSSID[sizeof(apSSID) - 1] = '\0';
    }
    char tmpPass[65] = "";
    if (extractStr("ap_pass", tmpPass, sizeof(tmpPass))) {
      if (strlen(tmpPass) > 0 && strlen(tmpPass) < 8) {
        server.send(400, "application/json", "{\"ok\":false,\"error\":\"AP password must be 8+ chars\"}");
        return;
      }
      preferences.putString("ap_pass", tmpPass);
      strncpy(apPass, tmpPass, sizeof(apPass) - 1);
      apPass[sizeof(apPass) - 1] = '\0';
    }
    LOG("[SETTINGS] AP credentials saved: \"%s\"\n", apSSID);
  }

  // MQTT settings
  if (extractStr("mqtt_host", tmp, sizeof(tmp))) {
    preferences.putString("mqtt_host", tmp);
    strncpy(mqttHost, tmp, sizeof(mqttHost) - 1);
    mqttHost[sizeof(mqttHost) - 1] = '\0';

    int port = extractInt("mqtt_port");
    if (port > 0 && port <= 65535) {
      preferences.putUShort("mqtt_port", (uint16_t)port);
      mqttPort = (uint16_t)port;
    }

    char tmpUser[33] = "";
    if (extractStr("mqtt_user", tmpUser, sizeof(tmpUser))) {
      preferences.putString("mqtt_user", tmpUser);
      strncpy(mqttUser, tmpUser, sizeof(mqttUser) - 1);
      mqttUser[sizeof(mqttUser) - 1] = '\0';
    }
    char tmpPass[65] = "";
    if (extractStr("mqtt_pass", tmpPass, sizeof(tmpPass))) {
      preferences.putString("mqtt_pass", tmpPass);
      strncpy(mqttBrokerPass, tmpPass, sizeof(mqttBrokerPass) - 1);
      mqttBrokerPass[sizeof(mqttBrokerPass) - 1] = '\0';
    }
    LOG("[SETTINGS] MQTT settings saved: %s:%d\n", mqttHost, mqttPort);
    // Force MQTT reconnect with new settings
    mqttClient.disconnect();
  }

  // Charger count
  int cc = extractInt("charger_count");
  if (cc >= 1 && cc <= 4) {
    if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      ctrl.chargerCount = (uint8_t)cc;
      xSemaphoreGive(controlMutex);
    }
    preferences.putUChar("charger_count", (uint8_t)cc);
    LOG("[SETTINGS] Charger count: %d\n", cc);
  }

  server.send(200, "application/json", "{\"ok\":true}");

  if (needRestart) {
    LOG("[SETTINGS] Restarting for WiFi changes...\n");
    delay(1500);
    ESP.restart();
  }
}

// Legacy POST /save — redirect to new settings API for backwards compatibility
void handleSave() {
  if (server.hasArg("ssid")) {
    String newSSID = server.arg("ssid");
    String newPass = server.arg("pass");
    if (!newSSID.isEmpty()) {
      preferences.putString("ssid", newSSID);
      preferences.putString("pass", newPass);
      server.send(200, "text/html",
        "<div style='font-family:Arial;text-align:center;padding:40px;"
        "background:#1a1a2e;color:#eee'>"
        "<h2 style='color:#e94560'>Saved!</h2><p>Restarting...</p></div>");
      delay(2000);
      ESP.restart();
      return;
    }
  }
  server.send(400, "text/plain", "Missing SSID");
}

// GET /api/status — JSON endpoint polled by the dashboard every 2 s
void handleApiStatus() {
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

  // Calculate SOC from monolith voltage
  int monolithSoc = calcSocFromVoltage(liveSnap.monolithVoltageDv);

  // Snapshot session data
  float sessWh = session.energyWh;
  float sessAh = session.chargeAh;

  // Fixed fields — use a stack buffer for the bulk of the response
  char buf[950];
  snprintf(buf, sizeof(buf),
    "{"
      "\"fresh\":%s,"
      "\"ap_mode\":%s,"
      "\"monolith_v\":%.1f,"
      "\"monolith_a\":%.0f,"
      "\"monolith_ah\":%.0f,"
      "\"monolith_tmin\":%d,"
      "\"monolith_tmax\":%d,"
      "\"monolith_crate\":%.3f,"
      "\"monolith_soc\":%d,"
      "\"powertank_present\":%s,"
      "\"powertank_decided\":%s,"
      "\"powertank_v\":%.1f,"
      "\"powertank_a\":%.0f,"
      "\"powertank_ah\":%.0f,"
      "\"powertank_tmin\":%d,"
      "\"powertank_tmax\":%d,"
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
      "\"charger_count\":%d,"
      "\"charging_enabled\":%s,"
      "\"target_power_w\":%d,"
      "\"current_power_w\":%d,"
      "\"chargers\":[",
    liveSnap.dataFresh         ? "true" : "false",
    apIsBroadcasting()         ? "true" : "false",
    liveSnap.monolithVoltageDv  / 10.0f,
    (float)liveSnap.monolithAmps,
    (float)liveSnap.monolithAH,
    (int)liveSnap.monolithMinTemp,
    (int)liveSnap.monolithMaxTemp,
    liveSnap.monolithMaxCRate   / 10.0f,
    monolithSoc,
    liveSnap.powerTankPresent  ? "true" : "false",
    liveSnap.powerTankDecided  ? "true" : "false",
    liveSnap.powerTankVoltageDv / 10.0f,
    (float)liveSnap.powerTankAmps,
    (float)liveSnap.powerTankAH,
    (int)liveSnap.powerTankMinTemp,
    (int)liveSnap.powerTankMaxTemp,
    liveSnap.powerTankMaxCRate  / 10.0f,
    sessWh,
    sessAh,
    (int)ctrl.rampStepW,
    (int)ctrl.targetVoltDv,
    millis() / 1000UL,
    (int)WiFi.RSSI(),
    (long long)VERSION,
    (int)statsSnap.load0,
    (int)statsSnap.load1,
    statsSnap.freeHeap / 1024.0f,
    chargerSnap.heartbeatOk ? "true" : "false",
    g_rampPhase == 1 ? "cv" : g_rampPhase == 2 ? "done" : "cc",
    (int)ctrl.chargerCount,
    ctrl.enabled ? "true" : "false",
    (int)ctrl.targetPowerW,
    (int)ctrl.currentPowerW
  );

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
  response += "]}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", response);
}

// POST /api/control — sets target power, enabled state, charger count,
//                     ramp rate, and target voltage (protected)
// Body: {"target_w": 3300, "enabled": true, "charger_count": 3,
//        "ramp_rate_wps": 50, "target_volt_dv": 1100, "reset_session": true}
void handleApiControl() {
  if (!requireAuth()) return;
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "No body");
    return;
  }
  String body = server.arg("plain");

  // Minimal hand-rolled JSON extraction — avoids ArduinoJson heap cost
  int tw = -1;
  int idx = body.indexOf("\"target_w\"");
  if (idx >= 0) {
    int colon = body.indexOf(':', idx);
    if (colon >= 0) tw = body.substring(colon + 1).toInt();
  }

  bool en = ctrl.enabled;
  idx = body.indexOf("\"enabled\"");
  if (idx >= 0) {
    int colon = body.indexOf(':', idx);
    if (colon >= 0) {
      String rest = body.substring(colon + 1);
      rest.trim();
      en = rest.startsWith("true");
    }
  }

  int cc = -1;
  idx = body.indexOf("\"charger_count\"");
  if (idx >= 0) {
    int colon = body.indexOf(':', idx);
    if (colon >= 0) cc = body.substring(colon + 1).toInt();
  }

  int rr = -1;
  idx = body.indexOf("\"ramp_rate_wps\"");
  if (idx >= 0) {
    int colon = body.indexOf(':', idx);
    if (colon >= 0) rr = body.substring(colon + 1).toInt();
  }

  int tvd = -1;
  idx = body.indexOf("\"target_volt_dv\"");
  if (idx >= 0) {
    int colon = body.indexOf(':', idx);
    if (colon >= 0) tvd = body.substring(colon + 1).toInt();
  }

  bool resetSession = false;
  idx = body.indexOf("\"reset_session\"");
  if (idx >= 0) {
    int colon = body.indexOf(':', idx);
    if (colon >= 0) {
      String rest = body.substring(colon + 1);
      rest.trim();
      resetSession = rest.startsWith("true");
    }
  }

  if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    if (tw >= 0) ctrl.targetPowerW = (uint16_t)constrain(tw, 0, 13200);
    ctrl.enabled = en;
    if (!en) ctrl.currentPowerW = 0; // immediate stop on disable
    if (cc >= 1 && cc <= 4) {
      ctrl.chargerCount = (uint8_t)cc;
      preferences.putUChar("charger_count", (uint8_t)cc);
    }
    if (rr >= 10 && rr <= 500) {
      ctrl.rampStepW = (uint16_t)rr;
      preferences.putUShort("ramp_step_w", (uint16_t)rr);
    }
    if (tvd >= TARGET_VOLT_PRESETS[0].dv && tvd <= (int)MAX_CHARGE_VOLTAGE_DV) {
      ctrl.targetVoltDv = (uint16_t)tvd;
    }
    xSemaphoreGive(controlMutex);
  }

  if (resetSession) {
    session.energyWh = 0.0f;
    session.chargeAh = 0.0f;
    session.startMs  = millis();
    LOG("[CTRL] Session reset\n");
  }

  LOG("[CTRL] target=%dW enabled=%s chargers=%d ramp=%dW/s tgtV=%ddV\n",
      tw, en ? "true" : "false",
      cc > 0 ? cc : (int)ctrl.chargerCount,
      rr > 0 ? rr : (int)ctrl.rampStepW,
      tvd > 0 ? tvd : (int)ctrl.targetVoltDv);
  server.send(200, "application/json", "{\"ok\":true}");
}

// GET /update — OTA page, HTTP Basic Auth
void handleOTAGet() {
  if (!server.authenticate(SECRET_OTA_USER, SECRET_OTA_PASS)) {
    return server.requestAuthentication();
  }
  server.send_P(200, "text/html", HTML_OTA);
}

// POST /update completion handler
void handleOTAPost() {
  if (!server.authenticate(SECRET_OTA_USER, SECRET_OTA_PASS)) {
    return server.requestAuthentication();
  }
  if (Update.hasError()) {
    server.send(500, "text/plain",
                String("Update failed: ") + Update.errorString());
    LOG("[OTA] FAILED: %s\n", Update.errorString());
  } else {
    server.send(200, "text/plain", "OK");
    LOG("[OTA] Success. Restarting...\n");
    delay(500);
    ESP.restart();
  }
}

// POST /update body handler — called by WebServer as multipart chunks arrive.
// Auth MUST be checked here — WebServer calls this for each chunk BEFORE
// the POST completion handler (handleOTAPost) runs.
void handleOTAUpload() {
  if (!server.authenticate(SECRET_OTA_USER, SECRET_OTA_PASS)) return;
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    LOG("[OTA] Start: field='%s' file='%s'\n",
                  upload.name.c_str(), upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      LOG("[OTA] begin() error: %s\n", Update.errorString());
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      LOG("[OTA] write() error: %s\n", Update.errorString());
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      LOG("[OTA] Done. %u bytes written.\n", upload.totalSize);
    } else {
      LOG("[OTA] end() error: %s\n", Update.errorString());
    }
  }
}

// ---------------------------------------------------------------------------
// BOOT button (GPIO 0) handler
//
// Polled from loop(). Two recognised hold patterns:
//   ≥ 5 s   on release  → wipe saved WiFi credentials only, restart into AP
//                         mode for re-provisioning. Keeps AP/MQTT/charger-count
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
      LOG("[BTN] BOOT held %lus → clearing WiFi creds, restarting into AP mode\n",
          heldMs / 1000UL);
      preferences.remove("ssid");
      preferences.remove("pass");
      delay(300);
      ESP.restart();
    } else if (heldMs >= 1000UL) {
      // 1–5 s window: log it so the user sees their press registered, but no action.
      LOG("[BTN] BOOT pressed %lums (no action — hold ≥5s for AP reset, ≥10s for factory)\n", heldMs);
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

void startAPMode() {
  WiFi.disconnect(true);
  // Hostname must be set BEFORE softAP() so it goes into the AP DHCP server's
  // client-name advertisement (clients that join the AP see this name).
  WiFi.softAPsetHostname(MDNS_HOSTNAME);
  WiFi.softAP(apSSID, apPass);
  LOG("[AP] Started \"%s\" — IP: %s\n", apSSID, WiFi.softAPIP().toString().c_str());
  startMdns();
}

// Attempt connection using SECRET_MQTT_SSID / SECRET_MQTT_PASS.
// Credentials are intentionally NOT written to preferences here —
// secrets remain a read-only bootstrap; the /save page is the only
// path that persists credentials.
void trySecretsConnect() {
  const char* ssid = SECRET_MQTT_SSID;
  const char* pass = SECRET_MQTT_PASS;
  if (ssid == nullptr || strlen(ssid) == 0) {
    LOG("[WIFI] Secrets SSID empty, starting AP mode.\n");
    startAPMode();
    currentState = STATE_SETUP_MODE;
    return;
  }
  LOG("[WIFI] No saved credentials. Trying secrets SSID \"%s\"...\n", ssid);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(MDNS_HOSTNAME);
  WiFi.begin(ssid, pass);
  wifiSource         = WIFI_SRC_SECRETS;
  wifiConnectStartMs = millis();
  currentState       = STATE_CONNECTING;
}

// Bring up AP+STA mode and start the background STA retry loop.
// Pulls credentials from NVS first, then SECRET_MQTT_SSID/PASS as fallback.
// If neither is available there's nothing to retry — drop to AP-only setup mode.
void enterApRetrying() {
  String s = preferences.getString("ssid", "");
  String p = preferences.getString("pass", "");
  if (s.length() == 0 && strlen(SECRET_MQTT_SSID) > 0) {
    s = SECRET_MQTT_SSID;
    p = SECRET_MQTT_PASS;
  }
  if (s.length() == 0) {
    LOG("[WIFI] No STA creds to retry — staying in AP-only setup mode\n");
    startAPMode();
    currentState = STATE_SETUP_MODE;
    return;
  }
  s.toCharArray(retryStaSsid, sizeof(retryStaSsid));
  p.toCharArray(retryStaPass, sizeof(retryStaPass));

  // AP+STA: both interfaces share the radio. When STA associates, the SoftAP
  // is force-moved to the STA's channel (clients on the AP may briefly drop).
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPsetHostname(MDNS_HOSTNAME);
  WiFi.softAP(apSSID, apPass);
  WiFi.setHostname(MDNS_HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(retryStaSsid, retryStaPass);
  startMdns();
  lastStaRetryAt = millis();
  currentState = STATE_AP_RETRYING;
  LOG("[WIFI] AP+STA retry mode — AP \"%s\" up at %s, STA → \"%s\"\n",
      apSSID, WiFi.softAPIP().toString().c_str(), retryStaSsid);
}

void monitorWifiStatus() {
  unsigned long now = millis();

  if (currentState == STATE_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      LOG("[WIFI] Connected (%s). IP: %s\n",
                    wifiSource == WIFI_SRC_SECRETS ? "secrets" : "prefs",
                    WiFi.localIP().toString().c_str());
      // Capture hostname for MQTT client ID and topic prefix
      const char* hn = WiFi.getHostname();
      if (hn && strlen(hn) > 0)
        snprintf(mqttHostname, sizeof(mqttHostname), "%s", hn);
      startMdns();
      currentState = STATE_CONNECTED;
      return;
    }
    // Not yet connected — check whether we've exceeded the timeout.
    if (now - wifiConnectStartMs >= wifiConnectTimeoutMs) {
      if (wifiSource == WIFI_SRC_PREFS) {
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
      LOG("[WIFI] STA dropped — switching to AP+STA retry mode\n");
      enterApRetrying();
    }

  } else if (currentState == STATE_AP_RETRYING) {
    // Track STA up/down transitions; AP stays up regardless.
    static bool lastStaUp = false;
    bool staUp = (WiFi.status() == WL_CONNECTED);

    if (staUp && !lastStaUp) {
      LOG("[WIFI] STA reconnected. IP: %s (AP stays up)\n",
          WiFi.localIP().toString().c_str());
      const char* hn = WiFi.getHostname();
      if (hn && strlen(hn) > 0)
        snprintf(mqttHostname, sizeof(mqttHostname), "%s", hn);
      // Re-bind mDNS — STA-side service registration needs the new IP.
      startMdns();
    } else if (!staUp && lastStaUp) {
      LOG("[WIFI] STA dropped again — driver auto-reconnect will retry\n");
    }
    lastStaUp = staUp;

    // Belt-and-braces: nudge the driver every staRetryIntervalMs while STA is down.
    // Arduino-ESP32 auto-reconnects on its own, but some failure modes leave the
    // STA stuck disconnected. WiFi.reconnect() kicks it again with the same creds.
    if (!staUp && (now - lastStaRetryAt) >= staRetryIntervalMs) {
      LOG("[WIFI] Nudging STA reconnect to \"%s\"...\n", retryStaSsid);
      WiFi.reconnect();
      lastStaRetryAt = now;
    }
  }
  // STATE_SETUP_MODE: nothing to do — wait for /save to write creds and restart.
}

// True whenever the SoftAP is currently broadcasting (covers both the
// "no creds" setup mode and the "creds present, retrying" recovery mode).
// Used by the dashboard's ap_mode JSON field so the UI can hint that the
// AP fallback is reachable.
static inline bool apIsBroadcasting() {
  return currentState == STATE_SETUP_MODE || currentState == STATE_AP_RETRYING;
}

// ---------------------------------------------------------------------------
// Setup & Loop
// ---------------------------------------------------------------------------

void setup() {
  // logBegin() initialises hardware serial and creates the ring buffer mutex.




  logBegin(115200);
  LOG("\n[BOOT] Supercharger firmware %lld\n", (long long)VERSION);

  // BOOT button (GPIO 0). Strapping pin — safe to configure as input AFTER
  // boot completes. Internal pullup is enough; the button shorts to GND.
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  preferences.begin("wifi-config", false);
  String savedSSID = preferences.getString("ssid", "");
  String savedPass = preferences.getString("pass", "");

  // Load persistent charger count (default 3 if not set)
  uint8_t savedCC = (uint8_t)preferences.getUChar("charger_count", 3);
  if (savedCC >= 1 && savedCC <= 4) ctrl.chargerCount = savedCC;
  LOG("[BOOT] Charger count: %d (from NVS)\n", ctrl.chargerCount);

  // Load AP settings from NVS, fallback to secrets, then defaults
  {
    String s = preferences.getString("ap_ssid", "");
    if (s.length() > 0) {
      s.toCharArray(apSSID, sizeof(apSSID));
    } else if (strlen(SECRET_SSID) > 0) {
      strncpy(apSSID, SECRET_SSID, sizeof(apSSID) - 1);
      apSSID[sizeof(apSSID) - 1] = '\0';
    }
    // else keep default "Supercharger"
    String p = preferences.getString("ap_pass", "");
    if (p.length() > 0) {
      p.toCharArray(apPass, sizeof(apPass));
    } else if (strlen(SECRET_PASS) > 0) {
      strncpy(apPass, SECRET_PASS, sizeof(apPass) - 1);
      apPass[sizeof(apPass) - 1] = '\0';
    }
    LOG("[BOOT] AP: \"%s\"\n", apSSID);
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

  if (savedSSID.length() > 0) {
    // Priority 1: saved preferences
    LOG("[WIFI] Connecting to saved SSID \"%s\"...\n", savedSSID.c_str());
    // Hostname must be set BEFORE begin() so it's included in the DHCP DISCOVER.
    // Routers that auto-register DHCP client names into local DNS (OpenWRT,
    // pfSense, UniFi, etc.) will then expose the controller as
    // supercharger.<your-domain> — independent of mDNS / .local resolution.
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(MDNS_HOSTNAME);
    WiFi.begin(savedSSID.c_str(), savedPass.c_str());
    wifiSource         = WIFI_SRC_PREFS;
    wifiConnectStartMs = millis();
    currentState       = STATE_CONNECTING;
  } else if (strlen(SECRET_MQTT_SSID) > 0) {
    // Priority 2: compile-time secrets (not persisted)
    trySecretsConnect();
  } else {
    // Priority 3: no credentials at all — AP setup mode
    LOG("[WIFI] No credentials available. Starting AP mode.\n");
    startAPMode();
    currentState = STATE_SETUP_MODE;
  }

  server.on("/",                HTTP_GET,  handleRoot);
  server.on("/save",            HTTP_POST, handleSave);
  server.on("/settings",        HTTP_GET,  handleSettingsPage);
  server.on("/api/settings",    HTTP_GET,  handleApiSettingsGet);
  server.on("/api/settings",    HTTP_POST, handleApiSettingsPost);
  server.on("/api/status",      HTTP_GET,  handleApiStatus);
  server.on("/api/control",     HTTP_POST, handleApiControl);
  server.on("/api/log/stream",  HTTP_GET,  handleLogStream);
  server.on("/log",             HTTP_GET,  handleLogPage);
  server.on("/update",          HTTP_GET,  handleOTAGet);
  server.on("/update",          HTTP_POST, handleOTAPost, handleOTAUpload);

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
}

void loop() {
  unsigned long now = millis();

  if (now - lastWifiCheck >= wifiCheckInterval) {
    lastWifiCheck = now;
    monitorWifiStatus();
  }

  server.handleClient();
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
    unsigned long* slot = nullptr;
    const char*    name = nullptr;

    if      (id == 0x388) { slot = &lastLog388; name = "BMS_CELL_VOLTAGE  (0x388)"; }
    else if (id == 0x288) { slot = &lastLog288; name = "BMS_PACK_CONFIG   (0x288)"; }
    else if (id == 0x488) { slot = &lastLog488; name = "BMS_PACK_TEMP_DATA(0x488)"; }
    else if (id == 0x508) { slot = &lastLog508; name = "BMS_PACK_TIME     (0x508)"; }
    else if (id == 0x389) { slot = &lastLog389; name = "BMS1_CELL_VOLTAGE (0x389)"; }
    else if (id == 0x289) { slot = &lastLog289; name = "BMS1_PACK_CONFIG  (0x289)"; }
    else if (id == 0x489) { slot = &lastLog489; name = "BMS1_PACK_TEMP    (0x489)"; }
    else if (id == 0x509) { slot = &lastLog509; name = "BMS1_PACK_TIME    (0x509)"; }
    else if (id == BMS_PACK_STATS.id)    { slot = &lastLog308; name = "BMS_PACK_STATS"; }

    unsigned long nowMs = millis();
    if (slot && (nowMs - *slot >= 5000)) {
      *slot = nowMs;
      LOG("[CAN] %s len=%d  %02X %02X %02X %02X %02X %02X %02X %02X\n",
          name, (int)len,
          buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
    }
  }

  unsigned long now = millis();

  if (xSemaphoreTake(liveMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    return; // drop frame rather than block the task
  }

  // --- Monolith (BMS0) ---
  if (zeroDecoder.hasMonolithVoltage(id)) {
    // BMS_CELL_VOLTAGE (0x388): voltage in bytes 3-6 (units: 0.001 V → store as dV)
    // NOTE: bytes 0-1 are NOT sagAdjust — sagAdjust lives in BMS_PACK_CONFIG (0x288)
    live.monolithVoltageDv = zeroDecoder.voltage(len, buf) / 100;
    if (!live.dataFresh) {
      live.dataFresh   = true;
      live.bms0FirstMs = now;
    }
  }
  else if (zeroDecoder.hasMonolithAmps(id)) {
    live.monolithAmps = zeroDecoder.amps(len, buf);
  }
  else if (zeroDecoder.hasMonolithPackConfig(id)) {
    // BMS_PACK_CONFIG (0x288): sagAdjust in bytes 0-1, AH candidate in bytes 5-6
    live.monolithSagAdjDv = zeroDecoder.sagAdjust(len, buf);
    live.monolithAH       = zeroDecoder.AH(len, buf);
  }
  else if (zeroDecoder.hasMonolithMaxCRate(id)) {
    // BMS_PACK_TIME (0x508) — C-rate assumed bytes 4-5; verify via raw log
    live.monolithMaxCRate = zeroDecoder.maxCRate(len, buf);
  }
  else if (id == BMS_PACK_TEMP_DATA.id) {
    // BMS_PACK_TEMP_DATA (0x488) — lowestTemp byte 0, highestTemp byte 1; verify via raw log
    live.monolithMaxTemp = zeroDecoder.highestTemp(len, buf);
    live.monolithMinTemp = zeroDecoder.lowestTemp(len, buf);
  }

  // --- PowerTank (BMS1) ---
  else if (zeroDecoder.hasPowerTankVoltage(id)) {
    // BMS1_CELL_VOLTAGE (0x389): same layout as BMS0
    live.powerTankVoltageDv = zeroDecoder.voltage(len, buf) / 100;
    if (!live.powerTankPresent) {
      live.powerTankPresent = true;
      live.powerTankDecided = true;
      live.bms1FirstMs      = now;
    }
  }
  else if (zeroDecoder.hasPowerTankAmps(id)) {
    live.powerTankAmps = zeroDecoder.amps(len, buf);
  }
  else if (zeroDecoder.hasPowerTankPackConfig(id)) {
    // BMS1_PACK_CONFIG (0x289): sagAdjust bytes 0-1, AH bytes 5-6
    live.powerTankSagAdjDv = zeroDecoder.sagAdjust(len, buf);
    live.powerTankAH       = zeroDecoder.AH(len, buf);
  }
  else if (zeroDecoder.hasPowerTankMaxCRate(id)) {
    // BMS1_PACK_TIME (0x509) — C-rate assumed bytes 4-5; verify via raw log
    live.powerTankMaxCRate = zeroDecoder.maxCRate(len, buf);
  }
  else if (id == BMS1_PACK_TEMP_DATA.id) {
    // BMS1_PACK_TEMP_DATA (0x489) — same layout as BMS0
    live.powerTankMaxTemp = zeroDecoder.highestTemp(len, buf);
    live.powerTankMinTemp = zeroDecoder.lowestTemp(len, buf);
  }

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
  if (rc != CAN_OK) {
    unsigned long now = millis();
    if (now - lastErrLogMs >= 5000) {
      // Error 6 = TX buffer timeout (no ACK from bus — expected with nothing connected)
      // Error 7 = send msg timeout
      LOG("[MCP] Heartbeat TX error %d (no charger connected?)\n", rc);
      lastErrLogMs = now;
    }

    // If the MCP2515 has gone bus-off (>256 consecutive errors), reset it.
    if (mcpCan.checkError() == CAN_CTRLERROR) {
#if MCP_LISTEN_ONLY
      // Bus-off shouldn't occur in listen-only mode — log but don't spam
      static unsigned long lastBusOffLog = 0;
      if (millis() - lastBusOffLog >= 10000) {
        lastBusOffLog = millis();
        LOG("[MCP] Bus-off in listen-only mode — unexpected, check wiring\n");
      }
#else
      LOG("[MCP] Bus-off detected — resetting MCP2515\n");
      mcpCan.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ);
      mcpCan.setMode(MCP_NORMAL);
#endif
    }
  } else {
    lastErrLogMs = 0; // reset so next error after a working period logs immediately
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

  uint16_t voltDv = ((uint16_t)buf[0] << 8) | buf[1]; // 0.1 V units
  uint16_t ampsDa = ((uint16_t)buf[2] << 8) | buf[3]; // 0.1 A units
  uint8_t  status = (len >= 5) ? buf[4] : 0;

  if (xSemaphoreTake(chargerMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    ChargerUnit& c = chargerBus.chargers[nibble];
    bool wasPresent = c.present;
    c.present    = true;
    c.voltDv     = voltDv;
    c.ampsDa     = ampsDa;
    c.status     = status;
    c.lastSeenMs = millis();

    // Recount active chargers whenever a new one appears
    if (!wasPresent) {
      uint8_t count = 0;
      for (int i = 0; i < MAX_CHARGERS; i++) {
        if (chargerBus.chargers[i].present) count++;
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

  unsigned long lastHeartbeatMs = 0;
  // Rate-limit raw frame logging in listen-only mode
  static unsigned long lastRawLogMs = 0;
  static uint32_t      frameCount   = 0;

  for (;;) {
    unsigned long now = millis();

#if !MCP_LISTEN_ONLY
    // --- Heartbeat (normal mode only) ---
    if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
      lastHeartbeatMs = now;
      sendHeartbeat();
    }
#endif

    // --- Drain incoming frames ---
    while (mcpCan.checkReceive() == CAN_MSGAVAIL) {
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
// Runs every 1 s. Steps currentPowerW toward targetPowerW by at most
// ctrl.rampStepW (default 50 W) per tick, then computes the voltage/current
// command and writes it to chargerBus under chargerMutex.
//
// Voltage selection:
//   Commanded voltage = raw monolithVoltageDv + sagAdjDv (sag compensation
//   from BMS_PACK_CONFIG 0x288 bytes 0-1, in dV). sagAdjDv is the BMS estimate
//   of how much the voltage is sagging under load — adding it gives the
//   open-circuit equivalent. PowerTankVoltageDv is added if present.
//   Result is clamped to the lower of ctrl.targetVoltDv and MAX_CHARGE_VOLTAGE_DV.
//
// Current calculation:
//   I = P / V  (W / V = A).  Stored as dA (0.1 A units) for the heartbeat.
//   A floor of 1 dA is applied so the charger doesn't see a zero-current
//   command while ramping through very low power steps.
//
// Session energy is accumulated each tick: Wh += P_W / 3600,  Ah += I_A / 3600
// ---------------------------------------------------------------------------

void rampTask(void* /*pvParameters*/) {
  LOG("[RAMP] rampTask running on core %d\n", xPortGetCoreID());
  session.startMs = millis();

  // CC / CV / DONE state machine
  // ─────────────────────────────
  // PHASE_CC  : Constant-current charge; voltage-based and hot cutbacks applied.
  // PHASE_CV  : Pack voltage has reached the ceiling; charger stays on at the
  //             ceiling voltage to fully absorb charge.  Transitions to DONE
  //             when actual delivered current drops below CV_TERM_A (≈ C/20)
  //             OR after CV_HOLD_MS elapses (safety timeout).
  // PHASE_DONE: Fully charged; charger stopped.  Resets to CC only after the
  //             pack has discharged at least 2 V below the ceiling.
  enum RampPhase : uint8_t { PHASE_CC = 0, PHASE_CV = 1, PHASE_DONE = 2 };
  static RampPhase phase      = PHASE_CC;
  static unsigned long cvMs   = 0;   // millis() when CV phase was entered
  static uint16_t cvAmpsDa    = 0;   // per-charger amps ceiling during CV

  const unsigned long CV_HOLD_MS = 20UL * 60UL * 1000UL; // 20 min timeout
  const unsigned long CV_MIN_MS  = 120000UL;              // don't terminate CV before 2 min
  const float         CV_TERM_A  = 2.0f;                  // stop CV when total amps < 2 A

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000)); // 1 s ramp tick

    // ── Control snapshot ──────────────────────────────────────────────────────
    uint16_t target     = 0;
    uint16_t current    = 0;
    bool     enabled    = false;
    uint8_t  nChargers  = 1;
    uint16_t rampStep   = DEFAULT_RAMP_STEP_W;
    uint16_t targetVolt = MAX_CHARGE_VOLTAGE_DV;

    if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      target     = ctrl.targetPowerW;
      current    = ctrl.currentPowerW;
      enabled    = ctrl.enabled;
      nChargers  = ctrl.chargerCount;
      rampStep   = ctrl.rampStepW;
      targetVolt = ctrl.targetVoltDv;
      if (nChargers < 1) nChargers = 1;
      xSemaphoreGive(controlMutex);
    }

    // Disabled: reset to CC, stop charger, skip rest of tick
    if (!enabled) {
      phase = PHASE_CC;
      if (xSemaphoreTake(chargerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        chargerBus.cmdVoltDv = 0;
        chargerBus.cmdAmpsDa = 0;
        chargerBus.cmdStart  = false;
        xSemaphoreGive(chargerMutex);
      }
      continue;
    }

    // ── Live pack snapshot ────────────────────────────────────────────────────
    long  monolithDv = 0;
    short sagAdjDv   = 0;
    long  ptDv       = 0;
    bool  ptPresent  = false;
    short monolithAH = 114; // nominal fallback
    short maxTemp    = -127;

    if (xSemaphoreTake(liveMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      monolithDv = live.monolithVoltageDv;
      sagAdjDv   = live.monolithSagAdjDv;
      ptDv       = live.powerTankVoltageDv;
      ptPresent  = live.powerTankPresent;
      if (live.monolithAH > 0) monolithAH = live.monolithAH;
      maxTemp    = live.monolithMaxTemp;
      xSemaphoreGive(liveMutex);
    }

    // Raw (unclamped) pack voltage — all cutback decisions use this.
    // sagAdjDv is the BMS sag-compensation offset (additive, not a replacement).
    long rawPackDv = monolithDv + (sagAdjDv > 0 ? (long)sagAdjDv : 0L);
    if (ptPresent && ptDv > 0) rawPackDv += ptDv;

    uint16_t voltCeiling = min(targetVolt, MAX_CHARGE_VOLTAGE_DV);
    const long hyst = 10; // 1.0 V in dV

    // ── Phase transitions ─────────────────────────────────────────────────────
    // DONE → CC : pack has discharged ≥ 2 V below ceiling (e.g., bike ridden)
    if (phase == PHASE_DONE && rawPackDv > 0 &&
        rawPackDv <= (long)(voltCeiling - hyst * 2)) {
      phase = PHASE_CC;
      LOG("[RAMP] DONE→CC: pack dropped to %ld dV\n", rawPackDv);
    }
    // CV → CC : voltage sag under load (bike in use) — re-enter CC to top up
    if (phase == PHASE_CV && rawPackDv > 0 &&
        rawPackDv < (long)(voltCeiling - hyst)) {
      phase = PHASE_CC;
      LOG("[RAMP] CV→CC: pack dropped to %ld dV (load sag)\n", rawPackDv);
    }
    // CC → CV : voltage target reached
    if (phase == PHASE_CC && rawPackDv > 0 &&
        rawPackDv >= (long)voltCeiling) {
      phase = PHASE_CV;
      cvMs  = millis();
      // CV amps ceiling: at least as generous as last CC command so the charger
      // doesn't lose current mid-transition; cap at C/5 per charger.
      long   clampedDv    = min(rawPackDv, (long)voltCeiling);
      float  packV        = clampedDv / 10.0f;
      float  ccAmpsPerCh  = (current > 0 && packV > 0 && nChargers > 0)
                              ? (current / packV / nChargers) : 0.0f;
      float  cvMaxPerCh   = (float)monolithAH * 0.2f / nChargers; // C/5
      cvAmpsDa = (uint16_t)(max(ccAmpsPerCh, cvMaxPerCh) * 10.0f);
      cvAmpsDa = max(cvAmpsDa, (uint16_t)10); // floor 1 A/charger
      LOG("[RAMP] CC→CV @ %ld dV (ceil %d dV), cvAmpsDa=%d/ch\n",
          rawPackDv, voltCeiling, (int)cvAmpsDa);
    }

    // ── Power cutback limits (CC phase only) ──────────────────────────────────
    float packAH = (monolithAH > 0) ? (float)monolithAH : 114.0f;
    float voltV  = (rawPackDv > 0) ? rawPackDv / 10.0f : voltCeiling / 10.0f;

    uint32_t voltCbK  = find_cutback((int)rawPackDv, CUTBACK_AT_OR_ABOVE, VOLTAGE_CUTBACK);
    uint32_t voltPwrW = (voltCbK == UINT32_MAX) ? UINT32_MAX
                       : (uint32_t)(voltCbK / 1000.0f * packAH * voltV);

    uint32_t hotCbK   = find_cutback((int)maxTemp,   CUTBACK_AT_OR_ABOVE, HOT_CUTBACK);
    uint32_t hotPwrW  = (hotCbK  == UINT32_MAX) ? UINT32_MAX
                       : (uint32_t)(hotCbK  / 1000.0f * packAH * voltV);

    // Effective CC target (cutbacks only apply in CC; CV/DONE ramp current to 0)
    uint32_t powerLimit = (uint32_t)target;
    if (phase == PHASE_CC) {
      if (voltPwrW != UINT32_MAX) powerLimit = min(powerLimit, voltPwrW);
      if (hotPwrW  != UINT32_MAX) powerLimit = min(powerLimit, hotPwrW);
    } else {
      powerLimit = 0; // CV/DONE: ramp CC current display to 0
    }
    uint16_t effectiveTarget = (uint16_t)min(powerLimit, (uint32_t)UINT16_MAX);

    // ── Ramp step ─────────────────────────────────────────────────────────────
    bool cutbackActive = (phase == PHASE_CC) && (effectiveTarget < target);
    if (current < effectiveTarget) {
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
    if (packDv > 0 && current > 0) {
      float v        = packDv / 10.0f;
      totalAmpsA     = current / v;
      float perCh    = totalAmpsA / nChargers;
      cmdAmpsDa      = (uint16_t)max(1.0f, perCh * 10.0f);
    }

    // Session energy (CC phase only — CV energy accumulated below with actual amps)
    if (phase == PHASE_CC && packDv > 0 && current > 0 && cmdAmpsDa > 0) {
      session.energyWh += current / 3600.0f;
      session.chargeAh += totalAmpsA / 3600.0f;
    }

    // ── Charger bus update + CV phase management ──────────────────────────────
    if (xSemaphoreTake(chargerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      bool chargersPresent = (chargerBus.chargerCount > 0);

      if (phase == PHASE_CV) {
        // Read actual total current delivered to determine when CV is complete
        unsigned long now     = millis();
        unsigned long elapsed = now - cvMs;
        float actualA = 0.0f;
        for (int i = 0; i < MAX_CHARGERS; i++) {
          if (chargerBus.chargers[i].present &&
              (now - chargerBus.chargers[i].lastSeenMs) < CHARGER_TIMEOUT_MS)
            actualA += chargerBus.chargers[i].ampsDa / 10.0f;
        }

        // Accumulate CV session energy using actual charger output
        bool terminating = false;
        if (packDv > 0) {
          // CV termination check (give at least CV_MIN_MS to settle)
          bool termCurrent = (elapsed >= CV_MIN_MS) && (actualA < CV_TERM_A);
          bool termTimeout = (elapsed >= CV_HOLD_MS);
          if (termCurrent || termTimeout) {
            phase = PHASE_DONE;
            terminating = true;
            LOG("[RAMP] CV→DONE: %s after %lus (%.2fA actual)\n",
                termTimeout ? "timeout" : "current taper",
                elapsed / 1000UL, actualA);
          }
        }
        if (!terminating && packDv > 0 && actualA > 0.0f) {
          float cvPwrW = actualA * (packDv / 10.0f);
          session.energyWh += cvPwrW / 3600.0f;
          session.chargeAh += actualA  / 3600.0f;
        }

        // Keep charger alive in CV mode
        if (phase == PHASE_CV && chargersPresent && packDv > 0) {
          chargerBus.cmdVoltDv = voltCeiling;
          chargerBus.cmdAmpsDa = cvAmpsDa;
          chargerBus.cmdStart  = true;
        } else {
          // Just transitioned to DONE this tick — send STOP immediately
          chargerBus.cmdVoltDv = 0;
          chargerBus.cmdAmpsDa = 0;
          chargerBus.cmdStart  = false;
        }
      } else {
        // CC phase: start when current > 0 and pack is known; DONE: current == 0 stops it
        chargerBus.cmdVoltDv = cmdVoltDv;
        chargerBus.cmdAmpsDa = cmdAmpsDa;
        chargerBus.cmdStart  = (current > 0) && chargersPresent && (packDv > 0);
      }

      xSemaphoreGive(chargerMutex);
    }

    // Expose phase to API/dashboard (atomic uint8_t write, no mutex needed)
    g_rampPhase = (uint8_t)phase;

    // ── Log ───────────────────────────────────────────────────────────────────
    const char* ps = (phase==PHASE_CC) ? "CC" : (phase==PHASE_CV) ? "CV" : "DONE";
    if (phase == PHASE_CV) {
      LOG("[RAMP] %s +%lus | %ddV %ddA/ch | raw %lddV ceil %ddV\n",
          ps, (millis()-cvMs)/1000UL, voltCeiling, (int)cvAmpsDa, rawPackDv, voltCeiling);
    } else if (current > 0 || phase == PHASE_DONE) {
      LOG("[RAMP] %s %dW(eff) tgt%dW cmd %ddV/%ddA raw %lddV%s%s\n",
          ps, current, target, cmdVoltDv, cmdAmpsDa, rawPackDv,
          voltCbK != UINT32_MAX ? " VcbON" : "",
          hotCbK  != UINT32_MAX ? " HOT"   : "");
    }
  }
}

// Called from setup() — creates controlMutex and launches ramp task on Core 1
void rampInit() {
  controlMutex = xSemaphoreCreateMutex();
  if (controlMutex == nullptr) {
    LOG("[RAMP] Failed to create mutex — halting\n");
    while (true) { delay(1000); }
  }

  // Set default target to zero — charger count unknown at boot,
  // so we can't select the correct preset minimum yet.
  // The dashboard will send the correct default once charger_count
  // becomes non-zero (rebuildPresets triggers sendControl on first build).
  ctrl.targetPowerW  = 0;
  ctrl.currentPowerW = 0;
  ctrl.enabled       = false; // stays off until chargers are detected

  // Restore persisted ramp rate (default 50 W/s if never saved)
  ctrl.rampStepW = preferences.getUShort("ramp_step_w", DEFAULT_RAMP_STEP_W);
  if (ctrl.rampStepW < 10 || ctrl.rampStepW > 500) ctrl.rampStepW = DEFAULT_RAMP_STEP_W;

  // Target voltage defaults to 100% (highest preset) on boot
  ctrl.targetVoltDv = TARGET_VOLT_PRESETS[TARGET_VOLT_PRESET_COUNT - 1].dv;

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
static void mqttDiscoverSensor(
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
      "\"pl_not_avail\":\"offline\""
      "%s%s%s,"
      "\"dev\":{"
        "\"ids\":[\"%s\"],"
        "\"name\":\"Supercharger\","
        "\"mdl\":\"ESP32-S3 Supercharger\","
        "\"mf\":\"DIY\""
      "}"
    "}",
    friendlyName, mqttHostname, name,
    stateTopic, availTopic,
    dcPart, scPart, unitPart,
    devId
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
  mqttDiscoverSensor("monolith_soc",    "Monolith State of Charge", "%",      "battery",     "measurement");
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
  // Button
  mqttDiscoverButton("reset_session",   "Reset Charge Session");

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

  if (strcmp(cmd, "target_power_w") == 0) {
    int tw = atoi(val);
    if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      ctrl.targetPowerW = (uint16_t)constrain(tw, 0, 13200);
      xSemaphoreGive(controlMutex);
    }
    LOG("[MQTT] cmd target_power_w = %d W\n", tw);

  } else if (strcmp(cmd, "charging_enabled") == 0) {
    bool en = (strcmp(val, "true") == 0 || strcmp(val, "1") == 0 ||
               strcmp(val, "ON")   == 0 || strcmp(val, "on") == 0);
    if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      ctrl.enabled = en;
      if (!en) ctrl.currentPowerW = 0;
      xSemaphoreGive(controlMutex);
    }
    LOG("[MQTT] cmd charging_enabled = %s\n", en ? "true" : "false");

  } else if (strcmp(cmd, "charger_count") == 0) {
    int cc = atoi(val);
    if (cc >= 1 && cc <= 4) {
      if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        ctrl.chargerCount = (uint8_t)cc;
        xSemaphoreGive(controlMutex);
      }
      preferences.putUChar("charger_count", (uint8_t)cc);
      LOG("[MQTT] cmd charger_count = %d\n", cc);
    }

  } else if (strcmp(cmd, "ramp_rate_wps") == 0) {
    int rr = atoi(val);
    if (rr >= 10 && rr <= 500) {
      if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        ctrl.rampStepW = (uint16_t)rr;
        xSemaphoreGive(controlMutex);
      }
      preferences.putUShort("ramp_step_w", (uint16_t)rr);
      LOG("[MQTT] cmd ramp_rate_wps = %d W/s\n", rr);
    }

  } else if (strcmp(cmd, "target_volt_v") == 0) {
    // HA number sends value in Volts (e.g. "113.2"); convert to dV internally.
    float tvV = atof(val);
    int   tvd = (int)(tvV * 10.0f + 0.5f); // round to nearest dV
    if (tvd >= TARGET_VOLT_PRESETS[0].dv && tvd <= (int)MAX_CHARGE_VOLTAGE_DV) {
      if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        ctrl.targetVoltDv = (uint16_t)tvd;
        xSemaphoreGive(controlMutex);
      }
      LOG("[MQTT] cmd target_volt_v = %.1f V (%d dV)\n", tvV, tvd);
    }

  } else if (strcmp(cmd, "reset_session") == 0) {
    // Any payload (HA button sends "PRESS") triggers the reset
    session.energyWh = 0.0f;
    session.chargeAh = 0.0f;
    session.startMs  = millis();
    // Force MQTT snapshot reset so new zeroed values publish immediately
    mqttLast.sessionWh = -1;
    mqttLast.sessionAh = -1;
    LOG("[MQTT] cmd reset_session\n");
  }
}

// Connect/reconnect to broker — called from mqttTask when not connected
static bool mqttConnect() {
  char baseTopic[80], willTopic[90];
  snprintf(baseTopic, sizeof(baseTopic), "supercharger/%s", mqttHostname);
  snprintf(willTopic, sizeof(willTopic), "%s/state",        baseTopic);

  // Subscribe to both command topics
  char cmdBase[90];
  snprintf(cmdBase, sizeof(cmdBase), "%s/command/#", baseTopic);

  bool ok = mqttClient.connect(
    mqttHostname,
    mqttUser,
    mqttBrokerPass,
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
    LOG("[MQTT] Connected to %s:%d as %s\n", mqttHost, mqttPort, mqttHostname);
  } else {
    LOG("[MQTT] Connect failed, rc=%d\n", mqttClient.state());
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

  // Session data is only written by rampTask (Core 1), same core as mqttTask — safe to read directly
  float sessWh = session.energyWh;
  float sessAh = session.chargeAh;

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

  // Monolith pack
  PUB_IF_CHANGED_F(monolithVoltageDv, "monolith_v",    ls.monolithVoltageDv / 10.0f, 1)
  PUB_IF_CHANGED_I(monolithAmps,      "monolith_a",    ls.monolithAmps)
  PUB_IF_CHANGED_I(monolithMinTemp,   "monolith_tmin", ls.monolithMinTemp)
  PUB_IF_CHANGED_I(monolithMaxTemp,   "monolith_tmax", ls.monolithMaxTemp)
  PUB_IF_CHANGED_I(monolithSoc,       "monolith_soc",  calcSocFromVoltage(ls.monolithVoltageDv))

  // PowerTank pack (only publish while present or during the cycle it disappears)
  if (ls.powerTankPresent || mqttLast.powerTankPresent) {
    PUB_IF_CHANGED_F(powerTankVoltageDv, "powertank_v",    ls.powerTankVoltageDv / 10.0f, 1)
    PUB_IF_CHANGED_I(powerTankAmps,      "powertank_a",    ls.powerTankAmps)
    PUB_IF_CHANGED_I(powerTankMinTemp,   "powertank_tmin", ls.powerTankMinTemp)
    PUB_IF_CHANGED_I(powerTankMaxTemp,   "powertank_tmax", ls.powerTankMaxTemp)
    mqttLast.powerTankPresent = ls.powerTankPresent;
  }

  // Charging control state
  PUB_IF_CHANGED_I(chargerCount,  "charger_count",   ks.chargerCount)
  PUB_IF_CHANGED_I(currentPowerW, "current_power_w", ks.currentPowerW)
  PUB_IF_CHANGED_I(targetPowerW,  "target_power_w",  ks.targetPowerW)
  PUB_IF_CHANGED_I(rampStepW,     "ramp_rate_wps",   ks.rampStepW)
  PUB_IF_CHANGED_F(targetVoltDv,  "target_volt_v",   ks.targetVoltDv / 10.0f, 1)

  if (mqttLast.enabledForced || ks.enabled != mqttLast.enabled) {
    mqttPublishSensor("charging_enabled", ks.enabled ? "true" : "false");
    mqttLast.enabled       = ks.enabled;
    mqttLast.enabledForced = false;
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

  #undef PUB_IF_CHANGED_F
  #undef PUB_IF_CHANGED_I

  return published;
}

void mqttTask(void* /*pvParameters*/) {
  LOG("[MQTT] mqttTask running on core %d\n", xPortGetCoreID());

  mqttClient.setServer(mqttHost, mqttPort);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512); // discovery payloads need > default 256 bytes

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

    if (!mqttClient.connected()) {
      unsigned long now = millis();
      if (now - lastConnectAttempt >= RECONNECT_INTERVAL) {
        lastConnectAttempt = now;
        // Re-apply server settings in case they were changed via /api/settings
        mqttClient.setServer(mqttHost, mqttPort);
        mqttConnect();
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
// Logs the offending task name then halts. Requires configCHECK_FOR_STACK_OVERFLOW >= 1
// in FreeRTOSConfig.h (enabled by default in ESP32 Arduino core).
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
  LOG("[FATAL] Stack overflow in task: %s — halting\n", pcTaskName);
  while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}
