# Supercharger 2.5 - ESP32 Controller

## User Manual

A WiFi connected CAN bus controller for charging a Zero motorcycle with up to 4 Elcon TC HK-J 3300W chargers in parallel. You get a web dashboard, optional Home Assistant integration, and firmware updates over the air.

---

## What You'll Need

- Supercharger 2.5 controller (LilyGo T-2CAN, ESP32-S3 based)
- 1 to 4 Elcon TC HK-J 3300W chargers
- Zero motorcycle with CAN bus access to the BMS
- A 2.4 GHz WiFi network (optional, but recommended)
- A phone, tablet, or laptop with a browser for setup

---

## Connecting the Hardware

*[TO BE WRITTEN]*

This section covers the 4 pin connector from the Elcon chargers, the 12 VDC supply, CAN bus wiring to the bike, and the splitters used for parallel charger builds.

---

## First Run

Once everything is wired up:

1. Power on the chargers. The 12 VDC from the Elcon cable feeds the controller.
2. The controller boots. Startup takes roughly 2 to 3 seconds.
3. On a fresh unit there are no saved WiFi credentials, so it falls back to AP (setup) mode. If you have connected a laptop to the USB-C port, you'll see a log line on serial saying `No credentials available. Starting AP mode.`
4. Move on to WiFi setup below.

If you want to watch it boot, connect a USB-C cable to the controller and open a serial monitor at 115200 baud.

---

## WiFi Setup (AP Mode)

On first boot, or any time the controller can't find a known network, it creates its own WiFi network for setup.

1. On your phone or laptop, scan for WiFi networks. Look for **Supercharger** (or whatever the builder set as `SECRET_SSID` — see the arduino_secrets.h section below).
2. Connect to it. The default password is **12345678** (or `SECRET_PASS` if the builder set one).
3. Your phone will probably warn you there's no internet. That's fine, ignore it.
4. Open a browser and go to **http://192.168.4.1** or **http://supercharger.local**.
5. You'll see a WiFi Setup page. Enter your home WiFi name (SSID) and password.
6. Hit **Save & Restart**.
7. The controller saves the credentials, reboots, and connects to your home WiFi.

Credentials are stored in non-volatile memory, so you only do this once per network. The AP name and password can also be changed at runtime from the **Settings** page once you're connected — handy if you want a stronger AP password but don't want to reflash.

### Switching to a Different Network Later

If the saved network is gone (moved house, router changed, etc), the controller will try the saved one for 15 seconds, fail, then fall back to AP mode on its own. From there, just redo the setup above with the new credentials.

If the old network is still around and you want to force setup mode anyway, you'll need to reflash the firmware to clear the saved credentials. A proper reset button is on the list for a future version. 

### Using the Controller Without Home WiFi

The controller works fine without internet. It'll sit in AP mode and you can use the dashboard by connecting straight to the AP (default **Supercharger**). Everything local still works:

- Charger monitoring
- Power slider and presets
- Start / stop charging
- Battery data
- Live log viewer

Only Home Assistant integration and OTA updates need internet.

---

## Finding the Dashboard

Once the controller joins your home WiFi, the easiest way in is mDNS:

- **mDNS**: open **http://supercharger.local** in any browser on the same network. Works on Windows 10+, macOS, iOS, Android (most), and any Linux box running Avahi. No IP guessing required. Also works in AP mode — connect to the controller's AP and the same URL resolves to 192.168.4.1.

If mDNS doesn't resolve (some corporate or guest networks block multicast), fall back to:

- **Serial log**: connect USB-C and open a serial monitor at 115200 baud. Look for a line like `[WIFI] Connected (prefs). IP: 192.168.1.42`.
- **Router**: log into your router's admin page and check the DHCP client list. The controller usually shows up as "espressif" or similar.

Type the IP into any browser on the same network and the dashboard loads.

---

## Using the Dashboard

The dashboard auto refreshes every 2 seconds.

### Battery Sections

**Monolith Pack** shows voltage, current, capacity, temperature, and max C rate. Always visible.

**PowerTank Pack** shows the same if a PowerTank is present. It's auto detected within 10 seconds of the first bike CAN frame. If it doesn't show up in that window, the section stays hidden.

### Chargers

One card per detected charger, with voltage, current, and status. Chargers appear automatically within a few seconds of being connected. A heartbeat badge at the top shows whether the controller is successfully talking to the bus.
**NOTE:** The DigiNow pack this was programmed against, came with one(and same) ID programmed towards all 3 chargers. If you have this setup, the system will only detect one charger, but you can set this static as a work-around.

### Charging Control

**ON / OFF button**: enables or disables charging. When disabled, the controller commands zero power immediately, no ramp down.

**Power slider**: set your target power in watts. The controller ramps from current to target at 50 W(adjustable) per second, so jumping from 500 W to 3300 W takes about a minute to get there. This is intentional.

**Preset buttons**: auto scale based on how many chargers you have connected. Just tap one to jump to that power.

Presets per charger count:

| Chargers | Presets (W) |
| --- | --- |
| 1 | 500, 1000, 1650, 2200, 3300 |
| 2 | 1000, 2200, 3300, 5000, 6600 |
| 3 | 1600, 3300, 5000, 6600, 9900 |
| 4 | 2200, 4400, 6600, 9900, 13200 |

**Auto behavior**: when chargers first appear, the controller auto enables charging at the lowest preset. When they all disappear (e.g. you unplug), it auto disables. You don't have to babysit it.

### Charge Limit Presets (% of Full)

Below the power slider you'll find a row of **70 / 80 / 90 / 100 %** buttons. These set the **target pack voltage** — i.e. how full the controller will let the pack get before it stops feeding it. They are the key knob for trading range against pack longevity, and they directly drive the CC / CV state machine described below.

| Preset | Pack voltage ceiling | Roughly what it gives you |
| --- | --- | --- |
| 70 % | 106.0 V | Daily commute / short rides. Easiest on the cells; a useful default if the bike sits for days between rides. |
| 80 % | 110.0 V | Recommended day-to-day setting. Most of the range, much less stress than topping right out. |
| 90 % | 113.2 V | Longer rides. A gentle compromise. |
| 100 % | 116.4 V | Use the day before a long ride or for periodic balancing. The pack will sit slightly hot and at full voltage, which accelerates calendar ageing if you leave it there for days. |

The percentages are based on the Zero monolith pack open-circuit voltage curve (28S Li-ion NMC). The mapping is not perfectly linear with state of charge (Li-ion never is), but it's close enough that "80 %" lands you near 80 % SOC.

The dashboard remembers your last selection across reboots. The same setting is exposed over MQTT (`target_volt_v`) for Home Assistant automations — e.g. set 100 % only the night before a trip.

### CC / CV / DONE — How the Controller Stops Charging

The controller runs a three-phase charge cycle modelled on what a "smart" Li-ion charger does. You don't need to interact with it; it's automatic. But understanding the phases makes the dashboard a lot less mysterious.

**Phase 1 — CC (Constant Current)**
This is the bulk of the charge. The controller commands the chargers to deliver the power you've set on the slider, ramping up at 50 W/s. Pack voltage rises gradually as the cells absorb energy. The dashboard phase indicator reads **CC**. Voltage and temperature cutbacks (see `battery_tables.h`) only apply in this phase — they trim power back automatically as the pack gets close to full or warm.

**Phase 2 — CV (Constant Voltage / Absorption)**
When the pack reaches the voltage ceiling you picked with the % preset, the controller switches to **CV**. It now holds the pack at that exact voltage and lets the chargers reduce current naturally as the cells finish topping up. The current target ramps down on the dashboard, but actual charger output continues — usually at a few amps total. This phase finishes when one of two things happens, whichever comes first:

- Actual delivered current drops below **2 A total** (after a 2 minute settling window).
- A **20 minute safety timeout** elapses.

CV is short for the lower presets (70 / 80 %) and longer for 90 / 100 %, where there's more energy left to absorb at the ceiling.

**Phase 3 — DONE**
Charging stops. Heartbeat to the chargers ends, the relay drops, and the dashboard shows **DONE**. The session totals (Wh, Ah) freeze at their final values.

**Recovery transitions** — the controller doesn't just sit in DONE forever:

- **CV → CC** if pack voltage sags more than 1 V below the ceiling (e.g. you start riding with the bike still plugged in). It re-enters CC to top up.
- **DONE → CC** if pack voltage drops at least 2 V below the ceiling (e.g. you've ridden the bike). The next time it sees the bike, it'll start a fresh charge cycle without you doing anything.

**Practical implication of the % presets**: choosing 70 % doesn't just mean "stop earlier" — it also means a much shorter CV phase (or none at all if the ceiling is below where the pack would naturally taper). That's why low presets feel quick: the bulk of cell stress in a Li-ion charge is during CV at high voltage, and you're skipping most of it.

### System Info

At the bottom: uptime, WiFi signal, firmware version, CPU load per core, free heap. Useful when you're debugging.

### Log Viewer

Click the Log link, or go to **/log**. Live serial log in your browser, updated in real time. Good for diagnosing CAN bus issues without needing a USB cable.

---

## For Builders: First Time Flashing

### Required Libraries

Install these from the Arduino Library Manager:

- `mcp_can` by coryjfowler
- `PubSubClient` by Nick O'Leary

The rest (`ESPmDNS`, `driver/twai.h`) ship with the Espressif ESP32 Arduino core, no separate install needed.

### Arduino IDE Board Settings

Install ESP32 board support via Boards Manager (search "esp32" by Espressif Systems) if you haven't already. Then match these settings under the Tools menu. These are LilyGo's recommended settings for the T-2CAN:

| Setting | Value |
| --- | --- |
| Board | ESP32S3 Dev Module |
| Upload Speed | 921600 |
| USB Mode | Hardware CDC and JTAG |
| USB CDC On Boot | Enabled |
| USB Firmware MSC On Boot | Disabled |
| USB DFU On Boot | Disabled |
| Upload Mode | UART0 / Hardware CDC |
| CPU Frequency | 240MHz (WiFi) |
| Flash Mode | QIO 80MHz |
| Flash Size | 16MB (128Mb) |
| Partition Scheme | 16M Flash (3MB APP/9.9MB FATFS) |
| Core Debug Level | None |
| PSRAM | OPI PSRAM |
| Arduino Runs On | Core 1 |
| Events Run On | Core 1 |
| Erase All Flash Before Sketch Upload | Enabled |
| JTAG Adapter | Disabled |
| Zigbee Mode | Disabled |

The 3 MB APP partition is what makes OTA work (OTA needs two equal sized app partitions to swap between).

### Compile and Upload

1. Open `Supercharger_ESP32S3.ino` in the Arduino IDE.
2. Edit `arduino_secrets.h` with your credentials (see the next section).
3. Plug the controller into your computer with a USB-C cable.
4. Select the correct port under Tools > Port.
5. Click Upload.

First upload over USB takes a minute or two. After that, you can push updates over the air via `/update` without needing the cable.

### Note on "Erase All Flash Before Sketch Upload"

This one is optional. It's Enabled in the table above because the original build had some early development issues that were solved by clean flashes. It's not a requirement for the firmware to work, and you can safely set it to Disabled if you prefer.

What the setting does: when Enabled, every USB upload wipes all flash including NVS, which is where saved WiFi credentials live. The controller will come back up in AP mode after every upload, and you'll have to redo the WiFi setup page each time. That gets old fast.

Set it to Disabled for a smoother dev loop. Just be aware that old NVS state can become incompatible with new firmware if you ever change the structure of what's stored.

Either way, OTA updates via the web don't touch NVS, so WiFi credentials survive OTA fine.

### ESP32 Arduino Core Version

This firmware has been developed and tested against recent versions of the Espressif ESP32 Arduino core. If something fails to compile, update your ESP32 core to the latest release via Boards Manager and try again. The TWAI and USB CDC APIs have changed across versions, so old cores may not work.

---

## For Builders: arduino_secrets.h

If you're compiling this yourself, you have to set credentials in `arduino_secrets.h` before flashing. You must create your own values. Don't ship with the defaults.

```
#define SECRET_SSID             "Supercharger"     // AP fallback name (optional override)
#define SECRET_PASS             "12345678"         // AP fallback password (optional override, min 8)

#define SECRET_MQTT_SSID        "your-home-wifi"
#define SECRET_MQTT_PASS        "your-wifi-password"
#define SECRET_MQTT_HOST        "192.168.1.100"   // broker IP or hostname
#define SECRET_MQTT_USER        "your-mqtt-user"
#define SECRET_MQTT_BROKER_PASS "your-mqtt-password"

#define SECRET_OTA_USER         "your-ota-username"
#define SECRET_OTA_PASS         "your-ota-password"
```

### Requirements

- **WiFi password must be at least 8 characters**. This is an ESP32 limitation, not mine.
- **OTA user and pass are required**. They protect the `/update` endpoint, as well as charge control, so random people on your network can't reflash or set insane charge values, on/with your controller. Pick something you'll remember but isn't trivial. During creation, this was tested with 32+ characters, so should not be a limiting factor.
- **MQTT credentials are optional**. If you don't use Home Assistant, leave them as empty strings. The MQTT client will keep trying, fail quietly, and not affect anything else.
- **`SECRET_MQTT_SSID` can be blank**. If it is, the controller goes straight to AP mode on first boot. You can still set up WiFi through the setup page. If you set up a WiFi network here, it will automatically set this up and try to connect to it.

### Compile-time AP Defaults (`SECRET_SSID` / `SECRET_PASS`)

`SECRET_SSID` and `SECRET_PASS` set the **default AP name and password** the controller falls back to when no home WiFi is reachable. They override the built-in defaults of `Supercharger` / `12345678`.

The full priority order for the AP credentials is:

1. **NVS** — whatever was last saved via the **Settings** page (highest priority).
2. **`SECRET_SSID` / `SECRET_PASS`** — compile-time defaults from `arduino_secrets.h`.
3. **Built-in defaults** — `Supercharger` / `12345678` (used only if both above are blank).

The point is to let a builder ship a unit with a stronger out-of-the-box AP password than `12345678`, without having to walk a new owner through the Settings page on first boot. If you don't care, just leave `SECRET_SSID` blank and you'll get the built-in defaults.

The same 8-character minimum applies to `SECRET_PASS` as to any WiFi password — that's an ESP32 limitation.

---

## OTA Firmware Updates

Once the controller is on your WiFi:

1. Compile a new firmware in the Arduino IDE and export the compiled `.bin` (Sketch > Export Compiled Binary).
2. Go to **http://\<controller-ip\>/update** in a browser. (or just click the OTA link from Dashboard web page)
3. Enter your OTA username and password (the ones you set in `arduino_secrets.h`).
4. Pick the `.bin` file and click Upload.
5. The controller flashes and reboots automatically.

If the update fails mid way, the old firmware is kept. You won't brick the controller unless the flash itself is cut mid write (e.g. power loss during upload).

---

## Home Assistant Integration

With MQTT credentials set, the controller auto publishes HA discovery topics under `homeassistant/` whenever it connects. A "Supercharger" device shows up in your HA integrations with:

**Sensors** (read only): pack voltages, currents, temperatures, Ah, C rate, charging power, target power, charger count, heartbeat status.

**Controls** (read / write): target power (0 to 13200 W), charging on / off switch.

Device base topic: `supercharger/<hostname>/`. Hostname is whatever the ESP32 assigns by default.

---

## Troubleshooting

**"No chargers detected" on the dashboard**
Check the 4 pin cable between the chargers and the controller. Confirm the charger CAN bus is at 250 kbps and has 120 Ω termination at both ends. The charger bus is physically separate from the bike bus.
**NOTE:** The Elcon chargers can be shipped both with and without termination. This _can_ work without, but is prone to generate errors in CAN frames. If you have no terminator in the charger end, try to keep the length of wire as short as possible.

**Heartbeat status shows STOPPED**
The controller is sending heartbeat frames but nothing is acking them. Usually means no charger is actually powered up, or a wiring fault on the charger bus.

**Battery data never appears**
Check the bike CAN wiring (GPIO 6 RX, GPIO 7 TX on the ESP32-S3). The bike bus runs at 500 kbps with standard 11 bit IDs. The controller needs at least one BMS0 frame before it shows pack data.

**Can't connect to the Supercharger AP**
Default password is exactly `12345678`, eight characters (or whatever the builder set in `SECRET_PASS`). Some phones cache old passwords, so forget the network and reconnect. The AP broadcasts on 2.4 GHz only.

**Lost WiFi during use**
If the home WiFi drops out while the controller is running, it falls back to AP mode automatically. It won't retry the home WiFi on its own. Power cycle the controller to make it try again.
**NOTE:** This might change in a future release. The author has spotty WiFi coverage where the bike is parked and fixing this point was not of priority.

**Forgot the IP address**
Try **http://supercharger.local** first — that's mDNS and works on most modern OSes. If that fails (corporate network, multicast disabled, ancient phone), plug in USB-C and check the serial log, or look at your router's DHCP client list.

**Dashboard blank or stuck**
Reload the page. If that doesn't help, check `/log` for errors. You can also power cycle the controller by cutting the 12 V from the chargers.

---

## Technical Reference

### WiFi Boot Priority

1. Saved credentials (set via the setup page)
2. Compile time secrets (`SECRET_MQTT_SSID` / `SECRET_MQTT_PASS`)
3. AP mode (`ESP32_Setup` / `12345678`)

Each attempt has a 15 second timeout before falling through. Maximum worst case fallback time is about 30 seconds (prefs fail, then secrets fail, then AP starts).

### CAN Buses

| Bus | Interface | Pins | Speed | IDs |
| --- | --- | --- | --- | --- |
| Bike | ESP32-S3 TWAI | RX=GPIO 6, TX=GPIO 7 | 500 kbps | 11 bit |
| Charger | MCP2515 over SPI | CS=10, SCLK=12, MOSI=11, MISO=13, RST=9 | 250 kbps | 29 bit extended |

### Key Parameters

- Heartbeat frame (controller to charger): ID `0x1806E5F4`, 1 Hz. Stops when charging is disabled.
- Ramp rate: 50 W per 1 second tick.
- Max charge voltage: capped by the highest threshold in the voltage cutback table (see `battery_tables.h`).
- Max total power: 13200 W (4 chargers at 3300 W each).

---
