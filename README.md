Ok, so.. I set out to build upon the work skonk made that is referred to in this thread:

https://www.electricmotorcycleforum.com/boards/index.php?topic=13489.0

My goal was to achieve some additional abilities.
I wanted to use better hardware. I found his approach to be very clunky and i could not see how I would make it fit on the bike.
It didn't help that the CAN cable on the bike was very short and limitting me to basically place it close to the fusebox underneath the seat. An area that is already quite space limitted.
Anyway.. I also wanted to include HomeAssistant control as i already have that on my other chargeable vehicles.

I tried to do the easy approach first, and simply employed CoPilot to just adapt the original code from Arduino to ESP32. This yielded a somewhat result but had issues.
I referred to that result in the same forumthread as well as here:

https://www.electricmotorcycleforum.com/boards/index.php?topic=13748.0

Having kept at it and starting from scratch, I formulated an approach as a project manager and set out to make something with Claude as my assistant. I started using the Sonnet 4.5 model, but eventually moved on to Opus 4.6.
My approach was to always be in control of what was created and dividing the codebase into smaller chunks where I was able to keep up with what was suggested from the AI.

So, the details are as follows:
I based this on the LilyGo T-2CAN which has 2 onboard, isolated CAN transceivers ready to go.
It has 12-24VDC input so can be connected directly to the DC output from the chargers, eliminating one of the PSU's from the original. (I would still highly recommend using an isolated 5VDC for the bike connector to keep these as seperate as possible.)

The controller sits between the bike's CAN bus and the charger's CAN bus and manages the full charge cycle automatically.

Hardware

LilyGo T-2CAN (ESP32-S3) (https://lilygo.cc/products/t-2can)

Charger CAN bus: MCP2515 via SPI at 250 kbps

Bike CAN bus: ESP32-S3 native TWAI peripheral at 500 kbps

Tested with Elcon TC HK-J 3300W chargers; multiple units can run in parallel

Bike CAN Bus Interface

The firmware decodes the following from the Zero CAN bus:

Pack voltage from both the Monolith (BMS 0x388) and PowerTank (BMS1 0x389)
Pack current from both BMS units (0x408 / 0x409)
Min and max battery temperatures from both BMS units (0x488 / 0x489), with a range filter to reject the non-thermocouple bytes that the Zero BMS puts in the same CAN frame
Rated pack capacity in Ah from BMS pack config (0x288 / 0x289)
Maximum allowed charge C-rate from the BMS (0x508 / 0x509)
Throttle position from the motor controller, used as a presence/riding check

Charge Control

The charger is commanded over CAN with a target voltage ceiling and a current limit. The firmware implements a three-phase state machine:

CC / Absorption: Constant current up to the configured power limit. Current ramps up gradually from zero at a configurable rate in W/s. Charge power is reduced as the pack voltage approaches the target through a lookup table (see below).
CV / Float: Once the pack voltage reaches the target, the firmware holds the voltage ceiling and monitors actual charger output current. The charger self-regulates in this region.
Complete: CV phase ends when actual charger current drops below 2 A (after a minimum 2-minute settle period) or after a 20-minute timeout, whichever comes first. The charger is then stopped and the session is marked complete.
The charger voltage ceiling is always set to the configured target, not the present pack voltage. This is important — setting it to the present pack voltage would leave the charger with no headroom and it would not regulate correctly.

Protection and Cutback

Three lookup tables apply independent power cutbacks. All three are evaluated every cycle and the most restrictive limit wins:

Voltage cutback: Reduces charge current progressively as the pack voltage approaches the target. The curve is intentionally aggressive toward the top end to avoid voltage overshoot. Thresholds are in decivolts of lifted (under-charge) pack voltage.
Cold temperature cutback: Reduces charge rate based on the lowest thermocouple reading across both BMS units. Keeps the cells within safe charge rates when cold.
Hot temperature cutback: Reduces and ultimately stops charging based on the highest thermocouple reading. Charging is stopped completely above 75 C.

Additional guards:

The charger will not be commanded to start unless a valid pack voltage is present on the CAN bus, preventing phantom starts at boot or after CAN loss.
Session Wh and Ah counters only accumulate when voltage, current, and a non-zero commanded current are all present simultaneously.

Session Tracking

Accumulated Wh and Ah for the current charge session
Both CC and CV phase energy are counted using actual charger current, not commanded current

Web Dashboard

The firmware runs a web server on the ESP32. No external server is required. The dashboard shows:

Charging card: actual power, target power, charge current, charge mode (Absorption / Float / Complete)
Per-pack data: voltage, current, Ah capacity, min and max temperature — separately for Monolith and PowerTank
Session Wh and Ah totals
Configurable parameters: target voltage (V), power limit (W), ramp rate (W/s), number of chargers, C+ mode
Status data is delivered via a JSON API endpoint and the page auto-refreshes without a full reload.

WiFi and OTA

Connects to a saved WiFi network on boot
Falls back to access point mode if no saved credentials, allowing initial setup via browser
mDNS hostname for easy access on the local network
OTA firmware update via browser at /update, protected with HTTP Basic Auth
MQTT and Home Assistant

Full MQTT discovery is included. The controller publishes itself to Home Assistant automatically with no manual YAML needed. (Tested with EMQX as MQTT server, should also work with mosquitto)

Read-only sensors published:

actual_volts, target_volts
monolith_volts, powertank_volts
charge_amps, monolith_amps, powertank_amps
monolith_ah, powertank_ah
monolith_min_temp, monolith_max_temp, powertank_min_temp, powertank_max_temp
charge_power, target_power_w, max_power_w
watt_hours, coulombs
charge mode / phase

Controllable entities:

Target voltage (V) — number slider
Target power (W) — number slider
Ramp rate (W/s) — number slider
Number of chargers — number slider
C+ mode — switch
Charging enabled — switch
Architecture

The firmware uses FreeRTOS on the ESP32-S3 dual core:

Core 0 handles the bike CAN listener and charger CAN output to makes sure the heartbeat is never interrupted by web/mqtt handling clogging the cpu
Core 1 handles the web server, charge ramp/control logic, and MQTT
Shared state between cores is protected with mutexes. A 4 KB ring buffer log is available over serial and as a server-sent event stream in the browser.

Required Libraries

mcp_can by coryjfowler

PubSubClient by Nick O'Leary

ArduinoJson by Benoit Blanchon (v6)

ESP32 Arduino core (includes TWAI driver, ESPmDNS, Update)


At current stage it should compile to ~1MB large binary file. I track changes through the VERSION tag that is also visible as reference on the dashboard.

And if you managed to read this far:
using arduino_secrets.h is the quickest way to set everything up, but is not required. First boot without configured secrets will default to a welcome page to set everything up with a wlan ap that has the default password set to "12345678" and the AP SSID set to "Supercharger".

I plan to add more detail here..
