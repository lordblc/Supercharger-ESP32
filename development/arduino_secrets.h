// Both SSID and password must be 8 characters or longer
#define SECRET_SSID "sc2.5"
#define SECRET_PASS "supersmartlongpassword"
#define SECRET_MQTT_SSID "myssid"
#define SECRET_MQTT_PASS "mymqttwlanpass"
#define SECRET_MQTT_HOST "10.0.0.1"  // or IP address
#define SECRET_MQTT_USER "zerobike"   // leave empty if EMQX is configured for anonymous access
#define SECRET_MQTT_BROKER_PASS "mymqttuserpass"

// OTA update credentials (HTTP Basic Auth on /update endpoint)
#define SECRET_OTA_USER "username"
#define SECRET_OTA_PASS "yetanothersupersmartlongpassword"

// When the controller falls back to AP mode it normally serves the dashboard
// at 192.168.4.1. Set these to give AP mode a FIXED IP that mirrors your home
// network, so an iOS "Add to Home Screen" shortcut pinned to that address
// works both at home and on the road.
// Leave all three "" to keep the default behaviour. Defining a non-empty
// SECRET_AP_IP automatically enables the feature (the Settings page can still
// override it at runtime). All three must be valid dotted-quad strings.
#define SECRET_AP_IP      ""   // e.g. "192.168.1.50"  — the dashboard IP
#define SECRET_AP_GATEWAY ""   // e.g. "192.168.1.1"   — usually your router
#define SECRET_AP_SUBNET  ""   // e.g. "255.255.255.0"